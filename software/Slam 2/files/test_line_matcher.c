/*
 * test_line_matcher.c
 *
 * Synthetic tests for line_matcher + map_manager.
 * No hardware needed — we fabricate FeaturePackets directly.
 *
 * Compile and run on PC:
 *   gcc -O0 -g -Wall \
 *       tests/test_line_matcher.c \
 *       slam_core/line_matcher.c \
 *       slam_core/map_manager.c \
 *       -lm -o build/test_line_matcher
 *   ./build/test_line_matcher
 *
 * Expected output: all tests PASS.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <assert.h>

#include "messages.h"
#include "line_matcher.h"
#include "map_manager.h"

/* ── Test helpers ──────────────────────────────────────────────────────── */

static int g_tests_run  = 0;
static int g_tests_pass = 0;

#define CHECK(cond, msg)                                          \
    do {                                                          \
        g_tests_run++;                                            \
        if (cond) {                                               \
            g_tests_pass++;                                       \
            printf("  PASS  %s\n", msg);                         \
        } else {                                                  \
            printf("  FAIL  %s  (line %d)\n", msg, __LINE__);    \
        }                                                         \
    } while (0)

/* Build a LineFeature in Hough form from angle+distance+length */
static LineFeature make_lf(float angle, float distance, float length, uint8_t quality)
{
    LineFeature lf;
    memset(&lf, 0, sizeof(lf));
    lf.angle    = angle;
    lf.distance = distance;
    lf.length   = length;
    lf.quality  = quality;
    /* Endpoints on the line: midpoint + half-length in each direction */
    float nx = -sinf(angle);
    float ny =  cosf(angle);
    float mx = nx * distance;
    float my = ny * distance;
    float cx =  cosf(angle);  /* line direction */
    float cy =  sinf(angle);
    lf.x1 = mx + cx * (length * 0.5f);
    lf.y1 = my + cy * (length * 0.5f);
    lf.x2 = mx - cx * (length * 0.5f);
    lf.y2 = my - cy * (length * 0.5f);
    return lf;
}

/* Build a FeaturePacket from an array of LineFeatures */
static FeaturePacket make_packet(LineFeature *lfs, int n, uint32_t ts)
{
    FeaturePacket pkt;
    memset(&pkt, 0, sizeof(pkt));
    pkt.timestamp_ms = ts;
    pkt.count = (uint8_t)(n < MAX_LINES_PER_SCAN ? n : MAX_LINES_PER_SCAN);
    for (int i = 0; i < pkt.count; i++)
        pkt.lines[i] = lfs[i];
    return pkt;
}

static Pose zero_pose(void) { Pose p = {0, 0, 0}; return p; }

/* ── Tests ─────────────────────────────────────────────────────────────── */

/*
 * Test 1: Static wall detection
 *
 * Scenario: robot is stationary. Same wall line appears scan after scan.
 * After CONFIRM_THRESH scans it must become LINE_STATIC.
 * Pose is zero so sensor frame == map frame.
 */
static void test_static_wall(void)
{
    printf("\n[test_static_wall]\n");
    Map m; map_init(&m);
    Pose pose = zero_pose();

    /* One wall: angle=0 (horizontal), distance=2.0m, length=3.0m */
    LineFeature wall = make_lf(0.0f, 2.0f, 3.0f, 90);
    FeaturePacket pkt = make_packet(&wall, 1, 0);
    MatchResult   mr;

    /* Feed CONFIRM_THRESH+1 identical scans */
    int scans = CONFIRM_THRESH + 1;
    for (int i = 0; i < scans; i++) {
        pkt.timestamp_ms = (uint32_t)(i * 100);
        match_lines(pkt.lines, pkt.count, m.lines, m.n_lines, &pose, &mr);
        map_update(&m, &pkt, &mr, &pose);
    }

    /* Map should contain exactly 1 active line, and it must be STATIC */
    int n_static = 0, n_active = 0;
    for (int i = 0; i < m.n_lines; i++) {
        if (!m.lines[i].active) continue;
        n_active++;
        if (m.lines[i].state == LINE_STATIC) n_static++;
    }

    CHECK(n_active == 1, "exactly 1 active map line after stable wall");
    CHECK(n_static == 1, "wall is classified LINE_STATIC");
    CHECK(m.lines[0].confirmed == 1, "wall is confirmed");
    CHECK(m.lines[0].observed >= CONFIRM_THRESH,
          "wall observed >= CONFIRM_THRESH times");
}

/*
 * Test 2: Dynamic obstacle detection
 *
 * Scenario: a stable wall exists (confirmed). A person walks by — one scan
 * contains an extra line that matches nothing in the map. That line must
 * be classified DYNAMIC (quality >= 60, no map match).
 */
static void test_dynamic_obstacle(void)
{
    printf("\n[test_dynamic_obstacle]\n");
    Map m; map_init(&m);
    Pose pose = zero_pose();
    MatchResult mr;

    /* First: establish a confirmed wall */
    LineFeature wall = make_lf(0.0f, 2.0f, 3.0f, 90);
    FeaturePacket wall_pkt = make_packet(&wall, 1, 0);
    for (int i = 0; i < CONFIRM_THRESH + 1; i++) {
        wall_pkt.timestamp_ms = (uint32_t)(i * 100);
        match_lines(wall_pkt.lines, wall_pkt.count, m.lines, m.n_lines, &pose, &mr);
        map_update(&m, &wall_pkt, &mr, &pose);
    }

    /* Count static lines after wall establishment */
    int n_static_before = 0;
    for (int i = 0; i < m.n_lines; i++)
        if (m.lines[i].active && m.lines[i].state == LINE_STATIC)
            n_static_before++;
    CHECK(n_static_before == 1, "wall confirmed before obstacle appears");

    /* Now: one scan with wall + person (person at angle=0.5, distance=1.0, short) */
    LineFeature scan2[2] = {
        make_lf(0.0f,  2.0f, 3.0f, 90),  /* wall — matches map */
        make_lf(0.5f,  1.0f, 0.5f, 75),  /* person — no map match, high quality */
    };
    FeaturePacket mixed_pkt = make_packet(scan2, 2, 600);
    match_lines(mixed_pkt.lines, mixed_pkt.count, m.lines, m.n_lines, &pose, &mr);

    CHECK(mr.n_matched == 1,  "wall matches (1 static pair)");
    CHECK(mr.n_dynamic == 1,  "person classified as DYNAMIC in scan");

    map_update(&m, &mixed_pkt, &mr, &pose);

    /* Map should now have: 1 STATIC + 1 DYNAMIC */
    int n_static = 0, n_dynamic = 0;
    for (int i = 0; i < m.n_lines; i++) {
        if (!m.lines[i].active) continue;
        if (m.lines[i].state == LINE_STATIC)  n_static++;
        if (m.lines[i].state == LINE_DYNAMIC) n_dynamic++;
    }
    CHECK(n_static  == 1, "wall remains STATIC after obstacle scan");
    CHECK(n_dynamic == 1, "person added to map as DYNAMIC");
}

/*
 * Test 3: Dynamic line pruned after disappearing
 *
 * Person appears once, then is gone for DYNAMIC_TTL scans.
 * Their entry must be deactivated.
 */
static void test_dynamic_prune(void)
{
    printf("\n[test_dynamic_prune]\n");
    Map m; map_init(&m);
    Pose pose = zero_pose();
    MatchResult mr;

    /* Establish wall */
    LineFeature wall = make_lf(0.0f, 2.0f, 3.0f, 90);
    FeaturePacket wall_pkt = make_packet(&wall, 1, 0);
    for (int i = 0; i < CONFIRM_THRESH + 1; i++) {
        wall_pkt.timestamp_ms = (uint32_t)(i * 100);
        match_lines(wall_pkt.lines, wall_pkt.count, m.lines, m.n_lines, &pose, &mr);
        map_update(&m, &wall_pkt, &mr, &pose);
    }

    /* One scan with person */
    LineFeature scan_person[2] = {
        make_lf(0.0f, 2.0f, 3.0f, 90),
        make_lf(0.5f, 1.0f, 0.5f, 75),
    };
    FeaturePacket person_pkt = make_packet(scan_person, 2, 500);
    match_lines(person_pkt.lines, person_pkt.count, m.lines, m.n_lines, &pose, &mr);
    map_update(&m, &person_pkt, &mr, &pose);

    /* Confirm dynamic entry exists */
    int has_dynamic = 0;
    for (int i = 0; i < m.n_lines; i++)
        if (m.lines[i].active && m.lines[i].state == LINE_DYNAMIC)
            has_dynamic++;
    CHECK(has_dynamic == 1, "dynamic entry created after person scan");

    /* Person is gone — DYNAMIC_TTL scans with only the wall */
    for (int i = 0; i < DYNAMIC_TTL + 1; i++) {
        wall_pkt.timestamp_ms = (uint32_t)((600 + i) * 100);
        match_lines(wall_pkt.lines, wall_pkt.count, m.lines, m.n_lines, &pose, &mr);
        map_update(&m, &wall_pkt, &mr, &pose);
    }
    map_prune(&m);

    /* Dynamic entry must be gone */
    int n_dynamic = 0, n_static = 0;
    for (int i = 0; i < m.n_lines; i++) {
        if (!m.lines[i].active) continue;
        if (m.lines[i].state == LINE_DYNAMIC) n_dynamic++;
        if (m.lines[i].state == LINE_STATIC)  n_static++;
    }
    CHECK(n_dynamic == 0, "dynamic entry pruned after DYNAMIC_TTL misses");
    CHECK(n_static  == 1, "static wall survives person disappearing");
}

/*
 * Test 4: Occlusion — wall temporarily hidden
 *
 * A confirmed wall stops appearing (something blocking it).
 * After OCCLUDE_THRESH misses it becomes LINE_OCCLUDED, not deleted.
 * When it reappears, it goes back to LINE_STATIC.
 */
static void test_occlusion(void)
{
    printf("\n[test_occlusion]\n");
    Map m; map_init(&m);
    Pose pose = zero_pose();
    MatchResult mr;

    LineFeature wall = make_lf(0.0f, 2.0f, 3.0f, 90);
    FeaturePacket wall_pkt = make_packet(&wall, 1, 0);

    /* Establish wall as STATIC */
    for (int i = 0; i < CONFIRM_THRESH + 1; i++) {
        wall_pkt.timestamp_ms = (uint32_t)(i * 100);
        match_lines(wall_pkt.lines, wall_pkt.count, m.lines, m.n_lines, &pose, &mr);
        map_update(&m, &wall_pkt, &mr, &pose);
    }
    CHECK(m.lines[0].state == LINE_STATIC, "wall confirmed STATIC before occlusion");

    /* Wall disappears: send empty scans for OCCLUDE_THRESH+1 scans */
    FeaturePacket empty_pkt;
    memset(&empty_pkt, 0, sizeof(empty_pkt));
    for (int i = 0; i < OCCLUDE_THRESH + 1; i++) {
        empty_pkt.timestamp_ms = (uint32_t)((CONFIRM_THRESH + 1 + i) * 100);
        empty_pkt.count = 0;
        match_lines(empty_pkt.lines, 0, m.lines, m.n_lines, &pose, &mr);
        map_update(&m, &empty_pkt, &mr, &pose);
    }

    int n_occluded = 0;
    for (int i = 0; i < m.n_lines; i++)
        if (m.lines[i].active && m.lines[i].state == LINE_OCCLUDED)
            n_occluded++;
    CHECK(n_occluded == 1, "wall becomes OCCLUDED after long absence");

    /* Wall reappears */
    wall_pkt.timestamp_ms = (uint32_t)((CONFIRM_THRESH + OCCLUDE_THRESH + 5) * 100);
    match_lines(wall_pkt.lines, wall_pkt.count, m.lines, m.n_lines, &pose, &mr);
    map_update(&m, &wall_pkt, &mr, &pose);

    int n_static = 0;
    for (int i = 0; i < m.n_lines; i++)
        if (m.lines[i].active && m.lines[i].state == LINE_STATIC)
            n_static++;
    CHECK(n_static == 1, "wall returns to STATIC after reappearing");
}

/*
 * Test 5: Transform — non-zero pose
 *
 * Robot is at pose (1.0, 0.5, pi/4).
 * A wall in sensor frame at angle=0, distance=1.0 should transform
 * correctly. We verify the transformed Hough parameters are consistent
 * with the known rotation.
 */
static void test_transform(void)
{
    printf("\n[test_transform]\n");

    LineFeature lf_sensor = make_lf(0.0f, 1.0f, 2.0f, 80);
    Pose pose = {1.0f, 0.5f, (float)(M_PI / 4.0)};
    LineFeature lf_map;

    transform_line_to_map(&lf_sensor, &pose, &lf_map);

    /* angle should be sensor_angle + theta = 0 + pi/4 */
    float expected_angle = (float)(M_PI / 4.0);
    float angle_err = fabsf(lf_map.angle - expected_angle);

    CHECK(angle_err < 0.001f, "angle transforms correctly with pi/4 rotation");

    /* Endpoints should be reachable from transformed midpoint */
    float mx = (lf_map.x1 + lf_map.x2) * 0.5f;
    float my = (lf_map.y1 + lf_map.y2) * 0.5f;
    /* Midpoint of sensor-frame line: (0, 1.0) → rotated + translated */
    float expected_mx = 1.0f + cosf(pose.theta) * 0.0f - sinf(pose.theta) * 1.0f;
    float expected_my = 0.5f + sinf(pose.theta) * 0.0f + cosf(pose.theta) * 1.0f;
    float mx_err = fabsf(mx - expected_mx);
    float my_err = fabsf(my - expected_my);

    CHECK(mx_err < 0.005f, "midpoint x transforms correctly");
    CHECK(my_err < 0.005f, "midpoint y transforms correctly");
}

/* ── Entry point ──────────────────────────────────────────────────────── */

int main(void)
{
    printf("=== line_matcher + map_manager unit tests ===\n");

    test_static_wall();
    test_dynamic_obstacle();
    test_dynamic_prune();
    test_occlusion();
    test_transform();

    printf("\n=== Results: %d / %d passed ===\n", g_tests_pass, g_tests_run);
    return (g_tests_pass == g_tests_run) ? 0 : 1;
}