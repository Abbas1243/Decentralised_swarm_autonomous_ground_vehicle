/*
 * test_correlative_match.c
 * ==========================
 * Mirrors correlative_match.py's own self-test (T1-T7; T8/T9 in the
 * Python version only exist to cross-check the numpy-vectorized scoring
 * path against the scalar reference — this C port has only ONE scoring
 * path, the scalar one, so there is nothing for a T8/T9 equivalent to
 * check here).
 *
 * Build:
 *   gcc -std=c11 -Wall -Wextra -O2 -I ../shared -o test_correlative_match \
 *       test_correlative_match.c ../slam_core/correlative_match.c -lm
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <assert.h>

#include "correlative_match.h"

#define PI_F 3.14159265358979323846f

/* ---- feature/map-entry builders ------------------------------------- */

static ScanFeature make_line(float angle, float x1, float y1, float x2, float y2) {
    ScanFeature f;
    memset(&f, 0, sizeof f);
    f.type = FEAT_LINE;
    f.angle = angle;
    f.distance = 0.0f;   /* unused by correlative_match — see test file docstring */
    f.cx = x1; f.cy = y1;      /* FEAT_LINE_X1/Y1 */
    f.t_start = x2; f.t_end = y2;  /* FEAT_LINE_X2/Y2 */
    f.length = hypotf(x2 - x1, y2 - y1);
    f.quality = 100;
    return f;
}

static ScanFeature make_arc(float cx, float cy, float r) {
    ScanFeature f;
    memset(&f, 0, sizeof f);
    f.type = FEAT_ARC;
    f.cx = cx; f.cy = cy; f.r = r;
    f.length = r * PI_F;
    f.quality = 100;
    return f;
}

static CorrMapEntry make_map_line(float angle, float distance, float mx, float my) {
    CorrMapEntry e;
    e.angle = angle; e.distance = distance; e.mx = mx; e.my = my;
    e.status = CORR_ENTRY_STATIC; e.active = true;
    return e;
}

static CorrMapEntry make_map_entry_status(float angle, float distance, float mx, float my,
                                           uint8_t status) {
    CorrMapEntry e = make_map_line(angle, distance, mx, my);
    e.status = status;
    return e;
}

static CorrMapEntry make_map_arc(float mx, float my, float r) {
    CorrMapEntry e;
    e.angle = CORR_ARC_ANGLE_SENTINEL; e.distance = r; e.mx = mx; e.my = my;
    e.status = CORR_ENTRY_STATIC; e.active = true;
    return e;
}

/* ---- geometry helpers ------------------------------------------------ */

static void rotate_point(float x, float y, float cos_t, float sin_t, float *ox, float *oy) {
    *ox = x * cos_t - y * sin_t;
    *oy = y * cos_t + x * sin_t;
}

static float wrap_angle(float a) {
    if (a > PI_F / 2.0f) a -= PI_F;
    else if (a < -PI_F / 2.0f) a += PI_F;
    return a;
}

/* What the SENSOR would report if the robot moved by (dx,dy,dtheta)
   relative to where `feat` was observed at the origin — inverse of the
   map-frame transform, same convention as correlative_match.py's own
   test-only _shift() helper. */
static ScanFeature shift_line(ScanFeature feat, float dx, float dy, float dtheta) {
    float cos_t = cosf(-dtheta), sin_t = sinf(-dtheta);
    float x1 = feat.cx - dx,     y1 = feat.cy - dy;
    float x2 = feat.t_start - dx, y2 = feat.t_end - dy;
    float x1r, y1r, x2r, y2r;
    rotate_point(x1, y1, cos_t, sin_t, &x1r, &y1r);
    rotate_point(x2, y2, cos_t, sin_t, &x2r, &y2r);
    float angle = wrap_angle(feat.angle - dtheta);
    return make_line(angle, x1r, y1r, x2r, y2r);
}

static ScanFeature shift_arc(ScanFeature feat, float dx, float dy, float dtheta) {
    float cos_t = cosf(-dtheta), sin_t = sinf(-dtheta);
    float cx = feat.cx - dx, cy = feat.cy - dy;
    float ncx, ncy;
    rotate_point(cx, cy, cos_t, sin_t, &ncx, &ncy);
    ScanFeature out = feat;
    out.cx = ncx; out.cy = ncy;
    return out;
}

int main(void) {
    printf("correlative_match C self-test\n");
    printf("==================================================\n");

    /* ---- shared base geometry (same as correlative_match.py T1) ----- */
    ScanFeature base_scan[3] = {
        make_line(0.0f,        -0.5f, 1.0f,  0.5f, 1.0f),
        make_line(PI_F / 2.0f,  2.0f, -0.5f, 2.0f, 0.5f),
        make_line(-PI_F / 3.0f, -1.0f, 0.2f, -0.3f, 1.0f),
    };
    CorrMapEntry map_e[3] = {
        make_map_line(0.0f,        1.0f, 0.0f,  1.0f),
        make_map_line(PI_F / 2.0f, 2.0f, 2.0f,  0.0f),
        make_map_line(-PI_F / 3.0f,1.4f, -0.6f, 0.8f),
    };

    /* ---- T1: recover a real 8cm/-5cm/6deg motion --------------------- */
    {
        float true_dx = 0.08f, true_dy = -0.05f, true_dtheta = 6.0f * PI_F / 180.0f;
        ScanFeature moved[3];
        for (int i = 0; i < 3; i++) moved[i] = shift_line(base_scan[i], true_dx, true_dy, true_dtheta);

        CoarseResult r = correlative_search(moved, 3, map_e, 3, 0.0f, 0.0f, 0.0f, CORR_ENTRY_STATIC);
        assert(r.valid);
        assert(fabsf(r.dx - true_dx) < CORR_WALK_DXY_STEP_M);
        assert(fabsf(r.dy - true_dy) < CORR_WALK_DXY_STEP_M);
        assert(fabsf(r.dtheta - true_dtheta) < CORR_WALK_DTHETA_STEP_RAD);
        printf("  T1 PASS  recovered pose within refined resolution "
               "(dx=%.4f dy=%.4f dtheta=%.2fdeg) ambiguous=%d\n",
               r.dx, r.dy, r.dtheta * 180.0f / PI_F, r.ambiguous);
    }

    /* ---- T2: too few STATIC entries -> invalid, no crash ------------- */
    {
        CorrMapEntry sparse_map[2] = { map_e[0], map_e[1] };
        CoarseResult r = correlative_search(base_scan, 3, sparse_map, 2, 0.0f, 0.0f, 0.0f, CORR_ENTRY_STATIC);
        assert(!r.valid);
        assert(r.dx == 0.0f && r.dy == 0.0f && r.dtheta == 0.0f);
        printf("  T2 PASS  too few STATIC anchors -> valid=false, zero delta\n");
    }

    /* ---- T3: DYNAMIC/UNCLASSIFIED entries excluded from the search --- */
    {
        float true_dx = 0.08f, true_dy = -0.05f, true_dtheta = 6.0f * PI_F / 180.0f;
        ScanFeature moved[3];
        for (int i = 0; i < 3; i++) moved[i] = shift_line(base_scan[i], true_dx, true_dy, true_dtheta);

        CorrMapEntry map_with_mover[5];
        memcpy(map_with_mover, map_e, sizeof map_e);
        map_with_mover[3] = make_map_entry_status(0.3f, 5.0f, 5.0f, 5.0f, CORR_ENTRY_DYNAMIC);
        map_with_mover[4] = make_map_entry_status(-0.9f, -3.0f, -3.0f, 3.0f, CORR_ENTRY_UNCLASSIFIED);

        CoarseResult r = correlative_search(moved, 3, map_with_mover, 5, 0.0f, 0.0f, 0.0f, CORR_ENTRY_STATIC);
        assert(r.valid);
        assert(fabsf(r.dx - true_dx) < CORR_WALK_DXY_STEP_M);
        assert(fabsf(r.dy - true_dy) < CORR_WALK_DXY_STEP_M);
        assert(r.n_static == 3);
        printf("  T3 PASS  DYNAMIC/UNCLASSIFIED entries excluded (n_static=%d)\n", r.n_static);
    }

    /* ---- T4: motion far outside the window -> invalid ---------------- */
    {
        ScanFeature far_scan[3];
        for (int i = 0; i < 3; i++) far_scan[i] = shift_line(base_scan[i], 1.0f, 1.0f, 0.0f);
        CoarseResult r = correlative_search(far_scan, 3, map_e, 3, 0.0f, 0.0f, 0.0f, CORR_ENTRY_STATIC);
        assert(!r.valid);
        printf("  T4 PASS  out-of-window motion rejected by score gate (score=%.3f)\n", r.score);
    }

    /* ---- T5: parallel-wall translation ambiguity must be flagged ----- */
    {
        CorrMapEntry map_parallel[3] = {
            make_map_line(0.0f, 1.0f, 0.0f, 1.0f),
            make_map_line(0.0f, -1.0f, 0.0f, -1.0f),
            make_map_line(0.0f, 2.0f, 0.0f, 2.0f),
        };
        ScanFeature scan_parallel[3] = {
            make_line(0.0f, -0.8f, 1.0f, 0.8f, 1.0f),
            make_line(0.0f, -0.8f, -1.0f, 0.8f, -1.0f),
            make_line(0.0f, -0.8f, 2.0f, 0.8f, 2.0f),
        };
        CoarseResult r = correlative_search(scan_parallel, 3, map_parallel, 3, 0.0f, 0.0f, 0.0f, CORR_ENTRY_STATIC);
        assert(r.valid);
        assert(r.ambiguous);
        printf("  T5 PASS  parallel-wall translation ambiguity flagged "
               "(best=%.3f second=%.3f)\n", r.score, r.second_score);
    }

    /* ---- T6: one arc anchors x -- ambiguity resolved ------------------ */
    {
        CorrMapEntry map_asym[4] = {
            make_map_line(0.0f, 1.0f, 0.0f, 1.0f),
            make_map_line(0.0f, -1.0f, 0.0f, -1.0f),
            make_map_line(0.0f, 2.0f, 0.0f, 2.0f),
            make_map_arc(0.5f, 0.5f, 0.3f),
        };
        ScanFeature scan_asym[4] = {
            make_line(0.0f, -0.8f, 1.0f, 0.8f, 1.0f),
            make_line(0.0f, -0.8f, -1.0f, 0.8f, -1.0f),
            make_line(0.0f, -0.8f, 2.0f, 0.8f, 2.0f),
            make_arc(0.5f, 0.5f, 0.3f),
        };
        CoarseResult r = correlative_search(scan_asym, 4, map_asym, 4, 0.0f, 0.0f, 0.0f, CORR_ENTRY_STATIC);
        assert(r.valid);
        assert(!r.ambiguous);
        printf("  T6 PASS  arc anchors translation -- ambiguity resolved "
               "(best=%.3f second=%.3f)\n", r.score, r.second_score);
    }

    /* ---- T7: fast motion beyond layer-1 window, within layer-2 reach - */
    {
        float true_dx7 = 0.22f, true_dy7 = 0.05f, true_dtheta7 = 25.0f * PI_F / 180.0f;
        assert(hypotf(true_dx7, true_dy7) > CORR_SEARCH_DXY_MAX_M
               || fabsf(true_dtheta7) > CORR_SEARCH_DTHETA_MAX_RAD);
        assert(hypotf(true_dx7, true_dy7) <= CORR_MAX_WALK_DXY_M
               && fabsf(true_dtheta7) <= CORR_MAX_WALK_DTHETA_RAD);

        CorrMapEntry map_e7[4] = {
            map_e[0], map_e[1], map_e[2],
            make_map_arc(0.3f, 1.6f, 0.4f),
        };
        ScanFeature scan_e7_base[4] = {
            base_scan[0], base_scan[1], base_scan[2],
            make_arc(0.3f, 1.6f, 0.4f),
        };
        ScanFeature fast_scan7[4];
        for (int i = 0; i < 3; i++) fast_scan7[i] = shift_line(scan_e7_base[i], true_dx7, true_dy7, true_dtheta7);
        fast_scan7[3] = shift_arc(scan_e7_base[3], true_dx7, true_dy7, true_dtheta7);

        CoarseResult r = correlative_search(fast_scan7, 4, map_e7, 4, 0.0f, 0.0f, 0.0f, CORR_ENTRY_STATIC);
        assert(r.valid);
        assert(fabsf(r.dx - true_dx7) < 0.04f);
        assert(fabsf(r.dy - true_dy7) < 0.04f);
        assert(fabsf(r.dtheta - true_dtheta7) < (4.0f * PI_F / 180.0f));
        printf("  T7 PASS  fast motion beyond layer-1 window recovered via "
               "layer-2 reach (dx=%.3f dy=%.3f dtheta=%.1fdeg)\n",
               r.dx, r.dy, r.dtheta * 180.0f / PI_F);
    }

    printf("\nAll tests passed.\n");
    return 0;
}
