/* bench_correlative_standalone.c
 * Pure-C, no-Python benchmark of correlative_search() -- the lowest-level,
 * most honest number available for "does the C port alone clear the
 * 100ms/10Hz budget on real Cortex-A53 hardware". No ctypes, no
 * marshalling, no interpreter -- just the algorithm, on the real chip. */
#define _POSIX_C_SOURCE 199309L
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>
#include <string.h>
#include "correlative_match.h"

static ScanFeature mk_line(float angle, float x1, float y1, float x2, float y2) {
    ScanFeature f; memset(&f, 0, sizeof f);
    f.type = FEAT_LINE; f.angle = angle;
    f.cx = x1; f.cy = y1; f.t_start = x2; f.t_end = y2;
    f.length = hypotf(x2 - x1, y2 - y1); f.quality = 100;
    return f;
}

static double now_ms(void) {
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return t.tv_sec * 1000.0 + t.tv_nsec / 1e6;
}

int main(int argc, char **argv) {
    int n_static = (argc > 1) ? atoi(argv[1]) : 60;
    int n_runs = (argc > 2) ? atoi(argv[2]) : 200;
    unsigned seed = 42;

    if (n_static > 500) n_static = 500;
    CorrMapEntry map[500];
    srand(seed);
    for (int i = 0; i < n_static; i++) {
        float angles[3] = {0.0f, 1.5708f, (float)(rand() % 628) / 100.0f - 3.14f};
        map[i].angle = angles[rand() % 3];
        map[i].distance = (float)(rand() % 500) / 100.0f;
        map[i].mx = (float)(rand() % 1000) / 100.0f - 5.0f;
        map[i].my = (float)(rand() % 1000) / 100.0f - 5.0f;
        map[i].status = CORR_ENTRY_STATIC;
        map[i].active = true;
    }

    ScanFeature scan[27];
    float base_walls[4][2] = {{0.0f,1.0f},{1.5708f,1.6f},{-1.047f,1.2f},{0.5236f,0.9f}};
    for (int i = 0; i < 27; i++) {
        int w = rand() % 4;
        float a = base_walls[w][0], d = base_walls[w][1];
        float dirx = cosf(a), diry = sinf(a);
        float fx = -sinf(a) * d, fy = cosf(a) * d;
        scan[i] = mk_line(a, fx - 0.4f * dirx, fy - 0.4f * diry, fx + 0.4f * dirx, fy + 0.4f * diry);
    }

    /* warm up (page faults, cache) before timing */
    for (int i = 0; i < 5; i++) correlative_search(scan, 27, map, n_static, 0.3f, -0.2f, 0.05f, CORR_ENTRY_STATIC);

    double t0 = now_ms();
    for (int i = 0; i < n_runs; i++) {
        CoarseResult r = correlative_search(scan, 27, map, n_static, 0.3f, -0.2f, 0.05f, CORR_ENTRY_STATIC);
        (void)r;
    }
    double t1 = now_ms();

    printf("n_static=%d  n_runs=%d  avg=%.3f ms/call  (%.1f%% of 100ms/10Hz budget)\n",
           n_static, n_runs, (t1 - t0) / n_runs, 100.0 * ((t1 - t0) / n_runs) / 100.0);
    return 0;
}
