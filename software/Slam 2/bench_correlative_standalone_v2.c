/* bench_correlative_standalone_v2.c
 * Fixes vs v1:
 *  - Map and scan now use INDEPENDENT, FIXED seeds -- v1 let scan generation
 *    continue the same rand() stream after map generation, so different
 *    n_static values were accidentally testing different scan data too,
 *    not just different map sizes. That alone can explain some of the
 *    non-monotonic behavior observed (n_static=60 slower than n_static=200).
 *  - Reports min/max/mean/median/p95/stddev across ALL individual calls,
 *    not just one averaged number -- a single average hides exactly the
 *    kind of run-to-run jitter that matters for a 10Hz real-time deadline.
 *  - Best-effort read of the current cpufreq governor and clock, so a
 *    frequency-scaling explanation for variance can be checked directly
 *    instead of guessed at.
 */
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

static void print_sys_file(const char *label, const char *path) {
    FILE *f = fopen(path, "r");
    if (!f) { printf("  %-28s (unreadable: %s)\n", label, path); return; }
    char buf[128];
    if (fgets(buf, sizeof buf, f)) {
        buf[strcspn(buf, "\n")] = 0;
        printf("  %-28s %s\n", label, buf);
    }
    fclose(f);
}

static int cmp_double(const void *a, const void *b) {
    double da = *(const double *)a, db = *(const double *)b;
    return (da > db) - (da < db);
}

int main(int argc, char **argv) {
    int n_static = (argc > 1) ? atoi(argv[1]) : 60;
    int n_runs   = (argc > 2) ? atoi(argv[2]) : 300;
    if (n_static > 500) n_static = 500;
    if (n_runs < 1) n_runs = 1;

    printf("System state before benchmark:\n");
    print_sys_file("cpufreq governor:", "/sys/devices/system/cpu/cpu0/cpufreq/scaling_governor");
    print_sys_file("cpufreq cur_freq (kHz):", "/sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq");
    print_sys_file("cpufreq max_freq (kHz):", "/sys/devices/system/cpu/cpu0/cpufreq/cpuinfo_max_freq");
    print_sys_file("thermal zone0 temp (m°C):", "/sys/class/thermal/thermal_zone0/temp");
    printf("\n");

    /* MAP: fixed seed, independent of n_runs/n_static choice -- the first
       n_static entries generated here are IDENTICAL regardless of how many
       total entries you ask for (sequential draws from the same seed), so
       runs at different n_static remain a fair "same scenario, more map"
       comparison for the shared prefix. */
    CorrMapEntry map[500];
    srand(42);
    for (int i = 0; i < n_static; i++) {
        float angles[3] = {0.0f, 1.5708f, (float)(rand() % 628) / 100.0f - 3.14f};
        map[i].angle = angles[rand() % 3];
        map[i].distance = (float)(rand() % 500) / 100.0f;
        map[i].mx = (float)(rand() % 1000) / 100.0f - 5.0f;
        map[i].my = (float)(rand() % 1000) / 100.0f - 5.0f;
        map[i].status = CORR_ENTRY_STATIC;
        map[i].active = true;
    }

    /* SCAN: SEPARATE fixed seed -- FIX vs v1. Scan is now byte-identical
       across every n_static value tested, so n_static is the ONLY thing
       that varies between runs. */
    srand(777);
    ScanFeature scan[27];
    float base_walls[4][2] = {{0.0f,1.0f},{1.5708f,1.6f},{-1.047f,1.2f},{0.5236f,0.9f}};
    for (int i = 0; i < 27; i++) {
        int w = rand() % 4;
        float a = base_walls[w][0], d = base_walls[w][1];
        float dirx = cosf(a), diry = sinf(a);
        float fx = -sinf(a) * d, fy = cosf(a) * d;
        scan[i] = mk_line(a, fx - 0.4f * dirx, fy - 0.4f * diry, fx + 0.4f * dirx, fy + 0.4f * diry);
    }

    /* warm up -- first few calls on an embedded board can be slower
       (icache cold, branch predictor cold, cpufreq governor still ramping
       up from idle) -- excluded from the reported statistics. */
    for (int i = 0; i < 10; i++) {
        CoarseResult r = correlative_search(scan, 27, map, n_static, 0.3f, -0.2f, 0.05f, CORR_ENTRY_STATIC);
        (void)r;
    }

    double *times = malloc(sizeof(double) * (size_t)n_runs);
    for (int i = 0; i < n_runs; i++) {
        double t0 = now_ms();
        CoarseResult r = correlative_search(scan, 27, map, n_static, 0.3f, -0.2f, 0.05f, CORR_ENTRY_STATIC);
        double t1 = now_ms();
        (void)r;
        times[i] = t1 - t0;
    }

    double sum = 0.0, sumsq = 0.0;
    for (int i = 0; i < n_runs; i++) { sum += times[i]; sumsq += times[i] * times[i]; }
    double mean = sum / n_runs;
    double variance = sumsq / n_runs - mean * mean;
    double stddev = variance > 0 ? sqrt(variance) : 0.0;

    double *sorted = malloc(sizeof(double) * (size_t)n_runs);
    memcpy(sorted, times, sizeof(double) * (size_t)n_runs);
    qsort(sorted, n_runs, sizeof(double), cmp_double);
    double median = sorted[n_runs / 2];
    double p95 = sorted[(int)(n_runs * 0.95)];
    double min = sorted[0];
    double max = sorted[n_runs - 1];

    printf("System state after benchmark:\n");
    print_sys_file("cpufreq cur_freq (kHz):", "/sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq");
    print_sys_file("thermal zone0 temp (m°C):", "/sys/class/thermal/thermal_zone0/temp");
    printf("\n");

    printf("n_static=%d  n_runs=%d\n", n_static, n_runs);
    printf("  mean   = %8.3f ms  (%.1f%% of 100ms budget)\n", mean, mean);
    printf("  median = %8.3f ms\n", median);
    printf("  min    = %8.3f ms\n", min);
    printf("  max    = %8.3f ms\n", max);
    printf("  p95    = %8.3f ms\n", p95);
    printf("  stddev = %8.3f ms  (%.0f%% of mean -- high = jitter, not just slow)\n",
           stddev, 100.0 * stddev / mean);

    free(times);
    free(sorted);
    return 0;
}
