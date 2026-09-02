/**
 * fit_first.h
 *
 * Public header for the fit-first 2D LiDAR feature extractor.
 * Shared between PC (libfit_first.so) and STM32F411 firmware.
 *
 * Build (PC):
 *   gcc -O2 -ffast-math -shared -fPIC -o libfit_first.so fit_first.c -lm
 *
 * Build (STM32 — later):
 *   arm-none-eabi-gcc -O2 -ffast-math -mfpu=fpv4-sp-d16 -mfloat-abi=hard \
 *       -c fit_first.c -o fit_first.o
 */

#ifndef FIT_FIRST_H
#define FIT_FIRST_H

#include <stdint.h>

/* ── Tunable algorithm constants ────────────────────────────────────────────
 *
 * These match the Python lidar_visualizer.py defaults exactly.
 * Change here only — both PC and STM32 pick them up automatically.
 */
#define MIN_PTS         5        /* min points to attempt any fit            */
#define MIN_LEN_M       0.15f    /* m  — discard features shorter than this  */
#define MAX_FEATURES    50       /* max features written per scan             */
#define GAP_THRESH_M    0.15f    /* m  — cartesian jump => new surface        */
#define LINE_TOLERANCE  0.04f    /* m  — max perpendicular deviation for line */
#define ARC_TOLERANCE   0.03f    /* m  — max radial deviation for arc         */
#define MAX_DEPTH       6        /* hard recursion depth limit                */

/* Radius sanity bounds for circle fits */
#define ARC_R_MAX       2.0f     /* m — above this is effectively a flat wall */
#define ARC_R_MIN       0.03f    /* m — below this is a noise spike           */

/* ── Data structures (must match messages.h on STM32) ──────────────────────
 *
 * Layout is fully explicit: no bitfields, no padding surprises.
 * Verified identical to the messages.h layout specification.
 */
typedef struct {
    uint8_t  type;          /* 0 = line, 1 = arc                             */

    /* Line fields (type == 0) */
    float    angle;         /* Hough angle [-pi/2, pi/2] radians             */
    float    distance;      /* perpendicular distance from scan origin (m)   */
    float    t_start;       /* start limit along line direction (m)          */
    float    t_end;         /* end   limit along line direction (m)          */

    /* Arc fields (type == 1) */
    float    cx;            /* arc centre x (m)                              */
    float    cy;            /* arc centre y (m)                              */
    float    r;             /* arc radius  (m)                               */
    float    theta_start;   /* arc start angle (rad)                         */
    float    theta_end;     /* arc end   angle (rad)                         */

    /* Common */
    float    length;        /* feature arc/chord length (m)                  */
    uint8_t  quality;       /* confidence 0-100, set to n_pts clamped to 100 */
} Feature;

typedef struct {
    uint32_t timestamp_ms;
    uint8_t  count;
    Feature  features[MAX_FEATURES];
} FeaturePacket;

/* ── Intermediate line-fit result (internal, exposed for testing) ───────────
 *
 * Not used by STM32 callers — only needed by the C implementation and
 * the PC test harness.  Kept in the header so test_fit.c can inspect it.
 */
typedef struct {
    float angle;        /* Hough angle                    */
    float distance;     /* perpendicular distance          */
    float x1, y1;       /* start endpoint on fitted line   */
    float x2, y2;       /* end   endpoint on fitted line   */
    float max_dev;      /* worst perpendicular deviation   */
} LineFitResult;

/* ── Intermediate arc-fit result (internal, exposed for testing) ────────── */
typedef struct {
    float cx, cy;       /* circle centre                   */
    float r;            /* radius                          */
    float max_dev;      /* worst radial deviation          */
} ArcFitResult;

/* ── Public API ─────────────────────────────────────────────────────────────
 *
 * fit_first_extract()
 *   Gap-detects continuous surfaces in the input point cloud, then
 *   recursively fits line/arc primitives to each surface.
 *
 *   xs, ys   — input Cartesian coordinates (metres), ordered by scan angle
 *   n_pts    — number of input points (xs and ys must each have n_pts elements)
 *   out      — caller-allocated Feature array; must hold at least max_out items
 *   max_out  — maximum number of features to write (use MAX_FEATURES)
 *
 *   Returns the number of Feature structs written to out (0 … max_out).
 */
int fit_first_extract(
    const float *xs,
    const float *ys,
    int          n_pts,
    Feature     *out,
    int          max_out
);

#endif /* FIT_FIRST_H */
