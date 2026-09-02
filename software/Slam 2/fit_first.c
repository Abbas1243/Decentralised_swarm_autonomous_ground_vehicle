/**
 * fit_first.c
 *
 * Fit-first 2D LiDAR feature extractor.
 * Direct C translation of the Python reference in lidar_visualizer.py.
 *
 * Compile as shared library (PC):
 *   gcc -O2 -ffast-math -shared -fPIC -o libfit_first.so fit_first.c -lm
 *
 * Compile as object for STM32 (later):
 *   arm-none-eabi-gcc -O2 -ffast-math -mfpu=fpv4-sp-d16 -mfloat-abi=hard \
 *       -c fit_first.c -o fit_first.o
 *
 * Constraints honoured:
 *   - No dynamic memory allocation (all arrays stack or caller-provided)
 *   - No external libraries beyond <math.h>, <stdint.h>, <string.h>
 *   - All floating point as float (STM32 FPU is single-precision)
 *   - Compiles clean -Wall -Wextra on both gcc and arm-none-eabi-gcc
 *   - Recursion hard-capped at MAX_DEPTH
 */

#include "fit_first.h"

#include <math.h>
#include <stdint.h>
#include <string.h>

/* ── Internal helpers ───────────────────────────────────────────────────── */

/**
 * fabsf_inline — keep a single call site so the STM32 ABS instruction
 * can be inlined by the compiler without pulling in a full libm call.
 */
static inline float _absf(float x) { return x < 0.0f ? -x : x; }

/* ── _fit_line ───────────────────────────────────────────────────────────
 *
 * Total least-squares (orthogonal regression) line through pts[s..e].
 *
 * Fills *out on success and returns 1.
 * Returns 0 if the segment has fewer than 2 points.
 *
 * Matches Python _fit_line() exactly:
 *   - Computes centroid (mx, my)
 *   - Builds scatter matrix Sxx, Syy, Sxy
 *   - Finds dominant eigenvector (dx, dy) of the 2×2 scatter matrix
 *   - Derives Hough (angle, distance) from the normal direction
 *   - Projects first and last points onto the line for endpoints
 *   - Returns worst perpendicular deviation over all points
 */
static int _fit_line(
        const float *xs, const float *ys,
        int s, int e,
        LineFitResult *out)
{
    int n = e - s + 1;
    if (n < 2)
        return 0;

    /* ── Centroid ── */
    float mx = 0.0f, my = 0.0f;
    for (int i = s; i <= e; i++) {
        mx += xs[i];
        my += ys[i];
    }
    mx /= (float)n;
    my /= (float)n;

    /* ── Scatter matrix ── */
    float Sxx = 0.0f, Syy = 0.0f, Sxy = 0.0f;
    for (int i = s; i <= e; i++) {
        float dx = xs[i] - mx;
        float dy = ys[i] - my;
        Sxx += dx * dx;
        Syy += dy * dy;
        Sxy += dx * dy;
    }

    /* ── Dominant eigenvector (dx, dy) of the scatter matrix ──
     *
     * The 2×2 symmetric scatter matrix is:
     *   [ Sxx  Sxy ]
     *   [ Sxy  Syy ]
     *
     * Eigenvalues: λ = ((Sxx+Syy) ± sqrt((Sxx-Syy)²+4Sxy²)) / 2
     * Dominant eigenvector corresponds to λ_max.
     *
     * Special case: if |Sxy| < eps the matrix is already diagonal,
     * eigenvectors are the coordinate axes — pick the longer axis.
     */
    float line_dx, line_dy;

    if (_absf(Sxy) < 1e-12f) {
        if (Sxx >= Syy) {
            line_dx = 1.0f; line_dy = 0.0f;
        } else {
            line_dx = 0.0f; line_dy = 1.0f;
        }
    } else {
        float diff    = Sxx - Syy;
        float hyp     = sqrtf(diff * diff + 4.0f * Sxy * Sxy);
        float lam_max = (Sxx + Syy + hyp) * 0.5f;
        line_dx = Sxy;
        line_dy = lam_max - Sxx;
        float L = sqrtf(line_dx * line_dx + line_dy * line_dy);
        line_dx /= L;
        line_dy /= L;
    }

    /* ── Hough parameterisation ──
     *
     * angle = atan2(dy, dx), wrapped to [-pi/2, pi/2]
     * normal = (-sin(angle), cos(angle))
     * distance = dot(normal, centroid)
     */
    float angle = atan2f(line_dy, line_dx);
    if (angle >  (float)(M_PI / 2.0)) angle -= (float)M_PI;
    if (angle < -(float)(M_PI / 2.0)) angle += (float)M_PI;

    float nx = -sinf(angle);
    float ny =  cosf(angle);
    float distance = nx * mx + ny * my;

    /* ── Max perpendicular deviation ── */
    float max_dev = 0.0f;
    for (int i = s; i <= e; i++) {
        float dev = _absf(nx * (xs[i] - mx) + ny * (ys[i] - my));
        if (dev > max_dev)
            max_dev = dev;
    }

    /* ── Projected endpoints: walk t = dot(pt - centroid, dir) ── */
    float tmin =  1e9f;
    float tmax = -1e9f;
    for (int i = s; i <= e; i++) {
        float t = (xs[i] - mx) * line_dx + (ys[i] - my) * line_dy;
        if (t < tmin) tmin = t;
        if (t > tmax) tmax = t;
    }

    out->angle    = angle;
    out->distance = distance;
    out->x1       = mx + tmin * line_dx;
    out->y1       = my + tmin * line_dy;
    out->x2       = mx + tmax * line_dx;
    out->y2       = my + tmax * line_dy;
    out->max_dev  = max_dev;

    return 1;
}


/* ── _fit_circle ────────────────────────────────────────────────────────
 *
 * 3-point circle fit using first, middle, and last point of pts[s..e].
 *
 * Fills *out on success and returns 1.
 * Returns 0 if:
 *   - points are collinear (|d| < eps — no unique circle exists)
 *   - fitted radius is outside [ARC_R_MIN, ARC_R_MAX]
 *
 * Matches Python _fit_circle() exactly.
 */
static int _fit_circle(
        const float *xs, const float *ys,
        int s, int e,
        ArcFitResult *out)
{
    float ax = xs[s],          ay = ys[s];
    float bx = xs[(s + e) / 2], by = ys[(s + e) / 2];
    float cx = xs[e],          cy = ys[e];

    /* Denominator of the circumcircle formula */
    float d = 2.0f * (ax * (by - cy) + bx * (cy - ay) + cx * (ay - by));
    if (_absf(d) < 1e-9f)
        return 0;   /* collinear — no circle */

    float a2 = ax * ax + ay * ay;
    float b2 = bx * bx + by * by;
    float c2 = cx * cx + cy * cy;

    float ux = (a2 * (by - cy) + b2 * (cy - ay) + c2 * (ay - by)) / d;
    float uy = (a2 * (cx - bx) + b2 * (ax - cx) + c2 * (bx - ax)) / d;

    float r = sqrtf((ax - ux) * (ax - ux) + (ay - uy) * (ay - uy));

    /* Radius sanity check — same limits as Python */
    if (r > ARC_R_MAX || r < ARC_R_MIN)
        return 0;

    /* ── Max radial deviation ── */
    float max_dev = 0.0f;
    for (int i = s; i <= e; i++) {
        float rad = sqrtf((xs[i] - ux) * (xs[i] - ux) +
                          (ys[i] - uy) * (ys[i] - uy));
        float dev = _absf(rad - r);
        if (dev > max_dev)
            max_dev = dev;
    }

    out->cx      = ux;
    out->cy      = uy;
    out->r       = r;
    out->max_dev = max_dev;

    return 1;
}


/* ── _fit_first (recursive) ─────────────────────────────────────────────
 *
 * Tries to fit pts[s..e] as a single line, then as a single arc.
 * If both fail, splits at the midpoint and recurses on each half.
 *
 * out     — caller-allocated Feature array
 * n_out   — pointer to current write index (incremented on each accepted fit)
 * max_out — hard ceiling on features written
 * depth   — current recursion depth; capped at MAX_DEPTH
 *
 * Matches Python _fit_first() exactly, including:
 *   - minimum segment length check (MIN_LEN_M)
 *   - arc angular length computed as r * |theta_end - theta_start|
 *   - quality = n_pts clamped to 100
 *   - mid = (s + e) / 2  (integer — same floor behaviour as Python // )
 */
static void _fit_first_recursive(
        const float *xs, const float *ys,
        int s, int e,
        Feature *out, int *n_out, int max_out,
        int depth)
{
    /* Guard: too few points or recursion limit reached */
    if ((e - s) < (MIN_PTS - 1))
        return;
    if (depth > MAX_DEPTH)
        return;
    if (*n_out >= max_out)
        return;

    int n_pts = e - s + 1;

    /* ── Try line ─────────────────────────────────────────────────────── */
    {
        LineFitResult lr;
        if (_fit_line(xs, ys, s, e, &lr)) {
            float length = sqrtf((lr.x2 - lr.x1) * (lr.x2 - lr.x1) +
                                 (lr.y2 - lr.y1) * (lr.y2 - lr.y1));
            if (lr.max_dev < LINE_TOLERANCE && length >= MIN_LEN_M) {
                Feature *f = &out[*n_out];
                memset(f, 0, sizeof(Feature));

                f->type     = 0;
                f->angle    = lr.angle;
                f->distance = lr.distance;

                /* Store absolute endpoints directly in cx/cy (x1/y1)
                 * and t_start/t_end (x2/y2).  These fields are unused
                 * for lines on the STM32 side and give the Python wrapper
                 * exact x1/y1/x2/y2 without any reconstruction error. */
                f->cx       = lr.x1;   /* x1 */
                f->cy       = lr.y1;   /* y1 */
                f->t_start  = lr.x2;   /* x2 */
                f->t_end    = lr.y2;   /* y2 */

                f->length   = length;
                f->quality  = (uint8_t)(n_pts > 100 ? 100 : n_pts);

                (*n_out)++;
                return;
            }
        }
    }

    /* ── Try arc ──────────────────────────────────────────────────────── */
    {
        ArcFitResult ar;
        if (_fit_circle(xs, ys, s, e, &ar)) {
            if (ar.max_dev < ARC_TOLERANCE) {
                float theta_start = atan2f(ys[s] - ar.cy, xs[s] - ar.cx);
                float theta_end   = atan2f(ys[e] - ar.cy, xs[e] - ar.cx);
                float arc_len     = ar.r * _absf(theta_end - theta_start);
                if (arc_len >= MIN_LEN_M) {
                    Feature *f = &out[*n_out];
                    memset(f, 0, sizeof(Feature));

                    f->type        = 1;
                    f->cx          = ar.cx;
                    f->cy          = ar.cy;
                    f->r           = ar.r;
                    f->theta_start = theta_start;
                    f->theta_end   = theta_end;
                    f->length      = arc_len;
                    f->quality     = (uint8_t)(n_pts > 100 ? 100 : n_pts);

                    (*n_out)++;
                    return;
                }
            }
        }
    }

    /* ── Neither fit — split in half and recurse ─────────────────────── */
    int mid = (s + e) / 2;
    _fit_first_recursive(xs, ys, s,   mid, out, n_out, max_out, depth + 1);
    _fit_first_recursive(xs, ys, mid, e,   out, n_out, max_out, depth + 1);
}


/* ── fit_first_extract (public entry point) ─────────────────────────────
 *
 * Gap detection: walk the ordered point array; if the Euclidean distance
 * between consecutive points exceeds GAP_THRESH_M, the current surface
 * ends and a new one begins.  Each continuous surface of >= MIN_PTS
 * points is passed to _fit_first_recursive.
 *
 * Matches split_merge() gap-detection logic in Python exactly:
 *   - gap checked as hypot(dx, dy) > GAP_THRESH
 *   - surface must have >= MIN_PTS points to be attempted
 *   - stops adding features once max_out is reached
 */
int fit_first_extract(
        const float *xs, const float *ys,
        int n_pts,
        Feature *out, int max_out)
{
    if (n_pts < MIN_PTS || out == NULL || max_out <= 0)
        return 0;

    int n_out   = 0;
    int seg_start = 0;

    for (int i = 1; i < n_pts; i++) {
        float dx = xs[i] - xs[i - 1];
        float dy = ys[i] - ys[i - 1];
        float gap = sqrtf(dx * dx + dy * dy);

        if (gap > GAP_THRESH_M) {
            /* End of current surface */
            int seg_len = i - seg_start;
            if (seg_len >= MIN_PTS) {
                _fit_first_recursive(xs, ys,
                                     seg_start, i - 1,
                                     out, &n_out, max_out, 0);
            }
            seg_start = i;

            if (n_out >= max_out)
                return n_out;
        }
    }

    /* Final surface (no trailing gap) */
    int seg_len = n_pts - seg_start;
    if (seg_len >= MIN_PTS && n_out < max_out) {
        _fit_first_recursive(xs, ys,
                             seg_start, n_pts - 1,
                             out, &n_out, max_out, 0);
    }

    return n_out;
}
