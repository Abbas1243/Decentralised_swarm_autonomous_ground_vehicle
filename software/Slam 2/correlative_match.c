/*
 * correlative_match.c
 * =====================
 * See correlative_match.h and correlative_match.py for the full algorithm
 * rationale. This is a direct, scalar, no-malloc translation of the
 * Python reference's SCALAR path (_score_candidate / _coarse_sweep_topk /
 * _greedy_refine before vectorization) — every array here is fixed-size
 * and stack-allocated, sized from the CORR_MAX_* constants in the header.
 *
 * No Eigen/OpenCV, no dynamic allocation, float throughout (STM32/A53 FPU
 * is single precision) — same constraints as every other module in
 * slam_core/.
 */

#include "correlative_match.h"

#include <math.h>
#include <string.h>

CorrDiagnostics corr_last_diagnostics = {0, 0, 0};

/* =========================================================================
 * Constants / small helpers
 * ========================================================================= */

#define CORR_PI     3.14159265358979323846f
#define CORR_PI_2   1.57079632679489661923f

static inline float corr_wrap_line_angle(float angle) {
    /* Wrap into [-pi/2, pi/2] — same Hough convention as line_matcher.py /
       map_manager.py / pose_estimator.py / slam.py. */
    if (angle > CORR_PI_2) {
        angle -= CORR_PI;
    } else if (angle < -CORR_PI_2) {
        angle += CORR_PI;
    }
    return angle;
}

static inline float corr_angle_diff_line(float a, float b) {
    /* PI-symmetric angular difference, result in [0, pi/2] — same as
       line_matcher._angle_diff_line / correlative_match.py's version. */
    float diff = fabsf(a - b);
    diff = fmodf(diff, CORR_PI);
    if (diff > CORR_PI_2) {
        diff = CORR_PI - diff;
    }
    return diff;
}

static inline bool corr_within_walk_bounds(float dx, float dy, float dtheta) {
    /* (dx, dy, dtheta) here are offsets from the ORIGINAL guess passed
       into correlative_search(), not from wherever a walk round started —
       see _greedy_refine's docstring in the Python source for why the
       cap must be measured from the original guess. */
    return fabsf(dtheta) <= CORR_MAX_WALK_DTHETA_RAD
        && hypotf(dx, dy) <= CORR_MAX_WALK_DXY_M;
}

/* =========================================================================
 * Static map preparation — nearest-N selection, no malloc
 *
 * Selects the CORR_MAX_STATIC_ENTRIES_SEARCHED active STATIC map entries
 * nearest (guess_x, guess_y) via a fixed-size insertion-sorted "best N"
 * array (O(n_map * cap), cap is small (70) so this is cheap even at
 * n_map = MAX_MAP_ENTRIES = 500) — no need to copy/sort the whole map.
 * A STATIC entry far from the guess contributes almost nothing to the
 * Gaussian soft-score anyway (SIGMA_DIST_M/SIGMA_CENTRE_M ~ 0.15m
 * falloff), so this cap loses negligible real signal — same reasoning
 * as correlative_match.py's _prepare_static_maps / MAX_STATIC_ENTRIES_
 * SEARCHED.
 * ========================================================================= */

typedef struct { float angle, distance; } CorrStaticLine;
typedef struct { float mx, my, r; } CorrStaticArc;

static void corr_prepare_static_maps(
    const CorrMapEntry *map_entries, int n_map, uint8_t static_status,
    float guess_x, float guess_y,
    CorrStaticLine *out_lines, int *n_lines,
    CorrStaticArc *out_arcs, int *n_arcs)
{
    typedef struct { int idx; float d2; } CandDist;
    CandDist best[CORR_MAX_STATIC_ENTRIES_SEARCHED];
    int n_best = 0;

    for (int i = 0; i < n_map; i++) {
        const CorrMapEntry *e = &map_entries[i];
        if (!e->active || e->status != static_status) {
            continue;
        }
        float dx = e->mx - guess_x;
        float dy = e->my - guess_y;
        float d2 = dx * dx + dy * dy;

        if (n_best < CORR_MAX_STATIC_ENTRIES_SEARCHED) {
            int j = n_best;
            while (j > 0 && best[j - 1].d2 > d2) {
                best[j] = best[j - 1];
                j--;
            }
            best[j].idx = i;
            best[j].d2 = d2;
            n_best++;
        } else if (d2 < best[n_best - 1].d2) {
            int j = n_best - 1;
            while (j > 0 && best[j - 1].d2 > d2) {
                best[j] = best[j - 1];
                j--;
            }
            best[j].idx = i;
            best[j].d2 = d2;
        }
    }

    *n_lines = 0;
    *n_arcs = 0;
    for (int k = 0; k < n_best; k++) {
        const CorrMapEntry *e = &map_entries[best[k].idx];
        if (corr_map_entry_is_arc(e)) {
            if (*n_arcs < CORR_MAX_STATIC_ENTRIES_SEARCHED) {
                out_arcs[*n_arcs].mx = e->mx;
                out_arcs[*n_arcs].my = e->my;
                out_arcs[*n_arcs].r = e->distance;   /* radius stored in .distance */
                (*n_arcs)++;
            }
        } else {
            if (*n_lines < CORR_MAX_STATIC_ENTRIES_SEARCHED) {
                out_lines[*n_lines].angle = e->angle;
                out_lines[*n_lines].distance = e->distance;
                (*n_lines)++;
            }
        }
    }
}

/* =========================================================================
 * Per-dtheta feature rotation — rotate every scan feature ONCE per dtheta
 * step, reused across the whole dx,dy sub-grid at that dtheta (same cost
 * structure as the Python reference's _prerotate_features).
 * ========================================================================= */

typedef struct {
    float rmx, rmy;      /* rotated (not yet translated) midpoint */
    float angle;          /* guess_theta + dtheta, wrapped */
    float nx, ny;          /* Hough unit normal at `angle` */
    float base_dist;       /* nx*rmx + ny*rmy — dx,dy added by the caller */
} CorrRotatedLine;

typedef struct {
    float rcx, rcy;   /* rotated (not yet translated) centre */
    float r;
} CorrRotatedArc;

static void corr_prerotate_features(
    const ScanFeature *scan_features, int n_scan,
    float cos_t, float sin_t, float total_theta,
    CorrRotatedLine *out_lines, int *n_lines,
    CorrRotatedArc *out_arcs, int *n_arcs)
{
    *n_lines = 0;
    *n_arcs = 0;

    for (int i = 0; i < n_scan; i++) {
        const ScanFeature *f = &scan_features[i];

        if (f->type == FEAT_LINE) {
            float x1 = FEAT_LINE_X1(*f), y1 = FEAT_LINE_Y1(*f);
            float x2 = FEAT_LINE_X2(*f), y2 = FEAT_LINE_Y2(*f);
            float rx1 = x1 * cos_t - y1 * sin_t;
            float ry1 = y1 * cos_t + x1 * sin_t;
            float rx2 = x2 * cos_t - y2 * sin_t;
            float ry2 = y2 * cos_t + x2 * sin_t;
            float rmx = (rx1 + rx2) * 0.5f;
            float rmy = (ry1 + ry2) * 0.5f;
            float angle = corr_wrap_line_angle(f->angle + total_theta);
            float nx = -sinf(angle), ny = cosf(angle);
            float base_dist = nx * rmx + ny * rmy;

            if (*n_lines < CORR_MAX_SCAN_FEATURES) {
                out_lines[*n_lines].rmx = rmx;
                out_lines[*n_lines].rmy = rmy;
                out_lines[*n_lines].angle = angle;
                out_lines[*n_lines].nx = nx;
                out_lines[*n_lines].ny = ny;
                out_lines[*n_lines].base_dist = base_dist;
                (*n_lines)++;
            }
        } else if (f->type == FEAT_ARC) {
            float rcx = f->cx * cos_t - f->cy * sin_t;
            float rcy = f->cy * cos_t + f->cx * sin_t;
            if (*n_arcs < CORR_MAX_SCAN_FEATURES) {
                out_arcs[*n_arcs].rcx = rcx;
                out_arcs[*n_arcs].rcy = rcy;
                out_arcs[*n_arcs].r = f->r;
                (*n_arcs)++;
            }
        }
        /* unknown type — skipped, mirrors the Python reference */
    }
}

/* =========================================================================
 * Scoring — one (dx, dy) candidate at the dtheta already baked into the
 * rotated feature arrays. Direct 1:1 port of _score_candidate (the
 * SCALAR reference in correlative_match.py, not the numpy-vectorized
 * path — see that file's docstring for why this is the shape to mirror).
 * ========================================================================= */

static void corr_score_candidate(
    const CorrRotatedLine *lines, int n_lines,
    const CorrRotatedArc *arcs, int n_arcs,
    float dx, float dy,
    const CorrStaticLine *slines, int n_slines,
    const CorrStaticArc *sarcs, int n_sarcs,
    float *out_score, int *out_n_scored)
{
    float total = 0.0f;
    int n_scored = 0;

    /* PERFORMANCE NOTE (found by profiling correlative_match.py — see
       slam_progress_correlative_match_c_port.md's follow-up entry):
       exp(-a) * exp(-b) == exp(-(a+b)) exactly (standard exponent
       identity, not an approximation) — combining the two Gaussian
       falloff terms into ONE expf() call instead of two removes half of
       this loop's transcendental-function calls, which is where nearly
       all of its cost is (a multiply/add is negligible next to a libm
       expf() call). Zero behavioural change, only reduces call count. */

    for (int i = 0; i < n_lines; i++) {
        float dist = lines[i].base_dist + lines[i].nx * dx + lines[i].ny * dy;
        float best_q = 0.0f;

        for (int j = 0; j < n_slines; j++) {
            float adiff = corr_angle_diff_line(lines[i].angle, slines[j].angle);

            /* EARLY-EXIT PREFILTER: beyond ~3 sigma the angle term alone
               already makes q negligible (exp(-9) ~ 1.2e-4) regardless of
               ddiff — skip the distance computation and the expf() call
               entirely for angularly-mismatched static lines. This is
               the same angle-bucket prefilter soft_scan_matcher.py's own
               docstring flags as "the first optimization to reach for"
               if profiling ever shows it's needed — profiling of the
               Python reference (see progress notes) showed it is. */
            if (adiff > 3.0f * CORR_SIGMA_ANGLE_RAD) {
                continue;
            }

            float d1 = fabsf(dist - slines[j].distance);
            float d2 = fabsf(dist + slines[j].distance);
            float ddiff = (d1 < d2) ? d1 : d2;   /* sign-aware, Hough distance
                                                     can be negative */
            float na = adiff / CORR_SIGMA_ANGLE_RAD;
            float nd = ddiff / CORR_SIGMA_DIST_M;
            float q = expf(-(na * na + nd * nd));
            if (q > best_q) {
                best_q = q;
            }
        }
        total += best_q;
        n_scored++;
    }

    for (int i = 0; i < n_arcs; i++) {
        float cx = arcs[i].rcx + dx;
        float cy = arcs[i].rcy + dy;
        float best_q = 0.0f;

        for (int j = 0; j < n_sarcs; j++) {
            float rdiff = fabsf(arcs[i].r - sarcs[j].r);

            /* Same early-exit prefilter idea applied to radius, which is
               cheaper to compute than the centre distance (no sqrtf). */
            if (rdiff > 3.0f * CORR_SIGMA_R_M) {
                continue;
            }

            float cdx = cx - sarcs[j].mx;
            float cdy = cy - sarcs[j].my;
            float centre_dist = sqrtf(cdx * cdx + cdy * cdy);
            float nc = centre_dist / CORR_SIGMA_CENTRE_M;
            float nr = rdiff / CORR_SIGMA_R_M;
            float q = expf(-(nc * nc + nr * nr));
            if (q > best_q) {
                best_q = q;
            }
        }
        total += best_q;
        n_scored++;
    }

    *out_score = total;
    *out_n_scored = n_scored;
}

/* =========================================================================
 * Top-K distinct candidate tracking (Layer 1)
 * ========================================================================= */

typedef struct {
    float score, dx, dy, dtheta;
    int n_scored;
} CorrCandidate;

static void corr_sort_topk_desc(CorrCandidate *arr, int n) {
    /* n <= CORR_TOP_K_CANDIDATES (small) — plain insertion sort. */
    for (int i = 1; i < n; i++) {
        CorrCandidate key = arr[i];
        int j = i - 1;
        while (j >= 0 && arr[j].score < key.score) {
            arr[j + 1] = arr[j];
            j--;
        }
        arr[j + 1] = key;
    }
}

static void corr_insert_topk_distinct(
    CorrCandidate *top, int *n_top, CorrCandidate cand, int k)
{
    /* Two candidates within MIN_PEAK_SEP_* of each other are the SAME
       peak (adjacent grid cells sampling the same local maximum) — keep
       only the higher-scoring one. See correlative_match.py's
       _insert_topk_distinct docstring for why this matters (without it,
       top-K fills with near-duplicates of one peak instead of K
       genuinely distinct candidates). */
    for (int i = 0; i < *n_top; i++) {
        if (fabsf(cand.dtheta - top[i].dtheta) < CORR_MIN_PEAK_SEP_DTHETA_RAD
            && hypotf(cand.dx - top[i].dx, cand.dy - top[i].dy) < CORR_MIN_PEAK_SEP_DXY_M) {
            if (cand.score > top[i].score) {
                top[i] = cand;
                corr_sort_topk_desc(top, *n_top);
            }
            return;
        }
    }

    if (*n_top < k) {
        top[*n_top] = cand;
        (*n_top)++;
        corr_sort_topk_desc(top, *n_top);
    } else if (cand.score > top[k - 1].score) {
        top[k - 1] = cand;
        corr_sort_topk_desc(top, k);
    }
}

/* =========================================================================
 * Layer 1 — coarse exhaustive sweep, keep top-K distinct candidates
 * ========================================================================= */

static void corr_coarse_sweep_topk(
    const ScanFeature *scan_features, int n_scan,
    float guess_x, float guess_y, float guess_theta,
    const CorrStaticLine *slines, int n_slines,
    const CorrStaticArc *sarcs, int n_sarcs,
    CorrCandidate *top, int *n_top)
{
    *n_top = 0;
    int n_candidates = 0;

    for (int t = 0; t < CORR_MAX_DTHETA_STEPS; t++) {
        float dtheta = -CORR_SEARCH_DTHETA_MAX_RAD + CORR_COARSE_DTHETA_STEP_RAD * (float)t;
        float cos_t = cosf(guess_theta + dtheta);
        float sin_t = sinf(guess_theta + dtheta);

        CorrRotatedLine rlines[CORR_MAX_SCAN_FEATURES];
        CorrRotatedArc rarcs[CORR_MAX_SCAN_FEATURES];
        int n_rlines, n_rarcs;
        corr_prerotate_features(scan_features, n_scan, cos_t, sin_t,
                                 guess_theta + dtheta, rlines, &n_rlines,
                                 rarcs, &n_rarcs);

        n_candidates += CORR_MAX_DXY_STEPS * CORR_MAX_DXY_STEPS;
        if (n_rlines == 0 && n_rarcs == 0) {
            continue;
        }

        for (int ix = 0; ix < CORR_MAX_DXY_STEPS; ix++) {
            float dx_off = -CORR_SEARCH_DXY_MAX_M + CORR_COARSE_DXY_STEP_M * (float)ix;
            float dx_abs = guess_x + dx_off;

            for (int iy = 0; iy < CORR_MAX_DXY_STEPS; iy++) {
                float dy_off = -CORR_SEARCH_DXY_MAX_M + CORR_COARSE_DXY_STEP_M * (float)iy;
                float dy_abs = guess_y + dy_off;

                float score;
                int n_scored;
                corr_score_candidate(rlines, n_rlines, rarcs, n_rarcs,
                                      dx_abs, dy_abs, slines, n_slines,
                                      sarcs, n_sarcs, &score, &n_scored);
                if (score <= 0.0f) {
                    continue;
                }

                CorrCandidate cand = { score, dx_off, dy_off, dtheta, n_scored };
                corr_insert_topk_distinct(top, n_top, cand, CORR_TOP_K_CANDIDATES);
            }
        }
    }

    corr_last_diagnostics.n_layer1_candidates = n_candidates;
}

/* =========================================================================
 * Layer 2 — greedy local refinement ("pattern search" family)
 * ========================================================================= */

static void corr_greedy_refine(
    const ScanFeature *scan_features, int n_scan,
    float guess_x, float guess_y, float guess_theta,
    const CorrStaticLine *slines, int n_slines,
    const CorrStaticArc *sarcs, int n_sarcs,
    float start_dx, float start_dy, float start_dtheta,
    float start_score, int start_n,
    float *out_dx, float *out_dy, float *out_dtheta,
    float *out_score, int *out_n)
{
    float cur_dx = start_dx, cur_dy = start_dy, cur_dtheta = start_dtheta;
    float cur_score = start_score;
    int cur_n = start_n;
    int n_candidates = 0;

    for (int round = 0; round < CORR_MAX_WALK_ROUNDS; round++) {
        float best_dx = cur_dx, best_dy = cur_dy, best_dtheta = cur_dtheta;
        float best_score = cur_score;
        int best_n = cur_n;

        for (int t = 0; t < CORR_MAX_WALK_DTHETA_STEPS; t++) {
            float dtheta_off = -CORR_WALK_WINDOW_DTHETA_RAD
                                + CORR_WALK_DTHETA_STEP_RAD * (float)t;
            float cand_dtheta = cur_dtheta + dtheta_off;

            float cos_t = cosf(guess_theta + cand_dtheta);
            float sin_t = sinf(guess_theta + cand_dtheta);
            CorrRotatedLine rlines[CORR_MAX_SCAN_FEATURES];
            CorrRotatedArc rarcs[CORR_MAX_SCAN_FEATURES];
            int n_rlines, n_rarcs;
            corr_prerotate_features(scan_features, n_scan, cos_t, sin_t,
                                     guess_theta + cand_dtheta, rlines, &n_rlines,
                                     rarcs, &n_rarcs);

            n_candidates += CORR_MAX_WALK_DXY_STEPS * CORR_MAX_WALK_DXY_STEPS;
            if (n_rlines == 0 && n_rarcs == 0) {
                continue;
            }

            for (int ix = 0; ix < CORR_MAX_WALK_DXY_STEPS; ix++) {
                float dx_off = -CORR_WALK_WINDOW_DXY_M + CORR_WALK_DXY_STEP_M * (float)ix;
                float cand_dx = cur_dx + dx_off;

                for (int iy = 0; iy < CORR_MAX_WALK_DXY_STEPS; iy++) {
                    float dy_off = -CORR_WALK_WINDOW_DXY_M + CORR_WALK_DXY_STEP_M * (float)iy;
                    float cand_dy = cur_dy + dy_off;

                    if (!corr_within_walk_bounds(cand_dx, cand_dy, cand_dtheta)) {
                        continue;
                    }

                    float score;
                    int n_scored;
                    corr_score_candidate(rlines, n_rlines, rarcs, n_rarcs,
                                          guess_x + cand_dx, guess_y + cand_dy,
                                          slines, n_slines, sarcs, n_sarcs,
                                          &score, &n_scored);

                    if (score > best_score + CORR_IMPROVEMENT_EPS) {
                        best_score = score;
                        best_n = n_scored;
                        best_dx = cand_dx;
                        best_dy = cand_dy;
                        best_dtheta = cand_dtheta;
                    }
                }
            }
        }

        if (best_score <= cur_score + CORR_IMPROVEMENT_EPS) {
            break;   /* no improvement this round — converged */
        }
        cur_dx = best_dx;
        cur_dy = best_dy;
        cur_dtheta = best_dtheta;
        cur_score = best_score;
        cur_n = best_n;
    }

    corr_last_diagnostics.n_layer2_candidates += n_candidates;

    *out_dx = cur_dx;
    *out_dy = cur_dy;
    *out_dtheta = cur_dtheta;
    *out_score = cur_score;
    *out_n = cur_n;
}

/* =========================================================================
 * Public entry point
 * ========================================================================= */

CoarseResult correlative_search(
    const ScanFeature *scan_features, int n_scan,
    const CorrMapEntry *map_entries, int n_map,
    float guess_x, float guess_y, float guess_theta,
    uint8_t static_status)
{
    CoarseResult invalid = { 0.0f, 0.0f, 0.0f, 0.0f, 0, false, false, 0.0f };

    corr_last_diagnostics.n_layer2_candidates = 0;   /* accumulated across all K walks */

    CorrStaticLine slines[CORR_MAX_STATIC_ENTRIES_SEARCHED];
    CorrStaticArc sarcs[CORR_MAX_STATIC_ENTRIES_SEARCHED];
    int n_slines, n_sarcs;
    corr_prepare_static_maps(map_entries, n_map, static_status, guess_x, guess_y,
                              slines, &n_slines, sarcs, &n_sarcs);

    int n_static = n_slines + n_sarcs;
    corr_last_diagnostics.n_static = n_static;

    if (n_static < CORR_MIN_STATIC_FEATURES) {
        invalid.n_static = n_static;
        return invalid;
    }

    /* ---- Layer 1 -------------------------------------------------------- */
    CorrCandidate top[CORR_TOP_K_CANDIDATES];
    int n_top = 0;
    corr_coarse_sweep_topk(scan_features, n_scan, guess_x, guess_y, guess_theta,
                            slines, n_slines, sarcs, n_sarcs, top, &n_top);

    if (n_top == 0) {
        invalid.n_static = n_static;
        return invalid;
    }

    /* ---- Layer 2 -------------------------------------------------------- */
    CorrCandidate refined[CORR_TOP_K_CANDIDATES];
    for (int i = 0; i < n_top; i++) {
        corr_greedy_refine(scan_features, n_scan, guess_x, guess_y, guess_theta,
                            slines, n_slines, sarcs, n_sarcs,
                            top[i].dx, top[i].dy, top[i].dtheta,
                            top[i].score, top[i].n_scored,
                            &refined[i].dx, &refined[i].dy, &refined[i].dtheta,
                            &refined[i].score, &refined[i].n_scored);
    }
    corr_sort_topk_desc(refined, n_top);

    float best_score = refined[0].score;
    float best_dx = refined[0].dx, best_dy = refined[0].dy, best_dtheta = refined[0].dtheta;
    int best_n = refined[0].n_scored;

    /* Two different layer-1 seeds can converge to the SAME final peak
       after refinement — only count a refined candidate as the "second"
       one for ambiguity purposes if it is STILL a genuinely separate peak
       after refinement (same separation test used before refinement). */
    float second_score = 0.0f;
    for (int i = 1; i < n_top; i++) {
        if (fabsf(refined[i].dtheta - best_dtheta) >= CORR_MIN_PEAK_SEP_DTHETA_RAD
            || hypotf(refined[i].dx - best_dx, refined[i].dy - best_dy) >= CORR_MIN_PEAK_SEP_DXY_M) {
            second_score = refined[i].score;
            break;
        }
    }

    if (best_n == 0) {
        invalid.n_static = n_static;
        return invalid;
    }

    float normalized = best_score / (float)best_n;
    if (normalized < CORR_MIN_SCORE_PER_FEATURE) {
        CoarseResult r = invalid;
        r.n_static = n_static;
        r.score = best_score;
        r.second_score = second_score;
        return r;
    }

    bool ambiguous = (n_top > 1)
        && (second_score >= best_score * (1.0f - CORR_AMBIGUITY_MARGIN_RATIO));

    CoarseResult result;
    result.dx = best_dx;
    result.dy = best_dy;
    result.dtheta = best_dtheta;
    result.score = best_score;
    result.n_static = n_static;
    result.valid = true;
    result.ambiguous = ambiguous;
    result.second_score = second_score;
    return result;
}
