/*
 * correlative_match.h
 * =====================
 * C port of correlative_match.py — see that file for the full algorithm
 * rationale (two-layer coarse-to-fine correlative search: Layer 1 is a
 * coarse exhaustive sweep keeping the top-K DISTINCT scoring candidates,
 * Layer 2 is an independent greedy local refinement of each candidate).
 * This header/source pair is a direct, literal translation of the
 * SCALAR reference path in that file (_score_candidate /
 * _coarse_sweep_topk / _greedy_refine before vectorization) — that is
 * the shape the Python module's own docstring says the C port should
 * mirror, since numpy has no C equivalent.
 *
 * SCOPE
 * -----
 * Only what correlative_match.c needs is defined here:
 *   - ScanFeature  = Feature, from shared/messages.h (the wire-protocol
 *     struct already carries type/angle/distance/endpoints/cx,cy,r —
 *     exactly what a scan feature dict carried in the Python version).
 *   - CorrMapEntry = a minimal slice of map_manager's MapEntry (angle,
 *     distance, mx, my, status, active). map_manager.c does not exist
 *     yet in this codebase (see SLAM_PROGRESS.md — C port pending for
 *     that module too). When map_manager.h is written, replace
 *     CorrMapEntry with the real MapEntry struct and drop this stub —
 *     field NAMES were chosen to match map_manager.py's MapEntry
 *     attributes exactly for that reason.
 *
 * CONSTRAINTS (slam_build_prompt.md — unchanged from the rest of the
 * codebase):
 *   - No Eigen/OpenCV/Ceres, no dynamic allocation, float not double,
 *     f-suffix every literal, must cross-compile aarch64-linux-gnu-g++
 *     and run correctly on QEMU before hardware.
 *   - Every array in this module is fixed-size, sized from the same
 *     constants correlative_match.py already exposes (MAX_STATIC_
 *     ENTRIES_SEARCHED, MAX_FEATURES_PER_SCAN, the coarse/walk grid
 *     step counts) — see the CORR_MAX_* #defines below, each with the
 *     arithmetic that derives it from the corresponding _RAD/_M pair,
 *     spelled out in a comment so a future constant change is caught
 *     instead of silently overflowing a stack array.
 */

#ifndef CORRELATIVE_MATCH_H
#define CORRELATIVE_MATCH_H

#include <stdint.h>
#include <stdbool.h>
#include "messages.h"   /* Feature, FEAT_LINE, FEAT_ARC, FEAT_LINE_X1/Y1/X2/Y2,
                            MAX_FEATURES_PER_SCAN */

#ifdef __cplusplus
extern "C" {
#endif

typedef Feature ScanFeature;   /* alias — see module docstring above */

/* =========================================================================
 * CorrMapEntry — minimal MapEntry slice (see module docstring "SCOPE")
 * ========================================================================= */

#define CORR_ARC_ANGLE_SENTINEL   (-10.0f)   /* matches map_manager.py */
#define CORR_ENTRY_UNCLASSIFIED   0u
#define CORR_ENTRY_STATIC         1u
#define CORR_ENTRY_DYNAMIC        2u

typedef struct {
    float   angle;      /* line: Hough angle rad | CORR_ARC_ANGLE_SENTINEL for arcs */
    float   distance;   /* line: Hough distance m | arc: radius m */
    float   mx, my;     /* line: midpoint m       | arc: centre m */
    uint8_t status;      /* CORR_ENTRY_* */
    bool    active;
} CorrMapEntry;

static inline bool corr_map_entry_is_arc(const CorrMapEntry *e) {
    /* Same -4.0 threshold as map_manager.py's MapEntry.is_arc() — well
       clear of the valid line-angle range [-pi/2, pi/2] with margin. */
    return e->angle < -4.0f;
}

/* =========================================================================
 * Result — mirrors correlative_match.py's CoarseResult namedtuple
 * ========================================================================= */

typedef struct {
    float dx, dy, dtheta;     /* corrections to ADD to (guess_x,guess_y,guess_theta) */
    float score;              /* winning candidate's score — diagnostics only */
    int   n_static;           /* STATIC map entries actually searched against */
    bool  valid;              /* false -> a confidence gate failed; dx/dy/dtheta
                                  are 0.0f, caller must not apply them */
    bool  ambiguous;          /* true -> a close second peak was found; caller
                                  should apply the delta but not trust it to
                                  author new map evidence yet */
    float second_score;       /* second-best DISTINCT candidate's score, 0 if none */
} CoarseResult;

/* =========================================================================
 * Tunables — mirror correlative_match.py's module-level constants exactly
 * ========================================================================= */

#define CORR_MAX_STATIC_ENTRIES_SEARCHED   70

#define CORR_SEARCH_DTHETA_MAX_RAD   1.39626340f   /* 80 deg */
#define CORR_SEARCH_DXY_MAX_M        0.15f
#define CORR_COARSE_DTHETA_STEP_RAD  0.17453293f   /* 10 deg */
#define CORR_COARSE_DXY_STEP_M       0.05f

#define CORR_TOP_K_CANDIDATES        3

#define CORR_MIN_PEAK_SEP_DTHETA_RAD 0.34906585f   /* 20 deg */
#define CORR_MIN_PEAK_SEP_DXY_M      0.15f

#define CORR_WALK_DTHETA_STEP_RAD    0.05235988f   /* 3 deg */
#define CORR_WALK_WINDOW_DTHETA_RAD  0.26179939f   /* 15 deg */
#define CORR_WALK_DXY_STEP_M         0.03f
#define CORR_WALK_WINDOW_DXY_M       0.06f
#define CORR_MAX_WALK_ROUNDS         3

#define CORR_MAX_WALK_DTHETA_RAD     0.52359878f   /* 30 deg — total reach cap,
                                                       measured from the ORIGINAL
                                                       guess, not per-step */
#define CORR_MAX_WALK_DXY_M          0.25f

#define CORR_IMPROVEMENT_EPS         1e-6f
#define CORR_AMBIGUITY_MARGIN_RATIO  0.10f

#define CORR_SIGMA_ANGLE_RAD         0.15f
#define CORR_SIGMA_DIST_M            0.15f
#define CORR_SIGMA_CENTRE_M          0.15f
#define CORR_SIGMA_R_M               0.08f

#define CORR_MIN_STATIC_FEATURES     3
#define CORR_MIN_SCORE_PER_FEATURE   0.35f

/* ---- Fixed grid sizes, derived from the pairs above ----------------------
 * n_steps = round(2*MAX/STEP) + 1, spelled out numerically so a future
 * change to the _RAD/_M constants that silently changes step count is
 * caught by CORR_MAX_SCAN_FEATURES-style array overflow at compile/test
 * time rather than a stack smash at runtime. Recompute by hand if you
 * change the constants above. */
#define CORR_MAX_DTHETA_STEPS        17   /* round(2*80/10)+1 */
#define CORR_MAX_DXY_STEPS           7    /* round(2*0.15/0.05)+1 */
#define CORR_MAX_WALK_DTHETA_STEPS   11   /* round(2*15/3)+1 */
#define CORR_MAX_WALK_DXY_STEPS      5    /* round(2*0.06/0.03)+1 */

#define CORR_MAX_SCAN_FEATURES       ((int)MAX_FEATURES_PER_SCAN)   /* 50, messages.h */

/* =========================================================================
 * Diagnostics — mirrors the module-level last_search_* attributes in
 * correlative_match.py. No wall-clock timing here (the Python version's
 * last_search_total_s/layer1_s/layer2_s are perf_counter reads, which the
 * C port leaves to the caller — slam.c can wrap the call in clock_gettime
 * itself); only the candidate-count diagnostics are reproduced, since
 * those are useful for tuning without pulling in a timing dependency this
 * module doesn't otherwise need.
 * ========================================================================= */

typedef struct {
    int n_static;
    int n_layer1_candidates;
    int n_layer2_candidates;
} CorrDiagnostics;

extern CorrDiagnostics corr_last_diagnostics;

/* =========================================================================
 * Public API
 * ========================================================================= */

/*
 * correlative_search — run the two-layer correlative coarse pose search.
 *
 * scan_features, n_scan   : this scan's features, SENSOR frame (same
 *                            convention line_matcher/pose_estimator use).
 * map_entries, n_map       : full map array (fixed-size, caller-owned,
 *                            e.g. MapManager's MAX_MAP_ENTRIES=500 array);
 *                            only .active && .status==static_status
 *                            entries are read.
 * guess_x, guess_y, guess_theta : current pose estimate (map frame) to
 *                            search around.
 * static_status            : MapEntry.status value meaning STATIC
 *                            (CORR_ENTRY_STATIC=1, matching map_manager.py).
 *
 * Returns a CoarseResult — see the struct's field comments above.
 */
CoarseResult correlative_search(
    const ScanFeature *scan_features, int n_scan,
    const CorrMapEntry *map_entries, int n_map,
    float guess_x, float guess_y, float guess_theta,
    uint8_t static_status);

#ifdef __cplusplus
}
#endif

#endif /* CORRELATIVE_MATCH_H */
