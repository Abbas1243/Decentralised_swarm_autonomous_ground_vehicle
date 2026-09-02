#ifndef LINE_MATCHER_H
#define LINE_MATCHER_H

#include "messages.h"

#ifdef __cplusplus
extern "C" {
#endif

/* -----------------------------------------------------------------------
 * Matching thresholds
 * Tuned for indoor rooms with the RPLIDAR A1.
 * ----------------------------------------------------------------------- */
#define MATCH_ANGLE_THRESH   0.15f   /* radians — ~8.6 degrees             */
#define MATCH_DIST_THRESH    0.30f   /* meters  — generous for first scan   */
#define MATCH_MIN_SCORE      2.0f    /* lower = better; reject above this   */
#define MATCH_MIN_PAIRS      2       /* minimum matches to attempt pose est  */

/* Weights for composite match score: score = w_a*angle_diff + w_d*dist_diff */
#define MATCH_W_ANGLE        2.0f
#define MATCH_W_DIST         1.0f

/* State transition thresholds */
#define CONFIRM_THRESH       3       /* observations to become STATIC        */
#define OCCLUDE_THRESH       8       /* consecutive misses → OCCLUDED        */
#define DYNAMIC_TTL          5       /* scans before unconfirmed line pruned */

/* -----------------------------------------------------------------------
 * MatchPair
 * One matched scan-line → map-line pair, plus classification result.
 * ----------------------------------------------------------------------- */
typedef struct {
    int   scan_idx;    /* index into FeaturePacket.lines[]              */
    int   map_idx;     /* index into map array                          */
    float score;       /* match quality (lower = better)                */
    LineState result;  /* classification: STATIC, DYNAMIC, or UNKNOWN   */
} MatchPair;

/* -----------------------------------------------------------------------
 * MatchResult
 * Output of one call to match_lines().
 * ----------------------------------------------------------------------- */
#define MAX_MATCH_PAIRS 50

typedef struct {
    MatchPair pairs[MAX_MATCH_PAIRS];
    int       n_matched;    /* number of STATIC matches (used for pose)  */
    int       n_dynamic;    /* number of DYNAMIC detections              */
    int       n_unknown;    /* number of UNKNOWN (first-time) lines      */
} MatchResult;

/* -----------------------------------------------------------------------
 * match_lines()
 *
 * Core function. For each line in the scan, try to match it against the
 * current map. Classify each line as STATIC, DYNAMIC, or UNKNOWN based
 * on match quality and map line history.
 *
 * scan_lines:  LineFeature array from current FeaturePacket (sensor frame)
 * n_scan:      number of valid scan lines
 * map:         current map array
 * n_map:       number of active map lines
 * pose:        current pose estimate (used to transform scan → map frame)
 * out:         filled with match results
 *
 * Returns number of STATIC matches (same as out->n_matched).
 * ----------------------------------------------------------------------- */
int match_lines(const LineFeature *scan_lines, int n_scan,
                const MapLine     *map,        int n_map,
                const Pose        *pose,
                MatchResult       *out);

/* -----------------------------------------------------------------------
 * transform_line_to_map()
 *
 * Transform a single LineFeature from sensor frame to map frame
 * given the current pose estimate. Used internally and by slam.c.
 *
 * In:  lf_sensor  — line in sensor frame
 *      pose       — current robot pose in map frame
 * Out: lf_map     — line transformed to map frame
 * ----------------------------------------------------------------------- */
void transform_line_to_map(const LineFeature *lf_sensor,
                           const Pose        *pose,
                           LineFeature       *lf_map);

/* -----------------------------------------------------------------------
 * line_match_score()
 *
 * Compute match score between two lines (both in map frame).
 * Returns MATCH_MIN_SCORE+1 if outside threshold (no match).
 * Lower score = better match.
 * ----------------------------------------------------------------------- */
float line_match_score(const LineFeature *a, const MapLine *b);

#ifdef __cplusplus
}
#endif

#endif /* LINE_MATCHER_H */