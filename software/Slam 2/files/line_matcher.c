#include "line_matcher.h"
#include <math.h>
#include <string.h>
#include <float.h>

/* -----------------------------------------------------------------------
 * angle_diff_lines — fold angle difference into [0, pi/2]
 * Lines have no direction so angle pi == angle 0.
 * ----------------------------------------------------------------------- */
static float angle_diff_lines(float a, float b)
{
    float d = fabsf(a - b);
    while (d > (float)M_PI)    d -= (float)M_PI;
    if    (d > (float)M_PI_2)  d  = (float)M_PI - d;
    return d;
}

/* -----------------------------------------------------------------------
 * transform_line_to_map
 * ----------------------------------------------------------------------- */
void transform_line_to_map(const LineFeature *lf_sensor,
                           const Pose        *pose,
                           LineFeature       *lf_map)
{
    float cs = cosf(pose->theta);
    float sn = sinf(pose->theta);

    lf_map->x1 = cs * lf_sensor->x1 - sn * lf_sensor->y1 + pose->x;
    lf_map->y1 = sn * lf_sensor->x1 + cs * lf_sensor->y1 + pose->y;
    lf_map->x2 = cs * lf_sensor->x2 - sn * lf_sensor->y2 + pose->x;
    lf_map->y2 = sn * lf_sensor->x2 + cs * lf_sensor->y2 + pose->y;

    float a = lf_sensor->angle + pose->theta;
    while (a >  (float)M_PI_2) a -= (float)M_PI;
    while (a < -(float)M_PI_2) a += (float)M_PI;
    lf_map->angle = a;

    float mx = (lf_map->x1 + lf_map->x2) * 0.5f;
    float my = (lf_map->y1 + lf_map->y2) * 0.5f;
    lf_map->distance = -sinf(a) * mx + cosf(a) * my;
    lf_map->length   = lf_sensor->length;
    lf_map->quality  = lf_sensor->quality;
}

/* -----------------------------------------------------------------------
 * line_match_score
 * ----------------------------------------------------------------------- */
float line_match_score(const LineFeature *scan_map_frame, const MapLine *map_line)
{
    float da = angle_diff_lines(scan_map_frame->angle, map_line->angle);
    if (da > MATCH_ANGLE_THRESH) return FLT_MAX;

    float dd = fabsf(scan_map_frame->distance - map_line->distance);
    if (dd > MATCH_DIST_THRESH) return FLT_MAX;

    return MATCH_W_ANGLE * da + MATCH_W_DIST * dd;
}

/* -----------------------------------------------------------------------
 * classify_scan_line
 *
 * KEY RULE: LINE_DYNAMIC is only valid when we have confirmed static
 * geometry to compare against. With an empty or unconfirmed map, every
 * unmatched line is LINE_UNKNOWN — we simply don't know yet.
 *
 * n_confirmed: number of LINE_STATIC/LINE_OCCLUDED lines in current map.
 * ----------------------------------------------------------------------- */
static LineState classify_scan_line(const LineFeature *scan_map_frame,
                                    const MapLine     *best_map_line,
                                    int                n_confirmed)
{
    if (best_map_line == NULL) {
        /*
         * No match anywhere in map.
         * Only call it DYNAMIC when we have a reliable static map to
         * compare against. If the map has no confirmed lines, we can't
         * distinguish "new obstacle" from "first scan of real wall".
         */
        if (n_confirmed > 0 && scan_map_frame->quality >= 60)
            return LINE_DYNAMIC;
        return LINE_UNKNOWN;
    }

    /* Match found */
    if (best_map_line->state == LINE_OCCLUDED)
        return LINE_OCCLUDED;

    if (best_map_line->confirmed)
        return LINE_STATIC;

    return LINE_UNKNOWN;
}

/* -----------------------------------------------------------------------
 * match_lines
 *
 * pairs[] layout (three non-overlapping regions):
 *   [0 .. n_matched)                  STATIC + OCCLUDED
 *   [n_matched .. MAX-n_dynamic-1]    UNKNOWN (with or without map_idx)
 *   [MAX-n_dynamic .. MAX-1]          DYNAMIC (fills from back)
 * ----------------------------------------------------------------------- */
int match_lines(const LineFeature *scan_lines, int n_scan,
                const MapLine     *map,        int n_map,
                const Pose        *pose,
                MatchResult       *out)
{
    memset(out, 0, sizeof(MatchResult));

    /* Count confirmed map lines — needed for DYNAMIC classification */
    int n_confirmed = 0;
    for (int mi = 0; mi < n_map; mi++) {
        if (map[mi].active && map[mi].confirmed)
            n_confirmed++;
    }

    for (int si = 0; si < n_scan; si++) {

        LineFeature lf_map;
        transform_line_to_map(&scan_lines[si], pose, &lf_map);

        /* Best-match search — skip DYNAMIC map lines */
        float best_score   = FLT_MAX;
        int   best_map_idx = -1;

        for (int mi = 0; mi < n_map; mi++) {
            if (!map[mi].active)                continue;
            if (map[mi].state == LINE_DYNAMIC)  continue;

            float score = line_match_score(&lf_map, &map[mi]);
            if (score < best_score) {
                best_score   = score;
                best_map_idx = mi;
            }
        }

        const MapLine *best = (best_map_idx >= 0) ? &map[best_map_idx] : NULL;
        LineState state = classify_scan_line(&lf_map, best, n_confirmed);

        /* Store in appropriate region */
        if (state == LINE_STATIC || state == LINE_OCCLUDED) {
            if (out->n_matched < MAX_MATCH_PAIRS) {
                MatchPair *p = &out->pairs[out->n_matched++];
                p->scan_idx = si;
                p->map_idx  = best_map_idx;
                p->score    = best_score;
                p->result   = state;
            }

        } else if (state == LINE_DYNAMIC) {
            if (out->n_dynamic < MAX_MATCH_PAIRS) {
                int idx = MAX_MATCH_PAIRS - 1 - out->n_dynamic;
                MatchPair *p = &out->pairs[idx];
                p->scan_idx = si;
                p->map_idx  = -1;
                p->score    = FLT_MAX;
                p->result   = LINE_DYNAMIC;
                out->n_dynamic++;
            }

        } else {
            /* LINE_UNKNOWN */
            int mid_idx       = out->n_matched + out->n_unknown;
            int dynamic_start = MAX_MATCH_PAIRS - out->n_dynamic;
            if (mid_idx < dynamic_start) {
                MatchPair *p  = &out->pairs[mid_idx];
                p->scan_idx   = si;
                p->map_idx    = best_map_idx;   /* -1 = truly new */
                p->score      = best_score;
                p->result     = LINE_UNKNOWN;
            }
            out->n_unknown++;
        }
    }

    return out->n_matched;
}