#include "map_manager.h"
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <float.h>

void map_init(Map *m)
{
    memset(m, 0, sizeof(Map));
}

static int find_free_slot(Map *m)
{
    for (int i = 0; i < m->n_lines; i++)
        if (!m->lines[i].active) return i;
    if (m->n_lines < MAX_MAP_LINES)
        return m->n_lines++;
    return -1;
}

static void add_line_to_map(Map *m, const LineFeature *lf_map, int scan_count)
{
    if (lf_map->quality < MIN_ADD_QUALITY) return;
    if (lf_map->length  < MIN_ADD_LENGTH)  return;

    int slot = find_free_slot(m);
    if (slot < 0) return;

    MapLine *ml = &m->lines[slot];
    memset(ml, 0, sizeof(MapLine));
    ml->angle              = lf_map->angle;
    ml->distance           = lf_map->distance;
    ml->length             = lf_map->length;
    ml->mx                 = (lf_map->x1 + lf_map->x2) * 0.5f;
    ml->my                 = (lf_map->y1 + lf_map->y2) * 0.5f;
    ml->observed           = 1;
    ml->last_seen          = scan_count;
    ml->consecutive_misses = 0;
    ml->confidence         = lf_map->quality;
    ml->active             = 1;
    ml->state              = LINE_UNKNOWN;
    ml->confirmed          = 0;
}

static void update_matched_line(MapLine *ml, const LineFeature *lf_map, int scan_count)
{
    float w_old = (float)ml->observed;
    float w_tot = w_old + 1.0f;

    ml->angle    = (ml->angle    * w_old + lf_map->angle)    / w_tot;
    ml->distance = (ml->distance * w_old + lf_map->distance) / w_tot;
    ml->length   = (ml->length   * w_old + lf_map->length)   / w_tot;

    float mx_new = (lf_map->x1 + lf_map->x2) * 0.5f;
    float my_new = (lf_map->y1 + lf_map->y2) * 0.5f;
    ml->mx = (ml->mx * w_old + mx_new) / w_tot;
    ml->my = (ml->my * w_old + my_new) / w_tot;

    uint32_t new_conf = (ml->confidence * (uint32_t)ml->observed + lf_map->quality)
                        / (uint32_t)(ml->observed + 1);
    ml->confidence = (new_conf > 100) ? 100 : new_conf;

    ml->observed++;
    ml->last_seen          = scan_count;
    ml->consecutive_misses = 0;
}

void map_apply_state_transitions(Map *m)
{
    for (int i = 0; i < m->n_lines; i++) {
        MapLine *ml = &m->lines[i];
        if (!ml->active) continue;

        switch ((LineState)ml->state) {

            case LINE_UNKNOWN:
                if (ml->observed >= CONFIRM_THRESH) {
                    ml->state     = LINE_STATIC;
                    ml->confirmed = 1;
                } else if (ml->consecutive_misses >= DYNAMIC_TTL) {
                    ml->active = 0;
                }
                break;

            case LINE_STATIC:
                if (ml->consecutive_misses >= OCCLUDE_THRESH)
                    ml->state = LINE_OCCLUDED;
                break;

            case LINE_OCCLUDED:
                if (ml->consecutive_misses == 0)
                    ml->state = LINE_STATIC;
                else if (ml->consecutive_misses >= OCCLUDE_THRESH * 5)
                    ml->active = 0;
                break;

            case LINE_DYNAMIC:
                if (ml->consecutive_misses >= DYNAMIC_TTL)
                    ml->active = 0;
                break;
        }
    }
}

void map_update(Map                 *m,
                const FeaturePacket *pkt,
                const MatchResult   *matches,
                const Pose          *pose)
{
    m->scan_count++;

    /* ── Bit arrays: which map lines and scan lines were handled ── */
    /* Stack arrays — 500 bytes + 50 bytes, negligible */
    uint8_t map_was_matched[MAX_MAP_LINES];
    uint8_t scan_handled[MAX_LINES_PER_SCAN];
    memset(map_was_matched, 0, sizeof(uint8_t) * m->n_lines);
    memset(scan_handled,    0, sizeof(uint8_t) * pkt->count);

    /* ── Step 1a: Process STATIC / OCCLUDED matches (front of pairs[]) ── */
    for (int i = 0; i < matches->n_matched; i++) {
        const MatchPair *p = &matches->pairs[i];
        int map_idx = p->map_idx;
        if (map_idx < 0 || map_idx >= m->n_lines) continue;
        if (p->scan_idx < 0 || p->scan_idx >= pkt->count) continue;

        map_was_matched[map_idx]  = 1;
        scan_handled[p->scan_idx] = 1;

        LineFeature lf_map;
        transform_line_to_map(&pkt->lines[p->scan_idx], pose, &lf_map);
        update_matched_line(&m->lines[map_idx], &lf_map, m->scan_count);
    }

    /* ── Step 1b: Process UNKNOWN matches (middle of pairs[]) ── */
    /* Upper bound is n_matched + n_unknown — not the dynamic_start, which would
     * include uninitialised (zeroed) pair slots and cause phantom updates. */
    int dynamic_start = MAX_MATCH_PAIRS - matches->n_dynamic;
    int unknown_end   = matches->n_matched + matches->n_unknown;
    if (unknown_end > dynamic_start) unknown_end = dynamic_start;
    for (int i = matches->n_matched; i < unknown_end; i++) {
        const MatchPair *p = &matches->pairs[i];
        if (p->result != LINE_UNKNOWN) break;
        int map_idx = p->map_idx;
        /* map_idx == -1 means truly new — handled in Step 3 below */
        if (map_idx < 0 || map_idx >= m->n_lines) {
            /* Truly new line — mark scan as needing insertion */
            if (p->scan_idx >= 0 && p->scan_idx < pkt->count)
                scan_handled[p->scan_idx] = 0;  /* ensure it reaches Step 3 */
            continue;
        }
        if (map_was_matched[map_idx]) continue;  /* already updated in 1a */

        map_was_matched[map_idx]  = 1;
        scan_handled[p->scan_idx] = 1;

        LineFeature lf_map;
        transform_line_to_map(&pkt->lines[p->scan_idx], pose, &lf_map);
        update_matched_line(&m->lines[map_idx], &lf_map, m->scan_count);
    }

    /* ── Step 2: Increment consecutive_misses for unmatched map lines ── */
    for (int i = 0; i < m->n_lines; i++) {
        if (!m->lines[i].active) continue;
        if (!map_was_matched[i])
            m->lines[i].consecutive_misses++;
    }

    /* ── Step 3: Handle DYNAMIC scan lines (back of pairs[]) ── */
    for (int i = 0; i < matches->n_dynamic; i++) {
        const MatchPair *p = &matches->pairs[MAX_MATCH_PAIRS - 1 - i];
        int si = p->scan_idx;
        if (si < 0 || si >= pkt->count) continue;
        scan_handled[si] = 2;   /* 2 = dynamic, add as DYNAMIC entry */
    }

    /* ── Step 4: Add unhandled scan lines to map ── */
    for (int si = 0; si < pkt->count; si++) {
        const LineFeature *lf_sensor = &pkt->lines[si];
        LineFeature lf_map;
        transform_line_to_map(lf_sensor, pose, &lf_map);

        if (scan_handled[si] == 0) {
            /* Truly new — no map match at all. Add as UNKNOWN. */
            add_line_to_map(m, &lf_map, m->scan_count);

        } else if (scan_handled[si] == 2) {
            /* DYNAMIC — check if already tracked as a dynamic map line */
            int already_tracked = 0;
            for (int mi = 0; mi < m->n_lines; mi++) {
                MapLine *ml = &m->lines[mi];
                if (!ml->active || ml->state != LINE_DYNAMIC) continue;
                if (fabsf(lf_map.angle    - ml->angle)    < 0.10f &&
                    fabsf(lf_map.distance - ml->distance) < 0.20f) {
                    update_matched_line(ml, &lf_map, m->scan_count);
                    map_was_matched[mi] = 1;  /* don't count as missed */
                    already_tracked = 1;
                    break;
                }
            }
            if (!already_tracked) {
                /* Add new dynamic entry */
                int before = m->n_lines;
                add_line_to_map(m, &lf_map, m->scan_count);
                /* Mark the newly added slot as DYNAMIC */
                for (int mi = 0; mi < m->n_lines; mi++) {
                    MapLine *ml = &m->lines[mi];
                    if (!ml->active || ml->state != LINE_UNKNOWN) continue;
                    if (ml->observed != 1 || ml->last_seen != m->scan_count) continue;
                    if (fabsf(ml->angle    - lf_map.angle)    < 0.01f &&
                        fabsf(ml->distance - lf_map.distance) < 0.05f) {
                        ml->state = LINE_DYNAMIC;
                        break;
                    }
                }
                (void)before;
            }
        }
        /* scan_handled[si] == 1: already updated in Steps 1a/1b */
    }

    /* ── Step 5: State machine transitions ── */
    map_apply_state_transitions(m);
}

void map_prune(Map *m)
{
    int write = 0;
    for (int read = 0; read < m->n_lines; read++) {
        if (m->lines[read].active) {
            if (write != read)
                m->lines[write] = m->lines[read];
            write++;
        }
    }
    m->n_lines = write;
}

void map_stats(const Map *m)
{
    int counts[4] = {0, 0, 0, 0};
    for (int i = 0; i < m->n_lines; i++) {
        if (!m->lines[i].active) continue;
        int s = (int)m->lines[i].state;
        if (s >= 0 && s < 4) counts[s]++;
    }
    fprintf(stderr,
            "[map] scan=%d  slots=%d  UNKNOWN=%d  STATIC=%d  DYNAMIC=%d  OCCLUDED=%d\n",
            m->scan_count, m->n_lines,
            counts[LINE_UNKNOWN], counts[LINE_STATIC],
            counts[LINE_DYNAMIC], counts[LINE_OCCLUDED]);
}