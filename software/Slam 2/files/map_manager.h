#ifndef MAP_MANAGER_H
#define MAP_MANAGER_H

#include "messages.h"
#include "line_matcher.h"

#ifdef __cplusplus
extern "C" {
#endif

/* -----------------------------------------------------------------------
 * Map storage limits
 * At 48 bytes/MapLine × 500 = 24KB — negligible RAM cost.
 * ----------------------------------------------------------------------- */
#define MAX_MAP_LINES       500
#define MIN_ADD_QUALITY     60      /* don't add lines below this quality  */
#define MIN_ADD_LENGTH      0.10f  /* meters — discard very short lines    */

/* -----------------------------------------------------------------------
 * Map
 * The full map state. Passed around by pointer — lives in slam.c.
 * ----------------------------------------------------------------------- */
typedef struct {
    MapLine lines[MAX_MAP_LINES];
    int     n_lines;        /* number of slots in use (includes inactive) */
    int     scan_count;     /* total scans processed — used as timestamp  */
} Map;

/* -----------------------------------------------------------------------
 * map_init()        — zero-initialise the map
 * map_update()      — core update: process one scan's MatchResult
 * map_prune()       — remove dead lines (called every N scans)
 * map_stats()       — print counts by state to stderr (debug)
 * ----------------------------------------------------------------------- */
void map_init(Map *m);

void map_update(Map              *m,
                const FeaturePacket *pkt,
                const MatchResult   *matches,
                const Pose          *pose);

void map_prune(Map *m);

void map_stats(const Map *m);

/* -----------------------------------------------------------------------
 * Internal state transition logic (exposed for unit tests)
 * ----------------------------------------------------------------------- */
void map_apply_state_transitions(Map *m);

#ifdef __cplusplus
}
#endif

#endif /* MAP_MANAGER_H */