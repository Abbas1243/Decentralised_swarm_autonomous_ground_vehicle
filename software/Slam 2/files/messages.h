#ifndef MESSAGES_H
#define MESSAGES_H

#include <stdint.h>
#include <stdbool.h>

/* -----------------------------------------------------------------------
 * Portable math constants
 * M_PI / M_PI_2 are POSIX extensions — not guaranteed by C standard.
 * Define them here so every file that includes messages.h gets them
 * without needing _USE_MATH_DEFINES or compiler-specific flags.
 * ----------------------------------------------------------------------- */
#ifndef M_PI
#  define M_PI   3.14159265358979323846
#endif
#ifndef M_PI_2
#  define M_PI_2 1.57079632679489661923
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* -----------------------------------------------------------------------
 * LineFeature
 * One line segment extracted from a LiDAR scan.
 * On PC phase: produced by split_merge.c reading RPLIDAR directly.
 * On STM32 phase: produced by STM32 and sent via UART.
 * Coordinates are in sensor frame (meters), origin = LiDAR center.
 * angle is in [-pi/2, pi/2] (normal/Hough form — lines have no direction).
 * ----------------------------------------------------------------------- */
typedef struct {
    float   x1, y1;      /* line start point (meters)                  */
    float   x2, y2;      /* line end point   (meters)                  */
    float   angle;        /* orientation in radians, range [-pi/2,pi/2] */
    float   distance;     /* perpendicular distance from origin (meters) */
    float   length;       /* line length (meters)                       */
    uint8_t quality;      /* confidence 0-100                           */
    uint8_t _pad[3];      /* padding to 32 bytes                        */
} LineFeature;
/* 7*float(28) + 1*uint8(1) + 3*pad = 32 bytes */

/* -----------------------------------------------------------------------
 * FeaturePacket
 * One complete LiDAR scan worth of line features (~10Hz).
 * Wire-safe: fread/fwrite compatible layout.
 * ----------------------------------------------------------------------- */
#define MAX_LINES_PER_SCAN 50

typedef struct {
    uint32_t    timestamp_ms;               /* ms since boot / wall clock  */
    uint8_t     count;                      /* valid entries in lines[]    */
    uint8_t     _pad[3];                    /* alignment padding           */
    LineFeature lines[MAX_LINES_PER_SCAN];
} FeaturePacket;

/* -----------------------------------------------------------------------
 * LineState — per-map-line classification
 *
 * The core of static/dynamic separation.
 * Every MapLine carries one of these states.
 *
 * State machine:
 *
 *   UNKNOWN ──(observed >= CONFIRM_THRESH)──► STATIC
 *      │                                         │
 *      │                               (consecutive_misses
 *      │                                >= OCCLUDE_THRESH)
 *      │                                         │
 *      └──(first seen, quality low)──► DYNAMIC   ▼
 *                                            OCCLUDED
 *                                         (still in map,
 *                                          not used for pose)
 *
 * UNKNOWN:  seen 1-2 times, not yet confirmed. Not used for pose estimation.
 * STATIC:   confirmed wall/fixture. Used for pose estimation. Never auto-deleted.
 * DYNAMIC:  appeared but never confirmed, OR confirmed then vanished fast.
 *           Used for obstacle avoidance only. Auto-pruned after DYNAMIC_TTL scans.
 * OCCLUDED: was STATIC, temporarily not visible (something blocking it).
 *           Kept in map, skipped in matching until it reappears.
 * ----------------------------------------------------------------------- */
typedef enum {
    LINE_UNKNOWN  = 0,
    LINE_STATIC   = 1,
    LINE_DYNAMIC  = 2,
    LINE_OCCLUDED = 3
} LineState;

/* -----------------------------------------------------------------------
 * MapLine
 * A line feature stored in the persistent map (Duo S / PC SLAM process).
 * Uses Hough / normal form: (angle, distance) uniquely identifies a line.
 * mx, my is the midpoint in map frame — used for overlap and proximity.
 *
 * Size: 48 bytes (cache-friendly, fits 2 per 96-byte cache line pair)
 * ----------------------------------------------------------------------- */
typedef struct {
    /* Geometry (Hough form) — 20 bytes */
    float    angle;        /* orientation in map frame, [-pi/2, pi/2]   */
    float    distance;     /* perp. distance from map origin (meters)   */
    float    length;       /* estimated length (meters)                 */
    float    mx, my;       /* midpoint in map frame (meters)            */

    /* Observation tracking — 16 bytes */
    int32_t  observed;            /* times matched consistently          */
    int32_t  last_seen;           /* scan index when last matched        */
    int32_t  consecutive_misses;  /* scans since last match              */
    int32_t  active;              /* 1 = in use, 0 = remove on prune    */

    /* Quality + state — 8 bytes */
    uint32_t confidence;          /* 0-100 aggregate quality score       */
    uint8_t  state;        /* LineState enum — UNKNOWN/STATIC/DYNAMIC/OCCLUDED */
    uint8_t  confirmed;    /* 1 once observed >= CONFIRM_THRESH         */
    uint8_t  _pad[2];      /* explicit pad to 48 bytes                  */
} MapLine;
/* 5*float(20) + 4*int32(16) + 1*uint32(4) + 2*uint8(2) + 2*pad(2) = 44 bytes
 * Note: actual size is 44. Static assert below checks 44. */

/* -----------------------------------------------------------------------
 * Pose
 * Robot pose in the map frame.
 * theta in radians, counter-clockwise positive.
 * ----------------------------------------------------------------------- */
typedef struct {
    float x;       /* meters */
    float y;       /* meters */
    float theta;   /* radians */
} Pose;

/* -----------------------------------------------------------------------
 * PoseNode / PoseConstraint  (Version 2 — pose graph)
 * ----------------------------------------------------------------------- */
typedef struct {
    Pose        pose;
    int32_t     id;
    uint32_t    timestamp_ms;
} PoseNode;

typedef struct {
    int32_t from_id;
    int32_t to_id;
    Pose    relative_transform;
    float   weight;
} PoseConstraint;

/* -----------------------------------------------------------------------
 * Bag file record
 * Layout on disk: [BagRecord][FeaturePacket] per entry.
 * ----------------------------------------------------------------------- */
typedef struct {
    int64_t  wall_time_ms;
    uint32_t seq;
    uint32_t _pad;
} BagRecord;

/* -----------------------------------------------------------------------
 * Compile-time size assertions
 * ----------------------------------------------------------------------- */
#ifndef SLAM_NO_STATIC_ASSERT
_Static_assert(sizeof(LineFeature)   == 32,  "LineFeature size mismatch");
_Static_assert(sizeof(FeaturePacket) == 8 + MAX_LINES_PER_SCAN * 32,
               "FeaturePacket size mismatch");
_Static_assert(sizeof(MapLine)       == 44,  "MapLine size mismatch");
_Static_assert(sizeof(Pose)          == 12,  "Pose size mismatch");
#endif

#ifdef __cplusplus
}
#endif

#endif /* MESSAGES_H */