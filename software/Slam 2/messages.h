/*
 * messages.h — Wire protocol: STM32 F411 → Milk-V Duo S
 *
 * SCOPE — this file defines ONLY the structs that cross the UART boundary
 * between STM32 and Duo S.  Nothing else belongs here.
 *
 *   IN scope : Feature, FeaturePacket, UART framing constants
 *   OUT of scope : MapEntry, Pose, PoseNode, PoseConstraint — those are
 *                  internal SLAM state on the Duo S and live in their own
 *                  module headers (map_manager.h, pose_graph.h, etc.)
 *
 * Targets
 *   STM32F411   arm-none-eabi-gcc   single-precision FPU, no dynamic alloc
 *   Milk-V Duo S  aarch64-linux-gnu-g++  ARM Cortex-A53
 *   PC tooling  gcc  testing / bag replay
 *
 * Rules — apply to every struct in this file
 *   All floats, no doubles        (STM32 FPU is single-precision only)
 *   No pointers, no dynamic alloc (STM32 has 128 KB RAM, no heap)
 *   f-suffix on every float literal
 *   __attribute__((packed)) on every struct — identical layout on all targets
 *
 * Verified sizes (run slam_core/test_messages.c to confirm on new toolchain)
 *   sizeof(Feature)       == 48
 *   sizeof(FeaturePacket) == 2408   (8 header + 50 * 48 payload)
 */

#ifndef MESSAGES_H
#define MESSAGES_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif


/* =========================================================================
 * Feature type tags
 * ========================================================================= */

#define FEAT_LINE  0u   /* line segment — Hough parameterisation             */
#define FEAT_ARC   1u   /* circular arc — centre + radius parameterisation   */


/* =========================================================================
 * Feature — 48 bytes
 *
 * Single extracted geometric feature from one LiDAR scan.
 * Produced by fit_first_extract() on STM32, consumed by map_manager on Duo S.
 * Layout MUST match fit_first.h on STM32 and fit_first_ctypes.py on PC.
 *
 * Memory layout (byte offsets):
 *    0   type        uint8_t      FEAT_LINE or FEAT_ARC
 *    1   _pad[3]     uint8_t[3]   alignment pad — do not use
 *    4   angle       float        line: Hough angle [-pi/2, pi/2] rad | arc: 0
 *    8   distance    float        line: perp distance from origin (m) | arc: 0
 *   12   t_start     float        line: endpoint x2 (m)              | arc: 0
 *   16   t_end       float        line: endpoint y2 (m)              | arc: 0
 *   20   cx          float        line: endpoint x1 (m)              | arc: centre x (m)
 *   24   cy          float        line: endpoint y1 (m)              | arc: centre y (m)
 *   28   r           float        line: 0                            | arc: radius (m)
 *   32   theta_start float        line: 0                            | arc: start angle (rad)
 *   36   theta_end   float        line: 0                            | arc: end angle (rad)
 *   40   length      float        line: chord length (m)             | arc: arc length (m)
 *   44   quality     uint8_t      confidence 0–100
 *   45   _pad2[3]    uint8_t[3]   trailing pad to 48 bytes — do not use
 *
 * LINE endpoint access — always use these macros, never access fields directly.
 * The field names cx/cy/t_start/t_end are reused for line endpoints; the macros
 * document the intended meaning and make grep-able call sites:
 *   FEAT_LINE_X1(f)  first  endpoint x
 *   FEAT_LINE_Y1(f)  first  endpoint y
 *   FEAT_LINE_X2(f)  second endpoint x
 *   FEAT_LINE_Y2(f)  second endpoint y
 * ========================================================================= */

typedef struct __attribute__((packed)) {
    uint8_t  type;          /* FEAT_LINE or FEAT_ARC                         */
    uint8_t  _pad[3];       /* alignment pad — do not use                    */
    float    angle;         /* line: Hough angle (rad)   | arc: 0            */
    float    distance;      /* line: Hough dist  (m)     | arc: 0            */
    float    t_start;       /* line: endpoint x2 (m)     | arc: 0            */
    float    t_end;         /* line: endpoint y2 (m)     | arc: 0            */
    float    cx;            /* line: endpoint x1 (m)     | arc: centre x (m) */
    float    cy;            /* line: endpoint y1 (m)     | arc: centre y (m) */
    float    r;             /* line: 0                   | arc: radius (m)   */
    float    theta_start;   /* line: 0                   | arc: start angle  */
    float    theta_end;     /* line: 0                   | arc: end angle    */
    float    length;        /* chord / arc length (m)                        */
    uint8_t  quality;       /* confidence 0–100                              */
    uint8_t  _pad2[3];      /* trailing pad — do not use                     */
} Feature;

/* Line endpoint accessors — single source of truth for the field reuse */
#define FEAT_LINE_X1(f)  ((f).cx)
#define FEAT_LINE_Y1(f)  ((f).cy)
#define FEAT_LINE_X2(f)  ((f).t_start)
#define FEAT_LINE_Y2(f)  ((f).t_end)


/* =========================================================================
 * FeaturePacket — 2408 bytes
 *
 * One complete LiDAR scan worth of features.  Transmitted over UART after
 * every scan (~10 Hz).
 *
 * UART compact framing (transport/uart_receiver.c):
 *   [SYNC0 0xAA][SYNC1 0x55][timestamp_ms uint32][count uint8][checksum uint8]
 *   [count * sizeof(Feature) bytes of Feature data]
 *
 * Only `count` Feature structs are transmitted — the 50-slot array in this
 * struct is the in-memory receive buffer on Duo S, not the wire layout.
 * Recommended baud rate: 460800.
 * ========================================================================= */

#define MAX_FEATURES_PER_SCAN  50u

typedef struct __attribute__((packed)) {
    uint32_t timestamp_ms;                      /* scan timestamp            */
    uint8_t  count;                             /* valid entries in features[]*/
    uint8_t  _pad[3];                           /* alignment pad             */
    Feature  features[MAX_FEATURES_PER_SCAN];   /* feature buffer            */
} FeaturePacket;

/* UART framing sync bytes */
#define UART_SYNC_BYTE0  0xAAu
#define UART_SYNC_BYTE1  0x55u


/* =========================================================================
 * Compile-time size checks
 * Fail loudly on any toolchain that produces a different layout.
 * ========================================================================= */

#ifdef __cplusplus
    static_assert(sizeof(Feature)       == 48,   "Feature must be 48 bytes");
    static_assert(sizeof(FeaturePacket) == 2408, "FeaturePacket must be 2408 bytes");
#else
    _Static_assert(sizeof(Feature)       == 48,   "Feature must be 48 bytes");
    _Static_assert(sizeof(FeaturePacket) == 2408, "FeaturePacket must be 2408 bytes");
#endif


#ifdef __cplusplus
}
#endif

#endif /* MESSAGES_H */