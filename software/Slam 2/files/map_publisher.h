#ifndef MAP_PUBLISHER_H
#define MAP_PUBLISHER_H

#include "messages.h"
#include "map_manager.h"

#ifdef __cplusplus
extern "C" {
#endif

/* -----------------------------------------------------------------------
 * Zenoh topic keys
 * ----------------------------------------------------------------------- */
#define TOPIC_STATIC_MAP     "slam/static_map"      /* confirmed walls      */
#define TOPIC_DYNAMIC_LINES  "slam/dynamic_lines"   /* live obstacles       */
#define TOPIC_POSE           "slam/pose"            /* current robot pose   */

/* -----------------------------------------------------------------------
 * Wire format for map publish
 *
 * We send a flat binary buffer — no JSON, no protobuf.
 * The Python bridge reads it with struct.unpack, same as the bag format.
 *
 * MapPublishHeader  (12 bytes)
 *   uint32  magic       0x534C414D  ('SLAM')
 *   uint32  n_lines     number of MapLineWire entries following
 *   uint32  scan_count  current scan index
 *
 * MapLineWire  (24 bytes each)  — stripped-down MapLine for the wire
 *   float   angle
 *   float   distance
 *   float   length
 *   float   mx
 *   float   my
 *   uint8   state       LineState enum value
 *   uint8   confidence  0-100
 *   uint8   observed    capped at 255
 *   uint8   _pad
 *
 * PoseWire  (12 bytes)
 *   float   x
 *   float   y
 *   float   theta
 *
 * Total worst-case per scan: 12 + 500*28 = 14012 bytes ~ 14KB
 * At 10Hz: ~140KB/s — well within WiFi bandwidth
 * ----------------------------------------------------------------------- */

#define MAP_PUBLISH_MAGIC  0x534C414D

typedef struct {
    uint32_t magic;
    uint32_t n_lines;
    uint32_t scan_count;
} MapPublishHeader;

typedef struct {
    float   angle;
    float   distance;
    float   length;
    float   mx;
    float   my;
    uint8_t state;
    uint8_t confidence;
    uint8_t observed;    /* capped at 255 */
    uint8_t _pad;
} MapLineWire;

typedef struct {
    float x;
    float y;
    float theta;
} PoseWire;

/* -----------------------------------------------------------------------
 * MapPublisher
 * Opaque handle — allocate with map_publisher_create().
 * ----------------------------------------------------------------------- */
typedef struct MapPublisher MapPublisher;

/* -----------------------------------------------------------------------
 * map_publisher_create()
 *
 * Initialise Zenoh session and declare publishers for all three topics.
 * locator: Zenoh locator string, e.g. "tcp/192.168.1.100:7447"
 *          Pass NULL to use default multicast discovery.
 *
 * Returns NULL on failure (Zenoh init error).
 * ----------------------------------------------------------------------- */
MapPublisher *map_publisher_create(const char *locator);

/* -----------------------------------------------------------------------
 * map_publisher_publish()
 *
 * Serialize the map and pose, then zenoh_put() on all three topics.
 * Call once per scan cycle after map_update().
 *
 * Internally splits lines into STATIC and DYNAMIC buffers and publishes
 * them on separate topics so RViz can show them with different colours
 * without any filtering on the Python side.
 * ----------------------------------------------------------------------- */
void map_publisher_publish(MapPublisher  *pub,
                           const Map     *map,
                           const Pose    *pose);

/* -----------------------------------------------------------------------
 * map_publisher_destroy()
 * Close Zenoh session and free resources.
 * ----------------------------------------------------------------------- */
void map_publisher_destroy(MapPublisher *pub);

/* -----------------------------------------------------------------------
 * map_serialize_to_buf()
 *
 * Pure serialization — no Zenoh. Writes into caller-supplied buffer.
 * Exposed for unit testing and the bag replayer.
 *
 * filter_state: only serialize lines with this state.
 *               Pass -1 to serialize all active lines.
 *
 * Returns number of bytes written, or -1 if buf_size too small.
 * ----------------------------------------------------------------------- */
int map_serialize_to_buf(const Map *map,
                         int        filter_state,
                         uint8_t   *buf,
                         int        buf_size);

#ifdef __cplusplus
}
#endif

#endif /* MAP_PUBLISHER_H */