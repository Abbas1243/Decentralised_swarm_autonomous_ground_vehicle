#include "map_publisher.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

/* -----------------------------------------------------------------------
 * Compile-time Zenoh guard
 *
 * Build WITH Zenoh (default for Duo S / PC WiFi publish):
 *   gcc ... -DSLAM_WITH_ZENOH -lzenoh_pico ...
 *
 * Build WITHOUT Zenoh (unit tests, bag replayer, no-WiFi builds):
 *   gcc ... (no flag) — serialization works, publish is a no-op fprintf
 * ----------------------------------------------------------------------- */
#ifdef SLAM_WITH_ZENOH
#  include <zenoh-pico.h>
#endif

/* -----------------------------------------------------------------------
 * Internal buffer sizes
 *
 * Worst case: 500 lines × 28 bytes + 12 byte header = 14012 bytes.
 * We allocate two buffers (static + dynamic) — same worst case each.
 * ----------------------------------------------------------------------- */
#define MAX_WIRE_BUF  (sizeof(MapPublishHeader) + MAX_MAP_LINES * sizeof(MapLineWire))

struct MapPublisher {
#ifdef SLAM_WITH_ZENOH
    z_owned_session_t    session;
    z_owned_publisher_t  pub_static;
    z_owned_publisher_t  pub_dynamic;
    z_owned_publisher_t  pub_pose;
#endif
    uint8_t  static_buf[MAX_WIRE_BUF];
    uint8_t  dynamic_buf[MAX_WIRE_BUF];
    uint8_t  pose_buf[sizeof(PoseWire)];
    int      initialised;
};

/* -----------------------------------------------------------------------
 * map_serialize_to_buf()
 * Pure serialization — no Zenoh dependency.
 * ----------------------------------------------------------------------- */
int map_serialize_to_buf(const Map *map,
                         int        filter_state,
                         uint8_t   *buf,
                         int        buf_size)
{
    /* Count matching lines first */
    uint32_t n = 0;
    for (int i = 0; i < map->n_lines; i++) {
        const MapLine *ml = &map->lines[i];
        if (!ml->active) continue;
        if (filter_state >= 0 && (int)ml->state != filter_state) continue;
        n++;
    }

    int needed = (int)(sizeof(MapPublishHeader) + n * sizeof(MapLineWire));
    if (needed > buf_size) return -1;

    /* Header */
    MapPublishHeader *hdr = (MapPublishHeader *)buf;
    hdr->magic      = MAP_PUBLISH_MAGIC;
    hdr->n_lines    = n;
    hdr->scan_count = (uint32_t)map->scan_count;

    /* Lines */
    MapLineWire *wire = (MapLineWire *)(buf + sizeof(MapPublishHeader));
    for (int i = 0; i < map->n_lines; i++) {
        const MapLine *ml = &map->lines[i];
        if (!ml->active) continue;
        if (filter_state >= 0 && (int)ml->state != filter_state) continue;

        wire->angle      = ml->angle;
        wire->distance   = ml->distance;
        wire->length     = ml->length;
        wire->mx         = ml->mx;
        wire->my         = ml->my;
        wire->state      = (uint8_t)ml->state;
        wire->confidence = (uint8_t)(ml->confidence > 100 ? 100 : ml->confidence);
        wire->observed   = (uint8_t)(ml->observed   > 255 ? 255 : ml->observed);
        wire->_pad       = 0;
        wire++;
    }

    return needed;
}

/* -----------------------------------------------------------------------
 * map_publisher_create()
 * ----------------------------------------------------------------------- */
MapPublisher *map_publisher_create(const char *locator)
{
    MapPublisher *pub = (MapPublisher *)calloc(1, sizeof(MapPublisher));
    if (!pub) return NULL;

#ifdef SLAM_WITH_ZENOH
    z_owned_config_t config = z_config_default();

    if (locator != NULL) {
        /* Point-to-point: connect directly to the PC running the bridge */
        zp_config_insert(z_loan(config),
                         Z_CONFIG_CONNECT_KEY,
                         z_string_make(locator));
    }
    /* else: multicast peer discovery (works on local network) */

    if (z_open(&pub->session, z_move(config)) < 0) {
        fprintf(stderr, "[map_pub] zenoh open failed\n");
        free(pub);
        return NULL;
    }

    /* Declare publishers — QoS: best-effort, no history needed for live map */
    z_publisher_options_t opts = z_publisher_options_default();
    opts.congestion_control    = Z_CONGESTION_CONTROL_DROP;

    z_owned_keyexpr_t ke_static  = z_keyexpr_new(TOPIC_STATIC_MAP);
    z_owned_keyexpr_t ke_dynamic = z_keyexpr_new(TOPIC_DYNAMIC_LINES);
    z_owned_keyexpr_t ke_pose    = z_keyexpr_new(TOPIC_POSE);

    if (z_declare_publisher(&pub->pub_static,  z_loan(pub->session),
                            z_loan(ke_static),  &opts) < 0 ||
        z_declare_publisher(&pub->pub_dynamic, z_loan(pub->session),
                            z_loan(ke_dynamic), &opts) < 0 ||
        z_declare_publisher(&pub->pub_pose,    z_loan(pub->session),
                            z_loan(ke_pose),    &opts) < 0) {
        fprintf(stderr, "[map_pub] failed to declare publishers\n");
        z_close(z_move(pub->session));
        free(pub);
        return NULL;
    }

    z_drop(z_move(ke_static));
    z_drop(z_move(ke_dynamic));
    z_drop(z_move(ke_pose));

    /* Start background Zenoh read/lease tasks */
    zp_start_read_task(z_loan(pub->session), NULL);
    zp_start_lease_task(z_loan(pub->session), NULL);

    printf("[map_pub] Zenoh session open — publishing on %s / %s / %s\n",
           TOPIC_STATIC_MAP, TOPIC_DYNAMIC_LINES, TOPIC_POSE);
#else
    printf("[map_pub] Built without Zenoh — publish is a no-op "
           "(recompile with -DSLAM_WITH_ZENOH to enable)\n");
    (void)locator;
#endif

    pub->initialised = 1;
    return pub;
}

/* -----------------------------------------------------------------------
 * map_publisher_publish()
 * ----------------------------------------------------------------------- */
void map_publisher_publish(MapPublisher  *pub,
                           const Map     *map,
                           const Pose    *pose)
{
    if (!pub || !pub->initialised) return;

    /* ── Serialize static lines ── */
    int static_bytes = map_serialize_to_buf(map,
                                            LINE_STATIC,
                                            pub->static_buf,
                                            (int)sizeof(pub->static_buf));

    /* ── Serialize dynamic lines ── */
    int dynamic_bytes = map_serialize_to_buf(map,
                                             LINE_DYNAMIC,
                                             pub->dynamic_buf,
                                             (int)sizeof(pub->dynamic_buf));

    /* ── Serialize pose ── */
    PoseWire *pw = (PoseWire *)pub->pose_buf;
    pw->x     = pose->x;
    pw->y     = pose->y;
    pw->theta = pose->theta;

#ifdef SLAM_WITH_ZENOH
    /* Put static map */
    if (static_bytes > 0) {
        z_publisher_put_options_t opts = z_publisher_put_options_default();
        z_publisher_put(z_loan(pub->pub_static),
                        pub->static_buf, (size_t)static_bytes, &opts);
    }

    /* Put dynamic obstacles */
    if (dynamic_bytes > 0) {
        z_publisher_put_options_t opts = z_publisher_put_options_default();
        z_publisher_put(z_loan(pub->pub_dynamic),
                        pub->dynamic_buf, (size_t)dynamic_bytes, &opts);
    }

    /* Put pose */
    {
        z_publisher_put_options_t opts = z_publisher_put_options_default();
        z_publisher_put(z_loan(pub->pub_pose),
                        pub->pose_buf, sizeof(PoseWire), &opts);
    }

#else
    /* No Zenoh — print stats to stderr so the loop still gives feedback */
    if (map->scan_count % 30 == 0) {  /* every 3 seconds at 10Hz */
        const MapPublishHeader *sh = (const MapPublishHeader *)pub->static_buf;
        const MapPublishHeader *dh = (const MapPublishHeader *)pub->dynamic_buf;
        fprintf(stderr,
                "[map_pub] scan=%u  static=%u lines (%d bytes)  "
                "dynamic=%u lines (%d bytes)  pose=(%.2f,%.2f,%.1f°)\n",
                map->scan_count,
                static_bytes  > 0 ? sh->n_lines : 0, static_bytes,
                dynamic_bytes > 0 ? dh->n_lines : 0, dynamic_bytes,
                (double)pose->x, (double)pose->y,
                (double)(pose->theta * 57.2958f));
    }
#endif
}

/* -----------------------------------------------------------------------
 * map_publisher_destroy()
 * ----------------------------------------------------------------------- */
void map_publisher_destroy(MapPublisher *pub)
{
    if (!pub) return;

#ifdef SLAM_WITH_ZENOH
    zp_stop_read_task(z_loan(pub->session));
    zp_stop_lease_task(z_loan(pub->session));
    z_undeclare_publisher(z_move(pub->pub_static));
    z_undeclare_publisher(z_move(pub->pub_dynamic));
    z_undeclare_publisher(z_move(pub->pub_pose));
    z_close(z_move(pub->session));
#endif

    free(pub);
}