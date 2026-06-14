/**
 * gc_ra_protocol.h
 * 
 * Binary protocol definition for GameCube RetroAchievements via ESP32-S3 (EXI/SPI)
 * 
 * This protocol replaces the text-based UART protocol from fpga-ra-adapter
 * with a compact binary protocol optimized for the GameCube's EXI bus (SPI).
 * 
 * Key differences from fpga-ra-adapter:
 * - Binary instead of ASCII text
 * - 32-bit addresses (GameCube) instead of 16-bit (NES)
 * - Snapshot-based (GC reads RAM and sends values) instead of write-notification-based
 * - SPI slave instead of UART serial
 * 
 * Communication flow:
 *   1. GameCube detects ESP32-RA on EXI bus via device ID query
 *   2. GameCube sends game identification (Game ID + optional hash)
 *   3. ESP32 loads game from RA, discovers memory addresses, sends watch list
 *   4. Every VBlank: GameCube reads watched addresses, sends snapshot to ESP32
 *   5. ESP32 runs rc_client_do_frame(), sends back events (achievements, etc.)
 *   6. ESP32 asks GC to read specific addresses from RAM.
 */

#ifndef GC_RA_PROTOCOL_H
#define GC_RA_PROTOCOL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * ============================================================================
 * Protocol Constants
 * ============================================================================
 */

/** Device ID returned by ESP32-RA when queried via EXI device detection */
#define RA_DEVICE_ID            0x52410001  /* "RA" + version 0.0.1 */

/** Protocol version */
#define RA_PROTOCOL_VERSION     0x01

/** Magic bytes for packet validation */
#define RA_MAGIC_GC_TO_ESP      0x52  /* 'R' - GameCube to ESP32 */
#define RA_MAGIC_ESP_TO_GC      0xAE  /* ESP32 to GameCube */

/** Maximum number of watched addresses.
 *  6144 since v0.24.7 — SMG's LIVE per-frame working set measured
 *  ~3700 on hw (119 active achievements x ~30 chain bytes each), which
 *  sat exactly ON the old 4096/3800 ceiling: eviction at capacity has
 *  no cold entries to pick, so it dropped live bytes, the evaluator
 *  re-requested them next frame, and the system livelocked. Capacity
 *  must sit comfortably above live demand.
 *  KEEP IN SYNC with the copy of this header in wii-ra-adapter/. */
#define RA_MAX_WATCH_ADDRS      6144

/** Number of addresses per watchlist chunk (fits in one EXI transaction) */
#define RA_WATCHLIST_CHUNK_ADDRS  1024

/** Maximum response data payload size */
#define RA_MAX_RESPONSE_DATA    256

/** Maximum achievement title length */
#define RA_MAX_TITLE_LEN        128

/** Game ID length (GameCube disc header) */
#define RA_GAME_ID_LEN          6

/** MD5 hash length */
#define RA_HASH_LEN             32

/*
 * ============================================================================
 * Command Types: GameCube → ESP32
 * ============================================================================
 */

typedef enum {
    /** Query device identity - ESP32 responds with RA_DEVICE_ID */
    RA_CMD_IDENTIFY         = 0x01,

    /** Send game identification data to ESP32 */
    RA_CMD_LOAD_GAME        = 0x02,

    /** Send memory snapshot (values of all watched addresses) */
    RA_CMD_SNAPSHOT          = 0x03,

    /** Poll for pending events/responses from ESP32 */
    RA_CMD_POLL              = 0x04,

    /** Notify ESP32 that the game was reset/changed */
    RA_CMD_GAME_RESET        = 0x05,

    /** Request current status from ESP32 */
    RA_CMD_STATUS            = 0x06,

    /**
     * Request one chunk of the pending watchlist update.
     */
    RA_CMD_GET_WATCHLIST_CHUNK = 0x07,

    /**
     * Send queried memory values back to ESP32.
     */
    RA_CMD_ADDR_RESPONSE     = 0x08,

    /**
     * Send a debug message (ASCII text) to ESP32 — ESP32 logs it via Serial.
     * Payload format: u8 msg_len, then msg_len bytes of ASCII text.
     * Use for diagnostics from custom IOS modules where LED-encoding numbers
     * is fragile / impractical.
     */
    RA_CMD_DEBUG_LOG         = 0x09,
} ra_gc_command_t;

/*
 * ============================================================================
 * Event Types: ESP32 → GameCube
 * ============================================================================
 */

typedef enum {
    /** No pending event */
    RA_EVT_NONE              = 0x00,

    /** Achievement triggered - data contains achievement info */
    RA_EVT_ACHIEVEMENT       = 0x01,

    /**
     * Watchlist changed - GC must fetch new list via RA_CMD_GET_WATCHLIST_CHUNK.
     */
    RA_EVT_WATCHLIST_UPDATE  = 0x02,

    /** Game info loaded - data contains game title */
    RA_EVT_GAME_INFO         = 0x03,

    /** Login status update */
    RA_EVT_LOGIN_STATUS      = 0x04,

    /** Error occurred */
    RA_EVT_ERROR             = 0x05,

    /** Leaderboard started */
    RA_EVT_LEADERBOARD_START = 0x06,

    /** Leaderboard submitted */
    RA_EVT_LEADERBOARD_SUBMIT = 0x07,

    /** Challenge indicator show/hide */
    RA_EVT_CHALLENGE         = 0x08,

    /** Rich presence update */
    RA_EVT_RICH_PRESENCE     = 0x09,

    /**
     * ESP32 asks GC to read specific addresses from RAM.
     */
    RA_EVT_ADDR_QUERY        = 0x0A,

    /**
     * ESP32 announces NEW addresses to APPEND to the GC's local watchlist.
     * After receiving, ra-module must include these addresses in all future
     * SNAPSHOTs (watchlist grows additively — never shrinks via this event).
     * Payload: ra_watchlist_append_t followed by addr_count u32 addresses (BE).
     * This avoids a GET_CHUNK round-trip for incremental growth and keeps
     * ra-module/ESP32 in sync without wiping memory_data.
     */
    RA_EVT_WATCHLIST_APPEND  = 0x0B,

    /**
     * ESP32 evicted dynamic addresses (LRU) and tells ra-module to TRUNCATE
     * its local watchlist to `keep_count` entries. ra-module discards
     * ra_watchlist[keep_count..ra_watchlist_count-1] and updates its count.
     * DEPRECATED in favor of RA_EVT_WATCHLIST_REMOVE — TRUNCATE drops the
     * tail wholesale which would evict byte-fanouts of static base addresses
     * along with stale chain leaves, breaking trigger evaluation.
     */
    RA_EVT_WATCHLIST_TRUNCATE = 0x0C,

    /**
     * ESP32 evicted specific addresses (LRU-selected) and tells ra-module
     * to remove them from its local watchlist. ra-module finds each address
     * by linear scan in ra_watchlist[], removes by shifting remaining entries
     * down (defrag — preserves insertion order of survivors). Both sides
     * arrive at identical compacted arrays.
     *
     * Payload: ra_watchlist_remove_t (u16 addr_count, BE) followed by
     * addr_count × u32 addresses (BE).
     *
     * Insertion order on both sides must match — ESP32 evicts the same
     * addresses on its side BEFORE sending REMOVE, so the resulting
     * compacted arrays match position-by-position. SNAPSHOT memcpy depends
     * on this alignment.
     *
     * Parse MUST be in both ra_send_snapshot AND ra_send_addr_response
     * (twin function divergence hazard).
     */
    RA_EVT_WATCHLIST_REMOVE  = 0x0D,

    /* Watchlist removal by INDEX (v0.25.0 — replaces address-based
     * REMOVE on the hot path). Both replicas are identical before the
     * removal, so the ESP sends the u16 array indices of the entries to
     * drop (ascending order) instead of their u32 addresses. ra-module
     * marks by index directly — O(R) instead of the O(R*N) linear
     * address search that cost ~25-30ms (≈2 frames!) at N=6144 — then
     * compacts in one pass. Index identity also eliminates the entire
     * address-ambiguity bug class (duplicate addresses, hash-tombstone
     * off-by-N).
     *
     * Payload: ra_watchlist_remove_idx_t (u16 idx_count, BE) followed
     * by idx_count x u16 indices (BE, ascending, all < current count).
     *
     * Parse MUST be in both ra_send_snapshot AND ra_send_addr_response
     * (twin function divergence hazard). */
    RA_EVT_WATCHLIST_REMOVE_IDX = 0x0E,
} ra_esp_event_t;

/**
 * Internal ESP32 event structure for the pending events queue.
 */
typedef struct {
    uint8_t type;       /* ra_esp_event_t */
    uint8_t data[RA_MAX_RESPONSE_DATA];
    uint16_t data_len;
} PendingEvent;

/*
 * ============================================================================
 * Status Codes
 * ============================================================================
 */

typedef enum {
    RA_STATUS_INITIALIZING   = 0x00,
    RA_STATUS_WIFI_CONNECTING = 0x01,
    RA_STATUS_WIFI_CONNECTED = 0x02,
    RA_STATUS_LOGGING_IN     = 0x03,
    RA_STATUS_LOGGED_IN      = 0x04,
    RA_STATUS_LOADING_GAME   = 0x05,
    RA_STATUS_GAME_LOADED    = 0x06,
    RA_STATUS_ACTIVE         = 0x07,  /* Fully operational, processing frames */
    RA_STATUS_ERROR_WIFI     = 0xE0,
    RA_STATUS_ERROR_LOGIN    = 0xE1,
    RA_STATUS_ERROR_GAME     = 0xE2,
    RA_STATUS_ERROR_PROTOCOL = 0xE3,
} ra_status_t;

/*
 * ============================================================================
 * Packet Structures: GameCube → ESP32
 * ============================================================================
 */

/**
 * Header for all GC→ESP32 packets
 * Sent at the start of every EXI transaction
 */
typedef struct __attribute__((packed)) {
    uint8_t  magic;          /* RA_MAGIC_GC_TO_ESP (0x52) */
    uint8_t  command;        /* ra_gc_command_t */
    uint16_t payload_len;    /* Length of data following this header */
} ra_gc_header_t;

/**
 * RA_CMD_LOAD_GAME payload
 */
typedef struct __attribute__((packed)) {
    char     game_id[RA_GAME_ID_LEN];  /* 6-byte Game ID from disc header */
    uint8_t  disc_number;
    uint8_t  has_hash;
    char     md5_hash[RA_HASH_LEN];
} ra_load_game_t;

/**
 * RA_CMD_SNAPSHOT payload
 *
 * wl_seq (v0.26.0 Phase D2): the sequence number of the LAST watchlist
 * mutation the Wii has applied. The ESP compares it against its own
 * emitted seq every frame — sync is VERIFIED, not inferred from count
 * equality. Behind → resend exactly the next mutation (idempotent, the
 * Wii drops seq != ra_seq+1); impossible state → full RESYNC.
 */
typedef struct __attribute__((packed)) {
    uint32_t frame_counter;
    uint16_t addr_count;
    uint16_t wl_seq;
} ra_snapshot_header_t;

/*
 * ============================================================================
 * Packet Structures: ESP32 → GameCube
 * ============================================================================
 */

/**
 * Header for all ESP32→GC responses
 * Always sent as the response in the same EXI transaction
 */
typedef struct __attribute__((packed)) {
    uint8_t  magic;          /* RA_MAGIC_ESP_TO_GC (0xAE) */
    uint8_t  status;         /* ra_status_t */
    uint8_t  event_type;     /* ra_esp_event_t */
    uint8_t  event_count;
    uint16_t data_len;
} ra_esp_header_t;

/**
 * RA_EVT_WATCHLIST_UPDATE notification data (6 bytes)
 * seq (v0.26.0): the mutation-sequence base of this (re)load — after the
 * chunk fetch completes, BOTH sides hold this seq and the list it names.
 */
typedef struct __attribute__((packed)) {
    uint16_t total_addr_count;
    uint16_t num_chunks;
    uint16_t seq;
} ra_watchlist_notify_t;

/**
 * RA_CMD_GET_WATCHLIST_CHUNK request payload
 */
typedef struct __attribute__((packed)) {
    uint16_t chunk_index;
} ra_watchlist_chunk_req_t;

/**
 * RA_CMD_GET_WATCHLIST_CHUNK response header
 */
typedef struct __attribute__((packed)) {
    uint16_t chunk_index;
    uint16_t addr_count;
    uint8_t  is_last;
    uint8_t  reserved;
} ra_watchlist_chunk_t;

/**
 * RA_EVT_WATCHLIST_UPDATE data (legacy)
 */
typedef struct __attribute__((packed)) {
    uint16_t addr_count;
} ra_watchlist_t;

/**
 * RA_EVT_ADDR_QUERY data
 */
typedef struct __attribute__((packed)) {
    uint16_t addr_count;
} ra_addr_query_t;

/**
 * RA_EVT_WATCHLIST_APPEND data — same wire format as ADDR_QUERY:
 * count followed by `count` u32 addresses (BE). Distinct struct so the
 * intent is clear at call sites.
 */
typedef struct __attribute__((packed)) {
    uint16_t seq;        /* v0.26.0: mutation seq — Wii applies iff seq == ra_seq+1 */
    uint16_t addr_count;
} ra_watchlist_append_t;

/**
 * RA_EVT_WATCHLIST_TRUNCATE data — single u16 telling ra-module the
 * new watchlist length. DEPRECATED — see RA_EVT_WATCHLIST_REMOVE.
 */
typedef struct __attribute__((packed)) {
    uint16_t keep_count;
} ra_watchlist_truncate_t;

/**
 * RA_EVT_WATCHLIST_REMOVE data — addr_count followed by addr_count u32
 * addresses (BE). ra-module locates each address in ra_watchlist[] and
 * removes it (shifts subsequent entries down). Insertion order of survivors
 * preserved on both sides so SNAPSHOT positions stay aligned.
 */
typedef struct __attribute__((packed)) {
    uint16_t addr_count;
} ra_watchlist_remove_t;

/**
 * RA_EVT_WATCHLIST_REMOVE_IDX data — idx_count followed by idx_count u16
 * array indices (BE, ascending, all < current watchlist count). ra-module
 * marks the slots directly and compacts in a single pass.
 */
typedef struct __attribute__((packed)) {
    uint16_t seq;        /* v0.26.0: mutation seq — Wii applies iff seq == ra_seq+1 */
    uint16_t idx_count;
} ra_watchlist_remove_idx_t;

/**
 * RA_CMD_ADDR_RESPONSE data
 */
typedef struct __attribute__((packed)) {
    uint16_t addr_count;
} ra_addr_response_t;

/**
 * RA_EVT_ACHIEVEMENT data
 */
typedef struct __attribute__((packed)) {
    uint32_t achievement_id;
    uint8_t  title_len;
} ra_achievement_t;

/**
 * RA_EVT_GAME_INFO data
 */
typedef struct __attribute__((packed)) {
    uint32_t game_id;
    uint16_t achievement_count;
    uint8_t  title_len;
} ra_game_info_t;

/**
 * RA_EVT_LEADERBOARD_START / RA_EVT_LEADERBOARD_SUBMIT data
 */
typedef struct __attribute__((packed)) {
    uint32_t leaderboard_id;
    uint8_t  title_len;
} ra_leaderboard_t;

#define RA_SNAPSHOT_PACKET_SIZE(n) \
    (sizeof(ra_gc_header_t) + sizeof(ra_snapshot_header_t) + (n))

#define RA_WATCHLIST_CHUNK_RESPONSE_SIZE(n) \
    (sizeof(ra_esp_header_t) + sizeof(ra_watchlist_chunk_t) + (n) * sizeof(uint32_t))

#define RA_WATCHLIST_NUM_CHUNKS(n) \
    (((n) + RA_WATCHLIST_CHUNK_ADDRS - 1) / RA_WATCHLIST_CHUNK_ADDRS)

#define RA_MIN_RESPONSE_SIZE    sizeof(ra_esp_header_t)

#ifdef ESP_PLATFORM
static inline uint16_t ra_be16_to_host(uint16_t v) {
    return __builtin_bswap16(v);
}
static inline uint32_t ra_be32_to_host(uint32_t v) {
    return __builtin_bswap32(v);
}
static inline uint16_t ra_host_to_be16(uint16_t v) {
    return __builtin_bswap16(v);
}
static inline uint32_t ra_host_to_be32(uint32_t v) {
    return __builtin_bswap32(v);
}
#else
#define ra_be16_to_host(v) (v)
#define ra_be32_to_host(v) (v)
#define ra_host_to_be16(v) (v)
#define ra_host_to_be32(v) (v)
#endif

#ifdef __cplusplus
}
#endif

#endif /* GC_RA_PROTOCOL_H */
