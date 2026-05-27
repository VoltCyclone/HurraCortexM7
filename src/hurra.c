// src/hurra.c — Hurra binary protocol parser, TinyFrame-based.
// See docs/specs/2026-05-23-hurra-binary-protocol-design.md
#include "TinyFrame.h"
#include "hurra.h"
#include "actions.h"
#include "kmbox.h"
#include "imxrt.h"
#include <string.h>

extern uint32_t millis(void);

// ── Command TYPE byte allocation (see spec §3) ──────────────────────────────
enum {
    // 0x00–0x0F admin
    TYPE_PING        = 0x00,
    TYPE_VERSION     = 0x01,
    TYPE_STATS       = 0x02,
    TYPE_INIT        = 0x03,
    TYPE_REBOOT      = 0x04,
    TYPE_BAUD        = 0x05,
    TYPE_SCREEN      = 0x06,
    // 0x10–0x2F mouse
    TYPE_MOUSE_MOVE        = 0x10,
    TYPE_MOUSE_MOVE_SMOOTH = 0x11,
    TYPE_MOUSE_SILENT_MOVE = 0x12,
    TYPE_MOUSE_MO          = 0x13,
    TYPE_MOUSE_CLICK       = 0x14,
    TYPE_MOUSE_WHEEL       = 0x15,
    TYPE_MOUSE_GETPOS      = 0x16,
    TYPE_INVERT_X          = 0x17,
    TYPE_INVERT_Y          = 0x18,
    TYPE_SWAP_XY           = 0x19,
    TYPE_BTN_LEFT          = 0x20,
    TYPE_BTN_RIGHT         = 0x21,
    TYPE_BTN_MIDDLE        = 0x22,
    TYPE_BTN_SIDE1         = 0x23,
    TYPE_BTN_SIDE2         = 0x24,
    // 0x40–0x4F keyboard
    TYPE_KB_DOWN        = 0x40,
    TYPE_KB_UP          = 0x41,
    TYPE_KB_PRESS       = 0x42,
    TYPE_KB_ISDOWN      = 0x43,
    TYPE_KB_MASK        = 0x44,
    TYPE_KB_STRING      = 0x45,
    TYPE_KB_MULTIDOWN   = 0x46,
    TYPE_KB_MULTIUP     = 0x47,
    TYPE_KB_MULTIPRESS  = 0x48,
    // 0x60–0x6F locks + catch
    TYPE_LOCK_ML  = 0x60,
    TYPE_LOCK_MR  = 0x61,
    TYPE_LOCK_MM  = 0x62,
    TYPE_LOCK_MS1 = 0x63,
    TYPE_LOCK_MS2 = 0x64,
    TYPE_LOCK_MX  = 0x65,
    TYPE_LOCK_MY  = 0x66,
    TYPE_CATCH_XY = 0x67,
    // 0x70–0x7F streams + callbacks
    TYPE_STREAM_AXIS  = 0x70,
    TYPE_STREAM_BTN   = 0x71,
    TYPE_STREAM_MOUSE = 0x72,
    TYPE_STREAM_KB    = 0x73,
    TYPE_CB_BUTTONS   = 0x74,
    TYPE_CB_AXES      = 0x75,
    TYPE_CB_KEYS      = 0x76,
    // 0x80–0x8F unsolicited telemetry
    TYPE_TLM_AXIS    = 0x80,
    TYPE_TLM_BUTTONS = 0x81,
    TYPE_TLM_MOUSE   = 0x82,
    TYPE_TLM_KB      = 0x83,
    TYPE_TLM_STATS   = 0x84,
    TYPE_TLM_LOG     = 0x85,
};

#define HURRA_IDENTITY "kmbox: Hurra v1"

// ── TinyFrame instance ──────────────────────────────────────────────────────
static TinyFrame s_tf;
static hurra_tx_fn s_tx;

void TF_WriteImpl(TinyFrame *tf, const uint8_t *buf, uint32_t len)
{
    (void)tf;
    if (s_tx) s_tx(buf, (uint16_t)len);
}

// ── stats counters (exposed via STATS frame in Phase 6) ─────────────────────
static uint32_t s_rx_frames_ok;
static uint32_t s_head_crc_err;
static uint32_t s_payload_crc_err;
static uint32_t s_id_gap_total;
static uint32_t s_idle_resync;
static uint32_t s_payload_invalid;
static uint16_t s_tx_ring_high_water;
static uint32_t s_tx_ring_skip;
static uint8_t  s_last_rx_id;
static bool     s_have_last_id;

// ── deferred actions ─────────────────────────────────────────────────────────
static uint32_t s_reboot_at;
static uint32_t s_baud_pending;
static uint32_t s_baud_apply_at;

// ── stream/callback state ───────────────────────────────────────────────────
typedef struct { uint8_t mode; uint8_t period_ms; uint32_t last_ms; } stream_t;
static stream_t s_str_axis, s_str_btn, s_str_mouse, s_str_kb;
static uint8_t  s_cb_buttons, s_cb_axes, s_cb_keys;

// ── snapshots populated by hurra_notify_* ───────────────────────────────────
static uint8_t  s_snap_buttons;
static int16_t  s_snap_dx, s_snap_dy;
static int8_t   s_snap_wheel;
static uint8_t  s_snap_keys[6];
static uint8_t  s_last_btn_emitted = 0;
static uint8_t  s_last_keys_emitted[6];

// ── screen ──────────────────────────────────────────────────────────────────
static int16_t s_screen_w, s_screen_h;

// ── catch_xy ────────────────────────────────────────────────────────────────
static struct {
    bool     active;
    uint32_t deadline;
    int32_t  accum_x, accum_y;
    uint8_t  reply_id;
} s_catch;

// ── auto stats push ─────────────────────────────────────────────────────────
#define STATS_PERIOD_MS 100
static uint32_t s_stats_next_ms;

// ── helpers ─────────────────────────────────────────────────────────────────
static inline int16_t  rd_i16le(const uint8_t *p) { return (int16_t)(p[0] | (p[1] << 8)); }
static inline uint16_t rd_u16le(const uint8_t *p) { return (uint16_t)(p[0] | (p[1] << 8)); }
static inline uint32_t rd_u32le(const uint8_t *p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

// ── helpers ──────────────────────────────────────────────────────────────────

static void send_reply(TF_Msg *req, const uint8_t *data, uint32_t len)
{
    TF_Msg r = *req;
    r.data = (uint8_t *)data;
    r.len  = (TF_LEN)len;
    r.is_response = true;
    TF_Respond(&s_tf, &r);
}

// Track ID gap for stats (called by every listener)
static void track_id(uint8_t id)
{
    s_rx_frames_ok++;
    if (s_have_last_id) {
        uint8_t gap = (uint8_t)((id - s_last_rx_id - 1) & 0xFFu);
        s_id_gap_total += gap;
    }
    s_last_rx_id = id;
    s_have_last_id = true;
}

// ── public API ──────────────────────────────────────────────────────────────
void hurra_init(void)
{
    memset(&s_tf, 0, sizeof(s_tf));
    TF_InitStatic(&s_tf, TF_MASTER);
    s_tx = NULL;
    s_rx_frames_ok = s_head_crc_err = s_payload_crc_err = 0;
    s_id_gap_total = s_idle_resync = s_payload_invalid = 0;
    s_tx_ring_high_water = 0;
    s_tx_ring_skip = 0;
    s_have_last_id = false;
    s_reboot_at = 0;
    s_baud_pending = 0;
    s_baud_apply_at = 0;
    memset(&s_str_axis, 0, sizeof(s_str_axis));
    memset(&s_str_btn,  0, sizeof(s_str_btn));
    memset(&s_str_mouse, 0, sizeof(s_str_mouse));
    memset(&s_str_kb,   0, sizeof(s_str_kb));
    s_cb_buttons = s_cb_axes = s_cb_keys = 0;
    s_snap_buttons = 0; s_snap_dx = s_snap_dy = 0; s_snap_wheel = 0;
    memset(s_snap_keys, 0, sizeof(s_snap_keys));
    memset(s_last_keys_emitted, 0, sizeof(s_last_keys_emitted));
    s_screen_w = s_screen_h = 0;
    memset(&s_catch, 0, sizeof(s_catch));
    s_stats_next_ms = STATS_PERIOD_MS;
    // Listeners registered in Phase 4-6 tasks.
}

void hurra_reset(void) { TF_ResetParser(&s_tf); }
void hurra_set_tx(hurra_tx_fn tx) { s_tx = tx; }

void hurra_feed_byte(uint8_t b) { TF_AcceptChar(&s_tf, b); }

void hurra_tick(void)
{
    uint32_t now = millis();
    TF_Tick(&s_tf);
    // Auto-stats push, stream emits, deferred actions land here in later tasks.
    if (s_reboot_at && now >= s_reboot_at) {
        SCB_AIRCR = 0x05FA0004;  // ARM SYSRESETREQ (macro from imxrt.h)
    }
}

void hurra_notify_buttons(uint8_t buttons) { s_snap_buttons = buttons; }
void hurra_notify_axes(int16_t dx, int16_t dy, int8_t scroll)
{
    s_snap_dx = dx; s_snap_dy = dy; s_snap_wheel = scroll;
    if (s_catch.active) { s_catch.accum_x += dx; s_catch.accum_y += dy; }
}
void hurra_notify_keys(const uint8_t keys[6]) { memcpy(s_snap_keys, keys, 6); }
