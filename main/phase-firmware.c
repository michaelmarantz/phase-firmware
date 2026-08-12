// ════════════════════════════════════════════════════════════════════════
//  Phase firmware — edition00
//
//  Hardware:
//    SK6812 RGBW × 138 on GPIO 5 (silkscreen D3), reset button on GPIO 7
//    (silkscreen D5).
//    LED 0 sits at 12 o'clock; indices advance clockwise.
//    Only the W channel is driven for normal moon rendering; the B channel
//    is used for the boot-AP and Wi-Fi-connecting animations.
//
//  Boot:
//    First boot (no saved creds) → open "phase" AP, slow blue moon-cycle
//    animation, captive setup page at phase.local. After credentials are
//    saved we restart into STA mode, play 3 slow blue pulses while we
//    connect, then drop into the normal moon render.
//
//  Web:
//    AP mode  : /  setup form, /scan JSON, /setup POST → save → restart.
//    STA mode : / landing, /debug auth-protected control panel with the
//               existing curve / brightness / face-gradient sliders plus
//               phase scrubber, date picker, manual/real toggle.
//
//  Reset button: hold GPIO 7 (silkscreen D5) for 3 s to erase Wi-Fi creds
//    and reboot to AP mode. Active-low, internal pull-up.
//
//  Note on UART: console logs flow over the secondary USB-Serial/JTAG
//    console (GPIO 18/19), so we never depend on UART0 (GPIO 20/21) at
//    runtime — both default UART pins are free for whatever the board
//    wires them to.
// ════════════════════════════════════════════════════════════════════════

#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <sys/time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_random.h"
#include "esp_mac.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_sntp.h"
#include "esp_http_server.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "mdns.h"
#include "led_strip.h"
#include "esp_ota_ops.h"
#include "esp_app_desc.h"
#include "esp_https_ota.h"
#include "esp_http_client.h"
#include "esp_crt_bundle.h"

// ── Hardware ───────────────────────────────────────────────
// LED_GPIO: chip GPIO 5 — silkscreen label D3 on this board (Xiao-style
// ESP32-C3 pinout: D3=GPIO5, D5=GPIO7, D8=GPIO8, and so on).
// Chosen because it's a fully clean pin — no strapping, no UART, no USB,
// no flash — so there's no chance of an SK6812 pull-down interfering at
// boot the way GPIO 8 (a strapping pin) theoretically could.
// History: v1 board originally targeted GPIO 21 (broken trace); switched
// to GPIO 8 as a workaround; hand-soldered prototypes are now routed to
// GPIO 5 as the production choice.
#define LED_GPIO         5
#define BUTTON_GPIO      7   // labelled D5 on the phase board silkscreen
                             // (D-labels on this board are not direct
                             // GPIO numbers — confirmed via runtime scan)
#define LED_COUNT        138
#define LED_RMT_RES_HZ   10000000

// ── Identity ───────────────────────────────────────────────
// FW_VERSION is the single source of truth: the top-level CMakeLists
// extracts it into PROJECT_VER, which lands in the app descriptor that
// OTA uses for is-this-new comparisons. Bump it for every release.
#define FW_VERSION       "edition00.3"
// AP_SSID and MDNS_HOSTNAME are BOTH suffixed at runtime with the
// device's unique tag: either its MAC-derived hex (default) or the
// user's friendly name set via /debug. Final forms look like
// "phase-a94290" (default) or "phase-e00-2-3" (friendly).
#define NAME_PREFIX      "phase"
#define MDNS_INSTANCE    "Lunar Objects — Phase"

// ── Reset button ───────────────────────────────────────────
#define BUTTON_HOLD_MS   3000

// ── Debug portal credentials ───────────────────────────────
#define DEBUG_USER       "REDACTED_DEBUG_USER"
#define DEBUG_PASS       "REDACTED_DEBUG_PASS"

// ── OTA (automatic over-the-air updates) ───────────────────
// Every connected lamp silently polls OTA_DEFAULT_URL. The URL is a
// GitHub "latest release" asset link, so it always points at the newest
// published phase.bin without the device needing to talk to the GitHub
// API. If the image at the URL reports a different app-descriptor
// version than the running firmware, the lamp downloads it into the
// spare OTA slot and reboots into it. The URL can be overridden per
// device via NVS key "ota_url" (namespace "phase") for testing against
// a local server.
#define OTA_DEFAULT_URL        "https://github.com/michaelmarantz/phase-firmware/releases/latest/download/phase.bin"
#define OTA_FIRST_CHECK_DELAY_MS  (30 * 1000)          // settle after boot
#define OTA_CHECK_INTERVAL_MS     (6 * 60 * 60 * 1000) // then every 6 h
#define OTA_HTTP_TIMEOUT_MS       (15 * 1000)

// ── Rendering defaults ─────────────────────────────────────
#define GRADIENT_WIDTH         0.28f
#define DEFAULT_FACE_GRADIENT  0.0f    // no body-gradient by default
#define DEFAULT_BRIGHTNESS     1.0f
#define DEFAULT_FLOOR          0.0f    // no brightness floor by default
#define DEFAULT_GLIMMER_ON     true    // glimmer is the new normal
#define DEFAULT_GLIMMER_BASE   0.04f   // tuned for the 138-pixel ring
#define DEFAULT_GLIMMER_EDGE   1.36f
#define DEFAULT_GLIMMER_SPEED  2.60f
#define DEFAULT_CURVE_Q1       0.259f
#define DEFAULT_CURVE_G1       0.501f
#define DEFAULT_CURVE_G3       0.500f
#define DEFAULT_CURVE_Q3       0.255f
// USB-safe by default: 138 SK6812 × 20 mA × 0.30 ≈ 830 mA peak.
// Crank to 1.0 when running on the external APV-35-5 supply.
#define DEFAULT_POWER_CAP      0.30f

// ── Animation knobs ────────────────────────────────────────
#define RENDER_PERIOD_MS       20      // 50 FPS — smooth enough that slow brightness fades look continuous
#define BOOT_ANIM_DIM          0.50f   // boot blue brightness cap
#define BOOT_ANIM_CYCLE_SEC    25.0f   // seconds per new-moon → full-moon → new-moon cycle
#define CONNECTING_PULSE_HZ    0.25f   // pulse rate during ANIM_CONNECTING (4 s per full pulse)
#define CONNECTING_PULSE_DIM   0.65f   // peak brightness of the connecting pulse
#define CONNECTING_PULSE_COUNT 3
#define FAILED_PULSE_HZ        3.5f    // ~285 ms per pulse — quick, alarming
#define FAILED_PULSE_DIM       0.70f
#define FAILED_PULSE_COUNT     4
#define WIFI_CONNECT_TIMEOUT_MS  60000 // give up after 60 s of failed connects
#define MOON_FADE_IN_SEC       2.0f    // smoothstep ramp when entering ANIM_MOON
#define MODE_EXIT_FADE_SEC     0.4f    // fade-out applied to current mode when a switch is requested

// ── Logging tag ────────────────────────────────────────────
#define TAG "phase"

// ──────────────────────────────────────────────────────────
// Forward decls
// ──────────────────────────────────────────────────────────
static void nvs_save_params(void);
static void nvs_load_params(void);
static bool load_wifi_creds(char *ssid, size_t ssid_sz, char *pass, size_t pass_sz);
static void save_wifi_creds(const char *ssid, const char *pass);
static void clear_wifi_creds(void);
static void mdns_setup(void);
static void start_webserver_ap(void);
static void start_webserver_sta(void);
static float calc_moon_phase(void);
static const char *phase_name(float phase);

// ──────────────────────────────────────────────────────────
// State
// ──────────────────────────────────────────────────────────

typedef enum {
    ANIM_BOOT           = 0,   // slow blue moon-phase cycle (AP/provisioning)
    ANIM_CONNECTING     = 1,   // 3 blue pulses while connecting
    ANIM_MOON           = 2,   // normal moon render
    ANIM_CONNECT_FAILED = 3,   // 4 quick red pulses before we wipe creds + reboot to AP
    ANIM_PREVIEW        = 4,   // /debug Preview Cycle — RGB+W color-picked moon-phase loop
} anim_mode_t;

static volatile anim_mode_t s_anim_mode = ANIM_BOOT;

static EventGroupHandle_t s_events;
#define EV_GOT_IP                 BIT0
#define EV_CONNECTING_DONE        BIT1
#define EV_FAILED_DONE            BIT2
#define EV_STANDALONE_REQUESTED   BIT3
#define EV_OTA_CHECK_NOW          BIT4   // /debug "Check for Update" button

// ── OTA state (read by /debug/status, written by ota_task) ─
typedef enum {
    OTA_IDLE        = 0,   // between checks
    OTA_CHECKING    = 1,   // fetching image header
    OTA_DOWNLOADING = 2,   // pulling the new image into the spare slot
    OTA_REBOOTING   = 3,   // update verified, restart imminent
} ota_state_t;

static volatile ota_state_t s_ota_state       = OTA_IDLE;
static volatile int         s_ota_progress    = 0;      // 0..100 while downloading
static char                 s_ota_last[96]    = "not yet checked";
static volatile time_t      s_ota_last_check  = 0;

// Mutex serialising led_strip_refresh() against any flash-touching code
// (NVS commit, scan, restart prep). Flash ops disable the CPU cache, which
// can stall the RMT refill ISR and corrupt the SK6812 frame mid-transmit.
static SemaphoreHandle_t led_mutex;

// Strip handle, shared between render task and the (rare) blocking flushes
// in main/handlers.
static led_strip_handle_t s_strip;

// Random session token for /debug cookies. 32 hex chars + null.
static char s_session_token[33];

// Device naming — see NAME_PREFIX comment. `s_mac_suffix` is always set at
// boot (6 hex chars from the STA MAC's last 3 bytes). `s_friendly_name` is
// empty by default, populated from NVS if the user has renamed the lamp via
// /debug. `device_tag()` returns whichever is currently in effect.
static char s_mac_suffix[8];      // e.g. "a94290"
static char s_friendly_name[32];  // "" = fall back to MAC suffix

// True while the lamp is in AP-provisioning / standalone mode (i.e., serving
// its own Wi-Fi network). In that state the mDNS hostname is just "phase"
// so a fresh user can always type `phase.local` after joining the AP —
// uniqueness only matters on shared home networks. Flipped in app_main.
static bool s_is_ap_mode = false;

// ── Live params ────────────────────────────────────────────
static float p_face_gradient = DEFAULT_FACE_GRADIENT;
static float p_brightness    = DEFAULT_BRIGHTNESS;
static float p_floor         = DEFAULT_FLOOR;
static bool  p_glimmer_on    = DEFAULT_GLIMMER_ON;
static float p_glimmer_base  = DEFAULT_GLIMMER_BASE;
static float p_glimmer_edge  = DEFAULT_GLIMMER_EDGE;
static float p_glimmer_speed = DEFAULT_GLIMMER_SPEED;
static float p_curve_q1      = DEFAULT_CURVE_Q1;
static float p_curve_g1      = DEFAULT_CURVE_G1;
static float p_curve_g3      = DEFAULT_CURVE_G3;
static float p_curve_q3      = DEFAULT_CURVE_Q3;
// Hard ceiling on output regardless of p_brightness — applied in every
// output_* function. Defaults USB-safe; raise to ~1.0 on external 5 V supply.
static float p_power_cap     = DEFAULT_POWER_CAP;

// ── Manual override ────────────────────────────────────────
static bool  manual_mode  = false;
static float manual_phase = 0.0f;

// ── Preview cycle (runtime only; not persisted) ────────────
static volatile uint8_t s_preview_r       = 0xFF;
static volatile uint8_t s_preview_g       = 0xFF;
static volatile uint8_t s_preview_b       = 0xFF;
static volatile float   s_preview_rgb_dim = 0.50f;
static volatile float   s_preview_w_dim   = 0.50f;
static volatile float   s_preview_speed   = 1.0f;   // 1× = BOOT_ANIM_CYCLE_SEC per loop

// ──────────────────────────────────────────────────────────
// NVS — render params
// ──────────────────────────────────────────────────────────

static void nvs_save_params(void)
{
    xSemaphoreTake(led_mutex, portMAX_DELAY);
    nvs_handle_t h;
    if (nvs_open("phase", NVS_READWRITE, &h) != ESP_OK) {
        xSemaphoreGive(led_mutex);
        return;
    }
    nvs_set_i32(h, "face_gradient", (int32_t)(p_face_gradient * 1000));
    nvs_set_i32(h, "brightness",    (int32_t)(p_brightness    * 1000));
    nvs_set_i32(h, "floor",         (int32_t)(p_floor         * 1000));
    nvs_set_i32(h, "glimmer_on",    p_glimmer_on ? 1 : 0);
    nvs_set_i32(h, "glimmer_base",  (int32_t)(p_glimmer_base  * 1000));
    nvs_set_i32(h, "glimmer_edge",  (int32_t)(p_glimmer_edge  * 1000));
    nvs_set_i32(h, "glimmer_speed", (int32_t)(p_glimmer_speed * 1000));
    nvs_set_i32(h, "curve_q1",      (int32_t)(p_curve_q1      * 1000));
    nvs_set_i32(h, "curve_g1",      (int32_t)(p_curve_g1      * 1000));
    nvs_set_i32(h, "curve_g3",      (int32_t)(p_curve_g3      * 1000));
    nvs_set_i32(h, "curve_q3",      (int32_t)(p_curve_q3      * 1000));
    nvs_set_i32(h, "power_cap",     (int32_t)(p_power_cap     * 1000));
    nvs_commit(h);
    nvs_close(h);
    xSemaphoreGive(led_mutex);
    ESP_LOGI(TAG, "Saved params. q1=%.3f g1=%.3f g3=%.3f q3=%.3f",
             p_curve_q1, p_curve_g1, p_curve_g3, p_curve_q3);
}

static void nvs_load_params(void)
{
    nvs_handle_t h;
    if (nvs_open("phase", NVS_READONLY, &h) != ESP_OK) return;
    int32_t v;
    if (nvs_get_i32(h, "face_gradient", &v) == ESP_OK) p_face_gradient = v / 1000.0f;
    if (nvs_get_i32(h, "brightness",    &v) == ESP_OK) p_brightness    = v / 1000.0f;
    if (nvs_get_i32(h, "floor",         &v) == ESP_OK) p_floor         = v / 1000.0f;
    if (nvs_get_i32(h, "glimmer_on",    &v) == ESP_OK) p_glimmer_on    = (v != 0);
    if (nvs_get_i32(h, "glimmer_base",  &v) == ESP_OK) p_glimmer_base  = v / 1000.0f;
    if (nvs_get_i32(h, "glimmer_edge",  &v) == ESP_OK) p_glimmer_edge  = v / 1000.0f;
    if (nvs_get_i32(h, "glimmer_speed", &v) == ESP_OK) p_glimmer_speed = v / 1000.0f;
    if (nvs_get_i32(h, "curve_q1",      &v) == ESP_OK) p_curve_q1      = v / 1000.0f;
    if (nvs_get_i32(h, "curve_g1",      &v) == ESP_OK) p_curve_g1      = v / 1000.0f;
    if (nvs_get_i32(h, "curve_g3",      &v) == ESP_OK) p_curve_g3      = v / 1000.0f;
    if (nvs_get_i32(h, "curve_q3",      &v) == ESP_OK) p_curve_q3      = v / 1000.0f;
    if (nvs_get_i32(h, "power_cap",     &v) == ESP_OK) p_power_cap     = v / 1000.0f;
    nvs_close(h);
    ESP_LOGI(TAG, "Loaded params. q1=%.3f g1=%.3f g3=%.3f q3=%.3f",
             p_curve_q1, p_curve_g1, p_curve_g3, p_curve_q3);
}

// ──────────────────────────────────────────────────────────
// NVS — Wi-Fi credentials
// ──────────────────────────────────────────────────────────

static bool load_wifi_creds(char *ssid, size_t ssid_sz, char *pass, size_t pass_sz)
{
    nvs_handle_t h;
    if (nvs_open("phase", NVS_READONLY, &h) != ESP_OK) return false;
    size_t s1 = ssid_sz, s2 = pass_sz;
    esp_err_t e1 = nvs_get_str(h, "wifi_ssid", ssid, &s1);
    esp_err_t e2 = nvs_get_str(h, "wifi_pass", pass, &s2);
    nvs_close(h);
    if (e1 != ESP_OK) { ssid[0] = '\0'; pass[0] = '\0'; return false; }
    if (e2 != ESP_OK) { pass[0] = '\0'; }
    return strlen(ssid) > 0;
}

static void save_wifi_creds(const char *ssid, const char *pass)
{
    xSemaphoreTake(led_mutex, portMAX_DELAY);
    nvs_handle_t h;
    if (nvs_open("phase", NVS_READWRITE, &h) == ESP_OK) {
        nvs_set_str(h, "wifi_ssid", ssid ? ssid : "");
        nvs_set_str(h, "wifi_pass", pass ? pass : "");
        nvs_commit(h);
        nvs_close(h);
    }
    xSemaphoreGive(led_mutex);
}

static void clear_wifi_creds(void)
{
    xSemaphoreTake(led_mutex, portMAX_DELAY);
    nvs_handle_t h;
    if (nvs_open("phase", NVS_READWRITE, &h) == ESP_OK) {
        nvs_erase_key(h, "wifi_ssid");
        nvs_erase_key(h, "wifi_pass");
        nvs_commit(h);
        nvs_close(h);
    }
    xSemaphoreGive(led_mutex);
}

// ──────────────────────────────────────────────────────────
// Device naming (friendly + MAC-based fallback)
// ──────────────────────────────────────────────────────────

// Compute the 6-hex-char suffix from the last 3 bytes of the STA MAC.
// Called once at boot before Wi-Fi is used.
static void compute_mac_suffix(void)
{
    uint8_t mac[6] = {0};
    if (esp_read_mac(mac, ESP_MAC_WIFI_STA) != ESP_OK) {
        strcpy(s_mac_suffix, "000000");
        return;
    }
    snprintf(s_mac_suffix, sizeof(s_mac_suffix),
             "%02x%02x%02x", mac[3], mac[4], mac[5]);
}

static void nvs_load_friendly_name(void)
{
    nvs_handle_t h;
    s_friendly_name[0] = '\0';
    if (nvs_open("phase", NVS_READONLY, &h) != ESP_OK) return;
    size_t sz = sizeof(s_friendly_name);
    if (nvs_get_str(h, "friendly", s_friendly_name, &sz) != ESP_OK) {
        s_friendly_name[0] = '\0';
    }
    nvs_close(h);
}

static void nvs_save_friendly_name(const char *name)
{
    xSemaphoreTake(led_mutex, portMAX_DELAY);
    nvs_handle_t h;
    if (nvs_open("phase", NVS_READWRITE, &h) == ESP_OK) {
        if (name && name[0]) {
            nvs_set_str(h, "friendly", name);
        } else {
            nvs_erase_key(h, "friendly");
        }
        nvs_commit(h);
        nvs_close(h);
    }
    xSemaphoreGive(led_mutex);
}

// The tag that gets suffixed onto NAME_PREFIX for mDNS + AP SSID.
// Friendly name wins if set, else the MAC hex.
static const char *device_tag(void)
{
    return s_friendly_name[0] ? s_friendly_name : s_mac_suffix;
}

// "phase-{tag}" written to `out` (max out_sz including null).
static void device_hostname(char *out, size_t out_sz)
{
    snprintf(out, out_sz, "%s-%s", NAME_PREFIX, device_tag());
}

// Force user input into an mDNS-safe label: lowercase, digits, hyphens.
// Spaces/underscores collapse to '-'. Everything else stripped. Leading /
// trailing hyphens trimmed. Cap at 24 chars (fits inside "phase-…" under
// mDNS's 63-char single-label limit with lots of slack).
static void sanitize_friendly(const char *in, char *out, size_t out_sz)
{
    size_t o = 0;
    size_t cap = (out_sz > 25) ? 25 : out_sz;   // room for null
    for (size_t i = 0; in[i] && o < cap - 1; i++) {
        char c = in[i];
        if (c >= 'A' && c <= 'Z') c = (char)(c - 'A' + 'a');
        if ((c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') || c == '-') {
            out[o++] = c;
        } else if (c == ' ' || c == '_' || c == '.') {
            if (o > 0 && out[o - 1] != '-') out[o++] = '-';
        }
    }
    while (o > 0 && out[o - 1] == '-') o--;
    out[o] = '\0';
}

// ──────────────────────────────────────────────────────────
// Moon phase math
// ──────────────────────────────────────────────────────────

static float calc_moon_phase(void)
{
    time_t now;
    time(&now);
    double known_new_moon = 947182440.0;
    double lunar_cycle    = 29.53058770576 * 24.0 * 3600.0;
    double elapsed = difftime(now, (time_t)known_new_moon);
    double phase   = fmod(elapsed, lunar_cycle) / lunar_cycle;
    if (phase < 0) phase += 1.0;
    return (float)phase;
}

static float phase_to_illumination(float phase)
{
    return 0.5f - 0.5f * cosf(phase * 2.0f * (float)M_PI);
}

static const char* phase_name(float phase)
{
    float ill = phase_to_illumination(phase);
    if (ill < 0.02f)                   return "New Moon";
    if (phase < 0.5f && ill < 0.48f)  return "Waxing Crescent";
    if (phase < 0.5f && ill < 0.52f)  return "First Quarter";
    if (phase < 0.5f && ill < 0.98f)  return "Waxing Gibbous";
    if (ill >= 0.98f)                  return "Full Moon";
    if (phase >= 0.5f && ill > 0.52f) return "Waning Gibbous";
    if (phase >= 0.5f && ill > 0.48f) return "Last Quarter";
    return "Waning Crescent";
}

// Custom illumination curve with 4 control points.
// New(0) → Q1 → Gibbous1 → Full(1) → Gibbous3 → Q3 → New(0)
static float apply_curve(float phase)
{
    float t, smooth_t, y0, y1;

    if (phase <= 0.25f) {
        t = phase / 0.25f;
        smooth_t = t * t * (3.0f - 2.0f * t);
        y0 = 0.0f; y1 = p_curve_q1;
    } else if (phase <= 0.375f) {
        t = (phase - 0.25f) / 0.125f;
        smooth_t = t * t * (3.0f - 2.0f * t);
        y0 = p_curve_q1; y1 = p_curve_g1;
    } else if (phase <= 0.5f) {
        t = (phase - 0.375f) / 0.125f;
        smooth_t = t * t * (3.0f - 2.0f * t);
        y0 = p_curve_g1; y1 = 1.0f;
    } else if (phase <= 0.625f) {
        t = (phase - 0.5f) / 0.125f;
        smooth_t = t * t * (3.0f - 2.0f * t);
        y0 = 1.0f; y1 = p_curve_g3;
    } else if (phase <= 0.75f) {
        t = (phase - 0.625f) / 0.125f;
        smooth_t = t * t * (3.0f - 2.0f * t);
        y0 = p_curve_g3; y1 = p_curve_q3;
    } else {
        t = (phase - 0.75f) / 0.25f;
        smooth_t = t * t * (3.0f - 2.0f * t);
        y0 = p_curve_q3; y1 = 0.0f;
    }

    float result = y0 + smooth_t * (y1 - y0);
    if (result < 0.0f) result = 0.0f;
    if (result > 1.0f) result = 1.0f;
    return result * 360.0f;
}

// ──────────────────────────────────────────────────────────
// LED rendering
// ──────────────────────────────────────────────────────────

// LED 0 sits at 12 o'clock; indices advance clockwise. Angle 0° = 12 o'clock.
// (lit_center swings 90° → 270° for waxing → waning, matching N-hemisphere
//  moon orientation when angle 90° = 3 o'clock side.)
//
// Pre-computed in init_geometry() so the render loop doesn't pay for 138
// fmodf calls every frame. C3 has no FPU — every avoided float op matters.
static float led_angle_lut[LED_COUNT];

static float   glimmer_offset[LED_COUNT];
static float   glimmer_rate[LED_COUNT];
static float   frame_b[LED_COUNT];       // brightness per pixel, 0..1
static uint8_t last_out[LED_COUNT][4];   // last R,G,B,W actually sent
static bool    last_out_valid = false;

static void init_geometry(void)
{
    for (int i = 0; i < LED_COUNT; i++) {
        led_angle_lut[i] = (float)i * (360.0f / (float)LED_COUNT);
    }
}

static void init_glimmer(void)
{
    for (int i = 0; i < LED_COUNT; i++) {
        glimmer_offset[i] = ((float)rand() / RAND_MAX) * 2.0f * (float)M_PI;
        glimmer_rate[i]   = 0.7f + ((float)rand() / RAND_MAX) * 0.6f;
    }
}

static float glimmer_value(int led, float time_f, float edge_proximity)
{
    float wave     = sinf(time_f * glimmer_rate[led] + glimmer_offset[led]);
    float wave2    = sinf(time_f * glimmer_rate[led] * 0.37f + glimmer_offset[led] * 1.3f);
    float combined = wave * 0.7f + wave2 * 0.3f;
    float amount   = p_glimmer_base + edge_proximity * p_glimmer_edge;
    return 1.0f - amount * (0.5f + 0.5f * combined);
}

// Fill frame_b[] with brightness 0..1 for the given moon phase.
// (Same math as legacy render_moon — separated from the output stage so the
//  boot animation can recolor the same shape.)
//
// breath_t is a real-time seconds accumulator independent of glimmer speed,
// used for the always-on subtle global breathing modulation that gives the
// moon "life" near full without revealing the rotational center.
static void compute_moon_frame(float phase, float time_f, float breath_t)
{
    float lit_arc      = apply_curve(phase);
    float lit_fraction = lit_arc * (1.0f / 360.0f);   // 0 (new) … 1 (full)

    float lit_center;
    float flip_window = 0.04f;
    if (phase < 0.5f - flip_window) {
        lit_center = 90.0f;
    } else if (phase > 0.5f + flip_window) {
        lit_center = 270.0f;
    } else {
        float t = (phase - (0.5f - flip_window)) / (2.0f * flip_window);
        float smooth_t = t * t * (3.0f - 2.0f * t);
        lit_center = 90.0f + smooth_t * 180.0f;
    }

    float gradient_deg = GRADIENT_WIDTH * 360.0f;
    float max_gradient = lit_arc * 0.25f;
    if (gradient_deg > max_gradient) gradient_deg = max_gradient;

    // Taper the face-gradient strength as the ring fills up. At full moon
    // (lit_fraction = 1) the gradient vanishes entirely, so the lit_center
    // position becomes invisible and the lit_center flip can't be seen as a
    // 180° rotation. Smoothstep makes the taper gentle, not sudden.
    float ft = lit_fraction;
    if (ft > 1.0f) ft = 1.0f;
    float fade = ft * ft * (3.0f - 2.0f * ft);     // smoothstep(0,1,lit_fraction)
    float face_grad_eff = p_face_gradient * (1.0f - fade);

    // Edge fade: hide the lit_center 180° rotation by filling the
    // gradient zone toward full brightness. Previously tied to
    // lit_fraction (activated at ~0.55, fully faded by ~0.85), which
    // made the moon *look* fully lit for the ~3-day range where
    // lit_fraction > 0.85 — a much longer "full moon" than reality.
    // Now tied to the SAME flip window as lit_center (phase ±0.04
    // around full). Outside the flip window: edge_fade = 1.0, normal
    // soft terminator visible, gibbous looks gibbous. Inside: fades
    // to 0 at exact full moon, hiding the rotation.
    float phase_from_half = fabsf(phase - 0.5f);
    float edge_fade = 1.0f;
    if (phase_from_half < flip_window) {
        float et = 1.0f - (phase_from_half / flip_window);
        float es = et * et * (3.0f - 2.0f * et);
        edge_fade = 1.0f - es;
    }

    // Global breath modulation. Amplitude scales with lit_fraction so it's
    // imperceptible at crescent phases but provides quiet life at full.
    // Period ~6 s.
    float breath_amp = 0.06f * lit_fraction;
    float breath_mod = 1.0f - breath_amp +
                       breath_amp * (0.5f + 0.5f * sinf(breath_t * (2.0f * (float)M_PI / 6.0f)));

    for (int i = 0; i < LED_COUNT; i++) {
        float angle = led_angle_lut[i];
        float rel   = angle - lit_center;
        // Branchless wrap — fabsf is cheap, while-loops can iterate twice.
        if (rel >  180.0f) rel -= 360.0f;
        if (rel < -180.0f) rel += 360.0f;

        float half_arc       = lit_arc * 0.5f;
        float dist           = fabsf(rel) - half_arc;
        float brightness     = 0.0f;
        float edge_proximity = 0.0f;

        if (dist < -gradient_deg) {
            float depth = 0.0f;
            if (half_arc > 0.0f) depth = (-dist) / half_arc;
            if (depth > 1.0f) depth = 1.0f;
            brightness = 1.0f - depth * face_grad_eff;
        } else if (dist > gradient_deg) {
            brightness = 0.0f;
        } else {
            float t      = dist / gradient_deg;
            float soft_b = 0.5f - 0.5f * sinf(t * ((float)M_PI / 2.0f));
            // Blend the soft terminator toward fully-lit as edge_fade → 0,
            // so the rotation that would otherwise appear during the
            // lit_center flip becomes invisible.
            brightness     = edge_fade * soft_b + (1.0f - edge_fade);
            edge_proximity = edge_fade * (1.0f - fabsf(t));
        }

        if (p_glimmer_on && brightness > 0.01f)
            brightness *= glimmer_value(i, time_f, edge_proximity);

        brightness *= p_brightness * breath_mod;
        if (brightness < p_floor) brightness = 0.0f;

        if (brightness < 0.0f) brightness = 0.0f;
        if (brightness > 1.0f) brightness = 1.0f;
        frame_b[i] = brightness;
    }
}

// Dedicated boot animation. Same moon-shape rendering as compute_moon_frame
// but completely independent of the user's debug parameters — no apply_curve
// (so no plateaus from q1/g1/g3/q3), no p_brightness, no p_floor, no glimmer.
// Just a smooth sin² illumination sweep that follows the moon-phase shape.
static void compute_boot_frame(float boot_phase)
{
    // sin(phase·π) gives 0 → 1 → 0 over phase 0 → 1. NOT squared: sin²
    // has zero velocity at both endpoints so the growing crescent hangs
    // on the first/last pixel for several seconds. Plain sin has finite
    // velocity there, so pixels roll in and out at a steady cadence.
    float s        = sinf(boot_phase * (float)M_PI);
    float ill      = s;
    float lit_arc  = ill * 360.0f;

    float lit_center;
    float flip_window = 0.04f;
    if (boot_phase < 0.5f - flip_window) {
        lit_center = 90.0f;
    } else if (boot_phase > 0.5f + flip_window) {
        lit_center = 270.0f;
    } else {
        float t = (boot_phase - (0.5f - flip_window)) / (2.0f * flip_window);
        float smooth_t = t * t * (3.0f - 2.0f * t);
        lit_center = 90.0f + smooth_t * 180.0f;
    }

    // Fixed gradient width & face fade — don't pull from the debug params.
    const float BOOT_GRADIENT_WIDTH  = 0.25f;
    const float BOOT_FACE_GRADIENT   = 0.20f;

    float gradient_deg = BOOT_GRADIENT_WIDTH * 360.0f;
    float max_gradient = lit_arc * 0.25f;
    if (gradient_deg > max_gradient) gradient_deg = max_gradient;

    // Taper face gradient at high illumination (same logic as moon mode) so
    // there's no visible rotation at the full-moon peak of the sweep.
    float fade = ill * ill * (3.0f - 2.0f * ill);
    float face_grad_eff = BOOT_FACE_GRADIENT * (1.0f - fade);

    // Edge fade — hide the lit_center 180° rotation by filling the
    // gradient zone toward full brightness during the flip. Tied to the
    // SAME flip window as lit_center (phase ±0.04) rather than ill,
    // so the "essentially full" plateau is short — mirrors the same
    // fix applied to compute_moon_frame.
    float boot_phase_from_half = fabsf(boot_phase - 0.5f);
    float edge_fade = 1.0f;
    if (boot_phase_from_half < flip_window) {
        float et = 1.0f - (boot_phase_from_half / flip_window);
        float es = et * et * (3.0f - 2.0f * et);
        edge_fade = 1.0f - es;
    }

    for (int i = 0; i < LED_COUNT; i++) {
        float angle = led_angle_lut[i];
        float rel   = angle - lit_center;
        if (rel >  180.0f) rel -= 360.0f;
        if (rel < -180.0f) rel += 360.0f;

        float half_arc   = lit_arc * 0.5f;
        float dist       = fabsf(rel) - half_arc;
        float brightness = 0.0f;

        if (dist < -gradient_deg) {
            float depth = 0.0f;
            if (half_arc > 0.0f) depth = (-dist) / half_arc;
            if (depth > 1.0f) depth = 1.0f;
            brightness = 1.0f - depth * face_grad_eff;
        } else if (dist > gradient_deg) {
            brightness = 0.0f;
        } else {
            float t      = dist / gradient_deg;
            float soft_b = 0.5f - 0.5f * sinf(t * ((float)M_PI / 2.0f));
            brightness   = edge_fade * soft_b + (1.0f - edge_fade);
        }

        // No BOOT_ANIM_DIM here — caller scales (ANIM_BOOT applies it; the
        // preview animation uses its own RGB/W brightness sliders).
        if (brightness < 0.0f) brightness = 0.0f;
        if (brightness > 1.0f) brightness = 1.0f;
        frame_b[i] = brightness;
    }
}

// Drive the W channel from frame_b[] (R=G=B=0).
static void output_white(void)
{
    float cap = p_power_cap;
    if (cap < 0.0f) cap = 0.0f;
    if (cap > 1.0f) cap = 1.0f;
    uint8_t out[LED_COUNT][4];
    for (int i = 0; i < LED_COUNT; i++) {
        out[i][0] = 0;
        out[i][1] = 0;
        out[i][2] = 0;
        out[i][3] = (uint8_t)(frame_b[i] * cap * 255.0f);
    }
    bool changed = !last_out_valid ||
                   memcmp(out, last_out, sizeof(out)) != 0;
    if (!changed) return;

    xSemaphoreTake(led_mutex, portMAX_DELAY);
    for (int i = 0; i < LED_COUNT; i++) {
        led_strip_set_pixel_rgbw(s_strip, i,
            out[i][0], out[i][1], out[i][2], out[i][3]);
    }
    led_strip_refresh(s_strip);
    xSemaphoreGive(led_mutex);
    memcpy(last_out, out, sizeof(out));
    last_out_valid = true;
}

// Drive all four channels for the preview cycle. R/G/B come from the color
// picker scaled by the RGB-brightness slider; W is driven independently by
// its own slider so you can mix the picked color with warm white at any
// ratio. p_power_cap is still applied as the hard current ceiling.
static void output_preview(void)
{
    float cap = p_power_cap;
    if (cap < 0.0f) cap = 0.0f;
    if (cap > 1.0f) cap = 1.0f;

    float rgb_dim = s_preview_rgb_dim;
    float w_dim   = s_preview_w_dim;
    float pr      = (float)s_preview_r * (1.0f / 255.0f);
    float pg      = (float)s_preview_g * (1.0f / 255.0f);
    float pb      = (float)s_preview_b * (1.0f / 255.0f);

    uint8_t out[LED_COUNT][4];
    for (int i = 0; i < LED_COUNT; i++) {
        float b = frame_b[i] * cap;
        out[i][0] = (uint8_t)(b * rgb_dim * pr * 255.0f);
        out[i][1] = (uint8_t)(b * rgb_dim * pg * 255.0f);
        out[i][2] = (uint8_t)(b * rgb_dim * pb * 255.0f);
        out[i][3] = (uint8_t)(b * w_dim   * 255.0f);
    }
    bool changed = !last_out_valid ||
                   memcmp(out, last_out, sizeof(out)) != 0;
    if (!changed) return;

    xSemaphoreTake(led_mutex, portMAX_DELAY);
    for (int i = 0; i < LED_COUNT; i++) {
        led_strip_set_pixel_rgbw(s_strip, i,
            out[i][0], out[i][1], out[i][2], out[i][3]);
    }
    led_strip_refresh(s_strip);
    xSemaphoreGive(led_mutex);
    memcpy(last_out, out, sizeof(out));
    last_out_valid = true;
}

// Drive only the R channel from frame_b[] (G=B=W=0). Used by
// ANIM_CONNECT_FAILED to flash red before falling back to AP mode.
static void output_red(void)
{
    float cap = p_power_cap;
    if (cap < 0.0f) cap = 0.0f;
    if (cap > 1.0f) cap = 1.0f;
    uint8_t out[LED_COUNT][4];
    for (int i = 0; i < LED_COUNT; i++) {
        out[i][0] = (uint8_t)(frame_b[i] * cap * 255.0f);
        out[i][1] = 0;
        out[i][2] = 0;
        out[i][3] = 0;
    }
    bool changed = !last_out_valid ||
                   memcmp(out, last_out, sizeof(out)) != 0;
    if (!changed) return;

    xSemaphoreTake(led_mutex, portMAX_DELAY);
    for (int i = 0; i < LED_COUNT; i++) {
        led_strip_set_pixel_rgbw(s_strip, i,
            out[i][0], out[i][1], out[i][2], out[i][3]);
    }
    led_strip_refresh(s_strip);
    xSemaphoreGive(led_mutex);
    memcpy(last_out, out, sizeof(out));
    last_out_valid = true;
}

// Drive only the B channel from frame_b[] (R=G=W=0). Used for boot AP +
// "connecting" animations.
static void output_blue(void)
{
    float cap = p_power_cap;
    if (cap < 0.0f) cap = 0.0f;
    if (cap > 1.0f) cap = 1.0f;
    uint8_t out[LED_COUNT][4];
    for (int i = 0; i < LED_COUNT; i++) {
        out[i][0] = 0;
        out[i][1] = 0;
        out[i][2] = (uint8_t)(frame_b[i] * cap * 255.0f);
        out[i][3] = 0;
    }
    bool changed = !last_out_valid ||
                   memcmp(out, last_out, sizeof(out)) != 0;
    if (!changed) return;

    xSemaphoreTake(led_mutex, portMAX_DELAY);
    for (int i = 0; i < LED_COUNT; i++) {
        led_strip_set_pixel_rgbw(s_strip, i,
            out[i][0], out[i][1], out[i][2], out[i][3]);
    }
    led_strip_refresh(s_strip);
    xSemaphoreGive(led_mutex);
    memcpy(last_out, out, sizeof(out));
    last_out_valid = true;
}

// ──────────────────────────────────────────────────────────
// Render task
// ──────────────────────────────────────────────────────────

static void render_task(void *arg)
{
    (void)arg;
    const float frame_sec      = (float)RENDER_PERIOD_MS / 1000.0f;
    const float boot_dphase    = frame_sec / BOOT_ANIM_CYCLE_SEC;        // phase units per frame
    const float pulse_dt       = frame_sec * CONNECTING_PULSE_HZ;        // 0..1 per pulse cycle
    const float exit_fade_rate = frame_sec / MODE_EXIT_FADE_SEC;
    float time_f      = 0.0f;
    float breath_t    = 0.0f;   // real-time seconds — independent of glimmer speed
    float boot_phase  = 0.0f;
    float pulse_t     = 0.0f;
    int   pulse_count = 0;
    float moon_fade_t = 0.0f;   // 0..1 ramp when entering ANIM_MOON
    float exit_fade   = 0.0f;   // 0..1, ramps up when active_mode ≠ requested
    anim_mode_t active_mode = (anim_mode_t)-1;

    while (1) {
        anim_mode_t requested = s_anim_mode;

        if (active_mode == (anim_mode_t)-1) {
            // First tick — adopt whatever's requested without fading anything.
            active_mode = requested;
        }

        // Universal mode transition: when a new mode is requested, fade the
        // active mode out over MODE_EXIT_FADE_SEC, then swap. This makes
        // BOOT → CONNECTING, BOOT → MOON (standalone), MOON ↔ PREVIEW, etc.
        // all read as one smooth gesture instead of a hard cut.
        if (requested != active_mode) {
            exit_fade += exit_fade_rate;
            if (exit_fade >= 1.0f) {
                active_mode    = requested;
                exit_fade      = 0.0f;
                last_out_valid = false;
                pulse_t        = 0.0f;
                pulse_count    = 0;
                moon_fade_t    = 0.0f;
            }
        } else if (exit_fade > 0.0f) {
            // Edge case: requested flipped back to active mid-fade — heal it.
            exit_fade = 0.0f;
        }

        float exit_mul = 1.0f - exit_fade;

        switch (active_mode) {
        case ANIM_BOOT: {
            // Dedicated boot renderer — smooth sin² sweep, no debug params.
            compute_boot_frame(boot_phase);
            for (int i = 0; i < LED_COUNT; i++) frame_b[i] *= BOOT_ANIM_DIM * exit_mul;
            output_blue();
            boot_phase = fmodf(boot_phase + boot_dphase, 1.0f);
            break;
        }
        case ANIM_PREVIEW: {
            // Same smooth sweep as boot, but full-range frame_b (caller
            // controls RGB and W brightness via output_preview).
            compute_boot_frame(boot_phase);
            for (int i = 0; i < LED_COUNT; i++) frame_b[i] *= exit_mul;
            output_preview();
            // Speed multiplier lets the user drag from 0.25× (long, calm
            // cycle) to 10× (fast color demo).
            boot_phase = fmodf(boot_phase + boot_dphase * s_preview_speed, 1.0f);
            break;
        }
        case ANIM_CONNECTING: {
            if (pulse_count >= CONNECTING_PULSE_COUNT) {
                // Pulses done — hold the strip dark while app_main finishes
                // mDNS + SNTP setup. ANIM_MOON's own fade-in then ramps the
                // moon up smoothly from black.
                for (int i = 0; i < LED_COUNT; i++) frame_b[i] = 0.0f;
                output_blue();
                break;
            }
            float bright = CONNECTING_PULSE_DIM *
                           (0.5f - 0.5f * cosf(pulse_t * 2.0f * (float)M_PI));
            for (int i = 0; i < LED_COUNT; i++) frame_b[i] = bright * exit_mul;
            output_blue();
            pulse_t += pulse_dt;
            if (pulse_t >= 1.0f) {
                pulse_t = 0.0f;
                pulse_count++;
                if (pulse_count >= CONNECTING_PULSE_COUNT) {
                    xEventGroupSetBits(s_events, EV_CONNECTING_DONE);
                }
            }
            break;
        }
        case ANIM_CONNECT_FAILED: {
            // Quick red blips so the user knows the connect timed out before
            // we wipe creds and reboot into AP mode.
            float bright = FAILED_PULSE_DIM *
                           (0.5f - 0.5f * cosf(pulse_t * 2.0f * (float)M_PI));
            for (int i = 0; i < LED_COUNT; i++) frame_b[i] = bright * exit_mul;
            output_red();
            pulse_t += frame_sec * FAILED_PULSE_HZ;
            if (pulse_t >= 1.0f) {
                pulse_t = 0.0f;
                pulse_count++;
                if (pulse_count >= FAILED_PULSE_COUNT) {
                    xEventGroupSetBits(s_events, EV_FAILED_DONE);
                }
            }
            break;
        }
        case ANIM_MOON:
        default: {
            // 2 s smoothstep fade-in any time we enter MOON mode (cold
            // start after pulses, return from preview, or after standalone).
            moon_fade_t += frame_sec / MOON_FADE_IN_SEC;
            if (moon_fade_t > 1.0f) moon_fade_t = 1.0f;
            float mf = moon_fade_t * moon_fade_t * (3.0f - 2.0f * moon_fade_t);

            float phase = manual_mode ? manual_phase : calc_moon_phase();
            compute_moon_frame(phase, time_f, breath_t);
            for (int i = 0; i < LED_COUNT; i++) frame_b[i] *= mf * exit_mul;
            output_white();
            break;
        }
        }

        // Glimmer time base advances in real seconds × p_glimmer_speed,
        // so animation speed is independent of RENDER_PERIOD_MS.
        time_f   += p_glimmer_speed * frame_sec;
        breath_t += frame_sec;
        vTaskDelay(pdMS_TO_TICKS(RENDER_PERIOD_MS));
    }
}

// ──────────────────────────────────────────────────────────
// Reset button — hold 3 s to forget Wi-Fi
// ──────────────────────────────────────────────────────────

static void button_task(void *arg)
{
    (void)arg;
    gpio_config_t io = {
        .pin_bit_mask = 1ULL << BUTTON_GPIO,
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&io);
    ESP_LOGI(TAG, "Reset button task on GPIO %d (active-low, pull-up). Hold %d ms to erase Wi-Fi creds.",
             BUTTON_GPIO, BUTTON_HOLD_MS);

    int held_ms = 0;
    while (1) {
        if (gpio_get_level(BUTTON_GPIO) == 0) {  // pressed
            held_ms += 50;
            if (held_ms == 1000 || held_ms == 2000) {
                ESP_LOGI(TAG, "Reset button held %d ms…", held_ms);
            }
            if (held_ms >= BUTTON_HOLD_MS) {
                ESP_LOGW(TAG, "Reset button held %d ms — clearing Wi-Fi creds and rebooting",
                         BUTTON_HOLD_MS);
                clear_wifi_creds();
                vTaskDelay(pdMS_TO_TICKS(100));
                esp_restart();
            }
        } else {
            held_ms = 0;
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

// ──────────────────────────────────────────────────────────
// mDNS
// ──────────────────────────────────────────────────────────

// Fill `out` with the mDNS hostname appropriate for the current mode:
//   - AP / standalone: just "phase" (isolated network, uniqueness moot,
//     user should always be able to hit `phase.local`)
//   - STA:             "phase-<friendly>" or "phase-<mac>" (unique per
//     device so multiple lamps can coexist on a shared network)
static void active_mdns_hostname(char *out, size_t out_sz)
{
    if (s_is_ap_mode) {
        strncpy(out, NAME_PREFIX, out_sz - 1);
        out[out_sz - 1] = '\0';
    } else {
        device_hostname(out, out_sz);
    }
}

static void mdns_setup(void)
{
    esp_err_t err = mdns_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "mdns_init failed: %s", esp_err_to_name(err));
        return;
    }
    char hostname[48];
    active_mdns_hostname(hostname, sizeof(hostname));
    mdns_hostname_set(hostname);
    mdns_instance_name_set(MDNS_INSTANCE);
    mdns_service_add(NULL, "_http", "_tcp", 80, NULL, 0);
    ESP_LOGI(TAG, "mDNS up — http://%s.local/", hostname);
}

// Re-apply the mDNS hostname without a full mDNS restart. Called after the
// user renames the lamp via /debug/name so the new URL starts working
// immediately, without waiting for the next reboot. In AP mode this is a
// no-op — hostname stays "phase" until the next reboot into STA mode.
static void mdns_refresh_hostname(void)
{
    char hostname[48];
    active_mdns_hostname(hostname, sizeof(hostname));
    if (mdns_hostname_set(hostname) == ESP_OK) {
        ESP_LOGI(TAG, "mDNS hostname now http://%s.local/", hostname);
    }
}

// ──────────────────────────────────────────────────────────
// Wi-Fi event handling
// ──────────────────────────────────────────────────────────

static void wifi_event_handler(void *arg, esp_event_base_t base,
                                int32_t id, void *data)
{
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        esp_wifi_connect();
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        xEventGroupSetBits(s_events, EV_GOT_IP);
        esp_netif_ip_info_t ip_info;
        esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
        if (netif && esp_netif_get_ip_info(netif, &ip_info) == ESP_OK)
            ESP_LOGI(TAG, "Got IP " IPSTR, IP2STR(&ip_info.ip));
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_AP_STACONNECTED) {
        ESP_LOGI(TAG, "Client joined the phase AP");
    }
}

static void wifi_init_sta_with_creds(const char *ssid, const char *pass)
{
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);
    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event_handler, NULL);
    esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_event_handler, NULL);

    wifi_config_t wifi_cfg = {0};
    strncpy((char *)wifi_cfg.sta.ssid, ssid, sizeof(wifi_cfg.sta.ssid) - 1);
    strncpy((char *)wifi_cfg.sta.password, pass, sizeof(wifi_cfg.sta.password) - 1);

    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(WIFI_IF_STA, &wifi_cfg);
    esp_wifi_start();
    // Critical for SK6812-on-C3 — see prototype-02.1 commit a13ae3e.
    esp_wifi_set_ps(WIFI_PS_NONE);
    ESP_LOGI(TAG, "STA mode — connecting to \"%s\"…", ssid);
}

// AP + STA so we can scan while the AP is up for the setup page.
static void wifi_init_ap(void)
{
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_ap();
    esp_netif_create_default_wifi_sta();   // scan-only, never connects

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);
    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event_handler, NULL);

    char ap_ssid[33] = {0};
    device_hostname(ap_ssid, sizeof(ap_ssid));   // "phase-<tag>"
    wifi_config_t ap_cfg = {
        .ap = {
            .channel         = 6,
            .authmode        = WIFI_AUTH_OPEN,
            .max_connection  = 4,
            // Default beacon interval is 100 ms — every beacon broadcast
            // is an ISR that can preempt the render task. 400 ms is still
            // fine for client discovery and frees a lot of slack.
            .beacon_interval = 400,
        },
    };
    strncpy((char *)ap_cfg.ap.ssid, ap_ssid, sizeof(ap_cfg.ap.ssid) - 1);
    ap_cfg.ap.ssid_len = (uint8_t)strlen(ap_ssid);

    esp_wifi_set_mode(WIFI_MODE_APSTA);
    esp_wifi_set_config(WIFI_IF_AP, &ap_cfg);
    esp_wifi_start();
    esp_wifi_set_ps(WIFI_PS_NONE);
    ESP_LOGI(TAG, "AP mode — SSID \"%s\" (open) — http://%s.local/", ap_ssid, ap_ssid);
}

// ──────────────────────────────────────────────────────────
// Time sync
// ──────────────────────────────────────────────────────────

static void time_sync(void)
{
    esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, "pool.ntp.org");
    esp_sntp_init();
    ESP_LOGI(TAG, "Waiting for SNTP…");
    time_t now = 0;
    struct tm timeinfo = {0};
    while (timeinfo.tm_year < (2024 - 1900)) {
        vTaskDelay(pdMS_TO_TICKS(500));
        time(&now);
        localtime_r(&now, &timeinfo);
    }
    ESP_LOGI(TAG, "Time synced: %s", asctime(&timeinfo));
}

// ──────────────────────────────────────────────────────────
// OTA — automatic over-the-air updates
// ──────────────────────────────────────────────────────────
//
// Model: "the fleet follows the latest GitHub release." ota_task wakes
// on boot (+30 s) and every 6 h, downloads the image header from the
// release URL, and compares its app-descriptor version against the
// running one. Different version → full download into the spare OTA
// slot → reboot. No user interaction anywhere.
//
// Safety:
//  - Rollback: CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE marks each fresh
//    OTA image PENDING_VERIFY; we only mark it valid once the lamp
//    reaches a healthy steady state (webserver up). If the new image
//    crashes before that, the bootloader falls back to the old slot.
//  - LED glitches: flash writes disable the CPU cache, which can stall
//    the RMT refill ISR mid-frame (see glitch-fix notes). Every
//    esp_https_ota_perform() chunk therefore holds led_mutex, same as
//    NVS commits. The render loop just pauses a beat between chunks.

// Read the effective update URL: NVS override ("ota_url") or default.
static void ota_get_url(char *out, size_t out_sz)
{
    nvs_handle_t h;
    if (nvs_open("phase", NVS_READONLY, &h) == ESP_OK) {
        size_t len = out_sz;
        if (nvs_get_str(h, "ota_url", out, &len) == ESP_OK && len > 1) {
            nvs_close(h);
            return;
        }
        nvs_close(h);
    }
    strlcpy(out, OTA_DEFAULT_URL, out_sz);
}

static void ota_set_last(const char *msg)
{
    strlcpy(s_ota_last, msg, sizeof(s_ota_last));
    ESP_LOGI(TAG, "OTA: %s", s_ota_last);
}

// If this boot is the first on a fresh OTA image, declare it healthy so
// the bootloader stops considering a rollback. Called once the device
// has reached a functional steady state.
static void ota_mark_boot_valid(void)
{
    const esp_partition_t *running = esp_ota_get_running_partition();
    esp_ota_img_states_t st;
    if (esp_ota_get_state_partition(running, &st) == ESP_OK &&
        st == ESP_OTA_IMG_PENDING_VERIFY) {
        esp_ota_mark_app_valid_cancel_rollback();
        ESP_LOGI(TAG, "OTA: new image confirmed healthy — rollback cancelled.");
    }
}

// One full check-and-maybe-update pass. Returns only if no update was
// applied (success ends in esp_restart()).
static void ota_check_and_update(void)
{
    char url[256];
    ota_get_url(url, sizeof(url));

    s_ota_state = OTA_CHECKING;
    s_ota_progress = 0;
    time((time_t *)&s_ota_last_check);

    esp_http_client_config_t http_cfg = {
        .url = url,
        .crt_bundle_attach = esp_crt_bundle_attach,
        .timeout_ms = OTA_HTTP_TIMEOUT_MS,
        .keep_alive_enable = true,
        // GitHub redirects the release asset to a long signed URL
        // (~1.5 KB) — both buffers must hold it or the redirect fails.
        .buffer_size = 4096,
        .buffer_size_tx = 2560,
    };
    esp_https_ota_config_t ota_cfg = {
        .http_config = &http_cfg,
    };

    esp_https_ota_handle_t handle = NULL;
    esp_err_t err = esp_https_ota_begin(&ota_cfg, &handle);
    if (err != ESP_OK) {
        ota_set_last("check failed: can't reach update server");
        s_ota_state = OTA_IDLE;
        return;
    }

    // Peek at the new image's app descriptor before committing to a
    // full download.
    esp_app_desc_t new_app;
    err = esp_https_ota_get_img_desc(handle, &new_app);
    if (err != ESP_OK) {
        ota_set_last("check failed: bad image header");
        esp_https_ota_abort(handle);
        s_ota_state = OTA_IDLE;
        return;
    }

    const esp_app_desc_t *cur_app = esp_app_get_description();
    if (strncmp(new_app.version, cur_app->version, sizeof(new_app.version)) == 0) {
        char msg[96];
        snprintf(msg, sizeof(msg), "up to date (%s)", cur_app->version);
        ota_set_last(msg);
        esp_https_ota_abort(handle);
        s_ota_state = OTA_IDLE;
        return;
    }

    ESP_LOGI(TAG, "OTA: new firmware \"%s\" available (running \"%s\") — updating.",
             new_app.version, cur_app->version);
    s_ota_state = OTA_DOWNLOADING;
    int total = esp_https_ota_get_image_size(handle);

    while (true) {
        // Hold led_mutex per chunk: TLS recv + flash write happen inside
        // perform(), and the flash write must not race led_strip_refresh().
        xSemaphoreTake(led_mutex, portMAX_DELAY);
        err = esp_https_ota_perform(handle);
        xSemaphoreGive(led_mutex);
        if (err != ESP_ERR_HTTPS_OTA_IN_PROGRESS) break;
        if (total > 0) {
            s_ota_progress = 100 * esp_https_ota_get_image_len_read(handle) / total;
        }
        // Give render/httpd air between chunks.
        vTaskDelay(pdMS_TO_TICKS(2));
    }

    if (err != ESP_OK || !esp_https_ota_is_complete_data_received(handle)) {
        ota_set_last("download failed — will retry next cycle");
        esp_https_ota_abort(handle);
        s_ota_state = OTA_IDLE;
        return;
    }

    err = esp_https_ota_finish(handle);   // validates image + sets boot slot
    if (err != ESP_OK) {
        ota_set_last(err == ESP_ERR_OTA_VALIDATE_FAILED
                         ? "image validation failed"
                         : "update finalize failed");
        s_ota_state = OTA_IDLE;
        return;
    }

    char msg[96];
    snprintf(msg, sizeof(msg), "updated to %s — rebooting", new_app.version);
    ota_set_last(msg);
    s_ota_state = OTA_REBOOTING;
    s_ota_progress = 100;
    vTaskDelay(pdMS_TO_TICKS(1500));      // let /debug/status show the result
    esp_restart();
}

static void ota_task(void *arg)
{
    // First check shortly after boot, then on a slow cycle; the /debug
    // "Check for Update" button can force a pass at any time.
    TickType_t wait = pdMS_TO_TICKS(OTA_FIRST_CHECK_DELAY_MS);
    while (true) {
        xEventGroupWaitBits(s_events, EV_OTA_CHECK_NOW, true, true, wait);
        ota_check_and_update();
        wait = pdMS_TO_TICKS(OTA_CHECK_INTERVAL_MS);
    }
}

// ──────────────────────────────────────────────────────────
// HTTP — session auth helpers
// ──────────────────────────────────────────────────────────

static void make_session_token(void)
{
    static const char hex[] = "0123456789abcdef";
    for (int i = 0; i < 32; i++) {
        s_session_token[i] = hex[esp_random() & 0xF];
    }
    s_session_token[32] = '\0';
}

static bool req_is_authed(httpd_req_t *req)
{
    size_t need = httpd_req_get_hdr_value_len(req, "Cookie") + 1;
    if (need <= 1) return false;
    char *cookie = malloc(need);
    if (!cookie) return false;
    if (httpd_req_get_hdr_value_str(req, "Cookie", cookie, need) != ESP_OK) {
        free(cookie);
        return false;
    }
    char *p = strstr(cookie, "phase_sess=");
    bool ok = false;
    if (p) {
        p += strlen("phase_sess=");
        ok = (strncmp(p, s_session_token, 32) == 0);
    }
    free(cookie);
    return ok;
}

// Send redirect, optionally with Set-Cookie.
static esp_err_t send_redirect(httpd_req_t *req, const char *location, const char *cookie_or_null)
{
    httpd_resp_set_status(req, "302 Found");
    httpd_resp_set_hdr(req, "Location", location);
    if (cookie_or_null) httpd_resp_set_hdr(req, "Set-Cookie", cookie_or_null);
    httpd_resp_send(req, NULL, 0);
    return ESP_OK;
}

// ──────────────────────────────────────────────────────────
// HTTP — pages (HTML)
// ──────────────────────────────────────────────────────────

// Shared style block — used by both the AP setup page and the debug UI.
#define PHASE_STYLE \
"*{box-sizing:border-box;margin:0;padding:0}" \
"body{background:#0a0a0a;color:#e8e0d0;font-family:'Courier New',monospace;" \
"display:flex;justify-content:center;min-height:100vh;padding:20px}" \
".card{background:#111;border:1px solid #2a2a2a;border-radius:4px;" \
"padding:32px;width:100%;max-width:420px;align-self:flex-start;margin-top:40px}" \
"h1{font-size:11px;letter-spacing:.35em;text-transform:uppercase;color:#555;margin-bottom:32px}" \
".phase-display{text-align:center;margin-bottom:36px}" \
".phase-name{font-size:24px;color:#e8e0d0;margin-bottom:8px;font-weight:normal}" \
".phase-pct{font-size:12px;color:#555;letter-spacing:.2em}" \
".mode-row{display:flex;gap:10px;margin-bottom:32px}" \
".mode-btn{flex:1;padding:11px;border:1px solid #333;background:transparent;" \
"color:#555;font-family:inherit;font-size:11px;letter-spacing:.2em;" \
"text-transform:uppercase;cursor:pointer;border-radius:2px;transition:all .2s}" \
".mode-btn.active{border-color:#e8e0d0;color:#e8e0d0}" \
".controls{transition:opacity .3s}" \
".controls.dim{opacity:.25;pointer-events:none}" \
"label{display:flex;justify-content:space-between;font-size:10px;letter-spacing:.2em;" \
"color:#555;text-transform:uppercase;margin-bottom:8px}" \
"label span{color:#e8e0d0;letter-spacing:0}" \
"input[type=range]{width:100%;accent-color:#e8e0d0;cursor:pointer;margin-bottom:20px}" \
"input[type=date],input[type=text],input[type=password],select{width:100%;background:#1a1a1a;" \
"border:1px solid #2a2a2a;color:#e8e0d0;font-family:inherit;font-size:13px;padding:11px;" \
"border-radius:2px;margin-bottom:20px;color-scheme:dark}" \
".slider-labels{display:flex;justify-content:space-between;" \
"font-size:10px;color:#444;margin-top:-16px;margin-bottom:20px}" \
"hr{border:none;border-top:1px solid #1e1e1e;margin:24px 0}" \
".section-title{font-size:10px;letter-spacing:.3em;text-transform:uppercase;" \
"color:#444;margin-bottom:20px}" \
".note{font-size:10px;color:#444;margin-top:-14px;margin-bottom:20px;line-height:1.6}" \
".save-btn,.primary-btn{width:100%;padding:13px;border:1px solid #e8e0d0;background:transparent;" \
"color:#e8e0d0;font-family:inherit;font-size:11px;letter-spacing:.25em;" \
"text-transform:uppercase;cursor:pointer;border-radius:2px;margin-top:4px;transition:all .2s}" \
".save-btn:hover,.primary-btn:hover{background:#e8e0d0;color:#0a0a0a}" \
".danger-btn{width:100%;padding:13px;border:1px solid #6a3030;background:transparent;" \
"color:#c66;font-family:inherit;font-size:11px;letter-spacing:.25em;" \
"text-transform:uppercase;cursor:pointer;border-radius:2px;margin-top:4px;transition:all .2s}" \
".danger-btn:hover{background:#6a3030;color:#f8e8e8}" \
".status{font-size:10px;color:#333;text-align:center;letter-spacing:.1em;margin-top:16px}" \
".saved-msg{color:#888;font-size:10px;text-align:center;margin-top:12px;" \
"letter-spacing:.15em;opacity:0;transition:opacity .5s}" \
".err{color:#c66;font-size:11px;text-align:center;margin-top:8px}"

// ───── AP setup page ─────
static const char *SETUP_PAGE =
"<!DOCTYPE html><html><head>"
"<meta name='viewport' content='width=device-width, initial-scale=1'>"
"<title>Phase — Setup</title>"
"<style>" PHASE_STYLE "</style></head><body><div class='card'>"
"<h1>Lunar Objects &mdash; Phase <span id='hdr-host' style='color:#e8e0d0;letter-spacing:.15em;text-transform:none'></span></h1>"
"<div class='phase-display'>"
"<div class='phase-name'>Hello.</div>"
"<div class='phase-pct'>Pick your Wi-Fi network</div>"
"</div>"
"<form method='POST' action='/setup'>"
"<label>Network<span></span></label>"
"<select name='ssid' id='ssid'><option value=''>scanning…</option></select>"
"<label>Password<span></span></label>"
"<input type='password' name='pass' autocomplete='off'/>"
"<button class='primary-btn' type='submit'>Connect</button>"
"</form>"
"<hr/>"
"<div class='section-title'>or, run without Wi-Fi</div>"
"<form method='POST' action='/setup_standalone'>"
"<label>Today's date<span></span></label>"
"<input type='date' name='date' id='date-today' required/>"
"<button class='primary-btn' type='submit'>Start Standalone</button>"
"</form>"
"<div class='note'>For venues without Wi-Fi. Phase will display the moon for the date you enter — no internet needed. The \"phase\" AP stays on so you can still reach /debug.</div>"
"<div class='status'>firmware " FW_VERSION "</div>"
"</div>"
"<script>"
"document.getElementById('date-today').value=new Date().toISOString().substring(0,10);"
"fetch('/whoami').then(function(r){return r.text();}).then(function(h){"
"document.getElementById('hdr-host').textContent='['+h+']';document.title='Setup — '+h;"
"}).catch(function(){});"
"fetch('/scan').then(function(r){return r.json();}).then(function(j){"
"var s=document.getElementById('ssid');s.innerHTML='';"
"if(!j.networks||!j.networks.length){var o=document.createElement('option');"
"o.value='';o.textContent='no networks found';s.appendChild(o);return;}"
"j.networks.forEach(function(n){var o=document.createElement('option');"
"o.value=n.ssid;o.textContent=n.ssid+'  ('+n.rssi+' dBm'+(n.locked?' \\uD83D\\uDD12':'')+')';"
"s.appendChild(o);});}).catch(function(){"
"var s=document.getElementById('ssid');s.innerHTML="
"'<option value=\"\">scan failed — type SSID manually</option>'"
"+'<option value=\"__manual__\">(enter manually)</option>';});"
"</script>"
"</body></html>";

// ───── STA mode landing ─────
static const char *LANDING_PAGE =
"<!DOCTYPE html><html><head>"
"<meta name='viewport' content='width=device-width, initial-scale=1'>"
"<meta http-equiv='refresh' content='0;url=/debug'>"
"<title>Phase</title>"
"<style>" PHASE_STYLE "</style></head><body><div class='card'>"
"<h1>Lunar Objects &mdash; Phase</h1>"
"<div class='phase-display'>"
"<div class='phase-name'>online.</div>"
"<div class='phase-pct'>→ /debug</div>"
"</div></div></body></html>";

// ───── Debug login page ─────
static const char *LOGIN_PAGE =
"<!DOCTYPE html><html><head>"
"<meta name='viewport' content='width=device-width, initial-scale=1'>"
"<title>Phase — Debug</title>"
"<style>" PHASE_STYLE "</style></head><body><div class='card'>"
"<h1>Lunar Objects &mdash; Debug <span id='hdr-host' style='color:#e8e0d0;letter-spacing:.15em;text-transform:none'></span></h1>"
"<div class='phase-display'>"
"<div class='phase-name'>Locked.</div>"
"<div class='phase-pct'>authenticate to continue</div>"
"</div>"
"<script>fetch('/whoami').then(function(r){return r.text();}).then(function(h){"
"document.getElementById('hdr-host').textContent='['+h+']';document.title=h+' — login';"
"}).catch(function(){});</script>"
"<form method='POST' action='/debug/login'>"
"<label>Username<span></span></label>"
"<input type='text' name='user' autocomplete='username'/>"
"<label>Password<span></span></label>"
"<input type='password' name='pass' autocomplete='current-password'/>"
"<button class='primary-btn' type='submit'>Sign In</button>"
"</form>"
"<div class='status'>firmware " FW_VERSION "</div>"
"</div></body></html>";

// ───── Debug main UI ─────
static const char *DEBUG_PAGE =
"<!DOCTYPE html><html><head>"
"<meta name='viewport' content='width=device-width, initial-scale=1'>"
"<title>Phase</title>"
"<style>" PHASE_STYLE "</style></head><body><div class='card'>"
"<h1>Lunar Objects &mdash; Phase <span id='hdr-host' style='color:#e8e0d0;letter-spacing:.15em;text-transform:none'></span></h1>"
"<div class='phase-display'>"
"<div class='phase-name' id='pname'>—</div>"
"<div class='phase-pct' id='ppct'>—</div>"
"</div>"
"<div class='mode-row'>"
"<button class='mode-btn active' id='btn-real' onclick='setMode(false)'>Real Moon</button>"
"<button class='mode-btn' id='btn-manual' onclick='setMode(true)'>Manual</button>"
"</div>"
"<div class='controls dim' id='moon-controls'>"
"<label>Phase cycle<span id='lbl-phase'></span></label>"
"<input type='range' id='slider' min='0' max='1000' value='0' oninput='onSlider(this.value)'/>"
"<div class='slider-labels'><span>New</span><span>First Qtr</span><span>Full</span><span>Last Qtr</span><span>New</span></div>"
"<label>Pick a date</label>"
"<input type='date' id='datepicker' onchange='onDate(this.value)'/>"
"</div>"
"<hr/>"
"<div class='section-title'>Device Name</div>"
"<label>Friendly name<span id='lbl-friendly'></span></label>"
"<input type='text' id='fname' maxlength='24' placeholder='e.g. e00-2-3' autocapitalize='off' autocorrect='off' spellcheck='false'/>"
"<button class='save-btn' onclick='saveName()'>Rename Lamp</button>"
"<div class='saved-msg' id='name-saved'>Renamed.</div>"
"<div class='note'>Both the <b>Wi-Fi setup SSID</b> and the <b>mDNS hostname</b> use this. Lowercase / digits / hyphens only. Leave empty to use the MAC-derived default (<code id='mac-tag'>—</code>). Multiple lamps on the same network need unique names.</div>"
"<hr/>"
"<div class='section-title'>Illumination Curve</div>"
"<label>First quarter<span id='lbl-q1'></span></label>"
"<input type='range' id='s-q1' min='100' max='900' oninput='onParam()'/>"
"<div class='note'>How lit the ring looks at first quarter. Lower = less lit.</div>"
"<label>Waxing gibbous<span id='lbl-g1'></span></label>"
"<input type='range' id='s-g1' min='500' max='999' oninput='onParam()'/>"
"<div class='note'>How quickly it fills toward full. Lower = slower.</div>"
"<label>Waning gibbous<span id='lbl-g3'></span></label>"
"<input type='range' id='s-g3' min='500' max='999' oninput='onParam()'/>"
"<div class='note'>How quickly it empties from full. Lower = stays full longer.</div>"
"<label>Last quarter<span id='lbl-q3'></span></label>"
"<input type='range' id='s-q3' min='100' max='900' oninput='onParam()'/>"
"<div class='note'>How lit the ring looks at last quarter. Lower = less lit.</div>"
"<hr/>"
"<div class='section-title'>Rendering Parameters</div>"
"<label>Power cap<span id='lbl-pcap'></span></label>"
"<input type='range' id='s-pcap' min='100' max='1000' oninput='onParam()'/>"
"<div class='note'>Hard ceiling on output current. Keep at 30% if powered through a USB hub (≈ 800 mA peak). Set to 100% on the external 5 V supply (APV-35-5).</div>"
"<label>Brightness<span id='lbl-bright'></span></label>"
"<input type='range' id='s-bright' min='100' max='1000' oninput='onParam()'/>"
"<label>Face gradient<span id='lbl-grad'></span></label>"
"<input type='range' id='s-grad' min='0' max='900' oninput='onParam()'/>"
"<label>Brightness floor (debug)<span id='lbl-floor'></span></label>"
"<input type='range' id='s-floor' min='0' max='500' oninput='onParam()'/>"
"<div class='note'>Cut-off for the lowest brightness pixels. Raise to fix terminator flicker, lower for a softer edge.</div>"
"<hr/>"
"<div class='section-title'>Glimmer (debug)</div>"
"<div class='mode-row'>"
"<button class='mode-btn active' id='btn-gon-off' onclick='setGlimmer(false)'>Off</button>"
"<button class='mode-btn' id='btn-gon-on' onclick='setGlimmer(true)'>On</button>"
"</div>"
"<div class='controls' id='glimmer-controls'>"
"<label>Body shimmer<span id='lbl-gbase'></span></label>"
"<input type='range' id='s-gbase' min='0' max='1000' oninput='onParam()'/>"
"<div class='note'>Brightness wobble across the lit face. Past 0.500 the pixels go fully dark at troughs (blink, not wobble).</div>"
"<label>Edge shimmer<span id='lbl-gedge'></span></label>"
"<input type='range' id='s-gedge' min='0' max='2000' oninput='onParam()'/>"
"<div class='note'>Extra wobble near the terminator. Past 1.000 the edges fully blink. May reintroduce flicker if data line is noisy.</div>"
"<label>Speed<span id='lbl-gspeed'></span></label>"
"<input type='range' id='s-gspeed' min='50' max='4000' oninput='onParam()'/>"
"<div class='note'>How fast the shimmer oscillates.</div>"
"</div>"
"<hr/>"
"<div class='section-title'>Preview Cycle</div>"
"<div class='mode-row'>"
"<button class='mode-btn active' id='btn-prev-stop' onclick='setPreview(false)'>Stop</button>"
"<button class='mode-btn' id='btn-prev-play' onclick='setPreview(true)'>Play</button>"
"</div>"
"<label>Color<span id='lbl-prev-color'>#ffffff</span></label>"
"<input type='color' id='prev-color' value='#ffffff' oninput='onPreviewParam()' onchange='onPreviewParam()'/>"
"<label>RGB brightness<span id='lbl-prev-rgb'></span></label>"
"<input type='range' id='s-prev-rgb' min='0' max='1000' oninput='onPreviewParam()'/>"
"<label>White brightness<span id='lbl-prev-w'></span></label>"
"<input type='range' id='s-prev-w' min='0' max='1000' oninput='onPreviewParam()'/>"
"<label>Speed<span id='lbl-prev-speed'></span></label>"
"<input type='range' id='s-prev-speed' min='25' max='1000' oninput='onPreviewParam()'/>"
"<div class='slider-labels'><span>0.25x</span><span>1x</span><span>10x</span></div>"
"<div class='note'>Runs a smooth moon-phase cycle in your picked color. Drive RGB only, W only, or mix.</div>"
"<hr/>"
"<button class='save-btn' onclick='saveParams()'>Save to Device</button>"
"<div class='saved-msg' id='saved-msg'>Saved.</div>"
"<hr/>"
"<div class='section-title'>Network</div>"
"<button class='danger-btn' onclick='resetWifi()'>Reset to AP Mode</button>"
"<div class='note'>Wipes the saved Wi-Fi credentials and reboots the device. \"phase\" AP will appear so you can re-provision. Render parameters are kept.</div>"
"<hr/>"
"<div class='section-title'>Firmware</div>"
"<button class='save-btn' onclick='checkOta()'>Check for Update Now</button>"
"<div class='note'>Updates are automatic — the lamp checks for a new release shortly after boot and every 6 hours, then installs and reboots on its own. This button just skips the wait.</div>"
"<div class='status' id='ota-status'>—</div>"
"<div class='status' id='status'>—</div>"
"<div class='status'>firmware " FW_VERSION "</div>"
"</div>"
"<script>"
"var isManual=false;"
// Trailing-edge throttle: fire immediately, then suppress further calls
// within `ms` and dispatch the latest value once the window closes.
// Otherwise rapid slider drags flood the C3's HTTPD queue and the LEDs
// visibly chase the cursor.
"function makeThrottle(fn,ms){var last=0,t=null,pendingArgs=null;"
"return function(){pendingArgs=arguments;var now=Date.now();"
"if(now-last>=ms){last=now;fn.apply(null,pendingArgs);pendingArgs=null;"
"if(t){clearTimeout(t);t=null;}}"
"else if(!t){var wait=ms-(now-last);"
"t=setTimeout(function(){last=Date.now();t=null;"
"if(pendingArgs){fn.apply(null,pendingArgs);pendingArgs=null;}},wait);}};}"
"var sendPhaseTh=makeThrottle(function(p){"
"var url=p<0?'/debug/set?mode=real':'/debug/set?mode=manual&phase='+p.toFixed(4);"
"fetch(url).catch(function(){});},60);"
"var sendParamsTh=makeThrottle(function(q){fetch(q).catch(function(){});},80);"
"var sendPreviewTh=makeThrottle(function(q){fetch(q).catch(function(){});},80);"
"function setPreview(on){"
"document.getElementById('btn-prev-stop').classList.toggle('active',!on);"
"document.getElementById('btn-prev-play').classList.toggle('active',on);"
"fetch('/debug/preview?on='+(on?'1':'0')).catch(function(){});}"
"function onPreviewParam(){"
"var color=document.getElementById('prev-color').value.substring(1);"
"var rgb=document.getElementById('s-prev-rgb').value/1000.0;"
"var w=document.getElementById('s-prev-w').value/1000.0;"
"var speed=document.getElementById('s-prev-speed').value/100.0;"
"document.getElementById('lbl-prev-color').textContent='#'+color;"
"document.getElementById('lbl-prev-rgb').textContent=rgb.toFixed(3);"
"document.getElementById('lbl-prev-w').textContent=w.toFixed(3);"
"document.getElementById('lbl-prev-speed').textContent=speed.toFixed(2)+'x';"
"sendPreviewTh('/debug/preview?color='+color+'&rgb='+rgb.toFixed(3)"
"+'&w='+w.toFixed(3)+'&speed='+speed.toFixed(3));}"
"function phaseName(p){"
"var ill=0.5-0.5*Math.cos(p*2*Math.PI);"
"if(ill<0.02)return'New Moon';"
"if(p<0.5&&ill<0.48)return'Waxing Crescent';"
"if(p<0.5&&ill<0.52)return'First Quarter';"
"if(p<0.5&&ill<0.98)return'Waxing Gibbous';"
"if(ill>=0.98)return'Full Moon';"
"if(p>=0.5&&ill>0.52)return'Waning Gibbous';"
"if(p>=0.5&&ill>0.48)return'Last Quarter';"
"return'Waning Crescent';}"
"function updateDisplay(p){"
"var ill=0.5-0.5*Math.cos(p*2*Math.PI);"
"document.getElementById('pname').textContent=phaseName(p);"
"document.getElementById('ppct').textContent=Math.round(ill*100)+'% illuminated';"
"document.getElementById('lbl-phase').textContent=Math.round(p*1000)/1000;}"
"function setMode(manual){"
"isManual=manual;"
"document.getElementById('btn-real').classList.toggle('active',!manual);"
"document.getElementById('btn-manual').classList.toggle('active',manual);"
"document.getElementById('moon-controls').classList.toggle('dim',!manual);"
"if(!manual)sendPhase(-1);}"
"function onSlider(v){"
"if(!isManual)return;"
"var p=v/1000.0;updateDisplay(p);sendPhase(p);}"
"function onDate(val){"
"if(!isManual||!val)return;"
"var d=new Date(val+'T12:00:00Z');"
"var known=new Date('2000-01-06T18:14:00Z');"
"var cycle=29.53058770576*24*3600*1000;"
"var p=((d-known)%cycle)/cycle;"
"if(p<0)p+=1.0;"
"document.getElementById('slider').value=Math.round(p*1000);"
"updateDisplay(p);sendPhase(p);}"
"function sendPhase(p){sendPhaseTh(p);}"
"function onParam(){"
"var q1=document.getElementById('s-q1').value/1000.0;"
"var g1=document.getElementById('s-g1').value/1000.0;"
"var g3=document.getElementById('s-g3').value/1000.0;"
"var q3=document.getElementById('s-q3').value/1000.0;"
"var bright=document.getElementById('s-bright').value/1000.0;"
"var grad=document.getElementById('s-grad').value/1000.0;"
"var floor=document.getElementById('s-floor').value/1000.0;"
"var pcap=document.getElementById('s-pcap').value/1000.0;"
"var gbase=document.getElementById('s-gbase').value/1000.0;"
"var gedge=document.getElementById('s-gedge').value/1000.0;"
"var gspeed=document.getElementById('s-gspeed').value/1000.0;"
"document.getElementById('lbl-q1').textContent=q1.toFixed(3);"
"document.getElementById('lbl-g1').textContent=g1.toFixed(3);"
"document.getElementById('lbl-g3').textContent=g3.toFixed(3);"
"document.getElementById('lbl-q3').textContent=q3.toFixed(3);"
"document.getElementById('lbl-bright').textContent=bright.toFixed(3);"
"document.getElementById('lbl-grad').textContent=grad.toFixed(3);"
"document.getElementById('lbl-floor').textContent=floor.toFixed(3);"
"document.getElementById('lbl-pcap').textContent=pcap.toFixed(3);"
"document.getElementById('lbl-gbase').textContent=gbase.toFixed(3);"
"document.getElementById('lbl-gedge').textContent=gedge.toFixed(3);"
"document.getElementById('lbl-gspeed').textContent=gspeed.toFixed(3);"
"sendParamsTh('/debug/params?q1='+q1.toFixed(3)+'&g1='+g1.toFixed(3)"
"+'&g3='+g3.toFixed(3)+'&q3='+q3.toFixed(3)"
"+'&bright='+bright.toFixed(3)+'&grad='+grad.toFixed(3)"
"+'&floor='+floor.toFixed(3)+'&pcap='+pcap.toFixed(3)"
"+'&gbase='+gbase.toFixed(3)+'&gedge='+gedge.toFixed(3)"
"+'&gspeed='+gspeed.toFixed(3));}"
"function setGlimmer(on){"
"document.getElementById('btn-gon-off').classList.toggle('active',!on);"
"document.getElementById('btn-gon-on').classList.toggle('active',on);"
"fetch('/debug/params?gon='+(on?'1':'0')).catch(function(){});}"
"function saveParams(){"
"fetch('/debug/params/save').then(function(){"
"var m=document.getElementById('saved-msg');"
"m.style.opacity=1;"
"setTimeout(function(){m.style.opacity=0;},2000);"
"}).catch(function(){});}"
"function resetWifi(){"
"if(!confirm('Wipe saved Wi-Fi credentials and reboot into AP mode?'))return;"
"fetch('/debug/reset_wifi').catch(function(){});}"
"function checkOta(){fetch('/debug/ota_check').catch(function(){});}"
"function saveName(){"
"var n=document.getElementById('fname').value;"
"fetch('/debug/name?name='+encodeURIComponent(n)).then(function(r){return r.text();})"
".then(function(host){"
"document.getElementById('hdr-host').textContent=host?('['+host+']'):'';"
"var m=document.getElementById('name-saved');"
"m.textContent=host?('Renamed to '+host):'Renamed.';m.style.opacity=1;"
"setTimeout(function(){m.style.opacity=0;},2500);"
"poll();"
"}).catch(function(){});}"
"function poll(){"
"fetch('/debug/status').then(function(r){return r.json();})"
".then(function(d){"
"if(!isManual){updateDisplay(d.phase);document.getElementById('slider').value=Math.round(d.phase*1000);}"
"document.getElementById('status').textContent='http://'+d.ip;"
"document.getElementById('s-q1').value=Math.round(d.q1*1000);"
"document.getElementById('s-g1').value=Math.round(d.g1*1000);"
"document.getElementById('s-g3').value=Math.round(d.g3*1000);"
"document.getElementById('s-q3').value=Math.round(d.q3*1000);"
"document.getElementById('s-bright').value=Math.round(d.bright*1000);"
"document.getElementById('s-grad').value=Math.round(d.grad*1000);"
"document.getElementById('s-floor').value=Math.round(d.floor*1000);"
"document.getElementById('s-pcap').value=Math.round(d.pcap*1000);"
"document.getElementById('s-gbase').value=Math.round(d.gbase*1000);"
"document.getElementById('s-gedge').value=Math.round(d.gedge*1000);"
"document.getElementById('s-gspeed').value=Math.round(d.gspeed*1000);"
"document.getElementById('btn-gon-off').classList.toggle('active',!d.gon);"
"document.getElementById('btn-gon-on').classList.toggle('active',d.gon);"
"document.getElementById('lbl-q1').textContent=d.q1.toFixed(3);"
"document.getElementById('lbl-g1').textContent=d.g1.toFixed(3);"
"document.getElementById('lbl-g3').textContent=d.g3.toFixed(3);"
"document.getElementById('lbl-q3').textContent=d.q3.toFixed(3);"
"document.getElementById('lbl-bright').textContent=d.bright.toFixed(3);"
"document.getElementById('lbl-grad').textContent=d.grad.toFixed(3);"
"document.getElementById('lbl-floor').textContent=d.floor.toFixed(3);"
"document.getElementById('lbl-pcap').textContent=d.pcap.toFixed(3);"
"document.getElementById('lbl-gbase').textContent=d.gbase.toFixed(3);"
"document.getElementById('lbl-gedge').textContent=d.gedge.toFixed(3);"
"document.getElementById('lbl-gspeed').textContent=d.gspeed.toFixed(3);"
"document.getElementById('btn-prev-stop').classList.toggle('active',!d.prev_on);"
"document.getElementById('btn-prev-play').classList.toggle('active',d.prev_on);"
"document.getElementById('prev-color').value='#'+d.prev_color;"
"document.getElementById('lbl-prev-color').textContent='#'+d.prev_color;"
"document.getElementById('s-prev-rgb').value=Math.round(d.prev_rgb*1000);"
"document.getElementById('s-prev-w').value=Math.round(d.prev_w*1000);"
"document.getElementById('s-prev-speed').value=Math.round(d.prev_speed*100);"
"document.getElementById('lbl-prev-rgb').textContent=d.prev_rgb.toFixed(3);"
"document.getElementById('lbl-prev-w').textContent=d.prev_w.toFixed(3);"
"document.getElementById('lbl-prev-speed').textContent=d.prev_speed.toFixed(2)+'x';"
"var o=d.ota_msg;"
"if(d.ota==='downloading')o='downloading… '+d.ota_pct+'%';"
"else if(d.ota==='checking')o='checking…';"
"else if(d.ota==='rebooting')o='update installed — rebooting';"
"document.getElementById('ota-status').textContent='update: '+o;"
"document.getElementById('hdr-host').textContent=d.host?('['+d.host+']'):'';"
"document.title=d.host||'Phase';"
"var f=document.getElementById('fname');"
"if(document.activeElement!==f)f.value=d.friendly||'';"
"document.getElementById('mac-tag').textContent='phase-'+d.mac_tag;"
"}).catch(function(){});}"
"poll();setInterval(poll,5000);"
"</script></body></html>";

// ──────────────────────────────────────────────────────────
// HTTP — helpers for parsing form bodies
// ──────────────────────────────────────────────────────────

static int url_decode_into(char *dst, size_t dst_sz, const char *src, size_t src_len)
{
    size_t o = 0;
    for (size_t i = 0; i < src_len && o + 1 < dst_sz; i++) {
        char c = src[i];
        if (c == '+') {
            dst[o++] = ' ';
        } else if (c == '%' && i + 2 < src_len) {
            char hex[3] = {src[i+1], src[i+2], 0};
            dst[o++] = (char)strtol(hex, NULL, 16);
            i += 2;
        } else {
            dst[o++] = c;
        }
    }
    dst[o] = '\0';
    return (int)o;
}

static bool form_get(const char *body, const char *key, char *out, size_t out_sz)
{
    size_t klen = strlen(key);
    const char *p = body;
    while (*p) {
        if (strncmp(p, key, klen) == 0 && p[klen] == '=') {
            const char *v = p + klen + 1;
            const char *end = strchr(v, '&');
            size_t vlen = end ? (size_t)(end - v) : strlen(v);
            url_decode_into(out, out_sz, v, vlen);
            return true;
        }
        const char *next = strchr(p, '&');
        if (!next) break;
        p = next + 1;
    }
    out[0] = '\0';
    return false;
}

// Drain a POST body (small forms only). Caller owns buffer.
static int read_post_body(httpd_req_t *req, char *buf, size_t buf_sz)
{
    int remaining = req->content_len;
    int total     = 0;
    while (remaining > 0 && total + 1 < (int)buf_sz) {
        int chunk = httpd_req_recv(req, buf + total,
                                   remaining < (int)(buf_sz - 1 - total)
                                       ? remaining : (int)(buf_sz - 1 - total));
        if (chunk <= 0) break;
        total     += chunk;
        remaining -= chunk;
    }
    buf[total] = '\0';
    return total;
}

// ──────────────────────────────────────────────────────────
// HTTP — handlers (AP mode)
// ──────────────────────────────────────────────────────────

// Open (no-auth) endpoint returning the currently-active mDNS hostname
// (which is just "phase" in AP mode, "phase-<tag>" in STA mode) so the
// setup / login page can display the URL the client should use.
static esp_err_t handle_name_get(httpd_req_t *req)
{
    char host[48];
    active_mdns_hostname(host, sizeof(host));
    httpd_resp_set_type(req, "text/plain");
    httpd_resp_send(req, host, strlen(host));
    return ESP_OK;
}

static esp_err_t handle_setup_root(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, SETUP_PAGE, strlen(SETUP_PAGE));
    return ESP_OK;
}

static esp_err_t handle_scan(httpd_req_t *req)
{
    wifi_scan_config_t scan = {0};
    esp_wifi_scan_start(&scan, true);
    uint16_t n = 0;
    esp_wifi_scan_get_ap_num(&n);
    if (n > 24) n = 24;
    wifi_ap_record_t *aps = calloc(n, sizeof(wifi_ap_record_t));
    if (!aps) {
        httpd_resp_set_type(req, "application/json");
        httpd_resp_send(req, "{\"networks\":[]}", 15);
        return ESP_OK;
    }
    esp_wifi_scan_get_ap_records(&n, aps);

    char *out = malloc(4096);
    if (!out) { free(aps); return ESP_FAIL; }
    int len = snprintf(out, 4096, "{\"networks\":[");
    for (int i = 0; i < n; i++) {
        char ssid_esc[64] = {0};
        // Naive JSON escape — SSIDs rarely contain quotes/backslashes.
        const char *s = (const char *)aps[i].ssid;
        if (!s[0]) continue;
        int j = 0;
        for (int k = 0; s[k] && j < (int)sizeof(ssid_esc) - 2; k++) {
            char c = s[k];
            if (c == '"' || c == '\\') ssid_esc[j++] = '\\';
            ssid_esc[j++] = c;
        }
        ssid_esc[j] = '\0';
        len += snprintf(out + len, 4096 - len,
            "%s{\"ssid\":\"%s\",\"rssi\":%d,\"locked\":%s}",
            (i == 0 ? "" : ","),
            ssid_esc, aps[i].rssi,
            aps[i].authmode == WIFI_AUTH_OPEN ? "false" : "true");
        if (len > 3900) break;
    }
    len += snprintf(out + len, 4096 - len, "]}");
    free(aps);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, out, len);
    free(out);
    return ESP_OK;
}

static esp_err_t handle_setup_post(httpd_req_t *req)
{
    char body[512];
    int n = read_post_body(req, body, sizeof(body));
    if (n <= 0) {
        httpd_resp_set_status(req, "400 Bad Request");
        httpd_resp_send(req, "no body", 7);
        return ESP_OK;
    }
    char ssid[64] = {0};
    char pass[64] = {0};
    if (!form_get(body, "ssid", ssid, sizeof(ssid)) || strlen(ssid) == 0) {
        httpd_resp_set_status(req, "400 Bad Request");
        httpd_resp_send(req, "missing ssid", 12);
        return ESP_OK;
    }
    form_get(body, "pass", pass, sizeof(pass));

    save_wifi_creds(ssid, pass);
    ESP_LOGI(TAG, "Saved creds for \"%s\" — rebooting…", ssid);

    const char *ok_html =
        "<!DOCTYPE html><html><head><meta name='viewport' "
        "content='width=device-width, initial-scale=1'>"
        "<style>" PHASE_STYLE "</style></head><body><div class='card'>"
        "<h1>Lunar Objects &mdash; Phase</h1>"
        "<div class='phase-display'>"
        "<div class='phase-name'>Saved.</div>"
        "<div class='phase-pct'>connecting…</div>"
        "</div></div></body></html>";
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, ok_html, strlen(ok_html));

    vTaskDelay(pdMS_TO_TICKS(500));
    esp_restart();
    return ESP_OK;
}

// Standalone mode: take a date from the user (when there's no Wi-Fi
// available), set the system clock to noon UTC on that date, and signal
// app_main to drop straight into ANIM_MOON. The AP stays up so /debug is
// still reachable. Session-only — does not persist across reboots.
static esp_err_t handle_setup_standalone(httpd_req_t *req)
{
    char body[256];
    int n = read_post_body(req, body, sizeof(body));
    if (n <= 0) {
        httpd_resp_set_status(req, "400 Bad Request");
        httpd_resp_send(req, "no body", 7);
        return ESP_OK;
    }
    char date_str[16] = {0};
    if (!form_get(body, "date", date_str, sizeof(date_str)) || strlen(date_str) < 8) {
        httpd_resp_set_status(req, "400 Bad Request");
        httpd_resp_send(req, "missing date", 12);
        return ESP_OK;
    }
    int y = 0, m = 0, d = 0;
    if (sscanf(date_str, "%d-%d-%d", &y, &m, &d) != 3 ||
        y < 1970 || y > 2100 || m < 1 || m > 12 || d < 1 || d > 31) {
        httpd_resp_set_status(req, "400 Bad Request");
        httpd_resp_send(req, "bad date format (want YYYY-MM-DD)", 34);
        return ESP_OK;
    }

    // Force UTC interpretation so mktime() doesn't pull in whatever TZ the
    // chip thinks it's in.
    setenv("TZ", "UTC0", 1);
    tzset();
    struct tm tm = {0};
    tm.tm_year = y - 1900;
    tm.tm_mon  = m - 1;
    tm.tm_mday = d;
    tm.tm_hour = 12;
    time_t epoch = mktime(&tm);
    if (epoch == (time_t)-1) {
        httpd_resp_set_status(req, "400 Bad Request");
        httpd_resp_send(req, "bad date", 8);
        return ESP_OK;
    }
    struct timeval tv = { .tv_sec = epoch, .tv_usec = 0 };
    settimeofday(&tv, NULL);
    ESP_LOGI(TAG, "Standalone: clock set to %04d-%02d-%02d 12:00 UTC — entering ANIM_MOON",
             y, m, d);

    xEventGroupSetBits(s_events, EV_STANDALONE_REQUESTED);

    const char *ok_html =
        "<!DOCTYPE html><html><head><meta name='viewport' "
        "content='width=device-width, initial-scale=1'>"
        "<style>" PHASE_STYLE "</style></head><body><div class='card'>"
        "<h1>Lunar Objects &mdash; Phase</h1>"
        "<div class='phase-display'>"
        "<div class='phase-name'>Standalone.</div>"
        "<div class='phase-pct'>moon is now showing for your date</div>"
        "</div>"
        "<div class='note' style='margin-top:24px'>The \"phase\" Wi-Fi network is still up so you can reach /debug to tune. Reboot to return to setup.</div>"
        "</div></body></html>";
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, ok_html, strlen(ok_html));
    return ESP_OK;
}

// ──────────────────────────────────────────────────────────
// HTTP — handlers (STA mode + debug portal)
// ──────────────────────────────────────────────────────────

static esp_err_t handle_landing(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, LANDING_PAGE, strlen(LANDING_PAGE));
    return ESP_OK;
}

static esp_err_t handle_login_get(httpd_req_t *req)
{
    if (req_is_authed(req)) {
        return send_redirect(req, "/debug", NULL);
    }
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, LOGIN_PAGE, strlen(LOGIN_PAGE));
    return ESP_OK;
}

static esp_err_t handle_login_post(httpd_req_t *req)
{
    char body[256];
    int n = read_post_body(req, body, sizeof(body));
    if (n <= 0) {
        httpd_resp_set_status(req, "400 Bad Request");
        httpd_resp_send(req, "no body", 7);
        return ESP_OK;
    }
    char user[64] = {0};
    char pass[64] = {0};
    form_get(body, "user", user, sizeof(user));
    form_get(body, "pass", pass, sizeof(pass));

    if (strcmp(user, DEBUG_USER) == 0 && strcmp(pass, DEBUG_PASS) == 0) {
        char cookie[96];
        snprintf(cookie, sizeof(cookie),
            "phase_sess=%s; Path=/; Max-Age=2592000; HttpOnly; SameSite=Lax",
            s_session_token);
        return send_redirect(req, "/debug", cookie);
    }
    // Wrong creds → re-render the login page with an error banner.
    const char *fail =
        "<!DOCTYPE html><html><head><meta name='viewport' "
        "content='width=device-width, initial-scale=1'>"
        "<style>" PHASE_STYLE "</style></head><body><div class='card'>"
        "<h1>Lunar Objects &mdash; Debug</h1>"
        "<div class='phase-display'>"
        "<div class='phase-name'>Denied.</div>"
        "<div class='phase-pct'>incorrect credentials</div>"
        "</div>"
        "<form method='POST' action='/debug/login'>"
        "<label>Username<span></span></label>"
        "<input type='text' name='user' autocomplete='username'/>"
        "<label>Password<span></span></label>"
        "<input type='password' name='pass' autocomplete='current-password'/>"
        "<button class='primary-btn' type='submit'>Sign In</button>"
        "</form>"
        "<div class='err'>Wrong username or password.</div>"
        "</div></body></html>";
    httpd_resp_set_status(req, "401 Unauthorized");
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, fail, strlen(fail));
    return ESP_OK;
}

static esp_err_t handle_logout(httpd_req_t *req)
{
    return send_redirect(req, "/debug/login",
        "phase_sess=; Path=/; Max-Age=0");
}

static esp_err_t handle_debug_root(httpd_req_t *req)
{
    if (!req_is_authed(req)) {
        return send_redirect(req, "/debug/login", NULL);
    }
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, DEBUG_PAGE, strlen(DEBUG_PAGE));
    return ESP_OK;
}

static esp_err_t handle_status(httpd_req_t *req)
{
    if (!req_is_authed(req)) {
        return send_redirect(req, "/debug/login", NULL);
    }
    float phase = manual_mode ? manual_phase : calc_moon_phase();
    float ill   = phase_to_illumination(phase);
    char ip_str[16] = "unknown";
    esp_netif_ip_info_t ip_info;
    esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    if (netif && esp_netif_get_ip_info(netif, &ip_info) == ESP_OK)
        snprintf(ip_str, sizeof(ip_str), IPSTR, IP2STR(&ip_info.ip));

    static const char *ota_state_names[] = {
        "idle", "checking", "downloading", "rebooting"
    };

    char host[48];
    device_hostname(host, sizeof(host));

    char buf[1024];
    snprintf(buf, sizeof(buf),
        "{\"phase\":%.4f,\"illumination\":%.1f,\"name\":\"%s\","
        "\"manual\":%s,\"ip\":\"%s\","
        "\"host\":\"%s\",\"friendly\":\"%s\",\"mac_tag\":\"%s\","
        "\"q1\":%.3f,\"g1\":%.3f,\"g3\":%.3f,\"q3\":%.3f,"
        "\"bright\":%.3f,\"grad\":%.3f,\"floor\":%.3f,\"pcap\":%.3f,"
        "\"gon\":%s,\"gbase\":%.3f,\"gedge\":%.3f,\"gspeed\":%.3f,"
        "\"prev_on\":%s,\"prev_color\":\"%02x%02x%02x\","
        "\"prev_rgb\":%.3f,\"prev_w\":%.3f,\"prev_speed\":%.3f,"
        "\"fw\":\"%s\",\"ota\":\"%s\",\"ota_msg\":\"%s\",\"ota_pct\":%d}",
        phase, ill * 100.0f, phase_name(phase),
        manual_mode ? "true" : "false", ip_str,
        host, s_friendly_name, s_mac_suffix,
        p_curve_q1, p_curve_g1, p_curve_g3, p_curve_q3,
        p_brightness, p_face_gradient, p_floor, p_power_cap,
        p_glimmer_on ? "true" : "false",
        p_glimmer_base, p_glimmer_edge, p_glimmer_speed,
        (s_anim_mode == ANIM_PREVIEW) ? "true" : "false",
        s_preview_r, s_preview_g, s_preview_b,
        s_preview_rgb_dim, s_preview_w_dim, s_preview_speed,
        FW_VERSION, ota_state_names[s_ota_state], s_ota_last,
        s_ota_progress);

    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, buf, strlen(buf));
    return ESP_OK;
}

static esp_err_t handle_set(httpd_req_t *req)
{
    if (!req_is_authed(req)) {
        return send_redirect(req, "/debug/login", NULL);
    }
    char query[128] = {0};
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK) {
        httpd_resp_send(req, "bad request", 11); return ESP_OK;
    }
    char mode_val[16] = {0};
    httpd_query_key_value(query, "mode", mode_val, sizeof(mode_val));
    if (strcmp(mode_val, "real") == 0) {
        manual_mode = false;
    } else if (strcmp(mode_val, "manual") == 0) {
        char phase_val[16] = {0};
        httpd_query_key_value(query, "phase", phase_val, sizeof(phase_val));
        manual_phase = strtof(phase_val, NULL);
        if (manual_phase < 0.0f) manual_phase = 0.0f;
        if (manual_phase > 1.0f) manual_phase = 1.0f;
        manual_mode = true;
    }
    httpd_resp_send(req, "ok", 2);
    return ESP_OK;
}

static esp_err_t handle_params(httpd_req_t *req)
{
    if (!req_is_authed(req)) {
        return send_redirect(req, "/debug/login", NULL);
    }
    char query[256] = {0};
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK) {
        char val[16];
        if (httpd_query_key_value(query, "q1",    val, sizeof(val)) == ESP_OK) p_curve_q1      = strtof(val, NULL);
        if (httpd_query_key_value(query, "g1",    val, sizeof(val)) == ESP_OK) p_curve_g1      = strtof(val, NULL);
        if (httpd_query_key_value(query, "g3",    val, sizeof(val)) == ESP_OK) p_curve_g3      = strtof(val, NULL);
        if (httpd_query_key_value(query, "q3",    val, sizeof(val)) == ESP_OK) p_curve_q3      = strtof(val, NULL);
        if (httpd_query_key_value(query, "bright",val, sizeof(val)) == ESP_OK) p_brightness    = strtof(val, NULL);
        if (httpd_query_key_value(query, "grad",  val, sizeof(val)) == ESP_OK) p_face_gradient = strtof(val, NULL);
        if (httpd_query_key_value(query, "floor", val, sizeof(val)) == ESP_OK) p_floor         = strtof(val, NULL);
        if (httpd_query_key_value(query, "pcap",  val, sizeof(val)) == ESP_OK) {
            float c = strtof(val, NULL);
            if (c < 0.0f) c = 0.0f;
            if (c > 1.0f) c = 1.0f;
            p_power_cap = c;
        }
        if (httpd_query_key_value(query, "gon",   val, sizeof(val)) == ESP_OK) p_glimmer_on    = (strcmp(val, "1") == 0);
        if (httpd_query_key_value(query, "gbase", val, sizeof(val)) == ESP_OK) p_glimmer_base  = strtof(val, NULL);
        if (httpd_query_key_value(query, "gedge", val, sizeof(val)) == ESP_OK) p_glimmer_edge  = strtof(val, NULL);
        if (httpd_query_key_value(query, "gspeed",val, sizeof(val)) == ESP_OK) p_glimmer_speed = strtof(val, NULL);
    }
    httpd_resp_send(req, "ok", 2);
    return ESP_OK;
}

static esp_err_t handle_params_save(httpd_req_t *req)
{
    if (!req_is_authed(req)) {
        return send_redirect(req, "/debug/login", NULL);
    }
    nvs_save_params();
    httpd_resp_send(req, "ok", 2);
    return ESP_OK;
}

// Manually trigger an AP-mode reset from the debug portal — same end state
// as holding the physical reset button for 3 s.
static esp_err_t handle_reset_wifi(httpd_req_t *req)
{
    if (!req_is_authed(req)) {
        return send_redirect(req, "/debug/login", NULL);
    }
    ESP_LOGW(TAG, "Reset to AP mode requested via /debug — clearing creds + rebooting");
    httpd_resp_send(req, "ok", 2);
    vTaskDelay(pdMS_TO_TICKS(150));  // let the response flush
    clear_wifi_creds();
    esp_restart();
    return ESP_OK;
}

// Rename the lamp. Empty name (or a name that sanitizes to empty) reverts
// to the MAC-derived default. mDNS hostname updates live; AP SSID applies
// on the next AP-mode boot.
static esp_err_t handle_name_set(httpd_req_t *req)
{
    if (!req_is_authed(req)) {
        return send_redirect(req, "/debug/login", NULL);
    }
    char query[128] = {0};
    char raw[64]    = {0};
    char clean[32]  = {0};
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK) {
        httpd_query_key_value(query, "name", raw, sizeof(raw));
    }
    sanitize_friendly(raw, clean, sizeof(clean));

    nvs_save_friendly_name(clean);
    strncpy(s_friendly_name, clean, sizeof(s_friendly_name) - 1);
    s_friendly_name[sizeof(s_friendly_name) - 1] = '\0';
    mdns_refresh_hostname();

    char hostname[48];
    device_hostname(hostname, sizeof(hostname));
    ESP_LOGI(TAG, "Renamed device to %s (input=\"%s\")", hostname, raw);
    httpd_resp_send(req, hostname, strlen(hostname));
    return ESP_OK;
}

// Nudge ota_task to run a check-and-update pass right now instead of
// waiting for the next 6-hour cycle. Purely a convenience for testing —
// the automatic cycle needs no interaction.
static esp_err_t handle_ota_check(httpd_req_t *req)
{
    if (!req_is_authed(req)) {
        return send_redirect(req, "/debug/login", NULL);
    }
    xEventGroupSetBits(s_events, EV_OTA_CHECK_NOW);
    httpd_resp_send(req, "ok", 2);
    return ESP_OK;
}

// Preview cycle controls. Runtime state only — does not persist in NVS.
//   on=0|1                turn the preview loop on/off
//   color=RRGGBB          6-hex-digit color (no leading #)
//   rgb=0..1              brightness of the RGB channels
//   w=0..1                brightness of the W channel
//   speed=0.05..20        cycle-rate multiplier (1× = ~25 s/cycle)
static esp_err_t handle_preview(httpd_req_t *req)
{
    if (!req_is_authed(req)) {
        return send_redirect(req, "/debug/login", NULL);
    }
    char query[256] = {0};
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK) {
        char val[16];
        if (httpd_query_key_value(query, "on", val, sizeof(val)) == ESP_OK) {
            bool want_on = (strcmp(val, "1") == 0);
            if (want_on) {
                s_anim_mode = ANIM_PREVIEW;
            } else if (s_anim_mode == ANIM_PREVIEW) {
                s_anim_mode = ANIM_MOON;
            }
        }
        if (httpd_query_key_value(query, "color", val, sizeof(val)) == ESP_OK) {
            if (strlen(val) == 6) {
                char rs[3] = { val[0], val[1], 0 };
                char gs[3] = { val[2], val[3], 0 };
                char bs[3] = { val[4], val[5], 0 };
                s_preview_r = (uint8_t)strtol(rs, NULL, 16);
                s_preview_g = (uint8_t)strtol(gs, NULL, 16);
                s_preview_b = (uint8_t)strtol(bs, NULL, 16);
            }
        }
        if (httpd_query_key_value(query, "rgb", val, sizeof(val)) == ESP_OK) {
            float v = strtof(val, NULL);
            if (v < 0.0f) v = 0.0f;
            if (v > 1.0f) v = 1.0f;
            s_preview_rgb_dim = v;
        }
        if (httpd_query_key_value(query, "w", val, sizeof(val)) == ESP_OK) {
            float v = strtof(val, NULL);
            if (v < 0.0f) v = 0.0f;
            if (v > 1.0f) v = 1.0f;
            s_preview_w_dim = v;
        }
        if (httpd_query_key_value(query, "speed", val, sizeof(val)) == ESP_OK) {
            float v = strtof(val, NULL);
            if (v < 0.05f) v = 0.05f;
            if (v > 20.0f) v = 20.0f;
            s_preview_speed = v;
        }
    }
    httpd_resp_send(req, "ok", 2);
    return ESP_OK;
}

// ──────────────────────────────────────────────────────────
// HTTP — server bring-up
// ──────────────────────────────────────────────────────────

static void register_debug_routes(httpd_handle_t server)
{
    httpd_uri_t routes[] = {
        { .uri="/",                 .method=HTTP_GET,  .handler=handle_landing      },
        { .uri="/whoami",           .method=HTTP_GET,  .handler=handle_name_get     },
        { .uri="/debug",            .method=HTTP_GET,  .handler=handle_debug_root   },
        { .uri="/debug/login",      .method=HTTP_GET,  .handler=handle_login_get    },
        { .uri="/debug/login",      .method=HTTP_POST, .handler=handle_login_post   },
        { .uri="/debug/logout",     .method=HTTP_GET,  .handler=handle_logout       },
        { .uri="/debug/status",     .method=HTTP_GET,  .handler=handle_status       },
        { .uri="/debug/set",        .method=HTTP_GET,  .handler=handle_set          },
        { .uri="/debug/params",     .method=HTTP_GET,  .handler=handle_params       },
        { .uri="/debug/params/save",.method=HTTP_GET,  .handler=handle_params_save  },
        { .uri="/debug/preview",    .method=HTTP_GET,  .handler=handle_preview      },
        { .uri="/debug/reset_wifi", .method=HTTP_GET,  .handler=handle_reset_wifi   },
        { .uri="/debug/name",       .method=HTTP_GET,  .handler=handle_name_set     },
        { .uri="/debug/ota_check",  .method=HTTP_GET,  .handler=handle_ota_check    },
    };
    for (size_t i = 0; i < sizeof(routes) / sizeof(routes[0]); i++)
        httpd_register_uri_handler(server, &routes[i]);
}

static void start_webserver_sta(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.max_uri_handlers = 20;
    // Recycle oldest socket when the table fills, so a slider flood can't
    // wedge the server against the connection limit.
    config.lru_purge_enable = true;
    httpd_handle_t server = NULL;
    if (httpd_start(&server, &config) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start STA web server");
        return;
    }
    register_debug_routes(server);
    ESP_LOGI(TAG, "STA web server up.");
}

static void start_webserver_ap(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.max_uri_handlers = 20;
    // Recycle oldest socket when the table fills, so a slider flood can't
    // wedge the server against the connection limit.
    config.lru_purge_enable = true;
    httpd_handle_t server = NULL;
    if (httpd_start(&server, &config) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start AP web server");
        return;
    }
    httpd_uri_t ap_routes[] = {
        { .uri="/",                 .method=HTTP_GET,  .handler=handle_setup_root       },
        { .uri="/whoami",           .method=HTTP_GET,  .handler=handle_name_get         },
        { .uri="/scan",             .method=HTTP_GET,  .handler=handle_scan             },
        { .uri="/setup",            .method=HTTP_POST, .handler=handle_setup_post       },
        { .uri="/setup_standalone", .method=HTTP_POST, .handler=handle_setup_standalone },
    };
    for (size_t i = 0; i < sizeof(ap_routes) / sizeof(ap_routes[0]); i++)
        httpd_register_uri_handler(server, &ap_routes[i]);
    // /debug routes are also available from the AP for on-bench tuning.
    register_debug_routes(server);
    ESP_LOGI(TAG, "AP web server up.");
}

// ──────────────────────────────────────────────────────────
// app_main
// ──────────────────────────────────────────────────────────

void app_main(void)
{
    ESP_LOGI(TAG, "Phase firmware version: %s", FW_VERSION);

    led_mutex = xSemaphoreCreateMutex();
    s_events  = xEventGroupCreate();
    make_session_token();

    // NVS first so we can read params and (maybe) Wi-Fi creds.
    esp_err_t nvs_err = nvs_flash_init();
    if (nvs_err == ESP_ERR_NVS_NO_FREE_PAGES || nvs_err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }
    nvs_load_params();
    nvs_load_friendly_name();
    compute_mac_suffix();
    {
        char hostname[48];
        device_hostname(hostname, sizeof(hostname));
        ESP_LOGI(TAG, "Device name: %s  (MAC suffix: %s%s%s)",
                 hostname, s_mac_suffix,
                 s_friendly_name[0] ? ", friendly=" : "",
                 s_friendly_name[0] ? s_friendly_name : "");
    }

    // LED strip — SK6812 RGBW, GRBW byte order on the wire.
    led_strip_config_t strip_config = {
        .strip_gpio_num         = LED_GPIO,
        .max_leds               = LED_COUNT,
        .led_model              = LED_MODEL_SK6812,
        .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRBW,
        .flags                  = { .invert_out = false },
    };
    led_strip_rmt_config_t rmt_config = {
        .resolution_hz     = LED_RMT_RES_HZ,
        // Doubled RMT FIFO — see prototype-02.1 commit a13ae3e for context.
        .mem_block_symbols = 128,
    };
    if (led_strip_new_rmt_device(&strip_config, &rmt_config, &s_strip) != ESP_OK) {
        ESP_LOGE(TAG, "led_strip_new_rmt_device failed");
    }
    led_strip_clear(s_strip);
    init_geometry();
    init_glimmer();

    // Render + button tasks come up first so the LEDs respond from boot.
    xTaskCreate(render_task, "render", 4096, NULL,
                configMAX_PRIORITIES - 3, NULL);
    xTaskCreate(button_task, "button", 2048, NULL,
                tskIDLE_PRIORITY + 2, NULL);

    // Decide AP vs STA based on saved creds.
    char ssid[64] = {0}, pass[64] = {0};
    bool have_creds = load_wifi_creds(ssid, sizeof(ssid), pass, sizeof(pass));

    if (!have_creds) {
        ESP_LOGI(TAG, "No saved Wi-Fi creds — entering AP provisioning mode.");
        s_is_ap_mode = true;   // → mDNS hostname is just "phase.local" here
        s_anim_mode = ANIM_BOOT;
        wifi_init_ap();
        mdns_setup();
        start_webserver_ap();
        // Two exits:
        //   /setup       → save creds + esp_restart() (this thread never returns)
        //   /setup_standalone → set system clock + signal EV_STANDALONE_REQUESTED
        // The standalone path drops into ANIM_MOON right here, leaving the AP
        // running so /debug stays reachable.
        // Device is functional (AP + webserver up) — if this is a fresh
        // OTA image, it survived far enough to count as healthy.
        ota_mark_boot_valid();
        xEventGroupWaitBits(s_events, EV_STANDALONE_REQUESTED,
                            true, true, portMAX_DELAY);
        ESP_LOGI(TAG, "Standalone mode active — moon now rendering against user-supplied date.");
        s_anim_mode = ANIM_MOON;
        return;
    }

    ESP_LOGI(TAG, "Have creds — STA mode, will connect to \"%s\".", ssid);
    s_anim_mode = ANIM_BOOT;        // boot animation while we negotiate
    wifi_init_sta_with_creds(ssid, pass);

    // Cap the connect attempt. If the saved network has vanished (moved,
    // SSID changed, router dead) we'd otherwise sit in the blue boot
    // animation forever. After WIFI_CONNECT_TIMEOUT_MS, blink red, wipe
    // creds, and reboot into AP mode for re-provisioning.
    EventBits_t bits = xEventGroupWaitBits(s_events, EV_GOT_IP, false, true,
                                            pdMS_TO_TICKS(WIFI_CONNECT_TIMEOUT_MS));
    if (!(bits & EV_GOT_IP)) {
        ESP_LOGW(TAG, "Wi-Fi connect timed out after %d s — flashing red, wiping creds, rebooting to AP",
                 WIFI_CONNECT_TIMEOUT_MS / 1000);
        s_anim_mode = ANIM_CONNECT_FAILED;
        xEventGroupWaitBits(s_events, EV_FAILED_DONE, true, true, portMAX_DELAY);
        clear_wifi_creds();
        vTaskDelay(pdMS_TO_TICKS(200));
        esp_restart();
        return;
    }

    // Connected — play 3 blue pulses, then drop into normal moon mode.
    s_anim_mode = ANIM_CONNECTING;
    xEventGroupWaitBits(s_events, EV_CONNECTING_DONE, true, true, portMAX_DELAY);

    mdns_setup();
    start_webserver_sta();
    time_sync();

    // Healthy steady state reached — confirm a fresh OTA image so the
    // bootloader stops considering a rollback, then start the silent
    // auto-update cycle. Stack is generous because TLS runs in-task.
    ota_mark_boot_valid();
    xTaskCreate(ota_task, "ota", 9216, NULL, tskIDLE_PRIORITY + 1, NULL);

    s_anim_mode = ANIM_MOON;
    float phase = calc_moon_phase();
    ESP_LOGI(TAG, "Moon phase: %.3f  %s  %.1f%%",
             phase, phase_name(phase),
             phase_to_illumination(phase) * 100.0f);
}
