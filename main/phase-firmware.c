// ════════════════════════════════════════════════════════════════════════
//  Phase firmware — prototype-03.0 ("edition00")
//
//  Hardware:
//    SK6812 RGBW × 138 on GPIO 21, reset button on GPIO 20.
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
//  Reset button: hold GPIO 20 (active-low, internal pull-up) for 3 s to
//    erase Wi-Fi credentials and reboot to AP mode.
//
//  Note on UART: GPIO 21/20 are the default UART0 TX/RX pins. The RMT
//    peripheral takes over GPIO 21 so it can drive the LED data line; UART
//    output on that pin goes nowhere. Logs still flow through the secondary
//    USB-Serial/JTAG console (GPIO 18/19).
// ════════════════════════════════════════════════════════════════════════

#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
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

// ── Hardware ───────────────────────────────────────────────
#define LED_GPIO         21
#define BUTTON_GPIO      20
#define LED_COUNT        138
#define LED_RMT_RES_HZ   10000000

// ── Identity ───────────────────────────────────────────────
#define FW_VERSION       "prototype-03.0"
#define AP_SSID          "phase"
#define MDNS_HOSTNAME    "phase"
#define MDNS_INSTANCE    "Lunar Objects — Phase"

// ── Reset button ───────────────────────────────────────────
#define BUTTON_HOLD_MS   3000

// ── Debug portal credentials ───────────────────────────────
#define DEBUG_USER       "REDACTED_DEBUG_USER"
#define DEBUG_PASS       "REDACTED_DEBUG_PASS"

// ── Rendering defaults ─────────────────────────────────────
#define GRADIENT_WIDTH         0.28f
#define DEFAULT_FACE_GRADIENT  0.45f
#define DEFAULT_BRIGHTNESS     1.0f
#define DEFAULT_FLOOR          0.15f
#define DEFAULT_GLIMMER_ON     false
#define DEFAULT_GLIMMER_BASE   0.10f
#define DEFAULT_GLIMMER_EDGE   0.30f
#define DEFAULT_GLIMMER_SPEED  0.30f
#define DEFAULT_CURVE_Q1       0.259f
#define DEFAULT_CURVE_G1       0.501f
#define DEFAULT_CURVE_G3       0.500f
#define DEFAULT_CURVE_Q3       0.255f

// ── Animation knobs ────────────────────────────────────────
#define BOOT_ANIM_DIM          0.50f   // boot blue brightness cap
#define BOOT_ANIM_PHASE_RATE   0.04f   // phase increments per render step (~25 s/cycle at 50 ms)
#define CONNECTING_PULSE_DIM   0.65f   // peak brightness of the connecting pulse
#define CONNECTING_PULSE_COUNT 3

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
    ANIM_BOOT       = 0,   // slow blue moon-phase cycle (AP/provisioning)
    ANIM_CONNECTING = 1,   // 3 blue pulses while connecting
    ANIM_MOON       = 2,   // normal moon render
} anim_mode_t;

static volatile anim_mode_t s_anim_mode = ANIM_BOOT;

static EventGroupHandle_t s_events;
#define EV_GOT_IP            BIT0
#define EV_CONNECTING_DONE   BIT1

// Mutex serialising led_strip_refresh() against any flash-touching code
// (NVS commit, scan, restart prep). Flash ops disable the CPU cache, which
// can stall the RMT refill ISR and corrupt the SK6812 frame mid-transmit.
static SemaphoreHandle_t led_mutex;

// Strip handle, shared between render task and the (rare) blocking flushes
// in main/handlers.
static led_strip_handle_t s_strip;

// Random session token for /debug cookies. 32 hex chars + null.
static char s_session_token[33];

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

// ── Manual override ────────────────────────────────────────
static bool  manual_mode  = false;
static float manual_phase = 0.0f;

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
static float led_angle(int i)
{
    return fmodf((float)i * (360.0f / (float)LED_COUNT), 360.0f);
}

static float   glimmer_offset[LED_COUNT];
static float   glimmer_rate[LED_COUNT];
static float   frame_b[LED_COUNT];       // brightness per pixel, 0..1
static uint8_t last_out[LED_COUNT][4];   // last R,G,B,W actually sent
static bool    last_out_valid = false;

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
static void compute_moon_frame(float phase, float time_f)
{
    float lit_arc = apply_curve(phase);

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

    for (int i = 0; i < LED_COUNT; i++) {
        float angle = led_angle(i);
        float rel   = angle - lit_center;
        while (rel >  180.0f) rel -= 360.0f;
        while (rel < -180.0f) rel += 360.0f;

        float half_arc       = lit_arc / 2.0f;
        float dist           = fabsf(rel) - half_arc;
        float brightness     = 0.0f;
        float edge_proximity = 0.0f;

        if (dist < -gradient_deg) {
            float depth = 0.0f;
            if (half_arc > 0.0f) depth = (-dist) / half_arc;
            if (depth > 1.0f) depth = 1.0f;
            brightness = 1.0f - depth * p_face_gradient;
        } else if (dist > gradient_deg) {
            brightness = 0.0f;
        } else {
            float t        = dist / gradient_deg;
            brightness     = 0.5f - 0.5f * sinf(t * ((float)M_PI / 2.0f));
            edge_proximity = 1.0f - fabsf(t);
        }

        if (p_glimmer_on && brightness > 0.01f)
            brightness *= glimmer_value(i, time_f, edge_proximity);

        brightness *= p_brightness;
        if (brightness < p_floor) brightness = 0.0f;

        if (brightness < 0.0f) brightness = 0.0f;
        if (brightness > 1.0f) brightness = 1.0f;
        frame_b[i] = brightness;
    }
}

// Drive the W channel from frame_b[] (R=G=B=0).
static void output_white(void)
{
    uint8_t out[LED_COUNT][4];
    for (int i = 0; i < LED_COUNT; i++) {
        out[i][0] = 0;
        out[i][1] = 0;
        out[i][2] = 0;
        out[i][3] = (uint8_t)(frame_b[i] * 255.0f);
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
    uint8_t out[LED_COUNT][4];
    for (int i = 0; i < LED_COUNT; i++) {
        out[i][0] = 0;
        out[i][1] = 0;
        out[i][2] = (uint8_t)(frame_b[i] * 255.0f);
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
    float time_f      = 0.0f;
    float boot_phase  = 0.0f;
    float pulse_t     = 0.0f;
    int   pulse_count = 0;
    anim_mode_t prev_mode = (anim_mode_t)-1;

    while (1) {
        anim_mode_t mode = s_anim_mode;
        if (mode != prev_mode) {
            // Force a redraw on mode change since the frame buffer hasn't
            // necessarily changed.
            last_out_valid = false;
            pulse_t = 0.0f;
            pulse_count = 0;
            prev_mode = mode;
        }

        switch (mode) {
        case ANIM_BOOT: {
            compute_moon_frame(boot_phase, time_f);
            for (int i = 0; i < LED_COUNT; i++) frame_b[i] *= BOOT_ANIM_DIM;
            output_blue();
            boot_phase = fmodf(boot_phase + BOOT_ANIM_PHASE_RATE * 0.05f, 1.0f);
            break;
        }
        case ANIM_CONNECTING: {
            // 1 Hz pulse, peak at CONNECTING_PULSE_DIM, stop after N pulses.
            float bright = CONNECTING_PULSE_DIM *
                           (0.5f - 0.5f * cosf(pulse_t * 2.0f * (float)M_PI));
            for (int i = 0; i < LED_COUNT; i++) frame_b[i] = bright;
            output_blue();
            pulse_t += 0.05f;
            if (pulse_t >= 1.0f) {
                pulse_t = 0.0f;
                pulse_count++;
                if (pulse_count >= CONNECTING_PULSE_COUNT) {
                    xEventGroupSetBits(s_events, EV_CONNECTING_DONE);
                }
            }
            break;
        }
        case ANIM_MOON:
        default: {
            float phase = manual_mode ? manual_phase : calc_moon_phase();
            compute_moon_frame(phase, time_f);
            output_white();
            break;
        }
        }

        time_f += (p_glimmer_speed * 0.001f) * 50.0f;
        vTaskDelay(pdMS_TO_TICKS(50));
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

    int held_ms = 0;
    while (1) {
        if (gpio_get_level(BUTTON_GPIO) == 0) {  // active low
            held_ms += 50;
            if (held_ms == 1000 || held_ms == 2000) {
                ESP_LOGI(TAG, "Reset button held %dms…", held_ms);
            }
            if (held_ms >= BUTTON_HOLD_MS) {
                ESP_LOGW(TAG, "Reset button held %dms — clearing Wi-Fi creds and rebooting",
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

static void mdns_setup(void)
{
    esp_err_t err = mdns_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "mdns_init failed: %s", esp_err_to_name(err));
        return;
    }
    mdns_hostname_set(MDNS_HOSTNAME);
    mdns_instance_name_set(MDNS_INSTANCE);
    mdns_service_add(NULL, "_http", "_tcp", 80, NULL, 0);
    ESP_LOGI(TAG, "mDNS up — http://%s.local/", MDNS_HOSTNAME);
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

    wifi_config_t ap_cfg = {
        .ap = {
            .ssid           = AP_SSID,
            .ssid_len       = (uint8_t)strlen(AP_SSID),
            .channel        = 6,
            .authmode       = WIFI_AUTH_OPEN,
            .max_connection = 4,
        },
    };

    esp_wifi_set_mode(WIFI_MODE_APSTA);
    esp_wifi_set_config(WIFI_IF_AP, &ap_cfg);
    esp_wifi_start();
    esp_wifi_set_ps(WIFI_PS_NONE);
    ESP_LOGI(TAG, "AP mode — SSID \"%s\" (open) — http://%s.local/", AP_SSID, MDNS_HOSTNAME);
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
"<h1>Lunar Objects &mdash; Phase Setup</h1>"
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
"<div class='status'>firmware " FW_VERSION "</div>"
"</div>"
"<script>"
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
"<h1>Lunar Objects &mdash; Debug</h1>"
"<div class='phase-display'>"
"<div class='phase-name'>Locked.</div>"
"<div class='phase-pct'>authenticate to continue</div>"
"</div>"
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
"<h1>Lunar Objects &mdash; Phase</h1>"
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
"<input type='range' id='s-gbase' min='0' max='500' oninput='onParam()'/>"
"<div class='note'>Subtle brightness wobble across the lit face.</div>"
"<label>Edge shimmer<span id='lbl-gedge'></span></label>"
"<input type='range' id='s-gedge' min='0' max='1000' oninput='onParam()'/>"
"<div class='note'>Extra wobble near the terminator. May reintroduce flicker if data line is noisy.</div>"
"<label>Speed<span id='lbl-gspeed'></span></label>"
"<input type='range' id='s-gspeed' min='50' max='2000' oninput='onParam()'/>"
"<div class='note'>How fast the shimmer oscillates.</div>"
"</div>"
"<button class='save-btn' onclick='saveParams()'>Save to Device</button>"
"<div class='saved-msg' id='saved-msg'>Saved.</div>"
"<div class='status' id='status'>—</div>"
"<div class='status'>firmware " FW_VERSION "</div>"
"</div>"
"<script>"
"var isManual=false;"
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
"function sendPhase(p){"
"var url=p<0?'/debug/set?mode=real':'/debug/set?mode=manual&phase='+p.toFixed(4);"
"fetch(url).catch(function(){});}"
"function onParam(){"
"var q1=document.getElementById('s-q1').value/1000.0;"
"var g1=document.getElementById('s-g1').value/1000.0;"
"var g3=document.getElementById('s-g3').value/1000.0;"
"var q3=document.getElementById('s-q3').value/1000.0;"
"var bright=document.getElementById('s-bright').value/1000.0;"
"var grad=document.getElementById('s-grad').value/1000.0;"
"var floor=document.getElementById('s-floor').value/1000.0;"
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
"document.getElementById('lbl-gbase').textContent=gbase.toFixed(3);"
"document.getElementById('lbl-gedge').textContent=gedge.toFixed(3);"
"document.getElementById('lbl-gspeed').textContent=gspeed.toFixed(3);"
"fetch('/debug/params?q1='+q1.toFixed(3)+'&g1='+g1.toFixed(3)"
"+'&g3='+g3.toFixed(3)+'&q3='+q3.toFixed(3)"
"+'&bright='+bright.toFixed(3)+'&grad='+grad.toFixed(3)"
"+'&floor='+floor.toFixed(3)"
"+'&gbase='+gbase.toFixed(3)+'&gedge='+gedge.toFixed(3)"
"+'&gspeed='+gspeed.toFixed(3)).catch(function(){});}"
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
"document.getElementById('lbl-gbase').textContent=d.gbase.toFixed(3);"
"document.getElementById('lbl-gedge').textContent=d.gedge.toFixed(3);"
"document.getElementById('lbl-gspeed').textContent=d.gspeed.toFixed(3);"
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

    char buf[512];
    snprintf(buf, sizeof(buf),
        "{\"phase\":%.4f,\"illumination\":%.1f,\"name\":\"%s\","
        "\"manual\":%s,\"ip\":\"%s\","
        "\"q1\":%.3f,\"g1\":%.3f,\"g3\":%.3f,\"q3\":%.3f,"
        "\"bright\":%.3f,\"grad\":%.3f,\"floor\":%.3f,"
        "\"gon\":%s,\"gbase\":%.3f,\"gedge\":%.3f,\"gspeed\":%.3f}",
        phase, ill * 100.0f, phase_name(phase),
        manual_mode ? "true" : "false", ip_str,
        p_curve_q1, p_curve_g1, p_curve_g3, p_curve_q3,
        p_brightness, p_face_gradient, p_floor,
        p_glimmer_on ? "true" : "false",
        p_glimmer_base, p_glimmer_edge, p_glimmer_speed);

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

// ──────────────────────────────────────────────────────────
// HTTP — server bring-up
// ──────────────────────────────────────────────────────────

static void register_debug_routes(httpd_handle_t server)
{
    httpd_uri_t routes[] = {
        { .uri="/",                 .method=HTTP_GET,  .handler=handle_landing      },
        { .uri="/debug",            .method=HTTP_GET,  .handler=handle_debug_root   },
        { .uri="/debug/login",      .method=HTTP_GET,  .handler=handle_login_get    },
        { .uri="/debug/login",      .method=HTTP_POST, .handler=handle_login_post   },
        { .uri="/debug/logout",     .method=HTTP_GET,  .handler=handle_logout       },
        { .uri="/debug/status",     .method=HTTP_GET,  .handler=handle_status       },
        { .uri="/debug/set",        .method=HTTP_GET,  .handler=handle_set          },
        { .uri="/debug/params",     .method=HTTP_GET,  .handler=handle_params       },
        { .uri="/debug/params/save",.method=HTTP_GET,  .handler=handle_params_save  },
    };
    for (size_t i = 0; i < sizeof(routes) / sizeof(routes[0]); i++)
        httpd_register_uri_handler(server, &routes[i]);
}

static void start_webserver_sta(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.max_uri_handlers = 16;
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
    config.max_uri_handlers = 16;
    httpd_handle_t server = NULL;
    if (httpd_start(&server, &config) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start AP web server");
        return;
    }
    httpd_uri_t ap_routes[] = {
        { .uri="/",      .method=HTTP_GET,  .handler=handle_setup_root },
        { .uri="/scan",  .method=HTTP_GET,  .handler=handle_scan       },
        { .uri="/setup", .method=HTTP_POST, .handler=handle_setup_post },
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
        s_anim_mode = ANIM_BOOT;
        wifi_init_ap();
        mdns_setup();
        start_webserver_ap();
        // Park here forever; the /setup POST handler triggers esp_restart()
        // once the user has saved credentials.
        vTaskDelay(portMAX_DELAY);
        return;
    }

    ESP_LOGI(TAG, "Have creds — STA mode, will connect to \"%s\".", ssid);
    s_anim_mode = ANIM_BOOT;        // boot animation while we negotiate
    wifi_init_sta_with_creds(ssid, pass);
    xEventGroupWaitBits(s_events, EV_GOT_IP, false, true, portMAX_DELAY);

    // Connected — play 3 blue pulses, then drop into normal moon mode.
    s_anim_mode = ANIM_CONNECTING;
    xEventGroupWaitBits(s_events, EV_CONNECTING_DONE, true, true, portMAX_DELAY);

    mdns_setup();
    start_webserver_sta();
    time_sync();

    s_anim_mode = ANIM_MOON;
    float phase = calc_moon_phase();
    ESP_LOGI(TAG, "Moon phase: %.3f  %s  %.1f%%",
             phase, phase_name(phase),
             phase_to_illumination(phase) * 100.0f);
}
