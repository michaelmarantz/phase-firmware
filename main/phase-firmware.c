#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_sntp.h"
#include "esp_http_server.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "led_strip.h"

// ── Your Wi-Fi credentials ─────────────────────────────────
#define WIFI_SSID      "REDACTED_WIFI_SSID"
#define WIFI_PASSWORD  "REDACTED_WIFI_PASSWORD"

// ── Hardware config ────────────────────────────────────────
#define LED_GPIO        2
#define LED_COUNT       52
#define LED_RMT_RES_HZ  10000000

// ── LED color (warm white) ─────────────────────────────────
#define WARM_R  255
#define WARM_G  120
#define WARM_B  40

// ── Firmware version ───────────────────────────────────────
#define FW_VERSION  "prototype-02.1"

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

// ── Internal ───────────────────────────────────────────────
#define TAG "phase"
static EventGroupHandle_t wifi_events;
#define WIFI_CONNECTED_BIT BIT0

// Serialises led_strip_refresh() against any flash-touching code (NVS commit).
// Flash ops disable the CPU cache, which can stall the RMT refill ISR and
// corrupt the WS2812 frame mid-transmit.
static SemaphoreHandle_t led_mutex;

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
// NVS
// ──────────────────────────────────────────────────────────

static void nvs_save_params(void)
{
    // Hold the LED mutex across the entire flash transaction. NVS commit
    // disables cache, which would otherwise stall the RMT ISR mid-frame.
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
    ESP_LOGI(TAG, "Saved. q1=%.3f g1=%.3f g3=%.3f q3=%.3f",
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
    ESP_LOGI(TAG, "Loaded. q1=%.3f g1=%.3f g3=%.3f q3=%.3f",
             p_curve_q1, p_curve_g1, p_curve_g3, p_curve_q3);
}

// ──────────────────────────────────────────────────────────
// Wi-Fi
// ──────────────────────────────────────────────────────────

static void wifi_event_handler(void *arg, esp_event_base_t base,
                                int32_t id, void *data)
{
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START)
        esp_wifi_connect();
    else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED)
        esp_wifi_connect();
    else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        xEventGroupSetBits(wifi_events, WIFI_CONNECTED_BIT);
        esp_netif_ip_info_t ip_info;
        esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
        esp_netif_get_ip_info(netif, &ip_info);
        ESP_LOGI(TAG, "Open http://" IPSTR " in your browser", IP2STR(&ip_info.ip));
    }
}

static void wifi_init(void)
{
    wifi_events = xEventGroupCreate();
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);
    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event_handler, NULL);
    esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_event_handler, NULL);
    wifi_config_t wifi_cfg = {
        .sta = { .ssid = WIFI_SSID, .password = WIFI_PASSWORD },
    };
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(WIFI_IF_STA, &wifi_cfg);
    esp_wifi_start();
    // Disable Wi-Fi modem sleep — on the single-core C3 it lets Wi-Fi ISRs
    // preempt the RMT refill ISR long enough to corrupt WS2812 frames.
    esp_wifi_set_ps(WIFI_PS_NONE);
    ESP_LOGI(TAG, "Connecting to Wi-Fi...");
    xEventGroupWaitBits(wifi_events, WIFI_CONNECTED_BIT, false, true, portMAX_DELAY);
}

// ──────────────────────────────────────────────────────────
// Time sync
// ──────────────────────────────────────────────────────────

static void time_sync(void)
{
    esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, "pool.ntp.org");
    esp_sntp_init();
    ESP_LOGI(TAG, "Waiting for time sync...");
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

// Custom illumination curve with 4 control points
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

static float led_angle(int i)
{
    return fmodf((i + 1) * (360.0f / 52.0f), 360.0f);
}

static float   glimmer_offset[LED_COUNT];
static float   glimmer_rate[LED_COUNT];
static uint8_t last_frame[LED_COUNT][3];
static bool    last_frame_valid = false;

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

static void render_moon(led_strip_handle_t strip, float phase, float time_f)
{
    float lit_arc = apply_curve(phase);

    // Smooth the lit_center flip around full moon
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

    uint8_t new_frame[LED_COUNT][3];

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

        new_frame[i][0] = (uint8_t)(WARM_R * brightness);
        new_frame[i][1] = (uint8_t)(WARM_G * brightness);
        new_frame[i][2] = (uint8_t)(WARM_B * brightness);
    }

    // Skip refresh if frame is identical to last — fewer transmissions,
    // fewer chances for the data line to latch a corrupted byte.
    bool changed = !last_frame_valid;
    if (last_frame_valid) {
        if (memcmp(new_frame, last_frame, sizeof(new_frame)) != 0) {
            changed = true;
        }
    }

    if (changed) {
        xSemaphoreTake(led_mutex, portMAX_DELAY);
        for (int i = 0; i < LED_COUNT; i++) {
            led_strip_set_pixel(strip, i,
                new_frame[i][0], new_frame[i][1], new_frame[i][2]);
        }
        led_strip_refresh(strip);
        xSemaphoreGive(led_mutex);
        memcpy(last_frame, new_frame, sizeof(new_frame));
        last_frame_valid = true;
    }
}

// ──────────────────────────────────────────────────────────
// Web portal
// ──────────────────────────────────────────────────────────

static const char *HTML_PAGE =
"<!DOCTYPE html><html><head>"
"<meta name='viewport' content='width=device-width, initial-scale=1'>"
"<title>Phase</title>"
"<style>"
"*{box-sizing:border-box;margin:0;padding:0}"
"body{background:#0a0a0a;color:#e8e0d0;font-family:'Courier New',monospace;"
"display:flex;justify-content:center;min-height:100vh;padding:20px}"
".card{background:#111;border:1px solid #2a2a2a;border-radius:4px;"
"padding:32px;width:100%;max-width:420px;align-self:flex-start;margin-top:40px}"
"h1{font-size:11px;letter-spacing:.35em;text-transform:uppercase;color:#555;margin-bottom:32px}"
".phase-display{text-align:center;margin-bottom:36px}"
".phase-name{font-size:24px;color:#e8e0d0;margin-bottom:8px;font-weight:normal}"
".phase-pct{font-size:12px;color:#555;letter-spacing:.2em}"
".mode-row{display:flex;gap:10px;margin-bottom:32px}"
".mode-btn{flex:1;padding:11px;border:1px solid #333;background:transparent;"
"color:#555;font-family:inherit;font-size:11px;letter-spacing:.2em;"
"text-transform:uppercase;cursor:pointer;border-radius:2px;transition:all .2s}"
".mode-btn.active{border-color:#e8e0d0;color:#e8e0d0}"
".controls{transition:opacity .3s}"
".controls.dim{opacity:.25;pointer-events:none}"
"label{display:flex;justify-content:space-between;font-size:10px;letter-spacing:.2em;"
"color:#555;text-transform:uppercase;margin-bottom:8px}"
"label span{color:#e8e0d0;letter-spacing:0}"
"input[type=range]{width:100%;accent-color:#e8e0d0;cursor:pointer;margin-bottom:20px}"
"input[type=date]{width:100%;background:#1a1a1a;border:1px solid #2a2a2a;"
"color:#e8e0d0;font-family:inherit;font-size:13px;padding:11px;"
"border-radius:2px;margin-bottom:20px;color-scheme:dark}"
".slider-labels{display:flex;justify-content:space-between;"
"font-size:10px;color:#444;margin-top:-16px;margin-bottom:20px}"
"hr{border:none;border-top:1px solid #1e1e1e;margin:24px 0}"
".section-title{font-size:10px;letter-spacing:.3em;text-transform:uppercase;"
"color:#444;margin-bottom:20px}"
".note{font-size:10px;color:#444;margin-top:-14px;margin-bottom:20px;line-height:1.6}"
".save-btn{width:100%;padding:13px;border:1px solid #e8e0d0;background:transparent;"
"color:#e8e0d0;font-family:inherit;font-size:11px;letter-spacing:.25em;"
"text-transform:uppercase;cursor:pointer;border-radius:2px;margin-top:4px;transition:all .2s}"
".save-btn:hover{background:#e8e0d0;color:#0a0a0a}"
".status{font-size:10px;color:#333;text-align:center;letter-spacing:.1em;margin-top:16px}"
".saved-msg{color:#888;font-size:10px;text-align:center;margin-top:12px;"
"letter-spacing:.15em;opacity:0;transition:opacity .5s}"
"</style></head><body><div class='card'>"
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
"var url=p<0?'/set?mode=real':'/set?mode=manual&phase='+p.toFixed(4);"
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
"fetch('/params?q1='+q1.toFixed(3)+'&g1='+g1.toFixed(3)"
"+'&g3='+g3.toFixed(3)+'&q3='+q3.toFixed(3)"
"+'&bright='+bright.toFixed(3)+'&grad='+grad.toFixed(3)"
"+'&floor='+floor.toFixed(3)"
"+'&gbase='+gbase.toFixed(3)+'&gedge='+gedge.toFixed(3)"
"+'&gspeed='+gspeed.toFixed(3)).catch(function(){});}"
"function setGlimmer(on){"
"document.getElementById('btn-gon-off').classList.toggle('active',!on);"
"document.getElementById('btn-gon-on').classList.toggle('active',on);"
"fetch('/params?gon='+(on?'1':'0')).catch(function(){});}"
"function saveParams(){"
"fetch('/params/save').then(function(){"
"var m=document.getElementById('saved-msg');"
"m.style.opacity=1;"
"setTimeout(function(){m.style.opacity=0;},2000);"
"}).catch(function(){});}"
"function poll(){"
"fetch('/status').then(function(r){return r.json();})"
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

static esp_err_t handle_root(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, HTML_PAGE, strlen(HTML_PAGE));
    return ESP_OK;
}

static esp_err_t handle_status(httpd_req_t *req)
{
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
    nvs_save_params();
    httpd_resp_send(req, "ok", 2);
    return ESP_OK;
}

static void start_webserver(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    httpd_handle_t server = NULL;
    if (httpd_start(&server, &config) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start web server"); return;
    }
    httpd_uri_t routes[] = {
        { .uri="/",            .method=HTTP_GET, .handler=handle_root        },
        { .uri="/status",      .method=HTTP_GET, .handler=handle_status      },
        { .uri="/set",         .method=HTTP_GET, .handler=handle_set         },
        { .uri="/params",      .method=HTTP_GET, .handler=handle_params      },
        { .uri="/params/save", .method=HTTP_GET, .handler=handle_params_save },
    };
    for (int i = 0; i < 5; i++)
        httpd_register_uri_handler(server, &routes[i]);
    ESP_LOGI(TAG, "Web server started.");
}

// ──────────────────────────────────────────────────────────
// Main
// ──────────────────────────────────────────────────────────

static void render_task(void *arg)
{
    led_strip_handle_t strip = (led_strip_handle_t)arg;
    float time_f = 0.0f;
    while (1) {
        float current_phase = manual_mode ? manual_phase : calc_moon_phase();
        render_moon(strip, current_phase, time_f);
        time_f += (p_glimmer_speed * 0.001f) * 50.0f;
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "Phase firmware version: %s", FW_VERSION);

    led_mutex = xSemaphoreCreateMutex();

    nvs_flash_init();
    nvs_load_params();

    led_strip_config_t strip_config = {
        .strip_gpio_num = LED_GPIO,
        .max_leds       = LED_COUNT,
    };
    led_strip_rmt_config_t rmt_config = {
        .resolution_hz     = LED_RMT_RES_HZ,
        // Larger RMT FIFO = more slack before a delayed refill ISR corrupts the
        // frame. Default 64 is marginal on C3 with Wi-Fi running; 128 fits in
        // one channel's symbol memory.
        .mem_block_symbols = 128,
    };
    led_strip_handle_t strip;
    led_strip_new_rmt_device(&strip_config, &rmt_config, &strip);

    wifi_init();
    time_sync();
    start_webserver();
    init_glimmer();

    float phase = calc_moon_phase();
    ESP_LOGI(TAG, "Moon phase: %.3f  %s  %.1f%%",
             phase, phase_name(phase),
             phase_to_illumination(phase) * 100.0f);

    // Dedicated, high-priority task so the HTTP server and Wi-Fi housekeeping
    // can't starve the render loop.
    xTaskCreate(render_task, "render", 4096, strip,
                configMAX_PRIORITIES - 3, NULL);
}

