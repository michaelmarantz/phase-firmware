# phase-firmware

ESP-IDF firmware for the Phase lamp — an ESP32-C3 driving a ring of WS2812 LEDs that renders the current moon phase, with a web UI for live tuning.

## Hardware

- **MCU:** ESP32-C3 (RISC-V, 2 MB flash)
- **LEDs:** 90× WS2812 on GPIO 2, driven via RMT at 10 MHz (prototype-02.x used 52)
- **Color:** warm white (R=255, G=120, B=40), brightness/floor/glimmer modulated per pixel

## Project conventions

- **CMake project name** is `phase-prototype` — stays constant across hardware revisions.
- **Branch name** identifies the hardware revision (`prototype-02`, `prototype-02.1`, …). `FW_VERSION` in `main/phase-firmware.c` should match the branch.
- Build output: `build/phase-prototype.bin`.

## Build & flash

ESP-IDF lives at `~/esp/esp-idf`. Source it once per shell:

```bash
source ~/esp/esp-idf/export.sh
idf.py build
idf.py -p /dev/cu.usbmodem101 flash monitor
```

`idf.py fullclean` after switching branches or changing the cmake project name.

## Code layout

Single-file app: `main/phase-firmware.c` (~715 lines). Sections inside:

- **NVS** — persists render params under namespace `phase`.
- **Wi-Fi / SNTP** — STA mode, hardcoded creds (`WIFI_SSID` / `WIFI_PASSWORD`), NTP pool for moon-phase time base.
- **Moon math** — `calc_moon_phase()` returns 0..1; `apply_curve()` shapes it via 4 control points (`q1, g1, g3, q3`) stored in NVS.
- **Rendering** — `render_moon()` runs every 50 ms in `app_main`'s loop. Computes illumination per LED with a gradient edge, optional glimmer, and a brightness floor. Frame state cached in `last_frame[]`.
- **Web server** — HTTP on port 80. Routes:
  - `GET /` — single-page HTML UI (the big `HTML_PAGE` string literal).
  - `GET /status` — JSON: phase, name, illumination, params.
  - `GET /set?phase=…` — manual phase override (also `manual=0` to release).
  - `GET /params?...` — live-update render params (non-persistent).
  - `GET /params/save` — commit current params to NVS.

## Dependencies

- `espressif/led_strip` (managed component, see `main/idf_component.yml`)
- Plus core IDF: `esp_wifi`, `esp_http_server`, `esp_sntp`, `nvs_flash`.

## Notes / gotchas

- Wi-Fi credentials are hardcoded in source — do not commit real creds for shipping units.
- `glimmer_offset[]` / `glimmer_rate[]` are seeded once in `init_glimmer()` — reseed if you change `LED_COUNT`.
- App partition is 1 MB; current binary is ~87% full. Watch size when adding features.

## WS2812 glitch fix (resolved, prototype-02.1)

Earlier prototype builds had random per-pixel flashes / wrong colors on the LED ring. This is the well-known **RMT-vs-Wi-Fi interrupt contention** failure mode on the single-core C3 — *not* a 3.3 V → 5 V level-shift problem. (Proof: WLED on the same hardware was clean.) The diode-drop-on-VDD trick is the wrong fix for this; it addresses VIH, not ISR timing.

What fixed it (all in software):

1. `esp_wifi_set_ps(WIFI_PS_NONE)` after `esp_wifi_start()` — disables Wi-Fi modem sleep so it doesn't preempt the RMT refill ISR.
2. `led_strip_rmt_config_t.mem_block_symbols = 128` — doubles the RMT FIFO so the refill ISR has more slack.
3. Dedicated render task at `configMAX_PRIORITIES - 3`, with a `led_mutex` guarding both `led_strip_refresh()` and `nvs_save_params()` so flash commits (which disable cache) can never overlap a frame in flight.

References: [WLED #4382](https://github.com/wled/WLED/issues/4382), [Espressif led_strip docs](https://components.espressif.com/components/espressif/led_strip/). If glitches ever return on a new revision, look here first — re-check `mem_block_symbols`, Wi-Fi PS state, and that nothing new is calling NVS/SPIFFS without taking `led_mutex`.

For a future hardware respin: add a 74AHCT125 buffer between GPIO 2 and DIN as belt-and-suspenders robustness (Adafruit-recommended). Skip the SB560-on-VDD trick.
