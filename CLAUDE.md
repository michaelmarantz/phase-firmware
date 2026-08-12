# phase-firmware

ESP-IDF firmware for the Phase lamp — an ESP32-C3 driving a ring of addressable LEDs that renders the current moon phase, with a web UI for live tuning. Branch `edition00` is a substantial rework: SK6812 RGBW strip, captive-portal Wi-Fi provisioning, reset button, mDNS, and an auth-gated debug portal. Branch `edition00.1` adds silent automatic OTA updates — see "OTA updates" below. Branch `edition00.2` moves to the 4 MB flash the chip actually has (partition slots resized), and restores the WPA3-SAE / IPv6 / WPA2-Enterprise support that had to be trimmed to fit the 2 MB OTA layout. Branch `edition00.3` adds per-device unique names — see "Device naming" below.

## Hardware (edition00)

- **MCU:** ESP32-C3 (RISC-V, single-core, 4 MB flash — the SPI chip was always 4 MB, we just told ESP-IDF "2 MB" through edition00.1 by mistake)
- **LEDs:** 138× **SK6812 RGBW** on **GPIO 8**, GRBW byte order, driven via RMT at 10 MHz
- LED 0 sits at **12 o'clock**; indices advance clockwise (`led_angle(i) = i * 360 / 138`)
- **Color:** W channel only for moon rendering (R=G=B=0); B channel used for boot AP + Wi-Fi-connecting animations
- **Reset button:** chip **GPIO 7** (silkscreen label **D5** on the phase board), active-low, internal pull-up. Hold 3 s to erase Wi-Fi creds and reboot. **Note:** the silkscreen D-numbers on this board are NOT direct GPIO numbers — confirmed by a runtime GPIO scanner. (LED data D8 happens to equal GPIO 8 by coincidence; D5 is GPIO 7.) Always verify any new wiring with the scanner before assuming the label matches.
- **UART note:** Serial logs flow over the secondary USB Serial/JTAG console (GPIO 18/19), so UART0 (GPIO 20/21) is never in the way at runtime — both default UART pins are free for whatever the board wires them to. The phase board originally targeted GPIO 21 for LED data but that trace is broken on v1, hence GPIO 8.
- **GPIO 8 caveat:** GPIO 8 is a strapping pin (selects whether bootloader logging is enabled). It works fine for RMT output at runtime, but the next board respin should move the data line to **GPIO 3/4/5/6/7/10** to avoid any strapping interaction at boot. (The v1 phase board originally wired data to GPIO 21, but that trace doesn't pass signal on this revision — confirmed by repeating with WLED. Moving to GPIO 8 made the strip light up.)

Previous revisions: `prototype-02.x` = 52× WS2812 on GPIO 2; `prototype-04` = 90× WS2812 on GPIO 2.

## Project conventions

- **CMake project name** is `phase-prototype` — stays constant across hardware revisions. Build artifact: `build/phase-prototype.bin`.
- **Branch name** identifies the hardware revision / edition (`prototype-02.1`, `prototype-04`, `edition00`, …). `FW_VERSION` in `main/phase-firmware.c` should match the branch.
- **`FW_VERSION` is the OTA trigger.** The top-level CMakeLists extracts it into `PROJECT_VER`, which lands in the app descriptor; deployed lamps compare it against the published binary's descriptor and update on any difference. Bump it for every release — publishing with an unchanged version is a fleet-wide no-op.

## Device naming (edition00.3+)

Every lamp gets a unique name so multiple units can coexist on one network.

- **Default (no config needed):** `phase-<mac6>` where `<mac6>` is the last 3 bytes of the STA MAC as 6 lowercase hex chars. E.g. `phase-a94290`. Deterministic per chip. Used as **both** the mDNS hostname AND the AP setup SSID.
- **Friendly override:** the `/debug` page has a "Device Name" section with a text field. Set e.g. `e00-2-3`; from then on the lamp advertises as `phase-e00-2-3.local` and its setup AP SSID becomes `phase-e00-2-3`. Stored in NVS key `friendly` (namespace `phase`). Sanitized on input: lowercase, digits and hyphens only, spaces/underscores/dots collapse to `-`, leading/trailing hyphens trimmed, capped at 24 chars.
- **Live update:** renaming updates the mDNS hostname immediately (via `mdns_hostname_set()` — no reboot). The AP SSID reflects the new name on the next AP-mode boot.
- **Clear the friendly name** (revert to MAC form) by submitting an empty name in the field.
- **Endpoints:** `GET /whoami` (open, both AP and STA — returns the current hostname as text) and `GET /debug/name?name=…` (auth-gated, sets/clears the friendly name).
- Every page's `<h1>` shows the active hostname in brackets, and the browser tab title becomes the hostname — handy when three lamps are open in adjacent tabs.
- Both `SETUP_PAGE` and `LOGIN_PAGE` fetch `/whoami` on load so you can always tell *which* lamp is asking for your Wi-Fi password / debug login.

## OTA updates (edition00.1+)

The fleet follows the **latest GitHub release** of this repo. Each STA-connected lamp checks `https://github.com/michaelmarantz/phase-firmware/releases/latest/download/phase.bin` ~30 s after boot and every 6 h, compares the image's app-descriptor version to its own, and on any difference downloads into the spare OTA slot and reboots. Fully silent — no user consent step anywhere.

- **Publish an update:** bump `FW_VERSION`, commit, run `./release.sh` (builds, size-checks against the OTA slot, tags, `gh release create` with `phase.bin`). Repo/releases must stay publicly downloadable.
- **Partition table** is custom (`partitions.csv`). On edition00.2 the layout is `nvs` + `phy_init` at their original offsets (creds/params survive the first serial reflash from any prior single-app build), then `otadata` + two **1.9375 MB** `ota_0`/`ota_1` slots fill the 4 MB chip. Roughly 2× the headroom of the edition00.1 layout — the sdkconfig no longer has to trim features to fit.
- **Rollback:** `CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE=y`. A fresh OTA image boots as `PENDING_VERIFY`; `ota_mark_boot_valid()` confirms it only after the device reaches a healthy steady state (webserver up). A crash-looping release rolls back to the previous slot on its own.
- **Glitch discipline:** every `esp_https_ota_perform()` chunk holds `led_mutex` (flash writes vs. RMT — same rule as NVS commits). Expect the render to breathe slightly during a download; that's by design.
- **Testing/debug:** `/debug` has a "Check for Update Now" button (`/debug/ota_check`), and `/debug/status` reports `fw`, `ota`, `ota_msg`, `ota_pct`. NVS key `ota_url` (namespace `phase`) overrides the update URL per device for bench testing.
- **Compiler is `-Os`** — kept from edition00.1. Standalone (no-Wi-Fi) lamps never update, by construction.
- **What's re-enabled on edition00.2** (thanks to the roomier 4 MB partition):
    * **WPA3-SAE** and WPA3-compatible mode — connects to WPA3-only routers.
    * **IPv6** (LWIP full stack) — dual-stack home/office networks.
    * **WPA2-Enterprise** (802.1X) — corporate / university networks.
    * `ESP_ERR_TO_NAME_LOOKUP` — readable error strings in the serial log.

## Build & flash

ESP-IDF lives at `~/esp/esp-idf`. Source it once per shell:

```bash
source ~/esp/esp-idf/export.sh
idf.py build
idf.py -p /dev/cu.usbmodem101 erase-flash       # only on first flash to a fresh chip
idf.py -p /dev/cu.usbmodem101 flash             # writes bootloader + partition + app
```

`idf.py fullclean` after switching branches or changing the cmake project name.

## Boot flow (edition00)

```
                  ┌──────────────────────────────┐
                  │   render_task starts in      │
                  │   ANIM_BOOT (blue cycle)     │
                  │   button_task starts polling │
                  └──────────────┬───────────────┘
                                 │
              ┌──────────────────┴───────────────────┐
              │ load_wifi_creds() from NVS namespace │
              │              "phase"                 │
              └──────────────┬───────────────────────┘
              ↓ no                          ↓ yes
   ┌─────────────────────┐      ┌────────────────────────┐
   │ wifi_init_ap()      │      │ wifi_init_sta_with_    │
   │  → SSID "phase"     │      │   creds(ssid, pass)    │
   │ mdns + start_       │      │ wait EV_GOT_IP         │
   │   webserver_ap()    │      │ ANIM_CONNECTING        │
   │ park here forever   │      │ wait EV_CONNECTING_    │
   │                     │      │   DONE (3 pulses)      │
   │ /setup POST writes  │      │ mdns + start_          │
   │ creds → esp_restart │      │   webserver_sta()      │
   └─────────────────────┘      │ time_sync() (SNTP)     │
                                │ ANIM_MOON              │
                                └────────────────────────┘
```

## Code layout — `main/phase-firmware.c`

Single file (~1100 lines). Sections:

- **State / forward decls** — `s_anim_mode`, `s_events`, `led_mutex`, `s_strip`, `s_session_token`.
- **NVS** — render params under key prefixes `face_gradient`/`brightness`/`curve_*`/…; Wi-Fi creds as `wifi_ssid`/`wifi_pass`. All NVS commits take `led_mutex`.
- **Moon math** — `calc_moon_phase()`, `apply_curve()` (unchanged from prototype-02.x — kept per spec).
- **Rendering** — split into `compute_moon_frame()` (writes `frame_b[]` brightness 0..1) and two output stages: `output_white()` (W channel) and `output_blue()` (B channel). Both delta-skip identical frames.
- **render_task** — state machine over `ANIM_BOOT` / `ANIM_CONNECTING` / `ANIM_MOON`. Signals `EV_CONNECTING_DONE` after 3 pulses.
- **button_task** — polls GPIO 20 every 50 ms, fires `clear_wifi_creds()` + `esp_restart()` after 3 s held.
- **Wi-Fi** — `wifi_init_sta_with_creds()` (with `WIFI_PS_NONE`) or `wifi_init_ap()` (APSTA so we can scan from the AP).
- **mDNS** — hostname `phase`, advertises `_http._tcp` on port 80.
- **HTTP** — single `httpd_handle_t`, routes registered conditional on mode:
  - **AP mode:** `/` (setup form), `/scan` (JSON of nearby networks), `POST /setup` (save creds → restart). `/debug` routes are also exposed for on-bench tuning.
  - **STA + AP:** `/` (landing → /debug), `/debug` + `/debug/login` (cookie auth), `/debug/status`, `/debug/set`, `/debug/params`, `/debug/params/save`, `/debug/logout`.
  - Auth: HTTP Basic-style POST → 32-char hex session token in RAM, set as `phase_sess` cookie. Regenerated on every boot.

## Dependencies (`main/idf_component.yml`)

- `espressif/led_strip` — SK6812 driver via RMT
- `espressif/mdns` — mDNS responder
- Core IDF: `esp_wifi`, `esp_http_server`, `esp_sntp`, `nvs_flash`, `driver/gpio`

## Notes / gotchas

- `glimmer_offset[]` / `glimmer_rate[]` / `frame_b[]` / `last_out[]` are sized at compile time from `LED_COUNT`. Bump `LED_COUNT` and they resize for free.
- App partitions are now the two 960 KB OTA slots; the edition00.1 binary lands at ~966 KB with **~16 KB headroom**. `release.sh` refuses to publish a binary that doesn't fit. Getting HTTPS-OTA to fit took: app `-Os` (was `-Og`), cert bundle → common-CA subset, mbedTLS TLS-client-only + no renegotiation + no error strings, Wi-Fi enterprise off, **WPA3-SAE off** (WPA3-only networks can't be joined — transition-mode routers still fine), IPv6 off, exotic ECC curves off (P-256/P-384/25519 kept), silent asserts, no esp_err_to_name tables. Next levers if a future build creeps over: serve the /debug HTML gzipped (~18 KB), or drop default log level to WARN (~20 KB).
- Debug portal creds (`DEBUG_USER` / `DEBUG_PASS`) and Wi-Fi creds are stored in NVS — they survive flash-app updates but are wiped by `idf.py erase-flash`.
- Reset button bug-class: if you ever wire GPIO 9 (BOOT button) here by accident, you'll hold the chip in download mode. GPIO 20 is correct on this revision.

## WS2812/SK6812 glitch fix — carried forward from prototype-02.1

Random per-pixel flashes were caused by RMT-vs-Wi-Fi interrupt contention on the single-core C3 — *not* a 3.3 V → 5 V level-shift problem. Fixed in software via:

1. `esp_wifi_set_ps(WIFI_PS_NONE)` after `esp_wifi_start()`.
2. `led_strip_rmt_config_t.mem_block_symbols = 128`.
3. Dedicated render task at `configMAX_PRIORITIES - 3` + `led_mutex` around `led_strip_refresh()` and any NVS commit.

Refs: [WLED #4382](https://github.com/wled/WLED/issues/4382), [Espressif led_strip docs](https://components.espressif.com/components/espressif/led_strip/). If glitches return on a new revision: re-check `mem_block_symbols`, Wi-Fi PS state, and that nothing new calls NVS/SPIFFS without taking `led_mutex`. Hardware-level backup: 74AHCT125 buffer between data GPIO and DIN. Skip the SB560-on-VDD trick — wrong failure mode.
