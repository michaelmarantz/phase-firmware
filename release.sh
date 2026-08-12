#!/usr/bin/env bash
# ════════════════════════════════════════════════════════════════════
#  release.sh — build + publish a Phase firmware release
#
#  Every connected lamp silently follows the *latest* GitHub release of
#  this repo (it polls .../releases/latest/download/phase.bin on boot
#  and every 6 hours). So publishing a release IS pushing an update to
#  the whole fleet. The lamps install it and reboot on their own —
#  nothing to do on the device side.
#
#  Usage:
#    1. Bump FW_VERSION in main/phase-firmware.c  (e.g. "edition00.2")
#       — lamps update when the published version differs from the one
#       they're running, so a release with an unchanged version is a no-op.
#    2. ./release.sh
#
#  Needs: ESP-IDF env sourced (source ~/esp/esp-idf/export.sh) and the
#  GitHub CLI (`brew install gh`, then `gh auth login`, one time).
#  The release must be publicly downloadable — keep the repo public, or
#  lamps won't be able to fetch updates.
# ════════════════════════════════════════════════════════════════════
set -euo pipefail
cd "$(dirname "$0")"

VER=$(sed -n 's/^#define[[:space:]]*FW_VERSION[[:space:]]*"\(.*\)".*/\1/p' main/phase-firmware.c)
[ -n "$VER" ] || { echo "Could not read FW_VERSION from main/phase-firmware.c"; exit 1; }

command -v idf.py >/dev/null || { echo "idf.py not found — run: source ~/esp/esp-idf/export.sh"; exit 1; }
command -v gh     >/dev/null || { echo "gh not found — brew install gh && gh auth login"; exit 1; }

# Refuse to ship uncommitted code — the release should match a commit.
if [ -n "$(git status --porcelain -- ':!*.DS_Store')" ]; then
    echo "Working tree has uncommitted changes — commit first."; exit 1
fi

if gh release view "$VER" >/dev/null 2>&1; then
    echo "Release \"$VER\" already exists on GitHub."
    echo "Bump FW_VERSION in main/phase-firmware.c before publishing."
    exit 1
fi

echo "── Building $VER ─────────────────────────────"
idf.py build

BIN=build/phase-prototype.bin
[ -f "$BIN" ] || { echo "Build product $BIN missing"; exit 1; }

# Sanity: the binary must fit the 960 KB OTA slot with a little margin.
SIZE=$(stat -f%z "$BIN" 2>/dev/null || stat -c%s "$BIN")
SLOT=$((0x1F0000))   # 4 MB layout: two 1.9375 MB OTA slots (edition00.2+)
echo "Binary: $SIZE bytes (slot $SLOT)"
if [ "$SIZE" -ge "$SLOT" ]; then
    echo "ERROR: binary does not fit the OTA slot — trim before releasing."; exit 1
fi

cp "$BIN" phase.bin
trap 'rm -f phase.bin' EXIT

echo "── Publishing release $VER ───────────────────"
git tag -f "$VER"
git push origin "HEAD" --tags
gh release create "$VER" phase.bin \
    --title "Phase firmware $VER" \
    --notes "Auto-update release. Deployed lamps pick this up within 6 hours (or on next power-cycle)."

echo ""
echo "✓ Published. Every connected lamp will be running $VER within ~6 hours."
echo "  To nudge one immediately: open its /debug page → Check for Update Now."
