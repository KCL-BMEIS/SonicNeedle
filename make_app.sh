#!/usr/bin/env bash
# Builds a double-clickable "Sonic Needle" app on the Desktop (macOS).
#
#   ./make_app.sh           # the real demo, using the Arduino
#   ./make_app.sh --demo    # "Sonic Needle Demo": simulated sensor, no hardware needed
#
# Run it from a terminal in which the Python environment with pyserial is active,
# e.g. after `conda activate ...`. The app remembers that Python and this folder,
# so after a `git pull` there's no need to rebuild it, unless the folder moves.
#
# Opening the app starts the server in the background and opens the display full
# screen in Chrome. Quit with Cmd+Q, which also stops the server.
set -euo pipefail

REPO="$(cd "$(dirname "$0")" && pwd)"
NAME="Sonic Needle"
MAIN_ARGS=""
if [[ "${1:-}" == "--demo" ]]; then
  NAME="Sonic Needle Demo"
  MAIN_ARGS="--mock"
fi
APP="${APP_DIR:-$HOME/Desktop}/$NAME.app"

# Find a Python that has pyserial
PYTHON=""
for candidate in ${PYTHON_BIN:-} python python3; do
  if command -v "$candidate" >/dev/null && "$candidate" -c 'import serial' 2>/dev/null; then
    PYTHON="$("$candidate" -c 'import sys; print(sys.executable)')"
    break
  fi
done
if [[ -z "$PYTHON" ]]; then
  echo "Couldn't find a Python with pyserial installed. Activate your environment" >&2
  echo "(or run: conda install pyserial) and try again." >&2
  exit 1
fi

if [[ ! -d "/Applications/Google Chrome.app" ]]; then
  echo "Warning: Google Chrome isn't installed. The app will fall back to the default"
  echo "browser, without kiosk mode, and the server will keep running after it closes."
fi

echo "Building $APP"
echo "  code:   $REPO"
echo "  python: $PYTHON"
rm -rf "$APP"
mkdir -p "$APP/Contents/MacOS" "$APP/Contents/Resources"

cat > "$APP/Contents/Info.plist" <<EOF
<?xml version="1.0" encoding="UTF-8"?>
<!DOCTYPE plist PUBLIC "-//Apple//DTD PLIST 1.0//EN" "http://www.apple.com/DTDs/PropertyList-1.0.dtd">
<plist version="1.0">
<dict>
  <key>CFBundleName</key><string>$NAME</string>
  <key>CFBundleDisplayName</key><string>$NAME</string>
  <key>CFBundleIdentifier</key><string>uk.ac.sonicneedle.launcher${MAIN_ARGS:+.demo}</string>
  <key>CFBundleExecutable</key><string>launch</string>
  <key>CFBundleIconFile</key><string>icon</string>
  <key>CFBundlePackageType</key><string>APPL</string>
  <key>CFBundleShortVersionString</key><string>1.0</string>
  <key>LSUIElement</key><true/>
</dict>
</plist>
EOF

# The launcher. Values from this build are baked in at the top.
{
  echo '#!/bin/bash'
  printf 'REPO=%q\n' "$REPO"
  printf 'PYTHON=%q\n' "$PYTHON"
  printf 'MAIN_ARGS=%q\n' "$MAIN_ARGS"
  cat <<'EOF'
PORT=8000
URL="http://localhost:$PORT"
LOG="$HOME/Library/Logs/SonicNeedle.log"
CHROME="/Applications/Google Chrome.app/Contents/MacOS/Google Chrome"
PROFILE="$HOME/Library/Application Support/SonicNeedle/chrome-profile"

alert() {
  osascript -e "display alert \"Sonic Needle\" message \"$1\" as critical" >/dev/null 2>&1
}

# Already open? Leave it alone rather than starting a second copy.
if pgrep -f -- "--user-data-dir=$PROFILE" >/dev/null; then
  exit 0
fi

# Stop a server left over from a previous run that didn't shut down cleanly
pkill -f -- "$REPO/main.py" 2>/dev/null && sleep 1

if [[ ! -f "$REPO/main.py" ]]; then
  alert "Can't find the Sonic Needle code in $REPO. If the folder has moved, run make_app.sh again."
  exit 1
fi

"$PYTHON" -u "$REPO/main.py" --http-port "$PORT" $MAIN_ARGS >"$LOG" 2>&1 &
SERVER=$!
trap 'kill $SERVER 2>/dev/null' EXIT

for _ in $(seq 50); do
  curl -s -o /dev/null "$URL" && break
  if ! kill -0 $SERVER 2>/dev/null; then
    alert "The Sonic Needle server didn't start. Details are in $LOG"
    exit 1
  fi
  sleep 0.2
done

if [[ -x "$CHROME" ]]; then
  # Runs until Chrome is quit (Cmd+Q), then the trap stops the server
  "$CHROME" --kiosk --no-first-run --no-default-browser-check \
    --autoplay-policy=no-user-gesture-required \
    --user-data-dir="$PROFILE" "$URL" >>"$LOG" 2>&1
else
  open "$URL"
  trap - EXIT  # no kiosk to wait for; leave the server running
fi
EOF
} > "$APP/Contents/MacOS/launch"
chmod +x "$APP/Contents/MacOS/launch"

# App icon from assets/icon.svg (cosmetic, so carry on if this fails)
ICON_TMP="$(mktemp -d)"
if qlmanage -t -s 1024 -o "$ICON_TMP" "$REPO/assets/icon.svg" >/dev/null 2>&1 \
    && [[ -f "$ICON_TMP/icon.svg.png" ]]; then
  ICONSET="$ICON_TMP/icon.iconset"
  mkdir "$ICONSET"
  for size in 16 32 128 256 512; do
    sips -z $size $size "$ICON_TMP/icon.svg.png" --out "$ICONSET/icon_${size}x${size}.png" >/dev/null
    sips -z $((size * 2)) $((size * 2)) "$ICON_TMP/icon.svg.png" --out "$ICONSET/icon_${size}x${size}@2x.png" >/dev/null
  done
  iconutil -c icns "$ICONSET" -o "$APP/Contents/Resources/icon.icns" || echo "  (couldn't build the icon)"
else
  echo "  (couldn't render the icon; the app will use the default one)"
fi
rm -rf "$ICON_TMP"
touch "$APP"  # nudge Finder to pick up the icon

echo "Done. Double-click \"$NAME\" on the Desktop to start; Cmd+Q to quit."
