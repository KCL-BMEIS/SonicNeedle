#!/usr/bin/env bash
# Start Sonic Needle full screen in Chrome kiosk mode (macOS).
# Any arguments are passed to main.py, e.g.  ./run_kiosk.sh --mock
# Quit Chrome with Cmd+Q, then Ctrl+C here to stop the server.
cd "$(dirname "$0")" || exit 1
PORT=8000

python main.py --http-port "$PORT" "$@" &
SERVER=$!
trap 'kill $SERVER 2>/dev/null' EXIT
sleep 1

# A separate profile makes Chrome honour the flags even if it is already open.
# The autoplay flag lets the sonar sound play without clicking first.
open -na "Google Chrome" --args \
  --kiosk \
  --autoplay-policy=no-user-gesture-required \
  --no-first-run \
  --user-data-dir="${TMPDIR:-/tmp}/sonic-needle-chrome" \
  "http://localhost:$PORT"

wait $SERVER
