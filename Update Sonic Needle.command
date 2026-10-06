#!/bin/bash
# Double-click to update the Sonic Needle demo: pulls the latest code and rebuilds
# the desktop app(s). Opens in Terminal; needs an internet connection.
REPO="$HOME/Documents/SonicNeedle"

finish() {
  echo
  read -n 1 -s -r -p "Press any key to close this window."
  echo
  exit "$1"
}

echo "=== Updating Sonic Needle ==="
echo

if pgrep -f -- "--user-data-dir=$HOME/Library/Application Support/SonicNeedle/chrome-profile" >/dev/null; then
  echo "The demo is still running. Quit it first (click the demo screen, press Cmd+Q),"
  echo "then double-click this again."
  finish 1
fi

if [[ ! -d "$REPO/.git" ]]; then
  echo "Can't find the Sonic Needle code at $REPO"
  finish 1
fi
cd "$REPO" || finish 1

echo "Getting the latest code..."
if ! git pull --ff-only; then
  echo
  echo "Couldn't update the code. Check the internet connection. If it says there are"
  echo "local changes, ask the research team."
  finish 1
fi
echo

# Reuse the Python the app was built with (conda isn't activated in this window)
LAUNCHER="$HOME/Desktop/Sonic Needle.app/Contents/MacOS/launch"
if [[ -f "$LAUNCHER" ]]; then
  PYTHON_BIN="$(eval "$(grep '^PYTHON=' "$LAUNCHER")"; echo "$PYTHON")"
  export PYTHON_BIN
fi

./make_app.sh || finish 1
if [[ -d "$HOME/Desktop/Sonic Needle Demo.app" ]]; then
  echo
  ./make_app.sh --demo || finish 1
fi

echo
echo "=== Update complete. Double-click Sonic Needle on the desktop to start. ==="
finish 0
