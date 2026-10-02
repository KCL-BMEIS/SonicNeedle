# Sonic Needle

Arduino and Python code for the ultrasound needle tracking demo at New Scientist Live.

Visitors guide a "smart" needle into a draped box (the patient). A US-100 ultrasound
pulser-receiver in the needle measures the distance to a reflective target inside. A live
display shows the echo, and when the needle touches the target an LED and buzzer go off and
the screen celebrates.

```
Arduino --USB serial--> main.py --Server-Sent Events--> browser (web/)
(echo time, target      (pulser.py, server.py)          (display, sound, game)
 switch)
```

## Setup

```bash
conda install pyserial        # or: pip install -r requirements.txt
```

Upload `NewScientistLive.ino` to the Arduino.

### Wiring

| Arduino pin | Connects to |
|---|---|
| 11 | US-100 Trig/TX |
| 12 | US-100 Echo/RX |
| 2  | Needle tip contact (target switch) |
| GND | Target plate (the other side of the switch) |
| 4  | LED (+ series resistor) to GND |
| 5  | Active buzzer to GND |

Remove the US-100 jumper so it runs in trigger/echo mode. Pin 2 uses the internal pull-up,
so touching the needle to the target pulls it to ground. The Arduino lights the LED and
sounds the buzzer itself, so local feedback is instant. To keep the original stand-alone
LED/buzzer circuit instead, set `FEEDBACK_ENABLED = false` and wire only pin 2 and GND
across the switch.

## Running

### Desktop app (for the event)

On the demo laptop, with the Python environment that has pyserial activated:

```bash
./make_app.sh             # creates "Sonic Needle" on the Desktop
./make_app.sh --demo      # optional: "Sonic Needle Demo", with a simulated sensor
```

Double-click the app to start the demo full screen in Chrome. Press Cmd+Q to quit; this
also stops the server. If something goes wrong, the server's output is in
`~/Library/Logs/SonicNeedle.log`.

macOS doesn't let apps like this read Desktop, Documents or Downloads, so the script
copies the code to `~/Library/Application Support/SonicNeedle`. **After a `git pull`,
run `./make_app.sh` again** to update the app.

### From a terminal

```bash
python main.py                    # find the Arduino automatically
python main.py cu.usbmodem101     # or name the serial port
python main.py --mock             # simulated visitor, no hardware needed
```

Then open http://localhost:8000 in Chrome. For the event, `./run_kiosk.sh` starts the
server and opens Chrome full screen with sound enabled (Cmd+Q quits Chrome).

If the Arduino is unplugged, the display shows a red warning and reconnects automatically
when it's plugged back in. It never silently falls back to simulated data, so use
`--mock` explicitly for demos without the hardware.

The needle tip offset and the depth range are set at the top of `main.py`.
`python main.py --help` lists the command line options, which override them:

- `--offset-cm`: how far the needle tip sticks out ahead of the sensor (default 9)
- `--min-cm`, `--max-cm`: depth scale (default -2 to 30)
- `--median`: median filter length to reject spurious echoes (default 3; 1 disables it)
- `--hit-distance CM`: also count a hit when closer than this, if the switch isn't wired

### Keys

| Key | Action |
|---|---|
| F | Toggle full screen |
| M | Mute or unmute sound |
| R | Reset the current round |
| Shift+R | Clear today's best time and target count |
| D | Show debug info |

### Tuning to the box

Round timing thresholds are at the top of `web/js/app.js` (`CONFIG`). The timer starts when
the target is closer than `startBelowCm`. After a hit, the next round starts once the needle
is withdrawn beyond `resetAboveCm`. Set these to suit the box depth. The best time and
target count are stored in the browser and reset each day.
