"""Sonic Needle: ultrasound needle guidance demo.

Reads distances from the needle's ultrasound sensor and serves a live display at
http://localhost:8000 (open it in Chrome, ideally full screen or in kiosk mode).

    python main.py                    # find the Arduino automatically
    python main.py cu.usbmodem101     # or name the serial port
    python main.py --mock             # simulated visitor, no hardware needed
"""
import argparse
import webbrowser
from threading import Timer

from pulser import ArduinoPulser, MockPulser, Pulser
from server import EventHub, make_server

# ---- Demo settings (each can also be overridden on the command line) ----

# How far the needle tip sticks out ahead of the ultrasound sensor. This is subtracted
# from the measured distance, so 0 cm means the needle tip is touching the target.
NEEDLE_TIP_OFFSET_CM = 9.0

# Vertical range of the echo trace, in cm from the needle tip to the target
MIN_DISTANCE_CM = -2.0
MAX_DISTANCE_CM = 30.0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('port', nargs='?',
                        help='Arduino serial port, e.g. cu.usbmodem101 or COM3 (default: auto-detect)')
    parser.add_argument('--mock', action='store_true', help='simulate the sensor instead of using the Arduino')
    parser.add_argument('--baud', type=int, default=115200)
    parser.add_argument('--rate', type=float, default=20, help='pulse rate in Hz (default: %(default)s)')
    parser.add_argument('--offset-cm', type=float, default=NEEDLE_TIP_OFFSET_CM,
                        help='how far the needle tip sticks out ahead of the sensor (default: %(default)s)')
    parser.add_argument('--min-cm', type=float, default=MIN_DISTANCE_CM,
                        help='top of the depth scale (default: %(default)s)')
    parser.add_argument('--max-cm', type=float, default=MAX_DISTANCE_CM,
                        help='bottom of the depth scale (default: %(default)s)')
    parser.add_argument('--median', type=int, default=3,
                        help='median filter length in samples, 1 to disable (default: %(default)s)')
    parser.add_argument('--hit-distance', type=float, default=None, metavar='CM',
                        help='also count the target as hit when closer than this '
                             '(use if the target switch is not wired)')
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--http-port', type=int, default=8000)
    parser.add_argument('--open', action='store_true', help='open the display in the default browser')
    return parser.parse_args()


def make_pulser(args: argparse.Namespace) -> Pulser:
    common = dict(rate_hz=args.rate, offset_cm=args.offset_cm, median_len=args.median,
                  hit_distance_cm=args.hit_distance)
    if args.mock:
        return MockPulser(**common)
    return ArduinoPulser(port=args.port, baud=args.baud, **common)


def main() -> None:
    args = parse_args()
    hub = EventHub()
    pulser = make_pulser(args)
    hub.publish('config', {'min_cm': args.min_cm, 'max_cm': args.max_cm,
                           'rate_hz': pulser.rate_hz, 'mode': pulser.mode}, sticky=True)

    server = make_server(hub, args.host, args.http_port)
    url = f'http://localhost:{args.http_port}'
    print(f'Sonic Needle display: {url}  (Ctrl+C to quit)')
    if args.mock:
        print('Running with a SIMULATED sensor.')

    pulser.start(on_reading=lambda reading: hub.publish('reading', reading.to_dict()),
                 on_status=lambda state, detail: hub.publish(
                     'status', {'state': state, 'detail': detail}, sticky=True))
    if args.open:
        Timer(0.5, webbrowser.open, args=(url,)).start()
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        pulser.stop()
        server.server_close()


if __name__ == '__main__':
    main()
