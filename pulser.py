"""Reading distances from the Sonic Needle's ultrasound sensor (or a simulated one)."""
import random
from abc import ABC, abstractmethod
from collections import deque
from dataclasses import asdict, dataclass
from statistics import median
from threading import Thread
from time import perf_counter, sleep
from typing import Callable, Deque, Generator, Optional, Tuple

import serial
from serial import SerialException
from serial.tools import list_ports

SOUND_SPEED_MPS = 343.0
MAX_VALID_DISTANCE_CM = 100.0
ARDUINO_USB_VIDS = {0x2341, 0x2A03, 0x1A86, 0x0403, 0x10C4}  # Arduino, clones' CH340/FTDI/CP210x
RECONNECT_INTERVAL_S = 1.0

# Raw measurement: (round-trip echo time in µs or None if no echo, target switch closed)
Sample = Tuple[Optional[float], bool]


@dataclass(frozen=True)
class Reading:
    echo_us: Optional[float]
    raw_cm: Optional[float]
    distance_cm: Optional[float]  # median filtered; None when there is no echo
    target_hit: bool

    def to_dict(self) -> dict:
        return asdict(self)


ReadingCallback = Callable[[Reading], None]
StatusCallback = Callable[[str, str], None]  # (state, human readable detail)


def echo_us_to_cm(echo_us: float, offset_cm: float) -> float:
    return SOUND_SPEED_MPS * echo_us * 1e-6 * 0.5 * 1e2 - offset_cm


def cm_to_echo_us(distance_cm: float, offset_cm: float) -> float:
    return (distance_cm + offset_cm) * 1e-2 * 2 / SOUND_SPEED_MPS * 1e6


class Pulser(ABC):
    def __init__(self, rate_hz: float, offset_cm: float, median_len: int = 3,
                 hit_distance_cm: Optional[float] = None):
        self._rate = rate_hz
        self._offset_cm = offset_cm
        self._hit_distance_cm = hit_distance_cm
        self._recent_cm: Deque[float] = deque(maxlen=max(1, median_len))
        self._is_running = False
        self._thread = Thread(target=self._run, daemon=True)
        self._on_reading: ReadingCallback = lambda reading: None
        self._on_status: StatusCallback = lambda state, detail: None

    @property
    def rate_hz(self) -> float:
        return self._rate

    @property
    @abstractmethod
    def mode(self) -> str:
        """'sensor' or 'simulated'."""

    def start(self, on_reading: ReadingCallback, on_status: StatusCallback) -> None:
        self._on_reading = on_reading
        self._on_status = on_status
        self._is_running = True
        self._thread.start()

    def stop(self) -> None:
        self._is_running = False
        if self._thread.is_alive():
            self._thread.join(timeout=2)
        self._close()

    def _run(self) -> None:
        period = 1 / self._rate
        next_t = perf_counter()
        while self._is_running:
            sample = self._measure()
            if sample is not None:
                self._on_reading(self._process(*sample))
            next_t += period
            delay = next_t - perf_counter()
            if delay > 0:
                sleep(delay)
            else:
                next_t = perf_counter()

    def _process(self, echo_us: Optional[float], target_hit: bool) -> Reading:
        raw_cm = None
        if echo_us:
            raw_cm = echo_us_to_cm(echo_us, self._offset_cm)
            if raw_cm > MAX_VALID_DISTANCE_CM:
                raw_cm = None
        if raw_cm is None:
            distance_cm = None
        else:
            self._recent_cm.append(raw_cm)
            distance_cm = median(self._recent_cm)
        if self._hit_distance_cm is not None and distance_cm is not None:
            target_hit = target_hit or distance_cm <= self._hit_distance_cm
        return Reading(echo_us=echo_us or None, raw_cm=raw_cm, distance_cm=distance_cm,
                       target_hit=target_hit)

    @abstractmethod
    def _measure(self) -> Optional[Sample]:
        """Take one measurement, or return None if the sensor is unavailable."""

    def _close(self) -> None:
        pass


def normalise_port_name(port: str) -> str:
    """Accept 'cu.usbmodem101' as well as '/dev/cu.usbmodem101' or 'COM3'."""
    if port.startswith('/') or port.upper().startswith('COM'):
        return port
    return '/dev/' + port


def find_arduino_port() -> Optional[str]:
    for port in list_ports.comports():
        device = port.device or ''
        if port.vid in ARDUINO_USB_VIDS or any(
                s in device for s in ('usbmodem', 'usbserial', 'ttyACM', 'ttyUSB')):
            return device
    return None


def parse_sample(line: str) -> Sample:
    """Parse '<echo_us>,<hit>' (or the original firmware's bare '<echo_us>')."""
    fields = line.strip().split(',')
    echo_us = float(fields[0])
    target_hit = len(fields) > 1 and fields[1].strip() == '1'
    return echo_us, target_hit


class ArduinoPulser(Pulser):
    """Polls the Arduino over USB serial, reconnecting automatically if it is unplugged."""

    def __init__(self, rate_hz: float, offset_cm: float, port: Optional[str] = None,
                 baud: int = 115200, **kwargs):
        super().__init__(rate_hz, offset_cm, **kwargs)
        self._port = normalise_port_name(port) if port else None
        self._baud = baud
        self._serial: Optional[serial.Serial] = None
        self._last_status: Optional[Tuple[str, str]] = None

    @property
    def mode(self) -> str:
        return 'sensor'

    def _set_status(self, state: str, detail: str) -> None:
        if (state, detail) != self._last_status:
            self._last_status = (state, detail)
            print(f'[sensor] {state}: {detail}')
            self._on_status(state, detail)

    def _connect(self) -> bool:
        port = self._port or find_arduino_port()
        if port is None:
            self._set_status('disconnected', 'No Arduino found - check the USB cable')
            return False
        try:
            self._set_status('connecting', f'Connecting to {port}')
            self._serial = serial.Serial(port, self._baud, timeout=0.1)
            sleep(2)  # opening the port resets the Arduino; wait for it to boot
            self._serial.reset_input_buffer()
        except (SerialException, OSError) as e:
            self._serial = None
            self._set_status('disconnected', f'Could not open {port}: {e}')
            return False
        self._set_status('connected', f'Connected to {port}')
        return True

    def _measure(self) -> Optional[Sample]:
        if self._serial is None and not self._connect():
            sleep(RECONNECT_INTERVAL_S)
            return None
        try:
            self._serial.write(b'?')
            line = self._serial.readline().decode(errors='ignore')
        except (SerialException, OSError) as e:
            self._close()
            self._set_status('disconnected', f'Lost connection: {e}')
            return None
        try:
            return parse_sample(line)
        except ValueError:
            return None, False  # timed out or garbled; treat as a missed echo

    def _close(self) -> None:
        if self._serial is not None:
            try:
                self._serial.close()
            except (SerialException, OSError):
                pass
            self._serial = None


class MockPulser(Pulser):
    """Simulates a visitor: insert the needle, wobble towards the target, touch it, withdraw."""

    def __init__(self, rate_hz: float, offset_cm: float, **kwargs):
        super().__init__(rate_hz, offset_cm, **kwargs)
        self._rng = random.Random()
        self._script = self._visitor_script()

    @property
    def mode(self) -> str:
        return 'simulated'

    def start(self, on_reading: ReadingCallback, on_status: StatusCallback) -> None:
        super().start(on_reading, on_status)
        on_status('simulated', 'Simulated sensor (demo mode)')

    def _measure(self) -> Optional[Sample]:
        return next(self._script)

    def _echo(self, distance_cm: float, hit: bool = False) -> Sample:
        if self._rng.random() < 0.03:
            return None, hit  # occasional missed echo
        noisy_cm = distance_cm + self._rng.gauss(0, 0.15)
        if self._rng.random() < 0.02:
            noisy_cm += self._rng.uniform(-4, 4)  # occasional spurious reflection
        return cm_to_echo_us(max(noisy_cm, 0.0), self._offset_cm), hit

    def _visitor_script(self) -> Generator[Sample, None, None]:
        rng = self._rng
        dt = 1 / self._rate
        while True:
            # Needle outside the box: nothing in range
            for _ in range(int(rng.uniform(1.5, 3) / dt)):
                yield None, False

            # Advance towards the target, sometimes easing off or backing up
            distance = rng.uniform(15, 18)
            t = 0.0
            while distance > 0:
                t += dt
                easing_off = (t % 4) > 3
                speed = -1.0 if easing_off else 1.4 + rng.uniform(0, 1.2)  # cm/s
                distance = max(distance - speed * dt, 0.0)
                yield self._echo(distance)

            # Touching the target
            for _ in range(int(1.5 / dt)):
                yield self._echo(rng.uniform(-0.1, 0.1), hit=True)

            # Pause, then withdraw
            for _ in range(int(1.0 / dt)):
                yield self._echo(0.3)
            distance = 0.3
            while distance < 19:
                distance += 9 * dt
                yield self._echo(distance)
