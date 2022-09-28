from abc import ABC, abstractmethod
from copy import copy
from math import floor, pi, sin
from queue import Empty, Queue
from threading import Thread
from typing import Any, Generator, List, Optional, Tuple

import serial
from time import perf_counter, sleep

from matplotlib import rcParams
from matplotlib.animation import FuncAnimation
from matplotlib.artist import Artist
from matplotlib.collections import PathCollection
from matplotlib.pyplot import imread, pause, show, subplots
from serial import SerialException

SOUND_SPEED_MPS = 343.0
MAX_DISTANCE_CM = 30

FONT_SIZE = 20
plot_params = {
    'axes.labelsize': FONT_SIZE,
    'axes.titlesize': FONT_SIZE,
    'font.size': FONT_SIZE,
    'legend.fontsize': FONT_SIZE,
    'xtick.labelsize': FONT_SIZE,
    'ytick.labelsize': FONT_SIZE,
    'toolbar': 'None',
}
rcParams.update(plot_params)


class Pulser(ABC):
    def __init__(self, rate_hz: float):
        self._is_running = True
        self._output_queue = Queue()
        self._pulse_thread = Thread(target=self._pulse_thread_function, args=(self._output_queue,))
        self._rate = rate_hz

    @property
    def rate_hz(self) -> float:
        return self._rate

    @property
    def distance_queue(self) -> Queue:
        return self._output_queue

    @property
    def is_running(self) -> bool:
        return self._is_running

    def start(self):
        self._pulse_thread.start()

    def stop(self):
        self._is_running = False
        self._pulse_thread.join()

    def _pulse_thread_function(self, output_queue: Queue) -> None:
        while self._is_running:
            output_queue.put(SOUND_SPEED_MPS * self._read_time_of_flight() * 0.5)
            sleep(1 / self._rate)

    @abstractmethod
    def _read_time_of_flight(self) -> float:
        raise NotImplementedError()


class ArduinoPulser(Pulser, serial.Serial):
    def __init__(self, rate_hz: float, port: str = '/dev/cu.usbmodem101', baud: int = 9600):
        Pulser.__init__(self, rate_hz)
        serial.Serial.__init__(self, port, baud, timeout=.1)
        print('Waiting for device...')
        sleep(3)
        print('Connected to ', self.name)

    def _read_time_of_flight(self) -> float:
        self.write(b'0')
        return 1e-6 * float(str(self.readline().decode().strip('\r\n')))


class MockPulser(Pulser):
    def __init__(self, rate_hz: float):
        super().__init__(rate_hz)
        self._phase_delta = 2 * pi / rate_hz
        self._phase = -self._phase_delta

    def _read_time_of_flight(self) -> float:
        self._phase += self._phase_delta
        return (sin(self._phase) + 1) * 400e-6 + 150e-6


def pulser_factory(port: str, pulse_rate_hz: float) -> Pulser:
    try:
        return ArduinoPulser(rate_hz=pulse_rate_hz,
                             port=port)
    except SerialException as e:
        print(e)
        print('Using mock Pulser instead.')
        return MockPulser(rate_hz=pulse_rate_hz)


class Plotter:
    def __init__(self, pulser: Pulser, plot_length_s: float, avg_len: int):
        self._pulser = pulser
        self._length_in_values = int(floor(plot_length_s * self._pulser.rate_hz))
        self._memory = [-1] * self._length_in_values
        self._fig, self._ax = subplots(1, 1)
        self._fig.canvas.mpl_connect('close_event', self._on_close)
        self._fig.canvas.mpl_connect('resize_event', self._on_resize)

        self._data_stream: Optional[Generator[List[float], None, None]] = None
        self._anim: Optional[FuncAnimation] = None

        self._alphas = _make_decaying_alphas(self._length_in_values)
        self._scatter_collection: Optional[PathCollection] = None
        self._avg_len = avg_len
        self._raw_needle_image = imread("assets/needle_2.png")
        self._plotted_needle_image = self._add_needle_image()
        self._fig.patch.set_facecolor('xkcd:pastel blue')
        self._ax.set_facecolor('xkcd:pastel blue')

    def _on_resize(self, event) -> None:
        self._plotted_needle_image.remove()
        self._plotted_needle_image = self._add_needle_image()

    def _add_needle_image(self) -> Artist:
        nx = int(self._fig.get_figwidth() * self._fig.dpi)
        ny = int(self._fig.get_figheight() * self._fig.dpi)
        return self._fig.figimage(self._raw_needle_image,
                                  xo=nx * 0.8,
                                  yo=ny * 0.87)

    def _on_close(self, event: Any):
        self._pulser.stop()

    def _init_plot(self) -> Tuple[Artist]:

        plot_data = [-1] * self._length_in_values
        self._scatter_collection = self._ax.scatter(range(len(plot_data)), plot_data,
                                                    c='xkcd:deep pink',
                                                    alpha=self._alphas)

        self._ax.set_ylim([0, MAX_DISTANCE_CM])
        self._ax.set_xlim([0, self._length_in_values * 1.12])
        self._ax.yaxis.tick_right()
        self._ax.set_ylabel('Distance (cm)')
        self._ax.yaxis.set_label_position("right")

        self._ax.invert_yaxis()

        self._ax.spines['top'].set_visible(False)
        self._ax.spines['bottom'].set_visible(False)
        self._ax.spines['left'].set_visible(False)
        self._ax.set_xticklabels([])
        self._ax.set_xticks([])
        return self._scatter_collection,

    def _update_plot(self, _) -> Tuple[Artist]:
        self._last_t = perf_counter()
        plot_data = next(self._data_stream)
        if self._scatter_collection is not None:
            self._scatter_collection.set_offsets(
                [[i, v] for (i, v) in enumerate(plot_data)])
        return self._scatter_collection,

    def _plot_data_generator(self) -> Generator[List[float], None, None]:
        while self._pulser.is_running:
            t = perf_counter()
            try:
                new_distance = self._pulser.distance_queue.get(timeout=1) * 1e2
                if new_distance > MAX_DISTANCE_CM * 5:
                    new_distance = MAX_DISTANCE_CM * 5
                self._memory.append(new_distance)
            except Empty:
                raise IOError("No data received from pulser, but pulser is running.")
            if len(self._memory) > self._length_in_values:
                self._memory.pop(0)

            plot_data = copy(self._memory)
            for n in range(self._avg_len, len(plot_data)):
                plot_data[n] = sum(self._memory[n - self._avg_len:n]) / self._avg_len
            yield plot_data

    def start(self) -> None:
        self._pulser.start()
        self._data_stream = self._plot_data_generator()
        self._anim = FuncAnimation(self._fig, self._update_plot,
                                   init_func=self._init_plot, blit=True, interval=5)
        show()


def _make_decaying_alphas(length: int) -> List[float]:
    return [i / length for i in range(length)]


if __name__ == '__main__':

    PULSE_RATE_HZ = 20
    ARDUINO_SERIAL_PORT = '/dev/cu.usbmodem101'

    pulser = pulser_factory(ARDUINO_SERIAL_PORT, PULSE_RATE_HZ)
    plotter = Plotter(pulser, plot_length_s=5.0, avg_len=1)
    plotter.start()
