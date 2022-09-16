import random
from abc import ABC, abstractmethod
from copy import copy
from math import floor, pi, sin
from queue import Empty, Queue
from threading import Thread
from typing import Any

import serial
from time import sleep

from matplotlib.artist import Artist
from matplotlib.pyplot import imread, pause, subplots

SOUND_SPEED_MPS = 343.0
MAX_DISTANCE_CM = 20


class Pulser(ABC):
    def __init__(self, output_queue: Queue, rate_hz: float):
        self._is_running = True
        self._output_queue = output_queue
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
    def __init__(self, output_queue: Queue, rate_hz: float, com: str = '/dev/cu.usbmodem101', baud: int = 9600):
        Pulser.__init__(self, output_queue, rate_hz)
        serial.Serial.__init__(self, com, baud, timeout=.1)
        print('Waiting for device...')
        sleep(3)
        print('Connected to ', self.name)

    def _read_time_of_flight(self) -> float:
        self.write(b'0')
        return 1e-6 * float(str(self.readline().decode().strip('\r\n')))


class MockPulser(Pulser):
    def __init__(self, output_queue: Queue, rate_hz: float):
        super().__init__(output_queue, rate_hz)
        self._phase_delta = 2 * pi / rate_hz
        self._phase = -self._phase_delta

    def _read_time_of_flight(self) -> float:
        self._phase += self._phase_delta
        return (sin(self._phase) + 1) * 400e-6 + 150e-6


class Plotter:
    def __init__(self, pulser: Pulser, length_s: float, avg_len: int):
        self._pulser = pulser
        self._length_in_values = int(floor(length_s * self._pulser.rate_hz))
        self._memory = []
        self._fig, self._ax = subplots(1, 1)
        self._fig.canvas.mpl_connect('close_event', self._on_close)
        self._fig.canvas.mpl_connect('resize_event', self._on_resize)
        self._ax.plot(self._memory)
        self._avg_len = avg_len
        self._needle_image = self._add_needle_image()
        self._fig.patch.set_facecolor('xkcd:pastel blue')
        self._ax.set_facecolor('xkcd:pastel blue')

    def _on_resize(self, event) -> None:
        self._needle_image.remove()
        self._needle_image = self._add_needle_image()

    def _add_needle_image(self) -> Artist:
        nx = int(self._fig.get_figwidth() * self._fig.dpi)
        ny = int(self._fig.get_figheight() * self._fig.dpi)
        needle_image = imread("assets/needle_2.png")
        return self._fig.figimage(needle_image,
                                  xo=nx * 0.8,
                                  yo=ny * 0.87)

    def _on_close(self, event: Any):
        self._pulser.stop()

    def _plotting_loop(self) -> None:
        self._pulser.start()
        while self._pulser.is_running:
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

            self._ax.cla()

            alphas = [i / self._length_in_values for i in range(self._length_in_values)]
            self._ax.scatter(range(len(plot_data)), plot_data, c='xkcd:deep pink', alpha=alphas)
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

            # self._ax.imshow(needle_image,
            #                 extent=[self._length_in_values * 0.98,
            #                         self._length_in_values * 1.02,
            #                         -80, 0],
            #                 aspect='equal')


            pause(10e-3)

    def start(self) -> None:
        self._plotting_loop()


if __name__ == '__main__':
    queue = Queue()
    pulser = MockPulser(queue, rate_hz=20.0)
    plotter = Plotter(pulser, length_s=5.0, avg_len=5)
    plotter.start()
