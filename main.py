from copy import copy
from math import floor
from queue import Empty, Queue
from threading import Thread
from typing import Any

import serial
from time import sleep

from matplotlib.pyplot import pause, subplots

SOUND_SPEED_MPS = 343.0
MAX_DISTANCE_CM = 20


class Pulser(serial.Serial):
    def __init__(self, output_queue: Queue, rate_hz: float, com: str = '/dev/cu.usbmodem101', baud: int = 9600):
        super().__init__(com, baud, timeout=.1)
        print('Waiting for device...')
        sleep(3)
        print('Connected to ', self.name)
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

    def _read_time_of_flight(self) -> float:
        return 1e-6 * float(str(self.readline().decode().strip('\r\n')))

    def _pulse_thread_function(self, output_queue: Queue) -> None:
        while self._is_running:
            self.write(b'0')
            output_queue.put(SOUND_SPEED_MPS * self._read_time_of_flight() * 0.5)
            sleep(1 / self._rate)


class Plotter:
    def __init__(self, pulser: Pulser, length_s: float, avg_len: int):
        self._pulser = pulser
        self._length_in_values = int(floor(length_s * self._pulser.rate_hz))
        self._memory = []
        fig, self._ax = subplots(1, 1)
        fig.canvas.mpl_connect('close_event', self._on_close)
        self._ax.plot(self._memory)
        self._avg_len = avg_len

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
                plot_data[n] = sum(self._memory[n-self._avg_len:n]) / self._avg_len

            self._ax.cla()
            self._ax.plot(plot_data)
            self._ax.set_ylim([0, MAX_DISTANCE_CM])
            pause(10e-3)

    def start(self) -> None:
        self._plotting_loop()


if __name__ == '__main__':

    queue = Queue()
    pulser = Pulser(queue, rate_hz=20.0)
    plotter = Plotter(pulser, length_s=5.0, avg_len=5)
    plotter.start()
