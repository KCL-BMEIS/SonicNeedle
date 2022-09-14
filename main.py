from math import floor
from threading import Thread

import serial
from time import sleep

from matplotlib.pyplot import pause, subplots


class Pulser(serial.Serial):
    def __init__(self, com: str = 'COM3', baud: int = 9600):
        super().__init__(com, baud, timeout=.1)
        print('Waiting for device...')
        sleep(3)
        print('Connected to ', self.name)

    def read_distance(self) -> float:
        return float(str(self.readline().decode().strip('\r\n')))


class Plotter:
    def __init__(self, pulser: Pulser, length_s: float, rate_hz: float):
        self._pulser = pulser
        self._length_in_values = int(floor(length_s * rate_hz))
        self._memory = []
        self._ax = subplots(1, 1)[1]
        self._ax.plot(self._memory)
        self._rate = rate_hz
        self._plot_thread = Thread(target=self._plot_thread_function)
        self._is_running = False

    def _plot_thread_function(self) -> None:
        while self._is_running:
            new_distance = self._pulser.read_distance()
            self._memory.append(new_distance)
            if len(self._memory) > self._length_in_values:
                self._memory.pop(0)
            self._ax.cla()
            self._ax.plot(self._memory)
            self._ax.set_ylim([0, 30])
            pause(1 / self._rate)

    def start(self) -> None:
        self._is_running = True
        self._plot_thread.start()

    def stop(self) -> None:
        self._is_running = False
        self._plot_thread.join()


if __name__ == '__main__':

    pulser = Pulser()
    plotter = Plotter(pulser, length_s=5.0, rate_hz=10)

    plotter.start()
