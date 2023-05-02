from time import time_ns

import numpy as np


class FPSHandler:
    def __init__(self, max_len=10):
        self.max_len = max_len

        self.timestamps = np.zeros(self.max_len)
        self.prev = self.timestamps[-1]
        self.curr = time_ns()

        self.timestamps = np.roll(self.timestamps, -1)
        self.timestamps[-1] = self.curr

    def update(self) -> None:
        """Updates timestamps.
        """
        self.prev = self.curr
        self.curr = time_ns()

        self.timestamps = np.roll(self.timestamps, -1)
        self.timestamps[-1] = self.curr

    def current(self) -> float:
        """Computes instantaneous FPS.

        Returns:
            float: fps
        """
        return 1e9 / (self.curr - self.prev)

    def average(self) -> float:
        """Computes moving average of FPS over last `max_len` frames.

        Returns:
            float: fps
        """
        return 1e9 / np.diff(self.timestamps).mean()
