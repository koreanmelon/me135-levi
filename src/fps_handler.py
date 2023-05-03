import cv2
import numpy.typing as npt
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

    def draw(self, frame: npt.NDArray, CONFIG: dict):
        """Draws FPS on frame.

        Args:
            frame (npt.NDArray): frame to draw FPS on
        """
        cv2.putText(
            img=frame,
            text=f"FPS: {int(self.average())}",
            org=(5, 30),
            fontFace=CONFIG["display"]["font"],
            fontScale=0.5,
            color=CONFIG["display"]["line_color"],
            lineType=CONFIG["display"]["line_type"]
        )
