from enum import Enum

import numpy.typing as npt
import numpy as np


class InstructionSet(Enum):
    UP = "up"
    DOWN = "down"
    LEFT = "left"
    RIGHT = "right"


class Controller:

    def __init__(self) -> None:
        self.orb_pos = np.array([0, 0, 0])

    def update_pos(self, orb_pos: npt.NDArray, car_pos: npt.NDArray):
        pass

    def next_instruction(self):
        pass
