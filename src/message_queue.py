import pickle
from collections import deque


class MessageQueue:
    def __init__(self, name: str, size: int) -> None:
        self.name = name
        self.queue = deque(maxlen=size)

    def add(self, item):
        self.queue.append(item)
