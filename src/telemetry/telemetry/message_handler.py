from dataclasses import dataclass
from abc import ABC, abstractmethod


@dataclass
class InvalidMessage:
    header: int
    body: bytes


class AbstractMessage(ABC):
    @abstractmethod
    def __init__(self, data: bytes):
        pass


class RemoteMovementIntent(AbstractMessage):
    def __init__(self, data: bytes):
        if len(data) != 2:
            raise ValueError(f"The given data is of length: {len(data)} instead of 2")
        self.drive = (data[0] - 128) / 128
        self.steering = (data[1] - 128) / 128


class BodyOnlyMessage(AbstractMessage):
    def __init__(self, data: bytes):
        self.body = data


class SoftPing(BodyOnlyMessage):
    pass


class HardPing(SoftPing):
    pass


MESSAGE_TYPES = (
    SoftPing,
    HardPing,
    RemoteMovementIntent
)


def parse_message(message: bytes):
    header = message[0]
    message = message[1::]

    if header >= len(MESSAGE_TYPES):
        return InvalidMessage(header, message)

    return MESSAGE_TYPES[header](message)
