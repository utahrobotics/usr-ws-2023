from dataclasses import dataclass


@dataclass
class InvalidMessage:
    header: int
    body: bytes


@dataclass
class BodyOnlyMessage:
    body: bytes


class SoftPing(BodyOnlyMessage):
    pass


class HardPing(SoftPing):
    pass


MESSAGE_TYPES = (
    SoftPing,
    HardPing
)


def parse_message(message: bytes):
    header = message[0]
    message = message[1::]

    if header >= len(MESSAGE_TYPES):
        return InvalidMessage(header, message)

    msg_type = MESSAGE_TYPES[header]
    if issubclass(msg_type, BodyOnlyMessage):
        return msg_type(message)
