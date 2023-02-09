from abc import ABC, abstractmethod


class IncompleteMessageException(Exception):
    """Could not parse message because there was not enough data."""


class AbstractMessage(ABC):
    """Represents a valid message that is created from a byte stream."""

    HEADER_BYTE = None

    @classmethod
    @abstractmethod
    def parse(cls, data: bytearray) -> "AbstractMessage":
        """
        Initialize the message with the given bytes.

        Once the message has been sucessfully parsed,
        it must be removed from data
        """
        pass

    @abstractmethod
    def to_bytes(self) -> bytes:
        """
        Convert self into a bytes object.

        Does not include the header byte.
        """
        pass


class RemoteMovementIntent(AbstractMessage):
    """
    A remote request to perform some movement.

    The first byte represents the drive (byte value of 0 represents -1.0 drive,
    and value of 255 represents 1.0 drive)
    The second byte represents the steering (calculated the same as drive)
    """

    HEADER_BYTE = 2

    def __init__(self, drive: float, steering: float) -> None:
        self.drive = drive
        self.steering = steering

    @classmethod
    def parse(cls, data: bytearray) -> "RemoteMovementIntent":
        if len(data) < 2:
            raise IncompleteMessageException()

        if data[0] == 255:
            drive = 1
        else:
            drive = (data[0] - 127) / 127

        if data[1] == 255:
            steering = 1
        else:
            steering = (data[1] - 127) / 127

        del data[0:2]
        return RemoteMovementIntent(drive, steering)

    def to_bytes(self) -> bytes:
        return bytes([
            255 if self.drive == 1 else (self.drive + 1) * 127,
            255 if self.drive == 1 else (self.steering + 1) * 127
        ])


class NoBodyMessage(AbstractMessage, ABC):
    """Represents a message that has no body."""

    @classmethod
    def parse(cls, data: bytearray) -> "AbstractMessage":
        pass

    def to_bytes(self) -> bytes:
        return b""


class BodyOnlyMessage(AbstractMessage, ABC):
    """
    Represents a message that does no parsing and stores the body.

    This means that the first two bytes of the body represent
    the size of the body
    """

    def __init__(self, body: bytes) -> None:
        self.body = body

    @classmethod
    def parse(cls, data: bytearray) -> "BodyOnlyMessage":
        if len(data) < 2:
            raise IncompleteMessageException()

        size = data[0] * 256 + data[1]

        if len(data) < 2 + size:
            raise IncompleteMessageException()

        body = data[2:2 + size]

        del data[0:2+size]
        return cls(body)

    def to_bytes(self) -> bytes:
        size = len(self.body)
        size_significant = size // 256
        size %= 256
        return bytearray([size_significant, size, *self.body])


class SoftPing(BodyOnlyMessage):
    """
    Represents a Software Ping.

    Should just be echoed back to the sender
    """

    HEADER_BYTE = 0


class HardPing(SoftPing):
    """
    Represents a Hardware Ping.

    Should be echoed back to the sender,
    and some sort of visual change must occur in the robot
    """

    HEADER_BYTE = 1


# The position of each message type represents the header that they use
# The HEADER_BYTE field of each message here must also correspond exactly
# to their position here
MESSAGE_TYPES = (
    SoftPing,
    HardPing,
    RemoteMovementIntent
)


def parse_message(message: bytearray):
    """Parse the given byte stream into a valid message."""
    header = message.pop(0)

    if header >= len(MESSAGE_TYPES):
        raise ValueError(f"The following header is unrecognized: {header}")

    return MESSAGE_TYPES[header].parse(message)


def message_to_bytes(message: AbstractMessage) -> bytes:
    """Convert the given message into bytes."""
    bin = bytearray(message.to_bytes())
    bin.insert(0, message.HEADER_BYTE)
    return bytes(bin)
