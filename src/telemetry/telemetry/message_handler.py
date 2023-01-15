from abc import ABC, abstractmethod


class IncompleteMessageException(Exception):
    """
    Raised when parsing of a message could not finish because
    there was not enough data.
    """


class AbstractMessage(ABC):
    """Represents a valid message that is created from a byte stream."""

    @classmethod
    @abstractmethod
    def parse(cls, data: bytearray) -> "AbstractMessage":
        """
        Initialize the message with the given bytes.
        The header must still be present.

        Once the message has been sucessfully parsed,
        it must be removed from data
        """
        pass


class RemoteMovementIntent(AbstractMessage):
    """
    A remote request to perform some movement.

    The first byte represents the drive (byte value of 0 represents -1.0 drive,
    and value of 255 represents 1.0 drive)
    The second byte represents the steering (calculated the same as drive)
    """
    def __init__(self, drive: float, steering: float) -> None:
        self.drive = drive
        self.steering = steering

    @classmethod
    def parse(cls, data: bytearray) -> "RemoteMovementIntent":
        if len(data) < 3:
            raise IncompleteMessageException()

        if data[1] == 255:
            drive = 1
        else:
            drive = (data[1] - 127) / 127

        if data[2] == 255:
            steering = 1
        else:
            steering = (data[2] - 127) / 127

        del data[0:3]
        return RemoteMovementIntent(drive, steering)


class NoBodyMessage(AbstractMessage, ABC):
    """
    """

    @classmethod
    def parse(cls, data: bytearray) -> "AbstractMessage":
        pass


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
        if len(data) < 3:
            raise IncompleteMessageException()

        size = data[1] * 256 + data[2]

        if len(data) < 3 + size:
            raise IncompleteMessageException()

        body = data[3:3 + size]

        del data[0:3+size]
        return cls(body)


class SoftPing(BodyOnlyMessage):
    """
    Represents a Software Ping.

    Should just be echoed back to the sender
    """


class HardPing(SoftPing):
    """
    Represents a Hardware Ping.

    Should be echoed back to the sender,
    and some sort of visual change must occur in the robot
    """


# The position of each message type represents the header that they use
MESSAGE_TYPES = (
    SoftPing,
    HardPing,
    RemoteMovementIntent
)


def parse_message(message: bytearray):
    """Parse the given byte stream into a valid message."""
    header = message[0]

    if header >= len(MESSAGE_TYPES):
        raise ValueError(f"The following header is unrecognized: {header}")

    return MESSAGE_TYPES[header].parse(message)
