from abc import ABC, abstractmethod


class IncompleteMessageException(Exception):
    """
    Raised when parsing of a message could not finish because
    there was not enough data.
    """


class AbstractMessage(ABC):
    """Represents a valid message that is created from a byte stream."""

    @abstractmethod
    def __init__(self, data: bytearray):
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

    def __init__(self, data: bytearray):
        if len(data) < 3:
            raise IncompleteMessageException()

        if data[1] == 255:
            self.drive = 1
        else:
            self.drive = (data[1] - 127) / 127

        if data[2] == 255:
            self.steering = 1
        else:
            self.steering = (data[2] - 127) / 127

        del data[0:3]


class BodyOnlyMessage(AbstractMessage, ABC):
    """
    Represents a message that does no parsing and stores the body.

    This means that the first two bytes of the body represent
    the size of the body
    """

    def __init__(self, data: bytearray):
        if len(data) < 3:
            raise IncompleteMessageException()

        size = data[1] * 256 + data[2]

        if len(data) < 3 + size:
            raise IncompleteMessageException()

        self.body = data[3:3 + size]

        del data[0:3+size]


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

    return MESSAGE_TYPES[header](message)
