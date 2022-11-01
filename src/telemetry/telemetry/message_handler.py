from abc import ABC, abstractmethod


class AbstractMessage(ABC):
    """Represents a valid message that is created from a byte stream."""

    @abstractmethod
    def __init__(self, data: bytes):
        """Initialize the message with the given bytes. The header must already be stripped."""
        pass


class RemoteMovementIntent(AbstractMessage):
    """
    A remote request to perform some movement.

    The first byte represents the drive (byte value of 0 represents -1.0 drive,
    and value of 255 represents 1.0 drive)
    The second byte represents the steering (calculated the same as drive)
    """

    def __init__(self, data: bytes):
        if len(data) != 2:
            raise ValueError(f"The given data is of length: {len(data)} instead of 2")

        if data[0] == 255:
            self.drive = 1
        else:
            self.drive = (data[0] - 128) / 128

        if data[1] == 255:
            self.steering = 1
        else:
            self.steering = (data[1] - 128) / 128


class BodyOnlyMessage(AbstractMessage, ABC):
    """Represents a message that does no parsing and stores the body."""

    def __init__(self, data: bytes):
        self.body = data


class SoftPing(BodyOnlyMessage):
    """
    Represents a Software Ping.

    Should just be echoed back to the sender
    """


class HardPing(SoftPing):
    """
    Represents a Hardware Ping.

    Should be echoed back to the sender, and some sort of visual change must occur in the robot
    """


# The position of each message type represents the header that they use
MESSAGE_TYPES = (
    SoftPing,
    HardPing,
    RemoteMovementIntent
)


def parse_message(message: bytes):
    """Parse the given byte stream into a valid message."""
    header = message[0]
    message = message[1::]

    if header >= len(MESSAGE_TYPES):
        raise ValueError(f"The following header is unrecognized: {header}")

    return MESSAGE_TYPES[header](message)
