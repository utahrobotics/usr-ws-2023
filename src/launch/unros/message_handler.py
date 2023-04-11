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


class RemoteControl(AbstractMessage):
    """
    A remote request to perform some movement.

    The first byte represents the drive (byte value of 0 represents -1.0 drive,
    and value of 255 represents 1.0 drive)
    The second byte represents the steering (calculated the same as drive)
    """

    HEADER_BYTE = 2
    REDUNDANCY = 3

    def __init__(
        self,
        drive: float,
        steering: float,
        arm_vel: float,
        drum_vel: float
    ):
        self.drive = drive
        self.steering = steering
        self.arm_vel = arm_vel
        self.drum_vel = drum_vel

    @classmethod
    def parse(cls, data: bytearray) -> "RemoteControl":
        if len(data) < 4 * cls.REDUNDANCY:
            raise IncompleteMessageException()

        drive = 0.0
        steering = 0.0
        arm_vel = 0.0
        drum_vel = 0.0

        for i in range(cls.REDUNDANCY):
            i *= 4
            if data[i] == 255:
                drive += 1.0
            else:
                drive += (data[i] - 127) / 127

            if data[i + 1] == 255:
                steering += 1.0
            else:
                steering += (data[i + 1] - 127) / 127

            if data[i + 2] == 255:
                arm_vel += 1.0
            else:
                arm_vel += (data[i + 2] - 127) / 127

            if data[i + 3] == 255:
                drum_vel += 1.0
            else:
                drum_vel += (data[i + 3] - 127) / 127

        del data[0:4 * cls.REDUNDANCY]
        drive /= cls.REDUNDANCY
        steering /= cls.REDUNDANCY
        arm_vel /= cls.REDUNDANCY
        drum_vel /= cls.REDUNDANCY
        return RemoteControl(drive, steering, arm_vel, drum_vel)

    def to_bytes(self) -> bytes:
        return bytes([
            255 if self.drive == 1 else int((self.drive + 1) * 127),
            255 if self.steering == 1 else int((self.steering + 1) * 127),
            255 if self.arm_vel == 1 else int((self.arm_vel + 1) * 127),
            255 if self.drum_vel == 1 else int((self.drum_vel + 1) * 127)
        ] * self.REDUNDANCY)


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

    def __init__(self, body: bytes):
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
    RemoteControl,
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
