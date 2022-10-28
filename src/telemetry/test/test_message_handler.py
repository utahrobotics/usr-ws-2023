import sys
import os

sys.path.append(os.path.abspath('../telemetry'))
from message_handler import parse_message, SoftPing


def test_soft_ping():
    msg = parse_message(bytes([0, 2, 4]))
    assert isinstance(msg, SoftPing)
    assert msg.body == bytes([2, 4])
