from telemetry.message_handler import parse_message, SoftPing, HardPing, RemoteMovementIntent
import pytest


def test_soft_ping_with_body():
    msg = parse_message(bytes([0, 2, 4]))
    assert isinstance(msg, SoftPing)
    assert msg.body == bytes([2, 4])


def test_soft_ping_no_body():
    msg = parse_message(bytes([0]))
    assert isinstance(msg, SoftPing)
    assert len(msg.body) == 0


def test_hard_ping_with_body():
    msg = parse_message(bytes([1, 42, 115, 243, 23]))
    assert isinstance(msg, HardPing)
    assert msg.body == bytes([42, 115, 243, 23])


def test_hard_ping_no_body():
    msg = parse_message(bytes([1]))
    assert isinstance(msg, HardPing)
    assert len(msg.body) == 0


def test_movement_intent01():
    msg = parse_message(bytes([2, 0, 0]))
    assert isinstance(msg, RemoteMovementIntent)
    assert msg.drive == -1
    assert msg.steering == -1


def test_movement_intent02():
    msg = parse_message(bytes([2, 53, 173]))
    assert isinstance(msg, RemoteMovementIntent)
    assert msg.drive == (53 - 128) / 128
    assert msg.steering == (173 - 128) / 128


def test_movement_intent03():
    msg = parse_message(bytes([2, 255, 255]))
    assert isinstance(msg, RemoteMovementIntent)
    assert msg.drive == 1
    assert msg.steering == 1


def test_movement_intent04():
    msg = parse_message(bytes([2, 128, 128]))
    assert isinstance(msg, RemoteMovementIntent)
    assert msg.drive == 0
    assert msg.steering == 0


def test_bad_movement_intent01():
    with pytest.raises(ValueError) as _:
        parse_message(bytes([2, 123]))


def test_bad_movement_intent02():
    with pytest.raises(ValueError) as _:
        parse_message(bytes([2]))


def test_invalid_header01():
    with pytest.raises(ValueError) as _:
        parse_message(bytes([4, 23, 72, 232]))


def test_invalid_header02():
    with pytest.raises(ValueError) as _:
        parse_message(bytes([153]))
