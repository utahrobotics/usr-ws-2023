import asyncio
from threading import Thread
from time import sleep

from serial import Serial
from pyvesc import VESC

from message_handler import parse_message, SoftPing, HardPing
from message_handler import IncompleteMessageException
from message_handler import RemoteControl
from drive_calculator import drive_steering
from angle_sense import AngleSensor

# The maximum size of a message that we can parse at once
BUFFER_SIZE = 128
# How long to wait to reconnect after a connection failure
RECONNECTION_DELAY = 2


async def main(server_addr: str, port: int):
    spare_controller = VESC(
        serial_port="/dev/spare_motor"
    )

    controller = Serial(
        "/dev/drive_control",
        115200
    )

    controller.write("printReadies()\r".encode())
    controller.readline()
    unready = []
    for _ in range(6):
        line = controller.readline().decode()
        # print(line)
        if "0" in line:
            unready.append(line)

    if len(unready) > 3:
        for line in unready:
            print(line)
        return

    arm_vel = 0
    drum_vel_scale = 0.5

    drum_motor = VESC(
        serial_port="/dev/fsesc_drum"
    )

    def arm_thread():
        angle_sensor = AngleSensor()
        rate = 1 / 50
        offset = -147.0
        max_angle = 18.0
        min_angle = -37.0
        scale = 0.6

        arm_motor = VESC(
            serial_port="/dev/fsesc_arm"
        )

        while True:
            sleep(rate)
            arm_angle = angle_sensor.get_angle() + offset

            if arm_vel > 0:
                if arm_angle >= max_angle:
                    arm_motor.set_duty_cycle(0)
                    continue
            elif arm_angle < min_angle:
                arm_motor.set_duty_cycle(0)
                continue

            arm_motor.set_duty_cycle(arm_vel * scale)

    Thread(
        target=arm_thread
    ).start()

    while True:  # outer loop
        while True:  # Connection loop
            try:
                reader, _ = await asyncio.open_connection(
                    server_addr,
                    port
                )
                break
            except ConnectionRefusedError:
                await asyncio.sleep(RECONNECTION_DELAY)
            except Exception as e:
                print(f"Error in Client: {e}")
                await asyncio.sleep(RECONNECTION_DELAY)

        print("TCP Connection established")

        data = bytearray()

        while True:  # Processing loop
            try:
                tmp = await reader.read(BUFFER_SIZE)
            except BrokenPipeError or ConnectionResetError:
                break

            if len(tmp) == 0:  # Connection closed
                break

            data += tmp

            try:
                result = parse_message(data)
            except IncompleteMessageException:
                continue
            except ValueError as e:
                print(e)
                continue

            if isinstance(result, HardPing):
                print("Received HardPing")

            elif isinstance(result, SoftPing):
                print("Received Soft Ping")

            elif isinstance(result, RemoteControl):
                drum_motor.set_duty_cycle(result.drum_vel * drum_vel_scale)
                arm_vel = result.arm_vel

                left_drive, right_drive = drive_steering(
                    result.drive,
                    result.steering
                )

                if abs(right_drive) < 0.01 and abs(left_drive) < 0.01:
                    controller.write(b'setEnable(0)\r')
                    controller.readline()
                    spare_controller.set_duty_cycle(0.0)
                else:
                    spare_controller.set_duty_cycle(right_drive)
                    # The command is
                    # s(enable, left_dir, right_dir, left_speed, right_speed)
                    controller.write((
                        f"s(1,{int(left_drive > 0)},{int(right_drive > 0)},"
                        f"{left_drive},{right_drive})\r"
                    ).encode())
                    controller.readline()

        print("TCP Connection lost. Reconnecting...")


if __name__ == "__main__":
    asyncio.run(
        main(
            "192.168.137.1",
            10000
        )
    )
