import rclpy
from rclpy.node import Node

from gpiozero import OutputDevice, PWMOutputDevice, DigitalInputDevice
from threading import Barrier

from drive.drive_calculator import drive_steering

from global_msgs.msg import MovementIntent


class Drive(Node):
    DUTY_FREQUENCY = 10000
    WAIT_FOR_READY_DURATION = 3

    def __init__(self):
        super().__init__("drive")
        self.enable_pin = OutputDevice(
            self.get_parameter("enable_pin")
                .get_parameter_value()
                .integer_value
        )
        self.right_duty_pin = PWMOutputDevice(
            self.get_parameter("right_duty_pin")
                .get_parameter_value()
                .integer_value,
            frequency=self.DUTY_FREQUENCY
        )
        self.left_duty_pin = PWMOutputDevice(
            self.get_parameter("left_duty_pin")
                .get_parameter_value()
                .integer_value,
            frequency=self.DUTY_FREQUENCY
        )
        self.right_dir_pin = OutputDevice(
            self.get_parameter("right_direction_pin")
                .get_parameter_value()
                .integer_value
        )
        self.left_dir_pin = OutputDevice(
            self.get_parameter("left_direction_pin")
                .get_parameter_value()
                .integer_value
        )
        self.front_left_ready_pin = DigitalInputDevice(
            self.get_parameter("front_left_ready_pin")
                .get_parameter_value()
                .integer_value
        )
        self.front_right_ready_pin = DigitalInputDevice(
            self.get_parameter("front_right_ready_pin")
                .get_parameter_value()
                .integer_value
        )
        self.back_left_ready_pin = DigitalInputDevice(
            self.get_parameter("back_left_ready_pin")
                .get_parameter_value()
                .integer_value
        )
        self.back_right_ready_pin = DigitalInputDevice(
            self.get_parameter("back_right_ready_pin")
                .get_parameter_value()
                .integer_value
        )

        barrier = Barrier(5)
        names = ["front_left", "front_right", "back_left", "back_right"]
        ready = [False, False, False, False]

        def on_activated(idx: int):
            self.get_logger().info(f"Wheel: {names[idx]} is ready")
            ready[idx] = True
            barrier.wait()

        self.front_left_ready_pin.when_activated(
            lambda: on_activated(0)
        )
        self.front_right_ready_pin.when_activated(
            lambda: on_activated(1)
        )
        self.back_left_ready_pin.when_activated(
            lambda: on_activated(2)
        )
        self.back_right_ready_pin.when_activated(
            lambda: on_activated(3)
        )

        logger = self.get_logger()
        while True:
            barrier.wait(self.WAIT_FOR_READY_DURATION)

            if False in ready:
                not_ready = []

                for (i, b) in enumerate(ready):
                    if not b:
                        not_ready.append(names[i])

                msg = ', '.join(not_ready)
                logger.warn(f"The following wheels are not ready: {msg}")
                continue

            logger.info("All wheels ready!")
            break

        self.movement_listener = self.create_subscription(
            MovementIntent,
            'movement_intent',
            self.movement_callback,
            10
        )

    def movement_callback(self, msg):
        if msg.drive == 0:
            self.enable_pin.value = 0
            return

        left_drive, right_drive = drive_steering(msg.drive, msg.steering)

        self.enable_pin = 1
        self.set_drive(True, right_drive)
        self.set_drive(False, left_drive)

    def set_drive(self, right: bool, drive: float):
        if not -1 <= drive <= 1:
            self.get_logger().error(f"Received invalid drive of: {drive}")
            return

        if right:
            dir_pin = self.right_dir_pin
            drive_pin = self.right_duty_pin
        else:
            dir_pin = self.left_dir_pin
            drive_pin = self.left_duty_pin

        dir_pin.value = int(drive < 0)
        drive_pin.value = abs(drive)


def main():
    rclpy.init()
    rclpy.spin(Drive())


if __name__ == "__main__":
    main()
