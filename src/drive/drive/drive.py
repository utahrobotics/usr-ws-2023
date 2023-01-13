import rclpy
from rclpy.node import Node

from gpiozero import OutputDevice, PWMOutputDevice, InputDevice

from global_msgs.msg import MovementIntent
from global_msgs.srv import DriveStatus
from drive.drive_calculator import drive_steering


class Drive(Node):
    DUTY_FREQUENCY = 10000

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
        self.front_left_ready_pin = InputDevice(
            self.get_parameter("front_left_ready_pin")
                .get_parameter_value()
                .integer_value
        )
        self.front_right_ready_pin = InputDevice(
            self.get_parameter("front_right_ready_pin")
                .get_parameter_value()
                .integer_value
        )
        self.back_left_ready_pin = InputDevice(
            self.get_parameter("back_left_ready_pin")
                .get_parameter_value()
                .integer_value
        )
        self.back_right_ready_pin = InputDevice(
            self.get_parameter("back_right_ready_pin")
                .get_parameter_value()
                .integer_value
        )
        self.movement_listener = self.create_subscription(
            MovementIntent,
            'movement_intent',
            self.movement_callback,
            10
        )
        self.status_listener = self.create_service(
            DriveStatus,
            "drive_status",
            self.status_callback
        )

    def movement_callback(self, msg):
        if msg.drive == 0:
            self.enable_pin.value = 0
            return

        left_drive, right_drive = drive_steering(msg.drive, msg.steering)

        self.enable_pin = 1
        self.set_drive(True, right_drive)
        self.set_drive(False, left_drive)

    def status_callback(self, request, response):
        response.front_left_ready = self.front_left_ready_pin.is_active
        response.front_right_ready = self.front_right_ready_pin.is_active
        response.back_left_ready = self.back_left_ready_pin.is_active
        response.back_right_ready = self.back_right_ready_pin.is_active
        return response

    def set_drive(
        self,
        right: bool,
        drive: float
    ):
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
