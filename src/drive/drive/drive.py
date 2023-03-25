import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor

from serial import Serial

from drive.drive_calculator import drive_steering

from global_msgs.msg import MovementIntent


class Drive(Node):
    DUTY_FREQUENCY = 10000
    WAIT_FOR_READY_DURATION = 3

    def __init__(self):
        super().__init__("drive")

        self.declare_parameter(
            "controller_port",
            "/dev/ttyACM2",
            ParameterDescriptor(
                description="The port of the motor controller"
            )
        )

        self.controller = Serial(
            self.get_parameter("controller_port")
                .get_parameter_value()
                .string_value,
            115200
        )

        # self.controller.write("printReadies()\r".encode())
        # self.controller.flush()
        # self.controller.readline()
        # ready = True
        # for _ in range(6):
        #     line = self.controller.readline().decode()
        #     split = line.split(" ")
        #     if split[-1] != "1":
        #         self.get_logger().error(" ".join(split[0::-1]) + " not ready!")
        #         ready = False
        # self.controller.read(4)

        # if not ready:
        #     return

        self.movement_listener = self.create_subscription(
            MovementIntent,
            'movement_intent',
            self.movement_callback,
            50
        )

    def movement_callback(self, msg):
        left_drive, right_drive = drive_steering(msg.drive, msg.steering)
        self.set_drive(left_drive, right_drive)

    def set_drive(self, left_drive: float, right_drive: float):
        if not -1 <= left_drive <= 1:
            self.get_logger().error(
                f"Received invalid left drive of: {left_drive}"
            )
            return
        if not -1 <= right_drive <= 1:
            self.get_logger().error(
                f"Received invalid right drive of: {right_drive}"
            )
            return

        # self.get_logger().info(f'setSpeed({left_drive},{right_drive})')

        if abs(right_drive) < 0.01 and abs(left_drive) < 0.01:
            self.controller.write(b'setEnable(0)\r')
        else:
            # set(enable, left_dir, right_dir, left_speed, right_speed)
            self.controller.write((
                f"s(1,{int(left_drive > 0)},{int(right_drive > 0)},"
                f"{left_drive},{right_drive})\r"
            ).encode())
            self.controller.flush()


def main():
    rclpy.init()
    drive = Drive()
    rclpy.spin(drive)
    drive.controller.close()


if __name__ == "__main__":
    main()
