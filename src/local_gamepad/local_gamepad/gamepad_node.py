import rclpy
from rclpy.node import Node
from global_msgs.msg import MovementIntent
from std_msgs.msg import Float32
from threading import Thread
import hid


# GamepadNode class
# Broadcasts gamepad joystick information, compatible with
class GamepadNode(Node):
    joy_max = 255  # maximum value for joystick
    joy_min = 0  # minimum value for joystick
    deadzone = 0.1  # deadzone value, can be changed
    vendor_id = 0x054C
    product_id = 0x05C4

    def __init__(self):
        super().__init__("gamepad_node")
        self.move_publisher = self.create_publisher(
            MovementIntent,
            "movement_intent",
            10
        )
        self.arm_publisher = self.create_publisher(
            Float32,
            "set_arm_velocity",
            10
        )
        self.drum_publisher = self.create_publisher(
            Float32,
            "set_drum_velocity",
            10
        )

        self.gamepad = hid.device()
        self.gamepad.open(self.vendor_id, self.product_id)
        self.gamepad.set_nonblocking(True)
        # self.gamepad = hid.Device(vid=self.vendor_id, pid=self.product_id)
        Thread(target=self.controller).start()
        # self.get_logger().info("Success")

    # normalizes joystick values on a range from [-1,1]
    def joy_normalize(self, val):
        return 2 * ((2 * (val - self.joy_min) / (self.joy_max - self.joy_min)) - 1) - 1

    # publish joystick values to "movement_intent" topic
    def controller(self):
        while True:
            report = self.gamepad.read(64)
            if len(report) == 0:
                continue
            # for x axis, 0 is left, 255 is right, 128 is middle
            # for y axis, 0 is up, 255 is down, 128 is middle
            # left stick x, left stick y, right stick y
            joysticks = [report[1], report[2], report[4]]
            # apply deadzone
            for i in range(len(joysticks)):
                if abs(joysticks[i] - 128) <= 128 * self.deadzone:
                    joysticks[i] = 128
            # get bumpers, 0 if not pressed, not 0 if pressed
            l_bumper = report[6] & 0b1
            r_bumper = report[6] & 0b10
            # publish movement_intent
            movement_intent = MovementIntent()
            movement_intent.steering = joysticks[0] / 128 - 1.0
            movement_intent.drive = - joysticks[1] / 128 + 1.0
            self.move_publisher.publish(movement_intent)
            # publish arm velocity
            self.arm_publisher.publish(Float32(data=joysticks[2] / 255))
            # publish drum velocity
            if r_bumper == l_bumper:
                self.drum_publisher.publish(Float32(data=0.0))
            elif r_bumper > 0:
                self.drum_publisher.publish(Float32(data=1.0))
            else:
                self.drum_publisher.publish(Float32(data=-1.0))


def main():
    rclpy.init()
    rclpy.spin(GamepadNode())


if __name__ == "__main__":
    main()
