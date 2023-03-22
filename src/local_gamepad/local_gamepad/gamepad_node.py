import rclpy
from rclpy.node import Node
from inputs import get_gamepad
from inputs import UnpluggedError
from global_msgs.msg import MovementIntent
from threading import Thread
from inputs import devices
import hid


# GamepadNode class
# Broadcasts gamepad joystick information, compatible with
class GamepadNode(Node):
    joy_max = 255  # maximum value for joystick
    joy_min = -255  # minimum value for joystick
    deadzone = 0.1  # deadzone value, can be changed

    def __init__(self):
        super().__init__("gamepad_node")
        self.publisher = self.create_publisher(
            MovementIntent,
            "movement_intent",
            10
        )
        self.get_logger().info(f"reading devices")
        while True:
            for device in hid.enumerate():
                self.get_logger().info(f"0x{device['vendor_id']:04x}:0x{device['product_id']:04x} {device['product_string']}")

        # Thread(target=self.controller).start()

    # normalizes joystick values on a range from [-1,1]
    def joy_normalize(self, val):
        return 2 * ((2 * (val - self.joy_min) / (self.joy_max - self.joy_min)) - 1) - 1

    # publish joystick values to "movement_intent" topic
    def controller(self):
        drive = 0.0
        steering = 0.0
        while True:
            try:
                events = get_gamepad()
                for event in events:
                    value = self.joy_normalize(event.state)

                    if abs(value) <= self.deadzone:
                        value = 0

                    # left axis joystick horizontal with deadzone
                    if event.code == 'ABS_X':
                        steering = float(value)

                    # left axis joystick vertical with deadzone
                    elif event.code == 'ABS_Y':
                        drive = - float(value)

                movement_intent = MovementIntent()
                movement_intent.steering = steering
                movement_intent.drive = drive
                # publish movement intent
                self.publisher.publish(movement_intent)
            except UnpluggedError:
                continue


def main():
    rclpy.init()
    rclpy.spin(GamepadNode())


if __name__ == "__main__":
    main()
