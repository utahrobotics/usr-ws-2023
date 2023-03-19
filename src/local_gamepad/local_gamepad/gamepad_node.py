import rclpy
from rclpy.node import Node
from inputs import get_gamepad
from inputs import UnpluggedError
from global_msgs.msg import MovementIntent


# GamepadNode class
# Broadcasts gamepad joystick information, compatible with
class GamepadNode(Node):
    joy_max = 32767  # maximum value for joystick
    joy_min = -32768  # minimum value for joystick
    deadzone = 0.1  # deadzone value, can be changed

    def __init__(self):
        super().__init__("gamepad_node")
        self.publisher = self.create_publisher(
            MovementIntent,
            "movement_intent",
            10
        )
        self.movement_intent = MovementIntent()
        self.controller()

    # normalizes joystick values on a range from [-1,1]
    def joy_normalize(self, val):
        return (2 * (val - self.joy_min) / (self.joy_max - self.joy_min)) - 1

    # publish joystick values to "movement_intent" topic
    def controller(self):
        drive = 0.0
        steering = 0.0
        while True:
            try:
                events = get_gamepad()
                self.get_logger().info('Event received')
                for event in events:
                    value = self.joy_normalize(event.state)

                    if abs(value) <= self.deadzone:
                        value = 0

                    # left axis joystick horizontal with deadzone
                    if event.code == 'ABS_X':
                        steering = value

                    # left axis joystick vertical with deadzone
                    elif event.code == 'ABS_Y':
                        drive = value

                self.movement_intent.steering = steering
                self.movement_intent.drive = drive
                # publish movement intent
                self.publisher.publish(self.movement_intent)
            except UnpluggedError:
                continue


def main():
    rclpy.init()
    rclpy.spin(GamepadNode())


if __name__ == "__main__":
    main()
