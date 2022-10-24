import rclpy
from rclpy.node import Node
from inputs import get_gamepad
from local_gamepad.msg import MovementIntent


# GamepadNode class
# Broadcasts gamepad joystick information, compatible with
class GamepadNode(Node):
    joy_max = 32767
    joy_min = -32768
    deadzone = 0.1

    def __init__(self):
        super().__init__("gamepad_node")
        self.publisher = self.create_publisher(MovementIntent, "movement_intent", 10)
        self.movement_intent = MovementIntent()
        # call the controller function
        self.controller()

    # normalizes joystick values
    def joy_normalize(self, val):
        return (2 * (val - self.joy_min) / (self.joy_max - self.joy_min)) - 1

    # publish joystick values [-1,1]
    def controller(self):
        drive = 0.0
        steering = 0.0
        while True:
            events = get_gamepad()
            for event in events:
                # left axis joystick horizontal
                if event.code == 'ABS_X':
                    steering = value if abs(value := self.joy_normalize(event.state)) > self.deadzone else 0.0
                # left axis joystick vertical
                elif event.code == 'ABS_Y':
                    drive = value if abs(value := self.joy_normalize(event.state)) > self.deadzone else 0.0

            self.movement_intent.steering = steering
            self.movement_intent.drive = drive
            self.publisher.publish(self.movement_intent)


if __name__ == "__main__":
    rclpy.init()
    rclpy.spin(GamepadNode())
