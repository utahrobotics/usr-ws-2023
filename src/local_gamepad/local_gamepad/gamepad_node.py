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
        self.gamepad = hid.Device(vid=self.vendor_id, pid=self.product_id)
        Thread(target=self.controller).start()

    # normalizes joystick values on a range from [-1,1]
    def joy_normalize(self, val):
        return 2 * ((2 * (val - self.joy_min) / (self.joy_max - self.joy_min)) - 1) - 1

    # publish joystick values to "movement_intent" topic
    def controller(self):
        drive = 0.0
        steering = 0.0
        while True:
            report = self.gamepad.read(64)
            # for x axis, 0 is left, 255 is right, 128 is middle
            # for y axis, 0 is up, 255 is down, 128 is middle
            # left stick x, left stick y, right stick y
            joysticks = [report[1], report[2], report[4]]
            # apply deadzone
            for i in range(len(joysticks)):
                if abs(joysticks[i] - 128) <= self.deadzone:
                    joysticks[i] = 0
            # get bumpers, 0 if not pressed, not 0 if pressed
            l_bumper = report[6] & 0b1
            r_bumper = report[6] & 0b10
            # publish movement_intent
            movement_intent = MovementIntent()
            movement_intent.steering = joysticks[0]
            movement_intent.drive = joysticks[1]
            self.move_publisher.publish(movement_intent)
            # publish arm velocity
            self.arm_publisher.publish(joysticks[2])
            # publish drum velocity
            if r_bumper == l_bumper:
                self.drum_publisher.publish(0)
            elif r_bumper > 0:
                self.drum_publisher.publish(1)
            else:
                self.drum_publisher.publish(-1)
            """ OLD VERSION, uses inputs library
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
                self.move_publisher.publish(movement_intent)
            except UnpluggedError:
                continue
            """


def main():
    rclpy.init()
    rclpy.spin(GamepadNode())


if __name__ == "__main__":
    main()
