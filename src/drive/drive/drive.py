import rclpy
from rclpy.node import Node
from rclpy.timer import Rate, Timer
from rcl_interfaces.msg import ParameterDescriptor

import Jetson.GPIO as GPIO
from multiprocessing import Value
from threading import Event, Thread
from itertools import count

from drive.drive_calculator import drive_steering

from global_msgs.msg import MovementIntent


class Pin:
    def __init__(self, pin_number: int):
        self.pin = pin_number

    def setup_output(self) -> "Pin":
        GPIO.setup(self.pin, GPIO.OUT, initial=GPIO.LOW)
        return self

    def setup_input(self) -> "Pin":
        GPIO.setup(self.pin, GPIO.IN)
        return self

    def set_high(self):
        GPIO.output(self.pin, GPIO.HIGH)

    def set_low(self):
        GPIO.output(self.pin, GPIO.LOW)


class PWM:
    def __init__(self, pin_number: int, frequency: int):
        self._enabled = Event()
        self._duty_cycle = 0.0
        self._duty_cycle_changed = Event()
        self.thr = Thread(
            target=self._main_loop,
            args=(
                pin_number,
                frequency,
                self._enabled,
                self.duty_cycle
            )
        )
        self.thr.start()

    @property
    def duty_cycle(self):
        return self._duty_cycle

    @property.setter
    def duty_cycle(self, value):
        if value == self._duty_cycle:
            return
        self._duty_cycle = value
        self._duty_cycle_changed.set()

    def _main_loop(self, pin_number: int, frequency: int):
        GPIO.setup(pin_number, GPIO.OUT)
        sleeper = Rate(Timer(timer_period_ns=1_000_000_000 / frequency))

        while True:
            GPIO.output(pin_number, GPIO.LOW)
            self._enabled.wait()
            on_cycles = 0
            ratio = self.duty_cycle
            self._duty_cycle_changed.clear()

            for cycles in count(1):
                if self._duty_cycle_changed.is_set():
                    self._duty_cycle_changed.clear()
                    break

                if on_cycles / cycles < ratio:
                    GPIO.output(pin_number, GPIO.HIGH)
                    on_cycles += 1
                else:
                    GPIO.output(pin_number, GPIO.LOW)

                sleeper.sleep()

    def enable(self):
        self._enabled.set()

    def disable(self):
        self._enabled.clear()


class Drive(Node):
    DUTY_FREQUENCY = 10000
    WAIT_FOR_READY_DURATION = 3

    def __init__(self):
        super().__init__("drive")
        GPIO.setmode(GPIO.BOARD)

        self.declare_parameter(
            "enable_pin",
            11,
            ParameterDescriptor(
                description="The pin that enables all drive motors"
            )
        )

        self.declare_parameter(
            "right_duty_pin",
            32,
            ParameterDescriptor(
                description="The pin that controls the speed of"
                " all motors on the right side"
            )
        )
        self.declare_parameter(
            "left_duty_pin",
            33,
            ParameterDescriptor(
                description="The pin that controls the speed of"
                " all motors on the left side"
            )
        )

        self.declare_parameter(
            "right_dir_pin",
            18,
            ParameterDescriptor(
                description="The pin that controls the direction"
                " (forwards or reverse) of all motors on the right side"
            )
        )
        self.declare_parameter(
            "left_dir_pin",
            13,
            ParameterDescriptor(
                description="The pin that controls the direction"
                " (forwards or reverse) of all motors on the left side"
            )
        )

        self.declare_parameter(
            "front_left_ready_pin",
            23,
            ParameterDescriptor(
                description="The pin that controls the speed of"
                " all motors on the right side"
            )
        )
        self.declare_parameter(
            "front_right_ready_pin",
            22,
            ParameterDescriptor(
                description="The pin that controls the speed of"
                " all motors on the left side"
            )
        )
        self.declare_parameter(
            "back_left_ready_pin",
            19,
            ParameterDescriptor(
                description="The pin that controls the direction"
                " (forwards or reverse) of all motors on the right side"
            )
        )
        self.declare_parameter(
            "back_right_ready_pin",
            26,
            ParameterDescriptor(
                description="The pin that controls the direction"
                " (forwards or reverse) of all motors on the left side"
            )
        )
        self.declare_parameter(
            "pwm_frequency",
            750,
            ParameterDescriptor(
                description="The frequency at which we perform PWM"
            )
        )

        self.enable_pin = Pin(
            self.get_parameter("enable_pin")
                .get_parameter_value()
                .integer_value
        ).setup_output()

        frequency = self.get_parameter("pwm_frequency") \
            .get_parameter_value() \
                .integer_value
        # GPIO.setup(32, GPIO.OUT)
        # self.right_duty_pin = GPIO.PWM(32, 100)
        # GPIO.setup(33, GPIO.OUT)
        # self.left_duty_pin = GPIO.PWM(33, 100)
        self.right_duty_pin = PWM(
            self.get_parameter("right_duty_pin")
                .get_parameter_value()
                .integer_value,
            frequency
        )
        self.left_duty_pin = PWM(
            self.get_parameter("left_duty_pin")
                .get_parameter_value()
                .integer_value,
            frequency
        )

        self.right_dir_pin = Pin(
            self.get_parameter("right_dir_pin")
                .get_parameter_value()
                .integer_value
        ).setup_output()
        self.left_dir_pin = Pin(
            self.get_parameter("left_dir_pin")
                .get_parameter_value()
                .integer_value
        ).setup_output()

        front_left_ready_pin = self.get_parameter_or("front_left_ready_pin")  \
            .get_parameter_value()  \
            .integer_value
        front_right_ready_pin = self.get_parameter_or("front_right_ready_pin")\
            .get_parameter_value()  \
            .integer_value
        back_left_ready_pin = self.get_parameter_or("back_left_ready_pin")   \
            .get_parameter_value()  \
            .integer_value
        back_right_ready_pin = self.get_parameter("back_right_ready_pin")   \
            .get_parameter_value()  \
            .integer_value

        GPIO.setup(front_left_ready_pin, GPIO.IN)
        GPIO.setup(front_right_ready_pin, GPIO.IN)
        GPIO.setup(back_left_ready_pin, GPIO.IN)
        GPIO.setup(back_right_ready_pin, GPIO.IN)

        front_left_is_ready = Event()
        front_right_is_ready = Event()
        back_left_is_ready = Event()
        back_right_is_ready = Event()

        if GPIO.input(front_left_ready_pin) == GPIO.HIGH:
            front_left_is_ready.set()
        if GPIO.input(front_right_ready_pin) == GPIO.HIGH:
            front_right_is_ready.set()
        if GPIO.input(back_left_ready_pin) == GPIO.HIGH:
            back_left_is_ready.set()
        if GPIO.input(back_right_ready_pin) == GPIO.HIGH:
            back_right_is_ready.set()

        logger = self.get_logger()

        def on_change(name: str, ready_event: Event, pin_num: int):
            if GPIO.input(pin_num) == GPIO.HIGH:
                logger.info(f"{name} wheel is ready")
                ready_event.set()
            else:
                logger.warn(f"{name} wheel unreadied!")
                ready_event.clear()
                self.enable_pin.set_low()

        GPIO.add_event_detect(
            front_left_ready_pin,
            GPIO.BOTH,
            callback=lambda pin_num: on_change(
                "Front left",
                front_left_is_ready,
                pin_num
            )
        )
        GPIO.add_event_detect(
            front_right_ready_pin,
            GPIO.BOTH,
            callback=lambda pin_num: on_change(
                "Front right",
                front_right_is_ready,
                pin_num
            )
        )
        GPIO.add_event_detect(
            back_left_ready_pin,
            GPIO.BOTH,
            callback=lambda pin_num: on_change(
                "Back left",
                back_left_is_ready,
                pin_num
            )
        )
        GPIO.add_event_detect(
            back_right_ready_pin,
            GPIO.BOTH,
            callback=lambda pin_num: on_change(
                "Back right",
                back_right_is_ready,
                pin_num
            )
        )

        self.ready_events = [
            front_left_is_ready,
            front_right_is_ready,
            back_left_is_ready,
            back_right_is_ready
        ]

        # Wait until all wheels are ready
        # This includes rewaiting for wheels
        # that unreadied while waiting for other wheels
        while True:
            for event in self.ready_events:
                event.wait()
            for event in self.ready_events:
                if not event.is_set():
                    break
            else:
                break

        logger.info("All wheels ready!")

        self.movement_listener = self.create_subscription(
            MovementIntent,
            'movement_intent',
            self.movement_callback,
            10
        )

    def movement_callback(self, msg):
        if msg.drive == 0:
            self.enable_pin.set_low()
            return

        left_drive, right_drive = drive_steering(msg.drive, msg.steering)

        self.enable_pin.set_high()
        self.set_drive(True, right_drive)
        self.set_drive(False, left_drive)

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

        for ready in self.ready_events:
            if not ready.is_set():
                return

        right_disabled = abs(right_drive) < 0.01
        left_disabled = abs(left_drive) < 0.01

        if right_disabled and left_disabled:
            self.enable_pin.set_low()
        else:
            self.enable_pin.set_high()

        if right_drive < 0:
            self.right_dir_pin.set_high()
        else:
            self.right_dir_pin.set_low()

        if left_drive < 0:
            self.left_dir_pin.set_high()
        else:
            self.left_dir_pin.set_low()

        if right_disabled:
            self.right_duty_pin.disable()
        else:
            self.right_duty_pin.enable()
            self.right_duty_pin.duty_cycle = abs(right_drive)

        if left_disabled:
            self.left_duty_pin.disable()
        else:
            self.left_duty_pin.enable()
            self.left_duty_pin.duty_cycle = abs(left_drive)



def main():
    rclpy.init()
    drive = Drive()
    rclpy.spin(drive)
    GPIO.cleanup()
    drive.get_logger().info("GPIO Cleaned up")


if __name__ == "__main__":
    main()
