from gpiozero import OutputDevice, PWMOutputDevice, DigitalInputDevice


class DriveMotorConnection:
    def __init__(self, duty_pin: int, enable_pin: int, direction_pin: int, ready_pin: int):
        self.duty_pin = PWMOutputDevice(duty_pin, frequency = 10000)

        self.enable_pin = OutputDevice(enable_pin)
        self.direction_pin = OutputDevice(direction_pin)
        self.ready_pin = DigitalInputDevice(ready_pin)  # currently unused

        self.invert_drive = False

    def set_power(self, value: float):
        assert 0 <= abs(value) <= 1 

        if value == 0:
            self.enable_pin.value = 0
            return

        self.enable_pin.value = 1
        self.duty_pin.value = value

        if self.invert_drive:
            self.direction_pin.value = int(value > 0)
        else:
            self.direction_pin.value = int(value < 0)
