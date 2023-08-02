from gpiozero import Servo
from gpiozero.pins.pigpio import PiGPIOFactory

factory = PiGPIOFactory()

top_servo = Servo(13, min_pulse_width=0.5/1000, max_pulse_width=2.5/1000, pin_factory=factory)
bot_servo = Servo(12, min_pulse_width=0.5/1000, max_pulse_width=2.5/1000, pin_factory=factory)

def angle_to_value(angle):
    # angle: 0, 180
    # value: -1, 1
    return (angle - 90) / 90

top_servo.value = angle_to_value(90.0)
bot_servo.value = angle_to_value(90.0)