import evdev
import RPi.GPIO as GPIO
import adafruit_servokit
import time

kit = adafruit_servokit.ServoKit(channels=16)


# gamepad = evdev.InputDevice("/dev/input/event0")
# print(gamepad)

# TODO: make sure these are right
SERVO_CHANNEL_MAPPINGS = {
    'LEFT_SHOULDER': 1,
    'RIGHT_SHOULDER': 3,
    'LEFT_ELBOW': 2,
    'RIGHT_ELBOW': 4,
}

kit.servo[2].angle = 0
kit.servo[0].angle = 0

kit.servo[1].angle = 180
kit.servo[3].angle = 0




GPIO.cleanup()
# gamepad.close()