import RPi.GPIO as GPIO
import adafruit_servokit

kit = adafruit_servokit.ServoKit(channels=16)

# kit.servo[0].angle = 90
kit.servo[9].angle = 90

GPIO.cleanup()