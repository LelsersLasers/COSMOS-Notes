import picamera
import picamera.array  # This needs to be imported explicitly
import cv2
import numpy as np

import RPi.GPIO as GPIO
# import adafruit_servokit
# import pigpio
from gpiozero import Servo
from gpiozero.pins.pigpio import PiGPIOFactory

import time

import face_detect

"""
from gpiozero import Servo
from gpiozero.pins.pigpio import PiGPIOFactory
factory = PiGPIOFactory()
servo = Servo(12, min_pulse=0.5/1000, max_pulse=2.5/1000, pin_factory=factory)

servo.value = (-1, 1)
"""


#------------------------------------------------------------------------------#
factory = PiGPIOFactory()

top_servo = Servo(13, min_pulse_width=0.5/1000, max_pulse_width=2.5/1000, pin_factory=factory)
bot_servo = Servo(12, min_pulse_width=0.5/1000, max_pulse_width=2.5/1000, pin_factory=factory)


def angle_to_value(angle):
    # angle: 0, 180
    # value: -1, 1
    return (angle - 90) / 90


# VERTICAL_ANGLE_SPEED = 10.0
VERTICAL_ANGLE_MIN = 15.0
VERTICAL_ANGLE_MAX = 165.0
VERTICAL_ANGLE_CENTER = 90.0
VERTICAL_SPEED_MODIFIER = 20.0

HORIZONTAL_ANGLE_MIN = 0.0
HORIZONTAL_ANGLE_MAX = 180.0
HORIZONTAL_ANGLE_CENTER = 90.0
HORIZONTAL_SPEED_MODIFIER = 20.0

top_servo.value = angle_to_value(VERTICAL_ANGLE_CENTER)
bot_servo.value = angle_to_value(HORIZONTAL_ANGLE_CENTER)

K_P_Y = 1.2
K_D_Y = 0.1

horizontal_movement = 0.0

vertical_angle = VERTICAL_ANGLE_CENTER
horizontal_angle = HORIZONTAL_ANGLE_CENTER
#------------------------------------------------------------------------------#
class ModeFSM:
    TRACKING = 0
    WAITING = 1
    SCANNING = 2

    MAX_WAIT_TIME = 10.0

    def __init__(self):
        self.current_state = ModeFSM.SCANNING
        self.time_in_state = 0.0

    def update(self, face, delta):
        self.time_in_state += delta
        if face is not None:
            self.current_state = ModeFSM.TRACKING
        elif self.current_state == ModeFSM.TRACKING:
            self.current_state = ModeFSM.WAITING
            self.time_in_state = 0.0
        elif self.current_state == ModeFSM.WAITING:
            if self.time_in_state >= ModeFSM.MAX_WAIT_TIME:
                self.current_state = ModeFSM.SCANNING
                self.time_in_state = 0.0

    def should_track(self):
        # track to center of face if possible, else to last center
        return self.current_state in [ModeFSM.TRACKING, ModeFSM.WAITING]
#------------------------------------------------------------------------------#

def clamp(n, smallest, largest): return max(smallest, min(n, largest))

#------------------------------------------------------------------------------#
t0 = time.time()
t1 = time.time()
delta = 1 / 100
#------------------------------------------------------------------------------#

#------------------------------------------------------------------------------#
SCREEN_SIZE = (640, 480)
# SCREEN_SIZE = (320, 240)
FRAMERATE = 20

camera = picamera.PiCamera()
camera.resolution = (SCREEN_SIZE[0], SCREEN_SIZE[1])
camera.framerate = FRAMERATE
camera.vflip = False
camera.hflip = False

rawframe = picamera.array.PiRGBArray(camera, size=(SCREEN_SIZE[0], SCREEN_SIZE[1]))
#------------------------------------------------------------------------------#


last_center_x = None
last_center_y = None

last_error_y = None
last_error_x = None

try:
    print("Starting camera...")
    time.sleep(0.5)
    print("Press CTRL+C to end the program.")

    for frame in camera.capture_continuous(rawframe, format="bgr", use_video_port=True):

        #----------------------------------------------------------------------#
        t1 = time.time()
        delta = t1 - t0
        t0 = t1

        # print(f"Delta: {int(delta * 1000)}\tFPS: {int(1 / delta)}")
        #----------------------------------------------------------------------#


        #----------------------------------------------------------------------#
        image = frame.array

        grey_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    
        face = face_detect.detect_largest_face(grey_image, SCREEN_SIZE)
        if face is not None:
            face.draw(image)
            last_center_x = face.prop_center_x
            last_center_y = face.prop_center_y
        else:
            # last_center_x = None
            # last_center_y = None
            pass

        # faces = face_detect.detect_faces(grey_image, SCREEN_SIZE)
        # for face in faces:
        #     face.draw(image)

        if last_center_x is not None and last_center_y is not None:
            cv2.circle(image, (int(last_center_x * SCREEN_SIZE[0]), int(last_center_y * SCREEN_SIZE[1])), 5, (0, 0, 255), -1)

            error_x = last_center_x - 0.5
            if last_error_x is not None:
                d_error_x = (error_x - last_error_x) / delta
            else:
                d_error_x = 0.0
            last_error_x = error_x

            total_error_x = K_P_Y * error_x + K_D_Y * d_error_x
            horizontal_angle -= total_error_x * delta * HORIZONTAL_SPEED_MODIFIER


            error_y = last_center_y - 0.5
            if last_error_y is not None:
                d_error_y = (error_y - last_error_y) / delta
            else:
                d_error_y = 0.0
            last_error_y = error_y

            total_error_y = K_P_Y * error_y + K_D_Y * d_error_y
            vertical_angle += total_error_y * delta * VERTICAL_SPEED_MODIFIER

            start_point = (int(SCREEN_SIZE[0] / 2), int(SCREEN_SIZE[1] / 2))
            end_point = (int(SCREEN_SIZE[0] * (total_error_x + 0.5)), int(SCREEN_SIZE[1] * (total_error_y + 0.5)))
            cv2.line(image, start_point, end_point, (255, 255, 0), 2)




        horizontal_angle = clamp(horizontal_angle, HORIZONTAL_ANGLE_MIN, HORIZONTAL_ANGLE_MAX)
        bot_servo.value = angle_to_value(horizontal_angle)
        print(f"horizontal_angle: {horizontal_angle}")


        vertical_angle = clamp(vertical_angle, VERTICAL_ANGLE_MIN, VERTICAL_ANGLE_MAX)
        top_servo.value = angle_to_value(vertical_angle)
        print(f"vertical_angle: {vertical_angle}")

        print("\n")


    

        # vertical line
        start_point = (int(SCREEN_SIZE[0] * 0.5), 0)
        end_point = (int(SCREEN_SIZE[0] * 0.5), SCREEN_SIZE[1])
        cv2.line(image, start_point, end_point, (0, 255, 0), 2)

        # horizontal line
        start_point = (0, int(SCREEN_SIZE[1] * 0.5))
        end_point = (SCREEN_SIZE[0], int(SCREEN_SIZE[1] * 0.5))
        cv2.line(image, start_point, end_point, (0, 255, 0), 2)

        cv2.imshow('image', image)
        cv2.waitKey(1)

        rawframe.truncate(0)
finally:
    print("Exiting...")

    try:
        top_servo.value = angle_to_value(VERTICAL_ANGLE_CENTER)
        bot_servo.value = angle_to_value(HORIZONTAL_ANGLE_CENTER)

        time.sleep(0.5)

    finally:

        cv2.destroyAllWindows()
        camera.close()

    print("Done.\n")