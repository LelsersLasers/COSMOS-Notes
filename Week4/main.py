import picamera
import picamera.array  # This needs to be imported explicitly
import cv2
import numpy as np

import RPi.GPIO as GPIO
import adafruit_servokit
import pigpio

import time

import face_detect


#------------------------------------------------------------------------------#
kit = adafruit_servokit.ServoKit(channels=16)

GPIO_Servo = 27

SERVO_CHANNEL = 9
pi = pigpio.pi()

def set_pulsewidth(speed):
    print(speed)
    # speed: -1 to 1

    duty_min = 1470
    duty_max = 1507

    middle = (duty_min + duty_max) / 2
    dif = middle - duty_min

    pulsewidth = middle + dif * speed
    print(pulsewidth)

    pi.set_servo_pulsewidth(GPIO_Servo, int(pulsewidth))


VERTICAL_ANGLE_SPEED = 10.0
VERTICAL_ANGLE_MIN = 30.0
VERTICAL_ANGLE_MAX = 150.0
VERTICAL_ANGLE_CENTER = 90.0

horizontal_movement = 0.0

vertical_angle = VERTICAL_ANGLE_CENTER
horizontal_speed = 0.0
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
            last_center_x = None
            last_center_y = None

        # faces = face_detect.detect_faces(grey_image, SCREEN_SIZE)
        # for face in faces:
        #     face.draw(image)

        if last_center_x is not None and last_center_y is not None:
            cv2.circle(image, (int(last_center_x * SCREEN_SIZE[0]), int(last_center_y * SCREEN_SIZE[1])), 5, (0, 0, 255), -1)

            if last_center_x < 0.5:
                horizontal_speed = 1.0
            elif last_center_x > 0.5:
                horizontal_speed = -1.0

            if last_center_y < 0.5:
                vertical_angle -= VERTICAL_ANGLE_SPEED * delta
            elif last_center_y > 0.5:
                vertical_angle += VERTICAL_ANGLE_SPEED * delta
        else:
            horizontal_speed = 0.0


        # print(f"Horizontal Speed: {horizontal_speed}\tVertical Angle: {vertical_angle}")
        
        vertical_angle = clamp(vertical_angle, VERTICAL_ANGLE_MIN, VERTICAL_ANGLE_MAX)
        kit.servo[SERVO_CHANNEL].angle = vertical_angle

        horizontal_movement += horizontal_speed * delta
        set_pulsewidth(horizontal_speed)


    

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

    kit.servo[SERVO_CHANNEL].angle = VERTICAL_ANGLE_CENTER

    GPIO.cleanup()

    pi.set_servo_pulsewidth(GPIO_Servo, 0)
    pi.stop()

    cv2.destroyAllWindows()
    camera.close()

    print("Done.\n")