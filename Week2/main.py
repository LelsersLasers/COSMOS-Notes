# This program illustrates how to capture frames in a video stream and
# and how to do extract pixels of a specific color
# It uses openCV

import picamera
import picamera.array  # This needs to be imported explicitly
import cv2
import numpy as np

import RPi.GPIO as GPIO

import time

"""
TODO:
- Improve color detection
    - And use different SAT/VAL ranges per color
- https://theailearner.com/2020/11/03/opencv-minimum-area-rectangle/
Possible ideas:
- Calibrate bottom cutoff
- Calibrate color ranges
"""

# ------------------------------------------------------------------------------#
GPIO.setmode(GPIO.BOARD)

GPIO_Ain1 = 11
GPIO_Ain2 = 13
GPIO_Apwm = 15
GPIO_Bin1 = 29
GPIO_Bin2 = 31
GPIO_Bpwm = 33

GPIO.setup(GPIO_Ain1, GPIO.OUT)
GPIO.setup(GPIO_Ain2, GPIO.OUT)
GPIO.setup(GPIO_Apwm, GPIO.OUT)
GPIO.setup(GPIO_Bin1, GPIO.OUT)
GPIO.setup(GPIO_Bin2, GPIO.OUT)
GPIO.setup(GPIO_Bpwm, GPIO.OUT)

GPIO.output(GPIO_Ain1, True)
GPIO.output(GPIO_Ain2, False)
GPIO.output(GPIO_Bin1, True)
GPIO.output(GPIO_Bin2, False)

pwm_frequency = 50

pwmA = GPIO.PWM(GPIO_Apwm, pwm_frequency)
pwmB = GPIO.PWM(GPIO_Bpwm, pwm_frequency)

pwmA.start(0)
pwmB.start(0)

left_speed = 0.0
right_speed = 0.0
#------------------------------------------------------------------------------#
COLOR_HUE_RANGES = {
    "GREEN": {
        "H": [40, 80],
        "S": [75, 255],
        "V": [75, 255],
    },
    "PINK": {
        "H": [150, 180],
        "S": [75, 255],
        "V": [75, 255],
    },
    "YELLOW": {
        "H": [25, 50],
        "S": [70, 255],
        "V": [75, 255],
    },
    "PURPLE": {
        "H": [135, 160],
        "S": [140, 255],
        "V": [10, 255],
    },
    "BLUE": {
        "H": [80, 110],
        "S": [140, 255],
        "V": [10, 255],
    },
    # "NAVY": [100, 120],  # NOT GREAT FIT?
}


SCREEN_SIZE = [640, 480]
FRAMERATE = 32
# SCREEN_SIZE = [320, 240]
# FRAMERATE = 60

BOTTOM_CUTOFF = 0.8

""


# Initialize the camera and grab a reference to the frame
camera = picamera.PiCamera()
camera.resolution = (SCREEN_SIZE[0], SCREEN_SIZE[1])
camera.framerate = FRAMERATE
camera.vflip = False
camera.hflip = False


# Create an array to store a frame
rawframe = picamera.array.PiRGBArray(camera, size=(SCREEN_SIZE[0], SCREEN_SIZE[1]))
#------------------------------------------------------------------------------#
"""
State: start
    - Wait until a color is detected
- State: green
    - While green is detected & the center is above the cutoff line: go forward
    - Then turn left until another tag is detected
- State: pink
    - Same as green, but turn right
- State: yellow
    - Go forward until another tag is detected
- State: left/right
    - Turn left/right until another tag is detected
"""
class FSM:
    START = 0
    GREEN = 1
    PINK = 2
    YELLOW = 3

    LEFT = 4
    RIGHT = 5

    @classmethod
    def color_to_state(cls, color):
        if color == "GREEN":
            return cls.GREEN
        elif color == "PINK":
            return cls.PINK
        elif color == "YELLOW":
            return cls.YELLOW
        
    @classmethod
    def state_to_color(cls, state):
        if state == cls.GREEN:
            return "GREEN"
        elif state == cls.PINK:
            return "PINK"
        elif state == cls.YELLOW:
            return "YELLOW"
        
    @classmethod
    def to_string(cls, state):
        if state == cls.START:
            return "START"
        elif state == cls.GREEN:
            return "GREEN"
        elif state == cls.PINK:
            return "PINK"
        elif state == cls.YELLOW:
            return "YELLOW"
        elif state == cls.LEFT:
            return "LEFT"
        elif state == cls.RIGHT:
            return "RIGHT"

currentState = FSM.START
#------------------------------------------------------------------------------#
def mask_image(image, image_hsv, color):
    # Note: color is like ["GREEN"]
    lowerThreshold = np.array([
        COLOR_HUE_RANGES[color]["H"][0],
        COLOR_HUE_RANGES[color]["S"][0],
        COLOR_HUE_RANGES[color]["V"][0],
    ])
    upperThreshold = np.array([
        COLOR_HUE_RANGES[color]["H"][1],
        COLOR_HUE_RANGES[color]["S"][1],
        COLOR_HUE_RANGES[color]["V"][1],
    ])

    color_mask = cv2.inRange(image_hsv, lowerThreshold, upperThreshold)
    image_masked = cv2.bitwise_and(image, image, mask=color_mask)

    cv2.imshow("Masked image", image_masked)

    return color_mask, image_masked

def center_of_mask(color_mask):
    cnts, _ = cv2.findContours(color_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    cx = None
    cy = None

    # for c in cnts:
    #     M = cv2.moments(c)
    #     if M["m00"] != 0:
    #         cx = int(M["m10"] / M["m00"])
    #         cy = int(M["m01"] / M["m00"])
    #         # Is break necessary?
    #         break
    # return cx, cy, cnts

    boxes = []

    for c in cnts:
        rect = cv2.minAreaRect(c)
        box = cv2.boxPoints(rect)
        box = np.int0(box)

        boxes.append(box)

    # choose the box with the largest area
    if len(boxes) > 0:
        max_box = max(boxes, key=cv2.contourArea)

        # cx and cy are averages of the corners
        cx = int(np.mean(max_box[:, 0]))
        cy = int(np.mean(max_box[:, 1]))

        cnts = [max_box]

    return cx, cy, cnts


t0 = time.time()
t1 = time.time()
delta = 1 / 32

MAX_SPEED = 100.0
SPIN_MOTOR_SPEED = 30.0
BLIND_MOTOR_SPEED = 30.0
MIN_VERTICAL_MODIFIER = 0.5

try:
    
    print("Starting camera...")
    time.sleep(2)
    print("Press CTRL+C to end the program.")

    for frame in camera.capture_continuous(rawframe, format="bgr", use_video_port=True):

        t1 = time.time()
        delta = t1 - t0
        t0 = t1

        fps = 1 / delta
        # print("FPS: %.2f\tDelta: %0.2f" % (fps, delta * 1000))

        image = frame.array
        image_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # print("Current state: " + FSM.to_string(currentState), left_speed, right_speed)


        if currentState in [FSM.START, FSM.YELLOW, FSM.LEFT, FSM.RIGHT]:
            if currentState == FSM.START:
                left_speed = 0.0
                right_speed = 0.0
            elif currentState == FSM.YELLOW:
                left_speed = BLIND_MOTOR_SPEED
                right_speed = BLIND_MOTOR_SPEED
            elif currentState == FSM.LEFT:
                left_speed = 0.0
                right_speed = SPIN_MOTOR_SPEED
            elif currentState == FSM.RIGHT:
                left_speed = SPIN_MOTOR_SPEED
                right_speed = 0.0

            # TODO
            # for color in COLOR_HUE_RANGES:
            for color in ["GREEN", "PINK"]:
                color_mask, _ = mask_image(image, image_hsv, color)
                cx, cy, cnts = center_of_mask(color_mask)

                cutoff = SCREEN_SIZE[1] * BOTTOM_CUTOFF
                if cy is not None and cy <= cutoff:
                    print("Found color: " + color)
                    currentState = FSM.color_to_state(color)
                    break
        elif currentState == FSM.GREEN or currentState == FSM.PINK:
            # cv2.destroyWindow(winname)
            color = FSM.state_to_color(currentState)
            color_mask, image_masked = mask_image(image, image_hsv, color)

            cx, cy, cnts = center_of_mask(color_mask)

            cv2.drawContours(image, cnts, -1, (255, 0, 0), 2)
            cv2.drawContours(image_masked, cnts, -1, (255, 0, 0), 2)

            if cx is not None and cy is not None:
                cv2.circle(image, (cx, cy), 5, (255, 255, 255), -1)
                cv2.circle(image_masked, (cx, cy), 5, (255, 255, 255), -1)
                    

            cutoff = SCREEN_SIZE[1] * BOTTOM_CUTOFF
            if cy is not None and cy > cutoff:
                if currentState == FSM.GREEN:
                    currentState = FSM.LEFT
                elif currentState == FSM.PINK:
                    currentState = FSM.RIGHT
            elif cx is not None and cy is not None:
                left_spacing = cx / SCREEN_SIZE[0]
                right_spacing = 1 - left_spacing

                top_spacing = cy / SCREEN_SIZE[1]
                vertical_modifier = (MIN_VERTICAL_MODIFIER - 1.0) / (BOTTOM_CUTOFF - 0.5) * (top_spacing - 0.5) + 1.0
                vertical_modifier = min(1.0, vertical_modifier)
                # print("Vertical modifier: %.2f" % vertical_modifier)

                max_spacing = max(left_spacing, right_spacing)
                left_spacing /= max_spacing
                right_spacing /= max_spacing

                left_speed = left_spacing * MAX_SPEED * vertical_modifier
                right_speed = right_spacing * MAX_SPEED * vertical_modifier

                cv2.circle(image, (cx, cy), 5, (255, 255, 255), -1)
                cv2.circle(image_masked, (cx, cy), 5, (255, 255, 255), -1)
            # else:
            #     left_speed = 0
            #     right_speed = 0


            # cv2.imshow("Masked image", image_masked)

            start_point = (0, int(SCREEN_SIZE[1] * BOTTOM_CUTOFF))
            end_point = (SCREEN_SIZE[0], int(SCREEN_SIZE[1] * BOTTOM_CUTOFF))

            cv2.line(image, start_point, end_point, (0, 255, 0), 2)
            cv2.line(image_masked, start_point, end_point, (0, 255, 0), 2)
        else:
            ...


        pwmB.ChangeDutyCycle(left_speed)
        pwmA.ChangeDutyCycle(right_speed)

        cv2.imshow("Frame in BGR", image)
        # cv2.imshow("Frame in HSV", image_hsv)
        # cv2.imshow("Mask", ourmask)
        # cv2.imshow("Masked image", image_masked)

        
        cv2.waitKey(1)

        # Clear the stream in preparation for the next frame
        rawframe.truncate(0)


# Quit the program when the user presses CTRL + C
except:
    print("Exiting...")

    pwmA.stop()
    pwmB.stop()
    GPIO.cleanup()

    cv2.destroyAllWindows()
    camera.close()

    print("Done")