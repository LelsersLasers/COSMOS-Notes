import picamera
import picamera.array  # This needs to be imported explicitly
import cv2
import numpy as np

import RPi.GPIO as GPIO

import time

"""
TODO:
- Calibrate bottom cutoff?
- Calibrate/improve color thresholds?
- Try to improve FPS
- Tweak speeds
- Non-linear vertical modifier?
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
        "LOWER": [40, 75, 75],
        "UPPER": [80, 255, 255],
    },
    "PINK": {
        "LOWER": [150, 75, 75],
        "UPPER": [180, 255, 255],
    },
    "YELLOW": {
        "LOWER": [25, 70, 75],
        "UPPER": [50, 255, 255],
    },
    "PURPLE": {
        "LOWER": [135, 140, 10],
        "UPPER": [160, 255, 255],
    },
    "BLUE": {
        "LOWER": [80, 140, 10],
        "UPPER": [140, 255, 255],
    },
    # "NAVY": [100, 120],  # NOT GREAT FIT?
}


# SCREEN_SIZE = [640, 480]
FRAMERATE = 32
SCREEN_SIZE = [480, 368]
# FRAMERATE = 60

BOTTOM_CUTOFF = 0.8

DISPLAY_DEBUG = True


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
    - Track to green while center is above the cutoff line
    - Then state = left
- State: pink
    - Same as green, but state = right
- State: yellow
    - Track to yellow while center is above the cutoff line
    - Then state = straight
- State: left/right/straight
    - Turn left/right/straight until another tag (above the cutoff) is detected
"""
class FSM:
    START = 0
    GREEN = 1
    PINK = 2
    YELLOW = 3

    LEFT = 4
    RIGHT = 5
    STRAIGHT = 6

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
        elif state == cls.STRAIGHT:
            return "STRAIGHT"

currentState = FSM.START
#------------------------------------------------------------------------------#
def mask_image(image_hsv, color):
    lowerThreshold = np.array(COLOR_HUE_RANGES[color]["LOWER"])
    upperThreshold = np.array(COLOR_HUE_RANGES[color]["UPPER"])

    color_mask = cv2.inRange(image_hsv, lowerThreshold, upperThreshold)

    return color_mask

def center_of_mask(color_mask):
    cnts, _ = cv2.findContours(color_mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

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

    boxes = (np.int0(cv2.boxPoints(cv2.minAreaRect(c))) for c in cnts)

    # choose the box with the largest area
    try:
        max_box = max(boxes, key=cv2.contourArea)

        # cx and cy are averages of the corners
        cx = int(np.mean(max_box[:, 0]))
        cy = int(np.mean(max_box[:, 1]))

        cnts = [max_box]
    except ValueError: # max() arg is an empty sequence
        pass

    return cx, cy, cnts

t0 = time.time()
t1 = time.time()
delta = 1 / 32

MAX_SPEED = 50.0
SPIN_MOTOR_SPEED = 30.0
STRAIGHT_MOTOR_SPEED = 30.0
MIN_VERTICAL_MODIFIER = 0.5

def vertical_modifier_linear(top_spacing):
    global MIN_VERTICAL_MODIFIER, BOTTOM_CUTOFF
    return (MIN_VERTICAL_MODIFIER - 1.0) / (BOTTOM_CUTOFF - 0.5) * (top_spacing - 0.5) + 1.0

def vertical_modifier_exponential(top_spacing):
    return -10.0 * (top_spacing - 0.5) ** 2 + 1.0

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


        if currentState in [FSM.START, FSM.LEFT, FSM.RIGHT, FSM.STRAIGHT]:
            if currentState == FSM.START:
                left_speed = 0.0
                right_speed = 0.0
            elif currentState == FSM.LEFT:
                left_speed = 0.0
                right_speed = SPIN_MOTOR_SPEED
            elif currentState == FSM.RIGHT:
                left_speed = SPIN_MOTOR_SPEED
                right_speed = 0.0
            elif currentState == FSM.STRAIGHT:
                left_speed = STRAIGHT_MOTOR_SPEED
                right_speed = STRAIGHT_MOTOR_SPEED

            # for color in COLOR_HUE_RANGES:
            for color in ["GREEN", "PINK", "YELLOW"]:
                color_mask = mask_image(image_hsv, color)
                cx, cy, cnts = center_of_mask(color_mask)

                cutoff = SCREEN_SIZE[1] * BOTTOM_CUTOFF
                if cy is not None and cy <= cutoff:
                    print("Found color: " + color)
                    currentState = FSM.color_to_state(color)
                    break
        elif currentState in [FSM.GREEN, FSM.PINK, FSM.YELLOW]:
            color = FSM.state_to_color(currentState)
            color_mask = mask_image(image_hsv, color)

            cx, cy, cnts = center_of_mask(color_mask)

            cutoff = SCREEN_SIZE[1] * BOTTOM_CUTOFF
            if cy is not None and cy > cutoff:
                if currentState == FSM.GREEN:
                    currentState = FSM.LEFT
                elif currentState == FSM.PINK:
                    currentState = FSM.RIGHT
                elif currentState == FSM.YELLOW:
                    currentState = FSM.STRAIGHT
            elif cx is not None and cy is not None:
                left_spacing = cx / SCREEN_SIZE[0]
                right_spacing = 1 - left_spacing

                top_spacing = cy / SCREEN_SIZE[1]
                vertical_modifier = vertical_modifier_exponential(top_spacing)
                # vertical_modifier = vertical_modifier_linear(top_spacing)
                vertical_modifier = min(1.0, vertical_modifier)
                # print("Vertical modifier: %.2f" % vertical_modifier)

                max_spacing = max(left_spacing, right_spacing)
                left_spacing /= max_spacing
                right_spacing /= max_spacing

                left_speed = left_spacing * MAX_SPEED * vertical_modifier
                right_speed = right_spacing * MAX_SPEED * vertical_modifier

                if DISPLAY_DEBUG:
                    cv2.circle(image, (cx, cy), 5, (255, 255, 255), -1)


            if DISPLAY_DEBUG:
                cv2.drawContours(image, cnts, -1, (255, 0, 0), 2)
                if cx is not None and cy is not None:
                    cv2.circle(image, (cx, cy), 5, (255, 255, 255), -1)

                start_point = (0, int(SCREEN_SIZE[1] * BOTTOM_CUTOFF))
                end_point = (SCREEN_SIZE[0], int(SCREEN_SIZE[1] * BOTTOM_CUTOFF))

                cv2.line(image, start_point, end_point, (0, 255, 0), 2)
                # cv2.line(image_masked, start_point, end_point, (0, 255, 0), 2)
        else:
            ...

        # Motors
        # pwmB.ChangeDutyCycle(left_speed)
        # pwmA.ChangeDutyCycle(right_speed)

        if DISPLAY_DEBUG:
            cv2.imshow("Frame in BGR", image)
            cv2.waitKey(1)
        
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