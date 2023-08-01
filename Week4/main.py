import picamera
import picamera.array  # This needs to be imported explicitly
import cv2
# import numpy as np
import math

import RPi.GPIO as GPIO
# import adafruit_servokit
# import pigpio
from gpiozero import Servo
from gpiozero.pins.pigpio import PiGPIOFactory

import Adafruit_GPIO.SPI as SPI
import Adafruit_MCP3008

import time
import json

import face_detect

"""
TODO:
- Fan
    - Mounting
    - Controling speed/on/off
- 3d print
    - Housing
    - Mounting
- Presentation
- Tuning
- Website controller (super reach)
"""

#------------------------------------------------------------------------------#
FILE = "state.json"

state = {
    "speed": 50,
    "angle1": -1.0,
    "angle2": -1.0,
    "state": "off"
}

def write_state():
    # Write state to file
    with open(FILE, "w") as f:
        json.dump(state, f)
    return

def read_state():
    # Read state from file
    global state
    try:
        with open(FILE, "r") as f:
            state = json.load(f)
            state["angle1"] = float(state["angle1"])
            state["angle2"] = float(state["angle2"])
    except:
        time.sleep(0.1)
        read_state()


#------------------------------------------------------------------------------#
factory = PiGPIOFactory()

top_servo = Servo(13, min_pulse_width=0.5/1000, max_pulse_width=2.5/1000, pin_factory=factory)
bot_servo = Servo(12, min_pulse_width=0.5/1000, max_pulse_width=2.5/1000, pin_factory=factory)


def angle_to_value(angle):
    # angle: 0, 180
    # value: -1, 1
    return (angle - 90) / 90


# VERTICAL_ANGLE_SPEED = 10.0
VERTICAL_ANGLE_MIN = 45.0
VERTICAL_ANGLE_MAX = 135.0
VERTICAL_ANGLE_CENTER = 75.0

HORIZONTAL_ANGLE_MIN = 0.0
HORIZONTAL_ANGLE_MAX = 180.0
HORIZONTAL_ANGLE_CENTER = 90.0

HORIZONTAL_TRACKING_SPEED = 30.0 # degrees per second
VERTICAL_TRACKING_TICK = 15.0 # degrees

top_servo.value = angle_to_value(VERTICAL_ANGLE_CENTER)
bot_servo.value = angle_to_value(HORIZONTAL_ANGLE_CENTER)

state["angle1"] = VERTICAL_ANGLE_CENTER
state["angle2"] = HORIZONTAL_ANGLE_CENTER

K_P = 0.8
K_D = 0.05

DEAD_ZONE_SIZE = 0.1


vertical_angle = VERTICAL_ANGLE_CENTER
horizontal_angle = HORIZONTAL_ANGLE_CENTER

vertical_tracking_direction = -1
horizontal_tracking_direction = 1
#------------------------------------------------------------------------------#
# AUDIO_THRESHOLD = 100
AUDIO_THRESHOLD = 200
CLAP_SPACING = 0.2
CLAP_WAIT = 2.0

AUDIO_CHANNEL = 7
mcp = Adafruit_MCP3008.MCP3008(spi=SPI.SpiDev(0,0))

def read_audio():
    # value = mcp.read_adc(AUDIO_CHANNEL)
    # print(value)
    # return value
    return mcp.read_adc(AUDIO_CHANNEL)

def check_clap():
    return read_audio() > AUDIO_THRESHOLD

# def wait_for_clap():
#     while True:
#         audio = read_audio()
#         if audio > AUDIO_THRESHOLD:
#             break

# def wait_for_two_claps():
#     wait_for_clap()
#     time.sleep(CLAP_SPACING)
#     wait_for_clap()
#------------------------------------------------------------------------------#
class ModeFSM:
    TRACKING = 0
    WAITING = 1
    SCANNING = 2
    OFF = 3
    MANUAL = 4

    MAX_WAIT_TIME = 10.0
    MAX_SCAN_TIME = 20.0

    def __init__(self):
        self.current_state = ModeFSM.OFF
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
        elif self.current_state == ModeFSM.SCANNING:
            if self.time_in_state >= ModeFSM.MAX_SCAN_TIME:
                self.current_state = ModeFSM.OFF
                self.time_in_state = 0.0

                state["state"] = "off"
                write_state()
                
                return True
        return False

    def __str__(self):
        if self.current_state == ModeFSM.TRACKING:
            return "TRACKING"
        elif self.current_state == ModeFSM.WAITING:
            return "WAITING"
        elif self.current_state == ModeFSM.SCANNING:
            return "SCANNING"
        elif self.current_state == ModeFSM.OFF:
            return "OFF"
        elif self.current_state == ModeFSM.MANUAL:
            return "MANUAL"            

    def fan_should_be_on(self):
        return self.current_state in [ModeFSM.TRACKING, ModeFSM.WAITING]

current_state = ModeFSM()
#------------------------------------------------------------------------------#

def clamp(n, smallest, largest): return max(smallest, min(n, largest))
def scale(n, lower_from, upper_from, lower_to, upper_to): return (n - lower_from) / (upper_from - lower_from) * (upper_to - lower_to) + lower_to

def deg_to_rad(deg): return deg * math.pi / 180.0
def rad_to_deg(rad): return rad * 180.0 / math.pi

#------------------------------------------------------------------------------#
t0 = time.time()
t1 = time.time()
delta = 1 / 100
#------------------------------------------------------------------------------#

#------------------------------------------------------------------------------#
# SCREEN_SIZE = (640, 480)
# SCREEN_SIZE = (320, 240)
SCREEN_SIZE = (480, 368)
FRAMERATE = 20

HORIZONTAL_CAMERA_FOV = 62.2
HORIZONTAL_CAMERA_FOV_SCALE_FACTOR = math.tan(deg_to_rad(HORIZONTAL_CAMERA_FOV / 2.0))
VERTICAL_CAMERA_FOV = 48.8
VERTICAL_CAMERA_FOV_SCALE_FACTOR = math.tan(deg_to_rad(VERTICAL_CAMERA_FOV / 2.0))

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

last_clap_time = None

write_state()

try:
    print("Starting camera...")
    time.sleep(0.5)
    print("Press CTRL+C to end the program.")

    for frame in camera.capture_continuous(rawframe, format="bgr", use_video_port=True):

        print(current_state)

        reset_pos = False


        old_state_state = state["state"]

        read_state()

        if old_state_state != state["state"]:
            if state["state"] == "manual":
                current_state.current_state = ModeFSM.MANUAL
            elif state["state"] == "off":
                current_state.current_state = ModeFSM.OFF
            elif state["state"] == "automatic":
                current_state.current_state = ModeFSM.SCANNING
            


        #----------------------------------------------------------------------#
        t1 = time.time()
        delta = t1 - t0
        t0 = t1
        # print(f"Delta: {int(delta * 1000)}ms\tFPS: {int(1 / delta)}")
        #----------------------------------------------------------------------#


        #----------------------------------------------------------------------#
        image = frame.array
        grey_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        #----------------------------------------------------------------------#


        #----------------------------------------------------------------------#
        if check_clap():
            if last_clap_time is None or t1 - last_clap_time > CLAP_WAIT:
                last_clap_time = t1
            elif t1 - last_clap_time < CLAP_WAIT and t1 - last_clap_time > CLAP_SPACING:
                last_clap_time = None
                if current_state.current_state == ModeFSM.OFF:
                    current_state.current_state = ModeFSM.SCANNING

                    state["state"] = "automatic"
                    write_state()
                else:
                    current_state.current_state = ModeFSM.OFF

                    state["state"] = "off"
                    write_state()
                    
                reset_pos = True
                print("2 claps")
        #----------------------------------------------------------------------#
        

        #----------------------------------------------------------------------#
        if current_state.current_state not in [ModeFSM.OFF, ModeFSM.MANUAL]:
            face = face_detect.detect_largest_face(grey_image, SCREEN_SIZE)
            should_reset_pos = current_state.update(face, delta)

            reset_pos = reset_pos or should_reset_pos

            if face is not None:
                face.draw(image)
            
            if (
                face is not None and (
                    face.prop_center_x < (1 - DEAD_ZONE_SIZE) / 2 or
                    face.prop_center_x > (1 + DEAD_ZONE_SIZE) / 2 or
                    face.prop_center_y < (1 - DEAD_ZONE_SIZE) / 2 or
                    face.prop_center_y > (1 + DEAD_ZONE_SIZE) / 2
                )
            ):
                last_center_x = face.prop_center_x
                last_center_y = face.prop_center_y
            else:
                last_center_x = None
                last_center_y = None
        #----------------------------------------------------------------------#

        if not current_state.fan_should_be_on():
            last_error_x = None
            last_error_y = None

        
        #----------------------------------------------------------------------#
        if last_center_x is not None and last_center_y is not None and current_state.current_state not in [ModeFSM.OFF, ModeFSM.MANUAL]:
            cv2.circle(image, (int(last_center_x * SCREEN_SIZE[0]), int(last_center_y * SCREEN_SIZE[1])), 5, (0, 0, 255), -1)

            error_x = last_center_x - 0.5
            scaled_error_x = scale(error_x, -0.5, 0.5, -HORIZONTAL_CAMERA_FOV_SCALE_FACTOR, HORIZONTAL_CAMERA_FOV_SCALE_FACTOR)
            angle_dif_rad_x = math.atan(scaled_error_x)
            angle_dif_x = rad_to_deg(angle_dif_rad_x)

            error_y = last_center_y - 0.5
            scaled_error_y = scale(error_y, -0.5, 0.5, -VERTICAL_CAMERA_FOV_SCALE_FACTOR, VERTICAL_CAMERA_FOV_SCALE_FACTOR)
            angle_dif_rad_y = math.atan(scaled_error_y)
            angle_dif_y = rad_to_deg(angle_dif_rad_y)

            if last_error_x is not None:
                d_error_x = (angle_dif_x - last_error_x) / delta
            else:
                d_error_x = 0.0
            last_error_x = angle_dif_x

            if last_error_y is not None:
                d_error_y = (angle_dif_y - last_error_y) / delta
            else:
                d_error_y = 0.0
            last_error_y = angle_dif_y

            total_error_x = K_P * angle_dif_x + K_D * d_error_x
            total_error_y = K_P * angle_dif_y + K_D * d_error_y

            horizontal_angle -= total_error_x
            vertical_angle += total_error_y


            horizontal_tracking_direction = -1 if total_error_x > 0 else 1
            vertical_tracking_direction = 1 if total_error_y > 0 else 1


            # convert errors back to screen posistions
            angle_dif_rad_x = deg_to_rad(total_error_x)
            pos_dif_scaled_x = math.tan(angle_dif_rad_x)
            pos_dif_x = scale(pos_dif_scaled_x, -HORIZONTAL_CAMERA_FOV_SCALE_FACTOR, HORIZONTAL_CAMERA_FOV_SCALE_FACTOR, -0.5, 0.5)

            angle_dif_rad_y = deg_to_rad(total_error_y)
            pos_dif_scaled_y = math.tan(angle_dif_rad_y)
            pos_dif_y = scale(pos_dif_scaled_y, -VERTICAL_CAMERA_FOV_SCALE_FACTOR, VERTICAL_CAMERA_FOV_SCALE_FACTOR, -0.5, 0.5)

            start_point = (int(SCREEN_SIZE[0] / 2), int(SCREEN_SIZE[1] / 2))
            end_point = (int(SCREEN_SIZE[0] * (pos_dif_x + 0.5)), int(SCREEN_SIZE[1] * (pos_dif_y + 0.5)))
            cv2.line(image, start_point, end_point, (255, 255, 0), 2)
        #----------------------------------------------------------------------#

        #----------------------------------------------------------------------#
        elif current_state.current_state == ModeFSM.SCANNING:
            horizontal_angle += HORIZONTAL_TRACKING_SPEED * delta * horizontal_tracking_direction
            # vertical_angle += TRACKING_SPEED * delta * vertical_tracking_direction / 3

            if horizontal_angle >= HORIZONTAL_ANGLE_MAX:
                horizontal_tracking_direction = -1
                horizontal_angle = HORIZONTAL_ANGLE_MAX
                vertical_angle += VERTICAL_TRACKING_TICK * vertical_tracking_direction
            elif horizontal_angle <= HORIZONTAL_ANGLE_MIN:
                horizontal_tracking_direction = 1
                horizontal_angle = HORIZONTAL_ANGLE_MIN
                vertical_angle += VERTICAL_TRACKING_TICK * vertical_tracking_direction
            
            if vertical_angle >= VERTICAL_ANGLE_MAX:
                vertical_tracking_direction = -1
                vertical_angle = VERTICAL_ANGLE_MAX
            elif vertical_angle <= VERTICAL_ANGLE_MIN:
                vertical_tracking_direction = 1
                vertical_angle = VERTICAL_ANGLE_MIN
        #----------------------------------------------------------------------#

        #----------------------------------------------------------------------#
        elif current_state.current_state == ModeFSM.MANUAL:
            vertical_angle = state["angle1"]
            horizontal_angle = state["angle2"]
        #----------------------------------------------------------------------#



        #----------------------------------------------------------------------#
        if reset_pos:
            vertical_angle = VERTICAL_ANGLE_CENTER
            horizontal_angle = HORIZONTAL_ANGLE_CENTER
        
        horizontal_angle = clamp(horizontal_angle, HORIZONTAL_ANGLE_MIN, HORIZONTAL_ANGLE_MAX)
        bot_servo.value = angle_to_value(horizontal_angle)
        state["angle2"] = horizontal_angle

        vertical_angle = clamp(vertical_angle, VERTICAL_ANGLE_MIN, VERTICAL_ANGLE_MAX)
        top_servo.value = angle_to_value(vertical_angle)
        state["angle1"] = vertical_angle

        write_state()
        #----------------------------------------------------------------------#
    

        #----------------------------------------------------------------------#
        # vertical line
        start_point = (int(SCREEN_SIZE[0] * 0.5), 0)
        end_point = (int(SCREEN_SIZE[0] * 0.5), SCREEN_SIZE[1])
        cv2.line(image, start_point, end_point, (0, 255, 0), 2)

        # horizontal line
        start_point = (0, int(SCREEN_SIZE[1] * 0.5))
        end_point = (SCREEN_SIZE[0], int(SCREEN_SIZE[1] * 0.5))
        cv2.line(image, start_point, end_point, (0, 255, 0), 2)

        # center dead zone
        top_left = (int(SCREEN_SIZE[0] * (1 - DEAD_ZONE_SIZE) / 2), int(SCREEN_SIZE[1] * (1 - DEAD_ZONE_SIZE) / 2))
        bottom_right = (int(SCREEN_SIZE[0] * (1 + DEAD_ZONE_SIZE) / 2), int(SCREEN_SIZE[1] * (1 + DEAD_ZONE_SIZE) / 2))
        cv2.rectangle(image, top_left, bottom_right, (0, 0, 255), 2)

        cv2.imshow('image', image)
        cv2.waitKey(1)
        #----------------------------------------------------------------------#

        rawframe.truncate(0)
finally:
    print("Exiting...")

    try:
        top_servo.value = angle_to_value(VERTICAL_ANGLE_CENTER)
        bot_servo.value = angle_to_value(HORIZONTAL_ANGLE_CENTER)

        state["angle1"] = VERTICAL_ANGLE_CENTER
        state["angle2"] = HORIZONTAL_ANGLE_CENTER
        write_state()

        time.sleep(0.5)

    finally:

        cv2.destroyAllWindows()
        camera.close()

    print("Done.\n")