import evdev
import RPi.GPIO as GPIO
import adafruit_servokit
import time


kit = adafruit_servokit.ServoKit(channels=16)

gamepad = evdev.InputDevice("/dev/input/event0")
print(f"Gamepad: {gamepad}")

"""
TODO:
- Actually go forward
    - Grippers on the top of the paddles
- Tweak the angles
    - For the overall
    - And for differences in left/right due to mounting and servo differences
- Test with controller
- Frictionless back
    - Use bar instead of circle
- Foam under the front
- Strength/durability/reliability
- Speed
"""


#------------------------------------------------------------------------------#
# SERVO_SHOULDERS = ['LEFT_SHOULDER', 'RIGHT_SHOULDER']
# SERVO_ELBOWS = ['LEFT_ELBOW', 'RIGHT_ELBOW']

SERVO_CHANNEL_MAPPINGS = {
    'LEFT': {
        'SHOULDER': 0,
        'ELBOW': 1,
    },
    'RIGHT': {
        'SHOULDER': 2,
        'ELBOW': 3,
    }
}

SERVO_REST_ANGLES = {
    'LEFT': {
        'SHOULDER': 150,
        'ELBOW': 0,
    },
    'RIGHT': {
        'SHOULDER': 30,
        'ELBOW': 180,
    }
}

SERVO_PADDLE_ANGLES = {
    'LEFT': {
        'SHOULDER': 65,
        'ELBOW': 180,
    },
    'RIGHT': {
        'SHOULDER': 115,
        'ELBOW': 0,
    }
}

SHOULDER_DELAY = 0.45
ELBOW_DELAY = 0.75

# To paddle: shoulder down, arm back, shoulder up, arm reset

def set_servo_posistion(side, joint, positions):
    global kit, SERVO_CHANNEL_MAPPINGS
    channel = SERVO_CHANNEL_MAPPINGS[side][joint]
    angle = positions[side][joint]
    kit.servo[channel].angle = angle
    # print(f"Set {side} {joint} ({channel}) to {angle} degrees")
#------------------------------------------------------------------------------#


#------------------------------------------------------------------------------#
BUTTON_CODE_MAPPINGS = {
    'X': 304,
    'A': 305,
    'B': 306,
    'Y': 307,
    'LB': 308,
    'RB': 309,
}

STICK_CODE_MAPPINGS = {
    'DOWNPRESSED': {
        'UP': [1, 0],
        'RIGHT': [0, 255],
        'DOWN': [1, 255],
        'LEFT': [0, 0],
    },
    'UPPRESSED': {
        'UP': [1, 128],
        'RIGHT': [0, 128],
        'DOWN': [1, 128],
        'LEFT': [0, 128],
    }
}

buttons_down = {
    'X': False,
    'A': False,
    'B': False,
    'Y': False,
    'LB': False,
    'RB': False,
}

sticks_down = {
    'UP': False,
    'RIGHT': False,
    'DOWN': False,
    'LEFT': False,
}

def set_button_downpressed(button, code_button, value_button):
    if BUTTON_CODE_MAPPINGS[button] == code_button:
        buttons_down[button] = value_button == 1

def set_all_buttons_downpressed(code_button, value_button):
    for button in buttons_down:
        set_button_downpressed(button, code_button, value_button)

def set_stick_downpressed(stick, code_stick, value_stick):
    stick_code_mapping_downpressed = STICK_CODE_MAPPINGS['DOWNPRESSED'][stick]
    if stick_code_mapping_downpressed[0] == code_stick and stick_code_mapping_downpressed[1] == value_stick:
        sticks_down[stick] = True

def set_stick_uppressed(stick, code_stick, value_stick):
    stick_code_mapping_uppressed = STICK_CODE_MAPPINGS['UPPRESSED'][stick]
    if stick_code_mapping_uppressed[0] == code_stick and stick_code_mapping_uppressed[1] == value_stick:
        sticks_down[stick] = False

def set_all_sticks_downpressed(code_stick, value_stick):
    for stick in sticks_down:
        set_stick_downpressed(stick, code_stick, value_stick)
        set_stick_uppressed(stick, code_stick, value_stick)
#------------------------------------------------------------------------------#


#------------------------------------------------------------------------------#
class ArmFSM:
    RESET = 0 # fully up
    SHOULDER_RESETTING = 1 # shoulder is moving to reset position
    ELBOW_RESETTING = 2 # elbow is moving to reset position

    PADDLE = 3 # fully down
    SHOULDER_PADDLING = 4 # shoulder is moving to paddle position
    ELBOW_PADDLING = 5 # elbow is moving to paddle position

    def __init__(self, side):
        self.current_state = self.RESET
        self.time_in_state = 0.0
        self.side = side

    def next_state(self, delta, fire):
        self.time_in_state += delta
        if self.current_state == ArmFSM.SHOULDER_RESETTING:
            if self.time_in_state > SHOULDER_DELAY:
                self.current_state = ArmFSM.ELBOW_RESETTING
                self.time_in_state = 0.0
                set_servo_posistion(self.side, 'ELBOW', SERVO_REST_ANGLES)
        elif self.current_state == ArmFSM.ELBOW_RESETTING:
            if self.time_in_state > ELBOW_DELAY:
                self.current_state = ArmFSM.RESET
                self.time_in_state = 0.0
        elif self.current_state == ArmFSM.SHOULDER_PADDLING:
            if self.time_in_state > SHOULDER_DELAY:
                self.current_state = ArmFSM.ELBOW_PADDLING
                self.time_in_state = 0.0
                set_servo_posistion(self.side, 'ELBOW', SERVO_PADDLE_ANGLES)
        elif self.current_state == ArmFSM.ELBOW_PADDLING:
            if self.time_in_state > ELBOW_DELAY:
                self.current_state = ArmFSM.PADDLE
                self.time_in_state = 0.0
        elif self.current_state == ArmFSM.RESET:
            if fire:
                self.current_state = ArmFSM.SHOULDER_PADDLING
                self.time_in_state = 0.0
                set_servo_posistion(self.side, 'SHOULDER', SERVO_PADDLE_ANGLES)
        elif self.current_state == ArmFSM.PADDLE:
            self.current_state = ArmFSM.SHOULDER_RESETTING
            self.time_in_state = 0.0
            set_servo_posistion(self.side, 'SHOULDER', SERVO_REST_ANGLES)

    @classmethod
    def reset(cls, left_state = None, right_state = None):
        print("Fully resetting...")
        if left_state is not None:
            left_state.current_state = cls.PADDLE
            left_state.next_state(0.0, False)
        if right_state is not None:
            right_state.current_state = cls.PADDLE
            right_state.next_state(0.0, False)

        t0 = time.time()
        t1 = time.time()
        delta = 1 / 100

        going = True
        while going:
            
            t1 = time.time()
            delta = t1 - t0
            t0 = t1

            if delta < 0.05:
                time.sleep(0.05 - delta)
                delta = 0.05

            if left_state is not None:
                left_state.next_state(delta, False)
            if right_state is not None:
                right_state.next_state(delta, False)
            
            going = not ((left_state is None or left_state.current_state != cls.RESET) and (right_state is None or right_state.current_state != cls.RESET))

        print("Fully reset")


class LeftRightFSM:
    LEFT_FIRST = 0
    RIGHT_FIRST = 1

    def __init__(self):
        self.current_state = self.LEFT_FIRST

    def next_state(self):
        if self.current_state == self.LEFT_FIRST:
            self.current_state = self.RIGHT_FIRST
        elif self.current_state == self.RIGHT_FIRST:
            self.current_state = self.LEFT_FIRST


left_state = ArmFSM('LEFT')
right_state = ArmFSM('RIGHT')

left_right_state = LeftRightFSM()
#------------------------------------------------------------------------------#

t0 = time.time()
t1 = time.time()
delta = 1 / 100


try:
    print("Starting...")
    ArmFSM.reset(left_state, right_state)
    print("Ready")

    print("Press CTRL+C to end the program.\n")

    while True:

        #----------------------------------------------------------------------#
        t1 = time.time()
        delta = t1 - t0
        t0 = t1
        #----------------------------------------------------------------------#

        #----------------------------------------------------------------------#
        new_button = False
        new_stick = False

        try:
            event = gamepad.read_one()

            eventinfo = evdev.categorize(event)
            if event.type == 1:
                new_button = True
                code_button = eventinfo.scancode
                value_button = eventinfo.keystate
            elif event.type == 3:
                new_stick = True
                code_stick = eventinfo.event.code
                value_stick = eventinfo.event.value
        except:
            pass

        if new_button:
            set_all_buttons_downpressed(code_button, value_button)
        if new_stick:
            set_all_sticks_downpressed(code_stick, value_stick)
        #----------------------------------------------------------------------#

        #----------------------------------------------------------------------#
        if buttons_down['B']:
            ArmFSM.reset(left_state, right_state)
        else:
            print(buttons_down['RB'], buttons_down['LB'])
            if buttons_down['RB'] and not buttons_down['LB']:
                left_state.next_state(delta, True)
                right_state.next_state(delta, False)
            elif buttons_down['LB'] and not buttons_down['RB']:
                right_state.next_state(delta, True)
                left_state.next_state(delta, False)
            else:
                if left_state.current_state == ArmFSM.RESET and right_state.current_state == ArmFSM.RESET:
                    left_right_state.next_state()

                    if left_right_state.current_state == LeftRightFSM.LEFT_FIRST:
                        left_state.next_state(delta, True)
                        right_state.next_state(delta, True)
                    elif left_right_state.current_state == LeftRightFSM.RIGHT_FIRST:
                        right_state.next_state(delta, True)
                        left_state.next_state(delta, True)
                else:
                    if left_right_state.current_state == LeftRightFSM.LEFT_FIRST:
                        left_state.next_state(delta, False)
                        right_state.next_state(delta, False)
                    elif left_right_state.current_state == LeftRightFSM.RIGHT_FIRST:
                        right_state.next_state(delta, False)
                        left_state.next_state(delta, False)
        #----------------------------------------------------------------------#
except:
    ...
finally:
    print("Exiting...")

    try:
        ArmFSM.reset(left_state, right_state)
    except:
        for side in SERVO_CHANNEL_MAPPINGS:
            for joint in SERVO_CHANNEL_MAPPINGS[side]:
                set_servo_posistion(side, joint, SERVO_REST_ANGLES)        

    GPIO.cleanup()
    gamepad.close()
    print("\nDone!")