import evdev
import RPi.GPIO as GPIO
import adafruit_servokit
import time


kit = adafruit_servokit.ServoKit(channels=16)

gamepad = evdev.InputDevice("/dev/input/event0")
print(gamepad)



#------------------------------------------------------------------------------#
SERVO_SHOULDERS = ['LEFT_SHOULDER', 'RIGHT_SHOULDER']
SERVO_ELBOWS = ['LEFT_ELBOW', 'RIGHT_ELBOW']

SERVO_CHANNEL_MAPPINGS = {
    'LEFT_SHOULDER': 0,
    'RIGHT_SHOULDER': 1,
    'LEFT_ELBOW': 2,
    'RIGHT_ELBOW': 3,
}

SERVO_REST_ANGLES = {
    'LEFT_SHOULDER': 180,
    'RIGHT_SHOULDER': 0,
    'LEFT_ELBOW': 180,
    'RIGHT_ELBOW': 0,
}

SERVO_PADDLE_ANGLES = {
    'LEFT_SHOULDER': 110,
    'RIGHT_SHOULDER': 60,
    'LEFT_ELBOW': 180,
    'RIGHT_ELBOW': 0,
}

SHOULDER_DELAY = 0.5
ELBOW_DELAY = 0.5

def set_servo_posistions(positions):
    global kit, SERVO_SHOULDERS, SERVO_ELBOWS, SERVO_CHANNEL_MAPPINGS
    for servo in SERVO_SHOULDERS:
        kit.servo[SERVO_CHANNEL_MAPPINGS[servo]].angle = positions[servo]
    time.sleep(SHOULDER_DELAY)
    for servo in SERVO_ELBOWS:
        kit.servo[SERVO_CHANNEL_MAPPINGS[servo]].angle = positions[servo]
    time.sleep(ELBOW_DELAY)

def reset_servo_posistions():
    set_servo_posistions(SERVO_REST_ANGLES)

def paddle_servo_posistions():
    set_servo_posistions(SERVO_PADDLE_ANGLES)
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
class FSM:
    RESET = 0
    PADDLE = 1

    @classmethod
    def get_next_state(cls, current_state):
        if current_state == cls.RESET:
            return cls.PADDLE
        elif current_state == cls.PADDLE:
            return cls.RESET
    
    @classmethod
    def get_state_name(cls, state):
        if state == cls.RESET:
            return 'RESET'
        elif state == cls.PADDLE:
            return 'PADDLE'

currentState = FSM.RESET    
#------------------------------------------------------------------------------#

print("Starting...")
reset_servo_posistions()
print("Ready!")

t0 = time.time()
t1 = time.time()
delta = 1 / 100

print("Press CTRL+C to end the program.\n")

try:
    # raise Exception("This is a test exception")
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
            # for event in gamepad.read():  
            # Use this option (and comment out the next line) to react to the latest event only
            # Use this option (and comment out the previous line) when you don't want to miss any event
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
        if currentState == FSM.PADDLE:
            reset_servo_posistions()
        elif currentState == FSM.PADDLE:
            paddle_servo_posistions()
        currentState = FSM.get_next_state(currentState)
        #----------------------------------------------------------------------#
except:
    ...
finally:
    print("Exiting...")
    reset_servo_posistions()

    GPIO.cleanup()
    gamepad.close()
    print("\nDone")