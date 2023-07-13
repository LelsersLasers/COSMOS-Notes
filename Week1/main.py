"""
    Simple movement code
    Controls:
    - RB + LB: forward
    - RB: right
    - LB: left
    - B: hold for straight backwards
    - Y: hold for high power
    - X: back arm posistion toggle
    - A: hold for snap front arm down
    - Up/Down: change front arm resting posistion
"""

# Libraries
import evdev
import RPi.GPIO as GPIO
import adafruit_servokit
import time


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

SERVO_CHANNEL_MAPPINGS = {
    'BACK': 0,
    'FRONT': 1,
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

# GPIO Mode (BOARD / BCM)
# GPIO.setmode(GPIO.BOARD)
 
# set GPIO Pins
# GPIO_Ain1 = 11
# GPIO_Ain2 = 13
# GPIO_Apwm = 15
# GPIO_Bin1 = 29
# GPIO_Bin2 = 31
# GPIO_Bpwm = 33

GPIO.setmode(GPIO.BCM)
GPIO_Ain1 = 17
GPIO_Ain2 = 27
GPIO_Apwm = 22
GPIO_Bin1 = 5
GPIO_Bin2 = 6
GPIO_Bpwm = 13

# Set GPIO direction (IN / OUT)
GPIO.setup(GPIO_Ain1, GPIO.OUT)
GPIO.setup(GPIO_Ain2, GPIO.OUT)
GPIO.setup(GPIO_Apwm, GPIO.OUT)
GPIO.setup(GPIO_Bin1, GPIO.OUT)
GPIO.setup(GPIO_Bin2, GPIO.OUT)
GPIO.setup(GPIO_Bpwm, GPIO.OUT)

# Both motors are stopped 
GPIO.output(GPIO_Ain1, False)
GPIO.output(GPIO_Ain2, False)
GPIO.output(GPIO_Bin1, False)
GPIO.output(GPIO_Bin2, False)

# Set PWM parameters
pwm_frequency = 50

# Create the PWM instances
pwmA = GPIO.PWM(GPIO_Apwm, pwm_frequency)
pwmB = GPIO.PWM(GPIO_Bpwm, pwm_frequency)

# Set the duty cycle (between 0 and 100)
# The duty cycle determines the speed of the wheels
pwmA.start(0)
pwmB.start(0)


kit = adafruit_servokit.ServoKit(channels=16)


gamepad = evdev.InputDevice("/dev/input/event0")
print(gamepad)
#------------------------------------------------------------------------------#
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


# ENUM, values aren't relevant
class Direction:
    FORWARD = 0
    BACKWARD = 1

# in degrees
class BackArmPosition:
    UP = 180.0
    DOWN = 15.0
    DEGREES_PER_SECOND = 60.0

class BackArmStatus:
    UP = 0
    DOWN = 1
    MOVING_UP = 2
    MOVING_DOWN = 3

    @classmethod
    def __str__(cls, back_arm_status):
        if back_arm_status == cls.UP:
            return "UP"
        elif back_arm_status == cls.DOWN:
            return "DOWN"
        elif back_arm_status == cls.MOVING_UP:
            return "MOVING_UP"
        elif back_arm_status == cls.MOVING_DOWN:
            return "MOVING_DOWN"

# in degrees
class FrontArmPosition:
    SNAP_DOWN = 180.0
    MINIMUM = 180.0
    MAXIMUM = 45.0
    DEGREES_PER_SECOND = 180.0
    SNAP_DOWN_TIMER = 0.75

# duty cycles
class Power:
    LOW = 50
    HIGH = 100
#------------------------------------------------------------------------------#

direction = Direction.FORWARD
power = Power.HIGH

back_arm_position = BackArmPosition.UP
front_arm_position = FrontArmPosition.MAXIMUM

front_arm_timer = 0.0

back_arm_status = BackArmStatus.UP

kit.servo[SERVO_CHANNEL_MAPPINGS['BACK']].angle = back_arm_position
kit.servo[SERVO_CHANNEL_MAPPINGS['FRONT']].angle = front_arm_position


t0 = time.time()
t1 = time.time()
delta = 1 / 100

print("Press CTRL+C to end the program.\n")

try:

    looping = True
    while looping:

        t1 = time.time()
        delta = t1 - t0
        t0 = t1

        front_arm_timer -= delta

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


        direction = Direction.BACKWARD if buttons_down['B'] else Direction.FORWARD
        power = Power.HIGH if buttons_down['Y'] else Power.LOW

        # only trigger on downpress
        if new_button and value_button == 1:
            if code_button == BUTTON_CODE_MAPPINGS['X']:
                if back_arm_status == BackArmStatus.UP:
                    back_arm_status = BackArmStatus.MOVING_DOWN
                elif back_arm_status == BackArmStatus.DOWN:
                    back_arm_status == BackArmStatus.MOVING_UP

        if sticks_down['UP']:
            front_arm_position -= FrontArmPosition.DEGREES_PER_SECOND * delta
            if front_arm_position < FrontArmPosition.MAXIMUM:
                front_arm_position = FrontArmPosition.MAXIMUM
        if sticks_down['DOWN']:
            front_arm_position += FrontArmPosition.DEGREES_PER_SECOND * delta
            if front_arm_position > FrontArmPosition.MINIMUM:
                front_arm_position = FrontArmPosition.MINIMUM

        if back_arm_status == BackArmStatus.MOVING_DOWN:
            back_arm_position -= BackArmPosition.DEGREES_PER_SECOND * delta
            if back_arm_position <= BackArmPosition.DOWN:
                back_arm_position = BackArmPosition.DOWN
                back_arm_status = BackArmStatus.DOWN
        elif back_arm_status == BackArmStatus.MOVING_UP:
            back_arm_position += BackArmPosition.DEGREES_PER_SECOND * delta
            if back_arm_position >= BackArmPosition.UP:
                back_arm_position = BackArmPosition.UP
                back_arm_status = BackArmStatus.UP

        print(BackArmStatus.__str__(back_arm_status))

        kit.servo[SERVO_CHANNEL_MAPPINGS['BACK']].angle = back_arm_position

        if buttons_down['A']:
            front_arm_timer = FrontArmPosition.SNAP_DOWN_TIMER

        if front_arm_timer >= 0.0:
            kit.servo[SERVO_CHANNEL_MAPPINGS['FRONT']].angle = FrontArmPosition.SNAP_DOWN
        else:
            kit.servo[SERVO_CHANNEL_MAPPINGS['FRONT']].angle = front_arm_position

        if direction == Direction.FORWARD:
            GPIO.output(GPIO_Ain1, True)
            GPIO.output(GPIO_Ain2, False)
            GPIO.output(GPIO_Bin1, True)
            GPIO.output(GPIO_Bin2, False)

            if buttons_down['RB']: pwmA.ChangeDutyCycle(power)
            else: pwmA.ChangeDutyCycle(0)
            
            if buttons_down['LB']: pwmB.ChangeDutyCycle(power)
            else: pwmB.ChangeDutyCycle(0)
        elif direction == Direction.BACKWARD:
            GPIO.output(GPIO_Ain1, False)
            GPIO.output(GPIO_Ain2, True)
            GPIO.output(GPIO_Bin1, False)
            GPIO.output(GPIO_Bin2, True)

            pwmA.ChangeDutyCycle(power)
            pwmB.ChangeDutyCycle(power)


# Quit the program when the user presses CTRL + C
except KeyboardInterrupt:
    print("Exiting...")

    try:
        # TODO: time.sleep()s don't really work
        while back_arm_position != BackArmPosition.UP:
            back_arm_position += BackArmPosition.DEGREES_PER_SECOND / 4
            if back_arm_position > BackArmPosition.UP:
                back_arm_position = BackArmPosition.UP
            kit.servo[SERVO_CHANNEL_MAPPINGS['BACK']].angle = BackArmPosition.UP

            time.sleep(0.5)
        
        time.sleep(2.0)

        kit.servo[SERVO_CHANNEL_MAPPINGS['FRONT']].angle = FrontArmPosition.MAXIMUM
    finally:
        try:
            kit.servo[SERVO_CHANNEL_MAPPINGS['BACK']].angle = BackArmPosition.UP
            time.sleep(3.0)
            kit.servo[SERVO_CHANNEL_MAPPINGS['FRONT']].angle = FrontArmPosition.MAXIMUM
        finally:
            gamepad.close()
            pwmA.stop()
            pwmB.stop()
            GPIO.cleanup()

            print("Done!")