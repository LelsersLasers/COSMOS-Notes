"""
    Simple movement code
    Controls:
    - RB: Right motor
    - LB: Left motor
    - B: straight backwards
    - A: toggle between high and low power
"""

# Libraries
import evdev
import RPi.GPIO as GPIO
import time


BUTTON_CODE_MAPPINGS = {
    'X': 304,
    'A': 305,
    'B': 306,
    'Y': 307,
    'LB': 308,
    'RB': 309,
}

buttons_down = {
    'X': False,
    'A': False,
    'B': False,
    'Y': False,
    'LB': False,
    'RB': False,
}

# GPIO Mode (BOARD / BCM)
GPIO.setmode(GPIO.BOARD)
 
# set GPIO Pins
GPIO_Ain1 = 11
GPIO_Ain2 = 13
GPIO_Apwm = 15
GPIO_Bin1 = 29
GPIO_Bin2 = 31
GPIO_Bpwm = 33

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
#------------------------------------------------------------------------------#
def set_button_downpressed(button, code_button, value_button):
    if BUTTON_CODE_MAPPINGS[button] == code_button:
        buttons_down[button] = value_button == 1

def set_all_buttons_downpressed(code_button, value_button):
    for button in buttons_down:
        set_button_downpressed(button, code_button, value_button)


def close():
    gamepad.close()
    pwmA.stop()
    pwmB.stop()
    GPIO.cleanup()

# ENUM basically
class Direction:
    FORWARD = 0
    BACKWARD = 1

# ENUM but uses values
class Power:
    LOW = 50
    HIGH = 100
#------------------------------------------------------------------------------#


# TODO: By id?
gamepad = evdev.InputDevice("/dev/input/event0")
print(gamepad)
print()


# FSM1State = 0
# FSM1NextState = 0

direction = Direction.FORWARD
power = Power.HIGH

# FSM1LastTime = time.time()

print("Press CTRL+C to end the program.\n")

# Main code
try:

    noError = True
    while noError:
        # currentTime = time.time()

        new_button = False

        # newstick = False
        try:
            # Use this option (and comment out the next line) to react to the latest event only
            # Use this option (and comment out the previous line) when you don't want to miss any event
            event = gamepad.read_one()

            # for event in gamepad.read():  
            eventinfo = evdev.categorize(event)
            if event.type == 1:
                new_button = True
                code_button = eventinfo.scancode
                value_button = eventinfo.keystate
                # elif event.type == 3:
                #     newstick = True
                #     codestick = eventinfo.event.code
                #     valuestick = eventinfo.event.value
        except:
            pass

        if new_button:
            set_all_buttons_downpressed(code_button, value_button)

        direction = Direction.BACKWARD if buttons_down['B'] else Direction.FORWARD
        if new_button and code_button == BUTTON_CODE_MAPPINGS['A'] and value_button == 1:
            power = Power.HIGH if power == Power.LOW else Power.LOW

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

        
        # Update the state
        # FSM1State = FSM1NextState

# Quit the program when the user presses CTRL + C
except KeyboardInterrupt:
    print("Exiting...")
    close()
