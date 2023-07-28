# Libraries
import RPi.GPIO as GPIO
import time
 
# GPIO Mode (BOARD / BCM)
GPIO.setmode(GPIO.BOARD)
 
# set GPIO Pins
GPIO_Servo = 7

# set GPIO direction (IN / OUT)
GPIO.setup(GPIO_Servo, GPIO.OUT)



# Set PWM parameters
pwm_frequency = 50
duty_min = 5 * float(pwm_frequency) / 50.0
duty_max = 10 * float(pwm_frequency) / 50.0
duty_A = 0.5*(duty_max + duty_min)
duty_B = 0.5*(duty_max - duty_min)
print(duty_A,'  ',duty_B)

# Set the duty cycle
def set_duty_speed(speed):
    #print('--> ',speed,'  ',duty_A + duty_B*float(speed))
    return (duty_A + duty_B*float(speed))

    
# Create a PWM instance
pwm_servo = GPIO.PWM(GPIO_Servo, pwm_frequency)

# Set the speed (between -1 and 1)
pwm_servo.start(0)


print("Press CTRL+C to end the program.")
 
# Main program 
try:
        
        noError = True
        while noError:
            
            speed = 0
            pwm_servo.ChangeDutyCycle(0)    # Clean way to get a full stop
            print("Speed: "+str(speed))
            time.sleep(1)
            
            speed = 0.5
            pwm_servo.ChangeDutyCycle(set_duty_speed(speed))
            print("Speed: "+str(speed))
            time.sleep(1)
            
            speed = 1
            pwm_servo.ChangeDutyCycle(set_duty_speed(speed))
            print("Speed: "+str(speed))
            time.sleep(1)

            speed = -1
            pwm_servo.ChangeDutyCycle(set_duty_speed(speed))
            print("Speed: "+str(speed))
            time.sleep(1)

            speed = -0.5
            pwm_servo.ChangeDutyCycle(set_duty_speed(speed))
            print("Speed: "+str(speed))
            time.sleep(1)
            

        # Clean up GPIO if there was an error
        GPIO.cleanup()
        pwm_servo.stop()


        
# Quit the program when the user presses CTRL + C
except KeyboardInterrupt:
        GPIO.cleanup()                          # Release resource
        pwm_servo.stop()
