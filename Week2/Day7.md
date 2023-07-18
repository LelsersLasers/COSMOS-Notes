# Day 7 - July 18

## Control (Motion planning)

- Feedback to stabilize the behavior of a dynamic system
- Can use difference between 2 frames to determine the motion of the robot
- Can use vertical (y) error (different) to determine forward speed
	- Avoid overshooting
	- Drive slower when closer to target
- Requires:
	- Sensor
	- Control input (ex: wheel speed)
	- "Plant" (ex: robot)
- Set point: desired state (ex: post it note in center of camera)
- Error: difference between set point and current state
- PID control
	- P: Proportional
		- `P = error * Kp`
		- How far away from the target
	- I: Integral
		- `I[i] = I[i-1] + Ki * error * dt`
			- `dt` = delta time
		- How much error has accumulated over time
		- Helps with steady state error
	- D: Derivative
		- `D = Kd * (error - previous_error) / dt`
		- How much the error is changing
		- Helps stabilize and avoid overshooting
	- `C = P + I + D`
		- Control input