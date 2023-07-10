# Day 1 - July 10

- RPI = Raspberry Pi
	- Pins are named by 2 styles
		- GPIO.BOARD: numbers 1-40
		- GPIO.BCM: Broadcom SOC channel
	- All ground pins on the board are connected (and equivalent)
	- Some pins have alternate functions
- Large ribon cable bends away from the board
- Breadboard
	- Edge 2 rows are all connected
		- The ones near the blue/red lines
			- Connected parallel to the blue/red lines
		- The one near the blue line: by convention, ground
	- Middle sections are connected vertically (perpendicular to the blue/red lines)
		- But not through the middle dent
	- Use black wires for ground
	- Use red wires for power

## Non-blocking

```python
import time

start = time.time()
interval = 2 # seconds

# use walrus operator
while (elapsed := time.time() - start) < interval:
	print(f"Elapsed: {elapsed:.2f} seconds")
```

## (Mealy) FSM

- FSM = Finite State Machine
	- Mealy: output depends on state and input
	- Moore: output depends only on state
- `[state] -> input/output -> [new state]`
- Many states and connections between them

## Examples

### Example 1

```python
state = 0 # some number: [0, 1]
input = None # some input: ['A', None]

while True:
	if state == 0:
		if input == 'A':
			state = 1
			print("Going to state 1")
	elif state == 1:
		if input == 'A':
			state = 0
			print("Going to state 0")
```

## Example 2

```python
import time

state = 0 # some number: [0, 1]
input = None # some input: ['A', None]

start = time.time()
delayTime = 4 # seconds

while True:
	if state == 0:
		if input == 'A':
			state = 1
			start = time.time()
			print("Going to state 1")
		if (now := time.time()) - start >= delayTime:
			state = 1
			start = now
			print("Going to state 1 after timeout")
	elif state == 1:
		if input == 'A':
			state = 0
			print("Going to state 0")
		if (now := time.time()) - start >= delayTime:
			state = 0
			start = now
			print("Going to state 0 after timeout")
		
```

## GPIO pins for motors

Link: https://drive.google.com/file/d/1-IVCpX_PRMTzjBmC6w4_H_NcGjcapAHK/view