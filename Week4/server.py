from flask import Flask, render_template, Response
import json

FILE = "state.json"

app=Flask(__name__)

state = {
	"speed": 50,
	"manual": False,
	"angle1": -1,
	"angle2": -1,
	"power": False
}

@app.route('/')
def index():
	# Return html page with control inputs
	return render_template('index.html')

@app.route('/set/speed/<speed>')
def speed(speed):
	# Set speed
	return

@app.route('/set/manual/<manual>')
def manual(manual):
	# Take manual control of anlges (True/False)
	# Override state machine
	return

@app.route('/set/angle1/<angle1>')
def angle1(angle1):
	# Set angle1, only if manual control is True
	return

@app.route('/set/angle2/<angle2>')
def angle2(angle2):
	# Set angle2, only if manual control is True
	return

@app.route('/set/power/<power>')
def power(power):
	# Set power (True/False)
	state["power"] = power
	write_state()
	return "OK"

@app.route('/get')
def get():
	# Return state as json
	read_state()
	return Response(json.dumps(state), mimetype='application/json')

def write_state():
	# Write state to file
	with open(FILE, "w") as f:
		json.dump(state, f)
	return

def read_state():
	# Read state from file
	global state
	with open(FILE, "r") as f:
		state = json.load(f)
	return

if __name__ == "__main__":
	app.run(debug=True, port=5000, host='0.0.0.0')
