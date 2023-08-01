from flask import Flask, render_template, Response
import json
import time

FILE = "state.json"

app=Flask(__name__)

state = {
	"speed": 50,
	"angle1": 75.0,
	"angle2": 90.0,
	"state": "off"
}


@app.route('/')
def index():
	return render_template('index.html')


@app.route('/set/speed/<speed>')
def speed(speed):
	state["speed"] = speed
	write_state()
	return "OK"

@app.route('/set/angle1/<angle1>')
def angle1(angle1):
	state["angle1"] = angle1
	write_state()
	return "OK"

@app.route('/set/angle2/<angle2>')
def angle2(angle2):
	state["angle2"] = angle2
	write_state()
	return "OK"

@app.route('/set/state/<new_state>')
def set_state(new_state):
	state["state"] = new_state
	write_state()

	return "OK"


@app.route('/get')
def get():
	read_state()
	return Response(json.dumps(state), mimetype='application/json')



def write_state():
	with open(FILE, "w") as f:
		json.dump(state, f)
	return

def read_state():
	global state
	try:
		with open(FILE, "r") as f:
			state = json.load(f)
	except:
		time.sleep(0.1)
		read_state()



if __name__ == "__main__":
	app.run(debug=True, port=5000, host='0.0.0.0')
