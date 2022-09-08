'''
This program provides a webserver to request data from the camera
and allows for some communication between the vehicles.
@author: torben
'''
from flask import Flask, jsonify, request
from markupsafe import escape
import json

connector = None
def run(connector_obj):
    global connector
    connector = connector_obj
    app.run(host="0.0.0.0", port=8081)


app = Flask(__name__)


@app.route("/")
def hello_world():
    return "Server for the camera system"

@app.route('/position/<int:marker_id>')
def show_position(marker_id):
    # shows the position of a car/marker
    return jsonify(connector.get_one_pos(marker_id))

@app.route('/positions')
def show_positions():
    # shows the position of a car/marker
    return jsonify(connector.cam_data)

@app.route('/distance/<int:marker_id_1>/<int:marker_id_2>')
def show_distance(marker_id_1, marker_id_2):
    # shows the distance from a car/marker to all other markers
    return jsonify(connector.distance(marker_id_1, marker_id_2))

@app.route('/anchors')
def show_anchors():
    # shows the distance from a car/marker to all other markers
    return jsonify(connector.anchors)

@app.route('/setparticles/', methods = ['POST'])
def set_particles():
    if request.method == 'POST':
        # .get('marker_id', type=int)
        json1 = json.loads(request.get_json())
        marker_id = json1["marker_id"]
        particles = json1["particles"]
        print("Received particles from marker " + str(marker_id))
        connector.receive_particles(marker_id, particles)
    # shows the distance from a car/marker to all other markers
    return jsonify("")

@app.route('/getparticles/<int:marker_id>')
def get_particles(marker_id):
    # shows the distance from a car/marker to all other markers
    val = connector.get_saved_particles(marker_id)
    return jsonify(val)

@app.route('/getparticles')
def get_all_particles():
    # shows the distance from a car/marker to all other markers
    val = connector.particle_storage
    return jsonify(val)

@app.route('/clearparticles/')
def clear_particles():
    # shows the distance from a car/marker to all other markers
    connector.clear_particles()
    return jsonify({})

@app.route('/setestimate/', methods = ['POST'])
def set_estimate():
    if request.method == 'POST':
        json_request = request.get_json()
        # .get('marker_id', type=int)
        # json1 = json.loads(json_request)
        if json_request:
            json1 = json.loads(json_request)
            marker_id = json1["marker_id"]
            estimate = json1["estimate"]
            print("Received estimate from marker " + str(marker_id))
            connector.receive_estimate(marker_id, estimate)
    # shows the distance from a car/marker to all other markers
    return jsonify("")

@app.route('/getestimate/<int:marker_id>')
def get_estimate(marker_id):
    # shows the distance from a car/marker to all other markers
    val = connector.get_saved_estimate(marker_id)
    return jsonify(val)

@app.route('/getestimate')
def get_all_estimates():
    # shows the distance from a car/marker to all other markers
    val = connector.estimate_storage
    return jsonify(val)

@app.route('/setdata', methods = ['POST'])
def set_data():
    '''
    Save the provided data on the server.
    '''
    if request.method == 'POST':
        json_request = request.get_json()
        if json_request:
            json1 = json.loads(json_request)
            data_keyword = json1["data_keyword"]
            data = json1["data"]
        # fill the data storage with information
        return jsonify(connector.set_data(data_keyword, data))
    return jsonify("")

@app.route('/getdata/<data_keyword>')
def get_data(data_keyword):
    # shows the distance from a car/marker to all other markers
    return jsonify(connector.get_data(data_keyword))
