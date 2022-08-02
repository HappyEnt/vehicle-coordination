import math
import random
import sys
from typing import Dict, Tuple
import numpy as np
from scipy.spatial import distance
from scipy.stats import norm
import threading
import uuid
import matplotlib.pyplot as plt

import time
import configparser
import requests
import json
import ast
import os

config = configparser.ConfigParser()
config.read("./config.ini")

SIDE_LENGTH_X = float(config['DEFAULT']['testing_area_length'])  # 10x10 m^2
SIDE_LENGTH_Y = float(config['DEFAULT']['testing_area_width']) # 10x10 m^2

NUM_PARTICLES = 1000 # number of particles
NUM_PART_EXCHANGE = 500  # The amount of particles that actually gets transmitted to the other nodes

PERCENTAGE_FOR_GUESS = 0.95 # amount of particles in %, which are included in the estimation radius

# error in distance measurements in m
MEASUREMENT_STDEV = 2

SERVER = "http://192.168.87.78:8081"

VISUALISATION = False


INT_ID = 4

# Particle nodes try to estimate their position using particles
class ParticleNode:
    '''
    Class for dynamic nodes.
    '''
    def __init__(self) -> None:
        self.uuid = uuid.uuid4() # give this node a uuid
        car_config = configparser.ConfigParser()
        car_config.read("./car_config.ini")                 # config for the car (holds id and size of car)
        self.id_int = INT_ID
        self.car_size_width = int(car_config['CAR']['width'])
        self.car_size_length = int(car_config['CAR']['length'])
        self.car_radius = (math.sqrt(self.car_size_width ** 2 + self.car_size_length ** 2)) / 2 # convert a rectangle-shaped car into a single radius
        self.particles = [] # we assume that the weights are equal for all particles
        self.last_movement_update = time.time()
        self.other_nodes_pos : Dict[str, Tuple[float, float, float]] = {} # positions of other nodes, key is the uuid of the other nodes
        # we initialize the particles with uniform random samples of the whole area
        for i in range(NUM_PARTICLES):
            p = (np.random.uniform(low=[0.0,0.0], high=[SIDE_LENGTH_X, SIDE_LENGTH_Y], size=2))
            self.particles.append(p)
        self.particles = self.particles

    def get_estimate(self):
        '''
        Returns the current estimated position of this node. This includes its uuid, a point and a radius.
        '''
        estimate_x = np.mean([p[0] for p in self.particles])
        estimate_y = np.mean([p[1] for p in self.particles])

        # calculate radius
        distances = []
        for p in self.particles:
            distances.append(distance.euclidean(p, [estimate_x,estimate_y]))

        dist_std = 2 * np.std(distances)
        guess_radius = dist_std + np.mean(distances)
        return (self.id_int, [estimate_x, estimate_y], guess_radius, self.car_radius)

    def get_particles(self):
        return self.particles

    def set_position_others(self, id_int, value):
        self.other_nodes_pos[id_int] = value

    def get_particles_for_exchange(self):
        # return random particles from particle list for exchange
        return random.choices(
            population = self.particles,
            k = NUM_PART_EXCHANGE
        )

    def handle_measurement(self, d, recv_particles, estimate_from_other) -> None:
        '''
        Handle incoming measurements by updating own particles.
        '''
        # save position of other node using their uuid as key
        # self.other_nodes_pos[estimate_from_other[0]] = (estimate_from_other[1],estimate_from_other[2])
        
        for (i, p) in enumerate(self.particles):
            noisy_p = (np.random.normal(p[0], MEASUREMENT_STDEV), np.random.normal(p[1], MEASUREMENT_STDEV))
            self.particles[i] = noisy_p
            continue

        # we update our particles and resample them directly
        # first initialize the weights -> we assume an equal weight for each particle
        # this means also that if particles had a bigger weight, they are just multiple times in the particles list
        start = time.time()
        weights = [1.0 / NUM_PARTICLES] * NUM_PARTICLES
        for (i, p) in enumerate(self.particles):
            weight_factor = 0.0
            norm_vals = []
            for rp in recv_particles:
                # estimate the probability P( p1, p2 | d)
                # P( p1, p2 | d) = P( d | p1, p2) * (P(p1, p2) / P(d))
                # we just assume that P(p1, p2) and P(d) are uniform and therefore all particles share this as the same factor
                # as we normalize the weights, we can ignore this factor and can just use P( d | p1, p2) which we can easily compute
                # expected_d = distance.euclidean(p, rp)
                expected_d = math.sqrt((p[0] - rp[0]) ** 2 + (p[1] - rp[1]) ** 2)
                actual_measured_d = d
                # normalize the value using the mean ("expected") and the standard deviation
                norm_val = (actual_measured_d-expected_d) / MEASUREMENT_STDEV
                norm_vals.append(norm_val) # collect values and compute later, to drastically increase performance of the algorithm
            probabilities = norm.pdf(norm_vals)
            weight_factor += sum(probabilities)
            weights[i] *= weight_factor

        sum_weights = sum(weights)
        normalized_weights = [x/sum_weights for x in weights]

        # resample so that the weights are approximately uniform (w_i = 1 / NUM_PARTICLES)
        self.particles = random.choices(
            population = self.particles,
            weights = normalized_weights,
            k = len(self.particles)
        )
        end = time.time() - start
        print("Time elapsed: " + str(end))


    def get_measurements_from_server(self):
        positions = ast.literal_eval(requests.get(SERVER + "/positions").text)
        positions.pop(str(self.id_int), None)
        anchors = ast.literal_eval(requests.get(SERVER + "/anchors").text)
        all_pos = {**positions, **anchors}
        # print(all_pos)
        all_keys = list(all_pos.keys())
        other_marker_id = all_keys[random.randint(0, len(all_keys) - 1)]
        dista = json.loads(requests.get(SERVER + "/distance/" + str(self.id_int) + "/" + str(other_marker_id)).text)
        other_pos = json.loads(requests.get(SERVER + "/position/" + str(other_marker_id)).text)
        particles = json.loads(requests.get(SERVER + "/getparticles/" + str(other_marker_id)).text)
        return other_marker_id, float(dista), ast.literal_eval(other_pos), particles

    def send_particles_to_server(self):
        dict1 = {}
        dict1["marker_id"] = self.id_int
        particles = self.get_particles_for_exchange()
        dict1["particles"] = list(particles)
        json1 = json.dumps(dict1)
        postr = requests.post(SERVER + "/setparticles", json=json1).text

if __name__ == "__main__":
    # read command line parameters to update particles
    if len(sys.argv) > 1:
        INT_ID = int(sys.argv[1])
    else:
        car_config = configparser.ConfigParser()
        car_config.read("./car_config.ini")                 # config for the car (holds id and size of car)
        INT_ID = int(car_config['CAR']['marker_id'])   # id of this cars
        # NUM_PARTICLES = int(sys.argv[1])
        # NUM_PART_EXCHANGE = int(sys.argv[1])

    node = ParticleNode()

    time.sleep(3)

    print("Starting localization with ID " + str(node.id_int) + " and " + str(NUM_PARTICLES) + " particles...")
    my_pos = ast.literal_eval(json.loads(requests.get(SERVER + "/position/" + str(node.id_int), timeout=10).text))

    def illustrate_nodes_and_particles(real_pos, estimate=(-100,-100)):
        plt.clf()
        pos = real_pos
        plt.scatter([pos[0]], [pos[1]], 100, marker="x", color="g")
        plt.scatter([estimate[0]], [estimate[1]], 100, marker="x", color="r")
        # we then scatter its particles
        particles = node.get_particles()
        plt.scatter(np.array([p[0] for p in particles]), np.array([p[1] for p in particles]), 25, alpha=0.05)
        plt.xlim([0, SIDE_LENGTH_X])
        plt.ylim([0, SIDE_LENGTH_Y])
        plt.gca().invert_yaxis()
        # plt.pause(0.5)
        plt.show()

    if VISUALISATION:
        illustrate_nodes_and_particles(my_pos)

    while (True):
        id_int, dista, other_position, particles = node.get_measurements_from_server()
        print("Got Measurement with " + str(id_int) + " with distance " + str(dista) + " and position of other node: " + str(other_position))
        try:
            node.handle_measurement(dista, particles, (id_int, other_position[0], other_position[1], 12))
        except ZeroDivisionError:
            continue
        print("Real position: " + str(my_pos))
        estimate = node.get_estimate()
        estimated_pos = (estimate[1], estimate[2])
        node.send_particles_to_server()
        print(estimate)
        print("Done with measurement, error now: " + str(distance.euclidean(estimated_pos, my_pos)))
        time.sleep(random.random())
        if VISUALISATION:
            illustrate_nodes_and_particles(my_pos, estimated_pos)