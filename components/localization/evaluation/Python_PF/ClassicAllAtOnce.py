from logging import debug, info, warning
import math
import random
from time import sleep, time
from typing import Dict, List, NoReturn
import os
import requests
import ast

import numpy as np
from scipy.stats import norm
import matplotlib.pyplot as plt

from filtering.AbstractLocalizationNode import NUM_PARTICLES, SIDE_LENGTH_X, SIDE_LENGTH_Y, MEASUREMENT_STDEV
from filtering.BaseParticleNode import BaseParticleNode

dirname = os.path.dirname(__file__)

SERVER_2 = "http://192.168.87.78:8081"

class ClassicAllAtOnce(BaseParticleNode):
    """
    Class for dynamic nodes.
    """

    def __init__(self) -> None:
        super().__init__()
        # we initialize the particles with uniform random samples of the whole area
        for _ in range(NUM_PARTICLES):
            p = np.random.uniform(
                low=[0, 0],
                high=[SIDE_LENGTH_X, SIDE_LENGTH_Y],
                size=2
                # low=[0.0, 0.0], high=[SIDE_LENGTH_X, SIDE_LENGTH_Y], size=2
            )
            self.particles.append(p)
        self.weights = [1.0 / NUM_PARTICLES] * NUM_PARTICLES

    def reset_particles(self):
        self.particles = []
        for _ in range(NUM_PARTICLES):
            p = np.random.uniform(
                low=[0, 0],
                high=[SIDE_LENGTH_X, SIDE_LENGTH_Y],
                size=2
                # low=[0.0, 0.0], high=[SIDE_LENGTH_X, SIDE_LENGTH_Y], size=2
            )
            self.particles.append(p)

    def handle_measurement(
        self, d, recv_particles_arr, estimate_from_other_arr
    ) -> None:
        """
        Handle incoming measurements by updating own particles.
        """
        # save position of other node using their id as key
        for i in estimate_from_other_arr:
            if i:
                self.other_nodes_pos[i[0]] = i

        info("Handling measurement")
        for (i, p) in enumerate(self.particles):
            self.particles[i] = (
                np.random.normal(p[0], MEASUREMENT_STDEV),
                np.random.normal(p[1], MEASUREMENT_STDEV),
            )
        # we update our particles and resample them directly
        # first initialize the weights -> we assume an equal weight for each particle
        # this means also that if particles had a bigger weight, they are just multiple times in
        # the particles list

        start = time()

        weights = [1.0 / NUM_PARTICLES] * NUM_PARTICLES
        # list of weights for all current measurements
        weights_current_measurem: List[List[float]] = []

        norm_vals_all_list = []
        norm_vals_decoder = []
        for (i1, d) in enumerate(d):
            recv_particles = recv_particles_arr[i1]
            norm_vals_current_measure = []
            norm_vals_decoder_inner = []
            for (i, p) in enumerate(self.particles):
                weight_factor = 0.0
                norm_vals = []
                for rp in recv_particles:
                    # estimate the probability P( p1, p2 | d)
                    # P( p1, p2 | d) = P( d | p1, p2) * (P(p1, p2) / P(d))
                    # we just assume that P(p1, p2) and P(d) are uniform and therefore all particles
                    #  share this as the same factor
                    # as we normalize the weights, we can ignore this factor and can just use
                    # P( d | p1, p2) which we can easily compute
                    # expected_d = distance.euclidean(p, rp)
                    expected_d = math.sqrt((p[0] - rp[0]) ** 2 + (p[1] - rp[1]) ** 2)
                    actual_measured_d = d
                    # normalize the value using the mean ("expected") and the standard deviation
                    norm_val = (actual_measured_d - expected_d) / MEASUREMENT_STDEV
                    norm_vals.append(norm_val)
                    # collect values and compute later, to drastically increase performance of the
                    # algorithm (REALLY: DRASTICALLY) because calling norm.pdf on an array rather
                    # than just from single values is WAY FASTER
                    norm_vals_all_list.append(norm_val)

                norm_vals_current_measure.append(norm_vals)
                norm_vals_decoder_inner.append(len(norm_vals))
                debug(f"Processing particle {p} new weight factor: {weight_factor}")

            norm_vals_decoder.append(norm_vals_decoder_inner)

        # calculate all norm values at once
        probabilities_all = norm.pdf(norm_vals_all_list)

        # calculate norm value list back and calculate weights
        index = 0
        for i, meas in enumerate(norm_vals_decoder):
            weights_current = weights.copy()
            for j, a in enumerate(meas):
                arr = probabilities_all[index : index + a]
                index = index + a
                weight_factor = 0.0
                weight_factor += sum(arr)
                weights_current[j] *= weight_factor
            weights_current_measurem.append(weights_current)

        # calculate weight of all measurements together
        weights = [np.prod(i) for i in zip(*weights_current_measurem)]

        # normalize
        sum_weights = sum(weights)
        normalized_weights = [x / sum_weights for x in weights]
        # resample so that the weights are approximately uniform (w_i = 1 / NUM_PARTICLES)
        self.particles = random.choices(
            population=self.particles, weights=normalized_weights, k=len(self.particles)
        )
        estimate = self.get_estimate()
        self.send_estimate_to_server(estimate)
        # self.send_locations_to_coordination(estimate)
        self.send_particles_to_server()
        end = time() - start
        end_string = "" + str(end) + "&"
        end_string = end_string + str(estimate) + "&"
        positions = ast.literal_eval((requests.get(SERVER_2 + "/positions").text))
        pos4 = None if not "4" in positions else positions["4"]
        end_string = end_string + str(pos4) + "\n"
        
        f = open(dirname + "/one_node_4_anchors_accuracy.csv", "a", encoding="utf-16")
        f.write(end_string)
        f.close()
        self.reset_particles()

    def illustrate_nodes_and_particles(
        self, real_pos=(-100, -100), estimate=(0, (-100, -100))
    ):
        plt.clf()
        # _, estimate_x, estimate_y, _, _ = self.get_estimate()
        plt.scatter([real_pos[0]], [real_pos[1]], 100, marker="x", color="g")
        plt.scatter([estimate[1][0]], [estimate[1][1]], 100, marker="x", color="b")
        plt.scatter(0, 0, 100, marker="x", color="r")
        plt.scatter(SIDE_LENGTH_X, 0, 100, marker="x", color="r")
        plt.scatter(SIDE_LENGTH_X, SIDE_LENGTH_Y, 100, marker="x", color="r")
        plt.scatter(0, SIDE_LENGTH_Y, 100, marker="x", color="r")
        # we then scatter its particles
        particles = self.get_particles()
        plt.scatter(
            np.array([p[0] for p in particles]),
            np.array([p[1] for p in particles]),
            25,
            alpha=0.05,
        )
        plt.xlim([-0.2, SIDE_LENGTH_X + 0.2])
        plt.ylim([-0.2, SIDE_LENGTH_Y + 0.2])
        plt.gca().invert_yaxis()
        # plt.pause(0.5)
        plt.show()

    def tick(self) -> bool:
        if self.measurement_queue:
            measurements = self.measurement_queue[:]
            self.measurement_queue = []
            measurement_dict: Dict[int, List[float]] = {}
            for m in measurements:
                # TODO: Think about ID structure
                if m.a >> 8 == self.int_id:
                    if m.b in measurement_dict:
                        measurement_dict[m.b >> 8].append(m.distance)
                    else:
                        measurement_dict[m.b >> 8] = [m.distance]
                if m.b >> 8 == self.int_id:
                    if m.a in measurement_dict:
                        measurement_dict[m.a >> 8].append(m.distance)
                    else:
                        measurement_dict[m.a >> 8] = [m.distance]

            distances = []
            recv_particle_arr = []
            estimate_from_other_arr = []
            for (other_id, dists) in measurement_dict.items():
                distances.append(sum(dists) / len(dists))
                particles_other = self.get_particles_from_server(other_id)
                if particles_other == None:
                    break
                recv_particle_arr.append(particles_other)
                estimate_from_other = self.get_estimate_from_server(other_id)
                if estimate_from_other == None:
                    break
                estimate_from_other_arr.append(estimate_from_other)
            if distances:
                self.handle_measurement(
                    distances, recv_particle_arr, estimate_from_other_arr
                )
            return True
        else:
            return False

    def run(self) -> NoReturn:
        while True:
            self.tick()
            # self.illustrate_nodes_and_particles((100,0))
            sleep(0.1)