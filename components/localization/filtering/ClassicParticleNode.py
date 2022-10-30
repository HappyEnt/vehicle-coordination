'''
This file contains a PArticle Filter which updates the particles with only one measurement from the Ranging.
'''
from abc import ABC, abstractmethod
from logging import debug, info, warning
import math
import random

import numpy as np
from scipy.stats import norm
import matplotlib.pyplot as plt

from filtering.BaseParticleNode import BaseParticleNode
from filtering.AbstractLocalizationNode import NUM_PARTICLES, SIDE_LENGTH_X, SIDE_LENGTH_Y, MEASUREMENT_STDEV

# GRPC imports (for communication with coordination)
import grpc
import interface_pb2
import interface_pb2_grpc

class ClassicParticleNode(BaseParticleNode):
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

    def handle_measurement(self, d, recv_particles, estimate_from_other) -> None:
        """
        Handle incoming measurements by updating own particles.
        """
        # save position of other node using their id as key
        # self.other_nodes_pos[estimate_from_other[0]] = estimate_from_other

        info("Handling measurement")

        for (i, p) in enumerate(self.particles):
            noisy_p = (
                np.random.normal(p[0], MEASUREMENT_STDEV),
                np.random.normal(p[1], MEASUREMENT_STDEV),
            )
            self.particles[i] = noisy_p

        # we update our particles and resample them directly
        # first initialize the weights -> we assume an equal weight for each particle
        # this means also that if particles had a bigger weight, they are just multiple times in the
        #  particles list
        # start = time.time()
        weights = [1.0 / NUM_PARTICLES] * NUM_PARTICLES
        for (i, p) in enumerate(self.particles):
            weight_factor = 0.0
            norm_vals = []
            for rp in recv_particles:
                # estimate the probability P( p1, p2 | d)
                # P( p1, p2 | d) = P( d | p1, p2) * (P(p1, p2) / P(d))
                # we just assume that P(p1, p2) and P(d) are uniform and therefore all particles
                # share this as the same factor
                # as we normalize the weights, we can ignore this factor and can just use
                # P( d | p1, p2) which we can easily compute
                # expected_d = distance.euclidean(p, rp)
                expected_d = math.sqrt((p[0] - rp[0]) ** 2 + (p[1] - rp[1]) ** 2)
                actual_measured_d = d
                # normalize the value using the mean ("expected") and the standard deviation
                norm_val = (actual_measured_d - expected_d) / MEASUREMENT_STDEV
                # collect values and compute later, to drastically increase performance of the
                # algorithm
                norm_vals.append(norm_val)
            probabilities = norm.pdf(norm_vals)
            weight_factor += sum(probabilities)
            weights[i] *= weight_factor
            debug(f"Processing particle {p} new weight factor: {weight_factor}")

        sum_weights = sum(weights)
        normalized_weights = [x / sum_weights for x in weights]

        # resample so that the weights are approximately uniform (w_i = 1 / NUM_PARTICLES)
        self.particles = random.choices(
            population=self.particles, weights=normalized_weights, k=len(self.particles)
        )
        self.send_estimate_to_server()
        self.send_particles_to_server()
        self.reset_particles()

        # end = time.time() - start
        # info("Time elapsed: " + str(end))

    def illustrate_nodes_and_particles(self, real_pos, estimate=(-100, -100)):
        plt.clf()
        pos = real_pos
        # _, estimate_x, estimate_y, _, _ = self.get_estimate()
        plt.scatter([pos[0]], [pos[1]], 100, marker="x", color="g")
        # plt.scatter([estimate_x], [estimate_y], 100, marker="x", color="b")
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

