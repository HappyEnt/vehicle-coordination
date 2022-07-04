import random
import math
import numpy as np
import matplotlib.pyplot as plt
from typing import List, Tuple, Optional
from scipy.spatial import distance
from scipy.stats import norm
import time


SIDE_LENGTH = 10.0  # 10x10 m^2
NUM_NODES = 2

NUM_PARTICLES = 500
NUM_PARTICLES_FOR_EXCHANGE = 10  # The amount of particles that actually gets transmitted to the other nodes

MEASUREMENT_STD = 0.1
POSITION_STD = MEASUREMENT_STD # TODO: this is just a wild guess atm

# Array of fixed nodes (or anchors in our setting)
fixed_nodes = [
    (0.0, 0.0)
]

dynamic_nodes = [
    ()
]


class Node:
    def __init__(self, pos) -> None:
        self.pos = pos

    def handle_measurement(self, d, recv_particles) -> None:
        pass

    def handle_passive_measurement(self, r, recv_particles_a, recv_particles_b) -> None:
        pass

    def get_particles(self) -> List[Tuple[float, float]]:
        return []

    def get_particles_for_exchange(self) -> List[Tuple[float, float]]:
        return []

    def get_pos(self) -> Tuple[float, float]:
        return self.pos

    # This estimates the position based on the list of particles
    def get_estimated_pos(self) -> Optional[Tuple[float, float]]:
        particles = self.get_particles()
        if len(particles) == 0:
            return None

        avg = (0.0, 0.0)
        for p in particles:
            avg = (avg[0] + p[0], avg[1]+ p[1])

        avg = (avg[0] / len(particles), avg[1] / len(particles))
        return avg

# Anchor nodes just know their position
class AnchorNode(Node):
    # Fixed nodes do not change their position estimation based on their measurements
    def handle_measurement(self, d, recv_particles) -> None:
        pass

    # Fixed nodes do not change their position estimation based on their measurements
    def handle_passive_measurement(self, r, recv_particles_a, recv_particles_b) -> None:
        pass

    # The fixed node does know its position and just returns this as a single particle (this means 100% probability)
    def get_particles(self) -> List[Tuple[float, float]]:
        return [self.pos]

    def get_particles_for_exchange(self) -> List[Tuple[float, float]]:
        return self.get_particles()


# Particle nodes try to estimate their position using particles (for now, just the static p
class ParticleNode(Node):
    def __init__(self, pos) -> None:
        super().__init__(pos)
        self.particles = [] # we assume that the weights are equal for all particles

        # we initialize the particles with uniform random samples of the whole area
        for i in range(NUM_PARTICLES):
            p = tuple(np.random.uniform(low=0.0, high=SIDE_LENGTH, size=2))
            self.particles.append(p)

    def get_particles(self) -> List[Tuple[float, float]]:
        return self.particles

    def get_particles_for_exchange(self) -> List[Tuple[float, float]]:
        return random.choices(
            population = self.particles,
            k = NUM_PARTICLES_FOR_EXCHANGE
        )

    def _add_particle_noise(self):
        # We "update" the particles by adding some noise to each of them
        # Usually, we would use our update function based on the velocity and add noise with that but as we are stationary, it is just noise for now
        for (i, p) in enumerate(self.particles):
            # TODO: I am currently not sure if this POSITION_STD - is good or bad ;)
            # normally you would add this noise while resampling
            noisy_p = (np.random.normal(p[0], POSITION_STD), np.random.normal(p[1], POSITION_STD))
            self.particles[i] = noisy_p

    # Resample using the given weights (do not need to be normalized yet)
    def _resample_particles(self, weights):
        sum_weights = sum(weights)
        normalized_weights = [x/sum_weights for x in weights]

        # resample so that the weights are approximately uniform (w_i = 1 / NUM_PARTICLES)
        self.particles = random.choices(
            population=self.particles,
            weights=normalized_weights,
            k=len(self.particles)
        )

    # we update our particles and resample them directly
    def handle_measurement(self, d, recv_particles) -> None:

        self._add_particle_noise()

        # estimate the probability P( p1, p2 | d)
        # P( p1, p2 | d) = P( d | p1, p2) * (P(p1, p2) / P(d))
        # we just assume that P(p1, p2) and P(d) are uniform and therefore all particles share this as the same factor
        # as we normalize the weights, we can ignore this factor and can just use P( d | p1, p2) which we can easily compute
        def estimate_prob(p1, p2):
            # the expected distance for the positions
            expected_d = distance.euclidean(p1, p2)
            actual_measured_d = d
            # normalize the value using the mean ("expected") and the standard deviation
            norm_val = (actual_measured_d-expected_d) / MEASUREMENT_STD
            prob = norm.pdf(norm_val)       # I am not sure if we would need to divide by the MEASUREMENT_STD again - but as this linear as well we drop it :)
            return prob

        # first initialize the weights -> we assume an equal weight for each particle
        # this means also that if particles had a bigger weight, they are just multiple times in the particles list
        weights = [1.0 / NUM_PARTICLES] * NUM_PARTICLES

        for (i, p) in enumerate(self.particles):
            weight_factor = 0.0
            for (k, rp) in enumerate(recv_particles):
                weight_factor += estimate_prob(p, rp)
            weights[i] *= weight_factor

        self._resample_particles(weights)

    # we update our particles and resample them directly
    def handle_passive_measurement(self, r, recv_particles_a, recv_particles_b) -> None:

        self._add_particle_noise()

        # estimate the probability P( p1, p2, p3 | z)
        # P( p1, p2, p3 | z) = P( z | p1, p2, p3) * (P(p1, p2, p3) / P(z))
        # we just assume that P(p1, p2, p3) and P(z) are uniform and therefore all particles share this as the same factor
        # as we normalize the weights, we can ignore this factor and can just use P( d | p1, p2) which we can easily compute
        def estimate_prob(p1, p2, p3):
            # the expected distance for the positions
            expected_d = distance.euclidean(p1, p2) - distance.euclidean(p1, p3)
            actual_measured_r = r
            # normalize the value using the mean ("expected") and the standard deviation
            norm_val = (actual_measured_r - expected_d) / MEASUREMENT_STD
            prob = norm.pdf(
                norm_val)  # I am not sure if we would need to divide by the MEASUREMENT_STD again - but as this linear as well we drop it :)
            return prob

        # first initialize the weights -> we assume an equal weight for each particle
        # this means also that if particles had a bigger weight, they are just multiple times in the particles list
        weights = [1.0 / NUM_PARTICLES] * NUM_PARTICLES

        for (i, p1) in enumerate(self.particles):
            weight_factor = 0.0
            for p2 in recv_particles_a:
                for p3 in recv_particles_b:
                    weight_factor += estimate_prob(p1, p2, p3)
            weights[i] *= weight_factor

        print(weights)
        self._resample_particles(weights)


anchors = [
    AnchorNode((0.0, 0.0)),
    AnchorNode((0.0, SIDE_LENGTH)),
    AnchorNode((SIDE_LENGTH, SIDE_LENGTH))
]

tags =[
    ParticleNode((SIDE_LENGTH*0.25, SIDE_LENGTH*0.25)),
    ParticleNode((SIDE_LENGTH*0.25, SIDE_LENGTH*0.75)),
    ParticleNode((SIDE_LENGTH*0.75, SIDE_LENGTH*0.75)),
    ParticleNode((SIDE_LENGTH*0.75, SIDE_LENGTH*0.25)),
]



# From -> To
messages = [
    (anchors[0], anchors[1], tags[0]),
    (anchors[0], anchors[2], tags[0]),
    (anchors[1], anchors[2], tags[0]),
    (anchors[1], anchors[2], tags[1]),
    (anchors[1], anchors[2], tags[1]),
    (anchors[1], anchors[2], tags[2]),
    (tags[1], tags[2], tags[0]),
    (tags[0], tags[2], tags[1]),
    (tags[0], tags[1], tags[2]),
]

nodes = anchors + tags




while True:
    for msg in messages:

        time.sleep(1.0)
        if len(msg) == 2:
            (from_node, to_node) = msg
            d = np.random.normal(distance.euclidean(from_node.get_pos(), to_node.get_pos()), MEASUREMENT_STD)
            to_node.handle_measurement(d, from_node.get_particles_for_exchange())
        else:
            (node_a, node_b, node_listening) = msg
            r = np.random.normal(distance.euclidean(node_listening.get_pos(), node_a.get_pos()) - distance.euclidean(node_listening.get_pos(), node_b.get_pos()), MEASUREMENT_STD)
            node_listening.handle_passive_measurement(r, node_a.get_particles_for_exchange(), node_b.get_particles_for_exchange())

        illustrate_nodes_and_particles()
