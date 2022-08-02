from typing import NamedTuple
from random import randrange, uniform

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.cm as cm

class MetropolisHastings():
    def __init__(self, proposal_distribution, target_distribution):
        self.proposal_distribution = proposal_distribution
        self.target_distribution = target_distribution


class Distribution():
    def sample(self, num = 1):
        raise NotImplementedError

    def evaluate(self, x):
        raise NotImplementedError


class Uniform(Distribution):
    dim: int
    lower:  list[float]
    higher: list[float]

    def __init__(self, dim, lower, higher):
        if len(lower) != dim or len(higher) != dim:
            raise ValueError

        for d in range(dim):
            if lower[d] > higher[d]:
                raise ValueError
        
        self.dim = dim
        self.lower = lower
        self.higher = higher
    
    def sample(self, num = 1):
        samples = []
        for n in range(num):
            x = []
            for d in range(self.dim):
                x.append(uniform(self.lower[d], self.higher[d]))
            samples.append(x)
        return samples
    
    def evaluate(self, x):
        agg = 1
        for d in range(self.dim):
            agg *= higher[d] - lower[d]
        return 1/agg

class Sample(NamedTuple):
    x: list[float]
    weight: float


class PBFMessage(Distribution):
    particles: list[Sample]
    ingoing_messages: list[Distribution]

    def __init__(self, particles, cross_node_potential, node_potential):
        self.particles = particles
        self.cross_node_potential = cross_node_potential
        self.node_potential = node_potential        

    def set_particles(particles):
        self.particles = particles

    def get_particles(self):
        return self.particles

    def sample(self):
        return NotImplementedError

    def evaluate(self, xt):
        agg = 0;

        for xs in samples:
            agg = cross_node_potential(xt.x, xs.x) * node_potential(xt.x) / xs.weight
            for im in ingoing_messages:
                agg *= im.evaluate(xt)

        return agg / len(self.particles)


class Belief(Distribution):
    ingoing_messages: list[PBFMessage]
    
    def __init__(self, node_potential):
        self.node_potential = node_potential

    def evaluate(self, xs):
        agg = node_potential(xs)
        for im in ingoing_messages:
            agg *= im.evaluate(xs)

class Node(NamedTuple):
    pos: list[float]
    belief: Distribution




# Testing the code
node = Node(pos = [0,0], belief = Uniform(2, [-1, -1], [1, 1]))

samples = node.belief.sample(num = 10000)

x, y = zip(*samples)
plt.scatter(x, y)

plt.show()
