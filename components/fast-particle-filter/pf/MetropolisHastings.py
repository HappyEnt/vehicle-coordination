from typing import NamedTuple
from random import randrange, uniform

import numpy as np
import numpy.typing as npt
import matplotlib.pyplot as plt
import matplotlib.cm as cm

from scipy.stats import norm
from scipy.stats import multivariate_normal

class MetropolisHastings():
    def __init__(self, proposal_distribution, target_distribution):
        self.proposal_distribution = proposal_distribution
        self.target_distribution = target_distribution

class Distribution():
    dim: int

    def illustrate(self):
        if self.dim > 2:
            raise NotImplementedError
        elif self.dim == 2:
            # sample directly from distribution
            samples = None

            # try:
            samples = self.sample(num = 1000)
            # except NotImplementedError:
                # print("unable to sample directly from distribution, use uniform distribution")
            # else:
            # samples = Uniform([-2, -2], [2, 2]).sample(num = 1000)


            print(len(samples))
            z = np.array([self.evaluate(sample) for sample in samples])

            x, y = zip(*list(map(lambda samp: (samp.x[0], samp.x[1]), samples)))

            fig, ax = plt.subplots(subplot_kw={"projection": "3d"})
            ax.plot_trisurf(np.array(x), np.array(y), z, cmap=plt.cm.Spectral)

    def sample(self, num = 1):
        raise NotImplementedError

    def evaluate(self, x):
        raise NotImplementedError

class Gaussian(Distribution):
    covariance: npt.ArrayLike
    mean: npt.ArrayLike

    def __init__(self, covariance, mean):
        self.covariance = covariance
        self.mean = mean
        self.dim = len(mean)

    def evaluate(self, x):
        return multivariate_normal.pdf(x.x, mean=self.mean, cov=self.covariance);

    def sample(self, num = 1):
        samples = multivariate_normal.rvs(size = num, mean=self.mean, cov=self.covariance)

        if num == 1:
            return Sample(samples, 1)

        weight = 1.0 / num
        return list(map(lambda sample: Sample(sample, weight), samples))


class Uniform(Distribution):
    lower:  list[float]
    higher: list[float]

    def __init__(self, lower, higher):
        self.dim = len(lower)
        if len(lower) != self.dim or len(higher) != self.dim:
            raise ValueError

        for d in range(self.dim):
            if lower[d] > higher[d]:
                raise ValueError

        self.lower = lower
        self.higher = higher

    def sample(self, num = 1):
        samples = []
        weight = 1.0 / num
        for n in range(num):
            x = []
            for d in range(self.dim):
                x.append(uniform(self.lower[d], self.higher[d]))
            samples.append(x)

        if num == 1:
            return Sample(samples[0], 1)

        return list(map(lambda sample: Sample(sample, weight), samples))

    def evaluate(self, x):
        agg = 1
        for d in range(self.dim):
            agg *= self.higher[d] - self.lower[d]
        return 1/agg

class Sample(NamedTuple):
    x: list[float]
    weight: float


class PBFMessage(Distribution):
    particles: list[Sample]
    ingoing_messages: list[Distribution]

    def __init__(self, particles, cross_node_potential, node_potential, ingoing_messages):
        self.particles = particles
        self.cross_node_potential = cross_node_potential
        self.node_potential = node_potential
        self.ingoing_messages = ingoing_messages
        self.dim = 2

    def set_particles(particles):
        self.particles = particles

    def get_particles(self):
        return self.particles

    def sample(self, num = 1):
        return NotImplementedError

    def evaluate(self, xt):
        agg = 0;

        for xs in self.particles:
            agg += self.cross_node_potential(xt, xs) * self.node_potential(xs) / xs.weight
            for im in self.ingoing_messages:
                agg *= im.evaluate(xs)

        return agg / len(self.particles)


class Belief(Distribution):
    ingoing_messages: list[PBFMessage]

    def __init__(self, node_potential, ingoing_messages):
        self.dim = 2
        self.ingoing_messages = ingoing_messages
        self.node_potential = node_potential

    def evaluate(self, xs):
        agg = self.node_potential.evaluate(xs)
        for im in self.ingoing_messages:
            agg *= im.evaluate(xs)

        return agg

    def sample(self, num = 1):
        samples = []
        uniform = Uniform([0], [1])

        current_sample = Uniform([-1, -1], [1, 1]).sample(num = 1)
        for i in range(num):
            p1 = Gaussian(covariance = np.matrix("2, 0; 0, 2"), mean = np.array(current_sample.x))
            next_sample = p1.sample(num = 1)

            accept = self.evaluate(next_sample) / self.evaluate(current_sample)

            alpha = uniform.sample(num = 1)
            if alpha.x < accept:
                current_sample = next_sample

            samples.append(current_sample)
        return samples

class Node(NamedTuple):
    pos: npt.ArrayLike
    prior: Distribution




def test_distributions():
    # Testing the code
    # node1 = Node(pos = [0,0], belief = Uniform([-1, -1], [1, 1]))
    node1 = Node(pos = np.array([-1,0]), prior = Gaussian(covariance = np.matrix("0.01, 0; 0, 0.01"), mean = np.array([-1, 0])))
    node2 = Node(pos = np.array([1,0]), prior = Gaussian(covariance = np.matrix("0.01, 0; 0, 0.01"), mean = np.array([1, 0])))
    node3 = Node(pos = np.array([-0.5,-0.2]), prior = Uniform([-1, -1], [1, 1]))

    noise = Gaussian(covariance = [0.02], mean=[0]).sample(num = 1)

    z23 = np.abs(np.linalg.norm(node2.pos - node3.pos)) + noise.x
    z13 = np.abs(np.linalg.norm(node1.pos - node3.pos)) + noise.x


    # first iteration particle filter as described in nbp
    # eigentlich auch eine distribution aber das wird mir jetzt zu aktig zu definieren
    noise = Gaussian(covariance = [0.02], mean=[0])
    def likelihood_probabilty23(x, y):
        # likelihood = noise.evaluate(Sample(np.abs(np.sqrt(x.x[0]*x.x[0]  + x.x[1]*x.x[1])) - z23, 1.0))
        likelihood = noise.evaluate(Sample(np.abs(np.linalg.norm(x.x - y.x)) - z23, 1.0))
        return likelihood

    def likelihood_probabilty13(x, y):
        # likelihood = noise.evaluate(Sample(np.abs(np.sqrt(x.x[0]*x.x[0]  + x.x[1]*x.x[1])) - z13, 1.0))
        likelihood = noise.evaluate(Sample(np.abs(np.linalg.norm(x.x - y.x)) - z13, 1.0))
        return likelihood

    # initialBelief = Belief(distance, likelihood)

    # node1.belief.illustrate()
    # node2.prior.illustrate()
    # samples = node.belief.sample(num = 10000)

    outgoing_message_node_2 = PBFMessage(node2.prior.sample(num = 100), likelihood_probabilty23, node2.prior.evaluate, ingoing_messages = [])
    outgoing_message_node_1 = PBFMessage(node1.prior.sample(num = 100), likelihood_probabilty13, node1.prior.evaluate, ingoing_messages = [])

    # outgoing_message_node_2.illustrate()
    # outgoing_message_node_1.illustrate()


    node3_marginalized = Belief(node_potential = node3.prior, ingoing_messages=[outgoing_message_node_2, outgoing_message_node_1])

    node3_marginalized.illustrate()

    # x, y = zip(*samples)
    # plt.scatter(x, y)

    plt.show()

test_distributions()
