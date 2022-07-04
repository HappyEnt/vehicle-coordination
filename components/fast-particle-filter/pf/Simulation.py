from Measurements import TWR_Measurement
from abc import ABC, abstractmethod
from c_implementation.ParticleFilter import CParticleFilter
from scipy.spatial import distance
import matplotlib.pyplot as plt

import numpy as np

# We have to look in what module we want to place these methods
def generate_uniform_distribution(side_length, num_particles):
    particles = []
    for i in range(num_particles):
        p = tuple(np.random.uniform(low=0.0, high=side_length, size=2))
        particles.append(p)
    return particles

# we should only be needed to pass the side_length, maybe add some abstraction for a space. num_particles could be taken
# from simulation parameters, i.e., method should be part of Simulation
def node_set_uniform_prior(node, side_length, num_particles):
    node.set_particles(generate_uniform_distribution(side_length, num_particles))

def node_set_dirac_delta_dist(node):
    node.set_particles([node.get_pos()])


    
class Event(ABC):
    @abstractmethod
    def __init__(self, recipient):
        self.recipient = recipient

    def get_recipient(self):
        return self.recipient

class Measurement(Event):
    def __init__(self, recipient):
        super().__init__(recipient)
    
    @abstractmethod
    def get_measurement():
        raise NotImplementedError

class Action(Event):
    def __init__(self, recipient):
        super().__init__(recipient)
        
    @abstractmethod
    def get_action(self):
        raise NotImplementedError    

# TODO sender/recipient is kinda really wrong here. Choose other name later
class TDOA(Measurement):
    def __init__(self, sender_1, sender_2, recipient):
        self.sender_1 = sender_1
        self.sender_2 = sender_2
        self.recipient_3 = recipient
        
        super().__init__(recipient)        

    def get_measurement(self):
        raise NotImplementedError

class TWR(Measurement):
    MEASUREMENT_STD = 0.1
    
    def __init__(self, sender, recipient):
        self.sender = sender
        self.recipient = recipient
        
        super().__init__(recipient)
  
    def get_measurement(self):
        real_distance = distance.euclidean(self.sender.get_pos(), self.recipient.get_pos())
        error_model_distance = np.random.normal(real_distance, self.MEASUREMENT_STD)
        return TWR_Measurement(self.sender.get_particles(), error_model_distance)
    
class Node:
    def __init__(self, pos):
        self.pos = pos

    def get_particles(self):
        return []

    def get_pos(self):
        return self.pos
    
    def get_estimated_pos(self):
        return self.pos                
    
# A node backed by some particle filter implementation
# i.e., pass measurements and events to the particle filter and retrieve
# the updated belief from the particle filter instance.
class ParticleNode(Node):
    def __init__(self, real_pos, particle_filter_instance) -> None:
        super().__init__(real_pos)
        
        self.particle_filter_instance = particle_filter_instance

    # We assume that the caller determines what information the ParticleNode
    def set_particles(self, particles):
        self.particle_filter_instance.set_particles(particles)
        
    def get_particles(self):
        return self.particle_filter_instance.get_particles()

    def get_estimated_pos(self):
        particles = self.get_particles()
        if len(particles) == 0:
            return None

        avg = (0.0, 0.0)
        for p in particles:
            avg = (avg[0] + p[0], avg[1]+ p[1])

        avg = (avg[0] / len(particles), avg[1] / len(particles))
        return avg

    def handle_event(self, event):
        if isinstance(event, Measurement):
            self.particle_filter_instance.correct(event.get_measurement())
        elif isinstance(event, Action):
            self.particle_filter_instance.correct(event.get_measurement())

class Simulation():
    nodes = []
    
    def __init__(self):
        "docstring"
        self.events = []

    def add_nodes(self, nodes):
        self.nodes = nodes

    def add_events(self, events):
        self.events.extend(events)

    def illustrate(self):
        plt.clf()
        fig, ax = plt.subplots()

        colors = plt.rcParams['axes.prop_cycle'].by_key()['color']

        for (i, node) in enumerate(self.nodes):
            # we draw the position
            pos = node.get_pos()
            plt.scatter([pos[0]], [pos[1]], 100, marker="x", c=colors[i])

            # we then scatter its particles
            particles = node.get_particles()
            plt.scatter(np.array([p[0] for p in particles]), np.array([p[1] for p in particles]), 25, c=colors[i], alpha=0.05, edgecolor='none')

            est_pos = node.get_estimated_pos()
            if est_pos:
                plt.scatter([est_pos[0]], [est_pos[1]], 100, marker="+", c=colors[i])
                print("Estimation error node {} of {}".format(i, round(distance.euclidean(pos, est_pos), 2)))

        plt.show()

    # def step(self):
    #     pass

    def execute_all_events(self):
        for event in self.events:
            event.get_recipient().handle_event(event)

SIDE_LENGTH = 10.0
NUM_PARTICLES = 500

anchors = [
    Node((0.0, 0.0)),
    Node((0.0, SIDE_LENGTH)),
    Node((SIDE_LENGTH, SIDE_LENGTH))
]

cache_distribution = True
tags = [
    ParticleNode((SIDE_LENGTH*0.25, SIDE_LENGTH*0.25), CParticleFilter(cache_distribution)),
    ParticleNode((SIDE_LENGTH*0.25, SIDE_LENGTH*0.75), CParticleFilter(cache_distribution)),
    ParticleNode((SIDE_LENGTH*0.75, SIDE_LENGTH*0.75), CParticleFilter(cache_distribution)),
    ParticleNode((SIDE_LENGTH*0.75, SIDE_LENGTH*0.25), CParticleFilter(cache_distribution)),
]

for tag in tags:
    node_set_uniform_prior(tag, SIDE_LENGTH, NUM_PARTICLES)

for tag in tags:
    tag.set_particles(generate_uniform_distribution(SIDE_LENGTH, NUM_PARTICLES))
            
events = [
    # TDOA(anchors[0], anchors[1], tags[0]),
    # TDOA(anchors[0], anchors[2], tags[0]),
    # TDOA(anchors[1], anchors[2], tags[0]),
    # TDOA(anchors[1], anchors[2], tags[1]),
    # TDOA(anchors[1], anchors[2], tags[1]),
    # TDOA(anchors[1], anchors[2], tags[2])
    # (tags[1], tags[2], tags[0]),
    # (tags[0], tags[2], tags[1]),
    # (tags[0], tags[1], tags[2]),

    TWR(sender = anchors[0], recipient = tags[0]),
    TWR(sender = anchors[1], recipient = tags[0]),
    TWR(sender = anchors[2], recipient = tags[0]),
]

sim = Simulation()

sim.add_nodes(tags)
sim.add_nodes(anchors)

sim.add_events(events)

sim.illustrate()

sim.execute_all_events()
