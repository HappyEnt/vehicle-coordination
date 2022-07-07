from Measurements import TWR_Measurement
from abc import ABC, abstractmethod
from c_implementation.ParticleFilter import CParticleFilter
from ReferenceParticleFilter import ReferenceParticleFilter
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
def particle_node_set_uniform_prior(node, side_length, num_particles):
    node.set_particles(generate_uniform_distribution(side_length, num_particles))

def particle_node_set_dirac_delta_dist(node):
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
    MEASUREMENT_STD = 0.1 # Parameter of the simulation
    
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

    def get_pos(self):
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
    events = []
    
    def __init__(self):
        "docstring"
        self.events = []

    def add_nodes(self, nodes):
        self.nodes.extend(nodes)

    def add_events(self, events):
        self.events.extend(events)
        # TODO for now hack it like that, this should be replaced ASAP
        self.fig, self.ax = plt.subplots(figsize=(1, len(events)))

    def illustrate(self):
        plt.show()

    # def step(self):
    #     pass

    def execute_all_events(self):
        for i,event in enumerate(self.events):
            event.get_recipient().handle_event(event)
            
            colors = plt.rcParams['axes.prop_cycle'].by_key()['color']
            
            self.fig.add_subplot(1, len(events), i+1)
            for (j, node) in enumerate(self.nodes):

                
                pos = node.get_pos()
                plt.scatter([pos[0]], [pos[1]], 100, marker="x", c=colors[j])

                particles = node.get_particles()
                plt.scatter(np.array([p[0] for p in particles]), np.array([p[1] for p in particles]), 25, c=colors[j], alpha=0.05, edgecolor='none')

                est_pos = node.get_estimated_pos()
                plt.scatter([est_pos[0]], [est_pos[1]], 100, marker="+", c=colors[j])
                print("Estimation error node {} of {}".format(j, round(distance.euclidean(pos, est_pos), 2)))
            

ANCHOR_SPACING = 10.0
SIM_SPACE_LENGTH = 20.0

NUM_PARTICLES = 50000


# TODO The right abstraction still has to be found. We should design general enough abstraction
# so the prior/posterior distributions can be described by a variety of distributions
# For example it should also be able to have nodes that represent their belief through
# parametric distributions like a gaussian. 
anchors = [
    ParticleNode((0.0, 0.0), ReferenceParticleFilter()),
    ParticleNode((0.0, ANCHOR_SPACING), ReferenceParticleFilter()),
    ParticleNode((ANCHOR_SPACING, ANCHOR_SPACING), ReferenceParticleFilter())
]

cache_distribution = False

tags = [
    ParticleNode((ANCHOR_SPACING*0.25, ANCHOR_SPACING*0.25), CParticleFilter(cache_distribution)),
    # ParticleNode((ANCHOR_SPACING*0.25, ANCHOR_SPACING*0.25), ReferenceParticleFilter()),    
    ParticleNode((ANCHOR_SPACING*0.25, ANCHOR_SPACING*0.75), CParticleFilter(cache_distribution)),
    # ParticleNode((ANCHOR_SPACING*0.75, ANCHOR_SPACING*0.75), CParticleFilter(cache_distribution)),
    # ParticleNode((ANCHOR_SPACING*0.75, ANCHOR_SPACING*0.25), CParticleFilter(cache_distribution)),
]

# tags = [
    # ParticleNode((ANCHOR_SPACING*0.25, ANCHOR_SPACING*0.25), ReferenceParticleFilter()),
    # ParticleNode((ANCHOR_SPACING*0.25, ANCHOR_SPACING*0.75), ReferenceParticleFilter()),
    # ParticleNode((ANCHOR_SPACING*0.75, ANCHOR_SPACING*0.75), ReferenceParticleFilter()),
    # ParticleNode((ANCHOR_SPACING*0.75, ANCHOR_SPACING*0.25), ReferenceParticleFilter()),
# ]

for tag in tags:
    particle_node_set_uniform_prior(tag, SIM_SPACE_LENGTH, NUM_PARTICLES)

for anchor in anchors:
    particle_node_set_dirac_delta_dist(anchor)
            
events = [
    # TDOA(anchors[0], anchors[1], tags[0]),
    # TDOA(anchors[0], anchors[2], tags[0]),
    # TDOA(anchors[1], anchors[2], tags[0]),
    # TDOA(anchors[1], anchors[2], tags[1]),
    # TDOA(anchors[1], anchors[2], tags[1]),

    TWR(sender = anchors[0], recipient = tags[0]),
    TWR(sender = anchors[1], recipient = tags[0]),
    # TWR(sender = anchors[2], recipient = tags[0]),

    TWR(sender = anchors[1], recipient = tags[1]),
    TWR(sender = anchors[2], recipient = tags[1]),

    TWR(sender = tags[0], recipient = tags[1]),
]

sim = Simulation()

sim.add_nodes(tags)
sim.add_nodes(anchors)

sim.add_events(events)

# sim.illustrate()

sim.execute_all_events()

sim.illustrate()
