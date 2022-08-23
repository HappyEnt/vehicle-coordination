from Measurements import TWR_Measurement
from abc import ABC, abstractmethod
from c_implementation.ParticleFilter import CParticleFilter
from ReferenceParticleFilter import ReferenceParticleFilter
from scipy.spatial import distance
import matplotlib.pyplot as plt
from matplotlib.widgets import Button

from time import sleep, time

import numpy as np

# We have to look in what module we want to place these methods
def generate_uniform_distribution(side_length, num_particles):
    particles = []
    for i in range(num_particles):
        p = tuple(np.random.uniform(low=-side_length, high=side_length, size=2))
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

class Iterate(Event):
    def __init__(self, recipient):
        super().__init__(recipient)

class Action(Event):
    def __init__(self, recipient):
        super().__init__(recipient)

    @abstractmethod
    def get_action(self):
        raise NotImplementedError


class Move(Action):
    def __init__(self, recipient, displacement: tuple[float, float]):
        self.recipient = recipient
        self.displacement = displacement

    def get_displacement(self):
        return self.displacement

    def get_action(self):
        # For now we only have one kind of action: Movement;
        # Therefore we for now omit an abstraction as classes and directly pass the moved_distance to
        # the particle filter instance
        dx, dy = self.get_displacement()
        distance_moved = np.sqrt(dx*dx  + dy*dy)
        return distance_moved

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

    def set_pos(self, pos):
        self.pos = pos


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
            self.particle_filter_instance.add_message(event.get_measurement())
        elif isinstance(event, Action):
            # self.particle_filter_instance.correct(event.get_measurement())
            print(self.get_pos())
            x, y = self.get_pos()
            dx, dy = event.get_displacement()
            self.set_pos( (x + dx, y + dy) )
            self.particle_filter_instance.predict(event.get_action())
        elif isinstance(event, Iterate):
            self.particle_filter_instance.iterate()

class Simulation():
    nodes = []
    events = []

    def __init__(self, dimensions):
        "docstring"
        self.ind = 0
        self.dimensions = dimensions
        self.events = []

    def add_nodes(self, nodes):
        self.nodes.extend(nodes)

    def add_events(self, events):
        self.events.extend(events)

        # TODO for now hack it like that, this should be replaced ASAP
        # self.fig, self.ax = plt.subplots(1, len(events))

    def next(self, event):
        self.ax.cla()

        self.ax.set_xlim([-self.dimensions, self.dimensions])
        self.ax.set_ylim([-self.dimensions, self.dimensions])


        while not isinstance(self.events[self.ind], Iterate):
            print("execute event {}".format(self.ind))
            self.events[self.ind].get_recipient().handle_event(self.events[self.ind])
            self.ind += 1

        print("execute event {}".format(self.ind))
        self.events[self.ind].get_recipient().handle_event(self.events[self.ind])
        self.draw_plot()

        self.ind += 1

        if self.ind >= len(self.events):
            self.ind = 0

        plt.draw()

    def create_buttons(self):
        plt.subplots_adjust(bottom=0.2)

        axnext = plt.axes([0.81, 0.05, 0.1, 0.075])
        bnext = Button(axnext, 'Next')
        bnext.on_clicked(self.next)

        axnext._button = bnext

    def illustrate(self):
        self.fig, self.ax = plt.subplots()

        self.create_buttons()

        # plt.connect('button_press_event', self.next)

        self.draw_plot()
        plt.draw()

        plt.show()

    def draw_plot(self):
        for (j, node) in enumerate(self.nodes):
            colors = plt.rcParams['axes.prop_cycle'].by_key()['color']
            pos = node.get_pos()
            self.ax.scatter([pos[0]], [pos[1]], 100, marker="x", c=colors[j])

            particles = node.get_particles()
            self.ax.scatter(np.array([p[0] for p in particles]), np.array([p[1] for p in particles]), 25, c=colors[j], alpha=0.25, edgecolor='none')

            est_pos = node.get_estimated_pos()
            self.ax.scatter([est_pos[0]], [est_pos[1]], 100, marker="+", c=colors[j])

    def execute_all_events_capture_time(self):
        start_time = time()
        for i,event in enumerate(self.events):
            event.get_recipient().handle_event(event)

        end_time = time()
        return end_time - start_time


    def execute_all_events(self):
        for i,event in enumerate(self.events):
            event.get_recipient().handle_event(event)

            draw_plot(j, node)

            print("Estimation error node {} of {}".format(j, round(distance.euclidean(pos, est_pos), 2)))

# Example simulation
cache_distribution = False
def start_interactive_sim(particles = 50):

    ANCHOR_SPACING = 20.0
    SIM_SPACE_LENGTH = 40.0

    anchors = [
        ParticleNode((0.0, 0.0), ReferenceParticleFilter()),
        ParticleNode((ANCHOR_SPACING, 0.0), ReferenceParticleFilter()),
        ParticleNode((ANCHOR_SPACING/2, ANCHOR_SPACING), ReferenceParticleFilter())
    ]

    tags = [
        ParticleNode((ANCHOR_SPACING*1, ANCHOR_SPACING*0.25), CParticleFilter(cache_distribution)),
        ParticleNode((0, ANCHOR_SPACING*0.75), CParticleFilter(cache_distribution)),
    ]

    events = [
        TWR(sender = anchors[0], recipient = tags[0]),
        TWR(sender = anchors[1], recipient = tags[0]),
        TWR(sender = tags[1], recipient = tags[0]),
        TWR(sender = anchors[0], recipient = tags[1]),
        TWR(sender = anchors[2], recipient = tags[1]),
        TWR(sender = tags[0], recipient = tags[1]),
        Iterate(recipient = tags[0]),
        Iterate(recipient = tags[1]),

        TWR(sender = anchors[0], recipient = tags[0]),
        TWR(sender = anchors[1], recipient = tags[0]),
        TWR(sender = tags[1], recipient = tags[0]),
        TWR(sender = anchors[0], recipient = tags[1]),
        TWR(sender = anchors[2], recipient = tags[1]),
        TWR(sender = tags[0], recipient = tags[1]),
        Iterate(recipient = tags[0]),
        Iterate(recipient = tags[1]),

    #     Move(recipient = tags[0], displacement = (-4, 0)),
    #     Iterate(recipient = tags[0]),

    #     Move(recipient = tags[1], displacement = (-4, 0)),
    #     Iterate(recipient = tags[1]),

    #     TWR(sender = tags[1], recipient = tags[0]),
    #     Iterate(recipient = tags[0]),

    #     Move(recipient = tags[0], displacement = (-4, 0)),
    #     Iterate(recipient = tags[0]),

    #     TWR(sender = tags[1], recipient = tags[0]),
    #     Iterate(recipient = tags[0]),
    ]


    for tag in tags:
        particle_node_set_uniform_prior(tag, SIM_SPACE_LENGTH, particles)

    for anchor in anchors:
        particle_node_set_dirac_delta_dist(anchor)

    sim = Simulation(SIM_SPACE_LENGTH)
    sim.add_nodes(tags)
    sim.add_nodes(anchors)
    sim.add_events(events)

    sim.illustrate()

def benchmark_particles():
    runtimes = []
    ANCHOR_SPACING = 20.0
    SIM_SPACE_LENGTH = 40.0

    for particles in range(1000, 3000, 250):
        anchors = [
            ParticleNode((0.0, 0.0), ReferenceParticleFilter()),
            ParticleNode((ANCHOR_SPACING, 0.0), ReferenceParticleFilter()),
            ParticleNode((ANCHOR_SPACING/2, ANCHOR_SPACING), ReferenceParticleFilter())
        ]

        tags = [
            ParticleNode((ANCHOR_SPACING*1, ANCHOR_SPACING*0.25), CParticleFilter(cache_distribution)),
            ParticleNode((0, ANCHOR_SPACING*0.75), CParticleFilter(cache_distribution)),
        ]

        events = [
            TWR(sender = anchors[0], recipient = tags[0]),
            TWR(sender = anchors[1], recipient = tags[0]),
            TWR(sender = tags[1], recipient = tags[0]),
            TWR(sender = anchors[0], recipient = tags[1]),
            TWR(sender = anchors[2], recipient = tags[1]),
            TWR(sender = tags[0], recipient = tags[1]),
            Iterate(recipient = tags[0]),
            Iterate(recipient = tags[1]),

            TWR(sender = anchors[0], recipient = tags[0]),
            TWR(sender = anchors[1], recipient = tags[0]),
            TWR(sender = tags[1], recipient = tags[0]),
            TWR(sender = anchors[0], recipient = tags[1]),
            TWR(sender = anchors[2], recipient = tags[1]),
            TWR(sender = tags[0], recipient = tags[1]),
            Iterate(recipient = tags[0]),
            Iterate(recipient = tags[1]),

            Move(recipient = tags[0], displacement = (-4, 0)),
            Iterate(recipient = tags[0]),

            Move(recipient = tags[1], displacement = (-4, 0)),
            Iterate(recipient = tags[1]),

            TWR(sender = tags[1], recipient = tags[0]),
            Iterate(recipient = tags[0]),

            Move(recipient = tags[0], displacement = (-4, 0)),
            Iterate(recipient = tags[0]),

            TWR(sender = tags[1], recipient = tags[0]),
            Iterate(recipient = tags[0]),
        ]


        for tag in tags:
            particle_node_set_uniform_prior(tag, SIM_SPACE_LENGTH, particles)

        for anchor in anchors:
            particle_node_set_dirac_delta_dist(anchor)

        sim = Simulation(SIM_SPACE_LENGTH)
        sim.add_nodes(tags)
        sim.add_nodes(anchors)
        sim.add_events(events)

        runtimes.append(sim.execute_all_events_capture_time())

    print(runtimes)

def benchmark_particles_statistics(repetetions, particles):
    runtimes = []
    ANCHOR_SPACING = 20.0
    SIM_SPACE_LENGTH = 40.0

    for _ in range(0, repetetions):
        anchors = [
            ParticleNode((0.0, 0.0), ReferenceParticleFilter()),
            ParticleNode((ANCHOR_SPACING, 0.0), ReferenceParticleFilter()),
            ParticleNode((ANCHOR_SPACING/2, ANCHOR_SPACING), ReferenceParticleFilter())
        ]

        tags = [
            ParticleNode((ANCHOR_SPACING*1, ANCHOR_SPACING*0.25), CParticleFilter(cache_distribution)),
            ParticleNode((0, ANCHOR_SPACING*0.75), CParticleFilter(cache_distribution)),
        ]

        events = [
            TWR(sender = anchors[0], recipient = tags[0]),
            TWR(sender = anchors[1], recipient = tags[0]),
            TWR(sender = tags[1], recipient = tags[0]),
            TWR(sender = anchors[0], recipient = tags[1]),
            TWR(sender = anchors[2], recipient = tags[1]),
            TWR(sender = tags[0], recipient = tags[1]),
            Iterate(recipient = tags[0]),
            Iterate(recipient = tags[1]),

            TWR(sender = anchors[0], recipient = tags[0]),
            TWR(sender = anchors[1], recipient = tags[0]),
            TWR(sender = tags[1], recipient = tags[0]),
            TWR(sender = anchors[0], recipient = tags[1]),
            TWR(sender = anchors[2], recipient = tags[1]),
            TWR(sender = tags[0], recipient = tags[1]),
            Iterate(recipient = tags[0]),
            Iterate(recipient = tags[1]),
        ]


        for tag in tags:
            particle_node_set_uniform_prior(tag, SIM_SPACE_LENGTH, particles)

        for anchor in anchors:
            particle_node_set_dirac_delta_dist(anchor)

        sim = Simulation(SIM_SPACE_LENGTH)
        sim.add_nodes(tags)
        sim.add_nodes(anchors)
        sim.add_events(events)

        runtimes.append(sim.execute_all_events_capture_time())

    print(runtimes)
    fig1, ax1 = plt.subplots()
    ax1.set_title('Basic Plot')
    ax1.boxplot(runtimes)
    plt.show()

# benchmark_particles_statistics(repetetions = 100, particles = 125)

# benchmark_particles()

start_interactive_sim(particles = 250)
