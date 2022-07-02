from Node import Node
from AnchorNode import AnchorNode

class Simulation():
    def __init__(self, particle_filter_implementation):
        "docstring"
        self.pf = particle_filter_implementation
        self.events = []

    def add_nodes(self, nodes):
        self.nodes = nodes

    def add_events(self, events):
        self.events.extend(events)

    def step(self):
        pass
        
class TDOA():
    def __init__(self, anchor_1, anchor_2, tag):
        self.anchor_1 = anchor_1
        self.anchor_2 = anchor_2
        self.tag = tag

anchors = [
    AnchorNode((0.0, 0.0)),
    AnchorNode((0.0, SIDE_LENGTH)),
    AnchorNode((SIDE_LENGTH, SIDE_LENGTH))
]

tags = [
    ParticleNode((SIDE_LENGTH*0.25, SIDE_LENGTH*0.25)),
    ParticleNode((SIDE_LENGTH*0.25, SIDE_LENGTH*0.75)),
    ParticleNode((SIDE_LENGTH*0.75, SIDE_LENGTH*0.75)),
    ParticleNode((SIDE_LENGTH*0.75, SIDE_LENGTH*0.25)),
]
        
events = [
    TDOA(anchors[0], anchors[1], tags[0]),
    TDOA(anchors[0], anchors[2], tags[0]),
    TDOA(anchors[1], anchors[2], tags[0]),
    TDOA(anchors[1], anchors[2], tags[1]),
    TDOA(anchors[1], anchors[2], tags[1]),
    TDOA(anchors[1], anchors[2], tags[2])
    # (tags[1], tags[2], tags[0]),
    # (tags[0], tags[2], tags[1]),
    # (tags[0], tags[1], tags[2]),
]
