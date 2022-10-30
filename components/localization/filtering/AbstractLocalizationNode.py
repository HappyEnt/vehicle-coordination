'''
This file contains an abstract class definition for a Particle Filter.
'''
from abc import ABC, abstractmethod
import configparser
import os
from typing import Any, Dict, Iterable, List, NoReturn, Tuple, Union

import grpc
import interface_pb2
import interface_pb2_grpc

from data import ActiveMeasurement, PassiveMeasurement

dirname = os.path.dirname(__file__)

config = configparser.ConfigParser()
config.read(dirname + "/config.ini")

SIDE_LENGTH_X = float(config["DEFAULT"]["testing_area_length"])  # size of area in m
SIDE_LENGTH_Y = float(config["DEFAULT"]["testing_area_width"])  # size of area in m

NUM_PARTICLES = 1_000  # number of particles
NUM_PART_EXCHANGE = (
    100  # The amount of particles that actually gets transmitted to the other nodes
)
GRID_SIZE = 100

# error in distance measurements in m
MEASUREMENT_STDEV = 0.02

SERVER = "http://192.168.87.78:8081"

GRPC_CHANNEL = "localhost:50052"

VISUALISATION = False
channel = grpc.insecure_channel(GRPC_CHANNEL)
stub = interface_pb2_grpc.CoordinationStub(channel)

class AbstractLocalizationNode(ABC):
    """
    Base class for localization.
    """

    @abstractmethod
    def receive_measurements(
        self, ds: Iterable[Union[ActiveMeasurement, PassiveMeasurement]]
    ):
        pass

    @abstractmethod
    def run(self):
        pass
