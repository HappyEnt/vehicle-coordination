'''
This file contains a superclass for all Particle Filters.
The class contains:
Communication to coordination, reading the configs, getting measurements from ranging, calculating an estimated position from the particles, etc.
'''
from abc import ABC, abstractmethod
import ast
import configparser
import json
from logging import debug, info, warning
import math
import random
from time import time, sleep
from typing import Any, Dict, Iterable, List, NoReturn, Tuple, Union
import os

import numpy as np
import requests
from scipy.stats import norm
from scipy.spatial import distance
import matplotlib.pyplot as plt

from filtering.AbstractLocalizationNode import AbstractLocalizationNode, dirname ,SERVER, NUM_PART_EXCHANGE, config, SIDE_LENGTH_X, SIDE_LENGTH_Y
from data import ActiveMeasurement, PassiveMeasurement

# GRPC imports (for communication with coordination)
import grpc
import interface_pb2
import interface_pb2_grpc



class BaseParticleNode(AbstractLocalizationNode):
    def __init__(self):
        car_config = configparser.ConfigParser()
        car_config.read(
            dirname + "/car_config.ini"
        )  # config for the car (holds id and size of car)
        self.int_id = int(car_config["CAR"]["marker_id"])
        self.car_size_width = float(car_config["CAR"]["width"])
        self.car_size_length = float(car_config["CAR"]["length"])
        self.car_radius = (
            math.sqrt(self.car_size_width**2 + self.car_size_length**2)
        ) / 2  # convert a rectangle-shaped car into a single radius
        self.particles: List[
            Union[
                Tuple[float, float], List[float], np.ndarray[Any, np.dtype[np.float64]]
            ]
        ] = []  # we assume that the weights are equal for all particles
        self.last_movement_update = time()
        self.velocity = [
            0.0,
            0.0,
        ]  # velocity of this vehicle, gotten from the coordination
        self.other_nodes_pos: Dict[
            str, Tuple[float, float, float]
        ] = {}  # positions of other nodes, key is the id of the other nodes
        self.measurement_queue: List[ActiveMeasurement] = []

    # @abstractmethod
    def reset_particles(self):
        pass

    def get_estimate(self):
        """
        Returns the current estimated position of this node.

        This includes its id, a point and a radius.
        """
        estimate_x = np.mean([p[0] for p in self.particles])
        estimate_y = np.mean([p[1] for p in self.particles])

        # calculate radius
        distances = []
        for p in self.particles:
            distances.append(distance.euclidean(p, [estimate_x, estimate_y]))

        dist_std = 2 * np.std(distances)
        guess_radius = dist_std + np.mean(distances)
        return (self.int_id, [estimate_x, estimate_y], self.car_radius, guess_radius)

    def get_particles(self):
        return self.particles

    def set_position_others(self, id_int, value):
        self.other_nodes_pos[id_int] = value

    def get_particles_for_exchange(self):
        # return random particles from particle list for exchange
        return random.choices(population=self.particles, k=NUM_PART_EXCHANGE)

    def get_measurements_from_server(self, other_marker_id):
        positions = ast.literal_eval(requests.get(SERVER + "/positions").text)
        positions.pop(str(self.int_id), None)
        anchors = ast.literal_eval(requests.get(SERVER + "/anchors").text)
        all_pos = {**positions, **anchors}
        # print(all_pos)
        all_keys = list(all_pos.keys())
        # other_marker_id = all_keys[random.randint(0, len(all_keys) - 1)]
        dista = json.loads(
            requests.get(
                SERVER + "/distance/" + str(self.int_id) + "/" + str(other_marker_id)
            ).text
        )
        other_pos = json.loads(
            requests.get(SERVER + "/position/" + str(other_marker_id)).text
        )
        particles = json.loads(
            requests.get(SERVER + "/getparticles/" + str(other_marker_id)).text
        )
        return other_marker_id, float(dista), ast.literal_eval(other_pos), particles

    def get_particles_from_server(self, other_marker_id):
        try:
            particles = json.loads(
                requests.get(SERVER + "/getparticles/" + str(other_marker_id)).text
            )
        except requests.exceptions.ConnectionError as e:
            warning(str(e))
            return None
        return particles

    def get_estimate_from_server(self, other_id):
        """
        Get the estimate from a node from the server.
        """
        try:
            other_estimate = json.loads(
                requests.get(SERVER + "/getestimate/" + str(other_id)).text
            )
        except requests.exceptions.ConnectionError as e:
            warning(str(e))
            return None
        return other_estimate

    def send_particles_to_server(self):
        """
        Send particles to the webserver.
        """
        dict1 = {}
        dict1["marker_id"] = self.int_id
        particles = self.get_particles_for_exchange()
        dict1["particles"] = list(particles)
        json1 = json.dumps(dict1)

        try:
            postr = requests.post(SERVER + "/setparticles", json=json1)
        except requests.exceptions.ConnectionError as e:
            warning(str(e))

        if postr.status_code != 200:
            warning(postr.text)

    def send_estimate_to_server(self, estimate=None):
        """
        Send estimate to the webserver.
        """
        dict1 = {}
        if not estimate:
            estimate = self.get_estimate()
        dict1["marker_id"] = self.int_id
        dict1["estimate"] = estimate
        json1 = json.dumps(dict1)
        try:
            postr = requests.post(SERVER + "/setestimate", json=json1)
        except requests.exceptions.ConnectionError as e:
            warning(str(e))
            return
        if postr.status_code != 200:
            warning(postr.text)

    def receive_measurements(self, ds):
        self.measurement_queue.extend(
            filter(lambda x: isinstance(x, ActiveMeasurement), ds)  # type: ignore
        )

    # @abstractmethod
    def handle_measurement(self, d, recv_particles, estimate_from_other) -> None:
        pass

    # @abstractmethod
    def illustrate_nodes_and_particles(self, real_pos, estimate=(-100, -100)):
        pass

    def send_locations_to_coordination(self, estimate = None):
        """
        Send information to a coordination module.

        The information includes the own position, the own size (radius of vehicle), the own
        inaccuracy in the position estimation (as a radius) and the position and size of the other
        vehicles.
        """
        if not estimate:
            estimate = self.get_estimate()
        try:
            # stub = interface_pb2_grpc.CoordinationStub(channel)
            others_list = []
            print("started")
            print(self.other_nodes_pos)
            for key, val in self.other_nodes_pos.items():
                other = interface_pb2.TickRequest.Participant(
                    id=int(key),
                    position=interface_pb2.Vec2(x=val[1][0], y=val[1][1]),
                    radius=val[2],
                    confidence=val[3],
                )
                others_list.append(other)
            print("ended")
            others_list = []
            response = stub.Tick(
                interface_pb2.TickRequest(
                    id=self.int_id,
                    position=interface_pb2.Vec2(x=estimate[1][0], y=estimate[1][1]),
                    radius=self.car_radius,
                    confidence=estimate[2],
                    others=others_list,
                )
            )
            self.last_movement_update = time()
            self.velocity = [response.new_velocity.x, response.new_velocity.y]

            if (
                config["EVALUATION"]["track_positions"]
                and config["EVALUATION"]["server_available"]
            ):
                real_position = ast.literal_eval(
                    requests.get(SERVER + "/positions").text
                )
                current_time = time()
                with open("positions.txt", "a", encoding="utf-8") as file:
                    file.write(
                        json.JSONEncoder().encode(
                            {
                                "time": current_time,
                                "estimated_position": estimate[1],
                                "real": real_position,
                            }
                        )
                        + "\n"
                    )
            info("Sent data to coordination")
        except grpc._channel._InactiveRpcError as e:
            warning(str(e))

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
            for (other_id, distances) in measurement_dict.items():
                # _, _, _, particles = self.get_measurements_from_server(other_id)

                # TODO: Remove hardcode
                other_id_dec = 0
                if other_id == 0x000:
                    particles = [(0, 0)]
                    other_id_dec = 0
                elif other_id == 0x100:
                    particles = [(SIDE_LENGTH_X, 0)]
                    other_id_dec = 1
                elif other_id == 0x200:
                    particles = [(0, SIDE_LENGTH_Y)]
                    other_id_dec = 2
                elif other_id == 0x300:
                    other_id_dec = 3
                    particles = [(SIDE_LENGTH_X, SIDE_LENGTH_Y)]
                else:
                    warning(f"Unkown id: {other_id}")
                    assert False
                self.handle_measurement(
                    sum(distances) / len(distances), particles, None
                )
            return True
        else:
            return False

    def run(self) -> NoReturn:
        while True:
            self.tick()
            # self.illustrate_nodes_and_particles((100,0))
            sleep(0.1)
