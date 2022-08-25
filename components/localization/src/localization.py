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

from data import ActiveMeasurement, PassiveMeasurement

# GRPC imports (for communication with coordination)
import grpc
import interface_pb2
import interface_pb2_grpc

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


class LocalizationNode(ABC):
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


####################################################################################################
class BaseParticleNode(LocalizationNode):
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

    @abstractmethod
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
        particles = json.loads(
            requests.get(SERVER + "/getparticles/" + str(other_marker_id)).text
        )
        return particles

    def get_estimate_from_server(self, other_id):
        """
        Get the estimate from a node from the server.
        """
        other_estimate = json.loads(
            requests.get(SERVER + "/getestimate/" + str(other_id)).text
        )
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
        postr = requests.post(SERVER + "/setparticles", json=json1)
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
        postr = requests.post(SERVER + "/setestimate", json=json1)
        if postr.status_code != 200:
            warning(postr.text)

    def receive_measurements(self, ds):
        self.measurement_queue.extend(
            filter(lambda x: isinstance(x, ActiveMeasurement), ds)  # type: ignore
        )

    @abstractmethod
    def handle_measurement(self, d, recv_particles, estimate_from_other) -> None:
        pass

    @abstractmethod
    def illustrate_nodes_and_particles(self, real_pos, estimate=(-100, -100)):
        pass

    def send_locations_to_coordination(self):
        """
        Send information to a coordination module.

        The information includes the own position, the own size (radius of vehicle), the own
        inaccuracy in the position estimation (as a radius) and the position and size of the other
        vehicles.
        """
        with grpc.insecure_channel(GRPC_CHANNEL) as channel:
            stub = interface_pb2_grpc.CoordinationStub(channel)
            estimate = self.get_estimate()
            others_list = []
            for key, val in self.other_nodes_pos:
                other = interface_pb2.TickRequest.Participant(
                    id=int(key),
                    position=interface_pb2.Vec2(x=val[1][0], y=val[1][1]),
                    radius=val[2],
                    confidence=val[3],
                )
                others_list.append(other)
            response = stub.Tick(
                interface_pb2.TickRequest(
                    id=self.int_id,
                    position=interface_pb2.Vec2(x=estimate[1][0], y=estimate[1][1]),
                    radius=self.car_radius,
                    confidence=estimate[2],
                    others=others_list,
                )
            )
            self.last_movement_update = time.time()
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


####################################################################################################
# Particle nodes try to estimate their position using particles
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


####################################################################################################
class ClassicAllAtOnce(BaseParticleNode):
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
        self.weights = [1.0 / NUM_PARTICLES] * NUM_PARTICLES

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

    def handle_measurement(
        self, d, recv_particles_arr, estimate_from_other_arr
    ) -> None:
        """
        Handle incoming measurements by updating own particles.
        """
        # save position of other node using their id as key
        for i in estimate_from_other_arr:
            if i:
                self.other_nodes_pos[i[0]] = i

        info("Handling measurement")
        for (i, p) in enumerate(self.particles):
            self.particles[i] = (
                np.random.normal(p[0], MEASUREMENT_STDEV),
                np.random.normal(p[1], MEASUREMENT_STDEV),
            )
        # we update our particles and resample them directly
        # first initialize the weights -> we assume an equal weight for each particle
        # this means also that if particles had a bigger weight, they are just multiple times in
        # the particles list

        # start = time.time()

        weights = [1.0 / NUM_PARTICLES] * NUM_PARTICLES
        # list of weights for all current measurements
        weights_current_measurem: List[List[float]] = []

        norm_vals_all_list = []
        norm_vals_decoder = []
        for (i1, d) in enumerate(d):
            recv_particles = recv_particles_arr[i1]
            norm_vals_current_measure = []
            norm_vals_decoder_inner = []
            for (i, p) in enumerate(self.particles):
                weight_factor = 0.0
                norm_vals = []
                for rp in recv_particles:
                    # estimate the probability P( p1, p2 | d)
                    # P( p1, p2 | d) = P( d | p1, p2) * (P(p1, p2) / P(d))
                    # we just assume that P(p1, p2) and P(d) are uniform and therefore all particles
                    #  share this as the same factor
                    # as we normalize the weights, we can ignore this factor and can just use
                    # P( d | p1, p2) which we can easily compute
                    # expected_d = distance.euclidean(p, rp)
                    expected_d = math.sqrt((p[0] - rp[0]) ** 2 + (p[1] - rp[1]) ** 2)
                    actual_measured_d = d
                    # normalize the value using the mean ("expected") and the standard deviation
                    norm_val = (actual_measured_d - expected_d) / MEASUREMENT_STDEV
                    norm_vals.append(norm_val)
                    # collect values and compute later, to drastically increase performance of the algorithm (REALLY: DRASTICALLY)
                    # because calling norm.pdf on an array rather than just from single values is WAY FASTER
                    norm_vals_all_list.append(norm_val)

                norm_vals_current_measure.append(norm_vals)
                norm_vals_decoder_inner.append(len(norm_vals))
                debug(f"Processing particle {p} new weight factor: {weight_factor}")

            norm_vals_decoder.append(norm_vals_decoder_inner)

        # calculate all norm values at once
        probabilities_all = norm.pdf(norm_vals_all_list)

        # calculate norm value list back and calculate weights
        index = 0
        for i, meas in enumerate(norm_vals_decoder):
            weights_current = weights.copy()
            for j, a in enumerate(meas):
                arr = probabilities_all[index : index + a]
                index = index + a
                weight_factor = 0.0
                weight_factor += sum(arr)
                weights_current[j] *= weight_factor
            weights_current_measurem.append(weights_current)

        # calculate weight of all measurements together
        weights = [np.prod(i) for i in zip(*weights_current_measurem)]

        # normalize
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

    def illustrate_nodes_and_particles(
        self, real_pos=(-100, -100), estimate=(0, (-100, -100))
    ):
        plt.clf()
        # _, estimate_x, estimate_y, _, _ = self.get_estimate()
        plt.scatter([real_pos[0]], [real_pos[1]], 100, marker="x", color="g")
        plt.scatter([estimate[1][0]], [estimate[1][1]], 100, marker="x", color="b")
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

            distances = []
            recv_particle_arr = []
            estimate_from_other_arr = []
            for (other_id, dists) in measurement_dict.items():
                distances.append(sum(dists) / len(dists))
                recv_particle_arr.append(self.get_particles_from_server(other_id))
                estimate_from_other_arr.append(self.get_estimate_from_server(other_id))
            if distances:
                self.handle_measurement(
                    distances, recv_particle_arr, estimate_from_other_arr
                )
            return True
        else:
            return False

    def run(self) -> NoReturn:
        while True:
            self.tick()
            # self.illustrate_nodes_and_particles((100,0))
            sleep(0.1)


####################################################################################################
# Particle nodes try to estimate their position using particles
class GridParticleNode(BaseParticleNode):
    """
    Class for dynamic nodes.
    """

    def __init__(self) -> None:
        super().__init__()
        # we initialize the particles with uniform random samples of the whole area
        for a in range(GRID_SIZE):
            for b in range(GRID_SIZE):
                p = (
                    SIDE_LENGTH_X * a / GRID_SIZE,
                    SIDE_LENGTH_Y * b / GRID_SIZE,
                )
                self.particles.append(p)
        self.measurement_queue: List[ActiveMeasurement] = []
        self.weights = [1.0 / (GRID_SIZE**2)] * GRID_SIZE**2

    def reset_particles(self):
        self.weights = [1.0 / (GRID_SIZE**2)] * GRID_SIZE**2

    def handle_measurement(self, d, recv_particles, estimate_from_other) -> None:
        """
        Handle incoming measurements by updating own particles.
        """
        # save position of other node using their id as key
        # self.other_nodes_pos[estimate_from_other[0]] = estimate_from_other

        info("Handling measurement")

        # for (i, p) in enumerate(self.particles):
        #    noisy_p = (
        #        np.random.normal(p[0], MEASUREMENT_STDEV),
        #        np.random.normal(p[1], MEASUREMENT_STDEV),
        #    )
        #    self.particles[i] = noisy_p

        # we update our particles and resample them directly
        # first initialize the weights -> we assume an equal weight for each particle
        # this means also that if particles had a bigger weight, they are just multiple times in
        # the particles list

        # start = time.time()
        # weights = [1.0 / NUM_PARTICLES] * NUM_PARTICLES

        weights = self.weights
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
            probabilities = norm.pdf(norm_vals) / MEASUREMENT_STDEV
            weight_factor += sum(probabilities)
            weights[i] *= weight_factor
            debug(f"Processing particle {p} new weight factor: {weight_factor}")

        # debug(f"Weights: {weights}")
        sum_weights = sum(weights)
        assert sum_weights != 0
        normalized_weights = [x / sum_weights for x in weights]
        self.weights = normalized_weights
        # resample so that the weights are approximately uniform (w_i = 1 / NUM_PARTICLES)
        # self.particles = random.choices(
        #    population=self.particles, weights=normalized_weights, k=len(self.particles)
        # )
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
        max_weight = max(self.weights)
        plt.scatter(
            np.array([p[0] for p in particles]),
            np.array([p[1] for p in particles]),
            5,
            alpha=np.array(self.weights) / max_weight,
        )
        plt.xlim([-0.2, SIDE_LENGTH_X + 0.2])
        plt.ylim([-0.2, SIDE_LENGTH_Y + 0.2])
        plt.gca().invert_yaxis()
        # plt.pause(0.5)
        plt.show()
