from copy import deepcopy
import json
from logging import warning
import math
from typing import Dict, List, Optional, Tuple

from requests import get
from scipy.constants import speed_of_light
import scipy.optimize

from src.config import TIME_UNIT
from src.data import ActiveMeasurement, Message
from src.twr import perform_twr


CAMERA_SERVER_LENGTH_UNIT = 0.01

# AVG_DELAY = 16450
# DELAY_SPAN = 10.0 * 1e-9 * TIME_UNIT
# TODO: find optimal range
AVG_DELAY = 0 # 1e-9 * TIME_UNIT  # 1 ns
DELAY_SPAN = 1e-8 / TIME_UNIT

MIN_CALIBRATION_DIST = 0.2


# def get_real_positions():
#     return {"0": [0,0], "64": [100,0]}
#     return {"1725": [0, 0], "64071": [223, 0], "50438": [0, 177]}
#     return {"59582": [0, 0], "50438": [0, 150]}  # "dump_file_150.txt"
#     # r_cars = get("http://192.168.87.78:8081/positions")
#     # r_anchors = get("http://192.168.87.78:8081/anchors")
#     # if r_cars.status_code == 200 and r_anchors.status_code == 200:
#     # car_positions_raw = r_cars.content.decode("utf-8")
#     # anchor_positions_raw = r_anchors.content.decode("utf-8")
#     car_positions_raw = (
#         '{"6": [9.77394644, 90.9556397775], "4": [203.034752275, 9.541160232500001]}'
#     )
#     anchor_positions_raw = (
#         '{"0": [0, 0], "1": [215.0, 0], "2": [215.0, 165.5], "3": [0, 165.5]}'
#     )
#     # return ({'6': [9.77394644, 90.9556397775], '4': [203.034752275, 9.541160232500001]}, {'0': [0, 0], '1': [215.0, 0], '2': [215.0, 165.5], '3': [0, 165.5]})
#     cars_json = json.JSONDecoder().decode(car_positions_raw)
#     anchors_json = json.JSONDecoder().decode(anchor_positions_raw)
#     return {**cars_json, **anchors_json}


# else:
#     return None


def build_tof_matrix(positions) -> Dict[int, Dict[int, float]]:
    matrix: Dict[int, Dict[int, float]] = {}
    for addr1 in positions.keys():
        matrix[int(addr1)] = {}
        for addr2 in positions.keys():
            if int(addr1) != int(addr2):
                distance = (
                    math.sqrt(
                        (positions[addr1][0] - positions[addr2][0]) ** 2
                        + (positions[addr1][1] - positions[addr2][1]) ** 2
                    )
                )
                tof = distance / speed_of_light
                tof_dtu = tof / TIME_UNIT
                matrix[int(addr1)][int(addr2)] = tof_dtu
            else:
                matrix[int(addr1)][int(addr2)] = 0.0
    return matrix


def build_tof_matrix_measured(
    messages: List[Message], tx_delays=None, rx_delays=None
) -> Dict[int, Dict[int, float]]:
    measurements: List[ActiveMeasurement] = []
    messages_copy = deepcopy(messages)
    while messages_copy:
        message = messages_copy.pop(0)
        measurements.extend(
            list(
                filter(
                    lambda x: isinstance(x, ActiveMeasurement),
                    perform_twr(
                        message, messages, tx_delays=tx_delays, rx_delays=rx_delays
                    ),
                )
            )  # type: ignore
        )

    measurements.sort()
    measurements = measurements[
        int(0.05 * len(measurements)) : int(
            len(measurements) - 0.05 * len(measurements)
        )
    ]

    temp_matrix: Dict[int, Dict[int, Tuple[List[float], int]]] = {}
    for measurement in measurements:
        if measurement.a not in temp_matrix:
            temp_matrix[measurement.a] = {measurement.b: ([measurement.distance], 1)}
        elif measurement.b not in temp_matrix[measurement.a]:
            temp_matrix[measurement.a][measurement.b] = [measurement.distance], 1
        else:
            temp_matrix[measurement.a][measurement.b] = (
                temp_matrix[measurement.a][measurement.b][0] + [measurement.distance],
                temp_matrix[measurement.a][measurement.b][1] + 1,
            )
    matrix: Dict[int, Dict[int, float]] = {}
    for i in temp_matrix.keys():
        matrix[i] = {}
        for j in temp_matrix[i].keys():
            matrix[i][j] = (
                sum(temp_matrix[i][j][0])
                / temp_matrix[i][j][1]
                / speed_of_light
                / TIME_UNIT
            )
    return matrix


def calibrate(messages, pos) -> Optional[Tuple[Dict, Dict]]:
    def convert_to_delays(res) -> Tuple[Dict, Dict]:
        assert len(participants) * 2 == len(res)
        tx_times = {}
        rx_times = {}
        for i in range(len(participants)):
            tx_times[participants[i]] = res[i]
        for i in range(len(participants)):
            rx_times[participants[i]] = res[i+len(participants)]
        return tx_times, rx_times

    def accuracy(res):
        tx_times, rx_times = convert_to_delays(res)
        measured_tof = build_tof_matrix_measured(
            messages, tx_delays=tx_times, rx_delays=rx_times
        )
        err_sum = 0
        err_num = 0
        for (a, b) in [(a, b) for a in participants for b in participants if a != b]:
            err = abs(real_tof[a][b] - measured_tof[a][b])
            err_sum += err * err
            err_num += 1
        return err_sum / err_num

    participants: List[int] = []
    # Build EDMs for the real (measured by the camera system) and measured (by UWB ranging) distances
    # pos = get_real_positions()
    if not pos:
        # TODO: Use proper logging
        warning("Could not get distance information from camera server")
        return None
    real_tof = build_tof_matrix(pos)
    measured_tof = build_tof_matrix_measured(messages)
    # We need a non changing list of ids
    # make sure `real_tof`` does not contain anything not covered by `measured_tof`

    for i in real_tof:
        if i in measured_tof:
            participants.append(i)

    # Find the optimal delays that minize the difference between the real and measured positions
    res = scipy.optimize.minimize(
        accuracy,
        [AVG_DELAY] * (len(participants) * 2),
        bounds=[(AVG_DELAY - DELAY_SPAN, AVG_DELAY + DELAY_SPAN)] * (len(participants) * 2),
        method="Powell",
        # options= {
        #     "ftol" : 0.000000000001 / TIME_UNIT,
        #     "xtol" : 0.000000000001 / TIME_UNIT

        # }
    )
    return convert_to_delays(res.x)
