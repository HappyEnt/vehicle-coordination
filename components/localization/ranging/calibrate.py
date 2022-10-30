from copy import deepcopy
import logging
import math
from typing import Dict, List, Mapping, MutableSequence, Optional, Tuple

from scipy.constants import speed_of_light
import scipy.optimize

from data import ActiveMeasurement, Message, parse_json_message
from ranging.twr import perform_twr, TIME_UNIT


RX_DELAYS = {
    0: 77.8977994853012,
    256: 12.41623643762934,
    512: -23.796636682151608,
    768: -22.17242755695339,
    1024: 42.23565013797095,
    1280: -0.7720913969190917,
    1536: -13.277699237393,
    1792: -9.39894945994878,
}


TX_DELAYS = {
    0: -153.8229213593596,
    256: -118.5449861820107,
    512: -96.22487798246239,
    768: -18.916975464011536,
    1024: -107.17416474198698,
    1280: -86.80434511866139,
    1536: -38.294127139040036,
    1792: -1.6150401078513477,
}



AVG_DELAY: float = 0
DELAY_SPAN: float = 1e-7 / TIME_UNIT


MIN_CALIBRATION_DIST: float = 0.2


def build_tof_matrix(
    positions: Mapping[int, Tuple[float, float]]
) -> Dict[int, Dict[int, float]]:
    """Build a Euclidian Distance Matrix (EDM) containing the ToFs for all nodes in ``positions``.

    Args:
        positions: A dictionary containing the ranging ids as keys and their positions as values.

    Returns:
        A matrix containing the real ToFs between all nodes in ``positions``.
    """
    matrix: Dict[int, Dict[int, float]] = {}
    for addr1 in positions.keys():
        matrix[int(addr1)] = {}
        for addr2 in positions.keys():
            if int(addr1) != int(addr2):
                distance = math.sqrt(
                    (positions[addr1][0] - positions[addr2][0]) ** 2
                    + (positions[addr1][1] - positions[addr2][1]) ** 2
                )
                tof = distance / speed_of_light
                tof_dtu = tof / TIME_UNIT
                matrix[int(addr1)][int(addr2)] = tof_dtu
            else:
                matrix[int(addr1)][int(addr2)] = 0.0
    return matrix


def build_tof_matrix_measured(
    messages: MutableSequence[Message],
    rx_delays: Optional[Dict[int, float]] = None,
    tx_delays: Optional[Dict[int, float]] = None,
) -> Dict[int, Dict[int, float]]:
    """Build a Matrix of measured ToFs according to exchanged messages.

    Args:
        messages: The messages exchanged by the UWB boards.
        rx_delays: The reception delays to be incorporated in the matrix calculation.
        tx_delays: The transmission delays to be incorporated in the matrix calculation.

    Returns:
        A matrix of measured ToFs adjusted with the ``rx_delays`` and ``tx_delays``.
    """
    measurements: List[ActiveMeasurement] = []
    messages_copy = deepcopy(messages)
    while messages_copy:
        message = messages_copy.pop(0)
        measurements.extend(
                filter(
                    lambda x: isinstance(x, ActiveMeasurement),
                    perform_twr(
                        message, messages, tx_delays=tx_delays, rx_delays=rx_delays
                )
            )  # type: ignore
        )

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

    for i, m_i in temp_matrix.items():
        matrix[i] = {}
        for j, m_ij in m_i.items():
            matrix[i][j] = (
                sum(m_ij[0])
                / m_ij[1]
                / speed_of_light
                / TIME_UNIT
            )
    return matrix


def calibrate(
    messages: MutableSequence[Message], pos: Mapping[int, Tuple[float, float]]
) -> Optional[Tuple[Dict[int, float], Dict[int, float]]]:
    """Find an optimal calibration for the UWB boards.

    Args:
        messages: A list of ranging messages.
        pos: A dictionary containing the ranging ids as keys and their positions as values.

    Returns:
        Optimal RX and TX delays or None.
    """

    def convert_to_delays(res) -> Tuple[Dict[int, float], Dict[int, float]]:
        assert len(participants) * 2 == len(res)
        tx_times = {}
        rx_times = {}
        for (i, p) in enumerate(participants):
            tx_times[p] = res[i]
        for (i, p) in enumerate(participants):
            rx_times[p] = res[i + len(participants)]
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
    # Build EDMs for the real and measured distances
    if not pos:
        logging.warning("Could not get distance information from camera server")
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
        method="Powell",
    )
    return convert_to_delays(res.x)


if __name__ == "__main__":
    logging.basicConfig(level=logging.ERROR)
    real_positions: Mapping[int,Tuple[float,float]] = {
        0x0000: (0, 0),
        0x0100: (2.15, 0),
        0x0200: (2.15, 1.655),
        0x0300: (0, 1.655),
    }
    sreenlog_file = "screenlog.2"
    with open(sreenlog_file, "r", encoding="UTF-8") as file:
        msg_list: List[Message] = list(
            filter(lambda x: x is not None, map(parse_json_message, file.readlines()))
        )  # type: ignore
        msg_list.reverse()

        calibration_result = calibrate(msg_list, real_positions)

        if calibration_result:
            optimal_tx_times, optimal_rx_times = calibration_result
            print("Calibration result:")
            print(f"RX Delays : {optimal_rx_times}")
            print(f"TX Delays : {optimal_tx_times}")
        else:
            print("Calibration error")
