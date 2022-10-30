from logging import ERROR, WARNING, basicConfig
from math import sqrt
from typing import Dict, List, Optional, Tuple

from data import ActiveMeasurement
from ranging import DumpFileRangingNode


def average_measurment(
    measurements: List[ActiveMeasurement], a: int, b: int, start: int, stop: int
) -> Optional[float]:
    distances_a_b = [m.distance for m in measurements if m.a == a and m.b == b]
    distances_a_b = distances_a_b[start:stop]
    distances_a_b.sort()
    distances_a_b = distances_a_b[
        int(0.1 * len(distances_a_b)) : int(
            len(distances_a_b) - 0.1 * len(distances_a_b)
        )
    ]
    if distances_a_b:
        return sum(distances_a_b) / len(distances_a_b)


def evaluate_calibration(
    file, addr, nodes_and_positions: Dict[int, Tuple[float, float]]
):
    basicConfig(level=ERROR)
    ranging_node = DumpFileRangingNode(addr, lambda _: None, file)
    ranging_node.set_calibration_ground_truth(nodes_and_positions)
    ranging_node.run()
    for (a, b) in [
        (a, b) for a in nodes_and_positions for b in nodes_and_positions if a != b
    ]:
        real_distance = sqrt(
            (nodes_and_positions[a][0] - nodes_and_positions[b][0]) ** 2
            + (nodes_and_positions[a][1] - nodes_and_positions[b][1]) ** 2
        )
        distance_pre_calibration = average_measurment(
            ranging_node.active_measurements, a, b, 0, ranging_node.calibrated_at
        )
        distance_post_calibration = average_measurment(
            ranging_node.active_measurements,
            a,
            b,
            ranging_node.calibrated_at,
            len(ranging_node.active_measurements),
        )
        if distance_pre_calibration and distance_post_calibration:
            print()
            print(
                f"Evaluating distance measurement between {a} and {b} real distance {real_distance}"
            )
            print(f"Before calibration {distance_pre_calibration}")
            print(f"After calibration {distance_post_calibration}")


def main():
    # evaluate_calibration("screenlog.txt", 0x40, {0x40: (0,0), 0x00: (1,0)})
    evaluate_calibration("screenlog.0", 0x00, {0x0000: (0,0), 0x0100: (2.15,0), 0x0200: (0,1.655)})
    #0x0300: (0,1.655), 0x0400: (0.96,0.60)})


if __name__ == "__main__":
    main()
