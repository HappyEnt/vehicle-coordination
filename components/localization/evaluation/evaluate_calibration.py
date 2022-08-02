from typing import List
from components.localization.src.data import ActiveMeasurement
from components.localization.src.ranging import DumpFileRangingNode


def average_measurment(
    measurements: List[ActiveMeasurement], a: int, b: int, start: int, stop: int
) -> float:
    distances_a_b = [
        m.distance
        for m in measurements
        if m.a == a and m.b == b or m.a == b and m.b == b
    ]
    distances_a_b = distances_a_b[start:stop]
    distances_a_b.sort()
    distances_a_b = distances_a_b[
        int(0.1 * len(distances_a_b)) : int(
            len(distances_a_b) - 0.1 * len(distances_a_b)
        )
    ]
    return sum(distances_a_b) / len(distances_a_b)


def main():
    ranging_node = DumpFileRangingNode(
        0x06BD, lambda _: None, "dump_file_three_nodes.txt"
    )
    ranging_node.run()
    distance_a_b_pre_calibration = average_measurment(
        ranging_node.active_measurements,
        1725,
        55808,
        0,
        ranging_node.calibrated_at
    )
    distance_a_c_pre_calibration = average_measurment(
        ranging_node.active_measurements,
        1725,
        64071,
        0,
        ranging_node.calibrated_at
    )
    distance_b_c_pre_calibration = average_measurment(
        ranging_node.active_measurements,
        55808,
        64071,
        0,
ranging_node.calibrated_at    )
    distance_a_b_post_calibration = average_measurment(
        ranging_node.active_measurements,
        1725,
        55808,
        ranging_node.calibrated_at,
        len(ranging_node.active_measurements),
    )
    distance_a_c_post_calibration = average_measurment(
        ranging_node.active_measurements,
        1725,
        64071,
        ranging_node.calibrated_at,
        len(ranging_node.active_measurements),
    )
    distance_b_c_post_calibration = average_measurment(
        ranging_node.active_measurements,
        55808,
        64071,
        ranging_node.calibrated_at,
        len(ranging_node.active_measurements),
    )

    print(f"Distance a b pre calibration {distance_a_b_pre_calibration}")
    print(f"Distance a c pre calibration {distance_a_c_pre_calibration}")
    print(f"Distance b c pre calibration {distance_b_c_pre_calibration}")
    print()
    print(f"Distance a b post calibration {distance_a_b_post_calibration}")
    print(f"Distance a c post calibration {distance_a_c_post_calibration}")
    print(f"Distance b c post calibration {distance_b_c_post_calibration}")


if __name__ == "__main__":
    main()
