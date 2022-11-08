from distutils.log import error
from fileinput import filename
from logging import ERROR, WARNING, basicConfig
from math import sqrt
import os
from typing import Dict, List, Optional, Tuple

from data import ActiveMeasurement
import matplotlib as mpl
import matplotlib.pyplot as plt
import numpy as np

from ranging import DumpFileRangingNode


def evaluate_calibration(addr, real_distance, configuration: str):
    # x_data = []
    # y_data_cal = []
    # y_err_cal = []
    # y_data_uncal = []
    # y_err_uncal = []
    basicConfig(level=ERROR)
    error_list_cal = []
    error_list_uncal = []
    for i in range(1, 8):
        name = f"experiment_{int(real_distance*100)}_with_{i}_{configuration}"
        file_name = os.path.join(
            "evaluation-data",
            "ranging-accuracy",
            f"{int(real_distance*100)}-{configuration}",
            f"{name}.txt",
        )
        try:
            ranging_node_calibrated = DumpFileRangingNode(
                lambda _: None, file_name
            )
            ranging_node_calibrated.run()
            ranging_node_uncalibrated = DumpFileRangingNode(
                lambda _: None, file_name
            )
            ranging_node_uncalibrated.rx_delays = {}
            ranging_node_uncalibrated.tx_delays = {}
            ranging_node_uncalibrated.run()

            error_a_b_with_calibration = [
                m.distance - real_distance
                for m in ranging_node_calibrated.active_measurements
                if m.a == 0 and m.b == i << 8 or m.b == 0 and m.a == i << 8
            ]
            error_a_b_without_calibration = [
                m.distance - real_distance
                for m in ranging_node_uncalibrated.active_measurements
                if m.a == 0 and m.b == i << 8 or m.b == 0 and m.a == i << 8
            ]

            error_list_cal.append(list(map(lambda x: x*100,error_a_b_with_calibration)))
            error_list_uncal.append(list(map(lambda x: x*100,error_a_b_without_calibration)))

        except FileNotFoundError:
            error(f'Could not open "{file_name}"')

    fig_box1, ax_box1 = plt.subplots()
    ax_box1.boxplot(error_list_cal, showfliers=False)
    ax_box1.grid(True, which="both", axis="y")
    ax_box1.set_title(f"Calibrated nodes, distance: {int(real_distance*100)} cm, {configuration}")
    ax_box1.set_ylabel("distance error in cm")
    ax_box1.set_xlabel("node id")
    ax_box1.set_ylim(-100,250)

    fig_box2, ax_box2 = plt.subplots()
    ax_box2.boxplot(error_list_uncal, showfliers=False)
    ax_box2.grid(True, which="both", axis="y")
    ax_box2.set_title(f"Uncalibrated  nodes, distance: {int(real_distance*100)} cm, {configuration}")
    ax_box2.set_ylabel("distance error in cm")
    ax_box2.set_xlabel("node id")
    ax_box2.set_ylim(-100,250)

    export_file_name1 = os.path.join(
        "resources",
        "evaluation",
        "ranging-accuracy",
        f"accuracy_{real_distance*100}_{configuration}_calibrated.pdf"
    )

    export_file_name2 = os.path.join(
        "resources",
        "evaluation",
        "ranging-accuracy",
        f"accuracy_{real_distance*100}_{configuration}_uncalibrated.pdf"
    )

    fig_box1.savefig(export_file_name1, format="pdf")
    fig_box2.savefig(export_file_name2, format="pdf")

def main():
    for d in [1, 2]:
        for c in ["standing", "lying"]:
            evaluate_calibration(0x0000, d, c)
    plt.show()


if __name__ == "__main__":
    main()
