"""
This file can track the estimated position of cars, using the server.
"""

import ast
import os
import requests
import sys
import time

import matplotlib.pyplot as plt

dirname = os.path.dirname(__file__)

SERVER = "http://192.168.87.78:8081"
SIDE_LENGTH_X = 2.155
SIDE_LENGTH_Y = 1.655

ANCHORS = [0, 1, 2, 3]


def show(anchors, positions, positions_id, figure, axis):
    axis.cla()
    if positions:
        axis.scatter(
            [x[0] for x in positions],
            [y[1] for y in positions],
            10,
            marker="o",
            color="b",
        )
    # axis.scatter(pos[0], pos[1], 100, marker="x", color="g")
    axis.scatter(
        [x[0] for x in anchors], [y[1] for y in anchors], 100, marker="x", color="r"
    )
    # we then scatter its particles
    axis.set_xlim([-0.2, SIDE_LENGTH_X + 0.2])
    axis.set_ylim([SIDE_LENGTH_Y + 0.2, -0.2])
    if positions_id:
        labels = ["ID " + str(x) for x in positions_id]
        print(labels)
        for i, label in enumerate(labels):
            plt.annotate(label, (positions[i][0], positions[i][1] - 0.025))

    plt.draw()
    # plt.pause(2)
    # plt.pause(0.1)


def show_estimates(anchors, estimates):
    axis.cla()
    axis.set_xlim([-0.2, SIDE_LENGTH_X + 0.2])
    axis.set_ylim([SIDE_LENGTH_Y + 0.2, -0.2])
    print(anchors)
    axis.scatter(
        [x[0] for x in anchors], [y[1] for y in anchors], 100, marker="x", color="r"
    )
    for k, val in estimates.items():
        if not (int(k) in ANCHORS):
            axis.scatter(val[1][0], val[1][1], 10, marker="o", color="b")
            circle1 = plt.Circle((val[1][0], val[1][1]), val[3], color="r", fill=False)
            axis.add_patch(circle1)
    plt.draw()


def show_line_plot(x, y):
    axis.cla()
    axis.plot(x, y)
    plt.draw()


if __name__ == "__main__":
    if requests.get(SERVER + "/").status_code != 200:
        print("Cannot reach server")
        sys.exit(0)
    else:
        print("Server found")

    anchors = ast.literal_eval((requests.get(SERVER + "/anchors").text))
    anchor_list = [x[1] for x in anchors.items()]
    figure, axis = plt.subplots()
    # show(anchor_list, None, None, figure, axis)
    positions_saved = []
    print(dirname)
    plt.ion()
    plt.show()
    # x=[time.time(), time.time()]
    # y=[0,0]
    while True:
        # x.append(time.time())
        # y.append(y[len(y) - 1] + 0.01)
        positions = ast.literal_eval((requests.get(SERVER + "/positions").text))
        estimates = ast.literal_eval((requests.get(SERVER + "/getestimate").text))
        positions_arr = [x[1] for x in positions.items()]
        positions_id = [x[0] for x in positions.items()]
        positions_saved.append((time.time(), positions))
        print(estimates)
        # print(positions_arr)
        show(anchor_list, positions_arr, positions_id, figure, axis)
        show_estimates(anchor_list, estimates)
        # show_line_plot(x,y)
        plt.pause(0.01)
