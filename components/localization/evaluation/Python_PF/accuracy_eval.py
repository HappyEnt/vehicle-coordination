import matplotlib.pyplot as plt
import numpy as np
import os
import csv
import ast
import math

dirname = os.path.dirname(__file__)
file = "one_node_4_anchors_accuracy.csv"
file_path = dirname + "/" + file

X_MAX =  1.66
Y_MAX = 2.035
OFFSET = 0.025

def load_plot_defaults():
    # Configure as needed
    plt.rc('lines', linewidth=2.0)
    plt.rc('legend', framealpha=1.0, fancybox=True)
    # plt.rc('errorbar', capsize=3)
    plt.rc('pdf', fonttype=42)
    plt.rc('ps', fonttype=42)
    plt.rc('font', size=20)


def load_from_csv_int(id):
    solution = []
    with open(file_path, encoding="utf-16") as csv_file:
        csv_reader = csv.reader(csv_file, delimiter='&')
        line_count = 0
        print(line_count)
        for row in csv_reader:
            if line_count == 0:
                # print(f'Column names are {", ".join(row)}')
                line_count += 1
            else:
                line_count += 1
                solution.append(int(row[id]))
    return solution

def load_from_csv_float(id):
    solution = []
    with open(file_path, encoding="utf-16") as csv_file:
        csv_reader = csv.reader(csv_file, delimiter='&')
        line_count = 0
        for row in csv_reader:
            if line_count == 0:
                # print(f'Column names are {", ".join(row)}')
                line_count += 1
            else:
                line_count += 1
                solution.append(row[id])
    return solution

def load_from_csv_ast(id):
    solution = []
    with open(file_path, encoding="utf-16") as csv_file:
        csv_reader = csv.reader(csv_file, delimiter='&')
        line_count = 0
        for row in csv_reader:
            if line_count == 0:
                # print(f'Column names are {", ".join(row)}')
                line_count += 1
            else:
                line_count += 1
                solution.append(ast.literal_eval((row[id])))
    return solution

def only_position(estimate_array):
    solution = []
    for i in estimate_array:
        solution.append(i[1])
    return solution


load_plot_defaults()
# plt.clf()
time = load_from_csv_float(0)
estimate = load_from_csv_ast(1)
estimates = only_position(estimate)
real_pos = load_from_csv_ast(2)


estimates_x = [x[0] for x in estimates]
estimates_y = [y[1] for y in estimates]
real_pos_x = np.mean([x[0] for x in real_pos])
real_pos_y = np.mean([y[1] for y in real_pos])

estimates_average = [np.mean(estimates_x), np.mean(estimates_y)]
distance = math.sqrt((real_pos_x - estimates_average[0]) ** 2 + (real_pos_y - estimates_average[1]) ** 2)

distance2 = np.mean([math.sqrt((real_pos_x - a[0]) ** 2 + (real_pos_y - a[1]) ** 2) for a in estimates])
print(distance2)


fig, ax = plt.subplots()

plt.scatter(estimates_x, estimates_y, marker="o", label="Estimated positions")
plt.scatter(real_pos_x, real_pos_y, marker="x", c="r", label="Real position", s=200)
plt.scatter([0, X_MAX, X_MAX, 0], [0, 0, Y_MAX, Y_MAX], marker="x", c="g", label="Anchor positions", s=200)
ax.yaxis.grid(True)
ax.xaxis.grid(True)
plt.xlabel("x-coordinate (metres)")
plt.ylabel("y-coordinate (metres)")
plt.legend(loc="upper center")
# plt.axis([None, None, 0, 1])

# Adapt the figure size as needed
# fig.set_size_inches(5.0, 8.0)
# plt.tight_layout()
# fig.set_size_inches(4.0, 4.0)
ax.set_title("Estimated positions in a scenario with one node and four anchors")
# plt.tight_layout()
plt.xlim([0 - OFFSET, X_MAX + OFFSET])
plt.ylim([0 - OFFSET, Y_MAX + OFFSET])
# plt.gca().set_aspect('equal', adjustable='box')
plt.show()
