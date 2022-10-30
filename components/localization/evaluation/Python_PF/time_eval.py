import matplotlib.pyplot as plt
import numpy as np
import os
import csv
import ast

dirname = os.path.dirname(__file__)
file = "one_node_4_anchors_accuracy.csv"
file_path = dirname + "/" + file

def load_plot_defaults():
    # Configure as needed
    plt.rc('lines', linewidth=2.0)
    plt.rc('legend', framealpha=1.0, fancybox=True)
    plt.rc('errorbar', capsize=3)
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

print(estimates)

estimates_x = [x[0] for x in estimates]
estimates_y = [y[1] for y in estimates]

real_pos_x = np.mean([x[0] for x in real_pos])
real_pos_y = np.mean([y[1] for y in real_pos])
# datas = []
# means = []
# stds = []

# for i in range(0,number_of_different_particles):
#     datas.append(np.array(yCSV[i * number_of_measurements_per_bar: (i+1) * number_of_measurements_per_bar]).astype(float))

# for i in range(0,len(datas)):
#     means.append(np.mean(datas[i]))
#     stds.append(np.std(datas[i]))

# data_a = data_0
# data_b = data_0

# mean_a = np.mean(data_a)
# mean_b = np.mean(data_b)
# mean_0 = np.mean(data_0)

# std_a = np.std(data_a)
# std_b = np.std(data_b)
# mean_0 = np.std(data_0)


# # plt.clf()

fig, ax = plt.subplots()

# ax.bar(["50", "100", "150", "200", "250", "300", "350", "400", "450", "500", "550", "600", "650", "700", "750", "800", "850", "900", "950", "1000"], means, yerr=stds, align='center',
#         ecolor='black', capsize=5)
plt.scatter(estimates_x, estimates_y, marker="o")
plt.scatter(real_pos_x, real_pos_y, marker="x", c="r")
ax.yaxis.grid(True)
ax.xaxis.grid(True)
plt.ylabel("X-Coordinate in m")
plt.xlabel("Y-Coordinate in m")
# plt.axis([None, None, 0, 1])

# Adapt the figure size as needed
# fig.set_size_inches(5.0, 8.0)
# plt.tight_layout()
# fig.set_size_inches(4.0, 4.0)
ax.set_title("Error in position estimation for one node and four anchors")
plt.tight_layout()
plt.xlim([0, 1.66])
plt.ylim([0, 2.035])
plt.show()