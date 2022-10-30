import matplotlib.pyplot as plt

from data import ActiveMeasurement
from ranging import DumpFileRangingNode


def evaluate_outliers(file: str, realdist):
    ranging_node = DumpFileRangingNode(0, lambda _: None, file)
    ranging_node.run()
    x_good_vals = []
    y_good_vals = []
    x_bad_vals = []
    y_bad_vals = []

    print(ranging_node.active_measurements)

    x = range(len(ranging_node.active_measurements))
    y = [m.distance for m in ranging_node.active_measurements]

    for (i, m) in enumerate(ranging_node.active_measurements):
        if m.distance <= 1.5 * realdist and m.distance >= 0.5 * realdist:
            x_good_vals.append(i)
            y_good_vals.append(m.distance)
        else:
            x_bad_vals.append(i)
            y_bad_vals.append(m.distance)

    fig, ax = plt.subplots()
    ax.scatter(x, y, alpha=0.25)
    plt.show()


def main():
    for i in range(1, 8):
        evaluate_outliers(
            f"evaluation-data/ranging-accuracy/100-standing/experiment_100_with_{i}_standing.txt",
            1,
        )


if __name__ == "__main__":
    main()
