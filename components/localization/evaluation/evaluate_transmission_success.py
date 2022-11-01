import logging
import os

import matplotlib.pyplot as plt

from data import parse_json_message, RX


def evaluate_transmission_success(num_nodes, intervals):
    x_vals = []
    y_vals = []
    for i in intervals:
        file_path = os.path.join(
            "evaluation-data",
            "transmission-success",
            f"transmission_success_{num_nodes}_{i}.txt",
        )
        try:
            with open(file_path, encoding="UTF-8") as file:
                message_counter = {}
                for line in file.readlines():
                    decoded_message = parse_json_message(line)
                    if decoded_message and decoded_message.type == RX:
                        if decoded_message.tx.addr not in message_counter:
                            message_counter[decoded_message.tx.addr] = 0
                        message_counter[decoded_message.tx.addr] += 1
                assert len(message_counter) <= num_nodes-1
                if len(message_counter) < num_nodes-1:
                    logging.warning(
                        f"It seems there were nodes, that did not transmit anything: {list(message_counter.keys())}"
                    )

                success_rate_percent = (
                    sum(message_counter.values()) / ((num_nodes - 1) * 256) * 100
                )
                x_vals.append(str(i))
                y_vals.append(success_rate_percent)
        except FileNotFoundError:
            logging.error(f'Could not open "{file_path}"')

    fig_bar, ax_bar = plt.subplots()
    bars = ax_bar.bar(x_vals, y_vals)
    ax_bar.set_title(f"Successful transmissions for {num_nodes} nodes")
    ax_bar.set_xlabel("transmission interval in ms (± 10 ms)")
    ax_bar.set_ylabel("successful transmission in percent")
    ax_bar.set_ylim(0,100)
    ax_bar.bar_label(bars, fmt="%.1f")
    export_file_name1 = os.path.join(
        "resources",
        "evaluation",
        "transmission-success",
        f"transmission_success_{num_nodes}.pdf"
    )
    fig_bar.savefig(export_file_name1, format="pdf")




def main():
    for num_nodes in [4, 8]:
        evaluate_transmission_success(num_nodes, [250, 125, 50, 25])
    plt.show()


if __name__ == "__main__":
    main()
