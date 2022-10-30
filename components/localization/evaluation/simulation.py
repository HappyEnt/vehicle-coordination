from math import sqrt
import random
from typing import Any, Callable, List

from scipy.constants import speed_of_light

from data import RX, TX, TimingInfo, jsonify_message, Message
from ranging.twr import TIME_UNIT


class SimulationNode:
    def __init__(self, addr, pos_x, pos_y):
        self.addr = addr
        self.clock_offset: float = random.uniform(0, 1 / TIME_UNIT)
        self.clock_drift: float = random.uniform(1 - 20e-6, 1 + 20e-6)
        self.sn = 0
        self.rx_times = []
        self.pos_x = pos_x
        self.pos_y = pos_y

    def simulate_rx(self, rx_message: Message, global_time: int):
        self.rx_times.insert(
            0, TimingInfo(addr=rx_message.tx.addr, sn=rx_message.tx.sn, ts=int(global_time*self.clock_drift+self.clock_offset))
        )
        self.rx_times = self.rx_times[:10]
        return Message(type=RX, clock_offset_ratio=None, tx=rx_message.tx, rx=rx_message.rx)

    def simulate_tx(self, global_time: int) -> Message:
        self.sn += 1
        return Message(
            type=TX,
            clock_offset_ratio=None,
            tx=TimingInfo(
                addr=self.addr,
                sn=self.sn,
                ts=int(global_time*self.clock_drift+self.clock_offset),
            ),
            rx=self.rx_times,
        )


def distance_to_time(node1, node2) -> int:
    return int(
        sqrt((node1.pos_x - node2.pos_x) ** 2 + (node1.pos_y - node2.pos_y) ** 2)
        / speed_of_light / TIME_UNIT
    )


def simulate(
    node: SimulationNode,
    other_nodes: List[SimulationNode],
    num_exchanges: int = 1000,
    transmisstion_success_rate: float = 1,
    time_step: float = 0.1,  # 100 msec,
    message_cb: Callable[[str], Any] = print,
):
    exchange_counter = 1
    global_clock: int = 0
    while exchange_counter <= num_exchanges:
        for other_node in other_nodes:
            global_clock += int(time_step / TIME_UNIT)
            tx_message = other_node.simulate_tx(global_clock)
            if random.uniform(0, 1) <= transmisstion_success_rate:
                message_cb(
                    jsonify_message(
                        node.simulate_rx(
                            tx_message,
                            global_clock + distance_to_time(node, other_node),
                        )
                    ) + "\n"
                )
            for another_node in other_nodes:
                if (
                    another_node != other_node
                    and random.uniform(0, 1) <= transmisstion_success_rate
                ):
                    another_node.simulate_rx(
                        tx_message,
                        global_clock + distance_to_time(other_node, another_node),
                    )
            exchange_counter += 1
        global_clock += int(time_step / TIME_UNIT)
        tx_message = node.simulate_tx(global_clock)
        message_cb(jsonify_message(tx_message) + "\n")
        for other_node in other_nodes:
            if random.uniform(0, 1) <= transmisstion_success_rate:
                other_node.simulate_rx(
                    tx_message, global_clock + distance_to_time(other_node, node)
                )


if __name__ == "__main__":
    with open("dump_file_simulated.txt", "a", encoding="UTF-8") as file:
        simulate(
            SimulationNode(0x0400, 1.115, 0.885),
            [
                SimulationNode(0x0000, 0, 0),
                SimulationNode(0x0100, 2.23, 0),
                SimulationNode(0x0200, 0, 1.77),
                SimulationNode(0x0300, 2.23, 1.77),
            ],
            num_exchanges=10_000,
            message_cb=file.write,
        )
