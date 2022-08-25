"""This module contains implementations for ranging modules."""

from abc import ABC, abstractmethod
from collections import deque
from time import sleep
from typing import Any, Callable, Dict, Iterable, List, Union

from serial import Serial

from calibrate import RX_DELAYS, TX_DELAYS
from data import (
    ActiveMeasurement,
    Message,
    PassiveMeasurement,
    parse_json_message,
)
from twr import perform_twr


MESSAGE_STORAGE_SIZE = 100


class RangingNode(ABC):
    """This class represents an UWB node.

    Attributes:
        addr:
        measurement_cb:
        msg_storage:
        active_measurements:
        tx_delays:
        rx_delays:
    """

    def __init__(
        self,
        # ranging_id,
        measurement_cb: Callable[
            [Iterable[Union[ActiveMeasurement, PassiveMeasurement]]], Any
        ],
    ):
        # self.addr: int = ranging_id
        self.measurement_cb: Callable[
            [Iterable[Union[ActiveMeasurement, PassiveMeasurement]]], Any
        ] = measurement_cb
        self.msg_storage: deque[Message] = deque(maxlen=MESSAGE_STORAGE_SIZE)
        self.active_measurements: List[ActiveMeasurement] = []
        self.tx_delays: Dict[int, float] = TX_DELAYS
        self.rx_delays: Dict[int, float] = RX_DELAYS

    def handle_message(self, message: Message):
        """Process a RX or TX Message.

        Args:
            message: The new incoming message.
        """

        measurements = list(
            filter(
                lambda m: m.distance >= 0 and m.distance < 1_000_000,
                perform_twr(
                    message,
                    self.msg_storage,
                    rx_delays=self.rx_delays,
                    tx_delays=self.tx_delays,
                ),
            )
        )

        self.msg_storage.appendleft(message)
        self.active_measurements.extend(
            list(filter(lambda x: isinstance(x, ActiveMeasurement), measurements))  # type: ignore
        )
        self.measurement_cb(measurements)

    @abstractmethod
    def send_data(self, data):
        """Send data to be transmitted by the UWB module."""

    @abstractmethod
    def run(self):
        """Start the operation (e.g. listening to a serial connection or reading a dump file)."""


class SerialRangingNode(RangingNode):
    """
    Extension of the `RangingNode` to work with a UWB board connected over a serial
    connection.

    Attributes:
        serial_connection:
    """

    def __init__(
        self,
        measurement_callback: Callable[
            [Iterable[Union[ActiveMeasurement, PassiveMeasurement]]], None
        ],
        serial_connection: Serial,
    ):
        super().__init__(measurement_callback)
        self.serial_connection = serial_connection

    def send_data(self, data):
        self.serial_connection.write(data)

    def run(self):
        while True:
            if line := self.serial_connection.readline():
                message = parse_json_message(str(line)[2:-3])
                if message:
                    self.handle_message(message)


class DumpFileRangingNode(RangingNode):
    """Extension of the `RangingNode` to work with a dump file.

    Attributes:
        file_name:
    """

    def __init__(
        self,
        measurement_callback: Callable[
            [Iterable[Union[ActiveMeasurement, PassiveMeasurement]]], None
        ],
        file_name: str,
    ):
        super().__init__(measurement_callback)
        self.file_name = file_name

    def run(self):
        with open(self.file_name, "r", encoding="UTF-8") as dump_file:
            for line in dump_file.readlines():
                decoded_message = parse_json_message(line)
                if decoded_message:
                    self.handle_message(decoded_message)
                    sleep(0.001)

    def send_data(self, data):
        print(data)
