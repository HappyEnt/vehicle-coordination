"""This module contains implementations for ranging modules."""

from abc import ABC, abstractmethod
from logging import debug, info, warning
from time import sleep
from typing import Any, Callable, Dict, List, Union

from serial import Serial

from calibrate import calibrate
from config import RX_DELAYS, TX_DELAYS
from data import (
    ActiveMeasurement,
    Message,
    PassiveMeasurement,
    parse_json_message,
)
from twr import perform_twr


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
        ranging_id,
        measurement_cb: Callable[
            [List[Union[ActiveMeasurement, PassiveMeasurement]]], Any
        ],
    ):
        self.addr: int = ranging_id
        self.measurement_cb: Callable[
            [List[Union[ActiveMeasurement, PassiveMeasurement]]], Any
        ] = measurement_cb
        self.msg_storage: List[Message] = []
        self.active_measurements: List[ActiveMeasurement] = []
        self.tx_delays: Dict[int, float] = TX_DELAYS
        self.rx_delays: Dict[int, float] = RX_DELAYS
        self.calibrated: bool = False
        self.calibrated_at: int = 0
        self.real_positions = None

    def set_calibration_ground_truth(self, positions):
        self.real_positions = positions

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

        # TODO: Make more efficient
        self.msg_storage.insert(0, message)
        debug(f"Measurement by {self.addr}: {measurements}")
        self.active_measurements.extend(
            list(filter(lambda x: isinstance(x, ActiveMeasurement), measurements))  # type: ignore
        )
        # info(f"Performed {len(self.active_measurements)} measurements")
        self.measurement_cb(measurements)

    def calibrate(self):
        if not self.calibrated and self.real_positions:
            calibration = calibrate(self.msg_storage, self.real_positions)
            warning(f"Calibration {calibration}")
            if calibration:
                self.tx_delays, self.rx_delays = calibration
                self.calibrated = True
                self.calibrated_at = len(self.active_measurements)

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
        ranging_id,
        measurement_callback: Callable[
            [List[Union[ActiveMeasurement, PassiveMeasurement]]], None
        ],
        serial_connection: Serial,
    ):
        super().__init__(ranging_id, measurement_callback)
        self.serial_connection = serial_connection

    def send_data(self, data):
        self.serial_connection.write(data)

    def run(self):
        while True:
            line = self.serial_connection.readline()
            if line:
                message = parse_json_message(str(line)[2:-3])
                if message:
                    self.handle_message(message)

            # if len(self.active_measurements) >= 500:
            #     self.calibrate()


class DumpFileRangingNode(RangingNode):
    """Extension of the `RangingNode` to work with a dump file.

    Attributes:
        file_name:
    """

    def __init__(
        self,
        ranging_id: int,
        measurement_callback: Callable[
            [List[Union[ActiveMeasurement, PassiveMeasurement]]], None
        ],
        file_name: str,
    ):
        super().__init__(ranging_id, measurement_callback)
        self.file_name = file_name

    def run(self):
        with open(self.file_name, "r", encoding="UTF-8") as dump_file:
            for line in dump_file.readlines():
                decoded_message = parse_json_message(line)
                if decoded_message:
                    self.handle_message(decoded_message)
                    sleep(0.001)
                if len(self.msg_storage) >= 100:
                    self.calibrate()

    def send_data(self, data):
        print(data)
