from abc import ABC, abstractmethod
from logging import info
from time import sleep
from typing import Callable, Dict, List, Union

from serial import Serial

from components.localization.src.calibrate import calibrate
from components.localization.src.data import (
    ActiveMeasurement,
    Message,
    PassiveMeasurement,
    parse_json_message,
)
from components.localization.src.twr import perform_twr


class RangingNode(ABC):
    """This class represents an UWB node."""

    def __init__(
        self,
        ranging_id,
        measurement_callback: Callable[
            [List[Union[ActiveMeasurement, PassiveMeasurement]]], None
        ],
    ):
        self.addr = ranging_id
        self.measurement_callback = measurement_callback
        self.msg_storage: List[Message] = []
        self.active_measurements: List[ActiveMeasurement] = []
        self.tx_delays: Dict[int, float] = {}
        self.rx_delays: Dict[int, float] = {}
        self.calibrated: bool = False
        self.calibrated_at: int = 0

    def handle_message(self, message: Message):
        """Process a RX or TX Message."""

        measurements = perform_twr(
            message,
            self.msg_storage,
            rx_delays=self.rx_delays,
            tx_delays=self.tx_delays,
        )

        # TODO: Make more efficient
        self.msg_storage.insert(0, message)
        self.active_measurements.extend(
            list(filter(lambda x: isinstance(x, ActiveMeasurement), measurements))
        )
        # info(f"Performed {len(self.active_measurements)} measurements")
        self.measurement_callback(measurements)

    def calibrate(self):
        if not self.calibrated:
            calibration = calibrate(self.msg_storage)
            info(f"Calibration {calibration}")
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
            if line := self.serial_connection.readline():
                message = parse_json_message(str(line)[2:-3])
                if message:
                    self.handle_message(message)

            if len(self.active_measurements) >= 500:
                self.calibrate()


class DumpFileRangingNode(RangingNode):
    """Extension of the `RangingNode` to work with a dump file."""

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
            for line in dump_file.readlines()[:600]:
                decoded_message = parse_json_message(line)
                if decoded_message:
                    self.handle_message(decoded_message)
                    sleep(0.01)
                if len(self.active_measurements) >= 500:
                    self.calibrate()

    def send_data(self, data):
        print(data)
