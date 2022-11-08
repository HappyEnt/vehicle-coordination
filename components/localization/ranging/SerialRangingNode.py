
from typing import Callable, Iterable,  Union, Optional

from serial import Serial

from data import (
    ActiveMeasurement,
    Message,
    PassiveMeasurement,
    parse_json_message,
)
from ranging.AbstractRangingNode import AbstractRangingNode


class SerialRangingNode(AbstractRangingNode):
    """
    Extension of the `RangingNode` to work with a UWB board connected over a serial
    connection.

    Attributes:
        serial_connection: The `Serial` connection to the connected board.
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
                message: Optional[Message] = parse_json_message(str(line)[2:-3])
                if message:
                    self.handle_message(message)
