from time import sleep
from typing import Callable, Iterable, Union


from data import (
    ActiveMeasurement,
    PassiveMeasurement,
    parse_json_message,
)
from ranging.AbstractRangingNode import AbstractRangingNode


class DumpFileRangingNode(AbstractRangingNode):
    """Extension of the `RangingNode` to work with a dump file.

    Attributes:
        file_name: The path to the file that is to be read
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
