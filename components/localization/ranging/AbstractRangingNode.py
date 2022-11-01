from abc import ABC, abstractmethod
from collections import deque
from typing import Any, Callable, Dict, Iterable, List, Union

from data import (
    ActiveMeasurement,
    Message,
    PassiveMeasurement,
)
from ranging.calibrate import RX_DELAYS, TX_DELAYS
from ranging.twr import perform_twr


MESSAGE_STORAGE_SIZE = 100


class AbstractRangingNode(ABC):
    """This class represents an UWB node.

    Attributes:
        measurement_cb: A callback for new measurements. Should be set to the `receive_measurement`
            method of the corresponding Localization system
        msg_storage: A storage for all messages
        tx_delays: The calibration delays for all nodes in TX mode
        rx_delays: The calibration delays for all nodes in RX mode
    """

    def __init__(
        self,
        measurement_cb: Callable[
            [Iterable[Union[ActiveMeasurement, PassiveMeasurement]]], Any
        ],
    ):
        self.measurement_cb: Callable[
            [Iterable[Union[ActiveMeasurement, PassiveMeasurement]]], Any
        ] = measurement_cb
        self.msg_storage: deque[Message] = deque(maxlen=MESSAGE_STORAGE_SIZE)
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
        self.measurement_cb(measurements)

    @abstractmethod
    def send_data(self, data: str):
        """Send data to be transmitted by the UWB module.

        Args:
            data: The data to be transmitted
        """

    @abstractmethod
    def run(self):
        """Start the operation (e.g. listening to a serial connection or reading a dump file)."""
