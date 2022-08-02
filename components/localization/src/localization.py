from typing import List, Union

from components.localization.src.data import ActiveMeasurement, PassiveMeasurement


class LocalizationNode:
    def __init__(self):
        pass

    def handle_measurement(self, d: List[Union[ActiveMeasurement, PassiveMeasurement]]):
        pass
