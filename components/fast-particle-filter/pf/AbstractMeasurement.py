from abc import ABC, abstractmethod

# Really simple wrapper for measurements. Since it is not really clear yet, what common information
# exists between different measurement sources, we just require a type field. Instead of a class we
# could also just have used a dictionary.
class AbstractMeasurement(ABC):
    @abstractmethod
    def __init__(self, type):
        self.type = type

    def get_type(self):
        return self.type
