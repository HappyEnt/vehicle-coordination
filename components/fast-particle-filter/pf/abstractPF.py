from abc import ABC, abstractmethod

# Actually an abstract class since python has no interfaces

class AbstractParticleFilter(ABC):

    @abstractmethod
    def resample(self, particles):
        raise NotImplementedError

    @abstractmethod
    def predict(self, particles, action):
        raise NotImplementedError
    
    @abstractmethod
    def correct(self, particles, measurement):
        raise NotImplementedError
    
    @abstractmethod
    def calculate_likelihood(self, measurement, particles):
        raise NotImplementedError
    
    @abstractmethod
    def value_from_normal_distribution(self, mean, std_dev, x):
        raise NotImplementedError
