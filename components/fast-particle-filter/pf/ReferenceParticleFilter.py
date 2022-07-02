from AbstractParticleFilter import AbstractParticleFilter

from abc import ABC, abstractmethod

# Actually an abstract class since python has no interfaces
class AbstractParticleFilter(ABC):
    
    # Returns the current internal state (i.e., particles) of particle filter
    @abstractmethod
    def get_current_state():
        raise NotImplementedError

    # These should only be used for verfication of the implementation and should usually not be called by a module using the Particle Filter.
    @abstractmethod
    def resample(self, particles):
        raise NotImplementedError

    @abstractmethod
    def predict(self, action):
        raise NotImplementedError

    # Measurement could be either self measurement z_self or measurement between two nodes z_ij
    @abstractmethod
    def correct(self, measurement):
        raise NotImplementedError
    
    @abstractmethod
    def calculate_likelihood(self, measurement, particles):
        raise NotImplementedError
    
    @abstractmethod
    def value_from_normal_distribution(self, mean, std_dev, x):
        raise NotImplementedError
