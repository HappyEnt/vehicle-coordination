from abc import ABC, abstractmethod

# Python offers no interfaces, but at least there are abstract classes...  Maybe should be split
# into an TestAbstractParticleFilter class for methods that are just exposed for verifying the
# implementation

# Usually every node will have one instance of some particle filter implementation implementing the
# AbstractParticleFilter interface. In a Simulation setting an instance will be created for every
# node while in a deployment setting there will be only one node representing the device running the
# particle filter.
class AbstractParticleFilter(ABC):
    @abstractmethod
    def get_particles(self):
        raise NotImplementedError

    # Usually only called initially to generate a uniform prior distribution showing that are
    # clueless about our whereabouts
    @abstractmethod
    def set_particles(self, particles):
        raise NotImplementedError

    @abstractmethod
    def predict(self, action):
        raise NotImplementedError

    @abstractmethod
    def correct(self, measurement):
        raise NotImplementedError

    # Exposed just for automatic testing of the implementation, not part of the public interface
    @abstractmethod
    def resample(self, particles):
        raise NotImplementedError
    
    @abstractmethod
    def calculate_likelihood(self, measurement, particles):
        raise NotImplementedError
    
    @abstractmethod
    def value_from_normal_distribution(self, mean, std_dev, x):
        raise NotImplementedError
