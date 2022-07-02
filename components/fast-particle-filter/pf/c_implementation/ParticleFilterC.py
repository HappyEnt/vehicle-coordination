import sys, os
from _pf_cffi import ffi, lib

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from AbstractParticleFilter import AbstractParticleFilter

class CParticleFilter(AbstractParticleFilter):
    cache_normal_distribution = False
    def __init__(self):
        "docstring"
        
        self.cache_normal_distribution = cache_distribution

    def __del__ (self):
        "call destructor"
        raise NotImplementedError
    
    def value_from_normal_distribution(self, mean, std_dev, x):
        return lib.value_from_normal_distribution(lib.generate_normal_distribution(mean, std_dev, self.cache_normal_distribution), x)
    
    def resample(self, particles):
         pass

    
    def predict(self, particles, action):
         pass
     
    
    def correct(self, particles, measurement):
         pass
     
    
    def calculate_likelihood(self, measurement, particles):
         pass
