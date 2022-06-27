import sys, os
from _pf_cffi import ffi, lib

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from pf.abstractPF import AbstractParticleFilter

class CParticleFilter(AbstractParticleFilter):
    def value_from_normal_distribution(self, mean, std_dev, x):
        return lib.value_from_normal_distribution(lib.generate_normal_distribution(1000, mean, std_dev), x)
    
    def resample(self, particles):
         pass

    
    def predict(self, particles, action):
         pass
     
    
    def correct(self, particles, measurement):
         pass
     
    
    def calculate_likelihood(self, measurement, particles):
         pass


