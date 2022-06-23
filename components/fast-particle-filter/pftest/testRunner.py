import sys, os
import numpy as np
from scipy.stats import norm


# TODO How to find modules in a less hacky way?
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from pf.abstractPF import AbstractParticleFilter



# For many of the following tests we will assume that numpy is correct

class ParticleFilterTestRunner():
    pf_impl = None
    epsilon = 1e-5

    def __init__(self, particle_filter_implementation):
        self.pf_impl = particle_filter_implementation
    
    def run_test_suite(self):
        self.test_value_from_normal(1000, 100, 2000)

    # @pytest.mark.parametrize('mean', range(0, 1000, 50))
    # @pytest.mark.parametrize('mean', range(0, 1000, 50))    
    def test_value_from_normal(self, mean, std_dev, value):
        np_val = norm.pdf(value, loc=mean, scale=std_dev)
        impl_val = self.pf_impl.value_from_normal_distribution(mean, std_dev, value)
        assert (abs(np_val - impl_val) < self.epsilon), "Value mismatch, Expected {}, Got {}!".format(np_val, impl_val)
