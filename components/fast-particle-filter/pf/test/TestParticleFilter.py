import sys, os
import numpy as np
import pytest
from scipy.stats import norm


# TODO How to find modules in a less hacky way?
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from pf.abstractPF import AbstractParticleFilter

epsilon = 1e-5

class TestParticleFilter():
    
    def test_value_from_normal(self, value_from_normal):
        np_val = norm.pdf(value, loc=mean, scale=std_dev)
        impl_val = self.getParticleFilter().value_from_normal_distribution(mean, std_dev, value)
        print("try {} {} {}".format(mean, std_dev, value))
        assert (abs(np_val - impl_val) < self.epsilon), "Value mismatch, Expected {}, Got {}!".format(np_val, impl_val)
