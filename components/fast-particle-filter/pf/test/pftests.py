import sys, os
import numpy as np
import pytest
from scipy.stats import norm

# TODO How to find modules in a less hacky way?
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from testc import particleFilter

epsilon = 1e-5

@pytest.mark.parametrize("mean", range(0, 1000, 50))
@pytest.mark.parametrize("std_dev", range(0, 1000, 50))
@pytest.mark.parametrize("value", range(0, 1000, 50))
def test_value_from_normal(particleFilter, mean, std_dev, value):
    np_val = norm.pdf(value, loc=mean, scale=std_dev)
    impl_val = particleFilter.value_from_normal_distribution(mean, std_dev, value)
    assert (abs(np_val - impl_val) < epsilon), "Value mismatch, Expected {}, Got {}!".format(np_val, impl_val)
