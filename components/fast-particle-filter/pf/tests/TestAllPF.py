import sys, os
import numpy as np
import pytest
from scipy.stats import norm

# TODO How to find modules in a less hacky way?
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from TestCPF import particleFilter

@pytest.mark.parametrize("mean", [0, 100, 10])
@pytest.mark.parametrize("std_dev", range(100, 200, 10))
@pytest.mark.parametrize("value", range(0, 100, 10))
def test_value_from_normal(particleFilter, mean, std_dev, value):
    np_val = norm.pdf(value, loc=mean, scale=std_dev)
    impl_val = particleFilter.value_from_normal_distribution(mean, std_dev, value)
    assert pytest.approx(np_val - impl_val, 0.05), "Value mismatch, Expected {}, Got {}!".format(np_val, impl_val)
