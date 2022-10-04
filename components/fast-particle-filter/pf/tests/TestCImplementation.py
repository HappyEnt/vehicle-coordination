import sys, os

sys.path.append(os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "c_implementation"))
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from c_implementation.ParticleFilter import CParticleFilter


import pytest
import numpy as np
from scipy.stats import norm


# @pytest.fixture(params=[True, False])
@pytest.fixture(params=[False])
def particleFilter(request):
    return CParticleFilter(request.param)

@pytest.mark.parametrize("particle_list", [[(0,0), (1,1), (4,2)]])
def test_set_get_particles(particle_list):
    pf = CParticleFilter(False)

    pf.set_particles(particle_list)

    print(pf.get_particles())

    assert((np.array(particle_list) == np.array(pf.get_particles())).all())

def test_correction_step():
    # foreign_particles = [ for [i, i] in range(0, 10)]
    # fp = ffi.new(struct particle[], foreign_particles)
    # m = ffi.new(struct measurement*, [0.1, fp])
    distance = 10

@pytest.mark.parametrize("mean", np.arange(-5, 5, 0.2))
@pytest.mark.parametrize("std_dev", np.arange(0.1, 5, 0.1))
@pytest.mark.parametrize("value", np.arange(-5, 5, 0.2))
def test_value_from_normal(particleFilter, mean, std_dev, value):
    np_val = norm.pdf(value, loc=mean, scale=std_dev)
    impl_val = particleFilter.value_from_normal_distribution(mean, std_dev, value)
    assert impl_val == pytest.approx(np_val, 0.05), "Value mismatch, Expected {}, Got {}!".format(np_val, impl_val)
