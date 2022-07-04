import sys, os
import pytest

sys.path.append(os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "c_implementation"))
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from c_implementation.ParticleFilter import CParticleFilter

@pytest.fixture(params=[True, False])
def particleFilter(request):
    return CParticleFilter(request.param)

@pytest.mark.parametrize("particle_list", [[]])
def test_set_get_particles(particle_list):
    pf = CParticleFilter(False)
    pf.set_particles(particle_list)
    p = pf.get_particles()
    assert(len(p) == 0)

def test_correction_step():
    # foreign_particles = [ for [i, i] in range(0, 10)]
    # fp = ffi.new(struct particle[], foreign_particles)
    # m = ffi.new(struct measurement*, [0.1, fp])
    distance = 10
    
    lib.correct()
    
