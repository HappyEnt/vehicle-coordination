import sys, os
import pytest

sys.path.append(os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "c_implementation"))
from c_implementation.ParticleFilterC import CParticleFilter

@pytest.fixture(params=[True, False])
def particleFilter(request):
    return CParticleFilter(request.param)
