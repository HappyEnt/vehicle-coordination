import sys, os
import pytest

sys.path.append(os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "c_implementation"))
from c_implementation.pfCFFI import CParticleFilter

@pytest.fixture()
def particleFilter():
    return CParticleFilter()
