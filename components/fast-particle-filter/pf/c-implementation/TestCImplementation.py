import sys, os
import pytest

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from pftest.TestParticleFilter import TestParticleFilter

from pfCFFI import CParticleFilter


class TestCParticleFilter(TestParticleFilter):
    @pytest.fixture(params = {
        "mean" : range(0, 1000, 50),
        "std_dev" : range(0, 1000, 50),
        "value" : range(0, 1000, 50)
    }, scope="class")
    def value_from_normal(self, request):
        return CParticleFilter().value_from_normal_distribution(request.param["mean"], request.param["std_dev"], request.param["value"])
