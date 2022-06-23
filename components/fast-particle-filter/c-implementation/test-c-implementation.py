import sys, os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from pftest.testRunner import ParticleFilterTestRunner
from pfCFFI import CParticleFilter

test_runner = ParticleFilterTestRunner(CParticleFilter())
test_runner.run_test_suite()
