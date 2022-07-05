import sys, os
import numpy as np
import pytest
from scipy.stats import norm

# TODO How to find modules in a less hacky way?
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

