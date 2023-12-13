import sys
from math import *
from robot import robot
import random
import numpy as np
import matplotlib.pyplot as plt
from datetime import datetime

ideal = robot()
real = robot()

ideal.set(0., 0., 0.)
ideal.set_noise(0.0, 0.0, 0.0)

real.set(0., 0., 0.)
real.set_noise(0.0, 0.0, 0.0)

ideal.__repr__()
real.__repr__()