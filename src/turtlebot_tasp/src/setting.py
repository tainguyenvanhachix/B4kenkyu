#!/usr/bin/env python3
from math import pi
from tf.transformations import quaternion_from_euler

SAMPLES_NUMBER = 20
CELL_LENGTH = 0.310
SAFE_DISTANCE_FOOTBASE = 0.5
LIDAR_TO_FOOTBASE = 0.064
SAFE_DISTANCE_FRONT = SAFE_DISTANCE_FOOTBASE + LIDAR_TO_FOOTBASE
SAFE_DISTANCE_BEHIND = SAFE_DISTANCE_FOOTBASE - LIDAR_TO_FOOTBASE
LINEAR_VEL = 0.05
ANGULAR_VEL = 0.2
QUAT_90DEGREE = quaternion_from_euler (0, 0, pi/2)
OMEGA_0 = 0.06
OMEGA_1 = 0.02
KA1 = 0.6
KA2 = 0.3