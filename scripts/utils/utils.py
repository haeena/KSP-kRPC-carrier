import math
import numpy as np

def norm(v):
    return np.linalg.norm(v)

def unit_vector(v):
    return v / norm(v)

def dot(v1, v2):
    v1_u = unit_vector(v1)
    v2_u = unit_vector(v2)
    return np.clip(np.dot(v1_u, v2_u), -1.0, 1.0)

def angle_between(v1, v2):
    """Calculate angle between two vector.

    Extended description of function.

    Args:
        v1: vector
        v2: vector

    Returns:
        radian between two vector
    """
    v1_u = unit_vector(v1)
    v2_u = unit_vector(v2)
    return np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))

def clamp_2pi(x):
    """
    clamp radians to a single revolution
    """
    while x > (2 * math.pi):
        x -= (2 * math.pi)

    return x