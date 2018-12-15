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

def r2d(x):
    """radian to degree
    """
    return x*180.0/np.pi
    
def d2r(x):
    """ degree to radian
    """
    return x*np.pi/180.0

def db2(lat1, lon1, lat2, lon2, body_radius):
    """calculate distance between 2 (lat,lon) coords
    """
    dlon = lon2 - lon1
    dlat = lat2 - lat1
    a = np.sin(0.5*dlat)**2.0 + np.cos(lat1)*np.cos(lat2)*(np.sin(0.5*dlon)**2.0)
    c = 2.0*np.arcsin(min(1.0, np.sqrt(a)))
    d = body.equatorial_radius*c
    return d
    
def latlon(x):
    """function to calculate (lat, long) from state vectors
    """
    lon = np.arctan2(x[1],x[0])
    p = (x[0]**2.0 + x[1]** 2.0)**0.5
    lat = np.arctan2(x[2],p)
    lat = r2d(lat)
    lon = r2d(lon)
    return np.array((lat, lon))

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