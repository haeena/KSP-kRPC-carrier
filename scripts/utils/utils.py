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

def angle_between_coords(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """calculate radian between 2 (lat,lon) coords in radian

    Extended description of function.

    Args:
        lat1: latitude of first coordination
        lon1: longtitude of first coordination
        lat2: latitude of first coordination
        lon2: longtitude of first coordination

    Returns:
        radian between two coordinations
    """
    lat1, lon1, lat2, lon2 = np.deg2rad((lat1, lon1, lat2, lon2))
    return np.arccos(np.sin(lat1)*np.sin(lat2) + np.cos(lat1)*np.cos(lat2) * np.cos(abs(lon1 - lon2)))

def distance_between_coords(lat1: float, lon1: float, lat2: float, lon2: float, body_radius: float) -> float:
    """calculate distance between 2 (lat,lon) coords

    Extended description of function.

    Args:
        lat1: latitude of first coordination
        lon1: longtitude of first coordination
        lat2: latitude of second coordination
        lon2: longtitude of second coordination
        body_radius: radius of body

    Returns:
        distance between two coordinations
    """ 
    lat1, lon1, lat2, lon2 = np.deg2rad((lat1, lon1, lat2, lon2))
    dlon = lon2 - lon1
    dlat = lat2 - lat1
    a = np.sin(0.5*dlat)**2.0 + np.cos(lat1)*np.cos(lat2)*(np.sin(0.5*dlon)**2.0)
    c = 2.0*np.arctan2(np.sqrt(a), np.sqrt(1-a))
    distance = body_radius * c
    return distance

def bearing_between_coords(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """calculate initial bearing between 2 (lat,lon) coords

    Extended description of function.

    Args:
        lat1: latitude of first coordination
        lon1: longtitude of first coordination
        lat2: latitude of second coordination
        lon2: longtitude of second coordination
        body_radius: radius of body

    Returns:
        initial bearing from first to second (degree)
    """ 
    lat1, lon1, lat2, lon2 = np.deg2rad((lat1, lon1, lat2, lon2))
    dlon = lon2 - lon1
    y = math.sin(dlon) * math.cos(lat2)
    x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dlon)
    bearing = np.rad2deg(np.arctan2(y, x)) % 360
    return bearing

def bearing_and_distance_between_coords(lat1: float, lon1: float, lat2: float, lon2: float, body_radius: float) -> ():
    """calculate distance and initial bearing between 2 (lat,lon) coords

    Extended description of function.

    Args:
        lat1: latitude of first coordination
        lon1: longtitude of first coordination
        lat2: latitude of second coordination
        lon2: longtitude of second coordination
        body_radius: radius of body

    Returns:
        distance between two coordinations
        initial bearing from first to second (degree)
    """ 
    return distance_between_coords(lat1, lon1, lat2, lon2, body_radius), bearing_between_coords(lat1, lon1, lat2, lon2)

def latlon(vector):
    """function to calculate (lat, lon) from state vectors (x,y,z)

    Extended description of function.

    Args:
        vector: (x,y,z) state vector

    Returns:
        lattitude in radian
        longtitude in radian
    """ 
    lon = np.arctan2(vector[1],vector[0])
    p = (vector[0]**2.0 + vector[1]** 2.0)**0.5
    lat = np.arctan2(vector[2] ,p)
    return lat, lon

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