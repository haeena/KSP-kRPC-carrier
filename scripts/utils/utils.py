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

    y = np.sin(dlon) * np.cos(lat2)
    x = np.cos(lat1) * np.sin(lat2) - np.sin(lat1) * np.cos(lat2) * np.cos(dlon)
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
        lattitude in degree
        longtitude in degree
    """ 
    lon = np.arctan2(vector[2], vector[0])
    p = (vector[0]**2.0 + vector[2]** 2.0)**0.5
    lat = np.arctan2(vector[1], p)
    lat, lon = np.rad2deg((lat, lon))
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

class PIDController(object):
    """ Robust, single parameter, proportional-integral-derivative controller
        http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/ """

    def __init__(self, ut:float = 0, Kp:float = 1, Ki:float = 0, Kd:float = 0):
        self.set_params(Kp = Kp, Ki = Ki, Kd = Kd)
        self.Ti = 0
        self.last_ut = ut
        self.last_error = 0

    def set_params(self, Kp:float = None, Ki:float = None, Kd:float = None):
        if Kp != None:
            self.Kp = Kp
        if Ki != None:
            self.Ki = Ki
        if Kd != None:
            self.Kd = Kd

    def update(self, input, set_point, ut, min_output, max_output):
        d_ut = ut - self.last_ut
        error = set_point - input
        d_error = error - self.last_error
        self.Ti += self.Ki * error * d_ut
        self.Ti = np.maximum(min_output, np.minimum(max_output, self.Ti))
        if d_ut != 0:
            output = self.Kp * error + self.Ti - self.Kd * d_error / d_ut
        else:
            output = self.Kp * error + self.Ti
        output = np.maximum(min_output, np.minimum(max_output, output))
        self.last_error = error
        self.latst_ut = ut
        return output
