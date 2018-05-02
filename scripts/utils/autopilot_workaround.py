import time
import math

# TODO: type hint for kRPC remote objects may need to be separated
from typing import Union, NewType, Tuple

Vessel = NewType("Vessel", object)
Body = NewType("Body", object)
Orbit = NewType("Orbit", object)
ReferenceFrame = NewType("ReferenceFrame", object)

from krpc.client import Client

import numpy as np
from numpy.linalg import norm
import quaternion

def autopilot_workaround(conn: Client,
                         target_pitch: float = None, target_heading: float = None, target_roll: float = None,
                         target_direction: Tuple[float, float, float] = None,
                         reference_frame: ReferenceFrame = None):
    """workaround script for kRPC autopilot

    Return velocity vector of orbit at ut
    If we could access KSP Orbit.getOrbitalVelocityAtUT() via kRPC api...

    Args:
        conn: kRPC connection object
        target_pitch: target pitch in degree
        target_heading: target heading in degree
        target_roll: target roll in degree
        target_direction: target direction vector
        reference_frame: reference_frame to be used, default is vessel.reference_frame

    Returns:
        return velocity unit vector
    """
    vessel = conn.space_center.active_vessel
    control = vessel.control
    drawing = conn.drawing

    if target_direction:
        target_up = np.array((0,1,0))
        d = target_direction
        s = np.cross(target_direction, target_up)
        u = np.cross(d, s)
        m = np.matrix((d, u, s))
        target_rotation = quaternion.from_rotation_matrix(m)
    else:
        if not reference_frame:
            reference_frame = vessel.surface_reference_frame
        # TODO: set default to current and convert from degree to radian, cramp to degree
        if not target_pitch:
            target_pitch = 0
        else:
            pass
        # TODO: 
        target_rotation = quaternion.from_euler_angles(target_pitch, target_heading, target_roll)
    
    line_target = drawing.add_direction(target_direction, reference_frame)
    line_target.color = (255,255,255)
    line_current = drawing.add_direction((0,1,0), vessel.reference_frame)
    line_current.color = (255,128,255)

    ut_stream = conn.add_stream(getattr, conn.space_center, 'ut')
    direction_stream = conn.add_stream(vessel.direction, reference_frame)
    rotation_stream = conn.add_stream(vessel.rotation, reference_frame)
    angular_velocity_stream = conn.add_stream(vessel.angular_velocity, reference_frame)

    ## initialize
    integrator = np.array((0,0,0))

    # current rotation
    current_rotation = quaternion.quaternion(*rotation_stream())
    # current angular_velocity in quartenion
    current_angular_velocity_q = quaternion.quaternion(0, *angular_velocity_stream())
    # current angular_velocity relative to vessel
    current_angular_velocity_vessel_q = current_rotation.sqrt().inverse() * current_angular_velocity_q * current_rotation.sqrt()
    current_angular_velocity_vessel = np.array((current_angular_velocity_vessel_q.x, current_angular_velocity_vessel_q.y, current_angular_velocity_vessel_q.z))

    last_angular_velocity_vessel = current_angular_velocity_vessel

    last_ut = ut_stream()

    ## loop
    while True:
        ut = ut_stream()
        dt = ut - last_ut
        if dt == 0:
            continue
        last_ut = ut

        # current direction
        current_direction = direction_stream()
        # current rotation
        current_rotation = quaternion.quaternion(*rotation_stream())
        # current angular_velocity in quartenion
        current_angular_velocity_q = quaternion.quaternion(0, *angular_velocity_stream())
        # current angular_velocity relative to vessel
        current_angular_velocity_vessel_q = current_rotation.sqrt().inverse() * current_angular_velocity_q * current_rotation.sqrt()
        current_angular_velocity_vessel = np.array((current_angular_velocity_vessel_q.x, current_angular_velocity_vessel_q.y, current_angular_velocity_vessel_q.z))
		# current angular_acceleration relative to vessel
        current_anglular_acceleration_vessel = (last_angular_velocity_vessel - current_angular_velocity_vessel) / dt

        last_angular_velocity_vessel = current_angular_velocity_vessel

        steer_rotation = current_rotation.inverse() * target_rotation

        c = quaternion.as_euler_angles(steer_rotation)

        #control.pitch = math.copysign(0.1, c[0])
        #control.yaw =  math.copysign(0.1, c[1])
        #control.roll = math.copysign(0.1, c[2])

        print(c)

        time.sleep(1)

if __name__ == "__main__":
    import krpc
    conn = krpc.connect(name='autopilot')
    vessel = conn.space_center.active_vessel
    reference_frame = vessel.surface_reference_frame
    target_direction = vessel.control.nodes[0].direction(reference_frame)
    autopilot_workaround(conn, target_direction=target_direction, reference_frame=reference_frame)

    #autopilot_workaround(conn, target_pitch = 45, target_heading=0, target_roll = 0)