import math

# TODO: type hint for kRPC remote objects may need to be separated
from typing import Union, NewType

Vessel = NewType("Vessel", object)
Body = NewType("Body", object)

import numpy as np

from krpc.client import Client
from scripts.utils.status_dialog import StatusDialog
from scripts.utils.execute_node import execute_next_node

def unit_vector(vector):
    return vector / np.linalg.norm(vector)

def dot(v1, v2):
    v1_u = unit_vector(v1)
    v2_u = unit_vector(v2)
    return np.clip(np.dot(v1_u, v2_u), -1.0, 1.0)

def circularize(conn: Client, at_apoapsis: bool = False, at_periapsis: bool = False, at_ut: float = None):
    """Execute circularize burn

    Execute circularize burn.
    At least one of (at_apoapsis, at_periapsis, at_ut) must be specified

    Args:
        conn: kRPC connection
        at_apoapsis: schedule burn at next apoapsis
        at_periapsis: schedule burn at next periapsis
        at_ut: schedule burn at specific time

    Returns:
        return nothing, return when procedure finished
    """
    vessel = conn.space_center.active_vessel
    attractor = vessel.orbit.body
    reference_frame = attractor.non_rotating_reference_frame

    # setup stream
    ut = conn.add_stream(getattr, conn.space_center, 'ut')

    note_ut = None
    if at_ut:
        node_ut = at_ut
    elif at_periapsis:
        node_ut = ut() + vessel.orbit.time_to_periapsis
    elif at_apoapsis:
        node_ut = ut() + vessel.orbit.time_to_apoapsis
    else:
        return

    circularize_radius = vessel.orbit.radius_at(node_ut)

    # v = sqrt(GM/r)
    circular_orbit_speed = math.sqrt(attractor.gravitational_parameter / circularize_radius)

    # TODO: wanna access KSP Orbit.getOrbitalVelocityAtUT() ...
    actual_orbit_speed = np.subtract(vessel.orbit.position_at(node_ut +0.5, reference_frame), vessel.orbit.position_at(node_ut-0.5, reference_frame))

    anti_radial_vector_at_node = unit_vector(vessel.orbit.position_at(node_ut, reference_frame))
    prograde_vector_at_node = unit_vector(actual_orbit_speed)
    horizontal_vector_at_node = unit_vector(prograde_vector_at_node - anti_radial_vector_at_node * dot(anti_radial_vector_at_node,prograde_vector_at_node))

    desired_orbit_speed = horizontal_vector_at_node * circular_orbit_speed

    dv_vector = desired_orbit_speed - actual_orbit_speed
    dv_prograde = dot(dv_vector, prograde_vector_at_node) * np.linalg.norm(dv_vector)
    dv_anti_radial = dot(dv_vector, anti_radial_vector_at_node) * np.linalg.norm(dv_vector)

    node = vessel.control.add_node(node_ut, radial=dv_anti_radial, prograde=dv_prograde, normal=0)

    ut.remove()

    execute_next_node(conn)

if __name__ == "__main__":
    import krpc
    conn = krpc.connect(name='circularize at next apoapsis')
    #circularize(conn, at_apoapsis=True)
    circularize(conn, at_ut=conn.space_center.ut + 300)
    