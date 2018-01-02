import math
from functools import lru_cache

# TODO: type hint for kRPC remote objects may need to be separated
from typing import Union, NewType

Vessel = NewType("Vessel", object)
Body = NewType("Body", object)

from krpc.client import Client
from scripts.utils.status_dialog import StatusDialog
from scripts.utils.execute_node import execute_next_node
from scripts.utils.krpc_poliastro import krpc_poliastro_bodies
from scripts.utils.maneuver import prograde_vector_at_ut

import numpy as np
from numpy.linalg import norm

from astropy import units as AstropyUnit
from poliastro.twobody import Orbit as PoliastroOrbit
from poliastro.maneuver import Maneuver

def unit_vector(vector):
    return vector / norm(vector)

def dot(v1, v2):
    v1_u = unit_vector(v1)
    v2_u = unit_vector(v2)
    return np.clip(np.dot(v1_u, v2_u), -1.0, 1.0)

def angle_between(v1, v2):
    v1_u = unit_vector(v1)
    v2_u = unit_vector(v2)
    return np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))

def hohmann_transfer_to_target_at_ut(vessel: Vessel, target: Union[Vessel, Body],
                                     ct: float, node_ut: float, trans_time: float = 0
                                    ) -> (float, float, float, float):
    """calculate (dv_a, dv_b), trans_time and sort of mean anomaly phase angle of hohmann transfer at time t

    calculate (dv_a, dv_b), trans_time and sort of mean anomaly phase angle of hohmann transfer at time t
    
    Args:
        vessel: active vessel for transfer
        target: target vessel or celesital body
        ct: currrent time
        node_ut: time to burn
        trans_time: appox transfer time

    Retruns:
        (dv_a, dv_b, trans_time, phase_angle)
        dv_a: deltaV (m/s) for first burn
        dv_b: deltaV (m/s) for second burn
        trans_time: time (s) for transfer
        phase_angle: phase_angle between vessel and target at t + trans_time
    """ 
    # assuming vessel and target are both in circular orbit of same body, in same plane
    attractor = vessel.orbit.body
    reference_frame = attractor.non_rotating_reference_frame

    time_to_burn = node_ut - ct

    r_i = vessel.orbit.radius_at(node_ut)
    v_i = vessel.orbit.orbital_speed_at(node_ut)
    r_f = target.orbit.radius_at(node_ut + trans_time)
    k = attractor.gravitational_parameter
    R = r_f / r_i
    dv_a = ((math.sqrt(2 * R / (1 + R)) - 1) * v_i)
    dv_b = (1 - math.sqrt(2 / (1 + R))) / math.sqrt(R) * v_i
    trans_time = (math.pi * math.sqrt((r_i * (1 + R) / 2) ** 3 / k))

    krpc_bodies, poliastro_bodies = krpc_poliastro_bodies()
    prograde_vector_at_node = prograde_vector_at_ut(vessel.orbit, node_ut)
    dv_vector = dv_a * prograde_vector_at_node
    hohmann_maneuver = Maneuver((time_to_burn * AstropyUnit.s, dv_vector * AstropyUnit.m / AstropyUnit.s ))

    r_v_ct = vessel.position(reference_frame) * AstropyUnit.m
    v_v_ct = vessel.velocity(reference_frame) * AstropyUnit.m / AstropyUnit.s
    ss_v_ct = PoliastroOrbit.from_vectors(poliastro_bodies[attractor.name], r_v_ct, v_v_ct)
    ss_i = ss_v_ct.propagate(time_to_burn * AstropyUnit.s)

    ss_a = ss_i.apply_maneuver(hohmann_maneuver)
    
    p_vessel_0 = ss_a.propagate(trans_time * AstropyUnit.s).sample(1).xyz.value.take([0,1,2])
    p_target_0 = target.orbit.position_at(ct + trans_time, reference_frame)
    p_target_1 = target.orbit.position_at(ct + trans_time + 1, reference_frame)
    v_target_0 = np.subtract(p_target_1, p_target_0)
    d_vessel_target_0 = np.subtract(p_vessel_0, p_target_0)

    phase_angle = math.copysign(angle_between(p_vessel_0, p_target_0), dot(d_vessel_target_0, v_target_0))

    return (dv_a, dv_b, trans_time, phase_angle)

def hohmann_transfer_to_target(conn: Client) -> None:
    """send active vessel into hohmann transfer orbit to the target.

    Extended description of function.

    Args:
        conn: kRPC connection

    Returns:
        return nothing, return when procedure finished
    """
    vessel = conn.space_center.active_vessel

    # setup stream
    ut = conn.add_stream(getattr, conn.space_center, 'ut')

    # Set up dialog
    dialog = StatusDialog(conn)
    dialog.status_update("calculating hohmann transfer orbit to target")

    # check target object
    target = None
    target_type = None
    if conn.space_center.target_body:
        target = conn.space_center.target_body
        target_type = "CelestialBody"
    elif conn.space_center.target_vessel:
        target = conn.space_center.target_vessel
        target_type = "Vessel"
    else:
        return

    # check if vessel and target is orbiting of same body
    if vessel.orbit.body != target.orbit.body:
        return

    # search near close phase_angle 
    min_time = ut()
    max_time = min_time + 1.5 * min(vessel.orbit.period, target.orbit.period)

    num_divisions = 30
    dt = (max_time - min_time) / num_divisions

    dv_a, dv_b, trans_time, pa = hohmann_transfer_to_target_at_ut(vessel, target, ut(), min_time)
    last_pa = pa
    min_abs_pa = abs(pa)
    last_pa_t_sign = None

    for i in range(1, num_divisions):
        t = min_time + dt * i
        dv_a, dv_b, trans_time, pa = hohmann_transfer_to_target_at_ut(vessel, target, ut(), t, trans_time)
        pa_t_sign = math.copysign(1, pa - last_pa)
        last_pa = pa

        abs_pa = abs(pa)

        if abs_pa < math.pi / 2 and last_pa_t_sign and last_pa_t_sign != pa_t_sign:
            pa_t_sign = last_pa_t_sign
            break

        if abs_pa < min_abs_pa:
            min_abs_pa = abs_pa

            min_time = t - dt
            max_time = t + dt

        last_pa_t_sign = pa_t_sign

    while (max_time - min_time > 0.01):
        t = (max_time + min_time) / 2
        dv_a, dv_b, trans_time, pa = hohmann_transfer_to_target_at_ut(vessel, target, ut(), t, trans_time)

        if math.copysign(1, pa) == pa_t_sign:
            max_time = t
        else:
            min_time = t

        last_pa = pa

    t = (max_time + min_time) / 2
    dv_a, dv_b, trans_time, pa = hohmann_transfer_to_target_at_ut(vessel, target, ut(), t, trans_time)
    node = vessel.control.add_node(t, prograde=dv_a)
    # vessel.control.add_node(t+trans_t, prograde=dv_b)

    # cleanup stream
    # TODO: is this needed?
    ut.remove()

    execute_next_node(conn)

if __name__ == "__main__":
    import krpc
    conn = krpc.connect(name='hohman transfer')
    hohmann_transfer_to_target(conn)


