import time
import math

# TODO: type hint for kRPC remote objects may need to be separated
from typing import Union, NewType

Vessel = NewType("Vessel", object)
Body = NewType("Body", object)

from krpc.client import Client
from status_dialog import StatusDialog
from execute_node import execute_next_node

import numpy as np

def unit_vector(vector):
    return vector / np.linalg.norm(vector)

def dot(v1, v2):
    v1_u = unit_vector(v1)
    v2_u = unit_vector(v2)
    return np.clip(np.dot(v1_u, v2_u), -1.0, 1.0)

def angle_between(v1, v2):
    v1_u = unit_vector(v1)
    v2_u = unit_vector(v2)
    return np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))

def hohmann_transfer_to_target_alt_at_ut(vessel: Vessel, target: Union[Vessel, Body], ct: float, ut: float) -> ((float, float), float, float):
    """calculate (dv_a, dv_b), trans_time and sort of mean anomaly phase angle of hohmann transfer at time t

    calculate (dv_a, dv_b), trans_time and sort of mean anomaly phase angle of hohmann transfer at time t
    
    Args:
        vessel: active vessel for transfer
        target: target vessel or celesital body
        ct: currrent time
        ut: time to burn

    Retruns:
        ((dv_a, dv_b), trans_time, phase_angle)
        dv_a: deltaV (m/s) for first burn
        dv_b: deltaV (m/s) for second burn
        trans_time: time (s) for transfer
        phase_angle: phase_angle between vessel and target at t + trans_time
    """ 
    ## TODO: use astropy instead of RPC?
    # radius_at calls RPC, so it takes lot of times...
    
    # assuming vessel and target are both in circular orbit of same body, in same plane
    r_i = vessel.orbit.radius_at(ut)
    v_i = vessel.orbit.orbital_speed_at(ut)
    r_f = target.orbit.radius_at(ut)
    k = vessel.orbit.body.gravitational_parameter
    R = r_f / r_i
    dv_a = ((math.sqrt(2 * R / (1 + R)) - 1) * v_i)
    dv_b = (1 - math.sqrt(2 / (1 + R))) / math.sqrt(R) * v_i
    t_trans = (math.pi * math.sqrt((r_i * (1 + R) / 2) ** 3 / k))
    
    ref_frame = vessel.orbit.body.non_rotating_reference_frame
    vp = vessel.orbit.position_at(ct + vessel.orbit.time_to_periapsis, ref_frame)
    vp_1 = vessel.orbit.position_at(1 + ct + vessel.orbit.time_to_periapsis, ref_frame)
    vv = np.subtract(vp_1, vp)
    tp = target.orbit.position_at(ct + target.orbit.time_to_periapsis, ref_frame)
    vt_angle = math.copysign(angle_between(vp, tp), dot(vp, tp))

    vessel_ta_at_t_trans = vessel.orbit.true_anomaly_at_ut(ut) + math.pi
    vessel_ta_at_ct =  vessel.orbit.true_anomaly_at_ut(ct)
    vessel_ta_trans = (vessel_ta_at_t_trans - vessel_ta_at_ct) % (2 * math.pi) - math.pi
    target_ta_at_t_trans = target.orbit.true_anomaly_at_ut(ut + t_trans)
    target_ta_at_ct =  target.orbit.true_anomaly_at_ut(ct)
    target_ta_trans = (target_ta_at_t_trans - target_ta_at_ct) % (2 * math.pi)

    phase_angle = (target_ta_trans + vessel_ta_trans + vt_angle) % (2 * math.pi) - math.pi

    return ((dv_a, dv_b), t_trans, phase_angle)

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

    _, _, pa = hohmann_transfer_to_target_alt_at_ut(vessel, target, ut(), min_time)
    last_pa = pa
    min_abs_pa = abs(pa)
    last_pa_t_sign = None

    for i in range(1, num_divisions):
        t = min_time + dt * i
        _, _, pa = hohmann_transfer_to_target_alt_at_ut(vessel, target, ut(), t)
        pa_t_sign = math.copysign(1, pa - last_pa)
        last_pa = pa

        abs_pa = abs(pa)
        if abs_pa < min_abs_pa:
            min_abs_pa = abs_pa

            if math.copysign(1, pa) == pa_t_sign:
                min_time = t - dt
                max_time = t
            else:
                min_time = t
                max_time = t + dt
        
        if last_pa_t_sign:
            if last_pa_t_sign != pa_t_sign:
                break
        last_pa_t_sign = pa_t_sign

    while (max_time - min_time > 0.01):
        t = (max_time + min_time) / 2
        dvs, _, pa = hohmann_transfer_to_target_alt_at_ut(vessel, target, ut(), t)

        # t = 58950
        # TMP
        node = vessel.control.add_node(t, prograde=dvs[0])
        node.remove()

        if math.copysign(1, pa) == pa_t_sign:
            min_time = t
        else:
            max_time = t

    t = (max_time + min_time) / 2
    (dv_a, dv_b), trans_t, pa = hohmann_transfer_to_target_alt_at_ut(vessel, target, ut(), t)
    node = vessel.control.add_node(t, prograde=dv_a)
    # vessel.control.add_node(t+trans_t, prograde=dv_b)

    ut.remove()

    execute_next_node(conn)

if __name__ == "__main__":
    import krpc
    conn = krpc.connect(name='hohman transfer')
    hohmann_transfer_to_target(conn)


