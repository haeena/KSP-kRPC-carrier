import math
from functools import lru_cache

# TODO: type hint for kRPC remote objects may need to be separated
from typing import Union, NewType

Vessel = NewType("Vessel", object)
Body = NewType("Body", object)

from krpc.client import Client

from scripts.utils.utils import *
from scripts.utils.status_dialog import StatusDialog
from scripts.utils.execute_node import execute_next_node
from scripts.utils.krpc_poliastro import krpc_poliastro_bodies
from scripts.utils.maneuver import prograde_vector_at_ut

from astropy import time
from astropy import units as AstropyUnit
from poliastro.twobody import Orbit as PoliastroOrbit
from poliastro.maneuver import Maneuver

def hohmann_transfer_to_target_at_ut(conn: Client, vessel: Vessel, target: Union[Vessel, Body],
                                     ct: float, node_ut: float, trans_time: float = 0
                                    ) -> (float, float, float, float):
    """calculate (dv_a, dv_b), trans_time and sort of mean anomaly phase angle of hohmann transfer at time t

    calculate (dv_a, dv_b), trans_time and sort of mean anomaly phase angle of hohmann transfer at time t
    
    Args:
        conn: kRPC connection
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

    krpc_bodies, poliastro_bodies = krpc_poliastro_bodies(conn)
    start_time_poliastro = time.Time(node_ut, format="gps")

    r_v_ct = vessel.position(reference_frame) * AstropyUnit.m
    v_v_ct = vessel.velocity(reference_frame) * AstropyUnit.m / AstropyUnit.s
    ss_v_ct = PoliastroOrbit.from_vectors(poliastro_bodies[attractor.name], r_v_ct, v_v_ct, epoch=start_time_poliastro)
    r_target = target.position(reference_frame) * AstropyUnit.m
    v_target = target.velocity(reference_frame) * AstropyUnit.m / AstropyUnit.s
    ss_target = PoliastroOrbit.from_vectors(poliastro_bodies[attractor.name], r_target, v_target, epoch=start_time_poliastro)

    ss_i = ss_v_ct.propagate(time_to_burn * AstropyUnit.s)
    r_f = target.orbit.radius_at(node_ut + trans_time) * AstropyUnit.m

    hohmann_maneuver = Maneuver.hohmann(ss_i, r_f)
    dv_a = norm(hohmann_maneuver[0][1])
    dv_b = norm(hohmann_maneuver[1][1])
    trans_time = hohmann_maneuver.get_total_time().value
    ss_a, ss_f = ss_i.apply_maneuver(hohmann_maneuver, intermediate=True)

    end_time_poliastro = time.Time(node_ut + trans_time, format="gps")
    end_time_1secb4_poliastro = time.Time(node_ut + trans_time - 1, format="gps")

    ss_a_sample = ss_a.sample([start_time_poliastro, end_time_1secb4_poliastro, end_time_poliastro])
    s_v_final = ss_a_sample[-1][-1]
    s_v_final_prev = ss_a_sample[-1][-2]
    v_v_final = np.subtract(s_v_final, s_v_final_prev)
    s_target_sample = ss_target.sample([start_time_poliastro, end_time_poliastro])
    s_target_final = s_target_sample[-1][-1]
    s_vt_final_d = np.subtract(s_v_final, s_target_final)

    phase_angle = math.copysign(angle_between(s_v_final.xyz.value, s_target_final.xyz.value), dot(v_v_final.xyz.value, s_vt_final_d.xyz.value))

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

    dv_a, dv_b, trans_time, pa = hohmann_transfer_to_target_at_ut(conn, vessel, target, ut(), min_time)
    last_pa = pa
    min_abs_pa = abs(pa)
    last_pa_t_sign = None

    for i in range(1, num_divisions):
        t = min_time + dt * i
        dv_a, dv_b, trans_time, pa = hohmann_transfer_to_target_at_ut(conn, vessel, target, ut(), t, trans_time)
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
        dv_a, dv_b, trans_time, pa = hohmann_transfer_to_target_at_ut(conn, vessel, target, ut(), t, trans_time)

        if math.copysign(1, pa) == pa_t_sign:
            max_time = t
        else:
            min_time = t

        last_pa = pa

    t = (max_time + min_time) / 2
    dv_a, dv_b, trans_time, pa = hohmann_transfer_to_target_at_ut(conn, vessel, target, ut(), t, trans_time)
    node = vessel.control.add_node(t, prograde=dv_a)
    # vessel.control.add_node(t+trans_t, prograde=dv_b)

    execute_next_node(conn)

if __name__ == "__main__":
    import os
    import krpc
    krpc_address = os.environ["KRPC_ADDRESS"]
    conn = krpc.connect(name='hohman transfer', address=krpc_address)
    hohmann_transfer_to_target(conn)


