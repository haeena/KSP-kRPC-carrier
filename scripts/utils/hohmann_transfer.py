import math
from functools import lru_cache

# TODO: type hint for kRPC remote objects may need to be separated
from typing import Union, NewType

Vessel = NewType("Vessel", object)
Body = NewType("Body", object)
Node = NewType("Node", object)

from krpc.client import Client

from scripts.utils.utils import *
from scripts.utils.status_dialog import StatusDialog
from scripts.utils.execute_node import execute_next_node
from scripts.utils.maneuver import prograde_vector_at_ut

def get_phase_angle(vessel: Vessel, target: Union[Vessel, Body]) -> float:
    '''
    returns the relative phase angle for a hohmann transfer
    '''
    vo=vessel.orbit
    to=target.orbit
    h=(vo.semi_major_axis+to.semi_major_axis)/2   # SMA of transfer orbit

    #calculate the percentage of the target orbit that goes by during the half period of transfer orbit
    p = 1/(2*math.sqrt(math.pow(to.semi_major_axis,3)/math.pow(h,3)))

    # convert that to an angle in radians
    a =  (2 * math.pi) - ((2* math.pi) *p)

    return a

def orbital_progress(vessel: Vessel, ut: float) -> float:
    '''
    returns the orbital progress in radians, referenced to the planet's origin
    of longitude.
    '''
    lan = vessel.orbit.longitude_of_ascending_node
    arg_p = vessel.orbit.argument_of_periapsis
    ma_ut = vessel.orbit.mean_anomaly_at_ut(ut)

    return clamp_2pi(lan + arg_p + ma_ut)

def time_transfer(vessel: Vessel, target: Union[Vessel, Body], ut: float, phase_angle: float) -> float:
    """
    Performs an iterative search for the next time vessel and target
    have the given relative phase_angle after ut
    """

    # search near close phase_angle 
    min_time = ut
    max_time = min_time + 1.5 * min(vessel.orbit.period, target.orbit.period)

    num_divisions = 30
    dt = (max_time - min_time) / num_divisions

    v_pos = orbital_progress(vessel, ut)
    t_pos =  orbital_progress(target, ut)
    angle_error = abs(t_pos - (v_pos - math.pi) - phase_angle)

    last_angle_error = angle_error
    min_abs_angle_error = abs(angle_error)
    last_angle_error_t_sign = None

    for i in range(1, num_divisions):
        t = min_time + dt * i

        v_pos = orbital_progress(vessel, ut)
        t_pos =  orbital_progress(target, ut)
        angle_error = abs(t_pos - (v_pos - math.pi) - phase_angle)

        angle_error_t_sign = math.copysign(1, angle_error - last_angle_error)
        last_angle_error = angle_error

        abs_angle_error = abs(angle_error)

        if abs_angle_error < math.pi / 2 and last_angle_error_t_sign and last_angle_error_t_sign != angle_error_t_sign:
            angle_error_t_sign = last_angle_error_t_sign
            break

        if abs_angle_error < min_abs_angle_error:
            min_abs_angle_error = abs_angle_error

            min_time = t - dt
            max_time = t + dt

        last_angle_error_t_sign = angle_error_t_sign

    while (max_time - min_time > 0.01):
        t = (max_time + min_time) / 2

        v_pos = orbital_progress(vessel, ut)
        t_pos =  orbital_progress(target, ut)
        angle_error = abs(t_pos - (v_pos - math.pi) - phase_angle)

        if math.copysign(1, angle_error) == angle_error_t_sign:
            max_time = t
        else:
            min_time = t

        last_angle_error = angle_error

    t = (max_time + min_time) / 2
    return t

def hohmann_transfer(vessel: Vessel, target: Union[Vessel, Body], node_ut: float) -> Node:
    '''
    Create a maneuver node for a hohmann transfer from vessel orbit to target
    orbit at the given time
    '''
    body = vessel.orbit.body
    GM = body.gravitational_parameter
    r1  = vessel.orbit.radius_at(node_ut)
    SMA_i = vessel.orbit.semi_major_axis
    SMA_t = (vessel.orbit.apoapsis + target.orbit.apoapsis) / 2
    v1 = math.sqrt(GM * ((2/r1) - (1 / SMA_i)))
    v2 = math.sqrt(GM * ((2/r1) - (1 / (SMA_t))))
    dv = v2 - v1

    return vessel.control.add_node(node_ut, prograde = dv)

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

    phase_angle = get_phase_angle(vessel, target)
    transfer_time = time_transfer(vessel, target, ut(), phase_angle)
    node = hohmann_transfer(vessel, target, transfer_time)

    execute_next_node(conn)

if __name__ == "__main__":
    import os
    import krpc
    krpc_address = os.environ["KRPC_ADDRESS"]
    conn = krpc.connect(name='hohman transfer', address=krpc_address)
    hohmann_transfer_to_target(conn)


