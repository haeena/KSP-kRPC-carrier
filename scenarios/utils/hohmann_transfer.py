import time
import math

# TODO: type hint for kRPC remote objects may need to be separated
from typing import Union, NewType

Vessel = NewType("Vessel", object)
Body = NewType("Body", object)



from krpc.client import Client
from status_dialog import StatusDialog

## TODO: astropy related code should be separeted
from astropy import units as u
from astropy.constants import Constant
from poliastro.bodies import Body as PoliastroBody
from poliastro.twobody import Orbit as PoliastroOrbit
from poliastro.maneuver import Maneuver


## TODO: astropy related code should be separeted
def _convert_body_krpc_to_poliastro(poliastro_bodies: dict, parent: PoliastroBody, krpc_body: Body):
    name = krpc_body.name
    GM = Constant('GM_k{}'.format(name), 'Kerbal {} gravitational constant'.format(name),
                  krpc_body.gravitational_parameter, 'm3 / (s2)', 0,
                  'kRPC space_center.bodies["{}"].gravitational_parameter'.format(name), system='si')
    R = Constant('R_k{}'.format(name), 'Kerbal {} equatorial radius'.format(name),
                      krpc_body.equatorial_radius, 'm', 0,
                     'kRPC space_center.bodies["{}"].equatorial_radius'.format(name), system='si')
    poliastro_body = PoliastroBody(parent, GM, "", name, R)
    poliastro_bodies[name] = poliastro_body
    for satelite in krpc_body.satellites:
        _convert_body_krpc_to_poliastro(poliastro_bodies, poliastro_body, satelite)
    return

## TODO: astropy related code should be separeted
def krpc_poliastro_bodies(conn: Client) -> (dict, dict):
    poliastro_bodies = {}
    krpc_bodies = conn.space_center.bodies

    krpc_Sun = krpc_bodies["Sun"]
    _convert_body_krpc_to_poliastro(poliastro_bodies, None, krpc_Sun)
    return (krpc_bodies, poliastro_bodies)

def hohmann_transfer_to_target_alt_at_ut(vessel: Vessel, target: Union[Vessel, Body], ut: float) -> ((float, float), float, float):
    """calculate (dv_a, dv_b), trans_time and sort of mean anomaly phase angle of hohmann transfer at time t

    calculate (dv_a, dv_b), trans_time and sort of mean anomaly phase angle of hohmann transfer at time t
    
    Args:
        vessel: active vessel for transfer
        target: target vessel or celesital body
        t: time to burn

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
    
    phase_angle = (target.orbit.true_anomaly_at_ut(ut + t_trans) - (vessel.orbit.true_anomaly_at_ut(ut) + math.pi) ) % (2 * math.pi)
    
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

    _, _, pa = hohmann_transfer_to_target_alt_at_ut(vessel, target, min_time)
    last_pa = pa
    min_abs_pa = pa

    for i in range(1, num_divisions):
        t = min_time + dt * i
        _, _, pa = hohmann_transfer_to_target_alt_at_ut(vessel, target, t)
        abs_pa = abs(pa)
        if abs_pa < min_abs_pa:
            min_abs_pa = abs_pa
            pa_t_sign = math.copysign(1, last_pa - pa)

            min_time = t - dt
            max_time = t

    while (max_time - min_time > 0.01):
        t = (max_time + min_time) / 2
        _, _, pa = hohmann_transfer_to_target_alt_at_ut(vessel, target, t)

        if math.copysign(1, pa) == pa_t_sign:
            min_time = t
        else:
            max_time = t

    t = (max_time + min_time) / 2
    (dv_a, dv_b), trans_t, pa = hohmann_transfer_to_target_alt_at_ut(vessel, target, t)
    node = vessel.control.add_node(t, prograde=dv_a)
    # vessel.control.add_node(t+trans_t, prograde=dv_b)

    ut.remove()

    ## TODO: separate codes below since it's very common
    ## Execute node
    ut = conn.add_stream(getattr, conn.space_center, 'ut')

    vessel.auto_pilot.engage()

    # Calculate burn time (using rocket equation)
    F = vessel.available_thrust
    Isp = vessel.specific_impulse * 9.82
    m0 = vessel.mass
    m1 = m0 / math.exp(dv_a/Isp)
    flow_rate = F / Isp
    burn_time = (m0 - m1) / flow_rate

    # Orientate ship
    dialog.status_update("Orientating ship for circularization burn")
    vessel.auto_pilot.reference_frame = node.reference_frame
    vessel.auto_pilot.target_direction = (0, 0, 0)
    vessel.auto_pilot.wait()

    # Wait until burn
    dialog.status_update("Waiting until circularization burn")
    burn_ut = t - (burn_time/2.0)
    lead_time = 5
    conn.space_center.warp_to(burn_ut - lead_time)

    # Execute burn
    dialog.status_update("Ready to execute burn")
    while ut() - (burn_time/2.0) > 0:
        pass
    dialog.status_update("Executing burn")
    vessel.control.throttle = 1.0
    time.sleep(burn_time - 0.1)
    dialog.status_update("Fine tuning")
    vessel.control.throttle = 0.05

    remaining_delta_v = conn.add_stream(getattr, node, "remaining_delta_v")
    min_delta_v = remaining_delta_v()
    point_passed = False
    while remaining_delta_v() > 0.1 and not point_passed:
        if min_delta_v < remaining_delta_v():
            point_passed = True
        else:
            min_delta_v = remaining_delta_v()
        pass
    vessel.control.throttle = 0.0
    remaining_delta_v.remove()
    node.remove()

    vessel.auto_pilot.disengage()

    ut.remove()

if __name__ == "__main__":
    import krpc
    conn = krpc.connect(name='hohman transfer')
    hohmann_transfer_to_target(conn)


