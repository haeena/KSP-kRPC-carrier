import time
import math
from functools import reduce
from krpc.client import Client

from scripts.utils.utils import *
from scripts.utils.status_dialog import StatusDialog
from scripts.utils.execute_node import execute_next_node
from scripts.utils.autostage import set_autostaging, unset_autostaging

def impact_prediction(radius: float, altitude: float,
                      vertical_speed: float, horizontal_speed: float,
                      surface_gravity: float):
    """impact_prediction

    Extended description of function.

    Args:
        radius: current radius from center of body 
        altitude: surface altitude
        vertical_speed: vertical speed
        horizontal_speed: horizontal speed
        surface_gravity: surface gravity

    Returns:
        return None if not impact
        return (seconds_until_impact, terminal_speed)
    """
    fall_speed = -vertical_speed
    downward_acceleration = surface_gravity - horizontal_speed * horizontal_speed / radius

    # do we land?
    if downward_acceleration < 0:
        max_fall_distance = -(fall_speed * fall_speed) / (2.0 * downward_acceleration)
        if max_fall_distance < (altitude + 1.0):
            return None

    sec_until_impact = (-fall_speed + math.sqrt(fall_speed * fall_speed + 2.0 * downward_acceleration * altitude)) / downward_acceleration

    vertical_speed_at_impact = fall_speed + sec_until_impact * downward_acceleration
    impact_speed = math.sqrt(vertical_speed_at_impact * vertical_speed_at_impact + horizontal_speed * horizontal_speed);
    return sec_until_impact, impact_speed

def burn_prediction(delta_v: float, acceleration: float):
    return delta_v / acceleration

def vertical_landing(conn: Client,
                     landing_speed: float = 5.0,
                     auto_stage: bool = True,
                     stop_stage: int = 0,
                     deploy_legs_on_decent: bool = True,
                     retract_palens_on_decent: bool = True,
                     use_rcs_on_landing: bool = False,
                     use_parachute: bool = True,
                    ) -> None:
    """Vertical landing

    Extended description of function.

    Args:
        conn: kRPC connection
        auto_stage: staging when no fuel left on the stage
        stop_stage: stop staging on the stage
        deploy_legs_on_decent: extend legs on landing
        retract_palens_on_decent: retract panels on landing
        use_rcs_on_landing: use rcs or not
        use_parachute: use parachute

    Returns:
        return nothing, return when procedure finished
    """
    vessel = conn.space_center.active_vessel

    ## check retrograde and radial hold capability
    use_sas = False
    try:
        vessel.control.sas = True
        vessel.control.sas_mode = vessel.control.sas_mode.retrograde
        vessel.control.sas_mode = vessel.control.sas_mode.radial
        use_sas = True
    except:
        pass        
    vessel.control.sas = False

    # Set up dialog and stream
    dialog = StatusDialog(conn)
    surface_gravity = vessel.orbit.body.surface_gravity
    ref_frame = conn.space_center.ReferenceFrame.create_hybrid(
        position=vessel.orbit.body.reference_frame,
        rotation=vessel.surface_reference_frame)
    flight = vessel.flight(ref_frame)
    ut = conn.add_stream(getattr, conn.space_center, 'ut')
    mass = conn.add_stream(getattr, vessel, 'mass')
    available_thrust = conn.add_stream(getattr, vessel, 'available_thrust')
    radius = conn.add_stream(getattr, vessel.orbit, 'radius')
    altitude = conn.add_stream(getattr, flight, 'surface_altitude')
    speed = conn.add_stream(getattr, flight, 'speed')
    vertical_speed = conn.add_stream(getattr, flight, 'vertical_speed')
    horizontal_speed = conn.add_stream(getattr, flight, 'horizontal_speed')

    vessel.control.sas = True
    vessel.control.rcs = use_rcs_on_landing
    vessel.control.speed_mode = vessel.control.speed_mode.surface

    # set staging
    if auto_stage:
        set_autostaging(conn, stop_stage=stop_stage)

    # kill horizontal velocity
    dialog.status_update("kill horizontal velocity")
    if use_sas:
        vessel.control.sas = True
        vessel.control.sas_mode = vessel.control.sas_mode.retrograde
        while angle_between(vessel.flight(vessel.surface_velocity_reference_frame).direction,(0,-1,0)) > 5/180 * math.pi:
            time.sleep(0.1)
    else:
        vessel.auto_pilot.engage()
        vessel.auto_pilot.reference_frame = vessel.surface_velocity_reference_frame
        vessel.auto_pilot.target_direction = (0, -1, 0)
        vessel.auto_pilot.wait()

    while True:
        a100 = available_thrust() / mass()
        dialog.status_update("Kill horizontal velocity: Alt: {: 5.3f}, Speed {: 5.3f} m/s (H: {: 5.3f}, V: {: 5.3f})"\
                             "".format(altitude(), speed(), horizontal_speed(), vertical_speed()))

        if horizontal_speed() > 0.1:
            vessel.control.throttle = max(0, min(1.0, speed() / a100))
        else:
            vessel.control.throttle = 0
            break

    last_ut = ut()

    # wait for burn loop
    while True:
        a100 = available_thrust() / mass()
        bounding_box = vessel.bounding_box(vessel.surface_reference_frame)
        lower_bound = bounding_box[0][0]
        landing_alt = altitude() + lower_bound

        sec_until_impact, terminal_speed = impact_prediction(radius(), landing_alt, vertical_speed(), horizontal_speed(), surface_gravity)
        burn_time = burn_prediction(terminal_speed, a100)
        burn_lead_time = sec_until_impact - burn_time

        if burn_lead_time and burn_lead_time !=0 and burn_lead_time > (ut() - last_ut)*1.5+2:
            if burn_lead_time > 30:
                dialog.status_update("Warp for decereration burn - 30sec: {: 5.3f}".format(burn_lead_time, ut()))
                conn.space_center.warp_to(ut() + burn_lead_time - 30)
                time.sleep(5)
            else:
                dialog.status_update("Wait for decereration burn: {: 5.3f} sec; ut: {: 5.3f}".format(burn_lead_time, ut()))
        else:
            break
        last_ut = ut()
        time.sleep(0.1)

    # on decent: deploy leg, retract panel
    if deploy_legs_on_decent:
        deploy_legs(conn)
    if retract_palens_on_decent:
        retract_panels(conn)

    # kill horizontal velocity again
    dialog.status_update("kill horizontal velocity again")
    if use_sas:
        vessel.control.sas = True
        last_sas_mode = vessel.control.sas_mode.retrograde
        while angle_between(vessel.flight(vessel.surface_velocity_reference_frame).direction,(0,-1,0)) > 5/180 * math.pi:
            time.sleep(0.1)
    else:
        vessel.auto_pilot.engage()
        vessel.auto_pilot.reference_frame = vessel.surface_velocity_reference_frame
        vessel.auto_pilot.target_direction = (0, -1, 0)
        vessel.auto_pilot.wait()

    while True:
        a100 = available_thrust() / mass()
        dialog.status_update("Kill horizontal velocity again: Alt: {: 5.3f}, Speed {: 5.3f} m/s (H: {: 5.3f}, V: {: 5.3f})".format(altitude(), speed(), horizontal_speed(), vertical_speed()))

        if horizontal_speed() > 0.1:
            vessel.control.throttle = max(0, min(1.0, speed() / a100))
        else:
            vessel.control.throttle = 0
            break

    # Main decent loop
    last_sas_mode = vessel.control.sas_mode
    while True:
        a100 = available_thrust() / mass()
        bounding_box = vessel.bounding_box(vessel.surface_reference_frame)
        lower_bound = bounding_box[0][0]
        landing_alt = altitude() + lower_bound

        sec_until_impact, terminal_speed = impact_prediction(radius(), landing_alt, vertical_speed(), horizontal_speed(), surface_gravity)
        burn_time = burn_prediction(terminal_speed, a100)
        burn_lead_time = sec_until_impact - burn_time

        dialog.status_update("Alt: {: 5.3f}, Speed {: 5.3f} m/s (H: {: 5.3f}, V: {: 5.3f}), "\
                             "a: {: 5.3f}, g: {: 5.3f}, "\
                             "landing in: {: 5.3f} sec, burn lead time: {: 5.3f} sec"\
                             "".format(altitude(), speed(), horizontal_speed(), vertical_speed(),
                                       a100, surface_gravity,
                                       sec_until_impact, burn_lead_time)
                            )

        if use_sas:
            if (horizontal_speed() > 0.5 or horizontal_speed() / speed() > 0.9) and last_sas_mode != vessel.control.sas_mode.retrograde:
                vessel.control.sas_mode = vessel.control.sas_mode.retrograde
                last_sas_mode = vessel.control.sas_mode.retrograde
            elif last_sas_mode != vessel.control.sas_mode.radial:
                vessel.control.sas_mode = vessel.control.sas_mode.radial
                last_sas_mode = vessel.control.sas_mode.radial
        else:
            # TODO: auto-pilot
            pass

        if landing_alt >= landing_speed/2 or not max([False] + [ l.is_grounded for l in vessel.parts.legs ]):
            target_speed = max(landing_speed, landing_alt / 5)
            throttle = max(0, min(1.0, (( speed() - target_speed ) + surface_gravity) / a100))
            vessel.control.throttle = throttle
        else:
            vessel.control.sas_mode.radial
            vessel.control.throttle = 0
            break
        last_ut = ut()

    dialog.status_update("Landed")

    # keep sas on for a bit to maintain landing stability
    time.sleep(5)

    if use_sas:
        vessel.control.sas = False
    else:
        vessel.auto_pilot.disengage() 

    if auto_stage:
        unset_autostaging()

    return

def retract_panels(conn: Client):
    vessel = conn.space_center.active_vessel
    dialog = StatusDialog(conn)

    dialog.status_update("Retract solar/radiator panels")
    for panel in vessel.parts.solar_panels + vessel.parts.radiators:
        if panel.deployable:
            panel.deployed = False    

def deploy_legs(conn: Client):
    vessel = conn.space_center.active_vessel
    dialog = StatusDialog(conn)

    dialog.status_update("Deploy legs")
    for leg in vessel.parts.legs:
        if leg.deployable:
            leg.deployed = True    

if __name__ == "__main__":
    import os
    import krpc
    krpc_address = os.environ["KRPC_ADDRESS"]
    con = krpc.connect(name='Landing', address=krpc_address)
    vertical_landing(con)
