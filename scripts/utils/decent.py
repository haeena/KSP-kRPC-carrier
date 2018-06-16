import time
import math
from functools import reduce
from krpc.client import Client

from scripts.utils.utils import *
from scripts.utils.status_dialog import StatusDialog
from scripts.utils.execute_node import execute_next_node
from scripts.utils.autostage import set_autostaging, unset_autostaging

def landing_time_prediction(altitude: float, vertical_speed: float, gravitational_parameter: float):
    dt = 1.0
    t = 0
    t_max = 10000
    sim_alt = altitude
    v = vertical_speed

    while sim_alt > 0 and t < t_max:
        g = gravitational_parameter / (1000000 * sim_alt * sim_alt)
        alt_change = (v - g * dt / 2) * dt
        sim_alt = sim_alt + alt_change
        t = t + dt

    if t >= t_max:
        # not land
        return None
    else:
        return t - dt

def vertical_landing(conn: Client,
                     decent_speed: float = 5.0,
                     auto_stage: bool = True,
                     stop_stage: int = 0,
                     use_rcs_on_landing: bool=False,
                    ) -> None:
    """Vertical landing

    Extended description of function.

    Args:
        conn: kRPC connection
        auto_stage: staging when no fuel left on the stage
        stop_stage: stop staging on the stage
        use_rcs_on_landing: use rcs or not

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
    gravitational_parameter = vessel.orbit.body.gravitational_parameter
    ref_frame = conn.space_center.ReferenceFrame.create_hybrid(
        position=vessel.orbit.body.reference_frame,
        rotation=vessel.surface_reference_frame)
    flight = vessel.flight(ref_frame)
    ut = conn.add_stream(getattr, conn.space_center, 'ut')
    mass = conn.add_stream(getattr, vessel, 'mass')
    available_thrust = conn.add_stream(getattr, vessel, 'available_thrust')
    altitude = conn.add_stream(getattr, flight, 'surface_altitude')
    speed = conn.add_stream(getattr, flight, 'speed')
    velocity = conn.add_stream(getattr, flight, 'velocity')
    vertical_speed = conn.add_stream(getattr, flight, 'vertical_speed')
    horizontal_speed = conn.add_stream(getattr, flight, 'horizontal_speed')
    direction = conn.add_stream(getattr, flight, 'direction')
    retrograde = conn.add_stream(getattr, flight, 'retrograde')
    radial = conn.add_stream(getattr, flight, 'radial')

    vessel.control.sas = True
    vessel.control.rcs = use_rcs_on_landing

    # set staging
    if auto_stage:
        set_autostaging(conn, stop_stage=stop_stage)

    # kill horizontal verocity
    if use_sas:
        vessel.control.sas = True
        vessel.control.sas_mode = vessel.control.sas_mode.retrograde
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
        dialog.status_update("Alt: {: 5.3f}, Speed {: 5.3f} m/s (H: {: 5.3f}, V: {: 5.3f})".format(altitude(), speed(), horizontal_speed(), vertical_speed()))

        if horizontal_speed() > 0.1:
            vessel.control.throttle = max(0, min(1.0, speed() / a100))
        else:
            vessel.control.throttle = 0
            break

    vessel.auto_pilot.disengage()

    # turn to retrograde
    if use_sas:
        vessel.control.sas = True
        vessel.control.sas_mode = vessel.control.sas_mode.retrograde
        last_sas_mode = vessel.control.sas_mode.retrograde
        while angle_between(vessel.flight(vessel.surface_velocity_reference_frame).direction,(0,-1,0)) > 5/180 * math.pi:
            time.sleep(0.1)
    else:
        vessel.auto_pilot.engage()
        vessel.auto_pilot.reference_frame = vessel.surface_velocity_reference_frame
        vessel.auto_pilot.target_direction = (0, -1, 0)
        vessel.auto_pilot.wait()
    
    # wait for decent
    a100 = available_thrust() / mass()
    lead_time = landing_time_prediction(altitude(), vertical_speed(), gravitational_parameter)

    while vertical_speed() + 0.8 * lead_time * a100 > 0:
        a100 = available_thrust() / mass()
        lead_time = landing_time_prediction(altitude(), vertical_speed(), gravitational_parameter)

        dialog.status_update("Wait for decereration burn: {: 5.3f} sec)".format(lead_time))
        time.sleep(1)

    # Main decent loop
    while True:
        a100 = available_thrust() / mass()
        dialog.status_update("Alt: {: 5.3f}, Speed {: 5.3f} m/s (H: {: 5.3f}, V: {: 5.3f})".format(altitude(), speed(), horizontal_speed(), vertical_speed()))

        if use_sas:
            if (horizontal_speed() > 0.1 or horizontal_speed() / speed() > 0.9) and last_sas_mode != vessel.control.sas_mode.retrograde:
                vessel.control.sas_mode = vessel.control.sas_mode.retrograde
                last_sas_mode = vessel.control.sas_mode.retrograde
            elif last_sas_mode != vessel.control.sas_mode.radial:
                vessel.control.sas_mode = vessel.control.sas_mode.radial
                last_sas_mode = vessel.control.sas_mode.radial
        else:
            # TODO: auto-pilot
            pass

        if altitude() >= 10:
            target_speed = speed() + decent_speed
            vessel.control.throttle = max(0, min(1.0, target_speed / a100))
        else:
            vessel.control.throttle = 0
            break

    if auto_stage:
        unset_autostaging()

    dialog.status_update("Landed")

    return

if __name__ == "__main__":
    import os
    import krpc
    krpc_address = os.environ["KRPC_ADDRESS"]
    con = krpc.connect(name='Landing', address=krpc_address)
    vertical_landing(con)
