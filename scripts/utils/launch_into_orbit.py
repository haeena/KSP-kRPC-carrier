import time
import math
from functools import reduce
from krpc.client import Client

from scripts.utils.status_dialog import StatusDialog
from scripts.utils.execute_node import execute_next_node
from scripts.utils.autostage import autostage

def vessel_current_stage(vessel) -> int:
    """Return current stage

    Search all parts and return max(parts.stage, parts.decouple_stage) of all parts

    Args:
        vessel: target vessel

    Returns:
        current stage of the vessel
    """

    return reduce(lambda x, y: max(x, y.stage, y.decouple_stage), vessel.parts.all, 0)

def launch_into_orbit(conn: Client,
                      target_alt: float, target_inc: float,
                      auto_launch: bool = True,
                      auto_stage: bool = True, stop_stage: int = 0,
                      pre_circulization_stage: int = None,
                      post_circulization_stage: int = None,
                      skip_circulization: bool = False,
                      use_rcs_on_ascent: bool = False,
                      use_rcs_on_circulization: bool = False
                     ) -> None:
    """Lunch active vessel into orbit.

    Extended description of function.

    Args:
        conn: kRPC connection
        target_alt: target altitude in m
        target_inc: target inclination in degree
        auto_launch: when everything set, launch automatically
        auto_stage: staging when no fuel left on the stage
        stop_stage: stop staging on the stage
        pre_circulization_stage: stage to this before circulization
        post_circulization_stage: stage to this after circulization
        skip_circulization: skip circulization after ascent
        use_rcs_on_ascent: turn on rcs during ascent
        use_rcs_on_circulization: turn on rcs during circulization

    Returns:
        return nothing, return when procedure finished

    """
    vessel = conn.space_center.active_vessel
    body = vessel.orbit.body

    obt_frame = vessel.orbit.body.non_rotating_reference_frame
    srf_frame = vessel.orbit.body.reference_frame

    atomosphere_depth = body.atmosphere_depth

    ## TODO: parametalize or default per body
    TURN_START_ALTITUDE = 250
    TURN_END_ALTITUDE = 45000

    # Set up dialog
    dialog = StatusDialog(conn)

    # Set up streams for telemetry
    ut = conn.add_stream(getattr, conn.space_center, 'ut')
    altitude = conn.add_stream(getattr, vessel.flight(), 'mean_altitude')
    apoapsis = conn.add_stream(getattr, vessel.orbit, 'apoapsis_altitude')
    time_to_apoapsis = conn.add_stream(getattr, vessel.orbit, 'time_to_apoapsis')
    vessel_stage = reduce(lambda x, y: max(x, y.stage, y.decouple_stage), vessel.parts.all, 0)

    resource_types_for_stage = ["LiquidFuel", "Oxidizer", "SolidFuel"]

    # Pre-launch setup
    vessel.control.sas = True
    vessel.control.rcs = use_rcs_on_ascent
    vessel.control.throttle = 1.0

    if vessel.situation.name == "pre_launch":
        if auto_launch:
            for i in range(-5, 1):
                dialog.status_update("T={} ...".format(i))
                time.sleep(1)
            dialog.status_update("Lift off!!")
            vessel.control.activate_next_stage()
        else:
            dialog.status_update("Ready to launch")

    # Main ascent loop

    ## TODO: this assumes launch from equato
    ascent_heading = (target_inc + 90) % 360
    vessel.auto_pilot.engage()
    vessel.auto_pilot.target_pitch_and_heading(90, ascent_heading)

    turn_angle = 0

    raise_apoapsis_last_throttle = vessel.control.throttle
    raise_apoapsis_last_apoapsis = apoapsis()
    raise_apoapsis_last_ut = ut()

    while True:
        current_stage = reduce(lambda x, y: max(x, y.decouple_stage, y.stage), vessel.parts.all, 0)
        if auto_stage and current_stage > stop_stage:
            autostage(conn)

        # Gravity turn
        if altitude() > TURN_START_ALTITUDE and altitude() < TURN_END_ALTITUDE:
            dialog.status_update("Gravity turn")

            frac = ((altitude() - TURN_START_ALTITUDE) /
                    (TURN_END_ALTITUDE - TURN_START_ALTITUDE))
            new_turn_angle = frac * 90
            if abs(new_turn_angle - turn_angle) > 0.5:
                turn_angle = new_turn_angle
                vessel.auto_pilot.target_pitch_and_heading(90-turn_angle, ascent_heading)

        # Decrease throttle when approaching target apoapsis
        if apoapsis() <= target_alt*0.9:
            vessel.control.throttle = 1
        elif apoapsis() < target_alt:
            dialog.status_update("Approaching target apoapsis")
            try:
                instant_rate_per_throttle = (apoapsis() - raise_apoapsis_last_apoapsis) / ((ut() - raise_apoapsis_last_ut) * raise_apoapsis_last_throttle)
                instant_rate_per_throttle = max(1.0, instant_rate_per_throttle)
                required_appoapsis_height = target_alt - apoapsis()
                vessel.control.throttle = min(1, max(0.05, required_appoapsis_height / instant_rate_per_throttle))
            except:
                vessel.control.throttle = 0.05
        else:
            vessel.control.throttle = 0
            if altitude() < atomosphere_depth:
                dialog.status_update("Coasting out of atmosphere")
            else:
                break

        raise_apoapsis_last_throttle = vessel.control.throttle 
        raise_apoapsis_last_apoapsis = apoapsis()
        raise_apoapsis_last_ut = ut()

    vessel.control.throttle = 0

    if pre_circulization_stage:
        while vessel_current_stage(vessel) <= pre_circulization_stage:
            vessel.control.activate_next_stage()

    if skip_circulization:
        return

    # pre-circularization setup
    vessel.control.rcs = use_rcs_on_circulization

    # Plan circularization burn (using vis-viva equation)
    dialog.status_update("Planning circularization burn")
    mu = vessel.orbit.body.gravitational_parameter
    r = vessel.orbit.apoapsis
    a1 = vessel.orbit.semi_major_axis
    a2 = r
    v1 = math.sqrt(mu*((2./r)-(1./a1)))
    v2 = math.sqrt(mu*((2./r)-(1./a2)))
    delta_v = v2 - v1
    node = vessel.control.add_node(
        ut() + vessel.orbit.time_to_apoapsis, prograde=delta_v)

    vessel.auto_pilot.disengage()

    execute_next_node(conn)

    if post_circulization_stage:
        while vessel_current_stage(vessel) <= post_circulization_stage:
            vessel.control.activate_next_stage()

    dialog.status_update("Launch complete")

    return

if __name__ == "__main__":
    import krpc
    connnection = krpc.connect(name='Launch into orbit')
    launch_into_orbit(connnection, 100000, 0)
