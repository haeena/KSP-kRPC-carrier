import time
import math
from functools import reduce
from krpc.client import Client

## TODO: separate this somewhere else
def status_update(panel_text, message):
    status_line = "Status: {}".format(message)
    print(status_line)
    panel_text.content = status_line


def vessel_current_stage(vessel) -> int:
    """Return current stage

    Search all parts and return max(parts.stage, parts.decouple_stage) of all parts

    Args:
        vessel: target vessel

    Returns:
        current stage of the vessel
    """

    return reduce(lambda x, y: max(x, y.stage, y.decouple_stage), vessel.parts.all, 0)

def lunch_into_orbit(conn: Client,
                     target_alt: float, target_inc: float,
                     auto_stage: bool = True, stop_stage: int = 0,
                     pre_circulization_stage: int = None,
                     post_circulization_stage: int = None,
                     skip_circulization: bool = False
                    ) -> None:
    """Lunch active vessel into orbit.

    Extended description of function.

    Args:
        conn: kRPC connection
        target_alt: target altitude in m
        target_inc: target inclination in degree
        auto_stage: staging when no fuel left on the stage
        stop_stage: stop staging on the stage
        pre_circulization_stage: stage to this before circulization
        post_circulization_stage: stage to this after circulization
        skip_circulization: skip circulization after ascent

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

    # setup canvas
    ## TODO: move this somewhere else
    canvas = conn.ui.stock_canvas
    screen_size = canvas.rect_transform.size
    panel = canvas.add_panel()
    rect = panel.rect_transform
    rect.size = (400, 50)
    rect.position = (110-(screen_size[0]/2), 0)

    text_status = panel.add_text("")
    text_status.rect_transform.position = (0, 0)
    text_status.color = (1, 1, 1)
    text_status.size = 12

    # Set up streams for telemetry
    ut = conn.add_stream(getattr, conn.space_center, 'ut')
    altitude = conn.add_stream(getattr, vessel.flight(), 'mean_altitude')
    apoapsis = conn.add_stream(getattr, vessel.orbit, 'apoapsis_altitude')
    time_to_apoapsis = conn.add_stream(getattr, vessel.orbit, 'time_to_apoapsis')
    vessel_stage = reduce(lambda x, y: max(x, y.stage, y.decouple_stage), vessel.parts.all, 0)

    resource_types_for_stage = ["LiquidFuel", "Oxidizer", "SolidFuel"]

    # Pre-launch setup
    vessel.control.sas = True
    vessel.control.rcs = False
    vessel.control.throttle = 0

    # Main ascent loop

    ## TODO: this assumes launch from equato
    ascent_heading = (target_inc + 90) % 360
    vessel.auto_pilot.engage()
    vessel.auto_pilot.target_pitch_and_heading(90, ascent_heading)

    turn_angle = 0

    # Main ascent loop
    turn_angle = 0

    raise_apoapsis_last_throttle = vessel.control.throttle 
    raise_apoapsis_last_apoapsis = apoapsis()
    raise_apoapsis_last_ut = ut()

    while True:
        ## TODO: this logic should be improved
        # Staging when fuel empty
        current_stage = reduce(lambda x, y: max(x, y.decouple_stage, y.stage), vessel.parts.all, 0)
        if auto_stage and current_stage > stop_stage:
            resources_of_stage = vessel.resources_in_decouple_stage(current_stage - 1)
            max_resource = max(map(lambda r: resources_of_stage.amount(r), resource_types_for_stage))

            if max_resource <= 0.1:
                status_update(text_status, "Staging: stage {}".format(current_stage - 1))
                vessel.control.activate_next_stage()

        # Gravity turn
        if altitude() > TURN_START_ALTITUDE and altitude() < TURN_END_ALTITUDE:
            status_update(text_status, "Gravity turn")

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
            status_update(text_status, "Approaching target apoapsis")
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
                status_update(text_status, "Coasting out of atmosphere")
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

    # Plan circularization burn (using vis-viva equation)
    status_update(text_status, "Planning circularization burn")
    mu = vessel.orbit.body.gravitational_parameter
    r = vessel.orbit.apoapsis
    a1 = vessel.orbit.semi_major_axis
    a2 = r
    v1 = math.sqrt(mu*((2./r)-(1./a1)))
    v2 = math.sqrt(mu*((2./r)-(1./a2)))
    delta_v = v2 - v1
    node = vessel.control.add_node(
        ut() + vessel.orbit.time_to_apoapsis, prograde=delta_v)

    # Calculate burn time (using rocket equation)
    F = vessel.available_thrust
    Isp = vessel.specific_impulse * 9.82
    m0 = vessel.mass
    m1 = m0 / math.exp(delta_v/Isp)
    flow_rate = F / Isp
    burn_time = (m0 - m1) / flow_rate

    # Orientate ship
    status_update(text_status, "Orientating ship for circularization burn")
    vessel.auto_pilot.reference_frame = node.reference_frame
    vessel.auto_pilot.target_direction = (0, 0, 0)
    vessel.auto_pilot.wait()

    # Wait until burn
    status_update(text_status, "Waiting until circularization burn")
    burn_ut = ut() + vessel.orbit.time_to_apoapsis - (burn_time/2.0)
    lead_time = 5
    conn.space_center.warp_to(burn_ut - lead_time)

    # Execute burn
    status_update(text_status, "Ready to execute burn")
    while time_to_apoapsis() - (burn_time/2.0) > 0:
        pass
    status_update(text_status, "Executing burn")
    vessel.control.throttle = 1.0
    time.sleep(burn_time - 0.1)
    status_update(text_status, "Fine tuning")
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
    node.remove()

    if post_circulization_stage:
        while vessel_current_stage(vessel) <= post_circulization_stage:
            vessel.control.activate_next_stage()

    status_update(text_status, "Launch complete")
    

if __name__ == "__main__":
    import krpc
    conn = krpc.connect(name='Launch into orbit')
    lunch_into_orbit(conn, 100000, 0)
