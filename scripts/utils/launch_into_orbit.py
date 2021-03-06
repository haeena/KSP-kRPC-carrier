import math
import time

from krpc.client import Client
from scripts.utils.autostage import set_autostaging, unset_autostaging
from scripts.utils.execute_node import execute_next_node
from scripts.utils.status_dialog import StatusDialog


# TODO: get staging condition per stage number
def launch_into_orbit(
    conn: Client,
    target_alt: float,
    target_inc: float,
    turn_start_alt: float = 250,
    turn_end_alt: float = 45000,
    auto_launch: bool = True,
    auto_stage: bool = True,
    stop_stage: int = 0,
    exit_atm_stage: int = None,
    pre_circulization_stage: int = None,
    post_circulization_stage: int = None,
    skip_circulization: bool = False,
    use_rcs_on_ascent: bool = False,
    use_rcs_on_circulization: bool = False,
    deploy_panel_atm_exit: bool = True,
    deploy_panel_stage: int = None,
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
        exit_atm_stage: stage to this on atm exit
        pre_circulization_stage: stage to this before circulization
        post_circulization_stage: stage to this after circulization
        skip_circulization: skip circulization after ascent
        use_rcs_on_ascent: turn on rcs during ascent
        use_rcs_on_circulization: turn on rcs during circulization
        deploy_panel_atm_exit: deploy solar/radiator panels after atm exit
        deploy_panel_stage: deploy solar/radiator panels delayed on stage

    Returns:
        return nothing, return when procedure finished

    """
    vessel = conn.space_center.active_vessel
    body = vessel.orbit.body

    # Set up dialog
    dialog = StatusDialog(conn)

    # Set up streams for telemetry
    ut = conn.add_stream(getattr, conn.space_center, "ut")
    atomosphere_depth = body.atmosphere_depth
    altitude = conn.add_stream(getattr, vessel.flight(), "mean_altitude")
    apoapsis = conn.add_stream(getattr, vessel.orbit, "apoapsis_altitude")

    # Pre-launch setup
    vessel.control.sas = True
    vessel.control.rcs = use_rcs_on_ascent
    vessel.control.throttle = 1.0

    if vessel.situation.name == "pre_launch":
        if auto_launch:
            for i in range(-5, 0):
                dialog.status_update(f"T={i} ...")
                time.sleep(1)
            dialog.status_update("T=0; Lift off!!")
            vessel.control.activate_next_stage()
        else:
            dialog.status_update("Ready to launch")

    # Main ascent loop
    set_ascent_autostaging(
        conn,
        auto_stage=auto_stage,
        stop_stage=stop_stage,
        exit_atm_stage=exit_atm_stage,
        pre_circulization_stage=pre_circulization_stage,
        post_circulization_stage=post_circulization_stage,
        skip_circulization=skip_circulization,
    )

    ascent_heading = (90 - target_inc) % 360
    vessel.auto_pilot.engage()
    vessel.auto_pilot.target_pitch_and_heading(90, ascent_heading)

    turn_angle = 0

    raise_apoapsis_last_throttle = vessel.control.throttle
    raise_apoapsis_last_apoapsis = apoapsis()
    raise_apoapsis_last_ut = ut()

    state_gravity_turn = False
    state_approach_target_ap = False
    state_coasting_out_of_atm = False
    while True:
        if apoapsis() <= target_alt * 0.9:
            vessel.control.throttle = 1
            # Gravity turn
            if altitude() > turn_start_alt and altitude() < turn_end_alt:
                if not state_gravity_turn:
                    dialog.status_update("Gravity turn")
                    state_gravity_turn = True

                frac = (altitude() - turn_start_alt) / (
                    turn_end_alt - turn_start_alt
                )
                new_turn_angle = frac * 90
                if abs(new_turn_angle - turn_angle) > 0.5:
                    turn_angle = new_turn_angle
                    vessel.auto_pilot.target_pitch_and_heading(
                        90 - turn_angle, ascent_heading
                    )

        # Decrease throttle when approaching target apoapsis
        elif apoapsis() < target_alt:
            if not state_approach_target_ap:
                dialog.status_update("Approaching target apoapsis")
                state_approach_target_ap = True
            vessel.auto_pilot.target_pitch_and_heading(0, ascent_heading)
            try:
                instant_rate_per_throttle = (
                    apoapsis() - raise_apoapsis_last_apoapsis
                ) / (
                    (ut() - raise_apoapsis_last_ut)
                    * raise_apoapsis_last_throttle
                )
                instant_rate_per_throttle = max(1.0, instant_rate_per_throttle)
                required_appoapsis_height = target_alt - apoapsis()
                vessel.control.throttle = min(
                    1,
                    max(
                        0.05,
                        required_appoapsis_height / instant_rate_per_throttle,
                    ),
                )
            except Exception:
                vessel.control.throttle = 0.05

        # target appoapsis reached
        else:
            vessel.control.throttle = 0
            if altitude() < atomosphere_depth:
                if not state_coasting_out_of_atm:
                    dialog.status_update("Coasting out of atmosphere")
                    state_coasting_out_of_atm = True
            else:
                break

        # wait an execute for exit_atm_stage, then re-set autostaging
        current_stage = vessel.control.current_stage
        if exit_atm_stage is not None:
            if (
                exit_atm_stage < current_stage
                and altitude() >= atomosphere_depth
            ):
                dialog.status_update(f"Staging: stage {current_stage - 1}")
                vessel.control.activate_next_stage()
                if exit_atm_stage == current_stage - 1:
                    set_ascent_autostaging(
                        conn,
                        auto_stage=auto_stage,
                        stop_stage=stop_stage,
                        pre_circulization_stage=pre_circulization_stage,
                        post_circulization_stage=post_circulization_stage,
                        skip_circulization=skip_circulization,
                    )

        raise_apoapsis_last_throttle = vessel.control.throttle
        raise_apoapsis_last_apoapsis = apoapsis()
        raise_apoapsis_last_ut = ut()

    vessel.control.throttle = 0

    if auto_stage:
        unset_autostaging()

    if deploy_panel_atm_exit:
        deploy_panels(conn)

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
    v1 = math.sqrt(mu * ((2.0 / r) - (1.0 / a1)))
    v2 = math.sqrt(mu * ((2.0 / r) - (1.0 / a2)))
    delta_v = v2 - v1
    vessel.control.add_node(
        ut() + vessel.orbit.time_to_apoapsis, prograde=delta_v
    )

    vessel.auto_pilot.disengage()

    if pre_circulization_stage:
        while vessel.control.current_stage <= pre_circulization_stage:
            vessel.control.activate_next_stage()

    execute_next_node(conn, auto_stage=auto_stage)

    if post_circulization_stage:
        while vessel.control.current_stage <= post_circulization_stage:
            vessel.control.activate_next_stage()

    dialog.status_update("Launch complete")

    return


def set_ascent_autostaging(
    conn: Client,
    auto_stage: bool = True,
    stop_stage: int = 0,
    exit_atm_stage: int = None,
    pre_circulization_stage: int = None,
    post_circulization_stage: int = None,
    skip_circulization: bool = False,
):
    if not auto_stage:
        return
    ascent_stop_stage = None
    ascent_stop_stages = [stop_stage] + [
        s + 1
        for s in [
            exit_atm_stage,
            pre_circulization_stage,
            post_circulization_stage,
        ]
        if s is not None
    ]
    if len(ascent_stop_stages) != 0:
        ascent_stop_stage = max(ascent_stop_stages)
    set_autostaging(conn, stop_stage=ascent_stop_stage)


def deploy_panels(conn: Client):
    vessel = conn.space_center.active_vessel
    dialog = StatusDialog(conn)

    dialog.status_update("Deploying solar/radiator panels")
    for panel in vessel.parts.solar_panels + vessel.parts.radiators:
        if panel.deployable:
            panel.deployed = True


def warp_for_longtitude(conn: Client, longtitude: float):
    dialog = StatusDialog(conn)

    vessel = conn.space_center.active_vessel
    body = vessel.orbit.body
    current_longtitude = (
        body.rotation_angle * 180 / math.pi + vessel.flight().longitude
    ) % 360
    longtitude_ut = (longtitude - current_longtitude) % 360 / (
        body.rotational_speed * 180 / math.pi
    ) + conn.space_center.ut

    dialog.status_update("Waiting for launch timing")
    lead_time = 5
    conn.space_center.warp_to(longtitude_ut - lead_time)


if __name__ == "__main__":
    import os
    import krpc

    krpc_address = os.environ["KRPC_ADDRESS"]
    connection = krpc.connect(name="Launch into orbit", address=krpc_address)
    warp_for_longtitude(connection, 205.8)
    launch_into_orbit(
        connection, 100000, 95, turn_start_alt=18000, turn_end_alt=550000
    )
