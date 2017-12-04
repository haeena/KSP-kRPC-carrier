import math
import time
import krpc

TURN_START_ALTITUDE = 250
TURN_END_ALTITUDE = 45000
TARGET_ALTITUDE = 120000
ASCENT_HEADING = 90

def status_update(panel_text, message):
    status_line = "Status: {}".format(message)
    print(status_line)
    panel_text.content = status_line

def main():
    conn = krpc.connect(name='Launch into orbit')
    vessel = conn.space_center.active_vessel

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

    # Pre-launch setup
    vessel.control.sas = True
    vessel.control.rcs = False
    vessel.control.throttle = 1.0

    # Countdown...
    status_update(text_status, "3...")
    time.sleep(1)
    status_update(text_status, "2...")
    time.sleep(1)
    status_update(text_status, "1...")
    time.sleep(1)
    status_update(text_status, "Launch!")

    # Activate the first stage
    vessel.control.activate_next_stage()
    vessel.auto_pilot.engage()
    vessel.auto_pilot.target_pitch_and_heading(90, ASCENT_HEADING)

    # Main ascent loop
    apoapsis_near = False
    apoapsis_reached = False
    turn_angle = 0

    while True:
        # Gravity turn
        if altitude() > TURN_START_ALTITUDE and altitude() < TURN_END_ALTITUDE:
            status_update(text_status, "Gravity turn")

            frac = ((altitude() - TURN_START_ALTITUDE) /
                    (TURN_END_ALTITUDE - TURN_START_ALTITUDE))
            new_turn_angle = frac * 90
            if abs(new_turn_angle - turn_angle) > 0.5:
                turn_angle = new_turn_angle
                vessel.auto_pilot.target_pitch_and_heading(90-turn_angle, ASCENT_HEADING)

        # Decrease throttle when approaching target apoapsis
        if not apoapsis_reached:
            if apoapsis_near and apoapsis() < TARGET_ALTITUDE:
                apoapsis_reached = True
                status_update(text_status, "Target apoapsis reached")
                vessel.control.throttle = 0.0
            elif apoapsis() > TARGET_ALTITUDE*0.9:
                apoapsis_near = True
                status_update(text_status, "Approaching target apoapsis")
                vessel.control.throttle = 0.25

        if apoapsis_reached:
            break

    # Wait until out of atmosphere
    status_update(text_status, "Coasting out of atmosphere")
    while altitude() < 70500:
        pass

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

    status_update(text_status, "Launch complete")

if __name__ == "__main__":
    main()
