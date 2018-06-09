import math
import time
import krpc

def status_update(panel_text, message):
    status_line = "Status: {}".format(message)
    print(status_line)
    panel_text.content = status_line

def main():
    krpc_address = os.environ["KRPC_ADDRESS"]
    conn = krpc.connect(name='Back to kerbin',  address=krpc_address)
    vessel = conn.space_center.active_vessel
    obt_frame = vessel.orbit.body.non_rotating_reference_frame
    srf_frame = vessel.orbit.body.reference_frame

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

    ## we plan de-orbit burn by hands
    # get first node
    node = vessel.control.nodes[0]
    remaining_delta_v = conn.add_stream(getattr, node, "remaining_delta_v")

    # Orientate ship
    status_update(text_status, "Orientating ship for circularization burn")
    vessel.auto_pilot.engage()
    vessel.auto_pilot.reference_frame = obt_frame
    vessel.auto_pilot.target_direction = node.direction(obt_frame)
    vessel.auto_pilot.wait()

    # Calculate burn time (using rocket equation)
    F = vessel.available_thrust
    Isp = vessel.specific_impulse * 9.82
    m0 = vessel.mass
    m1 = m0 / math.exp(remaining_delta_v()/Isp)
    flow_rate = F / Isp
    burn_time = (m0 - m1) / flow_rate

    # Wait until burn
    status_update(text_status, "Waiting until circularization burn")
    burn_ut = node.ut - (burn_time/2.0)
    lead_time = 5
    conn.space_center.warp_to(burn_ut - lead_time)

    # Execute burn
    status_update(text_status, "Ready to execute burn")
    time_to_node = conn.add_stream(getattr, node, 'time_to')
    while time_to_node() - (burn_time/2.0) > 0:
        pass
    status_update(text_status, "Executing burn")
    vessel.control.throttle = 1.0
    time.sleep(burn_time - 0.1)

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
    vessel.auto_pilot.disengage()
    time.sleep(1)

    # separate stage and prepare to re-entry
    status_update(text_status, "Stage Separation")
    vessel.control.activate_next_stage()

    srf_altitude = conn.get_call(getattr, vessel.flight(), 'surface_altitude')

    status_update(text_status, "Waiting for re-entry")
    expr = conn.krpc.Expression.less_than(
        conn.krpc.Expression.call(srf_altitude),
        conn.krpc.Expression.constant_double(70500))
    event = conn.krpc.add_event(expr)
    with event.condition:
        event.wait()

    status_update(text_status, "re-entry and waiting for deploying chute")

    expr = conn.krpc.Expression.less_than(
        conn.krpc.Expression.call(srf_altitude),
        conn.krpc.Expression.constant_double(2000))
    event = conn.krpc.add_event(expr)
    with event.condition:
        event.wait()

    status_update(text_status, "deploy Chute!")
    vessel.control.activate_next_stage()

    while vessel.flight(vessel.orbit.body.reference_frame).vertical_speed < -0.1:
        status_update(text_status, "Altitude = {} meters".format(vessel.flight().surface_altitude))
        time.sleep(1)
    status_update(text_status, "Landed!")

if __name__ == "__main__":
    main()