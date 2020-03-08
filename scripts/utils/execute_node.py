import math
import time

from krpc.client import Client
from scripts.utils.autostage import set_autostaging, unset_autostaging
from scripts.utils.status_dialog import StatusDialog
from scripts.utils.utils import angle_between


def execute_next_node(
    conn: Client, auto_stage: bool = True, stop_stage: int = 0
) -> None:
    vessel = conn.space_center.active_vessel
    nodes = vessel.control.nodes

    if not nodes:
        return
    node = nodes[0]

    # check manuever hold capability
    use_sas = False
    try:
        vessel.control.sas = True
        vessel.control.sas_mode = vessel.control.sas_mode.maneuver
        use_sas = True
    except Exception:
        pass
    vessel.control.sas = False

    # setup dialog and streams
    dialog = StatusDialog(conn)
    ut = conn.add_stream(getattr, conn.space_center, "ut")
    direction = conn.add_stream(
        getattr, vessel.flight(node.reference_frame), "direction"
    )

    # setup autostaging
    if auto_stage:
        set_autostaging(conn, stop_stage=stop_stage)

    # Calculate burn time (using rocket equation)
    F = vessel.available_thrust
    Isp = vessel.specific_impulse * 9.82
    m0 = vessel.mass
    m1 = m0 / math.exp(node.delta_v / Isp)
    flow_rate = F / Isp
    burn_time = (m0 - m1) / flow_rate

    # Orientate ship
    dialog.status_update("Orientating ship for next burn")
    if use_sas:
        vessel.control.sas = True
        vessel.control.sas_mode = vessel.control.sas_mode.maneuver
        while angle_between(direction(), (0, 1, 0)) > 0.5 / 180 * math.pi:
            time.sleep(0.1)
    else:
        vessel.auto_pilot.engage()
        vessel.auto_pilot.reference_frame = node.reference_frame
        vessel.auto_pilot.target_direction = (0, 1, 0)
        while angle_between(direction(), (0, 1, 0)) > 1 / 180 * math.pi:
            time.sleep(0.1)

    # Wait until burn
    dialog.status_update("Waiting until burn time")
    burn_ut = node.ut - (burn_time / 2.0)
    lead_time = 5
    conn.space_center.warp_to(burn_ut - lead_time)

    # Execute burn
    dialog.status_update("Ready to execute burn")
    while burn_ut - ut() > 0:
        pass
    dialog.status_update("Executing burn")
    vessel.control.throttle = 1.0

    remaining_delta_v = conn.add_stream(getattr, node, "remaining_delta_v")
    min_delta_v = remaining_delta_v()

    state_fine_tuning = False
    point_passed = False
    while remaining_delta_v() > 0.1 and not point_passed:
        a100 = vessel.available_thrust / vessel.mass
        if a100 == 0:
            if auto_stage:
                time.sleep(0.05)
                continue
            else:
                break
        throttle = max(0.05, min(1.0, remaining_delta_v() / a100))
        if throttle < 1.0:
            if not state_fine_tuning:
                dialog.status_update("Fine tuning")
                state_fine_tuning = True
        vessel.control.throttle = throttle
        if min_delta_v < remaining_delta_v():
            point_passed = True
        else:
            min_delta_v = remaining_delta_v()
        pass
    vessel.control.throttle = 0.0
    remaining_delta_v.remove()
    node.remove()
    if auto_stage:
        unset_autostaging()

    vessel.control.sas = True
    time.sleep(1)
    vessel.control.sas = False
    vessel.auto_pilot.disengage()

    return


if __name__ == "__main__":
    import os
    import krpc

    krpc_address = os.environ["KRPC_ADDRESS"]
    conn = krpc.connect(name="execute node", address=krpc_address)
    execute_next_node(conn)
