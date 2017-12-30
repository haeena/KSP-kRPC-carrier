import time
import math

from krpc.client import Client
from scripts.utils.status_dialog import StatusDialog

def execute_next_node(conn: Client) -> None:
    vessel = conn.space_center.active_vessel
    nodes = vessel.control.nodes

    if not nodes:
        return
    node = nodes[0]

    ## setup dialog and streams
    dialog = StatusDialog(conn)
    ut = conn.add_stream(getattr, conn.space_center, 'ut')

    # Calculate burn time (using rocket equation)
    F = vessel.available_thrust
    Isp = vessel.specific_impulse * 9.82
    m0 = vessel.mass
    m1 = m0 / math.exp(node.delta_v/Isp)
    flow_rate = F / Isp
    burn_time = (m0 - m1) / flow_rate

    # Orientate ship
    dialog.status_update("Orientating ship for next burn")
    vessel.auto_pilot.engage()
    vessel.auto_pilot.reference_frame = node.reference_frame
    vessel.auto_pilot.target_direction = (0, 0, 0)
    vessel.auto_pilot.wait()

    # Wait until burn
    dialog.status_update("Waiting until burn time")
    burn_ut = node.ut - (burn_time/2.0)
    lead_time = 5
    conn.space_center.warp_to(burn_ut - lead_time)

    # Execute burn
    dialog.status_update("Ready to execute burn")
    while ut() - burn_ut > 0:
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

    return

if __name__ == "__main__":
    import krpc
    conn = krpc.connect(name='execute node')
    execute_next_node(conn)