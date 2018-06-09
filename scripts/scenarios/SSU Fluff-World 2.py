import os
import math
import time
import krpc
from scripts.utils.status_dialog import StatusDialog
from scripts.utils.launch_into_orbit import launch_into_orbit
from scripts.utils.hohmann_transfer import hohmann_transfer_to_target
from scripts.utils.maneuver import circularize, change_apoapsis, change_periapsis

TARGET_ALTITUDE = 130000
TARGET_INCLINATION = 0

def main():
    krpc_address = os.environ["KRPC_ADDRESS"]
    conn = krpc.connect(name='SSU Fluff-World 2', address=krpc_address)

    # setup stream
    vessel = conn.space_center.active_vessel
    ut = conn.add_stream(getattr, conn.space_center, 'ut')

    # launch into low kerbal orbit
    launch_into_orbit(conn, TARGET_ALTITUDE, TARGET_INCLINATION,
                      turn_start_alt = 15000, turn_end_alt = 45000,
                      auto_stage=True, use_rcs_on_circulization=False)

    # circulize again
    next_time_ap = ut() + vessel.orbit.time_to_apoapsis 
    circularize(conn, next_time_ap)

    # our target body is Mun!
    conn.space_center.target_body = conn.space_center.bodies["Mun"]
    hohmann_transfer_to_target(conn)
    
    # warp to soi change (Kerbin -> Mun)
    conn.space_center.warp_to(ut() + vessel.orbit.time_to_soi_change + 30)
    time.sleep(5)

    ## Flyby Mun
    change_periapsis(conn, ut() + 300, 30000)

    # warp to soi change (Mun -> Kerbin)
    conn.space_center.warp_to(ut() + vessel.orbit.time_to_soi_change + 30)
    time.sleep(5)

    ## reentry
    change_periapsis(conn, ut() + 300, 40000)

    conn.space_center.warp_to(ut() + vessel.orbit.time_to_periapsis - 120)

    vessel.auto_pilot.engage()
    vessel.auto_pilot.reference_frame = vessel.surface_velocity_reference_frame
    vessel.auto_pilot.target_heading = 180
    time.sleep(5)
    vessel.auto_pilot.wait()

    vessel.control.throttle = 1.0

    periapsis_altitude = conn.get_call(getattr, vessel.orbit, 'periapsis_altitude')
    expr = conn.krpc.Expression.less_than(
        conn.krpc.Expression.call(periapsis_altitude),
        conn.krpc.Expression.constant_double(0))
    event = conn.krpc.add_event(expr)
    with event.condition:
        event.wait()
    vessel.control.throttle = 0.0
    time.sleep(5)

    vessel.control.activate_next_stage()

    srf_altitude = conn.get_call(getattr, vessel.flight(), 'surface_altitude')
    expr = conn.krpc.Expression.less_than(
        conn.krpc.Expression.call(srf_altitude),
        conn.krpc.Expression.constant_double(2000))
    event = conn.krpc.add_event(expr)
    with event.condition:
        event.wait()

    vessel.control.activate_next_stage()

if __name__ == "__main__":
    main()
