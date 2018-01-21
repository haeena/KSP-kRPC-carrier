import time
import krpc
from scripts.utils.launch_into_orbit import launch_into_orbit
from scripts.utils.hohmann_transfer import hohmann_transfer_to_target
from scripts.utils.maneuver import circularize, change_apoapsis, change_periapsis

TARGET_ALTITUDE = 120000
TARGET_INCLINATION = 0

def main():
    conn = krpc.connect(name='SSU Chocolate Pioneer 4')

    # setup stream
    vessel = conn.space_center.active_vessel
    ut = conn.add_stream(getattr, conn.space_center, 'ut')

    # launch into low kerbal orbit
    #launch_into_orbit(conn, TARGET_ALTITUDE, TARGET_INCLINATION, auto_stage=False)

    # circulize again
    #next_time_ap = ut() + vessel.orbit.time_to_apoapsis 
    #circularize(conn, next_time_ap)

    # our target body is Mun!
    #conn.space_center.target_body = conn.space_center.bodies["Mun"]
    #hohmann_transfer_to_target(conn)
    
    # warp to soi change
    #conn.space_center.warp_to(ut() + vessel.orbit.time_to_soi_change)
    #time.sleep(5)

    # change periapsis altitude of mun to 30km at 5min
    change_periapsis(conn, ut() + 300, 30000)
    # TODO: ok this does't work since prograde is targetting straight into Mun CoG

    # warp to soi change
    conn.space_center.warp_to(ut() + vessel.orbit.time_to_soi_change)



if __name__ == "__main__":
    main()
