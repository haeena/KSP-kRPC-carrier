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
    conn = krpc.connect(name='ACU Tabby Ranger II', address=krpc_address)

    # setup stream
    vessel = conn.space_center.active_vessel
    ut = conn.add_stream(getattr, conn.space_center, 'ut')
    dialog = StatusDialog(conn)

    # launch into low kerbal orbit
    launch_into_orbit(conn, TARGET_ALTITUDE, TARGET_INCLINATION,
                    turn_start_alt = 15000, turn_end_alt = 45000)

    # into transfer orbit
    dialog.status_update("Into keo-synchronous transfer orbit")
    change_apoapsis(conn, ut() + 300, 2863000)

    # separate relays
    dialog.status_update("Separate relays from launcher")
    while vessel.control.current_stage > 0:
        vessel.control.activate_next_stage()

    # rename relays
    vessel_name = vessel.name
    vessel_series = [ v for v in conn.space_center.vessels if v.name.startswith(vessel_name) and v.type.name == "relay" and v != vessel ]
    vessels_this_launch = [vessel]
    current_numbering = 0
    for v in vessel_series:
        # met <= 60 should be vessel just separated
        if v.met <= 60:
            vessels_this_launch.append(v)
            continue
        last_token = v.name.split(" ")[-1]
        try:
            current_numbering = max(int(last_token), current_numbering)
        except:
            continue
    for v in vessels_this_launch:
        current_numbering = current_numbering + 1
        old_name = v.name
        new_name = vessel_name + " {}".format(current_numbering)
        dialog.status_update("Renaming vessels: {} becomes {}".format(old_name, new_name))
        v.name = new_name

    # warp 60 sec for separation
    dialog.status_update("Waiting for safe separation")
    conn.space_center.warp_to(ut() + 60)

    # deploy solar panels
    for v in vessels_this_launch:
        conn.space_center.active_vessel = v
        dialog.status_update("Deploying solar panels: {}".format(v.name))
        for p in v.parts.solar_panels:
            p.deployed = True
        time.sleep(10)
    
    # circularize
    for v in vessels_this_launch:
        conn.space_center.active_vessel = v
        dialog.status_update("To final orbit: {}".format(v.name))
        change_apoapsis(conn, ut() + 300, 2863000)
        circularize(conn, ut() + v.orbit.time_to_apoapsis)
        time.sleep(10)

if __name__ == "__main__":
    main()
