import time
import math
import numpy
from functools import reduce
from krpc.client import Client

# TODO: type hint for kRPC remote objects may need to be separated
from typing import Union, NewType, Callable

Vessel = NewType("Vessel", object)
Body = NewType("Body", object)
Orbit = NewType("Orbit", object)
Node = NewType("Node", object)

from scripts.utils.utils import *
from scripts.utils.status_dialog import StatusDialog
from scripts.utils.execute_node import execute_next_node
from scripts.utils.autostage import set_autostaging, unset_autostaging

def vertical_landing(conn: Client,
                     landing_speed: float = 5.0,
                     auto_stage: bool = True,
                     stop_stage: int = 0,
                     target_lat: float = None, target_lon: float = None,
                     deploy_legs_on_entry: bool = True,
                     retract_palens_on_entry: bool = True,
                     use_rcs_on_entry: bool = False,
                     entry_attitude: str = "Retrograde",
                     entry_attitude_func: Callable[[Vessel], None] = None,
                     deploy_legs_on_decent: bool = True,
                     retract_palens_on_decent: bool = True,
                     use_rcs_on_landing: bool = False,
                     use_parachute: bool = True
                    ) -> None:
    """Vertical landing

    Extended description of function.

    Args:
        conn: kRPC connection
        auto_stage: staging when no fuel left on the stage
        stop_stage: stop staging on the stage
        deploy_legs_on_decent: extend legs on landing
        retract_palens_on_decent: retract panels on landing
        use_rcs_on_landing: use rcs or not
        use_parachute: use parachute

    Returns:
        return nothing, return when procedure finished
    """
    vessel = conn.space_center.active_vessel
    body = vessel.orbit.body

    ## check retrograde and radial hold capability
    use_sas = False
    try:
        vessel.control.sas = True
        vessel.control.sas_mode = vessel.control.sas_mode.retrograde
        vessel.control.sas_mode = vessel.control.sas_mode.radial
        use_sas = True
    except:
        pass        
    vessel.control.sas = False

    # Set up dialog and stream
    dialog = StatusDialog(conn)
    surface_gravity = body.surface_gravity
    equatorial_radius = body.equatorial_radius
    has_atmosphere = body.has_atmosphere
    atmosphere_depth = body.atmosphere_depth

    ref_frame = conn.space_center.ReferenceFrame.create_hybrid(
        position=body.reference_frame,
        rotation=vessel.surface_reference_frame)
    flight = vessel.flight(ref_frame)

    ut = conn.add_stream(getattr, conn.space_center, 'ut')
    mass = conn.add_stream(getattr, vessel, 'mass')
    available_thrust = conn.add_stream(getattr, vessel, 'available_thrust')
    radius = conn.add_stream(getattr, vessel.orbit, 'radius')
    apoapsis_altitude = conn.add_stream(getattr, vessel.orbit, 'apoapsis_altitude')
    altitude = conn.add_stream(getattr, flight, 'surface_altitude')
    mean_altitude = conn.add_stream(getattr, flight, 'mean_altitude')
    speed = conn.add_stream(getattr, flight, 'speed')
    vertical_speed = conn.add_stream(getattr, flight, 'vertical_speed')
    horizontal_speed = conn.add_stream(getattr, flight, 'horizontal_speed')
    terminal_velocity = conn.add_stream(getattr, flight, 'terminal_velocity')

    vessel.control.sas = True
    vessel.control.speed_mode = vessel.control.speed_mode.surface

    # set staging
    if auto_stage:
        set_autostaging(conn, stop_stage=stop_stage)

    ## check unguided or guided
    guided_landing = True
    if target_lat == None or target_lon == None:
        guided_landing = False

    if not guided_landing:
        kill_horizontal_velocity(conn, use_sas)

    #### 
    # pre-entry phase
    vessel.control.rcs = use_rcs_on_entry

    # pre-entry guidance
    if guided_landing:
        vessel.auto_pilot.reference_frame = ref_frame
        vessel.auto_pilot.engage()

        last_ut = ut()
        bearing, distance, landing_position_error = landing_target_steering(vessel, target_lat, target_lon, ut())
        last_landing_position_error = landing_position_error
        last_throttle = 0

        while True:
            a100 = available_thrust() / mass()
            bounding_box = vessel.bounding_box(vessel.surface_reference_frame)
            lower_bound = bounding_box[0][0]

            landing_radius = equatorial_radius + lower_bound
            if guided_landing:
                landing_radius = max(landing_radius, landing_radius + body.surface_height(target_lat, target_lon))

            bearing, distance, landing_position_error = landing_target_steering(vessel, target_lat, target_lon, ut())

            if has_atmosphere:
                atmosphere_radius = equatorial_radius + atmosphere_depth
                if atmosphere_depth > altitude() and vertical_speed() < 0:
                    break

                entry_ut, entry_speed = time_to_radius(vessel.orbit, atmosphere_radius, ut())
                entry_lead_time = entry_ut - ut()

                if landing_position_error/distance < 0.05:
                    if entry_lead_time > 120:
                        conn.space_center.warp_to(entry_ut - 60)
                    else:
                        break
            else:
                impact_ut, terminal_speed = time_to_radius(vessel.orbit, landing_radius, ut())
                burn_time = burn_prediction(terminal_speed, a100)
                burn_ut = impact_ut - burn_time
                burn_lead_time = burn_ut - ut()
                if burn_lead_time < 30:
                    break
                if landing_position_error/distance < 0.05:
                    if burn_lead_time > 10:
                        conn.space_center.warp_to(burn_ut - 60)
                    else:
                        break

            vessel.auto_pilot.target_pitch_and_heading(0, bearing)

            if vessel.auto_pilot.heading_error < 1:
                try:
                    landing_pos_corrected = last_landing_position_error - landing_position_error
                    dt = ut() - last_ut
                    instant_rate_per_throttle = landing_pos_corrected / dt / last_throttle
                    instant_rate_per_throttle = max(1.0, instant_rate_per_throttle)
                    vessel.control.throttle = min(1.0, max(0.05, landing_position_error / instant_rate_per_throttle))
                except:
                    vessel.control.throttle = 0.05
            else:
                vessel.control.throttle = 0
            
            dialog.status_update(f"landing_position error: {landing_position_error: 5.3f}, bearing: {bearing: 5.3f}")

            last_ut = ut()
            last_landing_position_error = landing_position_error
            last_throttle = vessel.control.throttle

        vessel.control.throttle = 0
        vessel.auto_pilot.disengage()

    ####
    # entry
    vessel.control.sas = True
    vessel.control.sas_mode = vessel.control.sas_mode.retrograde

    # on entry: deploy leg, retract panel
    if deploy_legs_on_entry:
        deploy_legs(conn)
    if retract_palens_on_entry:
        retract_panels(conn)

    # wait for entry
    if has_atmosphere and atmosphere_depth < altitude():
        warp_to_radius = atmosphere_depth + equatorial_radius
        entry_ut, terminal_speed = time_to_radius(vessel.orbit, warp_to_radius, ut())
        sec_until_entry = entry_ut - ut()
        if sec_until_entry > 30:
            dialog.status_update("Warp for entry - 5sec: {: 5.3f}".format(sec_until_entry))
            conn.space_center.warp_to(entry_ut - 5)
            time.sleep(5)

    ####
    # landing phase
    vessel.control.rcs = use_rcs_on_landing

    # warp for burn
    last_ut = ut()
    while True:
        a100 = available_thrust() / mass()
        lower_bound = vessel.bounding_box(vessel.surface_reference_frame)[0][0]

        landing_radius = equatorial_radius + lower_bound
        landing_altitude = altitude() + lower_bound
        if guided_landing:
            landing_radius = max(landing_radius, landing_radius + body.surface_height(target_lat, target_lon))
            landing_altitude = max(landing_altitude, landing_altitude + body.surface_height(target_lat, target_lon))

        impact_ut, terminal_speed = impact_prediction(radius(), landing_altitude, vertical_speed(), horizontal_speed(), surface_gravity, ut())
        burn_time = burn_prediction(terminal_speed, a100)
        burn_lead_time = impact_ut - burn_time - ut()

        if burn_lead_time and burn_lead_time > (ut() - last_ut)*1.5+2:
            if  not has_atmosphere and burn_lead_time > 30:
                dialog.status_update("Warp for decereration burn - 30sec: {: 5.3f}".format(burn_lead_time))
                conn.space_center.warp_to(ut() + burn_lead_time - 30)
                time.sleep(5)
            else:
                dialog.status_update("Wait for decereration burn: {: 5.3f} sec; ut: {: 5.3f}".format(burn_lead_time, ut()))
        else:
            break
        last_ut = ut()
        time.sleep(0.1)

    # on decent: deploy leg, retract panel
    if deploy_legs_on_decent:
        deploy_legs(conn)
    if retract_palens_on_decent:
        retract_panels(conn)

    if not guided_landing:
        # kill horizontal velocity again
        kill_horizontal_velocity(conn, use_sas)

    # Main decent loop
    last_sas_mode = vessel.control.sas_mode
    while True:
        a100 = available_thrust() / mass()
        bounding_box = vessel.bounding_box(vessel.surface_reference_frame)
        lower_bound = bounding_box[0][0]

        landing_radius = mean_altitude() + lower_bound
        landing_altitude = altitude() + lower_bound
        impact_ut, terminal_speed = impact_prediction(radius(), landing_altitude, vertical_speed(), horizontal_speed(), surface_gravity, ut())
        burn_time = burn_prediction(terminal_speed, a100)
        burn_lead_time = impact_ut - burn_time - ut()

        dialog.status_update("Alt: {: 5.3f}, Speed {: 5.3f} m/s (H: {: 5.3f}, V: {: 5.3f}), "\
                             "a: {: 5.3f}, g: {: 5.3f}, "\
                             "landing in: {: 5.3f} sec, burn lead time: {: 5.3f} sec"\
                             "".format(altitude(), speed(), horizontal_speed(), vertical_speed(),
                                       a100, surface_gravity,
                                       impact_ut - ut(), burn_lead_time)
                            )

        if use_sas:
            if (horizontal_speed() > 0.5 and speed() > 1.0) and last_sas_mode != vessel.control.sas_mode.retrograde:
                vessel.control.sas_mode = vessel.control.sas_mode.retrograde
                last_sas_mode = vessel.control.sas_mode.retrograde
            elif last_sas_mode != vessel.control.sas_mode.radial:
                vessel.control.sas_mode = vessel.control.sas_mode.radial
                last_sas_mode = vessel.control.sas_mode.radial
        else:
            # TODO: auto-pilot
            pass

        if burn_lead_time < 0.1:
            throttle = max(0, min(1.0, ( abs(speed_prediction(vessel, vertical_speed(), horizontal_speed(), surface_gravity)) - landing_speed) / a100))
            vessel.control.throttle = throttle

        if is_grounded(vessel):
            vessel.control.sas_mode.radial
            vessel.control.throttle = 0
            break

        last_ut = ut()

    dialog.status_update("Landed")

    # keep sas on for a bit to maintain landing stability
    time.sleep(5)

    if use_sas:
        vessel.control.sas = False
    else:
        vessel.auto_pilot.disengage() 

    if auto_stage:
        unset_autostaging()

    return

def speed_prediction(vessel: Vessel, vertical_speed: float, horizontal_speed: float, surface_gravity: float) -> float:
    return math.sqrt((vertical_speed - surface_gravity)**2 + horizontal_speed**2)

def time_to_radius(orbit: Orbit, radius: float, ut: float):
    """time to radius

    Extended description of function.

    Args:
        orbit: current orbit 
        radius: target radius

    Returns:
        return None if not impact
        return (time_ut, orbital speed at radius)
    """

    # radius is not achived
    if radius < orbit.periapsis or ( orbit.eccentricity < 1 and radius > orbit.apoapsis):
        return None        

    true_anomary1 = orbit.true_anomaly_at_radius(radius)
    true_anomary2 = 2 * math.pi - true_anomary1
    time1 = orbit.ut_at_true_anomaly(true_anomary1)
    time2 = orbit.ut_at_true_anomaly(true_anomary2)
    
    # KSP GetUTforTrueAnomaly may have a bug?
    while time1 - orbit.period > ut:
        time1 = time1 - orbit.period
    while time2 - orbit.period > ut:
        time2 = time2 - orbit.period

    if time2 < time1:
        return time2, orbit.orbital_speed_at(time2 - ut)
    else:
        return time1, orbit.orbital_speed_at(time1 - ut)

def impact_prediction(radius: float, target_altitude: float,
                      vertical_speed: float, horizontal_speed: float,
                      surface_gravity: float, ut: float):
    """impact_prediction

    Extended description of function.

    Args:
        radius: current radius from center of body 
        target_altitude: surface altitude
        vertical_speed: vertical speed
        horizontal_speed: horizontal speed
        surface_gravity: surface gravity
        ut: current ut

    Returns:
        return None if not impact
        return (impact_ut, terminal_speed)
    """
    fall_speed = -vertical_speed
    downward_acceleration = surface_gravity - horizontal_speed * horizontal_speed / radius

    # do we land?
    if downward_acceleration < 0:
        max_fall_distance = -(fall_speed * fall_speed) / (2.0 * downward_acceleration)
        if max_fall_distance < (target_altitude + 1.0):
            return None

    sec_until_impact = (-fall_speed + math.sqrt(fall_speed * fall_speed + 2.0 * downward_acceleration * target_altitude)) / downward_acceleration

    vertical_speed_at_impact = fall_speed + sec_until_impact * downward_acceleration
    impact_speed = math.sqrt(vertical_speed_at_impact * vertical_speed_at_impact + horizontal_speed * horizontal_speed)
    impact_ut = sec_until_impact + ut
    return impact_ut, impact_speed

def burn_prediction(delta_v: float, acceleration: float):
    return delta_v / acceleration

def landing_target_steering(vessel: Vessel, target_lat: float, target_lon: float, ut: float) -> (float, float, float):
    br = vessel.orbit.body.equatorial_radius
    bref = vessel.orbit.body.reference_frame
    brotate = vessel.orbit.body.rotational_speed

    current_pos = vessel.position(bref)
    target_pos = vessel.orbit.body.surface_position(target_lat, target_lon, bref)
    
    landing_time, terminal_speed = time_to_radius(vessel.orbit, br, ut)
    landing_pos = vessel.orbit.position_at(landing_time, bref)
    land_lat, land_lon = latlon(landing_pos)
    land_lon = clamp_2pi(land_lon + brotate * (landing_time - ut) ) - math.pi
    
    bearing = bearing_between_coords(land_lat, land_lon, target_lat, target_lon)
    distance = np.linalg.norm(np.subtract(current_pos, landing_pos))
    error = np.linalg.norm(np.subtract(target_pos, landing_pos))

    return bearing, distance, error

def kill_horizontal_velocity(conn: Client, use_sas: bool = True):
    vessel = conn.space_center.active_vessel

    # Set up dialog and stream
    dialog = StatusDialog(conn)
    ref_frame = conn.space_center.ReferenceFrame.create_hybrid(
        position=vessel.orbit.body.reference_frame,
        rotation=vessel.surface_reference_frame)
    flight = vessel.flight(ref_frame)
    mass = conn.add_stream(getattr, vessel, 'mass')
    available_thrust = conn.add_stream(getattr, vessel, 'available_thrust')
    altitude = conn.add_stream(getattr, flight, 'surface_altitude')
    speed = conn.add_stream(getattr, flight, 'speed')
    vertical_speed = conn.add_stream(getattr, flight, 'vertical_speed')
    horizontal_speed = conn.add_stream(getattr, flight, 'horizontal_speed')

    dialog.status_update("kill horizontal velocity")

    if use_sas:
        vessel.control.sas = True
        vessel.control.sas_mode = vessel.control.sas_mode.retrograde
        while angle_between(vessel.flight(vessel.surface_velocity_reference_frame).direction,(0,-1,0)) > 5/180 * math.pi:
            time.sleep(0.1)
    else:
        vessel.auto_pilot.engage()
        vessel.auto_pilot.reference_frame = vessel.surface_velocity_reference_frame
        vessel.auto_pilot.target_direction = (0, -1, 0)
        vessel.auto_pilot.wait()

    while True:
        a100 = available_thrust() / mass()
        dialog.status_update("kill horizontal velocity: Alt: {: 5.3f}, Speed {: 5.3f} m/s (H: {: 5.3f}, V: {: 5.3f})"\
                             "".format(altitude(), speed(), horizontal_speed(), vertical_speed()))

        if horizontal_speed() > 0.1:
            vessel.control.throttle = max(0, min(1.0, speed() / a100))
        else:
            vessel.control.throttle = 0
            break

def retract_panels(conn: Client):
    vessel = conn.space_center.active_vessel
    dialog = StatusDialog(conn)

    dialog.status_update("Retract solar/radiator panels")
    for panel in vessel.parts.solar_panels + vessel.parts.radiators:
        if panel.deployable:
            panel.deployed = False    

def deploy_legs(conn: Client):
    vessel = conn.space_center.active_vessel
    dialog = StatusDialog(conn)

    dialog.status_update("Deploy legs")
    for leg in vessel.parts.legs:
        if leg.deployable:
            leg.deployed = True    

def is_grounded(vessel: Vessel):
    return max([False] + [ l.is_grounded for l in vessel.parts.legs ])

if __name__ == "__main__":
    import os
    import krpc
    krpc_address = os.environ["KRPC_ADDRESS"]
    con = krpc.connect(name='Landing', address=krpc_address)
    vertical_landing(con, target_lat = -0.09, target_lon = -75.557, use_rcs_on_entry = True)
