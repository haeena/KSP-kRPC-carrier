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
        last_distance_error, bearing = landing_target_steering(vessel, target_lat, target_lon)
        last_throttle = 0

        while True:
            a100 = available_thrust() / mass()
            bounding_box = vessel.bounding_box(vessel.surface_reference_frame)
            lower_bound = bounding_box[0][0]

            landing_radius = body.equatorial_radius + lower_bound
            if guided_landing:
                landing_radius = max(landing_radius, landing_radius + body.surface_height(target_lat, target_lon))

            impact_ut, terminal_speed = impact_prediction(vessel.orbit, landing_radius)
            burn_time = burn_prediction(terminal_speed, a100)
            burn_lead_time = impact_ut - burn_time - ut()
            if burn_lead_time < 30:
                break

            if atmosphere_depth > apoapsis_altitude():
                break

            distance_error, bearing = landing_target_steering(vessel, target_lat, target_lon)
            vessel.auto_pilot.target_pitch_and_heading(0, bearing)

            if distance_error < 500:
                break
            
            if vessel.auto_pilot.heading_error < 1:
                try:
                    instant_rate_per_throttle = (distance_error - last_distance_error) / ((ut() - last_ut) * last_throttle)
                    instant_rate_per_throttle = max(1.0, instant_rate_per_throttle)
                    vessel.control.throttle = min(1, max(0.05, distance_error / instant_rate_per_throttle))
                except:
                    vessel.control.throttle = 0.05
            else:
                vessel.control.throttle = 0
            
            dialog.status_update("Distance error: {: 5.3f}".format(distance_error))

            last_distance_error = distance_error
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
        warp_to_radius = atmosphere_depth + body.equatorial_radius
        entry_ut, terminal_speed = impact_prediction(vessel.orbit, warp_to_radius)
        sec_until_entry = entry_ut - ut()
        if sec_until_entry > 30:
            dialog.status_update("Warp for entry - 5sec: {: 5.3f}".format(sec_until_entry))
            conn.space_center.warp_to(entry_ut - 5)
            time.sleep(5)

    ####
    # landing phase
    vessel.control.rcs = use_rcs_on_landing

    # warp for burn
    if not has_atmosphere:
        last_ut = ut()
        while True:
            a100 = available_thrust() / mass()
            lower_bound = vessel.bounding_box(vessel.surface_reference_frame)[0][0]

            landing_radius = body.equatorial_radius + lower_bound
            if guided_landing:
                landing_radius = max(landing_radius, landing_radius + body.surface_height(target_lat, target_lon))

            impact_ut, terminal_speed = impact_prediction(vessel.orbit, landing_radius)
            burn_time = burn_prediction(terminal_speed, a100)
            burn_lead_time = impact_ut - burn_time - ut()

            if burn_lead_time and burn_lead_time > (ut() - last_ut)*1.5+2:
                if burn_lead_time > 30:
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
        impact_ut, terminal_speed = impact_prediction(vessel.orbit, landing_radius)
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
            throttle = max(0, min(1.0, ( speed_prediction(vessel, vertical_speed(), horizontal_speed(), surface_gravity) - landing_speed) / a100))
            vessel.control.throttle = throttle
        else:
            vessel.control.throttle = 0.05

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

def landing_prediction(vessel: Vessel) -> (float, float, float):
    """landing position prediction

    return prediction of landing position at current body

    Args:
        vessel: current radius from center of body 

    Returns:
        return state vector (x, y, z) on body
    """
    bref = vessel.orbit.body.reference_frame
    bmu = vessel.orbit.body.gravitational_parameter
    br = vessel.orbit.body.equatorial_radius

    r = vessel.position(bref)
    v = vessel.velocity(bref) 

    r = [r[0], r[2], r[1]]    
    v = [v[0], v[2], v[1]]

    # calculate orbital elements from state vectors
    h = np.cross(r, v)
    e = np.subtract(np.divide(np.cross(v, h), bmu), np.divide(r, norm(r)))    
    ec = norm(e)

    n = np.transpose([-h[1], h[0], 0])

    if np.dot(r, v)>=0:
        nu = np.arccos(np.dot(e, r)/(ec*norm(r)))
    else:
        nu = 2*np.pi - np.arccos(np.dot(e, r)/(ec*norm(r)))

    i = np.arccos(h[2]/norm(h))
    if n[1]>=0:
        RAAN = np.arccos(n[0]/norm(n))
    else:
        RAAN = 2*np.pi - np.arccos(n[0]/norm(n))

    if e[2]>=0:
        w = np.arccos(np.dot(n, e)/(ec*norm(n)))
    else:
        w = 2*np.pi - np.arccos(np.dot(n, e)/(ec*norm(n)))

    a = (2.0/norm(r) - (norm(v)**2.0)/bmu)**-1
    
    elem = [a, ec, i, w, RAAN, nu]
    
    # calculate new true anomaly when distance to centre of body is equation radius on the orbit
    elem[5] = -np.arccos(((1.0-elem[1]**2.0)*(elem[0]/br)-1.0)/elem[1])
    
    # convert back to state vectors
    X = np.cos(elem[4])*np.cos(elem[3]+elem[5]) - np.sin(elem[4])*np.sin(elem[3]+elem[5])*np.cos(elem[2])
    Y = np.sin(elem[4])*np.cos(elem[3]+elem[5]) + np.cos(elem[4])*np.sin(elem[3]+elem[5])*np.cos(elem[2])
    Z = np.sin(elem[2])*np.sin(elem[3]+elem[5])
    
    X *= br
    Y *= br
    Z *= br
    
    R = [X, Y, Z]
    
    return R

def impact_prediction(orbit: Orbit, radius: float):
    """impact_prediction

    Extended description of function.

    Args:
        radius: current radius from center of body 
        target_altitude: altitude of target position
        vertical_speed: vertical speed
        horizontal_speed: horizontal speed
        mg: mass of body * gravitational_parameter

    Returns:
        return None if not impact
        return (impact_time, terminal_speed)
    """

    # radius is not achived
    if radius < orbit.periapsis or ( orbit.eccentricity < 1 and radius > orbit.apoapsis):
        return None        

    true_anomary1 = orbit.true_anomaly_at_radius(radius)
    true_anomary2 = 2 * math.pi - true_anomary1
    time1 = orbit.ut_at_true_anomaly(true_anomary1)
    time2 = orbit.ut_at_true_anomaly(true_anomary2)
    
    if time2 < time1:
        return time2, orbit.orbital_speed_at(time2)
    else:
        return time1, orbit.orbital_speed_at(time2)

def burn_prediction(delta_v: float, acceleration: float):
    return delta_v / acceleration

def landing_target_steering(vessel: Vessel, target_lat: float, target_lon: float) -> (float, float):
    bref = vessel.orbit.body.reference_frame
    br = vessel.orbit.body.equatorial_radius

    # vespos = vessel.position(bref)
    # tpos = vessel.orbit.body.surface_position(target_lat, target_lon, bref)
    predpos = landing_prediction(vessel)

    pred_lat, pred_lon = latlon(predpos)
    cur_lat = vessel.flight(bref).latitude
    cur_lon = vessel.flight(bref).longitude
    
    target_lat = d2r(target_lat)
    target_lon = d2r(target_lon)
    pred_lat = d2r(pred_lat)
    pred_lon = d2r(pred_lon)
    cur_lat = d2r(cur_lat)
    cur_lon = d2r(cur_lon)

    lat_error = target_lat - pred_lat
    lon_error = target_lon - pred_lon

    # calcurate great circule distance using Haversine formula
    a = math.sin(lat_error/2) * math.sin(lat_error/2) + math.cos(target_lat) * math.cos(pred_lat) * math.sin(lon_error/2) * math.sin(lon_error/2)
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
    distance_error = br * c

    y = math.sin(lon_error) * math.cos(target_lat)
    x = math.cos(pred_lat) * math.sin(target_lat) - math.sin(pred_lat) * math.cos(target_lat) * math.cos(lon_error)
    bearing = r2d(math.atan2(y, x)) % 360

    return distance_error, bearing

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
