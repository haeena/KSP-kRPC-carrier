import math
from typing import NewType

import numpy as np
from astropy import units as AstropyUnit
from krpc.client import Client
from numpy.linalg import norm
from poliastro.maneuver import Maneuver
from poliastro.twobody import Orbit as PoliastroOrbit
from scripts.utils.execute_node import execute_next_node
from scripts.utils.krpc_poliastro import krpc_poliastro_bodies
from scripts.utils.status_dialog import StatusDialog


# TODO: type hint for kRPC remote objects may need to be separated
Vessel = NewType("Vessel", object)
Body = NewType("Body", object)
Orbit = NewType("Orbit", object)
ReferenceFrame = NewType("ReferenceFrame", object)


def unit_vector(vector):
    return vector / norm(vector)


def dot(v1, v2):
    v1_u = unit_vector(v1)
    v2_u = unit_vector(v2)
    return np.clip(np.dot(v1_u, v2_u), -1.0, 1.0)


def velocity_at_ut(
    orbit: Orbit, at_ut: float, reference_frame: ReferenceFrame = None
):
    """Return velocity vector of orbit at ut

    Return velocity vector of orbit at ut
    If we could access KSP Orbit.getOrbitalVelocityAtUT() via kRPC api...

    Args:
        orbit: kRPC orbit object
        at_ut: at specific time
        reference_frame: reference_frame to be used

    Returns:
        return velocity vector
    """
    if not reference_frame:
        attractor = orbit.body
        reference_frame = attractor.non_rotating_reference_frame

    velocity_vector = np.subtract(
        orbit.position_at(at_ut + 0.5, reference_frame),
        orbit.position_at(at_ut - 0.5, reference_frame),
    )
    return velocity_vector


def prograde_vector_at_ut(
    orbit: Orbit, at_ut: float, reference_frame: ReferenceFrame = None
):
    """Return prograde vector of orbit at ut

    Return prograde vector of orbit at ut

    Args:
        orbit: kRPC orbit object
        at_ut: at specific time
        reference_frame: reference_frame to be used

    Returns:
        return prograde unit vector
    """
    if not reference_frame:
        attractor = orbit.body
        reference_frame = attractor.non_rotating_reference_frame

    orbit_speed = velocity_at_ut(orbit, at_ut, reference_frame)

    return unit_vector(orbit_speed)


def anti_radial_vector_at_ut(
    orbit: Orbit, at_ut: float, reference_frame: ReferenceFrame = None
):
    """Return anti-radial vector of orbit at ut

    Return anti-radial vector of orbit at ut

    Args:
        orbit: kRPC orbit object
        at_ut: at specific time
        reference_frame: reference_frame to be used

    Returns:
        return anti-radial unit vector
    """
    if not reference_frame:
        attractor = orbit.body
        reference_frame = attractor.non_rotating_reference_frame

    prograde_vector = prograde_vector_at_ut(orbit, at_ut, reference_frame)
    upward_vector = upward_vector_at_ut(orbit, at_ut, reference_frame)
    anti_radial_vector = upward_vector - prograde_vector * dot(
        prograde_vector, upward_vector
    )

    return unit_vector(anti_radial_vector)


def normal_vector_at_ut(
    orbit: Orbit, at_ut: float, reference_frame: ReferenceFrame = None
):
    """Return normal vector of orbit at ut

    Return normal vector of orbit at ut

    Args:
        orbit: kRPC orbit object
        at_ut: at specific time
        reference_frame: reference_frame to be used

    Returns:
        return normal unit vector
    """
    if not reference_frame:
        attractor = orbit.body
        reference_frame = attractor.non_rotating_reference_frame

    prograde_vector = prograde_vector_at_ut(orbit, at_ut, reference_frame)
    anti_radial_vector = anti_radial_vector_at_ut(orbit, at_ut, reference_frame)
    normal_vector = np.cross(prograde_vector, anti_radial_vector)

    return unit_vector(normal_vector)


def upward_vector_at_ut(
    orbit: Orbit, at_ut: float, reference_frame: ReferenceFrame = None
):
    """Return upward vector of orbit at ut

    Return upward vector of orbit at ut

    Args:
        orbit: kRPC orbit object
        at_ut: at specific time
        reference_frame: reference_frame to be used

    Returns:
        return upward unit vector
    """
    if not reference_frame:
        attractor = orbit.body
        reference_frame = attractor.non_rotating_reference_frame

    upward_vector = orbit.position_at(at_ut, reference_frame)
    return unit_vector(upward_vector)


def horizontal_vector_at_ut(
    orbit: Orbit, at_ut: float, reference_frame: ReferenceFrame = None
):
    """Return horizontal vector of orbit at ut

    Return horizontal vector of orbit at ut

    Args:
        orbit: kRPC orbit object
        at_ut: at specific time
        reference_frame: reference_frame to be used

    Returns:
        return horizontal unit vector
    """
    if not reference_frame:
        attractor = orbit.body
        reference_frame = attractor.non_rotating_reference_frame

    prograde_vector = prograde_vector_at_ut(orbit, at_ut, reference_frame)
    upward_vector = upward_vector_at_ut(orbit, at_ut, reference_frame)
    horizontal_vector = prograde_vector - upward_vector * dot(
        prograde_vector, upward_vector
    )
    return unit_vector(horizontal_vector)


def circularize(conn: Client, node_ut: float):
    """Execute circularize burn

    Execute circularize burn.
    At least one of (at_apoapsis, at_periapsis, at_ut) must be specified

    Args:
        conn: kRPC connection
        node_ut: schedule burn at specific time

    Returns:
        return nothing, return when procedure finished
    """
    vessel = conn.space_center.active_vessel
    attractor = vessel.orbit.body
    reference_frame = attractor.non_rotating_reference_frame

    circularize_radius = vessel.orbit.radius_at(node_ut)

    # v = sqrt(GM/r)
    circular_orbit_speed = math.sqrt(
        attractor.gravitational_parameter / circularize_radius
    )

    actual_orbit_speed = velocity_at_ut(vessel.orbit, node_ut, reference_frame)
    prograde_vector_at_node = prograde_vector_at_ut(
        vessel.orbit, node_ut, reference_frame
    )
    anti_radial_vector_at_node = anti_radial_vector_at_ut(
        vessel.orbit, node_ut, reference_frame
    )
    horizontal_vector_at_node = horizontal_vector_at_ut(
        vessel.orbit, node_ut, reference_frame
    )
    desired_orbit_speed = horizontal_vector_at_node * circular_orbit_speed

    dv_vector = desired_orbit_speed - actual_orbit_speed
    dv_prograde = dot(dv_vector, prograde_vector_at_node) * norm(dv_vector)
    dv_anti_radial = dot(dv_vector, anti_radial_vector_at_node) * norm(
        dv_vector
    )
    vessel.control.add_node(
        node_ut, radial=dv_anti_radial, prograde=dv_prograde, normal=0
    )

    # TODO: replace this logic to burn for dynamic circulize?
    # instead of just executing node, dynamically update direction for circulize
    execute_next_node(conn)


def change_apoapsis(conn: Client, node_ut: float, new_apoapsis_alt: float):
    """Execute to change apoapsis

    Execute to change apoapsis.
    To be simple, burn only prograde/retrograde, so it might be not optimal

    Args:
        conn: kRPC connection
        new_apoapsis_alt: new apoapsis altitude
        node_ut: schedule burn at specific time, if not specified burn at next periapsis

    Returns:
        return nothing, return when procedure finished
    """
    vessel = conn.space_center.active_vessel
    attractor = vessel.orbit.body
    reference_frame = attractor.non_rotating_reference_frame

    attractor_surface_radius = (
        vessel.orbit.apoapsis - vessel.orbit.apoapsis_altitude
    )
    new_apoapsis = new_apoapsis_alt + attractor_surface_radius

    if new_apoapsis <= vessel.orbit.periapsis:
        return

    krpc_bodies, poliastro_bodies = krpc_poliastro_bodies(conn)

    ut = conn.space_center.ut

    time_to_burn = node_ut - ut
    is_raising = new_apoapsis > vessel.orbit.apoapsis

    prograde_vector_at_node = prograde_vector_at_ut(vessel.orbit, node_ut)
    burn_direction = 1 if is_raising else -1
    burn_vector = burn_direction * prograde_vector_at_node

    r_i = vessel.position(reference_frame) * AstropyUnit.m
    v_i = vessel.velocity(reference_frame) * AstropyUnit.m / AstropyUnit.s
    ss_i = PoliastroOrbit.from_vectors(
        poliastro_bodies[attractor.name], r_i, v_i
    )

    # get upper bound of deltaV
    min_dv = 0
    max_dv = 0
    if is_raising:
        max_dv = 0.25
        tmp_new_ap = vessel.orbit.apoapsis
        while tmp_new_ap < new_apoapsis:
            max_dv *= 2
            tmp_burn = max_dv * burn_vector * AstropyUnit.m / AstropyUnit.s
            tmp_maneuver = Maneuver((time_to_burn * AstropyUnit.s, tmp_burn))
            tmp_new_ss = ss_i.apply_maneuver(tmp_maneuver)
            tmp_new_ap = abs(tmp_new_ss.r_a.to(AstropyUnit.m).value)
            if max_dv > 100000:
                break
    else:
        # orbital speed should be max_dv for lowering apoapsis
        max_dv = norm(velocity_at_ut(vessel.orbit, node_ut, reference_frame))

    # binary search
    while max_dv - min_dv > 0.01:
        tmp_dv = (max_dv + min_dv) / 2.0
        tmp_burn = tmp_dv * burn_vector * AstropyUnit.m / AstropyUnit.s
        tmp_maneuver = Maneuver((time_to_burn * AstropyUnit.s, tmp_burn))
        tmp_new_ss = ss_i.apply_maneuver(tmp_maneuver)
        tmp_new_ap = abs(tmp_new_ss.r_a.to(AstropyUnit.m).value)

        if (is_raising and tmp_new_ap > new_apoapsis) or (
            not is_raising and tmp_new_ap < new_apoapsis
        ):
            max_dv = tmp_dv
        else:
            min_dv = tmp_dv

    dv = burn_direction * (max_dv + min_dv) / 2.0
    vessel.control.add_node(node_ut, prograde=dv)

    # TODO: replace this logic to burn for dynamic change apoapsis?
    # instead of just executing node, dynamically update direction
    execute_next_node(conn)


def change_periapsis(conn: Client, node_ut: float, new_periapsis_alt: float):
    """Execute to change periapsis

    Execute to change periapsis.
    To be simple, burn only horizontal, so it might be not optimal

    Args:
        conn: kRPC connection
        new_periapsis_alt: new periapsis altitude
        node_ut: schedule burn at specific time, if not specified burn at next apoapsis

    Returns:
        return nothing, return when procedure finished
    """
    vessel = conn.space_center.active_vessel
    attractor = vessel.orbit.body
    reference_frame = attractor.non_rotating_reference_frame

    attractor_surface_radius = (
        vessel.orbit.apoapsis - vessel.orbit.apoapsis_altitude
    )
    new_periapsis = new_periapsis_alt + attractor_surface_radius

    if not vessel.orbit.apoapsis < 0 and new_periapsis >= vessel.orbit.apoapsis:
        return

    krpc_bodies, poliastro_bodies = krpc_poliastro_bodies(conn)

    ut = conn.space_center.ut

    time_to_burn = node_ut - ut
    is_raising = new_periapsis > vessel.orbit.periapsis

    prograde_vector_at_node = prograde_vector_at_ut(
        vessel.orbit, node_ut, reference_frame
    )
    anti_radial_vector_at_node = anti_radial_vector_at_ut(
        vessel.orbit, node_ut, reference_frame
    )
    normal_vector_at_node = normal_vector_at_ut(
        vessel.orbit, node_ut, reference_frame
    )
    horizontal_vector_at_node = horizontal_vector_at_ut(
        vessel.orbit, node_ut, reference_frame
    )
    burn_direction = 1 if is_raising else -1
    burn_vector = burn_direction * horizontal_vector_at_node

    r_i = vessel.position(reference_frame) * AstropyUnit.m
    v_i = vessel.velocity(reference_frame) * AstropyUnit.m / AstropyUnit.s
    ss_i = PoliastroOrbit.from_vectors(
        poliastro_bodies[attractor.name], r_i, v_i
    )

    # get upper bound of deltaV
    min_dv = 0
    max_dv = 0
    if is_raising:
        max_dv = 0.25
        tmp_new_pe = vessel.orbit.periapsis
        while tmp_new_pe < new_periapsis:
            max_dv *= 2
            tmp_burn = max_dv * burn_vector * AstropyUnit.m / AstropyUnit.s
            tmp_maneuver = Maneuver((time_to_burn * AstropyUnit.s, tmp_burn))
            tmp_new_ss = ss_i.apply_maneuver(tmp_maneuver)
            tmp_new_pe = abs(tmp_new_ss.r_p.to(AstropyUnit.m).value)
            if max_dv > 100000:
                break
    else:
        # horizontal speed should be max_dv for lowering periapsis
        max_dv = vessel.flight(reference_frame).horizontal_speed

    # binary search
    while max_dv - min_dv > 0.01:
        tmp_dv = (max_dv + min_dv) / 2.0
        tmp_burn = tmp_dv * burn_vector * AstropyUnit.m / AstropyUnit.s
        tmp_maneuver = Maneuver((time_to_burn * AstropyUnit.s, tmp_burn))
        tmp_new_ss = ss_i.apply_maneuver(tmp_maneuver)
        tmp_new_pe = abs(tmp_new_ss.r_p.to(AstropyUnit.m).value)

        if (is_raising and tmp_new_pe > new_periapsis) or (
            not is_raising and tmp_new_pe < new_periapsis
        ):
            max_dv = tmp_dv
        else:
            min_dv = tmp_dv

    dv_vector = burn_vector * (max_dv + min_dv) / 2.0

    dv_prograde = dot(dv_vector, prograde_vector_at_node) * norm(dv_vector)
    dv_anti_radial = dot(dv_vector, anti_radial_vector_at_node) * norm(
        dv_vector
    )
    dv_normal = dot(dv_vector, normal_vector_at_node) * norm(dv_vector)

    vessel.control.add_node(
        node_ut, prograde=dv_prograde, radial=dv_anti_radial, normal=dv_normal
    )

    # TODO: replace this logic to burn for dynamic change periapsis?
    # instead of just executing node, dynamically update direction
    execute_next_node(conn)


def match_plane_with_target(conn: Client):
    """match plane with target

    Extended description here

    Args:
        conn: kRPC connection

    Returns:
        return nothing, return when procedure finished
    """
    # Set up dialog
    dialog = StatusDialog(conn)

    # check target
    vessel = conn.space_center.active_vessel
    target = conn.space_center.target_vessel
    if not target:
        target = conn.space_center.target_body
    if not target:
        return

    v_orbit = vessel.orbit
    t_orbit = target.orbit

    # find sooner timing
    ut_an = v_orbit.ut_at_true_anomaly(v_orbit.true_anomaly_at_an(t_orbit))
    ut_dn = v_orbit.ut_at_true_anomaly(v_orbit.true_anomaly_at_dn(t_orbit))
    if ut_an < ut_dn:
        ascending = True
        node_ut = ut_an
    else:
        ascending = False
        node_ut = ut_dn

    # calculate plane change burn
    speed_at_node = v_orbit.orbital_speed_at(node_ut)
    rel_inc = v_orbit.relative_inclination(t_orbit)
    normal = speed_at_node * math.sin(rel_inc)
    prograde = speed_at_node * math.cos(rel_inc) - speed_at_node
    if ascending:
        normal *= -1

    node_desc = "ascending node" if ascending else "decending node"
    dialog.status_update(
        f"Match plane with {target.name} on {node_desc} (ut: {node_ut: .2f})"
    )

    vessel.control.add_node(node_ut, normal=normal, prograde=prograde)
    execute_next_node(conn)


if __name__ == "__main__":
    import os
    import krpc

    krpc_address = os.environ["KRPC_ADDRESS"]
    conn = krpc.connect(name="maneuver", address=krpc_address)
    circularize(
        conn,
        conn.space_center.ut
        + conn.space_center.active_vessel.orbit.time_to_apoapsis,
    )
    # circularize(conn, conn.space_center.ut + 300)
    # change_periapsis(conn, conn.space_center.ut + 300, 30000)
    # change_apoapsis(conn, conn.space_center.ut + conn.space_center.active_vessel.orbit.time_to_periapsis, 2863000)
    # circularize(conn, conn.space_center.ut + conn.space_center.active_vessel.orbit.time_to_apoapsis)
