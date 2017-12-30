from krpc.client import Client

# TODO: type hint for kRPC remote objects may need to be separated
from typing import Union, NewType

Vessel = NewType("Vessel", object)
Body = NewType("Body", object)

from astropy import units as u
from astropy.constants import Constant
from poliastro.bodies import Body as PoliastroBody
from poliastro.twobody import Orbit as PoliastroOrbit
from poliastro.maneuver import Maneuver

def _convert_body_krpc_to_poliastro(poliastro_bodies: dict, parent: PoliastroBody, krpc_body: Body):
    name = krpc_body.name
    GM = Constant('GM_k{}'.format(name), 'Kerbal {} gravitational constant'.format(name),
                  krpc_body.gravitational_parameter, 'm3 / (s2)', 0,
                  'kRPC space_center.bodies["{}"].gravitational_parameter'.format(name), system='si')
    R = Constant('R_k{}'.format(name), 'Kerbal {} equatorial radius'.format(name),
                      krpc_body.equatorial_radius, 'm', 0,
                     'kRPC space_center.bodies["{}"].equatorial_radius'.format(name), system='si')
    poliastro_body = PoliastroBody(parent, GM, "", name, R)
    poliastro_bodies[name] = poliastro_body
    for satelite in krpc_body.satellites:
        _convert_body_krpc_to_poliastro(poliastro_bodies, poliastro_body, satelite)
    return

def krpc_poliastro_bodies(conn: Client) -> (dict, dict):
    poliastro_bodies = {}
    krpc_bodies = conn.space_center.bodies

    krpc_Sun = krpc_bodies["Sun"]
    _convert_body_krpc_to_poliastro(poliastro_bodies, None, krpc_Sun)
    return (krpc_bodies, poliastro_bodies)



