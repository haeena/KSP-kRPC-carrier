from krpc.client import Client
from typing import Union, NewType
Vessel = NewType("Vessel", object)
Body = NewType("Body", object)

from astropy import units as AstropyUnit
from astropy.constants import Constant as AstropyConstant
from poliastro.bodies import Body as PoliastroBody
from poliastro.maneuver import Maneuver

KRPC_BODIES = None
POLIASTRO_BODIES = None

def _convert_body_krpc_to_poliastro(poliastro_bodies: dict, parent: PoliastroBody, krpc_body: Body):
    name = krpc_body.name
    GM = AstropyConstant('GM_k{}'.format(name), 'Kerbal {} gravitational constant'.format(name),
                  krpc_body.gravitational_parameter, 'm3 / (s2)', 0,
                  'kRPC space_center.bodies["{}"].gravitational_parameter'.format(name), system='si')
    R = AstropyConstant('R_k{}'.format(name), 'Kerbal {} equatorial radius'.format(name),
                      krpc_body.equatorial_radius, 'm', 0,
                     'kRPC space_center.bodies["{}"].equatorial_radius'.format(name), system='si')
    poliastro_body = PoliastroBody(parent, GM, "", name, R)
    poliastro_bodies[name] = poliastro_body
    for satelite in krpc_body.satellites:
        _convert_body_krpc_to_poliastro(poliastro_bodies, poliastro_body, satelite)
    return

def krpc_poliastro_bodies(conn: Client) -> (dict, dict):
    global KRPC_BODIES, POLIASTRO_BODIES
    if not POLIASTRO_BODIES:
        import krpc
        with krpc.connect(name='convert krpc bodies to poliastro bodies') as conn:
            poliastro_bodies = {}
            krpc_bodies = conn.space_center.bodies

            krpc_Sun = krpc_bodies["Sun"]
            _convert_body_krpc_to_poliastro(poliastro_bodies, None, krpc_Sun)
            KRPC_BODIES = krpc_bodies
            POLIASTRO_BODIES = poliastro_bodies

    return (KRPC_BODIES, POLIASTRO_BODIES)

