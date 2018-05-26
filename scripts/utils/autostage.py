import time
import math
from functools import reduce
from krpc.client import Client
from scripts.utils.status_dialog import StatusDialog

def autostage(conn: Client,
              liquid_fuel: bool = True, oxidizer: bool = True,
              solid_fuel: bool = True) -> None:
    """autostage if current stage is empty

    Extended description of function.

    Args:
        conn: kRPC connection

    Returns:
        return nothing, return when procedure finished

    """
    vessel = conn.space_center.active_vessel
    dialog = StatusDialog(conn)

    resource_types_for_stage = []
    if liquid_fuel:
        resource_types_for_stage.append("LiquidFuel")
    if oxidizer:
        resource_types_for_stage.append("Oxidizer")
    if solid_fuel:
        resource_types_for_stage.append("SolidFuel")

    ## TODO: this logic should be improved
    # Staging when fuel empty
    current_stage = reduce(lambda x, y: max(x, y.decouple_stage, y.stage), vessel.parts.all, 0)

    resources_of_stage = vessel.resources_in_decouple_stage(current_stage - 1)
    max_resource = max(map(lambda r: resources_of_stage.amount(r), resource_types_for_stage))

    if max_resource <= 0.1:
        dialog.status_update("Staging: stage {}".format(current_stage - 1))
        vessel.control.activate_next_stage()