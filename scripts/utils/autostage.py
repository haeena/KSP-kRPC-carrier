import time
import math
from functools import reduce
from krpc.client import Client
from scripts.utils.status_dialog import StatusDialog

is_autostaging = True

def set_autostaging(conn: Client,
                    liquid_fuel: bool = True,
                    oxidizer: bool = True,
                    solid_fuel: bool = True,
                    threashold: float = 1,
                    stop_stage: int = 0) -> None:
    """set next autostagin on resource is under certin level

    Extended description of function.

    Args:
        conn: kRPC connection

    Returns:
        return nothing, return when procedure finished

    """
    dialog = StatusDialog(conn)
    vessel = conn.space_center.active_vessel

    current_stage = reduce(lambda x, y: max(x, y.decouple_stage, y.stage), vessel.parts.all, 0)
    if current_stage <= stop_stage:
        return

    resource_types_for_stage = []
    if liquid_fuel:
        resource_types_for_stage.append("LiquidFuel")
    if oxidizer:
        resource_types_for_stage.append("Oxidizer")
    if solid_fuel:
        resource_types_for_stage.append("SolidFuel")
    if len(resource_types_for_stage) == 0:
        return

    resources_of_stage = vessel.resources_in_decouple_stage(current_stage - 1)
    # not understanding why this list conversion required, but at least we cannot iterate over map
    resources_calls = list(map(lambda r:conn.get_call(resources_of_stage.amount, r), resource_types_for_stage))

    expression = conn.krpc.Expression

    first_cond = True
    for resources_call in resources_calls:
        cond = expression.less_than(
                conn.krpc.Expression.call(resources_call),
                conn.krpc.Expression.constant_float(threashold)
            )
        if first_cond:
            first_cond = False
            staging_condition = cond
        else:
            staging_condition = expression.or_(staging_condition, cond)
    
    staging_event = conn.krpc.add_event(staging_condition)

    # TODO: global without lock
    def auto_staging():
        global is_autostaging
        staging_event.remove()
        if not is_autostaging:
            return
        dialog.status_update("Staging: stage {}".format(current_stage - 1))

        vessel.control.activate_next_stage()
        set_autostaging(conn, liquid_fuel, oxidizer, solid_fuel, threashold, stop_stage)
        
    staging_event.add_callback(auto_staging)
    staging_event.start()

# TODO: global without lock
def unset_autostaging():
    global is_autostaging
    is_autostaging = False

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

if __name__ == "__main__":
    import os
    import krpc
    krpc_address = os.environ["KRPC_ADDRESS"]
    conn = krpc.connect(name='autostage', address=krpc_address)
    set_autostaging(conn)

    print("start sleeping")
    while True:
        pass