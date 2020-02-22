from krpc.client import Client
from scripts.utils.status_dialog import StatusDialog


is_autostaging = True
staging_event = None


def set_autostaging(
    conn: Client,
    liquid_fuel: bool = True,
    oxidizer: bool = True,
    solid_fuel: bool = True,
    threashold: float = 0,
    stop_stage: int = 0,
) -> None:
    """set next autostagin on resource is under certin level

    Extended description of function.

    Args:
        conn: kRPC connection

    Returns:
        return nothing, return when procedure finished

    """
    # TODO: global without lock
    global is_autostaging
    is_autostaging = True

    dialog = StatusDialog(conn)
    vessel = conn.space_center.active_vessel

    current_stage = vessel.control.current_stage
    if current_stage <= stop_stage:
        return

    resources_in_next_decoupled_stage = vessel.resources_in_decouple_stage(
        current_stage - 1
    )

    # TODO: offset must take account unused (undrained) resource, not only SF, that could be LF and/or O also?
    # ...or, completely replace staging logic like: just check all engines in decoupled stage and stage when all active engine doens't have fuel
    # downside of this logic, is we cannot take fuel threashold other than 0, that make powered recovery impossible
    resource_types_for_stage = []
    if liquid_fuel and resources_in_next_decoupled_stage.has_resource(
        "LiquidFuel"
    ):
        resource_types_for_stage.append("LiquidFuel")
    if oxidizer and resources_in_next_decoupled_stage.has_resource("Oxidizer"):
        resource_types_for_stage.append("Oxidizer")
    if solid_fuel and resources_in_next_decoupled_stage.has_resource(
        "SolidFuel"
    ):
        # check solid fuel only if there's active srbs decoupled next
        srbs_next_decoupled = [
            e
            for e in vessel.parts.engines
            if e.part.resources.has_resource("SolidFuel")
            and e.part.decouple_stage == (current_stage - 1)
        ]
        if len([e for e in srbs_next_decoupled if e.active]) > 0:
            # calculate solid fuel offset, fuels in non active srbs decoupled next (separetron)
            solidfuel_unused_decoupled_in_next_stage = sum(
                [
                    e.part.resources.amount("SolidFuel")
                    for e in srbs_next_decoupled
                    if not e.active
                ]
            )
            resource_types_for_stage.append("SolidFuel")
    if len(resource_types_for_stage) == 0:
        # if current stage has empty resource (fairing etc.) just stage
        vessel.control.activate_next_stage()
        set_autostaging(
            conn, liquid_fuel, oxidizer, solid_fuel, threashold, stop_stage
        )
        return

    expression = conn.krpc.Expression

    call_current_stage = conn.get_call(getattr, vessel.control, "current_stage")
    staging_condition = expression.not_equal(
        expression.call(call_current_stage),
        expression.constant_int(current_stage),
    )

    for resource in resource_types_for_stage:
        resource_threashold = threashold
        if resource == "SolidFuel":
            resource_threashold += solidfuel_unused_decoupled_in_next_stage
        resource_amount_call = conn.get_call(
            resources_in_next_decoupled_stage.amount, resource
        )
        cond = expression.less_than_or_equal(
            conn.krpc.Expression.call(resource_amount_call),
            conn.krpc.Expression.constant_float(resource_threashold),
        )
        staging_condition = expression.or_(staging_condition, cond)

    global staging_event
    staging_event = conn.krpc.add_event(staging_condition)

    def auto_staging():
        global is_autostaging
        remove_staging_events()
        if not is_autostaging:
            return
        dialog.status_update("Staging: stage {}".format(current_stage - 1))

        # check if stage triggered by somewhere else
        if current_stage == vessel.control.current_stage:
            vessel.control.activate_next_stage()
        set_autostaging(
            conn, liquid_fuel, oxidizer, solid_fuel, threashold, stop_stage
        )

    staging_event.add_callback(auto_staging)
    staging_event.start()


def remove_staging_events():
    global staging_event
    if staging_event:
        staging_event.remove()
        staging_event = None


# TODO: global without lock
def unset_autostaging():
    global is_autostaging
    is_autostaging = False
    remove_staging_events()


if __name__ == "__main__":
    import os
    import krpc

    krpc_address = os.environ["KRPC_ADDRESS"]
    conn = krpc.connect(name="autostage", address=krpc_address)
    set_autostaging(conn)

    print("start sleeping")
    while True:
        pass
