import time
import krpc
conn = krpc.connect(name='Sub-orbital flight')

target_alt_min = 36000
target_alt_max = 40000
target_spd_min = 230.0
target_spd_max = 300.0


vessel = conn.space_center.active_vessel
obt_frame = vessel.orbit.body.non_rotating_reference_frame
srf_frame = vessel.orbit.body.reference_frame

alt = conn.add_stream(getattr, vessel.flight(srf_frame), 'mean_altitude')
spd = conn.add_stream(getattr, vessel.flight(srf_frame), 'speed')

vessel.auto_pilot.target_pitch_and_heading(90, 90)
vessel.auto_pilot.engage()
vessel.control.throttle = 0
time.sleep(1)

print('Launch!')
vessel.control.activate_next_stage()


while True:
    t = (target_spd_max - spd()) / (target_spd_max - target_spd_min)
    vessel.control.throttle = min(1, max(0, t))

    if ((alt() > target_alt_min and alt() < target_alt_max) and
            (spd() > target_spd_min and spd() < target_alt_max)):
        print('Test Condition Met')
        break

print('Test Start!')
vessel.control.activate_next_stage()

srf_altitude = conn.get_call(getattr, vessel.flight(), 'surface_altitude')
expr = conn.krpc.Expression.less_than(
    conn.krpc.Expression.call(srf_altitude),
    conn.krpc.Expression.constant_double(2000))
event = conn.krpc.add_event(expr)
with event.condition:
    event.wait()

print('deploy Chute!')
vessel.control.activate_next_stage()

while vessel.flight(vessel.orbit.body.reference_frame).vertical_speed < -0.1:
    print('Altitude = %.1f meters' % vessel.flight().surface_altitude)
    time.sleep(1)
print('Landed!')
