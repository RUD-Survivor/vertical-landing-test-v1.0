import os
import re

def process_camera_director():
    path = r'c:\antigravity_code\RocketSim3D\src\camera\camera_director.h'
    with open(path, 'r', encoding='utf-8') as f:
        content = f.read()

    # Change signature
    old_sig = 'CameraResult computeFlightCamera(\n        const RocketState& rocket_state,'
    new_sig = 'CameraResult computeFlightCamera(\n        const TransformComponent& trans,\n        const VelocityComponent& vel,'
    content = content.replace(old_sig, new_sig)

    # Replace usages
    content = content.replace('rocket_state.abs_px', 'trans.abs_px')
    content = content.replace('rocket_state.abs_py', 'trans.abs_py')
    content = content.replace('rocket_state.abs_pz', 'trans.abs_pz')
    content = content.replace('rocket_state.px', 'trans.px')
    content = content.replace('rocket_state.py', 'trans.py')
    content = content.replace('rocket_state.pz', 'trans.pz')
    content = content.replace('rocket_state.vx', 'vel.vx')
    content = content.replace('rocket_state.vy', 'vel.vy')
    content = content.replace('rocket_state.vz', 'vel.vz')

    with open(path, 'w', encoding='utf-8') as f:
        f.write(content)

def process_render_context():
    path = r'c:\antigravity_code\RocketSim3D\src\scene\render_context.h'
    with open(path, 'r', encoding='utf-8') as f:
        content = f.read()

    # Update signature
    old_sig = 'void update(RocketState& rocket_state, const RocketConfig& rocket_config,'
    new_sig = 'void update(TransformComponent& trans, VelocityComponent& vel, AttitudeComponent& att, TelemetryComponent& tele, const RocketConfig& rocket_config,'
    content = content.replace(old_sig, new_sig)
    
    # Replace usages
    content = content.replace('rocket_state.abs_px', 'trans.abs_px')
    content = content.replace('rocket_state.abs_py', 'trans.abs_py')
    content = content.replace('rocket_state.abs_pz', 'trans.abs_pz')
    content = content.replace('rocket_state.px', 'trans.px')
    content = content.replace('rocket_state.py', 'trans.py')
    content = content.replace('rocket_state.pz', 'trans.pz')
    content = content.replace('rocket_state.vx', 'vel.vx')
    content = content.replace('rocket_state.vy', 'vel.vy')
    content = content.replace('rocket_state.vz', 'vel.vz')
    
    content = content.replace('rocket_state.altitude', 'tele.altitude')
    content = content.replace('rocket_state.terrain_altitude', 'tele.terrain_altitude')
    
    content = content.replace('rocket_state.attitude_initialized', 'att.initialized')
    content = content.replace('rocket_state.attitude', 'att.attitude')
    content = content.replace('rocket_state.angle_roll', 'att.angle_roll')
    content = content.replace('rocket_state.angle_z', 'att.angle_z')
    content = content.replace('rocket_state.angle', 'att.angle')
    
    content = content.replace('rocket_state.status', 'guid.status') # wait, guid is not in signature. Let's add it.
    
    # Update signature again to include guid
    new_sig2 = 'void update(TransformComponent& trans, VelocityComponent& vel, AttitudeComponent& att, TelemetryComponent& tele, GuidanceComponent& guid, const RocketConfig& rocket_config,'
    content = content.replace(new_sig, new_sig2)
    
    # fix call to cam.computeFlightCamera
    content = content.replace('computeFlightCamera(\n            rocket_state', 'computeFlightCamera(\n            trans, vel')
    content = content.replace('computeFlightCamera(\n        rocket_state', 'computeFlightCamera(\n        trans, vel')
    # Maybe it's single line
    content = re.sub(r'computeFlightCamera\(\s*rocket_state\s*,', 'computeFlightCamera(trans, vel,', content)

    with open(path, 'w', encoding='utf-8') as f:
        f.write(content)

process_camera_director()
process_render_context()

print("Done")
