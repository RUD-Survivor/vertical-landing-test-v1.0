import re

BLOCK_ENTITY = """
        auto& trans = registry.get<TransformComponent>(entity);
        auto& vel   = registry.get<VelocityComponent>(entity);
        auto& att   = registry.get<AttitudeComponent>(entity);
        auto& prop  = registry.get<PropulsionComponent>(entity);
        auto& tele  = registry.get<TelemetryComponent>(entity);
        auto& guid  = registry.get<GuidanceComponent>(entity);
        auto& mnv   = registry.get<ManeuverComponent>(entity);
        auto& orb   = registry.get<OrbitComponent>(entity);
"""

BLOCK_ROCKET_ENTITY = """
        auto& trans = world.get<TransformComponent>(rocket_entity);
        auto& vel   = world.get<VelocityComponent>(rocket_entity);
        auto& att   = world.get<AttitudeComponent>(rocket_entity);
        auto& prop  = world.get<PropulsionComponent>(rocket_entity);
        auto& tele  = world.get<TelemetryComponent>(rocket_entity);
        auto& guid  = world.get<GuidanceComponent>(rocket_entity);
        auto& mnv   = world.get<ManeuverComponent>(rocket_entity);
        auto& orb   = world.get<OrbitComponent>(rocket_entity);
"""

def inject_after_content(path, target, block):
    with open(path, 'r', encoding='utf-8') as f:
        c = f.read()
    if target in c and block.strip() not in c:
        c = c.replace(target, target + block)
        with open(path, 'w', encoding='utf-8') as f:
            f.write(c)
            print(f"Fixed {path}")

inject_after_content(r'c:\antigravity_code\RocketSim3D\src\scene\orbit_system.h',
                     'auto& mnv = registry.get<ManeuverComponent>(entity);',
                     BLOCK_ENTITY)

inject_after_content(r'c:\antigravity_code\RocketSim3D\src\scene\hud_manager.h',
                     'hud_ctx.cam = &cam;',
                     BLOCK_ENTITY.replace('registry.get', 'registry->get').replace('(entity)', '(hud_ctx.entity)'))

# For flight_scene.h, let's fix onEnter, update, and any missing places.
path = r'c:\antigravity_code\RocketSim3D\src\scene\flight_scene.h'
with open(path, 'r', encoding='utf-8') as f:
    c = f.read()

# in update(double dt)
if 'real_dt = dt;' in c and 'auto& trans =' not in c.split('real_dt = dt;')[1][:200]:
    c = c.replace('real_dt = dt;', 'real_dt = dt;' + BLOCK_ROCKET_ENTITY)

# in onEnter
if 'auto& rocket_config = world.get<RocketConfig>(rocket_entity);' in c and 'auto& trans =' not in c:
    c = c.replace('auto& rocket_config = world.get<RocketConfig>(rocket_entity);', 
                  'auto& rocket_config = world.get<RocketConfig>(rocket_entity);' + BLOCK_ROCKET_ENTITY)

# replace any rocket_state arguments to the new signature in flight_scene
c = c.replace('ctx.update(rocket_state, rocket_config', 'ctx.update(trans, vel, att, tele, guid, rocket_config')
c = c.replace('StageManager::SyncActiveConfig(rocket_state', 'StageManager::SyncActiveConfig(prop')
c = c.replace('celestialRenderer.renderMacroPass(\n                r3d, rocket_state,', 'celestialRenderer.renderMacroPass(\n                r3d, tele,')

with open(path, 'w', encoding='utf-8') as f:
    f.write(c)

# We also had an error about StageManager::SyncActiveConfig in flight_scene.h
sm_path = r'c:\antigravity_code\RocketSim3D\src\simulation\stage_manager.h'
with open(sm_path, 'r', encoding='utf-8') as f:
    sm_c = f.read()
    sm_c = sm_c.replace('static void SyncActiveConfig(RocketState& state, RocketConfig& config) {', 'static void SyncActiveConfig(PropulsionComponent& prop, RocketConfig& config) {')
    sm_c = sm_c.replace('state.current_stage', 'prop.current_stage')
with open(sm_path, 'w', encoding='utf-8') as f:
    f.write(sm_c)

# flight_scene.h(214): error C2039: "attitude_initialized": not a member of "AttitudeComponent"
# Oh, it's called 'initialized' not 'attitude_initialized'
with open(path, 'r', encoding='utf-8') as f:
    c = f.read()
    c = c.replace('att.attitude_initialized', 'att.initialized')
with open(path, 'w', encoding='utf-8') as f:
    f.write(c)

print("Injections and fixes applied.")
