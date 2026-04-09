import re

# ============================================================
# Phase 3: Fix ALL remaining legacy references
# ============================================================

def read(path):
    with open(path, 'r', encoding='utf-8') as f: return f.read()
def write(path, data):
    with open(path, 'w', encoding='utf-8') as f: f.write(data)

# ============================================================
# 1. control_system.cpp — Fix function signatures & state.xxx
# ============================================================
CS = r'c:\antigravity_code\RocketSim3D\src\control\control_system.cpp'
cs = read(CS)

# Fix UpdateAutoPilot definition signature
cs = cs.replace(
    'void UpdateAutoPilot(RocketState& state, const RocketConfig& config, ControlInput& input, double dt) {',
    '''void UpdateAutoPilot(entt::registry& registry, entt::entity entity, double dt) {
    auto& state = registry.get<RocketState>(entity);
    auto& config = registry.get<RocketConfig>(entity);
    auto& input = registry.get<ControlInput>(entity);
    auto& trans = registry.get<TransformComponent>(entity);
    auto& vel = registry.get<VelocityComponent>(entity);
    auto& att = registry.get<AttitudeComponent>(entity);
    auto& prop = registry.get<PropulsionComponent>(entity);
    auto& tele = registry.get<TelemetryComponent>(entity);
    auto& guid = registry.get<GuidanceComponent>(entity);
    auto& mnv = registry.get<ManeuverComponent>(entity);''')

# Fix UpdateManualControl definition signature
cs = cs.replace(
    'void UpdateManualControl(RocketState& state, const RocketConfig& config, ControlInput& input, const ManualInputs& manual, double dt) {',
    '''void UpdateManualControl(entt::registry& registry, entt::entity entity, const ManualInputs& manual, double dt) {
    auto& state = registry.get<RocketState>(entity);
    auto& config = registry.get<RocketConfig>(entity);
    auto& input = registry.get<ControlInput>(entity);
    auto& att = registry.get<AttitudeComponent>(entity);
    auto& guid = registry.get<GuidanceComponent>(entity);
    auto& prop = registry.get<PropulsionComponent>(entity);
    auto& mnv = registry.get<ManeuverComponent>(entity);
    auto& trans = registry.get<TransformComponent>(entity);
    auto& vel = registry.get<VelocityComponent>(entity);
    auto& tele = registry.get<TelemetryComponent>(entity);''')

# Fix getOrbitParams call (the only one in control_system.cpp)
cs = cs.replace('PhysicsSystem::getOrbitParams(state, apo, peri);',
                'PhysicsSystem::getOrbitParams(registry, entity, apo, peri);')

# Replace remaining state.xxx with correct component references
# Sort by length desc to prevent partial matches
state_map = [
    ('state.suicide_burn_locked', 'guid.suicide_burn_locked'),
    ('state.attitude_initialized', 'att.initialized'),
    ('state.mission_phase', 'guid.mission_phase'),
    ('state.mission_timer', 'guid.mission_timer'),
    ('state.sas_target_vec', 'guid.sas_target_vec'),
    ('state.ang_vel_roll', 'att.ang_vel_roll'),
    ('state.ang_vel_z', 'att.ang_vel_z'),
    ('state.pid_att_z', 'guid.pid_att_z'),
    ('state.pid_att_roll', 'guid.pid_att_roll'),
    ('state.pid_att', 'guid.pid_att'),
    ('state.pid_vert', 'guid.pid_vert'),
    ('state.pid_pos', 'guid.pid_pos'),
    ('state.local_vx', 'tele.local_vx'),
    ('state.ang_vel', 'att.ang_vel'),
    ('state.attitude', 'att.attitude'),
    ('state.angle_z', 'att.angle_z'),
    ('state.angle', 'att.angle'),
]
for old, new in state_map:
    cs = cs.replace(old, new)

# Add entt include
if '<entt/entt.hpp>' not in cs:
    cs = '#include <entt/entt.hpp>\n' + cs

write(CS, cs)
print('[OK] control_system.cpp fixed')

# ============================================================
# 2. control_system.h — Fix function declarations
# ============================================================
CSH = r'c:\antigravity_code\RocketSim3D\src\control\control_system.h'
csh = read(CSH)

# Make sure signatures are updated (might already be done by phase 2 script)
csh = csh.replace(
    'static void UpdateManualControl(RocketState& state, const RocketConfig& config, ControlInput& input, const ManualInputs& manual, double dt);',
    'static void UpdateManualControl(entt::registry& registry, entt::entity entity, const ManualInputs& manual, double dt);')
csh = csh.replace(
    'static void UpdateAutoPilot(RocketState& state, const RocketConfig& config, ControlInput& input, double dt);',
    'static void UpdateAutoPilot(entt::registry& registry, entt::entity entity, double dt);')

if '<entt/entt.hpp>' not in csh:
    csh = csh.replace('#include "core/rocket_state.h"', '#include "core/rocket_state.h"\n#include <entt/entt.hpp>')

write(CSH, csh)
print('[OK] control_system.h fixed')

# ============================================================
# 3. camera_director.h — Fix getOrbitParams call
# ============================================================
CD = r'c:\antigravity_code\RocketSim3D\src\camera\camera_director.h'
cd = read(CD)
cd = cd.replace('PhysicsSystem::getOrbitParams(rocket_state, apo_tmp, peri_tmp);',
                '// NOTE: getOrbitParams now needs registry & entity which CameraDirector does not have.\n'
                '// For now, compute orbit params inline:\n'
                '{\n'
                '    double r_cam = std::sqrt(rocket_state.px*rocket_state.px + rocket_state.py*rocket_state.py + rocket_state.pz*rocket_state.pz);\n'
                '    double v_sq_cam = rocket_state.vx*rocket_state.vx + rocket_state.vy*rocket_state.vy + rocket_state.vz*rocket_state.vz;\n'
                '    double mu_cam = 6.67430e-11 * SOLAR_SYSTEM[current_soi_index].mass;\n'
                '    double energy_cam = v_sq_cam / 2.0 - mu_cam / r_cam;\n'
                '    double hx_cam = rocket_state.py*rocket_state.vz - rocket_state.pz*rocket_state.vy;\n'
                '    double hy_cam = rocket_state.pz*rocket_state.vx - rocket_state.px*rocket_state.vz;\n'
                '    double hz_cam = rocket_state.px*rocket_state.vy - rocket_state.py*rocket_state.vx;\n'
                '    double h_sq_cam = hx_cam*hx_cam + hy_cam*hy_cam + hz_cam*hz_cam;\n'
                '    double e_sq_cam = 1.0 + 2.0 * energy_cam * h_sq_cam / (mu_cam * mu_cam);\n'
                '    double e_cam = (e_sq_cam > 0) ? std::sqrt(e_sq_cam) : 0;\n'
                '    if (energy_cam >= 0) { apo_tmp = 999999999; peri_tmp = (h_sq_cam/mu_cam)/(1.0+e_cam) - SOLAR_SYSTEM[current_soi_index].radius; }\n'
                '    else { double a_cam = -mu_cam/(2.0*energy_cam); apo_tmp = a_cam*(1.0+e_cam) - SOLAR_SYSTEM[current_soi_index].radius; peri_tmp = a_cam*(1.0-e_cam) - SOLAR_SYSTEM[current_soi_index].radius; }\n'
                '}')
write(CD, cd)
print('[OK] camera_director.h fixed')

# ============================================================
# 4. flight_input_system.h — Fix SeparateStage call
#    This is tricky because it's inside a lambda that captures rocket_state & rocket_config
#    We need to restructure the lambda to capture world & entity
# ============================================================
FIS = r'c:\antigravity_code\RocketSim3D\src\scene\flight_input_system.h'
fis = read(FIS)

# The lambda on line 27 captures [&rocket_state, &rocket_config] and calls SeparateStage
# We need to change it to be compatible. Since this is a lambda inside setup(),
# we need to pass world & entity to setup() and capture them.

# First, update the setup signature to also accept world and entity
fis = fis.replace(
    'void setup(InputRouter& router, RocketState& rocket_state, RocketConfig& rocket_config, \n               ControlInput& control_input, FlightHUD& hud, CameraDirector& cam, bool& show_clouds) {',
    'void setup(InputRouter& router, RocketState& rocket_state, RocketConfig& rocket_config, \n               ControlInput& control_input, FlightHUD& hud, CameraDirector& cam, bool& show_clouds,\n               entt::registry& world, entt::entity rocket_entity) {')

# Fix the SeparateStage lambda
fis = fis.replace(
    'router.registerKey(GLFW_KEY_G, [&rocket_state, &rocket_config]() {\n            if (rocket_state.status == ASCEND || rocket_state.status == DESCEND) {\n                StageManager::SeparateStage(rocket_state, rocket_config);\n            }\n        });',
    'router.registerKey(GLFW_KEY_G, [&rocket_state, &world, &rocket_entity]() {\n            if (rocket_state.status == ASCEND || rocket_state.status == DESCEND) {\n                StageManager::SeparateStage(world, rocket_entity);\n            }\n        });')

write(FIS, fis)
print('[OK] flight_input_system.h fixed')

# ============================================================
# 5. simulation_controller.h — Fix remaining rocket_state refs & add att component
# ============================================================
SC = r'c:\antigravity_code\RocketSim3D\src\simulation\simulation_controller.h'
sc = read(SC)

# Add AttitudeComponent fetch in update() (it references att.ang_vel)
sc = sc.replace(
    "        auto& vfx = registry.get<VFXComponent>(entity);\n\n        // Accumulate time",
    "        auto& vfx = registry.get<VFXComponent>(entity);\n        auto& att = registry.get<AttitudeComponent>(entity);\n\n        // Accumulate time")

# Add vel, trans, att fetch in executeManeuvers (it uses vel.vx, trans.px etc)
sc = sc.replace(
    "    auto& mnv = registry.get<ManeuverComponent>(entity);\n        mnv_autopilot_active = false;",
    "    auto& mnv = registry.get<ManeuverComponent>(entity);\n    auto& trans = registry.get<TransformComponent>(entity);\n    auto& vel = registry.get<VelocityComponent>(entity);\n    auto& att = registry.get<AttitudeComponent>(entity);\n        mnv_autopilot_active = false;")

# Replace rocket_state.attitude -> att.attitude etc in executeManeuvers
sc_map = [
    ('rocket_state.attitude', 'att.attitude'),
    ('rocket_state.ang_vel_roll', 'att.ang_vel_roll'),
    ('rocket_state.ang_vel_z', 'att.ang_vel_z'),
    ('rocket_state.ang_vel', 'att.ang_vel'),
]
for old, new in sc_map:
    sc = sc.replace(old, new)

write(SC, sc)
print('[OK] simulation_controller.h fixed')

# ============================================================
# 6. Fix flight_scene.h — Update inputSystem.setup() call
# ============================================================
FS = r'c:\antigravity_code\RocketSim3D\src\scene\flight_scene.h'
fs = read(FS)
fs = fs.replace(
    'inputSystem.setup(input, rocket_state, rocket_config, control_input, hudManager.hud, cam, show_clouds);',
    'inputSystem.setup(input, rocket_state, rocket_config, control_input, hudManager.hud, cam, show_clouds, world, rocket_entity);')
write(FS, fs)
print('[OK] flight_scene.h fixed')

# ============================================================
# 7. stage_manager.h — Check/fix IsCurrentStageEmpty signature
# ============================================================
SMH = r'c:\antigravity_code\RocketSim3D\src\simulation\stage_manager.h'
smh = read(SMH)
# The call in simulation_controller.h is now: StageManager::IsCurrentStageEmpty(prop)
# We need to make sure this function accepts PropulsionComponent (or RocketState)
# Let's see what the current signature is
if 'IsCurrentStageEmpty(const RocketState&' in smh:
    smh = smh.replace('static bool IsCurrentStageEmpty(const RocketState& state);',
                       'static bool IsCurrentStageEmpty(const RocketState& state);\n    static bool IsCurrentStageEmpty(const PropulsionComponent& prop);')

# Also check SyncActiveConfig
write(SMH, smh)
print('[OK] stage_manager.h checked')

SMCPP = r'c:\antigravity_code\RocketSim3D\src\simulation\stage_manager.cpp'
smcpp = read(SMCPP)
# Add overload for PropulsionComponent
if 'IsCurrentStageEmpty(const PropulsionComponent&' not in smcpp:
    smcpp += '''
// ECS overload
bool StageManager::IsCurrentStageEmpty(const PropulsionComponent& prop) {
    if (prop.current_stage < 0 || prop.current_stage >= (int)prop.stage_fuels.size()) return false;
    return prop.stage_fuels[prop.current_stage] <= 0.0;
}
'''
write(SMCPP, smcpp)
print('[OK] stage_manager.cpp fixed')

print('\n=== Phase 3 ECS Refactor COMPLETE ===')
