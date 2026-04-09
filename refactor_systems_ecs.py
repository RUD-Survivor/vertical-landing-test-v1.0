import os
import re

# Files to refactor
CONTROLLER_H = r'c:\antigravity_code\RocketSim3D\src\simulation\simulation_controller.h'
STAGEM_H = r'c:\antigravity_code\RocketSim3D\src\simulation\stage_manager.h'
STAGEM_CPP = r'c:\antigravity_code\RocketSim3D\src\simulation\stage_manager.cpp'
CONTROL_H = r'c:\antigravity_code\RocketSim3D\src\control\control_system.h'
CONTROL_CPP = r'c:\antigravity_code\RocketSim3D\src\control\control_system.cpp'

mapping = {
    'px': 'trans.px', 'py': 'trans.py', 'pz': 'trans.pz',
    'abs_px': 'trans.abs_px', 'abs_py': 'trans.abs_py', 'abs_pz': 'trans.abs_pz',
    'surf_px': 'trans.surf_px', 'surf_py': 'trans.surf_py', 'surf_pz': 'trans.surf_pz',
    'vx': 'vel.vx', 'vy': 'vel.vy', 'vz': 'vel.vz',
    'fuel': 'prop.fuel', 'current_stage': 'prop.current_stage', 'total_stages': 'prop.total_stages',
    'stage_fuels': 'prop.stage_fuels', 'jettisoned_mass': 'prop.jettisoned_mass',
    'thrust_power': 'prop.thrust_power',
    'sim_time': 'tele.sim_time', 'altitude': 'tele.altitude', 'velocity': 'tele.velocity',
    'status': 'guid.status', 'mission_msg': 'guid.mission_msg', 'auto_mode': 'guid.auto_mode',
    'sas_active': 'guid.sas_active', 'rcs_active': 'guid.rcs_active', 'sas_mode': 'guid.sas_mode',
    'maneuvers': 'mnv.maneuvers', 'selected_maneuver_index': 'mnv.selected_maneuver_index'
}

def replace_state_access(content):
    sorted_keys = sorted(mapping.keys(), key=len, reverse=True)
    for key in sorted_keys:
        pattern = r'\bstate\.' + re.escape(key) + r'\b'
        content = re.sub(pattern, mapping[key], content)
        # Also handle rocket_state naming
        pattern2 = r'\brocket_state\.' + re.escape(key) + r'\b'
        content = re.sub(pattern2, mapping[key], content)
    return content

# --- 1. Refactor StageManager ---
with open(STAGEM_H, 'r', encoding='utf-8') as f:
    h = f.read()
h = h.replace('static void SeparateStage(RocketState& state, RocketConfig& config);', 'static void SeparateStage(entt::registry& registry, entt::entity entity);')
with open(STAGEM_H, 'w', encoding='utf-8') as f: f.write(h)

with open(STAGEM_CPP, 'r', encoding='utf-8') as f:
    cpp = f.read()
cpp = cpp.replace('void StageManager::SeparateStage(RocketState& state, RocketConfig& config) {', 
                  'void StageManager::SeparateStage(entt::registry& registry, entt::entity entity) {\n    auto& state = registry.get<RocketState>(entity);\n    auto& config = registry.get<RocketConfig>(entity);\n    auto& prop = registry.get<PropulsionComponent>(entity);\n    auto& guid = registry.get<GuidanceComponent>(entity);')
cpp = replace_state_access(cpp)
with open(STAGEM_CPP, 'w', encoding='utf-8') as f: f.write(cpp)

# --- 2. Refactor ControlSystem ---
with open(CONTROL_H, 'r', encoding='utf-8') as f:
    h = f.read()
h = h.replace('static void UpdateManualControl(RocketState& state, const RocketConfig& config, ControlInput& input, const ManualInputs& manual, double dt);',
              'static void UpdateManualControl(entt::registry& registry, entt::entity entity, const ManualInputs& manual, double dt);')
h = h.replace('static void UpdateAutoPilot(RocketState& state, const RocketConfig& config, ControlInput& input, double dt);',
              'static void UpdateAutoPilot(entt::registry& registry, entt::entity entity, double dt);')
with open(CONTROL_H, 'w', encoding='utf-8') as f: f.write(h)

with open(CONTROL_CPP, 'r', encoding='utf-8') as f:
    cpp = f.read()
fetch = '\n    auto& state = registry.get<RocketState>(entity);\n    auto& config = registry.get<RocketConfig>(entity);\n    auto& input = registry.get<ControlInput>(entity);\n    auto& guid = registry.get<GuidanceComponent>(entity);\n    auto& prop = registry.get<PropulsionComponent>(entity);\n    auto& att = registry.get<AttitudeComponent>(entity);\n'
cpp = cpp.replace('void ControlSystem::UpdateManualControl(RocketState& state, const RocketConfig& config, ControlInput& input, const ManualInputs& manual, double dt) {',
                  'void ControlSystem::UpdateManualControl(entt::registry& registry, entt::entity entity, const ManualInputs& manual, double dt) {' + fetch)
cpp = cpp.replace('void ControlSystem::UpdateAutoPilot(RocketState& state, const RocketConfig& config, ControlInput& input, double dt) {',
                  'void ControlSystem::UpdateAutoPilot(entt::registry& registry, entt::entity entity, double dt) {' + fetch)
cpp = replace_state_access(cpp)
with open(CONTROL_CPP, 'w', encoding='utf-8') as f: f.write(cpp)

# --- 3. Refactor SimulationController::executeManeuvers ---
with open(CONTROLLER_H, 'r', encoding='utf-8') as f:
    h = f.read()
h = h.replace('void executeManeuvers(RocketState& rocket_state, const RocketConfig& rocket_config, ControlInput& control_input, FlightHUD& hud) {',
              'void executeManeuvers(entt::registry& registry, entt::entity entity, FlightHUD& hud) {\n' + 
              '    auto& rocket_state = registry.get<RocketState>(entity);\n    auto& rocket_config = registry.get<RocketConfig>(entity);\n    auto& control_input = registry.get<ControlInput>(entity);\n' +
              '    auto& prop = registry.get<PropulsionComponent>(entity);\n    auto& tele = registry.get<TelemetryComponent>(entity);\n    auto& guid = registry.get<GuidanceComponent>(entity);\n    auto& mnv = registry.get<ManeuverComponent>(entity);')

# Special mapping for rocket_state inside controller
h = replace_state_access(h)
with open(CONTROLLER_H, 'w', encoding='utf-8') as f: f.write(h)

print('ECS Phase 2 Refactor for Control & Stage systems completed!')
