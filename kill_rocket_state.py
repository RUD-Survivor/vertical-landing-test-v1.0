"""
Phase: Kill remaining RocketState references across the codebase.
This script performs mechanical find-and-replace to remove RocketState
from all systems that already have ECS component getters nearby.
"""
import re

def patch(path, replacements):
    with open(path, 'r', encoding='utf-8') as f:
        content = f.read()
    for old, new in replacements:
        if old in content:
            content = content.replace(old, new)
            print(f"  [OK] {path}: replaced '{old[:60]}...'")
        else:
            print(f"  [SKIP] {path}: not found '{old[:60]}...'")
    with open(path, 'w', encoding='utf-8') as f:
        f.write(content)

# ============================================================
# 1. control_system.cpp — remove unused RocketState gets
# ============================================================
print("\n=== control_system.cpp ===")
patch(r'c:\antigravity_code\RocketSim3D\src\control\control_system.cpp', [
    # Line 14: remove auto& state = ... (only used on line 268 which we'll fix)
    ('    auto& state = registry.get<RocketState>(entity);\n    auto& config = registry.get<RocketConfig>(entity);',
     '    auto& config = registry.get<RocketConfig>(entity);'),
    # Line 195: second function also gets it
    ('    auto& state = registry.get<RocketState>(entity);\n    auto& config = registry.get<RocketConfig>(entity);\n    auto& input = registry.get<ControlInput>(entity);\n    auto& att = registry.get<AttitudeComponent>(entity);\n    auto& guid = registry.get<GuidanceComponent>(entity);\n    auto& prop = registry.get<PropulsionComponent>(entity);\n    auto& mnv = registry.get<ManeuverComponent>(entity);\n    auto& trans = registry.get<TransformComponent>(entity);\n    auto& vel = registry.get<VelocityComponent>(entity);\n    auto& tele = registry.get<TelemetryComponent>(entity);',
     '    auto& config = registry.get<RocketConfig>(entity);\n    auto& input = registry.get<ControlInput>(entity);\n    auto& att = registry.get<AttitudeComponent>(entity);\n    auto& guid = registry.get<GuidanceComponent>(entity);\n    auto& prop = registry.get<PropulsionComponent>(entity);\n    auto& mnv = registry.get<ManeuverComponent>(entity);\n    auto& trans = registry.get<TransformComponent>(entity);\n    auto& vel = registry.get<VelocityComponent>(entity);\n    auto& tele = registry.get<TelemetryComponent>(entity);'),
    # Line 268: calculateRemainingDV(state, node) -> need vel/tele
    ('ManeuverSystem::calculateRemainingDV(state, node)',
     'ManeuverSystem::calculateRemainingDV(vel, tele, node)'),
])

# ============================================================
# 2. maneuver_system.h — change signature to use components
# ============================================================
print("\n=== maneuver_system.h ===")
patch(r'c:\antigravity_code\RocketSim3D\src\simulation\maneuver_system.h', [
    ('static Vec3 calculateRemainingDV(const RocketState& state, const ManeuverNode& node)',
     'static Vec3 calculateRemainingDV(const VelocityComponent& vel, const TelemetryComponent& tele, const ManeuverNode& node)'),
    ('double cur_rel_vx = state.vx + SOLAR_SYSTEM[current_soi_index].vx - ref_b.vx;',
     'double cur_rel_vx = vel.vx + SOLAR_SYSTEM[current_soi_index].vx - ref_b.vx;'),
    ('double cur_rel_vy = state.vy + SOLAR_SYSTEM[current_soi_index].vy - ref_b.vy;',
     'double cur_rel_vy = vel.vy + SOLAR_SYSTEM[current_soi_index].vy - ref_b.vy;'),
    ('double cur_rel_vz = state.vz + SOLAR_SYSTEM[current_soi_index].vz - ref_b.vz;',
     'double cur_rel_vz = vel.vz + SOLAR_SYSTEM[current_soi_index].vz - ref_b.vz;'),
    ('double dt_snap = state.sim_time - node.snap_time;',
     'double dt_snap = tele.sim_time - node.snap_time;'),
])

# ============================================================
# 3. simulation_controller.h — remove RocketState gets
# ============================================================
print("\n=== simulation_controller.h ===")
patch(r'c:\antigravity_code\RocketSim3D\src\simulation\simulation_controller.h', [
    ('        auto& rocket_state = registry.get<RocketState>(entity);\n        auto& rocket_config = registry.get<RocketConfig>(entity);\n        auto& control_input = registry.get<ControlInput>(entity);\n        \n        auto& trans',
     '        auto& rocket_config = registry.get<RocketConfig>(entity);\n        auto& control_input = registry.get<ControlInput>(entity);\n        \n        auto& trans'),
    ('    auto& rocket_state = registry.get<RocketState>(entity);\n    auto& rocket_config = registry.get<RocketConfig>(entity);\n    auto& control_input = registry.get<ControlInput>(entity);\n    auto& prop',
     '    auto& rocket_config = registry.get<RocketConfig>(entity);\n    auto& control_input = registry.get<ControlInput>(entity);\n    auto& prop'),
])

# ============================================================
# 4. stage_manager.cpp — SeparateStage uses state.X -> prop/guid
# ============================================================
print("\n=== stage_manager.cpp ===")
patch(r'c:\antigravity_code\RocketSim3D\src\simulation\stage_manager.cpp', [
    ('    auto& state = registry.get<RocketState>(entity);\n    auto& config = registry.get<RocketConfig>(entity);',
     '    auto& prop = registry.get<PropulsionComponent>(entity);\n    auto& guid = registry.get<GuidanceComponent>(entity);\n    auto& config = registry.get<RocketConfig>(entity);'),
    ('state.current_stage >= state.total_stages', 'prop.current_stage >= prop.total_stages'),
    ('config.stage_configs[state.current_stage]', 'config.stage_configs[prop.current_stage]'),
    ('state.stage_fuels[state.current_stage]', 'prop.stage_fuels[prop.current_stage]'),
    ('state.jettisoned_mass += jettison;', 'prop.jettisoned_mass += jettison;'),
    ('<< (state.current_stage + 1)', '<< (prop.current_stage + 1)'),
    ('state.current_stage++;', 'prop.current_stage++;'),
    ('state.fuel = state.stage_fuels[state.current_stage];', 'prop.fuel = prop.stage_fuels[prop.current_stage];'),
    ('SyncActiveConfig(config, state.current_stage);', 'SyncActiveConfig(config, prop.current_stage);'),
    ('for (int i = state.current_stage + 1; i < state.total_stages; i++) {', 'for (int i = prop.current_stage + 1; i < prop.total_stages; i++) {'),
    ('state.stage_fuels[i]', 'prop.stage_fuels[i]'),
    ('<< state.total_stages', '<< prop.total_stages'),
    ('<< (int)state.fuel', '<< (int)prop.fuel'),
    ('state.mission_msg = ">> STAGE "', 'guid.mission_msg = ">> STAGE "'),
    ('std::to_string(state.current_stage + 1)', 'std::to_string(prop.current_stage + 1)'),
])

# ============================================================
# 5. stage_manager.h — fix function declarations
# ============================================================
print("\n=== stage_manager.h ===")
patch(r'c:\antigravity_code\RocketSim3D\src\simulation\stage_manager.h', [
    ('bool IsCurrentStageEmpty(const RocketState& state);',
     '// bool IsCurrentStageEmpty(const RocketState& state); // LEGACY'),
    ('double GetTotalMassFromStage(const RocketConfig& config, const RocketState& state, int from_stage);',
     'double GetTotalMassFromStage(const RocketConfig& config, const PropulsionComponent& prop, int from_stage);'),
])

# ============================================================
# 6. stage_manager.cpp — old IsCurrentStageEmpty and GetTotalMassFromStage
# ============================================================
print("\n=== stage_manager.cpp (legacy functions) ===")
patch(r'c:\antigravity_code\RocketSim3D\src\simulation\stage_manager.cpp', [
    # Replace legacy IsCurrentStageEmpty
    ('bool IsCurrentStageEmpty(const RocketState& state) {\n    if (state.current_stage < 0 || state.current_stage >= (int)state.stage_fuels.size()) return true;\n    return state.stage_fuels[state.current_stage] <= 0.0;\n}',
     '// Legacy RocketState overload removed — use PropulsionComponent overload'),
    # Replace GetTotalMassFromStage 
    ('double GetTotalMassFromStage(const RocketConfig& config, const RocketState& state, int from_stage) {',
     'double GetTotalMassFromStage(const RocketConfig& config, const PropulsionComponent& state, int from_stage) {'),
])

# ============================================================
# 7. plume_manager.h — remove unused rs_legacy
# ============================================================
print("\n=== plume_manager.h ===")
patch(r'c:\antigravity_code\RocketSim3D\src\scene\plume_manager.h', [
    ('        auto& rs_legacy = registry.get<RocketState>(entity); // Temporary for config if needed\n',
     ''),
])

# ============================================================
# 8. orbit_system.h — remove RocketState get
# ============================================================
print("\n=== orbit_system.h ===")
patch(r'c:\antigravity_code\RocketSim3D\src\scene\orbit_system.h', [
    ('        auto& rocket_state = registry.get<RocketState>(entity);\n', ''),
])

# ============================================================
# 9. hud_manager.h — remove RocketState get
# ============================================================
print("\n=== hud_manager.h ===")
patch(r'c:\antigravity_code\RocketSim3D\src\scene\hud_manager.h', [
    ('        auto& rocket_state = registry.get<RocketState>(entity);\n', ''),
])

# ============================================================
# 10. HUD_system.h — remove RocketState* from HUDContext and usage
# ============================================================
print("\n=== HUD_system.h ===")
patch(r'c:\antigravity_code\RocketSim3D\src\render\HUD_system.h', [
    ('    RocketState* rocket_state;\n', ''),
    ('        RocketState& rocket_state = *ctx.rocket_state;\n',
     '        // ECS components accessed via registry\n'),
])

# ============================================================
# 11. flight_scene.h — remove emplace<RocketState>, remove get<RocketState>
# ============================================================
print("\n=== flight_scene.h ===")
patch(r'c:\antigravity_code\RocketSim3D\src\scene\flight_scene.h', [
    ('        auto& rocket_state = world.emplace<RocketState>(rocket_entity);\n', ''),
])

# Remove all remaining get<RocketState> from flight_scene.h
with open(r'c:\antigravity_code\RocketSim3D\src\scene\flight_scene.h', 'r', encoding='utf-8') as f:
    content = f.read()
content = content.replace('        auto& rocket_state = world.get<RocketState>(rocket_entity);\n', '')
with open(r'c:\antigravity_code\RocketSim3D\src\scene\flight_scene.h', 'w', encoding='utf-8') as f:
    f.write(content)
print("  [OK] flight_scene.h: removed all get<RocketState> lines")

# ============================================================
# 12. physics_system.cpp — RocketState::MAX_SMOKE -> VFXComponent::MAX_SMOKE
# We need to add MAX_SMOKE to VFXComponent if not present
# ============================================================
print("\n=== physics_system.cpp ===")
patch(r'c:\antigravity_code\RocketSim3D\src\physics\physics_system.cpp', [
    ('RocketState::MAX_SMOKE', 'VFXComponent::MAX_SMOKE'),
])

# ============================================================
# 13. HUD_system.h — fix the calculateRemainingDV call
# ============================================================
print("\n=== HUD_system.h (calculateRemainingDV calls) ===")
with open(r'c:\antigravity_code\RocketSim3D\src\render\HUD_system.h', 'r', encoding='utf-8') as f:
    content = f.read()
content = content.replace('ManeuverSystem::calculateRemainingDV(rocket_state, node)', 
                          'ManeuverSystem::calculateRemainingDV(vel, tele, node)')
with open(r'c:\antigravity_code\RocketSim3D\src\render\HUD_system.h', 'w', encoding='utf-8') as f:
    f.write(content)
print("  [OK] HUD_system.h: updated calculateRemainingDV calls")

print("\n=== DONE ===")
