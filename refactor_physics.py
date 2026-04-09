import os
import re

H_FILE = r'c:\antigravity_code\RocketSim3D\src\physics\physics_system.h'
CPP_FILE = r'c:\antigravity_code\RocketSim3D\src\physics\physics_system.cpp'

# Mapping legacy fields to new component prefixes
mapping = {
    # Transform
    'px': 'trans.px', 'py': 'trans.py', 'pz': 'trans.pz',
    'abs_px': 'trans.abs_px', 'abs_py': 'trans.abs_py', 'abs_pz': 'trans.abs_pz',
    'surf_px': 'trans.surf_px', 'surf_py': 'trans.surf_py', 'surf_pz': 'trans.surf_pz',
    'launch_latitude': 'trans.launch_latitude', 'launch_longitude': 'trans.launch_longitude',
    'launch_site_px': 'trans.launch_site_px', 'launch_site_py': 'trans.launch_site_py', 'launch_site_pz': 'trans.launch_site_pz',

    # Velocity
    'vx': 'vel.vx', 'vy': 'vel.vy', 'vz': 'vel.vz',
    'abs_vx': 'vel.abs_vx', 'abs_vy': 'vel.abs_vy', 'abs_vz': 'vel.abs_vz',
    'acceleration': 'vel.acceleration', 'vertical_velocity': 'vel.vertical_velocity', 'horizontal_velocity': 'vel.horizontal_velocity',

    # Attitude
    'attitude': 'att.attitude', 'attitude_initialized': 'att.initialized',
    'angle': 'att.angle', 'ang_vel': 'att.ang_vel',
    'angle_z': 'att.angle_z', 'ang_vel_z': 'att.ang_vel_z',
    'angle_roll': 'att.angle_roll', 'ang_vel_roll': 'att.ang_vel_roll',

    # Propulsion
    'fuel': 'prop.fuel', 'current_stage': 'prop.current_stage', 'total_stages': 'prop.total_stages',
    'stage_fuels': 'prop.stage_fuels', 'jettisoned_mass': 'prop.jettisoned_mass',
    'fuel_consumption_rate': 'prop.fuel_consumption_rate', 'thrust_power': 'prop.thrust_power',

    # Telemetry
    'sim_time': 'tele.sim_time', 'altitude': 'tele.altitude', 'terrain_altitude': 'tele.terrain_altitude',
    'velocity': 'tele.velocity', 'local_vx': 'tele.local_vx', 'solar_occlusion': 'tele.solar_occlusion',

    # Guidance
    'suicide_burn_locked': 'guid.suicide_burn_locked', 'status': 'guid.status', 'mission_msg': 'guid.mission_msg',
    'mission_phase': 'guid.mission_phase', 'mission_timer': 'guid.mission_timer', 'show_absolute_time': 'guid.show_absolute_time',
    'auto_mode': 'guid.auto_mode', 'sas_active': 'guid.sas_active', 'rcs_active': 'guid.rcs_active',
    'sas_mode': 'guid.sas_mode', 'sas_target_vec': 'guid.sas_target_vec', 'leg_deploy_progress': 'guid.leg_deploy_progress',
    'pid_vert': 'guid.pid_vert', 'pid_pos': 'guid.pid_pos', 'pid_att': 'guid.pid_att',
    'pid_att_z': 'guid.pid_att_z', 'pid_att_roll': 'guid.pid_att_roll',

    # Orbit
    'predicted_path': 'orb.predicted_path', 'predicted_mnv_path': 'orb.predicted_mnv_path',
    'predicted_apsides': 'orb.predicted_apsides', 'predicted_mnv_apsides': 'orb.predicted_mnv_apsides',
    'predicted_ground_track': 'orb.predicted_ground_track', 'predicted_mnv_ground_track': 'orb.predicted_mnv_ground_track',
    'path_mutex': 'orb.path_mutex', 'last_prediction_sim_time': 'orb.last_prediction_sim_time', 'prediction_in_progress': 'orb.prediction_in_progress',

    # Maneuver
    'maneuvers': 'mnv.maneuvers', 'selected_maneuver_index': 'mnv.selected_maneuver_index',

    # VFX
    'smoke': 'vfx.smoke', 'smoke_idx': 'vfx.smoke_idx'
}

def replace_state_access(content):
    # Sort keys by length descending to prevent partial matches (e.g. replacing 'px' inside 'abs_px')
    sorted_keys = sorted(mapping.keys(), key=len, reverse=True)
    for key in sorted_keys:
        # Match 'state.key' ensuring it's a word boundary
        pattern = r'\bstate\.' + re.escape(key) + r'\b'
        content = re.sub(pattern, mapping[key], content)
    return content

# Read .h
with open(H_FILE, 'r', encoding='utf-8') as f:
    h_data = f.read()

# Add EnTT include if not present
if '<entt/entt.hpp>' not in h_data:
    h_data = h_data.replace('#include "core/rocket_state.h"', '#include "core/rocket_state.h"\n#include <entt/entt.hpp>')

# Change signatures in .h
h_data = h_data.replace('void getOrbitParams(const RocketState& state,', 'void getOrbitParams(entt::registry& registry, entt::entity entity,')
h_data = h_data.replace('void CheckSOI_Transitions(RocketState& state)', 'void CheckSOI_Transitions(entt::registry& registry, entt::entity entity)')
h_data = h_data.replace('double CalculateSolarOcclusion(const RocketState& state)', 'double CalculateSolarOcclusion(entt::registry& registry, entt::entity entity)')
h_data = h_data.replace('void Update(RocketState& state, const RocketConfig& config, const ControlInput& input, double dt)', 'void Update(entt::registry& registry, entt::entity entity, double dt)')
h_data = h_data.replace('void FastGravityUpdate(RocketState& state, const RocketConfig& config, double dt_total)', 'void FastGravityUpdate(entt::registry& registry, entt::entity entity, double dt_total)')
h_data = h_data.replace('void EmitSmoke(RocketState& state, const RocketConfig& config, double dt)', 'void EmitSmoke(entt::registry& registry, entt::entity entity, double dt)')
h_data = h_data.replace('void UpdateSmoke(RocketState& state, double dt)', 'void UpdateSmoke(entt::registry& registry, entt::entity entity, double dt)')

with open(H_FILE, 'w', encoding='utf-8') as f:
    f.write(h_data)

# Read .cpp
with open(CPP_FILE, 'r', encoding='utf-8') as f:
    cpp_data = f.read()

component_fetch_str = """
    auto& trans = registry.get<TransformComponent>(entity);
    auto& vel = registry.get<VelocityComponent>(entity);
    auto& att = registry.get<AttitudeComponent>(entity);
    auto& prop = registry.get<PropulsionComponent>(entity);
    auto& tele = registry.get<TelemetryComponent>(entity);
    auto& guid = registry.get<GuidanceComponent>(entity);
    auto& orb = registry.get<OrbitComponent>(entity);
    auto& mnv = registry.get<ManeuverComponent>(entity);
    auto& vfx = registry.get<VFXComponent>(entity);
    const auto& config = registry.get<RocketConfig>(entity);
    const auto& input = registry.get<ControlInput>(entity);
"""

component_fetch_partial = """
    auto& trans = registry.get<TransformComponent>(entity);
    auto& vel = registry.get<VelocityComponent>(entity);
    auto& tele = registry.get<TelemetryComponent>(entity);
"""

# Replace signatures in .cpp
cpp_data = cpp_data.replace('void getOrbitParams(const RocketState& state, double& apoapsis, double& periapsis) {', 
                            'void getOrbitParams(entt::registry& registry, entt::entity entity, double& apoapsis, double& periapsis) {\n' + component_fetch_partial)

cpp_data = cpp_data.replace('void CheckSOI_Transitions(RocketState& state) {', 
                            'void CheckSOI_Transitions(entt::registry& registry, entt::entity entity) {\n' + component_fetch_partial)

cpp_data = cpp_data.replace('double CalculateSolarOcclusion(const RocketState& state) {', 
                            'double CalculateSolarOcclusion(entt::registry& registry, entt::entity entity) {\n' + component_fetch_partial)

cpp_data = cpp_data.replace('void Update(RocketState& state, const RocketConfig& config, const ControlInput& input, double dt) {', 
                            'void Update(entt::registry& registry, entt::entity entity, double dt) {\n' + component_fetch_str)

cpp_data = cpp_data.replace('void FastGravityUpdate(RocketState& state, const RocketConfig& config, double dt_total) {', 
                            'void FastGravityUpdate(entt::registry& registry, entt::entity entity, double dt_total) {\n' + component_fetch_str)

cpp_data = cpp_data.replace('void EmitSmoke(RocketState& state, const RocketConfig& config, double dt) {', 
                            'void EmitSmoke(entt::registry& registry, entt::entity entity, double dt) {\n' + component_fetch_str)

cpp_data = cpp_data.replace('void UpdateSmoke(RocketState& state, double dt) {', 
                            'void UpdateSmoke(entt::registry& registry, entt::entity entity, double dt) {\n    auto& vfx = registry.get<VFXComponent>(entity);\n    auto& trans = registry.get<TransformComponent>(entity);\n    auto& vel = registry.get<VelocityComponent>(entity);')

cpp_data = replace_state_access(cpp_data)

with open(CPP_FILE, 'w', encoding='utf-8') as f:
    f.write(cpp_data)

print('ECS Physics refactor completed successfully!')
