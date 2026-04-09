"""
Refactor HUD_system.h: Replace rocket_state.xxx with ECS component references.
Phase 1: Add component unpacking at top of render()
Phase 2: Bulk replace all field accesses
"""
import re

FILE = r"c:\antigravity_code\RocketSim3D\src\render\HUD_system.h"

with open(FILE, 'r', encoding='utf-8') as f:
    content = f.read()

# ============================================================
# PHASE 1: Add ECS component references after existing unpack
# ============================================================
# Insert after the line: "RocketState& rocket_state = *ctx.rocket_state;"
INJECT_AFTER = "RocketState& rocket_state = *ctx.rocket_state;"
ECS_REFS = """
        // --- ECS Component References (replacing rocket_state access) ---
        auto& trans = ctx.registry->get<TransformComponent>(ctx.entity);
        auto& vel   = ctx.registry->get<VelocityComponent>(ctx.entity);
        auto& att   = ctx.registry->get<AttitudeComponent>(ctx.entity);
        auto& prop  = ctx.registry->get<PropulsionComponent>(ctx.entity);
        auto& tele  = ctx.registry->get<TelemetryComponent>(ctx.entity);
        auto& guid  = ctx.registry->get<GuidanceComponent>(ctx.entity);
        auto& mnv   = ctx.registry->get<ManeuverComponent>(ctx.entity);
        auto& orb   = ctx.registry->get<OrbitComponent>(ctx.entity);
"""

if INJECT_AFTER in content and "// --- ECS Component References" not in content:
    content = content.replace(INJECT_AFTER, INJECT_AFTER + ECS_REFS, 1)
    print("[Phase 1] Injected ECS component references.")
else:
    print("[Phase 1] Already injected or anchor not found.")

# ============================================================
# PHASE 2: Replace rocket_state.field -> component.field
# ============================================================
# Map: field_name -> component_alias
FIELD_MAP = {
    # TransformComponent
    'px': 'trans', 'py': 'trans', 'pz': 'trans',
    'abs_px': 'trans', 'abs_py': 'trans', 'abs_pz': 'trans',
    'surf_px': 'trans', 'surf_py': 'trans', 'surf_pz': 'trans',
    'launch_site_px': 'trans', 'launch_site_py': 'trans', 'launch_site_pz': 'trans',
    'launch_latitude': 'trans', 'launch_longitude': 'trans',
    
    # VelocityComponent
    'vx': 'vel', 'vy': 'vel', 'vz': 'vel',
    'abs_vx': 'vel', 'abs_vy': 'vel', 'abs_vz': 'vel',
    'acceleration': 'vel',
    
    # AttitudeComponent  
    'attitude': 'att', 'attitude_initialized': 'att',
    'angle': 'att', 'ang_vel': 'att',
    'angle_z': 'att', 'ang_vel_z': 'att',
    'angle_roll': 'att', 'ang_vel_roll': 'att',
    
    # PropulsionComponent
    'fuel': 'prop', 'current_stage': 'prop', 'total_stages': 'prop',
    'stage_fuels': 'prop', 'jettisoned_mass': 'prop',
    'fuel_consumption_rate': 'prop', 'thrust_power': 'prop',
    
    # TelemetryComponent
    'sim_time': 'tele', 'altitude': 'tele', 'velocity': 'tele',
    'local_vx': 'tele', 'terrain_altitude': 'tele', 'solar_occlusion': 'tele',
    
    # GuidanceComponent
    'status': 'guid', 'mission_msg': 'guid', 'mission_phase': 'guid',
    'mission_timer': 'guid', 'auto_mode': 'guid',
    'sas_active': 'guid', 'rcs_active': 'guid', 'sas_mode': 'guid',
    'sas_target_vec': 'guid', 'leg_deploy_progress': 'guid',
    'suicide_burn_locked': 'guid', 'show_absolute_time': 'guid',
    'pid_vert': 'guid', 'pid_pos': 'guid',
    'pid_att': 'guid', 'pid_att_z': 'guid', 'pid_att_roll': 'guid',
    
    # ManeuverComponent
    'maneuvers': 'mnv', 'selected_maneuver_index': 'mnv',
    
    # OrbitComponent
    'predicted_path': 'orb', 'predicted_mnv_path': 'orb',
    'predicted_ground_track': 'orb',
    'last_prediction_sim_time': 'orb', 'prediction_in_progress': 'orb',
    'path_mutex': 'orb',
}

# Sort by longest field name first to avoid partial matches
# e.g., "ang_vel_z" must be matched before "ang_vel"
sorted_fields = sorted(FIELD_MAP.keys(), key=len, reverse=True)

count = 0
for field in sorted_fields:
    comp = FIELD_MAP[field]
    old = f"rocket_state.{field}"
    new = f"{comp}.{field}"
    occurrences = content.count(old)
    if occurrences > 0:
        content = content.replace(old, new)
        count += occurrences
        print(f"  [{comp}] {old} -> {new}  ({occurrences}x)")

print(f"\n[Phase 2] Replaced {count} field accesses total.")

# ============================================================
# PHASE 3: Verify remaining rocket_state references
# ============================================================
remaining = [m.group(0) for m in re.finditer(r'rocket_state\.(\w+)', content)]
if remaining:
    unique = sorted(set(remaining))
    print(f"\n[Phase 3] WARNING: {len(remaining)} remaining rocket_state references:")
    for r in unique:
        print(f"  - {r} ({remaining.count(r)}x)")
else:
    print("\n[Phase 3] All rocket_state references replaced!")

with open(FILE, 'w', encoding='utf-8') as f:
    f.write(content)

print("\n[Done] File updated successfully.")
