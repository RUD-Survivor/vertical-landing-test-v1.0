"""
PHASE 1: Kill the Dual-Write Bridge
Master refactoring script for removing ALL rocket_state references
from files that already have entt::registry access.

Strategy: Same proven pattern as refactor_hud_ecs.py
"""
import re, os

SRC_ROOT = r"c:\antigravity_code\RocketSim3D\src"

# ============================================================
# FIELD MAPPING: rocket_state.FIELD -> COMPONENT.FIELD
# ============================================================
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
    'path_mutex': 'orb', 'predicted_apsides': 'orb', 'predicted_mnv_apsides': 'orb',
    # VFXComponent (if any fields are used)
}

# Sort by longest field name first to avoid partial matches
SORTED_FIELDS = sorted(FIELD_MAP.keys(), key=len, reverse=True)

def replace_rs_fields(content, filename):
    """Replace rocket_state.field -> component.field"""
    count = 0
    for field in SORTED_FIELDS:
        comp = FIELD_MAP[field]
        old = f"rocket_state.{field}"
        new = f"{comp}.{field}"
        occ = content.count(old)
        if occ > 0:
            content = content.replace(old, new)
            count += occ
    return content, count

def check_remaining(content, filename):
    """Report any remaining rocket_state references"""
    remaining = re.findall(r'rocket_state\.(\w+)', content)
    if remaining:
        unique = sorted(set(remaining))
        for r in unique:
            ct = remaining.count(r)
            print(f"    WARNING: rocket_state.{r} ({ct}x) not in mapping!")
    return remaining

# ============================================================
# FILES TO PROCESS (already have registry access)
# ============================================================
FILES = {
    # file_path: (needs_comp_injection, injection_anchor, components_needed)
    "scene/orbit_system.h": True,
    "scene/flight_input_system.h": True,
    "scene/hud_manager.h": True,
    "scene/celestial_renderer.h": True,
    "scene/plume_manager.h": True,
    "scene/spaceport_manager.h": True,
    "scene/rocket_visuals.h": True,
    "scene/maneuver_manager.h": True,
    "simulation/simulation_controller.h": True,
}

total_replaced = 0
total_remaining = 0

for rel_path, process in FILES.items():
    filepath = os.path.join(SRC_ROOT, rel_path)
    if not os.path.exists(filepath):
        print(f"[SKIP] {rel_path}: file not found")
        continue
    
    with open(filepath, 'r', encoding='utf-8') as f:
        content = f.read()
    
    # Count rocket_state references
    rs_count = content.count('rocket_state.')
    if rs_count == 0:
        print(f"[SKIP] {rel_path}: no rocket_state references")
        continue
    
    print(f"\n[PROCESS] {rel_path} ({rs_count} references)")
    
    # Phase 1: Replace fields
    content, replaced = replace_rs_fields(content, rel_path)
    total_replaced += replaced
    print(f"  Replaced: {replaced}")
    
    # Phase 2: Check remaining
    remaining = check_remaining(content, rel_path)
    total_remaining += len(remaining)
    
    # Write back
    with open(filepath, 'w', encoding='utf-8') as f:
        f.write(content)

print(f"\n{'='*60}")
print(f"TOTAL REPLACED: {total_replaced}")
print(f"TOTAL REMAINING: {total_remaining}")

# ============================================================
# SPECIAL: flight_scene.h
# This is the most complex file - handle separately
# ============================================================
print(f"\n[SPECIAL] Processing flight_scene.h...")
fs_path = os.path.join(SRC_ROOT, "scene/flight_scene.h")
with open(fs_path, 'r', encoding='utf-8') as f:
    content = f.read()

rs_count = content.count('rocket_state.')
print(f"  rocket_state references: {rs_count}")

# Replace field accesses (but NOT the sync functions themselves - we'll delete those)
content, replaced = replace_rs_fields(content, "flight_scene.h")
total_replaced += replaced
print(f"  Replaced: {replaced}")
remaining = check_remaining(content, "flight_scene.h")

with open(fs_path, 'w', encoding='utf-8') as f:
    f.write(content)

print(f"\n{'='*60}")
print(f"GRAND TOTAL REPLACED: {total_replaced}")
print(f"GRAND REMAINING: {total_remaining + len(remaining)}")
