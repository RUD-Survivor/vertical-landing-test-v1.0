def read(p):
    with open(p, 'r', encoding='utf-8') as f: return f.read()
def write(p, d):
    with open(p, 'w', encoding='utf-8') as f: f.write(d)

SMCPP = r'c:\antigravity_code\RocketSim3D\src\simulation\stage_manager.cpp'
smcpp = read(SMCPP)

# ============================================================
# Problem: SeparateStage still uses old signature AND references
# prop.xxx / guid.xxx without declaring them.
# Also IsCurrentStageEmpty and GetTotalMassFromStage have the same issue.
# 
# Strategy: For functions that have prop.xxx/guid.xxx references but
# take (RocketState& state, ...) — we need to add auto& prop = state; mappings.
# Since RocketState still has all these fields, we can just do:
#   state.current_stage, state.stage_fuels, etc.
# The cleanest fix: revert prop.xxx -> state.xxx and guid.xxx -> state.xxx
# for these functions that still take RocketState&.
# ============================================================

import re

# For SeparateStage: It still takes (RocketState& state, RocketConfig& config)
# Replace all prop.xxx with state.xxx and guid.xxx with state.xxx
# Only in the SeparateStage function body

prop_fields = [
    'prop.current_stage', 'prop.total_stages', 'prop.stage_fuels',
    'prop.jettisoned_mass', 'prop.fuel', 'prop.thrust_power',
    'prop.fuel_consumption_rate'
]
guid_fields = [
    'guid.status', 'guid.mission_msg', 'guid.auto_mode',
    'guid.sas_active', 'guid.rcs_active', 'guid.sas_mode',
    'guid.mission_phase', 'guid.mission_timer',
    'guid.suicide_burn_locked', 'guid.leg_deploy_progress',
]

for old in prop_fields:
    new = old.replace('prop.', 'state.')
    smcpp = smcpp.replace(old, new)

for old in guid_fields:
    new = old.replace('guid.', 'state.')
    smcpp = smcpp.replace(old, new)

# Also fix tele.xxx -> state.xxx
tele_fields = ['tele.sim_time', 'tele.altitude', 'tele.velocity', 'tele.local_vx', 'tele.solar_occlusion', 'tele.terrain_altitude']
for old in tele_fields:
    new = old.replace('tele.', 'state.')
    smcpp = smcpp.replace(old, new)

# Also fix trans.xxx -> state.xxx  
trans_fields = ['trans.px', 'trans.py', 'trans.pz', 'trans.abs_px', 'trans.abs_py', 'trans.abs_pz',
                'trans.surf_px', 'trans.surf_py', 'trans.surf_pz']
for old in trans_fields:
    new = old.replace('trans.', 'state.')
    smcpp = smcpp.replace(old, new)

# Also fix vel.xxx -> state.xxx
vel_fields = ['vel.vx', 'vel.vy', 'vel.vz', 'vel.abs_vx', 'vel.abs_vy', 'vel.abs_vz']
for old in vel_fields:
    new = old.replace('vel.', 'state.')
    smcpp = smcpp.replace(old, new)

# Remove the ECS overload at the bottom that's outside the namespace (it will fail to compile)
# Move it inside the namespace instead
if '// ECS overload\nbool StageManager::IsCurrentStageEmpty' in smcpp:
    # Remove the block at the end
    smcpp = smcpp.replace(
        '\n\n// ECS overload\nbool StageManager::IsCurrentStageEmpty(const PropulsionComponent& prop) {\n    if (prop.current_stage < 0 || prop.current_stage >= (int)prop.stage_fuels.size()) return false;\n    return prop.stage_fuels[prop.current_stage] <= 0.0;\n}\n',
        '')

# Also try with \r\n line endings
smcpp = smcpp.replace(
    '\r\n\r\n// ECS overload\r\nbool StageManager::IsCurrentStageEmpty(const PropulsionComponent& prop) {\r\n    if (prop.current_stage < 0 || prop.current_stage >= (int)prop.stage_fuels.size()) return false;\r\n    return prop.stage_fuels[prop.current_stage] <= 0.0;\r\n}\r\n',
    '')

# Add the ECS overload INSIDE the namespace, before the closing brace
smcpp = smcpp.replace(
    '} // namespace StageManager',
    '''// ECS overload for PropulsionComponent
bool IsCurrentStageEmpty(const PropulsionComponent& prop) {
    if (prop.current_stage < 0 || prop.current_stage >= (int)prop.stage_fuels.size()) return false;
    return prop.stage_fuels[prop.current_stage] <= 0.0;
}

} // namespace StageManager''')

write(SMCPP, smcpp)
print('[OK] stage_manager.cpp - reverted prop/guid/tele/trans -> state for legacy functions')

# ============================================================
# Also check: does stage_manager.h have the PropulsionComponent
# declaration? We need the include.
# ============================================================
SMH = r'c:\antigravity_code\RocketSim3D\src\simulation\stage_manager.h'
smh = read(SMH)

# Ensure the file knows about PropulsionComponent
# Since rocket_state.h defines it, the include should be enough
# Let's verify PropulsionComponent is defined in rocket_state.h
RSH = r'c:\antigravity_code\RocketSim3D\src\core\rocket_state.h'
rsh = read(RSH)
if 'struct PropulsionComponent' in rsh:
    print('[OK] PropulsionComponent is defined in rocket_state.h')
else:
    print('[WARN] PropulsionComponent NOT found in rocket_state.h!')

print('\n=== Phase 5 ECS Fix COMPLETE ===')
