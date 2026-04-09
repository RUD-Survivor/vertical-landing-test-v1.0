import re

def read(p):
    with open(p, 'r', encoding='utf-8') as f: return f.read()
def write(p, d):
    with open(p, 'w', encoding='utf-8') as f: f.write(d)

# ============================================================
# 1. stage_manager.h — Fix remaining old declarations
# ============================================================
SMH = r'c:\antigravity_code\RocketSim3D\src\simulation\stage_manager.h'
smh = read(SMH)

# Add entt include
if '<entt/entt.hpp>' not in smh:
    smh = smh.replace('#include "core/rocket_state.h"', '#include "core/rocket_state.h"\n#include <entt/entt.hpp>')

# Fix SeparateStage declaration (old one still there)
smh = smh.replace('bool SeparateStage(RocketState& state, RocketConfig& config);',
                  'bool SeparateStage(entt::registry& registry, entt::entity entity);')

# Add overload for IsCurrentStageEmpty with PropulsionComponent
if 'IsCurrentStageEmpty(const PropulsionComponent&' not in smh:
    smh = smh.replace('bool IsCurrentStageEmpty(const RocketState& state);',
                      'bool IsCurrentStageEmpty(const RocketState& state);\nbool IsCurrentStageEmpty(const PropulsionComponent& prop);')

write(SMH, smh)
print('[OK] stage_manager.h')

# ============================================================
# 2. stage_manager.cpp — Fix SeparateStage definition
# ============================================================
SMCPP = r'c:\antigravity_code\RocketSim3D\src\simulation\stage_manager.cpp'
smcpp = read(SMCPP)

# Add entt include
if '<entt/entt.hpp>' not in smcpp:
    smcpp = '#include <entt/entt.hpp>\n' + smcpp

# Fix SeparateStage definition — it may have been partially changed
# Let's check: if it still has old signature, fix it
if 'StageManager::SeparateStage(RocketState& state, RocketConfig& config)' in smcpp:
    smcpp = smcpp.replace(
        'bool StageManager::SeparateStage(RocketState& state, RocketConfig& config) {',
        'bool StageManager::SeparateStage(entt::registry& registry, entt::entity entity) {\n'
        '    auto& state = registry.get<RocketState>(entity);\n'
        '    auto& config = registry.get<RocketConfig>(entity);')

write(SMCPP, smcpp)
print('[OK] stage_manager.cpp')

# ============================================================
# 3. hud_manager.h — Fix trans.px/py/pz references
#    These are used for hud_ctx.ro_x/y/z but trans is not defined here.
#    The function receives rocket_state directly, so use rocket_state.px
# ============================================================
HM = r'c:\antigravity_code\RocketSim3D\src\scene\hud_manager.h'
hm = read(HM)

hm = hm.replace('hud_ctx.ro_x = trans.px;', 'hud_ctx.ro_x = rocket_state.px;')
hm = hm.replace('hud_ctx.ro_y = trans.py;', 'hud_ctx.ro_y = rocket_state.py;')
hm = hm.replace('hud_ctx.ro_z = trans.pz;', 'hud_ctx.ro_z = rocket_state.pz;')

write(HM, hm)
print('[OK] hud_manager.h')

# ============================================================
# 4. rocket_visuals.h — Fix rocket_state/rocket_config references
#    This file already takes registry & entity in its signature.
#    But uses rocket_state.current_stage without declaring it.
# ============================================================
RV = r'c:\antigravity_code\RocketSim3D\src\scene\rocket_visuals.h'
rv = read(RV)

# Add component extraction at the beginning of the function body
rv = rv.replace(
    '        // 微观近景火箭专用的相机矩阵 (极近裁剪面，用于精确绘制 40米的火箭)',
    '        auto& rocket_state = registry.get<RocketState>(entity);\n'
    '        auto& rocket_config = registry.get<RocketConfig>(entity);\n'
    '        // 微观近景火箭专用的相机矩阵 (极近裁剪面，用于精确绘制 40米的火箭)')

write(RV, rv)
print('[OK] rocket_visuals.h')

# ============================================================
# 5. plume_manager.h — Fix rocket_state.xxx & tele.xxx references
#    This file receives RocketState& directly but uses 'tele.altitude'
#    which was injected by a previous Python script. Need to fix.
# ============================================================
PM = r'c:\antigravity_code\RocketSim3D\src\scene\plume_manager.h'
pm = read(PM)

# Replace tele.altitude with rocket_state.altitude (since function still takes RocketState&)
pm = pm.replace('tele.altitude', 'rocket_state.altitude')

write(PM, pm)
print('[OK] plume_manager.h')

# ============================================================
# 6. spaceport_manager.h — Fix rocket_state.xxx & tele.xxx refs
#    This file already takes registry & entity but uses
#    rocket_state.sim_time, rocket_state.launch_site_px etc. and tele.altitude
#    Need to extract components at the top.
# ============================================================
SP = r'c:\antigravity_code\RocketSim3D\src\scene\spaceport_manager.h'
sp = read(SP)

# Add component extraction at the beginning of render()
sp = sp.replace(
    '        // 高度大于12000米则从视野中剔除发射台',
    '        auto& rocket_state = registry.get<RocketState>(entity);\n'
    '        auto& tele = registry.get<TelemetryComponent>(entity);\n'
    '        auto& trans = registry.get<TransformComponent>(entity);\n'
    '        // 高度大于12000米则从视野中剔除发射台')

write(SP, sp)
print('[OK] spaceport_manager.h')

# ============================================================
# 7. flight_input_system.h — The lambdas still capture rocket_state
#    and use rocket_state.xxx. This is fine as long as setup() and
#    poll() receive RocketState& as a parameter. The only issue is
#    the SeparateStage call which we fixed. But the error might be
#    that our earlier changes added world/entity to setup but
#    the actual function call in flight_scene.h is out of sync.
# ============================================================
FIS = r'c:\antigravity_code\RocketSim3D\src\scene\flight_input_system.h'
fis = read(FIS)

# Verify the setup signature includes world/entity
if 'entt::registry& world, entt::entity rocket_entity' not in fis:
    # Add world/entity to setup signature
    fis = fis.replace(
        'void setup(InputRouter& router, RocketState& rocket_state, RocketConfig& rocket_config, \n               ControlInput& control_input, FlightHUD& hud, CameraDirector& cam, bool& show_clouds) {',
        'void setup(InputRouter& router, RocketState& rocket_state, RocketConfig& rocket_config, \n               ControlInput& control_input, FlightHUD& hud, CameraDirector& cam, bool& show_clouds,\n               entt::registry& world, entt::entity rocket_entity) {')

# Ensure the SeparateStage lambda uses world/entity
if 'StageManager::SeparateStage(rocket_state, rocket_config)' in fis:
    fis = fis.replace(
        'router.registerKey(GLFW_KEY_G, [&rocket_state, &rocket_config]() {\n            if (rocket_state.status == ASCEND || rocket_state.status == DESCEND) {\n                StageManager::SeparateStage(rocket_state, rocket_config);\n            }\n        });',
        'router.registerKey(GLFW_KEY_G, [&rocket_state, &world, &rocket_entity]() {\n            if (rocket_state.status == ASCEND || rocket_state.status == DESCEND) {\n                StageManager::SeparateStage(world, rocket_entity);\n            }\n        });')

write(FIS, fis)
print('[OK] flight_input_system.h')

# ============================================================
# 8. simulation_controller.h — Make sure entt include is present
# ============================================================
SC = r'c:\antigravity_code\RocketSim3D\src\simulation\simulation_controller.h'
sc = read(SC)
if '<entt/entt.hpp>' not in sc:
    sc = sc.replace('#include "core/rocket_state.h"', '#include "core/rocket_state.h"\n#include <entt/entt.hpp>')
write(SC, sc)
print('[OK] simulation_controller.h')

# ============================================================
# 9. Verify flight_scene.h inputSystem.setup call has world/entity
# ============================================================
FS = r'c:\antigravity_code\RocketSim3D\src\scene\flight_scene.h'
fs = read(FS)
if 'inputSystem.setup(input, rocket_state, rocket_config, control_input, hudManager.hud, cam, show_clouds)' in fs:
    fs = fs.replace(
        'inputSystem.setup(input, rocket_state, rocket_config, control_input, hudManager.hud, cam, show_clouds)',
        'inputSystem.setup(input, rocket_state, rocket_config, control_input, hudManager.hud, cam, show_clouds, world, rocket_entity)')
write(FS, fs)
print('[OK] flight_scene.h')

print('\n=== Phase 4 ECS Refactor COMPLETE ===')
