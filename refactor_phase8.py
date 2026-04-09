import re

def read(p):
    with open(p, 'r', encoding='utf-8') as f: return f.read()
def write(p, d):
    with open(p, 'w', encoding='utf-8') as f: f.write(d)

MMH = r'c:\antigravity_code\RocketSim3D\src\scene\maneuver_manager.h'
mmh = read(MMH)

# 1. Remove the misplaced 'private:' at line ~250
# Looking for 'private:\n   void handleHandleDragging'
mmh = re.sub(r'private:\s+void handleHandleDragging', 'void ManeuverManager::handleHandleDragging', mmh)

# 2. Add Component injection to update()
# Find the start of the body
update_pattern = r'(void ManeuverManager::update\(.*?\{)'
mmh = re.sub(update_pattern, r'\1\n        auto& mnv = registry.get<ManeuverComponent>(entity);\n        auto& tele = registry.get<TelemetryComponent>(entity);', mmh, count=1, flags=re.DOTALL)

# 3. Add Component injection to render()
render_pattern = r'(void ManeuverManager::render\(.*?\{)'
mmh = re.sub(render_pattern, r'\1\n        auto& mnv = registry.get<ManeuverComponent>(entity);\n        auto& tele = registry.get<TelemetryComponent>(entity);', mmh, count=1, flags=re.DOTALL)

# 4. Add Component injection to updatePopupState()
popup_pattern = r'(void ManeuverManager::updatePopupState\(.*?\{)'
mmh = re.sub(popup_pattern, r'\1\n        auto& tele = registry.get<TelemetryComponent>(entity);\n        auto& mnv = registry.get<ManeuverComponent>(entity);', mmh, count=1, flags=re.DOTALL)

# 5. Fix remaining legacy references
mmh = mmh.replace('rocket_state.sim_time', 'tele.sim_time')
mmh = mmh.replace('rocket_state.maneuvers', 'mnv.maneuvers')
mmh = mmh.replace('rocket_state.selected_maneuver_index', 'mnv.selected_maneuver_index')

# 6. Ensure handleHandleDragging has the class prefix if it was missing
if 'void handleHandleDragging' in mmh and 'void ManeuverManager::handleHandleDragging' not in mmh:
    mmh = mmh.replace('void handleHandleDragging', 'void ManeuverManager::handleHandleDragging')

write(MMH, mmh)
print('[OK] ManeuverManager implementations fixed')
