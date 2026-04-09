def read(p):
    with open(p, 'r', encoding='utf-8') as f: return f.read()
def write(p, d):
    with open(p, 'w', encoding='utf-8') as f: f.write(d)

MMH = r'c:\antigravity_code\RocketSim3D\src\scene\maneuver_manager.h'
mmh = read(MMH)

# Remove duplicates of these specific lines
lines_to_fix = [
    'auto& mnv = registry.get<ManeuverComponent>(entity);',
    'auto& tele = registry.get<TelemetryComponent>(entity);'
]

new_lines = []
content_lines = mmh.split('\n')
for i in range(len(content_lines)):
    line = content_lines[i]
    is_dupe = False
    if line.strip() in lines_to_fix:
        # Check if the previous line was the same
        if i > 0 and content_lines[i-1].strip() == line.strip():
            is_dupe = True
    if not is_dupe:
        new_lines.append(line)

mmh = '\n'.join(new_lines)

# Fix remaining rocket_state access at ~line 240
mmh = mmh.replace('rocket_state.maneuvers', 'mnv.maneuvers')

write(MMH, mmh)
print('[OK] ManeuverManager cleaned up')
