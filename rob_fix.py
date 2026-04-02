import re
import codecs

path = r'c:\antigravity_code\RocketSim3D\src\scene\flight_scene.h'

with open(path, 'rb') as f:
    bts = f.read()

try:
    text = bts.decode('utf-8')
    enc = 'utf-8'
except:
    text = bts.decode('gbk', 'ignore')
    enc = 'gbk'

# 1. Add FlightHUD hud; after CameraDirector cam;
if "FlightHUD hud;" not in text:
    text = re.sub(r'(CameraDirector\s+cam;)', r'\1\n    FlightHUD hud;', text, count=1)

# 2. Add ws_d missing in update/render loops. We can just add it into the class fields.
if "double ws_d" not in text:
    text = re.sub(r'(int\s+frame\s*=\s*0;)', r'\1\n    double ws_d = 1.0;', text, count=1)
    
# 3. Fix GameContext& ctx duplicate redefinition in onEnter
text = re.sub(r'(GameContext&\s+ctx\s*=\s*GameContext::getInstance\(\);\s*)(\s*GameContext&\s+ctx\s*=\s*GameContext::getInstance\(\);\s*)', r'\1', text)

# 4. If ctx is still redefined manually somewhere else
lines = text.split('\n')
new_lines = []
in_on_enter = False
ctx_defined_here = False
for line in lines:
    if "void onEnter" in line:
        in_on_enter = True
        ctx_defined_here = False
    if in_on_enter and "GameContext& ctx = GameContext::getInstance();" in line:
        if not ctx_defined_here:
            ctx_defined_here = True
            new_lines.append(line)
        else:
            # Skip duplicate definition
            pass
    elif "void update" in line or "void render" in line:
        in_on_enter = False
        new_lines.append(line)
    else:
        new_lines.append(line)
        
text = '\n'.join(new_lines)

# 5. Fix another trailing duplicate GameContext line in onEnter
text = re.sub(r'GameContext&\s+ctx\s*=\s*GameContext::getInstance\(\);\s*(?P<rest>earthMesh\s*=\s*MeshGen::sphere)', r'\g<rest>', text)

# We must ensure `ctx` exists above `earthMesh = ...`
text = re.sub(r'(void\s+onEnter\(\)\s+override\s*\{)', r'\1\n        GameContext& ctx = GameContext::getInstance();', text)
# And clean duplicates again
text = re.sub(r'(GameContext&\s+ctx\s*=\s*GameContext::getInstance\(\);\s*)(GameContext&\s+ctx\s*=\s*GameContext::getInstance\(\);\s*)+', r'\1', text)

with open(path, 'w', encoding=enc) as f:
    f.write(text)

print("Applied robust fixes")
