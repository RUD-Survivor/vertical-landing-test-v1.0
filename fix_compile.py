import os
import re

def fix_workshop():
    file_path = r'c:\antigravity_code\RocketSim3D\src\scene\workshop_scene.h'
    try:
        with open(file_path, 'r', encoding='utf-8') as f:
            content = f.read()
            enc = 'utf-8'
    except:
        with open(file_path, 'r', encoding='gbk') as f:
            content = f.read()
            enc = 'gbk'

    if "auto& agency_state = ctx.agency_state;" not in content:
        content = content.replace("Renderer* renderer = ctx.renderer2d;", 
                                  "Renderer* renderer = ctx.renderer2d;\n        auto& agency_state = ctx.agency_state;")

    with open(file_path, 'w', encoding=enc) as f:
        f.write(content)

def fix_flight():
    file_path = r'c:\antigravity_code\RocketSim3D\src\scene\flight_scene.h'
    try:
        with open(file_path, 'r', encoding='utf-8') as f:
            content = f.read()
            enc = 'utf-8'
    except:
        with open(file_path, 'r', encoding='gbk') as f:
            content = f.read()
            enc = 'gbk'

    content = content.replace("auto last_req_time", "std::chrono::steady_clock::time_point last_req_time")

    lines = content.split('\n')
    frame_count = 0
    new_lines = []
    for line in lines:
        if "int frame = 0;" in line and "class FlightScene" not in line:
            if frame_count == 0:
                new_lines.append(line)
                frame_count += 1
            else:
                pass 
        else:
            new_lines.append(line)
    content = '\n'.join(new_lines)

    if ("FlightHUD hud;" not in content):
        content = content.replace("CameraDirector cam;", "CameraDirector cam;\n    FlightHUD hud;")
    if '#include "render/HUD_system.h"' not in content:
        content = '#include "render/HUD_system.h"\n' + content

    if "Renderer3D* r3d =" not in content[:5000]: 
        content = content.replace("double real_dt = dt;", "double real_dt = dt;\n        Renderer3D* r3d = GameContext::getInstance().renderer3d;")

    with open(file_path, 'w', encoding=enc) as f:
        f.write(content)

if __name__ == '__main__':
    fix_workshop()
    fix_flight()
    print("Fixed C++ syntax errors.")
