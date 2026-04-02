import io

def fix_final_flight_errors():
    path = r'c:\antigravity_code\RocketSim3D\src\scene\flight_scene.h'
    # Use fallback mode reading to strictly avoid decode crashes
    with open(path, 'rb') as f:
        bts = f.read()
    try:
        text = bts.decode('utf-8')
        enc = 'utf-8'
    except:
        text = bts.decode('gbk', 'ignore')
        enc = 'gbk'

    # 1. Provide FlightHUD hud; declaration
    if "FlightHUD hud;" not in text:
        text = text.replace("SimulationController sim_ctrl;", "FlightHUD hud;\n    SimulationController sim_ctrl;")
        
    if '#include "render/HUD_system.h"' not in text:
        text = '#include "render/HUD_system.h"\n' + text

    # 2. ws_d is missing inside the render/update loops where it was stripped.
    # We declare it at class level if missing.
    if "double ws_d" not in text:
        text = text.replace("int frame = 0;", "int frame = 0;\n    double ws_d = 1.0;")

    # 3. Handle orbit_predictor in case the previous script failed 
    if "orbit_predictor." in text and "GameContext::getInstance().orbit_predictor" not in text:
        text = text.replace("orbit_predictor.", "GameContext::getInstance().orbit_predictor->")

    with open(path, 'w', encoding=enc) as f:
        f.write(text)

if __name__ == '__main__':
    fix_final_flight_errors()
    print("Applied ultimate flight_scene header patches")
