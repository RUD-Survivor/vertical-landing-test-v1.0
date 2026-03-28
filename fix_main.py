import os

MAIN_FILE = r'c:\antigravity_code\RocketSim3D\src\main.cpp'

def fix_main_cpp():
    with open(MAIN_FILE, 'r', encoding='utf-8') as f:
        content = f.read()

    # Match the update call and add cam.mode
    content = content.replace('sim_ctrl.update(real_dt, rocket_state, rocket_config, control_input, hud, window);', 
                            'sim_ctrl.update(real_dt, rocket_state, rocket_config, control_input, hud, window, cam.mode);')

    # Fix remaining time_warp usages to sim_ctrl.time_warp
    # We should use word boundary to avoid partial replacing
    content = re.sub(r'\btime_warp\b', 'sim_ctrl.time_warp', content)
    
    # Wait, sim_ctrl.sim_ctrl.time_warp might happen if I rerun.
    # Actually, the python script uses regex, so it's safer.
    content = content.replace('sim_ctrl.sim_ctrl.time_warp', 'sim_ctrl.time_warp')

    with open(MAIN_FILE, 'w', encoding='utf-8') as f:
        f.write(content)

if __name__ == "__main__":
    import re
    fix_main_cpp()
