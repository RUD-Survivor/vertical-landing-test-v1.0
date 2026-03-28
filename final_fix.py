import os
import re

MAIN_FILE = r'c:\antigravity_code\RocketSim3D\src\main.cpp'

def fix():
    with open(MAIN_FILE, 'r', encoding='utf-8') as f:
        content = f.read()

    # 1. Remove duplicate declarations
    content = content.replace('SimulationController sim_ctrl;\n  SimulationController sim_ctrl;', 'SimulationController sim_ctrl;')

    # 2. Fix the hud_ctx mapping error
    content = content.replace('hud_ctx.sim_ctrl.time_warp', 'hud_ctx.time_warp')

    with open(MAIN_FILE, 'w', encoding='utf-8') as f:
        f.write(content)

if __name__ == "__main__":
    fix()
