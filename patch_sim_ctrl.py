import sys
import re

MAIN_FILE = r'c:\antigravity_code\RocketSim3D\src\main.cpp'

def patch():
    with open(MAIN_FILE, 'r', encoding='utf-8') as f:
        content = f.read()

    # 1. Add include
    if '#include "simulation/simulation_controller.h"' not in content:
        content = content.replace('#include "render/HUD_system.h"', '#include "render/HUD_system.h"\n#include "simulation/simulation_controller.h"')

    # 2. Add sim_ctrl instance before main loop
    # Match where the main loop starts
    loop_start_marker = 'while (!glfwWindowShouldClose(window)) {'
    if 'SimulationController sim_ctrl;' not in content:
        content = content.replace(loop_start_marker, 'SimulationController sim_ctrl;\n    double last_time = glfwGetTime();\n\n    ' + loop_start_marker)

    # 3. Calculate real_dt at the start of loop
    # Match the beginning of the loop body
    body_marker = 'while (!glfwWindowShouldClose(window)) {\n'
    content = content.replace(body_marker, body_marker + '        double current_time = glfwGetTime();\n        double real_dt = current_time - last_time;\n        last_time = current_time;\n        if (real_dt > 0.1) real_dt = 0.02; // Limit spikes\n\n')

    # 4. Remove moved simulation blocks
    # We need to find the range. 
    # Starts from "// --- 时间加速逻辑 ---" (approx L1119)
    # Ends before "// 只有每隔一定帧数才打印" (approx L1405)
    
    start_pattern = r'// --- 时间加速逻辑 ---(.*?)// 只有每隔一定帧数才打印'
    
    # Replacement call
    replacement = """// --- Simulation & Physics Update ---
        sim_ctrl.handleInput(window, rocket_state);
        sim_ctrl.update(real_dt, rocket_state, rocket_config, control_input, hud, window);

        """
    
    content = re.sub(start_pattern, replacement + '// 只有每隔一定帧数才打印', content, flags=re.DOTALL)

    # 5. Remove the sleep at the end of loop
    content = content.replace('std::this_thread::sleep_for(std::chrono::milliseconds(20)); // 限制帧率', '// Frame rate limited by vsync/simulation')

    # 6. Cleanup some leftover statics that might collide
    content = content.replace('static int time_warp = 1;', '// time_warp moved to sim_ctrl')
    
    with open(MAIN_FILE, 'w', encoding='utf-8') as f:
        f.write(content)

if __name__ == "__main__":
    patch()
