import re

HUD_FILE = r'c:\antigravity_code\RocketSim3D\src\render\HUD_system.h'
MAIN_FILE = r'c:\antigravity_code\RocketSim3D\src\main.cpp'

def patch():
    # 1. Update HUD_system.h
    with open(HUD_FILE, 'r', encoding='utf-8') as f:
        hud = f.read()

    # Includes
    includes_new = """#include "physics/physics_system.h" 
#include "simulation/rocket_builder.h"
#include "simulation/transfer_calculator.h"
#include "render/renderer3d.h"

// Forward declare formatTime from main.cpp
std::string formatTime(double seconds, bool absolute = false);
"""
    hud = hud.replace('#include "physics/physics_system.h" \n', includes_new)
    
    # Context
    context_old = "    CameraDirector* cam;\n    GLFWwindow* window;\n"
    context_new = """    CameraDirector* cam;
    GLFWwindow* window;
    const RocketAssembly* assembly;
    Renderer3D* r3d;
    int frame;
"""
    hud = hud.replace(context_old, context_new)
    
    # State vars
    state_old = "    int global_best_ref_node = -1;"
    state_new = """    int global_best_ref_node = -1;
    bool transfer_window_menu = false;
    int transfer_target_body = 4;
    TransferResult transfer_result;
    bool transfer_result_valid = false;
    int transfer_hover_dep = -1;
    int transfer_hover_tof = -1;
"""
    hud = hud.replace(state_old, state_new)

    # Local var unpack
    unpack_old = "        GLFWwindow* window = ctx.window;"
    unpack_new = """        GLFWwindow* window = ctx.window;
        const RocketAssembly& assembly = *ctx.assembly;
        Renderer3D* r3d = ctx.r3d;
        int frame = ctx.frame;
"""
    hud = hud.replace(unpack_old, unpack_new)
    
    # Quick fix for transfer vars calling from hud. -> since they are now in class FlightHUD we don't need 'hud.' but wait!
    # They were copied raw, there are no hud. prefixes in HUD_system.h for them anyway.

    with open(HUD_FILE, 'w', encoding='utf-8') as f:
        f.write(hud)


    # 2. Update main.cpp
    with open(MAIN_FILE, 'r', encoding='utf-8') as f:
        main_lines = f.readlines()
        
    main_new = []
    # Remove static transfer vars if they weren't removed
    remove_vars = ['transfer_window_menu', 'transfer_target_body', 'transfer_result', 'transfer_result_valid', 'transfer_hover_dep', 'transfer_hover_tof']
    
    for line in main_lines:
        skip = False
        for var in remove_vars:
            if re.search(r'\bstatic\s+.*?\b' + var + r'\b', line):
                skip = True
                break
        if skip:
            continue
            
        # Also map usages of transfer_* to hud.transfer_*
        # Note: the python script doesn't know context, but there shouldn't be any usages outside HUD for transfer_!
        # Actually wait, transfer_result might be used nowhere else.
        
        # Inject hud context
        if "hud_ctx.window = window;" in line:
            main_new.append("    hud_ctx.window = window;\n")
            main_new.append("    hud_ctx.assembly = &assembly;\n")
            main_new.append("    hud_ctx.r3d = r3d;\n")
            main_new.append("    hud_ctx.frame = frame;\n")
        else:
            main_new.append(line)
            
    with open(MAIN_FILE, 'w', encoding='utf-8') as f:
        f.writelines(main_new)

    print("Patched missing dependencies for FlightHUD")

if __name__ == '__main__':
    patch()
