import re
import os

MAIN_FILE = r'c:\antigravity_code\RocketSim3D\src\main.cpp'
HUD_FILE = r'c:\antigravity_code\RocketSim3D\src\render\HUD_system.h'

static_vars_to_move = [
    'show_galaxy_info', 'selected_body_idx', 'expanded_planet_idx', 'hlmb_prev_galaxy',
    'mnv_popup_visible', 'mnv_popup_index', 'mnv_popup_px', 'mnv_popup_py', 'mnv_popup_pw', 'mnv_popup_ph',
    'mnv_popup_node_scr_x', 'mnv_popup_node_scr_y', 'mnv_popup_dv', 'mnv_popup_close_hover', 'mnv_popup_del_hover',
    'mnv_popup_time_to_node', 'mnv_popup_burn_time', 'mnv_popup_ref_body', 'mnv_popup_slider_dragging',
    'mnv_popup_slider_drag_x', 'mnv_popup_burn_mode', 'mnv_popup_mode_hover', 'adv_mnv_world_pos',
    'adv_orbit_enabled', 'adv_orbit_menu', 'adv_orbit_pred_days', 'adv_orbit_iters', 'adv_orbit_ref_mode',
    'adv_orbit_ref_body', 'adv_orbit_secondary_ref_body', 'adv_warp_to_node', 'auto_exec_mnv',
    'flight_assist_menu', 'adv_embed_mnv', 'adv_embed_mnv_mini', 'mnv_popup_mini_hover',
    'show_hud', 'show_orbit', 'orbit_reference_sun', 'global_best_ref_node'
]

def refactor():
    with open(MAIN_FILE, 'r', encoding='utf-8') as f:
        # readlines to preserve exactly the line endings 
        lines = f.readlines()

    # Find the HUD block
    start_idx = -1
    end_idx = -1
    for i, line in enumerate(lines):
        if '// ================= 2D HUD 叠加层 =================' in line:
            start_idx = i
        if '渲染2DHUD逻辑结束' in line:
            end_idx = i + 1

    if start_idx == -1 or end_idx == -1:
        print("Could not find HUD boundaries")
        return

    # Extract HUD logic
    hud_logic_lines = lines[start_idx+3 : end_idx-3] 
    # adjust indices to just get the logic inside the block
    # +3: skips the 3 comment lines
    # -3: skips the end comments 
    
    # We also need to extract static variable definitions and remove them from lines
    extracted_statics = []
    lines_to_keep = []
    i = 0
    while i < len(lines):
        line = lines[i]
        skip_line = False
        
        # Don't process lines inside the HUD block for static removal, wait, the static vars are before the main loop
        if i < start_idx or i > end_idx:
            for var in static_vars_to_move:
                # Regex to match "static type varname" avoiding partial matches
                if re.search(r'\bstatic\s+.*?\b' + var + r'\b', line):
                    skip_line = True
                    extracted_statics.append(line.strip())
                    break
        
        if not skip_line:
            lines_to_keep.append(line)
        i += 1

    # Now replace the HUD block with the hud.render() call
    new_main_lines = []
    i = 0
    while i < len(lines_to_keep):
        line = lines_to_keep[i]
        if '// ================= 2D HUD 叠加层 =================' in line:
            new_main_lines.append(line)
            new_main_lines.append(lines_to_keep[i+1]) # second comment line
            new_main_lines.append(lines_to_keep[i+2]) # third comment line
            new_main_lines.append("""
    HUDContext hud_ctx;
    hud_ctx.renderer = renderer;
    hud_ctx.rocket_state = &rocket_state;
    hud_ctx.rocket_config = &rocket_config;
    hud_ctx.control_input = &control_input;
    hud_ctx.cam = &cam;
    hud_ctx.ww = ww;
    hud_ctx.wh = wh;
    hud_ctx.ro_x = ro_x;
    hud_ctx.ro_y = ro_y;
    hud_ctx.ro_z = ro_z;
    hud_ctx.viewMat = viewMat;
    hud_ctx.macroProjMat = macroProjMat;
    hud_ctx.camEye_rel = camEye_rel;
    hud_ctx.aspect = aspect;
    hud_ctx.time_warp = time_warp;
    hud_ctx.dt = dt;
    double mx, my; glfwGetCursorPos(window, &mx, &my);
    hud_ctx.mouse_x = (mx / ww * 2.0 - 1.0);
    hud_ctx.mouse_y = (1.0 - my / wh * 2.0);
    hud_ctx.lmb = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);
    static bool lmb_prev = false;
    hud_ctx.lmb_prev = lmb_prev;
    lmb_prev = hud_ctx.lmb;
    hud_ctx.rmb = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS);
    hud_ctx.window = window;

    hud.render(hud_ctx);
""")
            
            # fast forward i to end_idx-3
            # Wait, lines_to_keep has already removed the static vars, so indices changed.
            # I must skip the lines belonging to the HUD logic.
            # In lines_to_keep, the HUD logic still exists because I only skipped static lines.
            j = i
            while j < len(lines_to_keep) and '渲染2DHUD逻辑结束' not in lines_to_keep[j]:
                j += 1
            i = j # i is now at the '渲染2DHUD逻辑结束' line
            new_main_lines.append(lines_to_keep[i])
            if i+1 < len(lines_to_keep): # endFrame
                 new_main_lines.append(lines_to_keep[i+1])
            i += 1
        else:
            new_main_lines.append(line)
        i += 1

    # Before writing, we need to globally declare `FlightHUD hud` near Renderer* renderer;
    # And we need to replace instances of `static_var` usage in the rest of main.cpp with `hud.static_var`
    # e.g., auto_exec_mnv -> hud.auto_exec_mnv
    
    # We will do regex substitution for all the static vars moved to hud
    final_main_lines = []
    for line in new_main_lines:
        new_line = line
        # Check if line contains '#include'
        if not new_line.strip().startswith('#include'):
            for var in static_vars_to_move:
                # Replace whole word matches ONLY
                # But don't replace if it's already hud.var
                new_line = re.sub(r'(?<!hud\.)\b' + var + r'\b', 'hud.' + var, new_line)
        final_main_lines.append(new_line)
        
    # Insert #include "render/HUD_system.h" and `FlightHUD hud;` near Renderer *renderer;
    for i, line in enumerate(final_main_lines):
        if 'Renderer *renderer;' in line or 'Renderer* renderer;' in line:
            final_main_lines.insert(i, '#include "render/HUD_system.h"\nFlightHUD hud;\n')
            break

    # Construct HUD_system.h
    hud_h = """#pragma once
#include <vector>
#include <string>
#include <cmath>
#include <algorithm>
#include <GLFW/glfw3.h>
#include "render/renderer_2d.h"
#include "core/rocket_state.h"
#include "camera/camera_director.h"
#include "math/math3d.h"
#include "physics/physics_system.h" 

struct HUDContext {
    Renderer* renderer;
    RocketState* rocket_state;
    RocketConfig* rocket_config;
    ControlInput* control_input;
    CameraDirector* cam;
    GLFWwindow* window;

    int ww;
    int wh;
    double ro_x;
    double ro_y;
    double ro_z;
    Mat4 viewMat;
    Mat4 macroProjMat;
    Vec3 camEye_rel;
    float aspect;

    int time_warp;
    double dt;

    double mouse_x;
    double mouse_y;
    bool lmb;
    bool lmb_prev;
    bool rmb;
};

class FlightHUD {
public:
"""
    # Define statics as class members
    # Ensure they are initialized
    statics_clean = [
        "bool show_galaxy_info = false;",
        "int selected_body_idx = -1;",
        "int expanded_planet_idx = -1;",
        "bool hlmb_prev_galaxy = false;",
        "bool mnv_popup_visible = false;",
        "int mnv_popup_index = -1;",
        "float mnv_popup_px = 0, mnv_popup_py = 0, mnv_popup_pw = 0, mnv_popup_ph = 0;",
        "float mnv_popup_node_scr_x = 0, mnv_popup_node_scr_y = 0;",
        "Vec3 mnv_popup_dv = Vec3(0,0,0);",
        "bool mnv_popup_close_hover = false, mnv_popup_del_hover = false;",
        "double mnv_popup_time_to_node = 0;",
        "double mnv_popup_burn_time = 0;",
        "int mnv_popup_ref_body = -1;",
        "int mnv_popup_slider_dragging = -1;",
        "float mnv_popup_slider_drag_x = 0;",
        "int mnv_popup_burn_mode = 0;",
        "bool mnv_popup_mode_hover = false;",
        "Vec3 adv_mnv_world_pos = Vec3(0,0,0);",
        "bool adv_orbit_enabled = false;",
        "bool adv_orbit_menu = false;",
        "float adv_orbit_pred_days = 30.0f;",
        "int adv_orbit_iters = 4000;",
        "int adv_orbit_ref_mode = 0;",
        "int adv_orbit_ref_body = 3;",
        "int adv_orbit_secondary_ref_body = 4;",
        "bool adv_warp_to_node = false;",
        "bool auto_exec_mnv = false;",
        "bool flight_assist_menu = false;",
        "bool adv_embed_mnv = false;",
        "bool adv_embed_mnv_mini = false;",
        "bool mnv_popup_mini_hover = false;",
        "bool show_hud = true;",
        "bool show_orbit = true;",
        "bool orbit_reference_sun = false;",
        "int global_best_ref_node = -1;"
    ]
    for s in statics_clean:
        hud_h += "    " + s + "\n"

    hud_h += """

    void render(HUDContext& ctx) {
        // Unpack references to avoid changing the hud code too much
        Renderer* renderer = ctx.renderer;
        RocketState& rocket_state = *ctx.rocket_state;
        RocketConfig& rocket_config = *ctx.rocket_config;
        ControlInput& control_input = *ctx.control_input;
        CameraDirector& cam = *ctx.cam;
        GLFWwindow* window = ctx.window;

        int ww = ctx.ww;
        int wh = ctx.wh;
        double ro_x = ctx.ro_x;
        double ro_y = ctx.ro_y;
        double ro_z = ctx.ro_z;
        Mat4 viewMat = ctx.viewMat;
        Mat4 macroProjMat = ctx.macroProjMat;
        Vec3 camEye_rel = ctx.camEye_rel;
        float aspect = ctx.aspect;

        int time_warp = ctx.time_warp;
        double dt = ctx.dt;

        float mouse_x = (float)ctx.mouse_x;
        float mouse_y = (float)ctx.mouse_y;
        bool lmb = ctx.lmb;
        bool lmb_prev = ctx.lmb_prev;
        bool rmb = ctx.rmb;

"""
    for line in hud_logic_lines:
        line_clean = line
        # Avoid substituting `this->` explicitly if the raw code didn't have it.
        # However, the script removed the `static` declaration internally.
        # But wait, we shouldn't replace `adv_orbit_enabled` with `hud.adv_orbit_enabled`
        # in the HUD_system.h, because inside class FlightHUD it is just `adv_orbit_enabled`.
        # The HUD logic lines came from main.cpp *before* we applied `hud.` to main.cpp.
        # So we just copy them straight over!
        hud_h += line_clean
        
    hud_h += """
    }
};
"""

    with open(MAIN_FILE, 'w', encoding='utf-8') as f:
        f.writelines(final_main_lines)
        
    with open(HUD_FILE, 'w', encoding='utf-8') as f:
        f.write(hud_h)

    print("Refactoring complete.")

if __name__ == '__main__':
    refactor()
