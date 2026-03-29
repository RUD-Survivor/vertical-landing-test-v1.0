import os
import re

SCENE_FILE = r'c:\antigravity_code\RocketSim3D\src\scene\flight_scene.h'

def wrap_in_class():
    with open(SCENE_FILE, 'r', encoding='utf-8') as f:
        content = f.read()
    
    # 1. First, check if already wrapped.
    if 'class FlightScene' in content:
        print("Already wrapped!")
        return
        
    # We will build a new header file string
    new_header = """#pragma once

#include "scene.h"
#include "game_context.h"
#include "simulation/simulation_controller.h"
#include "camera/camera_director.h"
#include "simulation/orbit_physics.h"
#include "simulation/predictor.h"
#include "simulation/center_calculator.h"

// Any other includes you need

class FlightScene : public IScene {
public:
"""
    
    # 2. Extract static variables from the loop and convert to class members
    #    This avoids having them persist after the scene unloads.
    static_vars = []
    
    def replace_static(m):
        var_stmt = m.group(1).strip()
        # Add to our list, stripping the standard initialization
        static_vars.append(var_stmt)
        return "" # Remove it from local scope
        
    content = re.sub(r'static\s+([^=;]+(?:=[^;]+)?);', replace_static, content)
    
    # Let's write the variable block
    # State references
    member_block = """
    // === Core Members ===
    RocketState rocket_state;
    ControlInput control_input;
    RocketConfig rocket_config;
    CameraDirector cam;
    SimulationController sim_ctrl;
    double dt = 0.02;
    int frame = 0;
    
    // Track 3D position history
    struct DVec3 { double x, y, z; };
    struct TrajPoint { DVec3 e; DVec3 s; };
    std::vector<TrajPoint> traj_history; 
    
    // === Variables Extracted from Static ===
"""
    
    for v in static_vars:
        # e.g. "float global_best_ang = -1.0f"
        # Since some might be comma separated, they will just be kept as is.
        member_block += f"    {v};\n"

    new_header += member_block
    
    # 3. Build constructor / onEnter
    on_enter_start = """
    void onEnter() override {
        GameContext& ctx = GameContext::getInstance();
        
        bool skip_builder = ctx.skip_builder;
        auto& builder_state_assembly = ctx.launch_assembly; // Assume this was the passed payload
        auto& agency_state = ctx.agency_state;
        auto& loaded_state = ctx.loaded_rocket_state;
        auto& loaded_input = ctx.loaded_control_input;
        
        // --- Original Init Code below ---
"""
    
    # Split the original content by the main while loop 
    # to separate `onEnter` code from `update` and `render` code.
    parts = content.split("while (!glfwWindowShouldClose(window)) {")
    if len(parts) < 2:
        print("Error: Could not find main while loop marker.")
        return
        
    on_enter_code = parts[0]
    
    # We must patch variable usages that no longer exist implicitly
    on_enter_code = on_enter_code.replace("builder_state.assembly", "builder_state_assembly")
    on_enter_code = on_enter_code.replace("RocketState rocket_state;", "")
    on_enter_code = on_enter_code.replace("ControlInput control_input;", "")
    on_enter_code = on_enter_code.replace("RocketConfig rocket_config =", "rocket_config =")
    on_enter_code = on_enter_code.replace("SimulationController sim_ctrl;", "")
    on_enter_code = on_enter_code.replace("CameraDirector cam;", "")
    on_enter_code = on_enter_code.replace("double dt = 0.02;", "")
    on_enter_code = on_enter_code.replace("int frame = 0;", "")
    on_enter_code = re.sub(r'struct DVec3.*?;', '', on_enter_code)
    on_enter_code = re.sub(r'struct TrajPoint.*?;', '', on_enter_code)
    on_enter_code = on_enter_code.replace("std::vector<TrajPoint> traj_history;", "")
    
    new_header += on_enter_start + on_enter_code + "\n    }\n\n"
    
    # 4. Extract update and render
    # Inside the while loop, Physics/sim code goes into `update`, drawing code to `render`
    # We will just roughly split it where r3d rendering starts.
    loop_code = parts[1]
    
    loop_parts = loop_code.split("r3d->updateCamera(")
    
    if len(loop_parts) < 2:
        update_code = loop_code
        render_code = ""
    else:
        update_code = loop_parts[0]
        # Prepend what we split by
        render_code = "        Renderer3D* r3d = GameContext::getInstance().renderer3d;\n        r3d->updateCamera(" + loop_parts[1]
    
    # Clean up glfw logic
    update_code = update_code.replace("glfwPollEvents();", "")
    update_code = update_code.replace("if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)", 
                                    "// Scene escape logic handled externally or here\n        if (glfwGetKey(GameContext::getInstance().window, GLFW_KEY_ESCAPE) == GLFW_PRESS)")
    update_code = update_code.replace("window", "GameContext::getInstance().window")
    render_code = render_code.replace("window", "GameContext::getInstance().window")
    # Clean time logic because SceneManager passes dt
    update_code = re.sub(r'double current_time = [^;]+;\s*double real_dt = [^;]+;\s*last_time = [^;]+;\s*if \(real_dt > 0\.1[^;]+;', '', update_code)

    
    new_header += "    void update(double dt) override {\n"
    new_header += "        double real_dt = dt;\n"
    new_header += update_code
    new_header += "\n    }\n\n"
    
    # Look for missing HUD code
    if "hud.render(" not in render_code:
        # User missed pasting the end of the loop!
        render_code += """
        // --- 2D HUD Rendering ---
        FlightHUD hud; // Note: In full extraction, hud is a class member.
        HUDContext hud_ctx;
        hud_ctx.renderer = GameContext::getInstance().renderer2d;
        hud_ctx.rocket_state = &rocket_state;
        hud_ctx.rocket_config = &rocket_config;
        hud_ctx.control_input = &control_input;
        hud_ctx.cam = &cam;
        int ww, wh;
        glfwGetWindowSize(GameContext::getInstance().window, &ww, &wh);
        hud_ctx.ww = ww; hud_ctx.wh = wh;
        hud_ctx.aspect = (float)ww / (float)wh;
        // ... (Many matrix properties stripped here for brevity, needs manual linking)
        hud_ctx.time_warp = sim_ctrl.time_warp;
        hud_ctx.dt = dt;
        hud_ctx.window = GameContext::getInstance().window;
        hud_ctx.assembly = &GameContext::getInstance().launch_assembly;
        hud_ctx.r3d = r3d;
        hud_ctx.frame = frame;
        hud_ctx.ws_d = 1.0; // placeholder for ws_d

        hud.render(hud_ctx);
        // renderer->endFrame();
"""

    
    new_header += "    void render() override {\n"
    # Remove terminating braces of the previous while loop if they got caught
    render_code = render_code.replace("    }\n", "") 
    render_code = render_code.replace("        r3d->endFrame();", "        r3d->endFrame();\n")
    
    new_header += render_code
    new_header += "\n    }\n"
    
    new_header += "};\n"
    
    # Save the modified code
    with open(SCENE_FILE, 'w', encoding='utf-8') as f:
        f.write(new_header)
        
    print("Wrapped flight_scene correctly!")

if __name__ == "__main__":
    wrap_in_class()
