import os

AGENCY_FILE = r'c:\antigravity_code\RocketSim3D\src\scene\agency_scene.h'
WORKSHOP_FILE = r'c:\antigravity_code\RocketSim3D\src\scene\workshop_scene.h'

def read_file(filepath):
    try:
        with open(filepath, 'r', encoding='utf-8') as f:
            return f.read(), 'utf-8'
    except UnicodeDecodeError:
        with open(filepath, 'r', encoding='gbk') as f:
            return f.read(), 'gbk'

def patch_agency():
    content, enc = read_file(AGENCY_FILE)
    if 'class AgencyScene' in content:
        return
        
    new_header = """#pragma once
#include "scene.h"
#include "game_context.h"
#include "menu_system.h"
#include "save_system.h"
#include "simulation/factory_ui.h"
#include <thread>
#include <chrono>

extern float g_scroll_y;

class AgencyScene : public IScene {
public:
    int next_scene_flag = 0; // 0=none, 1=workshop

    void onEnter() override {}

    void update(double dt) override {
        GameContext& ctx = GameContext::getInstance();
        GLFWwindow* window = ctx.window;
        auto& agency_state = ctx.agency_state;
        auto& factory = ctx.factory;
        Renderer* renderer = ctx.renderer2d;

        MenuSystem::MenuState menu_state;
        MenuSystem::MenuChoice menu_choice = MenuSystem::MENU_AGENCY_HUB;
        bool up_pressed = false, down_pressed = false, enter_pressed = false;

"""
    new_header += content
    new_header += """
        if (menu_choice == MenuSystem::MENU_VAB || menu_choice == MenuSystem::MENU_CONTINUE) {
            ctx.skip_builder = (menu_choice == MenuSystem::MENU_CONTINUE);
            next_scene_flag = 1; // ready to switch
        }
    }

    void render() override {}
};
"""
    with open(AGENCY_FILE, 'w', encoding=enc) as f:
        f.write(new_header)

def patch_workshop():
    content, enc = read_file(WORKSHOP_FILE)
    if 'class WorkshopScene' in content:
        return
        
    new_header = """#pragma once
#include "scene.h"
#include "game_context.h"
#include "menu_system.h"
#include "save_system.h"
#include "simulation/rocket_builder.h"

extern float g_scroll_y;

class WorkshopScene : public IScene {
public:
    bool done = false;

    void update(double dt) override {
        GameContext& ctx = GameContext::getInstance();
        GLFWwindow* window = ctx.window;
        Renderer* renderer = ctx.renderer2d;
        
        bool load_from_save = ctx.skip_builder;
        MenuSystem::MenuChoice menu_choice = load_from_save ? MenuSystem::MENU_CONTINUE : MenuSystem::MENU_VAB;

"""
    new_header += content
    new_header += """
        // Store models that we don't want to destroy into context or destroy them
        // FlightScene expects Renderer3D
        ctx.launch_assembly = builder_state.assembly;
        ctx.skip_builder = skip_builder;
        ctx.loaded_rocket_state = loaded_state;
        ctx.loaded_control_input = loaded_input;
        ctx.renderer3d = r3d;
        
        // Let flight scene destroy Earth and Ring meshes or initialize its own
        
        done = true;
    }

    void render() override {}
};
"""
    with open(WORKSHOP_FILE, 'w', encoding=enc) as f:
        f.write(new_header)

if __name__ == "__main__":
    patch_agency()
    patch_workshop()
    print("Patched agency and workshop")
