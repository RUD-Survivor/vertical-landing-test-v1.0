#pragma once

#include "scene.h"
#include "scene_manager.h"
#include "game_context.h"
#include "menu_system.h"
#include "save_system.h"
#include <thread>
#include <chrono>

class MenuScene : public IScene {
public:
    int next_scene_flag = 0; // 0=none, 1=agency

    void onEnter() override {}

    void update(double dt) override {
        GameContext& ctx = GameContext::getInstance();
        GLFWwindow* window = ctx.window;
        Renderer* renderer = ctx.renderer2d;
        auto& agency_state = ctx.agency_state;
        auto& factory = ctx.factory;

        MenuSystem::MenuState menu_state;
        
        // --- 读取存档状态 ---
        menu_state.has_save = SaveSystem::HasSaveFile() || SaveSystem::HasAgencySave();
        if (SaveSystem::HasSaveFile()) {
            SaveSystem::GetSaveInfo(menu_state.save_time, menu_state.save_parts);
        } else {
            menu_state.save_time = 0; menu_state.save_parts = 0;
        }
        
        MenuSystem::MenuChoice menu_choice = MenuSystem::MENU_NONE;
        bool up_pressed = false, down_pressed = false, left_pressed = false, right_pressed = false, enter_pressed = false;

        // --- 核心循环 ---
        while (menu_choice != MenuSystem::MENU_EXIT && !glfwWindowShouldClose(window)) {
            glfwPollEvents();
            
            if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS && menu_choice != MenuSystem::MENU_SETTINGS) {
                glfwSetWindowShouldClose(window, true);
                break;
            }
            
            double mx, my;
            glfwGetCursorPos(window, &mx, &my);
            int ww, wh;
            glfwGetWindowSize(window, &ww, &wh);
            float mouse_x = (float)(mx / ww * 2.0 - 1.0);
            float mouse_y = (float)(1.0 - my / wh * 2.0);
            bool mouse_left = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS;

            if (menu_choice == MenuSystem::MENU_SETTINGS) {
                menu_choice = MenuSystem::HandleSettingsInput(window, menu_state, up_pressed, down_pressed, left_pressed, right_pressed, enter_pressed, mouse_x, mouse_y, mouse_left);
            } else {
                menu_choice = MenuSystem::HandleMenuInput(window, menu_state, up_pressed, down_pressed, enter_pressed, mouse_x, mouse_y, mouse_left);
                if (menu_choice != MenuSystem::MENU_NONE && menu_choice != MenuSystem::MENU_SETTINGS && menu_choice != MenuSystem::MENU_EXIT) {
                    // 有效的选择 => 跳出当前页
                    break;
                }
            }
            
            glClearColor(0.02f, 0.03f, 0.08f, 1.0f);
            glClear(GL_COLOR_BUFFER_BIT);
            renderer->beginFrame();
            if (menu_choice == MenuSystem::MENU_SETTINGS) {
                MenuSystem::DrawSettingsMenu(renderer, menu_state, (float)glfwGetTime());
            } else {
                MenuSystem::DrawMainMenu(renderer, menu_state, (float)glfwGetTime());
            }
            renderer->endFrame();
            glfwSwapBuffers(window);
            
            std::this_thread::sleep_for(std::chrono::milliseconds(16));
        }

        // --- 退出或跳转逻辑 ---
        if (menu_choice == MenuSystem::MENU_EXIT || glfwWindowShouldClose(window)) {
            return; 
        }

        bool load_from_save = (menu_choice == MenuSystem::MENU_CONTINUE);
        
        if (!load_from_save) { 
            // 如果是新游戏且有旧档，询问/删除 (逻辑简化：直接删)
            if (SaveSystem::HasSaveFile()) SaveSystem::DeleteSaveFile();
            // 重置 Agency
            agency_state = AgencyState();
            factory = FactorySystem();
            
            // Starter resources
            agency_state.funds = 100000.0;
            agency_state.addItem(ITEM_IRON_ORE, 50);
            agency_state.addItem(ITEM_COPPER_ORE, 30);
            agency_state.addItem(ITEM_COAL, 40);
            agency_state.addItem(ITEM_STEEL, 10);
            agency_state.addItem(PART_ENGINE, 2);
            agency_state.addItem(PART_FUEL_TANK, 4);
            agency_state.addItem(PART_NOSECONE, 2);
            agency_state.addItem(PART_STRUCTURAL, 5);
            agency_state.addItem(PART_COMMAND_POD, 1);

            factory.addNode(NODE_MINER, 0, 0);
            factory.addNode(NODE_MINER, 1, 0);
            factory.addNode(NODE_SMELTER, 3, 0);
            factory.addNode(NODE_STORAGE, 4, 1);
            ConveyorBelt b1; b1.from_node_id = 1; b1.to_node_id = 3; factory.belts.push_back(b1);
            ConveyorBelt b2; b2.from_node_id = 2; b2.to_node_id = 3; factory.belts.push_back(b2);
            ConveyorBelt b3; b3.from_node_id = 3; b3.to_node_id = 4; factory.belts.push_back(b3);
            
            next_scene_flag = 1; // 1 = 去 AgencyScene
        } else { 
            // 读档
            if (SaveSystem::HasAgencySave()) {
                SaveSystem::LoadAgencyFactory(agency_state, factory);
            }
            ctx.skip_builder = true; 
            next_scene_flag = 1;
        }
    }

    void render() override {}
};
