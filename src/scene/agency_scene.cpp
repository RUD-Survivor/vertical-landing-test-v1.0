#include "scene/agency_scene.h"
#include "scene/game_context.h"
#include "scene/scene_manager.h"
#include "scene/workshop_scene.h"
#include "save_system.h"
#include <GLFW/glfw3.h>

void AgencyScene::onEnter() {
    auto& ctx = GameContext::getInstance();
    menu_choice = MenuSystem::MENU_AGENCY_HUB;
    // Loading is already done conditionally by MenuScene.
}

void AgencyScene::update(double dt) {
    auto& ctx = GameContext::getInstance();
    auto window = ctx.window;
    
    // The agency loop logic
              while (menu_choice == MenuSystem::MENU_AGENCY_HUB && !glfwWindowShouldClose(window)) {
                  glfwPollEvents();
                  double mx, my; glfwGetCursorPos(window, &mx, &my);
                  int ww, wh; glfwGetWindowSize(window, &ww, &wh);
                  float mouse_x = (float)(mx / ww * 2.0 - 1.0);
                  float mouse_y = (float)(1.0 - my / wh * 2.0);
                  bool mouse_left = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS;

                  menu_choice = MenuSystem::HandleAgencyHubInput(window, menu_state, up_pressed, down_pressed, enter_pressed, mouse_x, mouse_y, mouse_left);
                  factory.tick(0.016f, agency_state);
                  
                  glClearColor(0.02f, 0.03f, 0.08f, 1.0f); glClear(GL_COLOR_BUFFER_BIT);
                  renderer->beginFrame();
                  MenuSystem::DrawAgencyHub(renderer, menu_state, agency_state, (float)glfwGetTime(), factory);
                  renderer->endFrame();
                  glfwSwapBuffers(window);
                  std::this_thread::sleep_for(std::chrono::milliseconds(16));
              }
              SaveSystem::SaveAgencyFactory(agency_state, factory);
          }
          
          if (menu_choice == MenuSystem::MENU_FACTORY) {
              FactoryUIState factory_ui; g_scroll_y = 0.0f; bool in_factory = true;
              while (in_factory && !glfwWindowShouldClose(window)) {
                  glfwPollEvents();
                  double fmx, fmy; glfwGetCursorPos(window, &fmx, &fmy);
                  int fww, fwh; glfwGetWindowSize(window, &fww, &fwh);
                  float mouse_x = (float)(fmx / fww * 2.0 - 1.0);
                  float mouse_y = (float)(1.0 - fmy / fwh * 2.0);
                  bool mouse_left = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS;
                  float scroll = g_scroll_y; g_scroll_y = 0.0f;
                  in_factory = HandleFactoryInput(window, factory_ui, factory, agency_state, mouse_x, mouse_y, mouse_left, scroll);
                  factory.tick(0.016f, agency_state);
                  glClearColor(0.04f, 0.05f, 0.07f, 1.0f); glClear(GL_COLOR_BUFFER_BIT);
                  renderer->beginFrame(); DrawFactoryUI(renderer, factory_ui, factory, agency_state, (float)glfwGetTime()); renderer->endFrame();
                  glfwSwapBuffers(window); std::this_thread::sleep_for(std::chrono::milliseconds(16));
              }
              SaveSystem::SaveAgencyFactory(agency_state, factory);
              menu_choice = MenuSystem::MENU_AGENCY_HUB;
              while (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS) glfwPollEvents();
          }
          
          if (menu_choice == MenuSystem::MENU_VAB || menu_choice == MenuSystem::MENU_CONTINUE) {
              break; // 玩家点击了“进入 VAB”或“继续”，跳出 HUB 循环，进入 3D 建造/飞行。
          }


    if (menu_choice == MenuSystem::MENU_VAB || menu_choice == MenuSystem::MENU_CONTINUE) {
        // Advance to Workshop
        SceneManager::getInstance().changeScene(std::make_unique<WorkshopScene>());
    } else if (menu_choice == MenuSystem::MENU_NONE || glfwWindowShouldClose(window)) {
        // Note: if user fully quit, we exit.
        glfwSetWindowShouldClose(window, true);
    }
}

void AgencyScene::render() {
    // Rendering is handled directly inside update() from main.cpp logic, 
    // to decouple it we can just call the render chunks here. 
    // But since the original logic embedded glClear and SwapBuffers inside the loop, we leave it empty for now.
}
