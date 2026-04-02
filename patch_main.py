import os

MAIN_FILE = r'c:\antigravity_code\RocketSim3D\src\main.cpp'

def patch_main():
    with open(MAIN_FILE, 'r', encoding='utf-8') as f:
        content = f.read()

    # Find the top part to keep
    idx_start = content.find('  orbit_predictor.Start();')
    if idx_start == -1:
        print("Could not find start marker!")
        return

    idx_start += len('  orbit_predictor.Start();\n')
    
    # We will also insert Scene headers near the top
    top_part = content[:idx_start]
    
    # Insert headers
    includes_to_add = """#include "scene/scene_manager.h"
#include "scene/game_context.h"
#include "scene/menu_scene.h"
#include "scene/agency_scene.h"
#include "scene/workshop_scene.h"
#include "scene/flight_scene.h"
"""
    top_part = top_part.replace('#include "simulation/factory_ui.h"', '#include "simulation/factory_ui.h"\n' + includes_to_add)

    # Build the new SceneManager loop
    new_loop = """
  // =========================================================
  // SceneManager State Machine Loop
  // =========================================================
  GameContext& ctx = GameContext::getInstance();
  ctx.window = window;
  ctx.renderer2d = renderer;

  SceneManager::getInstance().changeScene(std::make_unique<MenuScene>());

  double last_time = glfwGetTime();

  while (!glfwWindowShouldClose(window)) {
      double current_time = glfwGetTime();
      double real_dt = current_time - last_time;
      last_time = current_time;
      if (real_dt > 0.1) real_dt = 0.02;

      // Handle Scene Transitions based on flags
      IScene* cur = SceneManager::getInstance().getCurrentScene();
      if (cur) {
          if (auto* menu = dynamic_cast<MenuScene*>(cur)) {
              if (menu->next_scene_flag == 1) {
                  menu->next_scene_flag = 0;
                  SceneManager::getInstance().changeScene(std::make_unique<AgencyScene>());
              }
          } else if (auto* agency = dynamic_cast<AgencyScene*>(cur)) {
              if (agency->next_scene_flag == 1) {
                  agency->next_scene_flag = 0;
                  SceneManager::getInstance().changeScene(std::make_unique<WorkshopScene>());
              }
          } else if (auto* workshop = dynamic_cast<WorkshopScene*>(cur)) {
              if (workshop->done) {
                  workshop->done = false;
                  SceneManager::getInstance().changeScene(std::make_unique<FlightScene>());
              }
          }
      }

      SceneManager::getInstance().update(real_dt);
      
      // If window got closed by a scene
      if (glfwWindowShouldClose(window)) break;

      SceneManager::getInstance().render();
  }

  delete renderer;
  if (ctx.renderer3d) delete ctx.renderer3d;
  orbit_predictor.Stop();

  glfwTerminate();
  return 0;
}
"""

    with open(MAIN_FILE, 'w', encoding='utf-8') as f:
        f.write(top_part + new_loop)
        
    print("Rewrote main.cpp")

if __name__ == "__main__":
    patch_main()
