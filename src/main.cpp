// Build optimization test
#include<glad/glad.h>
#include <GLFW/glfw3.h>
#include <algorithm>
#include <chrono>
#include <thread>
#include <cmath>
#include <functional>
#include <mutex>

#include "math/math3d.h"
#include "render/renderer3d.h"
#include "render/model_loader.h"
#include "core/rocket_state.h"
#include "physics/physics_system.h"
#include "control/control_system.h"
#include "render/renderer_2d.h"
#include "simulation/orbit_physics.h"
#include "simulation/predictor.h"
#include "simulation/stage_manager.h"
#include "simulation/maneuver_system.h"
#include "simulation/transfer_calculator.h"
#include "math/spline.h"
#include "math/chebyshev.h"  // Leaving for reference but not using for rendering
#include "camera/camera_director.h"

using namespace std;

// ==========================================
// 全局变量与系统初始化 (Global Systems)
// ==========================================

// 鼠标滚轮全局变量：用于在编辑器里缩放或上下拖动。
static float g_scroll_y = 0.0f;
static void scroll_callback(GLFWwindow* /*w*/, double /*xoffset*/, double yoffset) {
  g_scroll_y += (float)yoffset;
}

#include "simulation/rocket_builder.h"
#include "simulation/center_calculator.h"
#include "simulation/center_visualizer.h"
#include "save_system.h"
#include "menu_system.h"
#include "simulation/factory_system.h"
#include "simulation/factory_ui.h"
#include "scene/scene_manager.h"
#include "scene/game_context.h"
#include "scene/menu_scene.h"
#include "scene/agency_scene.h"
#include "scene/workshop_scene.h"
#include "scene/flight_scene.h"


// ==========================================
// Part 2: Explorer Class -> Replaced by ECS (RocketState, PhysicsSystem)
// ==========================================


void framebuffer_size_callback(GLFWwindow *window, int width, int height) {
  glViewport(0, 0, width, height);
}
//函数用于处理窗口大小变化的事件。
//当窗口大小发生变化时，该函数会自动调整OpenGL的视口，以确保渲染输出正确地适应新的窗口大小。

#include "render/HUD_system.h"
#include "simulation/simulation_controller.h"
FlightHUD hud;
Renderer *renderer;


// ==========================================================
// Part 3: 火箭建造系统 (Rocket Builder) -> see rocket_builder.h
// ==========================================================

// drawBuilderUI -> replaced by drawBuilderUI_KSP in rocket_builder.h

void Report_Status(const RocketState& state, const ControlInput& input) {
  // ... (unchanged)
}
// ==========================================================
// 计时器格式打印
// ==========================================================
std::string formatTime(double seconds, bool absolute) {
    if (absolute) {
        // UT: Day 000, HH:MM:SS
        int day = (int)(seconds / (24 * 3600));
        int hour = (int)((seconds - day * 24 * 3600) / 3600);
        int min = (int)((seconds - day * 24 * 3600 - hour * 3600) / 60);
        int sec = (int)(seconds - day * 24 * 3600 - hour * 3600 - min * 60);
        char buf[64];
        snprintf(buf, sizeof(buf), "UT DAY %03d, %02d:%02d:%02d", day, hour, min, sec);
        return string(buf);
    } else {
        // MET: T+ DD:HH:MM:SS
        int day = (int)(seconds / (24 * 3600));
        int hour = (int)((seconds - day * 24 * 3600) / 3600);
        int min = (int)((seconds - day * 24 * 3600 - hour * 3600) / 60);
        int sec = (int)(seconds - day * 24 * 3600 - hour * 3600 - min * 60);
        char buf[64];
        snprintf(buf, sizeof(buf), "T+ %02d:%02d:%02d:%02d", day, hour, min, sec);
        return string(buf);
    }
}
// ==========================================================
// Part 4:主函数
// ==========================================================
int main() {
  // --- 游戏启动与初始化 ---
  
  
  // ==========================================================
  // 这里是程序的入口。我们首先初始化 GLFW（一个处理窗口和输入的库）
  // 并配置 OpenGL 环境（处理 3D 图形的标准）
  // #############等待被模块化进入AppFramework：仅负责基础设置（GLFW/GLAD）、窗口管理和最外层异常处理。#############
  // ==========================================================
  glfwInit();
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

  // 请求一个包含 4倍多重采样 的帧缓冲
  glfwWindowHint(GLFW_SAMPLES, 4);

  GLFWwindow *window = glfwCreateWindow(1000, 800, "3D Rocket Sim", NULL, NULL);
  if (!window) {
    glfwTerminate();
    return -1;
  }
  glfwMakeContextCurrent(window);
  glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);
  glfwSetScrollCallback(window, scroll_callback);

  if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
    return -1;
  // Enable transparency blending
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  // Enable hardware multisampling
  glEnable(GL_MULTISAMPLE);
  renderer = new Renderer();
  
  PhysicsSystem::InitSolarSystem();
  
  Simulation::AsyncOrbitPredictor orbit_predictor;
  orbit_predictor.Start();

  // =========================================================
  // SceneManager State Machine Loop
  // =========================================================
  GameContext& ctx = GameContext::getInstance();
  ctx.window = window;
  ctx.renderer2d = renderer;

  SceneManager::getInstance().changeScene(std::make_unique<MenuScene>());

  double last_time = glfwGetTime();

  while (!glfwWindowShouldClose(window)) {
      glfwPollEvents();

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
      glfwSwapBuffers(window);
  }

  delete renderer;
  if (ctx.renderer3d) delete ctx.renderer3d;
  orbit_predictor.Stop();

  glfwTerminate();
  return 0;
}
