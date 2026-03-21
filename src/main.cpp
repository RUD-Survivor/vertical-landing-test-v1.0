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

using namespace std;

// ==========================================
// Part X: 火箭建造系统 -> moved to rocket_builder.h
// ==========================================

// 鼠标滚轮全局变量
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

// ==========================================
// Part 2: Explorer Class -> Replaced by ECS (RocketState, PhysicsSystem)
// ==========================================

// ==========================================
// Part 3: 主函数
// ==========================================

void framebuffer_size_callback(GLFWwindow *window, int width, int height) {
  glViewport(0, 0, width, height);
}

Renderer *renderer;


// ==========================================================
// Part 5: 火箭建造系统 (Rocket Builder) -> see rocket_builder.h
// ==========================================================

// drawBuilderUI -> replaced by drawBuilderUI_KSP in rocket_builder.h

void Report_Status(const RocketState& state, const ControlInput& input) {
  // ... (unchanged)
}

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

int main() {

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
  // Main Menu Phase: Show Startup Menu
  // =========================================================
  // =========================================================
  // 全局游戏状态 (Global Game State)
  // =========================================================
  AgencyState agency_state;
  FactorySystem factory;
  bool agency_is_init = false;

  // =========================================================
  // 游戏主循环 (Main State Machine Loop)
  // =========================================================
  MenuSystem::MenuState menu_state;
  while (!glfwWindowShouldClose(window)) {
  // Check Save File
      menu_state.has_save = SaveSystem::HasSaveFile();
      if (menu_state.has_save) {
          SaveSystem::GetSaveInfo(menu_state.save_time, menu_state.save_parts);
      } else {
          menu_state.save_time = 0; menu_state.save_parts = 0;
      }
      
      MenuSystem::MenuChoice menu_choice = MenuSystem::MENU_NONE;
      bool up_pressed = false, down_pressed = false, left_pressed = false, right_pressed = false, enter_pressed = false;
  
  
  // 主菜单/设置循环
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
              // Valid choice to move to Hub or Game
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
  
      // Handle Menu Exit
      if (menu_choice == MenuSystem::MENU_EXIT || glfwWindowShouldClose(window)) {
          break;
      }
      
      bool load_from_save = (menu_choice == MenuSystem::MENU_CONTINUE);
      
      if (!load_from_save) {
          // 如果是新游戏且有旧档，询问/删除 (逻辑简化：直接删)
          if (SaveSystem::HasSaveFile()) SaveSystem::DeleteSaveFile();
          // 重置 Agency
          agency_state = AgencyState();
          factory = FactorySystem();
          
          // Starter resources
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
          agency_is_init = true;
      } else {
          // 加载存档
          SaveSystem::LoadAgencyFactory(agency_state, factory);
          agency_is_init = true;
      }
  
      // Agency / Factory Loop
      while (menu_choice != MenuSystem::MENU_NONE && !glfwWindowShouldClose(window)) {
          if (menu_choice == MenuSystem::MENU_AGENCY_HUB) {
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
              break; // Proceed to 3D phase
          }
      }
      
      if (menu_choice == MenuSystem::MENU_NONE || glfwWindowShouldClose(window)) {
          continue; // Back to Main Menu 
      }
  
  
  // Transition to Rocket Building/Flight
  load_from_save = (menu_choice == MenuSystem::MENU_CONTINUE);
  bool skip_builder = false; // Default behavior
  bool factory_mode_active = false;

  // =========================================================
  // 初始化 3D 渲染器和网格 (Early instantiation for Workshop)
  // =========================================================
  Renderer3D* r3d = new Renderer3D();
  Mesh earthMesh = MeshGen::sphere(256, 512, 1.0f);  // Extreme-res unit sphere for terrain detail
  Mesh ringMesh = MeshGen::ring(128, 1.11f, 2.35f);  // NASA Real Ratios: D-ring start to F-ring end (1.11R to 2.35R)
  Mesh rocketBody = MeshGen::cylinder(32, 1.0f, 1.0f);
  Mesh rocketNose = MeshGen::cone(32, 1.0f, 1.0f);
  Mesh rocketBox  = MeshGen::box(1.0f, 1.0f, 1.0f);
  
  // Try to load launch pad model
  Mesh launchPadMesh = ModelLoader::loadOBJ("assets/launch_pad.obj");
  bool has_launch_pad = (launchPadMesh.indexCount > 0);

  // =========================================================
  // BUILD 阶段：KSP-like 火箭组装界面
  // =========================================================
  BuilderState builder_state;
  RocketState loaded_state;
  ControlInput loaded_input;
  // skip_builder is already set by Hub transition logic above
  
  if (load_from_save) {
      // 从存档加载
      if (SaveSystem::LoadGame(builder_state.assembly, loaded_state, loaded_input)) {
          skip_builder = true;
          cout << ">> SAVE FILE LOADED SUCCESSFULLY!" << endl;
      } else {
          cout << ">> FAILED TO LOAD SAVE FILE, STARTING NEW GAME" << endl;
          // 加载失败,使用默认火箭
          builder_state.assembly.addPart(9);   // Raptor engine
          builder_state.assembly.addPart(6);   // Medium fuel tank 100t
          builder_state.assembly.addPart(0);   // Standard fairing nosecone
      }
  } else {
      // 新游戏,使用默认火箭
      builder_state.assembly.addPart(9);   // Raptor engine
      builder_state.assembly.addPart(6);   // Medium fuel tank 100t
      builder_state.assembly.addPart(0);   // Standard fairing nosecone
  }
  
  bool build_done = skip_builder;

  BuilderKeyState bk_prev = {};

  while (!build_done && !glfwWindowShouldClose(window)) {
    glfwPollEvents();
    if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
      glfwSetWindowShouldClose(window, true);

    // Read builder inputs
    BuilderKeyState bk_now;
    bk_now.up    = glfwGetKey(window, GLFW_KEY_UP) == GLFW_PRESS;
    bk_now.down  = glfwGetKey(window, GLFW_KEY_DOWN) == GLFW_PRESS;
    bk_now.left  = glfwGetKey(window, GLFW_KEY_LEFT) == GLFW_PRESS;
    bk_now.right = glfwGetKey(window, GLFW_KEY_RIGHT) == GLFW_PRESS;
    bk_now.enter = glfwGetKey(window, GLFW_KEY_ENTER) == GLFW_PRESS;
    bk_now.del   = glfwGetKey(window, GLFW_KEY_DELETE) == GLFW_PRESS || glfwGetKey(window, GLFW_KEY_BACKSPACE) == GLFW_PRESS;
    bk_now.tab   = glfwGetKey(window, GLFW_KEY_TAB) == GLFW_PRESS;
    bk_now.pgup  = glfwGetKey(window, GLFW_KEY_PAGE_UP) == GLFW_PRESS;
    bk_now.pgdn  = glfwGetKey(window, GLFW_KEY_PAGE_DOWN) == GLFW_PRESS;
    bk_now.space = glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_PRESS;
    bk_now.q = glfwGetKey(window, GLFW_KEY_Q) == GLFW_PRESS;
    bk_now.e = glfwGetKey(window, GLFW_KEY_E) == GLFW_PRESS;
    bk_now.a = glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS;
    bk_now.d = glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS;
    bk_now.w = glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS;
    bk_now.s = glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS;

    // Mouse for builder
    double bmx, bmy;
    glfwGetCursorPos(window, &bmx, &bmy);
    int bww, bwh;
    glfwGetWindowSize(window, &bww, &bwh);
    bk_now.mx = (float)(bmx / bww * 2.0 - 1.0);
    bk_now.my = (float)(1.0 - bmy / bwh * 2.0);
    bk_now.lmb = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS;
    bk_now.rmb = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS;

    // First process builder logic (hit detection, drag, context menu)
    build_done = builderHandleInput(builder_state, bk_now, bk_prev);
    bk_prev = bk_now;

    // Then process camera controls based on updated builder state
    static double workshop_prev_mx = 0, workshop_prev_my = 0;
    static bool workshop_is_dragging = false;
    static bool allowed_rotation = false;

    if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS) {
        double mx, my;
        glfwGetCursorPos(window, &mx, &my);
        
        if (!workshop_is_dragging) {
            workshop_prev_mx = mx;
            workshop_prev_my = my;
            workshop_is_dragging = true;
            // Only allow rotation if no part menu was JUST opened this frame
            allowed_rotation = !builder_state.show_part_menu;
        } else if (allowed_rotation) {
            float dx = (float)(mx - workshop_prev_mx) * 0.005f;
            float dy = (float)(my - workshop_prev_my) * 0.005f;
            builder_state.orbit_angle -= dx;
            builder_state.orbit_pitch = std::max(-0.5f, std::min(1.5f, builder_state.orbit_pitch + dy));
            workshop_prev_mx = mx;
            workshop_prev_my = my;
        }
    } else {
        workshop_is_dragging = false;
        allowed_rotation = false;
        if (!builder_state.show_part_menu) builder_state.orbit_angle += 0.001f;
    }
    
    // Zoom and Pan controls
    if (g_scroll_y != 0.0f) {
        if (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS || glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS) {
            builder_state.cam_dist *= powf(0.85f, g_scroll_y);
            builder_state.cam_dist = std::max(2.0f, std::min(100.0f, builder_state.cam_dist));
        } else {
            builder_state.cam_y_offset += g_scroll_y * (builder_state.cam_dist * 0.05f);
        }
        g_scroll_y = 0.0f;
    }

    // --- 3D WORKSHOP RENDER PASS ---
    int ww, wh;
    glfwGetFramebufferSize(window, &ww, &wh);
    r3d->lightDir = Vec3(0.5f, 1.0f, 0.3f).normalized(); // Angled floodlight

    // Dynamic camera based on rocket height and manual pan
    float current_height = std::max(5.0f, builder_state.assembly.total_height);
    float look_y = current_height * 0.4f + builder_state.cam_y_offset;
    
    Vec3 camTarget(0.0f, look_y, 0.0f);
    float dist = builder_state.cam_dist + current_height * 0.5f;
    float cy = sinf(builder_state.orbit_pitch) * dist;
    float cx = cosf(builder_state.orbit_pitch) * cosf(builder_state.orbit_angle) * dist;
    float cz = cosf(builder_state.orbit_pitch) * sinf(builder_state.orbit_angle) * dist;
    Vec3 camEye = camTarget + Vec3(cx, cy, cz);
    
    Mat4 viewMat = Mat4::lookAt(camEye, camTarget, Vec3(0.0f, 1.0f, 0.0f));
    Mat4 projMat = Mat4::perspective(1.0f, (float)ww / (float)wh, 0.1f, 1000.0f);

    r3d->beginFrame(viewMat, projMat, camEye);

    // Dark foggy background for the massive VAB interior
    glClearColor(0.02f, 0.025f, 0.03f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    
    // Draw Workshop Environment
    Mat4 padMat = Mat4::translate(Vec3(0, -0.1f, 0)) * Mat4::scale(Vec3(40.0f, 0.2f, 40.0f));
    r3d->drawMesh(rocketBox, padMat, 0.15f, 0.16f, 0.18f, 1.0f, 0.1f); // Concrete floor
    
    // Draw grid lines on the pad
    for (int i = -10; i <= 10; i++) {
        if (i == 0) continue;
        Mat4 lineX = Mat4::translate(Vec3(i * 2.0f, 0.02f, 0)) * Mat4::scale(Vec3(0.05f, 0.02f, 40.0f));
        Mat4 lineZ = Mat4::translate(Vec3(0, 0.02f, i * 2.0f)) * Mat4::scale(Vec3(40.0f, 0.02f, 0.05f));
        r3d->drawMesh(rocketBox, lineX, 0.3f, 0.3f, 0.1f, 1.0f, 0.2f); // Yellow hazard lines
        r3d->drawMesh(rocketBox, lineZ, 0.3f, 0.3f, 0.1f, 1.0f, 0.2f);
    }

    // Scaffold pillars
    Mat4 pillar1 = Mat4::translate(Vec3(-6, 15, -6)) * Mat4::scale(Vec3(1, 30, 1));
    Mat4 pillar2 = Mat4::translate(Vec3(-6, 15, 6)) * Mat4::scale(Vec3(1, 30, 1));
    r3d->drawMesh(rocketBox, pillar1, 0.2f, 0.2f, 0.2f, 1.0f, 0.1f);
    r3d->drawMesh(rocketBox, pillar2, 0.2f, 0.2f, 0.2f, 1.0f, 0.1f);

    // Draw Assembled Rocket Part Tree
    auto drawPartWithSymmetry = [&](const PartDef& def, Vec3 pos, Quat rot, bool highlight, bool selected, float alpha = 1.0f, int sym = 1, float rm = 1.0f, float gm = 1.0f, float bm = 1.0f) {
        float r = def.r * rm, g = def.g * gm, b = def.b * bm;
        if (highlight) {
            float blink = 0.5f + 0.5f * sinf((float)glfwGetTime() * 8.0f);
            r = std::min(1.0f, r + 0.4f * blink); g = std::min(1.0f, g + 0.6f * blink); b = std::min(1.0f, b + 0.3f * blink);
        }
        if (selected) {
            r = r * 0.4f; g = g * 0.6f; b = std::min(1.0f, b + 0.7f);
        }

        // --- Custom Model and Texture Loading ---
        Mesh* customMesh = nullptr;
        bool hasTexture = false;
        if (def.model_path) {
            std::string mp(def.model_path);
            if (r3d->meshCache.find(mp) == r3d->meshCache.end()) {
                r3d->meshCache[mp] = ModelLoader::loadOBJ(mp);
            }
            customMesh = &r3d->meshCache[mp];
        }
        if (def.texture_path) {
            std::string tp(def.texture_path);
            if (r3d->textureCache.find(tp) == r3d->textureCache.end()) {
                r3d->textureCache[tp] = Renderer3D::loadTGA(def.texture_path);
            }
            if (r3d->textureCache[tp].id != 0) {
                r3d->textureCache[tp].bind(0);
                hasTexture = true;
            }
        }
        glUniform1i(r3d->u_hasTexture, hasTexture ? 1 : 0);
        glUniform1i(r3d->u_sampler, 0);

        for (int s = 0; s < sym; s++) {
            float angle = (s * 2.0f * 3.14159f) / sym;
            Vec3 symPos = pos;
            if (sym > 1) {
                float dist = sqrt(pos.x*pos.x + pos.z*pos.z);
                if (dist > 0.01f) {
                   float curAngle = atan2(pos.z, pos.x);
                   symPos.x = cos(curAngle + angle) * dist;
                   symPos.z = sin(curAngle + angle) * dist;
                }
            }

            if (customMesh && customMesh->indexCount > 0) {
                // Use custom model
                Mat4 partMat = Mat4::translate(symPos) * Mat4::fromQuat(rot) * Mat4::fromQuat(Quat::fromAxisAngle(Vec3(0, 1, 0), angle));
                r3d->drawMesh(*customMesh, partMat, r, g, b, alpha, 0.2f);
            } else {
                // Procedural fallback
                float pd = def.diameter;
                if (def.category == CAT_NOSE_CONE) {
                    Mat4 partMat = Mat4::translate(symPos) * Mat4::fromQuat(rot) * Mat4::scale({pd, def.height, pd});
                    r3d->drawMesh(rocketNose, partMat, r, g, b, alpha, 0.2f);
                } else if (def.category == CAT_ENGINE) {
                    float bf = 0.4f; float nf = 1.0f - bf;
                    Mat4 rotMat = Mat4::fromQuat(rot);
                    Mat4 bodyMat = Mat4::translate(symPos) * rotMat * Mat4::translate(Vec3(0, def.height*(1.0f-bf*0.5f), 0)) * Mat4::scale({pd*0.6f, def.height*bf, pd*0.6f});
                    r3d->drawMesh(rocketBody, bodyMat, 0.2f*rm, 0.2f*gm, 0.22f*bm, alpha, 0.4f);
                    Mat4 bellMat = Mat4::translate(symPos) * rotMat * Mat4::scale({pd*0.85f, def.height*nf, pd*0.85f});
                    r3d->drawMesh(rocketNose, bellMat, r*0.8f, g*0.8f, b*0.8f, alpha, 0.1f);
                } else if (def.category == CAT_STRUCTURAL) {
                    if (strstr(def.name, "Fin") || strstr(def.name, "Solar")) {
                        Mat4 finMat = Mat4::translate(symPos + Vec3(0, def.height*0.5f, 0)) * Mat4::fromQuat(Quat::fromAxisAngle(Vec3(0, 1, 0), angle)) * Mat4::scale({pd*0.05f, def.height, pd*0.5f});
                        r3d->drawMesh(rocketBox, finMat, r, g, b, alpha, 0.1f);
                    } else if (strstr(def.name, "Leg")) {
                        Mat4 legMat = Mat4::translate(symPos + Vec3(0, def.height*0.5f, 0)) * Mat4::fromQuat(Quat::fromAxisAngle(Vec3(0, 1, 0), angle)) * Mat4::scale({pd*0.15f, def.height, pd*0.15f});
                        r3d->drawMesh(rocketBox, legMat, r, g, b, alpha, 0.1f);
                    } else {
                        Mat4 partMat = Mat4::translate(symPos) * Mat4::fromQuat(rot) * Mat4::translate(Vec3(0, def.height*0.5f, 0)) * Mat4::scale({pd, def.height, pd});
                        r3d->drawMesh(rocketBody, partMat, r, g, b, alpha, 0.2f);
                    }
                } else {
                    Mat4 partMat = Mat4::translate(symPos) * Mat4::fromQuat(rot) * Mat4::translate(Vec3(0, def.height*0.5f, 0)) * Mat4::scale({pd, def.height, pd});
                    r3d->drawMesh(rocketBody, partMat, r, g, b, alpha, 0.2f);
                }
            }
        }
        // Cleanup texture binding
        glUniform1i(r3d->u_hasTexture, 0);
    };


    for (int i = 0; i < (int)builder_state.assembly.parts.size(); i++) {
        const PlacedPart& pp = builder_state.assembly.parts[i];
        bool is_selected = builder_state.show_part_menu && (builder_state.r_clicked_part_idx == i);
        drawPartWithSymmetry(PART_CATALOG[pp.def_id], pp.pos, pp.rot, 
                            (builder_state.in_assembly_mode && builder_state.assembly_cursor == i), is_selected, 1.0f, pp.symmetry);
    }

    // Draw Dragging Ghost
    if (builder_state.dragging_def_id != -1) {
        float pl = -0.98f, pw = 0.65f;
        bool over_catalog = (bk_now.mx < pl + pw);
        
        float rt = 1.0f, gt = 1.0f, bt = 1.0f;
        float alpha = 0.4f;
        if (!builder_state.is_placement_valid) { 
            float pulse = 0.5f + 0.5f * sinf((float)glfwGetTime() * 10.0f);
            rt = 1.0f; gt = 0.1f * pulse; bt = 0.1f * pulse; 
            alpha = 0.5f + 0.2f * pulse;
        }
        if (over_catalog) { rt = 1.0f; gt = 0.0f; bt = 0.0f; alpha = 0.3f; }

        drawPartWithSymmetry(PART_CATALOG[builder_state.dragging_def_id], 
                            builder_state.dragging_pos, builder_state.dragging_rot, 
                            false, false, alpha, builder_state.current_symmetry, rt, gt, bt);
        
        if (over_catalog) {
            renderer->addRect(pl + pw/2.0f, 0.15f, pw, 1.40f, 0.5f, 0.0f, 0.0f, 0.3f);
            renderer->drawText(pl + 0.15f, 0.15f, "DROP TO DELETE", 0.015f, 1, 1, 1);
        }

        // Render potential snap nodes as glowy points
        for (const auto& p : builder_state.assembly.parts) {
            const auto& pdef = PART_CATALOG[p.def_id];
            for (const auto& node : pdef.snap_nodes) {
                Mat4 nodeMat = Mat4::translate(p.pos + node.pos) * Mat4::scale({0.3f, 0.3f, 0.3f});
                r3d->drawMesh(rocketBox, nodeMat, 0, 1, 0, 0.8f, 0); // Green glow
            }
        }
    }

    // Update center visualization state (detect assembly changes and recalculate)
    size_t current_hash = CenterCalculator::hashAssembly(builder_state.assembly);
    if (current_hash != builder_state.centerViz.lastAssemblyHash) {
        builder_state.centerViz.lastAssemblyHash = current_hash;
        
        // Recalculate all center positions
        builder_state.centerViz.comPos = CenterCalculator::calculateCenterOfMass(builder_state.assembly);
        builder_state.centerViz.colPos = CenterCalculator::calculateCenterOfLift(builder_state.assembly);
        builder_state.centerViz.cotPos = CenterCalculator::calculateCenterOfThrust(builder_state.assembly);
        
        // Update validity flags
        builder_state.centerViz.hasCoM = !builder_state.assembly.parts.empty();
        builder_state.centerViz.hasCoL = (builder_state.centerViz.colPos.y > 0.0f);
        builder_state.centerViz.hasCoT = builder_state.assembly.hasEngine();
    }
    
    // Render center point markers (if enabled)
    Mat4 rocketTransform; // Default constructor creates identity matrix
    
    if (builder_state.centerViz.showCoM && builder_state.centerViz.hasCoM) {
        std::cout << "Rendering CoM at y=" << builder_state.centerViz.comPos.y << std::endl;
        CenterVisualizer::renderMarker(r3d, builder_state.centerViz.comPos, 
                                      CenterVisualizer::MARKER_COM, rocketTransform);
    }
    
    if (builder_state.centerViz.showCoL && builder_state.centerViz.hasCoL) {
        std::cout << "Rendering CoL at y=" << builder_state.centerViz.colPos.y << std::endl;
        CenterVisualizer::renderMarker(r3d, builder_state.centerViz.colPos, 
                                      CenterVisualizer::MARKER_COL, rocketTransform);
    }
    
    if (builder_state.centerViz.showCoT && builder_state.centerViz.hasCoT) {
        std::cout << "Rendering CoT at y=" << builder_state.centerViz.cotPos.y << std::endl;
        CenterVisualizer::renderMarker(r3d, builder_state.centerViz.cotPos, 
                                      CenterVisualizer::MARKER_COT, rocketTransform);
    }

    r3d->endFrame();

    // Render builder UI OVERLAY (clear depth buffer so 2D renders on top)
    glClear(GL_DEPTH_BUFFER_BIT);
    renderer->beginFrame();
    
    // Render center point labels (2D overlay)
    if (builder_state.centerViz.showCoM && builder_state.centerViz.hasCoM) {
        CenterVisualizer::renderLabel(renderer, builder_state.centerViz.comPos, 
                                     "CoM", viewMat * projMat);
    }
    
    if (builder_state.centerViz.showCoL && builder_state.centerViz.hasCoL) {
        CenterVisualizer::renderLabel(renderer, builder_state.centerViz.colPos, 
                                     "CoL", viewMat * projMat);
    }
    
    if (builder_state.centerViz.showCoT && builder_state.centerViz.hasCoT) {
        CenterVisualizer::renderLabel(renderer, builder_state.centerViz.cotPos, 
                                     "CoT", viewMat * projMat);
    }
    
    drawBuilderUI_KSP(renderer, builder_state, agency_state, (float)glfwGetTime());
    renderer->endFrame();

    glfwSwapBuffers(window);

    std::this_thread::sleep_for(std::chrono::milliseconds(16));
  }

  // =========================================================
  // Build RocketConfig from assembled parts
  // =========================================================
  // =========================================================
  // RE-CENTER AROUND CoM BEFORE FLIGHT (Root Part logic)
  // =========================================================
  if (!skip_builder && !builder_state.assembly.parts.empty()) {
      Vec3 com = CenterCalculator::calculateCenterOfMass(builder_state.assembly);
      for (auto& p : builder_state.assembly.parts) {
          p.pos = p.pos - com;
      }
      builder_state.assembly.com = Vec3(0,0,0); // It's now at the origin for the simulation
      builder_state.assembly.recalculate();
  }
  RocketConfig rocket_config = builder_state.assembly.buildRocketConfig();
  
  // Consume parts from agency inventory
  for (const auto& p : builder_state.assembly.parts) {
      const PartDef& def = PART_CATALOG[p.def_id];
      ItemType it = ITEM_NONE;
      if (def.category == CAT_NOSE_CONE) it = PART_NOSECONE;
      else if (def.category == CAT_COMMAND_POD) it = PART_COMMAND_POD;
      else if (def.category == CAT_FUEL_TANK) it = PART_FUEL_TANK;
      else if (def.category == CAT_ENGINE) it = PART_ENGINE;
      else if (def.category == CAT_BOOSTER) it = PART_FUEL_TANK;
      else if (def.category == CAT_STRUCTURAL) it = PART_STRUCTURAL;
      if (it != ITEM_NONE) {
          agency_state.removeItem(it, 1);
      }
  }

  RocketState rocket_state;
  ControlInput control_input;
  
  if (skip_builder) {
      // 使用加载的状态
      rocket_state = loaded_state;
      control_input = loaded_input;
      // Sync config to loaded stage
      StageManager::SyncActiveConfig(rocket_config, rocket_state.current_stage);
  } else {
      // 新游戏初始化
      rocket_state.fuel = builder_state.assembly.total_fuel;
      rocket_state.status = PRE_LAUNCH;
      rocket_state.mission_msg = "READY ON PAD - PRESS SPACE TO LAUNCH";
      
      // Initialize multi-stage fuel distribution
      rocket_state.total_stages = rocket_config.stages;
      rocket_state.current_stage = 0;
      rocket_state.stage_fuels.clear();
      for (int i = 0; i < (int)rocket_config.stage_configs.size(); i++) {
          rocket_state.stage_fuels.push_back(rocket_config.stage_configs[i].fuel_capacity);
      }
      // Set initial fuel to stage 0’s capacity
      if (!rocket_state.stage_fuels.empty()) {
          rocket_state.fuel = rocket_state.stage_fuels[0];
      }
      
      // Calculate initial surface coordinates from launch latitude/longitude
      double lat_rad = rocket_state.launch_latitude * PI / 180.0;
      double lon_rad = rocket_state.launch_longitude * PI / 180.0;

      float lowest_y = 0.0f;
      if (!builder_state.assembly.parts.empty()) {
          lowest_y = 1e10f;
          for (const auto& p : builder_state.assembly.parts) {
              lowest_y = std::min(lowest_y, (float)p.pos.y);
          }
      }
      // Distance from planet center to CoM
      double R = SOLAR_SYSTEM[current_soi_index].radius - (double)lowest_y;

      // Z is the North-South axis, XY is the equatorial plane
      rocket_state.surf_px = R * cos(lat_rad) * cos(lon_rad);
      rocket_state.surf_py = R * cos(lat_rad) * sin(lon_rad);
      rocket_state.surf_pz = R * sin(lat_rad);

      // Initialize inertial coordinates immediately for the first frame
      CelestialBody& body = SOLAR_SYSTEM[current_soi_index];
      double theta = body.prime_meridian_epoch; // sim_time = 0
      Quat rot = Quat::fromAxisAngle(Vec3(0, 0, 1), (float)theta);
      Quat tilt = Quat::fromAxisAngle(Vec3(1, 0, 0), (float)body.axial_tilt);
      Quat full_rot = tilt * rot;
      
      Vec3 world_pos = full_rot.rotate(Vec3((float)rocket_state.surf_px, (float)rocket_state.surf_py, (float)rocket_state.surf_pz));
      rocket_state.px = (double)world_pos.x;
      rocket_state.py = (double)world_pos.y;
      rocket_state.pz = (double)world_pos.z;
  }

  // Keep a reference to the assembly for rendering
  const RocketAssembly& assembly = builder_state.assembly;

  int cam_mode_3d = 0; // 0=轨道 (优化型), 1=跟踪, 2=全景, 3=自由视角
  static double free_cam_px = 0, free_cam_py = 0, free_cam_pz = 0;
  static float free_cam_move_speed = 0.0f;
  static bool free_cam_init = false;
  static bool c_was_pressed = false;
  // 相机参数
  Quat cam_quat; // 自由模式四元数 (用于全景和追踪微调)
  
  // --- Galaxy Info UI State ---
  static bool show_galaxy_info = false;
  static int selected_body_idx = -1;
  static int expanded_planet_idx = -1;
  static bool hlmb_prev_galaxy = false;
  float orbit_yaw = 0.0f;   // 专用轨道视角偏航
  float orbit_pitch = 0.4f; // 专用轨道视角俯仰
  float cam_zoom_chase = 1.0f; // 轨道/跟踪模式缩放
  float cam_zoom_pan = 1.0f;   // 全景模式独立缩放
  double prev_mx = 0, prev_my = 0;
  bool mouse_dragging = false;

  cout << ">> ROCKET ASSEMBLED! (" << assembly.parts.size() << " parts)" << endl;
  cout << ">>   Dry Mass: " << (int)assembly.total_dry_mass << " kg" << endl;
  cout << ">>   Fuel: " << (int)assembly.total_fuel << " kg" << endl;
  cout << ">>   Height: " << (int)assembly.total_height << " m" << endl;
  cout << ">>   ISP: " << (int)assembly.avg_isp << " s" << endl;
  cout << ">>   Delta-V: " << (int)assembly.total_delta_v << " m/s" << endl;
  cout << ">>   TWR: " << assembly.twr << endl;
  cout << ">> PRESS [SPACE] TO LAUNCH!" << endl;
  cout << ">> [TAB] Toggle Auto/Manual | [WASD] Thrust & Attitude" << endl;
  cout << ">> [Z] Full Throttle | [X] Kill Throttle | [1-4] Time Warp" << endl;
  cout << ">> [O] Toggle Orbit Display" << endl;

  double dt = 0.02; // 50Hz 物理步长
  
  static bool tab_was_pressed = false; // Tab 防抖

  struct DVec3 { double x, y, z; };
  struct TrajPoint { DVec3 e; DVec3 s; };
  std::vector<TrajPoint> traj_history; // 记录火箭历史飞行的 3D 轨迹点

  int frame = 0;
  while (!glfwWindowShouldClose(window)) {
    frame++;
    glfwPollEvents();
    if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
      glfwSetWindowShouldClose(window, true);
    
    int ww, wh;
    glfwGetWindowSize(window, &ww, &wh);
    static float global_best_ang = -1.0f;
    static double global_best_mu = 0, global_best_a = 0, global_best_ecc = 0;
    static double global_current_M0 = 0, global_current_n = 0;
    static Vec3 global_best_pt, global_best_center, global_best_e_dir, global_best_perp_dir;
    static bool rmb_prev_mnv = false;
    static int global_best_ref_node = -1; // Index of SOI for node reference
    
    // Maneuver popup deferred rendering state (computed in 3D pass, rendered in 2D HUD pass)
    static bool mnv_popup_visible = false;
    static int mnv_popup_index = -1;
    static float mnv_popup_px = 0, mnv_popup_py = 0, mnv_popup_pw = 0, mnv_popup_ph = 0;
    static float mnv_popup_node_scr_x = 0, mnv_popup_node_scr_y = 0;
    static Vec3 mnv_popup_dv;
    static bool mnv_popup_close_hover = false, mnv_popup_del_hover = false;
    static double mnv_popup_time_to_node = 0;   // seconds until maneuver
    static double mnv_popup_burn_time = 0;       // estimated burn time
    static int mnv_popup_ref_body = -1;          // reference body index
    static int mnv_popup_slider_dragging = -1;   // -1=none, 0=prograde, 1=normal, 2=radial, 3=time
    static float mnv_popup_slider_drag_x = 0;    // mouse x at drag start
    static int mnv_popup_burn_mode = 0;          // 0=impulse, 1=sustained
    static bool mnv_popup_mode_hover = false;    // hover for mode button
    static Vec3 adv_mnv_world_pos(0,0,0);        // Exact N-body maneuver visual position
    
    // Advanced Orbit Settings
    static bool adv_orbit_enabled = false;
    static bool adv_orbit_menu = false;
    static float adv_orbit_pred_days = 30.0f;   // prediction time in days (user adjustable)
    static int adv_orbit_iters = 4000;           // number of iterations (user adjustable)
    static int adv_orbit_ref_mode = 0;    // 0 = Inertial, 1 = Co-rotating
    static int adv_orbit_ref_body = 3;    // Earth default
    static int adv_orbit_secondary_ref_body = 4; // Moon default
    static bool adv_warp_to_node = false; // warp-to-maneuver in progress
    static bool auto_exec_mnv = false;    // auto-execute next maneuver node
    static bool flight_assist_menu = false; // Flight Assist menu visibility
    static bool adv_embed_mnv = false;    // maneuver popup embedded in adv menu
    static bool adv_embed_mnv_mini = false; // deeply folded (mini) state
    static bool mnv_popup_mini_hover = false;

    // Transfer Window Calculator state
    static bool transfer_window_menu = false;
    static int transfer_target_body = 5;       // default Mars
    static PorkchopResult transfer_result;
    static bool transfer_result_valid = false;
    static int transfer_hover_dep = -1;
    static int transfer_hover_tof = -1;

    static bool space_was_pressed = true; // Start true to ignore the Builder's enter/space
    bool space_now = glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_PRESS;
    if (space_now && !space_was_pressed) {
      if (rocket_state.status == PRE_LAUNCH) {
          rocket_state.status = ASCEND;
          rocket_state.mission_msg = "T-0: IGNITION! LIFTOFF!";
      }
    }
    space_was_pressed = space_now;

    // --- G 键手动分级 (Manual Stage Separation) ---
    static bool g_was_pressed = false;
    bool g_now = glfwGetKey(window, GLFW_KEY_G) == GLFW_PRESS;
    if (g_now && !g_was_pressed) {
        if (rocket_state.status == ASCEND || rocket_state.status == DESCEND) {
            StageManager::SeparateStage(rocket_state, rocket_config);
        }
    }
    g_was_pressed = g_now;

    // --- N 键自动执行机动节点 (Auto-Execute Maneuver) ---
    static bool n_was_pressed = false;
    bool n_now = glfwGetKey(window, GLFW_KEY_N) == GLFW_PRESS;
    if (n_now && !n_was_pressed) {
        if (!rocket_state.maneuvers.empty()) {
            auto_exec_mnv = !auto_exec_mnv;
            if (auto_exec_mnv) {
                rocket_state.mission_msg = "MNV AUTO-EXEC: ARMED";
            } else {
                rocket_state.mission_msg = "MNV AUTO-EXEC: OFF";
                control_input.throttle = 0;
            }
        }
    }
    n_was_pressed = n_now;

    // --- C 键切换 3D 视角 ---
    bool c_now = glfwGetKey(window, GLFW_KEY_C) == GLFW_PRESS;
    if (c_now && !c_was_pressed) {
      cam_mode_3d = (cam_mode_3d + 1) % 4;
      const char* names[] = {"Orbit", "Chase", "Panorama", "Free"};
      cout << ">> Camera: " << names[cam_mode_3d] << endl;
      if (cam_mode_3d == 3) free_cam_init = false;
    }
    c_was_pressed = c_now;

    // --- 鼠标轨道控制 (3D模式下右键拖动) ---
      double mx, my;
      glfwGetCursorPos(window, &mx, &my);
      bool rmb = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS;
      bool lmb = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS;

      if (rmb || lmb) {
        if (mouse_dragging) {
          float dx = (float)(mx - prev_mx) * 0.003f;
          float dy = (float)(my - prev_my) * 0.003f;
          
          if (cam_mode_3d == 0) {
              // 轨道模式：更新球坐标 (仅 RMB)
              if (rmb) {
                  orbit_yaw -= dx;
                  orbit_pitch = std::max(-1.5f, std::min(1.5f, orbit_pitch + dy));
              }
          } else if (cam_mode_3d == 3) {
              // 自由视角 (Free Camera)
              if (lmb) {
                  // 左键：改变视角方向 (Look around)
                  Vec3 local_up = cam_quat.rotate(Vec3(0, 1, 0));
                  Vec3 local_right = cam_quat.rotate(Vec3(1, 0, 0));
                  Quat q_yaw = Quat::fromAxisAngle(local_up, -dx);
                  Quat q_pitch = Quat::fromAxisAngle(local_right, -dy);
                  cam_quat = (q_yaw * q_pitch * cam_quat).normalized();
              } else if (rmb) {
                  // 右键：绕着 SOI 星球旋转，保持中心天体在屏幕位置不变
                  Vec3 rel_v((float)free_cam_px, (float)free_cam_py, (float)free_cam_pz);
                  float dist = rel_v.length();
                  if (dist > 1.0f) {
                      Vec3 planet_z(0, 0, 1);
                      Vec3 pos_norm = rel_v / dist;
                      Vec3 orbit_right = planet_z.cross(pos_norm).normalized();
                      if (orbit_right.length() < 0.01f) orbit_right = Vec3(1,0,0);
                      
                      Quat q_orbit_yaw = Quat::fromAxisAngle(planet_z, -dx);
                      Quat q_orbit_pitch = Quat::fromAxisAngle(orbit_right, -dy);
                      Quat q_total = q_orbit_yaw * q_orbit_pitch;
                      
                      // 旋转位置
                      Vec3 new_pos = q_total.rotate(rel_v);
                      free_cam_px = (double)new_pos.x;
                      free_cam_py = (double)new_pos.y;
                      free_cam_pz = (double)new_pos.z;
                      
                      // 补偿相机旋转以保持视野中的天体位置不变
                      cam_quat = (q_total * cam_quat).normalized();
                  }
              }
          } else {
              // Panorama / Chase (仅 RMB)
              if (rmb) {
                  Quat yaw_rot = Quat::fromAxisAngle(Vec3(0.0f, 0.0f, 1.0f), -dx);
                  Vec3 cam_right = cam_quat.rotate(Vec3(1.0f, 0.0f, 0.0f));
                  Quat pitch_rot = Quat::fromAxisAngle(cam_right, -dy);
                  cam_quat = (yaw_rot * pitch_rot * cam_quat).normalized();
              }
          }
        }
        mouse_dragging = true;
      } else {
        mouse_dragging = false;
      }
      prev_mx = mx;
      prev_my = my;

      // 自由视角 EQ 键旋转 (Roll)
      if (cam_mode_3d == 3) {
          if (glfwGetKey(window, GLFW_KEY_Q) == GLFW_PRESS) {
              Quat q_roll = Quat::fromAxisAngle(cam_quat.rotate(Vec3(0, 0, 1)), 0.03f);
              cam_quat = (q_roll * cam_quat).normalized();
          }
          if (glfwGetKey(window, GLFW_KEY_E) == GLFW_PRESS) {
              Quat q_roll = Quat::fromAxisAngle(cam_quat.rotate(Vec3(0, 0, 1)), -0.03f);
              cam_quat = (q_roll * cam_quat).normalized();
          }
      }

      // 滚轮缩放：根据当前模式分离缩放级别
      if (g_scroll_y != 0.0f) {
        if (cam_mode_3d == 2) {
            // 全景模式：允许极其庞大的缩放以俯瞰太阳系
            cam_zoom_pan *= powf(0.85f, g_scroll_y);
            if (cam_zoom_pan < 0.05f) cam_zoom_pan = 0.05f;
            if (cam_zoom_pan > 500000.0f) cam_zoom_pan = 500000.0f; 
        } else if (cam_mode_3d == 3) {
            // 自由视角：决定前进速度
            if (free_cam_move_speed < 1.0f) free_cam_move_speed = 1.0f;
            free_cam_move_speed *= powf(1.25f, g_scroll_y);
            if (free_cam_move_speed < 1.1f) free_cam_move_speed = 0.0f;
            if (free_cam_move_speed > 1e11f) free_cam_move_speed = 1e11f;
        } else {
            // 轨道/跟踪模式：限制缩放防止火箭消失
            cam_zoom_chase *= powf(0.85f, g_scroll_y);
            if (cam_zoom_chase < 0.05f) cam_zoom_chase = 0.05f;
            if (cam_zoom_chase > 20.0f) cam_zoom_chase = 20.0f;
        }
        g_scroll_y = 0.0f;
      }

    // --- Maneuver Node Input Handling (Basic) ---
    static int dragging_handle = -1;
    static float last_drag_mx = 0;
    if (cam_mode_3d == 2) {
        double mx, my; glfwGetCursorPos(window, &mx, &my);
        float mouse_x = (float)(mx / ww * 2.0 - 1.0);
        float mouse_y = (float)(1.0 - my / wh * 2.0);
        static float frame_delta_mx = 0;
        frame_delta_mx = mouse_x - last_drag_mx;
        bool lmb = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS;
        if (!lmb) dragging_handle = -1;
        
        if (dragging_handle != -1 && lmb && rocket_state.selected_maneuver_index != -1) {
            ManeuverNode& node = rocket_state.maneuvers[rocket_state.selected_maneuver_index];
            if (dragging_handle == -2) { // Dragging the node itself (Time Slider)
                if (global_best_ang >= 0) {
                    double mu_body = 6.67430e-11 * SOLAR_SYSTEM[current_soi_index].mass;
                    double M_click = global_best_ang - global_best_ecc * sin(global_best_ang);
                    double r_mag = sqrt(rocket_state.px*rocket_state.px + rocket_state.py*rocket_state.py + rocket_state.pz*rocket_state.pz);
                    double cos_E = (global_best_a - r_mag) / (global_best_a * global_best_ecc);
                    double sin_E = (rocket_state.px*rocket_state.vx + rocket_state.py*rocket_state.vy + rocket_state.pz*rocket_state.vz) / (global_best_ecc * sqrt(mu_body * global_best_a));
                    double E0 = atan2(sin_E, cos_E);
                    double M0 = E0 - global_best_ecc * sin(E0);
                    double n = sqrt(mu_body / (global_best_a * global_best_a * global_best_a));
                    double dM = M_click - M0;
                    while (dM < 0) dM += 2.0 * PI;
                    while (dM > 2.0 * PI) dM -= 2.0 * PI;
                    node.sim_time = rocket_state.sim_time + (dM / n);
                }
            } else { // Dragging a Delta-V handle
                // This logic is now handled in the rendering pass where screen-space axis vectors are available
            }
        }
        last_drag_mx = mouse_x;
    }

    // --- H 键切换 HUD 显示 ---
    static bool show_hud = true;
    static bool h_was_pressed = false;
    if (glfwGetKey(window, GLFW_KEY_H) == GLFW_PRESS) {
      if (!h_was_pressed) {
        show_hud = !show_hud;
        h_was_pressed = true;
      }
    } else {
      h_was_pressed = false;
    }

    // --- Tab 键切换模式（带防抖）---
    if (glfwGetKey(window, GLFW_KEY_TAB) == GLFW_PRESS) {
      if (!tab_was_pressed) {
        rocket_state.auto_mode = !rocket_state.auto_mode;
        if (rocket_state.auto_mode) {
          rocket_state.pid_vert.reset();
          rocket_state.pid_pos.reset();
          rocket_state.pid_att.reset();
          rocket_state.pid_att_z.reset();
          rocket_state.mission_msg = ">> AUTOPILOT ENGAGED";
        } else {
          rocket_state.mission_msg = ">> MANUAL CONTROL ACTIVE";
        }
        tab_was_pressed = true;
      }
    } else {
      tab_was_pressed = false;
    }

    // --- O 键切换轨道显示 ---
    static bool show_orbit = true;
    static bool o_was_pressed = false;
    static bool orbit_reference_sun = false; // 0=地球, 1=太阳
    if (glfwGetKey(window, GLFW_KEY_O) == GLFW_PRESS) {
      if (!o_was_pressed) {
        show_orbit = !show_orbit; // 这里为了兼容旧逻辑，如果是双击才算切换？不如我们用 R 键来切换参考系
        o_was_pressed = true;
      }
    } else {
      o_was_pressed = false;
    }

    static bool r_was_pressed = false;
    if (glfwGetKey(window, GLFW_KEY_T) == GLFW_PRESS) {
      if (!r_was_pressed) {
        rocket_state.sas_active = !rocket_state.sas_active;
        cout << "[SAS] " << (rocket_state.sas_active ? "ON" : "OFF") << endl;
        r_was_pressed = true;
      }
    } else {
      r_was_pressed = false;
    }

    static bool rcs_key_prev = false;
    if (glfwGetKey(window, GLFW_KEY_R) == GLFW_PRESS) {
      if (!rcs_key_prev) {
        rocket_state.rcs_active = !rocket_state.rcs_active;
        cout << "[RCS] " << (rocket_state.rcs_active ? "ON" : "OFF") << endl;
        rcs_key_prev = true;
      }
    } else {
      rcs_key_prev = false;
    }

    static bool k_was_pressed = false;
    if (glfwGetKey(window, GLFW_KEY_K) == GLFW_PRESS) {
      if (!k_was_pressed) {
        orbit_reference_sun = !orbit_reference_sun;
        cout << "[REF FRAME] " << (orbit_reference_sun ? "SUN" : "EARTH") << endl;
        k_was_pressed = true;
      }
    } else {
      k_was_pressed = false;
    }
    
    // --- Shift+P 切换云层显示 ---
    static bool show_clouds = true;
    static bool p_was_pressed = false;
    bool shift_now = glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS || glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS;
    bool p_now = glfwGetKey(window, GLFW_KEY_P) == GLFW_PRESS;
    if (shift_now && p_now && !p_was_pressed) {
        show_clouds = !show_clouds;
        cout << "[CLOUDS] " << (show_clouds ? "ON" : "OFF") << endl;
    }
    p_was_pressed = (shift_now && p_now);

    // --- Ensure Rocket starts perfectly on the Terrain/SVO surface ---
    if (r3d->terrain) {
        Vec3 localUp(rocket_state.surf_px, rocket_state.surf_py, rocket_state.surf_pz);
        localUp = localUp.normalized();
        Quat unalign_from_z = Quat::fromAxisAngle(Vec3(1.0f, 0.0f, 0.0f), (float)(PI / 2.0));
        Vec3 qLocal = unalign_from_z.rotate(localUp);
        float terrH = r3d->terrain->getHeight(qLocal);
        rocket_state.terrain_altitude = (double)terrH * 1000.0; // km to meters
        
        static bool terrain_adjusted = false;
        if (rocket_state.status == PRE_LAUNCH && !terrain_adjusted) {
            double new_R = EARTH_RADIUS + rocket_state.terrain_altitude;
            rocket_state.surf_px = localUp.x * new_R;
            rocket_state.surf_py = localUp.y * new_R;
            rocket_state.surf_pz = localUp.z * new_R;
            terrain_adjusted = true;
            cout << "[TERRAIN] Adjusted launchpad altitude by " << rocket_state.terrain_altitude << " meters to match SVO bounds." << endl;
        }
    }

    // --- Shift+V 切换 SVO 系统 (Phase 1 测试) ---
    static bool v_was_pressed = false;
    bool v_now = glfwGetKey(window, GLFW_KEY_V) == GLFW_PRESS;
    if (shift_now && v_now && !v_was_pressed) {
        if (r3d->svoManager) {
            if (r3d->svoManager->hasActiveChunks()) {
                r3d->svoManager->deactivate();
                cout << "[SVO] System Deactivated (Quadtree active)" << endl;
            } else {
                // Quadtree local space expects Y-up, but surf_p is Z-up. 
                // We must apply the inverse of `align_to_z` (from drawPlanet) to rotate it to local space.
                Vec3 subNormal(rocket_state.surf_px, rocket_state.surf_py, rocket_state.surf_pz);
                subNormal = subNormal.normalized();
                Quat unalign_from_z = Quat::fromAxisAngle(Vec3(1.0f, 0.0f, 0.0f), (float)(PI / 2.0));
                subNormal = unalign_from_z.rotate(subNormal);
                
                r3d->svoManager->activate(subNormal, EARTH_RADIUS * 0.001, r3d->terrain);
                cout << "[SVO] System Activated at sub-point!" << endl;
            }
        }
    }
    v_was_pressed = (shift_now && v_now);

    // 全局帧计数器 (用于限制控制台打印频率)
    static int frame = 0;
    frame++;

    // --- 时间加速逻辑 ---
    static int time_warp = 1;
    // 条件：手动模式、没开推力(或者没燃料了)、处于真空(真空设为>100000m) 或者是 在地面且速度极低 才可以开启极速加速 (5,6,7,8)
    double surface_speed = std::sqrt(rocket_state.velocity * rocket_state.velocity + rocket_state.local_vx * rocket_state.local_vx);
    bool is_parked = (rocket_state.status == PRE_LAUNCH || rocket_state.status == LANDED) && surface_speed < 0.1;
    bool can_super_warp = (!rocket_state.auto_mode && (rocket_state.thrust_power <= 0.01 || rocket_state.fuel <= 0) && (rocket_state.altitude > 100000.0 || is_parked));

    if (rocket_state.auto_mode) {
      if (glfwGetKey(window, GLFW_KEY_1) == GLFW_PRESS || glfwGetKey(window, GLFW_KEY_KP_1) == GLFW_PRESS) time_warp = 1;
      if (glfwGetKey(window, GLFW_KEY_2) == GLFW_PRESS || glfwGetKey(window, GLFW_KEY_KP_2) == GLFW_PRESS) time_warp = 10;
      if (glfwGetKey(window, GLFW_KEY_3) == GLFW_PRESS || glfwGetKey(window, GLFW_KEY_KP_3) == GLFW_PRESS) time_warp = 100;
      if (glfwGetKey(window, GLFW_KEY_4) == GLFW_PRESS || glfwGetKey(window, GLFW_KEY_KP_4) == GLFW_PRESS) time_warp = 1000;
      // 在自动模式中降级时间加速
      if (time_warp > 1000) time_warp = 1; 
    } else {
      if (glfwGetKey(window, GLFW_KEY_1) == GLFW_PRESS || glfwGetKey(window, GLFW_KEY_KP_1) == GLFW_PRESS) time_warp = 1;
      if (glfwGetKey(window, GLFW_KEY_2) == GLFW_PRESS || glfwGetKey(window, GLFW_KEY_KP_2) == GLFW_PRESS) time_warp = 10;
      if (glfwGetKey(window, GLFW_KEY_3) == GLFW_PRESS || glfwGetKey(window, GLFW_KEY_KP_3) == GLFW_PRESS) time_warp = 100;
      if (glfwGetKey(window, GLFW_KEY_4) == GLFW_PRESS || glfwGetKey(window, GLFW_KEY_KP_4) == GLFW_PRESS) time_warp = 1000;

      if (can_super_warp) {
          if (glfwGetKey(window, GLFW_KEY_5) == GLFW_PRESS || glfwGetKey(window, GLFW_KEY_KP_5) == GLFW_PRESS) { time_warp = 10000; cout << "WARP: 10K SPEED ENGAGED!" << endl; }
          if (glfwGetKey(window, GLFW_KEY_6) == GLFW_PRESS || glfwGetKey(window, GLFW_KEY_KP_6) == GLFW_PRESS) { time_warp = 100000; cout << "WARP: 100K SPEED ENGAGED!" << endl; }
          if (glfwGetKey(window, GLFW_KEY_7) == GLFW_PRESS || glfwGetKey(window, GLFW_KEY_KP_7) == GLFW_PRESS) { time_warp = 1000000; cout << "WARP: 1M SPEED ENGAGED!" << endl; }
          if (glfwGetKey(window, GLFW_KEY_8) == GLFW_PRESS || glfwGetKey(window, GLFW_KEY_KP_8) == GLFW_PRESS) { time_warp = 10000000; cout << "WARP: 10M SPEED ENGAGED!" << endl; }
      } else {
          // 如果条件不满足，强制降级到最高 1000倍
          if (time_warp > 1000) {
              time_warp = 1; 
              static int last_warn = 0;
              if (frame - last_warn > 60) {
                 cout << ">> WARP DISENGAGED: Unsafe conditions (Alt < 100km or Engine Active)" << endl;
                 last_warn = frame;
              }
          }
      }
    }

    ControlSystem::ManualInputs manual;
    manual.throttle_up = glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS || 
                         glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS ||
                         glfwGetKey(window, GLFW_KEY_KP_ADD) == GLFW_PRESS || 
                         glfwGetKey(window, GLFW_KEY_RIGHT_BRACKET) == GLFW_PRESS;
    manual.throttle_down = glfwGetKey(window, GLFW_KEY_LEFT_CONTROL) == GLFW_PRESS || 
                           glfwGetKey(window, GLFW_KEY_RIGHT_CONTROL) == GLFW_PRESS ||
                           glfwGetKey(window, GLFW_KEY_KP_SUBTRACT) == GLFW_PRESS || 
                           glfwGetKey(window, GLFW_KEY_LEFT_BRACKET) == GLFW_PRESS;
    manual.throttle_max = glfwGetKey(window, GLFW_KEY_Z) == GLFW_PRESS;
    manual.throttle_min = glfwGetKey(window, GLFW_KEY_X) == GLFW_PRESS;
    // 在自由视角模式下，WASD 键被用于相机移动，仅保留方向键给火箭姿态。
    manual.yaw_left   = (cam_mode_3d == 3) ? (glfwGetKey(window, GLFW_KEY_LEFT) == GLFW_PRESS) : (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS || glfwGetKey(window, GLFW_KEY_LEFT) == GLFW_PRESS);
    manual.yaw_right  = (cam_mode_3d == 3) ? (glfwGetKey(window, GLFW_KEY_RIGHT) == GLFW_PRESS) : (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS || glfwGetKey(window, GLFW_KEY_RIGHT) == GLFW_PRESS);
    manual.pitch_up   = (cam_mode_3d == 3) ? (glfwGetKey(window, GLFW_KEY_UP) == GLFW_PRESS) : (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS || glfwGetKey(window, GLFW_KEY_UP) == GLFW_PRESS);
    manual.pitch_down = (cam_mode_3d == 3) ? (glfwGetKey(window, GLFW_KEY_DOWN) == GLFW_PRESS) : (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS || glfwGetKey(window, GLFW_KEY_DOWN) == GLFW_PRESS);
    manual.roll_left  = (cam_mode_3d == 3) ? false : glfwGetKey(window, GLFW_KEY_Q) == GLFW_PRESS;
    manual.roll_right = (cam_mode_3d == 3) ? false : glfwGetKey(window, GLFW_KEY_E) == GLFW_PRESS;

    // --- 物理更新 ---
    if (time_warp > 1000) {
        // 超级时间加速！
        if (is_parked) {
            // 确保状态正确并强制静止，不再重新计算 surf_px/py/pz 以免误差累加
            if (rocket_state.status != PRE_LAUNCH && rocket_state.status != LANDED) {
                rocket_state.status = LANDED;
                // 仅在首次切换到 LANDED 时捕捉地面坐标
                CelestialBody& cur_b = SOLAR_SYSTEM[current_soi_index];
                double theta = cur_b.prime_meridian_epoch + (rocket_state.sim_time * 2.0 * PI / cur_b.rotation_period);
                rocket_state.surf_px = rocket_state.px * std::cos(-theta) - rocket_state.py * std::sin(-theta);
                rocket_state.surf_py = rocket_state.px * std::sin(-theta) + rocket_state.py * std::cos(-theta);
                rocket_state.surf_pz = rocket_state.pz;
            }
            
            rocket_state.vx = 0; rocket_state.vy = 0; rocket_state.vz = 0;
            rocket_state.velocity = 0; rocket_state.local_vx = 0;
            rocket_state.ang_vel = 0; rocket_state.ang_vel_z = 0;
        }
        PhysicsSystem::FastGravityUpdate(rocket_state, rocket_config, dt * time_warp);
    } else {
        // === Auto-Execute Maneuver Node ===
        bool mnv_autopilot_active = false;
        if (!rocket_state.maneuvers.empty() && (rocket_state.status == ASCEND || rocket_state.status == DESCEND)) {
            auto& node = rocket_state.maneuvers[0];
            
            // Compute burn time for this node
            double total_dv_mag = node.delta_v.length();
            double current_mass = rocket_state.fuel + rocket_config.dry_mass + rocket_config.upper_stages_mass;
            double max_thrust = 0;
            if (rocket_state.current_stage < (int)rocket_config.stage_configs.size())
                max_thrust = rocket_config.stage_configs[rocket_state.current_stage].thrust;
            double ve = rocket_config.specific_impulse * 9.80665;
            double estimated_burn_time = 0;
            if (max_thrust > 0 && ve > 0 && total_dv_mag > 0) {
                double m_0 = current_mass;
                double m_f = m_0 * exp(-total_dv_mag / ve);
                double delta_m = m_0 - m_f;
                double mdot = max_thrust / ve;
                estimated_burn_time = delta_m / mdot;
            }
            node.burn_duration = estimated_burn_time;
            
            double time_to_node = node.sim_time - rocket_state.sim_time;
            double burn_start_offset = 0; // Ignition precisely at node per user request
            double time_to_burn_start = time_to_node - burn_start_offset;
            
            // Populate velocity snapshot when approaching burn time (first time only)
            if (!node.snap_valid && time_to_burn_start < 5.0) {
                // Use the node's specific reference body
                int ref_idx = (node.ref_body >= 0) ? node.ref_body : current_soi_index;
                CelestialBody& ref_b = SOLAR_SYSTEM[ref_idx];
                
                // 1. Get celestial state AT node time
                double rbpx, rbpy, rbpz, rbvx, rbvy, rbvz;
                PhysicsSystem::GetCelestialStateAt(ref_idx, node.sim_time, rbpx, rbpy, rbpz, rbvx, rbvy, rbvz);
                
                // 2. Project rocket state to node time relative to THAT body
                double mu_ref = G_const * ref_b.mass;
                double npx, npy, npz, nvx, nvy, nvz;
                get3DStateAtTime(rocket_state.px, rocket_state.py, rocket_state.pz, 
                                rocket_state.vx, rocket_state.vy, rocket_state.vz, 
                                mu_ref, node.sim_time - rocket_state.sim_time, 
                                npx, npy, npz, nvx, nvy, nvz);

                // 3. Compute burn direction in world space at node time
                ManeuverFrame frame = ManeuverSystem::getFrame(Vec3((float)npx, (float)npy, (float)npz), Vec3((float)nvx, (float)nvy, (float)nvz));
                Vec3 target_dv_world = (frame.prograde * node.delta_v.x + 
                                       frame.normal   * node.delta_v.y + 
                                       frame.radial   * node.delta_v.z);
                
                node.locked_burn_dir = target_dv_world.normalized();
                
                // 4. Capture TARGET ABSOLUTE STATE for guidance
                node.snap_px = rbpx + npx;
                node.snap_py = rbpy + npy;
                node.snap_pz = rbpz + npz;
                node.snap_vx = rbvx + nvx + target_dv_world.x;
                node.snap_vy = rbvy + nvy + target_dv_world.y;
                node.snap_vz = rbvz + nvz + target_dv_world.z;
                node.snap_time = node.sim_time; // Fallback: impulsive completion time
                node.snap_valid = true;
            }

            if (auto_exec_mnv) {
                // Compute remaining dv
                double remaining_dv = total_dv_mag;
                if (node.snap_valid) {
                    int ref_idx = (node.ref_body >= 0) ? node.ref_body : current_soi_index;
                    CelestialBody& ref_b = SOLAR_SYSTEM[ref_idx];
                    
                    // 1. Current relative state
                    double cur_rel_vx = rocket_state.vx + SOLAR_SYSTEM[current_soi_index].vx - ref_b.vx;
                    double cur_rel_vy = rocket_state.vy + SOLAR_SYSTEM[current_soi_index].vy - ref_b.vy;
                    double cur_rel_vz = rocket_state.vz + SOLAR_SYSTEM[current_soi_index].vz - ref_b.vz;
                    
                    // 2. Propagate Goal Orbit to current time
                    double mu_ref = G_const * ref_b.mass;
                    double snap_rel_px = node.snap_px - ref_b.px;
                    double snap_rel_py = node.snap_py - ref_b.py;
                    double snap_rel_pz = node.snap_pz - ref_b.pz;
                    double snap_rel_vx = node.snap_vx - ref_b.vx;
                    double snap_rel_vy = node.snap_vy - ref_b.vy;
                    double snap_rel_vz = node.snap_vz - ref_b.vz;
                    
                    double dt_snap = rocket_state.sim_time - node.snap_time;
                    double tpx, tpy, tpz, tvx, tvy, tvz;
                    get3DStateAtTime(snap_rel_px, snap_rel_py, snap_rel_pz, snap_rel_vx, snap_rel_vy, snap_rel_vz, 
                                     mu_ref, dt_snap, tpx, tpy, tpz, tvx, tvy, tvz);
                    
                    Vec3 rem_v((float)(tvx - cur_rel_vx), (float)(tvy - cur_rel_vy), (float)(tvz - cur_rel_vz));
                    
                    // PROJECTION-BASED Delta-V for shutdown precision
                    remaining_dv = (double)rem_v.dot(node.locked_burn_dir);
                    
                    double total_error_mag = (double)rem_v.length();
                    if (total_error_mag < 0.05) remaining_dv = 0; 
                }
            
            // Phase 1: Point toward burn direction (60s before burn start)
            if (time_to_burn_start < 60.0) {
                mnv_autopilot_active = true;
                
                // Compute burn direction in world space
                Vec3 burn_dir(0,0,0);
                if (node.snap_valid) {
                    // Inertial guidance locking
                    burn_dir = node.locked_burn_dir;
                } 
                
                if (burn_dir.length() < 0.1f) {
                    // Pre-burn or fallback: Point towards the planned prograde at the node time
                    double mu = 6.67430e-11 * SOLAR_SYSTEM[current_soi_index].mass;
                    double npx, npy, npz, nvx, nvy, nvz;
                    double dt_node = node.sim_time - rocket_state.sim_time;
                    get3DStateAtTime(rocket_state.px, rocket_state.py, rocket_state.pz, rocket_state.vx, rocket_state.vy, rocket_state.vz, mu, dt_node, npx, npy, npz, nvx, nvy, nvz);
                    ManeuverFrame frame = ManeuverSystem::getFrame(Vec3((float)npx, (float)npy, (float)npz), Vec3((float)nvx, (float)nvy, (float)nvz));
                    burn_dir = (frame.prograde * node.delta_v.x + frame.normal * node.delta_v.y + frame.radial * node.delta_v.z).normalized();
                }

                // Attitude control via PD controller
                Vec3 fwd = rocket_state.attitude.forward();
                float dot_prod = fwd.dot(burn_dir);
                Vec3 error_axis = fwd.cross(burn_dir);
                float error_mag = error_axis.length();

                // Singularity fix: if looking exactly AWAY from target
                if (dot_prod < -0.999f && error_mag < 0.01f) {
                    error_axis = rocket_state.attitude.right(); 
                    error_mag = 1.0f; 
                }

                if (error_mag > 0.001f) {
                    error_axis = error_axis / error_mag;
                    float error_angle = std::asin(std::min(error_mag, 1.0f));
                    if (dot_prod < 0) error_angle = (float)PI - error_angle; 

                    // PD torque control with MOI scaling
                    double total_mass = rocket_config.dry_mass + rocket_state.fuel + rocket_config.upper_stages_mass;
                    float moi = (float)(50000.0 * (total_mass / 50000.0));
                    float kp = moi * 32.0f;
                    float kd = moi * 12.0f; // Critical damping (zeta ~ 1.0)
                    
                    // Decompose error into rocket's local axes
                    Vec3 right_axis = rocket_state.attitude.right();
                    Vec3 up_axis = rocket_state.attitude.up();
                    float err_pitch = error_axis.dot(right_axis) * error_angle;
                    float err_yaw = error_axis.dot(up_axis) * error_angle;
                    
                    control_input.torque_cmd = kp * err_yaw - kd * (float)rocket_state.ang_vel;
                    control_input.torque_cmd_z = kp * err_pitch - kd * (float)rocket_state.ang_vel_z;
                } else {
                    double total_mass = rocket_config.dry_mass + rocket_state.fuel + rocket_config.upper_stages_mass;
                    float moi = (float)(50000.0 * (total_mass / 50000.0));
                    control_input.torque_cmd = -moi * 8.0f * (float)rocket_state.ang_vel;
                    control_input.torque_cmd_z = -moi * 8.0f * (float)rocket_state.ang_vel_z;
                }
                double total_mass = rocket_config.dry_mass + rocket_state.fuel + rocket_config.upper_stages_mass;
                float moi = (float)(50000.0 * (total_mass / 50000.0));
                control_input.torque_cmd_roll = -moi * 8.0f * (float)rocket_state.ang_vel_roll;
            }
            
            // Phase 2: Throttle control
            if (time_to_burn_start <= 0 && remaining_dv > 0.5) {
                // Currently burning
                control_input.throttle = 1.0;
                mnv_autopilot_active = true;
            } else if (node.snap_valid && remaining_dv <= 0.5) {
                // Burn complete!
                control_input.throttle = 0;
                rocket_state.mission_msg = "MNV COMPLETE";
                // Remove the completed maneuver node
                rocket_state.maneuvers.erase(rocket_state.maneuvers.begin());
                if (rocket_state.selected_maneuver_index >= 0) rocket_state.selected_maneuver_index--;
                if (rocket_state.maneuvers.empty()) {
                    auto_exec_mnv = false;
                } 
            } else if (!mnv_autopilot_active) {
                control_input.throttle = 0; // Not yet time
            }
            }
        }
        
        // 普通循环执行实现加速
        for (int i = 0; i < time_warp; i++) {
          // Skip normal autopilot if maneuver auto-exec is controlling attitude
          if (!mnv_autopilot_active) {
              if (rocket_state.auto_mode) ControlSystem::UpdateAutoPilot(rocket_state, rocket_config, control_input, dt);
              else ControlSystem::UpdateManualControl(rocket_state, rocket_config, control_input, manual, dt);
          }
          PhysicsSystem::Update(rocket_state, rocket_config, control_input, dt);
          
          // Auto-staging: when current stage fuel is depleted, advance to next
          if (StageManager::IsCurrentStageEmpty(rocket_state) 
              && rocket_state.current_stage < rocket_state.total_stages - 1
              && (rocket_state.status == ASCEND || rocket_state.status == DESCEND)) {
              StageManager::SeparateStage(rocket_state, rocket_config);
          }
          
          if (rocket_state.status == LANDED || rocket_state.status == CRASHED) break;
        }

        // --- 额外一步物理更新 (用于尾烟特效) ---
        if (time_warp == 1) {
            PhysicsSystem::EmitSmoke(rocket_state, rocket_config, dt);
            PhysicsSystem::UpdateSmoke(rocket_state, dt);
        }
    }

    // 只有每隔一定帧数才打印，防止控制台看不清
    if (frame % 10 == 0)
      Report_Status(rocket_state, control_input);

    std::this_thread::sleep_for(std::chrono::milliseconds(20)); // 限制帧率

    //画面刷新
    // NOTE: We move camera-centric logic into the rendering pass below to ensure
    // that visuals respond to the camera's location, not just the rocket's.

    // === 3D/HUD 共享变量与姿态计算 ===
    double ws_d = 0.001;
    Quat rocketQuat;
    Vec3 rocketUp, localNorth, localRight;
    float local_xy_mag = 0;

    // View variables shared with HUD for coordinate projection
    double ro_x = 0, ro_y = 0, ro_z = 0;
    Mat4 viewMat;
    Mat4 macroProjMat;


    // ================= 3D 渲染通道 =================
    {
      float earth_r = (float)EARTH_RADIUS * (float)ws_d;
      double r_px = rocket_state.abs_px * ws_d;
      double r_py = rocket_state.abs_py * ws_d;
      double r_pz = rocket_state.abs_pz * ws_d;
      
      CelestialBody& sun_body = SOLAR_SYSTEM[0];
      double sun_px = sun_body.px * ws_d;
      double sun_py = sun_body.py * ws_d;
      double sun_pz = sun_body.pz * ws_d;
      float sun_radius = (float)sun_body.radius * ws_d;
      double sun_dist_d = sqrt(r_px*r_px + r_py*r_py + r_pz*r_pz);

      // --- 按键监听：视点切换 ---
      static int focus_target = 3; // Default to Earth
      static bool comma_prev = false;
      static bool period_prev = false;
      bool comma_now = glfwGetKey(window, GLFW_KEY_COMMA) == GLFW_PRESS;
      bool period_now = glfwGetKey(window, GLFW_KEY_PERIOD) == GLFW_PRESS;
      if (comma_now && !comma_prev) {
          focus_target--;
          if (focus_target < 0) focus_target = SOLAR_SYSTEM.size() - 1;
      }
      if (period_now && !period_prev) {
          focus_target++;
          if (focus_target >= SOLAR_SYSTEM.size()) focus_target = 0;
      }
      comma_prev = comma_now; period_prev = period_now;

      // --- FLOATING ORIGIN FIX (True Double Precision) ---
      // 决定渲染世界的绝对中心：Camera Target
      if (cam_mode_3d == 0 || cam_mode_3d == 1) {
          ro_x = r_px; ro_y = r_py; ro_z = r_pz;
      } else if (cam_mode_3d == 2) {
          ro_x = SOLAR_SYSTEM[focus_target].px * ws_d; 
          ro_y = SOLAR_SYSTEM[focus_target].py * ws_d; 
          ro_z = SOLAR_SYSTEM[focus_target].pz * ws_d; 
      } else {
          // Free Camera Mode (cam_mode_3d == 3)
          if (!free_cam_init) {
              free_cam_px = rocket_state.px;
              free_cam_py = rocket_state.py;
              free_cam_pz = rocket_state.pz;
              free_cam_init = true;
          }
          // Apply movement based on WASD keys
          float fwd_move = 0.0f;
          float side_move = 0.0f;
          if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS) fwd_move += 1.0f;
          if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS) fwd_move -= 1.0f;
          if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS) side_move -= 1.0f;
          if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS) side_move += 1.0f;

          Vec3 gaze_dir = cam_quat.rotate(Vec3(0, 0, -1));
          Vec3 side_dir = cam_quat.rotate(Vec3(1, 0, 0));
          
          free_cam_px += (double)(gaze_dir.x * fwd_move + side_dir.x * side_move) * (double)free_cam_move_speed * dt;
          free_cam_py += (double)(gaze_dir.y * fwd_move + side_dir.y * side_move) * (double)free_cam_move_speed * dt;
          free_cam_pz += (double)(gaze_dir.z * fwd_move + side_dir.z * side_move) * (double)free_cam_move_speed * dt;
          
          ro_x = (free_cam_px + SOLAR_SYSTEM[current_soi_index].px) * ws_d;
          ro_y = (free_cam_py + SOLAR_SYSTEM[current_soi_index].py) * ws_d;
          ro_z = (free_cam_pz + SOLAR_SYSTEM[current_soi_index].pz) * ws_d;
      }
      
      // 计算相对于渲染中心的 3D 物体坐标 (直接转成 float 后不会再有远距离网格撕裂)
      Vec3 renderEarth((float)(SOLAR_SYSTEM[current_soi_index].px * ws_d - ro_x), (float)(SOLAR_SYSTEM[current_soi_index].py * ws_d - ro_y), (float)(SOLAR_SYSTEM[current_soi_index].pz * ws_d - ro_z));
      Vec3 renderSun((float)(sun_px - ro_x), (float)(sun_py - ro_y), (float)(sun_pz - ro_z));
      Vec3 renderRocketBase((float)(r_px - ro_x), (float)(r_py - ro_y), (float)(r_pz - ro_z));

      // 更新全局光照方向 (指向太阳)
      Vec3 lightVec = renderSun - renderRocketBase;
      r3d->lightDir = lightVec.normalized();

      // 【深空姿态锁定】：基于火箭在当前参考星系的本地坐标(以中心天体为原点)计算向上的表面法线！
      double local_dist_sq = rocket_state.px*rocket_state.px + rocket_state.py*rocket_state.py + rocket_state.pz*rocket_state.pz;
      double local_dist = sqrt(local_dist_sq);
      rocketUp = Vec3((float)(rocket_state.px / local_dist), (float)(rocket_state.py / local_dist), (float)(rocket_state.pz / local_dist));
      if (rocket_state.altitude > 2000000.0) { // 2000公里外，完全脱离近地轨道，进入深空
          rocketUp = Vec3(0.0f, 1.0f, 0.0f);
      }
      
      // ===== BUILD ROCKET ATTITUDE =====
      static int last_soi = -1;
      if (last_soi != current_soi_index) {
          rocket_state.attitude_initialized = false;
          last_soi = current_soi_index;
      }

      if (!rocket_state.attitude_initialized) {
           rocket_state.attitude = Quat::fromEuler((float)rocket_state.angle, (float)rocket_state.angle_z, (float)rocket_state.angle_roll);
           rocket_state.attitude_initialized = true;
      }
      rocketQuat = rocket_state.attitude;
      
      // Update rocketDir for rendering logic
      Vec3 rocketDir = rocketQuat.rotate(Vec3(0.0f, 1.0f, 0.0f));

      // --- 计算用于 Orbit 摄像机的轨道参考系 (基于当前相对于 SOI 的坐标) ---
      // Restore these for Orbit Camera mode (lines 898-900)
      local_xy_mag = sqrt(rocketUp.x*rocketUp.x + rocketUp.y*rocketUp.y);
      if (local_xy_mag > 1e-4) {
          localRight = Vec3((float)(-rocketUp.y/local_xy_mag), (float)(rocketUp.x/local_xy_mag), 0.0f);
      } else {
          localRight = Vec3(1.0f, 0.0f, 0.0f);
      }
      localNorth = rocketUp.cross(localRight).normalized();
      
      Vec3 v_vec_rel((float)rocket_state.vx, (float)rocket_state.vy, (float)rocket_state.vz);
      Vec3 p_vec_rel((float)rocket_state.px, (float)rocket_state.py, (float)rocket_state.pz);
      Vec3 h_vec_rel = p_vec_rel.cross(v_vec_rel);
      Vec3 orbit_normal_rel = h_vec_rel.normalized();
      if (orbit_normal_rel.length() < 0.01f) orbit_normal_rel = Vec3(0, 0, 1);
      Vec3 prograde_rel = v_vec_rel.normalized();
      if (prograde_rel.length() < 0.01f) prograde_rel = localNorth;
      Vec3 radial_rel = orbit_normal_rel.cross(prograde_rel).normalized();

      // 火箭尺寸
      float rocket_vis_scale = 1.0f;
      float rh = (float)rocket_config.height * (float)ws_d * rocket_vis_scale;
      float rw_3d = (float)rocket_config.diameter * (float)ws_d * 0.5f * rocket_vis_scale;

      // 火箭渲染锚点（将火箭向上偏移，解决2D物理质心带来的穿模问题）
      Vec3 renderRocketPos = renderRocketBase + rocketUp * (rh * 0.425f);

      // 相机设置
      int ww, wh;
      glfwGetFramebufferSize(window, &ww, &wh);
      r3d->initTAA(ww, wh);
      float aspect = (float)ww / (float)wh;

      Vec3 camEye_rel, camTarget_rel, camUpVec;
      if (cam_mode_3d == 0) {
        // --- 优化型 Orbit 视角 (带平滑过渡与时间平滑) ---
        double apo_tmp = 0, peri_tmp = 0;
        PhysicsSystem::getOrbitParams(rocket_state, apo_tmp, peri_tmp);
        
        // 1. 物理上的融合因子 (target_t)
        // 扩大过渡区间: 80km ~ 160km 让物理变化更平级
        float peri_min = 80000.0f;
        float peri_max = 160000.0f;
        float target_t = (float)(peri_tmp - peri_min) / (peri_max - peri_min);
        target_t = std::max(0.0f, std::min(1.0f, target_t));

        // 2. 时间上的融合因子 (current_t)
        // 使用静态变量存储当前视觉状态，实现“平滑延迟”效果
        // 这样即使时间加速或参数突变，视觉上也会在 1-2 秒内平滑过渡
        static float current_t = target_t;
        float lerp_speed = 1.5f; // 数值越小过渡越慢越丝滑
        float visual_dt = 0.02f; // 假设 ~50fps 的渲染间隔
        current_t += (target_t - current_t) * (1.0f - expf(-lerp_speed * visual_dt));

        float orbit_dist = rh * 8.0f * cam_zoom_chase;
        
        // 计算两种模式的方向
        Vec3 ground_view = localNorth * cosf(orbit_yaw) * cosf(orbit_pitch) + 
                           localRight * sinf(orbit_yaw) * cosf(orbit_pitch) + 
                           rocketUp   * sinf(orbit_pitch);
                           
        Vec3 orbit_view  = prograde_rel * cosf(orbit_yaw) * cosf(orbit_pitch) + 
                           radial_rel   * sinf(orbit_yaw) * cosf(orbit_pitch) + 
                           orbit_normal_rel * sinf(orbit_pitch);

        // 使用 current_t 进行插值融合
        Vec3 view_dir = Vec3::lerp(ground_view, orbit_view, current_t).normalized();
        camUpVec = Vec3::lerp(rocketUp, orbit_normal_rel, current_t).normalized();
        
        camEye_rel = renderRocketPos + view_dir * orbit_dist;
        camTarget_rel = renderRocketPos;
      } else if (cam_mode_3d == 1) {
        float chase_dist = rh * 8.0f * cam_zoom_chase;
        Vec3 chase_base = rocketDir * (-chase_dist * 0.4f) + rocketUp * (chase_dist * 0.15f);
        Vec3 slight_off = cam_quat.rotate(Vec3(0.0f, 0.0f, 1.0f));
        camEye_rel = renderRocketPos + chase_base + slight_off * (chase_dist * 0.05f);
        camTarget_rel = renderRocketPos + rocketDir * (rh * 3.0f);
        camUpVec = rocketUp;
      } else if (cam_mode_3d == 2) {
        float base_pan_radius = (float)SOLAR_SYSTEM[focus_target].radius * (float)ws_d;
        float pan_dist = fmaxf(base_pan_radius * 4.0f, earth_r * 2.0f) * cam_zoom_pan;
        if (focus_target == 0) pan_dist = sun_radius * 4.0f * cam_zoom_pan;
        
        Vec3 cam_offset = cam_quat.rotate(Vec3(0.0f, 0.0f, pan_dist));
        Vec3 renderFocus((float)(SOLAR_SYSTEM[focus_target].px * ws_d - ro_x), 
                         (float)(SOLAR_SYSTEM[focus_target].py * ws_d - ro_y), 
                         (float)(SOLAR_SYSTEM[focus_target].pz * ws_d - ro_z));
        camEye_rel = renderFocus + cam_offset;
        camTarget_rel = renderFocus;
        camUpVec = cam_quat.rotate(Vec3(0.0f, 1.0f, 0.0f));
      } else {
        // Free Camera (cam_mode_3d == 3)
        camEye_rel = Vec3(0, 0, 0); // Origin at camera
        camTarget_rel = cam_quat.rotate(Vec3(0.0f, 0.0f, -1.0f));
        camUpVec = cam_quat.rotate(Vec3(0.0f, 1.0f, 0.0f));
      }

      viewMat = Mat4::lookAt(camEye_rel, camTarget_rel, camUpVec);

      // --- Camera-Centric Visuals (Day/Night & Atmosphere) ---
      double day_blend = 1.0;
      float alt_factor = 1.0f;
      {
          // Absolute camera position in heliocentric km (ws_d already applied)
          double cam_abs_x = camEye_rel.x + ro_x;
          double cam_abs_y = camEye_rel.y + ro_y;
          double cam_abs_z = camEye_rel.z + ro_z;

          // Find closest celestial body to the camera to determine local environment
          int closest_idx = 0;
          double min_dist_sq = 1e30;
          for (int i = 1; i < SOLAR_SYSTEM.size(); i++) {
              double dx = SOLAR_SYSTEM[i].px * ws_d - cam_abs_x;
              double dy = SOLAR_SYSTEM[i].py * ws_d - cam_abs_y;
              double dz = SOLAR_SYSTEM[i].pz * ws_d - cam_abs_z;
              double d2 = dx*dx + dy*dy + dz*dz;
              if (d2 < min_dist_sq) { min_dist_sq = d2; closest_idx = i; }
          }

          CelestialBody& near_body = SOLAR_SYSTEM[closest_idx];
          double dist_to_center = sqrt(min_dist_sq);
          double body_r_km = near_body.radius * ws_d;
          double cam_alt_km = dist_to_center - body_r_km;
          
          // Atmosphere factor (0.0 at ground, 1.0 in space)
          alt_factor = (float)fmin(fmax(cam_alt_km / 50.0, 0.0), 1.0);

          // Shadow check (is this body blocking the sun from the camera's POV?)
          double to_sun_x = -cam_abs_x;
          double to_sun_y = -cam_abs_y;
          double to_sun_z = -cam_abs_z;
          double to_sun_len = sqrt(to_sun_x*to_sun_x + to_sun_y*to_sun_y + to_sun_z*to_sun_z);
          if (to_sun_len > 1.0) {
              double d_x = to_sun_x / to_sun_len;
              double d_y = to_sun_y / to_sun_len;
              double d_z = to_sun_z / to_sun_len;
              // Vector from camera to body center
              double oc_x = near_body.px * ws_d - cam_abs_x;
              double oc_y = near_body.py * ws_d - cam_abs_y;
              double oc_z = near_body.pz * ws_d - cam_abs_z;
              double t_closest = oc_x*d_x + oc_y*d_y + oc_z*d_z;
              if (t_closest > 0 && t_closest < to_sun_len) {
                  double cp_x = oc_x - d_x * t_closest;
                  double cp_y = oc_y - d_y * t_closest;
                  double cp_z = oc_z - d_z * t_closest;
                  double closest_dist = sqrt(cp_x*cp_x + cp_y*cp_y + cp_z*cp_z);
                  if (closest_dist < body_r_km) day_blend = 0.0;
                  else if (closest_dist < body_r_km * 1.02) day_blend = (closest_dist - body_r_km) / (body_r_km * 0.02);
              }
          }
      }

      // Clear screen with camera-centric sky color
      if (cam_mode_3d == 2) {
          glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
      } else {
          float sky_day = (float)(day_blend * (1.0 - alt_factor));
          glClearColor(0.5f * sky_day, 0.7f * sky_day, 1.0f * sky_day, 1.0f);
      }
      glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

      // 动态远裁剪面 
      float cam_dist = (camEye_rel - camTarget_rel).length();
      // Ensure far clipping plane covers the entire solar system (expanded to 1000 AU)
      float far_plane = 1000.0f * 149597870.0f; // 1000 AU in km (ws_d)

      // =========== PASS 1: MACRO BACKGROUND ===========
      // Compute a smart near plane that keeps the depth buffer ratio (far/near) within
      // ~1e6 to avoid z-fighting on planet surfaces. Find the closest planet surface
      // distance from the camera and use a fraction of that as the near plane.
      float closest_planet_dist = far_plane;
      for (size_t i = 1; i < SOLAR_SYSTEM.size(); i++) {
          Vec3 rp((float)(SOLAR_SYSTEM[i].px * ws_d - ro_x),
              (float)(SOLAR_SYSTEM[i].py * ws_d - ro_y),
              (float)(SOLAR_SYSTEM[i].pz * ws_d - ro_z));

          float body_r = (float)SOLAR_SYSTEM[i].radius * (float)ws_d;
          float atmo_thickness = (SOLAR_SYSTEM[i].type == GAS_GIANT || SOLAR_SYSTEM[i].type == RINGED_GAS_GIANT) ? body_r * 0.05f : 160.0f;
          float dist_to_center = (camEye_rel - rp).length();

          float true_surf_dist = dist_to_center - body_r;
          
          // Consider terrain displacement (max 25km for Earth)
          float terrain_buffer = (i == 3) ? 25.0f : 0.0f;
          float effective_surf_dist = true_surf_dist - terrain_buffer;

          float atmo_surf_dist = dist_to_center - (body_r + atmo_thickness);

          // Use the atmosphere boundary for camera clipping if it exists
          float geo_dist = (atmo_surf_dist > 0.0f) ? atmo_surf_dist : effective_surf_dist;
          if (geo_dist < closest_planet_dist) {
              closest_planet_dist = geo_dist;
          }
      }
      // Also consider distance to the Sun
      {
          float sun_surf = (camEye_rel - renderSun).length() - sun_radius;
          if (sun_surf < closest_planet_dist)
              closest_planet_dist = sun_surf;
      }
      
      // Industrial Grade Near Plane:
      // We use a small fraction of the distance to the actual surface (including mountains).
      // If we are extremely close (less than 1km), we force a very small near plane for micro-detail.
      float macro_near = fmaxf(0.00001f, closest_planet_dist * 0.05f); // 5% of distance
      macro_near = fminf(macro_near, 1.0f); // Cap macro near plane at 1km
      if (closest_planet_dist < 1.0f) macro_near = fminf(macro_near, 0.0001f); // 10cm when near ground
      macro_near = fmaxf(macro_near, cam_dist * 0.0001f); // but never clip behind target
      
      macroProjMat = Mat4::perspective(0.8f, aspect, macro_near, far_plane);
      
      r3d->beginTAAPass();
      r3d->beginFrame(viewMat, macroProjMat, camEye_rel);

      // ===== SKYBOX: Procedural Starfield + Milky Way =====
      // Calculate vibrancy: 1.0 in space or at night, 0.0 during bright day on ground
      // Note: sky_day factor is used to wash out stars when looking through a lit atmosphere
      float sky_day_local = (float)(day_blend * (1.0 - alt_factor));
      float sky_vibrancy = 1.0f - sky_day_local; 
      r3d->drawSkybox(sky_vibrancy, aspect);

      // ===== 太阳物理本体 =====
      if (cam_mode_3d == 2) { 
          // 仅在全景模式渲染巨型的物理太阳模型避免遮盖火箭本体细节
          Mat4 sunModel = Mat4::scale(Vec3(sun_radius, sun_radius, sun_radius));
          sunModel = Mat4::translate(renderSun) * sunModel;
          // 复用 earthMesh，修改极高环境光(ambient=2.0)让其纯亮发白发黄
          r3d->drawMesh(earthMesh, sunModel, 1.0f, 0.95f, 0.9f, 1.0f, 2.0f); 
          
          // NOTE: Hardcoded Earth orbit removed in favor of procedural gradient orbits for all planets
      }

      // 渲染整个太阳系
      for (size_t i = 1; i < SOLAR_SYSTEM.size(); i++) {
          CelestialBody& b = SOLAR_SYSTEM[i];
          float r = (float)b.radius * ws_d;
          Vec3 renderPlanet((float)(b.px * ws_d - ro_x), (float)(b.py * ws_d - ro_y), (float)(b.pz * ws_d - ro_z));
          
          // 应用主体的自转与极轴倾斜 (黄道坐标系中Z轴为原生北极，XY为轨道面)
          // 默认模型球体的极轴是Y轴，首先需要将它躺平(-90度X轴旋转)对齐Z轴
          Quat align_to_z = Quat::fromAxisAngle(Vec3(1.0f, 0.0f, 0.0f), -PI / 2.0f);
          // 主轴自转，绕Z轴（现在的极轴）旋转
          Quat spin = Quat::fromAxisAngle(Vec3(0.0f, 0.0f, 1.0f), b.prime_meridian_epoch + (rocket_state.sim_time * 2.0 * PI / b.rotation_period));
          // 极轴倾斜，绕X轴侧倾
          Quat tilt = Quat::fromAxisAngle(Vec3(1.0f, 0.0f, 0.0f), b.axial_tilt); 
          Quat rotation_quat = tilt * spin * align_to_z;

          Mat4 planetModel = Mat4::scale(Vec3(r, r, r));
          planetModel = Mat4::fromQuat(rotation_quat) * planetModel; // Apply rotation
          planetModel = Mat4::translate(renderPlanet) * planetModel;
          
          // DO NOT dim the base color with rocket occlusion (that makes the darkest side of Earth pitch black void).
           // Compute per-planet lightDir in double precision for correct sun-facing
          {
              double light_dx = sun_body.px - b.px;
              double light_dy = sun_body.py - b.py;
              double light_dz = sun_body.pz - b.pz;
              double light_len = sqrt(light_dx*light_dx + light_dy*light_dy + light_dz*light_dz);
              if (light_len > 1.0) {
                  r3d->lightDir = Vec3((float)(light_dx/light_len), (float)(light_dy/light_len), (float)(light_dz/light_len));
              }
          }
          r3d->drawPlanet(earthMesh, planetModel, b.type, b.r, b.g, b.b, 1.0f, r, (float)rocket_state.sim_time, (int)i);
          
          if ((b.type == TERRESTRIAL || b.type == GAS_GIANT) && i != 1 && i != 4) {
              // 修改前：float atmo_radius = r * 1.12f;
              // 修改后：使用恒定的物理边界，剥离厚重的多余网格
              float atmo_radius = r + 160.0f;

              Mat4 atmoModel = Mat4::scale(Vec3(atmo_radius, atmo_radius, atmo_radius));
              atmoModel = Mat4::translate(renderPlanet) * atmoModel;

              // New Volumetric Scattering integration with animated hardcore clouds
              r3d->drawAtmosphere(earthMesh, atmoModel, camEye_rel, r3d->lightDir, renderPlanet, r, atmo_radius, (float)rocket_state.sim_time, (int)i, (float)day_blend, show_clouds);
          }
          if (b.type == RINGED_GAS_GIANT) {
              Mat4 ringModel = Mat4::scale(Vec3(r, r, r)); // Mesh is now pre-scaled to R_planet ratios
              ringModel = Mat4::fromQuat(rotation_quat) * ringModel;
              ringModel = Mat4::translate(renderPlanet) * ringModel;
              r3d->drawRing(ringMesh, ringModel, b.r, b.g, b.b, 0.4f);
          }
          
          // 渲染行星轨道和标签 (仅在全景模式下显示)
          if (cam_mode_3d == 2) {
              double a = b.sma_base;
              double e = b.ecc_base;
              double i_inc = b.inc_base;
              double lan = b.lan_base;
              double arg_p = b.arg_peri_base;
              
              double planet_px = b.px; double planet_py = b.py; double planet_pz = b.pz;
              if (i == 4) { planet_px -= SOLAR_SYSTEM[3].px; planet_py -= SOLAR_SYSTEM[3].py; planet_pz -= SOLAR_SYSTEM[3].pz; }
              
              int segs = 181; // 181 points = 180 segments + closure
              
              // Scale orbit line width relative to camera distance so all orbits are visible
              float orbit_center_dist = renderPlanet.length(); // approx dist from render origin to planet
              float cam_to_origin = camEye_rel.length();
              float ref_dist = fmaxf(cam_to_origin, orbit_center_dist);
              float ring_w = fmaxf(earth_r * 0.008f, ref_dist * 0.0035f);
              if (i == 4) ring_w *= 0.5f; // Moon orbit thinner
              
              // Precompute trig for orbital transform
              double c_O = cos(lan), s_O = sin(lan);
              double c_w = cos(arg_p), s_w = sin(arg_p);
              double c_i = cos(i_inc), s_i = sin(i_inc);
              
              // Build ONE continuous ribbon — UNIFORM brightness, fully visible
              std::vector<Vec3> orbit_pts;
              std::vector<Vec4> orbit_cols;
              orbit_pts.reserve(segs);
              orbit_cols.reserve(segs);
              
              // Uniform orbit color — bright, like the rocket orbit reference
              float orb_r = fminf(1.0f, b.r * 0.6f + 0.3f);
              float orb_g = fminf(1.0f, b.g * 0.6f + 0.3f);
              float orb_b = fminf(1.0f, b.b * 0.6f + 0.3f);
              float orb_a = 0.7f;
              
              for(int k = 0; k < segs; k++) {
                  double E_k = (double)k / (segs - 1) * 2.0 * PI;
                  double nu_k = 2.0 * atan2(sqrt(1.0+e)*sin(E_k/2.0), sqrt(1.0-e)*cos(E_k/2.0));
                  double r_dist_k = a * (1.0 - e * cos(E_k));
                  double o_xk = r_dist_k * cos(nu_k);
                  double o_yk = r_dist_k * sin(nu_k);
                  
                  // Transform to heliocentric coordinates (double precision)
                  double wx = (c_O*c_w - s_O*s_w*c_i)*o_xk + (-c_O*s_w - s_O*c_w*c_i)*o_yk;
                  double wy = (s_O*c_w + c_O*s_w*c_i)*o_xk + (-s_O*s_w + c_O*c_w*c_i)*o_yk;
                  double wz = (s_w*s_i)*o_xk + (c_w*s_i)*o_yk;
                  
                  if (i == 4) { // Moon orbits Earth
                      wx += SOLAR_SYSTEM[3].px;
                      wy += SOLAR_SYSTEM[3].py;
                      wz += SOLAR_SYSTEM[3].pz;
                  }
                  
                  orbit_pts.push_back(Vec3(
                      (float)(wx * ws_d - ro_x),
                      (float)(wy * ws_d - ro_y),
                      (float)(wz * ws_d - ro_z)
                  ));
                  orbit_cols.push_back(Vec4(orb_r, orb_g, orb_b, orb_a));
              }
              
              r3d->drawRibbon(orbit_pts, orbit_cols, ring_w);
              
              // Draw planet marker label - size scales with camera distance
              float dist_to_cam = (renderPlanet - camEye_rel).length();
              float marker_size = fmaxf((float)b.radius * (float)ws_d * 2.0f, dist_to_cam * 0.015f);
              r3d->drawBillboard(renderPlanet, marker_size, b.r, b.g, b.b, 0.9f);
          }
      }

      // Restore lightDir to rocket's own Sun direction (for rocket mesh, trajectory rendering, etc.)
      lightVec = renderSun - renderRocketBase;
      r3d->lightDir = lightVec.normalized();

      // ===== 太阳与镜头光晕 (所有模式可见) =====
      std::vector<Vec4> sun_occluders;
      for (size_t i = 1; i < SOLAR_SYSTEM.size(); i++) {
          CelestialBody& b = SOLAR_SYSTEM[i];
          sun_occluders.push_back(Vec4(
              (float)(b.px * ws_d - ro_x),
              (float)(b.py * ws_d - ro_y),
              (float)(b.pz * ws_d - ro_z),
              (float)b.radius * (float)ws_d
          ));
      }
      r3d->drawSunAndFlare(renderSun, sun_occluders, ww, wh);

      // ===== 历史轨迹线 (实际走过的路径) =====
      {
        DVec3 curPos = {r_px, r_py, r_pz};
        if (traj_history.empty()) {
            traj_history.push_back({curPos, {r_px - sun_px, r_py - sun_py, r_pz - sun_pz}});
        } else {
            DVec3 bk = traj_history.back().e;
            double move_dist = sqrt((r_px-bk.x)*(r_px-bk.x) + (r_py-bk.y)*(r_py-bk.y) + (r_pz-bk.z)*(r_pz-bk.z));
            if (move_dist > earth_r * 0.002) {
               traj_history.push_back({curPos, {r_px - sun_px, r_py - sun_py, r_pz - sun_pz}});
               if (traj_history.size() > 800) {
                 traj_history.erase(traj_history.begin());
               }
            }
        }
        
        // 渲染历史轨迹 (更亮实线: 黄绿色), 增加基于相机拉远的线宽补偿 (仅在 Panorama 显示)
        if (cam_mode_3d == 2 && traj_history.size() >= 2) {
           float hist_w = fmaxf(earth_r * 0.01f, cam_dist * 0.0015f);
           float macro_fade = fminf(1.0f, fmaxf(0.0f, (cam_zoom_pan - 0.05f) / 0.1f));
           if (macro_fade > 0.01f) {
              std::vector<Vec3> relative_traj;
              for(auto& pt : traj_history) {
                 double w_px, w_py, w_pz;
                 if (orbit_reference_sun) {
                     w_px = sun_px + pt.s.x;
                     w_py = sun_py + pt.s.y;
                     w_pz = sun_pz + pt.s.z;
                 } else {
                     w_px = pt.e.x;
                     w_py = pt.e.y;
                     w_pz = pt.e.z;
                 }
                 relative_traj.push_back(Vec3((float)(w_px - ro_x), (float)(w_py - ro_y), (float)(w_pz - ro_z)));
              }
              r3d->drawRibbon(relative_traj, hist_w, 0.4f, 1.0f, 0.3f, 0.8f * macro_fade);
           }
        }
      }

      // ===== 火箭自身高亮标注 (方便在远景找到) =====
      if (cam_mode_3d == 2) {
        // Scale marker more aggressively with zoom, with a guaranteed minimum visible size
        float base_marker = earth_r * 0.025f * fmaxf(1.0f, cam_zoom_pan * 1.2f);
        // Guarantee a minimum screen-space size (proportional to camera distance)
        float cam_dist_marker = (renderRocketBase - camEye_rel).length();
        float min_marker = cam_dist_marker * 0.008f; // ~0.8% of camera distance = always visible
        float marker_size = fmaxf(base_marker, min_marker);
        r3d->drawBillboard(renderRocketBase, marker_size, 0.2f, 1.0f, 0.4f, 0.9f);
        // Draw a second, larger but fainter halo for extreme zoom-out findability
        if (cam_zoom_pan > 2.0f) {
            float halo_size = marker_size * 2.5f;
            r3d->drawBillboard(renderRocketBase, halo_size, 0.2f, 1.0f, 0.4f, 0.15f);
        }
      }

      // ===== 轨道预测线 (开普勒轨道) =====
      std::vector<Vec3> draw_points, draw_mnv_points;
      if (cam_mode_3d == 2) {
        if (adv_orbit_enabled) {
            // Perform asynchronous numerical orbit prediction
            if (!rocket_state.prediction_in_progress) {
                // Throttle requests: update every 0.1s of real time if not busy (provides "100x efficiency" at all warp rates)
                static auto last_req_time = std::chrono::steady_clock::now();
                auto now = std::chrono::steady_clock::now();
                float elapsed_real = std::chrono::duration<float>(now - last_req_time).count();

                if (elapsed_real > 0.1f || rocket_state.predicted_path.empty()) {
                    rocket_state.prediction_in_progress = true;
                    last_req_time = now;
                    
                    // Populate heliocentric state for the background predictor
                    CelestialBody& soi = SOLAR_SYSTEM[current_soi_index];
                    rocket_state.abs_px = rocket_state.px + soi.px;
                    rocket_state.abs_py = rocket_state.py + soi.py;
                    rocket_state.abs_pz = rocket_state.pz + soi.pz;
                    rocket_state.abs_vx = rocket_state.vx + soi.vx;
                    rocket_state.abs_vy = rocket_state.vy + soi.vy;
                    rocket_state.abs_vz = rocket_state.vz + soi.vz;
                    
                    // Reset only if engines are active or large drift (1 hour of sim time)
                    bool force_reset = (control_input.throttle > 0.01) || (std::abs(rocket_state.sim_time - rocket_state.last_prediction_sim_time) > 3600.0);
                    orbit_predictor.RequestUpdate(&rocket_state, rocket_state, rocket_config, adv_orbit_pred_days, adv_orbit_iters, adv_orbit_ref_mode, adv_orbit_ref_body, adv_orbit_secondary_ref_body, force_reset);
                }
            }

            {
                std::lock_guard<std::mutex> lock(*rocket_state.path_mutex);
                draw_points = rocket_state.predicted_path;
                draw_mnv_points = rocket_state.predicted_mnv_path;
            }

            // Render predicted paths from async buffers
            float ribbon_w = fmaxf(earth_r * 0.006f, cam_dist * 0.001f);
            
            // Get current reference body position for world reconstruction
            double rb_px, rb_py, rb_pz;
            PhysicsSystem::GetCelestialPositionAt(adv_orbit_ref_body, rocket_state.sim_time, rb_px, rb_py, rb_pz);
            
            // Get transformation from local to inertial (world)
            Quat q_local_to_inertial = PhysicsSystem::GetFrameRotation(adv_orbit_ref_mode, adv_orbit_ref_body, adv_orbit_secondary_ref_body, rocket_state.sim_time);

            static std::vector<Vec3> cached_rel_pts;
            static std::vector<Vec3> cached_mnv_rel_pts;
            static size_t last_draw_points_size = 0;
            static size_t last_draw_mnv_points_size = 0;
            static Vec3 last_first_pt, last_mnv_first_pt;

            if (!draw_points.empty()) {
                bool needs_update = (draw_points.size() != last_draw_points_size) || (draw_points[0].x != last_first_pt.x);
                if (needs_update) {
                    cached_rel_pts = CatmullRomSpline::interpolate(draw_points, 8);
                    last_draw_points_size = draw_points.size();
                    last_first_pt = draw_points[0];
                }
                
                std::vector<Vec3> world_pts;
                for (const auto& p : cached_rel_pts) {
                    Vec3 p_rot = q_local_to_inertial.rotate(p);
                    double wx = (rb_px + p_rot.x) * ws_d - ro_x;
                    double wy = (rb_py + p_rot.y) * ws_d - ro_y;
                    double wz = (rb_pz + p_rot.z) * ws_d - ro_z;
                    world_pts.push_back(Vec3((float)wx, (float)wy, (float)wz));
                }
                r3d->drawRibbon(world_pts, ribbon_w, 0.4f, 0.8f, 1.0f, 0.85f);
            }

            if (!draw_mnv_points.empty()) {
                bool needs_update = (draw_mnv_points.size() != last_draw_mnv_points_size) || (draw_mnv_points[0].x != last_mnv_first_pt.x);
                if (needs_update) {
                    cached_mnv_rel_pts = CatmullRomSpline::interpolate(draw_mnv_points, 8);
                    last_draw_mnv_points_size = draw_mnv_points.size();
                    last_mnv_first_pt = draw_mnv_points[0];
                }

                std::vector<Vec3> world_mnv_pts;
                for (const auto& p : cached_mnv_rel_pts) {
                    Vec3 p_rot = q_local_to_inertial.rotate(p);
                    double wx = (rb_px + p_rot.x) * ws_d - ro_x;
                    double wy = (rb_py + p_rot.y) * ws_d - ro_y;
                    double wz = (rb_pz + p_rot.z) * ws_d - ro_z;
                    world_mnv_pts.push_back(Vec3((float)wx, (float)wy, (float)wz));
                }
                
                // Efficient dashed rendering: draw larger batches
                for (size_t s = 0; s < world_mnv_pts.size(); s += 20) {
                    std::vector<Vec3> dash;
                    for (size_t j = 0; j < 12 && (s + j < world_mnv_pts.size()); j++) {
                        dash.push_back(world_mnv_pts[s + j]);
                    }
                    if (dash.size() >= 2) r3d->drawRibbon(dash, ribbon_w, 1.0f, 0.6f, 0.1f, 0.9f);
                }
            }
            // Restore current sim state
            PhysicsSystem::UpdateCelestialBodies(rocket_state.sim_time);

        } else {
            // ==========================================
            // STANDARD ORBIT PREDICTION: KEPLERIAN (SOI)
            // ==========================================
            // We calculate and draw BOTH Earth-relative AND Sun-relative orbits concurrently!
            for (int ref_idx = 0; ref_idx < 2; ref_idx++) {
              bool is_sun_ref = (ref_idx == 1);

          // 选择参考系
          double G_const = 6.67430e-11;
          double mu_body = is_sun_ref ? (G_const * SOLAR_SYSTEM[0].mass) : (G_const * SOLAR_SYSTEM[current_soi_index].mass);
          
          // 全物理量双精度计算 (标准米)
          double abs_px = rocket_state.px, abs_py = rocket_state.py, abs_pz = rocket_state.pz;
          double abs_vx = rocket_state.vx, abs_vy = rocket_state.vy, abs_vz = rocket_state.vz;

          if (is_sun_ref && current_soi_index != 0) {
              CelestialBody& cb = SOLAR_SYSTEM[current_soi_index];
              abs_px += cb.px; abs_py += cb.py; abs_pz += cb.pz;
              abs_vx += cb.vx; abs_vy += cb.vy; abs_vz += cb.vz;
          }

          double r_len = sqrt(abs_px*abs_px + abs_py*abs_py + abs_pz*abs_pz);
          double v_len = sqrt(abs_vx*abs_vx + abs_vy*abs_vy + abs_vz*abs_vz);

          if (v_len > 0.001 && r_len > SOLAR_SYSTEM[is_sun_ref ? 0 : current_soi_index].radius * 0.5) {
            double energy = 0.5 * v_len * v_len - mu_body / r_len;
            
            Vec3 h_vec( (float)(abs_py * abs_vz - abs_pz * abs_vy), 
                        (float)(abs_pz * abs_vx - abs_px * abs_vz),
                        (float)(abs_px * abs_vy - abs_py * abs_vx) );

            float h = h_vec.length();
            double a = -mu_body / (2.0 * energy);
            Vec3 v_vec((float)abs_vx, (float)abs_vy, (float)abs_vz);
            Vec3 p_vec((float)abs_px, (float)abs_py, (float)abs_pz);
            
            Vec3 e_vec = v_vec.cross(h_vec) / (float)mu_body - p_vec / (float)r_len;
            float ecc = e_vec.length();

            float opacity = (is_sun_ref == orbit_reference_sun) ? 0.9f : 0.3f;

            if (ecc < 1.0f) {
              // --- 椭圆轨道 (a > 0) ---
              float b = (float)a * sqrtf(fmaxf(0.0f, 1.0f - ecc * ecc));
              Vec3 e_dir = ecc > 1e-6f ? e_vec / ecc : Vec3(1.0f, 0.0f, 0.0f);
              Vec3 perp_dir = h_vec.normalized().cross(e_dir);

              float periapsis = (float)a * (1.0f - ecc);
              float apoapsis = (float)a * (1.0f + ecc);
              bool will_reenter = periapsis < SOLAR_SYSTEM[is_sun_ref ? 0 : current_soi_index].radius && !is_sun_ref;

              Vec3 center_off = e_dir * (-(float)a * ecc);

              // 生成预测轨迹点集
              std::vector<Vec3> orbit_points;
              int orbit_segs = 120;
              float best_orb_dist = 1.0f;
              float best_orb_ang = -1.0f;
              Vec3 best_orb_pt_tmp;
              
              double n_mean_mot = sqrt(mu_body / (a * a * a));
              
              double mx_curr, my_curr; 
              glfwGetCursorPos(window, &mx_curr, &my_curr);
              float mxf = (float)(mx_curr / ww * 2.0 - 1.0);
              float myf = (float)(1.0 - my_curr / wh * 2.0);

              for (int i = 0; i <= orbit_segs; i++) {
                float ang = (float)i / orbit_segs * 6.2831853f;
                Vec3 pt_rel = center_off + e_dir * ((float)a * cosf(ang)) + perp_dir * (b * sinf(ang));
                double px = pt_rel.x, py = pt_rel.y, pz = pt_rel.z;
                if (!is_sun_ref) { 
                    px += SOLAR_SYSTEM[current_soi_index].px; 
                    py += SOLAR_SYSTEM[current_soi_index].py; 
                    pz += SOLAR_SYSTEM[current_soi_index].pz; 
                }
                Vec3 pt = Vec3((float)(px * ws_d - ro_x), (float)(py * ws_d - ro_y), (float)(pz * ws_d - ro_z));
                // Do not clip orbit points beneath the surface so users can see where they crash
                orbit_points.push_back(pt);

                // --- Maneuver Click Hit-test ---
                if (is_sun_ref == orbit_reference_sun) {
                    Vec2 scr = ManeuverSystem::projectToScreen(pt, viewMat, macroProjMat, (float)ww/wh);
                    float d = sqrtf(powf(scr.x - mxf, 2) + powf(scr.y - myf, 2));
                    if (d < best_orb_dist) {
                        best_orb_dist = d;
                        best_orb_ang = ang;
                        best_orb_pt_tmp = pt;
                    }
                }
              }
              
              // Store best hit for this ref frame if it's the active one
              if (is_sun_ref == orbit_reference_sun && best_orb_dist < 0.05f) {
                  global_best_ang = best_orb_ang;
                  global_best_mu = mu_body;
                  global_best_a = a;
                  global_best_ecc = ecc;
                  global_best_pt = best_orb_pt_tmp;
                  global_best_center = center_off;
                  global_best_e_dir = e_dir;
                  global_best_perp_dir = perp_dir;
                  global_best_ref_node = is_sun_ref ? 0 : current_soi_index;

                  double n = sqrt(mu_body / (a * a * a));
                  double cos_E = (a - r_len) / (a * ecc);
                  double sin_E = (abs_px*abs_vx + abs_py*abs_vy + abs_pz*abs_vz) / (ecc * sqrt(mu_body * a));
                  double E0 = atan2(sin_E, cos_E);
                  global_current_M0 = E0 - ecc * sin(E0);
                  global_current_n = n;
              }
              
              // 渲染预测轨迹
              float pred_w = fmaxf(earth_r * 0.01f, cam_dist * 0.0015f);
              float macro_fade = fminf(1.0f, fmaxf(0.0f, (cam_zoom_pan - 0.05f) / 0.1f));
              if (macro_fade > 0.01f) {
                  if (will_reenter) {
                    r3d->drawRibbon(orbit_points, pred_w, 1.0f, 0.4f, 0.1f, opacity * macro_fade);
                  } else {
                    if (is_sun_ref) {
                       r3d->drawRibbon(orbit_points, pred_w, 0.2f, 0.6f, 1.0f, opacity * macro_fade); // Dimmer/bluer for Sun
                    } else {
                       r3d->drawRibbon(orbit_points, pred_w, 0.2f, 0.8f, 1.0f, opacity * macro_fade); 
                    }
                  }
              }

              float apsis_size = fminf(earth_r * 0.12f, earth_r * 0.025f * fmaxf(1.0f, cam_zoom_pan * 0.8f));
              apsis_size *= (is_sun_ref ? 10.0f : 1.0f);
              
              // 远地点标记
              Vec3 apoPos = e_dir * (-apoapsis);
              double ax = apoPos.x, ay = apoPos.y, az = apoPos.z;
              if (!is_sun_ref) { 
                  ax += SOLAR_SYSTEM[current_soi_index].px; 
                  ay += SOLAR_SYSTEM[current_soi_index].py; 
                  az += SOLAR_SYSTEM[current_soi_index].pz; 
              }
              Vec3 w_apoPos((float)(ax * ws_d - ro_x), (float)(ay * ws_d - ro_y), (float)(az * ws_d - ro_z));
              r3d->drawBillboard(w_apoPos, apsis_size, 0.2f, 0.4f, 1.0f, opacity);

              // 近地点标记
              Vec3 periPos = e_dir * periapsis;
              double px = periPos.x, py = periPos.y, pz = periPos.z;
              if (!is_sun_ref) { 
                  px += SOLAR_SYSTEM[current_soi_index].px; 
                  py += SOLAR_SYSTEM[current_soi_index].py; 
                  pz += SOLAR_SYSTEM[current_soi_index].pz; 
              }
              Vec3 w_periPos((float)(px * ws_d - ro_x), (float)(py * ws_d - ro_y), (float)(pz * ws_d - ro_z));
              r3d->drawBillboard(w_periPos, apsis_size, 1.0f, 0.5f, 0.1f, opacity);
            } else {
              // --- 双曲线/抛物线逃逸轨道 (ecc >= 1.0) ---
              float a_hyp = fabs(a); 
              float b_hyp = a_hyp * sqrtf(fmaxf(0.0f, ecc * ecc - 1.0f));
              Vec3 e_dir = e_vec / ecc;
              Vec3 perp_dir = h_vec.normalized().cross(e_dir);
              
              float periapsis = a_hyp * (ecc - 1.0f);
              Vec3 center_off = e_dir * (a_hyp * ecc);

              std::vector<Vec3> escape_points;
              int escape_segs = 60;
              float max_sinh = 3.0f; 
              for (int i = -escape_segs; i <= escape_segs; i++) {
                 float t = (float)i / escape_segs * max_sinh;
                 Vec3 pt_rel = center_off - e_dir * (a_hyp * coshf(t)) + perp_dir * (b_hyp * sinhf(t));
                 double px = pt_rel.x, py = pt_rel.y, pz = pt_rel.z;
                 if (!is_sun_ref) { 
                     px += SOLAR_SYSTEM[current_soi_index].px; 
                     py += SOLAR_SYSTEM[current_soi_index].py; 
                     pz += SOLAR_SYSTEM[current_soi_index].pz; 
                 }
                 
                 Vec3 pt((float)(px * ws_d - ro_x), (float)(py * ws_d - ro_y), (float)(pz * ws_d - ro_z));
                 
                 if (escape_points.empty() || (pt - escape_points.back()).length() > earth_r * 0.05f) {
                    escape_points.push_back(pt);
                 }
              }

              // 逃逸轨道的 Ribbon (紫色代表逃逸)
              float pred_w = fmaxf(earth_r * 0.006f, cam_dist * 0.001f);
              float macro_fade = fminf(1.0f, fmaxf(0.0f, (cam_zoom_pan - 0.05f) / 0.1f));
              if (macro_fade > 0.01f) {
                  if (is_sun_ref) {
                       r3d->drawRibbon(escape_points, pred_w, 0.9f, 0.2f, 0.8f, opacity * macro_fade);
                  } else {
                       r3d->drawRibbon(escape_points, pred_w, 0.8f, 0.3f, 1.0f, opacity * macro_fade);
                  }
              }

              // 近地点标记
              float apsis_size = fminf(earth_r * 0.12f, earth_r * 0.025f * fmaxf(1.0f, cam_zoom_pan * 0.8f));
              apsis_size *= (is_sun_ref ? 10.0f : 1.0f);
              Vec3 periPos = center_off - e_dir * a_hyp; 
              double px = periPos.x, py = periPos.y, pz = periPos.z;
              if (!is_sun_ref) { 
                  px += SOLAR_SYSTEM[current_soi_index].px; 
                  py += SOLAR_SYSTEM[current_soi_index].py; 
                  pz += SOLAR_SYSTEM[current_soi_index].pz; 
              }
              Vec3 w_periPos((float)(px * ws_d - ro_x), (float)(py * ws_d - ro_y), (float)(pz * ws_d - ro_z));
              r3d->drawBillboard(w_periPos, apsis_size, 1.0f, 0.5f, 0.1f, opacity);
            }
          }
        }
        } // close if (adv_orbit_enabled) else block

        // --- Maneuver Nodes & Predicted Orbits ---
        double mx_raw, my_raw; glfwGetCursorPos(window, &mx_raw, &my_raw);
        float mouse_x = (float)(mx_raw / ww * 2.0 - 1.0);
        float mouse_y = (float)(1.0 - my_raw / wh * 2.0);
        bool lmb = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS;
        bool rmb = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS;
        static bool lmb_prev_mnv = false;

        // Render hover circle if mouse is over orbit
        if (global_best_ang >= 0 && rocket_state.selected_maneuver_index == -1) {
            float hover_size = earth_r * 0.03f * fmaxf(1.0f, cam_zoom_pan * 0.5f);
            r3d->drawBillboard(global_best_pt, hover_size, 1.0f, 1.0f, 1.0f, 0.5f);
        }

        static bool popup_clicked_frame = false;
        popup_clicked_frame = false;
        
        // Early popup bounds check: if a popup is visible, pre-calculate whether the mouse is inside it
        // to prevent clicks in the popup area from triggering icon/handle/orbit interactions
        static float cached_popup_x = 0, cached_popup_y = 0, cached_popup_w = 0, cached_popup_h = 0;
        if (mnv_popup_index != -1 && cached_popup_w > 0) {
            bool mouse_in_popup_early = (mouse_x >= cached_popup_x - cached_popup_w/2 && mouse_x <= cached_popup_x + cached_popup_w/2 &&
                                         mouse_y >= cached_popup_y - cached_popup_h/2 && mouse_y <= cached_popup_y + cached_popup_h/2);
            if (mouse_in_popup_early) {
                popup_clicked_frame = true;
            }
        }

        double mu_body = 6.67430e-11 * SOLAR_SYSTEM[current_soi_index].mass;
        float as_ratio = (float)ww / wh;

        // --- Stable Orbit Parameter Extraction (Ensures nodes don't jitter) ---
        Vec3 cur_r_vec((float)rocket_state.px, (float)rocket_state.py, (float)rocket_state.pz);
        Vec3 cur_v_vec((float)rocket_state.vx, (float)rocket_state.vy, (float)rocket_state.vz);
        double cur_r_mag = cur_r_vec.length();
        double cur_v_sq = cur_v_vec.lengthSq();
        Vec3 cur_h_vec = cur_r_vec.cross(cur_v_vec);
        double cur_energy = 0.5 * cur_v_sq - mu_body / cur_r_mag;
        double stable_a = -mu_body / (2.0 * cur_energy);
        Vec3 cur_e_vec = cur_v_vec.cross(cur_h_vec) / (float)mu_body - cur_r_vec / (float)cur_r_mag;
        double stable_ecc = cur_e_vec.length();
        Vec3 stable_e_dir = (stable_ecc > 1e-7f) ? cur_e_vec.normalized() : Vec3(1, 0, 0);
        Vec3 stable_p_dir = cur_h_vec.normalized().cross(stable_e_dir).normalized(); 
        double stable_n = sqrt(mu_body / (stable_a * stable_a * stable_a));
        double s_b = stable_a * sqrt(fmax(0.0, 1.0 - stable_ecc * stable_ecc));
        double s_cosE = (stable_a - cur_r_mag) / (stable_a * stable_ecc);
        double s_sinE = cur_r_vec.dot(stable_p_dir) / s_b;
        double stable_M0 = atan2(s_sinE, s_cosE) - stable_ecc * sin(atan2(s_sinE, s_cosE));
        Vec3 stable_center = stable_e_dir * (-(float)stable_a * (float)stable_ecc);
        int stable_ref = adv_orbit_enabled ? adv_orbit_ref_body : current_soi_index;

        // Use a local delta for dragging (calculate once per frame)
        static float last_pass_mx = 0, last_pass_my = 0;
        float dmx = mouse_x - last_pass_mx;
        float dmy = mouse_y - last_pass_my;
        last_pass_mx = mouse_x;
        last_pass_my = mouse_y;

        for (int i = 0; i < (int)rocket_state.maneuvers.size(); i++) {
            ManeuverNode& node = rocket_state.maneuvers[i];
            
            // Use ARCHIVED stable orbit from node creation time to reconstruct 3D position
            double ref_px = SOLAR_SYSTEM[node.ref_body].px;
            double ref_py = SOLAR_SYSTEM[node.ref_body].py;
            double ref_pz = SOLAR_SYSTEM[node.ref_body].pz;

            double E_node = node.ref_M0; 

            float node_b = (float)node.ref_a * sqrtf(fmaxf(0.0f, 1.0f - (float)node.ref_ecc * (float)node.ref_ecc));
            Vec3 pt_node_rel = node.ref_center + node.ref_e_dir * ((float)node.ref_a * cosf((float)E_node)) + node.ref_p_dir * (node_b * sinf((float)E_node));
            
            Vec3 node_world = Vec3((float)(ref_px * ws_d + pt_node_rel.x * ws_d - ro_x), 
                              (float)(ref_py * ws_d + pt_node_rel.y * ws_d - ro_y), 
                              (float)(ref_pz * ws_d + pt_node_rel.z * ws_d - ro_z));
            if (adv_orbit_enabled && i == 0 && !draw_mnv_points.empty()) {
                // The maneuver node position is the first point of the post-burn path
                node_world = Vec3((float)(draw_mnv_points[0].x * ws_d - ro_x), 
                                  (float)(draw_mnv_points[0].y * ws_d - ro_y), 
                                  (float)(draw_mnv_points[0].z * ws_d - ro_z));
            }
            Vec2 n_scr = ManeuverSystem::projectToScreen(node_world, viewMat, macroProjMat, as_ratio);
            float d_mouse = sqrtf(powf(n_scr.x - mouse_x, 2) + powf(n_scr.y - mouse_y, 2));

            static bool hit_maneuver_icon = false;
            if (i == 0) hit_maneuver_icon = false; // Reset start of list

            // CLICK PRIORITY: Icon (only if popup isn't consuming the click)
            if (lmb && !lmb_prev_mnv && dragging_handle == -1 && !popup_clicked_frame) {
                if (d_mouse < 0.045f) {
                    rocket_state.selected_maneuver_index = i;
                    mnv_popup_index = i;
                    hit_maneuver_icon = true;
                }
            }

            // Draw Icon (Highlight if selected or hovered)
            float icon_size = earth_r * (d_mouse < 0.04f ? 0.1f : 0.08f);
            Vec3 icon_col = (rocket_state.selected_maneuver_index == i) ? Vec3(0.4f, 0.8f, 1.0f) : Vec3(0.2f, 0.6f, 1.0f);
            r3d->drawBillboard(node_world, icon_size, icon_col.x, icon_col.y, icon_col.z, 0.9f);
            
            if (rocket_state.selected_maneuver_index == i) {
                // Node relative state for handles (3D)
                // We use node-specific archived parameters for motion as well to ensure handles are spatially correct
                double E_dot = node.ref_n / (1.0 - node.ref_ecc * cos(E_node));
                Vec3 v_node_rel = node.ref_e_dir * (-(float)node.ref_a * sinf((float)E_node) * (float)E_dot) + node.ref_p_dir * (node_b * cosf((float)E_node) * (float)E_dot);
                ManeuverFrame m_frame = ManeuverSystem::getFrame(pt_node_rel, v_node_rel);

                for (int h = 0; h < 6; h++) {
                    Vec3 h_dir = ManeuverSystem::getHandleDir(m_frame, h);
                    float handle_dist = earth_r * (dragging_handle == h ? 0.35f : 0.2f);
                    Vec3 h_world = node_world + h_dir * handle_dist;
                    Vec2 h_scr = ManeuverSystem::projectToScreen(h_world, viewMat, macroProjMat, as_ratio);
                    float hd = sqrtf(powf(h_scr.x - mouse_x, 2) + powf(h_scr.y - mouse_y, 2));
                    
                    if (lmb && !lmb_prev_mnv && hd < 0.035f && !hit_maneuver_icon && !popup_clicked_frame) {
                        dragging_handle = h;
                        mnv_popup_index = -1; // Hide popup when dragging
                        cached_popup_w = 0; // Clear cache
                    }
                    
                    if (dragging_handle == h && lmb) {
                        Vec3 h_world_next = node_world + h_dir * (handle_dist + 1.0f);
                        Vec2 h_scr_next = ManeuverSystem::projectToScreen(h_world_next, viewMat, macroProjMat, as_ratio);
                        Vec2 axis2D = h_scr_next - n_scr;
                        float screen_len_sq = axis2D.lengthSq();
                        
                        if (screen_len_sq > 1e-8f) {
                            // 1. Immediate displacement-based change
                            float proj_drag = (dmx * axis2D.x + dmy * axis2D.y) / screen_len_sq;
                            float sensitivity = 100.0f * sqrtf(screen_len_sq); 
                            if (h >= 2) sensitivity *= 2.5f; 
                            float drag_amount = proj_drag * sensitivity;

                            // 2. Continuous rate-based change (Pressure)
                            // Calculate offset from current handle icon to mouse
                            float dx_h = mouse_x - h_scr.x;
                            float dy_h = mouse_y - h_scr.y;
                            float proj_offset = (dx_h * axis2D.x + dy_h * axis2D.y) / screen_len_sq;
                            
                            // If user pulls mouse away from handle center, increase velocity continuously
                            float rate = 30.0f; // 30 m/s^2 base rate per handle length of offset
                            if (h >= 2) rate *= 4.0f; 
                            float continuous_amount = proj_offset * rate * (float)dt;

                            float total_change = drag_amount + continuous_amount;

                            if (h == 0) node.delta_v.x += total_change; else if (h == 1) node.delta_v.x -= total_change;
                            else if (h == 2) node.delta_v.y += total_change; else if (h == 3) node.delta_v.y -= total_change;
                            else if (h == 4) node.delta_v.z += total_change; else if (h == 5) node.delta_v.z -= total_change;
                            node.active = true;
                        }
                    }
                    Vec3 h_col = ManeuverSystem::getHandleColor(h);
                    if (hd < 0.035f || dragging_handle == h) h_col = h_col * 1.3f;
                    r3d->drawBillboard(h_world, earth_r * (hd < 0.035f ? 0.05f : 0.04f), h_col.x, h_col.y, h_col.z, 0.9f);
                }
                
                if (node.active) {
                    // Predicted Orbit (Dashed)
                    Vec3 p_pos = pt_node_rel;
                    Vec3 p_vel = v_node_rel + m_frame.prograde * node.delta_v.x + m_frame.normal * node.delta_v.y + m_frame.radial * node.delta_v.z;
                    double p_energy = 0.5 * (double)p_vel.lengthSq() - global_best_mu / (double)p_pos.length();
                    double p_a = -global_best_mu / (2.0 * p_energy);
                    if (p_a > 0) {
                        Vec3 p_h_vec = p_pos.cross(p_vel);
                        Vec3 p_e_vec = p_vel.cross(p_h_vec) / (float)global_best_mu - p_pos / (float)p_pos.length();
                        float p_ecc = p_e_vec.length();
                        float p_b = (float)p_a * sqrtf(fmaxf(0.0f, 1.0f - p_ecc * p_ecc));
                        Vec3 p_e_dir = p_ecc > 1e-6f ? p_e_vec / p_ecc : Vec3(1,0,0);
                        Vec3 p_perp = p_h_vec.normalized().cross(p_e_dir);
                        Vec3 p_center = p_e_dir * (-(float)p_a * p_ecc);
                        std::vector<Vec3> p_pts;
                        int samples = 500; // High precision samples
                        for (int s = 0; s <= samples; s++) {
                            float ang = (float)s / (float)samples * 2.0f * PI;
                            Vec3 ptr = p_center + p_e_dir * ((float)p_a * cosf(ang)) + p_perp * (p_b * sinf(ang));
                            p_pts.push_back(Vec3((float)(ref_px * ws_d + ptr.x * ws_d - ro_x), (float)(ref_py * ws_d + ptr.y * ws_d - ro_y), (float)(ref_pz * ws_d + ptr.z * ws_d - ro_z)));
                        }
                        float ribbon_w = fmaxf(earth_r * 0.008f, cam_dist * 0.0012f);
                        // Draw as dashed lines: segments of 6 points with 4 point gaps
                        for (int s = 0; s < samples; s += 10) {
                            std::vector<Vec3> dash;
                            for (int j = 0; j < 6; j++) {
                                if (s + j <= samples) dash.push_back(p_pts[s + j]);
                            }
                            if (dash.size() >= 2) {
                                r3d->drawRibbon(dash, ribbon_w, 1.0f, 1.0f, 1.0f, 0.7f);
                            }
                        }
                    }
                }
            }
        }

        // --- Maneuver Node Popup: LOGIC ONLY (rendering deferred to 2D HUD pass) ---
        // NOTE: renderer->addRect/drawText calls here would be cleared by renderer->beginFrame() later
        mnv_popup_visible = false;
        if (mnv_popup_index != -1 && (size_t)mnv_popup_index < rocket_state.maneuvers.size()) {
            ManeuverNode& node = rocket_state.maneuvers[mnv_popup_index];
            float node_b = (float)node.ref_a * sqrtf(fmaxf(0.0f, 1.0f - (float)node.ref_ecc * (float)node.ref_ecc));
            Vec3 pt_node_rel = node.ref_center + node.ref_e_dir * ((float)node.ref_a * cosf((float)node.ref_M0)) + node.ref_p_dir * (node_b * sinf((float)node.ref_M0));
            Vec3 node_world = Vec3((float)(SOLAR_SYSTEM[node.ref_body].px * ws_d + pt_node_rel.x * ws_d - ro_x), 
                              (float)(SOLAR_SYSTEM[node.ref_body].py * ws_d + pt_node_rel.y * ws_d - ro_y), 
                              (float)(SOLAR_SYSTEM[node.ref_body].pz * ws_d + pt_node_rel.z * ws_d - ro_z));
            Vec2 n_scr = ManeuverSystem::projectToScreen(node_world, viewMat, macroProjMat, as_ratio);
            
            // Popup layout - enlarged for sliders, time info, reference frame
            float pop_x, pop_y, pw, ph;
            if (adv_embed_mnv && adv_orbit_menu) {
                pop_x = mnv_popup_px;
                pop_y = mnv_popup_py;
                pw = mnv_popup_pw;
                ph = mnv_popup_ph;
            } else {
                pop_x = n_scr.x + 0.22f;
                pop_y = n_scr.y;
                pw = 0.38f;
                ph = 0.55f;
                
                // Clamp popup to stay within screen bounds
                if (pop_x + pw/2 > 0.98f) pop_x = 0.98f - pw/2;
                if (pop_x - pw/2 < -0.98f) pop_x = -0.98f + pw/2;
                if (pop_y + ph/2 > 0.98f) pop_y = 0.98f - ph/2;
                if (pop_y - ph/2 < -0.98f) pop_y = -0.98f + ph/2;
            }
            
            // Cache popup bounds
            cached_popup_x = pop_x; cached_popup_y = pop_y;
            cached_popup_w = pw; cached_popup_h = ph;
            
            // Cache state for deferred rendering
            mnv_popup_visible = true;
            mnv_popup_px = pop_x; mnv_popup_py = pop_y;
            mnv_popup_pw = pw; mnv_popup_ph = ph;
            mnv_popup_node_scr_x = n_scr.x; mnv_popup_node_scr_y = n_scr.y;
            mnv_popup_dv = node.delta_v;
            mnv_popup_ref_body = node.ref_body;
            
            // Compute time to node or start of burn
            double target_t = node.sim_time;
            mnv_popup_time_to_node = target_t - rocket_state.sim_time;
            if (mnv_popup_time_to_node < -mnv_popup_burn_time) mnv_popup_time_to_node = -mnv_popup_burn_time; // keep showing count-up during burn
            
            // Compute remaining delta-v: if burn is in progress, compare current velocity
            // against the planned post-burn velocity from the snapshot
            float total_dv_val = node.delta_v.length();
            float remaining_dv = total_dv_val; // Default: full delta-v
            
            // Populate snapshot when approaching node time (consistent with autopilot)
            if (!node.snap_valid && rocket_state.sim_time >= node.sim_time - 5.0) {
                int ref_idx = (node.ref_body >= 0) ? node.ref_body : current_soi_index;
                CelestialBody& ref_b = SOLAR_SYSTEM[ref_idx];
                double rbpx, rbpy, rbpz, rbvx, rbvy, rbvz;
                PhysicsSystem::GetCelestialStateAt(ref_idx, node.sim_time, rbpx, rbpy, rbpz, rbvx, rbvy, rbvz);
                double mu_ref = G_const * ref_b.mass;
                double npx, npy, npz, nvx, nvy, nvz;
                get3DStateAtTime(rocket_state.px, rocket_state.py, rocket_state.pz, rocket_state.vx, rocket_state.vy, rocket_state.vz, mu_ref, node.sim_time - rocket_state.sim_time, npx, npy, npz, nvx, nvy, nvz);
                ManeuverFrame frame = ManeuverSystem::getFrame(Vec3((float)npx, (float)npy, (float)npz), Vec3((float)nvx, (float)nvy, (float)nvz));
                Vec3 target_dv_world = (frame.prograde * node.delta_v.x + frame.normal * node.delta_v.y + frame.radial * node.delta_v.z);
                node.locked_burn_dir = target_dv_world.normalized();
                
                // Absolute state snapshot (for prediction)
                node.snap_px = rbpx + npx;
                node.snap_py = rbpy + npy;
                node.snap_pz = rbpz + npz;
                node.snap_vx = rbvx + nvx + target_dv_world.x;
                node.snap_vy = rbvy + nvy + target_dv_world.y;
                node.snap_vz = rbvz + nvz + target_dv_world.z;
                
                // Relative state snapshot (for guidance stability) - Principia Style
                node.snap_rel_px = npx;
                node.snap_rel_py = npy;
                node.snap_rel_pz = npz;
                node.snap_rel_vx = nvx + target_dv_world.x;
                node.snap_rel_vy = nvy + target_dv_world.y;
                node.snap_rel_vz = nvz + target_dv_world.z;
                
                node.snap_time = node.sim_time;
                node.snap_valid = true;
            }
            
            if (node.snap_valid) {
                Vec3 rem_v = ManeuverSystem::calculateRemainingDV(rocket_state, node);
                remaining_dv = (float)rem_v.length();
                if (remaining_dv < 0.1f) remaining_dv = 0.0f;
            }
            
            // Compute estimated burn time using Tsiolkovsky equation
            double current_mass = rocket_state.fuel + rocket_config.dry_mass + rocket_config.upper_stages_mass;
            double max_thrust = 0;
            if (rocket_state.current_stage < (int)rocket_config.stage_configs.size()) {
                max_thrust = rocket_config.stage_configs[rocket_state.current_stage].thrust;
            }
            if (max_thrust > 0 && current_mass > 0) {
                double ve = rocket_config.specific_impulse * G0;
                if (ve > 0 && remaining_dv > 0) {
                    double mass_ratio = exp((double)remaining_dv / ve);
                    double fuel_needed = current_mass * (1.0 - 1.0 / mass_ratio);
                    double mdot = max_thrust / ve;
                    double accel = max_thrust / current_mass;
                    mnv_popup_burn_time = (mdot > 0) ? fuel_needed / mdot : (double)remaining_dv / accel;
                } else {
                    mnv_popup_burn_time = 0;
                }
            } else {
                mnv_popup_burn_time = (remaining_dv > 0) ? 9999.0 : 0;
            }
            
            // --- Slider layout constants (for hit testing) ---
            float slider_track_w = pw * 0.65f;
            float slider_track_h = 0.012f;
            float slider_thumb_w = 0.012f, slider_thumb_h = 0.022f;
            float title_y_layout = pop_y + ph/2 - (adv_embed_mnv ? 0.015f : 0.025f);
            float slider_base_y = title_y_layout - (adv_embed_mnv ? 0.04f : 0.06f);
            float slider_spacing = adv_embed_mnv ? 0.050f : 0.065f;
            float slider_cx = pop_x + 0.02f; // center of slider track
            
            float sep_y = slider_base_y - 4 * slider_spacing + 0.025f;
            float info_y = sep_y - 0.05f; // time to node
            info_y -= 0.025f; // estimated burn time
            float mode_btn_y = info_y - 0.045f;
            float del_btn_y = pop_y - ph/2 + (adv_embed_mnv ? 0.015f : 0.025f);
            
            // State for deferred rendering
            mnv_popup_burn_mode = node.burn_mode;

            // Close [X] button hit test
            float close_size = 0.028f;
            float close_x = pop_x + pw/2 - close_size/2 - 0.008f;
            float close_y = pop_y + ph/2 - close_size/2 - 0.008f;
            mnv_popup_close_hover = (!adv_embed_mnv) && (mouse_x >= close_x - close_size/2 && mouse_x <= close_x + close_size/2 &&
                                         mouse_y >= close_y - close_size/2 && mouse_y <= close_y + close_size/2);
            mnv_popup_mini_hover = (adv_embed_mnv) && (mouse_x >= close_x - close_size/2 && mouse_x <= close_x + close_size/2 &&
                                         mouse_y >= close_y - close_size/2 && mouse_y <= close_y + close_size/2);
            
            if (mnv_popup_mini_hover && lmb && !lmb_prev_mnv) {
                adv_embed_mnv_mini = !adv_embed_mnv_mini;
                popup_clicked_frame = true;
            }

            if (!adv_embed_mnv_mini) {
                // --- Mode Toggle Hit Test ---
                float mode_btn_w = pw * 0.8f, mode_btn_h = 0.032f;
                mnv_popup_mode_hover = (mouse_x >= pop_x - mode_btn_w/2 && mouse_x <= pop_x + mode_btn_w/2 &&
                                        mouse_y >= mode_btn_y - mode_btn_h/2 && mouse_y <= mode_btn_y + mode_btn_h/2);
                
                if (mnv_popup_mode_hover && lmb && !lmb_prev_mnv) {
                    node.burn_mode = 1 - node.burn_mode;
                    node.snap_valid = false; // Force re-prediction
                    popup_clicked_frame = true;
                }

                // DELETE button hit test
                float del_btn_w = 0.10f, del_btn_h = 0.03f;
                mnv_popup_del_hover = (mouse_x >= pop_x - del_btn_w/2 && mouse_x <= pop_x + del_btn_w/2 &&
                                       mouse_y >= del_btn_y - del_btn_h/2 && mouse_y <= del_btn_y + del_btn_h/2);
                
                // --- Slider dragging logic ---
                for (int s = 0; s < 4; s++) {
                    float sy = slider_base_y - s * slider_spacing;
                    bool on_track = (mouse_x >= slider_cx - slider_track_w/2 - 0.02f && mouse_x <= slider_cx + slider_track_w/2 + 0.02f &&
                                     mouse_y >= sy - slider_thumb_h && mouse_y <= sy + slider_thumb_h);
                    
                    if (lmb && !lmb_prev_mnv && on_track && mnv_popup_slider_dragging == -1) {
                        mnv_popup_slider_dragging = s;
                        mnv_popup_slider_drag_x = mouse_x;
                    }
                }
            }
            
            if (mnv_popup_slider_dragging >= 0 && lmb) {
                float drag_offset = mouse_x - mnv_popup_slider_drag_x;
                float sign = (drag_offset >= 0) ? 1.0f : -1.0f;
                float abs_offset = fabsf(drag_offset);
                
                if (mnv_popup_slider_dragging < 3) {
                    float rate = sign * abs_offset * abs_offset * 5000.0f * (float)dt;
                    if (mnv_popup_slider_dragging == 0) node.delta_v.x += rate;
                    else if (mnv_popup_slider_dragging == 1) node.delta_v.y += rate;
                    else if (mnv_popup_slider_dragging == 2) node.delta_v.z += rate;
                    node.snap_valid = false;
                } else if (mnv_popup_slider_dragging == 3) {
                    float t_rate = sign * abs_offset * abs_offset * 1000000.0f * (float)dt;
                    node.sim_time += t_rate;
                    if (node.sim_time < rocket_state.sim_time + 10.0) node.sim_time = rocket_state.sim_time + 10.0;
                    node.snap_valid = false;
                }
                
                node.active = true;
                mnv_popup_dv = node.delta_v;
            }
            
            if (!lmb) {
                mnv_popup_slider_dragging = -1;
            }
            
            // Check if mouse is inside the popup panel
            bool mouse_in_popup = (mouse_x >= pop_x - pw/2 && mouse_x <= pop_x + pw/2 &&
                                   mouse_y >= pop_y - ph/2 && mouse_y <= pop_y + ph/2);
            if (mouse_in_popup || mnv_popup_slider_dragging >= 0) {
                popup_clicked_frame = true;
            }
            
            // Click handling: ONLY close via [X] button or DELETE button
            if (lmb && !lmb_prev_mnv) {
                if (mnv_popup_close_hover) {
                    mnv_popup_index = -1;
                    mnv_popup_visible = false;
                    mnv_popup_slider_dragging = -1;
                    adv_embed_mnv = false;
                    cached_popup_w = 0;
                    popup_clicked_frame = true;
                } else if (mnv_popup_del_hover) {
                    rocket_state.maneuvers.erase(rocket_state.maneuvers.begin() + mnv_popup_index);
                    if (rocket_state.selected_maneuver_index == mnv_popup_index) rocket_state.selected_maneuver_index = -1;
                    else if (rocket_state.selected_maneuver_index > mnv_popup_index) rocket_state.selected_maneuver_index--;
                    mnv_popup_index = -1;
                    mnv_popup_visible = false;
                    mnv_popup_slider_dragging = -1;
                    cached_popup_w = 0;
                    popup_clicked_frame = true;
                } else if (mouse_in_popup) {
                    popup_clicked_frame = true;
                }
            }
        } else {
            mnv_popup_slider_dragging = -1;
        }
        
        // --- Click on empty orbit to create node (Moved to LMB) ---
        if (lmb && !lmb_prev_mnv && !popup_clicked_frame && dragging_handle == -1 && mnv_popup_index == -1 && rocket_state.selected_maneuver_index == -1 && global_best_ang >= 0) {
            double M_click = global_best_ang - global_best_ecc * sin(global_best_ang);
            double dM = M_click - global_current_M0;
            while (dM < 0) dM += 2.0 * PI;
            while (dM > 2.0 * PI) dM -= 2.0 * PI;
            
            ManeuverNode newNode;
            newNode.sim_time = rocket_state.sim_time + (dM / global_current_n);
            newNode.delta_v = Vec3(0,0,0);
            newNode.active = true;
            
            // ARCHIVE orbital elements at creation time for permanent anchoring
            newNode.ref_a = stable_a;
            newNode.ref_ecc = stable_ecc;
            newNode.ref_n = stable_n;
            newNode.ref_e_dir = stable_e_dir;
            newNode.ref_p_dir = stable_p_dir;
            newNode.ref_center = stable_center;
            newNode.ref_body = stable_ref;
            newNode.ref_M0 = global_best_ang; // Store Eccentric Anomaly actually
            
            rocket_state.maneuvers.push_back(newNode);
            rocket_state.selected_maneuver_index = (int)rocket_state.maneuvers.size() - 1;
        }
        
        // Empty click to deselect if didn't hit anything
        if (lmb && !lmb_prev_mnv && !popup_clicked_frame && mnv_popup_index == -1 && dragging_handle == -1 && global_best_ang < 0) {
            rocket_state.selected_maneuver_index = -1;
        }

        // Reset handles if mouse released
        if (!lmb) dragging_handle = -1;
        global_best_ang = -1.0f; // Reset for next frame
        lmb_prev_mnv = lmb;
        rmb_prev_mnv = rmb;
      }

      // Dynamic TAA blend: reduce temporal accumulation during fast changes
      // to prevent ghosting/smearing of orbit lines
      if (time_warp > 1 || mnv_popup_slider_dragging >= 0) {
          // In time warp or during slider drag, favor current frame heavily
          float warp_blend = (time_warp > 100) ? 0.8f : (time_warp > 1 ? 0.5f : 0.4f);
          if (mnv_popup_slider_dragging >= 0) warp_blend = fmaxf(warp_blend, 0.6f);
          r3d->taaBlendOverride = warp_blend;
      } else {
          r3d->taaBlendOverride = -1.0f; // Use default
      }
      r3d->resolveTAA();

      // =========== PASS 2: MICRO FOREGROUND ===========
      // 微观近景火箭专用的相机矩阵 (极近裁剪面，用于精确绘制 40米的火箭)
      if (cam_mode_3d != 2) {
          // 在近景模式下，清空深度缓存，将火箭置于绝对顶层，杜绝共用一套深度衰减。
          // 全景模式下不清空，保留真实的物理穿模(躲在地球后面会被遮挡)的正确视角。
          glClear(GL_DEPTH_BUFFER_BIT);
      }
      
      float micro_near = fmaxf(rh * 0.05f, cam_dist * 0.002f);
      float micro_far  = fmaxf(cam_dist * 10.0f, 15000.0f);
      Mat4 microProjMat = Mat4::perspective(0.8f, aspect, micro_near, micro_far);
      r3d->beginFrame(viewMat, microProjMat, camEye_rel);



      // ===== 火箭涂装 (Assembly-based per-part rendering) =====
      Vec3 engNozzlePos = renderRocketBase; // Fallback to base
      {
        int render_start = 0;
        if (rocket_state.current_stage < (int)rocket_config.stage_configs.size()) {
            render_start = rocket_config.stage_configs[rocket_state.current_stage].part_start_index;
        }

        // Find lowest active engine for flame positioning
        float min_ey = 1e10f;
        for (int pi = render_start; pi < (int)assembly.parts.size(); pi++) {
            const PlacedPart& pp = assembly.parts[pi];
            if (PART_CATALOG[pp.def_id].category == CAT_ENGINE) {
                if (pp.pos.y < min_ey) {
                    min_ey = pp.pos.y;
                    engNozzlePos = renderRocketBase + rocketQuat.rotate(pp.pos * (float)ws_d);
                }
            }
        }

        for (int pi = render_start; pi < (int)assembly.parts.size(); pi++) {
          const PlacedPart& pp = assembly.parts[pi];
          const PartDef& def = PART_CATALOG[pp.def_id];
          
          for (int s = 0; s < pp.symmetry; s++) {
            float symAngle = (s * 2.0f * 3.14159f) / pp.symmetry;
            Vec3 localPos = pp.pos;
            if (pp.symmetry > 1) {
                float dist = sqrtf(pp.pos.x*pp.pos.x + pp.pos.z*pp.pos.z);
                if (dist > 0.01f) {
                   float curAngle = atan2f(pp.pos.z, pp.pos.x);
                   localPos.x = cosf(curAngle + symAngle) * dist;
                   localPos.z = sinf(curAngle + symAngle) * dist;
                }
            }

            Vec3 partWorldPos = renderRocketBase + rocketQuat.rotate(localPos * (float)ws_d);
            Quat partWorldRot = rocketQuat * pp.rot * Quat::fromAxisAngle(Vec3(0, 1, 0), symAngle);
            
            float pd = def.diameter * (float)ws_d;
            float ph = def.height * (float)ws_d;
            Vec3 partCenter = partWorldPos + partWorldRot.rotate(Vec3(0, ph * 0.5f, 0));

            if (def.category == CAT_NOSE_CONE) {
                Mat4 partMat = Mat4::TRS(partWorldPos, partWorldRot, {pd, ph, pd});
                r3d->drawMesh(rocketNose, partMat, def.r, def.g, def.b, 1.0f, 0.2f);
            } else if (def.category == CAT_ENGINE) {
                float bf = 0.4f; float nf = 1.0f - bf;
                Vec3 bodyPos = partWorldPos + partWorldRot.rotate(Vec3(0, ph*(1.0f-bf*0.5f), 0));
                Mat4 bodyMat = Mat4::TRS(bodyPos, partWorldRot, {pd*0.6f, ph*bf, pd*0.6f});
                r3d->drawMesh(rocketBody, bodyMat, 0.2f, 0.2f, 0.22f, 1.0f, 0.4f);
                Mat4 bellMat = Mat4::TRS(partWorldPos, partWorldRot, {pd*0.85f, ph*nf, pd*0.85f});
                r3d->drawMesh(rocketNose, bellMat, def.r*0.8f, def.g*0.8f, def.b*0.8f, 1.0f, 0.1f);
            } else if (def.category == CAT_STRUCTURAL) {
                if (strstr(def.name, "Fin") || strstr(def.name, "Solar")) {
                    Mat4 finMat = Mat4::TRS(partCenter, partWorldRot, {pd*0.05f, ph, pd*0.5f});
                    r3d->drawMesh(rocketBox, finMat, def.r, def.g, def.b, 1.0f, 0.1f);
                } else if (strstr(def.name, "Leg")) {
                    Mat4 legMat = Mat4::TRS(partCenter, partWorldRot, {pd*0.15f, ph, pd*0.15f});
                    r3d->drawMesh(rocketBox, legMat, def.r, def.g, def.b, 1.0f, 0.1f);
                } else {
                    Mat4 partMat = Mat4::TRS(partCenter, partWorldRot, {pd, ph, pd});
                    r3d->drawMesh(rocketBody, partMat, def.r, def.g, def.b, 1.0f, 0.2f);
                }
            } else {
                Mat4 partMat = Mat4::TRS(partCenter, partWorldRot, {pd, ph, pd});
                r3d->drawMesh(rocketBody, partMat, def.r, def.g, def.b, 1.0f, 0.2f);
            }
          }
        }
      }

      // ===== 发射台渲染 (Launch Pad Generation / Rendering) =====
      if (rocket_state.altitude < 2000.0) {
          CelestialBody& body = SOLAR_SYSTEM[current_soi_index];
          
          // Launch pad should rotate with the body (Axial Tilt + Rotation)
          double theta = body.prime_meridian_epoch + (rocket_state.sim_time * 2.0 * PI / body.rotation_period);
          Quat rot = Quat::fromAxisAngle(Vec3(0, 0, 1), (float)theta);
          Quat tilt = Quat::fromAxisAngle(Vec3(1, 0, 0), (float)body.axial_tilt);
          Quat full_rot = tilt * rot;
          
          // Use rocket's launch site coordinates
          Vec3 s_pos((float)rocket_state.surf_px, (float)rocket_state.surf_py, (float)rocket_state.surf_pz);
          // Normalized local up vector
          Vec3 localUp = s_pos.normalized();
          // Position on surface (exactly at terrain elevation)
          Vec3 s_surf = localUp * s_pos.length();
          
          // Rotate to inertial frame
          Vec3 i_surf = full_rot.rotate(s_surf);
          Vec3 i_up   = full_rot.rotate(localUp);

          Vec3 padCenter(
              (float)((body.px + (double)i_surf.x) * ws_d - ro_x), 
              (float)((body.py + (double)i_surf.y) * ws_d - ro_y), 
              (float)((body.pz + (double)i_surf.z) * ws_d - ro_z)
          );
          
          // Calculate local orientation for the pad
          Vec3 padUp = i_up;
          // Calculate a local right/forward to orient the pad
          Vec3 defaultRight(1, 0, 0);
          if (fabs(padUp.dot(defaultRight)) > 0.9f) defaultRight = Vec3(0, 1, 0);
          Vec3 padRight = padUp.cross(defaultRight).normalized();
          Vec3 padForward = padUp.cross(padRight).normalized();
          
          // Construct rotation matrix for the pad to face "up"
          Mat4 padRot;
          padRot.m[0] = padRight.x; padRot.m[1] = padRight.y; padRot.m[2] = padRight.z; padRot.m[3] = 0;
          padRot.m[4] = padUp.x;    padRot.m[5] = padUp.y;    padRot.m[6] = padUp.z;    padRot.m[7] = 0;
          padRot.m[8] = padForward.x; padRot.m[9] = padForward.y; padRot.m[10] = padForward.z; padRot.m[11] = 0;
          padRot.m[12] = 0;         padRot.m[13] = 0;         padRot.m[14] = 0;         padRot.m[15] = 1;

          if (has_launch_pad) {
              float pad_scale = earth_r * 0.005f;
              Mat4 padModel = Mat4::scale(Vec3(pad_scale, pad_scale, pad_scale));
              padModel = padRot * padModel; // Orient pad
              padModel = Mat4::translate(padCenter) * padModel;
              r3d->drawMesh(launchPadMesh, padModel, 0.4f, 0.4f, 0.42f, 1.0f, 0.2f);
          } else {
              float pad_w = rw_3d * 20.0f;
              float pad_h = rh * 0.4f; // Increased pad height to bury it deeper
              
              Mat4 baseMdl = Mat4::scale(Vec3(pad_w, pad_h, pad_w));
              baseMdl = padRot * baseMdl;
              // Bury the bottom half of the pad base into the planet to eliminate gaps
              baseMdl = Mat4::translate(padCenter - padUp * (pad_h * 0.45f)) * baseMdl;
              r3d->drawMesh(rocketBox, baseMdl, 0.4f, 0.4f, 0.4f, 1.0f, 0.1f);
              
              float tower_h = rh * 1.5f;
              float tower_w = rw_3d * 3.0f;
              Vec3 towerCenter = padCenter + padRight * (rw_3d * 4.0f) + padUp * (tower_h * 0.5f - pad_h * 0.5f);
              Mat4 towerMdl = Mat4::scale(Vec3(tower_w, tower_h, tower_w));
              towerMdl = padRot * towerMdl;
              towerMdl = Mat4::translate(towerCenter) * towerMdl;
              r3d->drawMesh(rocketBox, towerMdl, 0.7f, 0.15f, 0.15f, 1.0f, 0.3f);
          }
      }

      // ===== 工业级 3D 体积尾焰 (Volumetric Raymarched Plume for ALL active engines) =====
      if (rocket_state.thrust_power > 0.01) {
        float thrust = (float)control_input.throttle;
        float expansion = (float)fmax(0.0, 1.0 - PhysicsSystem::get_air_density(rocket_state.altitude) / 1.225);
        float thrust_scale = 0.25f + 0.75f * powf(thrust, 1.5f);
        
        int render_start = 0;
        if (rocket_state.current_stage < (int)rocket_config.stage_configs.size()) {
            render_start = rocket_config.stage_configs[rocket_state.current_stage].part_start_index;
        }

        // Iterate through all active engines to render plumes
        for (int pi = render_start; pi < (int)assembly.parts.size(); pi++) {
            const PlacedPart& pp = assembly.parts[pi];
            const PartDef& def = PART_CATALOG[pp.def_id];
            
            if (def.category == CAT_ENGINE) {
                for (int s = 0; s < pp.symmetry; s++) {
                    float symAngle = (s * 2.0f * 3.14159f) / pp.symmetry;
                    Vec3 localPos = pp.pos;
                    if (pp.symmetry > 1) {
                        float dist = sqrtf(pp.pos.x*pp.pos.x + pp.pos.z*pp.pos.z);
                        if (dist > 0.01f) {
                            float curAngle = atan2f(pp.pos.z, pp.pos.x);
                            localPos.x = cosf(curAngle + symAngle) * dist;
                            localPos.z = sinf(curAngle + symAngle) * dist;
                        }
                    }

                    Vec3 nozzleWorldPos = renderRocketBase + rocketQuat.rotate(localPos * (float)ws_d);
                    Quat nozzleWorldRot = rocketQuat * pp.rot * Quat::fromAxisAngle(Vec3(0, 1, 0), symAngle);
                    Vec3 nozzleDir = nozzleWorldRot.rotate(Vec3(0, 1, 0)); // Direction engine is pointing

                    float engine_ph = def.height * (float)ws_d;
                    float plume_len = engine_ph * 4.2f * thrust_scale * (1.0f + expansion * 1.0f);
                    float plume_dia = def.diameter * (float)ws_d * 2.0f * (1.1f + expansion * 4.2f);
                    
                    float groundDist = (float)rocket_state.altitude * (float)ws_d;
                    float ground_contact_depth = fmaxf(0.0f, plume_len - groundDist);
                    float splash_factor = ground_contact_depth / fmaxf(0.001f, plume_len);
                    plume_dia *= (1.0f + splash_factor * 8.0f); 
                    
                    Vec3 plumePos = nozzleWorldPos - nozzleDir * (plume_len * 0.5f);
                    Mat4 plumeMdl = Mat4::TRS(plumePos, nozzleWorldRot, Vec3(plume_dia, plume_len, plume_dia));
                    
                    r3d->drawExhaustVolumetric(rocketBox, plumeMdl, thrust, expansion, (float)glfwGetTime(), groundDist, plume_len);
                }
            }
        }
      }

      r3d->endFrame();
    }

    // ================= 2D HUD 叠加层 =================
    glDisable(GL_DEPTH_TEST);
    renderer->beginFrame();

    // 坐标转换变量（HUD也需要）
    double scale = 1.0 / (rocket_state.altitude * 1.5 + 200.0);
    float cx = 0.0f;
    float cy = 0.0f;
    double rocket_r = sqrt(rocket_state.px * rocket_state.px + rocket_state.py * rocket_state.py);
    double rocket_theta = atan2(rocket_state.py, rocket_state.px);
    double cam_angle = PI / 2.0 - rocket_theta;
    double sin_c = sin(cam_angle);
    double cos_c = cos(cam_angle);
    auto toScreenX = [&](double wx, double wy) {
      double rx = wx * cos_c - wy * sin_c;
      return (float)(rx * scale + cx);
    };
    auto toScreenY = [&](double wx, double wy) {
      double ry = wx * sin_c + wy * cos_c;
      return (float)((ry - rocket_r) * scale + cy);
    };
    float w = max(0.015f, (float)(10.0 * scale));
    float h = max(0.06f, (float)(40.0 * scale));
    float y_offset = -h / 2.0f;


    // ====================================================================
    // ===== 2D 叠加层 HUD =====
    
    if (show_hud) {  // 用户按 H 键切换开关
        
        // --- 强制重置 2D 渲染器批处理状态 ---
        // 结束之前的可能遗留的 2D 绘制 (如烟雾特效)
        renderer->endFrame();
        // 彻底重置 OpenGL 混合和深度测试状态，防止 3D 尾焰泄露
        glUseProgram(0);
        glDisable(GL_DEPTH_TEST);
        glDisable(GL_CULL_FACE); // 确保2D矩形不会因为绘制方向被意外剔除
        glDepthMask(GL_TRUE);
        glEnable(GL_BLEND);
        glBlendEquation(GL_FUNC_ADD); // 修复：引擎(Shock Diamonds)用了 GL_MAX 导致 HUD 的 alpha 混合失效
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glBindVertexArray(0);
        glBindBuffer(GL_ARRAY_BUFFER, 0);
        // 重新开启一个新的 2D 批处理专供 HUD 使用 (确保着色器正确绑定)
        renderer->beginFrame();

        float hud_opacity = 0.8f;

        float gauge_w = 0.03f;
        float gauge_h = 0.45f;
        float gauge_y_center = 0.4f;
        float gauge_vel_x = -0.92f;
        float gauge_alt_x = -0.84f;
        float gauge_fuel_x = -0.76f;

    double current_vel = sqrt(rocket_state.vx*rocket_state.vx + rocket_state.vy*rocket_state.vy + rocket_state.vz*rocket_state.vz);
    double current_alt = rocket_state.altitude;
    int current_vvel = (int)rocket_state.velocity;

    if (orbit_reference_sun) {
        double G_const = 6.67430e-11;
        double M_sun = 1.989e30;
        double GM_sun = G_const * M_sun;
        double au = 149597870700.0;
        double sun_angular_vel = sqrt(GM_sun / (au * au * au));
        double sun_angle = -1.2 + sun_angular_vel * rocket_state.sim_time;
        double current_sun_px = cos(sun_angle) * au;
        double current_sun_py = sin(sun_angle) * au;
        double current_sun_vx = -sin(sun_angle) * au * sun_angular_vel;
        double current_sun_vy = cos(sun_angle) * au * sun_angular_vel;

        double rel_vx = rocket_state.vx - current_sun_vx;
        double rel_vy = rocket_state.vy - current_sun_vy;
        double rel_vz = rocket_state.vz;
        double rel_px = rocket_state.px - current_sun_px;
        double rel_py = rocket_state.py - current_sun_py;
        double rel_pz = rocket_state.pz;
        
        current_vel = sqrt(rel_vx * rel_vx + rel_vy * rel_vy + rel_vz * rel_vz);
        double dist_to_sun = sqrt(rel_px * rel_px + rel_py * rel_py + rel_pz * rel_pz);
        current_alt = dist_to_sun - 696340000.0; 
        
        double rel_vvel_real = (rel_vx * rel_px + rel_vy * rel_py + rel_vz * rel_pz) / dist_to_sun;
        current_vvel = (int)rel_vvel_real;
    }

    // --- HUD Shared Variables ---
    float num_size = 0.025f;
    float num_x = 0.85f;
    float label_x = num_x + 0.065f; 
    float bg_w = 0.22f;
    float bg_h = 0.05f;
    double current_fuel = rocket_state.fuel;

    // --- New Detailed Telemetry HUD (Bottom-Left) ---
    float tl_x = -0.98f;
    float tl_y = -0.65f;
    float tl_size = 0.011f;
    float tl_spacing = 0.025f;
    char tl_buf[128];

    auto draw_tl_line = [&](const char* label, const char* value, float r, float g, float b) {
        renderer->drawText(tl_x, tl_y, label, tl_size, 0.7f, 0.7f, 0.7f, hud_opacity);
        renderer->drawText(tl_x + 0.18f, tl_y, value, tl_size, r, g, b, hud_opacity);
        tl_y -= tl_spacing;
    };

    // 1. Latitude and Longitude
    double planet_r = SOLAR_SYSTEM[current_soi_index].radius;
    double lat = asin(fmax(-1.0, fmin(1.0, rocket_state.surf_pz / planet_r))) * 180.0 / PI;
    double lon = atan2(rocket_state.surf_py, rocket_state.surf_px) * 180.0 / PI;
    snprintf(tl_buf, sizeof(tl_buf), "%.4f, %.4f", lat, lon);
    draw_tl_line("LAT/LON:", tl_buf, 0.4f, 0.8f, 1.0f);

    // 1b. Speed and Altitude
    snprintf(tl_buf, sizeof(tl_buf), "%.2f m/s", current_vel);
    draw_tl_line("SPEED:", tl_buf, 1.0f, 1.0f, 0.4f);
    if (current_alt > 100000.0)
        snprintf(tl_buf, sizeof(tl_buf), "%.2f km", current_alt / 1000.0);
    else
        snprintf(tl_buf, sizeof(tl_buf), "%.1f m", current_alt);
    draw_tl_line("ALTITUDE:", tl_buf, 0.4f, 1.0f, 0.8f);

    // 2. Thrust and TWR
    double current_thrust = rocket_state.thrust_power;
    double g_local = 9.80665 * pow(6371000.0 / (6371000.0 + current_alt), 2);
    double total_mass = assembly.total_dry_mass + rocket_state.fuel;
    double twr = (total_mass > 0) ? current_thrust / (total_mass * g_local) : 0;
    snprintf(tl_buf, sizeof(tl_buf), "%.2f kN (TWR: %.2f)", current_thrust / 1000.0, twr);
    draw_tl_line("THRUST:", tl_buf, 1.0f, 0.6f, 0.2f);

    // 3. Mass and Fuel
    snprintf(tl_buf, sizeof(tl_buf), "%.1f t (Fuel: %.1f t)", total_mass / 1000.0, rocket_state.fuel / 1000.0);
    draw_tl_line("MASS:", tl_buf, 0.8f, 0.8f, 0.8f);

    // 4. Remaining Delta-V
    double dv_rem = 0;
    if (total_mass > assembly.total_dry_mass && assembly.avg_isp > 0) {
        dv_rem = assembly.avg_isp * 9.80665 * log(total_mass / assembly.total_dry_mass);
    }
    snprintf(tl_buf, sizeof(tl_buf), "%.1f m/s", dv_rem);
    draw_tl_line("REMAIN DV:", tl_buf, 0.3f, 0.9f, 0.4f);

    // 5. Gravity and Drag
    double drag_mag = fabs(rocket_state.acceleration) * total_mass - current_thrust; // Rough estimate
    if (drag_mag < 0) drag_mag = 0;
    snprintf(tl_buf, sizeof(tl_buf), "%.3f m/s2 (Drag: %.1f N)", g_local, drag_mag);
    draw_tl_line("GRAVITY:", tl_buf, 0.6f, 0.7f, 1.0f);

    // 6. Orbital Elements
    tl_y -= 0.015f; // Extra space for orbital section
    renderer->drawText(tl_x, tl_y, "ORBITAL ELEMENTS:", tl_size, 0.5f, 0.9f, 1.0f, hud_opacity);
    tl_y -= tl_spacing;

    // Calculate elements (Simple 2D/3D approximation for HUD)
    double mu = 3.986e14;
    double r_mag = sqrt(rocket_state.px*rocket_state.px + rocket_state.py*rocket_state.py + rocket_state.pz*rocket_state.pz);
    double v_sq = rocket_state.vx*rocket_state.vx + rocket_state.vy*rocket_state.vy + rocket_state.vz*rocket_state.vz;
    double specific_energy = v_sq / 2.0 - mu / r_mag;
    double a_val = -mu / (2.0 * specific_energy);
    
    Vec3 r_vec(rocket_state.px, rocket_state.py, rocket_state.pz);
    Vec3 v_vec(rocket_state.vx, rocket_state.vy, rocket_state.vz);
    Vec3 h_vec = r_vec.cross(v_vec);
    double h_mag = h_vec.length();
    double e_val = sqrt(1.0 + (2.0 * specific_energy * h_mag * h_mag) / (mu * mu));
    double inc = acos(h_vec.z / h_mag) * 180.0 / PI;

    snprintf(tl_buf, sizeof(tl_buf), "a: %.1f km, e: %.4f", a_val / 1000.0, e_val);
    draw_tl_line("A/E:", tl_buf, 1.0f, 1.0f, 1.0f);
    snprintf(tl_buf, sizeof(tl_buf), "i: %.2f deg", inc);
    draw_tl_line("INC:", tl_buf, 1.0f, 1.0f, 1.0f);

    // 燃油读数 + kg
    renderer->addRect(num_x, 0.4f, bg_w, bg_h, 0.0f, 0.0f, 0.0f, 0.5f);
    renderer->drawInt(num_x + 0.05f, 0.4f, (int)current_fuel, num_size, 0.9f, 0.7f, 0.2f, hud_opacity, Renderer::RIGHT);
    renderer->drawText(label_x, 0.4f, "kg", num_size * 0.7f, 1.0f, 0.8f, 0.3f, hud_opacity);

    // 油门读数 + %
    renderer->addRect(num_x, 0.25f, bg_w * 0.8f, bg_h * 0.8f, 0.0f, 0.0f, 0.0f, 0.5f);
    renderer->drawInt(num_x + 0.04f, 0.25f, (int)(control_input.throttle * 100), num_size * 0.8f, 0.8f, 0.8f, 0.8f, hud_opacity, Renderer::RIGHT);
    renderer->drawText(label_x - 0.02f, 0.25f, "%", num_size * 0.6f, 0.8f, 0.8f, 0.8f, hud_opacity);

    // 俯仰读数 (Pitch) + 度数
    renderer->addRect(num_x, 0.10f, bg_w * 0.8f, bg_h * 0.8f, 0.0f, 0.0f, 0.0f, 0.5f);
    int pitch_deg = (int)(abs(rocket_state.angle_z) * 180.0 / PI);
    float pr = 0.8f, pg = 0.8f, pb = 0.8f;
    if (pitch_deg > 20 && current_vel > 200.0) {
        pr = 1.0f; pg = 0.2f; pb = 0.2f; // Red Warning (high Q area)
    } else if (pitch_deg > 20) {
        pr = 1.0f; pg = 0.7f; pb = 0.2f; // Yellow Warning (Safe speed)
    }
    renderer->drawInt(num_x + 0.04f, 0.10f, pitch_deg, num_size * 0.8f, pr, pg, pb, hud_opacity, Renderer::RIGHT);
    renderer->drawText(label_x - 0.02f, 0.10f, "deg", num_size * 0.5f, pr, pg, pb, hud_opacity);

    // 垂直速度读数 + m/s
    renderer->addRect(num_x, -0.05f, bg_w, bg_h, 0.05f, 0.05f, 0.05f, 0.5f);
    float vr = current_vvel < 0 ? 1.0f : 0.3f;
    float vg = current_vvel >= 0 ? 1.0f : 0.3f;
    renderer->drawInt(num_x + 0.05f, -0.05f, current_vvel, num_size * 0.9f, vr, vg, 0.3f, hud_opacity, Renderer::RIGHT);
    renderer->drawText(label_x, -0.05f, "v_m/s", num_size * 0.5f, 0.7f, 0.7f, 0.7f, hud_opacity);


    // --- 6. 控制模式指示器 (HUD 右上角 - 点击切换) ---
    float mode_x = 0.88f; 
    float mode_y = 0.85f;
    float mode_w = 0.15f;
    float mode_h = 0.04f;
    
    // 获取鼠标状态用于 HUD 点击
    double hmx, hmy;
    glfwGetCursorPos(window, &hmx, &hmy);
    int hww, hwh;
    glfwGetWindowSize(window, &hww, &hwh);
    float hmouse_x = (float)(hmx / hww * 2.0 - 1.0);
    float hmouse_y = (float)(1.0 - hmy / hwh * 2.0);
    bool hlmb = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS;
    static bool hlmb_prev = false;
    
    bool is_hover_mode = (hmouse_x >= mode_x - mode_w/2.0f && hmouse_x <= mode_x + mode_w/2.0f &&
                          hmouse_y >= mode_y - mode_h/2.0f && hmouse_y <= mode_y + mode_h/2.0f);
    
    if (is_hover_mode && hlmb && !hlmb_prev) {
        rocket_state.auto_mode = !rocket_state.auto_mode;
        if (rocket_state.auto_mode) {
           rocket_state.pid_vert.reset();
           rocket_state.pid_pos.reset();
           rocket_state.pid_att.reset();
           rocket_state.pid_att_z.reset();
           rocket_state.mission_msg = ">> AUTOPILOT ENGAGED (MOUSE)";
        } else {
           rocket_state.mission_msg = ">> MANUAL CONTROL ACTIVE (MOUSE)";
        }
    }
    // hlmb_prev updated after all HUD interactions

    renderer->addRect(mode_x, mode_y, mode_w, mode_h, 0.05f, 0.05f, 0.05f, 0.7f);
    if (rocket_state.auto_mode) {
      float pulse = is_hover_mode ? 1.0f : 0.8f;
      renderer->addRect(mode_x, mode_y, mode_w - 0.02f, mode_h - 0.15f, 0.1f * pulse, 0.8f * pulse, 0.2f * pulse, 0.9f);
      renderer->drawText(mode_x, mode_y, "AUTO", 0.020f, 0.1f, 0.1f, 0.1f, 0.9f, false, Renderer::CENTER);
    } else {
      float pulse = is_hover_mode ? 1.0f : 0.8f;
      renderer->addRect(mode_x, mode_y, mode_w - 0.02f, mode_h - 0.15f, 1.0f * pulse, 0.6f * pulse, 0.1f * pulse, 0.9f);
      renderer->drawText(mode_x, mode_y, "MANUAL", 0.020f, 0.1f, 0.1f, 0.1f, 0.9f, false, Renderer::CENTER);
    }

    // --- 7. Mission Status & Time (Top Center) ---
    renderer->drawText(0.0f, 0.85f, "[MISSION CONTROL]", 0.016f, 0.4f, 1.0f, 0.4f, hud_opacity, true, Renderer::CENTER);
    renderer->drawText(0.0f, 0.80f, rocket_state.mission_msg.c_str(), 0.015f, 0.8f, 0.8f, 1.0f, hud_opacity, true, Renderer::CENTER);

    // Free Camera Speed Display
    if (cam_mode_3d == 3) {
      char speed_buf[64];
      if (free_cam_move_speed > 1000000.0f)
          snprintf(speed_buf, sizeof(speed_buf), "FREE CAM SPEED: %.2f km/s", free_cam_move_speed / 1000.0f);
      else
          snprintf(speed_buf, sizeof(speed_buf), "FREE CAM SPEED: %.1f m/s", free_cam_move_speed);
      renderer->drawText(0.0f, 0.75f, speed_buf, 0.015f, 0.4f, 1.0f, 1.0f, 0.9f, true, Renderer::CENTER);
    }

    // Time Display & Toggle
    float time_y = 0.92f;
    float time_w = 0.35f;
    float time_h = 0.04f;
    bool is_hover_time = (hmouse_x >= -time_w/2.0f && hmouse_x <= time_w/2.0f &&
                          hmouse_y >= time_y - time_h/2.0f && hmouse_y <= time_y + time_h/2.0f);
    
    if (is_hover_time && hlmb && !hlmb_prev) {
        rocket_state.show_absolute_time = !rocket_state.show_absolute_time;
    }

    renderer->addRect(0.0f, time_y, time_w, time_h, 0.05f, 0.05f, 0.05f, 0.7f);
    string time_str = formatTime(rocket_state.show_absolute_time ? rocket_state.sim_time : rocket_state.mission_timer, rocket_state.show_absolute_time);
    renderer->drawText(0.0f, time_y, time_str.c_str(), 0.018f, 1.0f, 1.0f, 1.0f, 0.9f, true, Renderer::CENTER);

    // Time Warp Display
    if (time_warp > 1) {
        char warp_buf[32];
        snprintf(warp_buf, sizeof(warp_buf), "WARP: %dx", time_warp);
        renderer->drawText(0.25f, time_y, warp_buf, 0.015f, 1.0f, 0.8f, 0.1f, 0.9f, true, Renderer::LEFT);
    }

    // --- 8. Stage Indicator (Below mode indicator) ---
    if (rocket_state.total_stages > 1) {
        float stage_x = 0.88f;
        float stage_y = mode_y - 0.06f;
        float stage_w = 0.15f;
        float stage_h = 0.04f;
        renderer->addRect(stage_x, stage_y, stage_w, stage_h, 0.05f, 0.05f, 0.1f, 0.7f);
        
        // Stage number display
        char stage_str[32];
        snprintf(stage_str, sizeof(stage_str), "STG %d/%d", rocket_state.current_stage + 1, rocket_state.total_stages);
        
        float sr = 0.3f, sg = 0.8f, sb = 1.0f;
        if (rocket_state.fuel < 1000 && rocket_state.current_stage < rocket_state.total_stages - 1) {
            // Blink warning when fuel low and can still stage
            float blink = 0.5f + 0.5f * sinf((float)glfwGetTime() * 6.0f);
            sr = 1.0f * blink; sg = 0.5f * blink; sb = 0.1f;
        }
        renderer->drawText(stage_x, stage_y, stage_str, 0.016f, sr, sg, sb, 0.9f, false, Renderer::CENTER);
        
        // Per-stage fuel bars (small bars below stage indicator)
        float bar_y_start = stage_y - 0.04f;
        float bar_w = stage_w * 0.8f;
        float bar_h = 0.012f;
        for (int si = 0; si < rocket_state.total_stages; si++) {
            float by = bar_y_start - si * (bar_h + 0.005f);
            // Background
            renderer->addRect(stage_x, by, bar_w, bar_h, 0.15f, 0.15f, 0.15f, 0.5f);
            // Fill
            if (si < (int)rocket_state.stage_fuels.size() && si < (int)rocket_config.stage_configs.size()) {
                float cap = (float)rocket_config.stage_configs[si].fuel_capacity;
                float ratio = (cap > 0) ? (float)(rocket_state.stage_fuels[si] / cap) : 0.0f;
                ratio = fmaxf(0.0f, fminf(1.0f, ratio));
                float fill = bar_w * ratio;
                float fx = stage_x - bar_w / 2.0f + fill / 2.0f;
                float fr = (si == rocket_state.current_stage) ? 0.2f : 0.4f;
                float fg = (si == rocket_state.current_stage) ? 1.0f : 0.5f;
                float fb = (si == rocket_state.current_stage) ? 0.4f : 0.3f;
                if (si < rocket_state.current_stage) { fr = 0.3f; fg = 0.3f; fb = 0.3f; } // jettisoned
                renderer->addRect(fx, by, fill, bar_h * 0.7f, fr, fg, fb, 0.8f);
            }
        }
    }
    
    // 保存指示器 (每次保存后闪烁3秒)
    static int last_save_frame = -1000;
    if (frame % 300 == 0 && rocket_state.status != PRE_LAUNCH) {
        last_save_frame = frame;
        SaveSystem::SaveGame(assembly, rocket_state, control_input);
    }
    int frames_since_save = frame - last_save_frame;
    if (frames_since_save < 180) { // 3秒 = 180帧
        float save_alpha = 1.0f - (float)frames_since_save / 180.0f;
        float save_pulse = 0.5f + 0.5f * sinf((float)frames_since_save * 0.3f);
        renderer->drawText(-0.88f, -0.85f, "AUTOSAVED", 0.012f, 
                          0.3f, 1.0f * save_pulse, 0.4f, save_alpha * hud_opacity);
    }

    // --- 6. 油门指示条 (HUD 底部中央) ---
    float thr_bar_x = 0.0f;
    float thr_bar_y = -0.92f;
    float thr_bar_w = 0.5f;
    float thr_bar_h = 0.025f;
    // 背景
    renderer->addRect(thr_bar_x, thr_bar_y, thr_bar_w + 0.02f,
                      thr_bar_h + 0.01f, 0.1f, 0.1f, 0.1f, 0.5f * hud_opacity);
    // 填充
    float thr_fill = thr_bar_w * (float)control_input.throttle;
    float thr_fill_x = thr_bar_x - thr_bar_w / 2.0f + thr_fill / 2.0f;
    float thr_r = (float)control_input.throttle > 0.8f
                      ? 1.0f
                      : 0.3f + (float)control_input.throttle * 0.7f;
    float thr_g = (float)control_input.throttle < 0.5f
                      ? 0.8f
                      : 0.8f - ((float)control_input.throttle - 0.5f) * 1.6f;
    renderer->addRect(thr_fill_x, thr_bar_y, thr_fill, thr_bar_h, thr_r, thr_g,
                      0.1f, hud_opacity);

    // --- 7. 姿态球 (Navball) ---
    float nav_x = 0.0f;
    float nav_y = -0.70f;
    float nav_rad = 0.18f;

    // 计算轨道向量 (相对于当前 SOI)
    Vec3 vPrograde(0, 0, 0), vNormal(0, 0, 0), vRadial(0, 0, 0);
    {
        double rel_vx = rocket_state.vx, rel_vy = rocket_state.vy, rel_vz = rocket_state.vz;
        double rel_px = rocket_state.px, rel_py = rocket_state.py, rel_pz = rocket_state.pz;
        
        if (orbit_reference_sun && current_soi_index != 0) {
            CelestialBody& cb = SOLAR_SYSTEM[current_soi_index];
            rel_vx += cb.vx; rel_vy += cb.vy; rel_vz += cb.vz;
            rel_px += cb.px; rel_py += cb.py; rel_pz += cb.pz;
        }
        
        double speed = sqrt(rel_vx*rel_vx + rel_vy*rel_vy + rel_vz*rel_vz);
        if (speed > 0.1) {
            vPrograde = Vec3((float)(rel_vx / speed), (float)(rel_vy / speed), (float)(rel_vz / speed));
            
            Vec3 posVec((float)rel_px, (float)rel_py, (float)rel_pz);
            vNormal = vPrograde.cross(posVec).normalized();
            vRadial = vNormal.cross(vPrograde).normalized();
        }
    }

    // Maneuver target calculation (3D & Real-time)
    Vec3 vManeuver(0,0,0);
    double dv_remaining = 0;
    if (rocket_state.selected_maneuver_index != -1 && (size_t)rocket_state.selected_maneuver_index < rocket_state.maneuvers.size()) {
        ManeuverNode& node = rocket_state.maneuvers[rocket_state.selected_maneuver_index];
        
        if (node.snap_valid) {
            // High-Precision Target Orbit Tracking (Principia Style)
            Vec3 rem_v = ManeuverSystem::calculateRemainingDV(rocket_state, node);
            dv_remaining = (double)rem_v.length();
            vManeuver = rem_v.normalized();
        } else {
            // Pre-ignition: Use projected state at node
            int ref_idx = (node.ref_body >= 0) ? node.ref_body : current_soi_index;
            double mu = G_const * SOLAR_SYSTEM[ref_idx].mass;
            double npx, npy, npz, nvx, nvy, nvz;
            get3DStateAtTime(rocket_state.px, rocket_state.py, rocket_state.pz, rocket_state.vx, rocket_state.vy, rocket_state.vz, mu, node.sim_time - rocket_state.sim_time, npx, npy, npz, nvx, nvy, nvz);
            ManeuverFrame frame = ManeuverSystem::getFrame(Vec3((float)npx, (float)npy, (float)npz), Vec3((float)nvx, (float)nvy, (float)nvz));
            vManeuver = (frame.prograde * node.delta_v.x + frame.normal * node.delta_v.y + frame.radial * node.delta_v.z).normalized();
            dv_remaining = (double)node.delta_v.length();
        }
    }

    // 直接使用四元数投影，彻底解决万向锁和翻转问题
    renderer->drawAttitudeSphere(nav_x, nav_y, nav_rad, rocketQuat, localRight, rocketUp, localNorth, rocket_state.sas_active, rocket_state.rcs_active, vPrograde, vNormal, vRadial, vManeuver);

    // --- Maneuver Info (Next to Navball) ---
    if (dv_remaining > 0.1) {
        renderer->drawText(nav_x + nav_rad + 0.05f, nav_y, "MNV BV", 0.015f, 0.2f, 0.6f, 1.0f, 1.0f);
        char dv_str[32]; snprintf(dv_str, sizeof(dv_str), "%.1f m/s", dv_remaining);
        renderer->drawText(nav_x + nav_rad + 0.05f, nav_y - 0.03f, dv_str, 0.02f, 1.0f, 1.0f, 1.0f, 1.0f);
    }

    // --- 8. SAS 模式按钮 (位于导航球右侧) ---
    float btn_start_x = nav_x + nav_rad + 0.12f;
    float btn_y_top = nav_y + nav_rad * 0.5f;
    float btn_w = 0.05f, btn_h = 0.05f;
    float spacing = 0.06f;

    struct SASBtn { SASMode mode; const char* label; float r, g, b; };
    std::vector<SASBtn> sas_btns = {
        {SAS_STABILITY, "STB", 0.7f, 0.7f, 0.7f},
        {SAS_PROGRADE, "PRO", 0.2f, 0.8f, 0.2f},
        {SAS_RETROGRADE, "RET", 0.8f, 0.8f, 0.2f},
        {SAS_NORMAL, "NRM", 0.8f, 0.2f, 0.8f},
        {SAS_ANTINORMAL, "ANT", 0.5f, 0.1f, 0.5f},
        {SAS_RADIAL_IN, "R-I", 0.2f, 0.5f, 0.8f},
        {SAS_RADIAL_OUT, "R-O", 0.1f, 0.3f, 0.6f},
        {SAS_MANEUVER, "MNV", 0.3f, 0.6f, 1.0f}
    };

    for (int i = 0; i < (int)sas_btns.size(); i++) {
        float bx = btn_start_x + (i % 2) * spacing;
        float by = btn_y_top - (i / 2) * spacing;
        
        bool hover = (hmouse_x >= bx - btn_w/2 && hmouse_x <= bx + btn_w/2 && hmouse_y >= by - btn_h/2 && hmouse_y <= by + btn_h/2);
        if (hover && hlmb && !hlmb_prev) {
            rocket_state.sas_mode = sas_btns[i].mode;
            rocket_state.sas_active = true;
            rocket_state.auto_mode = false; // Override autopilot
            cout << ">> SAS MODE: " << sas_btns[i].label << " (Active)" << endl;
        }
        
        float alpha = (rocket_state.sas_mode == sas_btns[i].mode) ? 0.9f : 0.4f;
        if (hover) alpha += 0.1f;
        renderer->addRect(bx, by, btn_w, btn_h, sas_btns[i].r, sas_btns[i].g, sas_btns[i].b, alpha);
        renderer->drawText(bx, by, sas_btns[i].label, 0.012f, 1, 1, 1, 0.9f, true, Renderer::CENTER);
    }
    
    // --- 9. Advanced Orbit UI (Right Edge) ---
    if (cam_mode_3d == 2 && !SOLAR_SYSTEM.empty()) {
        float adv_btn_w = 0.15f;
        float adv_btn_h = 0.05f;
        float adv_btn_x = 0.88f;
        float adv_btn_y = mode_y - 0.35f; // below stage UI
        
        bool hover_adv = (hmouse_x >= adv_btn_x - adv_btn_w/2 && hmouse_x <= adv_btn_x + adv_btn_w/2 && hmouse_y >= adv_btn_y - adv_btn_h/2 && hmouse_y <= adv_btn_y + adv_btn_h/2);
        if (hover_adv && hlmb && !hlmb_prev) adv_orbit_menu = !adv_orbit_menu;
        
        renderer->addRect(adv_btn_x, adv_btn_y, adv_btn_w, adv_btn_h, 0.2f, 0.4f, 0.8f, hover_adv ? 0.9f : 0.7f);
        renderer->drawText(adv_btn_x, adv_btn_y, "ADV ORBIT", 0.012f, 1, 1, 1, 1.0f, true, Renderer::CENTER);

        // --- Flight Assist Button (Below ADV ORBIT) ---
        float fa_btn_y = adv_btn_y - 0.06f;
        bool hover_fa = (hmouse_x >= adv_btn_x - adv_btn_w/2 && hmouse_x <= adv_btn_x + adv_btn_w/2 && hmouse_y >= fa_btn_y - adv_btn_h/2 && hmouse_y <= fa_btn_y + adv_btn_h/2);
        if (hover_fa && hlmb && !hlmb_prev) flight_assist_menu = !flight_assist_menu;
        
        renderer->addRect(adv_btn_x, fa_btn_y, adv_btn_w, adv_btn_h, 0.8f, 0.4f, 0.2f, hover_fa ? 0.9f : 0.7f);
        renderer->drawText(adv_btn_x, fa_btn_y, "FLIGHT ASSIST", 0.012f, 1, 1, 1, 1.0f, true, Renderer::CENTER);

        // --- Galaxy Info Button ---
        float galaxy_btn_y = fa_btn_y - 0.06f;
        bool hover_galaxy = (hmouse_x >= adv_btn_x - adv_btn_w/2 && hmouse_x <= adv_btn_x + adv_btn_w/2 && hmouse_y >= galaxy_btn_y - adv_btn_h/2 && hmouse_y <= galaxy_btn_y + adv_btn_h/2);
        if (hover_galaxy && hlmb && !hlmb_prev) show_galaxy_info = !show_galaxy_info;
        
        renderer->addRect(adv_btn_x, galaxy_btn_y, adv_btn_w, adv_btn_h, 0.1f, 0.6f, 0.3f, hover_galaxy ? 0.9f : 0.7f);
        renderer->drawText(adv_btn_x, galaxy_btn_y, "GALAXY INFO", 0.012f, 1, 1, 1, 1.0f, true, Renderer::CENTER);

        // --- Climate View Button ---
        float climate_btn_y = galaxy_btn_y - 0.06f;
        bool hover_climate = (hmouse_x >= adv_btn_x - adv_btn_w/2 && hmouse_x <= adv_btn_x + adv_btn_w/2 && hmouse_y >= climate_btn_y - adv_btn_h/2 && hmouse_y <= climate_btn_y + adv_btn_h/2);
        if (hover_climate && hlmb && !hlmb_prev) {
            r3d->setClimateViewMode((r3d->climateViewMode + 1) % 5);
        }
        
        static const char* climate_mode_names[] = {"NORMAL", "TEMPERATURE", "PRECIPITATION", "PRESSURE", "HYDROLOGY"};
        renderer->addRect(adv_btn_x, climate_btn_y, adv_btn_w, adv_btn_h, 0.4f, 0.2f, 0.6f, hover_climate ? 0.9f : 0.7f);
        renderer->drawText(adv_btn_x, climate_btn_y, "CLIMATE VIEW", 0.010f, 1, 1, 1, 1.0f, true, Renderer::CENTER);
        renderer->drawText(adv_btn_x, climate_btn_y - 0.022f, climate_mode_names[r3d->climateViewMode], 0.007f, 0.7f, 0.9f, 1.0f, 0.9f, true, Renderer::CENTER);

        if (show_galaxy_info) {
            float bar_h = 0.12f;
            float bar_y = 0.92f;
            renderer->addRect(0, bar_y, 2.0f, bar_h, 0.05f, 0.05f, 0.08f, 0.9f);
            renderer->addLine(-1.0f, bar_y - bar_h/2, 1.0f, bar_y - bar_h/2, 0.002f, 0.4f, 0.8f, 1.0f, 0.6f);
            
            float icon_start_x = -0.92f;
            float icon_spacing = 0.12f;
            float icon_r = 0.04f;
            
            // 1. Draw Suns and Planets (parent_index == -1)
            int main_count = 0;
            for (int i = 0; i < (int)SOLAR_SYSTEM.size(); i++) {
                if (SOLAR_SYSTEM[i].parent_index != -1) continue;
                
                float ix = icon_start_x + main_count * icon_spacing;
                bool hover_icon = (hmouse_x >= ix - icon_r && hmouse_x <= ix + icon_r && hmouse_y >= bar_y - icon_r && hmouse_y <= bar_y + icon_r);
                
                if (hover_icon && hlmb && !hlmb_prev) {
                    selected_body_idx = i;
                    // Logic: Planet click expands/collapses moons
                    bool has_moons = false;
                    for(int m=0; m<(int)SOLAR_SYSTEM.size(); m++) if(SOLAR_SYSTEM[m].parent_index == i) has_moons = true;
                    
                    if (has_moons) {
                        if (expanded_planet_idx == i) expanded_planet_idx = -1;
                        else expanded_planet_idx = i;
                    }
                }
                
                renderer->drawPlanetIcon(ix, bar_y, icon_r, SOLAR_SYSTEM[i], (float)glfwGetTime());
                if (hover_icon || selected_body_idx == i) 
                    renderer->addCircleOutline(ix, bar_y, icon_r * 1.1f, 0.003f, 0.4f, 0.8f, 1.0f, 1.0f);
                
                renderer->drawText(ix, bar_y - icon_r - 0.015f, SOLAR_SYSTEM[i].name.c_str(), 0.008f, 1, 1, 1, 1.0f, true, Renderer::CENTER);
                main_count++;
            }
            
            // 2. Draw expansion moons if a planet is selected
            if (expanded_planet_idx != -1) {
                float arrow_x = icon_start_x + (expanded_planet_idx == 0 ? 0 : (expanded_planet_idx < 4 ? expanded_planet_idx : expanded_planet_idx - 1)) * icon_spacing;
                renderer->drawText(arrow_x, bar_y - 0.07f, "[v MOONS v]", 0.007f, 0.4f, 1.0f, 0.6f, 1.0f, true, Renderer::CENTER);
                
                float moon_bar_y = bar_y - 0.15f;
                float moon_bar_h = 0.10f;
                renderer->addRect(0, moon_bar_y, 2.0f, moon_bar_h, 0.04f, 0.04f, 0.06f, 0.85f);
                
                int moon_count = 0;
                for (int i = 0; i < (int)SOLAR_SYSTEM.size(); i++) {
                    if (SOLAR_SYSTEM[i].parent_index == expanded_planet_idx) {
                        float mix = icon_start_x + moon_count * icon_spacing;
                        float mr = 0.03f;
                        bool hover_moon = (hmouse_x >= mix - mr && hmouse_x <= mix + mr && hmouse_y >= moon_bar_y - mr && hmouse_y <= moon_bar_y + mr);
                        
                        if (hover_moon && hlmb && !hlmb_prev) {
                            selected_body_idx = i;
                        }
                        
                        renderer->drawPlanetIcon(mix, moon_bar_y, mr, SOLAR_SYSTEM[i], (float)glfwGetTime());
                        if (hover_moon || selected_body_idx == i)
                             renderer->addCircleOutline(mix, moon_bar_y, mr * 1.1f, 0.002f, 0.4f, 1.0f, 0.6f, 1.0f);
                        
                        renderer->drawText(mix, moon_bar_y - mr - 0.012f, SOLAR_SYSTEM[i].name.c_str(), 0.007f, 0.8f, 1.0f, 0.8f, 1.0f, true, Renderer::CENTER);
                        moon_count++;
                    }
                }
            }
        }

        // --- Detailed Info Panel (Top Left) ---
        if (selected_body_idx != -1 && selected_body_idx < (int)SOLAR_SYSTEM.size()) {
            const CelestialBody& b = SOLAR_SYSTEM[selected_body_idx];
            float panel_w = 0.35f;
            float panel_h = 0.45f;
            float panel_x = -1.0f + panel_w/2.0f + 0.02f;
            float panel_y = 0.92f - (show_galaxy_info ? 0.12f : 0) - (expanded_planet_idx != -1 ? 0.10f : 0) - panel_h/2.0f - 0.02f;
            
            renderer->addRect(panel_x, panel_y, panel_w, panel_h, 0.03f, 0.03f, 0.05f, 0.85f);
            renderer->addRectOutline(panel_x, panel_y, panel_w, panel_h, 0.4f, 0.8f, 1.0f, 0.7f);
            
            float tx = panel_x - panel_w/2.0f + 0.02f;
            float ty = panel_y + panel_h/2.0f - 0.04f;
            float line_h = 0.025f;
            char buf[128];
            
            renderer->drawText(tx, ty, b.name.c_str(), 0.018f, b.r, b.g, b.b, 1.0f);
            ty -= line_h * 1.5f;
            
            const char* types[] = {"STAR", "TERRESTRIAL", "GAS GIANT", "MOON", "RINGED GIANT"};
            snprintf(buf, sizeof(buf), "TYPE: %s", types[b.type]);
            renderer->drawText(tx, ty, buf, 0.010f, 0.7f, 0.7f, 0.7f); ty -= line_h;
            
            snprintf(buf, sizeof(buf), "RADIUS: %.1f km", b.radius / 1000.0);
            renderer->drawText(tx, ty, buf, 0.009f, 1, 1, 1); ty -= line_h;
            
            snprintf(buf, sizeof(buf), "MASS: %.3e kg", b.mass);
            renderer->drawText(tx, ty, buf, 0.009f, 1, 1, 1); ty -= line_h;
            
            double g_surf = (6.674e-11 * b.mass) / (b.radius * b.radius);
            snprintf(buf, sizeof(buf), "SURFACE G: %.3f m/s2", g_surf);
            renderer->drawText(tx, ty, buf, 0.009f, 1, 1, 1); ty -= line_h;
            
            double volume = (4.0/3.0) * 3.14159 * pow(b.radius, 3);
            double density = b.mass / volume;
            snprintf(buf, sizeof(buf), "DENSITY: %.1f kg/m3", density);
            renderer->drawText(tx, ty, buf, 0.009f, 1, 1, 1); ty -= line_h;
            
            snprintf(buf, sizeof(buf), "ORBIT PERIOD: %.2f days", b.orbital_period / 86400.0);
            renderer->drawText(tx, ty, buf, 0.009f, 1, 1, 1); ty -= line_h;
            
            snprintf(buf, sizeof(buf), "ECCENTRICITY: %.4f", b.eccentricity);
            renderer->drawText(tx, ty, buf, 0.009f, 1, 1, 1); ty -= line_h;
            
            snprintf(buf, sizeof(buf), "INCLINATION: %.2f deg", b.inclination * 180.0 / 3.14159);
            renderer->drawText(tx, ty, buf, 0.009f, 1, 1, 1); ty -= line_h;
            
            snprintf(buf, sizeof(buf), "SEMI-MAJOR AXIS: %.2f AU", b.sma_base / 1.496e11);
            renderer->drawText(tx, ty, buf, 0.009f, 1, 1, 1); ty -= line_h;
            
            snprintf(buf, sizeof(buf), "ATMOSPHERE: %.2f hPa", b.surface_pressure);
            renderer->drawText(tx, ty, buf, 0.009f, 1, 1, 1); ty -= line_h;
            
            snprintf(buf, sizeof(buf), "TEMPERATURE: %.1f K", b.average_temp);
            renderer->drawText(tx, ty, buf, 0.009f, 1, 1, 1); ty -= line_h;
            
            snprintf(buf, sizeof(buf), "AXIAL TILT: %.2f deg", b.axial_tilt * 180.0 / 3.14159);
            renderer->drawText(tx, ty, buf, 0.009f, 1, 1, 1); ty -= line_h;
            
            int moon_count = 0;
            for(int m=0; m<(int)SOLAR_SYSTEM.size(); m++) if(SOLAR_SYSTEM[m].parent_index == selected_body_idx) moon_count++;
            snprintf(buf, sizeof(buf), "MOONS: %d", moon_count);
            renderer->drawText(tx, ty, buf, 0.009f, 1, 1, 1); ty -= line_h;
            
            snprintf(buf, sizeof(buf), "ALBEDO (SCAT): %.3f", b.scattering_coef);
            renderer->drawText(tx, ty, buf, 0.009f, 1, 1, 1); ty -= line_h;
        }

        if (flight_assist_menu) {
            float menu_w = 0.25f;
            float menu_h = 0.27f;
            float menu_x = adv_btn_x - adv_btn_w/2 - menu_w/2 - 0.02f;
            float menu_y = fa_btn_y;
            renderer->addRect(menu_x, menu_y, menu_w, menu_h, 0.1f, 0.05f, 0.05f, 0.85f);
            renderer->addRectOutline(menu_x, menu_y, menu_w, menu_h, 1.0f, 0.6f, 0.4f, 0.8f);

            auto draw_toggle = [&](float y, const char* label, bool active, bool& toggle_var, std::function<void()> on_true = nullptr) {
                float ty = menu_y + menu_h/2 - y;
                bool hover = (hmouse_x >= menu_x - menu_w/2 && hmouse_x <= menu_x + menu_w/2 && hmouse_y >= ty - 0.025f && hmouse_y <= ty + 0.025f);
                if (hover && hlmb && !hlmb_prev) {
                    toggle_var = !toggle_var;
                    if (toggle_var && on_true) on_true();
                }
                renderer->addRect(menu_x, ty, menu_w * 0.9f, 0.04f, hover ? 0.3f : 0.15f, hover ? 0.3f : 0.15f, hover ? 0.3f : 0.15f, 0.8f);
                renderer->drawText(menu_x - menu_w * 0.4f, ty, label, 0.012f, 1, 1, 1, 0.9f, true, Renderer::LEFT);
                renderer->drawText(menu_x + menu_w * 0.35f, ty, active ? "ON" : "OFF", 0.012f, active ? 0.2f : 1.0f, active ? 1.0f : 0.2f, 0.2f, 0.9f, true, Renderer::CENTER);
            };

            draw_toggle(0.05f, "AUTO EXECUTE MNV", auto_exec_mnv, auto_exec_mnv);
            
            bool is_ascent = (rocket_state.status == ASCEND);
            bool temp_ascent = is_ascent;
            draw_toggle(0.10f, "AUTO ORBIT", is_ascent, temp_ascent, [&](){
                rocket_state.status = ASCEND;
                rocket_state.mission_phase = 0;
                rocket_state.auto_mode = true;
                rocket_state.mission_msg = "AUTOPILOT: INITIATING ASCENT...";
            });

            bool is_descent = (rocket_state.status == DESCEND);
            bool temp_descent = is_descent;
            draw_toggle(0.15f, "AUTO LANDING", is_descent, temp_descent, [&](){
                rocket_state.status = DESCEND;
                rocket_state.auto_mode = true;
                rocket_state.mission_msg = "AUTOPILOT: INITIATING LANDING...";
            });

            // Transfer Window button
            {
                float ty = menu_y + menu_h/2 - 0.20f;
                bool hover = (hmouse_x >= menu_x - menu_w/2 && hmouse_x <= menu_x + menu_w/2 && hmouse_y >= ty - 0.025f && hmouse_y <= ty + 0.025f);
                if (hover && hlmb && !hlmb_prev) {
                    transfer_window_menu = !transfer_window_menu;
                }
                float btn_r = transfer_window_menu ? 0.3f : 0.15f;
                float btn_g = transfer_window_menu ? 0.5f : 0.15f;
                float btn_b = transfer_window_menu ? 0.8f : 0.15f;
                if (hover) { btn_r += 0.1f; btn_g += 0.1f; btn_b += 0.1f; }
                renderer->addRect(menu_x, ty, menu_w * 0.9f, 0.04f, btn_r, btn_g, btn_b, 0.8f);
                renderer->drawText(menu_x - menu_w * 0.4f, ty, "TRANSFER WINDOW", 0.011f, 0.3f, 0.8f, 1.0f, 0.9f, true, Renderer::LEFT);
                renderer->drawText(menu_x + menu_w * 0.35f, ty, transfer_window_menu ? "[v]" : "[>]", 0.011f, 0.6f, 0.8f, 1.0f, 0.9f, true, Renderer::CENTER);
            }
        }

        // === Transfer Window Popup Panel ===
        if (transfer_window_menu && flight_assist_menu) {
            float tw_w = 0.55f;
            float tw_h = 0.60f;
            float tw_x = adv_btn_x - adv_btn_w/2 - 0.25f - tw_w/2 - 0.04f;
            float tw_y = fa_btn_y - 0.05f;

            // Background panel
            renderer->addRect(tw_x, tw_y, tw_w, tw_h, 0.04f, 0.04f, 0.08f, 0.92f);
            renderer->addRectOutline(tw_x, tw_y, tw_w, tw_h, 0.3f, 0.6f, 1.0f, 0.9f);
            renderer->addRect(tw_x, tw_y + tw_h*0.42f, tw_w, tw_h*0.16f, 0.06f, 0.06f, 0.14f, 0.3f);

            // Title
            float title_y = tw_y + tw_h/2 - 0.02f;
            renderer->drawText(tw_x, title_y, "TRANSFER WINDOW CALCULATOR", 0.013f, 0.4f, 0.8f, 1.0f, 1.0f, true, Renderer::CENTER);

            // --- Target Body Selector ---
            float sel_y = title_y - 0.04f;
            renderer->drawText(tw_x - tw_w*0.4f, sel_y, "Target:", 0.011f, 0.8f, 0.8f, 0.8f, 1.0f, true, Renderer::LEFT);

            // Left arrow
            float arr_lx = tw_x - 0.02f;
            bool hover_tgt_l = (hmouse_x >= arr_lx - 0.015f && hmouse_x <= arr_lx + 0.015f && hmouse_y >= sel_y - 0.015f && hmouse_y <= sel_y + 0.015f);
            renderer->drawText(arr_lx, sel_y, "<", 0.014f, 1,1,1, hover_tgt_l ? 1.0f : 0.5f, true, Renderer::CENTER);
            if (hover_tgt_l && hlmb && !hlmb_prev) {
                int origin = TransferCalculator::getTransferOriginBody();
                do { transfer_target_body--; if (transfer_target_body < 1) transfer_target_body = (int)SOLAR_SYSTEM.size()-1; }
                while (transfer_target_body == origin || transfer_target_body == 4 || transfer_target_body == 0);
                transfer_result_valid = false;
            }

            // Body name
            if (transfer_target_body >= 0 && transfer_target_body < (int)SOLAR_SYSTEM.size())
                renderer->drawText(tw_x + 0.06f, sel_y, SOLAR_SYSTEM[transfer_target_body].name.c_str(), 0.012f, 1.0f, 0.9f, 0.3f, 1.0f, true, Renderer::CENTER);

            // Right arrow
            float arr_rx = tw_x + 0.14f;
            bool hover_tgt_r = (hmouse_x >= arr_rx - 0.015f && hmouse_x <= arr_rx + 0.015f && hmouse_y >= sel_y - 0.015f && hmouse_y <= sel_y + 0.015f);
            renderer->drawText(arr_rx, sel_y, ">", 0.014f, 1,1,1, hover_tgt_r ? 1.0f : 0.5f, true, Renderer::CENTER);
            if (hover_tgt_r && hlmb && !hlmb_prev) {
                int origin = TransferCalculator::getTransferOriginBody();
                do { transfer_target_body++; if (transfer_target_body >= (int)SOLAR_SYSTEM.size()) transfer_target_body = 1; }
                while (transfer_target_body == origin || transfer_target_body == 4 || transfer_target_body == 0);
                transfer_result_valid = false;
            }

            // --- CALCULATE Button ---
            float calc_y = sel_y - 0.045f;
            float calc_w = tw_w * 0.45f;
            float calc_h = 0.035f;
            bool hover_calc = (hmouse_x >= tw_x - calc_w/2 && hmouse_x <= tw_x + calc_w/2 && hmouse_y >= calc_y - calc_h/2 && hmouse_y <= calc_y + calc_h/2);
            renderer->addRect(tw_x, calc_y, calc_w, calc_h, hover_calc ? 0.2f : 0.1f, hover_calc ? 0.6f : 0.4f, hover_calc ? 1.0f : 0.8f, 0.9f);
            renderer->addRectOutline(tw_x, calc_y, calc_w, calc_h, 0.3f, 0.7f, 1.0f, 0.8f);
            renderer->drawText(tw_x, calc_y, "CALCULATE", 0.012f, 1,1,1, 1.0f, true, Renderer::CENTER);

            if (hover_calc && hlmb && !hlmb_prev) {
                int origin = TransferCalculator::getTransferOriginBody();
                transfer_result = TransferCalculator::computePorkchop(origin, transfer_target_body, rocket_state.sim_time, 40);
                transfer_result_valid = transfer_result.computed;
                transfer_hover_dep = -1;
                transfer_hover_tof = -1;
            }

            // --- Porkchop Plot ---
            if (transfer_result_valid && transfer_result.computed) {
                float plot_lx = tw_x - tw_w * 0.38f;  // left edge
                float plot_rx = tw_x + tw_w * 0.38f;  // right edge
                float plot_ty = calc_y - 0.04f;        // top (below calc button)
                float plot_by = tw_y - tw_h/2 + 0.08f; // bottom (above info area)
                float plot_w = plot_rx - plot_lx;
                float plot_h = plot_ty - plot_by;

                // Axis labels
                renderer->drawText(tw_x, plot_ty + 0.015f, "Departure (days from now) ->", 0.008f, 0.6f, 0.6f, 0.6f, 0.8f, true, Renderer::CENTER);
                // Y-axis label (rotated text not available, use short label)
                renderer->drawText(plot_lx - 0.02f, (plot_ty + plot_by)/2, "TOF", 0.008f, 0.6f, 0.6f, 0.6f, 0.8f, true, Renderer::CENTER);

                // Compute Δv range for color mapping
                double dv_min_plot = transfer_result.min_dv;
                double dv_max_plot = dv_min_plot * 5.0; // clamp upper range
                if (dv_max_plot < dv_min_plot + 1000.0) dv_max_plot = dv_min_plot + 1000.0;

                int gn = transfer_result.n_dep;
                float cell_w = plot_w / gn;
                float cell_h = plot_h / gn;

                transfer_hover_dep = -1;
                transfer_hover_tof = -1;

                for (int gi = 0; gi < gn; gi++) {
                    for (int gj = 0; gj < gn; gj++) {
                        int idx = gi * gn + gj;
                        const PorkchopPoint& pt = transfer_result.grid[idx];

                        float cx = plot_lx + (gi + 0.5f) * cell_w;
                        float cy = plot_by + (gj + 0.5f) * cell_h;

                        // Check hover
                        bool cell_hover = (hmouse_x >= cx - cell_w/2 && hmouse_x <= cx + cell_w/2 &&
                                          hmouse_y >= cy - cell_h/2 && hmouse_y <= cy + cell_h/2);
                        if (cell_hover) { transfer_hover_dep = gi; transfer_hover_tof = gj; }

                        if (!pt.valid) {
                            renderer->addRect(cx, cy, cell_w * 0.95f, cell_h * 0.95f, 0.08f, 0.08f, 0.08f, 0.6f);
                            continue;
                        }

                        // Color mapping: green (low Δv) -> yellow -> red (high Δv)
                        float t = (float)((pt.dv_total - dv_min_plot) / (dv_max_plot - dv_min_plot));
                        t = fmaxf(0.0f, fminf(1.0f, t));

                        float cr, cg, cb;
                        if (t < 0.5f) {
                            float s = t * 2.0f;
                            cr = s; cg = 1.0f; cb = 0.0f; // green -> yellow
                        } else {
                            float s = (t - 0.5f) * 2.0f;
                            cr = 1.0f; cg = 1.0f - s; cb = 0.0f; // yellow -> red
                        }

                        float alpha = cell_hover ? 1.0f : 0.85f;
                        renderer->addRect(cx, cy, cell_w * 0.95f, cell_h * 0.95f, cr, cg, cb, alpha);
                    }
                }

                // Mark minimum Δv cell with white crosshair
                if (transfer_result.min_dv_index >= 0) {
                    int mi = transfer_result.min_dv_index / gn;
                    int mj = transfer_result.min_dv_index % gn;
                    float mx = plot_lx + (mi + 0.5f) * cell_w;
                    float my = plot_by + (mj + 0.5f) * cell_h;
                    renderer->addRectOutline(mx, my, cell_w * 1.3f, cell_h * 1.3f, 1.0f, 1.0f, 1.0f, 1.0f, 0.003f);
                    renderer->addLine(mx - cell_w, my, mx + cell_w, my, 0.002f, 1.0f, 1.0f, 1.0f, 0.8f);
                    renderer->addLine(mx, my - cell_h, mx, my + cell_h, 0.002f, 1.0f, 1.0f, 1.0f, 0.8f);
                }

                // Axis tick labels (departure days)
                for (int ti = 0; ti <= 4; ti++) {
                    float fx = (float)ti / 4.0f;
                    double dep_day = (transfer_result.dep_start + fx * (transfer_result.dep_end - transfer_result.dep_start) - rocket_state.sim_time) / 86400.0;
                    char tick[32]; snprintf(tick, sizeof(tick), "%.0f", dep_day);
                    renderer->drawText(plot_lx + fx * plot_w, plot_by - 0.015f, tick, 0.007f, 0.5f, 0.5f, 0.5f, 0.8f, true, Renderer::CENTER);
                }
                // Axis tick labels (TOF days)
                for (int ti = 0; ti <= 4; ti++) {
                    float fy = (float)ti / 4.0f;
                    double tof_day = (transfer_result.tof_min + fy * (transfer_result.tof_max - transfer_result.tof_min)) / 86400.0;
                    char tick[32]; snprintf(tick, sizeof(tick), "%.0fd", tof_day);
                    renderer->drawText(plot_lx - 0.03f, plot_by + fy * plot_h, tick, 0.007f, 0.5f, 0.5f, 0.5f, 0.8f, true, Renderer::CENTER);
                }

                // --- Info Display ---
                float info_y = plot_by - 0.035f;
                float info_lx = tw_x - tw_w * 0.4f;

                // Show minimum Δv info
                if (transfer_result.min_dv_index >= 0) {
                    const PorkchopPoint& best = transfer_result.grid[transfer_result.min_dv_index];
                    char buf[128];
                    snprintf(buf, sizeof(buf), "MIN Dv: %.2f km/s  (Dep: %.1f  Arr: %.1f)", 
                             best.dv_total / 1000.0, best.dv_departure / 1000.0, best.dv_arrival / 1000.0);
                    renderer->drawText(info_lx, info_y, buf, 0.009f, 0.2f, 1.0f, 0.4f, 1.0f, true, Renderer::LEFT);

                    double dep_days = (best.departure_time - rocket_state.sim_time) / 86400.0;
                    double tof_days = best.tof / 86400.0;
                    snprintf(buf, sizeof(buf), "Depart: T+%.1f days | Travel: %.1f days", dep_days, tof_days);
                    renderer->drawText(info_lx, info_y - 0.018f, buf, 0.008f, 0.7f, 0.7f, 0.7f, 0.9f, true, Renderer::LEFT);
                }

                // Hover tooltip
                if (transfer_hover_dep >= 0 && transfer_hover_tof >= 0) {
                    int hidx = transfer_hover_dep * gn + transfer_hover_tof;
                    if (hidx >= 0 && hidx < (int)transfer_result.grid.size()) {
                        const PorkchopPoint& hpt = transfer_result.grid[hidx];
                        if (hpt.valid) {
                            float tt_x = hmouse_x + 0.05f;
                            float tt_y = hmouse_y + 0.03f;
                            float tt_w = 0.22f;
                            float tt_h = 0.06f;
                            renderer->addRect(tt_x, tt_y, tt_w, tt_h, 0.05f, 0.05f, 0.1f, 0.95f);
                            renderer->addRectOutline(tt_x, tt_y, tt_w, tt_h, 0.4f, 0.7f, 1.0f, 0.9f);
                            char tb[64];
                            snprintf(tb, sizeof(tb), "Dv: %.2f km/s", hpt.dv_total / 1000.0);
                            renderer->drawText(tt_x, tt_y + 0.012f, tb, 0.009f, 1,1,1, 1.0f, true, Renderer::CENTER);
                            double d_day = (hpt.departure_time - rocket_state.sim_time) / 86400.0;
                            double t_day = hpt.tof / 86400.0;
                            snprintf(tb, sizeof(tb), "T+%.0fd / %.0fd flight", d_day, t_day);
                            renderer->drawText(tt_x, tt_y - 0.012f, tb, 0.008f, 0.7f, 0.7f, 0.7f, 1.0f, true, Renderer::CENTER);
                        }
                    }
                }

                // --- CREATE MANEUVER NODE Button ---
                float cmn_y = tw_y - tw_h/2 + 0.025f;
                float cmn_w = tw_w * 0.55f;
                float cmn_h = 0.035f;
                bool hover_cmn = (transfer_result.min_dv_index >= 0) &&
                    (hmouse_x >= tw_x - cmn_w/2 && hmouse_x <= tw_x + cmn_w/2 &&
                     hmouse_y >= cmn_y - cmn_h/2 && hmouse_y <= cmn_y + cmn_h/2);
                float cmn_r = transfer_result.min_dv_index >= 0 ? 0.2f : 0.15f;
                float cmn_g = transfer_result.min_dv_index >= 0 ? 0.8f : 0.3f;
                float cmn_b = transfer_result.min_dv_index >= 0 ? 0.4f : 0.3f;
                if (hover_cmn) { cmn_r += 0.1f; cmn_g += 0.1f; cmn_b += 0.1f; }
                renderer->addRect(tw_x, cmn_y, cmn_w, cmn_h, cmn_r * 0.3f, cmn_g * 0.3f, cmn_b * 0.3f, 0.8f);
                renderer->addRectOutline(tw_x, cmn_y, cmn_w, cmn_h, cmn_r, cmn_g, cmn_b, 0.9f);
                renderer->drawText(tw_x, cmn_y, "CREATE MANEUVER NODE", 0.010f, cmn_r, cmn_g, cmn_b, 1.0f, true, Renderer::CENTER);

                if (hover_cmn && hlmb && !hlmb_prev && transfer_result.min_dv_index >= 0) {
                    const PorkchopPoint& best = transfer_result.grid[transfer_result.min_dv_index];

                    // Create maneuver node at optimal departure time
                    ManeuverNode node;
                    node.sim_time = best.departure_time;
                    node.active = true;
                    node.ref_body = current_soi_index;

                    // Convert heliocentric Δv to prograde/normal/radial frame
                    // Get rocket state projected to departure time
                    double mu_soi = G_const * SOLAR_SYSTEM[current_soi_index].mass;
                    double npx, npy, npz, nvx, nvy, nvz;
                    get3DStateAtTime(rocket_state.px, rocket_state.py, rocket_state.pz,
                                    rocket_state.vx, rocket_state.vy, rocket_state.vz,
                                    mu_soi, best.departure_time - rocket_state.sim_time,
                                    npx, npy, npz, nvx, nvy, nvz);

                    ManeuverFrame frame = ManeuverSystem::getFrame(
                        Vec3((float)npx, (float)npy, (float)npz),
                        Vec3((float)nvx, (float)nvy, (float)nvz));

                    // The departure Δv vector is in heliocentric frame.
                    // For the maneuver node, we need it in the local orbital frame relative to SOI body.
                    // Approximate: project the heliocentric dv into prograde/normal/radial
                    Vec3 dv_world = best.departure_dv_vec;
                    float pro_comp = dv_world.dot(frame.prograde);
                    float nrm_comp = dv_world.dot(frame.normal);
                    float rad_comp = dv_world.dot(frame.radial);
                    node.delta_v = Vec3(pro_comp, nrm_comp, rad_comp);

                    rocket_state.maneuvers.clear();
                    rocket_state.maneuvers.push_back(node);
                    rocket_state.selected_maneuver_index = 0;
                    global_best_ang = 0;
                    mnv_popup_index = 0;
                    mnv_popup_visible = true;
                    adv_embed_mnv = true;

                    rocket_state.mission_msg = "TRANSFER NODE CREATED";
                    cout << ">> Transfer maneuver created: dv=" << best.dv_total/1000.0 << " km/s" << endl;
                }
            } else {
                // No result yet - show instructions
                renderer->drawText(tw_x, tw_y - 0.03f, "Select target planet and press CALCULATE", 0.009f, 0.5f, 0.5f, 0.5f, 0.8f, true, Renderer::CENTER);
                renderer->drawText(tw_x, tw_y - 0.06f, "to generate porkchop plot.", 0.009f, 0.5f, 0.5f, 0.5f, 0.8f, true, Renderer::CENTER);

                // Origin info
                int origin = TransferCalculator::getTransferOriginBody();
                char orig_buf[64];
                snprintf(orig_buf, sizeof(orig_buf), "Origin: %s", SOLAR_SYSTEM[origin].name.c_str());
                renderer->drawText(tw_x, tw_y - 0.12f, orig_buf, 0.010f, 0.4f, 0.7f, 0.9f, 0.8f, true, Renderer::CENTER);
            }
        }
        if (adv_orbit_menu) {
            float menu_w = 0.30f;
            float menu_h = 0.58f;
            float menu_x = adv_btn_x - adv_btn_w/2 - menu_w/2 - 0.02f;
            float menu_y = adv_btn_y - 0.10f;
            renderer->addRect(menu_x, menu_y, menu_w, menu_h, 0.05f, 0.05f, 0.1f, 0.85f);
            renderer->addRectOutline(menu_x, menu_y, menu_w, menu_h, 0.4f, 0.6f, 1.0f, 0.8f);
            
            float tog_y = menu_y + menu_h/2 - 0.05f;
            bool hover_tog = (hmouse_x >= menu_x - menu_w/2 && hmouse_x <= menu_x + menu_w/2 && hmouse_y >= tog_y - 0.02f && hmouse_y <= tog_y + 0.02f);
            if (hover_tog && hlmb && !hlmb_prev) adv_orbit_enabled = !adv_orbit_enabled;
            renderer->drawText(menu_x - 0.12f, tog_y, "Mode:", 0.012f, 1, 1, 1, 1.0f);
            renderer->drawText(menu_x + 0.02f, tog_y, adv_orbit_enabled ? "SYM-LMM4 (TRANS-DT)" : "KEPLER (SOI)", 0.012f, adv_orbit_enabled?1:0.6f, adv_orbit_enabled?0.5f:1, 0.4f, 1.0f);

            // Frame Type Switch (Inertial vs Co-rotating vs Surface)
            float fr_y = tog_y - 0.06f;
            renderer->drawText(menu_x - 0.12f, fr_y, "Frame:", 0.012f, 1, 1, 1, 1.0f);
            bool hover_fr_l = (hmouse_x >= menu_x - 0.01f && hmouse_x <= menu_x + 0.03f && hmouse_y >= fr_y - 0.02f && hmouse_y <= fr_y + 0.02f);
            bool hover_fr_r = (hmouse_x >= menu_x + 0.11f && hmouse_x <= menu_x + 0.15f && hmouse_y >= fr_y - 0.02f && hmouse_y <= fr_y + 0.02f);
            if (hover_fr_l && hlmb && !hlmb_prev) adv_orbit_ref_mode = (adv_orbit_ref_mode+2)%3;
            if (hover_fr_r && hlmb && !hlmb_prev) adv_orbit_ref_mode = (adv_orbit_ref_mode+1)%3;
            renderer->drawText(menu_x + 0.01f, fr_y, "<", 0.012f, 1,1,1, hover_fr_l?1.0f:0.5f, true, Renderer::CENTER);
            renderer->drawText(menu_x + 0.07f, fr_y, adv_orbit_ref_mode==0 ? "INERTIAL" : (adv_orbit_ref_mode==1 ? "CO-ROT" : "SURFACE"), 0.010f, 1,1,1,1, true, Renderer::CENTER);
            renderer->drawText(menu_x + 0.13f, fr_y, ">", 0.012f, 1,1,1, hover_fr_r?1.0f:0.5f, true, Renderer::CENTER);

            // Ref Body Switch
            float bd_y = fr_y - 0.06f;
            renderer->drawText(menu_x - 0.12f, bd_y, "Primary:", 0.012f, 1, 1, 1, 1.0f);
            bool hover_bd_l = (hmouse_x >= menu_x - 0.01f && hmouse_x <= menu_x + 0.03f && hmouse_y >= bd_y - 0.02f && hmouse_y <= bd_y + 0.02f);
            bool hover_bd_r = (hmouse_x >= menu_x + 0.11f && hmouse_x <= menu_x + 0.15f && hmouse_y >= bd_y - 0.02f && hmouse_y <= bd_y + 0.02f);
            if (hover_bd_l && hlmb && !hlmb_prev) adv_orbit_ref_body = (adv_orbit_ref_body-1 < 0) ? (int)SOLAR_SYSTEM.size()-1 : adv_orbit_ref_body-1;
            if (hover_bd_r && hlmb && !hlmb_prev) adv_orbit_ref_body = (adv_orbit_ref_body+1) % (int)SOLAR_SYSTEM.size();
            renderer->drawText(menu_x + 0.01f, bd_y, "<", 0.012f, 1,1,1, hover_bd_l?1.0f:0.5f, true, Renderer::CENTER);
            if (!SOLAR_SYSTEM.empty()) {
                renderer->drawText(menu_x + 0.07f, bd_y, SOLAR_SYSTEM[adv_orbit_ref_body].name.c_str(), 0.010f, 1,1,1,1, true, Renderer::CENTER);
            }
            renderer->drawText(menu_x + 0.13f, bd_y, ">", 0.012f, 1,1,1, hover_bd_r?1.0f:0.5f, true, Renderer::CENTER);
            
            float next_y = bd_y - 0.06f;
            
            if (adv_orbit_ref_mode == 1) { // Co-rotating needs Secondary Body
                renderer->drawText(menu_x - 0.12f, next_y, "Second:", 0.012f, 1, 1, 1, 1.0f);
                bool hover_sbd_l = (hmouse_x >= menu_x - 0.01f && hmouse_x <= menu_x + 0.03f && hmouse_y >= next_y - 0.02f && hmouse_y <= next_y + 0.02f);
                bool hover_sbd_r = (hmouse_x >= menu_x + 0.11f && hmouse_x <= menu_x + 0.15f && hmouse_y >= next_y - 0.02f && hmouse_y <= next_y + 0.02f);
                if (hover_sbd_l && hlmb && !hlmb_prev) adv_orbit_secondary_ref_body = (adv_orbit_secondary_ref_body-1 < 0) ? (int)SOLAR_SYSTEM.size()-1 : adv_orbit_secondary_ref_body-1;
                if (hover_sbd_r && hlmb && !hlmb_prev) adv_orbit_secondary_ref_body = (adv_orbit_secondary_ref_body+1) % (int)SOLAR_SYSTEM.size();
                renderer->drawText(menu_x + 0.01f, next_y, "<", 0.012f, 1,1,1, hover_sbd_l?1.0f:0.5f, true, Renderer::CENTER);
                if (!SOLAR_SYSTEM.empty()) {
                    renderer->drawText(menu_x + 0.07f, next_y, SOLAR_SYSTEM[adv_orbit_secondary_ref_body].name.c_str(), 0.010f, 1,1,1,1, true, Renderer::CENTER);
                }
                renderer->drawText(menu_x + 0.13f, next_y, ">", 0.012f, 1,1,1, hover_sbd_r?1.0f:0.5f, true, Renderer::CENTER);
                next_y -= 0.06f;
            }

            // --- Prediction Time Slider ---
            float pred_y = next_y;
            renderer->drawText(menu_x - 0.12f, pred_y, "Predict:", 0.012f, 1, 1, 1, 1.0f);
            float pred_slider_w = menu_w * 0.50f;
            float pred_slider_x = menu_x + 0.04f;
            float pred_slider_h = 0.012f;
            renderer->addRect(pred_slider_x, pred_y, pred_slider_w, pred_slider_h, 0.15f, 0.15f, 0.2f, 0.8f);
            renderer->addRectOutline(pred_slider_x, pred_y, pred_slider_w, pred_slider_h, 0.3f, 0.6f, 0.9f, 0.6f);
            // Map log scale: 1 day to 3650 days (10 years)
            float pred_log_min = logf(1.0f);
            float pred_log_max = logf(3650.0f);
            float pred_ratio = (logf(adv_orbit_pred_days) - pred_log_min) / (pred_log_max - pred_log_min);
            pred_ratio = fmaxf(0.0f, fminf(1.0f, pred_ratio));
            float pred_thumb_x = pred_slider_x - pred_slider_w/2 + pred_ratio * pred_slider_w;
            renderer->addRect(pred_thumb_x, pred_y, 0.012f, 0.022f, 0.3f, 0.7f, 1.0f, 0.95f);
            
            // Drag logic for prediction slider
            static bool pred_slider_dragging = false;
            if (hlmb && !hlmb_prev && hmouse_x >= pred_slider_x - pred_slider_w/2 - 0.01f && hmouse_x <= pred_slider_x + pred_slider_w/2 + 0.01f && hmouse_y >= pred_y - 0.02f && hmouse_y <= pred_y + 0.02f) {
                pred_slider_dragging = true;
            }
            if (!hlmb) pred_slider_dragging = false;
            if (pred_slider_dragging) {
                float r = (hmouse_x - (pred_slider_x - pred_slider_w/2)) / pred_slider_w;
                r = fmaxf(0.0f, fminf(1.0f, r));
                adv_orbit_pred_days = expf(pred_log_min + r * (pred_log_max - pred_log_min));
            }
            char pred_str[64];
            if (adv_orbit_pred_days < 1.5f) snprintf(pred_str, sizeof(pred_str), "%.0f Day", adv_orbit_pred_days);
            else if (adv_orbit_pred_days < 365.0f) snprintf(pred_str, sizeof(pred_str), "%.0f Days", adv_orbit_pred_days);
            else snprintf(pred_str, sizeof(pred_str), "%.1f Yrs", adv_orbit_pred_days / 365.25f);
            renderer->drawText(pred_slider_x + pred_slider_w/2 + 0.015f, pred_y, pred_str, 0.009f, 0.8f, 0.8f, 0.8f, 1.0f, true, Renderer::LEFT);

            // --- Iteration Count Switch ---
            float iter_y = pred_y - 0.05f;
            renderer->drawText(menu_x - 0.12f, iter_y, "Iters:", 0.012f, 1, 1, 1, 1.0f);
            bool hover_it_l = (hmouse_x >= menu_x - 0.01f && hmouse_x <= menu_x + 0.03f && hmouse_y >= iter_y - 0.02f && hmouse_y <= iter_y + 0.02f);
            bool hover_it_r = (hmouse_x >= menu_x + 0.11f && hmouse_x <= menu_x + 0.15f && hmouse_y >= iter_y - 0.02f && hmouse_y <= iter_y + 0.02f);
            int iter_opts[] = {500, 1000, 2000, 4000, 8000, 16000, 32000};
            int cur_it_idx = 3; // default 4000
            for (int j=0; j<7; j++) if(adv_orbit_iters == iter_opts[j]) cur_it_idx = j;
            if (hover_it_l && hlmb && !hlmb_prev && cur_it_idx > 0) adv_orbit_iters = iter_opts[cur_it_idx-1];
            if (hover_it_r && hlmb && !hlmb_prev && cur_it_idx < 6) adv_orbit_iters = iter_opts[cur_it_idx+1];
            renderer->drawText(menu_x + 0.01f, iter_y, "<", 0.012f, 1,1,1, hover_it_l?1.0f:0.5f, true, Renderer::CENTER);
            char it_str[32]; snprintf(it_str, sizeof(it_str), "%d", adv_orbit_iters);
            renderer->drawText(menu_x + 0.07f, iter_y, it_str, 0.010f, 1,1,1,1, true, Renderer::CENTER);
            renderer->drawText(menu_x + 0.13f, iter_y, ">", 0.012f, 1,1,1, hover_it_r?1.0f:0.5f, true, Renderer::CENTER);

            // Computed Step Info
            float info_y = iter_y - 0.04f;
            char step_info[64];
            snprintf(step_info, sizeof(step_info), "Step: Adaptive (Sym-4)");
            renderer->drawText(menu_x - 0.12f, info_y, step_info, 0.009f, 0.6f, 0.8f, 0.6f, 0.8f);
            
            // Separator
            renderer->addRect(menu_x, info_y - 0.025f, menu_w - 0.04f, 0.002f, 0.3f, 0.5f, 0.7f, 0.5f);

            // Generate Maneuver Node Button
            float cr_btn_y = info_y - 0.05f;
            bool hover_cr = (hmouse_x >= menu_x - menu_w/2 + 0.02f && hmouse_x <= menu_x + menu_w/2 - 0.02f && hmouse_y >= cr_btn_y - 0.015f && hmouse_y <= cr_btn_y + 0.015f);
            renderer->addRectOutline(menu_x, cr_btn_y, menu_w - 0.04f, 0.03f, 0.4f, 0.8f, 0.4f, 0.8f);
            renderer->drawText(menu_x, cr_btn_y, "CREATE MANEUVER NODE", 0.010f, 0.4f, 0.8f, 0.4f, 1.0f, true, Renderer::CENTER);
            
            if (hover_cr && hlmb && !hlmb_prev) {
                ManeuverNode node;
                node.sim_time = rocket_state.sim_time + 600.0; // 10 minutes ahead
                node.delta_v = Vec3(0, 0, 0);
                node.active = true;
                node.ref_body = current_soi_index;
                rocket_state.maneuvers.clear();
                rocket_state.maneuvers.push_back(node);
                rocket_state.selected_maneuver_index = 0;
                global_best_ang = 0; // Disable keplerian hit-testing state
                mnv_popup_index = 0;
                mnv_popup_visible = true;
                adv_embed_mnv = true;
            }
            
            // Warp to Maneuver Node Button
            float warp_btn_y = cr_btn_y - 0.05f;
            bool has_mnv_btn = !rocket_state.maneuvers.empty();
            float warp_r = has_mnv_btn ? 1.0f : 0.4f, warp_g = has_mnv_btn ? 0.7f : 0.4f, warp_b = has_mnv_btn ? 0.2f : 0.4f;
            bool hover_warp = has_mnv_btn && (hmouse_x >= menu_x - menu_w/2 + 0.02f && hmouse_x <= menu_x + menu_w/2 - 0.02f && hmouse_y >= warp_btn_y - 0.015f && hmouse_y <= warp_btn_y + 0.015f);
            renderer->addRectOutline(menu_x, warp_btn_y, menu_w - 0.04f, 0.03f, warp_r, warp_g, warp_b, has_mnv_btn ? 0.8f : 0.3f);
            if (adv_warp_to_node) {
                renderer->addRect(menu_x, warp_btn_y, menu_w - 0.04f, 0.03f, 1.0f, 0.5f, 0.1f, 0.4f);
                renderer->drawText(menu_x, warp_btn_y, "WARPING...", 0.010f, 1.0f, 0.8f, 0.2f, 1.0f, true, Renderer::CENTER);
            } else {
                renderer->drawText(menu_x, warp_btn_y, "WARP TO NODE", 0.010f, warp_r, warp_g, warp_b, has_mnv_btn ? 1.0f : 0.4f, true, Renderer::CENTER);
            }
            
            if (hover_warp && hlmb && !hlmb_prev && has_mnv_btn) {
                adv_warp_to_node = !adv_warp_to_node; // Toggle
            }
            
            // Process warp-to-node: set time_warp to max safe level until near the node
            if (adv_warp_to_node && has_mnv_btn) {
                double target_t = rocket_state.maneuvers[0].sim_time;
                double time_to_start = target_t - rocket_state.sim_time;
                if (time_to_start <= 60.0) {
                    // Arrived at start window! Stop warping
                    time_warp = 1;
                    adv_warp_to_node = false;
                } else if (time_to_start > 86400.0 * 10) {
                    time_warp = 10000000; // 10M
                } else if (time_to_start > 86400.0) {
                    time_warp = 1000000; // 1M
                } else if (time_to_start > 3600.0 * 4) {
                    time_warp = 100000; // 100K
                } else if (time_to_start > 3600.0) {
                    time_warp = 10000; // 10K
                } else if (time_to_start > 600.0) {
                    time_warp = 1000;
                } else if (time_to_start > 120.0) {
                    time_warp = 100;
                } else {
                    time_warp = 10;
                }
            }
            
            // Toggle embedded Maneuver Popup
            float fold_btn_y = warp_btn_y - 0.05f;
            if (has_mnv_btn) {
                bool hover_fold = (hmouse_x >= menu_x - menu_w/2 + 0.02f && hmouse_x <= menu_x + menu_w/2 - 0.02f && hmouse_y >= fold_btn_y - 0.015f && hmouse_y <= fold_btn_y + 0.015f);
                renderer->addRectOutline(menu_x, fold_btn_y, menu_w - 0.04f, 0.03f, 0.4f, 0.8f, 1.0f, 0.8f);
                if (hover_fold) renderer->addRect(menu_x, fold_btn_y, menu_w-0.04f, 0.03f, 0.2f, 0.4f, 0.6f, 0.4f);
                renderer->drawText(menu_x, fold_btn_y, adv_embed_mnv ? "HIDE MANEUVER CONTROLS [v]" : "SHOW MANEUVER CONTROLS [>]", 0.009f, 0.6f,0.9f,1.0f,1.0f, true, Renderer::CENTER);
                if (hover_fold && hlmb && !hlmb_prev) {
                    adv_embed_mnv = !adv_embed_mnv;
                    if (adv_embed_mnv) { mnv_popup_index = 0; mnv_popup_visible = true; }
                }
                
                if (adv_embed_mnv) {
                   float target_top = adv_btn_y + 0.19f;
                   float adv_menu_bottom = target_top - 0.58f;
                   
                   mnv_popup_px = menu_x;
                   mnv_popup_pw = menu_w + 0.02f;
                   mnv_popup_ph = adv_embed_mnv_mini ? 0.12f : 0.40f;
                   mnv_popup_py = adv_menu_bottom - mnv_popup_ph / 2 - 0.005f;
                   mnv_popup_visible = true;
                   mnv_popup_index = 0;
                }
            }
        }
    }
    
    // === Advanced Orbit Apsides Rendering (2D HUD) ===
    if (adv_orbit_enabled) {
        float as_ratio = (float)ww / wh;
        double mx_raw, my_raw; glfwGetCursorPos(window, &mx_raw, &my_raw);
        float mouse_x = (float)(mx_raw / ww * 2.0 - 1.0);
        float mouse_y = (float)(1.0 - my_raw / wh * 2.0);

        std::vector<RocketState::Apsis> hud_apsides, hud_mnv_apsides;
        {
            std::lock_guard<std::mutex> lock(*rocket_state.path_mutex);
            hud_apsides = rocket_state.predicted_apsides;
            hud_mnv_apsides = rocket_state.predicted_mnv_apsides;
        }

        auto draw_apsis_hud = [&](const std::vector<RocketState::Apsis>& list, bool is_mnv) {
            double rb_px, rb_py, rb_pz;
            PhysicsSystem::GetCelestialPositionAt(adv_orbit_ref_body, rocket_state.sim_time, rb_px, rb_py, rb_pz);
            Quat q_inv = PhysicsSystem::GetFrameRotation(adv_orbit_ref_mode, adv_orbit_ref_body, adv_orbit_secondary_ref_body, rocket_state.sim_time);
            
            for (const auto& ap : list) {
                Vec3 p_rot = q_inv.rotate(ap.local_pos);
                Vec3 w_pos(
                    (float)((rb_px + p_rot.x) * ws_d - ro_x),
                    (float)((rb_py + p_rot.y) * ws_d - ro_y),
                    (float)((rb_pz + p_rot.z) * ws_d - ro_z)
                );
                
                Vec2 scr = ManeuverSystem::projectToScreen(w_pos, viewMat, macroProjMat, as_ratio);
                if (scr.x < -1.5f || scr.x > 1.5f || scr.y < -1.5f || scr.y > 1.5f) continue;

                // Draw Triangle
                float tri_w = 0.02f;
                float tri_h = 0.025f;
                float r = 1.0f, g = 0.8f, b = 0.1f; // Yellow
                if (is_mnv) { r = 1.0f; g = 0.6f; b = 0.1f; } // Orange for mnv
                
                if (ap.is_apoapsis) {
                    renderer->addRotatedTri(scr.x, scr.y + tri_h/2.0f, tri_w, tri_h, 0.0f, r, g, b, 0.9f);
                    renderer->drawText(scr.x, scr.y - 0.015f, "Ap", 0.012f, r, g, b, 0.9f, true, Renderer::CENTER);
                } else {
                    renderer->addRotatedTri(scr.x, scr.y - tri_h/2.0f, tri_w, tri_h, 3.14159265f, r, g, b, 0.9f);
                    renderer->drawText(scr.x, scr.y + 0.015f, "Pe", 0.012f, r, g, b, 0.9f, true, Renderer::CENTER);
                }

                // Hover Tooltip
                float d_mouse = sqrtf(powf(scr.x - mouse_x, 2) + powf(scr.y - mouse_y, 2));
                if (d_mouse < 0.035f) {
                    char tooltip[128];
                    char tooltip2[128];
                    double dt_val = ap.sim_time - rocket_state.sim_time;
                    int t_hr = (int)(std::fabs(dt_val) / 3600.0);
                    int t_min = (int)fmod(std::fabs(dt_val) / 60.0, 60.0);
                    int t_sec = (int)fmod(std::fabs(dt_val), 60.0);
                    
                    double alt_val = ap.altitude;
                    const char* unit = "m";
                    if (alt_val > 1000000.0) { alt_val /= 1000000.0; unit = "Mm"; }
                    else if (alt_val > 10000.0) { alt_val /= 1000.0; unit = "km"; }

                    snprintf(tooltip, sizeof(tooltip), "T%c %02dH %02dM %02dS", dt_val >= 0 ? '+' : '-', t_hr, t_min, t_sec);
                    snprintf(tooltip2, sizeof(tooltip2), "Alt: %.1f %s", alt_val, unit);
                        
                    float tt_x = scr.x;
                    float tt_y = scr.y + (ap.is_apoapsis ? -0.055f : 0.055f);
                    
                    float tw = 0.32f;
                    float th = 0.07f;
                    // Keep tooltip within screen
                    if (tt_x + tw/2 > 0.98f) tt_x = 0.98f - tw/2;
                    if (tt_x - tw/2 < -0.98f) tt_x = -0.98f + tw/2;
                    if (tt_y + th/2 > 0.98f) tt_y = 0.98f - th/2;
                    if (tt_y - th/2 < -0.98f) tt_y = -0.98f + th/2;

                    renderer->addRectOutline(tt_x, tt_y, tw, th, r, g, b, 0.9f);
                    renderer->addRect(tt_x, tt_y, tw, th, 0.05f, 0.05f, 0.08f, 0.95f);
                    renderer->drawText(tt_x, tt_y + 0.012f, tooltip, 0.012f, 1.0f, 1.0f, 1.0f, 1.0f, true, Renderer::CENTER);
                    renderer->drawText(tt_x, tt_y - 0.015f, tooltip2, 0.011f, 0.8f, 0.8f, 0.8f, 1.0f, true, Renderer::CENTER);
                }
            }
        };

        draw_apsis_hud(hud_apsides, false);
        draw_apsis_hud(hud_mnv_apsides, true);
    }

    hlmb_prev = hlmb;

    // ========================================================================
    } // end if (show_hud)
    
    // ===== Deferred Maneuver Node Popup Rendering (2D HUD pass) =====
    if (mnv_popup_visible) {
        float pop_x = mnv_popup_px, pop_y = mnv_popup_py;
        float pw = mnv_popup_pw, ph = mnv_popup_ph;
        
        // Connector line from popup to node icon
        if (!adv_embed_mnv || !adv_orbit_menu) {
            renderer->addLine(pop_x - pw/2, pop_y, mnv_popup_node_scr_x, mnv_popup_node_scr_y, 0.002f, 0.4f, 0.6f, 1.0f, 0.6f);
        }
        
        // Background panel with subtle gradient effect (two overlapping rects)
        renderer->addRect(pop_x, pop_y, pw, ph, 0.06f, 0.06f, 0.12f, 0.95f);
        renderer->addRect(pop_x, pop_y + ph * 0.35f, pw, ph * 0.3f, 0.08f, 0.08f, 0.18f, 0.3f); // Subtle highlight band
        renderer->addRectOutline(pop_x, pop_y, pw, ph, 0.3f, 0.5f, 1.0f, 0.8f);
        
        // Close / Mini button (top-right corner)
        float close_size = 0.028f;
        float close_x = pop_x + pw/2 - close_size/2 - 0.008f;
        float close_y = pop_y + ph/2 - close_size/2 - 0.008f;
        if (!adv_embed_mnv) {
            renderer->addRect(close_x, close_y, close_size, close_size, 
                              mnv_popup_close_hover ? 0.8f : 0.3f, 0.15f, 0.15f, 0.9f);
            renderer->drawText(close_x, close_y, "X", 0.014f, 1.0f, 1.0f, 1.0f, 1.0f, false, Renderer::CENTER);
        } else {
            renderer->addRect(close_x, close_y, close_size, close_size, 
                              mnv_popup_mini_hover ? 0.5f : 0.2f, mnv_popup_mini_hover ? 0.6f : 0.3f, 0.8f, 0.9f);
            renderer->drawText(close_x, close_y, adv_embed_mnv_mini ? "+" : "-", 0.016f, 1.0f, 1.0f, 1.0f, 1.0f, false, Renderer::CENTER);
        }
        
        if (adv_embed_mnv_mini) {
            // Render ONLY Time and Burn time for mini dock
            float info_y = pop_y + 0.01f;
            float info_lx = pop_x - pw/2 + 0.03f;
            char buf[64];
            int t_min = (int)(mnv_popup_time_to_node / 60.0);
            int t_sec = (int)fmod(abs((int)mnv_popup_time_to_node), 60.0);
            
            if (mnv_popup_time_to_node > 3600) {
                int t_hr = (int)(mnv_popup_time_to_node / 3600.0);
                t_min = (int)fmod(mnv_popup_time_to_node / 60.0, 60.0);
                snprintf(buf, sizeof(buf), "T-IGN: %dH %02dM %02dS", t_hr, t_min, t_sec);
            } else if (mnv_popup_time_to_node >= 0) {
                snprintf(buf, sizeof(buf), "T-IGN: %dM %02dS", t_min, t_sec);
            } else {
                snprintf(buf, sizeof(buf), "BURN+ %dS", (int)(-mnv_popup_time_to_node));
            }
            float time_ratio = (float)fmin(1.0, fmax(0.0, mnv_popup_time_to_node / 120.0));
            renderer->drawText(info_lx, info_y, buf, 0.010f, 
                              1.0f - time_ratio * 0.7f, 0.3f + time_ratio * 0.7f, 0.2f, 1.0f, true, Renderer::LEFT);
            
            info_y -= 0.025f;
            int b_min = (int)(mnv_popup_burn_time / 60.0);
            int b_sec = (int)fmod(mnv_popup_burn_time, 60.0);
            if (mnv_popup_burn_time >= 9999.0) {
                snprintf(buf, sizeof(buf), "BURN: ---");
            } else if (mnv_popup_burn_time > 60.0) {
                snprintf(buf, sizeof(buf), "BURN: %dM %02dS", b_min, b_sec);
            } else {
                snprintf(buf, sizeof(buf), "BURN: %.1fS", mnv_popup_burn_time);
            }
            renderer->drawText(info_lx, info_y, buf, 0.010f, 0.9f, 0.7f, 0.3f, 1.0f, true, Renderer::LEFT);
        } else {
            // Render full normal UI
            // Title
            float title_y = pop_y + ph/2 - (adv_embed_mnv ? 0.015f : 0.025f);
            renderer->drawText(pop_x, title_y, "MANEUVER NODE", 0.013f, 0.5f, 0.8f, 1.0f, 1.0f, true, Renderer::CENTER);
            
            // Reference body
            char ref_buf[64];
            if (mnv_popup_ref_body >= 0 && mnv_popup_ref_body < (int)SOLAR_SYSTEM.size()) {
                snprintf(ref_buf, sizeof(ref_buf), "REF: %s", SOLAR_SYSTEM[mnv_popup_ref_body].name.c_str());
            } else {
                snprintf(ref_buf, sizeof(ref_buf), "REF: ---");
            }
            renderer->drawText(pop_x, title_y - 0.022f, ref_buf, 0.009f, 0.4f, 0.7f, 0.9f, 0.8f, true, Renderer::CENTER);
            
            // --- Delta-V Sliders ---
            double pop_mx, pop_my; glfwGetCursorPos(window, &pop_mx, &pop_my);
            float mouse_x = (float)(pop_mx / ww * 2.0 - 1.0);
            
            float slider_track_w = pw * 0.65f;
            float slider_track_h = 0.012f;
            float slider_base_y = title_y - (adv_embed_mnv ? 0.04f : 0.06f);
            float slider_spacing = adv_embed_mnv ? 0.050f : 0.065f;
            float slider_cx = pop_x + 0.02f;
        float label_x = pop_x - pw/2 + 0.012f;
        
        const char* slider_labels[] = {"PRO", "NRM", "RAD", "T"};
        float slider_colors[][3] = {{1.0f, 0.9f, 0.1f}, {1.0f, 0.1f, 1.0f}, {0.1f, 0.8f, 1.0f}, {0.1f, 1.0f, 0.5f}};
        float dv_vals[] = {mnv_popup_dv.x, mnv_popup_dv.y, mnv_popup_dv.z, 0.0f}; // T slider snaps back to 0
        
        for (int s = 0; s < 4; s++) {
            float sy = slider_base_y - s * slider_spacing;
            float cr = slider_colors[s][0], cg = slider_colors[s][1], cb = slider_colors[s][2];
            
            // Label
            renderer->drawText(label_x, sy + 0.015f, slider_labels[s], 0.009f, cr, cg, cb, 0.9f, true, Renderer::LEFT);
            
            // Value display
            char val_buf[32];
            if (s < 3) {
                snprintf(val_buf, sizeof(val_buf), "%.1f", dv_vals[s]);
                renderer->drawText(label_x, sy - 0.01f, val_buf, 0.009f, 1.0f, 1.0f, 1.0f, 0.9f, true, Renderer::LEFT);
                renderer->drawText(label_x + 0.065f, sy - 0.01f, "M/S", 0.007f, 0.5f, 0.5f, 0.6f, 0.7f, false, Renderer::LEFT);
            } else {
                snprintf(val_buf, sizeof(val_buf), "SHIFT");
                renderer->drawText(label_x, sy - 0.01f, val_buf, 0.009f, 0.6f, 1.0f, 0.6f, 0.9f, true, Renderer::LEFT);
            }
            
            // Track background
            renderer->addRect(slider_cx, sy, slider_track_w, slider_track_h, 0.15f, 0.15f, 0.2f, 0.8f);
            renderer->addRectOutline(slider_cx, sy, slider_track_w, slider_track_h, cr * 0.4f, cg * 0.4f, cb * 0.4f, 0.6f);
            
            // Center line (zero marker)
            renderer->addRect(slider_cx, sy, 0.003f, slider_track_h * 1.5f, 0.5f, 0.5f, 0.5f, 0.7f);
            
            // Fill bar showing current value (clamped to track width)
            float fill_ratio = 0.0f;
            if (s < 3) {
                float max_display_dv = 500.0f; // Max dv for full track
                fill_ratio = dv_vals[s] / max_display_dv;
                fill_ratio = fmaxf(-1.0f, fminf(1.0f, fill_ratio));
            } else if (mnv_popup_slider_dragging == s) {
                // visual offset for time slider when dragging
                float raw_drag = (mouse_x - mnv_popup_slider_drag_x) / (slider_track_w / 2.0f);
                fill_ratio = fmaxf(-1.0f, fminf(1.0f, raw_drag));
            }
            
            float fill_w = fabsf(fill_ratio) * slider_track_w / 2.0f;
            float fill_cx = slider_cx + (fill_ratio >= 0 ? fill_w/2 : -fill_w/2);
            if (fill_w > 0.001f) {
                renderer->addRect(fill_cx, sy, fill_w, slider_track_h * 0.7f, cr * 0.6f, cg * 0.6f, cb * 0.6f, 0.7f);
            }
            
            // Thumb indicator (at current value position)
            float thumb_x = slider_cx + fill_ratio * slider_track_w / 2.0f;
            thumb_x = fmaxf(slider_cx - slider_track_w/2, fminf(slider_cx + slider_track_w/2, thumb_x));
            bool is_dragging = (mnv_popup_slider_dragging == s);
            float thumb_w = is_dragging ? 0.016f : 0.012f;
            float thumb_h = is_dragging ? 0.026f : 0.022f;
            renderer->addRect(thumb_x, sy, thumb_w, thumb_h, cr, cg, cb, is_dragging ? 1.0f : 0.85f);
            renderer->addRectOutline(thumb_x, sy, thumb_w, thumb_h, 1.0f, 1.0f, 1.0f, 0.9f);
        }
        
        // --- Separator line ---
        float sep_y = slider_base_y - 4 * slider_spacing + 0.025f;
        renderer->addRect(pop_x, sep_y, pw * 0.9f, 0.002f, 0.3f, 0.4f, 0.6f, 0.5f);
        
        // --- Total Delta-V ---
        float total_dv = mnv_popup_dv.length();
        char buf[64];
        snprintf(buf, sizeof(buf), "TOTAL DV: %.1f M/S", total_dv);
        renderer->drawText(pop_x, sep_y - 0.022f, buf, 0.011f, 1.0f, 1.0f, 1.0f, 1.0f, true, Renderer::CENTER);
        
        // --- Time to Node ---
        float info_y = sep_y - 0.05f;
        float info_lx = pop_x - pw/2 + 0.015f;
        float info_rx = pop_x + pw/2 - 0.015f;
        
        int t_min = (int)(mnv_popup_time_to_node / 60.0);
        int t_sec = (int)fmod(mnv_popup_time_to_node, 60.0);
        if (mnv_popup_time_to_node > 3600) {
            int t_hr = (int)(mnv_popup_time_to_node / 3600.0);
            t_min = (int)fmod(mnv_popup_time_to_node / 60.0, 60.0);
            snprintf(buf, sizeof(buf), "%s: %dH %02dM %02dS", (mnv_popup_burn_mode == 1 ? "T-START" : "T-NODE"), t_hr, t_min, t_sec);
        } else if (mnv_popup_time_to_node >= 0) {
            snprintf(buf, sizeof(buf), "%s: %dM %02dS", (mnv_popup_burn_mode == 1 ? "T-START" : "T-NODE"), t_min, t_sec);
        } else {
            // Counting up or burn in progress
            snprintf(buf, sizeof(buf), "BURN+ %dS", (int)(-mnv_popup_time_to_node));
        }
        // Color: green if far, yellow if close, red if very close
        float time_ratio = (float)fmin(1.0, fmax(0.0, mnv_popup_time_to_node / 120.0));
        renderer->drawText(info_lx, info_y, buf, 0.010f, 
                          1.0f - time_ratio * 0.7f, 0.3f + time_ratio * 0.7f, 0.2f, 1.0f, true, Renderer::LEFT);
        
        // --- Estimated Burn Time ---
        info_y -= 0.025f;
        int b_min = (int)(mnv_popup_burn_time / 60.0);
        int b_sec = (int)fmod(mnv_popup_burn_time, 60.0);
        if (mnv_popup_burn_time >= 9999.0) {
            snprintf(buf, sizeof(buf), "BURN: ---");
        } else if (mnv_popup_burn_time > 60.0) {
            snprintf(buf, sizeof(buf), "BURN: %dM %02dS", b_min, b_sec);
        } else {
            snprintf(buf, sizeof(buf), "BURN: %.1fS", mnv_popup_burn_time);
        }
        renderer->drawText(info_lx, info_y, buf, 0.010f, 0.9f, 0.7f, 0.3f, 1.0f, true, Renderer::LEFT);
        
        // --- Burn Mode Toggle ---
        float mode_y = info_y - 0.045f;
        float mbw = pw * 0.8f, mbh = 0.032f;
        renderer->addRectOutline(pop_x, mode_y, mbw, mbh, 0.4f, 0.6f, 1.0f, 0.8f);
        if (mnv_popup_mode_hover) {
            renderer->addRect(pop_x, mode_y, mbw, mbh, 0.2f, 0.3f, 0.5f, 0.4f);
        }
        const char* mode_names[] = {"INSTANT IMPULSE", "SUSTAINED BURN"};
        renderer->drawText(pop_x, mode_y, mode_names[mnv_popup_burn_mode], 0.009f, 0.7f, 0.9f, 1.0f, 1.0f, true, Renderer::CENTER);
        renderer->drawText(pop_x, mode_y - 0.022f, "(Click to Toggle)", 0.007f, 0.5f, 0.5f, 0.6f, 0.6f, false, Renderer::CENTER);
        
            // --- DELETE button (bottom center) ---
            float del_btn_y = pop_y - ph/2 + (adv_embed_mnv ? 0.015f : 0.025f);
            float del_btn_w = 0.10f, del_btn_h = 0.03f;
            renderer->addRect(pop_x, del_btn_y, del_btn_w, del_btn_h, 
                              mnv_popup_del_hover ? 0.9f : 0.5f, mnv_popup_del_hover ? 0.15f : 0.08f, mnv_popup_del_hover ? 0.15f : 0.08f, 0.9f);
            renderer->addRectOutline(pop_x, del_btn_y, del_btn_w, del_btn_h, 1.0f, 0.3f, 0.3f, 1.0f);
            renderer->drawText(pop_x, del_btn_y, "DELETE", 0.012f, 1.0f, 1.0f, 1.0f, 1.0f, false, Renderer::CENTER);
        }
    }
    
    renderer->endFrame();
    glfwSwapBuffers(window);
  }

  delete renderer;
  earthMesh.destroy();
  ringMesh.destroy();
  rocketBody.destroy();
  rocketNose.destroy();
  rocketBox.destroy();
  delete r3d;
  
  // Save final state on ending
  if (rocket_state.status == LANDED || rocket_state.status == CRASHED) {
    SaveSystem::SaveGame(assembly, rocket_state, control_input);
    cout << "\n>> GAME SAVED." << endl;
  }
  
  glfwTerminate();
  if (rocket_state.status == LANDED || rocket_state.status == CRASHED) {
    cout << "\n>> SIMULATION ENDED. PRESS ENTER." << endl;
    cin.get();
  }
  return 0;
}}
