#include<glad/glad.h>
#include <GLFW/glfw3.h>
#include <algorithm>
#include <chrono>
#include <thread>
#include <cmath>

#include "math/math3d.h"
#include "render/renderer3d.h"
#include "render/model_loader.h"
#include "core/rocket_state.h"
#include "physics/physics_system.h"
#include "control/control_system.h"
#include "render/renderer_2d.h"
#include "simulation/orbit_physics.h"
#include "simulation/stage_manager.h"
#include "simulation/maneuver_system.h"
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
  double apo = 0, peri = 0;
  PhysicsSystem::getOrbitParams(state, apo, peri);
  cout << "\n----------------------------------" << endl;
  cout << ">>> [MISSION CONTROL]: " << state.mission_msg << " <<<" << endl;
  cout << "----------------------------------" << endl;
  cout << "[Alt]: " << state.altitude << " m | [Vert_Vel]: " << state.velocity << " m/s";
  
  double surface_speed = std::sqrt(state.velocity * state.velocity + state.local_vx * state.local_vx);
  bool is_parked = (state.status == PRE_LAUNCH || state.status == LANDED) && surface_speed < 0.1;

  if (!state.auto_mode && (state.altitude > 100000.0 || is_parked) && input.throttle < 0.01) {
      cout << " | [WARP READY]" << endl;
  } else {
      cout << endl;
  }
  double velocity_mag = sqrt(state.vx*state.vx + state.vy*state.vy + state.vz*state.vz);
  cout << "[Pos_X]: " << state.px << " m | [Horz_Vel]: " << state.vx << " m/s" << endl;
  cout << "[Angle]: " << state.angle * 180.0 / PI
       << " deg | [Throttle]: " << input.throttle * 100 << "%" << endl;
  cout << "[Ground_Horz_Vel]: " << state.local_vx
       << " m/s | [Orbit_Vel]: " << velocity_mag << " m/s" << endl;
  cout << "[Thrust]: " << state.thrust_power / 1000 << " kN | [Fuel]: " << state.fuel
       << " kg" << endl;
  cout << "[Apoapsis]: " << apo / 1000.0
       << " km | [Periapsis]: " << peri / 1000.0 << " km" << endl;
  cout << "[Status]: " << state.status << endl;
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
  // ：开启透明度混合
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  // 激活硬件级多重采样抗锯齿
  glEnable(GL_MULTISAMPLE);
  renderer = new Renderer();
  
  PhysicsSystem::InitSolarSystem();

  // =========================================================
  // 主菜单阶段：显示启动菜单
  // =========================================================
  MenuSystem::MenuState menu_state;
  menu_state.has_save = SaveSystem::HasSaveFile();
  
  if (menu_state.has_save) {
      SaveSystem::GetSaveInfo(menu_state.save_time, menu_state.save_parts);
  }
  
  MenuSystem::MenuChoice menu_choice = MenuSystem::MENU_NONE;
  bool up_pressed = false, down_pressed = false, enter_pressed = false;
  
  // 主菜单循环
  while (menu_choice == MenuSystem::MENU_NONE && !glfwWindowShouldClose(window)) {
      glfwPollEvents();
      
      if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS) {
          glfwSetWindowShouldClose(window, true);
          break;
      }
      
      // 获取鼠标状态
      double mx, my;
      glfwGetCursorPos(window, &mx, &my);
      int ww, wh;
      glfwGetWindowSize(window, &ww, &wh);
      // 转换为渲染坐标 [-1, 1]
      float mouse_x = (float)(mx / ww * 2.0 - 1.0);
      float mouse_y = (float)(1.0 - my / wh * 2.0);
      bool mouse_left = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS;

      menu_choice = MenuSystem::HandleMenuInput(window, menu_state, up_pressed, down_pressed, enter_pressed, mouse_x, mouse_y, mouse_left);
      
      glClearColor(0.02f, 0.03f, 0.08f, 1.0f);
      glClear(GL_COLOR_BUFFER_BIT);
      renderer->beginFrame();
      MenuSystem::DrawMainMenu(renderer, menu_state, (float)glfwGetTime());
      renderer->endFrame();
      glfwSwapBuffers(window);
      
      std::this_thread::sleep_for(std::chrono::milliseconds(16));
  }
  
  // 处理菜单选择
  if (menu_choice == MenuSystem::MENU_EXIT || glfwWindowShouldClose(window)) {
      delete renderer;
      glfwTerminate();
      return 0;
  }
  
  bool load_from_save = (menu_choice == MenuSystem::MENU_CONTINUE);
  
  if (menu_choice == MenuSystem::MENU_NEW_GAME && menu_state.has_save) {
      SaveSystem::DeleteSaveFile();
  }

  // =========================================================
  // 初始化 3D 渲染器和网格 (Early instantiation for Workshop)
  // =========================================================
  Renderer3D* r3d = new Renderer3D();
  Mesh earthMesh = MeshGen::sphere(96, 128, 1.0f);  // High-res unit sphere for RSS-Reborn quality
  Mesh ringMesh = MeshGen::ring(64, 0.45f, 1.0f);   // 环形网格(内径0.45, 外径1.0)
  Mesh rocketBody = MeshGen::cylinder(32, 1.0f, 1.0f);
  Mesh rocketNose = MeshGen::cone(32, 1.0f, 1.0f);
  Mesh rocketBox  = MeshGen::box(1.0f, 1.0f, 1.0f);
  
  // 尝试加载发射台模型 (如果存在)
  Mesh launchPadMesh = ModelLoader::loadOBJ("assets/launch_pad.obj");
  bool has_launch_pad = (launchPadMesh.indexCount > 0);

  // =========================================================
  // BUILD 阶段：KSP-like 火箭组装界面
  // =========================================================
  BuilderState builder_state;
  RocketState loaded_state;
  ControlInput loaded_input;
  bool skip_builder = false;
  
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

  BuilderKeyState bk_prev = {false,false,false,false,false,false,false,false,false,false};

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

    // Mouse for builder
    double bmx, bmy;
    glfwGetCursorPos(window, &bmx, &bmy);
    int bww, bwh;
    glfwGetWindowSize(window, &bww, &bwh);
    bk_now.mx = (float)(bmx / bww * 2.0 - 1.0);
    bk_now.my = (float)(1.0 - bmy / bwh * 2.0);
    bk_now.lmb = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS;

    // Workshop camera controls (scroll and right click drag)
    static double workshop_prev_mx = 0, workshop_prev_my = 0;
    static bool workshop_is_dragging = false;

    if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS) {
        double mx, my;
        glfwGetCursorPos(window, &mx, &my);
        
        if (!workshop_is_dragging) {
            // Start of a new drag: anchor here
            workshop_prev_mx = mx;
            workshop_prev_my = my;
            workshop_is_dragging = true;
        } else {
            // Dragging: calculate delta
            float dx = (float)(mx - workshop_prev_mx) * 0.005f;
            float dy = (float)(my - workshop_prev_my) * 0.005f;
            builder_state.orbit_angle -= dx;
            builder_state.orbit_pitch = std::max(-0.5f, std::min(1.5f, builder_state.orbit_pitch + dy));
            workshop_prev_mx = mx;
            workshop_prev_my = my;
        }
    } else {
        // Button released: stop dragging and resume auto-rotate
        workshop_is_dragging = false;
        builder_state.orbit_angle += 0.001f;
    }
    
    // Zoom control
    if (g_scroll_y != 0.0f) {
        builder_state.cam_dist *= powf(0.85f, g_scroll_y);
        builder_state.cam_dist = std::max(2.0f, std::min(50.0f, builder_state.cam_dist));
        g_scroll_y = 0.0f;
    }

    build_done = builderHandleInput(builder_state, bk_now, bk_prev);
    bk_prev = bk_now;

    // --- 3D WORKSHOP RENDER PASS ---
    int ww, wh;
    glfwGetFramebufferSize(window, &ww, &wh);
    r3d->lightDir = Vec3(0.5f, 1.0f, 0.3f).normalized(); // Angled floodlight

    // Dynamic camera based on rocket height
    float current_height = std::max(5.0f, builder_state.assembly.total_height);
    float look_y = current_height * 0.4f;
    
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

    // Draw Assembled Rocket Stack
    float stack_y = 0.0f;
    for (int i = 0; i < builder_state.assembly.parts.size(); i++) {
        const PlacedPart& pp = builder_state.assembly.parts[i];
        const PartDef& def = PART_CATALOG[pp.def_id];
        
        float r = def.r, g = def.g, b = def.b;
        bool is_selected = (builder_state.in_assembly_mode && builder_state.assembly_cursor == i);
        if (is_selected) {
            float blink = 0.5f + 0.5f * sinf((float)glfwGetTime() * 5.0f);
            r = std::min(1.0f, r + 0.3f * blink);
            g = std::min(1.0f, g + 0.5f * blink);
            b = std::min(1.0f, b + 0.2f * blink);
        }

        float half_h = def.height * 0.5f;
        float py = stack_y + half_h;
        float pd = def.diameter;
        
        if (def.category == CAT_NOSE_CONE) {
            // Nose cones are pivoted at the base (y=0 in MeshGen::cone)
            Mat4 partMat = Mat4::translate(Vec3(0.0f, stack_y, 0.0f)) * Mat4::scale(Vec3(pd, def.height, pd));
            r3d->drawMesh(rocketNose, partMat, r, g, b, 1.0f, 0.2f);
        } else if (def.category == CAT_ENGINE) {
            // Engine Rendering (matching in-flight style: cylinder body + nozzle)
            float bodyFrac = 0.4f;
            float nozzleFrac = 1.0f - bodyFrac;
            float body_y = stack_y + def.height * (1.0f - bodyFrac * 0.5f);
            
            // Upper cylinder body
            Mat4 bodyMat = Mat4::translate(Vec3(0.0f, body_y, 0.0f)) * Mat4::scale(Vec3(pd * 0.6f, def.height * bodyFrac, pd * 0.6f));
            r3d->drawMesh(rocketBody, bodyMat, 0.2f, 0.2f, 0.22f, 1.0f, 0.4f);
            
            // Lower nozzle bell (cone mesh base at y=0)
            Mat4 bellMat = Mat4::translate(Vec3(0.0f, stack_y, 0.0f)) * Mat4::scale(Vec3(pd * 0.85f, def.height * nozzleFrac, pd * 0.85f));
            r3d->drawMesh(rocketNose, bellMat, r * 0.8f, g * 0.8f, b * 0.8f, 1.0f, 0.1f);
        } else if (def.category == CAT_COMMAND_POD) {
            // Command Pod Rendering (matching in-flight style: cylinder + cone)
            float bodyFrac = 0.5f;
            float coneFrac = 1.0f - bodyFrac;
            
            // Base cylinder
            Mat4 bodyMat = Mat4::translate(Vec3(0.0f, stack_y + half_h * bodyFrac, 0.0f)) * Mat4::scale(Vec3(pd, def.height * bodyFrac, pd));
            r3d->drawMesh(rocketBody, bodyMat, r, g, b, 1.0f, 0.3f);
            
            // Tapered top
            Mat4 capMat = Mat4::translate(Vec3(0.0f, stack_y + def.height * bodyFrac, 0.0f)) * Mat4::scale(Vec3(pd * 0.9f, def.height * coneFrac, pd * 0.9f));
            r3d->drawMesh(rocketNose, capMat, r * 1.1f, g * 1.1f, b * 1.1f, 1.0f, 0.3f);
        } else if (def.category == CAT_BOOSTER) {
            // Booster: Main body + side strap-ons
            Mat4 mainMat = Mat4::translate(Vec3(0.0f, py, 0.0f)) * Mat4::scale(Vec3(pd, def.height, pd));
            r3d->drawMesh(rocketBody, mainMat, r, g, b, 1.0f, 0.2f);
            
            for (int si = 0; si < 2; si++) {
                float side_angle = si * 3.14159f;
                float sx = cosf(side_angle) * pd * 0.6f;
                float sz = sinf(side_angle) * pd * 0.6f;
                Mat4 strapMat = Mat4::translate(Vec3(sx, py, sz)) * Mat4::scale(Vec3(pd * 0.3f, def.height * 0.8f, pd * 0.3f));
                r3d->drawMesh(rocketBody, strapMat, r * 0.9f, g * 0.9f, b * 0.9f, 1.0f, 0.2f);
            }
        } else {
            // Standard fuel tank/structural (center-pivoted cylinder)
            Mat4 partMat = Mat4::translate(Vec3(0.0f, py, 0.0f)) * Mat4::scale(Vec3(pd, def.height, pd));
            r3d->drawMesh(rocketBody, partMat, r, g, b, 1.0f, 0.2f);
        }
        
        stack_y += def.height;
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
    
    drawBuilderUI_KSP(renderer, builder_state, (float)glfwGetTime());
    renderer->endFrame();

    glfwSwapBuffers(window);

    std::this_thread::sleep_for(std::chrono::milliseconds(16));
  }

  // =========================================================
  // Build RocketConfig from assembled parts
  // =========================================================
  RocketConfig rocket_config = builder_state.assembly.buildRocketConfig();

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
      
      // Initialize surface coordinates exactly at the planet's radius
      rocket_state.surf_px = 0.0;
      rocket_state.surf_py = SOLAR_SYSTEM[current_soi_index].radius;
      rocket_state.surf_pz = 0.0;
  }

  // Keep a reference to the assembly for rendering
  const RocketAssembly& assembly = builder_state.assembly;

  int cam_mode_3d = 0; // 0=轨道 (优化型), 1=跟踪, 2=全景
  static bool c_was_pressed = false;
  // 相机参数
  Quat cam_quat; // 自由模式四元数 (用于全景和追踪微调)
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
    static bool adv_warp_to_node = false; // warp-to-maneuver in progress
    static bool adv_embed_mnv = false;    // maneuver popup embedded in adv menu
    static bool adv_embed_mnv_mini = false; // deeply folded (mini) state
    static bool mnv_popup_mini_hover = false;

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

    // --- C 键切换 3D 视角 ---
    bool c_now = glfwGetKey(window, GLFW_KEY_C) == GLFW_PRESS;
    if (c_now && !c_was_pressed) {
      cam_mode_3d = (cam_mode_3d + 1) % 3;
      const char* names[] = {"Orbit", "Chase", "Panorama"};
      cout << ">> Camera: " << names[cam_mode_3d] << endl;
    }
    c_was_pressed = c_now;

    // --- 鼠标轨道控制 (3D模式下右键拖动) ---
    {
      double mx, my;
      glfwGetCursorPos(window, &mx, &my);
      bool rmb = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS;
      if (rmb) {
        if (mouse_dragging) {
          float dx = (float)(mx - prev_mx) * 0.003f;
          float dy = (float)(my - prev_my) * 0.003f;
          
          if (cam_mode_3d == 0) {
              // 轨道模式：更新球坐标
              orbit_yaw -= dx;
              orbit_pitch = std::max(-1.5f, std::min(1.5f, orbit_pitch + dy));
          } else {
              // 其他模式：保持原有的自由四元数旋转
              Quat yaw_rot = Quat::fromAxisAngle(Vec3(0.0f, 0.0f, 1.0f), -dx);
              Vec3 cam_right = cam_quat.rotate(Vec3(1.0f, 0.0f, 0.0f));
              Quat pitch_rot = Quat::fromAxisAngle(cam_right, -dy);
              cam_quat = (yaw_rot * pitch_rot * cam_quat).normalized();
          }
        }
        mouse_dragging = true;
      } else {
        mouse_dragging = false;
      }
      prev_mx = mx;
      prev_my = my;

      // 滚轮缩放：根据当前模式分离缩放级别
      if (g_scroll_y != 0.0f) {
        if (cam_mode_3d == 2) {
            // 全景模式：允许极其庞大的缩放以俯瞰太阳系
            cam_zoom_pan *= powf(0.85f, g_scroll_y);
            if (cam_zoom_pan < 0.05f) cam_zoom_pan = 0.05f;
            if (cam_zoom_pan > 500000.0f) cam_zoom_pan = 500000.0f; 
        } else {
            // 轨道/跟踪模式：限制缩放防止火箭消失
            cam_zoom_chase *= powf(0.85f, g_scroll_y);
            if (cam_zoom_chase < 0.05f) cam_zoom_chase = 0.05f;
            if (cam_zoom_chase > 20.0f) cam_zoom_chase = 20.0f;
        }
        g_scroll_y = 0.0f;
      }
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
    manual.yaw_left = glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS || glfwGetKey(window, GLFW_KEY_LEFT) == GLFW_PRESS;
    manual.yaw_right = glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS || glfwGetKey(window, GLFW_KEY_RIGHT) == GLFW_PRESS;
    manual.pitch_up = glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS || glfwGetKey(window, GLFW_KEY_UP) == GLFW_PRESS;
    manual.pitch_down = glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS || glfwGetKey(window, GLFW_KEY_DOWN) == GLFW_PRESS;
    manual.roll_left = glfwGetKey(window, GLFW_KEY_Q) == GLFW_PRESS;
    manual.roll_right = glfwGetKey(window, GLFW_KEY_E) == GLFW_PRESS;

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
        // 普通循环执行实现加速
        for (int i = 0; i < time_warp; i++) {
          if (rocket_state.auto_mode) ControlSystem::UpdateAutoPilot(rocket_state, rocket_config, control_input, dt);
          else ControlSystem::UpdateManualControl(rocket_state, rocket_config, control_input, manual, dt);
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
      double ro_x = 0; double ro_y = 0; double ro_z = 0;
      if (cam_mode_3d == 0 || cam_mode_3d == 1) {
          ro_x = r_px; ro_y = r_py; ro_z = r_pz;
      } else {
          ro_x = SOLAR_SYSTEM[focus_target].px * ws_d; 
          ro_y = SOLAR_SYSTEM[focus_target].py * ws_d; 
          ro_z = SOLAR_SYSTEM[focus_target].pz * ws_d; 
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
      } else {
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
      }

      Mat4 viewMat = Mat4::lookAt(camEye_rel, camTarget_rel, camUpVec);

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
          float atmo_surf_dist = dist_to_center - (body_r + atmo_thickness);

          // Use the atmosphere boundary for camera clipping if it exists
          float geo_dist = (atmo_surf_dist > 0.0f) ? atmo_surf_dist : true_surf_dist;
          if (geo_dist > 0.0f && geo_dist < closest_planet_dist) {
              closest_planet_dist = geo_dist;
          }
      }
      // Also consider distance to the Sun
      {
          float sun_surf = (camEye_rel - renderSun).length() - sun_radius;
          if (sun_surf > 0.0f && sun_surf < closest_planet_dist)
              closest_planet_dist = sun_surf;
      }
      // Near plane = 10% of closest planet surface distance.
      // At low altitude this stays tiny (ground visible); at high altitude it grows
      // large enough to give the depth buffer sufficient precision on planet surfaces.
      // Near plane cap to prevent clipping volumetric atmosphere shells
      float macro_near = fmaxf(0.001f, closest_planet_dist * 0.1f);
      macro_near = fminf(macro_near, 10.0f); // Cap at 10 meters
      macro_near = fmaxf(macro_near, cam_dist * 0.001f); // but never clip behind target
      
      Mat4 macroProjMat = Mat4::perspective(0.8f, aspect, macro_near, far_plane);
      
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
          r3d->drawPlanet(earthMesh, planetModel, b.type, b.r, b.g, b.b, 1.0f, (float)rocket_state.sim_time, (int)i);
          
          if ((b.type == TERRESTRIAL || b.type == GAS_GIANT) && i != 1 && i != 4) {
              // 修改前：float atmo_radius = r * 1.12f;
              // 修改后：使用恒定的物理边界，剥离厚重的多余网格
              float atmo_radius = r + 160.0f;

              Mat4 atmoModel = Mat4::scale(Vec3(atmo_radius, atmo_radius, atmo_radius));
              atmoModel = Mat4::translate(renderPlanet) * atmoModel;

              // New Volumetric Scattering integration with animated hardcore clouds
              r3d->drawAtmosphere(earthMesh, atmoModel, camEye_rel, r3d->lightDir, renderPlanet, r, atmo_radius, (float)rocket_state.sim_time, (int)i, (float)day_blend);
          }
          if (b.type == RINGED_GAS_GIANT) {
              Mat4 ringModel = Mat4::scale(Vec3(r * 2.2f, r * 0.001f, r * 2.2f));
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
      if (cam_mode_3d == 2) {
        if (adv_orbit_enabled) {
            // ==========================================
            // ADVANCED ORBIT PREDICTION: N-BODY RK4
            // ==========================================
            std::vector<Vec3> adv_points;
            std::vector<Vec3> adv_mnv_points; // For prediction after maneuver
            std::vector<Vec3> adv_ground_track;
            std::vector<Vec3> adv_mnv_ground_track;

            int STEPS = adv_orbit_iters;
            double dt = (adv_orbit_pred_days * 86400.0) / (double)STEPS; // auto-calc step size
            
            // Get initial absolute state
            double cur_h_px = rocket_state.px + SOLAR_SYSTEM[current_soi_index].px;
            double cur_h_py = rocket_state.py + SOLAR_SYSTEM[current_soi_index].py;
            double cur_h_pz = rocket_state.pz + SOLAR_SYSTEM[current_soi_index].pz;
            double cur_h_vx = rocket_state.vx + SOLAR_SYSTEM[current_soi_index].vx;
            double cur_h_vy = rocket_state.vy + SOLAR_SYSTEM[current_soi_index].vy;
            double cur_h_vz = rocket_state.vz + SOLAR_SYSTEM[current_soi_index].vz;

            double t_sim = rocket_state.sim_time;
            
            bool has_mnv = (rocket_state.maneuvers.size() > 0);
            double mnv_t = has_mnv ? rocket_state.maneuvers[0].sim_time : 1e20;
            bool mnv_executed = false;
            bool burn_in_progress = false;
            
            double mnv_duration = 0;
            if (has_mnv) {
                double dv_val = rocket_state.maneuvers[0].delta_v.length();
                double m0 = rocket_state.fuel + rocket_config.dry_mass + rocket_config.upper_stages_mass;
                double ve = rocket_config.specific_impulse * 9.80665;
                double mdot = rocket_config.cosrate;
                if (ve > 0 && mdot > 0 && dv_val > 0) {
                    mnv_duration = m0 * (1.0 - 1.0/exp(dv_val/ve)) / mdot;
                }
            }
            double trigger_t = mnv_t;
            if (has_mnv && rocket_state.maneuvers[0].burn_mode == 1) {
                trigger_t = mnv_t - 0.5 * mnv_duration;
            }

            // If the burn is already in progress or completed, we need to:
            if (has_mnv && rocket_state.sim_time >= trigger_t) {
                burn_in_progress = true;
                mnv_executed = true; // Don't re-apply delta-v in the primary RK4
            }
            
            double orig_px = 0, orig_py = 0, orig_pz = 0;
            double orig_vx = 0, orig_vy = 0, orig_vz = 0;
            
            // Pre-calculate current basis for rendering mapping
            double b0_px = SOLAR_SYSTEM[adv_orbit_ref_body].px;
            double b0_py = SOLAR_SYSTEM[adv_orbit_ref_body].py;
            double b0_pz = SOLAR_SYSTEM[adv_orbit_ref_body].pz;
            double b0_vx = SOLAR_SYSTEM[adv_orbit_ref_body].vx;
            double b0_vy = SOLAR_SYSTEM[adv_orbit_ref_body].vy;
            double b0_vz = SOLAR_SYSTEM[adv_orbit_ref_body].vz;
            
            Vec3 ux0, uy0, uz0;
            if (adv_orbit_ref_mode == 1 && adv_orbit_ref_body != 0) {
                Vec3 R(b0_px, b0_py, b0_pz);
                Vec3 V(b0_vx, b0_vy, b0_vz);
                ux0 = R.normalized();
                uz0 = R.cross(V).normalized();
                uy0 = uz0.cross(ux0).normalized();
            }

            auto calc_acc = [&](double tx, double px, double py, double pz, double& ax, double& ay, double& az) {
                PhysicsSystem::UpdateCelestialBodies(tx);
                ax = 0; ay = 0; az = 0;
                for (size_t b = 0; b < SOLAR_SYSTEM.size(); b++) {
                    double dx = SOLAR_SYSTEM[b].px - px;
                    double dy = SOLAR_SYSTEM[b].py - py;
                    double dz = SOLAR_SYSTEM[b].pz - pz;
                    double dist_sq = dx*dx + dy*dy + dz*dz;
                    if (dist_sq > 0) {
                        double dist = sqrt(dist_sq);
                        double F = (6.67430e-11 * SOLAR_SYSTEM[b].mass) / dist_sq;
                        ax += F * dx / dist;
                        ay += F * dy / dist;
                        az += F * dz / dist;
                    }
                }
            };

            double max_pred_time = rocket_state.sim_time + adv_orbit_pred_days * 86400.0;
            const double FR_theta = 1.0 / (2.0 - cbrt(2.0));
            const double c1 = FR_theta / 2.0, c2 = (1.0 - FR_theta) / 2.0;
            const double d1 = FR_theta, d2 = 1.0 - 2.0 * FR_theta;

            int actual_steps = 0;
            while (t_sim < max_pred_time && actual_steps < adv_orbit_iters) {
                actual_steps++;
                
                if (has_mnv && !mnv_executed) {
                    if (t_sim >= trigger_t - 1e-6) {
                        t_sim = trigger_t; // Strict snap to maneuver time
                        
                        // Extract point before applying burn
                        if (!adv_points.empty()) {
                            adv_mnv_world_pos = adv_points.back();
                        } else {
                            double fx=cur_h_px, fy=cur_h_py, fz=cur_h_pz;
                            if (adv_orbit_ref_mode==0 && adv_orbit_ref_body!=0) { fx = (cur_h_px-SOLAR_SYSTEM[adv_orbit_ref_body].px)+b0_px; fy = (cur_h_py-SOLAR_SYSTEM[adv_orbit_ref_body].py)+b0_py; fz = (cur_h_pz-SOLAR_SYSTEM[adv_orbit_ref_body].pz)+b0_pz; }
                            adv_mnv_world_pos = Vec3((float)(fx*ws_d - ro_x), (float)(fy*ws_d - ro_y), (float)(fz*ws_d - ro_z));
                        }

                        PhysicsSystem::UpdateCelestialBodies(t_sim);
                        double r_rel_x = cur_h_px - SOLAR_SYSTEM[current_soi_index].px;
                        double r_rel_y = cur_h_py - SOLAR_SYSTEM[current_soi_index].py;
                        double r_rel_z = cur_h_pz - SOLAR_SYSTEM[current_soi_index].pz;
                        double v_rel_x = cur_h_vx - SOLAR_SYSTEM[current_soi_index].vx;
                        double v_rel_y = cur_h_vy - SOLAR_SYSTEM[current_soi_index].vy;
                        double v_rel_z = cur_h_vz - SOLAR_SYSTEM[current_soi_index].vz;
                        
                        // Dynamically update the node's osculating Kepler parameters so the UI handles rotate correctly
                        double r_mag = sqrt(r_rel_x*r_rel_x + r_rel_y*r_rel_y + r_rel_z*r_rel_z);
                        double v_sq_rel = v_rel_x*v_rel_x + v_rel_y*v_rel_y + v_rel_z*v_rel_z;
                        double mu = 6.67430e-11 * SOLAR_SYSTEM[current_soi_index].mass;
                        Vec3 r_vec((float)r_rel_x, (float)r_rel_y, (float)r_rel_z);
                        Vec3 v_vec((float)v_rel_x, (float)v_rel_y, (float)v_rel_z);
                        Vec3 h_vec = r_vec.cross(v_vec);
                        double energy = 0.5 * v_sq_rel - mu / r_mag;
                        double a = -mu / (2.0 * energy);
                        if (a != 0) {
                            Vec3 e_vec = v_vec.cross(h_vec) / (float)mu - r_vec / (float)r_mag;
                            double ecc = e_vec.length();
                            Vec3 e_dir = (ecc > 1e-7f) ? e_vec.normalized() : Vec3(1,0,0);
                            Vec3 p_dir = h_vec.normalized().cross(e_dir).normalized();
                            double s_cosE = (a - r_mag) / (a * ecc);
                            double s_sinE = r_vec.dot(p_dir) / (a * sqrt(fmax(0.0, 1.0 - ecc*ecc)));
                            double E0 = atan2(s_sinE, s_cosE);
                            rocket_state.maneuvers[0].ref_a = a;
                            rocket_state.maneuvers[0].ref_ecc = ecc;
                            rocket_state.maneuvers[0].ref_M0 = E0;
                            rocket_state.maneuvers[0].ref_n = sqrt(mu / (a * a * a));
                            rocket_state.maneuvers[0].ref_center = e_dir * (-(float)a * (float)ecc);
                            rocket_state.maneuvers[0].ref_e_dir = e_dir;
                            rocket_state.maneuvers[0].ref_p_dir = p_dir;
                        }
                        
                        ManeuverFrame frame = ManeuverSystem::getFrame(r_vec, v_vec);
                        Vec3 dv = frame.prograde * rocket_state.maneuvers[0].delta_v.x + 
                                  frame.normal   * rocket_state.maneuvers[0].delta_v.y + 
                                  frame.radial   * rocket_state.maneuvers[0].delta_v.z;
                                  
                        orig_px = cur_h_px; orig_py = cur_h_py; orig_pz = cur_h_pz;
                        orig_vx = cur_h_vx; orig_vy = cur_h_vy; orig_vz = cur_h_vz;
                        
                        cur_h_vx += dv.x; cur_h_vy += dv.y; cur_h_vz += dv.z;
                        
                        // Snapshot the post-burn state ONLY ONCE for stable orbit prediction.
                        if (!rocket_state.maneuvers[0].snap_valid) {
                            rocket_state.maneuvers[0].snap_px = cur_h_px;
                            rocket_state.maneuvers[0].snap_py = cur_h_py;
                            rocket_state.maneuvers[0].snap_pz = cur_h_pz;
                            rocket_state.maneuvers[0].snap_vx = cur_h_vx;
                            rocket_state.maneuvers[0].snap_vy = cur_h_vy;
                            rocket_state.maneuvers[0].snap_vz = cur_h_vz;
                            rocket_state.maneuvers[0].snap_valid = true;
                        }
                        
                        mnv_executed = true;
                        continue; // Skip the integration time-step this loop, maneuver is applied instantly in-place!
                    }
                }
                
                double min_dist_sq = 1e20;
                for (size_t b = 0; b < SOLAR_SYSTEM.size(); b++) {
                    double dx = cur_h_px - SOLAR_SYSTEM[b].px;
                    double dy = cur_h_py - SOLAR_SYSTEM[b].py;
                    double dz = cur_h_pz - SOLAR_SYSTEM[b].pz;
                    double dsq = dx*dx + dy*dy + dz*dz;
                    if (dsq > 1.0 && dsq < min_dist_sq) min_dist_sq = dsq;
                }
                double v_sq = cur_h_vx*cur_h_vx + cur_h_vy*cur_h_vy + cur_h_vz*cur_h_vz;
                
                // Sundman Time Transformation: dt scales strictly with local r/v curvature
                double step_dt = 0.05 * sqrt(min_dist_sq / fmax(v_sq, 1.0));
                
                if (step_dt > 3600.0 * 24.0) step_dt = 3600.0 * 24.0;
                if (step_dt < 0.1) step_dt = 0.1;
                
                if (has_mnv && !mnv_executed && t_sim + step_dt > trigger_t) {
                    step_dt = trigger_t - t_sim; // strict sync at burn bounds
                } else if (t_sim + step_dt > max_pred_time) {
                    step_dt = max_pred_time - t_sim;
                }
                
                if (step_dt <= 0) break;
                
                // 4th-Order Symplectic LMM evaluate (Forest-Ruth)
                // We use macros to cleanly represent the symmetric position and velocity stages
                #define SYM_Q(C) \
                    cur_h_px += (C) * cur_h_vx * step_dt; cur_h_py += (C) * cur_h_vy * step_dt; cur_h_pz += (C) * cur_h_vz * step_dt; \
                    t_sim += (C) * step_dt; \
                    if (mnv_executed) { orig_px += (C)*orig_vx*step_dt; orig_py += (C)*orig_vy*step_dt; orig_pz += (C)*orig_vz*step_dt; }

                #define SYM_P(D) \
                    { \
                        double ax, ay, az; calc_acc(t_sim, cur_h_px, cur_h_py, cur_h_pz, ax, ay, az); \
                        cur_h_vx += (D) * ax * step_dt; cur_h_vy += (D) * ay * step_dt; cur_h_vz += (D) * az * step_dt; \
                        if (mnv_executed) { \
                            double oax, oay, oaz; calc_acc(t_sim, orig_px, orig_py, orig_pz, oax, oay, oaz); \
                            orig_vx += (D) * oax * step_dt; orig_vy += (D) * oay * step_dt; orig_vz += (D) * oaz * step_dt; \
                        } \
                    }

                // Execute 4 completely symmetric stages to guarantee long term energy conservation
                SYM_Q(c1); SYM_P(d1);
                SYM_Q(c2); SYM_P(d2);
                SYM_Q(c2); SYM_P(d1);
                SYM_Q(c1);

                #undef SYM_Q
                #undef SYM_P

                // Sync planets to new t_sim to extract rendering reference
                PhysicsSystem::UpdateCelestialBodies(t_sim);
                
                double final_px = cur_h_px, final_py = cur_h_py, final_pz = cur_h_pz;

                if (adv_orbit_ref_body != 0) {
                    if (adv_orbit_ref_mode == 0) { // Inertial Translation
                        final_px = (cur_h_px - SOLAR_SYSTEM[adv_orbit_ref_body].px) + b0_px;
                        final_py = (cur_h_py - SOLAR_SYSTEM[adv_orbit_ref_body].py) + b0_py;
                        final_pz = (cur_h_pz - SOLAR_SYSTEM[adv_orbit_ref_body].pz) + b0_pz;
                    } else if (adv_orbit_ref_mode == 1) { // Co-rotating
                        Vec3 R(SOLAR_SYSTEM[adv_orbit_ref_body].px, SOLAR_SYSTEM[adv_orbit_ref_body].py, SOLAR_SYSTEM[adv_orbit_ref_body].pz);
                        Vec3 V(SOLAR_SYSTEM[adv_orbit_ref_body].vx, SOLAR_SYSTEM[adv_orbit_ref_body].vy, SOLAR_SYSTEM[adv_orbit_ref_body].vz);
                        Vec3 uxt = R.normalized();
                        Vec3 uzt = R.cross(V).normalized();
                        Vec3 uyt = uzt.cross(uxt).normalized();
                        
                        // Pos relative to Sun
                        Vec3 p_sc((float)cur_h_px, (float)cur_h_py, (float)cur_h_pz); 
                        float xc = p_sc.dot(uxt);
                        float yc = p_sc.dot(uyt);
                        float zc = p_sc.dot(uzt);

                        // Reconstruct in t0 basis
                        Vec3 recon = ux0 * xc + uy0 * yc + uz0 * zc;
                        final_px = recon.x; final_py = recon.y; final_pz = recon.z;
                    } else if (adv_orbit_ref_mode == 2) { // Surface
                        double dx = cur_h_px - SOLAR_SYSTEM[adv_orbit_ref_body].px;
                        double dy = cur_h_py - SOLAR_SYSTEM[adv_orbit_ref_body].py;
                        double dz = cur_h_pz - SOLAR_SYSTEM[adv_orbit_ref_body].pz;
                        
                        double rot_period = SOLAR_SYSTEM[adv_orbit_ref_body].rotation_period;
                        double delta_theta = (t_sim - rocket_state.sim_time) * (2.0 * PI / rot_period);
                        
                        double c = cos(delta_theta);
                        double s = sin(delta_theta);
                        
                        double recon_x = dx * c + dy * s;
                        double recon_y = -dx * s + dy * c;
                        double recon_z = dz;
                        
                        final_px = recon_x + b0_px;
                        final_py = recon_y + b0_py;
                        final_pz = recon_z + b0_pz;
                    }
                }
                
                Vec3 render_pt((float)(final_px * ws_d - ro_x), (float)(final_py * ws_d - ro_y), (float)(final_pz * ws_d - ro_z));
                // Ground track point calculation
                if (adv_orbit_ref_mode == 1) { // Ground track only makes sense in Co-rotating frame
                    double dx = final_px - b0_px;
                    double dy = final_py - b0_py;
                    double dz = final_pz - b0_pz;
                    double dist = sqrt(dx*dx + dy*dy + dz*dz);
                    if (dist > 0 && SOLAR_SYSTEM[adv_orbit_ref_body].radius > 0) {
                        double surf_r = SOLAR_SYSTEM[adv_orbit_ref_body].radius * 1.002; // slightly above surface
                        double gx = b0_px + dx/dist * surf_r;
                        double gy = b0_py + dy/dist * surf_r;
                        double gz = b0_pz + dz/dist * surf_r;
                        Vec3 gt_pt((float)(gx * ws_d - ro_x), (float)(gy * ws_d - ro_y), (float)(gz * ws_d - ro_z));
                        if (mnv_executed) adv_mnv_ground_track.push_back(gt_pt);
                        else adv_ground_track.push_back(gt_pt);
                    }
                }
                if (mnv_executed && !burn_in_progress) {
                    adv_mnv_points.push_back(render_pt);
                    
                    // Render shadowed original path
                    double o_final_px = orig_px, o_final_py = orig_py, o_final_pz = orig_pz;
                    if (adv_orbit_ref_mode == 0 && adv_orbit_ref_body != 0) { // Inertial
                        o_final_px = (orig_px - SOLAR_SYSTEM[adv_orbit_ref_body].px) + b0_px;
                        o_final_py = (orig_py - SOLAR_SYSTEM[adv_orbit_ref_body].py) + b0_py;
                        o_final_pz = (orig_pz - SOLAR_SYSTEM[adv_orbit_ref_body].pz) + b0_pz;
                    } else if (adv_orbit_ref_mode == 1) { // Co-rotating
                        Vec3 p_sc((float)orig_px, (float)orig_py, (float)orig_pz);
                        Vec3 R(SOLAR_SYSTEM[adv_orbit_ref_body].px, SOLAR_SYSTEM[adv_orbit_ref_body].py, SOLAR_SYSTEM[adv_orbit_ref_body].pz);
                        Vec3 V(SOLAR_SYSTEM[adv_orbit_ref_body].vx, SOLAR_SYSTEM[adv_orbit_ref_body].vy, SOLAR_SYSTEM[adv_orbit_ref_body].vz);
                        Vec3 uxt = R.normalized();
                        Vec3 uzt = R.cross(V).normalized();
                        Vec3 uyt = uzt.cross(uxt).normalized();
                        float xc = p_sc.dot(uxt);
                        float yc = p_sc.dot(uyt);
                        float zc = p_sc.dot(uzt);
                        Vec3 recon = ux0 * xc + uy0 * yc + uz0 * zc;
                        o_final_px = recon.x; o_final_py = recon.y; o_final_pz = recon.z;
                    } else if (adv_orbit_ref_mode == 2) { // Surface
                        double dx = orig_px - SOLAR_SYSTEM[adv_orbit_ref_body].px;
                        double dy = orig_py - SOLAR_SYSTEM[adv_orbit_ref_body].py;
                        double dz = orig_pz - SOLAR_SYSTEM[adv_orbit_ref_body].pz;
                        
                        double rot_period = SOLAR_SYSTEM[adv_orbit_ref_body].rotation_period;
                        double delta_theta = (t_sim - rocket_state.sim_time) * (2.0 * PI / rot_period);
                        
                        double c = cos(delta_theta);
                        double s = sin(delta_theta);
                        
                        double recon_x = dx * c + dy * s;
                        double recon_y = -dx * s + dy * c;
                        double recon_z = dz;
                        
                        o_final_px = recon_x + b0_px;
                        o_final_py = recon_y + b0_py;
                        o_final_pz = recon_z + b0_pz;
                    }
                    Vec3 o_render_pt((float)(o_final_px * ws_d - ro_x), (float)(o_final_py * ws_d - ro_y), (float)(o_final_pz * ws_d - ro_z));
                    adv_points.push_back(o_render_pt); // Original path continues
                    
                    if (adv_orbit_ref_mode == 1) {
                        double odx = o_final_px - b0_px;
                        double ody = o_final_py - b0_py;
                        double odz = o_final_pz - b0_pz;
                        double odist = sqrt(odx*odx + ody*ody + odz*odz);
                        if (odist > 0 && SOLAR_SYSTEM[adv_orbit_ref_body].radius > 0) {
                            double surf_r = SOLAR_SYSTEM[adv_orbit_ref_body].radius * 1.002;
                            double gx = b0_px + odx/odist * surf_r;
                            double gy = b0_py + ody/odist * surf_r;
                            double gz = b0_pz + odz/odist * surf_r;
                            Vec3 o_gt_pt((float)(gx * ws_d - ro_x), (float)(gy * ws_d - ro_y), (float)(gz * ws_d - ro_z));
                            adv_ground_track.push_back(o_gt_pt);
                        }
                    }
                } else {
                    // During burn or no maneuver: everything goes to the primary (cyan) trajectory
                    adv_points.push_back(render_pt);
                }
            }
            // Restore current sim state
            PhysicsSystem::UpdateCelestialBodies(rocket_state.sim_time);
            
            // === STABLE PLANNED ORBIT: If burn is in progress and we have a snapshot, ===
            // === run a SECOND independent RK4 from the snapshot to show the planned orange orbit ===
            if (burn_in_progress && has_mnv && rocket_state.maneuvers[0].snap_valid) {
                double s_px = rocket_state.maneuvers[0].snap_px;
                double s_py = rocket_state.maneuvers[0].snap_py;
                double s_pz = rocket_state.maneuvers[0].snap_pz;
                double s_vx = rocket_state.maneuvers[0].snap_vx;
                double s_vy = rocket_state.maneuvers[0].snap_vy;
                double s_vz = rocket_state.maneuvers[0].snap_vz;
                double s_t = trigger_t;
                
                const double FR_theta = 1.0 / (2.0 - cbrt(2.0));
                const double c1 = FR_theta / 2.0, c2 = (1.0 - FR_theta) / 2.0;
                const double d1 = FR_theta, d2 = 1.0 - 2.0 * FR_theta;
                
                int actual_steps = 0;
                double max_s_time = trigger_t + adv_orbit_pred_days * 86400.0;
                
                while (actual_steps < adv_orbit_iters && s_t < max_s_time) {
                    actual_steps++;
                    
                    double min_dist_sq = 1e20;
                    for (size_t b = 0; b < SOLAR_SYSTEM.size(); b++) {
                        double dx = s_px - SOLAR_SYSTEM[b].px, dy = s_py - SOLAR_SYSTEM[b].py, dz = s_pz - SOLAR_SYSTEM[b].pz;
                        double dsq = dx*dx + dy*dy + dz*dz;
                        if (dsq > 1.0 && dsq < min_dist_sq) min_dist_sq = dsq;
                    }
                    double v_sq = s_vx*s_vx + s_vy*s_vy + s_vz*s_vz;
                    
                    double step_dt = 0.05 * sqrt(min_dist_sq / fmax(v_sq, 1.0));
                    if (step_dt > 3600.0 * 24.0) step_dt = 3600.0 * 24.0;
                    if (step_dt < 0.1) step_dt = 0.1;
                    if (s_t + step_dt > max_s_time) step_dt = max_s_time - s_t;
                    
                    if (step_dt <= 0) break;
                    
                    #define SYM_S_Q(C) s_px += (C)*s_vx*step_dt; s_py += (C)*s_vy*step_dt; s_pz += (C)*s_vz*step_dt; s_t += (C)*step_dt;
                    #define SYM_S_P(D) { double ax,ay,az; calc_acc(s_t, s_px,s_py,s_pz, ax,ay,az); s_vx+=(D)*ax*step_dt; s_vy+=(D)*ay*step_dt; s_vz+=(D)*az*step_dt; }
                    
                    SYM_S_Q(c1); SYM_S_P(d1);
                    SYM_S_Q(c2); SYM_S_P(d2);
                    SYM_S_Q(c2); SYM_S_P(d1);
                    SYM_S_Q(c1);
                    
                    #undef SYM_S_Q
                    #undef SYM_S_P
                    
                    PhysicsSystem::UpdateCelestialBodies(s_t);
                    double fp = s_px, fpy = s_py, fpz = s_pz;
                    if (adv_orbit_ref_body != 0) {
                        if (adv_orbit_ref_mode == 0) { fp = (s_px-SOLAR_SYSTEM[adv_orbit_ref_body].px)+b0_px; fpy = (s_py-SOLAR_SYSTEM[adv_orbit_ref_body].py)+b0_py; fpz = (s_pz-SOLAR_SYSTEM[adv_orbit_ref_body].pz)+b0_pz; }
                        else if (adv_orbit_ref_mode == 2) {
                            double dx = s_px - SOLAR_SYSTEM[adv_orbit_ref_body].px;
                            double dy = s_py - SOLAR_SYSTEM[adv_orbit_ref_body].py;
                            double dz = s_pz - SOLAR_SYSTEM[adv_orbit_ref_body].pz;
                            double rot_period = SOLAR_SYSTEM[adv_orbit_ref_body].rotation_period;
                            double delta_theta = (s_t - rocket_state.sim_time) * (2.0 * PI / rot_period);
                            double c = cos(delta_theta), s = sin(delta_theta);
                            fp  = (dx * c + dy * s) + b0_px;
                            fpy = (-dx * s + dy * c) + b0_py;
                            fpz = dz + b0_pz;
                        }
                    }
                    Vec3 rp((float)(fp * ws_d - ro_x), (float)(fpy * ws_d - ro_y), (float)(fpz * ws_d - ro_z));
                    adv_mnv_points.push_back(rp);
                    
                    if (adv_orbit_ref_mode == 1) {
                        double mdx = fp - b0_px, mdy = fpy - b0_py, mdz = fpz - b0_pz;
                        double mdist = sqrt(mdx*mdx + mdy*mdy + mdz*mdz);
                        if (mdist > 0 && SOLAR_SYSTEM[adv_orbit_ref_body].radius > 0) {
                            double sr = SOLAR_SYSTEM[adv_orbit_ref_body].radius * 1.002;
                            Vec3 mgp((float)((b0_px + mdx/mdist*sr) * ws_d - ro_x), (float)((b0_py + mdy/mdist*sr) * ws_d - ro_y), (float)((b0_pz + mdz/mdist*sr) * ws_d - ro_z));
                            adv_mnv_ground_track.push_back(mgp);
                        }
                    }
                }
                PhysicsSystem::UpdateCelestialBodies(rocket_state.sim_time);
            }

            // Draw Predicted Path
            float ribbon_w = fmaxf(earth_r * 0.006f, cam_dist * 0.001f);
            if (adv_points.size() > 1) {
                // Original Predicted Path matches the celestial body color scheme basically matching Keplerian (Cyan/Blue)
                // --- Spline Smoothing (Centripetal Catmull-Rom) ---
                std::vector<Vec3> smooth_pts = CatmullRomSpline::interpolate(adv_points, 8);
                r3d->drawRibbon(smooth_pts, ribbon_w, 0.4f, 0.8f, 1.0f, 0.85f);
            }
            if (adv_mnv_points.size() > 1) {
                // Dashed line logic for maneuver prediction: Make it Orange
                // --- Spline Smoothing (Centripetal Catmull-Rom) ---
                std::vector<Vec3> smooth_mnv_pts = CatmullRomSpline::interpolate(adv_mnv_points, 8);

                for (size_t s = 0; s < smooth_mnv_pts.size(); s += 5) {
                    std::vector<Vec3> dash;
                    for (size_t j = 0; j < 3; j++) {
                        if (s + j < smooth_mnv_pts.size()) dash.push_back(smooth_mnv_pts[s + j]);
                    }
                    if (dash.size() >= 2) r3d->drawRibbon(dash, ribbon_w, 1.0f, 0.6f, 0.1f, 0.9f);
                }
            }
            // Draw Ground Tracks
            float gt_w = fmaxf(earth_r * 0.003f, cam_dist * 0.0005f);
            if (adv_ground_track.size() > 1) {
                r3d->drawRibbon(adv_ground_track, gt_w, 0.4f, 0.8f, 1.0f, 0.6f);
            }
            if (adv_mnv_ground_track.size() > 1) {
                for (size_t s = 0; s < adv_mnv_ground_track.size(); s += 5) {
                    std::vector<Vec3> dash;
                    for (size_t j = 0; j < 3; j++) {
                        if (s + j < adv_mnv_ground_track.size()) dash.push_back(adv_mnv_ground_track[s + j]);
                    }
                    if (dash.size() >= 2) r3d->drawRibbon(dash, gt_w, 1.0f, 0.6f, 0.1f, 0.6f);
                }
            }

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
        int stable_ref = current_soi_index;

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
            if (adv_orbit_enabled && i == 0) {
                node_world = adv_mnv_world_pos;
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
            if (node.burn_mode == 1) {
                target_t = node.sim_time - 0.5 * mnv_popup_burn_time;
            }
            mnv_popup_time_to_node = target_t - rocket_state.sim_time;
            if (mnv_popup_time_to_node < -mnv_popup_burn_time) mnv_popup_time_to_node = -mnv_popup_burn_time; // keep showing count-up during burn
            
            // Compute remaining delta-v: if burn is in progress, compare current velocity
            // against the planned post-burn velocity from the snapshot
            float total_dv_val = node.delta_v.length();
            float remaining_dv = total_dv_val; // Default: full delta-v
            
            if (node.snap_valid && rocket_state.sim_time >= node.sim_time) {
                // Burn is in progress! Compute remaining dv from velocity difference
                double cur_abs_vx = rocket_state.vx + SOLAR_SYSTEM[current_soi_index].vx;
                double cur_abs_vy = rocket_state.vy + SOLAR_SYSTEM[current_soi_index].vy;
                double cur_abs_vz = rocket_state.vz + SOLAR_SYSTEM[current_soi_index].vz;
                // Target velocity is the snapshot velocity (includes delta-v)
                double rem_vx = node.snap_vx - cur_abs_vx;
                double rem_vy = node.snap_vy - cur_abs_vy;
                double rem_vz = node.snap_vz - cur_abs_vz;
                remaining_dv = (float)sqrt(rem_vx*rem_vx + rem_vy*rem_vy + rem_vz*rem_vz);
                if (remaining_dv < 0.1f) remaining_dv = 0.0f; // Threshold for completion
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
                    double mdot = rocket_config.cosrate;
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
      Vec3 engNozzlePos = renderRocketPos - rocketDir * (rh * 0.50f); // default engine pos for flame
      {
        float total_h_3d = (float)rocket_config.height;
        float scale_3d = rh / fmaxf(total_h_3d, 1.0f); // meters-to-world scale
        Vec3 rocketBottom = renderRocketPos - rocketDir * (rh * 0.5f);
        
        // Determine which parts to render (skip jettisoned stages)
        int render_start = 0;
        if (rocket_state.current_stage < (int)rocket_config.stage_configs.size()) {
            render_start = rocket_config.stage_configs[rocket_state.current_stage].part_start_index;
        }
        // Recalculate stack_y offset for rendering from render_start
        float y_offset_jettison = 0;
        for (int pi = 0; pi < render_start && pi < (int)assembly.parts.size(); pi++) {
            y_offset_jettison += PART_CATALOG[assembly.parts[pi].def_id].height;
        }

        for (int pi = render_start; pi < (int)assembly.parts.size(); pi++) {
          const PlacedPart& pp = assembly.parts[pi];
          const PartDef& def = PART_CATALOG[pp.def_id];
          float part_h_3d = def.height * scale_3d;
          float part_w_3d = (def.diameter / fmaxf((float)rocket_config.diameter, 1.0f)) * rw_3d;
          float adjusted_stack_y = (pp.stack_y - y_offset_jettison) * scale_3d;
          float part_center_y = adjusted_stack_y + part_h_3d * 0.5f;
          Vec3 partPos = rocketBottom + rocketDir * part_center_y;
          // Bottom and top edges of this part's slot
          Vec3 partBot = rocketBottom + rocketDir * adjusted_stack_y;
          Vec3 partTop = rocketBottom + rocketDir * (adjusted_stack_y + part_h_3d);

          if (def.category == CAT_NOSE_CONE) {
            // Lower 40%: cylinder body flush with part below
            float bodyFrac = 0.4f;
            Vec3 bodyCenter = partBot + rocketDir * (part_h_3d * bodyFrac * 0.5f);
            Mat4 bodyMdl = Mat4::TRS(bodyCenter, rocketQuat, Vec3(part_w_3d, part_h_3d * bodyFrac, part_w_3d));
            r3d->drawMesh(rocketBody, bodyMdl, def.r * 0.95f, def.g * 0.95f, def.b * 0.95f, 1.0f, 0.25f);
            // Upper 60%: cone from body top to part top
            float coneFrac = 1.0f - bodyFrac;
            Vec3 coneStart = partBot + rocketDir * (part_h_3d * bodyFrac);
            Mat4 coneMdl = Mat4::TRS(coneStart, rocketQuat, Vec3(part_w_3d, part_h_3d * coneFrac, part_w_3d));
            r3d->drawMesh(rocketNose, coneMdl, def.r, def.g, def.b, 1.0f, 0.25f);
          } else if (def.category == CAT_ENGINE) {
            // Upper 40%: cylinder body flush with part above
            float bodyFrac = 0.4f;
            Vec3 bodyCenter = partTop - rocketDir * (part_h_3d * bodyFrac * 0.5f);
            Mat4 bodyMdl = Mat4::TRS(bodyCenter, rocketQuat, Vec3(part_w_3d * 0.55f, part_h_3d * bodyFrac, part_w_3d * 0.55f));
            r3d->drawMesh(rocketBody, bodyMdl, 0.18f, 0.18f, 0.20f, 1.0f, 0.4f);
            // Lower 60%: bell nozzle (cone mesh base at y=0, tip at y=h)
            // To flare out downwards, we place it at partBot and it grows up to the body.
            float nozzleFrac = 1.0f - bodyFrac;
            Mat4 nozzleMdl = Mat4::TRS(partBot, rocketQuat, Vec3(part_w_3d * 0.85f, part_h_3d * nozzleFrac, part_w_3d * 0.85f));
            r3d->drawMesh(rocketNose, nozzleMdl, def.r, def.g, def.b, 1.0f, 0.4f);
            engNozzlePos = partBot; // update for flame rendering
          } else if (def.category == CAT_STRUCTURAL && def.drag_coeff < 0) {
            // Fin set: 4 fins around this part (no body needed)
            for (int fi = 0; fi < 4; fi++) {
              float fin_angle = fi * 1.570796f;
              Quat finRot = rocketQuat * Quat::fromAxisAngle(Vec3(0.0f, 1.0f, 0.0f), fin_angle);
              Vec3 finPos = partPos + finRot.rotate(Vec3(part_w_3d * 1.2f, 0.0f, 0.0f));
              Mat4 finMdl = Mat4::TRS(finPos, finRot, Vec3(part_w_3d * 1.5f, part_h_3d * 0.8f, part_w_3d * 0.05f));
              r3d->drawMesh(rocketBox, finMdl, def.r, def.g, def.b, 1.0f, 0.2f);
            }
          } else if (def.category == CAT_BOOSTER) {
            // Full-height body
            Mat4 bstBody = Mat4::TRS(partPos, rocketQuat, Vec3(part_w_3d * 0.7f, part_h_3d, part_w_3d * 0.7f));
            r3d->drawMesh(rocketBody, bstBody, def.r, def.g, def.b, 1.0f, 0.25f);
            for (int si = 0; si < 2; si++) {
              float side_angle = si * 3.14159f;
              Quat sideRot = rocketQuat * Quat::fromAxisAngle(Vec3(0.0f, 1.0f, 0.0f), side_angle);
              Vec3 sidePos = partPos + sideRot.rotate(Vec3(part_w_3d * 0.6f, 0.0f, 0.0f));
              Mat4 sideMdl = Mat4::TRS(sidePos, rocketQuat, Vec3(part_w_3d * 0.25f, part_h_3d * 0.8f, part_w_3d * 0.25f));
              r3d->drawMesh(rocketBody, sideMdl, def.r * 0.8f, def.g * 0.8f, def.b * 0.8f, 1.0f, 0.3f);
            }
          } else if (def.category == CAT_COMMAND_POD) {
            // Lower 50%: cylinder body
            float bodyFrac = 0.5f;
            Vec3 bodyCenter = partBot + rocketDir * (part_h_3d * bodyFrac * 0.5f);
            Mat4 bodyMdl = Mat4::TRS(bodyCenter, rocketQuat, Vec3(part_w_3d, part_h_3d * bodyFrac, part_w_3d));
            r3d->drawMesh(rocketBody, bodyMdl, def.r, def.g, def.b, 1.0f, 0.3f);
            // Upper 50%: cone top
            float coneFrac = 1.0f - bodyFrac;
            Vec3 coneStart = partBot + rocketDir * (part_h_3d * bodyFrac);
            Mat4 coneMdl = Mat4::TRS(coneStart, rocketQuat, Vec3(part_w_3d * 0.9f, part_h_3d * coneFrac, part_w_3d * 0.9f));
            r3d->drawMesh(rocketNose, coneMdl, def.r * 1.1f, def.g * 1.1f, def.b * 1.1f, 1.0f, 0.3f);
          } else if (def.category == CAT_STRUCTURAL && def.drag_coeff >= 0) {
            // Decoupler/adapter
            Mat4 decMdl = Mat4::TRS(partPos, rocketQuat, Vec3(part_w_3d * 1.05f, part_h_3d, part_w_3d * 1.05f));
            r3d->drawMesh(rocketBody, decMdl, def.r, def.g, def.b, 1.0f, 0.4f);
          } else {
            // Default: fuel tank — full-height cylinder + inter-stage rings
            Mat4 tankMdl = Mat4::TRS(partPos, rocketQuat, Vec3(part_w_3d, part_h_3d, part_w_3d));
            r3d->drawMesh(rocketBody, tankMdl, def.r, def.g, def.b, 1.0f, 0.25f);
            Vec3 ringTopP = partTop;
            Mat4 ringTopMdl = Mat4::TRS(ringTopP, rocketQuat, Vec3(part_w_3d * 1.02f, part_h_3d * 0.02f, part_w_3d * 1.02f));
            r3d->drawMesh(rocketBody, ringTopMdl, 0.2f, 0.2f, 0.25f, 1.0f, 0.4f);
            Vec3 ringBotP = partBot;
            Mat4 ringBotMdl = Mat4::TRS(ringBotP, rocketQuat, Vec3(part_w_3d * 1.02f, part_h_3d * 0.02f, part_w_3d * 1.02f));
            r3d->drawMesh(rocketBody, ringBotMdl, 0.2f, 0.2f, 0.25f, 1.0f, 0.4f);
          }
        }
      }

      // ===== 发射台渲染 (Launch Pad Generation / Rendering) =====
      if (rocket_state.altitude < 2000.0 && current_soi_index == 3) {
          CelestialBody& earth = SOLAR_SYSTEM[3];
          
          // Launch pad should rotate with the Earth
          double theta = earth.prime_meridian_epoch + (rocket_state.sim_time * 2.0 * PI / earth.rotation_period);
          double cos_t = std::cos(theta);
          double sin_t = std::sin(theta);
          
          // Original surface position (0, R, 0)
          double s_px = 0, s_py = earth.radius, s_pz = 0;
          // Rotate to inertial
          double i_px = s_px * cos_t - s_py * sin_t;
          double i_py = s_px * sin_t + s_py * cos_t;
          double i_pz = s_pz;

          Vec3 padCenter(
              (float)((earth.px + i_px) * ws_d - ro_x), 
              (float)((earth.py + i_py) * ws_d - ro_y), 
              (float)((earth.pz + i_pz) * ws_d - ro_z)
          );
          
          // Calculate local up vector at pad
          Vec3 padUp((float)(i_px / earth.radius), (float)(i_py / earth.radius), (float)(i_pz / earth.radius));
          // Calculate a local right/forward to orient the pad (perpendicular to padUp in plane)
          Vec3 padRight( (float)-cos_t, (float)-sin_t, 0.0f );
          Vec3 padForward = padUp.cross(padRight);
          
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

      // ===== 工业级 3D 体积尾焰 (Volumetric Raymarched Plume) =====
      if (rocket_state.thrust_power > 0.1) {
        float thrust = (float)control_input.throttle;
        // 大气压力膨胀系数: 高空膨胀 (Expansion) = 1.0 - Density_Ratio
        float expansion = (float)fmax(0.0, 1.0 - PhysicsSystem::get_air_density(rocket_state.altitude) / 1.225);
        
        // 尾焰尺寸：提高节流阀敏感度，增加动态比例变化
        // 非线性映射让长度随推力更剧烈地改变 (0.1 推力时只有初始长度的 25%)
        float thrust_scale = 0.25f + 0.75f * powf(thrust, 1.5f);
        float plume_len = rh * 4.2f * thrust_scale * (1.0f + expansion * 1.0f);
        float plume_dia = rw_3d * 6.5f * (1.1f + expansion * 4.2f);
        
        // 动态增加包围盒宽度：当尾焰撞击地面时，必须增加包围盒直径以允许横向“飞溅”渲染，否则会被裁剪成方形
        float ground_contact_depth = fmaxf(0.0f, plume_len - (float)rocket_state.altitude * (float)ws_d);
        float splash_factor = ground_contact_depth / fmaxf(0.001f, plume_len);
        plume_dia *= (1.0f + splash_factor * 12.0f); // 显著增加包围盒宽度以解除裁剪限制
        
        // 尾焰渲染锚点：从发动机喷口向后延伸
        Vec3 plumePos = engNozzlePos - rocketDir * (plume_len * 0.5f);
        Mat4 plumeMdl = Mat4::TRS(plumePos, rocketQuat, Vec3(plume_dia, plume_len, plume_dia));
        
        // 传递地面高度：rocket_state.altitude 是相对于地面的高度
        // 我们假设 engNozzlePos 相对地面的距离近似为 altitude
        float groundDist = (float)rocket_state.altitude * (float)ws_d;
        r3d->drawExhaustVolumetric(rocketBox, plumeMdl, thrust, expansion, (float)glfwGetTime(), groundDist, plume_len);
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

    // --- 1. 速度计 (Velocity Gauge) ---
    float max_gauge_vel = 3000.0f;
    float vel_ratio = (float)min(1.0, current_vel / max_gauge_vel);
    renderer->addRect(gauge_vel_x, gauge_y_center, gauge_w * 1.2f,
                      gauge_h + 0.02f, 0.1f, 0.1f, 0.1f, 0.5f * hud_opacity);
    float r_vel = vel_ratio * 2.0f;
    if (r_vel > 1.0f)
      r_vel = 1.0f;
    float g_vel = (1.0f - vel_ratio) * 2.0f;
    if (g_vel > 1.0f)
      g_vel = 1.0f;
    float fill_h_vel = gauge_h * vel_ratio;
    renderer->addRect(
        gauge_vel_x, gauge_y_center - (gauge_h - fill_h_vel) / 2.0f,
        gauge_w * 0.8f, fill_h_vel, r_vel, g_vel, 0.0f, hud_opacity);

    // --- 2. 高度计 (Altitude Gauge) ---
    float max_gauge_alt = 100000.0f;
    float alt_ratio = (float)min(1.0, current_alt / max_gauge_alt);
    renderer->addRect(gauge_alt_x, gauge_y_center, gauge_w * 1.2f,
                      gauge_h + 0.02f, 0.1f, 0.1f, 0.1f, 0.5f * hud_opacity);
    float fill_h_alt = gauge_h * alt_ratio;
    renderer->addRect(gauge_alt_x,
                      gauge_y_center - (gauge_h - fill_h_alt) / 2.0f,
                      gauge_w * 0.8f, fill_h_alt, alt_ratio * 0.8f,
                      0.3f + alt_ratio * 0.7f, 1.0f, hud_opacity);

    // --- 3. 燃油计 (Fuel Gauge) ---
    double current_fuel = rocket_state.fuel;
    float max_gauge_fuel = 100000.0f; // 对应火箭初始化时的 100 吨满载燃料
    float fuel_ratio = (float)max(0.0, min(1.0, current_fuel / max_gauge_fuel));

    // 背景槽
    renderer->addRect(gauge_fuel_x, gauge_y_center, gauge_w * 1.2f,
                      gauge_h + 0.02f, 0.1f, 0.1f, 0.1f, 0.5f * hud_opacity);

    // 动态颜色：满燃料为绿，耗尽前变红
    float r_fuel = (1.0f - fuel_ratio) * 2.0f;
    if (r_fuel > 1.0f)
      r_fuel = 1.0f;
    float g_fuel = fuel_ratio * 2.0f;
    if (g_fuel > 1.0f)
      g_fuel = 1.0f;

    float fill_h_fuel = gauge_h * fuel_ratio;
    renderer->addRect(
        gauge_fuel_x, gauge_y_center - (gauge_h - fill_h_fuel) / 2.0f,
        gauge_w * 0.8f, fill_h_fuel, r_fuel, g_fuel, 0.1f, hud_opacity);

    // --- 4. 装饰性刻度线与标签框 ---
    int num_ticks = 10;
    for (int i = 0; i <= num_ticks; i++) {
      float tick_ratio = (float)i / num_ticks;
      float tick_y = (gauge_y_center - gauge_h / 2.0f) + gauge_h * tick_ratio;
      float tick_w = 0.02f;
      float tick_h = 0.003f;
      float alpha = (i % 5 == 0) ? 0.8f : 0.4f;
      if (i % 5 == 0)
        tick_w *= 1.5f;

      renderer->addRect(gauge_vel_x - gauge_w, tick_y, tick_w, tick_h, 1.0f,
                        1.0f, 1.0f, alpha * hud_opacity);
      renderer->addRect(gauge_alt_x + gauge_w, tick_y, tick_w, tick_h, 1.0f,
                        1.0f, 1.0f, alpha * hud_opacity);
      renderer->addRect(gauge_fuel_x + gauge_w, tick_y, tick_w, tick_h, 1.0f,
                        1.0f, 1.0f, alpha * hud_opacity);
    }

    // 底部颜色标签框
    renderer->addRect(gauge_vel_x, gauge_y_center - gauge_h / 2.0f - 0.03f,
                      gauge_w * 2.0f, 0.03f, 0.8f, 0.2f, 0.2f,
                      hud_opacity); // 红: VEL
    renderer->addRect(gauge_alt_x, gauge_y_center - gauge_h / 2.0f - 0.03f,
                      gauge_w * 2.0f, 0.03f, 0.2f, 0.5f, 0.9f,
                      hud_opacity); // 蓝: ALT
    renderer->addRect(gauge_fuel_x, gauge_y_center - gauge_h / 2.0f - 0.03f,
                      gauge_w * 2.0f, 0.03f, 0.9f, 0.6f, 0.1f,
                      hud_opacity); // 橙: FUEL
    // --- Telemetry Readings (Right Side - Final Refinement) ---
    float num_size = 0.025f; // Even smaller
    float num_x = 0.85f;     // Far right
    float label_x = num_x + 0.065f; 
    float bg_w = 0.22f;
    float bg_h = 0.05f;

    // 速度读数 + m/s
    renderer->addRect(num_x, 0.7f, bg_w, bg_h, 0.0f, 0.0f, 0.0f, 0.5f);
    renderer->drawInt(num_x + 0.05f, 0.7f, (int)current_vel, num_size, 1.0f, 0.4f, 0.4f, hud_opacity, Renderer::RIGHT);
    renderer->drawText(label_x, 0.7f, "m/s", num_size * 0.7f, 1.0f, 0.6f, 0.6f, hud_opacity);

    // 海拔读数 + m 或 km
    renderer->addRect(num_x, 0.55f, bg_w, bg_h, 0.0f, 0.0f, 0.0f, 0.5f);
    if (current_alt > 10000) {
      renderer->drawInt(num_x + 0.05f, 0.55f, (int)(current_alt / 1000.0), num_size, 0.4f, 0.7f, 1.0f, hud_opacity, Renderer::RIGHT);
      renderer->drawText(label_x, 0.55f, "km", num_size * 0.7f, 0.5f, 0.8f, 1.0f, hud_opacity);
    } else {
      renderer->drawInt(num_x + 0.05f, 0.55f, (int)current_alt, num_size, 0.4f, 0.7f, 1.0f, hud_opacity, Renderer::RIGHT);
      renderer->drawText(label_x, 0.55f, "m", num_size * 0.7f, 0.5f, 0.8f, 1.0f, hud_opacity);
    }

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

    // --- 7. Mission Status (Top Center) ---
    renderer->drawText(0.0f, 0.85f, "[MISSION CONTROL]", 0.016f, 0.4f, 1.0f, 0.4f, hud_opacity, true, Renderer::CENTER);
    renderer->drawText(0.0f, 0.80f, rocket_state.mission_msg.c_str(), 0.015f, 0.8f, 0.8f, 1.0f, hud_opacity, true, Renderer::CENTER);

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
        double mu = 6.67430e-11 * SOLAR_SYSTEM[current_soi_index].mass;
        double npx, npy, npz, nvx, nvy, nvz;
        
        // Project CURRENT state to node time
        get3DStateAtTime(rocket_state.px, rocket_state.py, rocket_state.pz, rocket_state.vx, rocket_state.vy, rocket_state.vz, mu, node.sim_time - rocket_state.sim_time, npx, npy, npz, nvx, nvy, nvz);
        ManeuverFrame frame = ManeuverSystem::getFrame(Vec3((float)npx, (float)npy, (float)npz), Vec3((float)nvx, (float)nvy, (float)nvz));
        
        // The delta-v vector required in world space
        Vec3 target_dv_world = frame.prograde * node.delta_v.x + frame.normal * node.delta_v.y + frame.radial * node.delta_v.z;
        dv_remaining = (double)target_dv_world.length();
        if (dv_remaining > 0.05) vManeuver = target_dv_world.normalized();
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
            renderer->drawText(menu_x - 0.12f, bd_y, "Ref Body:", 0.012f, 1, 1, 1, 1.0f);
            bool hover_bd_l = (hmouse_x >= menu_x - 0.01f && hmouse_x <= menu_x + 0.03f && hmouse_y >= bd_y - 0.02f && hmouse_y <= bd_y + 0.02f);
            bool hover_bd_r = (hmouse_x >= menu_x + 0.11f && hmouse_x <= menu_x + 0.15f && hmouse_y >= bd_y - 0.02f && hmouse_y <= bd_y + 0.02f);
            if (hover_bd_l && hlmb && !hlmb_prev) adv_orbit_ref_body = (adv_orbit_ref_body-1 < 0) ? (int)SOLAR_SYSTEM.size()-1 : adv_orbit_ref_body-1;
            if (hover_bd_r && hlmb && !hlmb_prev) adv_orbit_ref_body = (adv_orbit_ref_body+1) % (int)SOLAR_SYSTEM.size();
            renderer->drawText(menu_x + 0.01f, bd_y, "<", 0.012f, 1,1,1, hover_bd_l?1.0f:0.5f, true, Renderer::CENTER);
            if (!SOLAR_SYSTEM.empty()) {
                renderer->drawText(menu_x + 0.07f, bd_y, SOLAR_SYSTEM[adv_orbit_ref_body].name.c_str(), 0.010f, 1,1,1,1, true, Renderer::CENTER);
            }
            renderer->drawText(menu_x + 0.13f, bd_y, ">", 0.012f, 1,1,1, hover_bd_r?1.0f:0.5f, true, Renderer::CENTER);

            // --- Prediction Time Slider ---
            float pred_y = bd_y - 0.06f;
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
                if (rocket_state.maneuvers[0].burn_mode == 1) {
                    // In sustained mode, calculate duration to find start time
                    double dv_val = rocket_state.maneuvers[0].delta_v.length();
                    double m0 = rocket_state.fuel + rocket_config.dry_mass + rocket_config.upper_stages_mass;
                    double ve = rocket_config.specific_impulse * 9.80665;
                    double duration = 0;
                    if (ve > 0 && rocket_config.cosrate > 0 && dv_val > 0) duration = m0 * (1.0 - 1.0/exp(dv_val/ve)) / rocket_config.cosrate;
                    target_t -= 0.5 * duration;
                }
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
                snprintf(buf, sizeof(buf), "%s: %dH %02dM %02dS", (mnv_popup_burn_mode == 1 ? "T-START" : "T-NODE"), t_hr, t_min, t_sec);
            } else if (mnv_popup_time_to_node >= 0) {
                snprintf(buf, sizeof(buf), "%s: %dM %02dS", (mnv_popup_burn_mode == 1 ? "T-START" : "T-NODE"), t_min, t_sec);
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
  
  // 游戏结束时保存最终状态
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
}