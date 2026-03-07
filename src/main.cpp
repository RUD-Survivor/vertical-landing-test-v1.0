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
      
      menu_choice = MenuSystem::HandleMenuInput(window, menu_state, up_pressed, down_pressed, enter_pressed);
      
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

    r3d->endFrame();

    // Render builder UI OVERLAY (clear depth buffer so 2D renders on top)
    glClear(GL_DEPTH_BUFFER_BIT);
    renderer->beginFrame();
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
    if (glfwGetKey(window, GLFW_KEY_R) == GLFW_PRESS) {
      if (!r_was_pressed) {
        orbit_reference_sun = !orbit_reference_sun;
        cout << "[REF FRAME] " << (orbit_reference_sun ? "SUN" : "EARTH") << endl;
        r_was_pressed = true;
      }
    } else {
      r_was_pressed = false;
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
    manual.throttle_up = glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS;
    manual.throttle_down = glfwGetKey(window, GLFW_KEY_LEFT_CONTROL) == GLFW_PRESS;
    manual.throttle_max = glfwGetKey(window, GLFW_KEY_Z) == GLFW_PRESS;
    manual.throttle_min = glfwGetKey(window, GLFW_KEY_X) == GLFW_PRESS;
    manual.pitch_left = glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS || glfwGetKey(window, GLFW_KEY_LEFT) == GLFW_PRESS;
    manual.pitch_right = glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS || glfwGetKey(window, GLFW_KEY_RIGHT) == GLFW_PRESS;
    manual.pitch_forward = glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS || glfwGetKey(window, GLFW_KEY_UP) == GLFW_PRESS;
    manual.pitch_backward = glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS || glfwGetKey(window, GLFW_KEY_DOWN) == GLFW_PRESS;

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
          else ControlSystem::UpdateManualControl(rocket_state, control_input, manual, dt);
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

    // ================= 3D 渲染通道 =================
    {
      // 世界坐标缩放
      double ws_d = 0.001;
      float earth_r = (float)EARTH_RADIUS * (float)ws_d;

      // 2D→3D 坐标映射 (Double precision) - Use absolute heliocentric coordinates!
      double r_px = rocket_state.abs_px * ws_d;
      double r_py = rocket_state.abs_py * ws_d;
      double r_pz = rocket_state.abs_pz * ws_d;
      
      // ===== 太阳位置与真实尺寸 =====
      CelestialBody& sun_body = SOLAR_SYSTEM[0];
      double sun_px = sun_body.px * ws_d;
      double sun_py = sun_body.py * ws_d;
      double sun_pz = sun_body.pz * ws_d;
      float sun_radius = (float)sun_body.radius * ws_d;
      double sun_dist_d = sqrt(r_px*r_px + r_py*r_py + r_pz*r_pz);

      // ===== 按键监听：视点切换 =====
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
      Vec3 rocketUp((float)(rocket_state.px / local_dist), (float)(rocket_state.py / local_dist), (float)(rocket_state.pz / local_dist));
      if (rocket_state.altitude > 2000000.0) { // 2000公里外，完全脱离近地轨道，进入深空
          rocketUp = Vec3(0.0f, 1.0f, 0.0f);
      }
      
      // ===== 构建火箭完整的 3D 朝向四元数 =====
      // 保证局部坐标系的单位正交
      double local_xy_mag = sqrt(rocketUp.x*rocketUp.x + rocketUp.y*rocketUp.y);
      Vec3 localRight(0,0,0);
      if (local_xy_mag > 1e-4) {
          localRight = Vec3((float)(-rocketUp.y/local_xy_mag), (float)(rocketUp.x/local_xy_mag), 0.0f);
      } else {
          localRight = Vec3(1.0f, 0.0f, 0.0f);
      }
      Vec3 localNorth = rocketUp.cross(localRight).normalized();
      
      // --- 计算用于 Orbit 摄像机的轨道参考系 (基于当前相对于 SOI 的坐标) ---
      Vec3 v_vec_rel((float)rocket_state.vx, (float)rocket_state.vy, (float)rocket_state.vz);
      Vec3 p_vec_rel((float)rocket_state.px, (float)rocket_state.py, (float)rocket_state.pz);
      Vec3 h_vec_rel = p_vec_rel.cross(v_vec_rel);
      Vec3 orbit_normal_rel = h_vec_rel.normalized();
      if (orbit_normal_rel.length() < 0.01f) orbit_normal_rel = Vec3(0, 0, 1);
      Vec3 prograde_rel = v_vec_rel.normalized();
      if (prograde_rel.length() < 0.01f) prograde_rel = localNorth;
      Vec3 radial_rel = orbit_normal_rel.cross(prograde_rel).normalized();

      // 构建 2D 面内主朝向
      float rocket_angle = (float)rocket_state.angle;
      Vec3 rocketDir2D = rocketUp * cosf(rocket_angle) + localRight * sinf(rocket_angle);

      // 构建对应的主旋转四元数
      Vec3 defaultUp(0.0f, 1.0f, 0.0f);
      Vec3 rotAxis = defaultUp.cross(rocketDir2D);
      float rotAngle = acosf(fminf(fmaxf(defaultUp.dot(rocketDir2D), -1.0f), 1.0f));
      Quat baseQuat;
      if (rotAxis.length() > 1e-6f)
        baseQuat = Quat::fromAxisAngle(rotAxis.normalized(), rotAngle);

      // 提取动态翻滚轴 (Rotated Right) 用作俯仰轴，防止 Gimbal Lock 导致 angle_z 偏航失效
      Vec3 dynamicRight = localNorth.cross(rocketDir2D).normalized();

      // 组合基础 2D 旋转与局部的轴外俯仰 (Pitch) 旋转
      Quat pitchQuat = Quat::fromAxisAngle(dynamicRight, (float)rocket_state.angle_z);
      Quat rocketQuat = pitchQuat * baseQuat;
      
      // 更新 rocketDir 为包含 3D 俯仰的真实朝向
      Vec3 rocketDir = rocketQuat.rotate(Vec3(0.0f, 1.0f, 0.0f));

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
      r3d->drawSkybox(sky_vibrancy);

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
              float ring_w = fmaxf(earth_r * 0.002f, ref_dist * 0.0015f);
              if (i == 4) ring_w *= 0.4f; // Moon orbit thinner
              
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
           float hist_w = earth_r * 0.003f * fmaxf(1.0f, cam_zoom_pan * 0.5f);
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
        float marker_size = fminf(earth_r * 0.1f, earth_r * 0.02f * fmaxf(1.0f, cam_zoom_pan * 0.8f));
        r3d->drawBillboard(renderRocketBase, marker_size, 0.2f, 1.0f, 0.4f, 0.9f);
      }

      // ===== 轨道预测线 (开普勒轨道) =====
      if (cam_mode_3d == 2) {
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
              }
              
              // 渲染预测轨迹
              float pred_w = earth_r * 0.0025f * fmaxf(1.0f, cam_zoom_pan * 0.5f);
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
              float pred_w = earth_r * 0.0025f * fmaxf(1.0f, cam_zoom_pan * 0.5f);
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
        
        // 尾焰渲染锚点：从发动机喷口向后延伸
        Vec3 plumePos = engNozzlePos - rocketDir * (plume_len * 0.5f);
        Mat4 plumeMdl = Mat4::TRS(plumePos, rocketQuat, Vec3(plume_dia, plume_len, plume_dia));
        
        r3d->drawExhaustVolumetric(rocketBox, plumeMdl, thrust, expansion, (float)glfwGetTime());
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


    // --- 6. 控制模式指示器 (HUD 右上角, 更加收缩) ---
    float mode_x = 0.88f; 
    float mode_y = 0.85f;
    float mode_w = 0.15f;
    float mode_h = 0.04f;
    renderer->addRect(mode_x, mode_y, mode_w, mode_h, 0.05f, 0.05f, 0.05f, 0.7f);
    if (rocket_state.auto_mode) {
      renderer->addRect(mode_x, mode_y, mode_w - 0.02f, mode_h - 0.015f, 0.1f, 0.8f, 0.2f, 0.9f);
      renderer->drawText(mode_x, mode_y, "AUTO", 0.020f, 0.1f, 0.1f, 0.1f, 0.9f, false, Renderer::CENTER);
    } else {
      renderer->addRect(mode_x, mode_y, mode_w - 0.02f, mode_h - 0.015f, 1.0f, 0.6f, 0.1f, 0.9f);
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

    // ========================================================================
    } // end if (show_hud)
    
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