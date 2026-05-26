// Build optimization test
#ifdef USE_VULKAN
// Vulkan 相关宏必须在 GLFW 第一次 include 之前定义
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#define GLFW_INCLUDE_VULKAN
#endif

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

#ifdef USE_VULKAN
// vendor 目录已在 include 路径中（vcxproj: ..\vendor）
#include "vma/vk_mem_alloc.h"
#include "render/vulkan/vk_context.h"
#include "render/vulkan/vk_frame.h"
#include "render/vulkan/vk_mesh.h"
#include "render/vulkan/vk_descriptors.h"
#include "render/vulkan/vk_renderpass.h"
#include "render/vulkan/vk_pipeline.h"
#include "render/vulkan/vk_taa.h"
#include "render/vulkan/vk_texture.h"
#include "render/vulkan/vk_renderer3d.h"
#include "render/vulkan/systems/vk_hud.h"
// ImGui Vulkan 仪表板
#include "render/vulkan/vk_imgui.h"
#include "render/vulkan/systems/vk_game_ui.h"
#include "render/scene_snapshot.h"
static VulkanContext       g_vkCtx;
static FrameSync           g_frameSync;
static VkDescriptorManager g_vkDesc;
static VkTAA               g_vkTAA;
static VkRenderer3D        g_vkR3D;
static VkHUDSystem         g_vkHUD;
static VkImGuiSystem       g_vkImGui;
static VkGameUI            g_gameUI;
static FlightScene*        g_flightScene = nullptr;
static Terrain::QuadtreeTerrain* g_vkTerrain = nullptr;
static bool                g_vkSwapchainDirty = false;
static int                 warmup = 0;  // TAA 预热帧计数（飞行开始时重置）
static bool                g_workshopMeshesReady = false;

// skipBuilder=true  → 从 loaded_rocket_state 恢复存档，全景相机模式
// skipBuilder=false → 从 GameContext::launch_assembly 组装发射，地面发射台
static bool initVulkanFlight(bool skipBuilder = false) {
    delete g_flightScene;
    g_flightScene = new FlightScene();
    GameContext& gc = GameContext::getInstance();
    gc.skip_builder = skipBuilder;
    gc.vkRenderer3d = &g_vkR3D;

    if (skipBuilder) {
        // 地表初始位置（地球表面 + 10m，防止被地形裁剪）
        constexpr double ALT = 10.0;
        CelestialBody& earth = UniverseModel::getInstance().solar_system[3];
        RocketState& rs = gc.loaded_rocket_state;
        rs.abs_px = earth.px; rs.abs_py = earth.py + EARTH_RADIUS + ALT; rs.abs_pz = earth.pz;
        rs.px = 0.0; rs.py = EARTH_RADIUS + ALT; rs.pz = 0.0;
        rs.surf_px = 0.0; rs.surf_py = EARTH_RADIUS + ALT; rs.surf_pz = 0.0;
        rs.altitude = ALT; rs.fuel = 50000.0;
        rs.launch_site_px = rs.surf_px;
        rs.launch_site_py = rs.surf_py;
        rs.launch_site_pz = rs.surf_pz;
    }

    g_flightScene->onEnter();

    if (skipBuilder) {
        // 全景相机，朝向太阳方向
        CelestialBody& earth = UniverseModel::getInstance().solar_system[3];
        g_flightScene->cam.mode         = CameraDirector::PANORAMA;
        g_flightScene->cam.focus_target = 3;
        Vec3 sunDir(-(float)earth.px, -(float)earth.py, -(float)earth.pz);
        sunDir = sunDir.normalized();
        Vec3 zAxis(0,0,1);
        Vec3 rotAxis = zAxis.cross(sunDir);
        float dotVal = zAxis.dot(sunDir);
        if (rotAxis.length() < 1e-6f) {
            g_flightScene->cam.quat = (dotVal > 0) ? Quat() : Quat::fromAxisAngle(Vec3(0,1,0), 3.141592653589793f);
        } else {
            rotAxis = rotAxis.normalized();
            g_flightScene->cam.quat = Quat::fromAxisAngle(rotAxis, acosf(dotVal));
        }
    } else {
        // 发射台相机：聚焦火箭
        g_flightScene->cam.mode = CameraDirector::CHASE;
    }

    // 将 CPU 网格数据注册到 Vulkan
    auto reg = [](const char* id, const Mesh& m) -> bool {
        if (m.cpuIndices.empty()) return true;
        return g_vkR3D.registerMesh(g_vkCtx, id,
            m.cpuVerts.data(),   m.cpuVerts.size() * sizeof(Vertex3D),
            m.cpuIndices.data(), (uint32_t)m.cpuIndices.size());
    };
    bool ok = reg("earth",      g_flightScene->earthMesh)
           && reg("ring",       g_flightScene->ringMesh)
           && reg("rocketBody", g_flightScene->rocketBody)
           && reg("rocketNose", g_flightScene->rocketNose)
           && reg("rocketBox",  g_flightScene->rocketBox);
    if (!ok) return false;

    // ---- 外部 OBJ 模型加载 ----
    // 发射台 OBJ
    {
        g_flightScene->launchPadMesh = ModelLoader::loadOBJ("assets/launch_pad.obj");
        g_flightScene->hasLaunchPadOBJ = !g_flightScene->launchPadMesh.cpuIndices.empty();
        if (g_flightScene->hasLaunchPadOBJ) {
            auto& m = g_flightScene->launchPadMesh;
            if (!g_vkR3D.registerMesh(g_vkCtx, "launchPad",
                    m.cpuVerts.data(), m.cpuVerts.size() * sizeof(Vertex3D),
                    m.cpuIndices.data(), (uint32_t)m.cpuIndices.size())) {
                g_flightScene->hasLaunchPadOBJ = false;
                fprintf(stderr, "[Vulkan] Failed to register launchPad OBJ\n");
            } else {
                printf("[Vulkan] Loaded launch_pad.obj (%zu verts)\n", m.cpuVerts.size());
            }
        }
    }
    // 零件 OBJ 模型（预加载 PART_CATALOG 全部条目，供飞行 + 工坊共享）
    {
        auto resolveObjPath = [](const PartDef& def) -> std::string {
            if (def.model_path) return def.model_path;
            std::string tmp = def.name;
            for (char& c : tmp) { c=(char)std::tolower((unsigned char)c); if(c==' ')c='_'; }
            return "assets/models/" + tmp + ".obj";
        };
        for (int i = 0; i < PART_CATALOG_SIZE; i++) {
            const PartDef& def = PART_CATALOG[i];
            std::string path = resolveObjPath(def);
            if (g_flightScene->partObjMeshes.count(path)) continue;
            Mesh m = ModelLoader::loadOBJ(path);
            if (!m.cpuIndices.empty()) {
                if (g_vkR3D.registerMesh(g_vkCtx, path.c_str(),
                        m.cpuVerts.data(), m.cpuVerts.size() * sizeof(Vertex3D),
                        m.cpuIndices.data(), (uint32_t)m.cpuIndices.size())) {
                    printf("[Vulkan] Loaded OBJ part: %s (%zu verts)\n", path.c_str(), m.cpuVerts.size());
                }
                g_flightScene->partObjMeshes[path] = std::move(m);
            } else {
                g_flightScene->partObjMeshes[path] = Mesh();  // 标记为已尝试但未找到
            }
        }
    }

    // ---- 地形四叉树 + 纹理上传（只在首次飞行时创建）----
    if (!g_vkTerrain) {
        g_vkTerrain = new Terrain::QuadtreeTerrain(EARTH_RADIUS);

        // 打包构造纹理数据
        auto* tecSim = g_vkTerrain->sim;
        auto* hydSim = g_vkTerrain->hydroSim;
        const int tw = tecSim->width, th = tecSim->height;
        const int hw = hydSim->width, hh = hydSim->height;

        // Tectonic: RGBA8（高度值 → R 通道，[0,1] → [0,255]）
        std::vector<uint8_t> tecRGBA8(tw * th * 4);
        for (int i = 0; i < tw * th; i++) {
            uint8_t r = (uint8_t)(std::min(1.0f, std::max(0.0f, tecSim->gridHeight[i])) * 255.f);
            tecRGBA8[i * 4 + 0] = r;
            tecRGBA8[i * 4 + 1] = 0;
            tecRGBA8[i * 4 + 2] = 0;
            tecRGBA8[i * 4 + 3] = 255;
        }

        // Hydro: RGBA32F — R:filledHeight, G:accumulation, B:strahler(raw), A:flowDir
        std::vector<float> hydFloat4(hw * hh * 4);
        for (int i = 0; i < hw * hh; i++) {
            hydFloat4[i * 4 + 0] = hydSim->data.filledHeight[i];
            hydFloat4[i * 4 + 1] = hydSim->data.accumulation[i];
            hydFloat4[i * 4 + 2] = (float)hydSim->data.strahler[i];
            hydFloat4[i * 4 + 3] = (float)hydSim->data.flowDir[i];
        }

        if (!g_vkR3D.scene.uploadTerrainTextures(g_vkCtx,
                tecRGBA8.data(), (uint32_t)tw, (uint32_t)th,
                hydFloat4.data(), (uint32_t)hw, (uint32_t)hh)) {
            fprintf(stderr, "[Vulkan] Failed to upload terrain textures\n");
        } else {
            printf("[Vulkan] Terrain textures uploaded (%dx%d tectonic, %dx%d hydro)\n",
                tw, th, hw, hh);
        }
    }
    gc.terrain = g_vkTerrain;

    return true;
}

static void renderFlightSceneVulkan(VkCommandBuffer cmd, int frameSlot) {
    SceneSnapshot snap = g_flightScene->extractRenderSnapshot();
    g_vkTAA.updateMatrices(snap.view, snap.proj);
    g_vkTAA.uploadMatrices(frameSlot);
    g_vkR3D.render(cmd, 0, snap, g_vkCtx.swapExtent);
}

static void handleVkSwapchainResize(GLFWwindow* window) {
    g_vkCtx.recreateSwapchain(window);
    g_vkTAA.recreate(g_vkCtx, g_vkCtx.swapExtent, g_vkCtx.swapFormat,
        "src/render/shaders/spirv/taa.vert.spv",
        "src/render/shaders/spirv/taa.frag.spv");
    g_vkSwapchainDirty = false;
}

static void renderVulkanFrame(GLFWwindow* window) {
    if (g_vkSwapchainDirty) { handleVkSwapchainResize(window); return; }

    auto& frame = g_frameSync.current();
    VkSemaphore acquireSem = g_frameSync.nextAcquireSem();

    vkWaitForFences(g_vkCtx.device, 1, &frame.inFlightFence, VK_TRUE, UINT64_MAX);

    uint32_t imageIdx = 0;
    VkResult result = vkAcquireNextImageKHR(g_vkCtx.device, g_vkCtx.swapchain,
        UINT64_MAX, acquireSem, VK_NULL_HANDLE, &imageIdx);
    if (result == VK_ERROR_OUT_OF_DATE_KHR || result == VK_SUBOPTIMAL_KHR) {
        handleVkSwapchainResize(window); return;
    }
    if (result != VK_SUCCESS) return;

    VkSemaphore renderSem = g_frameSync.renderSems[imageIdx];

    vkResetFences(g_vkCtx.device, 1, &frame.inFlightFence);
    vkResetCommandBuffer(frame.commandBuffer, 0);

    VkCommandBufferBeginInfo beginInfo{ VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO };
    beginInfo.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
    vkBeginCommandBuffer(frame.commandBuffer, &beginInfo);

    int ww = (int)g_vkCtx.swapExtent.width;
    int wh = (int)g_vkCtx.swapExtent.height;

    // Pass 1 & 2: 几何 + TAA resolve
    bool inFlight   = (g_gameUI.state == VkGameState::FLIGHT) && (g_flightScene != nullptr);
    bool inWorkshop = (g_gameUI.state == VkGameState::WORKSHOP) && g_workshopMeshesReady;
    g_vkTAA.beginGeometryPass(frame.commandBuffer, g_vkCtx.swapExtent);
    VkFrameRenderer::setViewportScissor(frame.commandBuffer, g_vkCtx.swapExtent);
    if (inFlight) {
        renderFlightSceneVulkan(frame.commandBuffer, g_frameSync.currentFrame);
    } else if (inWorkshop) {
        SceneSnapshot snap = g_gameUI.buildWorkshopSnapshot(ww, wh, g_flightScene);
        g_vkTAA.updateMatrices(snap.view, snap.proj);
        g_vkTAA.uploadMatrices(g_frameSync.currentFrame);
        g_vkR3D.render(frame.commandBuffer, 0, snap, g_vkCtx.swapExtent);
    }
    g_vkTAA.endGeometryPass(frame.commandBuffer);

    g_vkTAA.beginResolvePass(frame.commandBuffer,
        g_vkCtx.swapImages[imageIdx],
        g_vkCtx.swapImageViews[imageIdx],
        g_vkCtx.swapExtent);
    VkFrameRenderer::setViewportScissor(frame.commandBuffer, g_vkCtx.swapExtent);
    float taaBlend = (++warmup < 60) ? 1.0f : 0.1f;
    g_vkTAA.drawResolve(frame.commandBuffer, taaBlend);
    g_vkTAA.endResolvePass(frame.commandBuffer, g_vkCtx.swapImages[imageIdx]);
    // swapchain 现在处于 PRESENT_SRC_KHR

    // Pass 3: ImGui UI / HUD（覆盖在 TAA 结果之上）
    g_vkImGui.newFrame();
    g_gameUI.draw(inFlight ? g_flightScene : nullptr, ww, wh);
    ImGui::Render();
    g_vkImGui.renderToSwapchain(frame.commandBuffer,
        g_vkCtx.swapImages[imageIdx],
        g_vkCtx.swapImageViews[imageIdx],
        g_vkCtx.swapExtent);
    // swapchain 已由 renderToSwapchain 转回 PRESENT_SRC_KHR

    vkEndCommandBuffer(frame.commandBuffer);

    // 提交（Vulkan 1.3 vkQueueSubmit2）
    VkCommandBufferSubmitInfo cmdInfo{ VK_STRUCTURE_TYPE_COMMAND_BUFFER_SUBMIT_INFO };
    cmdInfo.commandBuffer = frame.commandBuffer;

    VkSemaphoreSubmitInfo waitInfo{ VK_STRUCTURE_TYPE_SEMAPHORE_SUBMIT_INFO };
    waitInfo.semaphore = acquireSem;
    waitInfo.stageMask = VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT;

    VkSemaphoreSubmitInfo signalInfo{ VK_STRUCTURE_TYPE_SEMAPHORE_SUBMIT_INFO };
    signalInfo.semaphore = renderSem;
    signalInfo.stageMask = VK_PIPELINE_STAGE_2_ALL_COMMANDS_BIT;

    VkSubmitInfo2 submitInfo2{ VK_STRUCTURE_TYPE_SUBMIT_INFO_2 };
    submitInfo2.waitSemaphoreInfoCount   = 1;
    submitInfo2.pWaitSemaphoreInfos      = &waitInfo;
    submitInfo2.commandBufferInfoCount   = 1;
    submitInfo2.pCommandBufferInfos      = &cmdInfo;
    submitInfo2.signalSemaphoreInfoCount = 1;
    submitInfo2.pSignalSemaphoreInfos    = &signalInfo;

    vkQueueSubmit2(g_vkCtx.graphicsQueue, 1, &submitInfo2, frame.inFlightFence);

    VkPresentInfoKHR presentInfo{ VK_STRUCTURE_TYPE_PRESENT_INFO_KHR };
    presentInfo.waitSemaphoreCount = 1;
    presentInfo.pWaitSemaphores    = &renderSem;
    presentInfo.swapchainCount     = 1;
    presentInfo.pSwapchains        = &g_vkCtx.swapchain;
    presentInfo.pImageIndices      = &imageIdx;

    result = vkQueuePresentKHR(g_vkCtx.presentQueue, &presentInfo);
    if (result == VK_ERROR_OUT_OF_DATE_KHR || result == VK_SUBOPTIMAL_KHR)
        g_vkSwapchainDirty = true;

    g_vkTAA.swap();
    g_frameSync.advance();
}
#endif


// ==========================================
// Part 2: Explorer Class -> Replaced by ECS (RocketState, PhysicsSystem)
// ==========================================


void framebuffer_size_callback(GLFWwindow* /*window*/, int width, int height) {
#ifdef USE_VULKAN
  g_vkSwapchainDirty = true;
#else
  glViewport(0, 0, width, height);
#endif
}

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

#ifdef USE_VULKAN
  // Phase 5: 纯 Vulkan 窗口，不创建 OpenGL 上下文
  glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API);
#else
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
  glfwWindowHint(GLFW_SAMPLES, 4);
#endif

  GLFWwindow *window = glfwCreateWindow(1000, 800, "3D Rocket Sim", NULL, NULL);
  if (!window) {
    glfwTerminate();
    return -1;
  }

  glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);
  glfwSetScrollCallback(window, scroll_callback);

  // UniverseModel 初始化必须在 FlightScene::onEnter() 之前
  PhysicsSystem::InitSolarSystem();

#ifdef USE_VULKAN
  // 完整 Vulkan 初始化序列（Phase 5）
  if (!g_vkCtx.initCore()
   || !g_vkCtx.initSwapchain(window)
   || !g_frameSync.init(g_vkCtx.device, g_vkCtx.commandPool)
   || !g_vkDesc.init(g_vkCtx)
   || !g_vkR3D.init(g_vkCtx, g_vkDesc, VkTAA::kColorFmt, VkTAA::kDepthFmt)
   || !g_vkTAA.init(g_vkCtx, g_vkCtx.swapExtent, g_vkCtx.swapFormat,
                    "src/render/shaders/spirv/taa.vert.spv",
                    "src/render/shaders/spirv/taa.frag.spv")
   || !g_vkHUD.init(g_vkCtx, g_vkCtx.swapFormat)
   || !g_vkImGui.init(g_vkCtx, window, g_vkCtx.swapFormat)) {
    fprintf(stderr, "[Vulkan] Fatal: initialization failed\n");
    glfwTerminate();
    return -1;
  }
  // Vulkan 模式从主菜单启动（不立即进入飞行）
  g_gameUI.state = VkGameState::MENU;
#else
  glfwMakeContextCurrent(window);
  if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
    return -1;
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glEnable(GL_MULTISAMPLE);
#endif

#ifndef USE_VULKAN
  renderer = new Renderer();
#endif

  Simulation::AsyncOrbitPredictor orbit_predictor;
  orbit_predictor.Start();

  // =========================================================
  // SceneManager State Machine Loop
  // =========================================================
  GameContext& ctx = GameContext::getInstance();
  ctx.window = window;
#ifndef USE_VULKAN
  ctx.renderer2d = renderer;
#endif
  ctx.orbit_predictor = &orbit_predictor;

#ifndef USE_VULKAN
  SceneManager::getInstance().changeScene(std::make_unique<MenuScene>());
#endif

  double last_time = glfwGetTime();

  while (!glfwWindowShouldClose(window)) {
      glfwPollEvents();

      double current_time = glfwGetTime();
      double real_dt = current_time - last_time;
      last_time = current_time;
      if (real_dt > 0.1) real_dt = 0.02;

#ifndef USE_VULKAN
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
#endif

      // If window got closed by a scene
      if (glfwWindowShouldClose(window)) break;

#ifdef USE_VULKAN
      // --- 状态机转换 ---
      if (g_gameUI.goToExit) {
          glfwSetWindowShouldClose(window, true);
      }
      if (g_gameUI.goToAgency) {
          g_gameUI.goToAgency = false;
          g_gameUI.state = VkGameState::AGENCY;
      }
      if (g_gameUI.goToWorkshop) {
          g_gameUI.goToWorkshop = false;
          g_gameUI.beginWorkshop();
          // 注册基础网格（用于 Workshop 3D 预览）
          if (!g_workshopMeshesReady) {
              if (initVulkanFlight(true)) {
                  g_workshopMeshesReady = true;
              }
          }
          g_gameUI.state = VkGameState::WORKSHOP;
          warmup = 0;
      }
      if (g_gameUI.goToFlight) {
          g_gameUI.goToFlight = false;
          bool skip = GameContext::getInstance().skip_builder;
          if (!initVulkanFlight(skip)) {
              fprintf(stderr, "[Vulkan] initVulkanFlight failed\n");
          } else {
              g_workshopMeshesReady = true;  // Flight 初始化也注册了网格
              g_gameUI.state = VkGameState::FLIGHT;
              warmup = 0;  // 重置 TAA 预热
          }
      }
      // --- Workshop 输入更新（ImGui 帧之前）---
      if (g_gameUI.state == VkGameState::WORKSHOP) {
          g_gameUI.updateWorkshopInput(window);
          if (g_scroll_y != 0.0f) {
              g_gameUI.handleWorkshopScroll(g_scroll_y);
              g_scroll_y = 0.0f;
          }
      }
      // --- 场景更新（仅飞行状态） ---
      if (g_gameUI.state == VkGameState::FLIGHT && g_flightScene) {
          g_flightScene->update(real_dt);
          g_flightScene->render();  // ctx.update() → 相机矩阵，然后早退（USE_VULKAN 下）
      }
      renderVulkanFrame(window);
#else
      SceneManager::getInstance().render();
      glfwSwapBuffers(window);
#endif
  }

#ifndef USE_VULKAN
  delete renderer;
  if (ctx.renderer3d) delete ctx.renderer3d;
#endif
  orbit_predictor.Stop();

#ifdef USE_VULKAN
  vkDeviceWaitIdle(g_vkCtx.device);
  delete g_flightScene; g_flightScene = nullptr;
  delete g_vkTerrain;   g_vkTerrain   = nullptr;
  g_vkImGui.shutdown(g_vkCtx.device);
  g_vkR3D.shutdown(g_vkCtx);
  g_vkTAA.shutdown(g_vkCtx);
  g_vkHUD.shutdown(g_vkCtx);
  g_vkDesc.shutdown(g_vkCtx);
  g_frameSync.shutdown(g_vkCtx.device);
  g_vkCtx.shutdown();
#endif

  glfwTerminate();
  return 0;
}
