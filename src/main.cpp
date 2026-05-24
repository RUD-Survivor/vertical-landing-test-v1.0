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
#include "render/scene_snapshot.h"
static VulkanContext       g_vkCtx;
static FrameSync           g_frameSync;
static VkDescriptorManager g_vkDesc;
static VkTAA               g_vkTAA;
static VkRenderer3D        g_vkR3D;
static VkHUDSystem         g_vkHUD;
static FlightScene*        g_flightScene = nullptr;
static bool                g_vkSwapchainDirty = false;

static bool initVulkanFlight() {
    g_flightScene = new FlightScene();
    GameContext& gc = GameContext::getInstance();
    gc.skip_builder = true;
    gc.vkRenderer3d = &g_vkR3D;

    // 初始位置：地表 + 100km（以地球为中心，绝对坐标 = 地球轨道位置 + 地表偏移）
    constexpr double ALT = 100000.0;
    CelestialBody& earth = UniverseModel::getInstance().solar_system[3];
    RocketState& rs = gc.loaded_rocket_state;
    // 绝对坐标 = 地球轨道位置 + 地表位置（地球北极上方 100km）
    rs.abs_px = earth.px;
    rs.abs_py = earth.py + EARTH_RADIUS + ALT;
    rs.abs_pz = earth.pz;
    // 相对地球坐标
    rs.px  = 0.0;
    rs.py  = EARTH_RADIUS + ALT;
    rs.pz  = 0.0;
    rs.surf_px = 0.0;
    rs.surf_py = EARTH_RADIUS;
    rs.surf_pz = 0.0;
    rs.altitude = ALT;
    rs.fuel = 50000.0;

    g_flightScene->onEnter();

    // Vulkan 模式：skip_builder=true → 无火箭组件，从地球全景视角开始
    g_flightScene->cam.mode         = CameraDirector::PANORAMA;
    g_flightScene->cam.focus_target = 3; // Earth (solar_system[3])
    // 初始旋转：让相机面朝太阳（地球→太阳 = +X 方向）
    g_flightScene->cam.quat = Quat::fromAxisAngle(Vec3(0, 1, 0), 3.141592653589793 / 2.0);

    // 将 CPU 网格数据注册到 Vulkan
    auto reg = [](const char* id, const Mesh& m) -> bool {
        if (m.cpuIndices.empty()) return true;
        return g_vkR3D.registerMesh(g_vkCtx, id,
            m.cpuVerts.data(),   m.cpuVerts.size() * sizeof(Vertex3D),
            m.cpuIndices.data(), (uint32_t)m.cpuIndices.size());
    };
    // "earth" mesh 用于所有星球（unit sphere），"ring" 用于土星环
    return reg("earth",      g_flightScene->earthMesh)
        && reg("ring",       g_flightScene->ringMesh)
        && reg("rocketBody", g_flightScene->rocketBody)
        && reg("rocketNose", g_flightScene->rocketNose)
        && reg("rocketBox",  g_flightScene->rocketBox);
}

static void renderFlightSceneVulkan(VkCommandBuffer cmd, int frameSlot) {
    SceneSnapshot snap = g_flightScene->extractRenderSnapshot();


    // Debug: print first-frame matrices
    static int dbg = 0;
    if (dbg < 2) {
        printf("[VkFrame] frame=%d view[0]=%.2f proj[0]=%.2f camPos=(%.0f,%.0f,%.0f)\n",
            snap.frameIndex, snap.view[0], snap.proj[0],
            snap.camPos[0], snap.camPos[1], snap.camPos[2]);
        printf("[VkFrame] celestials=%zu plumes=%zu ribbons=%zu\n",
            snap.celestials.size(), snap.plumes.size(), snap.ribbons.size());
        dbg++;
    }

    // 更新 TAA 重投影矩阵
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

    // acquire 信号量从独立池取（不与 frame slot 绑定，避免 MAILBOX 下的重用冲突）
    VkSemaphore acquireSem = g_frameSync.nextAcquireSem();

    vkWaitForFences(g_vkCtx.device, 1, &frame.inFlightFence, VK_TRUE, UINT64_MAX);

    uint32_t imageIdx = 0;
    VkResult result = vkAcquireNextImageKHR(g_vkCtx.device, g_vkCtx.swapchain,
        UINT64_MAX, acquireSem, VK_NULL_HANDLE, &imageIdx);
    if (result == VK_ERROR_OUT_OF_DATE_KHR || result == VK_SUBOPTIMAL_KHR) {
        handleVkSwapchainResize(window);
        return;
    }
    if (result != VK_SUCCESS) return;

    // renderFinished 按 swapchain image index 取（避免 present 未消费就被再次发射）
    VkSemaphore renderSem = g_frameSync.renderSems[imageIdx];

    vkResetFences(g_vkCtx.device, 1, &frame.inFlightFence);
    vkResetCommandBuffer(frame.commandBuffer, 0);

    VkCommandBufferBeginInfo beginInfo{ VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO };
    beginInfo.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
    vkBeginCommandBuffer(frame.commandBuffer, &beginInfo);

    // Pass 1: 几何渲染 → RGBA16F ping-pong 缓冲
    g_vkTAA.beginGeometryPass(frame.commandBuffer, g_vkCtx.swapExtent);
    VkFrameRenderer::setViewportScissor(frame.commandBuffer, g_vkCtx.swapExtent);
    if (g_flightScene) renderFlightSceneVulkan(frame.commandBuffer, g_frameSync.currentFrame);
    g_vkTAA.endGeometryPass(frame.commandBuffer);

    // Pass 2: TAA resolve → swapchain image
    g_vkTAA.beginResolvePass(frame.commandBuffer,
        g_vkCtx.swapImages[imageIdx],
        g_vkCtx.swapImageViews[imageIdx],
        g_vkCtx.swapExtent);
    VkFrameRenderer::setViewportScissor(frame.commandBuffer, g_vkCtx.swapExtent);
    // 前 60 帧用 blend=1.0（仅当前帧，无历史混合），之后 TAA
    static int warmup = 0;
    float taaBlend = (++warmup < 60) ? 1.0f : 0.1f;
    g_vkTAA.drawResolve(frame.commandBuffer, taaBlend);
    g_vkTAA.endResolvePass(frame.commandBuffer, g_vkCtx.swapImages[imageIdx]);

    // Pass 3: HUD 覆盖层 → swapchain image（alpha 混合）
    if (g_flightScene) {
        SceneSnapshot snapHUD = g_flightScene->extractRenderSnapshot();
        // 基础遥测条
        g_vkHUD.pushQuad(-0.95f, 0.85f, 0.30f, 0.06f, 0.1f, 0.9f, 0.3f, 0.85f);
        g_vkHUD.pushQuad(-0.95f, 0.78f, 0.15f, 0.04f, 0.9f, 0.9f, 0.9f, 0.7f);
        // Cloud Tuner 叠加层 (Block F)
        if (snapHUD.cloudTuner.visible) {
            // 半透明背景面板
            g_vkHUD.pushQuad(-0.99f, 0.96f, 0.62f, 0.62f, 0.04f, 0.04f, 0.10f, 0.88f);
            // 标题文字（简化：用色块代替）
            g_vkHUD.pushQuad(-0.95f, 0.93f, 0.40f, 0.03f, 0.5f, 0.9f, 1.0f, 0.8f);
            // 9 个滑块
            auto& p = snapHUD.cloudTuner;
            float sliders[9][4] = {
                {p.covLo,0.10f,0.70f,0}, {p.covHi,0.30f,0.90f,0},
                {p.threshLo,0.40f,0.92f,0}, {p.threshHi,0.05f,0.55f,0},
                {p.erosion,0.00f,0.50f,0}, {p.density,0.50f,12.0f,0},
                {p.extinction,0.01f,0.60f,0}, {p.minAlt,0.50f,6.00f,0},
                {p.maxAlt,8.00f,22.0f,0}};
            for (int i=0; i<9; i++) {
                float y = 0.88f - 0.068f*i;
                float t = (sliders[i][0]-sliders[i][1])/(sliders[i][2]-sliders[i][1]);
                if (t<0)t=0; if(t>1)t=1;
                g_vkHUD.pushQuad(-0.75f, y, 0.50f, 0.012f, 0.15f, 0.15f, 0.25f, 1.0f);
                g_vkHUD.pushQuad(-0.75f + 0.50f*t*0.5f, y, 0.50f*t, 0.012f, 0.25f, 0.65f, 1.0f, 1.0f);
            }
        }
    }
    {
        // endResolvePass 已将 swapchain 转为 PRESENT_SRC_KHR，HUD 需要 COLOR_ATTACHMENT_OPTIMAL
        transitionImage(frame.commandBuffer, g_vkCtx.swapImages[imageIdx],
            VK_IMAGE_LAYOUT_PRESENT_SRC_KHR,
            VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL,
            VK_PIPELINE_STAGE_2_BOTTOM_OF_PIPE_BIT,          VK_ACCESS_2_NONE,
            VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT,
            VK_ACCESS_2_COLOR_ATTACHMENT_WRITE_BIT | VK_ACCESS_2_COLOR_ATTACHMENT_READ_BIT);

        VkRenderingAttachmentInfo hudAI{VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO};
        hudAI.imageView=g_vkCtx.swapImageViews[imageIdx];
        hudAI.imageLayout=VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
        hudAI.loadOp=VK_ATTACHMENT_LOAD_OP_LOAD; hudAI.storeOp=VK_ATTACHMENT_STORE_OP_STORE;
        VkRenderingInfo ri{VK_STRUCTURE_TYPE_RENDERING_INFO};
        ri.renderArea={{0,0},g_vkCtx.swapExtent}; ri.layerCount=1;
        ri.colorAttachmentCount=1; ri.pColorAttachments=&hudAI;
        vkCmdBeginRendering(frame.commandBuffer,&ri);
        VkFrameRenderer::setViewportScissor(frame.commandBuffer,g_vkCtx.swapExtent);
        g_vkHUD.flush(frame.commandBuffer);
        vkCmdEndRendering(frame.commandBuffer);

        // HUD 结束后转为 PRESENT_SRC_KHR 供呈现
        transitionToPresent(frame.commandBuffer, g_vkCtx.swapImages[imageIdx]);
    }

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
   || !initVulkanFlight()) {
    fprintf(stderr, "[Vulkan] Fatal: initialization failed\n");
    glfwTerminate();
    return -1;
  }
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
      if (g_flightScene) g_flightScene->update(real_dt);  // 物理模拟 + 相机输入
      if (g_flightScene) g_flightScene->render();          // ctx.update() → 相机矩阵，然后早退
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
