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
static VulkanContext       g_vkCtx;
static FrameSync           g_frameSync;
static VkDescriptorManager g_vkDesc;
static VkMeshPipeline      g_meshPipe;
static VkTAA               g_vkTAA;
static VkRenderer3D        g_vkR3D;
static FlightScene*        g_flightScene = nullptr;
static bool                g_vkSwapchainDirty = false;

// Minimal matrix helpers for renderVulkanFrame camera setup (column-major, GLSL-compatible)
struct VkMat4 { float m[16]; };
static VkMat4 mat4LookAt(float ex, float ey, float ez, float tx, float ty, float tz) {
    float fx=tx-ex, fy=ty-ey, fz=tz-ez;
    float fl=sqrtf(fx*fx+fy*fy+fz*fz); fx/=fl; fy/=fl; fz/=fl;
    float rx=-fz, ry=0.0f, rz=fx;
    float rl=sqrtf(rx*rx+rz*rz); rx/=rl; rz/=rl;
    float ux=ry*fz-rz*fy, uy=rz*fx-rx*fz, uz=rx*fy-ry*fx;
    float dd0=-(rx*ex+ry*ey+rz*ez), dd1=-(ux*ex+uy*ey+uz*ez), dd2=fx*ex+fy*ey+fz*ez;
    return {{ rx,ux,-fx,0, ry,uy,-fy,0, rz,uz,-fz,0, dd0,dd1,dd2,1 }};
}
static VkMat4 mat4Perspective(float fovY, float aspect, float zn, float zf) {
    float f = 1.0f / tanf(fovY * 0.5f);
    float C = (zf + zn) / (zn - zf), D = (2.0f * zf * zn) / (zn - zf);
    return {{ f/aspect,0,0,0, 0,f,0,0, 0,0,C,-1, 0,0,D,0 }};
}

static bool initVulkanFlight() {
    g_flightScene = new FlightScene();
    GameContext& gc = GameContext::getInstance();
    gc.skip_builder = true;
    gc.vkRenderer3d = &g_vkR3D;

    // 初始位置：地表 + 100km，避免 RenderContext 零除
    constexpr double ALT = 100000.0;
    RocketState& rs = gc.loaded_rocket_state;
    rs.py  = EARTH_RADIUS + ALT;
    rs.abs_py = EARTH_RADIUS + ALT;
    rs.surf_py = EARTH_RADIUS;
    rs.altitude = ALT;
    rs.fuel = 50000.0;

    g_flightScene->onEnter();

    // 将 CPU 网格数据注册到 Vulkan
    auto reg = [](const char* id, const Mesh& m) -> bool {
        if (m.cpuIndices.empty()) return true;
        return g_vkR3D.registerMesh(g_vkCtx, id,
            m.cpuVerts.data(),   m.cpuVerts.size() * sizeof(Vertex3D),
            m.cpuIndices.data(), (uint32_t)m.cpuIndices.size());
    };
    return reg("earth",      g_flightScene->earthMesh)
        && reg("rocketBody", g_flightScene->rocketBody)
        && reg("rocketNose", g_flightScene->rocketNose)
        && reg("rocketBox",  g_flightScene->rocketBox);
}

static void renderFlightSceneVulkan(VkCommandBuffer cmd, int frameIdx) {
    auto& ctx = g_flightScene->ctx;

    // 地球（球心在渲染空间原点的负方向）
    float er = ctx.earth_r;    // ≈ 6371 render units
    float ex = (float)(-ctx.ro_x);
    float ey = (float)(-ctx.ro_y);
    float ez = (float)(-ctx.ro_z);

    float viewArr[16], projArr[16];
    ctx.viewMat.toFloatArray(viewArr);
    ctx.macroProjMat.toFloatArray(projArr);

    // Vec3::normalized() 对零向量返回 (0,0,0) 而非 NaN，
    // 导致 lookAt(eye==target) 产生全零矩阵而非 NaN 矩阵。
    // 用对角线检测：若 viewArr[0]/[5]/[10] 全为 0 或任意元素 NaN/Inf，切换降级相机。
    {
        bool degenerate = (fabsf(viewArr[0]) < 1e-6f && fabsf(viewArr[5]) < 1e-6f && fabsf(viewArr[10]) < 1e-6f)
                       || !std::isfinite(viewArr[0]) || !std::isfinite(viewArr[5]);
        if (degenerate) {
            float aspect = (float)g_vkCtx.swapExtent.width / (float)g_vkCtx.swapExtent.height;
            float cx = er * 0.5f, cy = er * 1.6f, cz = 0.0f;
            VkMat4 fallView = mat4LookAt(cx, cy, cz, ex, ey, ez);
            VkMat4 fallProj = mat4Perspective(0.785f, aspect, er * 0.001f, er * 30.0f);
            memcpy(viewArr, fallView.m, 64);
            memcpy(projArr, fallProj.m, 64);
        }
    }

    // 归一化光照方向
    float ldlen = sqrtf(ctx.lightVec.x*ctx.lightVec.x + ctx.lightVec.y*ctx.lightVec.y + ctx.lightVec.z*ctx.lightVec.z);
    float ld[3];
    if (ldlen > 1e-6f) { ld[0]=ctx.lightVec.x/ldlen; ld[1]=ctx.lightVec.y/ldlen; ld[2]=ctx.lightVec.z/ldlen; }
    else               { ld[0]=0.577f; ld[1]=0.577f; ld[2]=0.577f; }

    // 相机世界位置（用于高光计算）
    bool vpBad = !std::isfinite(ctx.camEye_rel.x) || !std::isfinite(ctx.camEye_rel.y);
    float vp[3] = { vpBad ? ex + er * 0.5f : (float)ctx.camEye_rel.x,
                    vpBad ? ey + er * 1.6f : (float)ctx.camEye_rel.y,
                    vpBad ? ez              : (float)ctx.camEye_rel.z };

    float t = (float)glfwGetTime();
    g_vkR3D.beginFrame(cmd, frameIdx, viewArr, projArr, ld, vp, t);

    float earthModel[16] = {
        er, 0,  0,  0,
        0,  er, 0,  0,
        0,  0,  er, 0,
        ex, ey, ez, 1
    };
    g_vkR3D.drawMesh(cmd, "earth", earthModel, 0.28f, 0.52f, 1.0f);

    // 火箭在渲染原点（renderRocketBase ≈ 0）——用 earth_r 给出合理视觉尺寸
    float rw = er * 0.0008f;   // 约 5 render units 宽
    float rh = er * 0.005f;    // 约 32 render units 高
    float bodyModel[16] = {
        rw, 0,  0,  0,
        0,  rh, 0,  0,
        0,  0,  rw, 0,
        0,  0,  0,  1
    };
    g_vkR3D.drawMesh(cmd, "rocketBody", bodyModel, 0.92f, 0.92f, 0.92f);

    float noseModel[16] = {
        rw,   0,        0,  0,
        0,    rw * 2.f, 0,  0,
        0,    0,        rw, 0,
        0,    rh * 0.5f, 0, 1   // 圆锥基底接机体顶部
    };
    g_vkR3D.drawMesh(cmd, "rocketNose", noseModel, 0.88f, 0.88f, 0.88f);
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
    g_vkTAA.drawResolve(frame.commandBuffer, 0.1f);
    g_vkTAA.endResolvePass(frame.commandBuffer, g_vkCtx.swapImages[imageIdx]);

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
   || !g_meshPipe.init(g_vkCtx, g_vkDesc,
                       VkTAA::kColorFmt, VkTAA::kDepthFmt,
                       "src/render/shaders/spirv/mesh.vert.spv",
                       "src/render/shaders/spirv/mesh.frag.spv")
   || !g_vkTAA.init(g_vkCtx, g_vkCtx.swapExtent, g_vkCtx.swapFormat,
                    "src/render/shaders/spirv/taa.vert.spv",
                    "src/render/shaders/spirv/taa.frag.spv")
   || !g_vkR3D.init(g_vkCtx, g_meshPipe, g_vkDesc)
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
      if (g_flightScene) g_flightScene->render();  // ctx.update() → 相机矩阵，然后早退
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
  g_meshPipe.shutdown(g_vkCtx.device);
  g_vkDesc.shutdown(g_vkCtx);
  g_frameSync.shutdown(g_vkCtx.device);
  g_vkCtx.shutdown();
#endif

  glfwTerminate();
  return 0;
}
