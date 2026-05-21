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
static VulkanContext       g_vkCtx;
static FrameSync           g_frameSync;
static VkDescriptorManager g_vkDesc;
static VkMeshPipeline      g_meshPipe;
static VkTAA               g_vkTAA;
static bool                g_vkSwapchainDirty = false;

// --- Test scene resources ---
static VkMesh          g_testMesh;
static VkTexture2D     g_nullTex;
static VkDescriptorSet g_nullTexSet = VK_NULL_HANDLE;

// Column-major 4×4 matrix (matches GLSL mat4 memory layout)
struct VkMat4 { float m[16]; };
static VkMat4 mat4Mul(const VkMat4& a, const VkMat4& b) {
    VkMat4 c{};
    for (int col = 0; col < 4; col++)
        for (int row = 0; row < 4; row++)
            for (int k = 0; k < 4; k++)
                c.m[col*4+row] += a.m[k*4+row] * b.m[col*4+k];
    return c;
}
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

struct TestVertex { float pos[3], normal[3], uv[2], color[4]; };
static bool initTestScene() {
    static const TestVertex verts[] = {
        // Front  (z=+0.5, n=0,0,1)
        {{-0.5f,-0.5f, 0.5f},{0,0,1},{0,0},{1,1,1,1}}, {{ 0.5f,-0.5f, 0.5f},{0,0,1},{1,0},{1,1,1,1}},
        {{ 0.5f, 0.5f, 0.5f},{0,0,1},{1,1},{1,1,1,1}}, {{-0.5f, 0.5f, 0.5f},{0,0,1},{0,1},{1,1,1,1}},
        // Back   (z=-0.5, n=0,0,-1)
        {{ 0.5f,-0.5f,-0.5f},{0,0,-1},{0,0},{1,1,1,1}}, {{-0.5f,-0.5f,-0.5f},{0,0,-1},{1,0},{1,1,1,1}},
        {{-0.5f, 0.5f,-0.5f},{0,0,-1},{1,1},{1,1,1,1}}, {{ 0.5f, 0.5f,-0.5f},{0,0,-1},{0,1},{1,1,1,1}},
        // Left   (x=-0.5, n=-1,0,0)
        {{-0.5f,-0.5f,-0.5f},{-1,0,0},{0,0},{1,1,1,1}}, {{-0.5f,-0.5f, 0.5f},{-1,0,0},{1,0},{1,1,1,1}},
        {{-0.5f, 0.5f, 0.5f},{-1,0,0},{1,1},{1,1,1,1}}, {{-0.5f, 0.5f,-0.5f},{-1,0,0},{0,1},{1,1,1,1}},
        // Right  (x=+0.5, n=1,0,0)
        {{ 0.5f,-0.5f, 0.5f},{1,0,0},{0,0},{1,1,1,1}},  {{ 0.5f,-0.5f,-0.5f},{1,0,0},{1,0},{1,1,1,1}},
        {{ 0.5f, 0.5f,-0.5f},{1,0,0},{1,1},{1,1,1,1}},  {{ 0.5f, 0.5f, 0.5f},{1,0,0},{0,1},{1,1,1,1}},
        // Top    (y=+0.5, n=0,1,0)
        {{-0.5f, 0.5f, 0.5f},{0,1,0},{0,0},{1,1,1,1}},  {{ 0.5f, 0.5f, 0.5f},{0,1,0},{1,0},{1,1,1,1}},
        {{ 0.5f, 0.5f,-0.5f},{0,1,0},{1,1},{1,1,1,1}},  {{-0.5f, 0.5f,-0.5f},{0,1,0},{0,1},{1,1,1,1}},
        // Bottom (y=-0.5, n=0,-1,0)
        {{-0.5f,-0.5f,-0.5f},{0,-1,0},{0,0},{1,1,1,1}}, {{ 0.5f,-0.5f,-0.5f},{0,-1,0},{1,0},{1,1,1,1}},
        {{ 0.5f,-0.5f, 0.5f},{0,-1,0},{1,1},{1,1,1,1}}, {{-0.5f,-0.5f, 0.5f},{0,-1,0},{0,1},{1,1,1,1}},
    };
    static const uint32_t indices[] = {
         0, 1, 2,  2, 3, 0,   4, 5, 6,  6, 7, 4,
         8, 9,10, 10,11, 8,  12,13,14, 14,15,12,
        16,17,18, 18,19,16,  20,21,22, 22,23,20,
    };
    if (!g_testMesh.upload(g_vkCtx, verts, sizeof(verts), indices, 36))
        return false;
    static const uint8_t white[4] = {255,255,255,255};
    if (!g_nullTex.upload(g_vkCtx, white, 1, 1))
        return false;
    g_nullTexSet = g_vkDesc.allocateTextureSet(g_vkCtx.device, g_nullTex.view, g_nullTex.sampler);
    return g_nullTexSet != VK_NULL_HANDLE;
}
static void updateTestFrameUBO(int frameIdx, float time) {
    float angle = time * 0.5f;
    float ex = sinf(angle) * 3.0f, ey = 1.5f, ez = cosf(angle) * 3.0f;
    float aspect = (float)g_vkCtx.swapExtent.width / (float)g_vkCtx.swapExtent.height;
    VkMat4 view = mat4LookAt(ex, ey, ez, 0.0f, 0.0f, 0.0f);
    VkMat4 proj = mat4Perspective(1.047f, aspect, 0.1f, 100.0f);
    FrameUBO ubo{};
    memcpy(ubo.view, view.m, 64);
    memcpy(ubo.proj, proj.m, 64);
    ubo.lightDir[0] = 0.577f; ubo.lightDir[1] = 0.577f; ubo.lightDir[2] = 0.577f;
    ubo.viewPos[0] = ex; ubo.viewPos[1] = ey; ubo.viewPos[2] = ez;
    ubo.time = time;
    g_vkDesc.updateFrameUBO(frameIdx, ubo);
}
static void drawTestScene(VkCommandBuffer cmd, int frameIdx) {
    g_meshPipe.bind(cmd);
    VkDescriptorSet sets[] = { g_vkDesc.set0[frameIdx], g_nullTexSet };
    vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS,
        g_meshPipe.layout, 0, 2, sets, 0, nullptr);
    MeshPushConstants pc{};
    pc.model[0] = pc.model[5] = pc.model[10] = pc.model[15] = 1.0f;
    pc.baseColor[0] = pc.baseColor[1] = pc.baseColor[2] = pc.baseColor[3] = 1.0f;
    pc.ambientStr = 0.15f;
    pc.hasTexture = 0;
    vkCmdPushConstants(cmd, g_meshPipe.layout,
        VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT,
        0, sizeof(MeshPushConstants), &pc);
    g_testMesh.bind(cmd);
    g_testMesh.draw(cmd);
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
    updateTestFrameUBO(g_frameSync.currentFrame, (float)glfwGetTime());
    drawTestScene(frame.commandBuffer, g_frameSync.currentFrame);
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


void framebuffer_size_callback(GLFWwindow* /*window*/, int /*width*/, int /*height*/) {
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
   || !initTestScene()) {
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

  PhysicsSystem::InitSolarSystem();

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
  g_testMesh.destroy(g_vkCtx);
  g_nullTex.destroy(g_vkCtx);
  g_vkTAA.shutdown(g_vkCtx);
  g_meshPipe.shutdown(g_vkCtx.device);
  g_vkDesc.shutdown(g_vkCtx);
  g_frameSync.shutdown(g_vkCtx.device);
  g_vkCtx.shutdown();
#endif

  glfwTerminate();
  return 0;
}
