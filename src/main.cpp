// Vulkan-only build
// Vulkan 相关宏必须在 GLFW 第一次 include 之前定义
#define VK_USE_PLATFORM_WIN32_KHR
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#define GLFW_INCLUDE_VULKAN
#define __gl_h_  // 阻止 Windows SDK 的 GL/gl.h 引入

#include <windows.h>
#include <vulkan/vulkan.h>  // 确保 vulkan.h 在 VMA 之前正确加载
#include <GLFW/glfw3.h>
#include <algorithm>
#include <chrono>
#include <thread>
#include <cmath>
#include <functional>
#include <mutex>
#include <cstdlib>

using namespace std;

#include "math/math3d.h"
#include "render/renderer3d.h"
#include "render/model_loader.h"
#include "core/rocket_state.h"
#include "physics/physics_system.h"
#include "control/control_system.h"
#include "simulation/orbit_physics.h"
#include "simulation/predictor.h"
#include "simulation/stage_manager.h"
#include "simulation/maneuver_system.h"
#include "simulation/transfer_calculator.h"
#include "math/spline.h"
#include "math/chebyshev.h"
#include "camera/camera_director.h"

// Vulkan + VMA 在项目头文件之后、全局变量之前
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
#include "render/vulkan/vk_imgui.h"
#include "render/vulkan/systems/vk_game_ui.h"
#include "render/vulkan/imgui_cloud_tuner.h"
#include "render/vulkan/imgui_atmo_tuner.h"
#include "render/vulkan/imgui_sky_tuner.h"
#include "render/vulkan/imgui_perf_overlay.h"
#include "render/scene_snapshot.h"

// ==========================================
// 全局变量与系统初始化 (Global Systems)
// ==========================================

// 鼠标滚轮全局变量：用于在编辑器里缩放或上下拖动。
static float g_scroll_y = 0.0f;
// 中文说明：诊断开关仅由启动脚本设置，关闭对应渲染阶段以定位 GPU 掉线触发条件。
static bool g_diagnosticNoCloud = false;
static bool g_diagnosticNoAtmo = false;
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

static VulkanContext       g_vkCtx;
static FrameSync           g_frameSync;
static VkDescriptorManager g_vkDesc;
static VkTAA               g_vkTAA;
static VkRenderer3D        g_vkR3D;
static bool                g_perfOverlayOpen = false;  // F1 toggles
static bool                g_flowerCloudTunerOpen = false;  // F3 toggles
static bool                g_atmoTunerOpen = false;  // F4 toggles
static bool                g_skyTunerOpen = false;   // F5 toggles
static WeatherMapImGuiPreview g_weatherMapPreview;
static VkHUDSystem         g_vkHUD;
static VkImGuiSystem       g_vkImGui;
static VkGameUI            g_gameUI;
static FlightScene*        g_flightScene = nullptr;
static Terrain::QuadtreeTerrain* g_vkTerrain = nullptr;
static bool                g_vkSwapchainDirty = false;
static int                 warmup = 0;  // TAA 预热帧计数（飞行开始时重置）
static bool                g_workshopMeshesReady = false;
static SceneSnapshot       g_lastFlightSnap;

// skipBuilder=true  → 从 loaded_rocket_state 恢复存档，全景相机模式
// skipBuilder=false → 从 GameContext::launch_assembly 组装发射，地面发射台
static bool initVulkanFlight(bool skipBuilder = false) {
    delete g_flightScene;
    g_flightScene = new FlightScene();
    GameContext& gc = GameContext::getInstance();
    gc.skip_builder = skipBuilder;
    gc.vkRenderer3d = &g_vkR3D;

    if (skipBuilder) {
        // 存档已在 FlightScene::onEnter() 内调用 SaveSystem::LoadGame() 加载，
        // 此处仅设置相机模式 —— 存档读取后为全景（轨道俯瞰），新游戏为地面发射台。
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

        // Preserve metric shallow-water detail: no 8-bit height quantization.
        std::vector<float> tecFloat4(tw * th * 4);
        for (int i = 0; i < tw * th; i++) {
            tecFloat4[i * 4 + 0] = std::clamp(tecSim->gridHeight[i], 0.0f, 1.0f);
            tecFloat4[i * 4 + 1] = 0;
            tecFloat4[i * 4 + 2] = 0;
            tecFloat4[i * 4 + 3] = 1;
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
                tecFloat4.data(), (uint32_t)tw, (uint32_t)th,
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
    g_lastFlightSnap = g_flightScene->extractRenderSnapshot();
    const CelestialDraw* waterPlanet=nullptr;
    for(const auto& cd:g_lastFlightSnap.celestials)if(cd.bodyIdx==3){waterPlanet=&cd;break;}
    g_vkTAA.updateMatrices(g_lastFlightSnap.view, g_lastFlightSnap.proj,
        g_lastFlightSnap.camPos,waterPlanet?waterPlanet->center:nullptr,waterPlanet?waterPlanet->radius:0.f);
    g_vkTAA.uploadMatrices(g_vkCtx, frameSlot);
    g_vkR3D.render(cmd, frameSlot, g_lastFlightSnap, g_vkCtx.swapExtent);
}

static bool checkVkFrameResult(GLFWwindow* window, const char* operation, VkResult result) {
    if (result == VK_SUCCESS) return true;
    int width = 0, height = 0;
    glfwGetFramebufferSize(window, &width, &height);
    fprintf(stderr, "[Vulkan] Fatal: %s returned %d (%dx%d)\n", operation, (int)result, width, height);
    FILE* log = nullptr;
    if (fopen_s(&log, "vulkan_failure.log", "a") == 0 && log) {
        fprintf(log, "time=%.3f operation=%s result=%d framebuffer=%dx%d\n",
            glfwGetTime(), operation, (int)result, width, height);
        fclose(log);
    }
    glfwSetWindowShouldClose(window, GLFW_TRUE);
    return false;
}

static void handleVkSwapchainResize(GLFWwindow* window) {
    // 先等待呈现引擎释放旧信号量，再销毁旧交换链；渲染 fence 不能替代此等待。
    if (!checkVkFrameResult(window, "waitForPresent", g_frameSync.waitForPresent(g_vkCtx.device))) return;
    if (!g_vkCtx.recreateSwapchain(window)) {
        checkVkFrameResult(window, "recreateSwapchain failed (status synthesized; see stderr)", VK_ERROR_INITIALIZATION_FAILED);
        return;
    }
    g_frameSync.destroyPresentObjects(g_vkCtx.device);
    if (!g_frameSync.createPresentObjects(g_vkCtx.device, g_vkCtx.swapImages.size())) {
        checkVkFrameResult(window, "createPresentObjects", VK_ERROR_INITIALIZATION_FAILED);
        return;
    }
    if (!g_vkTAA.recreate(g_vkCtx, g_vkCtx.swapExtent, g_vkCtx.swapFormat,
        "src/render/shaders/spirv/taa.vert.spv",
        "src/render/shaders/spirv/taa.frag.spv")
        || !g_vkR3D.cloudSystem.recreate(g_vkCtx, g_vkCtx.swapExtent)) {
        checkVkFrameResult(window, "recreateRenderTargets failed (status synthesized; see stderr)", VK_ERROR_INITIALIZATION_FAILED);
        return;
    }
    warmup = 0;
    g_vkSwapchainDirty = false;
}

static void renderVulkanFrame(GLFWwindow* window) {
    if (g_vkSwapchainDirty) { handleVkSwapchainResize(window); return; }

    auto& frame = g_frameSync.current();
    VkSemaphore acquireSem = g_frameSync.nextAcquireSem();

    if (!checkVkFrameResult(window, "vkWaitForFences",
        vkWaitForFences(g_vkCtx.device, 1, &frame.inFlightFence, VK_TRUE, 5000000000ULL))) return;

    uint32_t imageIdx = 0;
    VkResult result = vkAcquireNextImageKHR(g_vkCtx.device, g_vkCtx.swapchain,
        UINT64_MAX, acquireSem, VK_NULL_HANDLE, &imageIdx);
    if (result == VK_ERROR_OUT_OF_DATE_KHR) {
        handleVkSwapchainResize(window); return;
    }
    // SUBOPTIMAL 仍然成功获取图像并置位信号量，必须提交消费后才能重建。
    if (result == VK_SUBOPTIMAL_KHR) g_vkSwapchainDirty = true;
    else if (!checkVkFrameResult(window, "vkAcquireNextImageKHR", result)) return;

    VkSemaphore renderSem = g_frameSync.renderSems[imageIdx];

    if (!checkVkFrameResult(window, "vkResetFences", vkResetFences(g_vkCtx.device, 1, &frame.inFlightFence))) return;
    if (!checkVkFrameResult(window, "vkResetCommandBuffer", vkResetCommandBuffer(frame.commandBuffer, 0))) return;

    VkCommandBufferBeginInfo beginInfo{ VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO };
    beginInfo.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
    if (!checkVkFrameResult(window, "vkBeginCommandBuffer", vkBeginCommandBuffer(frame.commandBuffer, &beginInfo))) return;
    // 多帧在途仍会复用云的中间图像和共享深度；同队列也需要显式内存依赖。
    // 先采用保守屏障保证正确性，后续可按各 pass 的读写范围细化以恢复并行度。
    VkMemoryBarrier2 sharedBarrier{ VK_STRUCTURE_TYPE_MEMORY_BARRIER_2 };
    sharedBarrier.srcStageMask = VK_PIPELINE_STAGE_2_ALL_COMMANDS_BIT;
    sharedBarrier.srcAccessMask = VK_ACCESS_2_MEMORY_READ_BIT | VK_ACCESS_2_MEMORY_WRITE_BIT;
    sharedBarrier.dstStageMask = VK_PIPELINE_STAGE_2_ALL_COMMANDS_BIT;
    sharedBarrier.dstAccessMask = VK_ACCESS_2_MEMORY_READ_BIT | VK_ACCESS_2_MEMORY_WRITE_BIT;
    VkDependencyInfo sharedDependency{ VK_STRUCTURE_TYPE_DEPENDENCY_INFO };
    sharedDependency.memoryBarrierCount = 1;
    sharedDependency.pMemoryBarriers = &sharedBarrier;
    vkCmdPipelineBarrier2(frame.commandBuffer, &sharedDependency);

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
        g_vkTAA.uploadMatrices(g_vkCtx, g_frameSync.currentFrame);
        g_vkR3D.render(frame.commandBuffer, g_frameSync.currentFrame, snap, g_vkCtx.swapExtent);
    }
    g_vkTAA.endGeometryPass(frame.commandBuffer);

    // Water writes surface depth before atmosphere/cloud composition. Its input
    // color and opaque depth are separate snapshots. Clear reactive outside flight.
    {
        SceneSnapshot emptyWaterScene{};
        g_vkR3D.oceanSystem.render(g_vkCtx, frame.commandBuffer, g_vkTAA, g_vkR3D.scene,
            inFlight ? g_lastFlightSnap : emptyWaterScene, g_frameSync.currentFrame,
            g_vkCtx.swapExtent, g_vkR3D.atmoTune);
    }

    // 大气重构：壳内/壳外路径，必须排在这里（几何 pass 刚结束、深度已经是只读纹理）
    // 且必须在体积云之前（云管线要求进来时 HDR 颜色已经是 SHADER_READ_ONLY_OPTIMAL，
    // 见 vk_renderer3d.h::renderAtmoAfterGeometry 顶部注释）。目前只覆盖 flight 场景
    // （workshop 的 snap 是上面 else-if 块里的局部变量，这里拿不到；workshop 是船体
    // 编辑器视角，本就不太需要看远处行星大气，先不管——如果之后发现需要，得把
    // workshop 的 snap 也提升成能在这里访问到的变量）。
    if (inFlight && !g_diagnosticNoAtmo) {
        g_vkR3D.renderAtmoAfterGeometry(g_vkCtx, frame.commandBuffer, g_vkTAA,
            g_lastFlightSnap, g_frameSync.currentFrame);
    }

    // Flower cloud Phase0: compute stub → composite onto HDR (sky depth >= 0.9995)
    if (inFlight && !g_diagnosticNoCloud && g_vkR3D.cloudSystem.enabled) {
        g_vkR3D.renderFlowerCloudAfterGeometry(g_vkCtx, frame.commandBuffer, g_vkTAA,
            g_lastFlightSnap, g_frameSync.currentFrame);
    }

    if(inFlight)g_vkR3D.cameraBloom.render(g_vkCtx,frame.commandBuffer,g_vkTAA,
        g_lastFlightSnap,g_frameSync.currentFrame,g_vkCtx.swapExtent,g_vkR3D.scene.billboardQuadVbo);
    if(inFlight)g_vkR3D.renderSolarOpticsAfterGeometry(g_vkCtx,frame.commandBuffer,g_vkTAA,
        g_lastFlightSnap,g_frameSync.currentFrame,g_vkCtx.swapExtent);

    g_vkTAA.beginResolvePass(frame.commandBuffer,
        g_vkCtx.swapImages[imageIdx],
        g_vkCtx.swapImageViews[imageIdx],
        g_vkCtx.swapExtent);
    VkFrameRenderer::setViewportScissor(frame.commandBuffer, g_vkCtx.swapExtent);
    float taaBlend = (++warmup < 60 || (inFlight && !g_diagnosticNoCloud
        && g_vkR3D.cloudSystem.enabled && g_vkR3D.cloudSystem.cameraCutThisFrame)) ? 1.0f : 0.1f;
    g_vkTAA.displayExposure = inFlight ? (g_vkR3D.sky.tune.observationMode ? 128.f : g_vkR3D.atmoTune.displayExposure) : 1.f;
    g_vkTAA.applyToneMap = inFlight;
    g_vkTAA.drawResolve(frame.commandBuffer, taaBlend);
    g_vkTAA.endResolvePass(frame.commandBuffer, g_vkCtx.swapImages[imageIdx]);
    // swapchain 现在处于 PRESENT_SRC_KHR

    // Pass 3: ImGui UI / HUD（覆盖在 TAA 结果之上）
    g_vkImGui.newFrame();
    g_gameUI.draw(inFlight ? g_flightScene : nullptr, ww, wh);
    {
        const char* presentName = "FIFO";
        switch (g_vkCtx.swapPresentMode) {
            case VK_PRESENT_MODE_IMMEDIATE_KHR:    presentName = "IMMEDIATE"; break;
            case VK_PRESENT_MODE_MAILBOX_KHR:      presentName = "MAILBOX"; break;
            case VK_PRESENT_MODE_FIFO_KHR:         presentName = "FIFO"; break;
            case VK_PRESENT_MODE_FIFO_RELAXED_KHR: presentName = "FIFO_RELAXED"; break;
            default: break;
        }
        PerfOverlayImGui(&g_perfOverlayOpen, ww, wh, presentName);
    }
    FlowerCloudTunerImGui(&g_flowerCloudTunerOpen, g_vkR3D.cloudSystem.tuneParams, &g_weatherMapPreview,
                          &g_vkR3D.shadowSystem, &g_vkR3D.cloudSystem.cloudShadow);
    AtmoTunerImGui(&g_atmoTunerOpen, g_vkR3D.atmoTune, g_vkR3D.scene.atmoLutCache);
    SkyTunerImGui(&g_skyTunerOpen, g_vkR3D.sky.tune);
    ImGui::Render();
    g_vkImGui.renderToSwapchain(frame.commandBuffer,
        g_vkCtx.swapImages[imageIdx],
        g_vkCtx.swapImageViews[imageIdx],
        g_vkCtx.swapExtent);
    // swapchain 已由 renderToSwapchain 转回 PRESENT_SRC_KHR

    if (!checkVkFrameResult(window, "vkEndCommandBuffer", vkEndCommandBuffer(frame.commandBuffer))) return;

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

    // 每两秒记录一次提交前状态；即使显卡掉线，也能保留最后的窗口尺寸和负载配置。
    static double lastTraceTime = -2.0;
    if (glfwGetTime() - lastTraceTime >= 2.0) {
        lastTraceTime = glfwGetTime();
        FILE* trace = nullptr;
        if (fopen_s(&trace, "gpu_trace.log", "a") == 0 && trace) {
            fprintf(trace, "time=%.3f size=%dx%d slot=%d flight=%d cloud=%d atmo=%d\n",
                lastTraceTime, ww, wh, g_frameSync.currentFrame, (int)inFlight,
                (int)(inFlight && !g_diagnosticNoCloud && g_vkR3D.cloudSystem.enabled),
                (int)(inFlight && !g_diagnosticNoAtmo));
            fclose(trace);
        }
    }
    if (!checkVkFrameResult(window, "vkQueueSubmit2",
        vkQueueSubmit2(g_vkCtx.graphicsQueue, 1, &submitInfo2, frame.inFlightFence))) return;

    VkPresentInfoKHR presentInfo{ VK_STRUCTURE_TYPE_PRESENT_INFO_KHR };
    VkFence presentFence = g_frameSync.presentFences[imageIdx];
    // 每张图像的呈现 fence 独立复用，重置前必须确认上一轮呈现已释放资源。
    if (!checkVkFrameResult(window, "waitPresentFence", vkWaitForFences(g_vkCtx.device, 1, &presentFence, VK_TRUE, UINT64_MAX))) return;
    if (!checkVkFrameResult(window, "resetPresentFence", vkResetFences(g_vkCtx.device, 1, &presentFence))) return;
    VkSwapchainPresentFenceInfoEXT presentFenceInfo{ VK_STRUCTURE_TYPE_SWAPCHAIN_PRESENT_FENCE_INFO_EXT };
    presentFenceInfo.swapchainCount = 1;
    presentFenceInfo.pFences = &presentFence;
    presentInfo.pNext = &presentFenceInfo;
    presentInfo.waitSemaphoreCount = 1;
    presentInfo.pWaitSemaphores    = &renderSem;
    presentInfo.swapchainCount     = 1;
    presentInfo.pSwapchains        = &g_vkCtx.swapchain;
    presentInfo.pImageIndices      = &imageIdx;

    result = vkQueuePresentKHR(g_vkCtx.presentQueue, &presentInfo);
    // 内存不足表示未入队；OUT_OF_DATE/SURFACE_LOST 则仍会执行呈现队列操作。
    g_frameSync.presentPending[imageIdx] = result == VK_SUCCESS || result == VK_SUBOPTIMAL_KHR
        || result == VK_ERROR_OUT_OF_DATE_KHR || result == VK_ERROR_SURFACE_LOST_KHR
        || result == VK_ERROR_FULL_SCREEN_EXCLUSIVE_MODE_LOST_EXT;
    if (result == VK_ERROR_OUT_OF_DATE_KHR || result == VK_SUBOPTIMAL_KHR)
        g_vkSwapchainDirty = true;
    else if (!checkVkFrameResult(window, "vkQueuePresentKHR", result)) return;

    g_vkTAA.swap();
    g_frameSync.advance();
}


// ==========================================
// Part 2: Explorer Class -> Replaced by ECS (RocketState, PhysicsSystem)
// ==========================================


void framebuffer_size_callback(GLFWwindow* /*window*/, int width, int height) {
  g_vkSwapchainDirty = true;
}


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
static int runApplication() {
  g_diagnosticNoCloud = std::getenv("ROCKET_DISABLE_CLOUD") != nullptr;
  g_diagnosticNoAtmo = std::getenv("ROCKET_DISABLE_ATMO") != nullptr;
  printf("[Diagnostic] cloud disabled=%d, atmosphere disabled=%d\n", (int)g_diagnosticNoCloud, (int)g_diagnosticNoAtmo);
  // --- 游戏启动与初始化 ---
  
  
  // ==========================================================
  // 初始化 GLFW（窗口和输入）+ Vulkan 渲染
  // ==========================================================
  glfwInit();

  // 纯 Vulkan 窗口，不创建 OpenGL 上下文
  glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API);

  GLFWwindow *window = glfwCreateWindow(1000, 800, "3D Rocket Sim", NULL, NULL);
  if (!window) {
    glfwTerminate();
    return -1;
  }

  glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);
  glfwSetScrollCallback(window, scroll_callback);

  // UniverseModel 初始化必须在 FlightScene::onEnter() 之前
  PhysicsSystem::InitSolarSystem();

  // 完整 Vulkan 初始化序列
  if (!g_vkCtx.initCore(window)
   || !g_vkCtx.initSwapchain(window)
   || !g_frameSync.init(g_vkCtx.device, g_vkCtx.commandPool, g_vkCtx.swapImages.size())
   || !g_vkDesc.init(g_vkCtx)
   || !g_vkR3D.init(g_vkCtx, g_vkDesc, VkTAA::kColorFmt, VkTAA::kDepthFmt)
   || !g_vkTAA.init(g_vkCtx, g_vkCtx.swapExtent, g_vkCtx.swapFormat,
                    "src/render/shaders/spirv/taa.vert.spv",
                    "src/render/shaders/spirv/taa.frag.spv")
  || !g_vkR3D.cloudSystem.init(g_vkCtx, g_vkCtx.swapExtent,
       g_vkR3D.scene.cloudNoiseTex.view, g_vkR3D.scene.cloudDetailTex.view, g_vkR3D.scene.cloudWeatherTex.view,
       g_vkR3D.scene.cloudCurlTex.view,
       g_vkR3D.scene.cloudProfileLutTex.view, g_vkR3D.scene.cloudHiWeatherTex.view)
   || !g_vkHUD.init(g_vkCtx, g_vkCtx.swapFormat)
   || !g_vkImGui.init(g_vkCtx, window, g_vkCtx.swapFormat)) {
    fprintf(stderr, "[Vulkan] Fatal: initialization failed\n");
    glfwTerminate();
    return -1;
  }
  if (!g_weatherMapPreview.registerTextures(
          g_vkR3D.scene.cloudWeatherTex.view,
          g_vkR3D.scene.weatherVisViewR,
          g_vkR3D.scene.weatherVisViewG,
          g_vkR3D.scene.weatherVisViewB,
          g_vkR3D.scene.weatherVisViewA)) {
      fprintf(stderr, "[Vulkan] Warning: weather map ImGui preview registration failed\n");
  }
  if (!g_weatherMapPreview.registerCurl(g_vkR3D.scene.curlVisView)) {
      fprintf(stderr, "[Vulkan] Warning: curl noise ImGui preview registration failed\n");
  }
  g_weatherMapPreview.registerProfileLut(g_vkR3D.scene.cloudProfileLutTex.view);
  g_weatherMapPreview.registerHiWeather(g_vkR3D.scene.cloudHiWeatherTex.view);
  // 从主菜单启动
  g_gameUI.state = VkGameState::MENU;

  Simulation::AsyncOrbitPredictor orbit_predictor;
  orbit_predictor.Start();

  // =========================================================
  // GameContext 初始化
  // =========================================================
  GameContext& ctx = GameContext::getInstance();
  ctx.window = window;
  ctx.orbit_predictor = &orbit_predictor;

  double last_time = glfwGetTime();

  while (!glfwWindowShouldClose(window)) {
      glfwPollEvents();

      double current_time = glfwGetTime();
      double real_dt = current_time - last_time;
      last_time = current_time;
      if (real_dt > 0.1) real_dt = 0.02;

      // If window got closed by a scene
      if (glfwWindowShouldClose(window)) break;

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
              g_workshopMeshesReady = true;
              g_gameUI.state = VkGameState::FLIGHT;
              warmup = 0;
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
          g_flightScene->render();
      }
      renderVulkanFrame(window);
  }

  orbit_predictor.Stop();

  // 正常退出也等待呈现资源释放；设备丢失时跳过不可完成的等待。
  if (vkDeviceWaitIdle(g_vkCtx.device) == VK_SUCCESS)
      g_frameSync.waitForPresent(g_vkCtx.device);
  delete g_flightScene; g_flightScene = nullptr;
  delete g_vkTerrain;   g_vkTerrain   = nullptr;
  g_weatherMapPreview.unregister();
  g_vkImGui.shutdown(g_vkCtx.device);
  g_vkR3D.shutdown(g_vkCtx);
  g_vkTAA.shutdown(g_vkCtx);
  g_vkHUD.shutdown(g_vkCtx);
  g_vkDesc.shutdown(g_vkCtx);
  g_frameSync.shutdown(g_vkCtx.device);
  g_vkCtx.shutdown();

  glfwTerminate();
  return 0;
}

int main() {
    try {
        return runApplication();
    } catch (const std::exception& error) {
        // 上传或映射失败时立即停止，避免后续使用无效 GPU 资源；详细错误已落盘。
        fprintf(stderr, "[Vulkan] Fatal: %s\n", error.what());
        return EXIT_FAILURE;
    }
}
