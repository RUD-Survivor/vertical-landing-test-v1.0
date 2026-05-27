#pragma once
#include "render/HUD_system.h"
#include "render/renderer_2d.h"
#include "core/rocket_state.h"
#include "simulation/rocket_builder.h"
#include "camera/camera_director.h"
#include "game_context.h"
#include "scene/maneuver_manager.h"
#include "simulation/simulation_controller.h"

/**
 * HUDManager - 2D HUD 接口与上下文适配器
 * =========================================================================
 * 负责从 FlightScene 的局部变量中收集并构建庞大的 HUDContext 结构，
 * 然后统一调度 2D UI 通道 (FlightHUD) 的帧清空、提交渲染、与渲染结束。
 */
class HUDManager {
public:
    FlightHUD hud;

    void render(Renderer* renderer, Renderer3D* r3d, entt::registry& registry, entt::entity entity, CameraDirector& cam, 
        SimulationController& sim_ctrl, ManeuverManager& mnvManager, const RocketAssembly& assembly, const RenderContext& ctx,
        double dt, int frame, double ws_d, float mouse_x, float mouse_y, 
        bool lmb, bool lmb_prev, bool rmb)
    {
        // Vulkan: HUD rendered via ImGui/VkHUD; GL clear removed
        renderer->beginFrame();
        hud.render(registry, entity, renderer, r3d, cam, assembly, ctx, dt, frame, ws_d, mnvManager.global_best_ang, sim_ctrl.time_warp, mouse_x, mouse_y, lmb, lmb_prev, rmb);
        renderer->endFrame();
    }
};
