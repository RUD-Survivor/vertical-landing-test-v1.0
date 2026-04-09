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
        SimulationController& sim_ctrl, ManeuverManager& mnvManager, 
        const Quat& rocketQuat, const Vec3& rocketUp, const Vec3& localNorth, const Vec3& localRight, 
        const Mat4& viewMat, const Mat4& macroProjMat, const Vec3& camEye_rel, 
        double dt, int frame, double ws_d, double mouse_x, double mouse_y, 
        bool lmb, bool lmb_prev, bool rmb)
    {
        glClear(GL_DEPTH_BUFFER_BIT);
        renderer->beginFrame();

        HUDContext hud_ctx;
        hud_ctx.renderer = renderer;
        hud_ctx.registry = &registry;
        hud_ctx.entity = entity;
        
        auto& trans = registry.get<TransformComponent>(entity);
        auto& vel   = registry.get<VelocityComponent>(entity);
        auto& att   = registry.get<AttitudeComponent>(entity);
        auto& prop  = registry.get<PropulsionComponent>(entity);
        auto& tele  = registry.get<TelemetryComponent>(entity);
        auto& guid  = registry.get<GuidanceComponent>(entity);
        auto& mnv   = registry.get<ManeuverComponent>(entity);
        auto& orb   = registry.get<OrbitComponent>(entity);
        auto& rocket_config = registry.get<RocketConfig>(entity);

        auto& control_input = registry.get<ControlInput>(entity);
        // rocket_state removed from HUDContext — ECS components accessed directly
        hud_ctx.rocket_config = &rocket_config;
        hud_ctx.control_input = &control_input;
        hud_ctx.cam = &cam;


        int ww, wh;
        glfwGetWindowSize(GameContext::getInstance().window, &ww, &wh);
        hud_ctx.ww = ww; hud_ctx.wh = wh;
        hud_ctx.aspect = (float)ww / (float)wh;
        hud_ctx.time_warp = sim_ctrl.time_warp;
        hud_ctx.dt = dt;
        hud_ctx.window = GameContext::getInstance().window;
        hud_ctx.assembly = &GameContext::getInstance().launch_assembly;
        hud_ctx.r3d = r3d;
        hud_ctx.frame = frame;
        hud_ctx.ws_d = ws_d;
        hud_ctx.rocketQuat = const_cast<Quat*>(&rocketQuat);
        hud_ctx.rocketUp = const_cast<Vec3*>(&rocketUp);
        hud_ctx.localNorth = const_cast<Vec3*>(&localNorth);
        hud_ctx.localRight = const_cast<Vec3*>(&localRight);
        hud_ctx.ro_x = trans.px;
        hud_ctx.ro_y = trans.py;
        hud_ctx.ro_z = trans.pz;
        hud_ctx.viewMat = viewMat;
        hud_ctx.macroProjMat = macroProjMat;
        hud_ctx.camEye_rel = camEye_rel;
        hud_ctx.mouse_x = mouse_x;
        hud_ctx.mouse_y = mouse_y;
        hud_ctx.lmb = lmb;
        hud_ctx.lmb_prev = lmb_prev;
        hud_ctx.rmb = rmb;
        hud_ctx.global_best_ang = &mnvManager.global_best_ang;

        hud.render(hud_ctx);
        
        renderer->endFrame();
    }
};
