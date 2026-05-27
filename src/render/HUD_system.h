#pragma once
#include "core/universe_model.h"
#include <vector>
#include <string>
#include <cmath>
#include <algorithm>
#include <GLFW/glfw3.h>
#include "render/renderer_2d.h"
#include "core/rocket_state.h"
#include "scene/render_context.h"
class SimulationController;
class ManeuverManager;

#include "camera/camera_director.h"
#include "math/math3d.h"
#include "physics/physics_system.h" 
#include "simulation/rocket_builder.h"
#include "simulation/transfer_calculator.h"
#include "render/renderer3d.h"

// Forward declare formatTime from main.cpp
std::string formatTime(double seconds, bool absolute = false);


class FlightHUD {
public:
    bool show_galaxy_info = false;
    int selected_body_idx = -1;
    int expanded_planet_idx = -1;
    bool hlmb_prev_galaxy = false;
    bool mnv_popup_visible = false;
    int mnv_popup_index = -1;
    float mnv_popup_px = 0, mnv_popup_py = 0, mnv_popup_pw = 0, mnv_popup_ph = 0;
    float mnv_popup_node_scr_x = 0, mnv_popup_node_scr_y = 0;
    Vec3 mnv_popup_dv = Vec3(0,0,0);
    bool mnv_popup_close_hover = false, mnv_popup_del_hover = false;
    double mnv_popup_time_to_node = 0;
    double mnv_popup_burn_time = 0;
    int mnv_popup_ref_body = -1;
    int mnv_popup_slider_dragging = -1;
    float mnv_popup_slider_drag_x = 0;
    int mnv_popup_burn_mode = 0;
    bool mnv_popup_mode_hover = false;
    Vec3 adv_mnv_world_pos = Vec3(0,0,0);
    bool adv_orbit_enabled = false;
    bool adv_orbit_menu = false;
    float adv_orbit_pred_days = 30.0f;
    int adv_orbit_iters = 4000;
    int adv_orbit_ref_mode = 0;
    int adv_orbit_ref_body = 3;
    int adv_orbit_secondary_ref_body = 4;
    bool adv_warp_to_node = false;
    bool auto_exec_mnv = false;
    bool flight_assist_menu = false;
    bool adv_embed_mnv = false;
    bool adv_embed_mnv_mini = false;
    bool mnv_popup_mini_hover = false;
    bool show_hud = true;
    bool show_orbit = true;
    bool orbit_reference_sun = false;
    int global_best_ref_node = -1;
    bool transfer_window_menu = false;
    int transfer_target_body = 4;
    PorkchopResult transfer_result;
    bool transfer_result_valid = false;
    int transfer_hover_dep = -1;
    int transfer_hover_tof = -1;




    void RenderTelemetry(entt::registry& registry, entt::entity entity, Renderer* renderer, Renderer3D* r3d, CameraDirector& cam, const RocketAssembly& assembly, const RenderContext& ctx, double dt, int frame, double ws_d, float global_best_ang, int time_warp, float mouse_x, float mouse_y, bool& hlmb_prev_ref);
    void RenderIndicators(entt::registry& registry, entt::entity entity, Renderer* renderer, Renderer3D* r3d, CameraDirector& cam, const RocketAssembly& assembly, const RenderContext& ctx, double dt, int frame, double ws_d, float global_best_ang, int time_warp, float mouse_x, float mouse_y, bool& hlmb_prev_ref);
    void RenderNavball(entt::registry& registry, entt::entity entity, Renderer* renderer, Renderer3D* r3d, CameraDirector& cam, const RocketAssembly& assembly, const RenderContext& ctx, double dt, int frame, double ws_d, float global_best_ang, int time_warp, float mouse_x, float mouse_y, bool& hlmb_prev_ref);
    void RenderAdvancedMenus(entt::registry& registry, entt::entity entity, Renderer* renderer, Renderer3D* r3d, CameraDirector& cam, const RocketAssembly& assembly, const RenderContext& ctx, double dt, int frame, double ws_d, float global_best_ang, int time_warp, float mouse_x, float mouse_y, bool& hlmb_prev_ref);
    void RenderApsides(entt::registry& registry, entt::entity entity, Renderer* renderer, Renderer3D* r3d, CameraDirector& cam, const RocketAssembly& assembly, const RenderContext& ctx, double dt, int frame, double ws_d, float global_best_ang, int time_warp, float mouse_x, float mouse_y, bool& hlmb_prev_ref);
    void RenderManeuver(entt::registry& registry, entt::entity entity, Renderer* renderer, Renderer3D* r3d, CameraDirector& cam, const RocketAssembly& assembly, const RenderContext& ctx, double dt, int frame, double ws_d, float global_best_ang, int time_warp, float mouse_x, float mouse_y, bool& hlmb_prev_ref);
    void render(entt::registry& registry, entt::entity entity, Renderer* renderer, Renderer3D* r3d, 
                CameraDirector& cam, const RocketAssembly& assembly,
                const RenderContext& ctx, double dt, int frame, double ws_d, float global_best_ang, int time_warp,
                float mouse_x, float mouse_y, bool lmb, bool lmb_prev, bool rmb) {
        
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
        
        GLFWwindow* window = GameContext::getInstance().window;
        int ww, wh;
        glfwGetWindowSize(window, &ww, &wh);
        
        const Quat& rocketQuat = ctx.rocketQuat;
        const Vec3& localRight = ctx.localRight;
        const Vec3& rocketUp = ctx.rocketUp;
        const Vec3& localNorth = ctx.localNorth;

        double ro_x = ctx.ro_x;
        double ro_y = ctx.ro_y;
        double ro_z = ctx.ro_z;
        Mat4 viewMat = ctx.viewMat;
        Mat4 macroProjMat = ctx.macroProjMat;
        Vec3 camEye_rel = ctx.camEye_rel;
        float aspect = ctx.aspect;
glDisable(GL_DEPTH_TEST);
    renderer->beginFrame();

    // 坐标转换变量（HUD也需要）
    double scale = 1.0 / (tele.altitude * 1.5 + 200.0);
    float cx = 0.0f;
    float cy = 0.0f;
    double rocket_r = sqrt(trans.px * trans.px + trans.py * trans.py);
    double rocket_theta = atan2(trans.py, trans.px);
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
    float w = std::max(0.015f, (float)(10.0 * scale));
    float h = std::max(0.06f, (float)(40.0 * scale));
    float y_offset = -h / 2.0f;


    // ====================================================================
    // ===== 2D 叠加层 HUD =====
    static bool hlmb_prev = false;

    if (show_hud) {  // 用户按 H 键切换开关
        
        // --- 强制重置 2D 渲染器批处理状态 ---
        // Vulkan: HUD rendered via ImGui/VkHUD; legacy GL calls removed
        renderer->endFrame();
        renderer->beginFrame();

        float hud_opacity = 0.8f;

        float gauge_w = 0.03f;
        float gauge_h = 0.45f;
        float gauge_y_center = 0.4f;
        float gauge_vel_x = -0.92f;
        float gauge_alt_x = -0.84f;
        float gauge_fuel_x = -0.76f;

    double current_vel = sqrt(vel.vx*vel.vx + vel.vy*vel.vy + vel.vz*vel.vz);
    double current_alt = tele.altitude;
    int current_vvel = (int)tele.velocity;

    if (orbit_reference_sun) {
        double G_const = 6.67430e-11;
        double M_sun = 1.989e30;
        double GM_sun = G_const * M_sun;
        double au = 149597870700.0;
        double sun_angular_vel = sqrt(GM_sun / (au * au * au));
        double sun_angle = -1.2 + sun_angular_vel * tele.sim_time;
        double current_sun_px = cos(sun_angle) * au;
        double current_sun_py = sin(sun_angle) * au;
        double current_sun_vx = -sin(sun_angle) * au * sun_angular_vel;
        double current_sun_vy = cos(sun_angle) * au * sun_angular_vel;

        double rel_vx = vel.vx - current_sun_vx;
        double rel_vy = vel.vy - current_sun_vy;
        double rel_vz = vel.vz;
        double rel_px = trans.px - current_sun_px;
        double rel_py = trans.py - current_sun_py;
        double rel_pz = trans.pz;
        
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
    double current_fuel = prop.fuel;

    RenderTelemetry(registry, entity, renderer, r3d, cam, assembly, ctx, dt, frame, ws_d, global_best_ang, time_warp, mouse_x, mouse_y, hlmb_prev);
    RenderIndicators(registry, entity, renderer, r3d, cam, assembly, ctx, dt, frame, ws_d, global_best_ang, time_warp, mouse_x, mouse_y, hlmb_prev);
    RenderNavball(registry, entity, renderer, r3d, cam, assembly, ctx, dt, frame, ws_d, global_best_ang, time_warp, mouse_x, mouse_y, hlmb_prev);
    RenderAdvancedMenus(registry, entity, renderer, r3d, cam, assembly, ctx, dt, frame, ws_d, global_best_ang, time_warp, mouse_x, mouse_y, hlmb_prev);
    RenderApsides(registry, entity, renderer, r3d, cam, assembly, ctx, dt, frame, ws_d, global_best_ang, time_warp, mouse_x, mouse_y, hlmb_prev);
    
    // update hlmb_prev
    bool hlmb = glfwGetMouseButton(GameContext::getInstance().window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS;
    hlmb_prev = hlmb;

    // ========================================================================
    } 
    

    RenderManeuver(registry, entity, renderer, r3d, cam, assembly, ctx, dt, frame, ws_d, global_best_ang, time_warp, mouse_x, mouse_y, hlmb_prev);
    }
};

#include "FlightHUD_Telemetry.h"
#include "FlightHUD_Indicators.h"
#include "FlightHUD_Navball.h"
#include "FlightHUD_AdvancedMenus.h"
#include "FlightHUD_Apsides.h"
#include "FlightHUD_Maneuver.h"
