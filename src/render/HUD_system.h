#pragma once
#include <vector>
#include <string>
#include <cmath>
#include <algorithm>
#include <GLFW/glfw3.h>
#include "render/renderer_2d.h"
#include "core/rocket_state.h"
#include "camera/camera_director.h"
#include "math/math3d.h"
#include "physics/physics_system.h" 
#include "simulation/rocket_builder.h"
#include "simulation/transfer_calculator.h"
#include "render/renderer3d.h"

// Forward declare formatTime from main.cpp
std::string formatTime(double seconds, bool absolute = false);

struct HUDContext {
    Renderer* renderer;
    entt::registry* registry;
    entt::entity entity;
    RocketConfig* rocket_config;
    ControlInput* control_input;
    CameraDirector* cam;
    GLFWwindow* window;
    const RocketAssembly* assembly;
    Renderer3D* r3d;
    int frame;
    const Quat* rocketQuat;
    const Vec3* localRight;
    const Vec3* rocketUp;
    const Vec3* localNorth;
    double ws_d;
    const float* global_best_ang;

    int ww;
    int wh;
    double ro_x;
    double ro_y;
    double ro_z;
    Mat4 viewMat;
    Mat4 macroProjMat;
    Vec3 camEye_rel;
    float aspect;

    int time_warp;
    double dt;

    double mouse_x;
    double mouse_y;
    bool lmb;
    bool lmb_prev;
    bool rmb;
};

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



    void render(HUDContext& ctx) {
        // Unpack references to avoid changing the hud code too much
        Renderer* renderer = ctx.renderer;
        // ECS components accessed via registry
        // --- ECS Component References (replacing rocket_state access) ---
        auto& trans = ctx.registry->get<TransformComponent>(ctx.entity);
        auto& vel   = ctx.registry->get<VelocityComponent>(ctx.entity);
        auto& att   = ctx.registry->get<AttitudeComponent>(ctx.entity);
        auto& prop  = ctx.registry->get<PropulsionComponent>(ctx.entity);
        auto& tele  = ctx.registry->get<TelemetryComponent>(ctx.entity);
        auto& guid  = ctx.registry->get<GuidanceComponent>(ctx.entity);
        auto& mnv   = ctx.registry->get<ManeuverComponent>(ctx.entity);
        auto& orb   = ctx.registry->get<OrbitComponent>(ctx.entity);

        RocketConfig& rocket_config = *ctx.rocket_config;
        ControlInput& control_input = *ctx.control_input;
        CameraDirector& cam = *ctx.cam;
        GLFWwindow* window = ctx.window;
        const RocketAssembly& assembly = *ctx.assembly;
        Renderer3D* r3d = ctx.r3d;
        int frame = ctx.frame;
        const Quat& rocketQuat = *ctx.rocketQuat;
        const Vec3& localRight = *ctx.localRight;
        const Vec3& rocketUp = *ctx.rocketUp;
        const Vec3& localNorth = *ctx.localNorth;
        double ws_d = ctx.ws_d;
        float global_best_ang = *ctx.global_best_ang;


        int ww = ctx.ww;
        int wh = ctx.wh;
        double ro_x = ctx.ro_x;
        double ro_y = ctx.ro_y;
        double ro_z = ctx.ro_z;
        Mat4 viewMat = ctx.viewMat;
        Mat4 macroProjMat = ctx.macroProjMat;
        Vec3 camEye_rel = ctx.camEye_rel;
        float aspect = ctx.aspect;

        int time_warp = ctx.time_warp;
        double dt = ctx.dt;

        float mouse_x = (float)ctx.mouse_x;
        float mouse_y = (float)ctx.mouse_y;
        bool lmb = ctx.lmb;
        bool lmb_prev = ctx.lmb_prev;
        bool rmb = ctx.rmb;

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

    // --- New Detailed Telemetry HUD (Bottom-Left) ---
    float tl_x = -0.98f;
    float tl_y = -0.65f;
    float tl_size = 0.011f;
    float tl_spacing = 0.025f;
    char tl_buf[128];

    auto draw_tl_line = [&](const char* label, const char* value, float r, float g, float b) {
        renderer->drawText(tl_x, tl_y, label, tl_size, 0.7f, 0.7f, 0.7f, hud_opacity);
        renderer->drawText(tl_x + 0.18f, tl_y, value, tl_size, r, g, b, hud_opacity);
        tl_y -= tl_spacing;
    };

    // 1. Latitude and Longitude
    double planet_r = SOLAR_SYSTEM[current_soi_index].radius;
    double lat = asin(fmax(-1.0, fmin(1.0, trans.surf_pz / planet_r))) * 180.0 / PI;
    double lon = atan2(trans.surf_py, trans.surf_px) * 180.0 / PI;
    snprintf(tl_buf, sizeof(tl_buf), "%.4f, %.4f", lat, lon);
    draw_tl_line("LAT/LON:", tl_buf, 0.4f, 0.8f, 1.0f);

    // 1b. Speed and Altitude
    snprintf(tl_buf, sizeof(tl_buf), "%.2f m/s", current_vel);
    draw_tl_line("SPEED:", tl_buf, 1.0f, 1.0f, 0.4f);
    if (current_alt > 100000.0)
        snprintf(tl_buf, sizeof(tl_buf), "%.2f km", current_alt / 1000.0);
    else
        snprintf(tl_buf, sizeof(tl_buf), "%.1f m", current_alt);
    draw_tl_line("ALTITUDE:", tl_buf, 0.4f, 1.0f, 0.8f);

    // 2. Thrust and TWR
    double current_thrust = prop.thrust_power;
    double g_local = 9.80665 * pow(6371000.0 / (6371000.0 + current_alt), 2);
    double total_mass = assembly.total_dry_mass + prop.fuel;
    double twr = (total_mass > 0) ? current_thrust / (total_mass * g_local) : 0;
    snprintf(tl_buf, sizeof(tl_buf), "%.2f kN (TWR: %.2f)", current_thrust / 1000.0, twr);
    draw_tl_line("THRUST:", tl_buf, 1.0f, 0.6f, 0.2f);

    // 3. Mass and Fuel
    snprintf(tl_buf, sizeof(tl_buf), "%.1f t (Fuel: %.1f t)", total_mass / 1000.0, prop.fuel / 1000.0);
    draw_tl_line("MASS:", tl_buf, 0.8f, 0.8f, 0.8f);

    // 4. Remaining Delta-V
    double dv_rem = 0;
    if (total_mass > assembly.total_dry_mass && assembly.avg_isp > 0) {
        dv_rem = assembly.avg_isp * 9.80665 * log(total_mass / assembly.total_dry_mass);
    }
    snprintf(tl_buf, sizeof(tl_buf), "%.1f m/s", dv_rem);
    draw_tl_line("REMAIN DV:", tl_buf, 0.3f, 0.9f, 0.4f);

    // 5. Gravity and Drag
    double drag_mag = fabs(vel.acceleration) * total_mass - current_thrust; // Rough estimate
    if (drag_mag < 0) drag_mag = 0;
    snprintf(tl_buf, sizeof(tl_buf), "%.3f m/s2 (Drag: %.1f N)", g_local, drag_mag);
    draw_tl_line("GRAVITY:", tl_buf, 0.6f, 0.7f, 1.0f);

    // 6. Orbital Elements
    tl_y -= 0.015f; // Extra space for orbital section
    renderer->drawText(tl_x, tl_y, "ORBITAL ELEMENTS:", tl_size, 0.5f, 0.9f, 1.0f, hud_opacity);
    tl_y -= tl_spacing;

    // Calculate elements (Simple 2D/3D approximation for HUD)
    double mu = 3.986e14;
    double r_mag = sqrt(trans.px*trans.px + trans.py*trans.py + trans.pz*trans.pz);
    double v_sq = vel.vx*vel.vx + vel.vy*vel.vy + vel.vz*vel.vz;
    double specific_energy = v_sq / 2.0 - mu / r_mag;
    double a_val = -mu / (2.0 * specific_energy);
    
    Vec3 r_vec(trans.px, trans.py, trans.pz);
    Vec3 v_vec(vel.vx, vel.vy, vel.vz);
    Vec3 h_vec = r_vec.cross(v_vec);
    double h_mag = h_vec.length();
    double e_val = sqrt(1.0 + (2.0 * specific_energy * h_mag * h_mag) / (mu * mu));
    double inc = acos(h_vec.z / h_mag) * 180.0 / PI;

    snprintf(tl_buf, sizeof(tl_buf), "a: %.1f km, e: %.4f", a_val / 1000.0, e_val);
    draw_tl_line("A/E:", tl_buf, 1.0f, 1.0f, 1.0f);
    snprintf(tl_buf, sizeof(tl_buf), "i: %.2f deg", inc);
    draw_tl_line("INC:", tl_buf, 1.0f, 1.0f, 1.0f);

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
    int pitch_deg = (int)(abs(att.angle_z) * 180.0 / PI);
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
        guid.auto_mode = !guid.auto_mode;
        if (guid.auto_mode) {
           guid.pid_vert.reset();
           guid.pid_pos.reset();
           guid.pid_att.reset();
           guid.pid_att_z.reset();
           guid.mission_msg = ">> AUTOPILOT ENGAGED (MOUSE)";
        } else {
           guid.mission_msg = ">> MANUAL CONTROL ACTIVE (MOUSE)";
        }
    }
    // hlmb_prev updated after all HUD interactions

    renderer->addRect(mode_x, mode_y, mode_w, mode_h, 0.05f, 0.05f, 0.05f, 0.7f);
    if (guid.auto_mode) {
      float pulse = is_hover_mode ? 1.0f : 0.8f;
      renderer->addRect(mode_x, mode_y, mode_w - 0.02f, mode_h - 0.15f, 0.1f * pulse, 0.8f * pulse, 0.2f * pulse, 0.9f);
      renderer->drawText(mode_x, mode_y, "AUTO", 0.020f, 0.1f, 0.1f, 0.1f, 0.9f, false, Renderer::CENTER);
    } else {
      float pulse = is_hover_mode ? 1.0f : 0.8f;
      renderer->addRect(mode_x, mode_y, mode_w - 0.02f, mode_h - 0.15f, 1.0f * pulse, 0.6f * pulse, 0.1f * pulse, 0.9f);
      renderer->drawText(mode_x, mode_y, "MANUAL", 0.020f, 0.1f, 0.1f, 0.1f, 0.9f, false, Renderer::CENTER);
    }

    // --- 7. Mission Status & Time (Top Center) ---
    renderer->drawText(0.0f, 0.85f, "[MISSION CONTROL]", 0.016f, 0.4f, 1.0f, 0.4f, hud_opacity, true, Renderer::CENTER);
    renderer->drawText(0.0f, 0.80f, guid.mission_msg.c_str(), 0.015f, 0.8f, 0.8f, 1.0f, hud_opacity, true, Renderer::CENTER);

    // Free Camera Speed Display
    if (cam.mode == 3) {
      char speed_buf[64];
      if (cam.free_move_speed > 1000000.0f)
          snprintf(speed_buf, sizeof(speed_buf), "FREE CAM SPEED: %.2f km/s", cam.free_move_speed / 1000.0f);
      else
          snprintf(speed_buf, sizeof(speed_buf), "FREE CAM SPEED: %.1f m/s", cam.free_move_speed);
      renderer->drawText(0.0f, 0.75f, speed_buf, 0.015f, 0.4f, 1.0f, 1.0f, 0.9f, true, Renderer::CENTER);
    }

    // Time Display & Toggle
    float time_y = 0.92f;
    float time_w = 0.35f;
    float time_h = 0.04f;
    bool is_hover_time = (hmouse_x >= -time_w/2.0f && hmouse_x <= time_w/2.0f &&
                          hmouse_y >= time_y - time_h/2.0f && hmouse_y <= time_y + time_h/2.0f);
    
    if (is_hover_time && hlmb && !hlmb_prev) {
        guid.show_absolute_time = !guid.show_absolute_time;
    }

    renderer->addRect(0.0f, time_y, time_w, time_h, 0.05f, 0.05f, 0.05f, 0.7f);
    string time_str = formatTime(guid.show_absolute_time ? tele.sim_time : guid.mission_timer, guid.show_absolute_time);
    renderer->drawText(0.0f, time_y, time_str.c_str(), 0.018f, 1.0f, 1.0f, 1.0f, 0.9f, true, Renderer::CENTER);

    // Time Warp Display
    if (time_warp > 1) {
        char warp_buf[32];
        snprintf(warp_buf, sizeof(warp_buf), "WARP: %dx", time_warp);
        renderer->drawText(0.25f, time_y, warp_buf, 0.015f, 1.0f, 0.8f, 0.1f, 0.9f, true, Renderer::LEFT);
    }

    // --- 8. Stage Indicator (Below mode indicator) ---
    if (prop.total_stages > 1) {
        float stage_x = 0.88f;
        float stage_y = mode_y - 0.06f;
        float stage_w = 0.15f;
        float stage_h = 0.04f;
        renderer->addRect(stage_x, stage_y, stage_w, stage_h, 0.05f, 0.05f, 0.1f, 0.7f);
        
        // Stage number display
        char stage_str[32];
        snprintf(stage_str, sizeof(stage_str), "STG %d/%d", prop.current_stage + 1, prop.total_stages);
        
        float sr = 0.3f, sg = 0.8f, sb = 1.0f;
        if (prop.fuel < 1000 && prop.current_stage < prop.total_stages - 1) {
            // Blink warning when fuel low and can still stage
            float blink = 0.5f + 0.5f * sinf((float)glfwGetTime() * 6.0f);
            sr = 1.0f * blink; sg = 0.5f * blink; sb = 0.1f;
        }
        renderer->drawText(stage_x, stage_y, stage_str, 0.016f, sr, sg, sb, 0.9f, false, Renderer::CENTER);
        
        // Per-stage fuel bars (small bars below stage indicator)
        float bar_y_start = stage_y - 0.04f;
        float bar_w = stage_w * 0.8f;
        float bar_h = 0.012f;
        for (int si = 0; si < prop.total_stages; si++) {
            float by = bar_y_start - si * (bar_h + 0.005f);
            // Background
            renderer->addRect(stage_x, by, bar_w, bar_h, 0.15f, 0.15f, 0.15f, 0.5f);
            // Fill
            if (si < (int)prop.stage_fuels.size() && si < (int)rocket_config.stage_configs.size()) {
                float cap = (float)rocket_config.stage_configs[si].fuel_capacity;
                float ratio = (cap > 0) ? (float)(prop.stage_fuels[si] / cap) : 0.0f;
                ratio = fmaxf(0.0f, fminf(1.0f, ratio));
                float fill = bar_w * ratio;
                float fx = stage_x - bar_w / 2.0f + fill / 2.0f;
                float fr = (si == prop.current_stage) ? 0.2f : 0.4f;
                float fg = (si == prop.current_stage) ? 1.0f : 0.5f;
                float fb = (si == prop.current_stage) ? 0.4f : 0.3f;
                if (si < prop.current_stage) { fr = 0.3f; fg = 0.3f; fb = 0.3f; } // jettisoned
                renderer->addRect(fx, by, fill, bar_h * 0.7f, fr, fg, fb, 0.8f);
            }
        }
    }
    
    // 保存指示器 (每次保存后闪烁3秒)
    static int last_save_frame = -1000;
    if (frame % 300 == 0 && guid.status != PRE_LAUNCH) {
        last_save_frame = frame;
        SaveSystem::SaveGame(*ctx.assembly, *ctx.registry, ctx.entity, *ctx.control_input);
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
        double rel_vx = vel.vx, rel_vy = vel.vy, rel_vz = vel.vz;
        double rel_px = trans.px, rel_py = trans.py, rel_pz = trans.pz;
        
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
    if (mnv.selected_maneuver_index != -1 && (size_t)mnv.selected_maneuver_index < mnv.maneuvers.size()) {
        ManeuverNode& node = mnv.maneuvers[mnv.selected_maneuver_index];
        
        if (node.snap_valid) {
            // High-Precision Target Orbit Tracking (Principia Style)
            Vec3 rem_v = ManeuverSystem::calculateRemainingDV(vel, tele, node);
            dv_remaining = (double)rem_v.length();
            vManeuver = rem_v.normalized();
        } else {
            // Pre-ignition: Use projected state at node
            int ref_idx = (node.ref_body >= 0) ? node.ref_body : current_soi_index;
            double mu = G_const * SOLAR_SYSTEM[ref_idx].mass;
            double npx, npy, npz, nvx, nvy, nvz;
            get3DStateAtTime(trans.px, trans.py, trans.pz, vel.vx, vel.vy, vel.vz, mu, node.sim_time - tele.sim_time, npx, npy, npz, nvx, nvy, nvz);
            ManeuverFrame frame = ManeuverSystem::getFrame(Vec3((float)npx, (float)npy, (float)npz), Vec3((float)nvx, (float)nvy, (float)nvz));
            vManeuver = (frame.prograde * node.delta_v.x + frame.normal * node.delta_v.y + frame.radial * node.delta_v.z).normalized();
            dv_remaining = (double)node.delta_v.length();
        }
    }

    // 直接使用四元数投影，彻底解决万向锁和翻转问题
    renderer->drawAttitudeSphere(nav_x, nav_y, nav_rad, rocketQuat, localRight, rocketUp, localNorth, guid.sas_active, guid.rcs_active, vPrograde, vNormal, vRadial, vManeuver);

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
            guid.sas_mode = sas_btns[i].mode;
            guid.sas_active = true;
            guid.auto_mode = false; // Override autopilot
            cout << ">> SAS MODE: " << sas_btns[i].label << " (Active)" << endl;
        }
        
        float alpha = (guid.sas_mode == sas_btns[i].mode) ? 0.9f : 0.4f;
        if (hover) alpha += 0.1f;
        renderer->addRect(bx, by, btn_w, btn_h, sas_btns[i].r, sas_btns[i].g, sas_btns[i].b, alpha);
        renderer->drawText(bx, by, sas_btns[i].label, 0.012f, 1, 1, 1, 0.9f, true, Renderer::CENTER);
    }
    
    // --- 9. Advanced Orbit UI (Right Edge) ---
    if (cam.mode == 2 && !SOLAR_SYSTEM.empty()) {
        float adv_btn_w = 0.15f;
        float adv_btn_h = 0.05f;
        float adv_btn_x = 0.88f;
        float adv_btn_y = mode_y - 0.35f; // below stage UI
        
        bool hover_adv = (hmouse_x >= adv_btn_x - adv_btn_w/2 && hmouse_x <= adv_btn_x + adv_btn_w/2 && hmouse_y >= adv_btn_y - adv_btn_h/2 && hmouse_y <= adv_btn_y + adv_btn_h/2);
        if (hover_adv && hlmb && !hlmb_prev) adv_orbit_menu = !adv_orbit_menu;
        
        renderer->addRect(adv_btn_x, adv_btn_y, adv_btn_w, adv_btn_h, 0.2f, 0.4f, 0.8f, hover_adv ? 0.9f : 0.7f);
        renderer->drawText(adv_btn_x, adv_btn_y, "ADV ORBIT", 0.012f, 1, 1, 1, 1.0f, true, Renderer::CENTER);

        // --- Flight Assist Button (Below ADV ORBIT) ---
        float fa_btn_y = adv_btn_y - 0.06f;
        bool hover_fa = (hmouse_x >= adv_btn_x - adv_btn_w/2 && hmouse_x <= adv_btn_x + adv_btn_w/2 && hmouse_y >= fa_btn_y - adv_btn_h/2 && hmouse_y <= fa_btn_y + adv_btn_h/2);
        if (hover_fa && hlmb && !hlmb_prev) flight_assist_menu = !flight_assist_menu;
        
        renderer->addRect(adv_btn_x, fa_btn_y, adv_btn_w, adv_btn_h, 0.8f, 0.4f, 0.2f, hover_fa ? 0.9f : 0.7f);
        renderer->drawText(adv_btn_x, fa_btn_y, "FLIGHT ASSIST", 0.012f, 1, 1, 1, 1.0f, true, Renderer::CENTER);

        // --- Galaxy Info Button ---
        float galaxy_btn_y = fa_btn_y - 0.06f;
        bool hover_galaxy = (hmouse_x >= adv_btn_x - adv_btn_w/2 && hmouse_x <= adv_btn_x + adv_btn_w/2 && hmouse_y >= galaxy_btn_y - adv_btn_h/2 && hmouse_y <= galaxy_btn_y + adv_btn_h/2);
        if (hover_galaxy && hlmb && !hlmb_prev) show_galaxy_info = !show_galaxy_info;
        
        renderer->addRect(adv_btn_x, galaxy_btn_y, adv_btn_w, adv_btn_h, 0.1f, 0.6f, 0.3f, hover_galaxy ? 0.9f : 0.7f);
        renderer->drawText(adv_btn_x, galaxy_btn_y, "GALAXY INFO", 0.012f, 1, 1, 1, 1.0f, true, Renderer::CENTER);

        // --- Climate View Button ---
        float climate_btn_y = galaxy_btn_y - 0.06f;
        bool hover_climate = (hmouse_x >= adv_btn_x - adv_btn_w/2 && hmouse_x <= adv_btn_x + adv_btn_w/2 && hmouse_y >= climate_btn_y - adv_btn_h/2 && hmouse_y <= climate_btn_y + adv_btn_h/2);
        if (hover_climate && hlmb && !hlmb_prev) {
            r3d->setClimateViewMode((r3d->climateViewMode + 1) % 5);
        }
        
        static const char* climate_mode_names[] = {"NORMAL", "TEMPERATURE", "PRECIPITATION", "PRESSURE", "HYDROLOGY"};
        renderer->addRect(adv_btn_x, climate_btn_y, adv_btn_w, adv_btn_h, 0.4f, 0.2f, 0.6f, hover_climate ? 0.9f : 0.7f);
        renderer->drawText(adv_btn_x, climate_btn_y, "CLIMATE VIEW", 0.010f, 1, 1, 1, 1.0f, true, Renderer::CENTER);
        renderer->drawText(adv_btn_x, climate_btn_y - 0.022f, climate_mode_names[r3d->climateViewMode], 0.007f, 0.7f, 0.9f, 1.0f, 0.9f, true, Renderer::CENTER);

        if (show_galaxy_info) {
            float bar_h = 0.12f;
            float bar_y = 0.92f;
            renderer->addRect(0, bar_y, 2.0f, bar_h, 0.05f, 0.05f, 0.08f, 0.9f);
            renderer->addLine(-1.0f, bar_y - bar_h/2, 1.0f, bar_y - bar_h/2, 0.002f, 0.4f, 0.8f, 1.0f, 0.6f);
            
            float icon_start_x = -0.92f;
            float icon_spacing = 0.12f;
            float icon_r = 0.04f;
            
            // 1. Draw Suns and Planets (parent_index == -1)
            int main_count = 0;
            for (int i = 0; i < (int)SOLAR_SYSTEM.size(); i++) {
                if (SOLAR_SYSTEM[i].parent_index != -1) continue;
                
                float ix = icon_start_x + main_count * icon_spacing;
                bool hover_icon = (hmouse_x >= ix - icon_r && hmouse_x <= ix + icon_r && hmouse_y >= bar_y - icon_r && hmouse_y <= bar_y + icon_r);
                
                if (hover_icon && hlmb && !hlmb_prev) {
                    selected_body_idx = i;
                    // Logic: Planet click expands/collapses moons
                    bool has_moons = false;
                    for(int m=0; m<(int)SOLAR_SYSTEM.size(); m++) if(SOLAR_SYSTEM[m].parent_index == i) has_moons = true;
                    
                    if (has_moons) {
                        if (expanded_planet_idx == i) expanded_planet_idx = -1;
                        else expanded_planet_idx = i;
                    }
                }
                
                renderer->drawPlanetIcon(ix, bar_y, icon_r, SOLAR_SYSTEM[i], (float)glfwGetTime());
                if (hover_icon || selected_body_idx == i) 
                    renderer->addCircleOutline(ix, bar_y, icon_r * 1.1f, 0.003f, 0.4f, 0.8f, 1.0f, 1.0f);
                
                renderer->drawText(ix, bar_y - icon_r - 0.015f, SOLAR_SYSTEM[i].name.c_str(), 0.008f, 1, 1, 1, 1.0f, true, Renderer::CENTER);
                main_count++;
            }
            
            // 2. Draw expansion moons if a planet is selected
            if (expanded_planet_idx != -1) {
                float arrow_x = icon_start_x + (expanded_planet_idx == 0 ? 0 : (expanded_planet_idx < 4 ? expanded_planet_idx : expanded_planet_idx - 1)) * icon_spacing;
                renderer->drawText(arrow_x, bar_y - 0.07f, "[v MOONS v]", 0.007f, 0.4f, 1.0f, 0.6f, 1.0f, true, Renderer::CENTER);
                
                float moon_bar_y = bar_y - 0.15f;
                float moon_bar_h = 0.10f;
                renderer->addRect(0, moon_bar_y, 2.0f, moon_bar_h, 0.04f, 0.04f, 0.06f, 0.85f);
                
                int moon_count = 0;
                for (int i = 0; i < (int)SOLAR_SYSTEM.size(); i++) {
                    if (SOLAR_SYSTEM[i].parent_index == expanded_planet_idx) {
                        float mix = icon_start_x + moon_count * icon_spacing;
                        float mr = 0.03f;
                        bool hover_moon = (hmouse_x >= mix - mr && hmouse_x <= mix + mr && hmouse_y >= moon_bar_y - mr && hmouse_y <= moon_bar_y + mr);
                        
                        if (hover_moon && hlmb && !hlmb_prev) {
                            selected_body_idx = i;
                        }
                        
                        renderer->drawPlanetIcon(mix, moon_bar_y, mr, SOLAR_SYSTEM[i], (float)glfwGetTime());
                        if (hover_moon || selected_body_idx == i)
                             renderer->addCircleOutline(mix, moon_bar_y, mr * 1.1f, 0.002f, 0.4f, 1.0f, 0.6f, 1.0f);
                        
                        renderer->drawText(mix, moon_bar_y - mr - 0.012f, SOLAR_SYSTEM[i].name.c_str(), 0.007f, 0.8f, 1.0f, 0.8f, 1.0f, true, Renderer::CENTER);
                        moon_count++;
                    }
                }
            }
        }

        // --- Detailed Info Panel (Top Left) ---
        if (selected_body_idx != -1 && selected_body_idx < (int)SOLAR_SYSTEM.size()) {
            const CelestialBody& b = SOLAR_SYSTEM[selected_body_idx];
            float panel_w = 0.35f;
            float panel_h = 0.45f;
            float panel_x = -1.0f + panel_w/2.0f + 0.02f;
            float panel_y = 0.92f - (show_galaxy_info ? 0.12f : 0) - (expanded_planet_idx != -1 ? 0.10f : 0) - panel_h/2.0f - 0.02f;
            
            renderer->addRect(panel_x, panel_y, panel_w, panel_h, 0.03f, 0.03f, 0.05f, 0.85f);
            renderer->addRectOutline(panel_x, panel_y, panel_w, panel_h, 0.4f, 0.8f, 1.0f, 0.7f);
            
            float tx = panel_x - panel_w/2.0f + 0.02f;
            float ty = panel_y + panel_h/2.0f - 0.04f;
            float line_h = 0.025f;
            char buf[128];
            
            renderer->drawText(tx, ty, b.name.c_str(), 0.018f, b.r, b.g, b.b, 1.0f);
            ty -= line_h * 1.5f;
            
            const char* types[] = {"STAR", "TERRESTRIAL", "GAS GIANT", "MOON", "RINGED GIANT"};
            snprintf(buf, sizeof(buf), "TYPE: %s", types[b.type]);
            renderer->drawText(tx, ty, buf, 0.010f, 0.7f, 0.7f, 0.7f); ty -= line_h;
            
            snprintf(buf, sizeof(buf), "RADIUS: %.1f km", b.radius / 1000.0);
            renderer->drawText(tx, ty, buf, 0.009f, 1, 1, 1); ty -= line_h;
            
            snprintf(buf, sizeof(buf), "MASS: %.3e kg", b.mass);
            renderer->drawText(tx, ty, buf, 0.009f, 1, 1, 1); ty -= line_h;
            
            double g_surf = (6.674e-11 * b.mass) / (b.radius * b.radius);
            snprintf(buf, sizeof(buf), "SURFACE G: %.3f m/s2", g_surf);
            renderer->drawText(tx, ty, buf, 0.009f, 1, 1, 1); ty -= line_h;
            
            double volume = (4.0/3.0) * 3.14159 * pow(b.radius, 3);
            double density = b.mass / volume;
            snprintf(buf, sizeof(buf), "DENSITY: %.1f kg/m3", density);
            renderer->drawText(tx, ty, buf, 0.009f, 1, 1, 1); ty -= line_h;
            
            snprintf(buf, sizeof(buf), "ORBIT PERIOD: %.2f days", b.orbital_period / 86400.0);
            renderer->drawText(tx, ty, buf, 0.009f, 1, 1, 1); ty -= line_h;
            
            snprintf(buf, sizeof(buf), "ECCENTRICITY: %.4f", b.eccentricity);
            renderer->drawText(tx, ty, buf, 0.009f, 1, 1, 1); ty -= line_h;
            
            snprintf(buf, sizeof(buf), "INCLINATION: %.2f deg", b.inclination * 180.0 / 3.14159);
            renderer->drawText(tx, ty, buf, 0.009f, 1, 1, 1); ty -= line_h;
            
            snprintf(buf, sizeof(buf), "SEMI-MAJOR AXIS: %.2f AU", b.sma_base / 1.496e11);
            renderer->drawText(tx, ty, buf, 0.009f, 1, 1, 1); ty -= line_h;
            
            snprintf(buf, sizeof(buf), "ATMOSPHERE: %.2f hPa", b.surface_pressure);
            renderer->drawText(tx, ty, buf, 0.009f, 1, 1, 1); ty -= line_h;
            
            snprintf(buf, sizeof(buf), "TEMPERATURE: %.1f K", b.average_temp);
            renderer->drawText(tx, ty, buf, 0.009f, 1, 1, 1); ty -= line_h;
            
            snprintf(buf, sizeof(buf), "AXIAL TILT: %.2f deg", b.axial_tilt * 180.0 / 3.14159);
            renderer->drawText(tx, ty, buf, 0.009f, 1, 1, 1); ty -= line_h;
            
            int moon_count = 0;
            for(int m=0; m<(int)SOLAR_SYSTEM.size(); m++) if(SOLAR_SYSTEM[m].parent_index == selected_body_idx) moon_count++;
            snprintf(buf, sizeof(buf), "MOONS: %d", moon_count);
            renderer->drawText(tx, ty, buf, 0.009f, 1, 1, 1); ty -= line_h;
            
            snprintf(buf, sizeof(buf), "ALBEDO (SCAT): %.3f", b.scattering_coef);
            renderer->drawText(tx, ty, buf, 0.009f, 1, 1, 1); ty -= line_h;
        }

        if (flight_assist_menu) {
            float menu_w = 0.25f;
            float menu_h = 0.27f;
            float menu_x = adv_btn_x - adv_btn_w/2 - menu_w/2 - 0.02f;
            float menu_y = fa_btn_y;
            renderer->addRect(menu_x, menu_y, menu_w, menu_h, 0.1f, 0.05f, 0.05f, 0.85f);
            renderer->addRectOutline(menu_x, menu_y, menu_w, menu_h, 1.0f, 0.6f, 0.4f, 0.8f);

            auto draw_toggle = [&](float y, const char* label, bool active, bool& toggle_var, std::function<void()> on_true = nullptr) {
                float ty = menu_y + menu_h/2 - y;
                bool hover = (hmouse_x >= menu_x - menu_w/2 && hmouse_x <= menu_x + menu_w/2 && hmouse_y >= ty - 0.025f && hmouse_y <= ty + 0.025f);
                if (hover && hlmb && !hlmb_prev) {
                    toggle_var = !toggle_var;
                    if (toggle_var && on_true) on_true();
                }
                renderer->addRect(menu_x, ty, menu_w * 0.9f, 0.04f, hover ? 0.3f : 0.15f, hover ? 0.3f : 0.15f, hover ? 0.3f : 0.15f, 0.8f);
                renderer->drawText(menu_x - menu_w * 0.4f, ty, label, 0.012f, 1, 1, 1, 0.9f, true, Renderer::LEFT);
                renderer->drawText(menu_x + menu_w * 0.35f, ty, active ? "ON" : "OFF", 0.012f, active ? 0.2f : 1.0f, active ? 1.0f : 0.2f, 0.2f, 0.9f, true, Renderer::CENTER);
            };

            draw_toggle(0.05f, "AUTO EXECUTE MNV", auto_exec_mnv, auto_exec_mnv);
            
            bool is_ascent = (guid.status == ASCEND);
            bool temp_ascent = is_ascent;
            draw_toggle(0.10f, "AUTO ORBIT", is_ascent, temp_ascent, [&](){
                guid.status = ASCEND;
                guid.mission_phase = 0;
                guid.auto_mode = true;
                guid.mission_msg = "AUTOPILOT: INITIATING ASCENT...";
            });

            bool is_descent = (guid.status == DESCEND);
            bool temp_descent = is_descent;
            draw_toggle(0.15f, "AUTO LANDING", is_descent, temp_descent, [&](){
                guid.status = DESCEND;
                guid.auto_mode = true;
                guid.mission_msg = "AUTOPILOT: INITIATING LANDING...";
            });

            // Transfer Window button
            {
                float ty = menu_y + menu_h/2 - 0.20f;
                bool hover = (hmouse_x >= menu_x - menu_w/2 && hmouse_x <= menu_x + menu_w/2 && hmouse_y >= ty - 0.025f && hmouse_y <= ty + 0.025f);
                if (hover && hlmb && !hlmb_prev) {
                    transfer_window_menu = !transfer_window_menu;
                }
                float btn_r = transfer_window_menu ? 0.3f : 0.15f;
                float btn_g = transfer_window_menu ? 0.5f : 0.15f;
                float btn_b = transfer_window_menu ? 0.8f : 0.15f;
                if (hover) { btn_r += 0.1f; btn_g += 0.1f; btn_b += 0.1f; }
                renderer->addRect(menu_x, ty, menu_w * 0.9f, 0.04f, btn_r, btn_g, btn_b, 0.8f);
                renderer->drawText(menu_x - menu_w * 0.4f, ty, "TRANSFER WINDOW", 0.011f, 0.3f, 0.8f, 1.0f, 0.9f, true, Renderer::LEFT);
                renderer->drawText(menu_x + menu_w * 0.35f, ty, transfer_window_menu ? "[v]" : "[>]", 0.011f, 0.6f, 0.8f, 1.0f, 0.9f, true, Renderer::CENTER);
            }
        }

        // === Transfer Window Popup Panel ===
        if (transfer_window_menu && flight_assist_menu) {
            float tw_w = 0.55f;
            float tw_h = 0.60f;
            float tw_x = adv_btn_x - adv_btn_w/2 - 0.25f - tw_w/2 - 0.04f;
            float tw_y = fa_btn_y - 0.05f;

            // Background panel
            renderer->addRect(tw_x, tw_y, tw_w, tw_h, 0.04f, 0.04f, 0.08f, 0.92f);
            renderer->addRectOutline(tw_x, tw_y, tw_w, tw_h, 0.3f, 0.6f, 1.0f, 0.9f);
            renderer->addRect(tw_x, tw_y + tw_h*0.42f, tw_w, tw_h*0.16f, 0.06f, 0.06f, 0.14f, 0.3f);

            // Title
            float title_y = tw_y + tw_h/2 - 0.02f;
            renderer->drawText(tw_x, title_y, "TRANSFER WINDOW CALCULATOR", 0.013f, 0.4f, 0.8f, 1.0f, 1.0f, true, Renderer::CENTER);

            // --- Target Body Selector ---
            float sel_y = title_y - 0.04f;
            renderer->drawText(tw_x - tw_w*0.4f, sel_y, "Target:", 0.011f, 0.8f, 0.8f, 0.8f, 1.0f, true, Renderer::LEFT);

            // Left arrow
            float arr_lx = tw_x - 0.02f;
            bool hover_tgt_l = (hmouse_x >= arr_lx - 0.015f && hmouse_x <= arr_lx + 0.015f && hmouse_y >= sel_y - 0.015f && hmouse_y <= sel_y + 0.015f);
            renderer->drawText(arr_lx, sel_y, "<", 0.014f, 1,1,1, hover_tgt_l ? 1.0f : 0.5f, true, Renderer::CENTER);
            if (hover_tgt_l && hlmb && !hlmb_prev) {
                int origin = TransferCalculator::getTransferOriginBody();
                do { transfer_target_body--; if (transfer_target_body < 1) transfer_target_body = (int)SOLAR_SYSTEM.size()-1; }
                while (transfer_target_body == origin || transfer_target_body == 4 || transfer_target_body == 0);
                transfer_result_valid = false;
            }

            // Body name
            if (transfer_target_body >= 0 && transfer_target_body < (int)SOLAR_SYSTEM.size())
                renderer->drawText(tw_x + 0.06f, sel_y, SOLAR_SYSTEM[transfer_target_body].name.c_str(), 0.012f, 1.0f, 0.9f, 0.3f, 1.0f, true, Renderer::CENTER);

            // Right arrow
            float arr_rx = tw_x + 0.14f;
            bool hover_tgt_r = (hmouse_x >= arr_rx - 0.015f && hmouse_x <= arr_rx + 0.015f && hmouse_y >= sel_y - 0.015f && hmouse_y <= sel_y + 0.015f);
            renderer->drawText(arr_rx, sel_y, ">", 0.014f, 1,1,1, hover_tgt_r ? 1.0f : 0.5f, true, Renderer::CENTER);
            if (hover_tgt_r && hlmb && !hlmb_prev) {
                int origin = TransferCalculator::getTransferOriginBody();
                do { transfer_target_body++; if (transfer_target_body >= (int)SOLAR_SYSTEM.size()) transfer_target_body = 1; }
                while (transfer_target_body == origin || transfer_target_body == 4 || transfer_target_body == 0);
                transfer_result_valid = false;
            }

            // --- CALCULATE Button ---
            float calc_y = sel_y - 0.045f;
            float calc_w = tw_w * 0.45f;
            float calc_h = 0.035f;
            bool hover_calc = (hmouse_x >= tw_x - calc_w/2 && hmouse_x <= tw_x + calc_w/2 && hmouse_y >= calc_y - calc_h/2 && hmouse_y <= calc_y + calc_h/2);
            renderer->addRect(tw_x, calc_y, calc_w, calc_h, hover_calc ? 0.2f : 0.1f, hover_calc ? 0.6f : 0.4f, hover_calc ? 1.0f : 0.8f, 0.9f);
            renderer->addRectOutline(tw_x, calc_y, calc_w, calc_h, 0.3f, 0.7f, 1.0f, 0.8f);
            renderer->drawText(tw_x, calc_y, "CALCULATE", 0.012f, 1,1,1, 1.0f, true, Renderer::CENTER);

            if (hover_calc && hlmb && !hlmb_prev) {
                int origin = TransferCalculator::getTransferOriginBody();
                transfer_result = TransferCalculator::computePorkchop(origin, transfer_target_body, tele.sim_time, 40);
                transfer_result_valid = transfer_result.computed;
                transfer_hover_dep = -1;
                transfer_hover_tof = -1;
            }

            // --- Porkchop Plot ---
            if (transfer_result_valid && transfer_result.computed) {
                float plot_lx = tw_x - tw_w * 0.38f;  // left edge
                float plot_rx = tw_x + tw_w * 0.38f;  // right edge
                float plot_ty = calc_y - 0.04f;        // top (below calc button)
                float plot_by = tw_y - tw_h/2 + 0.08f; // bottom (above info area)
                float plot_w = plot_rx - plot_lx;
                float plot_h = plot_ty - plot_by;

                // Axis labels
                renderer->drawText(tw_x, plot_ty + 0.015f, "Departure (days from now) ->", 0.008f, 0.6f, 0.6f, 0.6f, 0.8f, true, Renderer::CENTER);
                // Y-axis label (rotated text not available, use short label)
                renderer->drawText(plot_lx - 0.02f, (plot_ty + plot_by)/2, "TOF", 0.008f, 0.6f, 0.6f, 0.6f, 0.8f, true, Renderer::CENTER);

                // Compute Δv range for color mapping
                double dv_min_plot = transfer_result.min_dv;
                double dv_max_plot = dv_min_plot * 5.0; // clamp upper range
                if (dv_max_plot < dv_min_plot + 1000.0) dv_max_plot = dv_min_plot + 1000.0;

                int gn = transfer_result.n_dep;
                float cell_w = plot_w / gn;
                float cell_h = plot_h / gn;

                transfer_hover_dep = -1;
                transfer_hover_tof = -1;

                for (int gi = 0; gi < gn; gi++) {
                    for (int gj = 0; gj < gn; gj++) {
                        int idx = gi * gn + gj;
                        const PorkchopPoint& pt = transfer_result.grid[idx];

                        float cx = plot_lx + (gi + 0.5f) * cell_w;
                        float cy = plot_by + (gj + 0.5f) * cell_h;

                        // Check hover
                        bool cell_hover = (hmouse_x >= cx - cell_w/2 && hmouse_x <= cx + cell_w/2 &&
                                          hmouse_y >= cy - cell_h/2 && hmouse_y <= cy + cell_h/2);
                        if (cell_hover) { transfer_hover_dep = gi; transfer_hover_tof = gj; }

                        if (!pt.valid) {
                            renderer->addRect(cx, cy, cell_w * 0.95f, cell_h * 0.95f, 0.08f, 0.08f, 0.08f, 0.6f);
                            continue;
                        }

                        // Color mapping: green (low Δv) -> yellow -> red (high Δv)
                        float t = (float)((pt.dv_total - dv_min_plot) / (dv_max_plot - dv_min_plot));
                        t = fmaxf(0.0f, fminf(1.0f, t));

                        float cr, cg, cb;
                        if (t < 0.5f) {
                            float s = t * 2.0f;
                            cr = s; cg = 1.0f; cb = 0.0f; // green -> yellow
                        } else {
                            float s = (t - 0.5f) * 2.0f;
                            cr = 1.0f; cg = 1.0f - s; cb = 0.0f; // yellow -> red
                        }

                        float alpha = cell_hover ? 1.0f : 0.85f;
                        renderer->addRect(cx, cy, cell_w * 0.95f, cell_h * 0.95f, cr, cg, cb, alpha);
                    }
                }

                // Mark minimum Δv cell with white crosshair
                if (transfer_result.min_dv_index >= 0) {
                    int mi = transfer_result.min_dv_index / gn;
                    int mj = transfer_result.min_dv_index % gn;
                    float mx = plot_lx + (mi + 0.5f) * cell_w;
                    float my = plot_by + (mj + 0.5f) * cell_h;
                    renderer->addRectOutline(mx, my, cell_w * 1.3f, cell_h * 1.3f, 1.0f, 1.0f, 1.0f, 1.0f, 0.003f);
                    renderer->addLine(mx - cell_w, my, mx + cell_w, my, 0.002f, 1.0f, 1.0f, 1.0f, 0.8f);
                    renderer->addLine(mx, my - cell_h, mx, my + cell_h, 0.002f, 1.0f, 1.0f, 1.0f, 0.8f);
                }

                // Axis tick labels (departure days)
                for (int ti = 0; ti <= 4; ti++) {
                    float fx = (float)ti / 4.0f;
                    double dep_day = (transfer_result.dep_start + fx * (transfer_result.dep_end - transfer_result.dep_start) - tele.sim_time) / 86400.0;
                    char tick[32]; snprintf(tick, sizeof(tick), "%.0f", dep_day);
                    renderer->drawText(plot_lx + fx * plot_w, plot_by - 0.015f, tick, 0.007f, 0.5f, 0.5f, 0.5f, 0.8f, true, Renderer::CENTER);
                }
                // Axis tick labels (TOF days)
                for (int ti = 0; ti <= 4; ti++) {
                    float fy = (float)ti / 4.0f;
                    double tof_day = (transfer_result.tof_min + fy * (transfer_result.tof_max - transfer_result.tof_min)) / 86400.0;
                    char tick[32]; snprintf(tick, sizeof(tick), "%.0fd", tof_day);
                    renderer->drawText(plot_lx - 0.03f, plot_by + fy * plot_h, tick, 0.007f, 0.5f, 0.5f, 0.5f, 0.8f, true, Renderer::CENTER);
                }

                // --- Info Display ---
                float info_y = plot_by - 0.035f;
                float info_lx = tw_x - tw_w * 0.4f;

                // Show minimum Δv info
                if (transfer_result.min_dv_index >= 0) {
                    const PorkchopPoint& best = transfer_result.grid[transfer_result.min_dv_index];
                    char buf[128];
                    snprintf(buf, sizeof(buf), "MIN Dv: %.2f km/s  (Dep: %.1f  Arr: %.1f)", 
                             best.dv_total / 1000.0, best.dv_departure / 1000.0, best.dv_arrival / 1000.0);
                    renderer->drawText(info_lx, info_y, buf, 0.009f, 0.2f, 1.0f, 0.4f, 1.0f, true, Renderer::LEFT);

                    double dep_days = (best.departure_time - tele.sim_time) / 86400.0;
                    double tof_days = best.tof / 86400.0;
                    snprintf(buf, sizeof(buf), "Depart: T+%.1f days | Travel: %.1f days", dep_days, tof_days);
                    renderer->drawText(info_lx, info_y - 0.018f, buf, 0.008f, 0.7f, 0.7f, 0.7f, 0.9f, true, Renderer::LEFT);
                }

                // Hover tooltip
                if (transfer_hover_dep >= 0 && transfer_hover_tof >= 0) {
                    int hidx = transfer_hover_dep * gn + transfer_hover_tof;
                    if (hidx >= 0 && hidx < (int)transfer_result.grid.size()) {
                        const PorkchopPoint& hpt = transfer_result.grid[hidx];
                        if (hpt.valid) {
                            float tt_x = hmouse_x + 0.05f;
                            float tt_y = hmouse_y + 0.03f;
                            float tt_w = 0.22f;
                            float tt_h = 0.06f;
                            renderer->addRect(tt_x, tt_y, tt_w, tt_h, 0.05f, 0.05f, 0.1f, 0.95f);
                            renderer->addRectOutline(tt_x, tt_y, tt_w, tt_h, 0.4f, 0.7f, 1.0f, 0.9f);
                            char tb[64];
                            snprintf(tb, sizeof(tb), "Dv: %.2f km/s", hpt.dv_total / 1000.0);
                            renderer->drawText(tt_x, tt_y + 0.012f, tb, 0.009f, 1,1,1, 1.0f, true, Renderer::CENTER);
                            double d_day = (hpt.departure_time - tele.sim_time) / 86400.0;
                            double t_day = hpt.tof / 86400.0;
                            snprintf(tb, sizeof(tb), "T+%.0fd / %.0fd flight", d_day, t_day);
                            renderer->drawText(tt_x, tt_y - 0.012f, tb, 0.008f, 0.7f, 0.7f, 0.7f, 1.0f, true, Renderer::CENTER);
                        }
                    }
                }

                // --- CREATE MANEUVER NODE Button ---
                float cmn_y = tw_y - tw_h/2 + 0.025f;
                float cmn_w = tw_w * 0.55f;
                float cmn_h = 0.035f;
                bool hover_cmn = (transfer_result.min_dv_index >= 0) &&
                    (hmouse_x >= tw_x - cmn_w/2 && hmouse_x <= tw_x + cmn_w/2 &&
                     hmouse_y >= cmn_y - cmn_h/2 && hmouse_y <= cmn_y + cmn_h/2);
                float cmn_r = transfer_result.min_dv_index >= 0 ? 0.2f : 0.15f;
                float cmn_g = transfer_result.min_dv_index >= 0 ? 0.8f : 0.3f;
                float cmn_b = transfer_result.min_dv_index >= 0 ? 0.4f : 0.3f;
                if (hover_cmn) { cmn_r += 0.1f; cmn_g += 0.1f; cmn_b += 0.1f; }
                renderer->addRect(tw_x, cmn_y, cmn_w, cmn_h, cmn_r * 0.3f, cmn_g * 0.3f, cmn_b * 0.3f, 0.8f);
                renderer->addRectOutline(tw_x, cmn_y, cmn_w, cmn_h, cmn_r, cmn_g, cmn_b, 0.9f);
                renderer->drawText(tw_x, cmn_y, "CREATE MANEUVER NODE", 0.010f, cmn_r, cmn_g, cmn_b, 1.0f, true, Renderer::CENTER);

                if (hover_cmn && hlmb && !hlmb_prev && transfer_result.min_dv_index >= 0) {
                    const PorkchopPoint& best = transfer_result.grid[transfer_result.min_dv_index];

                    // Create maneuver node at optimal departure time
                    ManeuverNode node;
                    node.sim_time = best.departure_time;
                    node.active = true;
                    node.ref_body = current_soi_index;

                    // Convert heliocentric Δv to prograde/normal/radial frame
                    // Get rocket state projected to departure time
                    double mu_soi = G_const * SOLAR_SYSTEM[current_soi_index].mass;
                    double npx, npy, npz, nvx, nvy, nvz;
                    get3DStateAtTime(trans.px, trans.py, trans.pz,
                                    vel.vx, vel.vy, vel.vz,
                                    mu_soi, best.departure_time - tele.sim_time,
                                    npx, npy, npz, nvx, nvy, nvz);

                    ManeuverFrame frame = ManeuverSystem::getFrame(
                        Vec3((float)npx, (float)npy, (float)npz),
                        Vec3((float)nvx, (float)nvy, (float)nvz));

                    // The departure Δv vector is in heliocentric frame.
                    // For the maneuver node, we need it in the local orbital frame relative to SOI body.
                    // Approximate: project the heliocentric dv into prograde/normal/radial
                    Vec3 dv_world = best.departure_dv_vec;
                    float pro_comp = dv_world.dot(frame.prograde);
                    float nrm_comp = dv_world.dot(frame.normal);
                    float rad_comp = dv_world.dot(frame.radial);
                    node.delta_v = Vec3(pro_comp, nrm_comp, rad_comp);

                    {
                        auto& mnv_c = ctx.registry->get<ManeuverComponent>(ctx.entity);
                        mnv_c.maneuvers.clear();
                        mnv_c.maneuvers.push_back(node);
                        mnv_c.selected_maneuver_index = 0;
                    }
                    mnv.maneuvers.clear();
                    mnv.maneuvers.push_back(node);
                    mnv.selected_maneuver_index = 0;
                    global_best_ang = 0;
                    mnv_popup_index = 0;
                    mnv_popup_visible = true;
                    adv_embed_mnv = true;

                    guid.mission_msg = "TRANSFER NODE CREATED";
                    cout << ">> Transfer maneuver created: dv=" << best.dv_total/1000.0 << " km/s" << endl;
                }
            } else {
                // No result yet - show instructions
                renderer->drawText(tw_x, tw_y - 0.03f, "Select target planet and press CALCULATE", 0.009f, 0.5f, 0.5f, 0.5f, 0.8f, true, Renderer::CENTER);
                renderer->drawText(tw_x, tw_y - 0.06f, "to generate porkchop plot.", 0.009f, 0.5f, 0.5f, 0.5f, 0.8f, true, Renderer::CENTER);

                // Origin info
                int origin = TransferCalculator::getTransferOriginBody();
                char orig_buf[64];
                snprintf(orig_buf, sizeof(orig_buf), "Origin: %s", SOLAR_SYSTEM[origin].name.c_str());
                renderer->drawText(tw_x, tw_y - 0.12f, orig_buf, 0.010f, 0.4f, 0.7f, 0.9f, 0.8f, true, Renderer::CENTER);
            }
        }
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
            renderer->drawText(menu_x - 0.12f, bd_y, "Primary:", 0.012f, 1, 1, 1, 1.0f);
            bool hover_bd_l = (hmouse_x >= menu_x - 0.01f && hmouse_x <= menu_x + 0.03f && hmouse_y >= bd_y - 0.02f && hmouse_y <= bd_y + 0.02f);
            bool hover_bd_r = (hmouse_x >= menu_x + 0.11f && hmouse_x <= menu_x + 0.15f && hmouse_y >= bd_y - 0.02f && hmouse_y <= bd_y + 0.02f);
            if (hover_bd_l && hlmb && !hlmb_prev) adv_orbit_ref_body = (adv_orbit_ref_body-1 < 0) ? (int)SOLAR_SYSTEM.size()-1 : adv_orbit_ref_body-1;
            if (hover_bd_r && hlmb && !hlmb_prev) adv_orbit_ref_body = (adv_orbit_ref_body+1) % (int)SOLAR_SYSTEM.size();
            renderer->drawText(menu_x + 0.01f, bd_y, "<", 0.012f, 1,1,1, hover_bd_l?1.0f:0.5f, true, Renderer::CENTER);
            if (!SOLAR_SYSTEM.empty()) {
                renderer->drawText(menu_x + 0.07f, bd_y, SOLAR_SYSTEM[adv_orbit_ref_body].name.c_str(), 0.010f, 1,1,1,1, true, Renderer::CENTER);
            }
            renderer->drawText(menu_x + 0.13f, bd_y, ">", 0.012f, 1,1,1, hover_bd_r?1.0f:0.5f, true, Renderer::CENTER);
            
            float next_y = bd_y - 0.06f;
            
            if (adv_orbit_ref_mode == 1) { // Co-rotating needs Secondary Body
                renderer->drawText(menu_x - 0.12f, next_y, "Second:", 0.012f, 1, 1, 1, 1.0f);
                bool hover_sbd_l = (hmouse_x >= menu_x - 0.01f && hmouse_x <= menu_x + 0.03f && hmouse_y >= next_y - 0.02f && hmouse_y <= next_y + 0.02f);
                bool hover_sbd_r = (hmouse_x >= menu_x + 0.11f && hmouse_x <= menu_x + 0.15f && hmouse_y >= next_y - 0.02f && hmouse_y <= next_y + 0.02f);
                if (hover_sbd_l && hlmb && !hlmb_prev) adv_orbit_secondary_ref_body = (adv_orbit_secondary_ref_body-1 < 0) ? (int)SOLAR_SYSTEM.size()-1 : adv_orbit_secondary_ref_body-1;
                if (hover_sbd_r && hlmb && !hlmb_prev) adv_orbit_secondary_ref_body = (adv_orbit_secondary_ref_body+1) % (int)SOLAR_SYSTEM.size();
                renderer->drawText(menu_x + 0.01f, next_y, "<", 0.012f, 1,1,1, hover_sbd_l?1.0f:0.5f, true, Renderer::CENTER);
                if (!SOLAR_SYSTEM.empty()) {
                    renderer->drawText(menu_x + 0.07f, next_y, SOLAR_SYSTEM[adv_orbit_secondary_ref_body].name.c_str(), 0.010f, 1,1,1,1, true, Renderer::CENTER);
                }
                renderer->drawText(menu_x + 0.13f, next_y, ">", 0.012f, 1,1,1, hover_sbd_r?1.0f:0.5f, true, Renderer::CENTER);
                next_y -= 0.06f;
            }

            // --- Prediction Time Slider ---
            float pred_y = next_y;
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
                node.sim_time = tele.sim_time + 600.0; // 10 minutes ahead
                node.delta_v = Vec3(0, 0, 0);
                node.active = true;
                node.ref_body = current_soi_index;
                {
                    auto& mnv_c = ctx.registry->get<ManeuverComponent>(ctx.entity);
                    mnv_c.maneuvers.clear();
                    mnv_c.maneuvers.push_back(node);
                    mnv_c.selected_maneuver_index = 0;
                }
                mnv.maneuvers.clear();
                mnv.maneuvers.push_back(node);
                mnv.selected_maneuver_index = 0;
                global_best_ang = 0; // Disable keplerian hit-testing state
                mnv_popup_index = 0;
                mnv_popup_visible = true;
                adv_embed_mnv = true;
            }
            
            // Warp to Maneuver Node Button
            float warp_btn_y = cr_btn_y - 0.05f;
            bool has_mnv_btn = !mnv.maneuvers.empty();
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
                double target_t = mnv.maneuvers[0].sim_time;
                double time_to_start = target_t - tele.sim_time;
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
    
    // === Advanced Orbit Apsides Rendering (2D HUD) ===
    if (adv_orbit_enabled) {
        float as_ratio = (float)ww / wh;
        double mx_raw, my_raw; glfwGetCursorPos(window, &mx_raw, &my_raw);
        float mouse_x = (float)(mx_raw / ww * 2.0 - 1.0);
        float mouse_y = (float)(1.0 - my_raw / wh * 2.0);

        std::vector<OrbitComponent::Apsis> hud_apsides, hud_mnv_apsides;
        {
            std::lock_guard<std::mutex> lock(*orb.path_mutex);
            hud_apsides = orb.predicted_apsides;
            hud_mnv_apsides = orb.predicted_mnv_apsides;
        }

        auto draw_apsis_hud = [&](const std::vector<OrbitComponent::Apsis>& list, bool is_mnv) {
            double rb_px, rb_py, rb_pz;
            PhysicsSystem::GetCelestialPositionAt(adv_orbit_ref_body, tele.sim_time, rb_px, rb_py, rb_pz);
            Quat q_inv = PhysicsSystem::GetFrameRotation(adv_orbit_ref_mode, adv_orbit_ref_body, adv_orbit_secondary_ref_body, tele.sim_time);
            
            for (const auto& ap : list) {
                Vec3 p_rot = q_inv.rotate(ap.local_pos);
                Vec3 w_pos(
                    (float)((rb_px + p_rot.x) * ws_d - ro_x),
                    (float)((rb_py + p_rot.y) * ws_d - ro_y),
                    (float)((rb_pz + p_rot.z) * ws_d - ro_z)
                );
                
                Vec2 scr = ManeuverSystem::projectToScreen(w_pos, viewMat, macroProjMat, as_ratio);
                if (scr.x < -1.5f || scr.x > 1.5f || scr.y < -1.5f || scr.y > 1.5f) continue;

                // Draw Triangle
                float tri_w = 0.02f;
                float tri_h = 0.025f;
                float r = 1.0f, g = 0.8f, b = 0.1f; // Yellow
                if (is_mnv) { r = 1.0f; g = 0.6f; b = 0.1f; } // Orange for mnv
                
                if (ap.is_apoapsis) {
                    renderer->addRotatedTri(scr.x, scr.y + tri_h/2.0f, tri_w, tri_h, 0.0f, r, g, b, 0.9f);
                    renderer->drawText(scr.x, scr.y - 0.015f, "Ap", 0.012f, r, g, b, 0.9f, true, Renderer::CENTER);
                } else {
                    renderer->addRotatedTri(scr.x, scr.y - tri_h/2.0f, tri_w, tri_h, 3.14159265f, r, g, b, 0.9f);
                    renderer->drawText(scr.x, scr.y + 0.015f, "Pe", 0.012f, r, g, b, 0.9f, true, Renderer::CENTER);
                }

                // Hover Tooltip
                float d_mouse = sqrtf(powf(scr.x - mouse_x, 2) + powf(scr.y - mouse_y, 2));
                if (d_mouse < 0.035f) {
                    char tooltip[128];
                    char tooltip2[128];
                    double dt_val = ap.sim_time - tele.sim_time;
                    int t_hr = (int)(std::fabs(dt_val) / 3600.0);
                    int t_min = (int)fmod(std::fabs(dt_val) / 60.0, 60.0);
                    int t_sec = (int)fmod(std::fabs(dt_val), 60.0);
                    
                    double alt_val = ap.altitude;
                    const char* unit = "m";
                    if (alt_val > 1000000.0) { alt_val /= 1000000.0; unit = "Mm"; }
                    else if (alt_val > 10000.0) { alt_val /= 1000.0; unit = "km"; }

                    snprintf(tooltip, sizeof(tooltip), "T%c %02dH %02dM %02dS", dt_val >= 0 ? '+' : '-', t_hr, t_min, t_sec);
                    snprintf(tooltip2, sizeof(tooltip2), "Alt: %.1f %s", alt_val, unit);
                        
                    float tt_x = scr.x;
                    float tt_y = scr.y + (ap.is_apoapsis ? -0.055f : 0.055f);
                    
                    float tw = 0.32f;
                    float th = 0.07f;
                    // Keep tooltip within screen
                    if (tt_x + tw/2 > 0.98f) tt_x = 0.98f - tw/2;
                    if (tt_x - tw/2 < -0.98f) tt_x = -0.98f + tw/2;
                    if (tt_y + th/2 > 0.98f) tt_y = 0.98f - th/2;
                    if (tt_y - th/2 < -0.98f) tt_y = -0.98f + th/2;

                    renderer->addRectOutline(tt_x, tt_y, tw, th, r, g, b, 0.9f);
                    renderer->addRect(tt_x, tt_y, tw, th, 0.05f, 0.05f, 0.08f, 0.95f);
                    renderer->drawText(tt_x, tt_y + 0.012f, tooltip, 0.012f, 1.0f, 1.0f, 1.0f, 1.0f, true, Renderer::CENTER);
                    renderer->drawText(tt_x, tt_y - 0.015f, tooltip2, 0.011f, 0.8f, 0.8f, 0.8f, 1.0f, true, Renderer::CENTER);
                }
            }
        };

        draw_apsis_hud(hud_apsides, false);
        draw_apsis_hud(hud_mnv_apsides, true);
    }

    hlmb_prev = hlmb;

    // ========================================================================
    } 
    
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
                snprintf(buf, sizeof(buf), "T-IGN: %dH %02dM %02dS", t_hr, t_min, t_sec);
            } else if (mnv_popup_time_to_node >= 0) {
                snprintf(buf, sizeof(buf), "T-IGN: %dM %02dS", t_min, t_sec);
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
    }
};
