#pragma once
inline void FlightHUD::RenderIndicators(entt::registry& registry, entt::entity entity, Renderer* renderer, Renderer3D* r3d, CameraDirector& cam, const RocketAssembly& assembly, const RenderContext& ctx, double dt, int frame, double ws_d, float global_best_ang, int time_warp, float mouse_x, float mouse_y, bool& hlmb_prev_ref) 
{

    auto& trans = registry.get<TransformComponent>(entity);
    auto& vel   = registry.get<VelocityComponent>(entity);
    auto& att   = registry.get<AttitudeComponent>(entity);
    auto& prop  = registry.get<PropulsionComponent>(entity);
    auto& tele  = registry.get<TelemetryComponent>(entity);
    auto& guid  = registry.get<GuidanceComponent>(entity);
    auto& mnv   = registry.get<ManeuverComponent>(entity);
    auto& orb   = registry.get<OrbitComponent>(entity);
    auto& control_input = registry.get<ControlInput>(entity);
    auto& rocket_config = registry.get<RocketConfig>(entity);
    GLFWwindow* window = GameContext::getInstance().window;
    
    int ww, wh; glfwGetWindowSize(window, &ww, &wh);
    double hmx, hmy; glfwGetCursorPos(window, &hmx, &hmy);
    float hmouse_x = (float)(hmx / ww * 2.0 - 1.0);
    float hmouse_y = (float)(1.0 - hmy / wh * 2.0);
    bool hlmb = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS;

    const Quat& rocketQuat = ctx.rocketQuat;
    const Vec3& localRight = ctx.localRight;
    const Vec3& rocketUp = ctx.rocketUp;
    const Vec3& localNorth = ctx.localNorth;

    double ro_x = ctx.ro_x;
    double ro_y = ctx.ro_y;
    double ro_z = ctx.ro_z;

    const Mat4& viewMat = ctx.viewMat; 
    const Mat4& macroProjMat = ctx.macroProjMat;
    float aspect = ctx.aspect;

    float num_size = 0.025f;
    float num_x = 0.85f;
    float label_x = num_x + 0.065f; 
    float bg_w = 0.22f;
    float bg_h = 0.05f;
    double current_fuel = prop.fuel;
    
    float mode_x = 0.88f; 
    float mode_y = 0.85f;
    float mode_w = 0.15f;
    float mode_h = 0.04f;
    
    float hud_opacity = 0.8f;
    double current_vel = sqrt(vel.vx*vel.vx + vel.vy*vel.vy + vel.vz*vel.vz);
    double current_alt = tele.altitude;
    int current_vvel = (int)tele.velocity;
    
    if (this->orbit_reference_sun && UniverseModel::getInstance().current_soi_index == 0) {
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

    // Extracted code:
    
    bool is_hover_mode = (hmouse_x >= mode_x - mode_w/2.0f && hmouse_x <= mode_x + mode_w/2.0f &&
                          hmouse_y >= mode_y - mode_h/2.0f && hmouse_y <= mode_y + mode_h/2.0f);
    
    if (is_hover_mode && hlmb && !hlmb_prev_ref) {
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
    // hlmb_prev_ref updated after all HUD interactions

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
    
    if (is_hover_time && hlmb && !hlmb_prev_ref) {
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
        SaveSystem::SaveGame(assembly, registry, entity, control_input);
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


}
