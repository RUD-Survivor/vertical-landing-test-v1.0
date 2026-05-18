#pragma once
inline void FlightHUD::RenderManeuver(entt::registry& registry, entt::entity entity, Renderer* renderer, Renderer3D* r3d, CameraDirector& cam, const RocketAssembly& assembly, const RenderContext& ctx, double dt, int frame, double ws_d, float global_best_ang, int time_warp, float mouse_x, float mouse_y, bool& hlmb_prev_ref) 
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
            if (mnv_popup_ref_body >= 0 && mnv_popup_ref_body < (int)UniverseModel::getInstance().solar_system.size()) {
                snprintf(ref_buf, sizeof(ref_buf), "REF: %s", UniverseModel::getInstance().solar_system[mnv_popup_ref_body].name.c_str());
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
