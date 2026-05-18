#pragma once
inline void FlightHUD::RenderApsides(entt::registry& registry, entt::entity entity, Renderer* renderer, Renderer3D* r3d, CameraDirector& cam, const RocketAssembly& assembly, const RenderContext& ctx, double dt, int frame, double ws_d, float global_best_ang, int time_warp, float mouse_x, float mouse_y, bool& hlmb_prev_ref) 
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


}
