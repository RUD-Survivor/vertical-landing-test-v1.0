#pragma once
inline void FlightHUD::RenderTelemetry(entt::registry& registry, entt::entity entity, Renderer* renderer, Renderer3D* r3d, CameraDirector& cam, const RocketAssembly& assembly, const RenderContext& ctx, double dt, int frame, double ws_d, float global_best_ang, int time_warp, float mouse_x, float mouse_y, bool& hlmb_prev_ref) 
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
    double planet_r = UniverseModel::getInstance().solar_system[UniverseModel::getInstance().current_soi_index].radius;
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

}
