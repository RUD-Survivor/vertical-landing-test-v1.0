#pragma once
inline void FlightHUD::RenderAdvancedMenus(entt::registry& registry, entt::entity entity, Renderer* renderer, Renderer3D* r3d, CameraDirector& cam, const RocketAssembly& assembly, const RenderContext& ctx, double dt, int frame, double ws_d, float global_best_ang, int time_warp, float mouse_x, float mouse_y, bool& hlmb_prev_ref) 
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
    // --- 9. Advanced Orbit UI (Right Edge) ---
    if (cam.mode == 2 && !UniverseModel::getInstance().solar_system.empty()) {
        float adv_btn_w = 0.15f;
        float adv_btn_h = 0.05f;
        float adv_btn_x = 0.88f;
        float adv_btn_y = mode_y - 0.35f; // below stage UI
        
        bool hover_adv = (hmouse_x >= adv_btn_x - adv_btn_w/2 && hmouse_x <= adv_btn_x + adv_btn_w/2 && hmouse_y >= adv_btn_y - adv_btn_h/2 && hmouse_y <= adv_btn_y + adv_btn_h/2);
        if (hover_adv && hlmb && !hlmb_prev_ref) adv_orbit_menu = !adv_orbit_menu;
        
        renderer->addRect(adv_btn_x, adv_btn_y, adv_btn_w, adv_btn_h, 0.2f, 0.4f, 0.8f, hover_adv ? 0.9f : 0.7f);
        renderer->drawText(adv_btn_x, adv_btn_y, "ADV ORBIT", 0.012f, 1, 1, 1, 1.0f, true, Renderer::CENTER);

        // --- Flight Assist Button (Below ADV ORBIT) ---
        float fa_btn_y = adv_btn_y - 0.06f;
        bool hover_fa = (hmouse_x >= adv_btn_x - adv_btn_w/2 && hmouse_x <= adv_btn_x + adv_btn_w/2 && hmouse_y >= fa_btn_y - adv_btn_h/2 && hmouse_y <= fa_btn_y + adv_btn_h/2);
        if (hover_fa && hlmb && !hlmb_prev_ref) flight_assist_menu = !flight_assist_menu;
        
        renderer->addRect(adv_btn_x, fa_btn_y, adv_btn_w, adv_btn_h, 0.8f, 0.4f, 0.2f, hover_fa ? 0.9f : 0.7f);
        renderer->drawText(adv_btn_x, fa_btn_y, "FLIGHT ASSIST", 0.012f, 1, 1, 1, 1.0f, true, Renderer::CENTER);

        // --- Galaxy Info Button ---
        float galaxy_btn_y = fa_btn_y - 0.06f;
        bool hover_galaxy = (hmouse_x >= adv_btn_x - adv_btn_w/2 && hmouse_x <= adv_btn_x + adv_btn_w/2 && hmouse_y >= galaxy_btn_y - adv_btn_h/2 && hmouse_y <= galaxy_btn_y + adv_btn_h/2);
        if (hover_galaxy && hlmb && !hlmb_prev_ref) show_galaxy_info = !show_galaxy_info;
        
        renderer->addRect(adv_btn_x, galaxy_btn_y, adv_btn_w, adv_btn_h, 0.1f, 0.6f, 0.3f, hover_galaxy ? 0.9f : 0.7f);
        renderer->drawText(adv_btn_x, galaxy_btn_y, "GALAXY INFO", 0.012f, 1, 1, 1, 1.0f, true, Renderer::CENTER);

        // --- Climate View Button ---
        float climate_btn_y = galaxy_btn_y - 0.06f;
        bool hover_climate = (hmouse_x >= adv_btn_x - adv_btn_w/2 && hmouse_x <= adv_btn_x + adv_btn_w/2 && hmouse_y >= climate_btn_y - adv_btn_h/2 && hmouse_y <= climate_btn_y + adv_btn_h/2);
        if (hover_climate && hlmb && !hlmb_prev_ref) {
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
            for (int i = 0; i < (int)UniverseModel::getInstance().solar_system.size(); i++) {
                if (UniverseModel::getInstance().solar_system[i].parent_index != -1) continue;
                
                float ix = icon_start_x + main_count * icon_spacing;
                bool hover_icon = (hmouse_x >= ix - icon_r && hmouse_x <= ix + icon_r && hmouse_y >= bar_y - icon_r && hmouse_y <= bar_y + icon_r);
                
                if (hover_icon && hlmb && !hlmb_prev_ref) {
                    selected_body_idx = i;
                    // Logic: Planet click expands/collapses moons
                    bool has_moons = false;
                    for(int m=0; m<(int)UniverseModel::getInstance().solar_system.size(); m++) if(UniverseModel::getInstance().solar_system[m].parent_index == i) has_moons = true;
                    
                    if (has_moons) {
                        if (expanded_planet_idx == i) expanded_planet_idx = -1;
                        else expanded_planet_idx = i;
                    }
                }
                
                renderer->drawPlanetIcon(ix, bar_y, icon_r, UniverseModel::getInstance().solar_system[i], (float)glfwGetTime());
                if (hover_icon || selected_body_idx == i) 
                    renderer->addCircleOutline(ix, bar_y, icon_r * 1.1f, 0.003f, 0.4f, 0.8f, 1.0f, 1.0f);
                
                renderer->drawText(ix, bar_y - icon_r - 0.015f, UniverseModel::getInstance().solar_system[i].name.c_str(), 0.008f, 1, 1, 1, 1.0f, true, Renderer::CENTER);
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
                for (int i = 0; i < (int)UniverseModel::getInstance().solar_system.size(); i++) {
                    if (UniverseModel::getInstance().solar_system[i].parent_index == expanded_planet_idx) {
                        float mix = icon_start_x + moon_count * icon_spacing;
                        float mr = 0.03f;
                        bool hover_moon = (hmouse_x >= mix - mr && hmouse_x <= mix + mr && hmouse_y >= moon_bar_y - mr && hmouse_y <= moon_bar_y + mr);
                        
                        if (hover_moon && hlmb && !hlmb_prev_ref) {
                            selected_body_idx = i;
                        }
                        
                        renderer->drawPlanetIcon(mix, moon_bar_y, mr, UniverseModel::getInstance().solar_system[i], (float)glfwGetTime());
                        if (hover_moon || selected_body_idx == i)
                             renderer->addCircleOutline(mix, moon_bar_y, mr * 1.1f, 0.002f, 0.4f, 1.0f, 0.6f, 1.0f);
                        
                        renderer->drawText(mix, moon_bar_y - mr - 0.012f, UniverseModel::getInstance().solar_system[i].name.c_str(), 0.007f, 0.8f, 1.0f, 0.8f, 1.0f, true, Renderer::CENTER);
                        moon_count++;
                    }
                }
            }
        }

        // --- Detailed Info Panel (Top Left) ---
        if (selected_body_idx != -1 && selected_body_idx < (int)UniverseModel::getInstance().solar_system.size()) {
            const CelestialBody& b = UniverseModel::getInstance().solar_system[selected_body_idx];
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
            for(int m=0; m<(int)UniverseModel::getInstance().solar_system.size(); m++) if(UniverseModel::getInstance().solar_system[m].parent_index == selected_body_idx) moon_count++;
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
                if (hover && hlmb && !hlmb_prev_ref) {
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
                if (hover && hlmb && !hlmb_prev_ref) {
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
            if (hover_tgt_l && hlmb && !hlmb_prev_ref) {
                int origin = TransferCalculator::getTransferOriginBody();
                do { transfer_target_body--; if (transfer_target_body < 1) transfer_target_body = (int)UniverseModel::getInstance().solar_system.size()-1; }
                while (transfer_target_body == origin || transfer_target_body == 4 || transfer_target_body == 0);
                transfer_result_valid = false;
            }

            // Body name
            if (transfer_target_body >= 0 && transfer_target_body < (int)UniverseModel::getInstance().solar_system.size())
                renderer->drawText(tw_x + 0.06f, sel_y, UniverseModel::getInstance().solar_system[transfer_target_body].name.c_str(), 0.012f, 1.0f, 0.9f, 0.3f, 1.0f, true, Renderer::CENTER);

            // Right arrow
            float arr_rx = tw_x + 0.14f;
            bool hover_tgt_r = (hmouse_x >= arr_rx - 0.015f && hmouse_x <= arr_rx + 0.015f && hmouse_y >= sel_y - 0.015f && hmouse_y <= sel_y + 0.015f);
            renderer->drawText(arr_rx, sel_y, ">", 0.014f, 1,1,1, hover_tgt_r ? 1.0f : 0.5f, true, Renderer::CENTER);
            if (hover_tgt_r && hlmb && !hlmb_prev_ref) {
                int origin = TransferCalculator::getTransferOriginBody();
                do { transfer_target_body++; if (transfer_target_body >= (int)UniverseModel::getInstance().solar_system.size()) transfer_target_body = 1; }
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

            if (hover_calc && hlmb && !hlmb_prev_ref) {
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

                if (hover_cmn && hlmb && !hlmb_prev_ref && transfer_result.min_dv_index >= 0) {
                    const PorkchopPoint& best = transfer_result.grid[transfer_result.min_dv_index];

                    // Create maneuver node at optimal departure time
                    ManeuverNode node;
                    node.sim_time = best.departure_time;
                    node.active = true;
                    node.ref_body = UniverseModel::getInstance().current_soi_index;

                    // Convert heliocentric Δv to prograde/normal/radial frame
                    // Get rocket state projected to departure time
                    double mu_soi = G_const * UniverseModel::getInstance().solar_system[UniverseModel::getInstance().current_soi_index].mass;
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
                        auto& mnv_c = registry.get<ManeuverComponent>(entity);
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
                    std::cout << ">> Transfer maneuver created: dv=" << best.dv_total/1000.0 << " km/s" << std::endl;
                }
            } else {
                // No result yet - show instructions
                renderer->drawText(tw_x, tw_y - 0.03f, "Select target planet and press CALCULATE", 0.009f, 0.5f, 0.5f, 0.5f, 0.8f, true, Renderer::CENTER);
                renderer->drawText(tw_x, tw_y - 0.06f, "to generate porkchop plot.", 0.009f, 0.5f, 0.5f, 0.5f, 0.8f, true, Renderer::CENTER);

                // Origin info
                int origin = TransferCalculator::getTransferOriginBody();
                char orig_buf[64];
                snprintf(orig_buf, sizeof(orig_buf), "Origin: %s", UniverseModel::getInstance().solar_system[origin].name.c_str());
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
            if (hover_tog && hlmb && !hlmb_prev_ref) adv_orbit_enabled = !adv_orbit_enabled;
            renderer->drawText(menu_x - 0.12f, tog_y, "Mode:", 0.012f, 1, 1, 1, 1.0f);
            renderer->drawText(menu_x + 0.02f, tog_y, adv_orbit_enabled ? "SYM-LMM4 (TRANS-DT)" : "KEPLER (SOI)", 0.012f, adv_orbit_enabled?1:0.6f, adv_orbit_enabled?0.5f:1, 0.4f, 1.0f);

            // Frame Type Switch (Inertial vs Co-rotating vs Surface)
            float fr_y = tog_y - 0.06f;
            renderer->drawText(menu_x - 0.12f, fr_y, "Frame:", 0.012f, 1, 1, 1, 1.0f);
            bool hover_fr_l = (hmouse_x >= menu_x - 0.01f && hmouse_x <= menu_x + 0.03f && hmouse_y >= fr_y - 0.02f && hmouse_y <= fr_y + 0.02f);
            bool hover_fr_r = (hmouse_x >= menu_x + 0.11f && hmouse_x <= menu_x + 0.15f && hmouse_y >= fr_y - 0.02f && hmouse_y <= fr_y + 0.02f);
            if (hover_fr_l && hlmb && !hlmb_prev_ref) adv_orbit_ref_mode = (adv_orbit_ref_mode+2)%3;
            if (hover_fr_r && hlmb && !hlmb_prev_ref) adv_orbit_ref_mode = (adv_orbit_ref_mode+1)%3;
            renderer->drawText(menu_x + 0.01f, fr_y, "<", 0.012f, 1,1,1, hover_fr_l?1.0f:0.5f, true, Renderer::CENTER);
            renderer->drawText(menu_x + 0.07f, fr_y, adv_orbit_ref_mode==0 ? "INERTIAL" : (adv_orbit_ref_mode==1 ? "CO-ROT" : "SURFACE"), 0.010f, 1,1,1,1, true, Renderer::CENTER);
            renderer->drawText(menu_x + 0.13f, fr_y, ">", 0.012f, 1,1,1, hover_fr_r?1.0f:0.5f, true, Renderer::CENTER);

            // Ref Body Switch
            float bd_y = fr_y - 0.06f;
            renderer->drawText(menu_x - 0.12f, bd_y, "Primary:", 0.012f, 1, 1, 1, 1.0f);
            bool hover_bd_l = (hmouse_x >= menu_x - 0.01f && hmouse_x <= menu_x + 0.03f && hmouse_y >= bd_y - 0.02f && hmouse_y <= bd_y + 0.02f);
            bool hover_bd_r = (hmouse_x >= menu_x + 0.11f && hmouse_x <= menu_x + 0.15f && hmouse_y >= bd_y - 0.02f && hmouse_y <= bd_y + 0.02f);
            if (hover_bd_l && hlmb && !hlmb_prev_ref) adv_orbit_ref_body = (adv_orbit_ref_body-1 < 0) ? (int)UniverseModel::getInstance().solar_system.size()-1 : adv_orbit_ref_body-1;
            if (hover_bd_r && hlmb && !hlmb_prev_ref) adv_orbit_ref_body = (adv_orbit_ref_body+1) % (int)UniverseModel::getInstance().solar_system.size();
            renderer->drawText(menu_x + 0.01f, bd_y, "<", 0.012f, 1,1,1, hover_bd_l?1.0f:0.5f, true, Renderer::CENTER);
            if (!UniverseModel::getInstance().solar_system.empty()) {
                renderer->drawText(menu_x + 0.07f, bd_y, UniverseModel::getInstance().solar_system[adv_orbit_ref_body].name.c_str(), 0.010f, 1,1,1,1, true, Renderer::CENTER);
            }
            renderer->drawText(menu_x + 0.13f, bd_y, ">", 0.012f, 1,1,1, hover_bd_r?1.0f:0.5f, true, Renderer::CENTER);
            
            float next_y = bd_y - 0.06f;
            
            if (adv_orbit_ref_mode == 1) { // Co-rotating needs Secondary Body
                renderer->drawText(menu_x - 0.12f, next_y, "Second:", 0.012f, 1, 1, 1, 1.0f);
                bool hover_sbd_l = (hmouse_x >= menu_x - 0.01f && hmouse_x <= menu_x + 0.03f && hmouse_y >= next_y - 0.02f && hmouse_y <= next_y + 0.02f);
                bool hover_sbd_r = (hmouse_x >= menu_x + 0.11f && hmouse_x <= menu_x + 0.15f && hmouse_y >= next_y - 0.02f && hmouse_y <= next_y + 0.02f);
                if (hover_sbd_l && hlmb && !hlmb_prev_ref) adv_orbit_secondary_ref_body = (adv_orbit_secondary_ref_body-1 < 0) ? (int)UniverseModel::getInstance().solar_system.size()-1 : adv_orbit_secondary_ref_body-1;
                if (hover_sbd_r && hlmb && !hlmb_prev_ref) adv_orbit_secondary_ref_body = (adv_orbit_secondary_ref_body+1) % (int)UniverseModel::getInstance().solar_system.size();
                renderer->drawText(menu_x + 0.01f, next_y, "<", 0.012f, 1,1,1, hover_sbd_l?1.0f:0.5f, true, Renderer::CENTER);
                if (!UniverseModel::getInstance().solar_system.empty()) {
                    renderer->drawText(menu_x + 0.07f, next_y, UniverseModel::getInstance().solar_system[adv_orbit_secondary_ref_body].name.c_str(), 0.010f, 1,1,1,1, true, Renderer::CENTER);
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
            if (hlmb && !hlmb_prev_ref && hmouse_x >= pred_slider_x - pred_slider_w/2 - 0.01f && hmouse_x <= pred_slider_x + pred_slider_w/2 + 0.01f && hmouse_y >= pred_y - 0.02f && hmouse_y <= pred_y + 0.02f) {
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
            if (hover_it_l && hlmb && !hlmb_prev_ref && cur_it_idx > 0) adv_orbit_iters = iter_opts[cur_it_idx-1];
            if (hover_it_r && hlmb && !hlmb_prev_ref && cur_it_idx < 6) adv_orbit_iters = iter_opts[cur_it_idx+1];
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
            
            if (hover_cr && hlmb && !hlmb_prev_ref) {
                ManeuverNode node;
                node.sim_time = tele.sim_time + 600.0; // 10 minutes ahead
                node.delta_v = Vec3(0, 0, 0);
                node.active = true;
                node.ref_body = UniverseModel::getInstance().current_soi_index;
                {
                    auto& mnv_c = registry.get<ManeuverComponent>(entity);
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
            
            if (hover_warp && hlmb && !hlmb_prev_ref && has_mnv_btn) {
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
                if (hover_fold && hlmb && !hlmb_prev_ref) {
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
    

}
