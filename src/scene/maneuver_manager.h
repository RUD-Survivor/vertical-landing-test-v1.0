#pragma once
#include <GLFW/glfw3.h>
#include <vector>
#include <cmath>
#include <iostream>
#include "core/rocket_state.h"
#include "physics/physics_system.h"
#include "math/math3d.h"
#include "simulation/maneuver_system.h"
#include "render/renderer3d.h"

struct FlightHUD;
struct HUDContext;
extern std::vector<CelestialBody> SOLAR_SYSTEM;

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

struct ManeuverManager {
    // --- Hit-test / Hover State ---
    float global_best_ang = -1.0f;
    double global_best_mu = 0, global_best_a = 0, global_best_ecc = 0;
    double global_current_M0 = 0, global_current_n = 0;
    Vec3 global_best_pt, global_best_center, global_best_e_dir, global_best_perp_dir;
    
    // --- Dragging & Interaction ---
    int dragging_handle = -1; // -1: none, -2: time slider, 0-5: DV handles
    float last_drag_mx = 0;
    float last_pass_mx = 0, last_pass_my = 0;
    bool lmb_prev = false;
    bool popup_clicked_frame = false;
    bool hit_maneuver_icon = false;

    // --- Popup Visual Cache ---
    float cached_popup_x = 0, cached_popup_y = 0, cached_popup_w = 0, cached_popup_h = 0;

    void update(GLFWwindow* window, entt::registry& registry, entt::entity entity, FlightHUD& hud, const CameraDirector& cam, double dt, int ww, int wh);
    void render(entt::registry& registry, entt::entity entity, FlightHUD& hud, Renderer3D* r3d, const Mat4& view, const Mat4& proj, float aspect, float earth_r, float cam_dist, double ws_d, double ro_x, double ro_y, double ro_z, double dt);

private:
    void handleHandleDragging(ManeuverNode& node, float mouse_x, float mouse_y, const Vec2& n_scr, const Vec2& h_scr, const Vec2& axis2D, double dt);
    void updatePopupState(ManeuverNode& node, entt::registry& registry, entt::entity entity, FlightHUD& hud, const Mat4& view, const Mat4& proj, float aspect, double ws_d, double ro_x, double ro_y, double ro_z, float mouse_x, float mouse_y, bool lmb, double dt);
};



// Forward decls
struct FlightHUD;
struct HUDContext;





#include "render/HUD_system.h"

void ManeuverManager::update(GLFWwindow* window, entt::registry& registry, entt::entity entity, FlightHUD& hud, 
                const CameraDirector& cam, double dt, int ww, int wh) {
        auto& mnv = registry.get<ManeuverComponent>(entity);
        auto& tele = registry.get<TelemetryComponent>(entity);
        
        double mx_raw, my_raw;
        glfwGetCursorPos(window, &mx_raw, &my_raw);
        float mouse_x = (float)(mx_raw / ww * 2.0 - 1.0);
        float mouse_y = (float)(1.0 - my_raw / wh * 2.0);
        bool lmb = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS;

        popup_clicked_frame = false;
        // Early popup bounds check
        if (hud.mnv_popup_index != -1 && cached_popup_w > 0) {
            bool mouse_in_popup = (mouse_x >= cached_popup_x - cached_popup_w / 2 && mouse_x <= cached_popup_x + cached_popup_w / 2 &&
                                   mouse_y >= cached_popup_y - cached_popup_h / 2 && mouse_y <= cached_popup_y + cached_popup_h / 2);
            if (mouse_in_popup) popup_clicked_frame = true;
        }

        // --- Handle Node Creation Click ---
        // UI Block Check: Prevent interaction if mouse is on sidebar or top menus
        bool mouse_on_ui = (mouse_x > 0.7f) || (mouse_y > 0.7f) || (mouse_y < -0.85f);
        if (lmb && !lmb_prev && !popup_clicked_frame && !mouse_on_ui && dragging_handle == -1 && hud.mnv_popup_index == -1 && 
            mnv.selected_maneuver_index == -1 && global_best_ang >= 0) {
            
            ManeuverNode newNode;
            if (global_current_n > 0.0001) { // Normal Keplerian creation
                double M_click = global_best_ang - global_best_ecc * std::sin(global_best_ang);
                double dM = M_click - global_current_M0;
                while (dM < 0) dM += 2.0 * M_PI;
                while (dM > 2.0 * M_PI) dM -= 2.0 * M_PI;
                newNode.sim_time = tele.sim_time + (dM / global_current_n);
            } else { // Numerical / Advance Mode fallback: Node at current time
                newNode.sim_time = tele.sim_time + 300.0; // Place it 5 mins ahead by default in adv mode 
            }
            newNode.delta_v = Vec3(0, 0, 0);
            newNode.active = true;
            newNode.ref_a = global_best_a;
            newNode.ref_ecc = global_best_ecc;
            newNode.ref_n = global_current_n;
            newNode.ref_e_dir = global_best_e_dir;
            newNode.ref_p_dir = global_best_perp_dir;
            newNode.ref_center = global_best_center;
            newNode.ref_M0 = global_best_ang; // Eccentric anomaly
            newNode.ref_body = hud.global_best_ref_node;
            
            mnv.maneuvers.push_back(newNode);
            mnv.selected_maneuver_index = (int)mnv.maneuvers.size() - 1;
            hud.mnv_popup_index = mnv.selected_maneuver_index;
            hud.mnv_popup_visible = true;
            hud.mnv_popup_burn_mode = 0; // Relative mode
            return; // EXIT update to prevent same-frame deletion!
        }

        // Handle Time-Slider Dragging (Dragging the node itself along the orbit)
        if (lmb && dragging_handle == -2 && mnv.selected_maneuver_index != -1) {
            if (global_best_ang >= 0) {
                ManeuverNode& node = mnv.maneuvers[mnv.selected_maneuver_index];
                double M_click = global_best_ang - global_best_ecc * std::sin(global_best_ang);
                double dM = M_click - global_current_M0;
                while (dM < 0) dM += 2.0 * M_PI;
                while (dM > 2.0 * M_PI) dM -= 2.0 * M_PI;
                node.sim_time = tele.sim_time + (dM / global_current_n);
                // Also update the anchor ECC anomaly so it doesn't jump back when released
                node.ref_M0 = global_best_ang;
            }
        }

        // Deselection (Click empty space)
        if (lmb && !lmb_prev && !popup_clicked_frame && hud.mnv_popup_index == -1 && dragging_handle == -1 && global_best_ang < 0) {
            mnv.selected_maneuver_index = -1;
        }

        if (!lmb) dragging_handle = -1;
        
        // DO NOT update lmb_prev here! It must be updated after render() can see the click.
    }

    void ManeuverManager::render(entt::registry& registry, entt::entity entity, FlightHUD& hud, Renderer3D* r3d, 
                const Mat4& view, const Mat4& proj, float aspect, float earth_r, float cam_dist,
                double ws_d, double ro_x, double ro_y, double ro_z, double dt) {
        auto& mnv = registry.get<ManeuverComponent>(entity);
        auto& tele = registry.get<TelemetryComponent>(entity);
        
        GLFWwindow* window = GameContext::getInstance().window;
        int ww, wh; glfwGetWindowSize(window, &ww, &wh);
        double mx_raw, my_raw; glfwGetCursorPos(window, &mx_raw, &my_raw);
        float mouse_x = (float)(mx_raw / ww * 2.0 - 1.0);
        float mouse_y = (float)(1.0 - my_raw / wh * 2.0);
        bool lmb = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS;

        hit_maneuver_icon = false;

        for (int i = 0; i < (int)mnv.maneuvers.size(); i++) {
            ManeuverNode& node = mnv.maneuvers[i];
            
            // 1. Reconstruct 3D Position
            double ref_px = SOLAR_SYSTEM[node.ref_body].px;
            double ref_py = SOLAR_SYSTEM[node.ref_body].py;
            double ref_pz = SOLAR_SYSTEM[node.ref_body].pz;
            double E_node = node.ref_M0;
            float node_b = (float)node.ref_a * std::sqrt(std::max(0.0f, 1.0f - (float)node.ref_ecc * (float)node.ref_ecc));
            Vec3 pt_node_rel = node.ref_center + node.ref_e_dir * ((float)node.ref_a * std::cos((float)E_node)) + node.ref_p_dir * (node_b * std::sin((float)E_node));
            Vec3 node_world = Vec3((float)(ref_px * ws_d + pt_node_rel.x * ws_d - ro_x),
                                   (float)(ref_py * ws_d + pt_node_rel.y * ws_d - ro_y),
                                   (float)(ref_pz * ws_d + pt_node_rel.z * ws_d - ro_z));

            Vec2 n_scr = ManeuverSystem::projectToScreen(node_world, view, proj, aspect);
            float d_mouse = std::sqrt(std::pow(n_scr.x - mouse_x, 2) + std::pow(n_scr.y - mouse_y, 2));

            // 2. Interaction: Node Icon Click
            if (lmb && !lmb_prev && dragging_handle == -1 && !popup_clicked_frame) {
                if (d_mouse < 0.045f) {
                    mnv.selected_maneuver_index = i;
                    hud.mnv_popup_index = i;
                    dragging_handle = -2; // Start dragging node along orbit
                    hit_maneuver_icon = true;
                }
            }

            // 3. Draw Icon
            float icon_size = earth_r * (d_mouse < 0.04f || dragging_handle == -2 ? 0.1f : 0.08f);
            Vec3 icon_col = (mnv.selected_maneuver_index == i) ? Vec3(0.4f, 0.8f, 1.0f) : Vec3(0.2f, 0.6f, 1.0f);
            r3d->drawBillboard(node_world, icon_size, icon_col.x, icon_col.y, icon_col.z, 0.9f);

            // 4. Draw Handles & Process Dragging
            if (mnv.selected_maneuver_index == i) {
                double E_dot = node.ref_n / (1.0 - node.ref_ecc * std::cos(E_node));
                Vec3 v_node_rel = node.ref_e_dir * (-(float)node.ref_a * std::sin((float)E_node) * (float)E_dot) + node.ref_p_dir * (node_b * std::cos((float)E_node) * (float)E_dot);
                ManeuverFrame m_frame = ManeuverSystem::getFrame(pt_node_rel, v_node_rel);

                for (int h = 0; h < 6; h++) {
                    Vec3 h_dir = ManeuverSystem::getHandleDir(m_frame, h);
                    float handle_dist = earth_r * (dragging_handle == h ? 0.35f : 0.2f);
                    Vec3 h_world = node_world + h_dir * handle_dist;
                    Vec2 h_scr = ManeuverSystem::projectToScreen(h_world, view, proj, aspect);
                    float hd = std::sqrt(std::pow(h_scr.x - mouse_x, 2) + std::pow(h_scr.y - mouse_y, 2));

                    if (lmb && !lmb_prev && hd < 0.035f && !hit_maneuver_icon && !popup_clicked_frame) {
                        dragging_handle = h;
                        hud.mnv_popup_index = -1;
                        cached_popup_w = 0;
                    }

                    if (dragging_handle == h && lmb) {
                        Vec3 h_world_next = node_world + h_dir * (handle_dist + 1.0f);
                        Vec2 h_scr_next = ManeuverSystem::projectToScreen(h_world_next, view, proj, aspect);
                        handleHandleDragging(node, mouse_x, mouse_y, n_scr, h_scr, (h_scr_next - n_scr), dt);
                    }

                    Vec3 h_col = ManeuverSystem::getHandleColor(h);
                    if (hd < 0.035f || dragging_handle == h) h_col = h_col * 1.3f;
                    r3d->drawBillboard(h_world, earth_r * (hd < 0.035f ? 0.05f : 0.04f), h_col.x, h_col.y, h_col.z, 0.9f);
                }

                // 5. Draw Predicted Dash-Line Orbit
                if (node.active) {
                    Vec3 p_pos = pt_node_rel;
                    Vec3 p_vel = v_node_rel + m_frame.prograde * node.delta_v.x + m_frame.normal * node.delta_v.y + m_frame.radial * node.delta_v.z;
                    double mu_ref = 6.67430e-11 * SOLAR_SYSTEM[node.ref_body].mass;
                    double p_energy = 0.5 * (double)p_vel.lengthSq() - mu_ref / (double)p_pos.length();
                    double p_a = -mu_ref / (2.0 * p_energy);
                    if (p_a > 0) {
                        Vec3 p_h_vec = p_pos.cross(p_vel);
                        Vec3 p_e_vec = p_vel.cross(p_h_vec) / (float)mu_ref - p_pos / (float)p_pos.length();
                        float p_ecc = p_e_vec.length();
                        float p_b = (float)p_a * std::sqrt(std::max(0.0f, 1.0f - p_ecc * p_ecc));
                        Vec3 p_e_dir = p_ecc > 1e-6f ? p_e_vec / p_ecc : Vec3(1, 0, 0);
                        Vec3 p_perp = p_h_vec.normalized().cross(p_e_dir);
                        Vec3 p_center = p_e_dir * (-(float)p_a * p_ecc);
                        
                        std::vector<Vec3> p_pts;
                        int samples = 500;
                        for (int s = 0; s <= samples; s++) {
                            float ang = (float)s / (float)samples * 6.2831853f;
                            Vec3 ptr = p_center + p_e_dir * ((float)p_a * std::cos(ang)) + p_perp * (p_b * std::sin(ang));
                            p_pts.push_back(Vec3((float)(ref_px * ws_d + ptr.x * ws_d - ro_x), (float)(ref_py * ws_d + ptr.y * ws_d - ro_y), (float)(ref_pz * ws_d + ptr.z * ws_d - ro_z)));
                        }
                        
                        float ribbon_w = std::max(earth_r * 0.008f, cam_dist * 0.0012f);
                        for (int s = 0; s < samples; s += 10) {
                            std::vector<Vec3> dash;
                            for (int j = 0; j < 6; j++) { if (s + j <= samples) dash.push_back(p_pts[s + j]); }
                            if (dash.size() >= 2) r3d->drawRibbon(dash, ribbon_w, 1.0f, 1.0f, 1.0f, 0.7f);
                        }
                    }
                }
            }
        }

        // 6. Manage Popup Positioning & Common Logic
        if (hud.mnv_popup_index != -1 && (size_t)hud.mnv_popup_index < mnv.maneuvers.size()) {
            ManeuverNode& node = mnv.maneuvers[hud.mnv_popup_index];
            updatePopupState(node, registry, entity, hud, view, proj, aspect, ws_d, ro_x, ro_y, ro_z, mouse_x, mouse_y, lmb, dt);
        } else {
            hud.mnv_popup_visible = false;
        }

        // UPDATE state for next frame
        lmb_prev = lmb;
        last_pass_mx = mouse_x;
        last_pass_my = mouse_y;
    }

void ManeuverManager::handleHandleDragging(ManeuverNode& node, float mouse_x, float mouse_y, 
                              const Vec2& n_scr, const Vec2& h_scr, const Vec2& axis2D, double dt) {
        float screen_len_sq = axis2D.lengthSq();
        if (screen_len_sq < 1e-8f) return;

        float dmx = mouse_x - last_pass_mx;
        float dmy = mouse_y - last_pass_my;

        float proj_drag = (dmx * axis2D.x + dmy * axis2D.y) / screen_len_sq;
        float sensitivity = 100.0f * std::sqrt(screen_len_sq);
        if (dragging_handle >= 2) sensitivity *= 2.5f;
        float drag_amount = proj_drag * sensitivity;

        float dx_h = mouse_x - h_scr.x;
        float dy_h = mouse_y - h_scr.y;
        float proj_offset = (dx_h * axis2D.x + dy_h * axis2D.y) / screen_len_sq;
        float rate = 30.0f; 
        if (dragging_handle >= 2) rate *= 4.0f;
        float continuous_amount = proj_offset * rate * (float)dt;

        float total_change = drag_amount + continuous_amount;
        if      (dragging_handle == 0) node.delta_v.x += total_change; 
        else if (dragging_handle == 1) node.delta_v.x -= total_change;
        else if (dragging_handle == 2) node.delta_v.y += total_change; 
        else if (dragging_handle == 3) node.delta_v.y -= total_change;
        else if (dragging_handle == 4) node.delta_v.z += total_change; 
        else if (dragging_handle == 5) node.delta_v.z -= total_change;
        node.active = true;
    }

    void ManeuverManager::updatePopupState(ManeuverNode& node, entt::registry& registry, entt::entity entity, FlightHUD& hud, 
                         const Mat4& view, const Mat4& proj, float aspect, 
                         double ws_d, double ro_x, double ro_y, double ro_z, 
                         float mouse_x, float mouse_y, bool lmb, double dt) {
        auto& tele = registry.get<TelemetryComponent>(entity);
        auto& mnv = registry.get<ManeuverComponent>(entity);
        
        float node_b = (float)node.ref_a * std::sqrt(std::max(0.0f, 1.0f - (float)node.ref_ecc * (float)node.ref_ecc));
        Vec3 pt_node_rel = node.ref_center + node.ref_e_dir * ((float)node.ref_a * std::cos((float)node.ref_M0)) + node.ref_p_dir * (node_b * std::sin((float)node.ref_M0));
        Vec3 node_world = Vec3((float)(SOLAR_SYSTEM[node.ref_body].px * ws_d + pt_node_rel.x * ws_d - ro_x),
                               (float)(SOLAR_SYSTEM[node.ref_body].py * ws_d + pt_node_rel.y * ws_d - ro_y),
                               (float)(SOLAR_SYSTEM[node.ref_body].pz * ws_d + pt_node_rel.z * ws_d - ro_z));
        Vec2 n_scr = ManeuverSystem::projectToScreen(node_world, view, proj, aspect);

        float pop_x, pop_y, pw, ph;
        if (hud.adv_embed_mnv && hud.adv_orbit_menu) {
            pop_x = hud.mnv_popup_px; pop_y = hud.mnv_popup_py;
            pw = hud.mnv_popup_pw; ph = hud.mnv_popup_ph;
        } else {
            pop_x = n_scr.x + 0.22f; pop_y = n_scr.y;
            pw = 0.38f; ph = 0.55f;
            if (pop_x + pw/2 > 0.98f) pop_x = 0.98f - pw/2;
            if (pop_x - pw/2 < -0.98f) pop_x = -0.98f + pw/2;
            if (pop_y + ph/2 > 0.98f) pop_y = 0.98f - ph/2;
            if (pop_y - ph/2 < -0.98f) pop_y = -0.98f + ph/2;
        }
        cached_popup_x = pop_x; cached_popup_y = pop_y;
        cached_popup_w = pw; cached_popup_h = ph;

        hud.mnv_popup_visible = true;
        hud.mnv_popup_px = pop_x; hud.mnv_popup_py = pop_y;
        hud.mnv_popup_pw = pw; hud.mnv_popup_ph = ph;
        hud.mnv_popup_node_scr_x = n_scr.x; hud.mnv_popup_node_scr_y = n_scr.y;
        
        hud.mnv_popup_ref_body = node.ref_body;
        hud.mnv_popup_time_to_node = node.sim_time - tele.sim_time;

        // --- MANEUVER POPUP INTERACTION LOGIC ---
        float title_y = pop_y + ph/2 - (hud.adv_embed_mnv ? 0.015f : 0.025f);
        float slider_track_w = pw * 0.65f;
        float slider_track_h = 0.012f;
        float slider_base_y = title_y - (hud.adv_embed_mnv ? 0.04f : 0.06f);
        float slider_spacing = hud.adv_embed_mnv ? 0.050f : 0.065f;
        float slider_cx = pop_x + 0.02f;

        // 1. Slider Interaction: INCREMENTAL RATE MODE
        if (!lmb) hud.mnv_popup_slider_dragging = -1;
        for (int s = 0; s < 4; s++) {
            float sy = slider_base_y - s * slider_spacing;
            bool hover = (mouse_x >= slider_cx - slider_track_w/2 && mouse_x <= slider_cx + slider_track_w/2 && 
                          mouse_y >= sy - 0.02f && mouse_y <= sy + 0.02f);
            
            // On Click: Capture Initial Anchor (for all sliders)
            if (hover && lmb && !lmb_prev && hud.mnv_popup_slider_dragging == -1) {
                hud.mnv_popup_slider_dragging = s;
                hud.mnv_popup_slider_drag_x = mouse_x; // Click Anchor
            }

            if (hud.mnv_popup_slider_dragging == s) {
                if (s < 3) {
                    // Delta-V Rate Logic: further from center = faster increase/decrease
                    // Using slider_cx (Zero point) for visual center reference
                    float ratio = (mouse_x - slider_cx) / (slider_track_w / 2.0f);
                    float sens = 250.0f; // m/s increase speed at full track width
                    float delta = ratio * (float)dt * sens;

                    if (s == 0) node.delta_v.x += delta;
                    else if (s == 1) node.delta_v.y += delta;
                    else if (s == 2) node.delta_v.z += delta;
                } else {
                    // Time shift Rate Logic: further from click point = faster shift
                    float ratio_rel = (mouse_x - hud.mnv_popup_slider_drag_x) / (slider_track_w / 2.0f);
                    float time_sens = 1200.0f; // seconds shift per sec at full track width
                    node.sim_time += ratio_rel * (float)dt * time_sens;
                    // DO NOT update hud.mnv_popup_slider_drag_x here!
                }
                node.active = true;
                // Sync data back to UI (sliders s<3 use this)
                hud.mnv_popup_dv = node.delta_v;
            }
        }
        
        // Final sync if not dragging
        if (hud.mnv_popup_slider_dragging == -1) {
            hud.mnv_popup_dv = node.delta_v;
        }

        // 2. Button Interaction: CLOSE / MINI
        float close_size = 0.028f;
        float close_x = pop_x + pw/2 - close_size/2 - 0.008f;
        float close_y = pop_y + ph/2 - close_size/2 - 0.008f;
        hud.mnv_popup_close_hover = (mouse_x >= close_x - close_size/2 && mouse_x <= close_x + close_size/2 && 
                                     mouse_y >= close_y - close_size/2 && mouse_y <= close_y + close_size/2);
        hud.mnv_popup_mini_hover = hud.mnv_popup_close_hover;

        if (hud.mnv_popup_close_hover && lmb && !lmb_prev) {
            if (!hud.adv_embed_mnv) { hud.mnv_popup_index = -1; hud.mnv_popup_visible = false; cached_popup_w = 0; }
            else { hud.adv_embed_mnv_mini = !hud.adv_embed_mnv_mini; }
        }

        // 3. Button Interaction: BURN MODE
        float info_y = slider_base_y - 4 * slider_spacing + 0.025f - 0.05f - 0.025f;
        float mode_y = info_y - 0.045f;
        float mbw = pw * 0.8f, mbh = 0.032f;
        hud.mnv_popup_mode_hover = (mouse_x >= pop_x - mbw/2 && mouse_x <= pop_x + mbw/2 && 
                                    mouse_y >= mode_y - mbh/2 && mouse_y <= mode_y + mbh/2);
        if (hud.mnv_popup_mode_hover && lmb && !lmb_prev) {
            hud.mnv_popup_burn_mode = (hud.mnv_popup_burn_mode + 1) % 2;
        }

        // 4. Button Interaction: DELETE
        float del_btn_y = pop_y - ph/2 + (hud.adv_embed_mnv ? 0.015f : 0.025f);
        float del_btn_w = 0.10f, del_btn_h = 0.03f;
        hud.mnv_popup_del_hover = (mouse_x >= pop_x - del_btn_w/2 && mouse_x <= pop_x + del_btn_w/2 && 
                                   mouse_y >= del_btn_y - del_btn_h/2 && mouse_y <= del_btn_y + del_btn_h/2);
        if (hud.mnv_popup_del_hover && lmb && !lmb_prev) {
            mnv.maneuvers.erase(mnv.maneuvers.begin() + hud.mnv_popup_index);
            if (mnv.selected_maneuver_index == hud.mnv_popup_index) mnv.selected_maneuver_index = -1;
            hud.mnv_popup_index = -1;
            hud.mnv_popup_visible = false;
            cached_popup_w = 0;
        }
    }




