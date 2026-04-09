#pragma once
#include <vector>
#include <mutex>
#include "core/rocket_state.h"
#include "render/renderer3d.h"
#include "camera/camera_director.h"
#include "scene/maneuver_manager.h"
#include "render/HUD_system.h"

// 提供样条插值支持
#include "math/spline.h" 

class OrbitSystem {
public:
    std::vector<Vec3> cached_rel_pts;
    std::vector<Vec3> cached_mnv_rel_pts;
    size_t last_draw_points_size = 0;
    size_t last_draw_mnv_points_size = 0;
    Vec3 last_first_pt = { 0,0,0 }, last_mnv_first_pt = { 0,0,0 };
    struct DVec3 { double x, y, z; };
    struct TrajPoint { DVec3 e; DVec3 s; };
    std::vector<TrajPoint> traj_history;
    std::chrono::steady_clock::time_point last_req_time = std::chrono::steady_clock::now();

    /**
     * 执行轨道预测和渲染的主方法
     */
    void render(entt::registry& registry, entt::entity entity, FlightHUD& hud, ManeuverManager& mnvManager,
        Renderer3D* r3d, CameraDirector& cam,
        const Mat4& viewMat, const Mat4& macroProjMat,
        float aspect, double ws_d, double ro_x, double ro_y, double ro_z,
        int ww, int wh, double dt, int current_soi_index, float earth_r, float cam_dist, Vec3 renderRocketBase, Vec3 camEye_rel)
    {
        auto& rocket_config = registry.get<RocketConfig>(entity);
        auto& control_input = registry.get<ControlInput>(entity);
        auto& mnv = registry.get<ManeuverComponent>(entity);
        auto& trans = registry.get<TransformComponent>(entity);
        auto& vel   = registry.get<VelocityComponent>(entity);
        auto& att   = registry.get<AttitudeComponent>(entity);
        auto& prop  = registry.get<PropulsionComponent>(entity);
        auto& tele  = registry.get<TelemetryComponent>(entity);
        auto& guid  = registry.get<GuidanceComponent>(entity);
        auto& orb   = registry.get<OrbitComponent>(entity);


        // 这里将放置从 FlightScene 移入的庞大渲染逻辑
        // 包括：
        // 1. 高级数值轨道 (Advanced Orbit) 的样条插值绘制
        // 2. 标准开普勒轨道 (Keplerian) 的参考系选择与几何绘制
        // 3. Apsis (Ap/Pe) 标记的看板渲染 (Billboard)
        // 4. 计算并写回 mnvManager.global_best_ang 以支持右键点击轨道创建节点

        {
            double r_px = trans.px, r_py = trans.py, r_pz = trans.pz;
            double sun_px = SOLAR_SYSTEM[0].px, sun_py = SOLAR_SYSTEM[0].py, sun_pz = SOLAR_SYSTEM[0].pz;
            DVec3 curPos = { r_px, r_py, r_pz };
            if (traj_history.empty()) {
                traj_history.push_back({ curPos, {r_px - sun_px, r_py - sun_py, r_pz - sun_pz} });
            }
            else {
                DVec3 bk = traj_history.back().e;
                double move_dist = sqrt((r_px - bk.x) * (r_px - bk.x) + (r_py - bk.y) * (r_py - bk.y) + (r_pz - bk.z) * (r_pz - bk.z));
                if (move_dist > earth_r * 0.002) {
                    traj_history.push_back({ curPos, {r_px - sun_px, r_py - sun_py, r_pz - sun_pz} });
                    if (traj_history.size() > 800) {
                        traj_history.erase(traj_history.begin());
                    }
                }
            }
            // 渲染历史轨迹 (更亮实线: 黄绿色), 增加基于相机拉远的线宽补偿 (仅在 Panorama 显示)
            if (cam.mode == 2 && traj_history.size() >= 2) {
                float hist_w = fmaxf(earth_r * 0.01f, cam_dist * 0.0015f);
                float macro_fade = fminf(1.0f, fmaxf(0.0f, (cam.zoom_pan - 0.05f) / 0.1f));
                if (macro_fade > 0.01f) {
                    std::vector<Vec3> relative_traj;
                    for (auto& pt : traj_history) {
                        double w_px, w_py, w_pz;
                        if (hud.orbit_reference_sun) {
                            w_px = sun_px + pt.s.x;
                            w_py = sun_py + pt.s.y;
                            w_pz = sun_pz + pt.s.z;
                        }
                        else {
                            w_px = pt.e.x;
                            w_py = pt.e.y;
                            w_pz = pt.e.z;
                        }
                        relative_traj.push_back(Vec3((float)(w_px - ro_x), (float)(w_py - ro_y), (float)(w_pz - ro_z)));
                    }
                    r3d->drawRibbon(relative_traj, hist_w, 0.4f, 1.0f, 0.3f, 0.8f * macro_fade);
                }
            }
        }
        // ===== 火箭自身绿色高亮标注 (方便在远景找到) =====
        if (cam.mode == 2) {
            // Scale marker more aggressively with zoom, with a guaranteed minimum visible size
            float base_marker = earth_r * 0.025f * fmaxf(1.0f, cam.zoom_pan * 1.2f);
            // Guarantee a minimum screen-space size (proportional to camera distance)
            float cam_dist_marker = (renderRocketBase - camEye_rel).length();
            float min_marker = cam_dist_marker * 0.008f; // ~0.8% of camera distance = always visible
            float marker_size = fmaxf(base_marker, min_marker);
            r3d->drawBillboard(renderRocketBase, marker_size, 0.2f, 1.0f, 0.4f, 0.9f);
            // Draw a second, larger but fainter halo for extreme zoom-out findability
            if (cam.zoom_pan > 2.0f) {
                float halo_size = marker_size * 2.5f;
                r3d->drawBillboard(renderRocketBase, halo_size, 0.2f, 1.0f, 0.4f, 0.15f);
            }
        }
        // ===== 轨道预测线 (开普勒轨道) =====
        std::vector<Vec3> draw_points, draw_mnv_points;
        if (cam.mode == 2) {
            if (hud.adv_orbit_enabled) {
                // Perform asynchronous numerical orbit prediction
                if (!orb.prediction_in_progress) {
                    // Throttle requests: update every 0.1s of real time if not busy (provides "100x efficiency" at all warp rates)
                    auto now = std::chrono::steady_clock::now();
                    float elapsed_real = std::chrono::duration<float>(now - last_req_time).count();
                    if (elapsed_real > 0.1f || orb.predicted_path.empty()) {
                        orb.prediction_in_progress = true;
                        last_req_time = now;
                        // Populate heliocentric state for the background predictor
                        CelestialBody& soi = SOLAR_SYSTEM[current_soi_index];
                        trans.abs_px = trans.px + soi.px;
                        trans.abs_py = trans.py + soi.py;
                        trans.abs_pz = trans.pz + soi.pz;
                        vel.abs_vx = vel.vx + soi.vx;
                        vel.abs_vy = vel.vy + soi.vy;
                        vel.abs_vz = vel.vz + soi.vz;
                        // Reset only if engines are active or large drift (1 hour of sim time)
                        bool force_reset = (control_input.throttle > 0.01) || (std::abs(tele.sim_time - orb.last_prediction_sim_time) > 3600.0);
                        // Build temporary RocketState for predictor (still uses legacy struct internally)
                        RocketState pred_state;
                        pred_state.px = trans.px; pred_state.py = trans.py; pred_state.pz = trans.pz;
                        pred_state.vx = vel.vx; pred_state.vy = vel.vy; pred_state.vz = vel.vz;
                        pred_state.abs_px = trans.abs_px; pred_state.abs_py = trans.abs_py; pred_state.abs_pz = trans.abs_pz;
                        pred_state.abs_vx = vel.abs_vx; pred_state.abs_vy = vel.abs_vy; pred_state.abs_vz = vel.abs_vz;
                        pred_state.fuel = prop.fuel; pred_state.current_stage = prop.current_stage;
                        pred_state.total_stages = prop.total_stages; pred_state.stage_fuels = prop.stage_fuels;
                        pred_state.sim_time = tele.sim_time; pred_state.altitude = tele.altitude;
                        pred_state.status = guid.status; pred_state.auto_mode = guid.auto_mode;
                        pred_state.angle = att.angle; pred_state.ang_vel = att.ang_vel;
                        pred_state.jettisoned_mass = prop.jettisoned_mass;
                        pred_state.maneuvers = mnv.maneuvers;
                        GameContext::getInstance().orbit_predictor->RequestUpdate(&orb, &mnv, pred_state, rocket_config, hud.adv_orbit_pred_days, hud.adv_orbit_iters, hud.adv_orbit_ref_mode, hud.adv_orbit_ref_body, hud.adv_orbit_secondary_ref_body, force_reset);
                    }
                }
                {
                    std::lock_guard<std::mutex> lock(*orb.path_mutex);
                    draw_points = orb.predicted_path;
                    draw_mnv_points = orb.predicted_mnv_path;
                }
                // Render predicted paths from async buffers
                float ribbon_w = fmaxf(earth_r * 0.006f, cam_dist * 0.001f);
                // Get current reference body position for world reconstruction
                double rb_px, rb_py, rb_pz;
                PhysicsSystem::GetCelestialPositionAt(hud.adv_orbit_ref_body, tele.sim_time, rb_px, rb_py, rb_pz);
                // Get transformation from local to inertial (world)
                Quat q_local_to_inertial = PhysicsSystem::GetFrameRotation(hud.adv_orbit_ref_mode, hud.adv_orbit_ref_body, hud.adv_orbit_secondary_ref_body, tele.sim_time);
                if (!draw_points.empty()) {
                    bool needs_update = (draw_points.size() != last_draw_points_size) || (draw_points[0].x != last_first_pt.x);
                    if (needs_update) {
                        cached_rel_pts = CatmullRomSpline::interpolate(draw_points, 8);
                        last_draw_points_size = draw_points.size();
                        last_first_pt = draw_points[0];
                    }
                    std::vector<Vec3> world_pts;
                    for (const auto& p : cached_rel_pts) {
                        Vec3 p_rot = q_local_to_inertial.rotate(p);
                        double wx = (rb_px + p_rot.x) * ws_d - ro_x;
                        double wy = (rb_py + p_rot.y) * ws_d - ro_y;
                        double wz = (rb_pz + p_rot.z) * ws_d - ro_z;
                        Vec3 pt = Vec3((float)wx, (float)wy, (float)wz);
                        world_pts.push_back(pt);

                        // --- Advanced Maneuver Click Hit-test ---
                        // Only check if we are in the primary orbit reference frame
                        Vec2 scr = ManeuverSystem::projectToScreen(pt, viewMat, macroProjMat, aspect);
                        float mx_f = (float)mnvManager.last_pass_mx, my_f = (float)mnvManager.last_pass_my;
                        float d = sqrtf(powf(scr.x - mx_f, 2) + powf(scr.y - my_f, 2));
                        if (d < 0.05f) { // Interaction threshold
                             // We don't have Keplerian 'ang', but we can use this point for UI feedback if needed
                             // For now, let's just make it 'clickable' enough
                             mnvManager.global_best_ang = 0.0f; // Simplified flag: on-path
                        }
                    }
                    r3d->drawRibbon(world_pts, ribbon_w, 0.4f, 0.8f, 1.0f, 0.85f);
                }
                if (mnv.maneuvers.empty()) draw_mnv_points.clear(); 
                if (!draw_mnv_points.empty()) {
                    bool needs_update = (draw_mnv_points.size() != last_draw_mnv_points_size) || (draw_mnv_points[0].x != last_mnv_first_pt.x);
                    if (needs_update) {
                        cached_mnv_rel_pts = CatmullRomSpline::interpolate(draw_mnv_points, 8);
                        last_draw_mnv_points_size = draw_mnv_points.size();
                        last_mnv_first_pt = draw_mnv_points[0];
                    }
                    std::vector<Vec3> world_mnv_pts;
                    for (const auto& p : cached_mnv_rel_pts) {
                        Vec3 p_rot = q_local_to_inertial.rotate(p);
                        double wx = (rb_px + p_rot.x) * ws_d - ro_x;
                        double wy = (rb_py + p_rot.y) * ws_d - ro_y;
                        double wz = (rb_pz + p_rot.z) * ws_d - ro_z;
                        world_mnv_pts.push_back(Vec3((float)wx, (float)wy, (float)wz));
                    }
                    // Efficient dashed rendering: draw larger batches
                    for (size_t s = 0; s < world_mnv_pts.size(); s += 20) {
                        std::vector<Vec3> dash;
                        for (size_t j = 0; j < 12 && (s + j < world_mnv_pts.size()); j++) {
                            dash.push_back(world_mnv_pts[s + j]);
                        }
                        if (dash.size() >= 2) r3d->drawRibbon(dash, ribbon_w, 1.0f, 0.6f, 0.1f, 0.9f);
                    }
                }
                // Restore current sim state
                PhysicsSystem::UpdateCelestialBodies(tele.sim_time);
            }
            else
            {
                // ==========================================
                // STANDARD ORBIT PREDICTION: KEPLERIAN (SOI)
                // ==========================================
                // We calculate and draw BOTH Earth-relative AND Sun-relative orbits concurrently!
                for (int ref_idx = 0; ref_idx < 2; ref_idx++) {
                    bool is_sun_ref = (ref_idx == 1);
                    // 选择参考系
                    double G_const = 6.67430e-11;
                    double mu_body = is_sun_ref ? (G_const * SOLAR_SYSTEM[0].mass) : (G_const * SOLAR_SYSTEM[current_soi_index].mass);
                    // 全物理量双精度计算 (标准米)
                    double abs_px = trans.px, abs_py = trans.py, abs_pz = trans.pz;
                    double abs_vx = vel.vx, abs_vy = vel.vy, abs_vz = vel.vz;
                    if (is_sun_ref && current_soi_index != 0) {
                        CelestialBody& cb = SOLAR_SYSTEM[current_soi_index];
                        abs_px += cb.px; abs_py += cb.py; abs_pz += cb.pz;
                        abs_vx += cb.vx; abs_vy += cb.vy; abs_vz += cb.vz;
                    }
                    double r_len = sqrt(abs_px * abs_px + abs_py * abs_py + abs_pz * abs_pz);
                    double v_len = sqrt(abs_vx * abs_vx + abs_vy * abs_vy + abs_vz * abs_vz);
                    if (v_len > 0.001 && r_len > SOLAR_SYSTEM[is_sun_ref ? 0 : current_soi_index].radius * 0.5) {
                        double energy = 0.5 * v_len * v_len - mu_body / r_len;
                        Vec3 h_vec((float)(abs_py * abs_vz - abs_pz * abs_vy),
                            (float)(abs_pz * abs_vx - abs_px * abs_vz),
                            (float)(abs_px * abs_vy - abs_py * abs_vx));
                        float h = h_vec.length();
                        double a = -mu_body / (2.0 * energy);
                        Vec3 v_vec((float)abs_vx, (float)abs_vy, (float)abs_vz);
                        Vec3 p_vec((float)abs_px, (float)abs_py, (float)abs_pz);
                        Vec3 e_vec = v_vec.cross(h_vec) / (float)mu_body - p_vec / (float)r_len;
                        float ecc = e_vec.length();
                        float opacity = (is_sun_ref == hud.orbit_reference_sun) ? 0.9f : 0.3f;
                        if (ecc < 1.0f) {
                            // --- 椭圆轨道 (a > 0) ---
                            float b = (float)a * sqrtf(fmaxf(0.0f, 1.0f - ecc * ecc));
                            Vec3 e_dir = ecc > 1e-6f ? e_vec / ecc : Vec3(1.0f, 0.0f, 0.0f);
                            Vec3 perp_dir = h_vec.normalized().cross(e_dir);
                            float periapsis = (float)a * (1.0f - ecc);
                            float apoapsis = (float)a * (1.0f + ecc);
                            bool will_reenter = periapsis < SOLAR_SYSTEM[is_sun_ref ? 0 : current_soi_index].radius && !is_sun_ref;
                            Vec3 center_off = e_dir * (-(float)a * ecc);
                            // 生成预测轨迹点集
                            std::vector<Vec3> orbit_points;
                            int orbit_segs = 120;
                            float best_orb_dist = 1.0f;
                            float best_orb_ang = -1.0f;
                            Vec3 best_orb_pt_tmp;
                            double n_mean_mot = sqrt(mu_body / (a * a * a));
                            double mx_curr, my_curr;
                            glfwGetCursorPos(GameContext::getInstance().window, &mx_curr, &my_curr);
                            float mxf = (float)(mx_curr / ww * 2.0 - 1.0);
                            float myf = (float)(1.0 - my_curr / wh * 2.0);
                            for (int i = 0; i <= orbit_segs; i++) {
                                float ang = (float)i / orbit_segs * 6.2831853f;
                                Vec3 pt_rel = center_off + e_dir * ((float)a * cosf(ang)) + perp_dir * (b * sinf(ang));
                                double px = pt_rel.x, py = pt_rel.y, pz = pt_rel.z;
                                if (!is_sun_ref) {
                                    px += SOLAR_SYSTEM[current_soi_index].px;
                                    py += SOLAR_SYSTEM[current_soi_index].py;
                                    pz += SOLAR_SYSTEM[current_soi_index].pz;
                                }
                                Vec3 pt = Vec3((float)(px * ws_d - ro_x), (float)(py * ws_d - ro_y), (float)(pz * ws_d - ro_z));
                                // Do not clip orbit points beneath the surface so users can see where they crash
                                orbit_points.push_back(pt);
                                // --- Maneuver Click Hit-test ---
                                if (is_sun_ref == hud.orbit_reference_sun) {
                                    Vec2 scr = ManeuverSystem::projectToScreen(pt, viewMat, macroProjMat, (float)ww / wh);
                                    float d = sqrtf(powf(scr.x - mxf, 2) + powf(scr.y - myf, 2));
                                    if (d < best_orb_dist) {
                                        best_orb_dist = d;
                                        best_orb_ang = ang;
                                        best_orb_pt_tmp = pt;
                                    }
                                }
                            }
                            // Store best hit for this ref frame if it's the active one
                            if (is_sun_ref == hud.orbit_reference_sun && best_orb_dist < 0.05f) {
                                mnvManager.global_best_ang = best_orb_ang;
                                mnvManager.global_best_mu = mu_body;
                                mnvManager.global_best_a = a;
                                mnvManager.global_best_ecc = ecc;
                                mnvManager.global_best_pt = best_orb_pt_tmp;
                                mnvManager.global_best_center = center_off;
                                mnvManager.global_best_e_dir = e_dir;
                                mnvManager.global_best_perp_dir = perp_dir;
                                hud.global_best_ref_node = is_sun_ref ? 0 : current_soi_index;
                                double n = sqrt(mu_body / (a * a * a));
                                double cos_E = (a - r_len) / (a * ecc);
                                double sin_E = (abs_px * abs_vx + abs_py * abs_vy + abs_pz * abs_vz) / (ecc * sqrt(mu_body * a));
                                double E0 = atan2(sin_E, cos_E);
                                mnvManager.global_current_M0 = E0 - ecc * sin(E0);
                                mnvManager.global_current_n = n;
                            }
                            // 渲染预测轨迹
                            float pred_w = fmaxf(earth_r * 0.01f, cam_dist * 0.0015f);
                            float macro_fade = fminf(1.0f, fmaxf(0.0f, (cam.zoom_pan - 0.05f) / 0.1f));
                            if (macro_fade > 0.01f) {
                                if (will_reenter) {
                                    r3d->drawRibbon(orbit_points, pred_w, 1.0f, 0.4f, 0.1f, opacity * macro_fade);
                                }
                                else {
                                    if (is_sun_ref) {
                                        r3d->drawRibbon(orbit_points, pred_w, 0.2f, 0.6f, 1.0f, opacity * macro_fade); // Dimmer/bluer for Sun
                                    }
                                    else {
                                        r3d->drawRibbon(orbit_points, pred_w, 0.2f, 0.8f, 1.0f, opacity * macro_fade);
                                    }
                                }
                            }
                            float apsis_size = fminf(earth_r * 0.12f, earth_r * 0.025f * fmaxf(1.0f, cam.zoom_pan * 0.8f));
                            apsis_size *= (is_sun_ref ? 10.0f : 1.0f);
                            // 远地点标记
                            Vec3 apoPos = e_dir * (-apoapsis);
                            double ax = apoPos.x, ay = apoPos.y, az = apoPos.z;
                            if (!is_sun_ref) {
                                ax += SOLAR_SYSTEM[current_soi_index].px;
                                ay += SOLAR_SYSTEM[current_soi_index].py;
                                az += SOLAR_SYSTEM[current_soi_index].pz;
                            }
                            Vec3 w_apoPos((float)(ax * ws_d - ro_x), (float)(ay * ws_d - ro_y), (float)(az * ws_d - ro_z));
                            r3d->drawBillboard(w_apoPos, apsis_size, 0.2f, 0.4f, 1.0f, opacity);
                            // 近地点标记
                            Vec3 periPos = e_dir * periapsis;
                            double px = periPos.x, py = periPos.y, pz = periPos.z;
                            if (!is_sun_ref) {
                                px += SOLAR_SYSTEM[current_soi_index].px;
                                py += SOLAR_SYSTEM[current_soi_index].py;
                                pz += SOLAR_SYSTEM[current_soi_index].pz;
                            }
                            Vec3 w_periPos((float)(px * ws_d - ro_x), (float)(py * ws_d - ro_y), (float)(pz * ws_d - ro_z));
                            r3d->drawBillboard(w_periPos, apsis_size, 1.0f, 0.5f, 0.1f, opacity);
                        }
                        else {
                            // --- 双曲线/抛物线逃逸轨道 (ecc >= 1.0) ---
                            float a_hyp = (float)fabs(a);
                            float b_hyp = a_hyp * sqrtf(fmaxf(0.0f, ecc * ecc - 1.0f));
                            Vec3 e_dir = ecc > 1e-6f ? e_vec / ecc : Vec3(1.0f, 0.0f, 0.0f);
                            Vec3 perp_dir = h_vec.normalized().cross(e_dir);
                            Vec3 center_off = e_dir * (a_hyp * ecc);
                            std::vector<Vec3> escape_points;
                            int escape_segs = 60;
                            float max_sinh = 3.0f;
                            for (int i = -escape_segs; i <= escape_segs; i++) {
                                float t = (float)i / escape_segs * max_sinh;
                                Vec3 pt_rel = center_off - e_dir * (a_hyp * coshf(t)) + perp_dir * (b_hyp * sinhf(t));
                                double dpx = pt_rel.x, dpy = pt_rel.y, dpz = pt_rel.z;
                                if (!is_sun_ref) {
                                    dpx += SOLAR_SYSTEM[current_soi_index].px;
                                    dpy += SOLAR_SYSTEM[current_soi_index].py;
                                    dpz += SOLAR_SYSTEM[current_soi_index].pz;
                                }
                                Vec3 pt((float)(dpx * ws_d - ro_x), (float)(dpy * ws_d - ro_y), (float)(dpz * ws_d - ro_z));
                                if (escape_points.empty() || (pt - escape_points.back()).length() > earth_r * 0.05f) {
                                    escape_points.push_back(pt);
                                }
                            }
                            // 逃逸轨道的 Ribbon
                            float pred_w = fmaxf(earth_r * 0.006f, cam_dist * 0.001f);
                            float macro_fade = fminf(1.0f, fmaxf(0.0f, (cam.zoom_pan - 0.05f) / 0.1f));
                            if (macro_fade > 0.01f) {
                                if (is_sun_ref) r3d->drawRibbon(escape_points, pred_w, 0.9f, 0.2f, 0.8f, opacity * macro_fade);
                                else           r3d->drawRibbon(escape_points, pred_w, 0.8f, 0.3f, 1.0f, opacity * macro_fade);
                            }
                            // 近地点标记
                            float apsis_size = fminf(earth_r * 0.12f, earth_r * 0.025f * fmaxf(1.0f, cam.zoom_pan * 0.8f));
                            apsis_size *= (is_sun_ref ? 10.0f : 1.0f);
                            Vec3 w_periPos((float)((center_off.x - e_dir.x * a_hyp + (is_sun_ref ? 0 : SOLAR_SYSTEM[current_soi_index].px)) * ws_d - ro_x),
                                (float)((center_off.y - e_dir.y * a_hyp + (is_sun_ref ? 0 : SOLAR_SYSTEM[current_soi_index].py)) * ws_d - ro_y),
                                (float)((center_off.z - e_dir.z * a_hyp + (is_sun_ref ? 0 : SOLAR_SYSTEM[current_soi_index].pz)) * ws_d - ro_z));
                            r3d->drawBillboard(w_periPos, apsis_size, 1.0f, 0.5f, 0.1f, opacity);
                        }
                    }
                }
            }
        }
    }
};

