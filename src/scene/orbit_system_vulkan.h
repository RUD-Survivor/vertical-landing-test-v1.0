#pragma once
// ==========================================================================
// orbit_system_vulkan.h — Vulkan 轨道计算补充
// 提供 computeOrbitData() 和 extractMarkers()，从 OpenGL render() 中
// 提取纯数学计算部分，供 Vulkan 快照管线消费。
// ==========================================================================
#include "orbit_system.h"
#include "core/universe_model.h"
#include "physics/physics_system.h"

// 在 OrbitSystem 类外部定义辅助结构体（避免修改 orbit_system.h 类定义）
struct KepOrbitCache {
    std::vector<Vec3> bodyPts, sunPts;      // 行星参考系 / 太阳参考系轨道点
    std::vector<Vec4> bodyCols, sunCols;     // 逐点颜色
    Vec3  apoBody, periBody;                 // 远/近地点（行星参考系）
    Vec3  apoSun,  periSun;                  // 远/近地点（太阳参考系）
    float apsisSizeBody = 0, apsisSizeSun = 0;
    bool  valid = false;
};

struct AdvOrbitCache {
    std::vector<Vec3> worldPts;     // 高级预测 Spline 插值后世界坐标
    std::vector<Vec4> cols;
    std::vector<Vec3> mnvDash;      // 变轨虚线
    std::vector<Vec4> mnvCols;
    bool valid = false;
};

struct RocketMarkerCache {
    Vec3  pos;
    float size;
    float haloSize;
    bool  valid = false;
};

// 在主 OrbitSystem 实例上挂载 Vulkan 缓存（通过外部全局/static）
// FlightScene 在 update() 中调用 computeOrbitData()，然后 extractRibbonsVK() /
// extractMarkersVK() 读取缓存

// ---- 计算标准开普勒轨道 + Ap/Pe + 火箭标记 ----
inline void computeOrbitDataVK(OrbitSystem& osys,
    entt::registry& registry, entt::entity entity,
    FlightHUD& hud, CameraDirector& cam,
    double ws_d, double ro_x, double ro_y, double ro_z,
    float earth_r, float cam_dist, Vec3 renderRocketBase, Vec3 camEye_rel,
    KepOrbitCache& kep, AdvOrbitCache& adv, RocketMarkerCache& rkt)
{
    auto& trans = registry.get<TransformComponent>(entity);
    auto& vel   = registry.get<VelocityComponent>(entity);
    auto& tele  = registry.get<TelemetryComponent>(entity);
    auto& mnv   = registry.get<ManeuverComponent>(entity);
    auto& orb   = registry.get<OrbitComponent>(entity);
    auto& guid  = registry.get<GuidanceComponent>(entity);
    auto& prop  = registry.get<PropulsionComponent>(entity);
    auto& att   = registry.get<AttitudeComponent>(entity);
    auto& control_input = registry.get<ControlInput>(entity);
    auto& rocket_config = registry.get<RocketConfig>(entity);

    int soi = UniverseModel::getInstance().current_soi_index;
    CelestialBody& soiBody = UniverseModel::getInstance().solar_system[soi];

    // ===== 火箭绿色标记 =====
    {
        float base_marker = earth_r * 0.025f * fmaxf(1.0f, cam.zoom_pan * 1.2f);
        float cam_dist_marker = (renderRocketBase - camEye_rel).length();
        float min_marker = cam_dist_marker * 0.008f;
        rkt.size = fmaxf(base_marker, min_marker);
        rkt.pos = renderRocketBase;
        rkt.haloSize = (cam.zoom_pan > 2.0f) ? rkt.size * 2.5f : 0.f;
        rkt.valid = true;
    }

    // ===== 高级数值轨道 =====
    if (hud.adv_orbit_enabled) {
        kep = {}; // 清除 Kepler 缓存，防止旧轨道残留
        
        // 预测请求条件：首次 / 参数变化 / 时间跳跃 / 火箭状态变化时周期性更新
        static int  s_last_ref_mode = -1, s_last_ref_body = -1, s_last_sec_body = -1;
        static float s_last_pred_days = -1; static int s_last_iters = -1;
        static double s_last_sim_time = -1.0;
        static auto s_last_periodic = std::chrono::steady_clock::now();
        bool params_changed = (hud.adv_orbit_ref_mode != s_last_ref_mode || hud.adv_orbit_ref_body != s_last_ref_body ||
            hud.adv_orbit_secondary_ref_body != s_last_sec_body ||
            hud.adv_orbit_pred_days != s_last_pred_days || hud.adv_orbit_iters != s_last_iters);
        bool sim_jumped = (std::abs(tele.sim_time - s_last_sim_time) > 3600.0);
        // 火箭在推力或大气中时轨道实时变化，需要周期性更新
        bool state_changing = (control_input.throttle > 0.01) || (tele.altitude < 200000.0);
        auto now = std::chrono::steady_clock::now();
        float elapsed_periodic = std::chrono::duration<float>(now - s_last_periodic).count();
        bool needs_periodic = state_changing && (elapsed_periodic > 0.05f);
        
        if (!orb.prediction_in_progress && (orb.predicted_path.empty() || params_changed || sim_jumped || needs_periodic)) {
            orb.prediction_in_progress = true;
            s_last_ref_mode = hud.adv_orbit_ref_mode; s_last_ref_body = hud.adv_orbit_ref_body;
            s_last_sec_body = hud.adv_orbit_secondary_ref_body;
            s_last_pred_days = hud.adv_orbit_pred_days; s_last_iters = hud.adv_orbit_iters;
            s_last_sim_time = tele.sim_time;
            s_last_periodic = now;
            s_last_sim_time = tele.sim_time;
            trans.abs_px = trans.px + soiBody.px;
            trans.abs_py = trans.py + soiBody.py;
            trans.abs_pz = trans.pz + soiBody.pz;
            vel.abs_vx = vel.vx + soiBody.vx;
            vel.abs_vy = vel.vy + soiBody.vy;
            vel.abs_vz = vel.vz + soiBody.vz;
            bool force_reset = params_changed || sim_jumped || state_changing;
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
                GameContext::getInstance().orbit_predictor->RequestUpdate(&orb, &mnv, pred_state, rocket_config,
                    hud.adv_orbit_pred_days, hud.adv_orbit_iters, hud.adv_orbit_ref_mode,
                    hud.adv_orbit_ref_body, hud.adv_orbit_secondary_ref_body, force_reset);
            }
        {
            std::vector<Vec3> draw_points, draw_mnv_points;
            {
                std::lock_guard<std::mutex> lock(*orb.path_mutex);
                draw_points = orb.predicted_path;
                draw_mnv_points = orb.predicted_mnv_path;
            }
            double rb_px, rb_py, rb_pz;
            PhysicsSystem::GetCelestialPositionAt(hud.adv_orbit_ref_body, tele.sim_time, rb_px, rb_py, rb_pz);
            Quat qLI = PhysicsSystem::GetFrameRotation(hud.adv_orbit_ref_mode, hud.adv_orbit_ref_body,
                hud.adv_orbit_secondary_ref_body, tele.sim_time);
            if (!draw_points.empty()) {
                // 每帧直接从 draw_points 重建 spline（与 OpenGL 路径行为一致）
                osys.cached_rel_pts = CatmullRomSpline::interpolate(draw_points, 8);
                // 先构建临时 buffer，成功后才替换 adv，防止空数据冲掉上一帧的轨道
                std::vector<Vec3> newPts; std::vector<Vec4> newCols;
                for (const auto& p : osys.cached_rel_pts) {
                    Vec3 p_rot = qLI.rotate(p);
                    newPts.push_back(Vec3(
                        (float)((rb_px + p_rot.x) * ws_d - ro_x),
                        (float)((rb_py + p_rot.y) * ws_d - ro_y),
                        (float)((rb_pz + p_rot.z) * ws_d - ro_z)));
                    newCols.push_back(Vec4(0.4f, 0.8f, 1.0f, 0.85f));
                }
                if (!newPts.empty()) {
                    adv.worldPts = std::move(newPts);
                    adv.cols    = std::move(newCols);
                    adv.valid   = true;
                }
            }
            if (mnv.maneuvers.empty()) draw_mnv_points.clear();
            if (!draw_mnv_points.empty()) {
                osys.cached_mnv_rel_pts = CatmullRomSpline::interpolate(draw_mnv_points, 8);
                std::vector<Vec3> newDash; std::vector<Vec4> newDashCols;
                for (const auto& p : osys.cached_mnv_rel_pts) {
                    Vec3 p_rot = qLI.rotate(p);
                    newDash.push_back(Vec3(
                        (float)((rb_px + p_rot.x) * ws_d - ro_x),
                        (float)((rb_py + p_rot.y) * ws_d - ro_y),
                        (float)((rb_pz + p_rot.z) * ws_d - ro_z)));
                    newDashCols.push_back(Vec4(1.0f, 0.6f, 0.1f, 0.9f));
                }
                if (!newDash.empty()) {
                    adv.mnvDash = std::move(newDash);
                    adv.mnvCols = std::move(newDashCols);
                }
            }
        }
        PhysicsSystem::UpdateCelestialBodies(tele.sim_time);
    }

    // ===== 标准开普勒轨道 =====
    if (!hud.adv_orbit_enabled) {
        adv = {}; // 清除高级轨道缓存
        kep = {}; // reset
        for (int ref_idx = 0; ref_idx < 2; ref_idx++) {
            bool is_sun_ref = (ref_idx == 1);
            double G_const = 6.67430e-11;
            double mu_body = is_sun_ref ? (G_const * UniverseModel::getInstance().solar_system[0].mass)
                                        : (G_const * soiBody.mass);
            double abs_px = trans.px, abs_py = trans.py, abs_pz = trans.pz;
            double abs_vx = vel.vx, abs_vy = vel.vy, abs_vz = vel.vz;
            if (is_sun_ref && soi != 0) {
                abs_px += soiBody.px; abs_py += soiBody.py; abs_pz += soiBody.pz;
                abs_vx += soiBody.vx; abs_vy += soiBody.vy; abs_vz += soiBody.vz;
            }
            double r_len = sqrt(abs_px*abs_px + abs_py*abs_py + abs_pz*abs_pz);
            double v_len = sqrt(abs_vx*abs_vx + abs_vy*abs_vy + abs_vz*abs_vz);
            auto& refBody = UniverseModel::getInstance().solar_system[is_sun_ref ? 0 : soi];
            if (v_len <= 0.001 || r_len <= refBody.radius * 0.5) continue;

            double energy = 0.5*v_len*v_len - mu_body/r_len;
            Vec3 h_vec((float)(abs_py*abs_vz - abs_pz*abs_vy),
                       (float)(abs_pz*abs_vx - abs_px*abs_vz),
                       (float)(abs_px*abs_vy - abs_py*abs_vx));
            float h = h_vec.length();
            if (h < 1e-6f) continue;
            double a = -mu_body / (2.0*energy);
            Vec3 p_vec((float)abs_px, (float)abs_py, (float)abs_pz);
            Vec3 e_vec = Vec3((float)abs_vx, (float)abs_vy, (float)abs_vz).cross(h_vec) / (float)mu_body - p_vec / (float)r_len;
            float ecc = e_vec.length();
            float opacity = (is_sun_ref == hud.orbit_reference_sun) ? 0.9f : 0.3f;

            if (ecc < 1.0f) {
                float b = (float)a * sqrtf(fmaxf(0.0f, 1.0f - ecc*ecc));
                Vec3 e_dir = ecc > 1e-6f ? e_vec/ecc : Vec3(1,0,0);
                Vec3 perp_dir = h_vec.normalized().cross(e_dir);
                float periapsis = (float)a*(1.0f - ecc);
                float apoapsis  = (float)a*(1.0f + ecc);
                Vec3 center_off = e_dir * (-(float)a * ecc);

                auto& pts  = is_sun_ref ? kep.sunPts  : kep.bodyPts;
                auto& cols = is_sun_ref ? kep.sunCols : kep.bodyCols;
                int orbit_segs = 120;
                for (int i = 0; i <= orbit_segs; i++) {
                    float ang = (float)i / orbit_segs * 6.2831853f;
                    Vec3 pt_rel = center_off + e_dir*((float)a*cosf(ang)) + perp_dir*(b*sinf(ang));
                    double px = pt_rel.x, py = pt_rel.y, pz = pt_rel.z;
                    if (!is_sun_ref) { px += soiBody.px; py += soiBody.py; pz += soiBody.pz; }
                    pts.push_back(Vec3((float)(px*ws_d - ro_x), (float)(py*ws_d - ro_y), (float)(pz*ws_d - ro_z)));
                    float will_reenter = (!is_sun_ref && periapsis < soiBody.radius) ? 1.0f : 0.0f;
                    cols.push_back(will_reenter ? Vec4(1,0.4f,0.1f,opacity) :
                        (is_sun_ref ? Vec4(0.2f,0.6f,1,opacity) : Vec4(0.2f,0.8f,1,opacity)));
                }

                // Ap/Pe
                float aps = fminf(earth_r * 0.12f, earth_r * 0.025f * fmaxf(1.0f, cam.zoom_pan * 0.8f));
                aps *= (is_sun_ref ? 10.f : 1.f);
                auto apoRel = e_dir * (-apoapsis);
                auto periRel = e_dir * periapsis;
                double ax=apoRel.x, ay=apoRel.y, az=apoRel.z;
                double px2=periRel.x, py2=periRel.y, pz2=periRel.z;
                if (!is_sun_ref) {
                    ax+=soiBody.px; ay+=soiBody.py; az+=soiBody.pz;
                    px2+=soiBody.px; py2+=soiBody.py; pz2+=soiBody.pz;
                }
                if (is_sun_ref) {
                    kep.apoSun  = Vec3((float)(ax*ws_d-ro_x), (float)(ay*ws_d-ro_y), (float)(az*ws_d-ro_z));
                    kep.periSun = Vec3((float)(px2*ws_d-ro_x), (float)(py2*ws_d-ro_y), (float)(pz2*ws_d-ro_z));
                    kep.apsisSizeSun = aps;
                } else {
                    kep.apoBody  = Vec3((float)(ax*ws_d-ro_x), (float)(ay*ws_d-ro_y), (float)(az*ws_d-ro_z));
                    kep.periBody = Vec3((float)(px2*ws_d-ro_x), (float)(py2*ws_d-ro_y), (float)(pz2*ws_d-ro_z));
                    kep.apsisSizeBody = aps;
                }
            } else {
                // 逃逸轨道（简化：只画近地点标记）
                float a_hyp = (float)fabs(a);
                float b_hyp = a_hyp * sqrtf(fmaxf(0, ecc*ecc-1));
                Vec3 e_dir = ecc > 1e-6f ? e_vec/ecc : Vec3(1,0,0);
                Vec3 perp_dir = h_vec.normalized().cross(e_dir);
                Vec3 center_off = e_dir * (a_hyp * ecc);
                auto& pts  = is_sun_ref ? kep.sunPts  : kep.bodyPts;
                auto& cols = is_sun_ref ? kep.sunCols : kep.bodyCols;
                int esc_segs = 60; float max_sinh = 3.0f;
                for (int i = -esc_segs; i <= esc_segs; i++) {
                    float t = (float)i/esc_segs*max_sinh;
                    Vec3 pt_rel = center_off - e_dir*(a_hyp*coshf(t)) + perp_dir*(b_hyp*sinhf(t));
                    double dpx=pt_rel.x, dpy=pt_rel.y, dpz=pt_rel.z;
                    if (!is_sun_ref) { dpx+=soiBody.px; dpy+=soiBody.py; dpz+=soiBody.pz; }
                    Vec3 pt((float)(dpx*ws_d-ro_x),(float)(dpy*ws_d-ro_y),(float)(dpz*ws_d-ro_z));
                    if (pts.empty() || (pt-pts.back()).length() > earth_r*0.05f) {
                        pts.push_back(pt);
                        cols.push_back(is_sun_ref ? Vec4(0.9f,0.2f,0.8f,opacity) : Vec4(0.8f,0.3f,1,opacity));
                    }
                }
                float aps = fminf(earth_r*0.12f, earth_r*0.025f*fmaxf(1.f,cam.zoom_pan*0.8f)) * (is_sun_ref?10:1);
                Vec3 peri((float)((center_off.x-e_dir.x*a_hyp+(!is_sun_ref?soiBody.px:0))*ws_d-ro_x),
                          (float)((center_off.y-e_dir.y*a_hyp+(!is_sun_ref?soiBody.py:0))*ws_d-ro_y),
                          (float)((center_off.z-e_dir.z*a_hyp+(!is_sun_ref?soiBody.pz:0))*ws_d-ro_z));
                if (is_sun_ref) { kep.periSun=peri; kep.apsisSizeSun=aps; }
                else           { kep.periBody=peri; kep.apsisSizeBody=aps; }
            }
        }
        kep.valid = !kep.bodyPts.empty() || !kep.sunPts.empty();
    }
}

// ---- 将 Keplerian 缓存追加到 ribbon 输出 ----
inline void extractKepRibbons(const KepOrbitCache& kep, float earth_r, float cam_dist,
                               std::vector<RibbonSegment>& out) {
    float rw = fminf(fmaxf(earth_r * 0.006f, cam_dist * 0.001f), cam_dist * 0.5f);
    if (kep.bodyPts.size() >= 2) out.push_back({kep.bodyPts, kep.bodyCols, rw});
    if (kep.sunPts.size() >= 2)  out.push_back({kep.sunPts,  kep.sunCols,  rw});
}

// ---- 将高级轨道缓存追加到 ribbon 输出 ----
inline void extractAdvRibbons(const AdvOrbitCache& adv, float earth_r, float cam_dist,
                               std::vector<RibbonSegment>& out) {
    float rw = fminf(fmaxf(earth_r * 0.006f, cam_dist * 0.001f), cam_dist * 0.5f);
    if (adv.worldPts.size() >= 2) out.push_back({adv.worldPts, adv.cols, rw});
    // 变轨虚线：每 12 点一个 dash
    for (size_t s = 0; s < adv.mnvDash.size(); s += 20) {
        std::vector<Vec3> dashPts; std::vector<Vec4> dashCols;
        for (size_t j = 0; j < 12 && (s+j) < adv.mnvDash.size(); j++) {
            dashPts.push_back(adv.mnvDash[s+j]);
            dashCols.push_back(adv.mnvCols[s+j]);
        }
        if (dashPts.size() >= 2) out.push_back({dashPts, dashCols, rw});
    }
}

// ---- 将火箭标记 + Ap/Pe 追加到 billboard 输出 ----
inline void extractOrbitMarkers(const KepOrbitCache& kep, const RocketMarkerCache& rkt,
                                 std::vector<BillboardDraw>& out) {
    // 火箭标记
    if (rkt.valid) {
        BillboardDraw bd{};
        bd.pos[0]=rkt.pos.x; bd.pos[1]=rkt.pos.y; bd.pos[2]=rkt.pos.z;
        bd.size=rkt.size; bd.r=0.2f; bd.g=1.0f; bd.b=0.4f; bd.a=0.9f;
        out.push_back(bd);
        if (rkt.haloSize > 0) {
            BillboardDraw hd{};
            hd.pos[0]=rkt.pos.x; hd.pos[1]=rkt.pos.y; hd.pos[2]=rkt.pos.z;
            hd.size=rkt.haloSize; hd.r=0.2f; hd.g=1.0f; hd.b=0.4f; hd.a=0.15f;
            out.push_back(hd);
        }
    }
    // 开普勒 Ap/Pe
    if (kep.valid) {
        if (kep.apsisSizeBody > 0) {
            BillboardDraw ap{};
            ap.pos[0]=kep.apoBody.x; ap.pos[1]=kep.apoBody.y; ap.pos[2]=kep.apoBody.z;
            ap.size=kep.apsisSizeBody; ap.r=0.2f; ap.g=0.4f; ap.b=1.0f; ap.a=0.9f;
            out.push_back(ap);
            BillboardDraw pe{};
            pe.pos[0]=kep.periBody.x; pe.pos[1]=kep.periBody.y; pe.pos[2]=kep.periBody.z;
            pe.size=kep.apsisSizeBody; pe.r=1.0f; pe.g=0.5f; pe.b=0.1f; pe.a=0.9f;
            out.push_back(pe);
        }
        if (kep.apsisSizeSun > 0) {
            BillboardDraw ap{};
            ap.pos[0]=kep.apoSun.x; ap.pos[1]=kep.apoSun.y; ap.pos[2]=kep.apoSun.z;
            ap.size=kep.apsisSizeSun; ap.r=0.2f; ap.g=0.4f; ap.b=1.0f; ap.a=0.3f;
            out.push_back(ap);
            BillboardDraw pe{};
            pe.pos[0]=kep.periSun.x; pe.pos[1]=kep.periSun.y; pe.pos[2]=kep.periSun.z;
            pe.size=kep.apsisSizeSun; pe.r=1.0f; pe.g=0.5f; pe.b=0.1f; pe.a=0.3f;
            out.push_back(pe);
        }
    }
}
