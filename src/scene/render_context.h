#pragma once
#include "core/rocket_state.h"
#include "camera/camera_director.h"
#include "simulation/orbit_physics.h"
#include "game_context.h"
#include "math/math3d.h"

/**
 * RenderContext - 渲染帧上下文与参考系计算中心
 * =========================================================================
 * 渲染流水线的大脑，负责在 3D 场景实际开始渲染图元前，处理庞杂的矩阵与物理空间的坐标系转换。
 * 
 * 职责：
 *  1. 浮动原点纠正：计算漫游相机偏移 (ro_x, ro_y, ro_z)，防止天文数字坐标导致的 Z-fighting 与精度丢失。
 *  2. 轨道几何与参考系：推导出当前的顺行、法线、反作用力射线、火箭机身坐标系(北向和切向)。
 *  3. 相机矩阵投影：推演 FlightCamera 逻辑，给出当前帧稳定的 ViewMatrix 及相机相对世界的 eye / target 矢量。
 */
class RenderContext {
public:
    double ro_x = 0.0, ro_y = 0.0, ro_z = 0.0;
    float aspect = 1.0f;
    float earth_r = 0.0f;
    double r_px = 0.0, r_py = 0.0, r_pz = 0.0;
    
    double sun_px = 0.0, sun_py = 0.0, sun_pz = 0.0;
    float sun_radius = 0.0f;
    double sun_dist_d = 0.0;

    Quat rocketQuat;
    Vec3 rocketUp, localNorth, localRight;
    Vec3 orbit_normal_rel, prograde_rel, radial_rel;
    
    Vec3 renderRocketBase;
    Vec3 renderSun;
    Vec3 lightVec;
    
    Vec3 camEye_rel;
    Vec3 camTarget_rel;
    Mat4 viewMat;
    Mat4 macroProjMat;
    float cam_dist = 0.0f;
    
    float rh = 0.0f, rw_3d = 0.0f;

    void update(TransformComponent& trans, VelocityComponent& vel, AttitudeComponent& att, TelemetryComponent& tele, GuidanceComponent& guid, const RocketConfig& rocket_config, int current_soi_index, 
                int& last_soi, bool& comma_prev, bool& period_prev, CameraDirector& cam, 
                double ws_d, double dt) 
    {
        earth_r = (float)EARTH_RADIUS * (float)ws_d;
        r_px = trans.abs_px * ws_d;
        r_py = trans.abs_py * ws_d;
        r_pz = trans.abs_pz * ws_d;
        
        CelestialBody& sun_body = SOLAR_SYSTEM[0];
        sun_px = sun_body.px * ws_d;
        sun_py = sun_body.py * ws_d;
        sun_pz = sun_body.pz * ws_d;
        sun_radius = (float)sun_body.radius * ws_d;
        sun_dist_d = sqrt(r_px * r_px + r_py * r_py + r_pz * r_pz);
        
        // --- 按键监听：视点切换 (CameraDirector) ---
        bool comma_now = glfwGetKey(GameContext::getInstance().window, GLFW_KEY_COMMA) == GLFW_PRESS;
        bool period_now = glfwGetKey(GameContext::getInstance().window, GLFW_KEY_PERIOD) == GLFW_PRESS;
        if (comma_now && !comma_prev) cam.cycleFocusTarget(-1);
        if (period_now && !period_prev) cam.cycleFocusTarget(1);
        comma_prev = comma_now; period_prev = period_now;

        // 【深空姿态锁定】
        double local_dist_sq = trans.px * trans.px + trans.py * trans.py + trans.pz * trans.pz;
        double local_dist = sqrt(local_dist_sq);
        rocketUp = Vec3((float)(trans.px / local_dist), (float)(trans.py / local_dist), (float)(trans.pz / local_dist));
        if (tele.altitude > 2000000.0) {
            rocketUp = Vec3(0.0f, 1.0f, 0.0f);
        }

        // ===== BUILD ROCKET ATTITUDE =====
        if (last_soi != current_soi_index) {
            att.initialized = false;
            last_soi = current_soi_index;
        }
        if (!att.initialized) {
            att.attitude = Quat::fromEuler((float)att.angle, (float)att.angle_z, (float)att.angle_roll);
            att.initialized = true;
        }
        rocketQuat = att.attitude;
        Vec3 rocketDir = rocketQuat.rotate(Vec3(0.0f, 1.0f, 0.0f));

        // --- 轨道参考系 ---
        float local_xy_mag = sqrt(rocketUp.x * rocketUp.x + rocketUp.y * rocketUp.y);
        if (local_xy_mag > 1e-4) {
            localRight = Vec3((float)(-rocketUp.y / local_xy_mag), (float)(rocketUp.x / local_xy_mag), 0.0f);
        }
        else {
            localRight = Vec3(1.0f, 0.0f, 0.0f);
        }
        localNorth = rocketUp.cross(localRight).normalized();
        Vec3 v_vec_rel((float)vel.vx, (float)vel.vy, (float)vel.vz);
        Vec3 p_vec_rel((float)trans.px, (float)trans.py, (float)trans.pz);
        Vec3 h_vec_rel = p_vec_rel.cross(v_vec_rel);
        orbit_normal_rel = h_vec_rel.normalized();
        if (orbit_normal_rel.length() < 0.01f) orbit_normal_rel = Vec3(0, 0, 1);
        prograde_rel = v_vec_rel.normalized();
        if (prograde_rel.length() < 0.01f) prograde_rel = localNorth;
        radial_rel = orbit_normal_rel.cross(prograde_rel).normalized();

        // 火箭尺寸
        float rocket_vis_scale = 1.0f;
        rh = (float)rocket_config.height * (float)ws_d * rocket_vis_scale;
        rw_3d = (float)rocket_config.diameter * (float)ws_d * 0.5f * rocket_vis_scale;

        // ===============================================================
        // --- CameraDirector: 计算浮动原点 + 视图矩阵 ---
        // 第一遍: 计算浮动原点 (需传入临时的 renderRocketPos)
        renderRocketBase = Vec3(0.0f, 0.0f, 0.0f); // 暂时置零对应之前的 (float)(r_px - r_px)
        Vec3 renderRocketPos(0, 0, 0); // placeholder
        bool cam_key_w = glfwGetKey(GameContext::getInstance().window, GLFW_KEY_W) == GLFW_PRESS;
        bool cam_key_a = glfwGetKey(GameContext::getInstance().window, GLFW_KEY_A) == GLFW_PRESS;
        bool cam_key_s = glfwGetKey(GameContext::getInstance().window, GLFW_KEY_S) == GLFW_PRESS;
        bool cam_key_d = glfwGetKey(GameContext::getInstance().window, GLFW_KEY_D) == GLFW_PRESS;
        
        CameraResult camResult = cam.computeFlightCamera(
            trans, vel, ws_d, rh, renderRocketPos, rocketDir, rocketUp,
            localNorth, localRight, prograde_rel, radial_rel, orbit_normal_rel,
            dt, cam_key_w, cam_key_a, cam_key_s, cam_key_d);
        
        ro_x = camResult.origin_x;
        ro_y = camResult.origin_y;
        ro_z = camResult.origin_z;
        
        // 用最终浮动原点计算相对坐标 (renderEarth 未被保留在外层，不需要作为成员)
        renderSun = Vec3((float)(sun_px - ro_x), (float)(sun_py - ro_y), (float)(sun_pz - ro_z));
        renderRocketBase = Vec3((float)(r_px - ro_x), (float)(r_py - ro_y), (float)(r_pz - ro_z));
        renderRocketPos = (guid.status == PRE_LAUNCH || guid.status == LANDED)
            ? (renderRocketBase + rocketUp * (rh * 0.425f))
            : renderRocketBase;
            
        // 更新全局光照方向 (指向太阳)
        lightVec = renderSun - renderRocketBase;
        
        // 第二遍: 用最终的 renderRocketPos 计算视图矩阵 (Orbit/Chase 模式依赖它)
        camResult = cam.computeFlightCamera(
            trans, vel, ws_d, rh, renderRocketPos, rocketDir, rocketUp,
            localNorth, localRight, prograde_rel, radial_rel, orbit_normal_rel,
            0.0, false, false, false, false); // dt=0 避免重复移动 Free 相机
            
        camEye_rel = camResult.eye;
        camTarget_rel = camResult.target;
        viewMat = camResult.viewMatrix;
        
        cam_dist = (camEye_rel - camTarget_rel).length();
        
        int ww, wh;
        glfwGetFramebufferSize(GameContext::getInstance().window, &ww, &wh);
        aspect = (float)ww / (float)wh;
        macroProjMat = Mat4::perspective(80.0f, aspect, 10.0f, 10000000000.0f);
    }
};
