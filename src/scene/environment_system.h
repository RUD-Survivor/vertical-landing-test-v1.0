#pragma once
#include "core/rocket_state.h"
#include "simulation/orbit_physics.h" // For SOLAR_SYSTEM
#include <glad/glad.h>
#include <math.h>

/**
 * EnvironmentSystem - 场景与环境光效系统
 * =========================================================================
 * 抽离计算摄像机所处的环境状态（高空大气系数、昼夜交替光照）并设置 OpenGL 的基础天空盒背景底色。
 *
 * 职责：
 *  1. 寻址并计算距离摄像机视角最近的天体
 *  2. 推导当前摄像机视角的大气内/外高度系数 (alt_factor)
 *  3. 基于太阳连线的射线检测，动态计算行星遮挡阴影比例 (day_blend) 的昼夜循环
 *  4. 接管并处理 `glClearColor` (天空蓝光散射与深空纯黑过渡)
 */
class EnvironmentSystem {
public:
    double day_blend;
    float alt_factor;

    EnvironmentSystem() : day_blend(1.0), alt_factor(1.0f) {}

    void updateAndApply(const Vec3& camEye_rel, double ro_x, double ro_y, double ro_z, double ws_d, int cam_mode) {
        // Absolute camera position in heliocentric km (ws_d already applied)
        double cam_abs_x = camEye_rel.x + ro_x;
        double cam_abs_y = camEye_rel.y + ro_y;
        double cam_abs_z = camEye_rel.z + ro_z;
        
        // Find closest celestial body to the camera to determine local environment
        int closest_idx = 0;
        double min_dist_sq = 1e30;
        for (int i = 1; i < (int)SOLAR_SYSTEM.size(); i++) {
            double dx = SOLAR_SYSTEM[i].px * ws_d - cam_abs_x;
            double dy = SOLAR_SYSTEM[i].py * ws_d - cam_abs_y;
            double dz = SOLAR_SYSTEM[i].pz * ws_d - cam_abs_z;
            double d2 = dx * dx + dy * dy + dz * dz;
            if (d2 < min_dist_sq) { min_dist_sq = d2; closest_idx = i; }
        }
        CelestialBody& near_body = SOLAR_SYSTEM[closest_idx];
        double dist_to_center = sqrt(min_dist_sq);
        double body_r_km = near_body.radius * ws_d;
        double cam_alt_km = dist_to_center - body_r_km;
        
        // Atmosphere factor (0.0 at ground, 1.0 in space)
        alt_factor = (float)fmin(fmax(cam_alt_km / 50.0, 0.0), 1.0);
        
        // Shadow check (is this body blocking the sun from the camera's POV?)
        day_blend = 1.0;
        double to_sun_x = -cam_abs_x;
        double to_sun_y = -cam_abs_y;
        double to_sun_z = -cam_abs_z;
        double to_sun_len = sqrt(to_sun_x * to_sun_x + to_sun_y * to_sun_y + to_sun_z * to_sun_z);
        if (to_sun_len > 1.0) {
            double d_x = to_sun_x / to_sun_len;
            double d_y = to_sun_y / to_sun_len;
            double d_z = to_sun_z / to_sun_len;
            // Vector from camera to body center
            double oc_x = near_body.px * ws_d - cam_abs_x;
            double oc_y = near_body.py * ws_d - cam_abs_y;
            double oc_z = near_body.pz * ws_d - cam_abs_z;
            double t_closest = oc_x * d_x + oc_y * d_y + oc_z * d_z;
            if (t_closest > 0 && t_closest < to_sun_len) {
                double cp_x = oc_x - d_x * t_closest;
                double cp_y = oc_y - d_y * t_closest;
                double cp_z = oc_z - d_z * t_closest;
                double closest_dist = sqrt(cp_x * cp_x + cp_y * cp_y + cp_z * cp_z);
                if (closest_dist < body_r_km) {
                    day_blend = 0.0;
                }
                else if (closest_dist < body_r_km * 1.02) {
                    day_blend = (closest_dist - body_r_km) / (body_r_km * 0.02);
                }
            }
        }

        // Clear screen with camera-centric sky color
        if (cam_mode == 2) {
            glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
        }
        else {
            float sky_day = (float)(day_blend * (1.0 - alt_factor));
            glClearColor(0.5f * sky_day, 0.7f * sky_day, 1.0f * sky_day, 1.0f);
        }
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    }
};
