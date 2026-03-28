// ==========================================================
// camera_director.h — 飞行模式相机控制器 (Flight Camera Director)
// ----------------------------------------------------------
// 封装 4 种相机模式 (Orbit / Chase / Panorama / Free) 的全部状态与
// 计算逻辑，提供统一的接口供 main.cpp 调用。
// Header-only 设计，无需额外的 .cpp 文件。
// ==========================================================
#pragma once

#include "math/math3d.h"
#include "core/rocket_state.h"  // SOLAR_SYSTEM, current_soi_index
#include "physics/physics_system.h"
#include <cmath>
#include <algorithm>
#include <iostream>

// ==========================================================
// CameraResult — 相机计算的输出
// ==========================================================
struct CameraResult {
    Vec3 eye;        // 相机位置 (相对浮动原点)
    Vec3 target;     // 相机目标 (相对浮动原点)
    Vec3 up;         // 上方向向量
    Mat4 viewMatrix; // 最终视图矩阵
    double origin_x, origin_y, origin_z; // 浮动原点 (双精度世界坐标)
};

// ==========================================================
// CameraDirector — 飞行模式相机控制器
// ==========================================================
class CameraDirector {
public:
    // --- 相机模式枚举 ---
    enum Mode { ORBIT = 0, CHASE = 1, PANORAMA = 2, FREE = 3 };

    // --- 公有状态 (供外部只读访问) ---
    int mode = ORBIT;

    // Orbit 模式参数
    float orbit_yaw   = 0.0f;
    float orbit_pitch  = 0.4f;
    float zoom_chase   = 1.0f;   // Orbit/Chase 缩放
    float zoom_pan     = 1.0f;   // Panorama 独立缩放

    // Panorama 焦点天体
    int focus_target = 3; // 默认 Earth

    // Free 模式
    double free_px = 0, free_py = 0, free_pz = 0;
    float  free_move_speed = 0.0f;

    // 共享四元数 (Panorama / Chase / Free 模式旋转)
    Quat quat;

    // --- 构造函数 ---
    CameraDirector() = default;

    // ============================================
    // cycleMode — 切换到下一个相机模式 (C 键)
    // ============================================
    void cycleMode() {
        mode = (mode + 1) % 4;
        const char* names[] = {"Orbit", "Chase", "Panorama", "Free"};
        std::cout << ">> Camera: " << names[mode] << std::endl;
        if (mode == FREE) free_init_ = false;
    }

    // ============================================
    // cycleFocusTarget — 切换全景焦点天体 (逗号/句号键)
    //   delta: -1 = 上一个, +1 = 下一个
    // ============================================
    void cycleFocusTarget(int delta) {
        int n = (int)SOLAR_SYSTEM.size();
        if (n == 0) return;
        focus_target = ((focus_target + delta) % n + n) % n;
    }

    // ============================================
    // handleMouseInput — 处理鼠标拖拽旋转
    //   rmb/lmb: 右键/左键是否按下
    //   mx/my: 当前鼠标像素坐标
    // ============================================
    void handleMouseInput(bool rmb, bool lmb, double mx, double my) {
        if (rmb || lmb) {
            if (mouse_dragging_) {
                float dx = (float)(mx - prev_mx_) * 0.003f;
                float dy = (float)(my - prev_my_) * 0.003f;

                if (mode == ORBIT) {
                    if (rmb) {
                        orbit_yaw -= dx;
                        orbit_pitch = std::max(-1.5f, std::min(1.5f, orbit_pitch + dy));
                    }
                } else if (mode == FREE) {
                    if (lmb) {
                        // 左键：改变视角方向
                        Vec3 local_up = quat.rotate(Vec3(0, 1, 0));
                        Vec3 local_right = quat.rotate(Vec3(1, 0, 0));
                        Quat q_yaw = Quat::fromAxisAngle(local_up, -dx);
                        Quat q_pitch = Quat::fromAxisAngle(local_right, -dy);
                        quat = (q_yaw * q_pitch * quat).normalized();
                    } else if (rmb) {
                        // 右键：绕 SOI 星球旋转
                        Vec3 rel_v((float)free_px, (float)free_py, (float)free_pz);
                        float dist = (float)rel_v.length();
                        if (dist > 1.0f) {
                            Vec3 planet_z(0, 0, 1);
                            Vec3 pos_norm = rel_v / dist;
                            Vec3 orbit_right = planet_z.cross(pos_norm).normalized();
                            if (orbit_right.length() < 0.01f) orbit_right = Vec3(1, 0, 0);

                            Quat q_orbit_yaw = Quat::fromAxisAngle(planet_z, -dx);
                            Quat q_orbit_pitch = Quat::fromAxisAngle(orbit_right, -dy);
                            Quat q_total = q_orbit_yaw * q_orbit_pitch;

                            Vec3 new_pos = q_total.rotate(rel_v);
                            free_px = (double)new_pos.x;
                            free_py = (double)new_pos.y;
                            free_pz = (double)new_pos.z;

                            quat = (q_total * quat).normalized();
                        }
                    }
                } else {
                    // Panorama / Chase (仅 RMB)
                    if (rmb) {
                        Quat yaw_rot = Quat::fromAxisAngle(Vec3(0.0f, 0.0f, 1.0f), -dx);
                        Vec3 cam_right = quat.rotate(Vec3(1.0f, 0.0f, 0.0f));
                        Quat pitch_rot = Quat::fromAxisAngle(cam_right, -dy);
                        quat = (yaw_rot * pitch_rot * quat).normalized();
                    }
                }
            }
            mouse_dragging_ = true;
        } else {
            mouse_dragging_ = false;
        }
        prev_mx_ = mx;
        prev_my_ = my;
    }

    // ============================================
    // handleFreeCamKeys — 自由模式 QE 滚转
    //   q_key/e_key: 按键是否按下
    // ============================================
    void handleFreeCamRoll(bool q_key, bool e_key) {
        if (mode != FREE) return;
        if (q_key) {
            Quat q_roll = Quat::fromAxisAngle(quat.rotate(Vec3(0, 0, 1)), 0.03f);
            quat = (q_roll * quat).normalized();
        }
        if (e_key) {
            Quat q_roll = Quat::fromAxisAngle(quat.rotate(Vec3(0, 0, 1)), -0.03f);
            quat = (q_roll * quat).normalized();
        }
    }

    // ============================================
    // handleScroll — 处理滚轮缩放
    //   scroll_y: 滚轮增量
    // ============================================
    void handleScroll(float scroll_y) {
        if (scroll_y == 0.0f) return;

        if (mode == PANORAMA) {
            zoom_pan *= powf(0.85f, scroll_y);
            zoom_pan = std::max(0.05f, std::min(500000.0f, zoom_pan));
        } else if (mode == FREE) {
            if (free_move_speed < 1.0f) free_move_speed = 1.0f;
            free_move_speed *= powf(1.25f, scroll_y);
            if (free_move_speed < 1.1f) free_move_speed = 0.0f;
            if (free_move_speed > 1e11f) free_move_speed = 1e11f;
        } else {
            // Orbit / Chase
            zoom_chase *= powf(0.85f, scroll_y);
            zoom_chase = std::max(0.05f, std::min(20.0f, zoom_chase));
        }
    }

    // ============================================
    // computeFlightCamera — 计算飞行模式的视图矩阵
    // ============================================
    // 参数说明：
    //   rocket_state: 火箭的当前状态
    //   ws_d: 世界缩放系数 (通常 0.001)
    //   rocket_height: 火箭渲染高度 (float, 已乘 ws_d)
    //   renderRocketPos: 火箭渲染位置 (相对浮动原点)
    //   rocketDir: 火箭朝向 (unit vector)
    //   rocketUp: 火箭局部 "上" 方向
    //   localNorth, localRight: 地面参考系
    //   prograde_rel, radial_rel, orbit_normal_rel: 轨道参考系
    //   dt: 物理步长 (用于 Free 相机移动)
    //   w/a/s/d: Free 模式 WASD 按键状态
    // ============================================
    CameraResult computeFlightCamera(
        const RocketState& rocket_state,
        double ws_d,
        float rocket_height,
        const Vec3& renderRocketPos,
        const Vec3& rocketDir,
        const Vec3& rocketUp,
        const Vec3& localNorth,
        const Vec3& localRight,
        const Vec3& prograde_rel,
        const Vec3& radial_rel,
        const Vec3& orbit_normal_rel,
        double dt,
        bool key_w, bool key_a, bool key_s, bool key_d
    ) {
        CameraResult result;
        float rh = rocket_height;

        // --- 计算浮动原点 (Floating Origin) ---
        double r_px = rocket_state.abs_px * ws_d;
        double r_py = rocket_state.abs_py * ws_d;
        double r_pz = rocket_state.abs_pz * ws_d;

        if (mode == ORBIT || mode == CHASE) {
            result.origin_x = r_px;
            result.origin_y = r_py;
            result.origin_z = r_pz;
        } else if (mode == PANORAMA) {
            result.origin_x = SOLAR_SYSTEM[focus_target].px * ws_d;
            result.origin_y = SOLAR_SYSTEM[focus_target].py * ws_d;
            result.origin_z = SOLAR_SYSTEM[focus_target].pz * ws_d;
        } else { // FREE
            if (!free_init_) {
                free_px = rocket_state.px;
                free_py = rocket_state.py;
                free_pz = rocket_state.pz;
                free_init_ = true;
            }
            // WASD 移动
            float fwd_move = 0.0f, side_move = 0.0f;
            if (key_w) fwd_move += 1.0f;
            if (key_s) fwd_move -= 1.0f;
            if (key_a) side_move -= 1.0f;
            if (key_d) side_move += 1.0f;

            Vec3 gaze_dir = quat.rotate(Vec3(0, 0, -1));
            Vec3 side_dir = quat.rotate(Vec3(1, 0, 0));

            free_px += (double)(gaze_dir.x * fwd_move + side_dir.x * side_move) * (double)free_move_speed * dt;
            free_py += (double)(gaze_dir.y * fwd_move + side_dir.y * side_move) * (double)free_move_speed * dt;
            free_pz += (double)(gaze_dir.z * fwd_move + side_dir.z * side_move) * (double)free_move_speed * dt;

            result.origin_x = (free_px + SOLAR_SYSTEM[current_soi_index].px) * ws_d;
            result.origin_y = (free_py + SOLAR_SYSTEM[current_soi_index].py) * ws_d;
            result.origin_z = (free_pz + SOLAR_SYSTEM[current_soi_index].pz) * ws_d;
        }

        // --- 计算视图矩阵 ---
        if (mode == ORBIT) {
            // 优化型 Orbit 视角 (带平滑过渡)
            double apo_tmp = 0, peri_tmp = 0;
            PhysicsSystem::getOrbitParams(rocket_state, apo_tmp, peri_tmp);

            float peri_min = 80000.0f;
            float peri_max = 160000.0f;
            float target_t = (float)(peri_tmp - peri_min) / (peri_max - peri_min);
            target_t = std::max(0.0f, std::min(1.0f, target_t));

            float lerp_speed = 1.5f;
            float visual_dt = 0.02f;
            orbit_blend_t_ += (target_t - orbit_blend_t_) * (1.0f - expf(-lerp_speed * visual_dt));

            float orbit_dist = rh * 8.0f * zoom_chase;

            Vec3 ground_view = localNorth * cosf(orbit_yaw) * cosf(orbit_pitch) +
                               localRight * sinf(orbit_yaw) * cosf(orbit_pitch) +
                               rocketUp   * sinf(orbit_pitch);

            Vec3 orbit_view  = prograde_rel * cosf(orbit_yaw) * cosf(orbit_pitch) +
                               radial_rel   * sinf(orbit_yaw) * cosf(orbit_pitch) +
                               orbit_normal_rel * sinf(orbit_pitch);

            Vec3 view_dir = Vec3::lerp(ground_view, orbit_view, orbit_blend_t_).normalized();
            result.up = Vec3::lerp(rocketUp, orbit_normal_rel, orbit_blend_t_).normalized();

            result.eye = renderRocketPos + view_dir * orbit_dist;
            result.target = renderRocketPos;

        } else if (mode == CHASE) {
            float chase_dist = rh * 8.0f * zoom_chase;
            Vec3 chase_base = rocketDir * (-chase_dist * 0.4f) + rocketUp * (chase_dist * 0.15f);
            Vec3 slight_off = quat.rotate(Vec3(0.0f, 0.0f, 1.0f));
            result.eye = renderRocketPos + chase_base + slight_off * (chase_dist * 0.05f);
            result.target = renderRocketPos + rocketDir * (rh * 3.0f);
            result.up = rocketUp;

        } else if (mode == PANORAMA) {
            float base_pan_radius = (float)SOLAR_SYSTEM[focus_target].radius * (float)ws_d;
            float earth_r = (float)SOLAR_SYSTEM[current_soi_index].radius * (float)ws_d;
            float sun_radius = (float)SOLAR_SYSTEM[0].radius * (float)ws_d;
            float pan_dist = fmaxf(base_pan_radius * 4.0f, earth_r * 2.0f) * zoom_pan;
            if (focus_target == 0) pan_dist = sun_radius * 4.0f * zoom_pan;

            Vec3 cam_offset = quat.rotate(Vec3(0.0f, 0.0f, pan_dist));
            Vec3 renderFocus((float)(SOLAR_SYSTEM[focus_target].px * ws_d - result.origin_x),
                             (float)(SOLAR_SYSTEM[focus_target].py * ws_d - result.origin_y),
                             (float)(SOLAR_SYSTEM[focus_target].pz * ws_d - result.origin_z));
            result.eye = renderFocus + cam_offset;
            result.target = renderFocus;
            result.up = quat.rotate(Vec3(0.0f, 1.0f, 0.0f));

        } else { // FREE
            result.eye = Vec3(0, 0, 0); // Origin 就是相机位置
            result.target = quat.rotate(Vec3(0.0f, 0.0f, -1.0f));
            result.up = quat.rotate(Vec3(0.0f, 1.0f, 0.0f));
        }

        result.viewMatrix = Mat4::lookAt(result.eye, result.target, result.up);
        return result;
    }

private:
    // --- 内部状态 ---
    bool   mouse_dragging_ = false;
    double prev_mx_ = 0, prev_my_ = 0;
    bool   free_init_ = false;
    float  orbit_blend_t_ = 0.0f;   // Orbit 模式的地面/轨道插值因子
};
