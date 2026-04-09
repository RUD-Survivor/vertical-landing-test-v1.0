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

    // Panorama 焦点目标 (-1 = 火箭, 0+ = SOLAR_SYSTEM 索引)
    int focus_target = -1; // 默认锁定为火箭焦点

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
        // 选项范围为 [-1, n-1]，总共有 n+1 个焦点可选
        int total = n + 1;
        int current = focus_target + 1; // 映射到 [0, n] 区间进行模运算
        current = (current + delta % total + total) % total;
        focus_target = current - 1; // 映射回 [-1, n-1]

        // 打印当前焦点名称到控制台以便调试
        if (focus_target == -1) {
            std::cout << ">> Panorama Focus: Rocket (火箭)" << std::endl;
        } else {
            std::cout << ">> Panorama Focus: " << SOLAR_SYSTEM[focus_target].name << std::endl;
        }
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

        // 确定浮动原点 (Floating Origin)
        // ORBIT, CHASE 以及 焦点为火箭的全景模式 均以火箭为原点以保证近距离渲染精度
        if (mode == ORBIT || mode == CHASE || (mode == PANORAMA && focus_target == -1)) {
            result.origin_x = r_px;
            result.origin_y = r_py;
            result.origin_z = r_pz;
        } else if (mode == PANORAMA) {
            // 焦点为天体时的全景模式，以天体中心为原点（此时火箭在远方可能由于精度抖动而消失，但天体显示极其稳定）
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
            // NOTE: getOrbitParams now needs registry & entity which CameraDirector does not have.
// For now, compute orbit params inline:
{
    double r_cam = std::sqrt(rocket_state.px*rocket_state.px + rocket_state.py*rocket_state.py + rocket_state.pz*rocket_state.pz);
    double v_sq_cam = rocket_state.vx*rocket_state.vx + rocket_state.vy*rocket_state.vy + rocket_state.vz*rocket_state.vz;
    double mu_cam = 6.67430e-11 * SOLAR_SYSTEM[current_soi_index].mass;
    double energy_cam = v_sq_cam / 2.0 - mu_cam / r_cam;
    double hx_cam = rocket_state.py*rocket_state.vz - rocket_state.pz*rocket_state.vy;
    double hy_cam = rocket_state.pz*rocket_state.vx - rocket_state.px*rocket_state.vz;
    double hz_cam = rocket_state.px*rocket_state.vy - rocket_state.py*rocket_state.vx;
    double h_sq_cam = hx_cam*hx_cam + hy_cam*hy_cam + hz_cam*hz_cam;
    double e_sq_cam = 1.0 + 2.0 * energy_cam * h_sq_cam / (mu_cam * mu_cam);
    double e_cam = (e_sq_cam > 0) ? std::sqrt(e_sq_cam) : 0;
    if (energy_cam >= 0) { apo_tmp = 999999999; peri_tmp = (h_sq_cam/mu_cam)/(1.0+e_cam) - SOLAR_SYSTEM[current_soi_index].radius; }
    else { double a_cam = -mu_cam/(2.0*energy_cam); apo_tmp = a_cam*(1.0+e_cam) - SOLAR_SYSTEM[current_soi_index].radius; peri_tmp = a_cam*(1.0-e_cam) - SOLAR_SYSTEM[current_soi_index].radius; }
}

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
            Vec3 renderFocus;
            float base_pan_radius;

            if (focus_target == -1) {
                // 情况 A: 焦点锁定在火箭上
                renderFocus = renderRocketPos;
                base_pan_radius = rh * 2.0f; // 以火箭渲染高度为基础参考半径
            } else {
                // 情况 B: 焦点锁定在某个星球上
                base_pan_radius = (float)SOLAR_SYSTEM[focus_target].radius * (float)ws_d;
                renderFocus = Vec3((float)(SOLAR_SYSTEM[focus_target].px * ws_d - result.origin_x),
                                   (float)(SOLAR_SYSTEM[focus_target].py * ws_d - result.origin_y),
                                   (float)(SOLAR_SYSTEM[focus_target].pz * ws_d - result.origin_z));
            }

            // 环境参考尺寸
            float earth_r = (float)SOLAR_SYSTEM[current_soi_index].radius * (float)ws_d;
            float sun_radius = (float)SOLAR_SYSTEM[0].radius * (float)ws_d;
            
            // 计算相机观察距离：统一火箭与星球的基础观察尺度
            // 采用 [目标物体参考半径*4] 与 [当前所在星球半径*2] 的较大值，以确保全景模式的宏大感
            float pan_dist = fmaxf(base_pan_radius * 4.0f, earth_r * 2.0f) * zoom_pan;

            // 计算最终位置
            Vec3 cam_offset = quat.rotate(Vec3(0.0f, 0.0f, pan_dist));
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
