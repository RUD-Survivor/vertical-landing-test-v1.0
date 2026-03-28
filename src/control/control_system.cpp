#include "control_system.h"
#include "physics/physics_system.h"
#include "simulation/maneuver_system.h"
#include "simulation/orbit_physics.h"
#include <algorithm>
#include <cmath>

namespace ControlSystem {

// 自动驾驶逻辑 (Autopilot Logic)
// 负责控制火箭在任务各个阶段的自动操作，如：起飞、重力转向、圆化轨道、重返大气层和软着陆。
void UpdateAutoPilot(RocketState& state, const RocketConfig& config, ControlInput& input, double dt) {
    // 如果火箭还未发射、已经着陆或坠毁，不需要任何控制。
    if (state.status == PRE_LAUNCH || state.status == LANDED || state.status == CRASHED)
        return;

    // 1. Z轴力矩清零（保持在 2D 轨道平面内）
    input.torque_cmd_z = state.pid_att_z.update(0.0, state.angle_z, dt);

    double max_thrust_vac = config.specific_impulse * G0 * config.cosrate;
    double current_total_mass = config.dry_mass + state.fuel + config.upper_stages_mass;
    double current_r = std::sqrt(state.px * state.px + state.py * state.py);
    double current_g = PhysicsSystem::get_gravity(current_r);

    if (state.status == ASCEND) {
        double apo, peri;
        PhysicsSystem::getOrbitParams(state, apo, peri); // 获取当前轨道的远地点和近地点

        if (state.mission_phase == 0) {
            // 阶段 0：上升与初级转向 (Gravity Turn)
            input.throttle = 1.0; // 全功率起飞
            double target_pitch = 0;
            // 经典的重力转向逻辑：在 1000 米以上开始缓慢倾斜
            if (state.altitude > 1000) {
                // 根据高度逐渐将仰角从 90 度降至 0 度 (水平)
                target_pitch = -std::min(1.2, (state.altitude - 1000) / 80000.0 * 1.57);
            }
            // 使用 PID 控制器计算所需的俯仰力矩
            input.torque_cmd = state.pid_att.update(target_pitch, state.angle, dt);

            if (apo > 150000) {
                // 当预定轨道高度达到 150km 时，关闭主发动机 (MECO)
                state.mission_phase = 1;
                state.mission_msg = "MECO! COASTING TO APOAPSIS.";
            }
        } else if (state.mission_phase == 1) {
            // 阶段 1：惯性滑行 (Coasting)
            input.throttle = 0;
            // 姿态调整：指向水平方向，为下一次点火做准备
            input.torque_cmd = state.pid_att.update(-PI / 2.0, state.angle, dt);

            if (state.altitude > 130000 && state.velocity < 50) {
                state.mission_phase = 2;
                state.mission_msg = "CIRCULARIZATION BURN STARTED!";
            }
            if (state.velocity < -50)
                state.mission_phase = 2;
        } else if (state.mission_phase == 2) {
            // 阶段 2：圆化轨道点火 (Circularization)
            input.throttle = 1.0;
            input.torque_cmd = state.pid_att.update(-PI / 2.0, state.angle, dt); 

            if (peri > 140000) {
                state.mission_phase = 3;
                state.mission_timer = 0;
                state.mission_msg = "ORBIT CIRCULARIZED! CRUISING.";
            }
        } else if (state.mission_phase == 3) {
            input.throttle = 0;
            input.torque_cmd = state.pid_att.update(-PI / 2.0, state.angle, dt);

            state.mission_timer += dt;
            if (state.mission_timer > 5000.0) {
                state.mission_phase = 4;
                state.mission_msg = "DE-ORBIT SEQUENCE START.";
            }
        } else if (state.mission_phase == 4) {
            // 阶段 4：脱轨点火 (De-orbit)
            // 计算逆向 (Retrograde) 飞行的角度
            double vel_angle = std::atan2(state.vy, state.vx);
            double align_angle = (vel_angle + PI) - std::atan2(state.py, state.px); 
            while (align_angle > PI) align_angle -= 2 * PI;
            while (align_angle < -PI) align_angle += 2 * PI;

            // 必须先对齐姿态，再进行点火
            input.torque_cmd = state.pid_att.update(align_angle, state.angle, dt);

            if (std::abs(state.angle - align_angle) < 0.1)
                input.throttle = 1.0;
            else
                input.throttle = 0.0;

            if (peri < 30000) {
                // 近地点已降入大气层，停止点火，进入下落状态。
                input.throttle = 0;
                state.status = DESCEND; 
                state.pid_vert.reset();
                state.mission_msg = ">> RE-ENTRY BURN COMPLETE. AEROBRAKING INITIATED.";
            }
        }
        return;
    }
    
    // --- 降落阶段逻辑 (Landing Logic / Hoverslam) ---
    // 这是最复杂的控制逻辑之：在没有GPS的情况下，仅凭物理计算实现软着陆。
    
    // 计算为消除垂直速度所需的加速度 (v^2 = 2ad)
    double req_a_y = (state.velocity * state.velocity) / (2.0 * std::max(1.0, state.altitude));
    if (state.velocity > 0) req_a_y = 0; 
    double req_F_y = current_total_mass * (current_g + req_a_y);

    // 计算为消除水平速度所需的加速度
    double ref_vel = std::max(2.0, std::abs(state.velocity));
    double req_a_x = (-state.local_vx * ref_vel) / (2.0 * std::max(1.0, state.altitude));
    req_a_x += -state.local_vx * 0.5;
    double req_F_x = current_total_mass * req_a_x;

    double req_thrust = std::sqrt(req_F_x * req_F_x + req_F_y * req_F_y);
    bool need_burn = state.suicide_burn_locked;

    if (!need_burn && req_thrust > max_thrust_vac * 0.95 && state.altitude < 4000) {
        // 自杀烧 (Suicide Burn / Hoverslam) 点火触发：
        // 只有当发动机以全功率点火时，刚好能让飞船触地时速度为 0 时，才点火。
        state.suicide_burn_locked = true;
        need_burn = true;
        state.mission_msg = ">> SYNCHRONIZED HOVERSLAM IGNITION!";
    }

    if (state.suicide_burn_locked && state.altitude > 100.0 &&
        req_thrust < current_total_mass * current_g * 0.8) {
        state.suicide_burn_locked = false;
        need_burn = false;
        state.mission_msg = ">> AEROBRAKING COAST... WAITING.";
    }

    if (!need_burn) {
        input.throttle = 0;
        double vel_angle = std::atan2(state.vy, state.vx);
        double align_angle = (vel_angle + PI) - std::atan2(state.py, state.px);
        while (align_angle > PI) align_angle -= 2 * PI;
        while (align_angle < -PI) align_angle += 2 * PI;
        input.torque_cmd = state.pid_att.update(align_angle, state.angle, dt);
    } else {
        double target_angle = std::atan2(req_F_x, req_F_y);

        double max_safe_tilt = 1.5;
        if (req_F_y < max_thrust_vac) {
            max_safe_tilt = std::acos(req_F_y / max_thrust_vac);
        } else {
            max_safe_tilt = 0.0;
        }

        if (state.altitude < 100.0) {
            double base_tilt = (state.altitude / 100.0) * 0.5;
            double rescue_tilt = std::abs(state.local_vx) * 0.025;
            max_safe_tilt = std::min(max_safe_tilt, base_tilt + rescue_tilt);
        }

        if (state.altitude < 2.0 && std::abs(state.local_vx) < 1.0)
            max_safe_tilt = 0.0;

        if (target_angle > max_safe_tilt) target_angle = max_safe_tilt;
        if (target_angle < -max_safe_tilt) target_angle = -max_safe_tilt;

        input.torque_cmd = state.pid_att.update(target_angle, state.angle, dt);

        if (state.altitude > 2.0) {
            double final_thrust = req_F_y / std::max(0.1, std::cos(target_angle));
            input.throttle = final_thrust / max_thrust_vac;
        } else {
            double hover_throttle = (current_total_mass * current_g) / max_thrust_vac;
            input.throttle = hover_throttle + ((-1.5 - state.velocity) * 2.0);
            input.throttle /= std::max(0.8, std::cos(target_angle));
        }
    }

    if (input.throttle > 1.0) input.throttle = 1.0;
    if (input.throttle < 0.0) input.throttle = 0.0;
}

// 手动控制更新 (Manual Control)
// 如果玩家在操作 WASD，我们在这里把玩家的输入转换为发动机推力和 RCS 姿态力矩。
void UpdateManualControl(RocketState& state, const RocketConfig& config, ControlInput& input, const ManualInputs& manual, double dt) {
    if (state.status == PRE_LAUNCH || state.status == LANDED || state.status == CRASHED)
        return;

    if (manual.throttle_up) input.throttle = std::min(1.0, input.throttle + 1.5 * dt);
    if (manual.throttle_down) input.throttle = std::max(0.0, input.throttle - 1.5 * dt);
    if (manual.throttle_max) input.throttle = 1.0;
    if (manual.throttle_min) input.throttle = 0.0;

    input.torque_cmd = 0;
    input.torque_cmd_z = 0;
    input.torque_cmd_roll = 0;
    
    if (state.rcs_active) {
        // RCS (姿态控制系统) 动力强度
        double torque_magnitude = 60000.0;

        bool manual_pitch = false;
        if (manual.pitch_up) { input.torque_cmd_z = torque_magnitude; manual_pitch = true; }
        if (manual.pitch_down) { input.torque_cmd_z = -torque_magnitude; manual_pitch = true; }

        bool manual_yaw = false;
        if (manual.yaw_left) { input.torque_cmd = torque_magnitude; manual_yaw = true; }
        if (manual.yaw_right) { input.torque_cmd = -torque_magnitude; manual_yaw = true; }

        bool manual_roll = false;
        if (manual.roll_left) { input.torque_cmd_roll = torque_magnitude; manual_roll = true; }
        if (manual.roll_right) { input.torque_cmd_roll = -torque_magnitude; manual_roll = true; }

        // SAS (辅助稳定性系统) 逻辑
        if (state.sas_active) {
            double damping_gain = 40000.0;
            
            if (state.sas_mode == SAS_STABILITY) {
                // 稳定性模式：通过产生反向力矩来消除任何不属于玩家操作的旋转（消减角速度）。
                if (!manual_yaw) input.torque_cmd = -state.ang_vel * damping_gain;
                if (!manual_pitch) input.torque_cmd_z = -state.ang_vel_z * damping_gain;
                if (!manual_roll) input.torque_cmd_roll = -state.ang_vel_roll * damping_gain;
            } else {
                double rel_vx = state.vx, rel_vy = state.vy, rel_vz = state.vz;
                double rel_px = state.px, rel_py = state.py, rel_pz = state.pz;
                double speed = std::sqrt(rel_vx*rel_vx + rel_vy*rel_vy + rel_vz*rel_vz);
                
                if (speed > 0.1) {
                    // 计算轨道局部矢量 (Orbital Vectors)
                    Vec3 vP = Vec3((float)(rel_vx / speed), (float)(rel_vy / speed), (float)(rel_vz / speed)); // 顺向 (Prograde)
                    Vec3 posV((float)rel_px, (float)rel_py, (float)rel_pz);
                    Vec3 vN = vP.cross(posV).normalized(); // 法向 (Normal)
                    Vec3 vR = vN.cross(vP).normalized();  // 径向 (Radial)

                    // 根据选中的 SAS 模式确定目标指向矢量
                    if (state.sas_mode == SAS_PROGRADE) state.sas_target_vec = vP;
                    else if (state.sas_mode == SAS_RETROGRADE) state.sas_target_vec = vP * -1.0f;
                    else if (state.sas_mode == SAS_NORMAL) state.sas_target_vec = vN;
                    else if (state.sas_mode == SAS_ANTINORMAL) state.sas_target_vec = vN * -1.0f;
                    else if (state.sas_mode == SAS_RADIAL_IN) state.sas_target_vec = vR * -1.0f;
                    else if (state.sas_mode == SAS_RADIAL_OUT) state.sas_target_vec = vR;
                    else if (state.sas_mode == SAS_MANEUVER && state.selected_maneuver_index != -1 && (size_t)state.selected_maneuver_index < state.maneuvers.size()) {
                        ManeuverNode& node = state.maneuvers[state.selected_maneuver_index];
                        double dt_node = node.sim_time - state.sim_time;
                        double mu = 6.67430e-11 * SOLAR_SYSTEM[current_soi_index].mass;
                        
                        // If we are close to or during the burn, steer towards the remaining delta-v vector (more stable)
                        if (node.snap_valid) {
                           Vec3 rem_v = ManeuverSystem::calculateRemainingDV(state, node);
                           if (rem_v.length() > 0.01f) state.sas_target_vec = rem_v.normalized();
                           else state.sas_target_vec = Vec3(0,0,0);
                        } else {
                           // Fallback: Point towards the planned direction at node time
                           double npx, npy, npz, nvx, nvy, nvz;
                           get3DStateAtTime(state.px, state.py, state.pz, state.vx, state.vy, state.vz, mu, dt_node, npx, npy, npz, nvx, nvy, nvz);
                           ManeuverFrame frame = ManeuverSystem::getFrame(Vec3((float)npx, (float)npy, (float)npz), Vec3((float)nvx, (float)nvy, (float)nvz));
                           Vec3 target_dv_world = frame.prograde * node.delta_v.x + frame.normal * node.delta_v.y + frame.radial * node.delta_v.z;
                           state.sas_target_vec = target_dv_world.normalized();
                           if (target_dv_world.length() < 0.05f) state.sas_target_vec = Vec3(0,0,0);
                        }
                    }
                }

                if (state.sas_target_vec.length() > 0.1f) {
                    // 姿态对齐算法 (Alignment Algorithm)
                    // 本质：通过叉乘计算“火箭头”与“目标矢量”之间的叉积，得到旋转轴和旋转角度。
                    Vec3 rocketNose = state.attitude.forward(); // 火箭当前的朝向
                    Vec3 targetDir = state.sas_target_vec;      // 目标朝向
                    
                    float dot_prod = rocketNose.dot(targetDir);
                    Vec3 errorVec = rocketNose.cross(targetDir); // 误差矢量：方向为旋转轴，模长与角度偏差相关
                    float error_mag = errorVec.length();

                    // 奇异点修复：如果正好背对目标点，叉积为 0，需要随机选一个旋转轴进行 180 度调头。
                    if (dot_prod < -0.999f && error_mag < 0.01f) {
                        errorVec = state.attitude.right();
                        error_mag = 1.0f;
                    }

                    float error_angle = std::acos(std::max(-1.0f, std::min(1.0f, dot_prod)));
                    Vec3 error_axis = (error_mag > 1e-6f) ? (errorVec / error_mag) : Vec3(0,0,0);
                    
                    Vec3 worldError = error_axis * error_angle;
                    // 将世界空间的偏差转换到火箭的局部轴 (Local Frame)
                    Vec3 localError = state.attitude.conjugate().rotate(worldError);
                    
                    double torque_mag = 60000.0;
                    double total_mass = config.dry_mass + state.fuel + config.upper_stages_mass;
                    double moi = (50000.0) * (total_mass / 50000.0);
                    
                    // 统一的 PD 控制律 (PD Controller)
                    // SAS 系统需要计算出“应该以多快的速度旋转”来消除误差。
                    auto get_target_v = [&](float err) {
                        float abs_e = std::abs(err);
                        if (abs_e < 1e-4f) return 0.0;
                        
                        // 刹车曲线 (Braking Curve): v = sqrt(2*a*d)
                        // 这是最完美的物理曲线：在最大扭矩限制下，以最小时间停在目标位置。
                        double max_a = torque_mag / moi;
                        double v_curve = std::sqrt(2.0 * (max_a * 0.5) * abs_e);
                        
                        // 线性区：在角度误差非常小时，降低增益防止震荡，让收尾更加平滑。
                        double v_linear = 4.0 * abs_e; 
                        double v = std::min(v_curve, v_linear);
                        
                        return (double)(err > 0 ? v : -v);
                    };

                    double tvx = get_target_v(localError.x);
                    double tvz = get_target_v(localError.z);
                    
                    double max_v = 1.0; 
                    if (tvx > max_v) tvx = max_v;
                    if (tvx < -max_v) tvx = -max_v;
                    if (tvz > max_v) tvz = max_v;
                    if (tvz < -max_v) tvz = -max_v;

                    // 计算为达到目标速度所需的力矩 (Torque)
                    // 这里的 K_gain 是一个阻尼增益，决定了响应有多快。
                    // 我们希望 K 能让系统恰好在目标速度附近“收敛”，而不产生震荡。
                    double K_gain = moi * 8.0; 
                    double tx = (tvx - state.ang_vel_z) * K_gain;
                    double tz = (tvz - state.ang_vel) * K_gain;

                    // 限制最大力矩：RCS 发动机的推力是有限的。
                    if (tx > torque_mag) tx = torque_mag;
                    if (tx < -torque_mag) tx = -torque_mag;
                    if (tz > torque_mag) tz = torque_mag;
                    if (tz < -torque_mag) tz = -torque_mag;

                    if (!manual_pitch) input.torque_cmd_z = tx;
                    if (!manual_yaw) input.torque_cmd = tz;
                    if (!manual_roll) input.torque_cmd_roll = -state.ang_vel_roll * (moi * 8.0);
                }
            }
        }
    }

    if (state.status == ASCEND && state.velocity < 0 && state.altitude > 1000)
        state.status = DESCEND;
}

} // namespace ControlSystem
