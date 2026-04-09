#include <entt/entt.hpp>
#include "control_system.h"
#include "physics/physics_system.h"
#include "simulation/maneuver_system.h"
#include "simulation/orbit_physics.h"
#include <algorithm>
#include <cmath>

namespace ControlSystem {

// 自动驾驶逻辑 (Autopilot Logic)
// 负责控制火箭在任务各个阶段的自动操作，如：起飞、重力转向、圆化轨道、重返大气层和软着陆。
void UpdateAutoPilot(entt::registry& registry, entt::entity entity, double dt) {
    auto& config = registry.get<RocketConfig>(entity);
    auto& input = registry.get<ControlInput>(entity);
    auto& trans = registry.get<TransformComponent>(entity);
    auto& vel = registry.get<VelocityComponent>(entity);
    auto& att = registry.get<AttitudeComponent>(entity);
    auto& prop = registry.get<PropulsionComponent>(entity);
    auto& tele = registry.get<TelemetryComponent>(entity);
    auto& guid = registry.get<GuidanceComponent>(entity);
    auto& mnv = registry.get<ManeuverComponent>(entity);
    // 如果火箭还未发射、已经着陆或坠毁，不需要任何控制。
    if (guid.status == PRE_LAUNCH || guid.status == LANDED || guid.status == CRASHED)
        return;

    // 1. Z轴力矩清零（保持在 2D 轨道平面内）
    input.torque_cmd_z = guid.pid_att_z.update(0.0, att.angle_z, dt);

    double max_thrust_vac = config.specific_impulse * G0 * config.cosrate;
    double current_total_mass = config.dry_mass + prop.fuel + config.upper_stages_mass;
    double current_r = std::sqrt(trans.px * trans.px + trans.py * trans.py);
    double current_g = PhysicsSystem::get_gravity(current_r);

    if (guid.status == ASCEND) {
        double apo, peri;
        PhysicsSystem::getOrbitParams(registry, entity, apo, peri); // 获取当前轨道的远地点和近地点

        if (guid.mission_phase == 0) {
            // 阶段 0：上升与初级转向 (Gravity Turn)
            input.throttle = 1.0; // 全功率起飞
            double target_pitch = 0;
            // 经典的重力转向逻辑：在 1000 米以上开始缓慢倾斜
            if (tele.altitude > 1000) {
                // 根据高度逐渐将仰角从 90 度降至 0 度 (水平)
                target_pitch = -std::min(1.2, (tele.altitude - 1000) / 80000.0 * 1.57);
            }
            // 使用 PID 控制器计算所需的俯仰力矩
            input.torque_cmd = guid.pid_att.update(target_pitch, att.angle, dt);

            if (apo > 150000) {
                // 当预定轨道高度达到 150km 时，关闭主发动机 (MECO)
                guid.mission_phase = 1;
                guid.mission_msg = "MECO! COASTING TO APOAPSIS.";
            }
        } else if (guid.mission_phase == 1) {
            // 阶段 1：惯性滑行 (Coasting)
            input.throttle = 0;
            // 姿态调整：指向水平方向，为下一次点火做准备
            input.torque_cmd = guid.pid_att.update(-PI / 2.0, att.angle, dt);

            if (tele.altitude > 130000 && tele.velocity < 50) {
                guid.mission_phase = 2;
                guid.mission_msg = "CIRCULARIZATION BURN STARTED!";
            }
            if (tele.velocity < -50)
                guid.mission_phase = 2;
        } else if (guid.mission_phase == 2) {
            // 阶段 2：圆化轨道点火 (Circularization)
            input.throttle = 1.0;
            input.torque_cmd = guid.pid_att.update(-PI / 2.0, att.angle, dt); 

            if (peri > 140000) {
                guid.mission_phase = 3;
                guid.mission_timer = 0;
                guid.mission_msg = "ORBIT CIRCULARIZED! CRUISING.";
            }
        } else if (guid.mission_phase == 3) {
            input.throttle = 0;
            input.torque_cmd = guid.pid_att.update(-PI / 2.0, att.angle, dt);

            guid.mission_timer += dt;
            if (guid.mission_timer > 5000.0) {
                guid.mission_phase = 4;
                guid.mission_msg = "DE-ORBIT SEQUENCE START.";
            }
        } else if (guid.mission_phase == 4) {
            // 阶段 4：脱轨点火 (De-orbit)
            // 计算逆向 (Retrograde) 飞行的角度
            double vel_angle = std::atan2(vel.vy, vel.vx);
            double align_angle = (vel_angle + PI) - std::atan2(trans.py, trans.px); 
            while (align_angle > PI) align_angle -= 2 * PI;
            while (align_angle < -PI) align_angle += 2 * PI;

            // 必须先对齐姿态，再进行点火
            input.torque_cmd = guid.pid_att.update(align_angle, att.angle, dt);

            if (std::abs(att.angle - align_angle) < 0.1)
                input.throttle = 1.0;
            else
                input.throttle = 0.0;

            if (peri < 30000) {
                // 近地点已降入大气层，停止点火，进入下落状态。
                input.throttle = 0;
                guid.status = DESCEND; 
                guid.pid_vert.reset();
                guid.mission_msg = ">> RE-ENTRY BURN COMPLETE. AEROBRAKING INITIATED.";
            }
        }
        return;
    }
    
    // --- 降落阶段逻辑 (Landing Logic / Hoverslam) ---
    // 这是最复杂的控制逻辑之：在没有GPS的情况下，仅凭物理计算实现软着陆。
    
    // 计算为消除垂直速度所需的加速度 (v^2 = 2ad)
    double req_a_y = (tele.velocity * tele.velocity) / (2.0 * std::max(1.0, tele.altitude));
    if (tele.velocity > 0) req_a_y = 0; 
    double req_F_y = current_total_mass * (current_g + req_a_y);

    // 计算为消除水平速度所需的加速度
    double ref_vel = std::max(2.0, std::abs(tele.velocity));
    double req_a_x = (-tele.local_vx * ref_vel) / (2.0 * std::max(1.0, tele.altitude));
    req_a_x += -tele.local_vx * 0.5;
    double req_F_x = current_total_mass * req_a_x;

    double req_thrust = std::sqrt(req_F_x * req_F_x + req_F_y * req_F_y);
    bool need_burn = guid.suicide_burn_locked;

    if (!need_burn && req_thrust > max_thrust_vac * 0.95 && tele.altitude < 4000) {
        // 自杀烧 (Suicide Burn / Hoverslam) 点火触发：
        // 只有当发动机以全功率点火时，刚好能让飞船触地时速度为 0 时，才点火。
        guid.suicide_burn_locked = true;
        need_burn = true;
        guid.mission_msg = ">> SYNCHRONIZED HOVERSLAM IGNITION!";
    }

    if (guid.suicide_burn_locked && tele.altitude > 100.0 &&
        req_thrust < current_total_mass * current_g * 0.8) {
        guid.suicide_burn_locked = false;
        need_burn = false;
        guid.mission_msg = ">> AEROBRAKING COAST... WAITING.";
    }

    if (!need_burn) {
        input.throttle = 0;
        double vel_angle = std::atan2(vel.vy, vel.vx);
        double align_angle = (vel_angle + PI) - std::atan2(trans.py, trans.px);
        while (align_angle > PI) align_angle -= 2 * PI;
        while (align_angle < -PI) align_angle += 2 * PI;
        input.torque_cmd = guid.pid_att.update(align_angle, att.angle, dt);
    } else {
        double target_angle = std::atan2(req_F_x, req_F_y);

        double max_safe_tilt = 1.5;
        if (req_F_y < max_thrust_vac) {
            max_safe_tilt = std::acos(req_F_y / max_thrust_vac);
        } else {
            max_safe_tilt = 0.0;
        }

        if (tele.altitude < 100.0) {
            double base_tilt = (tele.altitude / 100.0) * 0.5;
            double rescue_tilt = std::abs(tele.local_vx) * 0.025;
            max_safe_tilt = std::min(max_safe_tilt, base_tilt + rescue_tilt);
        }

        if (tele.altitude < 2.0 && std::abs(tele.local_vx) < 1.0)
            max_safe_tilt = 0.0;

        if (target_angle > max_safe_tilt) target_angle = max_safe_tilt;
        if (target_angle < -max_safe_tilt) target_angle = -max_safe_tilt;

        input.torque_cmd = guid.pid_att.update(target_angle, att.angle, dt);

        if (tele.altitude > 2.0) {
            double final_thrust = req_F_y / std::max(0.1, std::cos(target_angle));
            input.throttle = final_thrust / max_thrust_vac;
        } else {
            double hover_throttle = (current_total_mass * current_g) / max_thrust_vac;
            input.throttle = hover_throttle + ((-1.5 - tele.velocity) * 2.0);
            input.throttle /= std::max(0.8, std::cos(target_angle));
        }
    }

    if (input.throttle > 1.0) input.throttle = 1.0;
    if (input.throttle < 0.0) input.throttle = 0.0;
}

// 手动控制更新 (Manual Control)
// 如果玩家在操作 WASD，我们在这里把玩家的输入转换为发动机推力和 RCS 姿态力矩。
void UpdateManualControl(entt::registry& registry, entt::entity entity, const ManualInputs& manual, double dt) {
    auto& config = registry.get<RocketConfig>(entity);
    auto& input = registry.get<ControlInput>(entity);
    auto& att = registry.get<AttitudeComponent>(entity);
    auto& guid = registry.get<GuidanceComponent>(entity);
    auto& prop = registry.get<PropulsionComponent>(entity);
    auto& mnv = registry.get<ManeuverComponent>(entity);
    auto& trans = registry.get<TransformComponent>(entity);
    auto& vel = registry.get<VelocityComponent>(entity);
    auto& tele = registry.get<TelemetryComponent>(entity);
    if (guid.status == PRE_LAUNCH || guid.status == LANDED || guid.status == CRASHED)
        return;

    if (manual.throttle_up) input.throttle = std::min(1.0, input.throttle + 1.5 * dt);
    if (manual.throttle_down) input.throttle = std::max(0.0, input.throttle - 1.5 * dt);
    if (manual.throttle_max) input.throttle = 1.0;
    if (manual.throttle_min) input.throttle = 0.0;

    input.torque_cmd = 0;
    input.torque_cmd_z = 0;
    input.torque_cmd_roll = 0;
    
    if (guid.rcs_active) {
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
        if (guid.sas_active) {
            double damping_gain = 40000.0;
            
            if (guid.sas_mode == SAS_STABILITY) {
                // 稳定性模式：通过产生反向力矩来消除任何不属于玩家操作的旋转（消减角速度）。
                if (!manual_yaw) input.torque_cmd = -att.ang_vel * damping_gain;
                if (!manual_pitch) input.torque_cmd_z = -att.ang_vel_z * damping_gain;
                if (!manual_roll) input.torque_cmd_roll = -att.ang_vel_roll * damping_gain;
            } else {
                double rel_vx = vel.vx, rel_vy = vel.vy, rel_vz = vel.vz;
                double rel_px = trans.px, rel_py = trans.py, rel_pz = trans.pz;
                double speed = std::sqrt(rel_vx*rel_vx + rel_vy*rel_vy + rel_vz*rel_vz);
                
                if (speed > 0.1) {
                    // 计算轨道局部矢量 (Orbital Vectors)
                    Vec3 vP = Vec3((float)(rel_vx / speed), (float)(rel_vy / speed), (float)(rel_vz / speed)); // 顺向 (Prograde)
                    Vec3 posV((float)rel_px, (float)rel_py, (float)rel_pz);
                    Vec3 vN = vP.cross(posV).normalized(); // 法向 (Normal)
                    Vec3 vR = vN.cross(vP).normalized();  // 径向 (Radial)

                    // 根据选中的 SAS 模式确定目标指向矢量
                    if (guid.sas_mode == SAS_PROGRADE) guid.sas_target_vec = vP;
                    else if (guid.sas_mode == SAS_RETROGRADE) guid.sas_target_vec = vP * -1.0f;
                    else if (guid.sas_mode == SAS_NORMAL) guid.sas_target_vec = vN;
                    else if (guid.sas_mode == SAS_ANTINORMAL) guid.sas_target_vec = vN * -1.0f;
                    else if (guid.sas_mode == SAS_RADIAL_IN) guid.sas_target_vec = vR * -1.0f;
                    else if (guid.sas_mode == SAS_RADIAL_OUT) guid.sas_target_vec = vR;
                    else if (guid.sas_mode == SAS_MANEUVER && mnv.selected_maneuver_index != -1 && (size_t)mnv.selected_maneuver_index < mnv.maneuvers.size()) {
                        ManeuverNode& node = mnv.maneuvers[mnv.selected_maneuver_index];
                        double dt_node = node.sim_time - tele.sim_time;
                        double mu = 6.67430e-11 * SOLAR_SYSTEM[current_soi_index].mass;
                        
                        // If we are close to or during the burn, steer towards the remaining delta-v vector (more stable)
                        if (node.snap_valid) {
                           Vec3 rem_v = ManeuverSystem::calculateRemainingDV(vel, tele, node);
                           if (rem_v.length() > 0.01f) guid.sas_target_vec = rem_v.normalized();
                           else guid.sas_target_vec = Vec3(0,0,0);
                        } else {
                           // Fallback: Point towards the planned direction at node time
                           double npx, npy, npz, nvx, nvy, nvz;
                           get3DStateAtTime(trans.px, trans.py, trans.pz, vel.vx, vel.vy, vel.vz, mu, dt_node, npx, npy, npz, nvx, nvy, nvz);
                           ManeuverFrame frame = ManeuverSystem::getFrame(Vec3((float)npx, (float)npy, (float)npz), Vec3((float)nvx, (float)nvy, (float)nvz));
                           Vec3 target_dv_world = frame.prograde * node.delta_v.x + frame.normal * node.delta_v.y + frame.radial * node.delta_v.z;
                           guid.sas_target_vec = target_dv_world.normalized();
                           if (target_dv_world.length() < 0.05f) guid.sas_target_vec = Vec3(0,0,0);
                        }
                    }
                }

                if (guid.sas_target_vec.length() > 0.1f) {
                    // 姿态对齐算法 (Alignment Algorithm)
                    // 本质：通过叉乘计算“火箭头”与“目标矢量”之间的叉积，得到旋转轴和旋转角度。
                    Vec3 rocketNose = att.attitude.forward(); // 火箭当前的朝向
                    Vec3 targetDir = guid.sas_target_vec;      // 目标朝向
                    
                    float dot_prod = rocketNose.dot(targetDir);
                    Vec3 errorVec = rocketNose.cross(targetDir); // 误差矢量：方向为旋转轴，模长与角度偏差相关
                    float error_mag = errorVec.length();

                    // 奇异点修复：如果正好背对目标点，叉积为 0，需要随机选一个旋转轴进行 180 度调头。
                    if (dot_prod < -0.999f && error_mag < 0.01f) {
                        errorVec = att.attitude.right();
                        error_mag = 1.0f;
                    }

                    float error_angle = std::acos(std::max(-1.0f, std::min(1.0f, dot_prod)));
                    Vec3 error_axis = (error_mag > 1e-6f) ? (errorVec / error_mag) : Vec3(0,0,0);
                    
                    Vec3 worldError = error_axis * error_angle;
                    // 将世界空间的偏差转换到火箭的局部轴 (Local Frame)
                    Vec3 localError = att.attitude.conjugate().rotate(worldError);
                    
                    double torque_mag = 60000.0;
                    double total_mass = config.dry_mass + prop.fuel + config.upper_stages_mass;
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
                    double tx = (tvx - att.ang_vel_z) * K_gain;
                    double tz = (tvz - att.ang_vel) * K_gain;

                    // 限制最大力矩：RCS 发动机的推力是有限的。
                    if (tx > torque_mag) tx = torque_mag;
                    if (tx < -torque_mag) tx = -torque_mag;
                    if (tz > torque_mag) tz = torque_mag;
                    if (tz < -torque_mag) tz = -torque_mag;

                    if (!manual_pitch) input.torque_cmd_z = tx;
                    if (!manual_yaw) input.torque_cmd = tz;
                    if (!manual_roll) input.torque_cmd_roll = -att.ang_vel_roll * (moi * 8.0);
                }
            }
        }
    }

    if (guid.status == ASCEND && tele.velocity < 0 && tele.altitude > 1000)
        guid.status = DESCEND;
}

} // namespace ControlSystem
