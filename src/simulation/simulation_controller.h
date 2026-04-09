#pragma once

#include <GLFW/glfw3.h>
#include <iostream>
#include <cmath>
#include <vector>
#include <chrono>
#include <thread>

#include "core/rocket_state.h"
#include <entt/entt.hpp>
#include "physics/physics_system.h"
#include "control/control_system.h"
#include "simulation/maneuver_system.h"
#include "simulation/stage_manager.h"
#include "simulation/orbit_physics.h"
#include "render/HUD_system.h"

class SimulationController {
public:
    int time_warp = 1;
    double dt = 0.02; // 50Hz Fixed Time Step
    double accumulator = 0.0;
    bool mnv_autopilot_active = false;

    void handleInput(GLFWwindow* window, entt::registry& registry, entt::entity entity) {
        auto& tele = registry.get<TelemetryComponent>(entity);
        auto& guid = registry.get<GuidanceComponent>(entity);
        auto& prop = registry.get<PropulsionComponent>(entity);
        // --- 时间加速逻辑 ---
        double surface_speed = std::sqrt(tele.velocity * tele.velocity + tele.local_vx * tele.local_vx);
        bool is_parked = (guid.status == PRE_LAUNCH || guid.status == LANDED) && surface_speed < 0.1;
        bool can_super_warp = (!guid.auto_mode && (prop.thrust_power <= 0.01 || prop.fuel <= 0) && (tele.altitude > 100000.0 || is_parked));

        // Basic Warp (1-4)
        if (glfwGetKey(window, GLFW_KEY_1) == GLFW_PRESS || glfwGetKey(window, GLFW_KEY_KP_1) == GLFW_PRESS) time_warp = 1;
        if (glfwGetKey(window, GLFW_KEY_2) == GLFW_PRESS || glfwGetKey(window, GLFW_KEY_KP_2) == GLFW_PRESS) time_warp = 10;
        if (glfwGetKey(window, GLFW_KEY_3) == GLFW_PRESS || glfwGetKey(window, GLFW_KEY_KP_3) == GLFW_PRESS) time_warp = 100;
        if (glfwGetKey(window, GLFW_KEY_4) == GLFW_PRESS || glfwGetKey(window, GLFW_KEY_KP_4) == GLFW_PRESS) time_warp = 1000;

        // Auto-mode downgrades warp
        if (guid.auto_mode && time_warp > 1000) time_warp = 1;

        // Super Warp (5-8)
        if (!guid.auto_mode) {
            if (can_super_warp) {
                if (glfwGetKey(window, GLFW_KEY_5) == GLFW_PRESS || glfwGetKey(window, GLFW_KEY_KP_5) == GLFW_PRESS) { time_warp = 10000; std::cout << "WARP: 10K SPEED ENGAGED!" << std::endl; }
                if (glfwGetKey(window, GLFW_KEY_6) == GLFW_PRESS || glfwGetKey(window, GLFW_KEY_KP_6) == GLFW_PRESS) { time_warp = 100000; std::cout << "WARP: 100K SPEED ENGAGED!" << std::endl; }
                if (glfwGetKey(window, GLFW_KEY_7) == GLFW_PRESS || glfwGetKey(window, GLFW_KEY_KP_7) == GLFW_PRESS) { time_warp = 1000000; std::cout << "WARP: 1M SPEED ENGAGED!" << std::endl; }
                if (glfwGetKey(window, GLFW_KEY_8) == GLFW_PRESS || glfwGetKey(window, GLFW_KEY_KP_8) == GLFW_PRESS) { time_warp = 10000000; std::cout << "WARP: 10M SPEED ENGAGED!" << std::endl; }
            } else if (time_warp > 1000) {
                // Safety forced reset
                time_warp = 1;
                std::cout << ">> WARP DISENGAGED: Unsafe conditions (Alt < 100km or Engine Active)" << std::endl;
            }
        }
    }

    void update(double real_dt, entt::registry& registry, entt::entity entity, FlightHUD& hud, GLFWwindow* window, int cam_mode) {
        auto& rocket_config = registry.get<RocketConfig>(entity);
        auto& control_input = registry.get<ControlInput>(entity);
        
        auto& trans = registry.get<TransformComponent>(entity);
        auto& vel = registry.get<VelocityComponent>(entity);
        auto& prop = registry.get<PropulsionComponent>(entity);
        auto& tele = registry.get<TelemetryComponent>(entity);
        auto& guid = registry.get<GuidanceComponent>(entity);
        auto& vfx = registry.get<VFXComponent>(entity);
        auto& att = registry.get<AttitudeComponent>(entity);

        // Accumulate time scaled by warp
        accumulator += real_dt * time_warp;

        // Constants used in maneuver logic
        const double G_const = 6.67430e-11;

        // Super acceleration (Physically inaccurate positions, skips integration)
        if (time_warp > 1000) {
            double surface_speed = std::sqrt(tele.velocity * tele.velocity + tele.local_vx * tele.local_vx);
            bool is_parked = (guid.status == PRE_LAUNCH || guid.status == LANDED) && surface_speed < 0.1;
            
            if (is_parked) {
                if (guid.status != PRE_LAUNCH && guid.status != LANDED) {
                    guid.status = LANDED;
                    CelestialBody& cur_b = SOLAR_SYSTEM[current_soi_index];
                    double theta = cur_b.prime_meridian_epoch + (tele.sim_time * 2.0 * PI / cur_b.rotation_period);
                    trans.surf_px = trans.px * std::cos(-theta) - trans.py * std::sin(-theta);
                    trans.surf_py = trans.px * std::sin(-theta) + trans.py * std::cos(-theta);
                    trans.surf_pz = trans.pz;
                }
                vel.vx = 0; vel.vy = 0; vel.vz = 0;
                tele.velocity = 0; tele.local_vx = 0;
                att.ang_vel = 0; att.ang_vel_z = 0; // Notice: this logic should refer to appropriate component
            }
            
            // Advance simulation in big jump
            PhysicsSystem::FastGravityUpdate(registry, entity, accumulator);
            accumulator = 0;
            mnv_autopilot_active = false;
        } else {
            // Standard integration loop
            while (accumulator >= dt) {
                executeManeuvers(registry, entity, hud); 

                if (!mnv_autopilot_active) {
                    ControlSystem::ManualInputs manual = getManualInputs(window, cam_mode); 
                    if (guid.auto_mode) ControlSystem::UpdateAutoPilot(registry, entity, dt);
                    else ControlSystem::UpdateManualControl(registry, entity, manual, dt);
                }

                PhysicsSystem::Update(registry, entity, dt);

                // Auto-staging
                if (StageManager::IsCurrentStageEmpty(prop) 
                    && prop.current_stage < prop.total_stages - 1
                    && (guid.status == ASCEND || guid.status == DESCEND)) {
                    StageManager::SeparateStage(registry, entity);
                }

                accumulator -= dt;
                if (guid.status == LANDED || guid.status == CRASHED) {
                    accumulator = 0;
                    break;
                }
            }

            // Visual smoke updates (only at real time)
            if (time_warp == 1) {
                PhysicsSystem::EmitSmoke(registry, entity, real_dt);
                PhysicsSystem::UpdateSmoke(registry, entity, real_dt);
            }
        }
    }

private:
    void executeManeuvers(entt::registry& registry, entt::entity entity, FlightHUD& hud) {
    auto& rocket_config = registry.get<RocketConfig>(entity);
    auto& control_input = registry.get<ControlInput>(entity);
    auto& prop = registry.get<PropulsionComponent>(entity);
    auto& tele = registry.get<TelemetryComponent>(entity);
    auto& guid = registry.get<GuidanceComponent>(entity);
    auto& mnv = registry.get<ManeuverComponent>(entity);
    auto& trans = registry.get<TransformComponent>(entity);
    auto& vel = registry.get<VelocityComponent>(entity);
    auto& att = registry.get<AttitudeComponent>(entity);
        mnv_autopilot_active = false;
        if (mnv.maneuvers.empty()) return;
        if (!(guid.status == ASCEND || guid.status == DESCEND)) return;

        auto& node = mnv.maneuvers[0];
        
        // 1. Estimate burn
        double total_dv_mag = node.delta_v.length();
        double current_mass = prop.fuel + rocket_config.dry_mass + rocket_config.upper_stages_mass;
        double max_thrust = 0;
        if (prop.current_stage < (int)rocket_config.stage_configs.size())
            max_thrust = rocket_config.stage_configs[prop.current_stage].thrust;
        double ve = rocket_config.specific_impulse * 9.80665;
        
        if (max_thrust > 0 && ve > 0 && total_dv_mag > 0) {
            double m_0 = current_mass;
            double m_f = m_0 * exp(-total_dv_mag / ve);
            node.burn_duration = (m_0 - m_f) / (max_thrust / ve);
        }
        
        double time_to_burn_start = node.sim_time - tele.sim_time;
        
        // 2. Snapshot target state
        if (!node.snap_valid && time_to_burn_start < 5.0) {
            int ref_idx = (node.ref_body >= 0) ? node.ref_body : current_soi_index;
            CelestialBody& ref_b = SOLAR_SYSTEM[ref_idx];
            
            // Ships current relative state to reference body
            double ship_rel_vx = vel.vx + SOLAR_SYSTEM[current_soi_index].vx - ref_b.vx;
            double ship_rel_vy = vel.vy + SOLAR_SYSTEM[current_soi_index].vy - ref_b.vy;
            double ship_rel_vz = vel.vz + SOLAR_SYSTEM[current_soi_index].vz - ref_b.vz;
            double ship_rel_px = trans.px + SOLAR_SYSTEM[current_soi_index].px - ref_b.px;
            double ship_rel_py = trans.py + SOLAR_SYSTEM[current_soi_index].py - ref_b.py;
            double ship_rel_pz = trans.pz + SOLAR_SYSTEM[current_soi_index].pz - ref_b.pz;

            double mu_ref = 6.67430e-11 * ref_b.mass;
            double npx, npy, npz, nvx, nvy, nvz;
            get3DStateAtTime(ship_rel_px, ship_rel_py, ship_rel_pz, 
                            ship_rel_vx, ship_rel_vy, ship_rel_vz, 
                            mu_ref, node.sim_time - tele.sim_time, 
                            npx, npy, npz, nvx, nvy, nvz);

            ManeuverFrame frame = ManeuverSystem::getFrame(Vec3((float)npx, (float)npy, (float)npz), Vec3((float)nvx, (float)nvy, (float)nvz));
            Vec3 target_dv_world = (frame.prograde * node.delta_v.x + frame.normal * node.delta_v.y + frame.radial * node.delta_v.z);
            node.locked_burn_dir = target_dv_world.normalized();
            
            // Capture relative target state for high-precision propagation
            node.snap_rel_px = npx; node.snap_rel_py = npy; node.snap_rel_pz = npz;
            node.snap_rel_vx = nvx + target_dv_world.x;
            node.snap_rel_vy = nvy + target_dv_world.y;
            node.snap_rel_vz = nvz + target_dv_world.z;
            
            // Absolute coordinates for legacy compatibility
            double rbpx, rbpy, rbpz, rbvx, rbvy, rbvz;
            PhysicsSystem::GetCelestialStateAt(ref_idx, node.sim_time, rbpx, rbpy, rbpz, rbvx, rbvy, rbvz);
            node.snap_px = rbpx + npx; node.snap_py = rbpy + npy; node.snap_pz = rbpz + npz;
            node.snap_vx = rbvx + node.snap_rel_vx;
            node.snap_vy = rbvy + node.snap_rel_vy;
            node.snap_vz = rbvz + node.snap_rel_vz;

            node.snap_time = node.sim_time;
            node.snap_valid = true;
        }

        if (!hud.auto_exec_mnv) return;

        // 3. Guidance calculation
        double remaining_dv = total_dv_mag;
        if (node.snap_valid) {
            int ref_idx = (node.ref_body >= 0) ? node.ref_body : current_soi_index;
            CelestialBody& ref_b = SOLAR_SYSTEM[ref_idx];
            
            double cur_rel_vx = vel.vx + SOLAR_SYSTEM[current_soi_index].vx - ref_b.vx;
            double cur_rel_vy = vel.vy + SOLAR_SYSTEM[current_soi_index].vy - ref_b.vy;
            double cur_rel_vz = vel.vz + SOLAR_SYSTEM[current_soi_index].vz - ref_b.vz;
            
            double mu_ref = 6.67430e-11 * ref_b.mass;
            double dt_snap = tele.sim_time - node.snap_time;
            double tpx, tpy, tpz, tvx, tvy, tvz;
            
            // Propagate target trajectory forward to current time
            get3DStateAtTime(node.snap_rel_px, node.snap_rel_py, node.snap_rel_pz, 
                             node.snap_rel_vx, node.snap_rel_vy, node.snap_rel_vz, 
                             mu_ref, dt_snap, tpx, tpy, tpz, tvx, tvy, tvz);
            
            Vec3 rem_v((float)(tvx - cur_rel_vx), (float)(tvy - cur_rel_vy), (float)(tvz - cur_rel_vz));
            remaining_dv = (double)rem_v.dot(node.locked_burn_dir);
            if (rem_v.length() < 0.05) remaining_dv = 0; 
        }

        // 4. Attitude Control
        if (time_to_burn_start < 60.0) {
            mnv_autopilot_active = true;
            Vec3 burn_dir = node.snap_valid ? node.locked_burn_dir : Vec3(0,0,0);
            
            if (burn_dir.length() < 0.1f) {
                double mu = 6.67430e-11 * SOLAR_SYSTEM[current_soi_index].mass;
                double npx, npy, npz, nvx, nvy, nvz;
                get3DStateAtTime(trans.px, trans.py, trans.pz, vel.vx, vel.vy, vel.vz, mu, node.sim_time - tele.sim_time, npx, npy, npz, nvx, nvy, nvz);
                ManeuverFrame frame = ManeuverSystem::getFrame(Vec3((float)npx, (float)npy, (float)npz), Vec3((float)nvx, (float)nvy, (float)nvz));
                burn_dir = (frame.prograde * node.delta_v.x + frame.normal * node.delta_v.y + frame.radial * node.delta_v.z).normalized();
            }

            Vec3 fwd = att.attitude.forward();
            float dot_prod = fwd.dot(burn_dir);
            Vec3 error_axis = fwd.cross(burn_dir);
            float error_mag = error_axis.length();

            if (dot_prod < -0.999f && error_mag < 0.01f) { error_axis = att.attitude.right(); error_mag = 1.0f; }

            double total_mass = rocket_config.dry_mass + prop.fuel + rocket_config.upper_stages_mass;
            float moi = (float)(50000.0 * (total_mass / 50000.0));

            if (error_mag > 0.001f) {
                error_axis = error_axis / error_mag;
                float error_angle = std::asin(std::min(error_mag, 1.0f));
                if (dot_prod < 0) error_angle = (float)PI - error_angle; 
                
                float kp = moi * 32.0f; float kd = moi * 12.0f;
                Vec3 right_axis = att.attitude.right();
                Vec3 up_axis = att.attitude.up();
                control_input.torque_cmd = kp * error_axis.dot(up_axis) * error_angle - kd * (float)att.ang_vel;
                control_input.torque_cmd_z = kp * error_axis.dot(right_axis) * error_angle - kd * (float)att.ang_vel_z;
            } else {
                control_input.torque_cmd = -moi * 8.0f * (float)att.ang_vel;
                control_input.torque_cmd_z = -moi * 8.0f * (float)att.ang_vel_z;
            }
            control_input.torque_cmd_roll = -moi * 8.0f * (float)att.ang_vel_roll;
        }

        // 5. Throttle
        if (time_to_burn_start <= 0 && remaining_dv > 0.5) {
            control_input.throttle = 1.0;
            mnv_autopilot_active = true;
        } else if (node.snap_valid && remaining_dv <= 0.5) {
            control_input.throttle = 0;
            guid.mission_msg = "MNV COMPLETE";
            mnv.maneuvers.erase(mnv.maneuvers.begin());
            if (mnv.selected_maneuver_index >= 0) mnv.selected_maneuver_index--;
            if (mnv.maneuvers.empty()) hud.auto_exec_mnv = false;
        } else if (!mnv_autopilot_active) {
            control_input.throttle = 0;
        }
    }

    ControlSystem::ManualInputs getManualInputs(GLFWwindow* window, int cam_mode) {
        ControlSystem::ManualInputs manual;
        bool shift = glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS || glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS;
        bool ctrl = glfwGetKey(window, GLFW_KEY_LEFT_CONTROL) == GLFW_PRESS || glfwGetKey(window, GLFW_KEY_RIGHT_CONTROL) == GLFW_PRESS;
        manual.throttle_up = shift || glfwGetKey(window, GLFW_KEY_KP_ADD) == GLFW_PRESS || glfwGetKey(window, GLFW_KEY_RIGHT_BRACKET) == GLFW_PRESS;
        manual.throttle_down = ctrl || glfwGetKey(window, GLFW_KEY_KP_SUBTRACT) == GLFW_PRESS || glfwGetKey(window, GLFW_KEY_LEFT_BRACKET) == GLFW_PRESS;
        manual.throttle_max = glfwGetKey(window, GLFW_KEY_Z) == GLFW_PRESS;
        manual.throttle_min = glfwGetKey(window, GLFW_KEY_X) == GLFW_PRESS;
        
        bool free_cam = (cam_mode == 3);
        manual.yaw_left   = free_cam ? (glfwGetKey(window, GLFW_KEY_LEFT) == GLFW_PRESS) : (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS || glfwGetKey(window, GLFW_KEY_LEFT) == GLFW_PRESS);
        manual.yaw_right  = free_cam ? (glfwGetKey(window, GLFW_KEY_RIGHT) == GLFW_PRESS) : (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS || glfwGetKey(window, GLFW_KEY_RIGHT) == GLFW_PRESS);
        manual.pitch_up   = free_cam ? (glfwGetKey(window, GLFW_KEY_UP) == GLFW_PRESS) : (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS || glfwGetKey(window, GLFW_KEY_UP) == GLFW_PRESS);
        manual.pitch_down = free_cam ? (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS) : (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS || glfwGetKey(window, GLFW_KEY_DOWN) == GLFW_PRESS);
        manual.roll_left  = free_cam ? false : glfwGetKey(window, GLFW_KEY_Q) == GLFW_PRESS;
        manual.roll_right = free_cam ? false : glfwGetKey(window, GLFW_KEY_E) == GLFW_PRESS;
        return manual;
    }
};
