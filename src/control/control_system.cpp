#include "control_system.h"
#include "physics/physics_system.h"
#include <algorithm>
#include <cmath>

namespace ControlSystem {

void UpdateAutoPilot(RocketState& state, const RocketConfig& config, ControlInput& input, double dt) {
    if (state.status == PRE_LAUNCH || state.status == LANDED || state.status == CRASHED)
        return;

    // Zero out-of-plane torque
    input.torque_cmd_z = state.pid_att_z.update(0.0, state.angle_z, dt);

    double max_thrust_vac = config.specific_impulse * G0 * config.cosrate;
    double current_total_mass = config.dry_mass + state.fuel + config.upper_stages_mass;
    double current_r = std::sqrt(state.px * state.px + state.py * state.py);
    double current_g = PhysicsSystem::get_gravity(current_r);

    if (state.status == ASCEND) {
        double apo, peri;
        PhysicsSystem::getOrbitParams(state, apo, peri); 

        if (state.mission_phase == 0) {
            input.throttle = 1.0;
            double target_pitch = 0;
            if (state.altitude > 1000) {
                target_pitch = -std::min(1.2, (state.altitude - 1000) / 80000.0 * 1.57);
            }
            input.torque_cmd = state.pid_att.update(target_pitch, state.angle, dt);

            if (apo > 150000) {
                state.mission_phase = 1;
                state.mission_msg = "MECO! COASTING TO APOAPSIS.";
            }
        } else if (state.mission_phase == 1) {
            input.throttle = 0;
            input.torque_cmd = state.pid_att.update(-PI / 2.0, state.angle, dt);

            if (state.altitude > 130000 && state.velocity < 50) {
                state.mission_phase = 2;
                state.mission_msg = "CIRCULARIZATION BURN STARTED!";
            }
            if (state.velocity < -50)
                state.mission_phase = 2;
        } else if (state.mission_phase == 2) {
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
            double vel_angle = std::atan2(state.vy, state.vx);
            double align_angle = (vel_angle + PI) - std::atan2(state.py, state.px); 
            while (align_angle > PI) align_angle -= 2 * PI;
            while (align_angle < -PI) align_angle += 2 * PI;

            input.torque_cmd = state.pid_att.update(align_angle, state.angle, dt);

            if (std::abs(state.angle - align_angle) < 0.1)
                input.throttle = 1.0;
            else
                input.throttle = 0.0;

            if (peri < 30000) {
                input.throttle = 0;
                state.status = DESCEND; 
                state.pid_vert.reset();
                state.mission_msg = ">> RE-ENTRY BURN COMPLETE. AEROBRAKING INITIATED.";
            }
        }
        return;
    }
    
    // --- DESCEND Phase ---
    state.status = DESCEND;

    double req_a_y = (state.velocity * state.velocity) / (2.0 * std::max(1.0, state.altitude));
    if (state.velocity > 0) req_a_y = 0; 
    double req_F_y = current_total_mass * (current_g + req_a_y);

    double ref_vel = std::max(2.0, std::abs(state.velocity));
    double req_a_x = (-state.local_vx * ref_vel) / (2.0 * std::max(1.0, state.altitude));
    req_a_x += -state.local_vx * 0.5;
    double req_F_x = current_total_mass * req_a_x;

    double req_thrust = std::sqrt(req_F_x * req_F_x + req_F_y * req_F_y);
    bool need_burn = state.suicide_burn_locked;

    if (!need_burn && req_thrust > max_thrust_vac * 0.95 && state.altitude < 4000) {
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

        if (state.sas_active) {
            double damping_gain = 40000.0;
            
            if (state.sas_mode == SAS_STABILITY) {
                if (!manual_yaw) input.torque_cmd = -state.ang_vel * damping_gain;
                if (!manual_pitch) input.torque_cmd_z = -state.ang_vel_z * damping_gain;
                if (!manual_roll) input.torque_cmd_roll = -state.ang_vel_roll * damping_gain;
            } else {
                double rel_vx = state.vx, rel_vy = state.vy, rel_vz = state.vz;
                double rel_px = state.px, rel_py = state.py, rel_pz = state.pz;
                double speed = std::sqrt(rel_vx*rel_vx + rel_vy*rel_vy + rel_vz*rel_vz);
                
                if (speed > 0.1) {
                    Vec3 vP = Vec3((float)(rel_vx / speed), (float)(rel_vy / speed), (float)(rel_vz / speed));
                    Vec3 posV((float)rel_px, (float)rel_py, (float)rel_pz);
                    Vec3 vN = vP.cross(posV).normalized();
                    Vec3 vR = vN.cross(vP).normalized();

                    if (state.sas_mode == SAS_PROGRADE) state.sas_target_vec = vP;
                    else if (state.sas_mode == SAS_RETROGRADE) state.sas_target_vec = vP * -1.0f;
                    else if (state.sas_mode == SAS_NORMAL) state.sas_target_vec = vN;
                    else if (state.sas_mode == SAS_ANTINORMAL) state.sas_target_vec = vN * -1.0f;
                    else if (state.sas_mode == SAS_RADIAL_IN) state.sas_target_vec = vR * -1.0f;
                    else if (state.sas_mode == SAS_RADIAL_OUT) state.sas_target_vec = vR;
                }

                if (state.sas_target_vec.length() > 0.1f) {
                    Vec3 rocketNose = state.attitude.forward();
                    Vec3 targetDir = state.sas_target_vec;
                    Vec3 errorVec = rocketNose.cross(targetDir);

                    // Singularity fix: if looking exactly AWAY from target, errorVec is zero.
                    // Dot product will be -1.0 in this case.
                    if (rocketNose.dot(targetDir) < -0.999f && errorVec.length() < 0.01f) {
                        // Inject a small perturbation to break the symmetry
                        // Pick an axis we aren't currently aligned with to start a turn
                        errorVec = rocketNose.cross(Vec3(0, 0, 1.0f));
                        if (errorVec.length() < 0.01f) errorVec = rocketNose.cross(Vec3(1.0f, 0, 0));
                    }

                    Vec3 localError = state.attitude.conjugate().rotate(errorVec);
                    
                    double torque_mag = 60000.0; // Synchronized with manual control
                    double base_moi = 50000.0;
                    double total_mass = config.dry_mass + state.fuel + config.upper_stages_mass;
                    double moi = base_moi * (total_mass / 50000.0);
                    double max_a = torque_mag / moi;
                    
                    // High-performance velocity law with linear region near zero to prevent chatter
                    auto get_v = [&](float err) {
                        float abs_e = std::abs(err);
                        if (abs_e < 1e-4f) return 0.0;
                        
                        // sqrt(2*a*d) braking curve
                        double v_curve = std::sqrt(2.0 * (max_a * 0.5) * abs_e);
                        
                        // Linear region for smooth landing at target: v = k * error
                        // We blend or use a transition to ensure stability
                        double v_linear = 5.0 * abs_e; 
                        double v = std::min(v_curve, v_linear);
                        
                        return (double)(err > 0 ? v : -v);
                    };

                    double target_vx = get_v(localError.x);
                    double target_vz = get_v(localError.z);
                    
                    double max_v = 1.0; // Slightly slower for better stability
                    if (target_vx > max_v) target_vx = max_v;
                    if (target_vx < -max_v) target_vx = -max_v;
                    if (target_vz > max_v) target_vz = max_v;
                    if (target_vz < -max_v) target_vz = -max_v;

                    // Lower gain for stability
                    double K = moi * 2.0; 
                    double tx = (target_vx - state.ang_vel_z) * K;
                    double tz = (target_vz - state.ang_vel) * K;

                    if (tx > torque_mag) tx = torque_mag;
                    if (tx < -torque_mag) tx = -torque_mag;
                    if (tz > torque_mag) tz = torque_mag;
                    if (tz < -torque_mag) tz = -torque_mag;

                    if (!manual_pitch) input.torque_cmd_z = tx;
                    if (!manual_yaw) input.torque_cmd = tz;
                    if (!manual_roll) input.torque_cmd_roll = -state.ang_vel_roll * (moi * 5.0);
                }
            }
        }
    }

    if (state.status == ASCEND && state.velocity < 0 && state.altitude > 1000)
        state.status = DESCEND;
}

} // namespace ControlSystem
