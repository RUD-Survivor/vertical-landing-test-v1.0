#include "control_system.h"
#include "physics_system.h"
#include <algorithm>

namespace ControlSystem {

void UpdateAutoPilot(RocketState& state, const RocketConfig& config, ControlInput& input, double dt) {
    if (state.status == PRE_LAUNCH || state.status == LANDED || state.status == CRASHED)
        return;

    // Zero out-of-plane torque
    input.torque_cmd_z = state.pid_att_z.update(0.0, state.angle_z, dt);

    double max_thrust_vac = config.specific_impulse * G0 * config.cosrate;
    double current_total_mass = config.dry_mass + state.fuel;
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

void UpdateManualControl(RocketState& state, ControlInput& input, const ManualInputs& manual, double dt) {
    if (state.status == PRE_LAUNCH || state.status == LANDED || state.status == CRASHED)
        return;

    if (manual.throttle_up) input.throttle = std::min(1.0, input.throttle + 1.5 * dt);
    if (manual.throttle_down) input.throttle = std::max(0.0, input.throttle - 1.5 * dt);
    if (manual.throttle_max) input.throttle = 1.0;
    if (manual.throttle_min) input.throttle = 0.0;

    input.torque_cmd = 0;
    input.torque_cmd_z = 0;
    
    if (manual.pitch_left) input.torque_cmd = 30000.0;
    if (manual.pitch_right) input.torque_cmd = -30000.0;
    
    if (manual.pitch_forward) input.torque_cmd_z = 30000.0;
    if (manual.pitch_backward) input.torque_cmd_z = -30000.0;

    if (state.status == ASCEND && state.velocity < 0 && state.altitude > 1000)
        state.status = DESCEND;
}

} // namespace ControlSystem
