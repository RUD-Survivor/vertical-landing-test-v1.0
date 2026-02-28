#include "physics_system.h"
#include <algorithm>

namespace PhysicsSystem {

double get_gravity(double r) { 
    return G0 * pow(EARTH_RADIUS / r, 2); 
}

double get_pressure(double h) {
    if (h > 100000) return 0;
    if (h < 0) return SLP;
    return SLP * std::exp(-h / 7000.0); // Simple exponential atmosphere
}

double get_air_density(double h) {
    if (h > 100000) return 0;
    return 1.225 * std::exp(-h / 7000.0);
}

void getOrbitParams(const RocketState& state, double& apoapsis, double& periapsis) {
    double r = std::sqrt(state.px * state.px + state.py * state.py + state.pz * state.pz);
    double v_sq = state.vx * state.vx + state.vy * state.vy + state.vz * state.vz;
    double mu = G0 * EARTH_RADIUS * EARTH_RADIUS; // Standard gravitational parameter

    double energy = v_sq / 2.0 - mu / r; // Specific orbital energy
    
    // 3D Specific Angular Momentum: h = r x v
    double hx = state.py * state.vz - state.pz * state.vy;
    double hy = state.pz * state.vx - state.px * state.vz;
    double hz = state.px * state.vy - state.py * state.vx;
    double h_sq = hx * hx + hy * hy + hz * hz;

    // Squared eccentricity
    double e_sq = 1.0 + 2.0 * energy * h_sq / (mu * mu);
    double e = (e_sq > 0) ? std::sqrt(e_sq) : 0;

    if (energy >= 0) { // Escape orbit
        apoapsis = 999999999;
        periapsis = (h_sq / mu) / (1.0 + e) - EARTH_RADIUS; // h^2/mu
    } else {           // Closed elliptical orbit
        double a = -mu / (2.0 * energy); // Semi-major axis
        apoapsis = a * (1.0 + e) - EARTH_RADIUS;
        periapsis = a * (1.0 - e) - EARTH_RADIUS;
    }
}

void Update(RocketState& state, const RocketConfig& config, const ControlInput& input, double dt) {
    if (state.status == PRE_LAUNCH) {
        state.px = 0;
        state.py = EARTH_RADIUS;
        state.pz = 0;
        state.vx = 0;
        state.vy = 0;
        state.vz = 0;
        state.altitude = 0;
        return;
    }
    if (state.status == LANDED || state.status == CRASHED)
        return;

    // A. Base State Calculation
    double r = std::sqrt(state.px * state.px + state.py * state.py); // Distance to Earth center horizontally
    state.altitude = r - EARTH_RADIUS;

    // B. Force Analysis
    double total_mass = config.dry_mass + state.fuel;

    // 1. Earth Gravity
    double g_earth = get_gravity(r);
    double Fg_x = -g_earth * (state.px / r) * total_mass;
    double Fg_y = -g_earth * (state.py / r) * total_mass;

    // 1.5 Sun Gravity (Tidal force perturbation)
    state.sim_time += dt;
    
    double sun_angular_vel = std::sqrt(GM_sun / (au_meters * au_meters * au_meters));
    double sun_angle = -1.2 + sun_angular_vel * state.sim_time; 
    
    // Sun position from Earth center
    double sun_px = std::cos(sun_angle) * au_meters;
    double sun_py = std::sin(sun_angle) * au_meters;
    
    // Rocket to Sun vector
    double dx_sun = sun_px - state.px;
    double dy_sun = sun_py - state.py;
    
    double r_sun_rocket_sq = dx_sun*dx_sun + dy_sun*dy_sun;
    double dist_sun_rocket = std::sqrt(r_sun_rocket_sq);
    double r_sun_rocket3 = r_sun_rocket_sq * dist_sun_rocket;
    
    double dist_sun_earth = au_meters;
    double r_sun_earth3 = dist_sun_earth * dist_sun_earth * dist_sun_earth;
    
    // Third body perturbation
    double Fg_sun_x = GM_sun * (dx_sun / r_sun_rocket3 - sun_px / r_sun_earth3) * total_mass;
    double Fg_sun_y = GM_sun * (dy_sun / r_sun_rocket3 - sun_py / r_sun_earth3) * total_mass;
    
    Fg_x += Fg_sun_x;
    Fg_y += Fg_sun_y;

    // 2. Thrust
    state.thrust_power = 0;
    if (state.fuel > 0) {
        double max_thrust = config.specific_impulse * G0 * config.cosrate;
        double pressure_loss = 100 * get_pressure(state.altitude) * config.nozzle_area;
        double current_thrust = input.throttle * max_thrust - pressure_loss;
        if (current_thrust < 0) current_thrust = 0;
        state.thrust_power = current_thrust;

        double m_dot = state.thrust_power / (config.specific_impulse * G0);
        state.fuel -= m_dot * dt;
        state.fuel_consumption_rate = m_dot; 
    } else {
        state.thrust_power = 0;
        state.fuel_consumption_rate = 0;
    }

    // --- 3D Geometric Frame and Thrust Projection ---
    double r_mag = std::sqrt(state.px*state.px + state.py*state.py + state.pz*state.pz);
    double Ux = state.px / r_mag;
    double Uy = state.py / r_mag;
    double Uz = state.pz / r_mag;

    double r_xy_mag = std::sqrt(Ux*Ux + Uy*Uy);
    double Rx = 0, Ry = 0, Rz = 0;
    if (r_xy_mag > 1e-6) {
        Rx = -Uy / r_xy_mag;
        Ry = Ux / r_xy_mag;
        Rz = 0.0;
    } else {
        Rx = 1.0; Ry = 0.0; Rz = 0.0;
    }

    double Nx = Uy * Rz - Uz * Ry;
    double Ny = Uz * Rx - Ux * Rz;
    double Nz = Ux * Ry - Uy * Rx;
    double N_mag = std::sqrt(Nx*Nx + Ny*Ny + Nz*Nz);
    Nx /= N_mag; Ny /= N_mag; Nz /= N_mag;

    double cos_a = std::cos(state.angle);
    double sin_a = std::sin(state.angle);
    double Fx = Ux * cos_a + Rx * sin_a;
    double Fy = Uy * cos_a + Ry * sin_a;
    double Fz = Uz * cos_a + Rz * sin_a;

    double cos_z = std::cos(state.angle_z);
    double sin_z = std::sin(state.angle_z);
    
    double thrust_dir_x = Fx * cos_z + Nx * sin_z;
    double thrust_dir_y = Fy * cos_z + Ny * sin_z;
    double thrust_dir_z = Fz * cos_z + Nz * sin_z;

    double Ft_x = state.thrust_power * thrust_dir_x;
    double Ft_y = state.thrust_power * thrust_dir_y;
    double Ft_z = state.thrust_power * thrust_dir_z;

    // 3. Aerodynamic Drag & Torque
    double v_sq = state.vx * state.vx + state.vy * state.vy + state.vz * state.vz;
    double v_mag = std::sqrt(v_sq);
    double local_up_angle = std::atan2(state.py, state.px);
    double Fd_x = 0, Fd_y = 0;
    double aero_torque = 0;

    if (v_mag > 0.1 && state.altitude < 80000) {
        double rho = get_air_density(state.altitude);

        double base_area = 10.0;
        double side_area = config.height * config.diameter; 
        double effective_area = base_area + side_area * std::abs(std::sin(state.angle_z));

        // Drag (F = 0.5 * rho * v^2 * Cd * A), assuming Cd=0.5
        double drag_mag = 0.5 * rho * v_sq * 0.5 * effective_area;
        Fd_x = -drag_mag * (state.vx / v_mag);
        Fd_y = -drag_mag * (state.vy / v_mag);

        // Structural limits check
        double dynamic_pressure = 0.5 * rho * v_sq;
        if (dynamic_pressure > 50000.0 && std::abs(state.angle_z) > 0.35) { // ~20 degrees
            state.status = CRASHED;
            state.mission_msg = ">> STRUCTURAL FAILURE: HIGH Q OUT-OF-PLANE PITCH!";
            state.vx = 0; state.vy = 0; 
            state.ang_vel = 0; state.ang_vel_z = 0;
            return;
        }

        // Aerodynamic Torques
        aero_torque -= state.ang_vel * 0.1 * v_mag * rho;
    }

    // C. Integration (RK4)
    auto calc_accel = [&](double temp_px, double temp_py, double temp_pz, double temp_vx, double temp_vy, double temp_vz, double& out_ax, double& out_ay, double& out_az) {
        double r_inner = std::sqrt(temp_px * temp_px + temp_py * temp_py + temp_pz * temp_pz);
        double alt = r_inner - EARTH_RADIUS;
        
        // Earth Gravity
        double g_earth_inner = get_gravity(r_inner);
        double Fgx = -g_earth_inner * (temp_px / r_inner) * total_mass;
        double Fgy = -g_earth_inner * (temp_py / r_inner) * total_mass;
        double Fgz = -g_earth_inner * (temp_pz / r_inner) * total_mass;
        
        // Sun Gravity Perturbation
        double dx = sun_px - temp_px;
        double dy = sun_py - temp_py;
        double dz = -temp_pz;
        double r_rocket_sq_inner = dx*dx + dy*dy + dz*dz;
        double dist_rocket = std::sqrt(r_rocket_sq_inner);
        double r_rocket3 = r_rocket_sq_inner * dist_rocket;
        double Fg_sun_x_inner = GM_sun * (dx / r_rocket3 - sun_px / r_sun_earth3) * total_mass;
        double Fg_sun_y_inner = GM_sun * (dy / r_rocket3 - sun_py / r_sun_earth3) * total_mass;
        double Fg_sun_z_inner = GM_sun * (dz / r_rocket3 - 0.0) * total_mass;
        
        // Air Drag
        double Fdx = 0, Fdy = 0, Fdz = 0;
        double temp_v_sq = temp_vx * temp_vx + temp_vy * temp_vy + temp_vz * temp_vz;
        double temp_v_mag = std::sqrt(temp_v_sq);
        if (temp_v_mag > 0.1 && alt < 80000) {
            double rho = get_air_density(alt);
            double base_area = 10.0;
            double side_area = config.height * config.diameter;
            double effective_area = base_area + side_area * std::abs(std::sin(state.angle_z));
            double drag_mag = 0.5 * rho * temp_v_sq * 0.5 * effective_area;
            Fdx = -drag_mag * (temp_vx / temp_v_mag);
            Fdy = -drag_mag * (temp_vy / temp_v_mag);
            Fdz = -drag_mag * (temp_vz / temp_v_mag);
        }
        
        out_ax = (Fgx + Fg_sun_x_inner + Ft_x + Fdx) / total_mass;
        out_ay = (Fgy + Fg_sun_y_inner + Ft_y + Fdy) / total_mass;
        out_az = (Fgz + Fg_sun_z_inner + Ft_z + Fdz) / total_mass;
    };

    double k1_vx, k1_vy, k1_vz, k1_px, k1_py, k1_pz;
    calc_accel(state.px, state.py, state.pz, state.vx, state.vy, state.vz, k1_vx, k1_vy, k1_vz);
    k1_px = state.vx; k1_py = state.vy; k1_pz = state.vz;

    double k2_vx, k2_vy, k2_vz, k2_px, k2_py, k2_pz;
    calc_accel(state.px + 0.5 * dt * k1_px, state.py + 0.5 * dt * k1_py, state.pz + 0.5 * dt * k1_pz, 
               state.vx + 0.5 * dt * k1_vx, state.vy + 0.5 * dt * k1_vy, state.vz + 0.5 * dt * k1_vz, 
               k2_vx, k2_vy, k2_vz);
    k2_px = state.vx + 0.5 * dt * k1_vx; k2_py = state.vy + 0.5 * dt * k1_vy; k2_pz = state.vz + 0.5 * dt * k1_vz;

    double k3_vx, k3_vy, k3_vz, k3_px, k3_py, k3_pz;
    calc_accel(state.px + 0.5 * dt * k2_px, state.py + 0.5 * dt * k2_py, state.pz + 0.5 * dt * k2_pz, 
               state.vx + 0.5 * dt * k2_vx, state.vy + 0.5 * dt * k2_vy, state.vz + 0.5 * dt * k2_vz, 
               k3_vx, k3_vy, k3_vz);
    k3_px = state.vx + 0.5 * dt * k2_vx; k3_py = state.vy + 0.5 * dt * k2_vy; k3_pz = state.vz + 0.5 * dt * k2_vz;

    double k4_vx, k4_vy, k4_vz, k4_px, k4_py, k4_pz;
    calc_accel(state.px + dt * k3_px, state.py + dt * k3_py, state.pz + dt * k3_pz, 
               state.vx + dt * k3_vx, state.vy + dt * k3_vy, state.vz + dt * k3_vz, 
               k4_vx, k4_vy, k4_vz);
    k4_px = state.vx + dt * k3_vx; k4_py = state.vy + dt * k3_vy; k4_pz = state.vz + dt * k3_vz;

    state.vx += (dt / 6.0) * (k1_vx + 2.0 * k2_vx + 2.0 * k3_vx + k4_vx);
    state.vy += (dt / 6.0) * (k1_vy + 2.0 * k2_vy + 2.0 * k3_vy + k4_vy);
    state.vz += (dt / 6.0) * (k1_vz + 2.0 * k2_vz + 2.0 * k3_vz + k4_vz);
    
    state.px += (dt / 6.0) * (k1_px + 2.0 * k2_px + 2.0 * k3_px + k4_px);
    state.py += (dt / 6.0) * (k1_py + 2.0 * k2_py + 2.0 * k3_py + k4_py);
    state.pz += (dt / 6.0) * (k1_pz + 2.0 * k2_pz + 2.0 * k3_pz + k4_pz);
    
    // Update displayed acceleration
    double final_ax, final_ay, final_az;
    calc_accel(state.px, state.py, state.pz, state.vx, state.vy, state.vz, final_ax, final_ay, final_az);
    state.acceleration = std::sqrt(final_ax * final_ax + final_ay * final_ay + final_az * final_az);

    // Derived velocities
    state.velocity = state.vx * std::cos(local_up_angle) + state.vy * std::sin(local_up_angle);
    state.local_vx = -state.vx * std::sin(local_up_angle) + state.vy * std::cos(local_up_angle);

    // E. Collision Detection
    double current_r = std::sqrt(state.px * state.px + state.py * state.py + state.pz * state.pz);
    double current_alt = current_r - EARTH_RADIUS;

    if (current_alt <= 0.0) {
        if (state.status == ASCEND) {
            state.px = 0; state.py = EARTH_RADIUS;
            state.vx = 0; state.vy = 0;
            state.altitude = 0;
        } else if (state.velocity < 0.1) {
            state.altitude = 0;
            state.px = 0; state.py = EARTH_RADIUS;

            if (state.status != PRE_LAUNCH) {
                if (std::abs(state.velocity) > 10 || std::abs(state.local_vx) > 10) {
                    state.status = CRASHED;
                } else {
                    state.status = LANDED;
                }
                state.vx = 0; state.vy = 0;
                state.ang_vel = 0; state.ang_vel_z = 0;
                state.angle = 0; state.angle_z = 0;
                state.suicide_burn_locked = false; 
            }
        } else {
            state.altitude = current_alt;
        }
    } else {
        state.altitude = current_alt;
    }

    // Angular Motion
    double moment_of_inertia = 50000.0;
    
    double final_torque = input.torque_cmd;
    double ang_accel = (final_torque + aero_torque) / moment_of_inertia;
    state.ang_vel += ang_accel * dt;
    state.angle += state.ang_vel * dt;

    // Z Axis Out of plane Pitch
    double rho_z = (state.altitude < 80000) ? get_air_density(state.altitude) : 0.0;
    double aero_torque_z = -state.ang_vel_z * 0.1 * v_mag * rho_z; 
    double ang_accel_z = (input.torque_cmd_z + aero_torque_z) / moment_of_inertia;
    state.ang_vel_z += ang_accel_z * dt;

    if (state.altitude > 80000) {
        state.ang_vel_z *= std::pow(0.95, dt);
    }
    state.angle_z += state.ang_vel_z * dt;

    while (state.angle_z > PI) state.angle_z -= 2 * PI;
    while (state.angle_z < -PI) state.angle_z += 2 * PI;
}

void FastGravityUpdate(RocketState& state, const RocketConfig& config, double dt_total) {
    if (state.status == PRE_LAUNCH || state.status == LANDED || state.status == CRASHED) return;

    double dt_step = 5.0; 
    double t_remaining = dt_total;
    
    double total_mass = config.dry_mass + state.fuel;
    double sun_angular_vel = std::sqrt(GM_sun / (au_meters * au_meters * au_meters));
    double mu_earth = G0 * std::pow(EARTH_RADIUS, 2);

    while (t_remaining > 0) {
        double dt = std::min(t_remaining, dt_step);
        t_remaining -= dt;
        state.sim_time += dt;
      
        auto calc_accel = [&](double temp_px, double temp_py, double temp_pz, double temp_time, double& out_ax, double& out_ay, double& out_az) {
            double r2 = temp_px * temp_px + temp_py * temp_py + temp_pz * temp_pz;
            double r = std::sqrt(r2);
            
            double g_earth = mu_earth / r2;
            double Fgx = -g_earth * (temp_px / r) * total_mass;
            double Fgy = -g_earth * (temp_py / r) * total_mass;
            double Fgz = -g_earth * (temp_pz / r) * total_mass;
            
            double sun_angle = -1.2 + sun_angular_vel * temp_time; 
            double sun_px = std::cos(sun_angle) * au_meters;
            double sun_py = std::sin(sun_angle) * au_meters;
            
            double dx = sun_px - temp_px;
            double dy = sun_py - temp_py;
            double dz = -temp_pz;
            double r_rocket_sq = dx*dx + dy*dy + dz*dz;
            double dist_rocket = std::sqrt(r_rocket_sq);
            double r_rocket3 = r_rocket_sq * dist_rocket;
            double r_sun_earth3 = au_meters * au_meters * au_meters;
            
            double Fg_sun_x = GM_sun * (dx / r_rocket3 - sun_px / r_sun_earth3) * total_mass;
            double Fg_sun_y = GM_sun * (dy / r_rocket3 - sun_py / r_sun_earth3) * total_mass;
            double Fg_sun_z = GM_sun * (dz / r_rocket3 - 0.0) * total_mass;
            
            out_ax = (Fgx + Fg_sun_x) / total_mass;
            out_ay = (Fgy + Fg_sun_y) / total_mass;
            out_az = (Fgz + Fg_sun_z) / total_mass;
        };

        double k1_vx, k1_vy, k1_vz, k1_px, k1_py, k1_pz;
        calc_accel(state.px, state.py, state.pz, state.sim_time - dt, k1_vx, k1_vy, k1_vz);
        k1_px = state.vx; k1_py = state.vy; k1_pz = state.vz;

        double k2_vx, k2_vy, k2_vz, k2_px, k2_py, k2_pz;
        calc_accel(state.px + 0.5 * dt * k1_px, state.py + 0.5 * dt * k1_py, state.pz + 0.5 * dt * k1_pz, state.sim_time - dt + 0.5 * dt, k2_vx, k2_vy, k2_vz);
        k2_px = state.vx + 0.5 * dt * k1_vx; k2_py = state.vy + 0.5 * dt * k1_vy; k2_pz = state.vz + 0.5 * dt * k1_vz;

        double k3_vx, k3_vy, k3_vz, k3_px, k3_py, k3_pz;
        calc_accel(state.px + 0.5 * dt * k2_px, state.py + 0.5 * dt * k2_py, state.pz + 0.5 * dt * k2_pz, state.sim_time - dt + 0.5 * dt, k3_vx, k3_vy, k3_vz);
        k3_px = state.vx + 0.5 * dt * k2_vx; k3_py = state.vy + 0.5 * dt * k2_vy; k3_pz = state.vz + 0.5 * dt * k2_vz;

        double k4_vx, k4_vy, k4_vz, k4_px, k4_py, k4_pz;
        calc_accel(state.px + dt * k3_px, state.py + dt * k3_py, state.pz + dt * k3_pz, state.sim_time, k4_vx, k4_vy, k4_vz);
        k4_px = state.vx + dt * k3_vx; k4_py = state.vy + dt * k3_vy; k4_pz = state.vz + dt * k3_vz;

        state.vx += (dt / 6.0) * (k1_vx + 2.0 * k2_vx + 2.0 * k3_vx + k4_vx);
        state.vy += (dt / 6.0) * (k1_vy + 2.0 * k2_vy + 2.0 * k3_vy + k4_vy);
        state.vz += (dt / 6.0) * (k1_vz + 2.0 * k2_vz + 2.0 * k3_vz + k4_vz);
        state.px += (dt / 6.0) * (k1_px + 2.0 * k2_px + 2.0 * k3_px + k4_px);
        state.py += (dt / 6.0) * (k1_py + 2.0 * k2_py + 2.0 * k3_py + k4_py);
        state.pz += (dt / 6.0) * (k1_pz + 2.0 * k2_pz + 2.0 * k3_pz + k4_pz);
        
        state.altitude = std::sqrt(state.px*state.px + state.py*state.py + state.pz*state.pz) - EARTH_RADIUS;
        
        if (state.altitude <= 0.0) {
            state.altitude = 0;
            state.status = CRASHED;
            break;
        }
    }
}

void EmitSmoke(RocketState& state, const RocketConfig& config, double dt) {
    if (state.thrust_power < 1000.0) return;
    double local_up = std::atan2(state.py, state.px);
    double nozzle_dir = local_up + state.angle + PI;
    double nozzle_wx = state.px + std::cos(nozzle_dir) * 20.0;
    double nozzle_wy = state.py + std::sin(nozzle_dir) * 20.0;
    
    for (int k = 0; k < 3; k++) {
        SmokeParticle& p = state.smoke[state.smoke_idx % RocketState::MAX_SMOKE];
        float rnd1 = hash11(state.smoke_idx * 1337 + k * 997) - 0.5f;
        float rnd2 = hash11(state.smoke_idx * 7919 + k * 773) - 0.5f;
        p.wx = nozzle_wx + rnd1 * 15.0;
        p.wy = nozzle_wy + rnd2 * 15.0;
        
        double exhaust_speed = 30.0 + hash11(state.smoke_idx * 3571 + k) * 20.0;
        p.vwx = std::cos(nozzle_dir) * exhaust_speed + rnd1 * 10.0;
        p.vwy = std::sin(nozzle_dir) * exhaust_speed + rnd2 * 10.0;
        p.alpha = 0.6f;
        p.size = 10.0f + hash11(state.smoke_idx * 4567 + k) * 8.0f;
        p.life = 1.0f;
        p.active = true;
        state.smoke_idx++;
    }
}

void UpdateSmoke(RocketState& state, double dt) {
    for (int i = 0; i < RocketState::MAX_SMOKE; i++) {
        SmokeParticle& p = state.smoke[i];
        if (!p.active) continue;
        
        p.life -= (float)(dt * 0.25);
        p.alpha = std::min(0.25f, p.life * 0.3f); 
        p.size += (float)(dt * 20.0);

        p.wx += p.vwx * dt;
        p.wy += p.vwy * dt;

        double r = std::sqrt(p.wx * p.wx + p.wy * p.wy);
        if (r < EARTH_RADIUS && r > 0) {
            p.wx = p.wx / r * EARTH_RADIUS;
            p.wy = p.wy / r * EARTH_RADIUS;
            double nx = p.wx / r, ny = p.wy / r;
            double v_radial = p.vwx * nx + p.vwy * ny;
            double v_tang = -p.vwx * ny + p.vwy * nx;
            v_radial = std::abs(v_radial) * 0.3;
            float rnd_dir = (hash11(i * 8731) - 0.5f) * 2.0f;
            v_tang = std::abs(v_tang) * (1.5f + rnd_dir) * (rnd_dir > 0 ? 1.0 : -1.0);
            p.vwx = nx * v_radial - ny * v_tang;
            p.vwy = ny * v_radial + nx * v_tang;
            p.size += 5.0f;
        }

        if (r > 0) {
            p.vwx += (p.wx / r) * dt * 8.0;
            p.vwy += (p.wy / r) * dt * 8.0;
        }
        p.vwx *= (1.0 - dt * 0.8); 
        p.vwy *= (1.0 - dt * 0.8);

        if (p.life <= 0) p.active = false;
    }
}

} // namespace PhysicsSystem
