#pragma once
#include "core/universe_model.h"
#include "core/rocket_state.h"
#include "physics/physics_system.h"
#include "math/math3d.h"
#include <cmath>
#include <vector>
#include <algorithm>
#include <iostream>

// ============================================================
// Transfer Window Calculator — Lambert Solver & Porkchop Plot
// ============================================================

struct PorkchopPoint {
    double departure_time;   // sim_time of departure (seconds)
    double tof;              // time of flight (seconds)
    double dv_departure;     // departure delta-v magnitude (m/s)
    double dv_arrival;       // arrival delta-v magnitude (m/s)
    double dv_total;         // total delta-v (m/s)
    Vec3 departure_dv_vec;   // departure delta-v vector (heliocentric)
    bool valid;              // whether Lambert solution converged
};

struct PorkchopResult {
    std::vector<PorkchopPoint> grid;
    int n_dep;               // grid size in departure axis
    int n_tof;               // grid size in TOF axis
    double dep_start;        // departure window start (sim_time seconds)
    double dep_end;          // departure window end
    double tof_min;          // minimum TOF (seconds)
    double tof_max;          // maximum TOF (seconds)
    int min_dv_index;        // index of minimum total delta-v point
    double min_dv;           // minimum total delta-v value (m/s)
    bool computed;
    
    PorkchopResult() : n_dep(0), n_tof(0), dep_start(0), dep_end(0), 
                       tof_min(0), tof_max(0), min_dv_index(-1), 
                       min_dv(1e30), computed(false) {}
};

namespace TransferCalculator {

// Stumpff functions C(z) and S(z) for universal variable Lambert solver
inline double stumpff_C(double z) {
    if (std::abs(z) < 1e-6) return 1.0/2.0;
    if (z > 0) {
        double sz = std::sqrt(z);
        return (1.0 - std::cos(sz)) / z;
    } else {
        double sz = std::sqrt(-z);
        return (std::cosh(sz) - 1.0) / (-z);
    }
}

inline double stumpff_S(double z) {
    if (std::abs(z) < 1e-6) return 1.0/6.0;
    if (z > 0) {
        double sz = std::sqrt(z);
        return (sz - std::sin(sz)) / (sz * sz * sz);
    } else {
        double sz = std::sqrt(-z);
        return (std::sinh(sz) - sz) / (sz * sz * sz);
    }
}

// Lambert solver using universal variable method
// Given r1_vec, r2_vec (position vectors), tof (time of flight), mu (gravitational parameter),
// prograde (true = short way, false = long way)
// Returns v1, v2 (velocity vectors at departure and arrival)
// Returns true if converged
inline bool solveLambert(
    double r1x, double r1y, double r1z,
    double r2x, double r2y, double r2z,
    double tof, double mu, bool prograde,
    double& v1x, double& v1y, double& v1z,
    double& v2x, double& v2y, double& v2z)
{
    double r1 = std::sqrt(r1x*r1x + r1y*r1y + r1z*r1z);
    double r2 = std::sqrt(r2x*r2x + r2y*r2y + r2z*r2z);
    
    if (r1 < 1e3 || r2 < 1e3 || tof < 1.0) return false;
    
    // Cross product to determine transfer direction
    double cross_z = r1x * r2y - r1y * r2x;
    
    // Compute cos(delta_nu)
    double cos_dnu = (r1x*r2x + r1y*r2y + r1z*r2z) / (r1 * r2);
    cos_dnu = std::max(-1.0, std::min(1.0, cos_dnu));
    
    // Determine transfer angle
    double sin_dnu;
    if (prograde) {
        sin_dnu = (cross_z >= 0) ? std::sqrt(1.0 - cos_dnu*cos_dnu) 
                                 : -std::sqrt(1.0 - cos_dnu*cos_dnu);
    } else {
        sin_dnu = (cross_z < 0) ? std::sqrt(1.0 - cos_dnu*cos_dnu) 
                                : -std::sqrt(1.0 - cos_dnu*cos_dnu);
    }
    
    double A = sin_dnu * std::sqrt(r1 * r2 / (1.0 - cos_dnu));
    
    if (std::abs(A) < 1e-10) return false;
    
    // Newton-Raphson iteration on z
    double z = 0.0;  // initial guess (z=0 corresponds to parabolic)
    
    // Better initial guess based on geometry
    {
        double s = (r1 + r2 + std::sqrt((r2x-r1x)*(r2x-r1x) + (r2y-r1y)*(r2y-r1y) + (r2z-r1z)*(r2z-r1z))) / 2.0;
        double a_min = s / 2.0;
        double n_guess = std::sqrt(mu / (a_min * a_min * a_min));
        double t_min_p = PI / n_guess; // minimum energy TOF (approximate)
        if (tof > t_min_p) z = 2.0; // start above parabolic
        else z = -2.0; // hyperbolic side
    }
    
    for (int iter = 0; iter < 50; iter++) {
        double C = stumpff_C(z);
        double S = stumpff_S(z);
        
        double sqrt_mu = std::sqrt(mu);
        
        // y(z) function
        double y = r1 + r2 + A * (z * S - 1.0) / std::sqrt(C);
        
        if (y < 0) {
            // Adjust z to make y positive
            z = z + 0.5;
            continue;
        }
        
        double xi = std::sqrt(y / C);
        double t_z = (xi * xi * xi * S + A * std::sqrt(y)) / sqrt_mu;
        
        // Check convergence
        if (std::abs(t_z - tof) < 1e-4 * tof + 1.0) {
            // Compute Lagrange coefficients
            double f = 1.0 - y / r1;
            double g = A * std::sqrt(y / mu);
            double g_dot = 1.0 - y / r2;
            
            if (std::abs(g) < 1e-20) return false;
            
            v1x = (r2x - f * r1x) / g;
            v1y = (r2y - f * r1y) / g;
            v1z = (r2z - f * r1z) / g;
            
            v2x = (g_dot * r2x - r1x) / g;
            v2y = (g_dot * r2y - r1y) / g;
            v2z = (g_dot * r2z - r1z) / g;
            
            return true;
        }
        
        // Derivative dt/dz for Newton's method
        double dt_dz;
        if (std::abs(z) > 1e-6) {
            dt_dz = (xi*xi*xi * (S - 3.0*S*z/(2.0*C) + 1.0/(2.0*C*std::sqrt(C))) 
                     + (A/8.0) * (3.0*S*std::sqrt(y)/C + A/xi)) / sqrt_mu;
            // Simplified: use finite difference as fallback for robustness
            double eps = 1e-6;
            double C2 = stumpff_C(z + eps);
            double S2 = stumpff_S(z + eps);
            double y2 = r1 + r2 + A * ((z+eps) * S2 - 1.0) / std::sqrt(C2);
            if (y2 > 0) {
                double xi2 = std::sqrt(y2 / C2);
                double t2 = (xi2*xi2*xi2 * S2 + A * std::sqrt(y2)) / sqrt_mu;
                dt_dz = (t2 - t_z) / eps;
            }
        } else {
            // Near z=0, use finite difference
            double eps = 1e-4;
            double C2 = stumpff_C(z + eps);
            double S2 = stumpff_S(z + eps);
            double y2 = r1 + r2 + A * ((z+eps) * S2 - 1.0) / std::sqrt(C2);
            if (y2 < 0) { z += 0.1; continue; }
            double xi2 = std::sqrt(y2 / C2);
            double t2 = (xi2*xi2*xi2 * S2 + A * std::sqrt(y2)) / sqrt_mu;
            dt_dz = (t2 - t_z) / eps;
        }
        
        if (std::abs(dt_dz) < 1e-30) return false;
        
        z = z + (tof - t_z) / dt_dz;
        
        // Clamp z to avoid divergence
        if (z > 200.0) z = 200.0;
        if (z < -200.0) z = -200.0;
    }
    
    return false; // Did not converge
}

// Compute approximate synodic period between two bodies orbiting the Sun
inline double getSynodicPeriod(int origin_idx, int target_idx) {
    if (origin_idx <= 0 || target_idx <= 0) return 365.25 * 86400.0;
    if (origin_idx >= (int)UniverseModel::getInstance().solar_system.size() || target_idx >= (int)UniverseModel::getInstance().solar_system.size()) return 365.25 * 86400.0;
    
    double n1 = UniverseModel::getInstance().solar_system[origin_idx].mean_anom_rate; // rad/sec
    double n2 = UniverseModel::getInstance().solar_system[target_idx].mean_anom_rate;
    
    if (std::abs(n1 - n2) < 1e-20) return 365.25 * 86400.0 * 10.0;
    
    return 2.0 * PI / std::abs(n1 - n2);
}

// Compute Hohmann-like approximate TOF between two bodies
inline double getApproxTransferTOF(int origin_idx, int target_idx) {
    double a1 = UniverseModel::getInstance().solar_system[origin_idx].sma_base;
    double a2 = UniverseModel::getInstance().solar_system[target_idx].sma_base;
    double a_transfer = (a1 + a2) / 2.0;
    double T_transfer = 2.0 * PI * std::sqrt(a_transfer * a_transfer * a_transfer / GM_sun);
    return T_transfer / 2.0; // Half the transfer orbit period
}

// Determine the parent body index for a given body (used for Moon -> Earth hierarchy)
inline int getParentBody(int body_idx) {
    if (body_idx == 4) return 3; // Moon's parent is Earth
    return 0; // All other planets orbit Sun
}

// Get the heliocentric orbit body index for transfer calculations
// For the rocket: if in Earth SOI, the departure body is Earth (3)
// For Moon SOI, departure body is also Earth (3) for interplanetary transfers
inline int getTransferOriginBody() {
    int soi = UniverseModel::getInstance().current_soi_index;
    if (soi == 4) return 3; // Moon -> treat as departing from Earth
    if (soi == 0) return 3; // Sun SOI -> default to Earth
    return soi;
}

// Compute porkchop plot
inline PorkchopResult computePorkchop(int origin_body, int target_body, double current_sim_time,
                                       int grid_size = 40) {
    PorkchopResult result;
    result.n_dep = grid_size;
    result.n_tof = grid_size;
    
    // Compute scan ranges
    double synodic = getSynodicPeriod(origin_body, target_body);
    double hohmann_tof = getApproxTransferTOF(origin_body, target_body);
    
    // Departure window: from now to ~1.2 synodic periods
    result.dep_start = current_sim_time;
    result.dep_end = current_sim_time + synodic * 1.2;
    
    // TOF window: 0.3x to 2.5x Hohmann TOF
    result.tof_min = hohmann_tof * 0.3;
    result.tof_max = hohmann_tof * 2.5;
    
    // Clamp minimum TOF
    if (result.tof_min < 86400.0 * 10.0) result.tof_min = 86400.0 * 10.0;
    
    double dep_step = (result.dep_end - result.dep_start) / (grid_size - 1);
    double tof_step = (result.tof_max - result.tof_min) / (grid_size - 1);
    
    result.grid.resize(grid_size * grid_size);
    result.min_dv = 1e30;
    result.min_dv_index = -1;
    
    double mu = GM_sun;
    // For Moon as target, we still use heliocentric Lambert
    // The Moon case is special - skip it as interplanetary target
    
    for (int i = 0; i < grid_size; i++) {
        double t_dep = result.dep_start + i * dep_step;
        
        // Get origin body state at departure
        double o_px, o_py, o_pz, o_vx, o_vy, o_vz;
        PhysicsSystem::GetCelestialStateAt(origin_body, t_dep, o_px, o_py, o_pz, o_vx, o_vy, o_vz);
        
        for (int j = 0; j < grid_size; j++) {
            double tof = result.tof_min + j * tof_step;
            double t_arr = t_dep + tof;
            int idx = i * grid_size + j;
            
            PorkchopPoint& pt = result.grid[idx];
            pt.departure_time = t_dep;
            pt.tof = tof;
            pt.valid = false;
            pt.dv_total = 1e30;
            
            // Get target body state at arrival
            double t_px, t_py, t_pz, t_vx, t_vy, t_vz;
            PhysicsSystem::GetCelestialStateAt(target_body, t_arr, t_px, t_py, t_pz, t_vx, t_vy, t_vz);
            
            // Solve Lambert problem (prograde)
            double v1x, v1y, v1z, v2x, v2y, v2z;
            bool ok = solveLambert(o_px, o_py, o_pz, t_px, t_py, t_pz,
                                   tof, mu, true, v1x, v1y, v1z, v2x, v2y, v2z);
            
            if (ok) {
                // Departure delta-v: difference between Lambert v1 and planet orbital velocity
                double dv1x = v1x - o_vx;
                double dv1y = v1y - o_vy;
                double dv1z = v1z - o_vz;
                pt.dv_departure = std::sqrt(dv1x*dv1x + dv1y*dv1y + dv1z*dv1z);
                
                // Arrival delta-v
                double dv2x = v2x - t_vx;
                double dv2y = v2y - t_vy;
                double dv2z = v2z - t_vz;
                pt.dv_arrival = std::sqrt(dv2x*dv2x + dv2y*dv2y + dv2z*dv2z);
                
                pt.dv_total = pt.dv_departure + pt.dv_arrival;
                pt.departure_dv_vec = Vec3((float)dv1x, (float)dv1y, (float)dv1z);
                pt.valid = true;
                
                // Reject unreasonable values
                if (pt.dv_total > 100000.0) { // > 100 km/s is unreasonable
                    pt.valid = false;
                    pt.dv_total = 1e30;
                }
                
                if (pt.valid && pt.dv_total < result.min_dv) {
                    result.min_dv = pt.dv_total;
                    result.min_dv_index = idx;
                }
            }
        }
    }
    
    result.computed = true;
    return result;
}

} // namespace TransferCalculator
