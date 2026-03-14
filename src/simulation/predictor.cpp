#include "predictor.h"
#include "physics/physics_system.h"
#include "simulation/maneuver_system.h"
#include <algorithm>
#include <cmath>

namespace Simulation {

AsyncOrbitPredictor::AsyncOrbitPredictor() : m_running(false), m_busy(false) {}

AsyncOrbitPredictor::~AsyncOrbitPredictor() {
    Stop();
}

void AsyncOrbitPredictor::Start() {
    if (m_running) return;
    m_running = true;
    m_worker = std::thread(&AsyncOrbitPredictor::WorkerLoop, this);
}

void AsyncOrbitPredictor::Stop() {
    m_running = false;
    m_cv.notify_all();
    if (m_worker.joinable()) {
        m_worker.join();
    }
}

void AsyncOrbitPredictor::RequestUpdate(RocketState* target, const RocketState& state, const RocketConfig& config, double pred_days, int iters, int ref_mode, int ref_body, int secondary_ref_body, bool force_reset) {
    if (!target) return;
    std::lock_guard<std::mutex> lock(m_request_mutex);
    
    // Auto reset if reference frame changed
    if (m_request.ref_body != ref_body || m_request.ref_mode != ref_mode || m_request.secondary_ref_body != secondary_ref_body) force_reset = true;
    
    m_request.target = target;
    m_request.state = state;
    m_request.config = config;
    m_request.pred_days = pred_days;
    m_request.iters = iters;
    m_request.ref_mode = ref_mode;
    m_request.ref_body = ref_body;
    m_request.secondary_ref_body = secondary_ref_body;
    m_request.celestial_snapshot = SOLAR_SYSTEM;
    m_request.force_reset = force_reset;
    m_request.pending = true;
    
    m_cv.notify_one();
}

void AsyncOrbitPredictor::WorkerLoop() {
    while (m_running) {
        PredictionRequest req;
        {
            std::unique_lock<std::mutex> lock(m_request_mutex);
            m_cv.wait(lock, [this] { return !m_running || m_request.pending; });
            if (!m_running) break;
            req = std::move(m_request);
            m_request.pending = false;
        }

        m_busy = true;

        // Progressive Logic: Should we clear the context?
        double t_start = req.state.sim_time;
        bool reset_needed = req.force_reset || (m_context.t_last < 0);
        
        // Deviation Check: If t_start has moved backwards or far forward, or if the initial state changed
        if (!reset_needed) {
            // Check if the current request's rocket state corresponds to our starting epoch
            // For industrial grade: we'd ideally simulate from epoch to t_start and compare.
            // Simpler: if sim_time is within a small window of where we expect, or if throttle was 0.
            if (t_start < m_context.t_epoch || t_start > m_context.t_last + 3600.0) {
                reset_needed = true;
            } else if (req.state.maneuvers.size() != m_context.last_maneuvers.size()) {
                reset_needed = true;
            } else if (!req.state.maneuvers.empty()) {
                auto& curr_m = req.state.maneuvers[0];
                auto& last_m = m_context.last_maneuvers[0];
                if (std::abs(curr_m.sim_time - last_m.sim_time) > 1.0 ||
                    std::abs(curr_m.delta_v.x - last_m.delta_v.x) > 0.1 ||
                    std::abs(curr_m.delta_v.y - last_m.delta_v.y) > 0.1 ||
                    std::abs(curr_m.delta_v.z - last_m.delta_v.z) > 0.1) {
                    reset_needed = true;
                }
            }
        }

        bool has_mnv = !req.state.maneuvers.empty();
        
        bool freeze_mnv = false;
        double trigger_t = has_mnv ? req.state.maneuvers[0].sim_time : -1.0;
        
        if (has_mnv && reset_needed && m_context.last_ref_body == req.ref_body) {
            if (m_context.last_maneuvers.size() == req.state.maneuvers.size()) {
                auto& curr_m = req.state.maneuvers[0];
                auto& last_m = m_context.last_maneuvers[0];
                // Check if the maneuver node itself hasn't been drastically altered
                if (std::abs(curr_m.sim_time - last_m.sim_time) < 1.0 &&
                    std::abs(curr_m.delta_v.x - last_m.delta_v.x) < 0.1 &&
                    std::abs(curr_m.delta_v.y - last_m.delta_v.y) < 0.1 &&
                    std::abs(curr_m.delta_v.z - last_m.delta_v.z) < 0.1) {
                    
                    // If we are arriving at or executing the maneuver (within 300 seconds of node time)
                    // We freeze the dashed trajectory so it remains a fixed "Flight Plan".
                    if (t_start > trigger_t - 300.0) {
                        freeze_mnv = true;
                    }
                }
            }
        }
        
        if (reset_needed) {
            m_context.t_epoch = t_start;
            m_context.t_last = t_start;
            m_context.init_px = req.state.abs_px; m_context.init_py = req.state.abs_py; m_context.init_pz = req.state.abs_pz;
            m_context.init_vx = req.state.abs_vx; m_context.init_vy = req.state.abs_vy; m_context.init_vz = req.state.abs_vz;
            m_context.px = req.state.abs_px; m_context.py = req.state.abs_py; m_context.pz = req.state.abs_pz;
            m_context.vx = req.state.abs_vx; m_context.vy = req.state.abs_vy; m_context.vz = req.state.abs_vz;
            m_context.points.clear();
            m_context.point_times.clear();
            m_context.crashed = false;
            m_context.mnv_crashed = false;
            
            if (!freeze_mnv) {
                m_context.mnv_points.clear();
                m_context.mnv_point_times.clear();
                m_context.mnv_done = false;
                m_context.mnv_burning = false;
                m_context.mnv_px = req.state.abs_px; m_context.mnv_py = req.state.abs_py; m_context.mnv_pz = req.state.abs_pz;
                m_context.mnv_vx = req.state.abs_vx; m_context.mnv_vy = req.state.abs_vy; m_context.mnv_vz = req.state.abs_vz;
            }
            m_context.mnv_remaining_dv = 0;
            
            m_context.apsides.clear();
            m_context.mnv_apsides.clear();
            m_context.last_r = -1.0;
            m_context.last_dr = 0.0;
            m_context.last_mnv_r = -1.0;
            m_context.last_mnv_dr = 0.0;
            
            m_context.last_maneuvers = req.state.maneuvers;
            m_context.last_ref_body = req.ref_body;
        } else {
            // PRUNING: Remove points older than current t_start to avoid memory/lag ballooning
            auto it = std::lower_bound(m_context.point_times.begin(), m_context.point_times.end(), t_start - 30.0);
            size_t idx = std::distance(m_context.point_times.begin(), it);
            if (idx > 0) {
                m_context.points.erase(m_context.points.begin(), m_context.points.begin() + idx);
                m_context.point_times.erase(m_context.point_times.begin(), it);
            }
            
            auto it_m = std::lower_bound(m_context.mnv_point_times.begin(), m_context.mnv_point_times.end(), t_start - 30.0);
            size_t idx_m = std::distance(m_context.mnv_point_times.begin(), it_m);
            if (idx_m > 0) {
                m_context.mnv_points.erase(m_context.mnv_points.begin(), m_context.mnv_points.begin() + idx_m);
                m_context.mnv_point_times.erase(m_context.mnv_point_times.begin(), it_m);
            }
            // Prune apsides as well
            m_context.apsides.erase(std::remove_if(m_context.apsides.begin(), m_context.apsides.end(), 
                                                   [t_start](const RocketState::Apsis& a) { return a.sim_time < t_start - 30.0; }), m_context.apsides.end());
            m_context.mnv_apsides.erase(std::remove_if(m_context.mnv_apsides.begin(), m_context.mnv_apsides.end(), 
                                                       [t_start](const RocketState::Apsis& a) { return a.sim_time < t_start - 30.0; }), m_context.mnv_apsides.end());
        }

        // Integration Constants
        const double FR_theta = 1.0 / (2.0 - std::cbrt(2.0));
        const double c1 = FR_theta / 2.0, c2 = (1.0 - FR_theta) / 2.0;
        const double d1 = FR_theta, d2 = 1.0 - 2.0 * FR_theta;

        double t_sim = m_context.t_last;
        double max_pred_time = t_start + req.pred_days * 86400.0;
        
        double cur_h_px = m_context.px, cur_h_py = m_context.py, cur_h_pz = m_context.pz;
        double cur_h_vx = m_context.vx, cur_h_vy = m_context.vy, cur_h_vz = m_context.vz;

        // Snapshot of post-maneuver state for dashed line
        double mnv_h_px = m_context.mnv_px, mnv_h_py = m_context.mnv_py, mnv_h_pz = m_context.mnv_pz;
        double mnv_h_vx = m_context.mnv_vx, mnv_h_vy = m_context.mnv_vy, mnv_h_vz = m_context.mnv_vz;

        bool loop_has_mnv = has_mnv;
        if (freeze_mnv) {
            loop_has_mnv = false; // Disable new maneuver line integration to preserve the frozen flight plan orbit
        }
        
        auto calc_acc = [&](double t, double x, double y, double z, double& ax, double& ay, double& az) {
            ax = 0; ay = 0; az = 0;
            if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) return;

            for (int b_idx = 0; b_idx < (int)req.celestial_snapshot.size(); ++b_idx) {
                const auto& body = req.celestial_snapshot[b_idx];
                double bpx, bpy, bpz;
                PhysicsSystem::GetCelestialPositionAt(b_idx, t, bpx, bpy, bpz);
                double dx = bpx - x, dy = bpy - y, dz = bpz - z;
                double r2 = dx*dx + dy*dy + dz*dz;
                double r = std::sqrt(r2);
                if (r > body.radius * 0.1) { // Basic singularity protection
                    double f = (6.67430e-11 * body.mass) / (r2 * r);
                    ax += dx * f; ay += dy * f; az += dz * f;
                }
            }
        };

        double record_interval = 600.0; // 10-minute resolution
        double last_record_t = t_sim;

        for (int step = 0; step < req.iters; ++step) {
            if (!m_running || m_request.pending) break;
            if (t_sim >= max_pred_time) break;

            // Adaptive step
            double min_dist_sq = 1e30;
            int nearest_body = 0;
            for (int b=0; b<(int)req.celestial_snapshot.size(); b++) {
                double bpx, bpy, bpz;
                PhysicsSystem::GetCelestialPositionAt(b, t_sim, bpx, bpy, bpz);
                double dx = cur_h_px - bpx, dy = cur_h_py - bpy, dz = cur_h_pz - bpz;
                double dsq = dx*dx + dy*dy + dz*dz;
                if (dsq < min_dist_sq) { min_dist_sq = dsq; nearest_body = b; }
            }
            double v_sq = cur_h_vx*cur_h_vx + cur_h_vy*cur_h_vy + cur_h_vz*cur_h_vz;
            double step_dt = 0.1 * std::sqrt(min_dist_sq / std::max(v_sq, 1.0));
            step_dt = std::clamp(step_dt, 5.0, 14400.0);

            if (loop_has_mnv && !m_context.mnv_done) {
                if (t_sim < trigger_t && t_sim + step_dt > trigger_t) {
                    step_dt = trigger_t - t_sim;
                }
            }
            if (t_sim + step_dt > max_pred_time) {
                step_dt = max_pred_time - t_sim;
            }

            if (step_dt <= 0) {
                if (t_sim >= max_pred_time - 1e-4) break;
                step_dt = 1e-5;
            }

            // Maneuver handling
            if (loop_has_mnv && !m_context.mnv_done && t_sim >= trigger_t - 1e-4) {
                const auto& node = req.state.maneuvers[0];
                double bpx, bpy, bpz, bvx, bvy, bvz;
                PhysicsSystem::GetCelestialStateAt(nearest_body, t_sim, bpx, bpy, bpz, bvx, bvy, bvz);
                Vec3 r_rel((float)(cur_h_px - bpx), (float)(cur_h_py - bpy), (float)(cur_h_pz - bpz));
                Vec3 v_rel((float)(cur_h_vx - bvx), (float)(cur_h_vy - bvy), (float)(cur_h_vz - bvz));
                ManeuverFrame frame = ManeuverSystem::getFrame(r_rel, v_rel);
                Vec3 dv_dir = frame.prograde * node.delta_v.x + 
                              frame.normal   * node.delta_v.y + 
                              frame.radial   * node.delta_v.z;
                
                if (node.burn_mode == 0) {
                    // === IMPULSE MODE: instant delta-v ===
                    mnv_h_px = cur_h_px; mnv_h_py = cur_h_py; mnv_h_pz = cur_h_pz;
                    mnv_h_vx = cur_h_vx + dv_dir.x; mnv_h_vy = cur_h_vy + dv_dir.y; mnv_h_vz = cur_h_vz + dv_dir.z;
                    m_context.mnv_done = true;
                } else {
                    // === SUSTAINED/FINITE BURN MODE ===
                    if (!m_context.mnv_burning) {
                        // Initialize burn state
                        m_context.mnv_burning = true;
                        m_context.mnv_remaining_dv = dv_dir.length();
                        m_context.mnv_mass = req.state.fuel + req.config.dry_mass + req.config.upper_stages_mass;
                        // Fork the mnv trajectory from current state
                        mnv_h_px = cur_h_px; mnv_h_py = cur_h_py; mnv_h_pz = cur_h_pz;
                        mnv_h_vx = cur_h_vx; mnv_h_vy = cur_h_vy; mnv_h_vz = cur_h_vz;
                    }
                }
                if (step_dt < 1e-5 && !m_context.mnv_burning) continue;
            }
            
            // Finite burn thrust application (on the mnv trajectory)
            if (m_context.mnv_burning && !m_context.mnv_done) {
                // Clamp step for burn accuracy
                step_dt = std::min(step_dt, 10.0);
                
                // Compute thrust parameters
                double thrust = 0;
                if (req.state.current_stage < (int)req.config.stage_configs.size())
                    thrust = req.config.stage_configs[req.state.current_stage].thrust;
                double ve = req.config.specific_impulse * 9.80665;
                double mdot = (ve > 0) ? thrust / ve : 0;
                
                if (thrust > 0 && m_context.mnv_mass > req.config.dry_mass && m_context.mnv_remaining_dv > 0.1) {
                    // Dynamic prograde attitude guidance: thrust along current velocity direction
                    double vx_rel = mnv_h_vx, vy_rel = mnv_h_vy, vz_rel = mnv_h_vz;
                    // Subtract nearest body velocity for local prograde
                    double nbpx, nbpy, nbpz, nbvx, nbvy, nbvz;
                    PhysicsSystem::GetCelestialStateAt(nearest_body, t_sim, nbpx, nbpy, nbpz, nbvx, nbvy, nbvz);
                    vx_rel -= nbvx; vy_rel -= nbvy; vz_rel -= nbvz;
                    double v_mag = std::sqrt(vx_rel*vx_rel + vy_rel*vy_rel + vz_rel*vz_rel);
                    
                    // Use maneuver delta_v direction for thrust orientation
                    const auto& node = req.state.maneuvers[0];
                    Vec3 r_now((float)(mnv_h_px - nbpx), (float)(mnv_h_py - nbpy), (float)(mnv_h_pz - nbpz));
                    Vec3 v_now((float)vx_rel, (float)vy_rel, (float)vz_rel);
                    ManeuverFrame frame_now = ManeuverSystem::getFrame(r_now, v_now);
                    Vec3 thrust_dir = (frame_now.prograde * node.delta_v.x + 
                                       frame_now.normal   * node.delta_v.y + 
                                       frame_now.radial   * node.delta_v.z).normalized();
                    
                    // Apply thrust acceleration
                    double accel = thrust / m_context.mnv_mass;
                    double dv_this_step = accel * step_dt;
                    if (dv_this_step > m_context.mnv_remaining_dv) {
                        // Don't overshoot - adjust step
                        dv_this_step = m_context.mnv_remaining_dv;
                        step_dt = dv_this_step / accel;
                    }
                    
                    mnv_h_vx += thrust_dir.x * dv_this_step;
                    mnv_h_vy += thrust_dir.y * dv_this_step;
                    mnv_h_vz += thrust_dir.z * dv_this_step;
                    
                    // Mass decay
                    m_context.mnv_mass -= mdot * step_dt;
                    m_context.mnv_remaining_dv -= dv_this_step;
                    
                    // Record during burn at higher frequency
                    record_interval = 30.0;
                } else {
                    // Burn complete
                    m_context.mnv_burning = false;
                    m_context.mnv_done = true;
                    record_interval = 600.0;
                }
            }

            // Integrator
            #define SYM_W_Q(C) { \
                if (!m_context.crashed) { cur_h_px += (C)*cur_h_vx*step_dt; cur_h_py += (C)*cur_h_vy*step_dt; cur_h_pz += (C)*cur_h_vz*step_dt; } \
                if (m_context.mnv_done && !m_context.mnv_crashed) { mnv_h_px+=(C)*mnv_h_vx*step_dt; mnv_h_py+=(C)*mnv_h_vy*step_dt; mnv_h_pz+=(C)*mnv_h_vz*step_dt; } \
                t_sim += (C)*step_dt; \
            }
            #define SYM_W_P(D) { \
                if (!m_context.crashed) { \
                    double ax,ay,az; calc_acc(t_sim, cur_h_px,cur_h_py,cur_h_pz, ax,ay,az); \
                    cur_h_vx+=(D)*ax*step_dt; cur_h_vy+=(D)*ay*step_dt; cur_h_vz+=(D)*az*step_dt; \
                } \
                if (m_context.mnv_done && !m_context.mnv_crashed) { \
                    double max,may,maz; calc_acc(t_sim, mnv_h_px,mnv_h_py,mnv_h_pz, max,may,maz); \
                    mnv_h_vx+=(D)*max*step_dt; mnv_h_vy+=(D)*may*step_dt; mnv_h_vz+=(D)*maz*step_dt; \
                } \
            }
            SYM_W_Q(c1); SYM_W_P(d1);
            SYM_W_Q(c2); SYM_W_P(d2);
            SYM_W_Q(c2); SYM_W_P(d1);
            SYM_W_Q(c1);
            #undef SYM_W_Q
            #undef SYM_W_P

            // Collision Detection
            if (!m_context.crashed || (m_context.mnv_done && !m_context.mnv_crashed)) {
                for (int b=0; b<(int)req.celestial_snapshot.size(); b++) {
                    const auto& body = req.celestial_snapshot[b];
                    double bpx, bpy, bpz; PhysicsSystem::GetCelestialPositionAt(b, t_sim, bpx, bpy, bpz);
                    if (!m_context.crashed) {
                        double dx = cur_h_px - bpx, dy = cur_h_py - bpy, dz = cur_h_pz - bpz;
                        if (dx*dx+dy*dy+dz*dz < body.radius*body.radius) m_context.crashed = true;
                    }
                    if (m_context.mnv_done && !m_context.mnv_crashed) {
                        double mdx = mnv_h_px - bpx, mdy = mnv_h_py - bpy, mdz = mnv_h_pz - bpz;
                        if (mdx*mdx+mdy*mdy+mdz*mdz < body.radius*body.radius) m_context.mnv_crashed = true;
                    }
                }
            }

            if (m_context.crashed && (!m_context.mnv_done || m_context.mnv_crashed)) break;

            // ADAPTIVE RECORDING: Much higher resolution near bodies
            double rb_dist = std::sqrt(min_dist_sq);
            double v_mag = std::sqrt(v_sq);
            record_interval = std::clamp(rb_dist / (v_mag + 1.0) * 0.05, 10.0, 1200.0);
            if (m_context.mnv_burning) record_interval = 10.0;

            // Recording
            if (t_sim >= last_record_t + record_interval || (loop_has_mnv && std::abs(t_sim - trigger_t) < 1e-4)) {
                double rbpx, rbpy, rbpz;
                PhysicsSystem::GetCelestialPositionAt(req.ref_body, t_sim, rbpx, rbpy, rbpz);
                
                Quat q_inv = PhysicsSystem::GetFrameRotation(req.ref_mode, req.ref_body, req.secondary_ref_body, t_sim).conjugate();
                
                if (!m_context.crashed) {
                    Vec3 p_inertial((float)(cur_h_px - rbpx), (float)(cur_h_py - rbpy), (float)(cur_h_pz - rbpz));
                    Vec3 p_local = q_inv.rotate(p_inertial);
                    if (std::isfinite(p_local.x)) {
                        m_context.points.push_back(p_local);
                        m_context.point_times.push_back(t_sim);
                    }
                    
                    // Apsis detection for primary path
                    double double_r = p_inertial.length();
                    double rbx, rby, rbz, rbvx, rbvy, rbvz;
                    PhysicsSystem::GetCelestialStateAt(req.ref_body, t_sim, rbx, rby, rbz, rbvx, rbvy, rbvz);
                    double drx = cur_h_vx - rbvx, dry = cur_h_vy - rbvy, drz = cur_h_vz - rbvz;
                    double dr = (p_inertial.x * drx + p_inertial.y * dry + p_inertial.z * drz); // dot product tells us if distance is expanding or contracting
                    
                    if (m_context.last_r > 0) {
                        if (m_context.last_dr < 0 && dr >= 0) {
                            if (m_context.apsides.size() < 10) m_context.apsides.push_back({false, p_local, t_sim, double_r - req.celestial_snapshot[req.ref_body].radius});
                        } else if (m_context.last_dr > 0 && dr <= 0) {
                            if (m_context.apsides.size() < 10) m_context.apsides.push_back({true, p_local, t_sim, double_r - req.celestial_snapshot[req.ref_body].radius});
                        }
                    }
                    m_context.last_r = double_r;
                    m_context.last_dr = dr;
                }
                
                if (m_context.mnv_done && !m_context.mnv_crashed) {
                    Vec3 mnv_inertial((float)(mnv_h_px - rbpx), (float)(mnv_h_py - rbpy), (float)(mnv_h_pz - rbpz));
                    Vec3 mnv_local = q_inv.rotate(mnv_inertial);
                    if (std::isfinite(mnv_local.x)) {
                        m_context.mnv_points.push_back(mnv_local);
                        m_context.mnv_point_times.push_back(t_sim);
                    }
                    
                    // Apsis detection for maneuver path
                    double double_r = mnv_inertial.length();
                    double rbx, rby, rbz, rbvx, rbvy, rbvz;
                    PhysicsSystem::GetCelestialStateAt(req.ref_body, t_sim, rbx, rby, rbz, rbvx, rbvy, rbvz);
                    double drx = mnv_h_vx - rbvx, dry = mnv_h_vy - rbvy, drz = mnv_h_vz - rbvz;
                    double dr = (mnv_inertial.x * drx + mnv_inertial.y * dry + mnv_inertial.z * drz);
                    
                    if (m_context.last_mnv_r > 0) {
                        if (m_context.last_mnv_dr < 0 && dr >= 0) {
                            if (m_context.mnv_apsides.size() < 10) m_context.mnv_apsides.push_back({false, mnv_local, t_sim, double_r - req.celestial_snapshot[req.ref_body].radius});
                        } else if (m_context.last_mnv_dr > 0 && dr <= 0) {
                            if (m_context.mnv_apsides.size() < 10) m_context.mnv_apsides.push_back({true, mnv_local, t_sim, double_r - req.celestial_snapshot[req.ref_body].radius});
                        }
                    }
                    m_context.last_mnv_r = double_r;
                    m_context.last_mnv_dr = dr;
                }
                last_record_t = t_sim;
            }
        }

        m_context.t_last = t_sim;
        m_context.px = cur_h_px; m_context.py = cur_h_py; m_context.pz = cur_h_pz;
        m_context.vx = cur_h_vx; m_context.vy = cur_h_vy; m_context.vz = cur_h_vz;
        m_context.mnv_px = mnv_h_px; m_context.mnv_py = mnv_h_py; m_context.mnv_pz = mnv_h_pz;
        m_context.mnv_vx = mnv_h_vx; m_context.mnv_vy = mnv_h_vy; m_context.mnv_vz = mnv_h_vz;

        if (req.target && !m_request.pending) {
            std::lock_guard<std::mutex> lock(*req.target->path_mutex);
            req.target->predicted_path = m_context.points;
            req.target->predicted_mnv_path = m_context.mnv_points;
            req.target->predicted_apsides = m_context.apsides;
            req.target->predicted_mnv_apsides = m_context.mnv_apsides;
            req.target->prediction_in_progress = false;
            req.target->last_prediction_sim_time = req.state.sim_time;
        }

        m_busy = false;
    }
}

} // namespace Simulation
