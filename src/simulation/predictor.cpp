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

void AsyncOrbitPredictor::RequestUpdate(RocketState* target, const RocketState& state, double pred_days, int iters, int ref_mode, int ref_body, bool force_reset) {
    if (!target) return;
    std::lock_guard<std::mutex> lock(m_request_mutex);
    
    // Auto reset if reference frame changed
    if (m_request.ref_body != ref_body || m_request.ref_mode != ref_mode) force_reset = true;
    
    m_request.target = target;
    m_request.state = state;
    m_request.pred_days = pred_days;
    m_request.iters = iters;
    m_request.ref_mode = ref_mode;
    m_request.ref_body = ref_body;
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

        if (reset_needed) {
            m_context.t_epoch = t_start;
            m_context.t_last = t_start;
            m_context.init_px = req.state.abs_px; m_context.init_py = req.state.abs_py; m_context.init_pz = req.state.abs_pz;
            m_context.init_vx = req.state.abs_vx; m_context.init_vy = req.state.abs_vy; m_context.init_vz = req.state.abs_vz;
            m_context.px = req.state.abs_px; m_context.py = req.state.abs_py; m_context.pz = req.state.abs_pz;
            m_context.vx = req.state.abs_vx; m_context.vy = req.state.abs_vy; m_context.vz = req.state.abs_vz;
            m_context.points.clear();
            m_context.mnv_points.clear();
            m_context.mnv_done = false;
            m_context.last_maneuvers = req.state.maneuvers;
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

        bool has_mnv = !req.state.maneuvers.empty();
        double trigger_t = has_mnv ? req.state.maneuvers[0].sim_time : -1.0;
        
        auto calc_acc = [&](double t, double x, double y, double z, double& ax, double& ay, double& az) {
            ax = 0; ay = 0; az = 0;
            for (int b_idx = 0; b_idx < (int)req.celestial_snapshot.size(); ++b_idx) {
                const auto& body = req.celestial_snapshot[b_idx];
                double bpx, bpy, bpz;
                PhysicsSystem::GetCelestialPositionAt(b_idx, t, bpx, bpy, bpz);
                double dx = bpx - x, dy = bpy - y, dz = bpz - z;
                double r2 = dx*dx + dy*dy + dz*dz;
                double r = std::sqrt(r2);
                if (r > 10.0) {
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

            if (has_mnv && !m_context.mnv_done && t_sim + step_dt > trigger_t) {
                step_dt = trigger_t - t_sim;
            } else if (t_sim + step_dt > max_pred_time) {
                step_dt = max_pred_time - t_sim;
            }

            if (step_dt <= 0) {
                if (t_sim >= max_pred_time - 1e-4) break;
                if (has_mnv && !m_context.mnv_done && std::abs(t_sim - trigger_t) < 0.1) {
                    step_dt = 1e-5; // force forward progress across node
                } else {
                    break;
                }
            }

            // Maneuver
            if (has_mnv && !m_context.mnv_done && t_sim >= trigger_t - 1e-4) {
                double bpx, bpy, bpz, bvx, bvy, bvz;
                PhysicsSystem::GetCelestialStateAt(nearest_body, t_sim, bpx, bpy, bpz, bvx, bvy, bvz);
                Vec3 r_rel((float)(cur_h_px - bpx), (float)(cur_h_py - bpy), (float)(cur_h_pz - bpz));
                Vec3 v_rel((float)(cur_h_vx - bvx), (float)(cur_h_vy - bvy), (float)(cur_h_vz - bvz));
                ManeuverFrame frame = ManeuverSystem::getFrame(r_rel, v_rel);
                Vec3 dv = frame.prograde * req.state.maneuvers[0].delta_v.x + 
                          frame.normal   * req.state.maneuvers[0].delta_v.y + 
                          frame.radial   * req.state.maneuvers[0].delta_v.z;
                
                mnv_h_px = cur_h_px; mnv_h_py = cur_h_py; mnv_h_pz = cur_h_pz;
                mnv_h_vx = cur_h_vx + dv.x; mnv_h_vy = cur_h_vy + dv.y; mnv_h_vz = cur_h_vz + dv.z;
                
                m_context.mnv_done = true;
                if (step_dt < 1e-5) continue;
            }

            // Integrator
            #define SYM_W_Q(C) { cur_h_px += (C)*cur_h_vx*step_dt; cur_h_py += (C)*cur_h_vy*step_dt; cur_h_pz += (C)*cur_h_vz*step_dt; t_sim += (C)*step_dt; \
                               if (m_context.mnv_done) { mnv_h_px+=(C)*mnv_h_vx*step_dt; mnv_h_py+=(C)*mnv_h_vy*step_dt; mnv_h_pz+=(C)*mnv_h_vz*step_dt; } }
            #define SYM_W_P(D) { double ax,ay,az; calc_acc(t_sim, cur_h_px,cur_h_py,cur_h_pz, ax,ay,az); cur_h_vx+=(D)*ax*step_dt; cur_h_vy+=(D)*ay*step_dt; cur_h_vz+=(D)*az*step_dt; \
                                if (m_context.mnv_done) { double max,may,maz; calc_acc(t_sim, mnv_h_px,mnv_h_py,mnv_h_pz, max,may,maz); mnv_h_vx+=(D)*max*step_dt; mnv_h_vy+=(D)*may*step_dt; mnv_h_vz+=(D)*maz*step_dt; } }
            SYM_W_Q(c1); SYM_W_P(d1);
            SYM_W_Q(c2); SYM_W_P(d2);
            SYM_W_Q(c2); SYM_W_P(d1);
            SYM_W_Q(c1);
            #undef SYM_W_Q
            #undef SYM_W_P

            // Recording
            if (t_sim >= last_record_t + record_interval || (has_mnv && std::abs(t_sim - trigger_t) < 1e-4)) {
                double rbpx, rbpy, rbpz;
                PhysicsSystem::GetCelestialPositionAt(req.ref_body, t_sim, rbpx, rbpy, rbpz);
                
                m_context.points.push_back(Vec3((float)(cur_h_px - rbpx), (float)(cur_h_py - rbpy), (float)(cur_h_pz - rbpz)));
                if (m_context.mnv_done) {
                    m_context.mnv_points.push_back(Vec3((float)(mnv_h_px - rbpx), (float)(mnv_h_py - rbpy), (float)(mnv_h_pz - rbpz)));
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
            req.target->prediction_in_progress = false;
            req.target->last_prediction_sim_time = req.state.sim_time;
        }

        m_busy = false;
    }
}

} // namespace Simulation
