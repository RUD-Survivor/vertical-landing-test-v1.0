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

// 启动后台线程
void AsyncOrbitPredictor::Start() {
    if (m_running) return;
    m_running = true;
    m_worker = std::thread(&AsyncOrbitPredictor::WorkerLoop, this);
}

// 停止后台线程并回收资源
void AsyncOrbitPredictor::Stop() {
    m_running = false;
    m_cv.notify_all(); // 唤醒正在等待的工作线程
    if (m_worker.joinable()) {
        m_worker.join();
    }
}

// 请求轨道预测更新 (RequestUpdate)
// 此函数由主线程调用，用于提交一个新的初始状态供后台预测。
void AsyncOrbitPredictor::RequestUpdate(RocketState* target, const RocketState& state, const RocketConfig& config, double pred_days, int iters, int ref_mode, int ref_body, int secondary_ref_body, bool force_reset) {
    if (!target) return;
    std::lock_guard<std::mutex> lock(m_request_mutex);
    
    // 自动重置判断：如果参考系（目标天体）发生了变化，之前的预测点就完全失效了。
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
    
    m_cv.notify_one(); // 通知工作线程开始
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

        // 1. 偏差检查 (Deviation Check)
        // 决定是直接“修剪”旧路径并继续计算，还是彻底“从头重算”。
        double t_start = req.state.sim_time;
        bool reset_needed = req.force_reset || (m_context.t_last < 0);
        
        if (!reset_needed) {
            // 如果模拟时间跳跃太大（超过 1 小时）或者时间倒流，必须重算。
            if (t_start < m_context.t_epoch || t_start > m_context.t_last + 3600.0) {
                reset_needed = true;
            } else if (req.state.maneuvers.size() != m_context.last_maneuvers.size()) {
                reset_needed = true;
            } else if (!req.state.maneuvers.empty()) {
                // 如果用户修改了变轨节点 (Maneuver Node)，之前的预测就作废了。
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
            // 从当前位置和速度开始新的轨迹积分
            m_context.t_epoch = t_start;
            m_context.t_last = t_start;
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
            m_context.last_maneuvers = req.state.maneuvers;
            m_context.last_ref_body = req.ref_body;
        } else {
            // 路径修剪 (Pruning)
            // 移除那些已经过去的路径点（早于当前时间 - 30秒），防止预测线无限拉长导致渲染和内存开销。
            auto it = std::lower_bound(m_context.point_times.begin(), m_context.point_times.end(), t_start - 30.0);
            size_t idx = std::distance(m_context.point_times.begin(), it);
            if (idx > 0) {
                m_context.points.erase(m_context.points.begin(), m_context.points.begin() + idx);
                m_context.point_times.erase(m_context.point_times.begin(), it);
            }
        }

        // 2. 数值积分 (Numerical Integration)
        // 使用 4 阶辛积分 (FR Integrator) 来保证轨道的能量守恒。
        // 这对于长时间的轨道预测比普通 RK4 更稳定。
        const double FR_theta = 1.0 / (2.0 - std::cbrt(2.0));
        const double c1 = FR_theta / 2.0, c2 = (1.0 - FR_theta) / 2.0;
        const double d1 = FR_theta, d2 = 1.0 - 2.0 * FR_theta;

        double t_sim = m_context.t_last;
        double max_pred_time = t_start + req.pred_days * 86400.0;
        
        double cur_h_px = m_context.px, cur_h_py = m_context.py, cur_h_pz = m_context.pz;
        double cur_h_vx = m_context.vx, cur_h_vy = m_context.vy, cur_h_vz = m_context.vz;

        double mnv_h_px = m_context.mnv_px, mnv_h_py = m_context.mnv_py, mnv_h_pz = m_context.mnv_pz;
        double mnv_h_vx = m_context.mnv_vx, mnv_h_vy = m_context.mnv_vy, mnv_h_vz = m_context.mnv_vz;

        bool loop_has_mnv = has_mnv;
        if (freeze_mnv) {
            loop_has_mnv = false; // 冻结构思中的轨道，不再进行新的变轨积分
        }
        
        // 重力加速度计算函数
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
                if (r > body.radius * 0.1) { // 简单的奇点保护
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

            // 3. 自适应步长 (Adaptive Step)
            // 距离天体越近，重力梯度变化越剧烈，需要越小的步长来维持数值稳定。
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
            // 启发式步长公式：根据距离和速度的比例动态调整。
            double step_dt = 0.1 * std::sqrt(min_dist_sq / std::max(v_sq, 1.0));
            step_dt = std::clamp(step_dt, 5.0, 14400.0); // 最小 5 秒，最大 4 小时

            // 如果即将到达变轨点，对齐步长
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

            // 4. 变轨特殊处理 (Maneuver Handling)
            if (loop_has_mnv && !m_context.mnv_done && t_sim >= trigger_t - 1e-4) {
                const auto& node = req.state.maneuvers[0];
                double bpx, bpy, bpz, bvx, bvy, bvz;
                PhysicsSystem::GetCelestialStateAt(nearest_body, t_sim, bpx, bpy, bpz, bvx, bvy, bvz);
                // 计算当前局部轨道系的坐标轴（Prograde, Normal, Radial）
                Vec3 r_rel((float)(cur_h_px - bpx), (float)(cur_h_py - bpy), (float)(cur_h_pz - bpz));
                Vec3 v_rel((float)(cur_h_vx - bvx), (float)(cur_h_vy - bvy), (float)(cur_h_vz - bvz));
                ManeuverFrame frame = ManeuverSystem::getFrame(r_rel, v_rel);
                Vec3 dv_dir = frame.prograde * node.delta_v.x + 
                              frame.normal   * node.delta_v.y + 
                              frame.radial   * node.delta_v.z;
                
                if (node.burn_mode == 0) {
                    // === 瞬间冲量模式 (Impulse Mode) ===
                    // 模拟理想的瞬间速度增益（常见于简单的轨道力学模型）
                    mnv_h_px = cur_h_px; mnv_h_py = cur_h_py; mnv_h_pz = cur_h_pz;
                    mnv_h_vx = cur_h_vx + dv_dir.x; mnv_h_vy = cur_h_vy + dv_dir.y; mnv_h_vz = cur_h_vz + dv_dir.z;
                    m_context.mnv_done = true;
                } else {
                    // === 有限时长点火模式 (Finite Burn Mode) ===
                    if (!m_context.mnv_burning) {
                        // 初始化点火状态：捕捉这一瞬间的推进方向并锁定（惯性锁定）
                        m_context.mnv_burning = true;
                        m_context.mnv_remaining_dv = dv_dir.length();
                        m_context.mnv_mass = req.state.fuel + req.config.dry_mass + req.config.upper_stages_mass;
                        m_context.mnv_thrust_dir = dv_dir.normalized();
                        // 变轨轨迹从当前常规轨迹处“分叉”出来
                        mnv_h_px = cur_h_px; mnv_h_py = cur_h_py; mnv_h_pz = cur_h_pz;
                        mnv_h_vx = cur_h_vx; mnv_h_vy = cur_h_vy; mnv_h_vz = cur_h_vz;
                    }
                }
                if (step_dt < 1e-5 && !m_context.mnv_burning) continue;
            }
            
            // 持续点火时的推力积分 (Finite Burn Integration)
            if (m_context.mnv_burning && !m_context.mnv_done) {
                // 点火期间需要极高的计算频率 (1秒或更低)，因为质量在变，加速度也在变。
                step_dt = std::min(step_dt, 1.0);
                
                double thrust = 0;
                if (req.state.current_stage < (int)req.config.stage_configs.size())
                    thrust = req.config.stage_configs[req.state.current_stage].thrust;
                double ve = req.config.specific_impulse * 9.80665; // 有效排气速度
                double mdot = (ve > 0) ? thrust / ve : 0;        // 质量流率 (齐奥尔科夫斯基公式)
                
                if (thrust > 0 && m_context.mnv_mass > req.config.dry_mass && m_context.mnv_remaining_dv > 0.01) {
                    Vec3 thrust_dir = m_context.mnv_thrust_dir; 
                    
                    // 根据 F=ma 应用加速度
                    double accel = thrust / m_context.mnv_mass;
                    double dv_this_step = accel * step_dt;
                    if (dv_this_step > m_context.mnv_remaining_dv) {
                        dv_this_step = m_context.mnv_remaining_dv;
                        step_dt = dv_this_step / accel;
                    }
                    
                    mnv_h_vx += thrust_dir.x * dv_this_step;
                    mnv_h_vy += thrust_dir.y * dv_this_step;
                    mnv_h_vz += thrust_dir.z * dv_this_step;
                    
                    // 燃料消耗导致的质量衰减
                    m_context.mnv_mass -= mdot * step_dt;
                    m_context.mnv_remaining_dv -= dv_this_step;
                    
                    if (m_context.mnv_remaining_dv <= 0.01) {
                        m_context.mnv_burning = false;
                        m_context.mnv_done = true;
                    }
                    
                    // 将积分出来的关键点位反馈回主线程，用于导航引导。
                    if (!req.target->maneuvers.empty()) {
                        auto& node = req.target->maneuvers[0];
                        std::lock_guard<std::mutex> lock(*req.target->path_mutex);
                        
                        // Capture Absolute state
                        node.snap_px = mnv_h_px; node.snap_py = mnv_h_py; node.snap_pz = mnv_h_pz;
                        node.snap_vx = mnv_h_vx; node.snap_vy = mnv_h_vy; node.snap_vz = mnv_h_vz;
                        
                        // Capture Relative state (Critical for High-Precision Guidance)
                        double rbpx, rbpy, rbpz, rbvx, rbvy, rbvz;
                        PhysicsSystem::GetCelestialStateAt(req.ref_body, t_sim, rbpx, rbpy, rbpz, rbvx, rbvy, rbvz);
                        node.snap_rel_px = mnv_h_px - rbpx;
                        node.snap_rel_py = mnv_h_py - rbpy;
                        node.snap_rel_pz = mnv_h_pz - rbpz;
                        node.snap_rel_vx = mnv_h_vx - rbvx;
                        node.snap_rel_vy = mnv_h_vy - rbvy;
                        node.snap_rel_vz = mnv_h_vz - rbvz;

                        node.snap_time = t_sim;
                        node.locked_burn_dir = m_context.mnv_thrust_dir;
                    }
                }
            }

            // 5. 应用辛积分子步 (Symplectic Substeps)
            // 分别更新位置 (Q) 和速度 (P)
            #define SYM_W_Q(C) { \
                if (!m_context.crashed) { cur_h_px += (C)*cur_h_vx*step_dt; cur_h_py += (C)*cur_h_vy*step_dt; cur_h_pz += (C)*cur_h_vz*step_dt; } \
                if (loop_has_mnv && (m_context.mnv_done || m_context.mnv_burning) && !m_context.mnv_crashed) { \
                    mnv_h_px+=(C)*mnv_h_vx*step_dt; mnv_h_py+=(C)*mnv_h_vy*step_dt; mnv_h_pz+=(C)*mnv_h_vz*step_dt; \
                } \
                t_sim += (C)*step_dt; \
            }
            #define SYM_W_P(D) { \
                if (!m_context.crashed) { \
                    double ax,ay,az; calc_acc(t_sim, cur_h_px,cur_h_py,cur_h_pz, ax,ay,az); \
                    cur_h_vx+=(D)*ax*step_dt; cur_h_vy+=(D)*ay*step_dt; cur_h_vz+=(D)*az*step_dt; \
                } \
                if (loop_has_mnv && (m_context.mnv_done || m_context.mnv_burning) && !m_context.mnv_crashed) { \
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

            // 碰撞检测 (Collision Detection)
            // 简单的球面距离检测，判断火箭是否已经撞上任何行星或恒星。
            if (!m_context.crashed || (m_context.mnv_done && !m_context.mnv_crashed)) {
                for (int b=0; b<(int)req.celestial_snapshot.size(); b++) {
                    const auto& body = req.celestial_snapshot[b];
                    double bpx, bpy, bpz; PhysicsSystem::GetCelestialPositionAt(b, t_sim, bpx, bpy, bpz);
                    if (!m_context.crashed) {
                        double dx = cur_h_px - bpx, dy = cur_h_py - bpy, dz = cur_h_pz - bpz;
                        if (dx*dx+dy*dy+dz*dz < body.radius*body.radius) m_context.crashed = true;
                    }
                    if (loop_has_mnv && (m_context.mnv_done || m_context.mnv_burning) && !m_context.mnv_crashed) {
                        double mdx = mnv_h_px - bpx, mdy = mnv_h_py - bpy, mdz = mnv_h_pz - bpz;
                        if (mdx*mdx+mdy*mdy+mdz*mdz < body.radius*body.radius) m_context.mnv_crashed = true;
                    }
                }
            }

            if (m_context.crashed && (!m_context.mnv_done || m_context.mnv_crashed)) break;

            // 6. 录制路径点 (Recording)
            // 我们不能把每一秒的位置都存下来，那会拖慢渲染。
            // 这里根据路径的“弯曲程度”和重要性来动态决定记录间隔。
            double rb_dist = std::sqrt(min_dist_sq);
            double v_mag = std::sqrt(v_sq);
            record_interval = std::clamp(rb_dist / (v_mag + 1.0) * 0.05, 10.0, 1200.0);
            if (m_context.mnv_burning) record_interval = 10.0;

            if (t_sim >= last_record_t + record_interval || (loop_has_mnv && std::abs(t_sim - trigger_t) < 1e-4)) {
                double rbpx, rbpy, rbpz;
                PhysicsSystem::GetCelestialPositionAt(req.ref_body, t_sim, rbpx, rbpy, rbpz);
                
                // 将坐标从绝对的“日心系”转换到渲染所用的“局部参考系”。
                Quat q_inv = PhysicsSystem::GetFrameRotation(req.ref_mode, req.ref_body, req.secondary_ref_body, t_sim).conjugate();
                
                if (!m_context.crashed) {
                    Vec3 p_inertial((float)(cur_h_px - rbpx), (float)(cur_h_py - rbpy), (float)(cur_h_pz - rbpz));
                    Vec3 p_local = q_inv.rotate(p_inertial);
                    if (std::isfinite(p_local.x)) {
                        m_context.points.push_back(p_local);
                        m_context.point_times.push_back(t_sim);
                    }
                    
                    // 轨道近地点/远地点检测 (Apsis Detection)
                    // 通过检测径向速度 (dr) 的正负切换来寻找距离的极值点。
                    double double_r = p_inertial.length();
                    double rbx, rby, rbz, rbvx, rbvy, rbvz;
                    PhysicsSystem::GetCelestialStateAt(req.ref_body, t_sim, rbx, rby, rbz, rbvx, rbvy, rbvz);
                    double drx = cur_h_vx - rbvx, dry = cur_h_vy - rbvy, drz = cur_h_vz - rbvz;
                    double dr = (p_inertial.x * drx + p_inertial.y * dry + p_inertial.z * drz);
                    
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
                
                if (loop_has_mnv && (m_context.mnv_done || m_context.mnv_burning) && !m_context.mnv_crashed) {
                    Vec3 mnv_inertial((float)(mnv_h_px - rbpx), (float)(mnv_h_py - rbpy), (float)(mnv_h_pz - rbpz));
                    Vec3 mnv_local = q_inv.rotate(mnv_inertial);
                    if (std::isfinite(mnv_local.x)) {
                        m_context.mnv_points.push_back(mnv_local);
                        m_context.mnv_point_times.push_back(t_sim);
                    }
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
