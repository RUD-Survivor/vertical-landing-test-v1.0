#pragma once
// ==========================================================
// physics_pipeline_system.h — FULL 物理主循环管线
//
// 使用 view<FullPhysicsTag, ...>() 遍历所有受控火箭实体
// 1 个或 1000 个——代码完全一样，零改动即可扩展
//
// 时间加速策略（Principia 风格 coast / burn 分离）：
// - warp>1 且可滑行（大气外、无推力）：引力 coast，轨道尺度步长
// - 推力 / 大气 / 近地：完整 PhysicsSystem::Update + 机身尺度子步
// ==========================================================

#include "../system_scheduler.h"
#include "core/rocket_state.h"
#include "core/universe_model.h"
#include "physics/physics_system.h"
#include "control/control_system.h"
#include "simulation/stage_manager.h"
#include "simulation/structural_state.h"
#include <algorithm>
#include <cmath>

struct PhysicsPipelineSystem : ISystem {
    PhysicsPipelineSystem() : ISystem("PhysicsPipeline") {}

    bool update(entt::registry& registry, SystemContext& ctx) override {
        // 统一全局物理时钟——所有实体用同一个 dt
        double dt = ctx.real_dt * ctx.time_warp;

        auto view = registry.view<FullPhysicsTag,
            RocketConfig, ControlInput,
            TransformComponent, VelocityComponent, AttitudeComponent,
            PropulsionComponent, TelemetryComponent, GuidanceComponent>();

        for (auto e : view) {
            auto& guid = view.get<GuidanceComponent>(e);
            auto& prop = view.get<PropulsionComponent>(e);
            auto& tele = view.get<TelemetryComponent>(e);
            auto& input = view.get<ControlInput>(e);

            // 可滑行：从 10× 起走引力 coast，不必等到 >1000
            if (canCoast(ctx.time_warp, guid, prop, tele, input)) {
                coastPath(registry, e, dt);
                continue;
            }

            // 1) 自动驾驶（仅完整物理路径；coast 时跳过，避免大 dt 打爆 PID）
            if (!ctx.mnv_autopilot_active || e != ctx.focused_entity) {
                if (guid.auto_mode) {
                    ControlSystem::UpdateAutoPilot(registry, e, dt);
                }
            }

            // 2) 物理积分：高速或高速旋转时拆子步（机身尺度，仅 burn/大气）
            const RocketConfig& config = StructuralState::physicsConfig(registry, e);
            auto& vel = view.get<VelocityComponent>(e);
            auto& att = view.get<AttitudeComponent>(e);
            int substeps = computeSubsteps(config, vel, att, dt, ctx.time_warp);
            double stepDt = dt / (double)substeps;
            for (int i = 0; i < substeps; i++) {
                PhysicsSystem::Update(registry, e, stepDt);
            }

            // 3) 自动级间分离
            if (StageManager::IsCurrentStageEmpty(prop)
                && prop.current_stage < prop.total_stages - 1
                && (guid.status == ASCEND || guid.status == DESCEND)) {
                StageManager::SeparateStage(registry, e);
                if (registry.all_of<StructuralStateComponent>(e)) {
                    const auto& structural = registry.get<StructuralStateComponent>(e);
                    if (structural.structurally_damaged) {
                        StructuralState::rebuild(registry, e, false);
                    }
                }
            }
        }

        // === 实时烟雾（仅焦点实体，且无加速时） ===
        if (ctx.time_warp == 1 && registry.valid(ctx.focused_entity)) {
            PhysicsSystem::EmitSmoke(registry, ctx.focused_entity, ctx.real_dt);
            PhysicsSystem::UpdateSmoke(registry, ctx.focused_entity, ctx.real_dt);
        }

        return true;
    }

private:
    // 与 aerodynamics_system / 超级加速安全条件一致：100 km 外视为可忽略大气
    static constexpr double kCoastMinAltitudeM = 100000.0;
    static constexpr double kCoastMaxThrust = 0.01;
    static constexpr double kCoastMaxThrottle = 0.01;
    // 完整物理在加速下的每帧子步预算（避免 100× 点火时顶满 4096）
    static constexpr double kMaxRealtimeSubsteps = 4096.0;
    static constexpr double kMaxWarpFullPhysicsSubsteps = 256.0;

    static bool canCoast(int time_warp,
                         const GuidanceComponent& guid,
                         const PropulsionComponent& prop,
                         const TelemetryComponent& tele,
                         const ControlInput& input) {
        if (time_warp <= 1) return false;

        double surface_speed = std::sqrt(tele.velocity * tele.velocity + tele.local_vx * tele.local_vx);
        bool parked = (guid.status == PRE_LAUNCH) && surface_speed < 0.1;
        if (parked) return true;

        bool thrusting = prop.thrust_power > kCoastMaxThrust
                      || (input.throttle > kCoastMaxThrottle && prop.fuel > 0.0);
        if (thrusting) return false;

        // 大气内 / 近地仍走完整物理（气动 + 碰撞需要机身尺度子步）
        if (tele.altitude < kCoastMinAltitudeM) return false;

        return true;
    }

    static int computeSubsteps(const RocketConfig& config,
                               const VelocityComponent& vel,
                               const AttitudeComponent& att,
                               double dt,
                               int time_warp) {
        if (dt <= 0.0) return 1;
        double speed = std::sqrt(vel.vx * vel.vx + vel.vy * vel.vy + vel.vz * vel.vz);
        double omega = std::sqrt(att.ang_vel * att.ang_vel +
                                 att.ang_vel_z * att.ang_vel_z +
                                 att.ang_vel_roll * att.ang_vel_roll);
        if (!std::isfinite(speed)) speed = 0.0;
        if (!std::isfinite(omega)) omega = 0.0;

        double characteristic = std::max(0.5, std::min(std::max(1.0, config.height),
                                                       std::max(0.5, config.diameter)));
        double maxLinearTravel = std::max(0.25, characteristic * 0.25);
        double maxAngularTravel = 0.20; // radians per substep, numerical accuracy constraint
        double linearSteps = (speed > 0.0) ? std::ceil((speed * dt) / maxLinearTravel) : 1.0;
        double angularSteps = (omega > 0.0) ? std::ceil((omega * dt) / maxAngularTravel) : 1.0;
        double required = std::max(1.0, std::max(linearSteps, angularSteps));
        double cap = (time_warp > 1) ? kMaxWarpFullPhysicsSubsteps : kMaxRealtimeSubsteps;
        return (int)std::min(required, cap);
    }

    static void coastPath(entt::registry& reg, entt::entity e, double dt) {
        auto& trans = reg.get<TransformComponent>(e);
        auto& vel   = reg.get<VelocityComponent>(e);
        auto& att   = reg.get<AttitudeComponent>(e);
        auto& tele  = reg.get<TelemetryComponent>(e);
        auto& guid  = reg.get<GuidanceComponent>(e);

        double ss = std::sqrt(tele.velocity * tele.velocity + tele.local_vx * tele.local_vx);
        bool parked = (guid.status == PRE_LAUNCH) && ss < 0.1;
        if (parked) {
            if (guid.status != PRE_LAUNCH && guid.status != LANDED) {
                guid.status = LANDED;
                CelestialBody& cb = UniverseModel::getInstance().solar_system[UniverseModel::getInstance().current_soi_index];
                double th = cb.prime_meridian_epoch + (tele.sim_time * 2.0 * PI / cb.rotation_period);
                trans.surf_px = trans.px * std::cos(-th) - trans.py * std::sin(-th);
                trans.surf_py = trans.px * std::sin(-th) + trans.py * std::cos(-th);
                trans.surf_pz = trans.pz;
            }
            vel.vx = 0; vel.vy = 0; vel.vz = 0;
            tele.velocity = 0; tele.local_vx = 0;
            att.ang_vel = 0; att.ang_vel_z = 0;
        }
        PhysicsSystem::FastGravityUpdate(reg, e, dt);
    }
};
