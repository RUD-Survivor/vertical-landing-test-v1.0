#pragma once
// ==========================================================
// physics_pipeline_system.h — FULL 物理主循环管线
//
// 使用 view<FullPhysicsTag, ...>() 遍历所有受控火箭实体
// 1 个或 1000 个——代码完全一样，零改动即可扩展
// ==========================================================

#include "../system_scheduler.h"
#include "core/rocket_state.h"
#include "core/universe_model.h"
#include "physics/physics_system.h"
#include "control/control_system.h"
#include "simulation/stage_manager.h"

struct PhysicsPipelineSystem : ISystem {
    PhysicsPipelineSystem() : ISystem("PhysicsPipeline") {}

    bool update(entt::registry& registry, SystemContext& ctx) override {
        // 统一全局物理时钟——所有实体用同一个 dt
        double dt = ctx.real_dt * ctx.time_warp;

        // === 超级加速路径（跳过精细积分） ===
        if (ctx.time_warp > 1000) {
            auto view = registry.view<FullPhysicsTag, TransformComponent, VelocityComponent, AttitudeComponent, TelemetryComponent, GuidanceComponent>();
            for (auto e : view) {
                superWarpPath(registry, e, dt);
            }
            return true;
        }

        // === 标准物理：每帧一步，所有实体共用同一个 dt ===
        auto view = registry.view<FullPhysicsTag,
            RocketConfig, ControlInput,
            TransformComponent, VelocityComponent, AttitudeComponent,
            PropulsionComponent, TelemetryComponent, GuidanceComponent>();

        for (auto e : view) {
            auto& guid = view.get<GuidanceComponent>(e);
            if (guid.status == LANDED || guid.status == CRASHED) continue;

            // 1) 自动驾驶
            if (!ctx.mnv_autopilot_active || e != ctx.focused_entity) {
                if (guid.auto_mode) {
                    ControlSystem::UpdateAutoPilot(registry, e, dt);
                }
            }

            // 2) 物理积分
            PhysicsSystem::Update(registry, e, dt);

            // 3) 自动级间分离
            auto& prop = view.get<PropulsionComponent>(e);
            if (StageManager::IsCurrentStageEmpty(prop)
                && prop.current_stage < prop.total_stages - 1
                && (guid.status == ASCEND || guid.status == DESCEND)) {
                StageManager::SeparateStage(registry, e);
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
    static void superWarpPath(entt::registry& reg, entt::entity e, double dt) {
        auto& trans = reg.get<TransformComponent>(e);
        auto& vel   = reg.get<VelocityComponent>(e);
        auto& att   = reg.get<AttitudeComponent>(e);
        auto& tele  = reg.get<TelemetryComponent>(e);
        auto& guid  = reg.get<GuidanceComponent>(e);

        double ss = std::sqrt(tele.velocity*tele.velocity + tele.local_vx*tele.local_vx);
        bool parked = (guid.status==PRE_LAUNCH||guid.status==LANDED) && ss<0.1;
        if (parked) {
            if (guid.status!=PRE_LAUNCH && guid.status!=LANDED) {
                guid.status=LANDED;
                CelestialBody& cb = UniverseModel::getInstance().solar_system[UniverseModel::getInstance().current_soi_index];
                double th = cb.prime_meridian_epoch + (tele.sim_time*2.0*PI/cb.rotation_period);
                trans.surf_px=trans.px*std::cos(-th)-trans.py*std::sin(-th);
                trans.surf_py=trans.px*std::sin(-th)+trans.py*std::cos(-th);
                trans.surf_pz=trans.pz;
            }
            vel.vx=0;vel.vy=0;vel.vz=0; tele.velocity=0;tele.local_vx=0;
            att.ang_vel=0;att.ang_vel_z=0;
        }
        PhysicsSystem::FastGravityUpdate(reg, e, dt);
    }
};
