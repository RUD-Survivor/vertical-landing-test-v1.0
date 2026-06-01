#pragma once
// ==========================================================
// debris_physics_system.h — SIMPLE 物理（碎片/不受控实体）
//
// 使用 view<...>(entt::exclude<FullPhysicsTag>) 
// 自动遍历所有没有 FullPhysicsTag 的实体
// ==========================================================

#include "../system_scheduler.h"
#include "core/rocket_state.h"
#include "core/universe_model.h"
#include "math/math3d.h"
#include "physics/physics_system.h"
#include <cmath>

struct DebrisPhysicsSystem : ISystem {
    DebrisPhysicsSystem() : ISystem("DebrisPhysics") {}

    bool update(entt::registry& registry, SystemContext& ctx) override {
        // 排除 FullPhysicsTag —— 只处理不受控实体
        auto view = registry.view<TransformComponent, VelocityComponent, AttitudeComponent>(
            entt::exclude<FullPhysicsTag, PendingDestroy>);

        // 统一全局物理时钟（含时间加速）
        double dt = ctx.real_dt * ctx.time_warp;
        CelestialBody& body = UniverseModel::getInstance().solar_system[UniverseModel::getInstance().current_soi_index];

        for (auto e : view) {
            auto& trans = view.get<TransformComponent>(e);
            auto& vel   = view.get<VelocityComponent>(e);
            auto& att   = view.get<AttitudeComponent>(e);

            double dist = std::sqrt(trans.px*trans.px + trans.py*trans.py + trans.pz*trans.pz);
            if (dist < body.radius * 0.5) { registry.emplace<PendingDestroy>(e); continue; }

            // 引力
            double g = PhysicsSystem::get_gravity(dist);
            double nx = -trans.px/dist, ny = -trans.py/dist, nz = -trans.pz/dist;

            // 大气阻力（简化）
            double alt = dist - body.radius, speed = std::sqrt(vel.vx*vel.vx+vel.vy*vel.vy+vel.vz*vel.vz);
            double drag = 0;
            if (alt < 100000.0 && speed > 0.01) {
                double rho = PhysicsSystem::get_air_density(alt);
                drag = 0.5 * rho * speed * speed * 0.5 * 10.0 / 2000.0; // Cd*A/m ≈ 0.5*10/2000
            }

            // 速度积分
            vel.vx += g*nx*dt; vel.vy += g*ny*dt; vel.vz += g*nz*dt;
            if (drag > 0) {
                vel.vx -= (vel.vx/speed)*drag*dt;
                vel.vy -= (vel.vy/speed)*drag*dt;
                vel.vz -= (vel.vz/speed)*drag*dt;
            }

            // 位置积分
            trans.px += vel.vx*dt; trans.py += vel.vy*dt; trans.pz += vel.vz*dt;

            // 惯性翻滚
            att.ang_vel *= 0.999;
            att.attitude = att.attitude * Quat::fromAxisAngle(Vec3(0,0,1), (float)(att.ang_vel*dt));
            att.attitude = att.attitude * Quat::fromAxisAngle(Vec3(1,0,0), (float)(att.ang_vel_z*dt));
        }
        return true;
    }
};
