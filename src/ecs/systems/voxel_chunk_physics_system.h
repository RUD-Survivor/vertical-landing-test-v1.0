#pragma once

#include "../system_scheduler.h"
#include "core/rocket_state.h"
#include "core/universe_model.h"
#include "math/math3d.h"
#include "physics/aerodynamics_system.h"
#include "physics/ground_collision_system.h"
#include <algorithm>
#include <cmath>

struct VoxelChunkPhysicsSystem : ISystem {
    VoxelChunkPhysicsSystem() : ISystem("VoxelChunkPhysics") {}

    bool update(entt::registry& registry, SystemContext& ctx) override {
        if (UniverseModel::getInstance().solar_system.empty()) return true;
        double dt = ctx.real_dt * ctx.time_warp;
        if (dt <= 0.0) return true;

        CelestialBody& body =
            UniverseModel::getInstance().solar_system[UniverseModel::getInstance().current_soi_index];

        auto view = registry.view<ChunkPhysicsTag, VoxelBodyComponent, RigidChunkComponent,
                                  TransformComponent, VelocityComponent, AttitudeComponent>(
            entt::exclude<FullPhysicsTag>);

        TelemetryComponent tele{};

        for (auto e : view) {
            auto& voxel = view.get<VoxelBodyComponent>(e);
            auto& chunk = view.get<RigidChunkComponent>(e);
            auto& trans = view.get<TransformComponent>(e);
            auto& vel = view.get<VelocityComponent>(e);
            auto& att = view.get<AttitudeComponent>(e);
            if (!voxel.model || chunk.mass <= 0.0) continue;

            double speed = std::sqrt(vel.vx * vel.vx + vel.vy * vel.vy + vel.vz * vel.vz);
            int substeps = (int)std::clamp(std::ceil(speed * dt / 1.5), 1.0, 8.0);
            double subDt = dt / substeps;

            for (int step = 0; step < substeps; step++) {
                Vec3 previousCom(trans.px, trans.py, trans.pz);
                Vec3 pos = previousCom;
                Vec3 linVel(vel.vx, vel.vy, vel.vz);

                double r = std::max(1.0, pos.length());
                Vec3 normal = pos.lengthSq() > 1e-9 ? pos.normalized() : Vec3(0, 1, 0);
                Vec3 accel = normal * (-(G_const * body.mass) / (r * r));

                RocketConfig pseudoConfig{};
                pseudoConfig.height = std::max(1.0, chunk.local_bounds_max.y - chunk.local_bounds_min.y);
                pseudoConfig.diameter = std::max(0.5, std::max(chunk.local_bounds_max.x - chunk.local_bounds_min.x,
                                                               chunk.local_bounds_max.z - chunk.local_bounds_min.z));
                AerodynamicsSystem::Sample aero =
                    AerodynamicsSystem::ComputeForces(pseudoConfig, att, body, 0.0, pos, linVel, &voxel, &chunk);
                accel += aero.force / std::max(1.0, chunk.mass);

                linVel += accel * subDt;
                pos += linVel * subDt;

                trans.px = pos.x;
                trans.py = pos.y;
                trans.pz = pos.z;
                vel.vx = linVel.x;
                vel.vy = linVel.y;
                vel.vz = linVel.z;

                GroundCollisionSystem::ResolveChunkGroundContacts(
                    trans, vel, att, voxel, chunk, body, tele, 0.0, subDt, previousCom);

                integrateAttitude(att, chunk, subDt);
            }

            trans.abs_px = body.px + trans.px;
            trans.abs_py = body.py + trans.py;
            trans.abs_pz = body.pz + trans.pz;
            vel.abs_vx = body.vx + vel.vx;
            vel.abs_vy = body.vy + vel.vy;
            vel.abs_vz = body.vz + vel.vz;
            vel.acceleration = std::sqrt(vel.vx * vel.vx + vel.vy * vel.vy + vel.vz * vel.vz);
        }
        return true;
    }

private:
    static void integrateAttitude(AttitudeComponent& att, const RigidChunkComponent& chunk, double dt) {
        Vec3 localOmega(att.ang_vel_z, att.ang_vel_roll, att.ang_vel);
        double damp = std::max(0.0, 1.0 - dt * 0.015);
        localOmega *= damp;
        double omega = localOmega.length();
        if (omega > 1e-8) {
            att.attitude = (att.attitude * Quat::fromAxisAngle(localOmega / omega, omega * dt)).normalized();
        }
        att.ang_vel_z = localOmega.x;
        att.ang_vel_roll = localOmega.y;
        att.ang_vel = localOmega.z;
        (void)chunk;
    }
};
