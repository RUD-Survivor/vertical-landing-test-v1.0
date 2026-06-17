#pragma once

#include "../system_scheduler.h"
#include "core/rocket_state.h"
#include "physics/chunk_body_collision.h"

struct VoxelChunkCollisionSystem : ISystem {
    VoxelChunkCollisionSystem() : ISystem("VoxelChunkCollision") {}

    bool update(entt::registry& registry, SystemContext& ctx) override {
        double dt = ctx.real_dt * ctx.time_warp;
        if (dt <= 0.0) return true;
        ChunkBodyCollision::resolveRegistryPairs(registry, dt, 2);
        return true;
    }
};
