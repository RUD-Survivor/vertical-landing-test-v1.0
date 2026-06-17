#pragma once

#include "../system_scheduler.h"
#include "core/rocket_state.h"
#include "scene/game_context.h"
#include "physics/voxel/vessel_voxelizer.h"
#include <algorithm>

struct VoxelStructureSystem : ISystem {
    VoxelStructureSystem() : ISystem("VoxelStructure") {}

    bool update(entt::registry& registry, SystemContext&) override {
        auto view = registry.view<VoxelBodyComponent, RocketConfig>();
        const RocketAssembly& assembly = GameContext::getInstance().launch_assembly;
        for (auto e : view) {
            auto& voxel = view.get<VoxelBodyComponent>(e);
            if (voxel.model && !voxel.dirty_chunks) continue;

            VoxelPhysics::VoxelizationSettings settings;
            settings.voxel_size = voxel.voxel_size > 0.0 ? voxel.voxel_size : 1.0;
            voxel.model = VoxelPhysics::VoxelizeAssembly(assembly, settings);
            voxel.active_chunk = 0;
            voxel.dirty_chunks = false;

            const auto& chunks = voxel.model->getChunks();
            if (!chunks.empty()) {
                if (registry.all_of<RigidChunkComponent>(e)) {
                    auto& chunk = registry.get<RigidChunkComponent>(e);
                    chunk.chunk_id = chunks[0].id;
                    chunk.mass = std::max(1.0, chunks[0].mass);
                    chunk.local_center_of_mass = chunks[0].center_of_mass;
                    chunk.inertia_diag = chunks[0].inertia_diag;
                    chunk.local_bounds_min = chunks[0].bounds_min;
                    chunk.local_bounds_max = chunks[0].bounds_max;
                    chunk.attached_to_parent = true;
                }
            }
        }
        return true;
    }
};
