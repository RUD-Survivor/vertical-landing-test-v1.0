#pragma once

#include "../system_scheduler.h"
#include "core/rocket_state.h"
#include "simulation/structural_state.h"
#include <algorithm>
#include <cmath>
#include <unordered_set>
#include <vector>

struct VoxelFractureSystem : ISystem {
    VoxelFractureSystem() : ISystem("VoxelFracture") {}

    bool update(entt::registry& registry, SystemContext&) override {
        auto zoneView = registry.view<BreakableBodyTag, VoxelBodyComponent, TransformComponent,
                                      VelocityComponent, AttitudeComponent, DeformableZoneComponent>();
        for (auto e : zoneView) {
            auto& voxel = zoneView.get<VoxelBodyComponent>(e);
            auto& zone = zoneView.get<DeformableZoneComponent>(e);
            if (!zone.ready_to_rechunk) continue;
            if (!voxel.model) {
                registry.remove<DeformableZoneComponent>(e);
                continue;
            }

            voxel.model->rebuildSurfaces();
            auto chunks = voxel.model->buildConnectedChunks();
            voxel.dirty_chunks = false;
            auto& debug = registry.emplace_or_replace<VoxelDamageDebugComponent>(e);
            debug.phase = "RECHUNKED";
            debug.ttl = 2.0;
            debug.age = zone.age;
            debug.radius = zone.radius;
            debug.max_node_speed = zone.max_node_speed;
            debug.node_count = (int)zone.nodes.size();
            debug.constraint_count = (int)zone.constraints.size();
            debug.broken_constraint_count = 0;
            for (const auto& c : zone.constraints) {
                if (c.bond == VoxelPhysics::kInvalidBond) debug.broken_constraint_count++;
            }
            debug.chunk_count = 0;
            CopyVoxelZoneDebugMarkers(debug, zone);
            if (!chunks.empty()) {
                debug.chunk_count = finalizeFracture(registry, e, voxel,
                                    zoneView.get<TransformComponent>(e),
                                    zoneView.get<VelocityComponent>(e),
                                    zoneView.get<AttitudeComponent>(e),
                                    chunks,
                                    zone.impulse);
            }
            if (registry.all_of<DamageEventComponent>(e)) {
                registry.remove<DamageEventComponent>(e);
            }
            registry.remove<DeformableZoneComponent>(e);
        }

        return true;
    }

private:
    static bool chunkHasActiveVoxels(const VoxelPhysics::VesselVoxelModel& model,
                                     const VoxelPhysics::VoxelChunkSummary& chunk) {
        const auto& cells = model.getCells();
        for (VoxelPhysics::VoxelId id : chunk.voxels) {
            if (id >= 0 && id < (VoxelPhysics::VoxelId)cells.size() && cells[(size_t)id].active) {
                return true;
            }
        }
        return false;
    }

    // 返回 spawn 的 detached chunk 数量（用于 HUD）
    static int finalizeFracture(entt::registry& registry,
                                entt::entity source,
                                VoxelBodyComponent& sourceVoxel,
                                const TransformComponent& trans,
                                const VelocityComponent& vel,
                                const AttitudeComponent& att,
                                const std::vector<VoxelPhysics::VoxelChunkSummary>& chunks,
                                double impactImpulse) {
        if (chunks.empty() || !sourceVoxel.model) return 0;

        auto largest = std::max_element(chunks.begin(), chunks.end(),
            [](const auto& a, const auto& b) { return a.mass < b.mass; });
        VoxelPhysics::ChunkId keepId = largest != chunks.end() ? largest->id : 0;
        const VoxelPhysics::VoxelChunkSummary& keptChunk = chunks[(size_t)keepId];
        Vec3 keepCom = keptChunk.center_of_mass;

        sourceVoxel.active_chunk = keepId;
        sourceVoxel.structure_fractured = true;

        if (registry.all_of<RigidChunkComponent>(source)) {
            auto& mainChunk = registry.get<RigidChunkComponent>(source);
            mainChunk.chunk_id = keepId;
            mainChunk.owned_voxels.clear();
            {
                const auto& cells = sourceVoxel.model->getCells();
                mainChunk.owned_voxels.reserve(keptChunk.voxels.size());
                for (VoxelPhysics::VoxelId id : keptChunk.voxels) {
                    if (id >= 0 && id < (VoxelPhysics::VoxelId)cells.size() && cells[(size_t)id].active) {
                        mainChunk.owned_voxels.push_back(id);
                    }
                }
            }
            mainChunk.mass = std::max(1.0, keptChunk.mass);
            mainChunk.local_center_of_mass = keptChunk.center_of_mass;
            mainChunk.inertia_diag = keptChunk.inertia_diag;
            mainChunk.local_bounds_min = keptChunk.bounds_min;
            mainChunk.local_bounds_max = keptChunk.bounds_max;
            mainChunk.attached_to_parent = true;
        }

        std::vector<const VoxelPhysics::VoxelChunkSummary*> detached;
        detached.reserve(chunks.size());
        for (const auto& chunk : chunks) {
            if (chunk.id == keepId || chunk.voxels.empty()) continue;
            if (!chunkHasActiveVoxels(*sourceVoxel.model, chunk)) continue;
            detached.push_back(&chunk);
        }
        std::sort(detached.begin(), detached.end(),
            [](const auto* a, const auto* b) { return a->mass > b->mass; });

        int spawned = 0;
        for (const VoxelPhysics::VoxelChunkSummary* chunkPtr : detached) {
            const auto& chunk = *chunkPtr;
            if (chunk.mass <= 0.0) continue;

            std::vector<VoxelPhysics::VoxelId> owned;
            owned.reserve(chunk.voxels.size());
            const auto& cells = sourceVoxel.model->getCells();
            for (VoxelPhysics::VoxelId id : chunk.voxels) {
                if (id >= 0 && id < (VoxelPhysics::VoxelId)cells.size() && cells[(size_t)id].active) {
                    owned.push_back(id);
                }
            }
            if (owned.empty()) continue;

            sourceVoxel.model->deactivateVoxels(owned);

            entt::entity debris = registry.create();
            registry.emplace<TagComponent>(debris, EntityTag::DEBRIS,
                "Fractured Chunk " + std::to_string(chunk.id));

            TransformComponent debrisTrans = trans;
            Vec3 worldOffset = att.attitude.rotate(chunk.center_of_mass);
            debrisTrans.px += worldOffset.x;
            debrisTrans.py += worldOffset.y;
            debrisTrans.pz += worldOffset.z;
            debrisTrans.abs_px += worldOffset.x;
            debrisTrans.abs_py += worldOffset.y;
            debrisTrans.abs_pz += worldOffset.z;
            registry.emplace<TransformComponent>(debris) = debrisTrans;

            Vec3 sepLocal = chunk.center_of_mass - keepCom;
            if (sepLocal.lengthSq() < 1e-6) sepLocal = Vec3(0.0, 1.0, 0.0);
            Vec3 sepWorld = att.attitude.rotate(sepLocal.normalized());
            double sepSpeed = std::sqrt(std::max(0.0, impactImpulse) / std::max(1.0, chunk.mass)) * 0.35;
            sepSpeed = std::max(0.5, std::min(sepSpeed, 30.0));
            VelocityComponent debrisVel = vel;
            debrisVel.vx += sepWorld.x * sepSpeed;
            debrisVel.vy += sepWorld.y * sepSpeed;
            debrisVel.vz += sepWorld.z * sepSpeed;
            registry.emplace<VelocityComponent>(debris) = debrisVel;
            registry.emplace<AttitudeComponent>(debris) = att;
            registry.emplace<ChunkPhysicsTag>(debris);

            VoxelBodyComponent childVoxel;
            childVoxel.model = sourceVoxel.model;
            childVoxel.active_chunk = chunk.id;
            childVoxel.voxel_size = sourceVoxel.voxel_size;
            registry.emplace<VoxelBodyComponent>(debris) = childVoxel;

            RigidChunkComponent rigid;
            rigid.chunk_id = chunk.id;
            rigid.owned_voxels = std::move(owned);
            rigid.mass = std::max(1.0, chunk.mass);
            rigid.local_center_of_mass = chunk.center_of_mass;
            rigid.inertia_diag = chunk.inertia_diag;
            rigid.local_bounds_min = chunk.bounds_min;
            rigid.local_bounds_max = chunk.bounds_max;
            registry.emplace<RigidChunkComponent>(debris) = rigid;
            spawned++;
        }

        sourceVoxel.model->rebuildDerivedData();
        const auto& liveChunks = sourceVoxel.model->getChunks();
        if (!liveChunks.empty()) {
            auto keptIt = std::max_element(liveChunks.begin(), liveChunks.end(),
                [](const auto& a, const auto& b) { return a.mass < b.mass; });
            const auto& kept = *keptIt;
            sourceVoxel.active_chunk = kept.id;
            if (registry.all_of<RigidChunkComponent>(source)) {
                auto& mainChunk = registry.get<RigidChunkComponent>(source);
                mainChunk.chunk_id = kept.id;
                mainChunk.mass = std::max(1.0, kept.mass);
                mainChunk.local_center_of_mass = kept.center_of_mass;
                mainChunk.inertia_diag = kept.inertia_diag;
                mainChunk.local_bounds_min = kept.bounds_min;
                mainChunk.local_bounds_max = kept.bounds_max;
                mainChunk.owned_voxels.clear();
                const auto& cells = sourceVoxel.model->getCells();
                mainChunk.owned_voxels.reserve(kept.voxels.size());
                for (VoxelPhysics::VoxelId id : kept.voxels) {
                    if (id >= 0 && id < (VoxelPhysics::VoxelId)cells.size() && cells[(size_t)id].active) {
                        mainChunk.owned_voxels.push_back(id);
                    }
                }
            }
        }

        if (sourceVoxel.model->countActiveVoxels() <= 0) {
            registry.remove<BreakableBodyTag>(source);
        }

        StructuralState::rebuild(registry, source, true);
        return spawned;
    }
};
