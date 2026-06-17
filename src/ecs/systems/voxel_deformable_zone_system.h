#pragma once

#include "../system_scheduler.h"
#include "core/rocket_state.h"
#include <algorithm>
#include <cmath>
#include <utility>
#include <vector>

struct VoxelDeformableZoneSystem : ISystem {
    VoxelDeformableZoneSystem() : ISystem("VoxelDeformableZone") {}

    bool update(entt::registry& registry, SystemContext& ctx) override {
        double dt = std::min(0.033, std::max(0.001, ctx.real_dt * ctx.time_warp));
        decayDebugTelemetry(registry, dt);
        consumeDamageEvents(registry);

        auto view = registry.view<BreakableBodyTag, VoxelBodyComponent, DeformableZoneComponent, TelemetryComponent>(
            entt::exclude<PendingDestroy>);
        for (auto e : view) {
            auto& voxel = view.get<VoxelBodyComponent>(e);
            auto& zone = view.get<DeformableZoneComponent>(e);
            const double simTime = view.get<TelemetryComponent>(e).sim_time;
            if (!voxel.model) {
                zone.ready_to_rechunk = true;
                updateDebugTelemetry(registry, e, zone);
                continue;
            }
            if (zone.ready_to_rechunk) {
                updateDebugTelemetry(registry, e, zone);
                continue;
            }

            if (!zone.initialized) {
                initializeZone(*voxel.model, zone, dt);
            }
            solveZone(*voxel.model, zone, dt, simTime);
            updateDebugTelemetry(registry, e, zone);
        }

        return true;
    }

private:
    static void decayDebugTelemetry(entt::registry& registry, double dt) {
        auto view = registry.view<VoxelDamageDebugComponent>(
            entt::exclude<DeformableZoneComponent, DamageEventComponent>);
        for (auto e : view) {
            auto& debug = view.get<VoxelDamageDebugComponent>(e);
            debug.ttl -= dt;
            if (debug.ttl <= 0.0) {
                registry.remove<VoxelDamageDebugComponent>(e);
            }
        }
    }

    static void updateDebugTelemetry(entt::registry& registry,
                                     entt::entity e,
                                     const DeformableZoneComponent& zone) {
        auto& debug = registry.emplace_or_replace<VoxelDamageDebugComponent>(e);
        debug.phase = zone.ready_to_rechunk ? "RECHUNK" : (zone.initialized ? "XPBD" : "SPAWN");
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
    }

    static void consumeDamageEvents(entt::registry& registry) {
        auto damageView = registry.view<BreakableBodyTag, VoxelBodyComponent, DamageEventComponent>(
            entt::exclude<PendingDestroy>);
        for (auto e : damageView) {
            auto& voxel = damageView.get<VoxelBodyComponent>(e);
            auto& damage = damageView.get<DamageEventComponent>(e);
            if (!voxel.model || damage.impulse <= 0.0 || damage.radius <= 0.0) {
                registry.remove<DamageEventComponent>(e);
                continue;
            }

            if (voxel.model->countActiveVoxels() <= 0) {
                registry.remove<DamageEventComponent>(e);
                continue;
            }

            if (registry.all_of<DeformableZoneComponent>(e)) {
                // Zone already owns this impact; ignore bounce spam so stable_time can settle.
                registry.remove<DamageEventComponent>(e);
                continue;
            }

            double spawnSimTime = 0.0;
            if (registry.all_of<TelemetryComponent>(e)) {
                spawnSimTime = registry.get<TelemetryComponent>(e).sim_time;
            }

            auto& zone = registry.emplace_or_replace<DeformableZoneComponent>(e);
            zone.local_center = damage.local_point;
            zone.radius = std::max(voxel.voxel_size * 2.0, damage.radius);
            zone.impulse = damage.impulse;
            zone.spawn_sim_time = spawnSimTime;
            zone.max_age = damage.hard_impact ? 0.12 : 0.25;
            zone.settle_time = damage.hard_impact ? 0.04 : 0.06;
            zone.solver_iterations = damage.hard_impact ? 8 : 5;
            zone.hard_impact = damage.hard_impact;
            auto& debug = registry.emplace_or_replace<VoxelDamageDebugComponent>(e);
            debug.phase = "SPAWN";
            debug.ttl = 2.0;
            debug.local_center = zone.local_center;
            debug.radius = zone.radius;
            CopyVoxelZoneDebugMarkers(debug, zone);
            registry.remove<DamageEventComponent>(e);
        }
    }

    static void applyImpulseToActiveZone(DeformableZoneComponent& zone,
                                         const Vec3& localPoint,
                                         double radius,
                                         double impulse) {
        if (!zone.initialized || zone.nodes.empty()) return;
        double safeRadius = std::max(0.001, radius);
        double r2 = safeRadius * safeRadius;
        for (auto& node : zone.nodes) {
            if (node.pinned) continue;
            Vec3 offset = node.position - localPoint;
            double d2 = offset.lengthSq();
            if (d2 > r2) continue;
            double falloff = 1.0 - std::sqrt(d2) / safeRadius;
            Vec3 dir = offset.normalized();
            if (dir.lengthSq() < 1e-10) dir = Vec3(0.0, -1.0, 0.0);
            double mass = node.inv_mass > 0.0 ? 1.0 / node.inv_mass : 1.0;
            double kick = std::clamp((impulse / std::max(1.0, mass)) * 0.002 * falloff, 0.0, 25.0);
            node.velocity += dir * kick;
        }
    }

    static void snapZoneCenterToNearestCell(const VoxelPhysics::VesselVoxelModel& model,
                                           DeformableZoneComponent& zone) {
        const auto& cells = model.getCells();
        VoxelPhysics::VoxelId nearest = VoxelPhysics::kInvalidVoxel;
        double bestD2 = 1e30;
        for (VoxelPhysics::VoxelId id = 0; id < (VoxelPhysics::VoxelId)cells.size(); id++) {
            const auto& cell = cells[(size_t)id];
            if (!cell.active) continue;
            double d2 = (cell.center - zone.local_center).lengthSq();
            if (d2 < bestD2) {
                bestD2 = d2;
                nearest = id;
            }
        }
        if (nearest != VoxelPhysics::kInvalidVoxel) {
            zone.local_center = cells[(size_t)nearest].center;
        }
    }

    static void collectZoneCandidates(const VoxelPhysics::VesselVoxelModel& model,
                                      const DeformableZoneComponent& zone,
                                      std::vector<std::pair<double, VoxelPhysics::VoxelId>>& candidates) {
        const auto& cells = model.getCells();
        candidates.clear();
        double r2 = zone.radius * zone.radius;
        for (VoxelPhysics::VoxelId id = 0; id < (VoxelPhysics::VoxelId)cells.size(); id++) {
            const auto& cell = cells[(size_t)id];
            if (!cell.active) continue;
            double d2 = (cell.center - zone.local_center).lengthSq();
            if (d2 <= r2) candidates.emplace_back(d2, id);
        }

        if (candidates.empty()) {
            for (VoxelPhysics::VoxelId id = 0; id < (VoxelPhysics::VoxelId)cells.size(); id++) {
                const auto& cell = cells[(size_t)id];
                if (!cell.active) continue;
                double d2 = (cell.center - zone.local_center).lengthSq();
                candidates.emplace_back(d2, id);
            }
            std::sort(candidates.begin(), candidates.end(),
                [](const auto& a, const auto& b) { return a.first < b.first; });
            double expandedR2 = (zone.radius * 2.0) * (zone.radius * 2.0);
            if (candidates.size() > 128) candidates.resize(128);
            if (!candidates.empty() && candidates.front().first > expandedR2) {
                candidates.resize(std::min<size_t>(64, candidates.size()));
            } else {
                std::vector<std::pair<double, VoxelPhysics::VoxelId>> filtered;
                for (const auto& c : candidates) {
                    if (c.first <= expandedR2) filtered.push_back(c);
                }
                if (!filtered.empty()) candidates = std::move(filtered);
            }
        }
    }

    static void initializeZone(VoxelPhysics::VesselVoxelModel& model,
                               DeformableZoneComponent& zone,
                               double dt) {
        zone.nodes.clear();
        zone.constraints.clear();
        zone.initialized = true;
        zone.ready_to_rechunk = false;
        zone.age = 0.0;
        zone.stable_time = 0.0;

        const auto& cells = model.getCells();
        if (cells.empty()) {
            zone.ready_to_rechunk = true;
            return;
        }

        std::vector<std::pair<double, VoxelPhysics::VoxelId>> candidates;
        collectZoneCandidates(model, zone, candidates);
        if (candidates.empty()) {
            snapZoneCenterToNearestCell(model, zone);
            collectZoneCandidates(model, zone, candidates);
        }

        model.applyRadialDamage(zone.local_center, zone.radius, zone.impulse);
        if (zone.hard_impact && zone.impulse > 0.0) {
            model.applyRadialDamage(zone.local_center, zone.radius * 0.65, zone.impulse * 0.35);
        }

        if (candidates.empty()) {
            collectZoneCandidates(model, zone, candidates);
        }
        if (candidates.empty()) {
            zone.ready_to_rechunk = true;
            return;
        }

        std::sort(candidates.begin(), candidates.end(),
            [](const auto& a, const auto& b) { return a.first < b.first; });
        if (candidates.size() > 2048) candidates.resize(2048);

        std::vector<int> voxelToNode(cells.size(), -1);
        zone.nodes.reserve(candidates.size());
        double pinRadius = zone.radius * 0.78;
        for (const auto& candidate : candidates) {
            VoxelPhysics::VoxelId id = candidate.second;
            const auto& cell = cells[(size_t)id];
            double distance = std::sqrt(std::max(0.0, candidate.first));
            double falloff = std::clamp(1.0 - distance / std::max(0.001, zone.radius), 0.0, 1.0);

            DeformableZoneNode node;
            node.voxel = id;
            node.rest_local = cell.center;
            node.position = cell.center;
            Vec3 dir = (cell.center - zone.local_center).normalized();
            if (dir.lengthSq() < 1e-10) dir = Vec3(0.0, -1.0, 0.0);
            double kick = std::clamp((zone.impulse / std::max(1.0, cell.mass)) * 0.002 * falloff, 0.0, 25.0);
            node.velocity = dir * kick;
            node.previous_position = node.position - node.velocity * dt;
            node.inv_mass = cell.mass > 0.0 ? 1.0 / cell.mass : 0.0;
            node.pinned = distance > pinRadius;
            voxelToNode[(size_t)id] = (int)zone.nodes.size();
            zone.nodes.push_back(node);
        }

        if (zone.nodes.size() <= 4) {
            for (auto& node : zone.nodes) node.pinned = false;
        }

        auto& bonds = model.getBonds();
        for (VoxelPhysics::BondId bondId = 0; bondId < (VoxelPhysics::BondId)bonds.size(); bondId++) {
            auto& bond = bonds[(size_t)bondId];
            if (bond.broken || bond.a == VoxelPhysics::kInvalidVoxel || bond.b == VoxelPhysics::kInvalidVoxel) continue;
            int nodeA = voxelToNode[(size_t)bond.a];
            int nodeB = voxelToNode[(size_t)bond.b];
            if (nodeA < 0 || nodeB < 0) continue;

            Vec3 mid = (cells[(size_t)bond.a].center + cells[(size_t)bond.b].center) * 0.5;
            double distance = (mid - zone.local_center).length();
            double falloff = std::clamp(1.0 - distance / std::max(0.001, zone.radius), 0.0, 1.0);
            double impactDamage = zone.impulse * falloff / std::max(5000.0, bond.toughness * 0.05);
            bond.damage += impactDamage;
            if (bond.damage >= 1.0) {
                bond.broken = true;
                continue;
            }

            DeformableZoneConstraint constraint;
            constraint.bond = bondId;
            constraint.node_a = nodeA;
            constraint.node_b = nodeB;
            constraint.rest_length = std::max(0.001, bond.rest_length);
            constraint.compliance = 1.0 / std::max(1.0e5, bond.shear_strength);
            constraint.break_strain = std::clamp(0.10 + bond.toughness / 2.0e6, 0.12, 0.35);
            zone.constraints.push_back(constraint);
        }
    }

    static void solveZone(VoxelPhysics::VesselVoxelModel& model,
                          DeformableZoneComponent& zone,
                          double dt,
                          double simTime) {
        if (zone.nodes.empty()) {
            zone.ready_to_rechunk = true;
            return;
        }

        zone.age = std::max(0.0, simTime - zone.spawn_sim_time);
        for (auto& node : zone.nodes) {
            if (node.pinned) {
                node.previous_position = node.position;
                node.velocity = Vec3(0.0, 0.0, 0.0);
                continue;
            }
            node.previous_position = node.position;
            node.position += node.velocity * dt;
            node.velocity *= 0.985;
        }

        auto& bonds = model.getBonds();
        int iterations = std::max(1, zone.solver_iterations);
        for (int iter = 0; iter < iterations; iter++) {
            for (auto& c : zone.constraints) {
                if (c.bond == VoxelPhysics::kInvalidBond || bonds[(size_t)c.bond].broken) continue;
                DeformableZoneNode& a = zone.nodes[(size_t)c.node_a];
                DeformableZoneNode& b = zone.nodes[(size_t)c.node_b];
                Vec3 delta = b.position - a.position;
                double length = delta.length();
                if (length < 1e-8) continue;

                double wA = a.pinned ? 0.0 : a.inv_mass;
                double wB = b.pinned ? 0.0 : b.inv_mass;
                double wSum = wA + wB;
                if (wSum <= 0.0) continue;

                double alpha = c.compliance / (dt * dt);
                double constraintValue = length - c.rest_length;
                double dlambda = (-constraintValue - alpha * c.lambda) / (wSum + alpha);
                c.lambda += dlambda;
                Vec3 correction = (delta / length) * dlambda;
                if (!a.pinned) a.position -= correction * wA;
                if (!b.pinned) b.position += correction * wB;
            }
        }

        zone.max_node_speed = 0.0;
        auto& cells = model.getCells();
        for (auto& node : zone.nodes) {
            if (!node.pinned) {
                node.velocity = (node.position - node.previous_position) / dt;
                node.velocity *= 0.82;
            }
            zone.max_node_speed = std::max(zone.max_node_speed, node.velocity.length());
            if (node.voxel != VoxelPhysics::kInvalidVoxel) {
                cells[(size_t)node.voxel].center = node.position;
            }
        }

        for (auto& c : zone.constraints) {
            if (c.bond == VoxelPhysics::kInvalidBond || bonds[(size_t)c.bond].broken) continue;
            const DeformableZoneNode& a = zone.nodes[(size_t)c.node_a];
            const DeformableZoneNode& b = zone.nodes[(size_t)c.node_b];
            double strain = std::abs((b.position - a.position).length() - c.rest_length) / c.rest_length;
            bonds[(size_t)c.bond].damage += std::max(0.0, strain / std::max(0.01, c.break_strain)) * 0.02;
            if (strain > c.break_strain || bonds[(size_t)c.bond].damage >= 1.0) {
                model.breakBond(c.bond);
                c.bond = VoxelPhysics::kInvalidBond;
            }
        }

        if (zone.max_node_speed < 0.25) {
            zone.stable_time += dt;
        } else {
            zone.stable_time = 0.0;
        }

        const bool timedOut = zone.age >= zone.max_age;
        const bool settled = zone.stable_time >= zone.settle_time;
        if (zone.hard_impact ? timedOut : (settled || timedOut)) {
            model.rebuildSurfaces();
            zone.ready_to_rechunk = true;
        }
    }
};
