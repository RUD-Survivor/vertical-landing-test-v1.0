#include "structural_state.h"

#include "physics/voxel/vessel_voxel_model.h"
#include "scene/game_context.h"
#include "simulation/rocket_builder.h"
#include "simulation/stage_manager.h"
#include <algorithm>
#include <cmath>

namespace StructuralState {
namespace {

static Vec3 computeComFiltered(const RocketAssembly& assembly,
                             const std::vector<uint8_t>& partActive) {
    double totalMass = 0.0;
    Vec3 weighted(0, 0, 0);
    for (int i = 0; i < (int)assembly.parts.size(); i++) {
        if (i >= (int)partActive.size() || !partActive[(size_t)i]) continue;
        const PlacedPart& pp = assembly.parts[(size_t)i];
        if (pp.def_id < 0 || pp.def_id >= PART_CATALOG_SIZE) continue;
        const PartDef& def = PART_CATALOG[pp.def_id];
        double partMass = (def.dry_mass + def.fuel_capacity) * std::max(1, pp.symmetry);
        if (partMass <= 0.0) continue;

        Vec3 localCom = pp.pos + Vec3(0, def.height * 0.5, 0);
        if (pp.symmetry > 1) {
            localCom.x = 0.0;
            localCom.z = 0.0;
        }
        totalMass += partMass;
        weighted = weighted + localCom * partMass;
    }
    if (totalMass <= 0.0) return Vec3(0, 0, 0);
    return weighted * (1.0 / totalMass);
}

static double computeBoundsBottomFiltered(const RocketAssembly& assembly,
                                          const std::vector<uint8_t>& partActive) {
    double minY = 0.0;
    bool found = false;
    for (int i = 0; i < (int)assembly.parts.size(); i++) {
        if (i >= (int)partActive.size() || !partActive[(size_t)i]) continue;
        const PlacedPart& pp = assembly.parts[(size_t)i];
        if (pp.def_id < 0 || pp.def_id >= PART_CATALOG_SIZE) continue;
        double y = pp.pos.y;
        if (!found || y < minY) {
            minY = y;
            found = true;
        }
    }
    return found ? minY : 0.0;
}

static void rebuildStageConfigsFiltered(const RocketAssembly& assembly,
                                        const std::vector<uint8_t>& partActive,
                                        RocketConfig& config) {
    config.stage_configs.clear();
    if (assembly.parts.empty()) {
        config.stages = 0;
        return;
    }

    StageConfig current;
    current.part_start_index = 0;
    double thrustIspSum = 0.0;
    double totalThrustStage = 0.0;

    for (int i = 0; i < (int)assembly.parts.size(); i++) {
        const PlacedPart& pp = assembly.parts[(size_t)i];
        if (pp.def_id < 0 || pp.def_id >= PART_CATALOG_SIZE) continue;
        const PartDef& def = PART_CATALOG[pp.def_id];
        const bool active = i < (int)partActive.size() && partActive[(size_t)i];

        if (active) {
            int sym = std::max(1, pp.symmetry);
            current.dry_mass += def.dry_mass * sym;
            current.fuel_capacity += def.fuel_capacity * sym;
            current.height += def.height;
            if (def.diameter > current.diameter) current.diameter = def.diameter;
            if (def.thrust > 0.0) {
                totalThrustStage += def.thrust * sym;
                current.consumption_rate += def.consumption * sym;
                thrustIspSum += def.thrust * sym * def.isp;
            }
        }

        if (def.category == CAT_STRUCTURAL && pp.def_id == 15) {
            current.thrust = totalThrustStage;
            current.specific_impulse = (totalThrustStage > 0.0) ? (thrustIspSum / totalThrustStage) : 0.0;
            current.nozzle_area = 0.5 * (current.diameter / 3.7);
            current.part_end_index = i + 1;
            config.stage_configs.push_back(current);

            current = StageConfig();
            current.part_start_index = i + 1;
            thrustIspSum = 0.0;
            totalThrustStage = 0.0;
        }
    }

    current.thrust = totalThrustStage;
    current.specific_impulse = (totalThrustStage > 0.0) ? (thrustIspSum / totalThrustStage) : 0.0;
    current.nozzle_area = 0.5 * (current.diameter / 3.7);
    current.part_end_index = (int)assembly.parts.size();
    config.stage_configs.push_back(current);
    config.stages = (int)config.stage_configs.size();

    for (int i = 0; i < (int)config.stage_configs.size(); i++) {
        StageConfig& stage = config.stage_configs[i];
        stage.aero_profile = assembly.buildAerodynamicProfileFiltered(
            partActive, stage.part_start_index, stage.part_end_index);
        stage.active_aero_profile = assembly.buildAerodynamicProfileFiltered(
            partActive, stage.part_start_index, (int)assembly.parts.size());
    }
}

static void syncEffectiveActiveStage(RocketConfig& config, int stageIndex) {
    if (stageIndex < 0 || stageIndex >= (int)config.stage_configs.size()) return;

    const StageConfig& sc = config.stage_configs[(size_t)stageIndex];
    config.dry_mass = sc.dry_mass;
    config.specific_impulse = sc.specific_impulse;
    config.cosrate = sc.consumption_rate;
    config.nozzle_area = sc.nozzle_area;
    config.aero_profile = sc.active_aero_profile;

    double upper = 0.0;
    for (int i = stageIndex + 1; i < (int)config.stage_configs.size(); i++) {
        upper += config.stage_configs[(size_t)i].dry_mass
               + config.stage_configs[(size_t)i].fuel_capacity;
    }
    config.upper_stages_mass = upper;

    double totalH = 0.0;
    double maxD = 0.0;
    for (int i = stageIndex; i < (int)config.stage_configs.size(); i++) {
        totalH += config.stage_configs[(size_t)i].height;
        maxD = std::max(maxD, config.stage_configs[(size_t)i].diameter);
    }
    config.height = (float)totalH;
    config.diameter = (float)maxD;
}

static double computeFuelCapacityFiltered(const RocketAssembly& assembly,
                                          const std::vector<uint8_t>& partActive,
                                          int partStart) {
    double capacity = 0.0;
    partStart = std::max(0, partStart);
    for (int i = partStart; i < (int)assembly.parts.size(); i++) {
        if (i >= (int)partActive.size() || !partActive[(size_t)i]) continue;
        const PlacedPart& pp = assembly.parts[(size_t)i];
        if (pp.def_id < 0 || pp.def_id >= PART_CATALOG_SIZE) continue;
        const PartDef& def = PART_CATALOG[pp.def_id];
        if (def.fuel_capacity > 0.0f) {
            capacity += def.fuel_capacity * std::max(1, pp.symmetry);
        }
    }
    return capacity;
}

static void derivePartActiveFromVoxels(const VoxelPhysics::VesselVoxelModel& model,
                                       int partCount,
                                       std::vector<uint8_t>& partActive) {
    partActive.assign((size_t)std::max(0, partCount), 0);
    const auto& cells = model.getCells();
    for (VoxelPhysics::VoxelId id = 0; id < (VoxelPhysics::VoxelId)cells.size(); id++) {
        const auto& cell = cells[(size_t)id];
        if (!cell.active || cell.part_index < 0 || cell.part_index >= partCount) continue;
        partActive[(size_t)cell.part_index] = 1;
    }
}

} // namespace

void initialize(entt::registry& registry, entt::entity entity) {
    auto& structural = registry.emplace_or_replace<StructuralStateComponent>(entity);
    const RocketAssembly& assembly = GameContext::getInstance().launch_assembly;
    const auto& baseConfig = registry.get<RocketConfig>(entity);
    const auto& prop = registry.get<PropulsionComponent>(entity);

    structural.part_active.assign(assembly.parts.size(), 1);
    structural.active_part_count = (int)assembly.parts.size();
    structural.com_local = Vec3(0, 0, 0);
    structural.structurally_damaged = false;
    structural.effective_config = baseConfig;
    structural.effective_fuel_capacity = computeFuelCapacityFiltered(
        assembly, structural.part_active, 0);
    structural.initialized = true;

    if (registry.all_of<RigidChunkComponent>(entity)) {
        auto& chunk = registry.get<RigidChunkComponent>(entity);
        chunk.mass = std::max(1.0, baseConfig.dry_mass + prop.fuel + baseConfig.upper_stages_mass);
    }
    (void)prop;
}

void rebuild(entt::registry& registry, entt::entity entity, bool applyComShift) {
    if (!registry.all_of<RocketConfig, PropulsionComponent>(entity)) return;

    auto& structural = registry.get_or_emplace<StructuralStateComponent>(entity);
    const RocketAssembly& assembly = GameContext::getInstance().launch_assembly;
    const auto& baseConfig = registry.get<RocketConfig>(entity);
    auto& prop = registry.get<PropulsionComponent>(entity);

    const int partCount = (int)assembly.parts.size();
    if (!structural.initialized) {
        initialize(registry, entity);
        structural = registry.get<StructuralStateComponent>(entity);
    }

    structural.part_active.assign((size_t)partCount, 1);
    structural.structurally_damaged = false;

    if (registry.all_of<VoxelBodyComponent>(entity)) {
        const auto& voxel = registry.get<VoxelBodyComponent>(entity);
        if (voxel.model && voxel.structure_fractured) {
            derivePartActiveFromVoxels(*voxel.model, partCount, structural.part_active);
            structural.structurally_damaged = true;
        }
    }

    structural.active_part_count = 0;
    for (uint8_t flag : structural.part_active) {
        if (flag) structural.active_part_count++;
    }

    Vec3 newCom = computeComFiltered(assembly, structural.part_active);
    if (applyComShift && registry.all_of<TransformComponent, AttitudeComponent>(entity)) {
        Vec3 delta = newCom - structural.com_local;
        if (delta.lengthSq() > 1e-12) {
            auto& trans = registry.get<TransformComponent>(entity);
            auto& att = registry.get<AttitudeComponent>(entity);
            Vec3 worldDelta = att.attitude.rotate(delta);
            trans.px += worldDelta.x;
            trans.py += worldDelta.y;
            trans.pz += worldDelta.z;
        }
    }
    structural.com_local = newCom;

    structural.effective_config = baseConfig;
    if (structural.structurally_damaged) {
        rebuildStageConfigsFiltered(assembly, structural.part_active, structural.effective_config);
        syncEffectiveActiveStage(structural.effective_config, prop.current_stage);
        structural.effective_config.bounds_bottom = computeBoundsBottomFiltered(
            assembly, structural.part_active);
    }

    int partStart = 0;
    if (prop.current_stage >= 0
        && prop.current_stage < (int)structural.effective_config.stage_configs.size()) {
        partStart = structural.effective_config.stage_configs[(size_t)prop.current_stage].part_start_index;
    }
    structural.effective_fuel_capacity = computeFuelCapacityFiltered(
        assembly, structural.part_active, partStart);

    if (structural.effective_fuel_capacity >= 0.0) {
        prop.fuel = std::min(prop.fuel, structural.effective_fuel_capacity);
        if (prop.current_stage >= 0 && prop.current_stage < (int)prop.stage_fuels.size()) {
            prop.stage_fuels[(size_t)prop.current_stage] =
                std::min(prop.stage_fuels[(size_t)prop.current_stage], prop.fuel);
        }
    }

    if (registry.all_of<RigidChunkComponent>(entity)) {
        auto& chunk = registry.get<RigidChunkComponent>(entity);
        double structuralMass = structural.effective_config.dry_mass
                              + structural.effective_config.upper_stages_mass;
        chunk.mass = std::max(1.0, structuralMass + prop.fuel);
        chunk.local_center_of_mass = structural.com_local;
        if (registry.all_of<VoxelBodyComponent>(entity)) {
            const auto& voxel = registry.get<VoxelBodyComponent>(entity);
            if (voxel.model && !chunk.owned_voxels.empty()) {
                Vec3 mn(1e30, 1e30, 1e30);
                Vec3 mx(-1e30, -1e30, -1e30);
                const auto& cells = voxel.model->getCells();
                for (VoxelPhysics::VoxelId id : chunk.owned_voxels) {
                    if (id < 0 || id >= (VoxelPhysics::VoxelId)cells.size()) continue;
                    const auto& cell = cells[(size_t)id];
                    mn.x = std::min(mn.x, cell.center.x);
                    mn.y = std::min(mn.y, cell.center.y);
                    mn.z = std::min(mn.z, cell.center.z);
                    mx.x = std::max(mx.x, cell.center.x);
                    mx.y = std::max(mx.y, cell.center.y);
                    mx.z = std::max(mx.z, cell.center.z);
                }
                if (mx.x >= mn.x) {
                    chunk.local_bounds_min = mn - structural.com_local;
                    chunk.local_bounds_max = mx - structural.com_local;
                }
            }
        }
    }
}

const RocketConfig& physicsConfig(entt::registry& registry, entt::entity entity) {
    if (registry.all_of<StructuralStateComponent>(entity)) {
        const auto& structural = registry.get<StructuralStateComponent>(entity);
        if (structural.initialized && structural.structurally_damaged) {
            return structural.effective_config;
        }
    }
    return registry.get<RocketConfig>(entity);
}

} // namespace StructuralState
