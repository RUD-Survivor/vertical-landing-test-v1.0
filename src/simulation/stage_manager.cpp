#include "stage_manager.h"
#include "render/renderer_2d.h"
#include "simulation/rocket_builder.h"
#include <iostream>
#include <cmath>

namespace StageManager {

void BuildStages(const RocketAssembly& assembly, RocketConfig& config) {
    config.stage_configs.clear();

    if (assembly.parts.empty()) {
        config.stages = 0;
        return;
    }

    // Scan parts bottom-to-top and split at Decouplers
    StageConfig current;
    current.part_start_index = 0;
    float thrust_isp_sum = 0;
    float total_thrust_stage = 0;

    for (int i = 0; i < (int)assembly.parts.size(); i++) {
        const PartDef& def = PART_CATALOG[assembly.parts[i].def_id];

        current.dry_mass += def.dry_mass;
        current.fuel_capacity += def.fuel_capacity;
        current.height += def.height;
        if (def.diameter > current.diameter) current.diameter = def.diameter;

        if (def.thrust > 0) {
            total_thrust_stage += def.thrust;
            current.consumption_rate += def.consumption;
            thrust_isp_sum += def.thrust * def.isp;
        }

        // Check if this part is a Decoupler (stage boundary)
        if (def.category == CAT_STRUCTURAL && assembly.parts[i].def_id == 15) {
            // Finalize this stage
            current.thrust = total_thrust_stage;
            current.specific_impulse = (total_thrust_stage > 0) ? (thrust_isp_sum / total_thrust_stage) : 0;
            current.nozzle_area = 0.5 * (current.diameter / 3.7);
            current.part_end_index = i + 1;
            config.stage_configs.push_back(current);

            // Reset for next stage
            current = StageConfig();
            current.part_start_index = i + 1;
            thrust_isp_sum = 0;
            total_thrust_stage = 0;
        }
    }

    // Finalize the last (top) stage
    current.thrust = total_thrust_stage;
    current.specific_impulse = (total_thrust_stage > 0) ? (thrust_isp_sum / total_thrust_stage) : 0;
    current.nozzle_area = 0.5 * (current.diameter / 3.7);
    current.part_end_index = (int)assembly.parts.size();
    config.stage_configs.push_back(current);

    config.stages = (int)config.stage_configs.size();

    // Sync top-level config to stage 0 (the bottom stage that fires first)
    SyncActiveConfig(config, 0);
}

void SyncActiveConfig(RocketConfig& config, int stage_index) {
    if (stage_index < 0 || stage_index >= (int)config.stage_configs.size()) return;

    const StageConfig& sc = config.stage_configs[stage_index];
    config.dry_mass = sc.dry_mass;
    config.specific_impulse = sc.specific_impulse;
    config.cosrate = sc.consumption_rate;
    config.nozzle_area = sc.nozzle_area;
    config.height = sc.height;
    config.diameter = sc.diameter;

    // Compute upper stages mass (everything above active stage)
    double upper = 0;
    for (int i = stage_index + 1; i < (int)config.stage_configs.size(); i++) {
        upper += config.stage_configs[i].dry_mass + config.stage_configs[i].fuel_capacity;
    }
    config.upper_stages_mass = upper;

    // Recompute total height and max diameter across remaining stages
    double total_h = 0;
    double max_d = 0;
    for (int i = stage_index; i < (int)config.stage_configs.size(); i++) {
        total_h += config.stage_configs[i].height;
        if (config.stage_configs[i].diameter > max_d) max_d = config.stage_configs[i].diameter;
    }
    config.height = total_h;
    config.diameter = max_d;
}

bool SeparateStage(RocketState& state, RocketConfig& config) {
    if (state.current_stage >= state.total_stages - 1) {
        std::cout << ">> [STAGING] CANNOT SEPARATE: Already on final stage!" << std::endl;
        return false;
    }

    const StageConfig& old_stage = config.stage_configs[state.current_stage];
    double jettison = old_stage.dry_mass + state.stage_fuels[state.current_stage];
    state.jettisoned_mass += jettison;

    std::cout << ">> [STAGING] === STAGE " << (state.current_stage + 1)
              << " SEPARATION! === Jettisoned " << (int)jettison << " kg" << std::endl;

    state.current_stage++;

    // Set the "fuel" shorthand to the new active stage's fuel
    state.fuel = state.stage_fuels[state.current_stage];

    // Sync config to the new active stage
    SyncActiveConfig(config, state.current_stage);

    // Recalculate upper stages mass using actual remaining fuel
    double upper = 0;
    for (int i = state.current_stage + 1; i < state.total_stages; i++) {
        upper += config.stage_configs[i].dry_mass + state.stage_fuels[i];
    }
    config.upper_stages_mass = upper;

    std::cout << ">> [STAGING] Now on Stage " << (state.current_stage + 1)
              << "/" << state.total_stages
              << " | Thrust: " << (int)(config.stage_configs[state.current_stage].thrust / 1000) << " kN"
              << " | Fuel: " << (int)state.fuel << " kg" << std::endl;

    state.mission_msg = ">> STAGE " + std::to_string(state.current_stage + 1) + " IGNITION!";

    return true;
}

bool IsCurrentStageEmpty(const RocketState& state) {
    if (state.current_stage < 0 || state.current_stage >= (int)state.stage_fuels.size()) return true;
    return state.stage_fuels[state.current_stage] <= 0.0;
}

const StageConfig& GetActiveStage(const RocketConfig& config, int stage_index) {
    return config.stage_configs[stage_index];
}

double GetTotalMassFromStage(const RocketConfig& config, const RocketState& state, int from_stage) {
    double total = state.jettisoned_mass > 0 ? 0 : 0; // jettisoned mass is gone
    for (int i = from_stage; i < (int)config.stage_configs.size(); i++) {
        total += config.stage_configs[i].dry_mass;
        if (i < (int)state.stage_fuels.size()) {
            total += state.stage_fuels[i];
        }
    }
    return total;
}

} // namespace StageManager
