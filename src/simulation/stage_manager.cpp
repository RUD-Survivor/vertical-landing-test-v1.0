#include <entt/entt.hpp>
#include "stage_manager.h"
#include "render/renderer_2d.h"
#include "simulation/rocket_builder.h"
#include <iostream>
#include <cmath>

namespace StageManager {

// 构建分级 (BuildStages)
// 扫描火箭的所有零件，并根据“级间分离器”（Decoupler）来切分不同的物理阶段。
// 这就像是将一个长长的零件列表切成几段，每一段都有自己的质量和发动机属性。
void BuildStages(const RocketAssembly& assembly, RocketConfig& config) {
    config.stage_configs.clear();

    if (assembly.parts.empty()) {
        config.stages = 0;
        return;
    }

    // 从下往上遍历零件，并在遇到分离器时进行切分
    StageConfig current;
    current.part_start_index = 0;
    float thrust_isp_sum = 0;      // 用于计算该级的推力加权平均比冲
    float total_thrust_stage = 0;  // 该级的总推力

    for (int i = 0; i < (int)assembly.parts.size(); i++) {
        const PartDef& def = PART_CATALOG[assembly.parts[i].def_id];

        // 累加该级的物理属性
        current.dry_mass += def.dry_mass;
        current.fuel_capacity += def.fuel_capacity;
        current.height += def.height;
        if (def.diameter > current.diameter) current.diameter = def.diameter;

        // 如果该零件是开启状态的发动机，累加其推力和比冲贡献
        if (def.thrust > 0) {
            total_thrust_stage += def.thrust;
            current.consumption_rate += def.consumption;
            thrust_isp_sum += def.thrust * def.isp;
        }

        // 核心逻辑：检查这个零件是否为“级间分离器”（Decoupler，ID 为 15）
        // 如果是分离器，说明当前“级”已经结束，需要将其打包并开始下一级的计算。
        if (def.category == CAT_STRUCTURAL && assembly.parts[i].def_id == 15) {
            current.thrust = total_thrust_stage;
            // 计算有效比冲 (Effective ISP)：推力加权平均值
            current.specific_impulse = (total_thrust_stage > 0) ? (thrust_isp_sum / total_thrust_stage) : 0;
            // 估算喷嘴总面积（用于空气阻力计算）
            current.nozzle_area = 0.5 * (current.diameter / 3.7);
            current.part_end_index = i + 1;
            config.stage_configs.push_back(current);

            // 重置状态，开始准备下一级 (Next Stage)
            current = StageConfig();
            current.part_start_index = i + 1;
            thrust_isp_sum = 0;
            total_thrust_stage = 0;
        }
    }

    // 处理最后一个（最顶端）的分级
    current.thrust = total_thrust_stage;
    current.specific_impulse = (total_thrust_stage > 0) ? (thrust_isp_sum / total_thrust_stage) : 0;
    current.nozzle_area = 0.5 * (current.diameter / 3.7);
    current.part_end_index = (int)assembly.parts.size();
    config.stage_configs.push_back(current);

    config.stages = (int)config.stage_configs.size();

    // 初始化：将火箭物理参数同步到第 0 级（起飞级）
    SyncActiveConfig(config, 0);
}

// 同步活跃级参数 (SyncActiveConfig)
// 每当分级发生变化（分离或初始化）时，需要更新物理引擎关注的全局变量。
void SyncActiveConfig(RocketConfig& config, int stage_index) {
    if (stage_index < 0 || stage_index >= (int)config.stage_configs.size()) return;

    const StageConfig& sc = config.stage_configs[stage_index];
    config.dry_mass = sc.dry_mass;           // 当前段的干重 (不含燃料)
    config.specific_impulse = sc.specific_impulse; // 发动机比冲
    config.cosrate = sc.consumption_rate;    // 燃料消耗率 (kg/s)
    config.nozzle_area = sc.nozzle_area;     // 喷嘴截面积
    
    // 计算上方级的总质量（所有处于当前活跃级之上的零件，包括其满载燃料）
    // 这对于物理引擎计算整体重力加速度至关重要。
    double upper = 0;
    for (int i = stage_index + 1; i < (int)config.stage_configs.size(); i++) {
        upper += config.stage_configs[i].dry_mass + config.stage_configs[i].fuel_capacity;
    }
    config.upper_stages_mass = upper;

    // 重新计算火箭可见部分的总高度和最大直径
    double total_h = 0;
    double max_d = 0;
    for (int i = stage_index; i < (int)config.stage_configs.size(); i++) {
        total_h += config.stage_configs[i].height;
        if (config.stage_configs[i].diameter > max_d) max_d = config.stage_configs[i].diameter;
    }
    config.height = (float)total_h;
    config.diameter = (float)max_d;
}

// 执行级间分离 (SeparateStage)
// 抛弃已经烧完燃料的底层结构，并将系统的重点转移到下一级发动机。
bool SeparateStage(entt::registry& registry, entt::entity entity) {
    auto& state = registry.get<RocketState>(entity);
    auto& config = registry.get<RocketConfig>(entity);
    // 检查是否已经是最后一级（无法再分离）
    if (state.current_stage >= state.total_stages - 1) {
        std::cout << ">> [STAGING] CANNOT SEPARATE: Already on final stage!" << std::endl;
        return false;
    }

    // 1. 计算被抛弃部分的质量
    const StageConfig& old_stage = config.stage_configs[state.current_stage];
    // 抛弃质量 = 该级的干重 + 剩下的残余燃料
    double jettison = old_stage.dry_mass + state.stage_fuels[state.current_stage];
    state.jettisoned_mass += jettison; // 累加到已抛弃的总质量中

    std::cout << ">> [STAGING] === STAGE " << (state.current_stage + 1)
              << " SEPARATION! === Jettisoned " << (int)jettison << " kg" << std::endl;

    // 2. 推进到下一级
    state.current_stage++;

    // 3. 将全局燃料引用重定向到新级别的油箱
    state.fuel = state.stage_fuels[state.current_stage];

    // 4. 同步新的物理参数（推力、质量属性等）
    SyncActiveConfig(config, state.current_stage);

    // 5. 再次精确更新上方级的剩余总质量
    double upper = 0;
    for (int i = state.current_stage + 1; i < state.total_stages; i++) {
        upper += config.stage_configs[i].dry_mass + state.stage_fuels[i];
    }
    config.upper_stages_mass = upper;

    // UI 提示信息
    std::cout << ">> [STAGING] Now on Stage " << (state.current_stage + 1)
              << "/" << state.total_stages
              << " | Thrust: " << (int)(config.stage_configs[state.current_stage].thrust / 1000) << " kN"
              << " | Fuel: " << (int)state.fuel << " kg" << std::endl;

    state.mission_msg = ">> STAGE " + std::to_string(state.current_stage + 1) + " IGNITION!";

    return true;
}

// 检查当前级燃料是否耗尽
bool IsCurrentStageEmpty(const RocketState& state) {
    if (state.current_stage < 0 || state.current_stage >= (int)state.stage_fuels.size()) return true;
    return state.stage_fuels[state.current_stage] <= 0.0;
}

const StageConfig& GetActiveStage(const RocketConfig& config, int stage_index) {
    return config.stage_configs[stage_index];
}

// 获取全箭动态总质量 (GetTotalMassFromStage)
// 实时计算当前级以及上方所有级的（干重 + 实时燃料）总和。
// 这是牛顿第二定律 (F=ma) 中 'm' 的实时取值。
double GetTotalMassFromStage(const RocketConfig& config, const RocketState& state, int from_stage) {
    double total = 0; 
    for (int i = from_stage; i < (int)config.stage_configs.size(); i++) {
        // 累加该级的结构重量
        total += config.stage_configs[i].dry_mass;
        // 累加该级的当前剩余燃料量
        if (i < (int)state.stage_fuels.size()) {
            total += state.stage_fuels[i];
        }
    }
    return total;
}

// ECS overload for PropulsionComponent
bool IsCurrentStageEmpty(const PropulsionComponent& prop) {
    if (prop.current_stage < 0 || prop.current_stage >= (int)prop.stage_fuels.size()) return false;
    return prop.stage_fuels[prop.current_stage] <= 0.0;
}

} // namespace StageManager
