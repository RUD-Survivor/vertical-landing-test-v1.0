#pragma once
// ==========================================================
// stage_manager.h — 多级火箭管理系统 (Multi-Stage Manager)
// 负责处理火箭的分级构建、分离事件以及活动级的参数同步。
// ==========================================================

#include "core/rocket_state.h"
#include <entt/entt.hpp>
#include <vector>

struct RocketAssembly; // 前向声明：指向组装体数据

namespace StageManager {

// -------------------------------------------------------
// BuildStages — 扫描组装体中的分离器 (Decouplers)
// 并根据其位置将零件划分为不同的“分级”。
// -------------------------------------------------------
void BuildStages(const RocketAssembly& assembly, RocketConfig& config);

// -------------------------------------------------------
// SyncActiveConfig — 将当前活动级的性能参数（如推力、比冲）
// 复制到上层 RocketConfig 的速记字段中，便于物理引擎直接读取。
// -------------------------------------------------------
void SyncActiveConfig(RocketConfig& config, int stage_index);

// -------------------------------------------------------
// SeparateStage — 抛弃当前（最底层）的分级，并切换至下一级。
// 这通常对应于玩家按下“空格键”或自动分离逻辑。
// 返回值：是否成功分离。
// -------------------------------------------------------
bool SeparateStage(entt::registry& registry, entt::entity entity);

// -------------------------------------------------------
// IsCurrentStageEmpty — 检查当前活动级的燃料是否已耗尽。
// -------------------------------------------------------
// bool IsCurrentStageEmpty(const RocketState& state); // LEGACY
bool IsCurrentStageEmpty(const PropulsionComponent& prop);

// -------------------------------------------------------
// GetActiveStage — 获取指定索引级的详细配置数据。
// -------------------------------------------------------
const StageConfig& GetActiveStage(const RocketConfig& config, int stage_index);

// -------------------------------------------------------
// GetTotalMassFromStage — 计算从给定级别向上（包含燃料）的总质量。
// 用于精确计算加速度和 Delta-V。
// -------------------------------------------------------
double GetTotalMassFromStage(const RocketConfig& config, const PropulsionComponent& prop, int from_stage);

} // namespace StageManager

