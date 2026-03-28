#ifndef CONTROL_SYSTEM_H
#define CONTROL_SYSTEM_H

// ==========================================================
// control_system.h — 火箭控制系统接口
// 定义了手动操作（键盘/手柄）与自动驾驶（Autopilot）的控制逻辑接口。
// ==========================================================

#include "core/rocket_state.h"

namespace ControlSystem {

    // -------------------------------------------------------
    // UpdateAutoPilot — 自动驾驶逻辑更新
    // 根据设定的导航目标（如：保持姿态、自动入轨）来计算推力和扭矩。
    // -------------------------------------------------------
    void UpdateAutoPilot(RocketState& state, const RocketConfig& config, ControlInput& input, double dt);
    
    // -------------------------------------------------------
    // ManualInputs — 手动输入状态结构
    // 这是一个抽象层，用于将底层（如 GLFW）的按键按下状态映射到控制逻辑中。
    // -------------------------------------------------------
    struct ManualInputs {
        bool throttle_up;    // 增加推力 (通常是左 Shift)
        bool throttle_down;  // 减小推力 (通常是左 Ctrl)
        bool throttle_max;   // 直接最大推力 (通常是 Z)
        bool throttle_min;   // 直接切断推力 (通常是 X)
        
        // 姿态控制键 (通常是 WASD 和 QE)
        bool yaw_left;       // 左偏 (A)
        bool yaw_right;      // 右偏 (D)
        bool pitch_up;       // 仰角 (S)
        bool pitch_down;     // 俯角 (W)
        bool roll_left;      // 左滚 (Q)
        bool roll_right;     // 右滚 (E)
    };
    
    // -------------------------------------------------------
    // UpdateManualControl — 手动控制逻辑更新
    // 将玩家的按键输入实时转换为火箭的推力百分比和姿态扭矩。
    // -------------------------------------------------------
    void UpdateManualControl(RocketState& state, const RocketConfig& config, ControlInput& input, const ManualInputs& manual, double dt);
}

#endif // CONTROL_SYSTEM_H

