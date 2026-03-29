#pragma once
#include <GLFW/glfw3.h>

#include "render/renderer_2d.h"
#include "render/renderer3d.h"
#include "core/rocket_state.h"
#include "simulation/rocket_builder.h"
#include "simulation/factory_system.h"
#include "menu_system.h" // For AgencyState etc.

/**
 * 全局共享状态区 (GameContext)
 * 职责：
 * 1. 跨场景（从一个类到另一个类）传递大对象（比如 Builder 组装完的火箭，要扔给 Flight）。
 * 2. 保存需要贯穿整个游戏生命周期的核心引用（如 GLFWwindow，Renderer 等）。
 */
struct GameContext {
public:
    static GameContext& getInstance() {
        static GameContext instance;
        BuilderState builder_state;
        RocketState loaded_state;
        ControlInput loaded_input;
        return instance;
    }

    // ==== 核心图形接口 (Core Graphics Handles) ====
    GLFWwindow* window = nullptr;
    Renderer* renderer2d = nullptr;
    Renderer3D* renderer3d = nullptr;

    // ==== 跨模式持久化存档状态 (Persistent State) ====
    // 玩家在 Agency 中积攒的资金、物品
    AgencyState agency_state;
    // 玩家在 Factory 中打造的产业链
    FactorySystem factory;

    // ==== 场景交接桥梁 (Scene Transition Payloads) ====
    // 玩家在 Builder 组装好的运载火箭装配方案
    RocketAssembly launch_assembly;
    
    // 是否跳过组装车间 (Load Save 时用到)
    bool skip_builder = false;
    RocketState loaded_rocket_state;
    ControlInput loaded_control_input;

private:
    GameContext() = default;
};
