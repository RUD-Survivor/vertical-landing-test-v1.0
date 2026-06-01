#pragma once
// ==========================================================
// system_scheduler.h — ECS 系统调度器
// 
// 告别 SimulationController 上帝对象！
// 每个系统声明自己读/写哪些组件，调度器自动管理执行顺序。
//
// 设计原则：
// 1. ISystem：每个系统的抽象接口
// 2. SystemScheduler：按声明顺序执行，未来可做依赖图分析
// 3. SystemContext：系统间共享的帧上下文（避免全局变量）
// ==========================================================

#include <entt/entt.hpp>
#include <vector>
#include <memory>
#include <string>
#include <functional>
#include <GLFW/glfw3.h>

// 前向声明：避免全局依赖
struct FlightHUD;

// ---- 帧上下文：系统间共享的数据（非 ECS 组件） ----
struct SystemContext {
    GLFWwindow* window = nullptr;
    double real_dt = 0.02;       // 统一全局物理时钟——所有实体共用，绝不独立
    int cam_mode = 0;            // 相机模式
    int time_warp = 1;           // 时间加速倍率
    bool mnv_autopilot_active = false; // 变轨自动执行中

    // 帧内共享引用（由场景注入）
    FlightHUD* hud = nullptr;

    // 焦点实体：玩家当前操控/观察的火箭（ManualInput/ManeuverExec/HUD 只看这个）
    entt::entity focused_entity = entt::null;
};

// ---- 系统基类 ----
// 重要：系统不再接收单个 entity 参数。
// 每个系统通过 registry.view<T...>() 自己找到需要处理的实体。
// 这样 1 个实体和 1000 个实体对系统代码完全透明。
struct ISystem {
    std::string name;
    bool enabled = true;

    ISystem(std::string n) : name(std::move(n)) {}
    virtual ~ISystem() = default;

    // 返回 false = 请求跳过后续系统
    virtual bool update(entt::registry& registry, SystemContext& ctx) = 0;
};

// ---- 系统调度器 ----
class SystemScheduler {
public:
    // 按添加顺序注册系统
    void add(std::unique_ptr<ISystem> system) {
        systems.push_back(std::move(system));
    }

    // 便捷方法：用 lambda 注册轻量系统
    void addLambda(const std::string& name, std::function<bool(entt::registry&, SystemContext&)> fn) {
        struct LambdaSystem : ISystem {
            std::function<bool(entt::registry&, SystemContext&)> func;
            LambdaSystem(std::string n, decltype(func) f) : ISystem(std::move(n)), func(std::move(f)) {}
            bool update(entt::registry& r, SystemContext& c) override { return func(r, c); }
        };
        systems.push_back(std::make_unique<LambdaSystem>(name, std::move(fn)));
    }

    // 执行全部系统
    void run(entt::registry& registry, SystemContext& ctx) {
        for (auto& sys : systems) {
            if (!sys->enabled) continue;
            if (!sys->update(registry, ctx)) {
                break;
            }
        }
    }

    // 清空所有系统（用于场景退出时重建）
    void clear() { systems.clear(); }

    size_t size() const { return systems.size(); }

private:
    std::vector<std::unique_ptr<ISystem>> systems;
};
