#pragma once
// ==========================================================
// time_warp_system.h — 时间加速系统
//
// 读取键盘 1-8 切换时间倍率
// 仅当 focused_entity 处于安全状态时才允许超级加速
// 签名：update(registry, ctx)
// ==========================================================

#include "../system_scheduler.h"
#include "core/rocket_state.h"
#include <iostream>

struct TimeWarpSystem : ISystem {
    TimeWarpSystem() : ISystem("TimeWarp") {}

    bool update(entt::registry& registry, SystemContext& ctx) override {
        GLFWwindow* window = ctx.window;
        if (!window) return true;

        // ---- 基础加速 (1-4) ----
        if (glfwGetKey(window, GLFW_KEY_1) == GLFW_PRESS || glfwGetKey(window, GLFW_KEY_KP_1) == GLFW_PRESS) ctx.time_warp = 1;
        if (glfwGetKey(window, GLFW_KEY_2) == GLFW_PRESS || glfwGetKey(window, GLFW_KEY_KP_2) == GLFW_PRESS) ctx.time_warp = 10;
        if (glfwGetKey(window, GLFW_KEY_3) == GLFW_PRESS || glfwGetKey(window, GLFW_KEY_KP_3) == GLFW_PRESS) ctx.time_warp = 100;
        if (glfwGetKey(window, GLFW_KEY_4) == GLFW_PRESS || glfwGetKey(window, GLFW_KEY_KP_4) == GLFW_PRESS) ctx.time_warp = 1000;

        // ---- 安全检测：基于焦点实体 ----
        if (registry.valid(ctx.focused_entity)) {
            auto& tele = registry.get<TelemetryComponent>(ctx.focused_entity);
            auto& guid = registry.get<GuidanceComponent>(ctx.focused_entity);
            auto& prop = registry.get<PropulsionComponent>(ctx.focused_entity);

            double surface_speed = std::sqrt(tele.velocity * tele.velocity + tele.local_vx * tele.local_vx);
            bool is_parked = (guid.status == PRE_LAUNCH) && surface_speed < 0.1;
            bool can_super_warp = (!guid.auto_mode && (prop.thrust_power <= 0.01 || prop.fuel <= 0)
                                   && (tele.altitude > 100000.0 || is_parked));

            // 自动驾驶时限制最高加速
            if (guid.auto_mode && ctx.time_warp > 1000) ctx.time_warp = 1;

            // 超级加速 (5-8)
            if (!guid.auto_mode) {
                if (can_super_warp) {
                    if (glfwGetKey(window, GLFW_KEY_5) == GLFW_PRESS || glfwGetKey(window, GLFW_KEY_KP_5) == GLFW_PRESS) {
                        ctx.time_warp = 10000;
                        std::cout << "WARP: 10K SPEED ENGAGED!" << std::endl;
                    }
                    if (glfwGetKey(window, GLFW_KEY_6) == GLFW_PRESS || glfwGetKey(window, GLFW_KEY_KP_6) == GLFW_PRESS) {
                        ctx.time_warp = 100000;
                        std::cout << "WARP: 100K SPEED ENGAGED!" << std::endl;
                    }
                    if (glfwGetKey(window, GLFW_KEY_7) == GLFW_PRESS || glfwGetKey(window, GLFW_KEY_KP_7) == GLFW_PRESS) {
                        ctx.time_warp = 1000000;
                        std::cout << "WARP: 1M SPEED ENGAGED!" << std::endl;
                    }
                    if (glfwGetKey(window, GLFW_KEY_8) == GLFW_PRESS || glfwGetKey(window, GLFW_KEY_KP_8) == GLFW_PRESS) {
                        ctx.time_warp = 10000000;
                        std::cout << "WARP: 10M SPEED ENGAGED!" << std::endl;
                    }
                } else if (ctx.time_warp > 1000) {
                    ctx.time_warp = 1;
                    std::cout << ">> WARP DISENGAGED: Unsafe conditions (Alt < 100km or Engine Active)" << std::endl;
                }
            }
        }

        return true;
    }
};
