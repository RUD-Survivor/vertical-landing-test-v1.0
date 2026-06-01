#pragma once
// ==========================================================
// manual_input_system.h — 手动输入采集系统
//
// 只作用于 ctx.focused_entity（玩家当前操控的火箭）
// 签名已升级为 update(registry, ctx)，不再接收单个 entity
// ==========================================================

#include "../system_scheduler.h"
#include "core/rocket_state.h"
#include "control/control_system.h"

struct ManualInputSystem : ISystem {
    ManualInputSystem() : ISystem("ManualInput") {}

    bool update(entt::registry& registry, SystemContext& ctx) override {
        if (!ctx.window) return true;
        if (!registry.valid(ctx.focused_entity)) return true;

        auto& guid = registry.get<GuidanceComponent>(ctx.focused_entity);
        auto& control_input = registry.get<ControlInput>(ctx.focused_entity);

        ControlSystem::ManualInputs manual = pollKeys(ctx.window, ctx.cam_mode);

        if (!guid.auto_mode || ctx.mnv_autopilot_active) {
            ControlSystem::UpdateManualControl(registry, ctx.focused_entity, manual, ctx.real_dt * ctx.time_warp);
        }

        return true;
    }

private:
    static ControlSystem::ManualInputs pollKeys(GLFWwindow* window, int cam_mode) {
        ControlSystem::ManualInputs manual;
        bool shift = glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS
                  || glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS;
        bool ctrl  = glfwGetKey(window, GLFW_KEY_LEFT_CONTROL) == GLFW_PRESS
                  || glfwGetKey(window, GLFW_KEY_RIGHT_CONTROL) == GLFW_PRESS;

        manual.throttle_up   = shift || glfwGetKey(window, GLFW_KEY_KP_ADD) == GLFW_PRESS
                                    || glfwGetKey(window, GLFW_KEY_RIGHT_BRACKET) == GLFW_PRESS;
        manual.throttle_down = ctrl || glfwGetKey(window, GLFW_KEY_KP_SUBTRACT) == GLFW_PRESS
                                    || glfwGetKey(window, GLFW_KEY_LEFT_BRACKET) == GLFW_PRESS;
        manual.throttle_max  = glfwGetKey(window, GLFW_KEY_Z) == GLFW_PRESS;
        manual.throttle_min  = glfwGetKey(window, GLFW_KEY_X) == GLFW_PRESS;

        bool free_cam = (cam_mode == 3);
        manual.yaw_left   = free_cam ? (glfwGetKey(window, GLFW_KEY_LEFT)  == GLFW_PRESS)
                                     : (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS || glfwGetKey(window, GLFW_KEY_LEFT) == GLFW_PRESS);
        manual.yaw_right  = free_cam ? (glfwGetKey(window, GLFW_KEY_RIGHT) == GLFW_PRESS)
                                     : (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS || glfwGetKey(window, GLFW_KEY_RIGHT) == GLFW_PRESS);
        manual.pitch_up   = free_cam ? (glfwGetKey(window, GLFW_KEY_UP)    == GLFW_PRESS)
                                     : (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS || glfwGetKey(window, GLFW_KEY_UP) == GLFW_PRESS);
        manual.pitch_down = free_cam ? (glfwGetKey(window, GLFW_KEY_S)     == GLFW_PRESS)
                                     : (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS || glfwGetKey(window, GLFW_KEY_DOWN) == GLFW_PRESS);
        manual.roll_left  = free_cam ? false : glfwGetKey(window, GLFW_KEY_Q) == GLFW_PRESS;
        manual.roll_right = free_cam ? false : glfwGetKey(window, GLFW_KEY_E) == GLFW_PRESS;

        return manual;
    }
};
