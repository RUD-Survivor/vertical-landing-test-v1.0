#pragma once
// ==========================================================
// debris_cleanup_system.h — 碎片清理系统
//
// 自动销毁过期/坠毁的非受控实体
// 使用 view<...>(entt::exclude<FullPhysicsTag>)
// ==========================================================

#include "../system_scheduler.h"
#include "core/rocket_state.h"
#include "core/universe_model.h"
#include <iostream>
#include <cmath>
#include <vector>

struct DebrisCleanupSystem : ISystem {
    DebrisCleanupSystem() : ISystem("DebrisCleanup") {}

    bool update(entt::registry& registry, SystemContext& /*ctx*/) override {
        std::vector<entt::entity> toDestroy;

        // 只销毁挂了 PendingDestroy 标签的实体——纯 tag 驱动，无需计时器
        auto view = registry.view<PendingDestroy>(entt::exclude<FullPhysicsTag>);

        for (auto e : view) {
            toDestroy.push_back(e);
        }

        for (auto e : toDestroy) {
            if (registry.valid(e)) {
                auto& tag = registry.get<TagComponent>(e);
                std::cout << "[DEBRIS] Destroyed '" << tag.name << "'" << std::endl;
                registry.destroy(e);
            }
        }
        return true;
    }
};
