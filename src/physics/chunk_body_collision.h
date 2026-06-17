#pragma once

#include "core/rocket_state.h"
#include <entt/entt.hpp>

namespace ChunkBodyCollision {

// 刚体 chunk 两两碰撞（debris↔debris、debris↔主火箭），不含地形
void resolveRegistryPairs(entt::registry& registry, double dt, int iterations = 2);

} // namespace ChunkBodyCollision
