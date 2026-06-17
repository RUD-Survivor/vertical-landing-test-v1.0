#pragma once

#include "core/rocket_state.h"
#include <entt/entt.hpp>

namespace StructuralState {

void initialize(entt::registry& registry, entt::entity entity);
void rebuild(entt::registry& registry, entt::entity entity, bool applyComShift);
const RocketConfig& physicsConfig(entt::registry& registry, entt::entity entity);

} // namespace StructuralState
