"""
Phase 7: ManeuverManager ECS Refactor.
"""
import re

def read(p):
    with open(p, 'r', encoding='utf-8') as f: return f.read()
def write(p, d):
    with open(p, 'w', encoding='utf-8') as f: f.write(d)

MMH = r'c:\antigravity_code\RocketSim3D\src\scene\maneuver_manager.h'
mmh = read(MMH)

# 1. Update implementations signatures
mmh = mmh.replace(
    'void ManeuverManager::update(GLFWwindow* window, RocketState& rocket_state, FlightHUD& hud,',
    'void ManeuverManager::update(GLFWwindow* window, entt::registry& registry, entt::entity entity, FlightHUD& hud,')

mmh = mmh.replace(
    'void ManeuverManager::render(RocketState& rocket_state, FlightHUD& hud, Renderer3D* r3d,',
    'void ManeuverManager::render(entt::registry& registry, entt::entity entity, FlightHUD& hud, Renderer3D* r3d,')

mmh = mmh.replace(
    'void ManeuverManager::updatePopupState(ManeuverNode& node, RocketState& rocket_state, FlightHUD& hud,',
    'void ManeuverManager::updatePopupState(ManeuverNode& node, entt::registry& registry, entt::entity entity, FlightHUD& hud,')

# 2. Inject component extraction at function starts
mmh = mmh.replace(
    'void ManeuverManager::update(GLFWwindow* window, entt::registry& registry, entt::entity entity, FlightHUD& hud,\n                const CameraDirector& cam, double dt, int ww, int wh) {\n        ',
    'void ManeuverManager::update(GLFWwindow* window, entt::registry& registry, entt::entity entity, FlightHUD& hud,\n                const CameraDirector& cam, double dt, int ww, int wh) {\n        auto& mnv = registry.get<ManeuverComponent>(entity);\n        auto& tele = registry.get<TelemetryComponent>(entity);\n        ')

mmh = mmh.replace(
    'void ManeuverManager::render(entt::registry& registry, entt::entity entity, FlightHUD& hud, Renderer3D* r3d, \n                const Mat4& view, const Mat4& proj, float aspect, float earth_r, float cam_dist,\n                double ws_d, double ro_x, double ro_y, double ro_z, double dt) {\n        ',
    'void ManeuverManager::render(entt::registry& registry, entt::entity entity, FlightHUD& hud, Renderer3D* r3d, \n                const Mat4& view, const Mat4& proj, float aspect, float earth_r, float cam_dist,\n                double ws_d, double ro_x, double ro_y, double ro_z, double dt) {\n        auto& mnv = registry.get<ManeuverComponent>(entity);\n        ')

mmh = mmh.replace(
    'void ManeuverManager::updatePopupState(ManeuverNode& node, entt::registry& registry, entt::entity entity, FlightHUD& hud, \n                         const Mat4& view, const Mat4& proj, float aspect, \n                         double ws_d, double ro_x, double ro_y, double ro_z, \n                         float mouse_x, float mouse_y, bool lmb, double dt) {\n        ',
    'void ManeuverManager::updatePopupState(ManeuverNode& node, entt::registry& registry, entt::entity entity, FlightHUD& hud, \n                         const Mat4& view, const Mat4& proj, float aspect, \n                         double ws_d, double ro_x, double ro_y, double ro_z, \n                         float mouse_x, float mouse_y, bool lmb, double dt) {\n        auto& tele = registry.get<TelemetryComponent>(entity);\n        auto& mnv = registry.get<ManeuverComponent>(entity);\n        ')

# 3. Mass replacements for variables
mmh = mmh.replace('rocket_state.maneuvers', 'mnv.maneuvers')
mmh = mmh.replace('rocket_state.selected_maneuver_index', 'mnv.selected_maneuver_index')
mmh = mmh.replace('rocket_state.sim_time', 'tele.sim_time')

write(MMH, mmh)
print('[OK] maneuver_manager.h - ECS Refactored')
