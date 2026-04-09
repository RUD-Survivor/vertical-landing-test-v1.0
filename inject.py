import re
import os

BLOCK = """
        auto& trans = world.get<TransformComponent>(rocket_entity);
        auto& vel   = world.get<VelocityComponent>(rocket_entity);
        auto& att   = world.get<AttitudeComponent>(rocket_entity);
        auto& prop  = world.get<PropulsionComponent>(rocket_entity);
        auto& tele  = world.get<TelemetryComponent>(rocket_entity);
        auto& guid  = world.get<GuidanceComponent>(rocket_entity);
        auto& mnv   = world.get<ManeuverComponent>(rocket_entity);
        auto& orb   = world.get<OrbitComponent>(rocket_entity);
"""

def inject(file_path, function_signature):
    with open(file_path, 'r', encoding='utf-8') as f:
        content = f.read()
    
    # Simple search for the signature and insert block right after the opening brace
    content = content.replace(function_signature, function_signature + BLOCK)
    
    with open(file_path, 'w', encoding='utf-8') as f:
        f.write(content)

# flight_scene.h -> onEnter()
inject(r'c:\antigravity_code\RocketSim3D\src\scene\flight_scene.h', 'void onEnter(int save_slot = -1) override {')

# flight_scene.h -> update()
inject(r'c:\antigravity_code\RocketSim3D\src\scene\flight_scene.h', 'void update(double dt) override {\n        real_dt = dt;')

# hud_manager.h -> HUDManager(Renderer* renderer, entt::registry& registry, entt::entity entity, CameraDirector& cam, FlightHUD& hud, double dt, int frame, double ws_d, const Quat& rocketQuat, const Vec3& rocketUp, const Vec3& localNorth, const Vec3& localRight, const Mat4& viewMat, const Mat4& macroProjMat, const Vec3& camEye_rel, double mouse_x, double mouse_y, bool lmb, bool lmb_prev, bool rmb, ManeuverManager& mnvManager, SimulationController& sim_ctrl, Renderer3D* r3d) {
# Wait, hud_manager is basically just one big inline constructor
path = r'c:\antigravity_code\RocketSim3D\src\scene\hud_manager.h'
with open(path, 'r', encoding='utf-8') as f:
    c = f.read()
    if 'auto& rocket_config = registry.get<RocketConfig>(entity);' in c:
        c = c.replace('auto& rocket_config = registry.get<RocketConfig>(entity);', 
"""
        auto& trans = registry.get<TransformComponent>(entity);
        auto& vel   = registry.get<VelocityComponent>(entity);
        auto& att   = registry.get<AttitudeComponent>(entity);
        auto& prop  = registry.get<PropulsionComponent>(entity);
        auto& tele  = registry.get<TelemetryComponent>(entity);
        auto& guid  = registry.get<GuidanceComponent>(entity);
        auto& mnv   = registry.get<ManeuverComponent>(entity);
        auto& orb   = registry.get<OrbitComponent>(entity);
        auto& rocket_config = registry.get<RocketConfig>(entity);
""")
    with open(path, 'w', encoding='utf-8') as f:
        f.write(c)

# orbit_system.h -> render
path = r'c:\antigravity_code\RocketSim3D\src\scene\orbit_system.h'
with open(path, 'r', encoding='utf-8') as f:
    c = f.read()
    sig = 'void render(entt::registry& registry, entt::entity rocket_entity, FlightHUD& hud, ManeuverManager& mnvManager, Renderer3D* r3d, const CameraDirector& cam, const Mat4& viewMat, const Mat4& projMat, float aspect, double ws_d, double ro_x, double ro_y, double ro_z, int ww, int wh, double dt, int current_soi_index, float earth_r, float cam_dist, const Vec3& renderRocketBase, const Vec3& camEye_rel) {'
    c = c.replace(sig, sig + "\n" + BLOCK.replace("world.get", "registry.get"))
    with open(path, 'w', encoding='utf-8') as f:
        f.write(c)

print("Done injections")
