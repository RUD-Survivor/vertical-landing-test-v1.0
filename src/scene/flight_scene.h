#pragma once
extern float g_scroll_y;
#include "core/universe_model.h"
#include "scene.h"
#include "save_system.h"
#include "render/part_renderer.h"
#include "game_context.h"
#include "simulation/simulation_controller.h"  // [兼容层] vk_game_ui 仍引用 sim_ctrl
#include "ecs/system_scheduler.h"
#include "ecs/systems/time_warp_system.h"
#include "ecs/systems/manual_input_system.h"
#include "ecs/systems/maneuver_exec_system.h"
#include "ecs/systems/physics_pipeline_system.h"
#include "ecs/systems/voxel_structure_system.h"
#include "ecs/systems/voxel_deformable_zone_system.h"
#include "ecs/systems/voxel_fracture_system.h"
#include "ecs/systems/voxel_chunk_physics_system.h"
#include "ecs/systems/voxel_chunk_collision_system.h"
#include "ecs/systems/debris_physics_system.h"
#include "ecs/systems/debris_cleanup_system.h"
#include "camera/camera_director.h"
#include "simulation/orbit_physics.h"
#include "simulation/predictor.h"
#include "simulation/center_calculator.h"
#include "simulation/structural_state.h"
#include "input/input_router.h"
#include "math/math3d.h"
#include "render/HUD_system.h"
#include "render/scene_snapshot.h"
#include "physics/voxel/vessel_voxel_model.h"
#include <entt/entt.hpp>
#include <algorithm>
#include <cmath>
#include <unordered_set>


// Any other includes you need
#include "flight_input_system.h"
#include "maneuver_manager.h"
#include "orbit_system.h"
#include "plume_manager.h"
#include "spaceport_manager.h"
#include "celestial_renderer.h"
#include "rocket_visuals.h"
#include "render/cloud_tuner.h"
#include "environment_system.h"
#include "hud_manager.h"
#include "render_context.h"
#include "orbit_system_vulkan.h"

namespace {

bool computePartKeptBounds(const VoxelPhysics::VesselVoxelModel& model,
                           const std::unordered_set<VoxelPhysics::VoxelId>& keepVoxels,
                           int partIndex,
                           Vec3& outMin,
                           Vec3& outMax) {
    const auto& cells = model.getCells();
    bool found = false;
    outMin = Vec3(1e30, 1e30, 1e30);
    outMax = Vec3(-1e30, -1e30, -1e30);
    double half = model.voxelSize() * 0.5;
    for (VoxelPhysics::VoxelId vid : keepVoxels) {
        if (vid < 0 || vid >= (VoxelPhysics::VoxelId)cells.size()) continue;
        const auto& cell = cells[(size_t)vid];
        if (!cell.active || cell.part_index != partIndex) continue;
        found = true;
        outMin.x = std::min(outMin.x, cell.center.x - half);
        outMin.y = std::min(outMin.y, cell.center.y - half);
        outMin.z = std::min(outMin.z, cell.center.z - half);
        outMax.x = std::max(outMax.x, cell.center.x + half);
        outMax.y = std::max(outMax.y, cell.center.y + half);
        outMax.z = std::max(outMax.z, cell.center.z + half);
    }
    return found;
}

void appendVoxelSolidDraws(std::vector<RocketPartDraw>& out,
                           const VoxelPhysics::VesselVoxelModel& model,
                           const std::vector<VoxelPhysics::VoxelId>& voxelIds,
                           const Vec3& chunkCom,
                           const Vec3& anchorRender,
                           const Quat& worldRot,
                           float ws_d,
                           float r, float g, float b,
                           bool offsetFromChunkCom,
                           bool includeInactive) {
    const auto& cells = model.getCells();
    if (voxelIds.empty()) return;

    float vs = (float)(model.voxelSize() * ws_d);

    for (VoxelPhysics::VoxelId vid : voxelIds) {
        if (vid < 0 || vid >= (VoxelPhysics::VoxelId)cells.size()) continue;
        const auto& cell = cells[(size_t)vid];
        if (!cell.active && !includeInactive) continue;

        Vec3 localPos = cell.center;
        if (offsetFromChunkCom) localPos = localPos - chunkCom;
        Vec3 wp = anchorRender + worldRot.rotate(localPos * ws_d);

        RocketPartDraw rp;
        Mat4 m = Mat4::translate(wp) * Mat4::fromQuat(worldRot) * Mat4::scale(Vec3(vs, vs, vs));
        m.toFloatArray(rp.model);
        rp.r = r;
        rp.g = g;
        rp.b = b;
        rp.meshType = 2;
        out.push_back(rp);
    }
}

void appendChunkSolidDraws(std::vector<RocketPartDraw>& out,
                           const VoxelPhysics::VesselVoxelModel& model,
                           const VoxelPhysics::VoxelChunkSummary& chunk,
                           const Vec3& anchorRender,
                           const Quat& worldRot,
                           float ws_d,
                           float r, float g, float b,
                           bool offsetFromChunkCom) {
    appendVoxelSolidDraws(out, model, chunk.voxels, chunk.center_of_mass,
                          anchorRender, worldRot, ws_d, r, g, b,
                          offsetFromChunkCom, false);
}

void appendSinglePartDraws(std::vector<RocketPartDraw>& out,
                           const std::map<std::string, Mesh>& partObjMeshes,
                           const RocketAssembly& assembly,
                           int pi,
                           const Vec3& anchorRender,
                           const Quat& worldRot,
                           float ws_d,
                           const Vec3* cropMin = nullptr,
                           const Vec3* cropMax = nullptr) {
    if (pi < 0 || pi >= (int)assembly.parts.size()) return;
    const PlacedPart& pp = assembly.parts[(size_t)pi];
    if (pp.def_id < 0 || pp.def_id >= PART_CATALOG_SIZE) return;
    const PartDef& def = PART_CATALOG[pp.def_id];

    std::string objPath;
    if (def.model_path) {
        objPath = def.model_path;
    } else {
        std::string tmp = def.name;
        for (char& c : tmp) { c = (char)tolower((unsigned char)c); if (c == ' ') c = '_'; }
        objPath = "assets/models/" + tmp + ".obj";
    }
    auto oit = partObjMeshes.find(objPath);
    const Mesh* objMesh = (oit != partObjMeshes.end() && !oit->second.cpuIndices.empty())
        ? &oit->second : nullptr;

    Vec3 cropCenter = pp.pos;
    float cropScaleX = 1.0f, cropScaleY = 1.0f, cropScaleZ = 1.0f;
    if (cropMin && cropMax) {
        Vec3 keptSize = *cropMax - *cropMin;
        cropCenter = (*cropMin + *cropMax) * 0.5;
        cropScaleX = (float)std::max(0.08, keptSize.x / std::max(0.01f, def.diameter));
        cropScaleY = (float)std::max(0.08, keptSize.y / std::max(0.01f, def.height));
        cropScaleZ = (float)std::max(0.08, keptSize.z / std::max(0.01f, def.diameter));
    }

    constexpr float k2Pi = 6.28318530718f;
    for (int s = 0; s < pp.symmetry; s++) {
        float symAngle = (s * k2Pi) / pp.symmetry;
        Vec3 localPos = cropCenter;
        if (pp.symmetry > 1 && !cropMin) {
            localPos = pp.pos;
            float dist = sqrtf(pp.pos.x * pp.pos.x + pp.pos.z * pp.pos.z);
            if (dist > 0.01f) {
                float ca = atan2f(pp.pos.z, pp.pos.x);
                localPos.x = cosf(ca + symAngle) * dist;
                localPos.z = sinf(ca + symAngle) * dist;
            }
        } else if (pp.symmetry > 1 && cropMin) {
            float dist = sqrtf(cropCenter.x * cropCenter.x + cropCenter.z * cropCenter.z);
            if (dist > 0.01f) {
                float ca = atan2f(cropCenter.z, cropCenter.x);
                localPos.x = cosf(ca + symAngle) * dist;
                localPos.z = sinf(ca + symAngle) * dist;
                localPos.y = cropCenter.y;
            }
        }
        Vec3 wp = anchorRender + worldRot.rotate(localPos * ws_d);
        Quat wr = worldRot * pp.rot * Quat::fromAxisAngle(Vec3(0, 1, 0), symAngle);
        RocketPartDraw rp;
        if (objMesh) {
            float sx = (def.diameter / objMesh->width) * ws_d * cropScaleX;
            float sy = (def.height / objMesh->height) * ws_d * cropScaleY;
            float sz = (def.diameter / objMesh->depth) * ws_d * cropScaleZ;
            Mat4 m = Mat4::translate(wp) * Mat4::fromQuat(wr)
                   * Mat4::scale(Vec3(sx, sy, sz))
                   * Mat4::translate(Vec3(-objMesh->centerX, -objMesh->minY, -objMesh->centerZ));
            m.toFloatArray(rp.model);
            rp.r = def.r; rp.g = def.g; rp.b = def.b;
            rp.meshType = 0;
            rp.meshId = objPath;
        } else {
            float h = def.height * ws_d * cropScaleY;
            float d = def.diameter * ws_d * std::max(cropScaleX, cropScaleZ);
            Mat4 m = Mat4::TRS(wp, wr, Vec3(d, h, d));
            m.toFloatArray(rp.model);
            rp.r = cropMin ? 0.82f : 0.9f;
            rp.g = cropMin ? 0.80f : 0.9f;
            rp.b = cropMin ? 0.78f : 0.9f;
            rp.meshType = (def.category == CAT_NOSE_CONE || def.category == CAT_COMMAND_POD) ? 1 : 0;
        }
        out.push_back(rp);
    }
}

} // namespace

class FlightScene : public IScene {
public:
    // === Core Members ===
    InputRouter input;
    double ws_d = 0.001;
    RenderContext ctx; // 新增的统一参考系上下文
    // Shared between update() and render()
    double mouse_x = 0, mouse_y = 0;
    bool lmb = false, lmb_prev = false, rmb = false;
    // ECS World and Entities
    entt::registry world;
    entt::entity rocket_entity;

    CameraDirector cam;
    Mesh earthMesh;
    Mesh ringMesh;
    Mesh rocketBody;
    Mesh rocketNose;
    Mesh rocketBox;
    // OBJ 外部模型（Vulkan 路径加载后填充）
    Mesh launchPadMesh;
    bool hasLaunchPadOBJ = false;
    std::map<std::string, Mesh> partObjMeshes;  // OBJ 路径 → 已加载 Mesh（空 cpuIndices = 未找到）
    // === ECS 系统调度器（替代 SimulationController 上帝对象） ===
    SystemScheduler scheduler;
    SystemContext sysCtx;
    // [兼容层] vk_game_ui.h 仍通过 sim_ctrl.time_warp 访问时间加速
    SimulationController sim_ctrl;
    double dt = 0.02;
    double real_dt = 0.02;
    int frame = 0;
    // === Variables Extracted from Static ===
    OrbitSystem orbitSystem;
    PlumeManager plumeManager;
    SpaceportManager spaceportManager;
    CelestialRenderer celestialRenderer;
    RocketVisuals rocketVisuals;
    EnvironmentSystem environmentSystem;
    HUDManager hudManager;
    ManeuverManager mnvManager;
    bool terrain_adjusted = false;
    bool comma_prev = false;
    bool period_prev = false;
    FlightInputSystem inputSystem;
    int last_soi = -1;
    bool show_clouds = true;
    int last_save_frame = -1000;  // 自动存档闪烁指示器
    CloudTuner cloudTuner;
    // Vulkan 轨道缓存（update 中计算，extractRenderSnapshot 中消费）
    KepOrbitCache     vkKepCache;
    AdvOrbitCache     vkAdvCache;
    RocketMarkerCache vkRktCache;
    void onEnter() override {


        GameContext& ctx = GameContext::getInstance();
        
        // --- 1. Initialize ECS Entity ---
        rocket_entity = world.create();
        // Emplace legacy bridging components
        auto& rocket_config = world.emplace<RocketConfig>(rocket_entity);
        auto& control_input = world.emplace<ControlInput>(rocket_entity);
        // Emplace new ECS data components
        world.emplace<TransformComponent>(rocket_entity);
        world.emplace<VelocityComponent>(rocket_entity);
        world.emplace<AttitudeComponent>(rocket_entity);
        world.emplace<PropulsionComponent>(rocket_entity);
        world.emplace<TelemetryComponent>(rocket_entity);
        world.emplace<GuidanceComponent>(rocket_entity);
        world.emplace<OrbitComponent>(rocket_entity);
        world.emplace<ManeuverComponent>(rocket_entity);
        world.emplace<VFXComponent>(rocket_entity);
        // 多实体支持：标签组件 + 物理精度
        world.emplace<TagComponent>(rocket_entity, EntityTag::ROCKET, "Player Rocket");
        world.emplace<FullPhysicsTag>(rocket_entity);  // 受控 → FULL 物理
        world.emplace<BreakableBodyTag>(rocket_entity);
        auto& voxelBody = world.emplace<VoxelBodyComponent>(rocket_entity);
        voxelBody.voxel_size = 1.0;
        world.emplace<RigidChunkComponent>(rocket_entity).attached_to_parent = true;
        world.emplace<StructuralStateComponent>(rocket_entity);

        auto& trans = world.get<TransformComponent>(rocket_entity);
        auto& vel   = world.get<VelocityComponent>(rocket_entity);
        auto& att   = world.get<AttitudeComponent>(rocket_entity);
        auto& prop  = world.get<PropulsionComponent>(rocket_entity);
        auto& tele  = world.get<TelemetryComponent>(rocket_entity);
        auto& guid  = world.get<GuidanceComponent>(rocket_entity);
        auto& mnv   = world.get<ManeuverComponent>(rocket_entity);
        auto& orb   = world.get<OrbitComponent>(rocket_entity);
        earthMesh = MeshGen::sphere(256, 512, 1.0f);
        ringMesh = MeshGen::ring(128, 1.11f, 2.35f);
        rocketBody = MeshGen::cylinder(32, 1.0f, 1.0f);
        rocketNose = MeshGen::cone(32, 1.0f, 1.0f);
        rocketBox = MeshGen::box(1.0f, 1.0f, 1.0f);
        spaceportManager.init();
        bool skip_builder = ctx.skip_builder;
        auto& builder_state_assembly = ctx.launch_assembly; // Assume this was the passed payload
        auto& agency_state = ctx.agency_state;
        auto& loaded_state = ctx.loaded_rocket_state;
        auto& loaded_input = ctx.loaded_control_input;
        // --- Original Init Code below ---
// =========================================================
// 4. 准备飞向太空：从组装零件构建物理配置
// 此时我们将静态的 3D 模型转换为具有质量、推力和重心的物理实体。
// 注意：物理引擎不需要知道 3D 模型长什么样，它只需要知道“多重”、“多快”。
// =========================================================
if (!skip_builder && !builder_state_assembly.parts.empty()) {
    // 重新对齐重心 (CoM)：让火箭的坐标系中心刚好在物理重心上，这样转弯才自然。
    Vec3 com = CenterCalculator::calculateCenterOfMass(builder_state_assembly);
    for (auto& p : builder_state_assembly.parts) {
        p.pos = p.pos - com;
    }
    builder_state_assembly.com = Vec3(0, 0, 0); // 重心归零
    builder_state_assembly.recalculate();
}
rocket_config = builder_state_assembly.buildRocketConfig();
// 从机构库存中消耗零件
for (const auto& p : builder_state_assembly.parts) {
    const PartDef& def = PART_CATALOG[p.def_id];
    ItemType it = ITEM_NONE;
    if (def.category == CAT_NOSE_CONE) it = PART_NOSECONE;
    else if (def.category == CAT_COMMAND_POD) it = PART_COMMAND_POD;
    else if (def.category == CAT_FUEL_TANK) it = PART_FUEL_TANK;
    else if (def.category == CAT_ENGINE) it = PART_ENGINE;
    else if (def.category == CAT_BOOSTER) it = PART_FUEL_TANK;
    else if (def.category == CAT_STRUCTURAL) it = PART_STRUCTURAL;
    if (it != ITEM_NONE) {
        agency_state.removeItem(it, 1);
    }
}
if (skip_builder) {
    // ==== 从存档文件加载火箭状态 ====
    bool loaded = SaveSystem::LoadGame(builder_state_assembly, world, rocket_entity, control_input);
    if (loaded) {
        // 重建 rocket_config 以匹配加载的装配
        rocket_config = builder_state_assembly.buildRocketConfig();
        // 确保 stage 同步
        StageManager::SyncActiveConfig(rocket_config, prop.current_stage);
        // 将 assemble 回写到 GameContext（供后续渲染等引用）
        ctx.launch_assembly = builder_state_assembly;
        guid.mission_msg = "存档已加载 - 飞行继续";
        cout << "[SAVE] Loaded saved game: " << builder_state_assembly.parts.size()
             << " parts, fuel=" << (int)prop.fuel << " kg, sim_time=" << tele.sim_time << "s" << endl;
    } else {
        // 存档加载失败，回退到新游戏初始化
        cout << "[SAVE] Failed to load save file, starting new game." << endl;
        prop.fuel = builder_state_assembly.total_fuel;
        guid.status = PRE_LAUNCH;
        guid.mission_msg = "READY ON PAD - PRESS SPACE TO LAUNCH";
        prop.total_stages = rocket_config.stages;
        prop.current_stage = 0;
        prop.stage_fuels.clear();
        for (int i = 0; i < (int)rocket_config.stage_configs.size(); i++) {
            prop.stage_fuels.push_back(rocket_config.stage_configs[i].fuel_capacity);
        }
        if (!prop.stage_fuels.empty()) prop.fuel = prop.stage_fuels[0];
        // Fallback surface coordinate initialization
        double lat_rad = trans.launch_latitude * PI / 180.0;
        double lon_rad = trans.launch_longitude * PI / 180.0;
        float lowest_y = 0.0f;
        if (!builder_state_assembly.parts.empty()) {
            lowest_y = 1e10f;
            for (const auto& p : builder_state_assembly.parts)
                lowest_y = std::min(lowest_y, (float)p.pos.y);
        }
        double R = UniverseModel::getInstance().solar_system[UniverseModel::getInstance().current_soi_index].radius - (double)lowest_y;
        trans.surf_px = R * cos(lat_rad) * cos(lon_rad);
        trans.surf_py = R * cos(lat_rad) * sin(lon_rad);
        trans.surf_pz = R * sin(lat_rad);
        trans.launch_site_px = trans.surf_px;
        trans.launch_site_py = trans.surf_py;
        trans.launch_site_pz = trans.surf_pz;
        CelestialBody& body = UniverseModel::getInstance().solar_system[UniverseModel::getInstance().current_soi_index];
        double theta = body.prime_meridian_epoch;
        Quat rot = Quat::fromAxisAngle(Vec3(0, 0, 1), (float)theta);
        Quat tilt = Quat::fromAxisAngle(Vec3(1, 0, 0), (float)body.axial_tilt);
        Quat full_rot = tilt * rot;
        Vec3 world_pos = full_rot.rotate(Vec3((float)trans.surf_px, (float)trans.surf_py, (float)trans.surf_pz));
        trans.px = (double)world_pos.x;
        trans.py = (double)world_pos.y;
        trans.pz = (double)world_pos.z;
    }
}
else {
    // 新游戏初始化
    prop.fuel = builder_state_assembly.total_fuel;
    guid.status = PRE_LAUNCH;
    guid.mission_msg = "READY ON PAD - PRESS SPACE TO LAUNCH";
    // Initialize multi-stage fuel distribution
    prop.total_stages = rocket_config.stages;
    prop.current_stage = 0;
    prop.stage_fuels.clear();
    for (int i = 0; i < (int)rocket_config.stage_configs.size(); i++) {
        prop.stage_fuels.push_back(rocket_config.stage_configs[i].fuel_capacity);
    }
    // Set initial fuel to stage 0’s capacity
    if (!prop.stage_fuels.empty()) {
        prop.fuel = prop.stage_fuels[0];
    }
    // Calculate initial surface coordinates from launch latitude/longitude
    double lat_rad = trans.launch_latitude * PI / 180.0;
    double lon_rad = trans.launch_longitude * PI / 180.0;
    float lowest_y = 0.0f;
    if (!builder_state_assembly.parts.empty()) {
        lowest_y = 1e10f;
        for (const auto& p : builder_state_assembly.parts) {
            lowest_y = std::min(lowest_y, (float)p.pos.y);
        }
    }
    // Distance from planet center to CoM
    double R = UniverseModel::getInstance().solar_system[UniverseModel::getInstance().current_soi_index].radius - (double)lowest_y;
    // Z is the North-South axis, XY is the equatorial plane
    trans.surf_px = R * cos(lat_rad) * cos(lon_rad);
    trans.surf_py = R * cos(lat_rad) * sin(lon_rad);
    trans.surf_pz = R * sin(lat_rad);
    // Store fixed launch site for pad rendering
    trans.launch_site_px = trans.surf_px;
    trans.launch_site_py = trans.surf_py;
    trans.launch_site_pz = trans.surf_pz;
    // Initialize inertial coordinates immediately for the first frame
    CelestialBody& body = UniverseModel::getInstance().solar_system[UniverseModel::getInstance().current_soi_index];
    double theta = body.prime_meridian_epoch; // sim_time = 0
    Quat rot = Quat::fromAxisAngle(Vec3(0, 0, 1), (float)theta);
    Quat tilt = Quat::fromAxisAngle(Vec3(1, 0, 0), (float)body.axial_tilt);
    Quat full_rot = tilt * rot;
    Vec3 world_pos = full_rot.rotate(Vec3((float)trans.surf_px, (float)trans.surf_py, (float)trans.surf_pz));
    trans.px = (double)world_pos.x;
    trans.py = (double)world_pos.y;
    trans.pz = (double)world_pos.z;
    }

    StructuralState::initialize(world, rocket_entity);
    
    // ======== INPUT ROUTER INITIALIZATION ========
    inputSystem.setup(input, rocket_config, control_input, hudManager.hud, cam, show_clouds, world, rocket_entity, cloudTuner);
    // Keep a reference to the assembly for rendering
    const RocketAssembly& assembly = builder_state_assembly;
     // 飞行模式相机控制器
    // --- Galaxy Info UI State ---
    cout << ">> ROCKET ASSEMBLED! (" << assembly.parts.size() << " parts)" << endl;
    cout << ">>   Dry Mass: " << (int)assembly.total_dry_mass << " kg" << endl;
    cout << ">>   Fuel: " << (int)assembly.total_fuel << " kg" << endl;
    cout << ">>   Height: " << (int)assembly.total_height << " m" << endl;
    cout << ">>   ISP: " << (int)assembly.avg_isp << " s" << endl;
    cout << ">>   Delta-V: " << (int)assembly.total_delta_v << " m/s" << endl;
    cout << ">>   TWR: " << assembly.twr << endl;
    cout << ">> PRESS [SPACE] TO LAUNCH!" << endl;
    cout << ">> [TAB] Toggle Auto/Manual | [WASD] Thrust & Attitude" << endl;
    cout << ">> [Z] Full Throttle | [X] Kill Throttle | [1-4] Time Warp" << endl;
    cout << ">> [O] Toggle Orbit Display" << endl;



     // 50Hz 物理步长
     // Tab 防抖
    // 记录火箭历史飞行的 3D 轨迹点
    // --- 5. 核心飞行模拟循环 (The Main Flight Loop) ---
    // 这里是游戏最核心的部分：物理计算、姿态控制、轨道绘图都在这里执行。

    // ======== ECS 系统调度器注册 ========
    // 按执行顺序注册：时间加速 → 手动输入 → 变轨执行 → 刚体物理 → 局部变形 → 断裂重聚合 → 碎片物理
    scheduler.add(std::make_unique<TimeWarpSystem>());
    scheduler.add(std::make_unique<ManualInputSystem>());
    scheduler.add(std::make_unique<ManeuverExecSystem>());
    scheduler.add(std::make_unique<VoxelStructureSystem>());
    scheduler.add(std::make_unique<PhysicsPipelineSystem>());
    scheduler.add(std::make_unique<VoxelDeformableZoneSystem>());
    scheduler.add(std::make_unique<VoxelFractureSystem>());
    scheduler.add(std::make_unique<VoxelChunkPhysicsSystem>());
    scheduler.add(std::make_unique<VoxelChunkCollisionSystem>());
    scheduler.add(std::make_unique<DebrisPhysicsSystem>());
    scheduler.add(std::make_unique<DebrisCleanupSystem>());
    sysCtx.focused_entity = rocket_entity;  // 玩家焦点：ManualInput/ManeuverExec 只看这个
    // =======================================
    }




    void update(double dt) override {
        auto& trans = world.get<TransformComponent>(rocket_entity);
        auto& vel   = world.get<VelocityComponent>(rocket_entity);
        auto& att   = world.get<AttitudeComponent>(rocket_entity);
        auto& prop  = world.get<PropulsionComponent>(rocket_entity);
        auto& tele  = world.get<TelemetryComponent>(rocket_entity);
        auto& guid  = world.get<GuidanceComponent>(rocket_entity);
        auto& mnv   = world.get<ManeuverComponent>(rocket_entity);
        auto& orb   = world.get<OrbitComponent>(rocket_entity);


        real_dt = dt;

        auto& rocket_config = world.get<RocketConfig>(rocket_entity);
        auto& control_input = world.get<ControlInput>(rocket_entity);


        const RocketAssembly& assembly = GameContext::getInstance().launch_assembly;
        Renderer3D* r3d = GameContext::getInstance().renderer3d;
        // Limit spikes
        frame++;
        // Scene escape logic handled externally or here
        if (glfwGetKey(GameContext::getInstance().GameContext::getInstance().window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
            glfwSetWindowShouldClose(GameContext::getInstance().window, true);
        int ww, wh;
        glfwGetWindowSize(GameContext::getInstance().window, &ww, &wh);
        // Maneuver popup deferred rendering state (computed in 3D pass, rendered in 2D HUD pass)
        // Advanced Orbit Settings

        // --- 鼠标轨道控制 (CameraDirector) ---
        {
            double mx, my;
            glfwGetCursorPos(GameContext::getInstance().window, &mx, &my);
            bool rmb = glfwGetMouseButton(GameContext::getInstance().window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS;
            bool lmb = glfwGetMouseButton(GameContext::getInstance().window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS;
            cam.handleMouseInput(rmb, lmb, mx, my);
        }
        // 自由视角 EQ 键旋转 (Roll)
        cam.handleFreeCamRoll(glfwGetKey(GameContext::getInstance().window, GLFW_KEY_Q) == GLFW_PRESS,
            glfwGetKey(GameContext::getInstance().window, GLFW_KEY_E) == GLFW_PRESS);
        // 滚轮缩放 (CameraDirector)
        if (g_scroll_y != 0.0f) {
            cam.handleScroll(g_scroll_y);
            g_scroll_y = 0.0f;
        }
        // --- Ensure Rocket starts perfectly on the Terrain/SVO surface ---
        if (r3d && r3d->terrain) {
            Vec3 localUp(trans.surf_px, trans.surf_py, trans.surf_pz);
            localUp = localUp.normalized();
            Quat unalign_from_z = Quat::fromAxisAngle(Vec3(1.0f, 0.0f, 0.0f), (float)(PI / 2.0));
            Vec3 qLocal = unalign_from_z.rotate(localUp);
            float terrH = r3d->terrain->getHeight(qLocal);
            tele.terrain_altitude = (double)terrH * 1000.0; // km to meters
            if (guid.status == PRE_LAUNCH && !terrain_adjusted) {
                double platform_height = 8.5; // Thickness of the visual launch platform
                double rocket_R = EARTH_RADIUS + tele.terrain_altitude - rocket_config.bounds_bottom + platform_height;
                double pad_R = EARTH_RADIUS + tele.terrain_altitude + platform_height;
                trans.surf_px = localUp.x * rocket_R;
                trans.surf_py = localUp.y * rocket_R;
                trans.surf_pz = localUp.z * rocket_R;
                trans.launch_site_px = localUp.x * pad_R;
                trans.launch_site_py = localUp.y * pad_R;
                trans.launch_site_pz = localUp.z * pad_R;
                terrain_adjusted = true;
                cout << "[TERRAIN] Adjusted launchpad altitude by " << tele.terrain_altitude << " meters to match SVO bounds." << endl;
            }
        }
        // SVO Automation and Input Polling
        inputSystem.poll(GameContext::getInstance().window, input, world, rocket_entity, r3d);
        // --- ECS 系统调度：时间加速 → 输入采集 → 变轨执行 → 物理管线 ---
        sysCtx.window = GameContext::getInstance().window;
        sysCtx.real_dt = real_dt;
        sysCtx.cam_mode = cam.mode;
        sysCtx.hud = &hudManager.hud;
        // 从兼容层同步：vk_game_ui 可能在上帧修改了 time_warp（如 warp-to-node）
        sysCtx.time_warp = sim_ctrl.time_warp;
        scheduler.run(world, sysCtx);
        // 同步回兼容层：供 vk_game_ui 显示/读取
        sim_ctrl.time_warp = sysCtx.time_warp;
        // 轨迹记录（Vulkan 兼容：每帧采样火箭位置用于轨道丝带）
        orbitSystem.recordTrajectory(world, rocket_entity, ctx.earth_r);
        // Update mouse state for HUD
        {
            double mx_raw, my_raw;
            glfwGetCursorPos(GameContext::getInstance().window, &mx_raw, &my_raw);
            int _ww, _wh;
            glfwGetWindowSize(GameContext::getInstance().window, &_ww, &_wh);
            lmb = glfwGetMouseButton(GameContext::getInstance().window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS;
            rmb = glfwGetMouseButton(GameContext::getInstance().window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS;
            mouse_x = mx_raw / _ww * 2.0 - 1.0;
            mouse_y = 1.0 - my_raw / _wh * 2.0;
        }

        // ==== 自动存档（每 300 帧 = 约 6 秒）====
        if (frame % 300 == 0 && guid.status != PRE_LAUNCH) {
            SaveSystem::SaveGame(assembly, world, rocket_entity, control_input);
            last_save_frame = frame;
        }
    }
    void render() override {
        auto& trans = world.get<TransformComponent>(rocket_entity);
        auto& vel   = world.get<VelocityComponent>(rocket_entity);
        auto& att   = world.get<AttitudeComponent>(rocket_entity);
        auto& prop  = world.get<PropulsionComponent>(rocket_entity);
        auto& tele  = world.get<TelemetryComponent>(rocket_entity);
        auto& guid  = world.get<GuidanceComponent>(rocket_entity);
        auto& mnv   = world.get<ManeuverComponent>(rocket_entity);
        auto& orb   = world.get<OrbitComponent>(rocket_entity);


        auto& rocket_config = world.get<RocketConfig>(rocket_entity);
        auto& control_input = world.get<ControlInput>(rocket_entity);


        // Vulkan: renderer2d 已移除；HUD 由 ImGui/VkHUD 处理
        Renderer3D* r3d = GameContext::getInstance().renderer3d;
        const RocketAssembly& assembly = GameContext::getInstance().launch_assembly;
        // ================= 3D 渲染通道 =================
        {
            // === 1. 构建本帧渲染参考系上下文 (浮动原点、物理变换、投影矩阵) ===
            ctx.update(trans, vel, att, tele, guid, rocket_config, UniverseModel::getInstance().current_soi_index, last_soi, comma_prev, period_prev, cam, ws_d, dt);
            // 更新 day_blend / alt_factor（extractRenderSnapshot() 消费）
            environmentSystem.updateAndApply(ctx.camEye_rel, ctx.ro_x, ctx.ro_y, ctx.ro_z, ws_d, cam.mode);
            // 轨道计算（仅 Panorama 模式）
            if (cam.mode == 2) {
                computeOrbitDataVK(orbitSystem, world, rocket_entity, hudManager.hud, cam,
                    ws_d, ctx.ro_x, ctx.ro_y, ctx.ro_z, ctx.earth_r, ctx.cam_dist,
                    ctx.renderRocketBase, ctx.camEye_rel, vkKepCache, vkAdvCache, vkRktCache);
            }
            // 实际渲染由 main.cpp 中的 renderVulkanFrame() 通过 extractRenderSnapshot() 驱动
            return;
        }
    }

    // ========================================================================
    // extractRenderSnapshot — 从当前帧提取纯数据快照，供 Vulkan 渲染器消费
    //
    // 调用前必须已执行 ctx.update()（在 render() 开头自动完成）
    // 返回的 Snapshot 不持有任何 GL/Vulkan 资源，可跨线程传递
    // ========================================================================
    SceneSnapshot extractRenderSnapshot() {
        SceneSnapshot snap;

        // ---- 1. 相机 ----
        ctx.viewMat.toFloatArray(snap.view);
        snap.camPos[0] = (float)ctx.camEye_rel.x;
        snap.camPos[1] = (float)ctx.camEye_rel.y;
        snap.camPos[2] = (float)ctx.camEye_rel.z;
        snap.time = (float)glfwGetTime();
        snap.frameIndex = frame;

        // ---- 2. 光照方向 ----
        float ldlen = sqrtf(ctx.lightVec.x*ctx.lightVec.x +
                            ctx.lightVec.y*ctx.lightVec.y +
                            ctx.lightVec.z*ctx.lightVec.z);
        if (ldlen > 1e-6f) {
            snap.lightDir[0] = ctx.lightVec.x / ldlen;
            snap.lightDir[1] = ctx.lightVec.y / ldlen;
            snap.lightDir[2] = ctx.lightVec.z / ldlen;
        } else {
            snap.lightDir[0] = 0.577f; snap.lightDir[1] = 0.577f; snap.lightDir[2] = 0.577f;
        }

        // ---- 3. 屏幕宽高比 ----
        int ww, wh;
        glfwGetFramebufferSize(GameContext::getInstance().window, &ww, &wh);
        snap.aspect = (float)ww / (float)wh;

        // ---- 投影矩阵（匹配 celestialRenderer 的 smart near/far，覆盖 ctx.macroProjMat 的 FOV 错误） ----
        {
            double _ws_d = ws_d, _ro_x = ctx.ro_x, _ro_y = ctx.ro_y, _ro_z = ctx.ro_z;
            float far_plane = 1000.0f * 149597870.0f;
            float closest_dist = far_plane;
            auto& ss = UniverseModel::getInstance().solar_system;
            for (size_t i = 1; i < ss.size(); i++) {
                float px=(float)(ss[i].px*_ws_d-_ro_x), py=(float)(ss[i].py*_ws_d-_ro_y), pz=(float)(ss[i].pz*_ws_d-_ro_z);
                float br = (float)(ss[i].radius * _ws_d);
                float d = sqrtf((px-snap.camPos[0])*(px-snap.camPos[0])+
                                (py-snap.camPos[1])*(py-snap.camPos[1])+
                                (pz-snap.camPos[2])*(pz-snap.camPos[2])) - br;
                if (d < closest_dist) closest_dist = d;
            }
            {
                float d = sqrtf((ctx.renderSun.x-snap.camPos[0])*(ctx.renderSun.x-snap.camPos[0])+
                                (ctx.renderSun.y-snap.camPos[1])*(ctx.renderSun.y-snap.camPos[1])+
                                (ctx.renderSun.z-snap.camPos[2])*(ctx.renderSun.z-snap.camPos[2]))
                          - (float)ctx.sun_radius;
                if (d < closest_dist) closest_dist = d;
            }
            float macro_near = fmaxf(0.00001f, closest_dist * 0.05f);
            macro_near = fminf(macro_near, 1.0f);
            if (closest_dist < 1.0f) macro_near = fminf(macro_near, 0.0001f);
            macro_near = fmaxf(macro_near, (float)ctx.cam_dist * 0.0001f);
            // Orbit/Chase: 相机紧贴火箭，近平面必须小于火箭距离，防止火箭被裁
            if (cam.mode <= 1) {
                macro_near = fminf(macro_near, (float)ctx.cam_dist * 0.1f);
            }
            Mat4 projMat = Mat4::perspective(0.8, snap.aspect, (double)macro_near, (double)far_plane);
            projMat.toFloatArray(snap.proj);
        }

        // ---- 4. 天体（行星/卫星，跳过太阳） ----
        auto& ss = UniverseModel::getInstance().solar_system;
        double ro_x = ctx.ro_x, ro_y = ctx.ro_y, ro_z = ctx.ro_z;
        auto& tele = world.get<TelemetryComponent>(rocket_entity);
        double simT = tele.sim_time;
        constexpr double kPI = 3.14159265358979323846;

        static const char* kShaderKey[] = {
            "", "mercury","venus","earth","moon","mars",
            "jupiter","saturn","uranus","neptune"
        };
        static const int kAtmoIdx[] = { 0, 0, 2, 3, 0, 5, 6, 7, 8, 9 };

        for (size_t i = 1; i < ss.size(); i++) {
            auto& b = ss[i];
            float r  = (float)(b.radius * ws_d);
            float px = (float)(b.px * ws_d - ro_x);
            float py = (float)(b.py * ws_d - ro_y);
            float pz = (float)(b.pz * ws_d - ro_z);

            Quat alignZ = Quat::fromAxisAngle(Vec3(1,0,0), -(float)(kPI/2.0));
            Quat spin   = Quat::fromAxisAngle(Vec3(0,0,1),
                (float)(b.prime_meridian_epoch + simT * 2.0 * kPI / b.rotation_period));
            Quat tiltQ  = Quat::fromAxisAngle(Vec3(1,0,0), (float)b.axial_tilt);
            Quat rot    = tiltQ * spin * alignZ;

            Mat4 model = Mat4::translate(Vec3(px,py,pz)) *
                         Mat4::fromQuat(rot) *
                         Mat4::scale(Vec3(r,r,r));

            CelestialDraw cd;
            model.toFloatArray(cd.model);
            cd.center[0] = px; cd.center[1] = py; cd.center[2] = pz;
            cd.radius = r;
            cd.r = b.r; cd.g = b.g; cd.b = b.b;
            cd.shaderKey = (i < 10) ? kShaderKey[i] : "barren";
            cd.bodyIdx   = (int)i;
            cd.atmoIdx   = (i < 10) ? kAtmoIdx[i] : 0;
            cd.hasRing   = (b.type == RINGED_GAS_GIANT);
            cd.showClouds = show_clouds;
            snap.celestials.push_back(cd);

            // 行星轨道 ribbon（仅 Panorama 模式）
            if (cam.mode == 2 && i < 10) {
                double a = b.sma_base, e = b.ecc_base, inc = b.inc_base;
                double lan = b.lan_base, argP = b.arg_peri_base;
                int segs = 90;
                double cO = cos(lan), sO = sin(lan);
                double cw = cos(argP), sw = sin(argP);
                double ci = cos(inc), si = sin(inc);
                std::vector<Vec3> opts; std::vector<Vec4> ocols;
                for (int k = 0; k < segs; k++) {
                    double Ek = (double)k / (segs-1) * 2.0 * kPI;
                    double nuk = 2.0 * atan2(sqrt(1.0+e)*sin(Ek/2.0), sqrt(1.0-e)*cos(Ek/2.0));
                    double rk = a * (1.0 - e*cos(Ek));
                    double ox = rk*cos(nuk), oy = rk*sin(nuk);
                    double wx = (cO*cw - sO*sw*ci)*ox + (-cO*sw - sO*cw*ci)*oy;
                    double wy = (sO*cw + cO*sw*ci)*ox + (-sO*sw + cO*cw*ci)*oy;
                    double wz = (sw*si)*ox + (cw*si)*oy;
                    if (i == 4) { wx+=ss[3].px; wy+=ss[3].py; wz+=ss[3].pz; }
                    opts.push_back(Vec3((float)(wx*ws_d-ro_x),(float)(wy*ws_d-ro_y),(float)(wz*ws_d-ro_z)));
                    ocols.push_back(Vec4(b.r*0.6f+0.3f, b.g*0.6f+0.3f, b.b*0.6f+0.3f, 0.7f));
                }
                float orbW = fmaxf(ctx.earth_r*0.008f, fmaxf((ctx.camEye_rel-Vec3(px,py,pz)).length(), ctx.camEye_rel.length())*0.0035f);
                if (i==4) orbW*=0.5f;
                snap.ribbons.push_back({opts, ocols, orbW});
            }
        }

        // ---- 5. 火箭 ----
        float er = ctx.earth_r;
        float rh = (ctx.rh    > 0.f) ? ctx.rh    : er * 0.00008f;
        float rw = (ctx.rw_3d > 0.f) ? ctx.rw_3d : er * 0.000008f;
        float rx = (float)ctx.renderRocketBase.x, ry = (float)ctx.renderRocketBase.y, rz = (float)ctx.renderRocketBase.z;

        // Body: translate * scale（Y 轴圆柱）
        snap.rocketBodyModel[0] = rw; snap.rocketBodyModel[5] = rh;
        snap.rocketBodyModel[10]= rw; snap.rocketBodyModel[15]= 1.0f;
        snap.rocketBodyModel[12]= rx; snap.rocketBodyModel[13]= ry; snap.rocketBodyModel[14]= rz;

        // Nose: translate * scale（上半部锥体）
        snap.rocketNoseModel[0] = rw; snap.rocketNoseModel[5] = rw * 2.f;
        snap.rocketNoseModel[10]= rw; snap.rocketNoseModel[15]= 1.0f;
        snap.rocketNoseModel[12]= rx; snap.rocketNoseModel[13]= ry + rh * 0.5f; snap.rocketNoseModel[14]= rz;

        // ---- 6. 排气羽流 ----
        auto& prop   = world.get<PropulsionComponent>(rocket_entity);
        auto& config = world.get<RocketConfig>(rocket_entity);
        auto& input  = world.get<ControlInput>(rocket_entity);
        const RocketAssembly& assembly = GameContext::getInstance().launch_assembly;
        const StructuralStateComponent* structural = world.all_of<StructuralStateComponent>(rocket_entity)
            ? &world.get<StructuralStateComponent>(rocket_entity) : nullptr;
        const RocketConfig& physicsCfg = StructuralState::physicsConfig(world, rocket_entity);

        if (prop.thrust_power >= 0.01 && !assembly.parts.empty()) {
            float thrust      = (float)input.throttle;
            float expansion   = (float)fmax(0.0,
                1.0 - PhysicsSystem::get_air_density(tele.altitude) / 1.225);
            float thrust_scale = 0.25f + 0.75f * powf(thrust, 1.5f);

            int render_start = 0;
            if (prop.current_stage < (int)physicsCfg.stage_configs.size())
                render_start = physicsCfg.stage_configs[prop.current_stage].part_start_index;

            const Quat& rocketQuat      = ctx.rocketQuat;
            const Vec3& renderRocketBase = ctx.renderRocketBase;
            constexpr float kTwoPi      = 6.28318530718f;

            for (int pi = render_start; pi < (int)assembly.parts.size(); pi++) {
                if (!isPartStructurallyActive(structural, pi)) continue;
                const PlacedPart& pp  = assembly.parts[pi];
                const PartDef&    def = PART_CATALOG[pp.def_id];
                if (def.category != CAT_ENGINE) continue;

                for (int s = 0; s < pp.symmetry; s++) {
                    float symAngle = (s * kTwoPi) / pp.symmetry;
                    Vec3 localPos  = pp.pos;
                    if (pp.symmetry > 1) {
                        float dist = sqrtf(pp.pos.x*pp.pos.x + pp.pos.z*pp.pos.z);
                        if (dist > 0.01f) {
                            float ca = atan2f(pp.pos.z, pp.pos.x);
                            localPos.x = cosf(ca + symAngle) * dist;
                            localPos.z = sinf(ca + symAngle) * dist;
                        }
                    }

                    Vec3 nozzlePos = renderRocketBase +
                        rocketQuat.rotate(localPos * (float)ws_d);
                    Quat nozzleRot = rocketQuat * pp.rot
                                   * Quat::fromAxisAngle(Vec3(0,1,0), symAngle);
                    Vec3 nozzleDir = nozzleRot.rotate(Vec3(0,1,0));

                    float groundDist  = (float)tele.altitude * (float)ws_d;
                    float engine_ph   = def.height * (float)ws_d;
                    float plume_len   = engine_ph * 4.2f * thrust_scale * (1.0f + expansion);
                    float plume_dia   = def.diameter * (float)ws_d * 2.0f * (1.1f + expansion * 4.2f);
                    float gcd         = fmaxf(0.f, plume_len - groundDist);
                    float splash      = gcd / fmaxf(0.001f, plume_len);
                    plume_dia        *= (1.0f + splash * 8.0f);

                    Vec3 plumePos = nozzlePos - nozzleDir * (plume_len * 0.5f);
                    Mat4 plumeMdl = Mat4::TRS(plumePos, nozzleRot,
                                              Vec3(plume_dia, plume_len, plume_dia));

                    ExhaustDraw ed;
                    plumeMdl.toFloatArray(ed.model);
                    ed.throttle   = thrust;
                    ed.expansion  = expansion;
                    ed.groundDist = groundDist;
                    ed.plumeLen   = plume_len;
                    snap.plumes.push_back(ed);
                }
            }
        }

        // ---- 7. 太阳屏幕坐标（镜头光晕） ----
        // 用 snap.proj（正确 FOV）而非 ctx.macroProjMat（80.0 弧度 FOV）
        Vec3 sunWorld(ctx.renderSun.x, ctx.renderSun.y, ctx.renderSun.z);
        Mat4 snapProj;
        for (int i = 0; i < 16; i++) snapProj.m[i] = snap.proj[i];
        Mat4 vp = snapProj * ctx.viewMat;
        float cx = vp.m[0]*sunWorld.x + vp.m[4]*sunWorld.y + vp.m[8]*sunWorld.z + vp.m[12];
        float cy = vp.m[1]*sunWorld.x + vp.m[5]*sunWorld.y + vp.m[9]*sunWorld.z + vp.m[13];
        float cz = vp.m[2]*sunWorld.x + vp.m[6]*sunWorld.y + vp.m[10]*sunWorld.z + vp.m[14];
        float cw = vp.m[3]*sunWorld.x + vp.m[7]*sunWorld.y + vp.m[11]*sunWorld.z + vp.m[15];
        if (cw > 0.0f && fabsf(cx/cw) < 3.5f && fabsf(cy/cw) < 3.5f) {
            snap.sunScreen[0] =  cx / cw;
            snap.sunScreen[1] = -(cy / cw); // Vulkan NDC Y 向下，取反映射到屏幕 UV
        } else {
            snap.sunScreen[0] = -3.0f; // 哨兵值：太阳在屏幕外/背后，镜头光晕隐藏
        }
        float sunDist = (sunWorld - ctx.camEye_rel).length();
        // 严格物理平方反比，1AU 归一化=1.0：真实 1AU=149597870.7km（render 单位=km，
        // sun_px=sun_body.px*ws_d 已把米转成 km，见 render_context.h）。
        // 原来这里写的是 149597.87f（少了3个0=全局缩小1000倍）且只做线性衰减（没平方），
        // 是云在地球轨道附近直射光需要把 SunIntensityMul 调到 30000 才有立体感的根本原因：
        // 直射光的量纲比环境光/地面反射低了3个数量级，全靠 mul 硬顶凑。改成真实平方反比后，
        // 地球轨道附近 sunIntensity≈1.0，和其余~O(1)手调光照项回到同一量纲。
        const float kAuRenderKm = 149597870.7f;
        float sunDistClamped = fmaxf(1.0f, sunDist);
        snap.sunIntensity = (kAuRenderKm * kAuRenderKm) / (sunDistClamped * sunDistClamped);
        snap.sunWorldPos[0] = sunWorld.x; snap.sunWorldPos[1] = sunWorld.y; snap.sunWorldPos[2] = sunWorld.z;

        // ---- 8. 天空 + 大气 ----
        // 匹配 OpenGL CelestialRenderer: sky_vibrancy = 1.0 - day_blend * (1 - alt_factor)
        // day_blend=1/alt_factor=1(太空) → vibrancy=1(满星空)
        // day_blend=1/alt_factor=0(地面白天) → vibrancy=0(蓝天遮星)
        snap.skyVibrancy = 1.0f - (float)environmentSystem.day_blend * (1.0f - environmentSystem.alt_factor);
        snap.dayBlend = (float)environmentSystem.day_blend;
        snap.cameraMode = cam.mode;

        // ---- 9. 镜头光晕遮挡体（每个星球中心+半径） ----
        for (size_t i = 1; i < ss.size(); i++) {
            SceneSnapshot::Occluder oc;
            oc.cx = (float)(ss[i].px * ws_d - ro_x);
            oc.cy = (float)(ss[i].py * ws_d - ro_y);
            oc.cz = (float)(ss[i].pz * ws_d - ro_z);
            oc.radius = (float)(ss[i].radius * ws_d);
            snap.occluders.push_back(oc);
        }

        // ---- 10. 火箭零件 + 断裂体素 ----
        if (!assembly.parts.empty()) {
            auto& prop = world.get<PropulsionComponent>(rocket_entity);
            const RocketConfig& physicsCfg = StructuralState::physicsConfig(world, rocket_entity);
            const StructuralStateComponent* structural = world.all_of<StructuralStateComponent>(rocket_entity)
                ? &world.get<StructuralStateComponent>(rocket_entity) : nullptr;
            int render_start = 0;
            if (prop.current_stage < (int)physicsCfg.stage_configs.size())
                render_start = physicsCfg.stage_configs[prop.current_stage].part_start_index;
            const Quat& rq = ctx.rocketQuat;
            const Vec3& rb = ctx.renderRocketBase;

            bool fractured = structural && structural->structurally_damaged;
            int mainActiveChunk = 0;
            std::unordered_set<VoxelPhysics::VoxelId> keepVoxels;
            std::unordered_map<int, int> partTotalVoxels;
            std::unordered_map<int, int> partKeptVoxels;
            const VoxelPhysics::VesselVoxelModel* mainModel = nullptr;

            if (world.all_of<VoxelBodyComponent>(rocket_entity)) {
                const auto& mainVoxel = world.get<VoxelBodyComponent>(rocket_entity);
                if (mainVoxel.model && fractured) {
                    mainModel = mainVoxel.model.get();
                    mainActiveChunk = mainVoxel.active_chunk;
                    const auto& chunks = mainVoxel.model->getChunks();
                    const auto& cells = mainVoxel.model->getCells();
                    if (world.all_of<RigidChunkComponent>(rocket_entity)) {
                        const auto& mainRigid = world.get<RigidChunkComponent>(rocket_entity);
                        for (VoxelPhysics::VoxelId vid : mainRigid.owned_voxels) {
                            keepVoxels.insert(vid);
                        }
                    }
                    if (keepVoxels.empty() && mainActiveChunk >= 0 && mainActiveChunk < (int)chunks.size()) {
                        for (VoxelPhysics::VoxelId vid : chunks[(size_t)mainActiveChunk].voxels) {
                            if (vid >= 0 && vid < (VoxelPhysics::VoxelId)cells.size()) {
                                keepVoxels.insert(vid);
                            }
                        }
                    }
                    for (VoxelPhysics::VoxelId vid = 0; vid < (VoxelPhysics::VoxelId)cells.size(); vid++) {
                        const auto& cell = cells[(size_t)vid];
                        if (!cell.active || cell.part_index < 0) continue;
                        partTotalVoxels[cell.part_index]++;
                        if (keepVoxels.count(vid)) partKeptVoxels[cell.part_index]++;
                    }
                }
            }

            auto isPartFullyKept = [&](int pi) -> bool {
                if (!fractured) return true;
                if (structural) return isPartStructurallyActive(structural, pi);
                auto totalIt = partTotalVoxels.find(pi);
                auto keptIt = partKeptVoxels.find(pi);
                if (totalIt == partTotalVoxels.end() || totalIt->second <= 0) return false;
                return keptIt != partKeptVoxels.end() && keptIt->second == totalIt->second;
            };

            auto isPartPartiallyKept = [&](int pi) -> bool {
                if (!fractured) return false;
                auto totalIt = partTotalVoxels.find(pi);
                auto keptIt = partKeptVoxels.find(pi);
                if (totalIt == partTotalVoxels.end() || totalIt->second <= 0) return false;
                return keptIt != partKeptVoxels.end() && keptIt->second > 0
                    && keptIt->second < totalIt->second;
            };

            for (int pi = render_start; pi < (int)assembly.parts.size(); pi++) {
                if (!isPartStructurallyActive(structural, pi)) continue;
                if (isPartPartiallyKept(pi) && mainModel != nullptr) {
                    Vec3 cropMin, cropMax;
                    if (computePartKeptBounds(*mainModel, keepVoxels, pi, cropMin, cropMax)) {
                        appendSinglePartDraws(
                            snap.rocketParts, partObjMeshes, assembly, pi,
                            rb, rq, (float)ws_d, &cropMin, &cropMax);
                    }
                } else {
                    appendSinglePartDraws(
                        snap.rocketParts, partObjMeshes, assembly, pi,
                        rb, rq, (float)ws_d, nullptr, nullptr);
                }
            }
        }

        // ---- 10a. 断裂 chunk 碎片（按体素实心绘制，不依赖 surface_indices）----
        {
            auto debrisView = world.view<ChunkPhysicsTag, VoxelBodyComponent, RigidChunkComponent,
                                         TransformComponent, AttitudeComponent>();
            for (auto e : debrisView) {
                if (e == rocket_entity) continue;
                const auto& dVoxel = debrisView.get<VoxelBodyComponent>(e);
                const auto& dRigid = debrisView.get<RigidChunkComponent>(e);
                const auto& dTrans = debrisView.get<TransformComponent>(e);
                const auto& dAtt = debrisView.get<AttitudeComponent>(e);
                if (!dVoxel.model) continue;

                Vec3 debrisRb(
                    (float)(dTrans.abs_px * ws_d - ctx.ro_x),
                    (float)(dTrans.abs_py * ws_d - ctx.ro_y),
                    (float)(dTrans.abs_pz * ws_d - ctx.ro_z));

                const std::vector<VoxelPhysics::VoxelId>& drawVoxels = dRigid.owned_voxels;
                if (drawVoxels.empty()) {
                    const auto& chunks = dVoxel.model->getChunks();
                    if (dRigid.chunk_id < 0 || dRigid.chunk_id >= (int)chunks.size()) continue;
                    appendChunkSolidDraws(
                        snap.rocketParts,
                        *dVoxel.model,
                        chunks[(size_t)dRigid.chunk_id],
                        debrisRb,
                        dAtt.attitude,
                        (float)ws_d,
                        0.85f, 0.50f, 0.35f,
                        true);
                } else {
                    appendVoxelSolidDraws(
                        snap.rocketParts,
                        *dVoxel.model,
                        drawVoxels,
                        dRigid.local_center_of_mass,
                        debrisRb,
                        dAtt.attitude,
                        (float)ws_d,
                        0.85f, 0.50f, 0.35f,
                        true,
                        true);
                }
            }
        }

        // ---- 11. 发射台 (Block C) ----
        if (tele.altitude < 12000.0) {
            auto& trans = world.get<TransformComponent>(rocket_entity);
            CelestialBody& body = ss[UniverseModel::getInstance().current_soi_index];
            double theta = body.prime_meridian_epoch + (tele.sim_time * 2.0 * kPI / body.rotation_period);
            Quat rot = Quat::fromAxisAngle(Vec3(0,0,1), (float)theta);
            Quat tilt = Quat::fromAxisAngle(Vec3(1,0,0), (float)body.axial_tilt);
            Quat full_rot = tilt * rot;
            Vec3 s_pos((float)trans.launch_site_px, (float)trans.launch_site_py, (float)trans.launch_site_pz);
            Vec3 localUp = s_pos.normalized();
            Vec3 i_surf = full_rot.rotate(localUp * s_pos.length());
            Vec3 i_up = full_rot.rotate(localUp);
            Vec3 padCenter(
                (float)((body.px + (double)i_surf.x) * ws_d - ro_x),
                (float)((body.py + (double)i_surf.y) * ws_d - ro_y),
                (float)((body.pz + (double)i_surf.z) * ws_d - ro_z));
            Vec3 padUp = i_up;
            Vec3 defaultRight(1,0,0);
            if (fabs(padUp.dot(defaultRight)) > 0.9f) defaultRight = Vec3(0,1,0);
            Vec3 padRight = padUp.cross(defaultRight).normalized();
            Vec3 padForward = padUp.cross(padRight).normalized();
            Mat4 padRot;
            padRot.m[0]=padRight.x;padRot.m[1]=padRight.y;padRot.m[2]=padRight.z;padRot.m[3]=0;
            padRot.m[4]=padUp.x;padRot.m[5]=padUp.y;padRot.m[6]=padUp.z;padRot.m[7]=0;
            padRot.m[8]=padForward.x;padRot.m[9]=padForward.y;padRot.m[10]=padForward.z;padRot.m[11]=0;
            padRot.m[12]=0;padRot.m[13]=0;padRot.m[14]=0;padRot.m[15]=1;
            float ps = (float)ws_d;
            Vec3 correctedPos = padCenter - padUp * (8.f * ps);
            Mat4 padModel = Mat4::translate(correctedPos) * padRot * Mat4::scale(Vec3(ps,ps,ps));
            padModel.toFloatArray(snap.launchPad.model);
            snap.launchPad.useObjMesh = hasLaunchPadOBJ;
            snap.hasLaunchPad = true;
        }

        // ---- 12. 太阳物理球体 (Block E, 仅全景模式) ----
        if (cam.mode == 2) {
            float sr = (float)ctx.sun_radius;
            Vec3 sunWorld(ctx.renderSun.x, ctx.renderSun.y, ctx.renderSun.z);
            Mat4 sm = Mat4::translate(sunWorld) * Mat4::scale(Vec3(sr,sr,sr));
            sm.toFloatArray(snap.sunBody.model);
            snap.sunBody.radius = sr;
            snap.drawSunBody = true;
        }

        // ---- 13. 轨道线 + 变轨节点 + Ap/Pe + 火箭标记 (Block A, 仅 Panorama) ----
        if (cam.mode == 2) {
            orbitSystem.extractRibbons(snap.ribbons, world, rocket_entity, hudManager.hud, mnvManager,
                cam, ctx.viewMat, ctx.aspect, ws_d, ctx.ro_x, ctx.ro_y, ctx.ro_z, dt,
                UniverseModel::getInstance().current_soi_index, ctx.earth_r, ctx.cam_dist, ctx.renderRocketBase, ctx.camEye_rel);
            extractKepRibbons(vkKepCache, ctx.earth_r, ctx.cam_dist, snap.ribbons);
            extractAdvRibbons(vkAdvCache, ctx.earth_r, ctx.cam_dist, snap.ribbons);
            mnvManager.extractBillboards(snap.billboards, world, rocket_entity, hudManager.hud,
                ctx.viewMat, ctx.aspect, ctx.earth_r, ctx.cam_dist, ws_d, ctx.ro_x, ctx.ro_y, ctx.ro_z, dt);
            extractOrbitMarkers(vkKepCache, vkRktCache, snap.billboards);
        }

        // ---- 14. 云调试面板 (Block F) ----
        snap.cloudTuner.visible = cloudTuner.visible;
        if (cloudTuner.visible && GameContext::getInstance().renderer3d) {
            auto& p = GameContext::getInstance().renderer3d->cloudSystem.tuneParams;
            snap.cloudTuner.covLo=p.covLo; snap.cloudTuner.covHi=p.covHi;
            snap.cloudTuner.threshLo=p.threshLo; snap.cloudTuner.threshHi=p.threshHi;
            snap.cloudTuner.erosion=p.erosion; snap.cloudTuner.density=p.density;
            snap.cloudTuner.extinction=p.extinction;
            snap.cloudTuner.minAlt=p.minAlt; snap.cloudTuner.maxAlt=p.maxAlt;
            snap.cloudTuner.debugMode=p.debugMode;
        }

        // ---- 15. SVO + Terrain + Vegetation (Block D) ----
        {
            auto* r3d_check = GameContext::getInstance().renderer3d;
            bool needBlockD = (GameContext::getInstance().terrain != nullptr);
            if (needBlockD) {
                _extractBlockD(snap, ss, ctx, ws_d, ro_x, ro_y, ro_z);
            }
        }

        return snap;
    }

private:
    // Block D 数据提取（SVO / Terrain Quadtree / Vegetation 实例）
    static void _extractBlockD(SceneSnapshot& snap,
                               const std::vector<CelestialBody>& ss,
                               const RenderContext& ctx,
                               double ws_d, double ro_x, double ro_y, double ro_z) {
        auto* r3d = GameContext::getInstance().renderer3d;
        // r3d 在 Vulkan 模式下为 nullptr，SVO 块内部已自守卫；terrain 块用 activeTerrain

        // 查找地球的 CelestialDraw 数据（bodyIdx==3）
        const CelestialDraw* earthCD = nullptr;
        for (auto& cd : snap.celestials) {
            if (cd.bodyIdx == 3) { earthCD = &cd; break; }
        }
        if (!earthCD) return;

        Vec3 planetCenter(earthCD->model[12], earthCD->model[13], earthCD->model[14]);
        Vec3 planetCenterRel = planetCenter - Vec3(snap.camPos[0], snap.camPos[1], snap.camPos[2]);
        Vec3 axisX(earthCD->model[0], earthCD->model[1], earthCD->model[2]);
        Vec3 axisY(earthCD->model[4], earthCD->model[5], earthCD->model[6]);
        Vec3 axisZ(earthCD->model[8], earthCD->model[9], earthCD->model[10]);
        axisX = axisX.normalized(); axisY = axisY.normalized(); axisZ = axisZ.normalized();

        // --- SVO ---
        if (r3d && r3d->svoManager && r3d->svoManager->hasActiveChunks()) {
            SVO::meshAllDirty(r3d->svoManager->chunks, r3d->svoManager->pool, r3d->svoManager->region);

            Vec3 E = r3d->svoManager->region.east;
            Vec3 U = r3d->svoManager->region.up;
            Vec3 N = r3d->svoManager->region.north;
            Vec3 OLocal = r3d->svoManager->region.centerNorm * r3d->svoManager->region.centerRadius;

            Vec3 wE = axisX*E.x + axisY*E.y + axisZ*E.z;
            Vec3 wU = axisX*U.x + axisY*U.y + axisZ*U.z;
            Vec3 wN = axisX*N.x + axisY*N.y + axisZ*N.z;
            Vec3 wO = axisX*OLocal.x + axisY*OLocal.y + axisZ*OLocal.z;
            wE = wE.normalized(); wU = wU.normalized(); wN = wN.normalized();
            Vec3 wOCamRel = wO + planetCenterRel;

            SVOChunkDraw sc;
            sc.svoMat[0]=wE.x;sc.svoMat[1]=wE.y;sc.svoMat[2]=wE.z;sc.svoMat[3]=0;
            sc.svoMat[4]=wU.x;sc.svoMat[5]=wU.y;sc.svoMat[6]=wU.z;sc.svoMat[7]=0;
            sc.svoMat[8]=wN.x;sc.svoMat[9]=wN.y;sc.svoMat[10]=wN.z;sc.svoMat[11]=0;
            sc.svoMat[12]=wOCamRel.x;sc.svoMat[13]=wOCamRel.y;sc.svoMat[14]=wOCamRel.z;sc.svoMat[15]=1;
            snap.svoChunks.push_back(sc);
        }

        // --- Terrain Quadtree ---
        Terrain::QuadtreeTerrain* activeTerrain = GameContext::getInstance().terrain;
        if (activeTerrain) {
            Vec3 camPosInertialRel = -planetCenterRel;
            Vec3 camPosLocalRel(camPosInertialRel.dot(axisX),
                                camPosInertialRel.dot(axisY),
                                camPosInertialRel.dot(axisZ));

            for (int i = 0; i < 6; i++) {
                activeTerrain->updateSubdivision(activeTerrain->roots[i].get(), camPosLocalRel, earthCD->radius);
                _extractTerrainLeaves(snap.terrainPatches,
                    activeTerrain->roots[i].get(), earthCD->radius, earthCD->model, snap.time, planetCenterRel);
            }

            // --- Vegetation 实例 ---
            float camAlt = (Vec3(snap.camPos[0], snap.camPos[1], snap.camPos[2]) - planetCenter).length()
                           - earthCD->radius;
            if (camAlt < 300.0f) {
                Vec3 camNorm = (Vec3(snap.camPos[0], snap.camPos[1], snap.camPos[2]) - planetCenter).normalized();
                for (int i = 0; i < 500; i++) {
                    float offX = ((i * 7) % 1000 / 500.f - 1.f) * 0.002f;
                    float offZ = ((i * 13) % 1000 / 500.f - 1.f) * 0.002f;
                    Vec3 p = (camNorm + Vec3(offX, 0, offZ)).normalized();
                    float h = activeTerrain->getHeight(p) / earthCD->radius;
                    if (h > 0.0f) {
                        Vec3 localPos = p * (earthCD->radius + h * earthCD->radius);
                        Vec3 worldP = localPos + planetCenter;
                        // 存储: pos(3) + scale(1) + rot(1) = 5 floats per instance
                        snap.vegInstanceData.push_back(worldP.x);
                        snap.vegInstanceData.push_back(worldP.y);
                        snap.vegInstanceData.push_back(worldP.z);
                        snap.vegInstanceData.push_back((0.5f + 0.5f * ((i * 3) % 100 / 100.f)) * 0.02f);
                        snap.vegInstanceData.push_back(((i * 3) % 100 / 100.f) * 6.28f);
                    }
                }
                snap.vegInstanceCount = (uint32_t)(snap.vegInstanceData.size() / 5);
            }
        }
    }

    static void _extractTerrainLeaves(std::vector<TerrainPatchDraw>& out,
                                       Terrain::TerrainNode* node, float planetRadius,
                                       const float model[16], float time,
                                       const Vec3& planetCenterRel) {
        if (!node) return;
        if (node->isLeaf) {
            TerrainPatchDraw tp;
            memcpy(tp.model, model, 64);
            tp.planetRadius = planetRadius;
            tp.maxElev = 25.f;
            tp.time = time;
            tp.nodeCenter[0] = node->center.x;
            tp.nodeCenter[1] = node->center.y;
            tp.nodeCenter[2] = node->center.z;
            tp.nodeSideA[0]  = node->sideA.x;
            tp.nodeSideA[1]  = node->sideA.y;
            tp.nodeSideA[2]  = node->sideA.z;
            tp.nodeSideB[0]  = node->sideB.x;
            tp.nodeSideB[1]  = node->sideB.y;
            tp.nodeSideB[2]  = node->sideB.z;
            tp.planetCenterRel[0] = planetCenterRel.x;
            tp.planetCenterRel[1] = planetCenterRel.y;
            tp.planetCenterRel[2] = planetCenterRel.z;
            tp.nodeLevel = node->level;
            out.push_back(tp);
        } else {
            for (int i = 0; i < 4; i++)
                _extractTerrainLeaves(out, node->children[i].get(), planetRadius, model, time, planetCenterRel);
        }
    }
};