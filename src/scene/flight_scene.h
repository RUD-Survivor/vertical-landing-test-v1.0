#pragma once
#include "scene.h"
#include "save_system.h"
#include "render/part_renderer.h"
#include "game_context.h"
#include "simulation/simulation_controller.h"
#include "camera/camera_director.h"
#include "simulation/orbit_physics.h"
#include "simulation/predictor.h"
#include "simulation/center_calculator.h"
#include "input/input_router.h"
#include "math/math3d.h"
#include "render/HUD_system.h"
#include <entt/entt.hpp>


// Any other includes you need
#include "flight_input_system.h"
#include "maneuver_manager.h"
#include "orbit_system.h"
#include "plume_manager.h"
#include "spaceport_manager.h"
#include "celestial_renderer.h"
#include "rocket_visuals.h"
#include "environment_system.h"
#include "hud_manager.h"
#include "render_context.h"
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
    void onEnter() override {
        GameContext& ctx = GameContext::getInstance();
        
        // --- 1. Initialize ECS Entity ---
        rocket_entity = world.create();
        // Emplace legacy bridging components
        auto& rocket_state = world.emplace<RocketState>(rocket_entity);
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
    // 使用加载的状态
    rocket_state = loaded_state;
    control_input = loaded_input;
    // Sync config to loaded stage
    StageManager::SyncActiveConfig(rocket_config, rocket_state.current_stage);
}
else {
    // 新游戏初始化
    rocket_state.fuel = builder_state_assembly.total_fuel;
    rocket_state.status = PRE_LAUNCH;
    rocket_state.mission_msg = "READY ON PAD - PRESS SPACE TO LAUNCH";
    // Initialize multi-stage fuel distribution
    rocket_state.total_stages = rocket_config.stages;
    rocket_state.current_stage = 0;
    rocket_state.stage_fuels.clear();
    for (int i = 0; i < (int)rocket_config.stage_configs.size(); i++) {
        rocket_state.stage_fuels.push_back(rocket_config.stage_configs[i].fuel_capacity);
    }
    // Set initial fuel to stage 0’s capacity
    if (!rocket_state.stage_fuels.empty()) {
        rocket_state.fuel = rocket_state.stage_fuels[0];
    }
    // Calculate initial surface coordinates from launch latitude/longitude
    double lat_rad = rocket_state.launch_latitude * PI / 180.0;
    double lon_rad = rocket_state.launch_longitude * PI / 180.0;
    float lowest_y = 0.0f;
    if (!builder_state_assembly.parts.empty()) {
        lowest_y = 1e10f;
        for (const auto& p : builder_state_assembly.parts) {
            lowest_y = std::min(lowest_y, (float)p.pos.y);
        }
    }
    // Distance from planet center to CoM
    double R = SOLAR_SYSTEM[current_soi_index].radius - (double)lowest_y;
    // Z is the North-South axis, XY is the equatorial plane
    rocket_state.surf_px = R * cos(lat_rad) * cos(lon_rad);
    rocket_state.surf_py = R * cos(lat_rad) * sin(lon_rad);
    rocket_state.surf_pz = R * sin(lat_rad);
    // Store fixed launch site for pad rendering
    rocket_state.launch_site_px = rocket_state.surf_px;
    rocket_state.launch_site_py = rocket_state.surf_py;
    rocket_state.launch_site_pz = rocket_state.surf_pz;
    // Initialize inertial coordinates immediately for the first frame
    CelestialBody& body = SOLAR_SYSTEM[current_soi_index];
    double theta = body.prime_meridian_epoch; // sim_time = 0
    Quat rot = Quat::fromAxisAngle(Vec3(0, 0, 1), (float)theta);
    Quat tilt = Quat::fromAxisAngle(Vec3(1, 0, 0), (float)body.axial_tilt);
    Quat full_rot = tilt * rot;
    Vec3 world_pos = full_rot.rotate(Vec3((float)rocket_state.surf_px, (float)rocket_state.surf_py, (float)rocket_state.surf_pz));
    rocket_state.px = (double)world_pos.x;
    rocket_state.py = (double)world_pos.y;
    rocket_state.pz = (double)world_pos.z;
    }
    
    // ======== INPUT ROUTER INITIALIZATION ========
    inputSystem.setup(input, rocket_state, rocket_config, control_input, hudManager.hud, cam, show_clouds, world, rocket_entity);
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

    // --- 5. ECS Data Synchronization (Legacy -> Components) ---
    auto& trans = world.get<TransformComponent>(rocket_entity);
    trans.px = rocket_state.px; trans.py = rocket_state.py; trans.pz = rocket_state.pz;
    trans.abs_px = rocket_state.abs_px; trans.abs_py = rocket_state.abs_py; trans.abs_pz = rocket_state.abs_pz;
    trans.surf_px = rocket_state.surf_px; trans.surf_py = rocket_state.surf_py; trans.surf_pz = rocket_state.surf_pz;
    trans.launch_latitude = rocket_state.launch_latitude; trans.launch_longitude = rocket_state.launch_longitude;
    trans.launch_site_px = rocket_state.launch_site_px; trans.launch_site_py = rocket_state.launch_site_py; trans.launch_site_pz = rocket_state.launch_site_pz;

    auto& vel = world.get<VelocityComponent>(rocket_entity);
    vel.vx = rocket_state.vx; vel.vy = rocket_state.vy; vel.vz = rocket_state.vz;
    vel.abs_vx = rocket_state.abs_vx; vel.abs_vy = rocket_state.abs_vy; vel.abs_vz = rocket_state.abs_vz;
    vel.acceleration = rocket_state.acceleration;

    auto& att = world.get<AttitudeComponent>(rocket_entity);
    att.attitude = rocket_state.attitude; att.initialized = rocket_state.attitude_initialized;
    att.angle = rocket_state.angle; att.ang_vel = rocket_state.ang_vel;
    att.angle_z = rocket_state.angle_z; att.ang_vel_z = rocket_state.ang_vel_z;
    att.angle_roll = rocket_state.angle_roll; att.ang_vel_roll = rocket_state.ang_vel_roll;

    auto& prop = world.get<PropulsionComponent>(rocket_entity);
    prop.fuel = rocket_state.fuel; prop.current_stage = rocket_state.current_stage;
    prop.total_stages = rocket_state.total_stages; prop.stage_fuels = rocket_state.stage_fuels;
    prop.jettisoned_mass = rocket_state.jettisoned_mass; prop.fuel_consumption_rate = rocket_state.fuel_consumption_rate;
    prop.thrust_power = rocket_state.thrust_power;

    auto& tele = world.get<TelemetryComponent>(rocket_entity);
    tele.sim_time = rocket_state.sim_time; tele.altitude = rocket_state.altitude;
    tele.terrain_altitude = rocket_state.terrain_altitude; tele.solar_occlusion = rocket_state.solar_occlusion;

    auto& guid = world.get<GuidanceComponent>(rocket_entity);
    guid.status = rocket_state.status; guid.mission_msg = rocket_state.mission_msg;
    guid.mission_phase = rocket_state.mission_phase; guid.mission_timer = rocket_state.mission_timer;
    guid.auto_mode = rocket_state.auto_mode; guid.sas_active = rocket_state.sas_active;
    guid.rcs_active = rocket_state.rcs_active; guid.sas_mode = rocket_state.sas_mode;
    guid.sas_target_vec = rocket_state.sas_target_vec; guid.leg_deploy_progress = rocket_state.leg_deploy_progress;
    guid.suicide_burn_locked = rocket_state.suicide_burn_locked; guid.show_absolute_time = rocket_state.show_absolute_time;
    guid.pid_vert = rocket_state.pid_vert; guid.pid_pos = rocket_state.pid_pos;
    guid.pid_att = rocket_state.pid_att; guid.pid_att_z = rocket_state.pid_att_z; guid.pid_att_roll = rocket_state.pid_att_roll;

    auto& orb = world.get<OrbitComponent>(rocket_entity);
    orb.predicted_path = rocket_state.predicted_path; orb.predicted_ground_track = rocket_state.predicted_ground_track;
    orb.last_prediction_sim_time = rocket_state.last_prediction_sim_time; orb.prediction_in_progress = rocket_state.prediction_in_progress;
    
    auto& mnv = world.get<ManeuverComponent>(rocket_entity);
    mnv.maneuvers = rocket_state.maneuvers; mnv.selected_maneuver_index = rocket_state.selected_maneuver_index;

     // 50Hz 物理步长
     // Tab 防抖
    // 记录火箭历史飞行的 3D 轨迹点
    // --- 5. 核心飞行模拟循环 (The Main Flight Loop) ---
    // 这里是游戏最核心的部分：物理计算、姿态控制、轨道绘图都在这里执行。
    }


    // ===================================================================
    // ECS Bridge: Bidirectional Synchronization
    // -------------------------------------------------------------------
    // During the transition period, some systems write to RocketState
    // (e.g., input lambdas) while others write to ECS components
    // (e.g., PhysicsSystem). These two functions keep them in sync.
    // ===================================================================
    
    void syncLegacyToECS() {
        auto& rs = world.get<RocketState>(rocket_entity);
        auto& trans = world.get<TransformComponent>(rocket_entity);
        auto& vel   = world.get<VelocityComponent>(rocket_entity);
        auto& att   = world.get<AttitudeComponent>(rocket_entity);
        auto& prop  = world.get<PropulsionComponent>(rocket_entity);
        auto& tele  = world.get<TelemetryComponent>(rocket_entity);
        auto& guid  = world.get<GuidanceComponent>(rocket_entity);
        auto& mnv   = world.get<ManeuverComponent>(rocket_entity);

        trans.px = rs.px; trans.py = rs.py; trans.pz = rs.pz;
        trans.abs_px = rs.abs_px; trans.abs_py = rs.abs_py; trans.abs_pz = rs.abs_pz;
        trans.surf_px = rs.surf_px; trans.surf_py = rs.surf_py; trans.surf_pz = rs.surf_pz;
        trans.launch_site_px = rs.launch_site_px; trans.launch_site_py = rs.launch_site_py; trans.launch_site_pz = rs.launch_site_pz;
        trans.launch_latitude = rs.launch_latitude; trans.launch_longitude = rs.launch_longitude;

        vel.vx = rs.vx; vel.vy = rs.vy; vel.vz = rs.vz;
        vel.abs_vx = rs.abs_vx; vel.abs_vy = rs.abs_vy; vel.abs_vz = rs.abs_vz;
        vel.acceleration = rs.acceleration;

        att.attitude = rs.attitude; att.initialized = rs.attitude_initialized;
        att.angle = rs.angle; att.ang_vel = rs.ang_vel;
        att.angle_z = rs.angle_z; att.ang_vel_z = rs.ang_vel_z;
        att.angle_roll = rs.angle_roll; att.ang_vel_roll = rs.ang_vel_roll;

        prop.fuel = rs.fuel; prop.current_stage = rs.current_stage;
        prop.total_stages = rs.total_stages; prop.stage_fuels = rs.stage_fuels;
        prop.jettisoned_mass = rs.jettisoned_mass; prop.fuel_consumption_rate = rs.fuel_consumption_rate;
        prop.thrust_power = rs.thrust_power;

        tele.sim_time = rs.sim_time; tele.altitude = rs.altitude;
        tele.velocity = rs.velocity; tele.local_vx = rs.local_vx;
        tele.terrain_altitude = rs.terrain_altitude; tele.solar_occlusion = rs.solar_occlusion;

        guid.status = rs.status; guid.mission_msg = rs.mission_msg;
        guid.mission_phase = rs.mission_phase; guid.mission_timer = rs.mission_timer;
        guid.auto_mode = rs.auto_mode; guid.sas_active = rs.sas_active;
        guid.rcs_active = rs.rcs_active; guid.sas_mode = rs.sas_mode;
        guid.sas_target_vec = rs.sas_target_vec; guid.leg_deploy_progress = rs.leg_deploy_progress;
        guid.suicide_burn_locked = rs.suicide_burn_locked; guid.show_absolute_time = rs.show_absolute_time;
        guid.pid_vert = rs.pid_vert; guid.pid_pos = rs.pid_pos;
        guid.pid_att = rs.pid_att; guid.pid_att_z = rs.pid_att_z; guid.pid_att_roll = rs.pid_att_roll;

        mnv.maneuvers = rs.maneuvers; mnv.selected_maneuver_index = rs.selected_maneuver_index;
    }

    void syncECSToLegacy() {
        auto& rs = world.get<RocketState>(rocket_entity);
        auto& trans = world.get<TransformComponent>(rocket_entity);
        auto& vel   = world.get<VelocityComponent>(rocket_entity);
        auto& att   = world.get<AttitudeComponent>(rocket_entity);
        auto& prop  = world.get<PropulsionComponent>(rocket_entity);
        auto& tele  = world.get<TelemetryComponent>(rocket_entity);
        auto& guid  = world.get<GuidanceComponent>(rocket_entity);
        auto& mnv   = world.get<ManeuverComponent>(rocket_entity);
        auto& vfx   = world.get<VFXComponent>(rocket_entity);

        rs.px = trans.px; rs.py = trans.py; rs.pz = trans.pz;
        rs.abs_px = trans.abs_px; rs.abs_py = trans.abs_py; rs.abs_pz = trans.abs_pz;
        rs.surf_px = trans.surf_px; rs.surf_py = trans.surf_py; rs.surf_pz = trans.surf_pz;
        rs.launch_site_px = trans.launch_site_px; rs.launch_site_py = trans.launch_site_py; rs.launch_site_pz = trans.launch_site_pz;
        rs.launch_latitude = trans.launch_latitude; rs.launch_longitude = trans.launch_longitude;

        rs.vx = vel.vx; rs.vy = vel.vy; rs.vz = vel.vz;
        rs.abs_vx = vel.abs_vx; rs.abs_vy = vel.abs_vy; rs.abs_vz = vel.abs_vz;
        rs.acceleration = vel.acceleration;

        rs.attitude = att.attitude; rs.attitude_initialized = att.initialized;
        rs.angle = att.angle; rs.ang_vel = att.ang_vel;
        rs.angle_z = att.angle_z; rs.ang_vel_z = att.ang_vel_z;
        rs.angle_roll = att.angle_roll; rs.ang_vel_roll = att.ang_vel_roll;

        rs.fuel = prop.fuel; rs.current_stage = prop.current_stage;
        rs.total_stages = prop.total_stages; rs.stage_fuels = prop.stage_fuels;
        rs.jettisoned_mass = prop.jettisoned_mass; rs.fuel_consumption_rate = prop.fuel_consumption_rate;
        rs.thrust_power = prop.thrust_power;

        rs.sim_time = tele.sim_time; rs.altitude = tele.altitude;
        rs.velocity = tele.velocity; rs.local_vx = tele.local_vx;
        rs.terrain_altitude = tele.terrain_altitude; rs.solar_occlusion = tele.solar_occlusion;

        rs.status = guid.status; rs.mission_msg = guid.mission_msg;
        rs.mission_phase = guid.mission_phase; rs.mission_timer = guid.mission_timer;
        rs.auto_mode = guid.auto_mode; rs.sas_active = guid.sas_active;
        rs.rcs_active = guid.rcs_active; rs.sas_mode = guid.sas_mode;
        rs.sas_target_vec = guid.sas_target_vec; rs.leg_deploy_progress = guid.leg_deploy_progress;
        rs.suicide_burn_locked = guid.suicide_burn_locked; rs.show_absolute_time = guid.show_absolute_time;
        rs.pid_vert = guid.pid_vert; rs.pid_pos = guid.pid_pos;
        rs.pid_att = guid.pid_att; rs.pid_att_z = guid.pid_att_z; rs.pid_att_roll = guid.pid_att_roll;

        rs.maneuvers = mnv.maneuvers; rs.selected_maneuver_index = mnv.selected_maneuver_index;

    }

    void update(double dt) override {
        real_dt = dt;
        auto& rocket_state = world.get<RocketState>(rocket_entity);
        auto& rocket_config = world.get<RocketConfig>(rocket_entity);
        auto& control_input = world.get<ControlInput>(rocket_entity);

        const RocketAssembly& assembly = GameContext::getInstance().launch_assembly;
        Renderer3D* r3d = GameContext::getInstance().renderer3d;
        // Limit spikes
        frame++;
        if (frame == 1) {
            cout << "[FlightScene] r3d=" << (void*)r3d << " earthProg=" << r3d->earthProgram << " atmoProg=" << r3d->atmoProg << " terrainProg=" << r3d->terrainProg << endl;
        }
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
        // --- Maneuver Node Input Handling (Delegated to ManeuverManager) ---
        if (cam.mode == 2) {
            mnvManager.update(GameContext::getInstance().window, rocket_state, hudManager.hud, cam, dt, ww, wh);
        }
        // --- Ensure Rocket starts perfectly on the Terrain/SVO surface ---
        if (r3d->terrain) {
            Vec3 localUp(rocket_state.surf_px, rocket_state.surf_py, rocket_state.surf_pz);
            localUp = localUp.normalized();
            Quat unalign_from_z = Quat::fromAxisAngle(Vec3(1.0f, 0.0f, 0.0f), (float)(PI / 2.0));
            Vec3 qLocal = unalign_from_z.rotate(localUp);
            float terrH = r3d->terrain->getHeight(qLocal);
            rocket_state.terrain_altitude = (double)terrH * 1000.0; // km to meters
            if (rocket_state.status == PRE_LAUNCH && !terrain_adjusted) {
                double platform_height = 8.5; // Thickness of the visual launch platform
                double new_R = EARTH_RADIUS + rocket_state.terrain_altitude - rocket_config.bounds_bottom + platform_height;
                rocket_state.surf_px = localUp.x * new_R;
                rocket_state.surf_py = localUp.y * new_R;
                rocket_state.surf_pz = localUp.z * new_R;
                terrain_adjusted = true;
                cout << "[TERRAIN] Adjusted launchpad altitude by " << rocket_state.terrain_altitude << " meters to match SVO bounds." << endl;
            }
        }
        // SVO Automation and Input Polling
        inputSystem.poll(GameContext::getInstance().window, input, rocket_state, r3d);
        // --- Simulation & Physics Update ---
        syncLegacyToECS(); // Bridge: input changes -> ECS
        sim_ctrl.handleInput(GameContext::getInstance().window, world, rocket_entity);
        sim_ctrl.update(real_dt, world, rocket_entity, hudManager.hud, GameContext::getInstance().window, cam.mode);
        syncECSToLegacy(); // Bridge: physics results -> legacy rendering
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
    }
    void render() override {
        auto& rocket_state = world.get<RocketState>(rocket_entity);
        auto& rocket_config = world.get<RocketConfig>(rocket_entity);
        auto& control_input = world.get<ControlInput>(rocket_entity);

        Renderer* renderer = GameContext::getInstance().renderer2d;
        Renderer3D* r3d = GameContext::getInstance().renderer3d;
        const RocketAssembly& assembly = GameContext::getInstance().launch_assembly;
        // ================= 3D 渲染通道 =================
        {
            // === 1. 构建本帧渲染参考系上下文 (浮动原点、物理变换、投影矩阵) ===
            ctx.update(rocket_state, rocket_config, current_soi_index, last_soi, comma_prev, period_prev, cam, ws_d, dt);

            // 相机设置 (TAA 与 GL 状态清理)
            int ww, wh;
            glfwGetFramebufferSize(GameContext::getInstance().window, &ww, &wh);
            r3d->initTAA(ww, wh);
            
            // === CRITICAL: Full GL state reset before 3D pass ===
            glViewport(0, 0, ww, wh);
            glDisable(GL_BLEND);
            glEnable(GL_DEPTH_TEST);
            glDepthMask(GL_TRUE);
            glDepthFunc(GL_LESS);
            glDisable(GL_CULL_FACE);
            glFrontFace(GL_CCW);
            glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
            glBindFramebuffer(GL_FRAMEBUFFER, 0); 

            // --- Camera-Centric Visuals (Day/Night & Atmosphere) 已转移至 EnvironmentSystem ---
            environmentSystem.updateAndApply(ctx.camEye_rel, ctx.ro_x, ctx.ro_y, ctx.ro_z, ws_d, cam.mode);
            float day_blend = (float)environmentSystem.day_blend;
            float alt_factor = environmentSystem.alt_factor;
            
            // =========== PASS 1: MACRO BACKGROUND (天体渲染已转移至 CelestialRenderer) ===========
            celestialRenderer.renderMacroPass(
                r3d, rocket_state, current_soi_index, cam, ctx.camEye_rel, ctx.cam_dist,
                ws_d, ctx.ro_x, ctx.ro_y, ctx.ro_z, ctx.aspect, (float)day_blend, alt_factor, 
                show_clouds, frame, ww, wh, ctx.renderRocketBase, ctx.renderSun, ctx.sun_radius, ctx.earth_r,
                earthMesh, ringMesh, ctx.viewMat, ctx.macroProjMat, ctx.lightVec
            );
            if (cam.mode == 2) {
                // ===== 历史轨迹线与轨道预测线 (已转移至 OrbitSystem) =====
                orbitSystem.render(rocket_state, rocket_config, hudManager.hud, mnvManager, r3d, cam, control_input, ctx.viewMat, ctx.macroProjMat, ctx.aspect, ws_d, ctx.ro_x, ctx.ro_y, ctx.ro_z, ww, wh, dt, current_soi_index, ctx.earth_r, ctx.cam_dist, ctx.renderRocketBase, ctx.camEye_rel);
                // --- Maneuver Nodes & Interaction ---
                mnvManager.render(rocket_state, hudManager.hud, r3d, ctx.viewMat, ctx.macroProjMat, ctx.aspect, ctx.earth_r, ctx.cam_dist, ws_d, ctx.ro_x, ctx.ro_y, ctx.ro_z, dt);
            }
     
            // Dynamic TAA blend: reduce temporal accumulation during fast changes
            // to prevent ghosting/smearing of orbit lines
            if (sim_ctrl.time_warp > 1 || hudManager.hud.mnv_popup_slider_dragging >= 0) {
                // In time warp or during slider drag, favor current frame heavily
                float warp_blend = (sim_ctrl.time_warp > 100) ? 0.8f : (sim_ctrl.time_warp > 1 ? 0.5f : 0.4f);
                if (hudManager.hud.mnv_popup_slider_dragging >= 0) warp_blend = fmaxf(warp_blend, 0.6f);
                r3d->taaBlendOverride = warp_blend;
            }
            else {
                r3d->taaBlendOverride = -1.0f; // Use default
            }
            r3d->resolveTAA();
            // =========== PASS 2: MICRO FOREGROUND (已转移至 RocketVisuals) ===========
            rocketVisuals.render(r3d, cam, ctx.cam_dist, ctx.rh, ctx.aspect, world, rocket_entity, assembly, 
                                 ctx.renderRocketBase, ctx.rocketQuat, ws_d, rocketBody, rocketNose, rocketBox, 
                                 ctx.viewMat, ctx.camEye_rel);
            //=============火箭涂装==========================
            
            // ===== 发射台渲染 (已转移至 SpaceportManager) =====
            spaceportManager.render(r3d, world, rocket_entity, current_soi_index, ws_d, ctx.ro_x, ctx.ro_y, ctx.ro_z, rocketBox, ctx.rw_3d, ctx.rh);

            // ===== 体积尾焰渲染 (已转移至 PlumeManager) =====
            plumeManager.render(rocket_state, rocket_config, control_input, assembly, r3d, rocketBox, ctx.rocketQuat, ctx.renderRocketBase, ws_d);
            r3d->endFrame();
        }
        // ================= 2D HUD 渲染通道 (已转移至 HUDManager) =================
        hudManager.render(renderer, r3d, rocket_state, rocket_config, control_input, cam, sim_ctrl, mnvManager, 
                          ctx.rocketQuat, ctx.rocketUp, ctx.localNorth, ctx.localRight, ctx.viewMat, ctx.macroProjMat, ctx.camEye_rel, dt, 
                          frame, ws_d, mouse_x, mouse_y, lmb, lmb_prev, rmb);
        // Update state for next frame ONLY at the very end
        lmb_prev = lmb;
    }
};