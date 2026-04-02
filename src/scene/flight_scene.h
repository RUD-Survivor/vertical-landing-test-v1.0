#include "render/HUD_system.h"
#pragma once
#include "scene.h"
#include "game_context.h"
#include "simulation/simulation_controller.h"
#include "camera/camera_director.h"
#include "simulation/orbit_physics.h"
#include "simulation/predictor.h"
#include "simulation/center_calculator.h"
// Any other includes you need
class FlightScene : public IScene {
public:
    // === Core Members ===
    Quat rocketQuat;
    Vec3 rocketUp, localNorth, localRight;
    FlightHUD hud;
    double ws_d = 1.0;
    // Shared between update() and render()
    Mat4 viewMat;
    Mat4 macroProjMat;
    Vec3 camEye_rel;
    double mouse_x = 0, mouse_y = 0;
    bool lmb = false, lmb_prev = false, rmb = false;
    RocketState rocket_state;
    ControlInput control_input;
    RocketConfig rocket_config;
    CameraDirector cam;
    Mesh earthMesh;
    Mesh ringMesh;
    Mesh rocketBody;
    Mesh rocketNose;
    Mesh rocketBox;
    Mesh launchPadMesh;
    bool has_launch_pad;
    SimulationController sim_ctrl;
    double dt = 0.02;
    int frame = 0;
    // Track 3D position history
    struct DVec3 { double x, y, z; };
    struct TrajPoint { DVec3 e; DVec3 s; };
    std::vector<TrajPoint> traj_history; 
    // === Variables Extracted from Static ===
    bool tab_was_pressed = false;
    float global_best_ang = -1.0f;
    double global_best_mu = 0, global_best_a = 0, global_best_ecc = 0;
    double global_current_M0 = 0, global_current_n = 0;
    Vec3 global_best_pt, global_best_center, global_best_e_dir, global_best_perp_dir;
    bool rmb_prev_mnv = false;
    bool space_was_pressed = true;
    bool g_was_pressed = false;
    bool n_was_pressed = false;
    bool c_was_pressed = false;
    int dragging_handle = -1;
    float last_drag_mx = 0;
    float frame_delta_mx = 0;
    bool h_was_pressed = false;
    bool o_was_pressed = false;
    bool r_was_pressed = false;
    bool rcs_key_prev = false;
    bool k_was_pressed = false;
    bool show_clouds = true;
    bool p_was_pressed = false;
    bool terrain_adjusted = false;
    bool v_was_pressed = false;
    bool svo_auto_activated = false;
    int svo_dig_mode = 0;
    bool b_was_pressed = false;
    int svo_op_cooldown = 0;
    bool comma_prev = false;
    bool period_prev = false;
    int last_soi = -1;
    std::chrono::steady_clock::time_point last_req_time = std::chrono::steady_clock::now();
    std::vector<Vec3> cached_rel_pts;
    std::vector<Vec3> cached_mnv_rel_pts;
    size_t last_draw_points_size = 0;
    size_t last_draw_mnv_points_size = 0;
    Vec3 last_first_pt, last_mnv_first_pt;
    bool lmb_prev_mnv = false;
    bool popup_clicked_frame = false;
    float cached_popup_x = 0, cached_popup_y = 0, cached_popup_w = 0, cached_popup_h = 0;
    float last_pass_mx = 0, last_pass_my = 0;
    bool hit_maneuver_icon = false;
    void onEnter() override {
        GameContext& ctx = GameContext::getInstance();
        earthMesh = MeshGen::sphere(256, 512, 1.0f);
        ringMesh = MeshGen::ring(128, 1.11f, 2.35f);
        rocketBody = MeshGen::cylinder(32, 1.0f, 1.0f);
        rocketNose = MeshGen::cone(32, 1.0f, 1.0f);
        rocketBox = MeshGen::box(1.0f, 1.0f, 1.0f);
        launchPadMesh = ModelLoader::loadOBJ("assets/launch_pad.obj");
        has_launch_pad = (launchPadMesh.indexCount > 0);
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
    }
    void update(double dt) override {
        double real_dt = dt;
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
    // Transfer Window Calculator state
    // --- 鼠标输入与发射控制 ---
     // 起始设为 true 以忽略编辑器里的空格。
    bool space_now = glfwGetMouseButton(GameContext::getInstance().window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS || glfwGetKey(GameContext::getInstance().window, GLFW_KEY_SPACE) == GLFW_PRESS;
    if (space_now && !space_was_pressed) {
        if (rocket_state.status == PRE_LAUNCH) {
            // 点击空格或左键：点火发射！火箭状态从“待命”变为“上升”。
            rocket_state.status = ASCEND;
            rocket_state.mission_msg = "T-0: IGNITION! LIFTOFF!";
        }
    }
    space_was_pressed = space_now;
    // --- G 键手动分级 (Manual Stage Separation) ---
    // 就像真火箭一样，烧完一级就扔掉一级，减轻负担。
    bool g_now = glfwGetKey(GameContext::getInstance().window, GLFW_KEY_G) == GLFW_PRESS;
    if (g_now && !g_was_pressed) {
        if (rocket_state.status == ASCEND || rocket_state.status == DESCEND) {
            StageManager::SeparateStage(rocket_state, rocket_config);
        }
    }
    g_was_pressed = g_now;
    // --- N 键自动执行机动节点 (Auto-Execute Maneuver) ---
    bool n_now = glfwGetKey(GameContext::getInstance().window, GLFW_KEY_N) == GLFW_PRESS;
    if (n_now && !n_was_pressed) {
        if (!rocket_state.maneuvers.empty()) {
            hud.auto_exec_mnv = !hud.auto_exec_mnv;
            if (hud.auto_exec_mnv) {
                rocket_state.mission_msg = "MNV AUTO-EXEC: ARMED";
            }
            else {
                rocket_state.mission_msg = "MNV AUTO-EXEC: OFF";
                control_input.throttle = 0;
            }
        }
    }
    n_was_pressed = n_now;
    // --- C 键切换 3D 视角 ---
    {
        bool c_now = glfwGetKey(GameContext::getInstance().window, GLFW_KEY_C) == GLFW_PRESS;
        if (c_now && !c_was_pressed) cam.cycleMode();
        c_was_pressed = c_now;
    }
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
    // --- Maneuver Node Input Handling (Basic) ---
    if (cam.mode == 2) {
        double mx, my; glfwGetCursorPos(GameContext::getInstance().window, &mx, &my);
        float mouse_x = (float)(mx / ww * 2.0 - 1.0);
        float mouse_y = (float)(1.0 - my / wh * 2.0);
        frame_delta_mx = mouse_x - last_drag_mx;
        bool lmb = glfwGetMouseButton(GameContext::getInstance().window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS;
        if (!lmb) dragging_handle = -1;
        if (dragging_handle != -1 && lmb && rocket_state.selected_maneuver_index != -1) {
            ManeuverNode& node = rocket_state.maneuvers[rocket_state.selected_maneuver_index];
            if (dragging_handle == -2) { // Dragging the node itself (Time Slider)
                if (global_best_ang >= 0) {
                    double mu_body = 6.67430e-11 * SOLAR_SYSTEM[current_soi_index].mass;
                    double M_click = global_best_ang - global_best_ecc * sin(global_best_ang);
                    double r_mag = sqrt(rocket_state.px * rocket_state.px + rocket_state.py * rocket_state.py + rocket_state.pz * rocket_state.pz);
                    double cos_E = (global_best_a - r_mag) / (global_best_a * global_best_ecc);
                    double sin_E = (rocket_state.px * rocket_state.vx + rocket_state.py * rocket_state.vy + rocket_state.pz * rocket_state.vz) / (global_best_ecc * sqrt(mu_body * global_best_a));
                    double E0 = atan2(sin_E, cos_E);
                    double M0 = E0 - global_best_ecc * sin(E0);
                    double n = sqrt(mu_body / (global_best_a * global_best_a * global_best_a));
                    double dM = M_click - M0;
                    while (dM < 0) dM += 2.0 * PI;
                    while (dM > 2.0 * PI) dM -= 2.0 * PI;
                    node.sim_time = rocket_state.sim_time + (dM / n);
                }
            }
            else { // Dragging a Delta-V handle
                // This logic is now handled in the rendering pass where screen-space axis vectors are available
            }
        }
        last_drag_mx = mouse_x;
    }
    // --- H 键切换 HUD 显示 ---
    if (glfwGetKey(GameContext::getInstance().window, GLFW_KEY_H) == GLFW_PRESS) {
        if (!h_was_pressed) {
            hud.show_hud = !hud.show_hud;
            h_was_pressed = true;
        }
    }
    else {
        h_was_pressed = false;
    }
    // --- Tab 键切换模式（带防抖）---
    if (glfwGetKey(GameContext::getInstance().window, GLFW_KEY_TAB) == GLFW_PRESS) {
        if (!tab_was_pressed) {
            rocket_state.auto_mode = !rocket_state.auto_mode;
            if (rocket_state.auto_mode) {
                rocket_state.pid_vert.reset();
                rocket_state.pid_pos.reset();
                rocket_state.pid_att.reset();
                rocket_state.pid_att_z.reset();
                rocket_state.mission_msg = ">> AUTOPILOT ENGAGED";
            }
            else {
                rocket_state.mission_msg = ">> MANUAL CONTROL ACTIVE";
            }
            tab_was_pressed = true;
        }
    }
    else {
        tab_was_pressed = false;
    }
    // --- O 键切换轨道显示 ---
    if (glfwGetKey(GameContext::getInstance().window, GLFW_KEY_O) == GLFW_PRESS) {
        if (!o_was_pressed) {
            hud.show_orbit = !hud.show_orbit; // 这里为了兼容旧逻辑，如果是双击才算切换？不如我们用 R 键来切换参考系
            o_was_pressed = true;
        }
    }
    else {
        o_was_pressed = false;
    }
    if (glfwGetKey(GameContext::getInstance().window, GLFW_KEY_T) == GLFW_PRESS) {
        if (!r_was_pressed) {
            rocket_state.sas_active = !rocket_state.sas_active;
            cout << "[SAS] " << (rocket_state.sas_active ? "ON" : "OFF") << endl;
            r_was_pressed = true;
        }
    }
    else {
        r_was_pressed = false;
    }
    if (glfwGetKey(GameContext::getInstance().window, GLFW_KEY_R) == GLFW_PRESS) {
        if (!rcs_key_prev) {
            rocket_state.rcs_active = !rocket_state.rcs_active;
            cout << "[RCS] " << (rocket_state.rcs_active ? "ON" : "OFF") << endl;
            rcs_key_prev = true;
        }
    }
    else {
        rcs_key_prev = false;
    }
    if (glfwGetKey(GameContext::getInstance().window, GLFW_KEY_K) == GLFW_PRESS) {
        if (!k_was_pressed) {
            hud.orbit_reference_sun = !hud.orbit_reference_sun;
            cout << "[REF FRAME] " << (hud.orbit_reference_sun ? "SUN" : "EARTH") << endl;
            k_was_pressed = true;
        }
    }
    else {
        k_was_pressed = false;
    }
    // --- Shift+P 切换云层显示 ---
    bool shift_now = glfwGetKey(GameContext::getInstance().window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS || glfwGetKey(GameContext::getInstance().window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS;
    bool p_now = glfwGetKey(GameContext::getInstance().window, GLFW_KEY_P) == GLFW_PRESS;
    if (shift_now && p_now && !p_was_pressed) {
        show_clouds = !show_clouds;
        cout << "[CLOUDS] " << (show_clouds ? "ON" : "OFF") << endl;
    }
    p_was_pressed = (shift_now && p_now);
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
    // --- Shift+V 切换 SVO 系统 / Auto-activate on landing ---
      // Track auto-activation to avoid re-triggering
     // 0=off, 1=dig, 2=build
    bool v_now = glfwGetKey(GameContext::getInstance().window, GLFW_KEY_V) == GLFW_PRESS;
    // Manual toggle: Shift+V
    if (shift_now && v_now && !v_was_pressed) {
        if (r3d->svoManager) {
            if (r3d->svoManager->hasActiveChunks()) {
                r3d->svoManager->deactivate();
                svo_auto_activated = false;
                svo_dig_mode = 0;
                cout << "[SVO] System Deactivated" << endl;
            }
            else {
                Vec3 subNormal(rocket_state.surf_px, rocket_state.surf_py, rocket_state.surf_pz);
                subNormal = subNormal.normalized();
                Quat unalign_from_z = Quat::fromAxisAngle(Vec3(1.0f, 0.0f, 0.0f), (float)(PI / 2.0));
                subNormal = unalign_from_z.rotate(subNormal);
                r3d->svoManager->activate(subNormal, EARTH_RADIUS * 0.001, r3d->terrain);
                svo_auto_activated = true;
                cout << "[SVO] System Activated at landing site" << endl;
            }
        }
    }
    v_was_pressed = (shift_now && v_now);
    // Auto-activate SVO when rocket lands
    if (r3d->svoManager && !r3d->svoManager->hasActiveChunks() && !svo_auto_activated) {
        if (rocket_state.status == LANDED && rocket_state.altitude < 1000.0) {
            Vec3 subNormal(rocket_state.surf_px, rocket_state.surf_py, rocket_state.surf_pz);
            subNormal = subNormal.normalized();
            Quat unalign_from_z = Quat::fromAxisAngle(Vec3(1.0f, 0.0f, 0.0f), (float)(PI / 2.0));
            subNormal = unalign_from_z.rotate(subNormal);
            r3d->svoManager->activate(subNormal, EARTH_RADIUS * 0.001, r3d->terrain);
            svo_auto_activated = true;
            cout << "[SVO] Auto-activated on landing" << endl;
        }
    }
    // Auto-deactivate SVO when launching
    if (r3d->svoManager && r3d->svoManager->hasActiveChunks()) {
        if (rocket_state.status == ASCEND && rocket_state.altitude > 2000.0) {
            r3d->svoManager->deactivate();
            svo_auto_activated = false;
            svo_dig_mode = 0;
            cout << "[SVO] Auto-deactivated on ascent" << endl;
        }
    }
    // --- B key: Cycle SVO mode (off → dig → build → off) ---
    bool b_now = glfwGetKey(GameContext::getInstance().window, GLFW_KEY_B) == GLFW_PRESS;
    if (b_now && !b_was_pressed && !shift_now) {
        if (r3d->svoManager && r3d->svoManager->hasActiveChunks()) {
            svo_dig_mode = (svo_dig_mode + 1) % 3;
            const char* modeNames[] = { "OFF", "DIG", "BUILD" };
            cout << "[SVO] Mode: " << modeNames[svo_dig_mode] << endl;
        }
    }
    b_was_pressed = b_now;
    // --- Execute dig/build on mouse click (Left mouse button while in mode) ---
    if (r3d->svoManager && r3d->svoManager->hasActiveChunks() && svo_dig_mode > 0) {
        if (svo_op_cooldown > 0) svo_op_cooldown--;
        if (glfwGetMouseButton(GameContext::getInstance().window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS && svo_op_cooldown == 0) {
            // Operate at rocket position in SVO local frame
            Vec3 rocketWorldPos((float)rocket_state.px, (float)rocket_state.py, (float)rocket_state.pz);
            Vec3 svoLocal = r3d->svoManager->planetLocalToSVOLocal(rocketWorldPos);
            double opRadius = 0.005; // ~5 meters
            if (svo_dig_mode == 1) {
                r3d->svoManager->dig(svoLocal, opRadius);
            }
            else if (svo_dig_mode == 2) {
                r3d->svoManager->build(svoLocal, opRadius, SVO::Material::ROCK);
            }
            svo_op_cooldown = 5; // Cooldown frames
        }
    }
    // 全局帧计数器 (用于限制控制台打印频率)
    frame++;
    // --- Simulation & Physics Update ---
    sim_ctrl.handleInput(GameContext::getInstance().window, rocket_state);
    sim_ctrl.update(real_dt, rocket_state, rocket_config, control_input, hud, GameContext::getInstance().window, cam.mode);
    // 只有每隔一定帧数才打印，防止控制台看不清
    // if (frame % 10 == 0)
    //    Report_Status(rocket_state, control_input);
    // Frame rate limited by vsync/simulation
    //画面刷新
    // === 3D/HUD 共享变量与姿态计算 ===
    double ws_d = 0.001;
    
    
    float local_xy_mag = 0;
    // View variables shared with HUD for coordinate projection
    double ro_x = 0, ro_y = 0, ro_z = 0;
    // viewMat, macroProjMat, camEye_rel are class members
    float aspect;
    // Update mouse state for HUD
    {
        double mx_raw, my_raw;
        glfwGetCursorPos(GameContext::getInstance().window, &mx_raw, &my_raw);
        int _ww, _wh;
        glfwGetWindowSize(GameContext::getInstance().window, &_ww, &_wh);
        lmb_prev = lmb;
        lmb = glfwGetMouseButton(GameContext::getInstance().window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS;
        rmb = glfwGetMouseButton(GameContext::getInstance().window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS;
        mouse_x = mx_raw / _ww * 2.0 - 1.0;
        mouse_y = 1.0 - my_raw / _wh * 2.0;
    }
    // ================= 3D 渲染通道 =================
    {
        float earth_r = (float)EARTH_RADIUS * (float)ws_d;
        double r_px = rocket_state.abs_px * ws_d;
        double r_py = rocket_state.abs_py * ws_d;
        double r_pz = rocket_state.abs_pz * ws_d;
        CelestialBody& sun_body = SOLAR_SYSTEM[0];
        double sun_px = sun_body.px * ws_d;
        double sun_py = sun_body.py * ws_d;
        double sun_pz = sun_body.pz * ws_d;
        float sun_radius = (float)sun_body.radius * ws_d;
        double sun_dist_d = sqrt(r_px * r_px + r_py * r_py + r_pz * r_pz);
        // --- 按键监听：视点切换 (CameraDirector) ---
        {
            bool comma_now = glfwGetKey(GameContext::getInstance().window, GLFW_KEY_COMMA) == GLFW_PRESS;
            bool period_now = glfwGetKey(GameContext::getInstance().window, GLFW_KEY_PERIOD) == GLFW_PRESS;
            if (comma_now && !comma_prev) cam.cycleFocusTarget(-1);
            if (period_now && !period_prev) cam.cycleFocusTarget(1);
            comma_prev = comma_now; period_prev = period_now;
        }
        // 【深空姿态锁定】
        double local_dist_sq = rocket_state.px * rocket_state.px + rocket_state.py * rocket_state.py + rocket_state.pz * rocket_state.pz;
        double local_dist = sqrt(local_dist_sq);
        rocketUp = Vec3((float)(rocket_state.px / local_dist), (float)(rocket_state.py / local_dist), (float)(rocket_state.pz / local_dist));
        if (rocket_state.altitude > 2000000.0) {
            rocketUp = Vec3(0.0f, 1.0f, 0.0f);
        }
        // ===== BUILD ROCKET ATTITUDE =====
        if (last_soi != current_soi_index) {
            rocket_state.attitude_initialized = false;
            last_soi = current_soi_index;
        }
        if (!rocket_state.attitude_initialized) {
            rocket_state.attitude = Quat::fromEuler((float)rocket_state.angle, (float)rocket_state.angle_z, (float)rocket_state.angle_roll);
            rocket_state.attitude_initialized = true;
        }
        rocketQuat = rocket_state.attitude;
        Vec3 rocketDir = rocketQuat.rotate(Vec3(0.0f, 1.0f, 0.0f));
        // --- 轨道参考系 ---
        local_xy_mag = sqrt(rocketUp.x * rocketUp.x + rocketUp.y * rocketUp.y);
        if (local_xy_mag > 1e-4) {
            localRight = Vec3((float)(-rocketUp.y / local_xy_mag), (float)(rocketUp.x / local_xy_mag), 0.0f);
        }
        else {
            localRight = Vec3(1.0f, 0.0f, 0.0f);
        }
        localNorth = rocketUp.cross(localRight).normalized();
        Vec3 v_vec_rel((float)rocket_state.vx, (float)rocket_state.vy, (float)rocket_state.vz);
        Vec3 p_vec_rel((float)rocket_state.px, (float)rocket_state.py, (float)rocket_state.pz);
        Vec3 h_vec_rel = p_vec_rel.cross(v_vec_rel);
        Vec3 orbit_normal_rel = h_vec_rel.normalized();
        if (orbit_normal_rel.length() < 0.01f) orbit_normal_rel = Vec3(0, 0, 1);
        Vec3 prograde_rel = v_vec_rel.normalized();
        if (prograde_rel.length() < 0.01f) prograde_rel = localNorth;
        Vec3 radial_rel = orbit_normal_rel.cross(prograde_rel).normalized();
        // 火箭尺寸
        float rocket_vis_scale = 1.0f;
        float rh = (float)rocket_config.height * (float)ws_d * rocket_vis_scale;
        float rw_3d = (float)rocket_config.diameter * (float)ws_d * 0.5f * rocket_vis_scale;
        //==========================================================
        // 新增来自camera.h
        // ===============================================================
        // --- CameraDirector: 计算浮动原点 + 视图矩阵 ---
        // 第一遍: 计算浮动原点 (需传入临时的 renderRocketPos)
        Vec3 renderRocketBase((float)(r_px - r_px), (float)(r_py - r_py), (float)(r_pz - r_pz)); // 暂时置零
        Vec3 renderRocketPos(0, 0, 0); // placeholder
        bool cam_key_w = glfwGetKey(GameContext::getInstance().window, GLFW_KEY_W) == GLFW_PRESS;
        bool cam_key_a = glfwGetKey(GameContext::getInstance().window, GLFW_KEY_A) == GLFW_PRESS;
        bool cam_key_s = glfwGetKey(GameContext::getInstance().window, GLFW_KEY_S) == GLFW_PRESS;
        bool cam_key_d = glfwGetKey(GameContext::getInstance().window, GLFW_KEY_D) == GLFW_PRESS;
        CameraResult camResult = cam.computeFlightCamera(
            rocket_state, ws_d, rh, renderRocketPos, rocketDir, rocketUp,
            localNorth, localRight, prograde_rel, radial_rel, orbit_normal_rel,
            dt, cam_key_w, cam_key_a, cam_key_s, cam_key_d);
        ro_x = camResult.origin_x;
        ro_y = camResult.origin_y;
        ro_z = camResult.origin_z;
        // 用最终浮动原点计算相对坐标
        Vec3 renderEarth((float)(SOLAR_SYSTEM[current_soi_index].px * ws_d - ro_x), (float)(SOLAR_SYSTEM[current_soi_index].py * ws_d - ro_y), (float)(SOLAR_SYSTEM[current_soi_index].pz * ws_d - ro_z));
        Vec3 renderSun((float)(sun_px - ro_x), (float)(sun_py - ro_y), (float)(sun_pz - ro_z));
        renderRocketBase = Vec3((float)(r_px - ro_x), (float)(r_py - ro_y), (float)(r_pz - ro_z));
        renderRocketPos = (rocket_state.status == PRE_LAUNCH || rocket_state.status == LANDED)
            ? (renderRocketBase + rocketUp * (rh * 0.425f))
            : renderRocketBase;
        // 更新全局光照方向 (指向太阳)
        Vec3 lightVec = renderSun - renderRocketBase;
        r3d->lightDir = lightVec.normalized();
        // 第二遍: 用最终的 renderRocketPos 计算视图矩阵 (Orbit/Chase 模式依赖它)
        camResult = cam.computeFlightCamera(
            rocket_state, ws_d, rh, renderRocketPos, rocketDir, rocketUp,
            localNorth, localRight, prograde_rel, radial_rel, orbit_normal_rel,
            0.0, false, false, false, false); // dt=0 避免重复移动 Free 相机
        camEye_rel = camResult.eye;
        Vec3 camTarget_rel = camResult.target;
        viewMat = camResult.viewMatrix;
        // 相机设置
        int ww, wh;
        glfwGetFramebufferSize(GameContext::getInstance().window, &ww, &wh);
        r3d->initTAA(ww, wh);
        // === CRITICAL: Full GL state reset before 3D pass ===
        // 2D renderer endFrame() leaves GL_BLEND enabled and various states dirty.
        // We MUST clean up before 3D rendering or everything renders with alpha blending.
        glViewport(0, 0, ww, wh);
        glDisable(GL_BLEND);
        glEnable(GL_DEPTH_TEST);
        glDepthMask(GL_TRUE);
        glDepthFunc(GL_LESS);
        glDisable(GL_CULL_FACE);
        glFrontFace(GL_CCW);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glBindFramebuffer(GL_FRAMEBUFFER, 0); // Ensure we're on default FBO before glClear
        aspect = (float)ww / (float)wh;
        // --- Camera-Centric Visuals (Day/Night & Atmosphere) ---
        double day_blend = 1.0;
        float alt_factor = 1.0f;
        {
            // Absolute camera position in heliocentric km (ws_d already applied)
            double cam_abs_x = camEye_rel.x + ro_x;
            double cam_abs_y = camEye_rel.y + ro_y;
            double cam_abs_z = camEye_rel.z + ro_z;
            // Find closest celestial body to the camera to determine local environment
            int closest_idx = 0;
            double min_dist_sq = 1e30;
            for (int i = 1; i < SOLAR_SYSTEM.size(); i++) {
                double dx = SOLAR_SYSTEM[i].px * ws_d - cam_abs_x;
                double dy = SOLAR_SYSTEM[i].py * ws_d - cam_abs_y;
                double dz = SOLAR_SYSTEM[i].pz * ws_d - cam_abs_z;
                double d2 = dx * dx + dy * dy + dz * dz;
                if (d2 < min_dist_sq) { min_dist_sq = d2; closest_idx = i; }
            }
            CelestialBody& near_body = SOLAR_SYSTEM[closest_idx];
            double dist_to_center = sqrt(min_dist_sq);
            double body_r_km = near_body.radius * ws_d;
            double cam_alt_km = dist_to_center - body_r_km;
            // Atmosphere factor (0.0 at ground, 1.0 in space)
            alt_factor = (float)fmin(fmax(cam_alt_km / 50.0, 0.0), 1.0);
            // Shadow check (is this body blocking the sun from the camera's POV?)
            double to_sun_x = -cam_abs_x;
            double to_sun_y = -cam_abs_y;
            double to_sun_z = -cam_abs_z;
            double to_sun_len = sqrt(to_sun_x * to_sun_x + to_sun_y * to_sun_y + to_sun_z * to_sun_z);
            if (to_sun_len > 1.0) {
                double d_x = to_sun_x / to_sun_len;
                double d_y = to_sun_y / to_sun_len;
                double d_z = to_sun_z / to_sun_len;
                // Vector from camera to body center
                double oc_x = near_body.px * ws_d - cam_abs_x;
                double oc_y = near_body.py * ws_d - cam_abs_y;
                double oc_z = near_body.pz * ws_d - cam_abs_z;
                double t_closest = oc_x * d_x + oc_y * d_y + oc_z * d_z;
                if (t_closest > 0 && t_closest < to_sun_len) {
                    double cp_x = oc_x - d_x * t_closest;
                    double cp_y = oc_y - d_y * t_closest;
                    double cp_z = oc_z - d_z * t_closest;
                    double closest_dist = sqrt(cp_x * cp_x + cp_y * cp_y + cp_z * cp_z);
                    if (closest_dist < body_r_km) day_blend = 0.0;
                    else if (closest_dist < body_r_km * 1.02) day_blend = (closest_dist - body_r_km) / (body_r_km * 0.02);
                }
            }
        }
        // Clear screen with camera-centric sky color
        if (cam.mode == 2) {
            glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
        }
        else {
            float sky_day = (float)(day_blend * (1.0 - alt_factor));
            glClearColor(0.5f * sky_day, 0.7f * sky_day, 1.0f * sky_day, 1.0f);
        }
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        // 动态远裁剪面 
        float cam_dist = (camEye_rel - camTarget_rel).length();
        // Ensure far clipping plane covers the entire solar system (expanded to 1000 AU)
        float far_plane = 1000.0f * 149597870.0f; // 1000 AU in km (ws_d)
        // =========== PASS 1: MACRO BACKGROUND ===========
        // Compute a smart near plane that keeps the depth buffer ratio (far/near) within
        // ~1e6 to avoid z-fighting on planet surfaces. Find the closest planet surface
        // distance from the camera and use a fraction of that as the near plane.
        float closest_planet_dist = far_plane;
        for (size_t i = 1; i < SOLAR_SYSTEM.size(); i++) {
            Vec3 rp((float)(SOLAR_SYSTEM[i].px * ws_d - ro_x),
                (float)(SOLAR_SYSTEM[i].py * ws_d - ro_y),
                (float)(SOLAR_SYSTEM[i].pz * ws_d - ro_z));
            float body_r = (float)SOLAR_SYSTEM[i].radius * (float)ws_d;
            float atmo_thickness = (SOLAR_SYSTEM[i].type == GAS_GIANT || SOLAR_SYSTEM[i].type == RINGED_GAS_GIANT) ? body_r * 0.05f : 160.0f;
            float dist_to_center = (camEye_rel - rp).length();
            float true_surf_dist = dist_to_center - body_r;
            // Consider terrain displacement (max 25km for Earth)
            float terrain_buffer = (i == 3) ? 25.0f : 0.0f;
            float effective_surf_dist = true_surf_dist - terrain_buffer;
            float atmo_surf_dist = dist_to_center - (body_r + atmo_thickness);
            // Use the atmosphere boundary for camera clipping if it exists
            float geo_dist = (atmo_surf_dist > 0.0f) ? atmo_surf_dist : effective_surf_dist;
            if (geo_dist < closest_planet_dist) {
                closest_planet_dist = geo_dist;
            }
        }
        // Also consider distance to the Sun
        {
            float sun_surf = (camEye_rel - renderSun).length() - sun_radius;
            if (sun_surf < closest_planet_dist)
                closest_planet_dist = sun_surf;
        }
        // Industrial Grade Near Plane:
        // We use a small fraction of the distance to the actual surface (including mountains).
        // If we are extremely close (less than 1km), we force a very small near plane for micro-detail.
        float macro_near = fmaxf(0.00001f, closest_planet_dist * 0.05f); // 5% of distance
        macro_near = fminf(macro_near, 1.0f); // Cap macro near plane at 1km
        if (closest_planet_dist < 1.0f) macro_near = fminf(macro_near, 0.0001f); // 10cm when near ground
        macro_near = fmaxf(macro_near, cam_dist * 0.0001f); // but never clip behind target
        macroProjMat = Mat4::perspective(0.8f, aspect, macro_near, far_plane);
        r3d->beginTAAPass();
        r3d->beginFrame(viewMat, macroProjMat, camEye_rel);
        // ===== SKYBOX: Procedural Starfield + Milky Way =====
        // Calculate vibrancy: 1.0 in space or at night, 0.0 during bright day on ground
        // Note: sky_day factor is used to wash out stars when looking through a lit atmosphere
        float sky_day_local = (float)(day_blend * (1.0 - alt_factor));
        float sky_vibrancy = 1.0f - sky_day_local;
        r3d->drawSkybox(sky_vibrancy, aspect);
        // ===== 太阳物理本体 =====
        if (cam.mode == 2) {
            // 仅在全景模式渲染巨型的物理太阳模型避免遮盖火箭本体细节
            Mat4 sunModel = Mat4::scale(Vec3(sun_radius, sun_radius, sun_radius));
            sunModel = Mat4::translate(renderSun) * sunModel;
            // 复用 earthMesh，修改极高环境光(ambient=2.0)让其纯亮发白发黄
            r3d->drawMesh(earthMesh, sunModel, 1.0f, 0.95f, 0.9f, 1.0f, 2.0f);
            // NOTE: Hardcoded Earth orbit removed in favor of procedural gradient orbits for all planets
        }
        // 渲染整个太阳系
        for (size_t i = 1; i < SOLAR_SYSTEM.size(); i++) {
            CelestialBody& b = SOLAR_SYSTEM[i];
            float r = (float)b.radius * ws_d;
            Vec3 renderPlanet((float)(b.px * ws_d - ro_x), (float)(b.py * ws_d - ro_y), (float)(b.pz * ws_d - ro_z));
            // 应用主体的自转与极轴倾斜 (黄道坐标系中Z轴为原生北极，XY为轨道面)
            // 默认模型球体的极轴是Y轴，首先需要将它躺平(-90度X轴旋转)对齐Z轴
            Quat align_to_z = Quat::fromAxisAngle(Vec3(1.0f, 0.0f, 0.0f), -PI / 2.0f);
            // 主轴自转，绕Z轴（现在的极轴）旋转
            Quat spin = Quat::fromAxisAngle(Vec3(0.0f, 0.0f, 1.0f), b.prime_meridian_epoch + (rocket_state.sim_time * 2.0 * PI / b.rotation_period));
            // 极轴倾斜，绕X轴侧倾
            Quat tilt = Quat::fromAxisAngle(Vec3(1.0f, 0.0f, 0.0f), b.axial_tilt);
            Quat rotation_quat = tilt * spin * align_to_z;
            Mat4 planetModel = Mat4::scale(Vec3(r, r, r));
            planetModel = Mat4::fromQuat(rotation_quat) * planetModel; // Apply rotation
            planetModel = Mat4::translate(renderPlanet) * planetModel;
            // DO NOT dim the base color with rocket occlusion (that makes the darkest side of Earth pitch black void).
             // Compute per-planet lightDir in double precision for correct sun-facing
            {
                double light_dx = sun_body.px - b.px;
                double light_dy = sun_body.py - b.py;
                double light_dz = sun_body.pz - b.pz;
                double light_len = sqrt(light_dx * light_dx + light_dy * light_dy + light_dz * light_dz);
                if (light_len > 1.0) {
                    r3d->lightDir = Vec3((float)(light_dx / light_len), (float)(light_dy / light_len), (float)(light_dz / light_len));
                }
            }
            r3d->drawPlanet(earthMesh, planetModel, b.type, b.r, b.g, b.b, 1.0f, r, (float)rocket_state.sim_time, (int)i);
            if ((b.type == TERRESTRIAL || b.type == GAS_GIANT) && i != 1 && i != 4) {
                // 修改前：float atmo_radius = r * 1.12f;
                // 修改后：使用恒定的物理边界，剥离厚重的多余网格
                float atmo_radius = r + 160.0f;
                Mat4 atmoModel = Mat4::scale(Vec3(atmo_radius, atmo_radius, atmo_radius));
                atmoModel = Mat4::translate(renderPlanet) * atmoModel;
                // New Volumetric Scattering integration with animated hardcore clouds
                r3d->drawAtmosphere(earthMesh, atmoModel, camEye_rel, r3d->lightDir, renderPlanet, r, atmo_radius, (float)rocket_state.sim_time, (int)i, (float)day_blend, show_clouds);
                if (frame <= 2) cout << "[Atmo] body=" << i << " r=" << r << " atmo_r=" << atmo_radius << " day=" << day_blend << " clouds=" << show_clouds << " planet=(" << renderPlanet.x << "," << renderPlanet.y << "," << renderPlanet.z << ")" << endl;
            }
            if (b.type == RINGED_GAS_GIANT) {
                Mat4 ringModel = Mat4::scale(Vec3(r, r, r)); // Mesh is now pre-scaled to R_planet ratios
                ringModel = Mat4::fromQuat(rotation_quat) * ringModel;
                ringModel = Mat4::translate(renderPlanet) * ringModel;
                r3d->drawRing(ringMesh, ringModel, b.r, b.g, b.b, 0.4f);
            }
            //=====================================================
            // 渲染行星轨道和标签 (仅在全景模式下显示)
            //================================================
            if (cam.mode == 2) {
                double a = b.sma_base;
                double e = b.ecc_base;
                double i_inc = b.inc_base;
                double lan = b.lan_base;
                double arg_p = b.arg_peri_base;
                double planet_px = b.px; double planet_py = b.py; double planet_pz = b.pz;
                if (i == 4) { planet_px -= SOLAR_SYSTEM[3].px; planet_py -= SOLAR_SYSTEM[3].py; planet_pz -= SOLAR_SYSTEM[3].pz; }
                int segs = 181; // 181 points = 180 segments + closure
                // Scale orbit line width relative to camera distance so all orbits are visible
                float orbit_center_dist = renderPlanet.length(); // approx dist from render origin to planet
                float cam_to_origin = camEye_rel.length();
                float ref_dist = fmaxf(cam_to_origin, orbit_center_dist);
                float ring_w = fmaxf(earth_r * 0.008f, ref_dist * 0.0035f);
                if (i == 4) ring_w *= 0.5f; // Moon orbit thinner
                // Precompute trig for orbital transform
                double c_O = cos(lan), s_O = sin(lan);
                double c_w = cos(arg_p), s_w = sin(arg_p);
                double c_i = cos(i_inc), s_i = sin(i_inc);
                // Build ONE continuous ribbon — UNIFORM brightness, fully visible
                std::vector<Vec3> orbit_pts;
                std::vector<Vec4> orbit_cols;
                orbit_pts.reserve(segs);
                orbit_cols.reserve(segs);
                // Uniform orbit color — bright, like the rocket orbit reference
                float orb_r = fminf(1.0f, b.r * 0.6f + 0.3f);
                float orb_g = fminf(1.0f, b.g * 0.6f + 0.3f);
                float orb_b = fminf(1.0f, b.b * 0.6f + 0.3f);
                float orb_a = 0.7f;
                for (int k = 0; k < segs; k++) {
                    double E_k = (double)k / (segs - 1) * 2.0 * PI;
                    double nu_k = 2.0 * atan2(sqrt(1.0 + e) * sin(E_k / 2.0), sqrt(1.0 - e) * cos(E_k / 2.0));
                    double r_dist_k = a * (1.0 - e * cos(E_k));
                    double o_xk = r_dist_k * cos(nu_k);
                    double o_yk = r_dist_k * sin(nu_k);
                    // Transform to heliocentric coordinates (double precision)
                    double wx = (c_O * c_w - s_O * s_w * c_i) * o_xk + (-c_O * s_w - s_O * c_w * c_i) * o_yk;
                    double wy = (s_O * c_w + c_O * s_w * c_i) * o_xk + (-s_O * s_w + c_O * c_w * c_i) * o_yk;
                    double wz = (s_w * s_i) * o_xk + (c_w * s_i) * o_yk;
                    if (i == 4) { // Moon orbits Earth
                        wx += SOLAR_SYSTEM[3].px;
                        wy += SOLAR_SYSTEM[3].py;
                        wz += SOLAR_SYSTEM[3].pz;
                    }
                    orbit_pts.push_back(Vec3(
                        (float)(wx * ws_d - ro_x),
                        (float)(wy * ws_d - ro_y),
                        (float)(wz * ws_d - ro_z)
                    ));
                    orbit_cols.push_back(Vec4(orb_r, orb_g, orb_b, orb_a));
                }
                r3d->drawRibbon(orbit_pts, orbit_cols, ring_w);
                // Draw planet marker label - size scales with camera distance
                float dist_to_cam = (renderPlanet - camEye_rel).length();
                float marker_size = fmaxf((float)b.radius * (float)ws_d * 2.0f, dist_to_cam * 0.015f);
                r3d->drawBillboard(renderPlanet, marker_size, b.r, b.g, b.b, 0.9f);
            }
        }
        // Restore lightDir to rocket's own Sun direction (for rocket mesh, trajectory rendering, etc.)
        lightVec = renderSun - renderRocketBase;
        r3d->lightDir = lightVec.normalized();
        // ===== 太阳与镜头光晕 (所有模式可见) =====
        std::vector<Vec4> sun_occluders;
        for (size_t i = 1; i < SOLAR_SYSTEM.size(); i++) {
            CelestialBody& b = SOLAR_SYSTEM[i];
            sun_occluders.push_back(Vec4(
                (float)(b.px * ws_d - ro_x),
                (float)(b.py * ws_d - ro_y),
                (float)(b.pz * ws_d - ro_z),
                (float)b.radius * (float)ws_d
            ));
        }
        r3d->drawSunAndFlare(renderSun, sun_occluders, ww, wh);
        // ===== 历史轨迹线 (实际走过的路径) =====
        {
            DVec3 curPos = { r_px, r_py, r_pz };
            if (traj_history.empty()) {
                traj_history.push_back({ curPos, {r_px - sun_px, r_py - sun_py, r_pz - sun_pz} });
            }
            else {
                DVec3 bk = traj_history.back().e;
                double move_dist = sqrt((r_px - bk.x) * (r_px - bk.x) + (r_py - bk.y) * (r_py - bk.y) + (r_pz - bk.z) * (r_pz - bk.z));
                if (move_dist > earth_r * 0.002) {
                    traj_history.push_back({ curPos, {r_px - sun_px, r_py - sun_py, r_pz - sun_pz} });
                    if (traj_history.size() > 800) {
                        traj_history.erase(traj_history.begin());
                    }
                }
            }
            // 渲染历史轨迹 (更亮实线: 黄绿色), 增加基于相机拉远的线宽补偿 (仅在 Panorama 显示)
            if (cam.mode == 2 && traj_history.size() >= 2) {
                float hist_w = fmaxf(earth_r * 0.01f, cam_dist * 0.0015f);
                float macro_fade = fminf(1.0f, fmaxf(0.0f, (cam.zoom_pan - 0.05f) / 0.1f));
                if (macro_fade > 0.01f) {
                    std::vector<Vec3> relative_traj;
                    for (auto& pt : traj_history) {
                        double w_px, w_py, w_pz;
                        if (hud.orbit_reference_sun) {
                            w_px = sun_px + pt.s.x;
                            w_py = sun_py + pt.s.y;
                            w_pz = sun_pz + pt.s.z;
                        }
                        else {
                            w_px = pt.e.x;
                            w_py = pt.e.y;
                            w_pz = pt.e.z;
                        }
                        relative_traj.push_back(Vec3((float)(w_px - ro_x), (float)(w_py - ro_y), (float)(w_pz - ro_z)));
                    }
                    r3d->drawRibbon(relative_traj, hist_w, 0.4f, 1.0f, 0.3f, 0.8f * macro_fade);
                }
            }
        }
        // ===== 火箭自身绿色高亮标注 (方便在远景找到) =====
        if (cam.mode == 2) {
            // Scale marker more aggressively with zoom, with a guaranteed minimum visible size
            float base_marker = earth_r * 0.025f * fmaxf(1.0f, cam.zoom_pan * 1.2f);
            // Guarantee a minimum screen-space size (proportional to camera distance)
            float cam_dist_marker = (renderRocketBase - camEye_rel).length();
            float min_marker = cam_dist_marker * 0.008f; // ~0.8% of camera distance = always visible
            float marker_size = fmaxf(base_marker, min_marker);
            r3d->drawBillboard(renderRocketBase, marker_size, 0.2f, 1.0f, 0.4f, 0.9f);
            // Draw a second, larger but fainter halo for extreme zoom-out findability
            if (cam.zoom_pan > 2.0f) {
                float halo_size = marker_size * 2.5f;
                r3d->drawBillboard(renderRocketBase, halo_size, 0.2f, 1.0f, 0.4f, 0.15f);
            }
        }
        // ===== 轨道预测线 (开普勒轨道) =====
        std::vector<Vec3> draw_points, draw_mnv_points;
        if (cam.mode == 2) {
            if (hud.adv_orbit_enabled) {
                // Perform asynchronous numerical orbit prediction
                if (!rocket_state.prediction_in_progress) {
                    // Throttle requests: update every 0.1s of real time if not busy (provides "100x efficiency" at all warp rates)
                    auto now = std::chrono::steady_clock::now();
                    float elapsed_real = std::chrono::duration<float>(now - last_req_time).count();
                    if (elapsed_real > 0.1f || rocket_state.predicted_path.empty()) {
                        rocket_state.prediction_in_progress = true;
                        last_req_time = now;
                        // Populate heliocentric state for the background predictor
                        CelestialBody& soi = SOLAR_SYSTEM[current_soi_index];
                        rocket_state.abs_px = rocket_state.px + soi.px;
                        rocket_state.abs_py = rocket_state.py + soi.py;
                        rocket_state.abs_pz = rocket_state.pz + soi.pz;
                        rocket_state.abs_vx = rocket_state.vx + soi.vx;
                        rocket_state.abs_vy = rocket_state.vy + soi.vy;
                        rocket_state.abs_vz = rocket_state.vz + soi.vz;
                        // Reset only if engines are active or large drift (1 hour of sim time)
                        bool force_reset = (control_input.throttle > 0.01) || (std::abs(rocket_state.sim_time - rocket_state.last_prediction_sim_time) > 3600.0);
                        GameContext::getInstance().orbit_predictor->RequestUpdate(&rocket_state, rocket_state, rocket_config, hud.adv_orbit_pred_days, hud.adv_orbit_iters, hud.adv_orbit_ref_mode, hud.adv_orbit_ref_body, hud.adv_orbit_secondary_ref_body, force_reset);
                    }
                }
                {
                    std::lock_guard<std::mutex> lock(*rocket_state.path_mutex);
                    draw_points = rocket_state.predicted_path;
                    draw_mnv_points = rocket_state.predicted_mnv_path;
                }
                // Render predicted paths from async buffers
                float ribbon_w = fmaxf(earth_r * 0.006f, cam_dist * 0.001f);
                // Get current reference body position for world reconstruction
                double rb_px, rb_py, rb_pz;
                PhysicsSystem::GetCelestialPositionAt(hud.adv_orbit_ref_body, rocket_state.sim_time, rb_px, rb_py, rb_pz);
                // Get transformation from local to inertial (world)
                Quat q_local_to_inertial = PhysicsSystem::GetFrameRotation(hud.adv_orbit_ref_mode, hud.adv_orbit_ref_body, hud.adv_orbit_secondary_ref_body, rocket_state.sim_time);
                if (!draw_points.empty()) {
                    bool needs_update = (draw_points.size() != last_draw_points_size) || (draw_points[0].x != last_first_pt.x);
                    if (needs_update) {
                        cached_rel_pts = CatmullRomSpline::interpolate(draw_points, 8);
                        last_draw_points_size = draw_points.size();
                        last_first_pt = draw_points[0];
                    }
                    std::vector<Vec3> world_pts;
                    for (const auto& p : cached_rel_pts) {
                        Vec3 p_rot = q_local_to_inertial.rotate(p);
                        double wx = (rb_px + p_rot.x) * ws_d - ro_x;
                        double wy = (rb_py + p_rot.y) * ws_d - ro_y;
                        double wz = (rb_pz + p_rot.z) * ws_d - ro_z;
                        world_pts.push_back(Vec3((float)wx, (float)wy, (float)wz));
                    }
                    r3d->drawRibbon(world_pts, ribbon_w, 0.4f, 0.8f, 1.0f, 0.85f);
                }
                if (!draw_mnv_points.empty()) {
                    bool needs_update = (draw_mnv_points.size() != last_draw_mnv_points_size) || (draw_mnv_points[0].x != last_mnv_first_pt.x);
                    if (needs_update) {
                        cached_mnv_rel_pts = CatmullRomSpline::interpolate(draw_mnv_points, 8);
                        last_draw_mnv_points_size = draw_mnv_points.size();
                        last_mnv_first_pt = draw_mnv_points[0];
                    }
                    std::vector<Vec3> world_mnv_pts;
                    for (const auto& p : cached_mnv_rel_pts) {
                        Vec3 p_rot = q_local_to_inertial.rotate(p);
                        double wx = (rb_px + p_rot.x) * ws_d - ro_x;
                        double wy = (rb_py + p_rot.y) * ws_d - ro_y;
                        double wz = (rb_pz + p_rot.z) * ws_d - ro_z;
                        world_mnv_pts.push_back(Vec3((float)wx, (float)wy, (float)wz));
                    }
                    // Efficient dashed rendering: draw larger batches
                    for (size_t s = 0; s < world_mnv_pts.size(); s += 20) {
                        std::vector<Vec3> dash;
                        for (size_t j = 0; j < 12 && (s + j < world_mnv_pts.size()); j++) {
                            dash.push_back(world_mnv_pts[s + j]);
                        }
                        if (dash.size() >= 2) r3d->drawRibbon(dash, ribbon_w, 1.0f, 0.6f, 0.1f, 0.9f);
                    }
                }
                // Restore current sim state
                PhysicsSystem::UpdateCelestialBodies(rocket_state.sim_time);
            }
            else
            {
                // ==========================================
                // STANDARD ORBIT PREDICTION: KEPLERIAN (SOI)
                // ==========================================
                // We calculate and draw BOTH Earth-relative AND Sun-relative orbits concurrently!
                for (int ref_idx = 0; ref_idx < 2; ref_idx++) {
                    bool is_sun_ref = (ref_idx == 1);
                    // 选择参考系
                    double G_const = 6.67430e-11;
                    double mu_body = is_sun_ref ? (G_const * SOLAR_SYSTEM[0].mass) : (G_const * SOLAR_SYSTEM[current_soi_index].mass);
                    // 全物理量双精度计算 (标准米)
                    double abs_px = rocket_state.px, abs_py = rocket_state.py, abs_pz = rocket_state.pz;
                    double abs_vx = rocket_state.vx, abs_vy = rocket_state.vy, abs_vz = rocket_state.vz;
                    if (is_sun_ref && current_soi_index != 0) {
                        CelestialBody& cb = SOLAR_SYSTEM[current_soi_index];
                        abs_px += cb.px; abs_py += cb.py; abs_pz += cb.pz;
                        abs_vx += cb.vx; abs_vy += cb.vy; abs_vz += cb.vz;
                    }
                    double r_len = sqrt(abs_px * abs_px + abs_py * abs_py + abs_pz * abs_pz);
                    double v_len = sqrt(abs_vx * abs_vx + abs_vy * abs_vy + abs_vz * abs_vz);
                    if (v_len > 0.001 && r_len > SOLAR_SYSTEM[is_sun_ref ? 0 : current_soi_index].radius * 0.5) {
                        double energy = 0.5 * v_len * v_len - mu_body / r_len;
                        Vec3 h_vec((float)(abs_py * abs_vz - abs_pz * abs_vy),
                            (float)(abs_pz * abs_vx - abs_px * abs_vz),
                            (float)(abs_px * abs_vy - abs_py * abs_vx));
                        float h = h_vec.length();
                        double a = -mu_body / (2.0 * energy);
                        Vec3 v_vec((float)abs_vx, (float)abs_vy, (float)abs_vz);
                        Vec3 p_vec((float)abs_px, (float)abs_py, (float)abs_pz);
                        Vec3 e_vec = v_vec.cross(h_vec) / (float)mu_body - p_vec / (float)r_len;
                        float ecc = e_vec.length();
                        float opacity = (is_sun_ref == hud.orbit_reference_sun) ? 0.9f : 0.3f;
                        if (ecc < 1.0f) {
                            // --- 椭圆轨道 (a > 0) ---
                            float b = (float)a * sqrtf(fmaxf(0.0f, 1.0f - ecc * ecc));
                            Vec3 e_dir = ecc > 1e-6f ? e_vec / ecc : Vec3(1.0f, 0.0f, 0.0f);
                            Vec3 perp_dir = h_vec.normalized().cross(e_dir);
                            float periapsis = (float)a * (1.0f - ecc);
                            float apoapsis = (float)a * (1.0f + ecc);
                            bool will_reenter = periapsis < SOLAR_SYSTEM[is_sun_ref ? 0 : current_soi_index].radius && !is_sun_ref;
                            Vec3 center_off = e_dir * (-(float)a * ecc);
                            // 生成预测轨迹点集
                            std::vector<Vec3> orbit_points;
                            int orbit_segs = 120;
                            float best_orb_dist = 1.0f;
                            float best_orb_ang = -1.0f;
                            Vec3 best_orb_pt_tmp;
                            double n_mean_mot = sqrt(mu_body / (a * a * a));
                            double mx_curr, my_curr;
                            glfwGetCursorPos(GameContext::getInstance().window, &mx_curr, &my_curr);
                            float mxf = (float)(mx_curr / ww * 2.0 - 1.0);
                            float myf = (float)(1.0 - my_curr / wh * 2.0);
                            for (int i = 0; i <= orbit_segs; i++) {
                                float ang = (float)i / orbit_segs * 6.2831853f;
                                Vec3 pt_rel = center_off + e_dir * ((float)a * cosf(ang)) + perp_dir * (b * sinf(ang));
                                double px = pt_rel.x, py = pt_rel.y, pz = pt_rel.z;
                                if (!is_sun_ref) {
                                    px += SOLAR_SYSTEM[current_soi_index].px;
                                    py += SOLAR_SYSTEM[current_soi_index].py;
                                    pz += SOLAR_SYSTEM[current_soi_index].pz;
                                }
                                Vec3 pt = Vec3((float)(px * ws_d - ro_x), (float)(py * ws_d - ro_y), (float)(pz * ws_d - ro_z));
                                // Do not clip orbit points beneath the surface so users can see where they crash
                                orbit_points.push_back(pt);
                                // --- Maneuver Click Hit-test ---
                                if (is_sun_ref == hud.orbit_reference_sun) {
                                    Vec2 scr = ManeuverSystem::projectToScreen(pt, viewMat, macroProjMat, (float)ww / wh);
                                    float d = sqrtf(powf(scr.x - mxf, 2) + powf(scr.y - myf, 2));
                                    if (d < best_orb_dist) {
                                        best_orb_dist = d;
                                        best_orb_ang = ang;
                                        best_orb_pt_tmp = pt;
                                    }
                                }
                            }
                            // Store best hit for this ref frame if it's the active one
                            if (is_sun_ref == hud.orbit_reference_sun && best_orb_dist < 0.05f) {
                                global_best_ang = best_orb_ang;
                                global_best_mu = mu_body;
                                global_best_a = a;
                                global_best_ecc = ecc;
                                global_best_pt = best_orb_pt_tmp;
                                global_best_center = center_off;
                                global_best_e_dir = e_dir;
                                global_best_perp_dir = perp_dir;
                                hud.global_best_ref_node = is_sun_ref ? 0 : current_soi_index;
                                double n = sqrt(mu_body / (a * a * a));
                                double cos_E = (a - r_len) / (a * ecc);
                                double sin_E = (abs_px * abs_vx + abs_py * abs_vy + abs_pz * abs_vz) / (ecc * sqrt(mu_body * a));
                                double E0 = atan2(sin_E, cos_E);
                                global_current_M0 = E0 - ecc * sin(E0);
                                global_current_n = n;
                            }
                            // 渲染预测轨迹
                            float pred_w = fmaxf(earth_r * 0.01f, cam_dist * 0.0015f);
                            float macro_fade = fminf(1.0f, fmaxf(0.0f, (cam.zoom_pan - 0.05f) / 0.1f));
                            if (macro_fade > 0.01f) {
                                if (will_reenter) {
                                    r3d->drawRibbon(orbit_points, pred_w, 1.0f, 0.4f, 0.1f, opacity * macro_fade);
                                }
                                else {
                                    if (is_sun_ref) {
                                        r3d->drawRibbon(orbit_points, pred_w, 0.2f, 0.6f, 1.0f, opacity * macro_fade); // Dimmer/bluer for Sun
                                    }
                                    else {
                                        r3d->drawRibbon(orbit_points, pred_w, 0.2f, 0.8f, 1.0f, opacity * macro_fade);
                                    }
                                }
                            }
                            float apsis_size = fminf(earth_r * 0.12f, earth_r * 0.025f * fmaxf(1.0f, cam.zoom_pan * 0.8f));
                            apsis_size *= (is_sun_ref ? 10.0f : 1.0f);
                            // 远地点标记
                            Vec3 apoPos = e_dir * (-apoapsis);
                            double ax = apoPos.x, ay = apoPos.y, az = apoPos.z;
                            if (!is_sun_ref) {
                                ax += SOLAR_SYSTEM[current_soi_index].px;
                                ay += SOLAR_SYSTEM[current_soi_index].py;
                                az += SOLAR_SYSTEM[current_soi_index].pz;
                            }
                            Vec3 w_apoPos((float)(ax * ws_d - ro_x), (float)(ay * ws_d - ro_y), (float)(az * ws_d - ro_z));
                            r3d->drawBillboard(w_apoPos, apsis_size, 0.2f, 0.4f, 1.0f, opacity);
                            // 近地点标记
                            Vec3 periPos = e_dir * periapsis;
                            double px = periPos.x, py = periPos.y, pz = periPos.z;
                            if (!is_sun_ref) {
                                px += SOLAR_SYSTEM[current_soi_index].px;
                                py += SOLAR_SYSTEM[current_soi_index].py;
                                pz += SOLAR_SYSTEM[current_soi_index].pz;
                            }
                            Vec3 w_periPos((float)(px * ws_d - ro_x), (float)(py * ws_d - ro_y), (float)(pz * ws_d - ro_z));
                            r3d->drawBillboard(w_periPos, apsis_size, 1.0f, 0.5f, 0.1f, opacity);
                        }
                        else {
                            // --- 双曲线/抛物线逃逸轨道 (ecc >= 1.0) ---
                            float a_hyp = fabs(a);
                            float b_hyp = a_hyp * sqrtf(fmaxf(0.0f, ecc * ecc - 1.0f));
                            Vec3 e_dir = e_vec / ecc;
                            Vec3 perp_dir = h_vec.normalized().cross(e_dir);
                            float periapsis = a_hyp * (ecc - 1.0f);
                            Vec3 center_off = e_dir * (a_hyp * ecc);
                            std::vector<Vec3> escape_points;
                            int escape_segs = 60;
                            float max_sinh = 3.0f;
                            for (int i = -escape_segs; i <= escape_segs; i++) {
                                float t = (float)i / escape_segs * max_sinh;
                                Vec3 pt_rel = center_off - e_dir * (a_hyp * coshf(t)) + perp_dir * (b_hyp * sinhf(t));
                                double px = pt_rel.x, py = pt_rel.y, pz = pt_rel.z;
                                if (!is_sun_ref) {
                                    px += SOLAR_SYSTEM[current_soi_index].px;
                                    py += SOLAR_SYSTEM[current_soi_index].py;
                                    pz += SOLAR_SYSTEM[current_soi_index].pz;
                                }
                                Vec3 pt((float)(px * ws_d - ro_x), (float)(py * ws_d - ro_y), (float)(pz * ws_d - ro_z));
                                if (escape_points.empty() || (pt - escape_points.back()).length() > earth_r * 0.05f) {
                                    escape_points.push_back(pt);
                                }
                            }
                            // 逃逸轨道的 Ribbon (紫色代表逃逸)
                            float pred_w = fmaxf(earth_r * 0.006f, cam_dist * 0.001f);
                            float macro_fade = fminf(1.0f, fmaxf(0.0f, (cam.zoom_pan - 0.05f) / 0.1f));
                            if (macro_fade > 0.01f) {
                                if (is_sun_ref) {
                                    r3d->drawRibbon(escape_points, pred_w, 0.9f, 0.2f, 0.8f, opacity * macro_fade);
                                }
                                else {
                                    r3d->drawRibbon(escape_points, pred_w, 0.8f, 0.3f, 1.0f, opacity * macro_fade);
                                }
                            }
                            // 近地点标记
                            float apsis_size = fminf(earth_r * 0.12f, earth_r * 0.025f * fmaxf(1.0f, cam.zoom_pan * 0.8f));
                            apsis_size *= (is_sun_ref ? 10.0f : 1.0f);
                            Vec3 periPos = center_off - e_dir * a_hyp;
                            double px = periPos.x, py = periPos.y, pz = periPos.z;
                            if (!is_sun_ref) {
                                px += SOLAR_SYSTEM[current_soi_index].px;
                                py += SOLAR_SYSTEM[current_soi_index].py;
                                pz += SOLAR_SYSTEM[current_soi_index].pz;
                            }
                            Vec3 w_periPos((float)(px * ws_d - ro_x), (float)(py * ws_d - ro_y), (float)(pz * ws_d - ro_z));
                            r3d->drawBillboard(w_periPos, apsis_size, 1.0f, 0.5f, 0.1f, opacity);
                        }
                    }
                }
            } // close if (hud.adv_orbit_enabled) else block
            // --- Maneuver Nodes & Predicted Orbits ---
            double mx_raw, my_raw; glfwGetCursorPos(GameContext::getInstance().window, &mx_raw, &my_raw);
            float mouse_x = (float)(mx_raw / ww * 2.0 - 1.0);
            float mouse_y = (float)(1.0 - my_raw / wh * 2.0);
            bool lmb = glfwGetMouseButton(GameContext::getInstance().window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS;
            bool rmb = glfwGetMouseButton(GameContext::getInstance().window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS;
            // Render hover circle if mouse is over orbit
            if (global_best_ang >= 0 && rocket_state.selected_maneuver_index == -1) {
                float hover_size = earth_r * 0.03f * fmaxf(1.0f, cam.zoom_pan * 0.5f);
                r3d->drawBillboard(global_best_pt, hover_size, 1.0f, 1.0f, 1.0f, 0.5f);
            }
            popup_clicked_frame = false;
            // Early popup bounds check: if a popup is visible, pre-calculate whether the mouse is inside it
            // to prevent clicks in the popup area from triggering icon/handle/orbit interactions
            if (hud.mnv_popup_index != -1 && cached_popup_w > 0) {
                bool mouse_in_popup_early = (mouse_x >= cached_popup_x - cached_popup_w / 2 && mouse_x <= cached_popup_x + cached_popup_w / 2 &&
                    mouse_y >= cached_popup_y - cached_popup_h / 2 && mouse_y <= cached_popup_y + cached_popup_h / 2);
                if (mouse_in_popup_early) {
                    popup_clicked_frame = true;
                }
            }
            double mu_body = 6.67430e-11 * SOLAR_SYSTEM[current_soi_index].mass;
            float as_ratio = (float)ww / wh;
            // --- Stable Orbit Parameter Extraction (Ensures nodes don't jitter) ---
            Vec3 cur_r_vec((float)rocket_state.px, (float)rocket_state.py, (float)rocket_state.pz);
            Vec3 cur_v_vec((float)rocket_state.vx, (float)rocket_state.vy, (float)rocket_state.vz);
            double cur_r_mag = cur_r_vec.length();
            double cur_v_sq = cur_v_vec.lengthSq();
            Vec3 cur_h_vec = cur_r_vec.cross(cur_v_vec);
            double cur_energy = 0.5 * cur_v_sq - mu_body / cur_r_mag;
            double stable_a = -mu_body / (2.0 * cur_energy);
            Vec3 cur_e_vec = cur_v_vec.cross(cur_h_vec) / (float)mu_body - cur_r_vec / (float)cur_r_mag;
            double stable_ecc = cur_e_vec.length();
            Vec3 stable_e_dir = (stable_ecc > 1e-7f) ? cur_e_vec.normalized() : Vec3(1, 0, 0);
            Vec3 stable_p_dir = cur_h_vec.normalized().cross(stable_e_dir).normalized();
            double stable_n = sqrt(mu_body / (stable_a * stable_a * stable_a));
            double s_b = stable_a * sqrt(fmax(0.0, 1.0 - stable_ecc * stable_ecc));
            double s_cosE = (stable_a - cur_r_mag) / (stable_a * stable_ecc);
            double s_sinE = cur_r_vec.dot(stable_p_dir) / s_b;
            double stable_M0 = atan2(s_sinE, s_cosE) - stable_ecc * sin(atan2(s_sinE, s_cosE));
            Vec3 stable_center = stable_e_dir * (-(float)stable_a * (float)stable_ecc);
            int stable_ref = hud.adv_orbit_enabled ? hud.adv_orbit_ref_body : current_soi_index;
            // Use a local delta for dragging (calculate once per frame)
            float dmx = mouse_x - last_pass_mx;
            float dmy = mouse_y - last_pass_my;
            last_pass_mx = mouse_x;
            last_pass_my = mouse_y;
            for (int i = 0; i < (int)rocket_state.maneuvers.size(); i++) {
                ManeuverNode& node = rocket_state.maneuvers[i];
                // Use ARCHIVED stable orbit from node creation time to reconstruct 3D position
                double ref_px = SOLAR_SYSTEM[node.ref_body].px;
                double ref_py = SOLAR_SYSTEM[node.ref_body].py;
                double ref_pz = SOLAR_SYSTEM[node.ref_body].pz;
                double E_node = node.ref_M0;
                float node_b = (float)node.ref_a * sqrtf(fmaxf(0.0f, 1.0f - (float)node.ref_ecc * (float)node.ref_ecc));
                Vec3 pt_node_rel = node.ref_center + node.ref_e_dir * ((float)node.ref_a * cosf((float)E_node)) + node.ref_p_dir * (node_b * sinf((float)E_node));
                Vec3 node_world = Vec3((float)(ref_px * ws_d + pt_node_rel.x * ws_d - ro_x),
                    (float)(ref_py * ws_d + pt_node_rel.y * ws_d - ro_y),
                    (float)(ref_pz * ws_d + pt_node_rel.z * ws_d - ro_z));
                if (hud.adv_orbit_enabled && i == 0 && !draw_mnv_points.empty()) {
                    // The maneuver node position is the first point of the post-burn path
                    node_world = Vec3((float)(draw_mnv_points[0].x * ws_d - ro_x),
                        (float)(draw_mnv_points[0].y * ws_d - ro_y),
                        (float)(draw_mnv_points[0].z * ws_d - ro_z));
                }
                Vec2 n_scr = ManeuverSystem::projectToScreen(node_world, viewMat, macroProjMat, as_ratio);
                float d_mouse = sqrtf(powf(n_scr.x - mouse_x, 2) + powf(n_scr.y - mouse_y, 2));
                if (i == 0) hit_maneuver_icon = false; // Reset start of list
                // CLICK PRIORITY: Icon (only if popup isn't consuming the click)
                if (lmb && !lmb_prev_mnv && dragging_handle == -1 && !popup_clicked_frame) {
                    if (d_mouse < 0.045f) {
                        rocket_state.selected_maneuver_index = i;
                        hud.mnv_popup_index = i;
                        hit_maneuver_icon = true;
                    }
                }
                // Draw Icon (Highlight if selected or hovered)
                float icon_size = earth_r * (d_mouse < 0.04f ? 0.1f : 0.08f);
                Vec3 icon_col = (rocket_state.selected_maneuver_index == i) ? Vec3(0.4f, 0.8f, 1.0f) : Vec3(0.2f, 0.6f, 1.0f);
                r3d->drawBillboard(node_world, icon_size, icon_col.x, icon_col.y, icon_col.z, 0.9f);
                if (rocket_state.selected_maneuver_index == i) {
                    // Node relative state for handles (3D)
                    // We use node-specific archived parameters for motion as well to ensure handles are spatially correct
                    double E_dot = node.ref_n / (1.0 - node.ref_ecc * cos(E_node));
                    Vec3 v_node_rel = node.ref_e_dir * (-(float)node.ref_a * sinf((float)E_node) * (float)E_dot) + node.ref_p_dir * (node_b * cosf((float)E_node) * (float)E_dot);
                    ManeuverFrame m_frame = ManeuverSystem::getFrame(pt_node_rel, v_node_rel);
                    for (int h = 0; h < 6; h++) {
                        Vec3 h_dir = ManeuverSystem::getHandleDir(m_frame, h);
                        float handle_dist = earth_r * (dragging_handle == h ? 0.35f : 0.2f);
                        Vec3 h_world = node_world + h_dir * handle_dist;
                        Vec2 h_scr = ManeuverSystem::projectToScreen(h_world, viewMat, macroProjMat, as_ratio);
                        float hd = sqrtf(powf(h_scr.x - mouse_x, 2) + powf(h_scr.y - mouse_y, 2));
                        if (lmb && !lmb_prev_mnv && hd < 0.035f && !hit_maneuver_icon && !popup_clicked_frame) {
                            dragging_handle = h;
                            hud.mnv_popup_index = -1; // Hide popup when dragging
                            cached_popup_w = 0; // Clear cache
                        }
                        if (dragging_handle == h && lmb) {
                            Vec3 h_world_next = node_world + h_dir * (handle_dist + 1.0f);
                            Vec2 h_scr_next = ManeuverSystem::projectToScreen(h_world_next, viewMat, macroProjMat, as_ratio);
                            Vec2 axis2D = h_scr_next - n_scr;
                            float screen_len_sq = axis2D.lengthSq();
                            if (screen_len_sq > 1e-8f) {
                                // 1. Immediate displacement-based change
                                float proj_drag = (dmx * axis2D.x + dmy * axis2D.y) / screen_len_sq;
                                float sensitivity = 100.0f * sqrtf(screen_len_sq);
                                if (h >= 2) sensitivity *= 2.5f;
                                float drag_amount = proj_drag * sensitivity;
                                // 2. Continuous rate-based change (Pressure)
                                // Calculate offset from current handle icon to mouse
                                float dx_h = mouse_x - h_scr.x;
                                float dy_h = mouse_y - h_scr.y;
                                float proj_offset = (dx_h * axis2D.x + dy_h * axis2D.y) / screen_len_sq;
                                // If user pulls mouse away from handle center, increase velocity continuously
                                float rate = 30.0f; // 30 m/s^2 base rate per handle length of offset
                                if (h >= 2) rate *= 4.0f;
                                float continuous_amount = proj_offset * rate * (float)dt;
                                float total_change = drag_amount + continuous_amount;
                                if (h == 0) node.delta_v.x += total_change; else if (h == 1) node.delta_v.x -= total_change;
                                else if (h == 2) node.delta_v.y += total_change; else if (h == 3) node.delta_v.y -= total_change;
                                else if (h == 4) node.delta_v.z += total_change; else if (h == 5) node.delta_v.z -= total_change;
                                node.active = true;
                            }
                        }
                        Vec3 h_col = ManeuverSystem::getHandleColor(h);
                        if (hd < 0.035f || dragging_handle == h) h_col = h_col * 1.3f;
                        r3d->drawBillboard(h_world, earth_r * (hd < 0.035f ? 0.05f : 0.04f), h_col.x, h_col.y, h_col.z, 0.9f);
                    }
                    if (node.active) {
                        // Predicted Orbit (Dashed)
                        Vec3 p_pos = pt_node_rel;
                        Vec3 p_vel = v_node_rel + m_frame.prograde * node.delta_v.x + m_frame.normal * node.delta_v.y + m_frame.radial * node.delta_v.z;
                        double p_energy = 0.5 * (double)p_vel.lengthSq() - global_best_mu / (double)p_pos.length();
                        double p_a = -global_best_mu / (2.0 * p_energy);
                        if (p_a > 0) {
                            Vec3 p_h_vec = p_pos.cross(p_vel);
                            Vec3 p_e_vec = p_vel.cross(p_h_vec) / (float)global_best_mu - p_pos / (float)p_pos.length();
                            float p_ecc = p_e_vec.length();
                            float p_b = (float)p_a * sqrtf(fmaxf(0.0f, 1.0f - p_ecc * p_ecc));
                            Vec3 p_e_dir = p_ecc > 1e-6f ? p_e_vec / p_ecc : Vec3(1, 0, 0);
                            Vec3 p_perp = p_h_vec.normalized().cross(p_e_dir);
                            Vec3 p_center = p_e_dir * (-(float)p_a * p_ecc);
                            std::vector<Vec3> p_pts;
                            int samples = 500; // High precision samples
                            for (int s = 0; s <= samples; s++) {
                                float ang = (float)s / (float)samples * 2.0f * PI;
                                Vec3 ptr = p_center + p_e_dir * ((float)p_a * cosf(ang)) + p_perp * (p_b * sinf(ang));
                                p_pts.push_back(Vec3((float)(ref_px * ws_d + ptr.x * ws_d - ro_x), (float)(ref_py * ws_d + ptr.y * ws_d - ro_y), (float)(ref_pz * ws_d + ptr.z * ws_d - ro_z)));
                            }
                            float ribbon_w = fmaxf(earth_r * 0.008f, cam_dist * 0.0012f);
                            // Draw as dashed lines: segments of 6 points with 4 point gaps
                            for (int s = 0; s < samples; s += 10) {
                                std::vector<Vec3> dash;
                                for (int j = 0; j < 6; j++) {
                                    if (s + j <= samples) dash.push_back(p_pts[s + j]);
                                }
                                if (dash.size() >= 2) {
                                    r3d->drawRibbon(dash, ribbon_w, 1.0f, 1.0f, 1.0f, 0.7f);
                                }
                            }
                        }
                    }
                }
            }
            // --- Maneuver Node Popup: LOGIC ONLY (rendering deferred to 2D HUD pass) ---
            // NOTE: renderer->addRect/drawText calls here would be cleared by renderer->beginFrame() later
            hud.mnv_popup_visible = false;
            if (hud.mnv_popup_index != -1 && (size_t)hud.mnv_popup_index < rocket_state.maneuvers.size()) {
                ManeuverNode& node = rocket_state.maneuvers[hud.mnv_popup_index];
                float node_b = (float)node.ref_a * sqrtf(fmaxf(0.0f, 1.0f - (float)node.ref_ecc * (float)node.ref_ecc));
                Vec3 pt_node_rel = node.ref_center + node.ref_e_dir * ((float)node.ref_a * cosf((float)node.ref_M0)) + node.ref_p_dir * (node_b * sinf((float)node.ref_M0));
                Vec3 node_world = Vec3((float)(SOLAR_SYSTEM[node.ref_body].px * ws_d + pt_node_rel.x * ws_d - ro_x),
                    (float)(SOLAR_SYSTEM[node.ref_body].py * ws_d + pt_node_rel.y * ws_d - ro_y),
                    (float)(SOLAR_SYSTEM[node.ref_body].pz * ws_d + pt_node_rel.z * ws_d - ro_z));
                Vec2 n_scr = ManeuverSystem::projectToScreen(node_world, viewMat, macroProjMat, as_ratio);
                // Popup layout - enlarged for sliders, time info, reference frame
                float pop_x, pop_y, pw, ph;
                if (hud.adv_embed_mnv && hud.adv_orbit_menu) {
                    pop_x = hud.mnv_popup_px;
                    pop_y = hud.mnv_popup_py;
                    pw = hud.mnv_popup_pw;
                    ph = hud.mnv_popup_ph;
                }
                else {
                    pop_x = n_scr.x + 0.22f;
                    pop_y = n_scr.y;
                    pw = 0.38f;
                    ph = 0.55f;
                    // Clamp popup to stay within screen bounds
                    if (pop_x + pw / 2 > 0.98f) pop_x = 0.98f - pw / 2;
                    if (pop_x - pw / 2 < -0.98f) pop_x = -0.98f + pw / 2;
                    if (pop_y + ph / 2 > 0.98f) pop_y = 0.98f - ph / 2;
                    if (pop_y - ph / 2 < -0.98f) pop_y = -0.98f + ph / 2;
                }
                // Cache popup bounds
                cached_popup_x = pop_x; cached_popup_y = pop_y;
                cached_popup_w = pw; cached_popup_h = ph;
                // Cache state for deferred rendering
                hud.mnv_popup_visible = true;
                hud.mnv_popup_px = pop_x; hud.mnv_popup_py = pop_y;
                hud.mnv_popup_pw = pw; hud.mnv_popup_ph = ph;
                hud.mnv_popup_node_scr_x = n_scr.x; hud.mnv_popup_node_scr_y = n_scr.y;
                hud.mnv_popup_dv = node.delta_v;
                hud.mnv_popup_ref_body = node.ref_body;
                // Compute time to node or start of burn
                double target_t = node.sim_time;
                hud.mnv_popup_time_to_node = target_t - rocket_state.sim_time;
                if (hud.mnv_popup_time_to_node < -hud.mnv_popup_burn_time) hud.mnv_popup_time_to_node = -hud.mnv_popup_burn_time; // keep showing count-up during burn
                // Compute remaining delta-v: if burn is in progress, compare current velocity
                // against the planned post-burn velocity from the snapshot
                float total_dv_val = node.delta_v.length();
                float remaining_dv = total_dv_val; // Default: full delta-v
                // Populate snapshot when approaching node time (consistent with autopilot)
                if (!node.snap_valid && rocket_state.sim_time >= node.sim_time - 5.0) {
                    int ref_idx = (node.ref_body >= 0) ? node.ref_body : current_soi_index;
                    CelestialBody& ref_b = SOLAR_SYSTEM[ref_idx];
                    double rbpx, rbpy, rbpz, rbvx, rbvy, rbvz;
                    PhysicsSystem::GetCelestialStateAt(ref_idx, node.sim_time, rbpx, rbpy, rbpz, rbvx, rbvy, rbvz);
                    double mu_ref = G_const * ref_b.mass;
                    double npx, npy, npz, nvx, nvy, nvz;
                    get3DStateAtTime(rocket_state.px, rocket_state.py, rocket_state.pz, rocket_state.vx, rocket_state.vy, rocket_state.vz, mu_ref, node.sim_time - rocket_state.sim_time, npx, npy, npz, nvx, nvy, nvz);
                    ManeuverFrame frame = ManeuverSystem::getFrame(Vec3((float)npx, (float)npy, (float)npz), Vec3((float)nvx, (float)nvy, (float)nvz));
                    Vec3 target_dv_world = (frame.prograde * node.delta_v.x + frame.normal * node.delta_v.y + frame.radial * node.delta_v.z);
                    node.locked_burn_dir = target_dv_world.normalized();
                    // Absolute state snapshot (for prediction)
                    node.snap_px = rbpx + npx;
                    node.snap_py = rbpy + npy;
                    node.snap_pz = rbpz + npz;
                    node.snap_vx = rbvx + nvx + target_dv_world.x;
                    node.snap_vy = rbvy + nvy + target_dv_world.y;
                    node.snap_vz = rbvz + nvz + target_dv_world.z;
                    // Relative state snapshot (for guidance stability) - Principia Style
                    node.snap_rel_px = npx;
                    node.snap_rel_py = npy;
                    node.snap_rel_pz = npz;
                    node.snap_rel_vx = nvx + target_dv_world.x;
                    node.snap_rel_vy = nvy + target_dv_world.y;
                    node.snap_rel_vz = nvz + target_dv_world.z;
                    node.snap_time = node.sim_time;
                    node.snap_valid = true;
                }
                if (node.snap_valid) {
                    Vec3 rem_v = ManeuverSystem::calculateRemainingDV(rocket_state, node);
                    remaining_dv = (float)rem_v.length();
                    if (remaining_dv < 0.1f) remaining_dv = 0.0f;
                }
                // Compute estimated burn time using Tsiolkovsky equation
                double current_mass = rocket_state.fuel + rocket_config.dry_mass + rocket_config.upper_stages_mass;
                double max_thrust = 0;
                if (rocket_state.current_stage < (int)rocket_config.stage_configs.size()) {
                    max_thrust = rocket_config.stage_configs[rocket_state.current_stage].thrust;
                }
                if (max_thrust > 0 && current_mass > 0) {
                    double ve = rocket_config.specific_impulse * G0;
                    if (ve > 0 && remaining_dv > 0) {
                        double mass_ratio = exp((double)remaining_dv / ve);
                        double fuel_needed = current_mass * (1.0 - 1.0 / mass_ratio);
                        double mdot = max_thrust / ve;
                        double accel = max_thrust / current_mass;
                        hud.mnv_popup_burn_time = (mdot > 0) ? fuel_needed / mdot : (double)remaining_dv / accel;
                    }
                    else {
                        hud.mnv_popup_burn_time = 0;
                    }
                }
                else {
                    hud.mnv_popup_burn_time = (remaining_dv > 0) ? 9999.0 : 0;
                }
                // --- Slider layout constants (for hit testing) ---
                float slider_track_w = pw * 0.65f;
                float slider_track_h = 0.012f;
                float slider_thumb_w = 0.012f, slider_thumb_h = 0.022f;
                float title_y_layout = pop_y + ph / 2 - (hud.adv_embed_mnv ? 0.015f : 0.025f);
                float slider_base_y = title_y_layout - (hud.adv_embed_mnv ? 0.04f : 0.06f);
                float slider_spacing = hud.adv_embed_mnv ? 0.050f : 0.065f;
                float slider_cx = pop_x + 0.02f; // center of slider track
                float sep_y = slider_base_y - 4 * slider_spacing + 0.025f;
                float info_y = sep_y - 0.05f; // time to node
                info_y -= 0.025f; // estimated burn time
                float mode_btn_y = info_y - 0.045f;
                float del_btn_y = pop_y - ph / 2 + (hud.adv_embed_mnv ? 0.015f : 0.025f);
                // State for deferred rendering
                hud.mnv_popup_burn_mode = node.burn_mode;
                // Close [X] button hit test
                float close_size = 0.028f;
                float close_x = pop_x + pw / 2 - close_size / 2 - 0.008f;
                float close_y = pop_y + ph / 2 - close_size / 2 - 0.008f;
                hud.mnv_popup_close_hover = (!hud.adv_embed_mnv) && (mouse_x >= close_x - close_size / 2 && mouse_x <= close_x + close_size / 2 &&
                    mouse_y >= close_y - close_size / 2 && mouse_y <= close_y + close_size / 2);
                hud.mnv_popup_mini_hover = (hud.adv_embed_mnv) && (mouse_x >= close_x - close_size / 2 && mouse_x <= close_x + close_size / 2 &&
                    mouse_y >= close_y - close_size / 2 && mouse_y <= close_y + close_size / 2);
                if (hud.mnv_popup_mini_hover && lmb && !lmb_prev_mnv) {
                    hud.adv_embed_mnv_mini = !hud.adv_embed_mnv_mini;
                    popup_clicked_frame = true;
                }
                if (!hud.adv_embed_mnv_mini) {
                    // --- Mode Toggle Hit Test ---
                    float mode_btn_w = pw * 0.8f, mode_btn_h = 0.032f;
                    hud.mnv_popup_mode_hover = (mouse_x >= pop_x - mode_btn_w / 2 && mouse_x <= pop_x + mode_btn_w / 2 &&
                        mouse_y >= mode_btn_y - mode_btn_h / 2 && mouse_y <= mode_btn_y + mode_btn_h / 2);
                    if (hud.mnv_popup_mode_hover && lmb && !lmb_prev_mnv) {
                        node.burn_mode = 1 - node.burn_mode;
                        node.snap_valid = false; // Force re-prediction
                        popup_clicked_frame = true;
                    }
                    // DELETE button hit test
                    float del_btn_w = 0.10f, del_btn_h = 0.03f;
                    hud.mnv_popup_del_hover = (mouse_x >= pop_x - del_btn_w / 2 && mouse_x <= pop_x + del_btn_w / 2 &&
                        mouse_y >= del_btn_y - del_btn_h / 2 && mouse_y <= del_btn_y + del_btn_h / 2);
                    // --- Slider dragging logic ---
                    for (int s = 0; s < 4; s++) {
                        float sy = slider_base_y - s * slider_spacing;
                        bool on_track = (mouse_x >= slider_cx - slider_track_w / 2 - 0.02f && mouse_x <= slider_cx + slider_track_w / 2 + 0.02f &&
                            mouse_y >= sy - slider_thumb_h && mouse_y <= sy + slider_thumb_h);
                        if (lmb && !lmb_prev_mnv && on_track && hud.mnv_popup_slider_dragging == -1) {
                            hud.mnv_popup_slider_dragging = s;
                            hud.mnv_popup_slider_drag_x = mouse_x;
                        }
                    }
                }
                if (hud.mnv_popup_slider_dragging >= 0 && lmb) {
                    float drag_offset = mouse_x - hud.mnv_popup_slider_drag_x;
                    float sign = (drag_offset >= 0) ? 1.0f : -1.0f;
                    float abs_offset = fabsf(drag_offset);
                    if (hud.mnv_popup_slider_dragging < 3) {
                        float rate = sign * abs_offset * abs_offset * 5000.0f * (float)dt;
                        if (hud.mnv_popup_slider_dragging == 0) node.delta_v.x += rate;
                        else if (hud.mnv_popup_slider_dragging == 1) node.delta_v.y += rate;
                        else if (hud.mnv_popup_slider_dragging == 2) node.delta_v.z += rate;
                        node.snap_valid = false;
                    }
                    else if (hud.mnv_popup_slider_dragging == 3) {
                        float t_rate = sign * abs_offset * abs_offset * 1000000.0f * (float)dt;
                        node.sim_time += t_rate;
                        if (node.sim_time < rocket_state.sim_time + 10.0) node.sim_time = rocket_state.sim_time + 10.0;
                        node.snap_valid = false;
                    }
                    node.active = true;
                    hud.mnv_popup_dv = node.delta_v;
                }
                if (!lmb) {
                    hud.mnv_popup_slider_dragging = -1;
                }
                // Check if mouse is inside the popup panel
                bool mouse_in_popup = (mouse_x >= pop_x - pw / 2 && mouse_x <= pop_x + pw / 2 &&
                    mouse_y >= pop_y - ph / 2 && mouse_y <= pop_y + ph / 2);
                if (mouse_in_popup || hud.mnv_popup_slider_dragging >= 0) {
                    popup_clicked_frame = true;
                }
                // Click handling: ONLY close via [X] button or DELETE button
                if (lmb && !lmb_prev_mnv) {
                    if (hud.mnv_popup_close_hover) {
                        hud.mnv_popup_index = -1;
                        hud.mnv_popup_visible = false;
                        hud.mnv_popup_slider_dragging = -1;
                        hud.adv_embed_mnv = false;
                        cached_popup_w = 0;
                        popup_clicked_frame = true;
                    }
                    else if (hud.mnv_popup_del_hover) {
                        rocket_state.maneuvers.erase(rocket_state.maneuvers.begin() + hud.mnv_popup_index);
                        if (rocket_state.selected_maneuver_index == hud.mnv_popup_index) rocket_state.selected_maneuver_index = -1;
                        else if (rocket_state.selected_maneuver_index > hud.mnv_popup_index) rocket_state.selected_maneuver_index--;
                        hud.mnv_popup_index = -1;
                        hud.mnv_popup_visible = false;
                        hud.mnv_popup_slider_dragging = -1;
                        cached_popup_w = 0;
                        popup_clicked_frame = true;
                    }
                    else if (mouse_in_popup) {
                        popup_clicked_frame = true;
                    }
                }
            }
            else {
                hud.mnv_popup_slider_dragging = -1;
            }
            // --- Click on empty orbit to create node (Moved to LMB) ---
            if (lmb && !lmb_prev_mnv && !popup_clicked_frame && dragging_handle == -1 && hud.mnv_popup_index == -1 && rocket_state.selected_maneuver_index == -1 && global_best_ang >= 0) {
                double M_click = global_best_ang - global_best_ecc * sin(global_best_ang);
                double dM = M_click - global_current_M0;
                while (dM < 0) dM += 2.0 * PI;
                while (dM > 2.0 * PI) dM -= 2.0 * PI;
                ManeuverNode newNode;
                newNode.sim_time = rocket_state.sim_time + (dM / global_current_n);
                newNode.delta_v = Vec3(0, 0, 0);
                newNode.active = true;
                // ARCHIVE orbital elements at creation time for permanent anchoring
                newNode.ref_a = stable_a;
                newNode.ref_ecc = stable_ecc;
                newNode.ref_n = stable_n;
                newNode.ref_e_dir = stable_e_dir;
                newNode.ref_p_dir = stable_p_dir;
                newNode.ref_center = stable_center;
                newNode.ref_body = stable_ref;
                newNode.ref_M0 = global_best_ang; // Store Eccentric Anomaly actually
                rocket_state.maneuvers.push_back(newNode);
                rocket_state.selected_maneuver_index = (int)rocket_state.maneuvers.size() - 1;
            }
            // Empty click to deselect if didn't hit anything
            if (lmb && !lmb_prev_mnv && !popup_clicked_frame && hud.mnv_popup_index == -1 && dragging_handle == -1 && global_best_ang < 0) {
                rocket_state.selected_maneuver_index = -1;
            }
            // Reset handles if mouse released
            if (!lmb) dragging_handle = -1;
            global_best_ang = -1.0f; // Reset for next frame
            lmb_prev_mnv = lmb;
            rmb_prev_mnv = rmb;
        }
        //<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
        //======================================================================
        //轨道渲染和机动节点物理逻辑
        //======================================================================
        // Dynamic TAA blend: reduce temporal accumulation during fast changes
        // to prevent ghosting/smearing of orbit lines
        if (sim_ctrl.time_warp > 1 || hud.mnv_popup_slider_dragging >= 0) {
            // In time warp or during slider drag, favor current frame heavily
            float warp_blend = (sim_ctrl.time_warp > 100) ? 0.8f : (sim_ctrl.time_warp > 1 ? 0.5f : 0.4f);
            if (hud.mnv_popup_slider_dragging >= 0) warp_blend = fmaxf(warp_blend, 0.6f);
            r3d->taaBlendOverride = warp_blend;
        }
        else {
            r3d->taaBlendOverride = -1.0f; // Use default
        }
        r3d->resolveTAA();
        // =========== PASS 2: MICRO FOREGROUND ===========
        // 微观近景火箭专用的相机矩阵 (极近裁剪面，用于精确绘制 40米的火箭)
        if (cam.mode != 2) {
            // 在近景模式下，清空深度缓存，将火箭置于绝对顶层，杜绝共用一套深度衰减。这里有点问题，被地形遮挡也能显示
            // 全景模式下不清空，保留真实的物理穿模(躲在地球后面会被遮挡)的正确视角。
            glClear(GL_DEPTH_BUFFER_BIT);
        }
        float micro_near = fmaxf(rh * 0.05f, cam_dist * 0.002f);
        float micro_far = fmaxf(cam_dist * 10.0f, 15000.0f);
        Mat4 microProjMat = Mat4::perspective(0.8f, aspect, micro_near, micro_far);
        r3d->beginFrame(viewMat, microProjMat, camEye_rel);
        // ===== 火箭涂装 (Assembly-based per-part rendering) =====
        Vec3 engNozzlePos = renderRocketBase; // Fallback to base
        {
            int render_start = 0;
            if (rocket_state.current_stage < (int)rocket_config.stage_configs.size()) {
                render_start = rocket_config.stage_configs[rocket_state.current_stage].part_start_index;
            }
            // Find lowest active engine for flame positioning
            float min_ey = 1e10f;
            for (int pi = render_start; pi < (int)assembly.parts.size(); pi++) {
                const PlacedPart& pp = assembly.parts[pi];
                if (PART_CATALOG[pp.def_id].category == CAT_ENGINE) {
                    if (pp.pos.y < min_ey) {
                        min_ey = pp.pos.y;
                        engNozzlePos = renderRocketBase + rocketQuat.rotate(pp.pos * (float)ws_d);
                    }
                }
            }
            for (int pi = render_start; pi < (int)assembly.parts.size(); pi++) {
                const PlacedPart& pp = assembly.parts[pi];
                const PartDef& def = PART_CATALOG[pp.def_id];
                for (int s = 0; s < pp.symmetry; s++) {
                    float symAngle = (s * 2.0f * 3.14159f) / pp.symmetry;
                    Vec3 localPos = pp.pos;
                    if (pp.symmetry > 1) {
                        float dist = sqrtf(pp.pos.x * pp.pos.x + pp.pos.z * pp.pos.z);
                        if (dist > 0.01f) {
                            float curAngle = atan2f(pp.pos.z, pp.pos.x);
                            localPos.x = cosf(curAngle + symAngle) * dist;
                            localPos.z = sinf(curAngle + symAngle) * dist;
                        }
                    }
                    Vec3 partWorldPos = renderRocketBase + rocketQuat.rotate(localPos * (float)ws_d);
                    Quat partWorldRot = rocketQuat * pp.rot * Quat::fromAxisAngle(Vec3(0, 1, 0), symAngle);
                    float pd = def.diameter * (float)ws_d;
                    float ph = def.height * (float)ws_d;
                    Vec3 partCenter = partWorldPos + partWorldRot.rotate(Vec3(0, ph * 0.5f, 0));
                    if (def.category == CAT_NOSE_CONE) {
                        Mat4 partMat = Mat4::TRS(partWorldPos, partWorldRot, { pd, ph, pd });
                        r3d->drawMesh(rocketNose, partMat, def.r, def.g, def.b, 1.0f, 0.2f);
                    }
                    else if (def.category == CAT_ENGINE) {
                        float bf = 0.4f; float nf = 1.0f - bf;
                        Vec3 bodyPos = partWorldPos + partWorldRot.rotate(Vec3(0, ph * (1.0f - bf * 0.5f), 0));
                        Mat4 bodyMat = Mat4::TRS(bodyPos, partWorldRot, { pd * 0.6f, ph * bf, pd * 0.6f });
                        r3d->drawMesh(rocketBody, bodyMat, 0.2f, 0.2f, 0.22f, 1.0f, 0.4f);
                        Mat4 bellMat = Mat4::TRS(partWorldPos, partWorldRot, { pd * 0.85f, ph * nf, pd * 0.85f });
                        r3d->drawMesh(rocketNose, bellMat, def.r * 0.8f, def.g * 0.8f, def.b * 0.8f, 1.0f, 0.1f);
                    }
                    else if (def.category == CAT_STRUCTURAL) {
                        if (strstr(def.name, "Fin") || strstr(def.name, "Solar")) {
                            Mat4 finMat = Mat4::TRS(partCenter, partWorldRot, { pd * 0.05f, ph, pd * 0.5f });
                            r3d->drawMesh(rocketBox, finMat, def.r, def.g, def.b, 1.0f, 0.1f);
                        }
                        else if (strstr(def.name, "Leg")) {
                            Mat4 legMat = Mat4::TRS(partCenter, partWorldRot, { pd * 0.15f, ph, pd * 0.15f });
                            r3d->drawMesh(rocketBox, legMat, def.r, def.g, def.b, 1.0f, 0.1f);
                        }
                        else {
                            Mat4 partMat = Mat4::TRS(partCenter, partWorldRot, { pd, ph, pd });
                            r3d->drawMesh(rocketBody, partMat, def.r, def.g, def.b, 1.0f, 0.2f);
                        }
                    }
                    else {
                        Mat4 partMat = Mat4::TRS(partCenter, partWorldRot, { pd, ph, pd });
                        r3d->drawMesh(rocketBody, partMat, def.r, def.g, def.b, 1.0f, 0.2f);
                    }
                }
            }
        }
        //=============火箭涂装==========================
        // ===== 发射台渲染 (Launch Pad Generation / Rendering) =====
        if (rocket_state.altitude < 12000.0) {
            CelestialBody& body = SOLAR_SYSTEM[current_soi_index];
            // Launch pad should rotate with the body (Axial Tilt + Rotation)
            double theta = body.prime_meridian_epoch + (rocket_state.sim_time * 2.0 * PI / body.rotation_period);
            Quat rot = Quat::fromAxisAngle(Vec3(0, 0, 1), (float)theta);
            Quat tilt = Quat::fromAxisAngle(Vec3(1, 0, 0), (float)body.axial_tilt);
            Quat full_rot = tilt * rot;
            // Use FIXED launch site coordinates instead of dynamic ground track
            Vec3 s_pos((float)rocket_state.launch_site_px, (float)rocket_state.launch_site_py, (float)rocket_state.launch_site_pz);
            // Normalized local up vector
            Vec3 localUp = s_pos.normalized();
            // Position on surface (exactly at terrain elevation)
            Vec3 s_surf = localUp * s_pos.length();
            // Rotate to inertial frame
            Vec3 i_surf = full_rot.rotate(s_surf);
            Vec3 i_up = full_rot.rotate(localUp);
            Vec3 padCenter(
                (float)((body.px + (double)i_surf.x) * ws_d - ro_x),
                (float)((body.py + (double)i_surf.y) * ws_d - ro_y),
                (float)((body.pz + (double)i_surf.z) * ws_d - ro_z)
            );
            // Calculate local orientation for the pad
            Vec3 padUp = i_up;
            // Calculate a local right/forward to orient the pad
            Vec3 defaultRight(1, 0, 0);
            if (fabs(padUp.dot(defaultRight)) > 0.9f) defaultRight = Vec3(0, 1, 0);
            Vec3 padRight = padUp.cross(defaultRight).normalized();
            Vec3 padForward = padUp.cross(padRight).normalized();
            // Construct rotation matrix for the pad to face "up"
            Mat4 padRot;
            padRot.m[0] = padRight.x; padRot.m[1] = padRight.y; padRot.m[2] = padRight.z; padRot.m[3] = 0;
            padRot.m[4] = padUp.x;    padRot.m[5] = padUp.y;    padRot.m[6] = padUp.z;    padRot.m[7] = 0;
            padRot.m[8] = padForward.x; padRot.m[9] = padForward.y; padRot.m[10] = padForward.z; padRot.m[11] = 0;
            padRot.m[12] = 0;         padRot.m[13] = 0;         padRot.m[14] = 0;         padRot.m[15] = 1;
            if (has_launch_pad) {
                float pad_scale = earth_r * 0.005f;
                Mat4 padModel = Mat4::scale(Vec3(pad_scale, pad_scale, pad_scale));
                padModel = padRot * padModel; // Orient pad
                padModel = Mat4::translate(padCenter) * padModel;
                r3d->drawMesh(launchPadMesh, padModel, 0.4f, 0.4f, 0.42f, 1.0f, 0.2f);
            }
            else {
                float pad_w = rw_3d * 20.0f;
                float pad_h = rh * 0.4f; // Increased pad height to bury it deeper
                Mat4 baseMdl = Mat4::scale(Vec3(pad_w, pad_h, pad_w));
                baseMdl = padRot * baseMdl;
                // Bury the bottom half of the pad base into the planet to eliminate gaps
                baseMdl = Mat4::translate(padCenter - padUp * (pad_h * 0.45f)) * baseMdl;
                r3d->drawMesh(rocketBox, baseMdl, 0.4f, 0.4f, 0.4f, 1.0f, 0.1f);
                float tower_h = rh * 1.5f;
                float tower_w = rw_3d * 3.0f;
                Vec3 towerCenter = padCenter + padRight * (rw_3d * 4.0f) + padUp * (tower_h * 0.5f - pad_h * 0.5f);
                Mat4 towerMdl = Mat4::scale(Vec3(tower_w, tower_h, tower_w));
                towerMdl = padRot * towerMdl;
                towerMdl = Mat4::translate(towerCenter) * towerMdl;
                r3d->drawMesh(rocketBox, towerMdl, 0.7f, 0.15f, 0.15f, 1.0f, 0.3f);
            }
        }
        //=================================================
        //发射台渲染逻辑
        // ===== 工业级 3D 体积尾焰 (Volumetric Raymarched Plume for ALL active engines) =====
        //==============================================================
        //这个应当被模块化进入pulme.h和pulme.cpp
        //===============================================================
        if (rocket_state.thrust_power > 0.01) {
            float thrust = (float)control_input.throttle;
            float expansion = (float)fmax(0.0, 1.0 - PhysicsSystem::get_air_density(rocket_state.altitude) / 1.225);
            float thrust_scale = 0.25f + 0.75f * powf(thrust, 1.5f);
            int render_start = 0;
            if (rocket_state.current_stage < (int)rocket_config.stage_configs.size()) {
                render_start = rocket_config.stage_configs[rocket_state.current_stage].part_start_index;
            }
            // Iterate through all active engines to render plumes
            for (int pi = render_start; pi < (int)assembly.parts.size(); pi++) {
                const PlacedPart& pp = assembly.parts[pi];
                const PartDef& def = PART_CATALOG[pp.def_id];
                if (def.category == CAT_ENGINE) {
                    for (int s = 0; s < pp.symmetry; s++) {
                        float symAngle = (s * 2.0f * 3.14159f) / pp.symmetry;
                        Vec3 localPos = pp.pos;
                        if (pp.symmetry > 1) {
                            float dist = sqrtf(pp.pos.x * pp.pos.x + pp.pos.z * pp.pos.z);
                            if (dist > 0.01f) {
                                float curAngle = atan2f(pp.pos.z, pp.pos.x);
                                localPos.x = cosf(curAngle + symAngle) * dist;
                                localPos.z = sinf(curAngle + symAngle) * dist;
                            }
                        }
                        Vec3 nozzleWorldPos = renderRocketBase + rocketQuat.rotate(localPos * (float)ws_d);
                        Quat nozzleWorldRot = rocketQuat * pp.rot * Quat::fromAxisAngle(Vec3(0, 1, 0), symAngle);
                        Vec3 nozzleDir = nozzleWorldRot.rotate(Vec3(0, 1, 0)); // Direction engine is pointing
                        float engine_ph = def.height * (float)ws_d;
                        float plume_len = engine_ph * 4.2f * thrust_scale * (1.0f + expansion * 1.0f);
                        float plume_dia = def.diameter * (float)ws_d * 2.0f * (1.1f + expansion * 4.2f);
                        float groundDist = (float)rocket_state.altitude * (float)ws_d;
                        float ground_contact_depth = fmaxf(0.0f, plume_len - groundDist);
                        float splash_factor = ground_contact_depth / fmaxf(0.001f, plume_len);
                        plume_dia *= (1.0f + splash_factor * 8.0f);
                        Vec3 plumePos = nozzleWorldPos - nozzleDir * (plume_len * 0.5f);
                        Mat4 plumeMdl = Mat4::TRS(plumePos, nozzleWorldRot, Vec3(plume_dia, plume_len, plume_dia));
                        r3d->drawExhaustVolumetric(rocketBox, plumeMdl, thrust, expansion, (float)glfwGetTime(), groundDist, plume_len);
                    }
                }
            }
        }
        r3d->endFrame();
    }
    }
    void render() override {
        Renderer* renderer = GameContext::getInstance().renderer2d;
        Renderer3D* r3d = GameContext::getInstance().renderer3d;
        // --- 2D HUD Rendering (on top of 3D scene) ---
        glClear(GL_DEPTH_BUFFER_BIT);
        renderer->beginFrame();
        HUDContext hud_ctx;
        hud_ctx.renderer = renderer;
        hud_ctx.rocket_state = &rocket_state;
        hud_ctx.rocket_config = &rocket_config;
        hud_ctx.control_input = &control_input;
        hud_ctx.cam = &cam;
        int ww, wh;
        glfwGetWindowSize(GameContext::getInstance().window, &ww, &wh);
        hud_ctx.ww = ww; hud_ctx.wh = wh;
        hud_ctx.aspect = (float)ww / (float)wh;
        hud_ctx.time_warp = sim_ctrl.time_warp;
        hud_ctx.dt = dt;
        hud_ctx.window = GameContext::getInstance().window;
        hud_ctx.assembly = &GameContext::getInstance().launch_assembly;
        hud_ctx.r3d = r3d;
        hud_ctx.frame = frame;
        hud_ctx.ws_d = ws_d;
        hud_ctx.rocketQuat = &rocketQuat;
        hud_ctx.rocketUp = &rocketUp;
        hud_ctx.localNorth = &localNorth;
        hud_ctx.localRight = &localRight;
        hud_ctx.ro_x = rocket_state.px;
        hud_ctx.ro_y = rocket_state.py;
        hud_ctx.ro_z = rocket_state.pz;
        hud_ctx.viewMat = viewMat;
        hud_ctx.macroProjMat = macroProjMat;
        hud_ctx.camEye_rel = camEye_rel;
        hud_ctx.mouse_x = mouse_x;
        hud_ctx.mouse_y = mouse_y;
        hud_ctx.lmb = lmb;
        hud_ctx.lmb_prev = lmb_prev;
        hud_ctx.rmb = rmb;
        hud_ctx.global_best_ang = &global_best_ang;
        hud.render(hud_ctx);
        renderer->endFrame();
    }
};