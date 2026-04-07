#include "render/HUD_system.h"
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

// Any other includes you need
#include "flight_input_system.h"
#include "maneuver_manager.h"
#include "orbit_system.h"
#include "plume_manager.h"
#include "spaceport_manager.h"
class FlightScene : public IScene {
public:
    // === Core Members ===
    Quat rocketQuat;
    Vec3 rocketUp, localNorth, localRight;
    FlightHUD hud;
    InputRouter input;
    double ws_d = 0.001;
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
    SimulationController sim_ctrl;
    double dt = 0.02;
    double real_dt = 0.02;
    int frame = 0;
    // === Variables Extracted from Static ===
    OrbitSystem orbitSystem;
    PlumeManager plumeManager;
    SpaceportManager spaceportManager;
    ManeuverManager mnvManager;
    bool terrain_adjusted = false;
    bool comma_prev = false;
    bool period_prev = false;
    FlightInputSystem inputSystem;
    int last_soi = -1;
    bool show_clouds = true;
    void onEnter() override {
        GameContext& ctx = GameContext::getInstance();
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
    inputSystem.setup(input, rocket_state, rocket_config, control_input, hud, cam, show_clouds);
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
        real_dt = dt;
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
            mnvManager.update(GameContext::getInstance().window, rocket_state, hud, cam, dt, ww, wh);
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
        sim_ctrl.handleInput(GameContext::getInstance().window, rocket_state);
        sim_ctrl.update(real_dt, rocket_state, rocket_config, control_input, hud, GameContext::getInstance().window, cam.mode);
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
        Renderer* renderer = GameContext::getInstance().renderer2d;
        Renderer3D* r3d = GameContext::getInstance().renderer3d;
        const RocketAssembly& assembly = GameContext::getInstance().launch_assembly;
        // ================= 3D 渲染通道 =================
        {
            float local_xy_mag = 0;
            double ro_x = 0, ro_y = 0, ro_z = 0;
            float aspect = 1.0f;
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
            if (cam.mode == 2) {
                // ===== 历史轨迹线与轨道预测线 (已转移至 OrbitSystem) =====
                orbitSystem.render(rocket_state, rocket_config, hud, mnvManager, r3d, cam, control_input, viewMat, macroProjMat, aspect, ws_d, ro_x, ro_y, ro_z, ww, wh, dt, current_soi_index, earth_r, cam_dist, renderRocketBase, camEye_rel);
                // --- Maneuver Nodes & Interaction ---
                mnvManager.render(rocket_state, hud, r3d, viewMat, macroProjMat, aspect, earth_r, cam_dist, ws_d, ro_x, ro_y, ro_z, dt);
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
                        PartRenderer::drawPartWithSymmetry(r3d, PART_CATALOG[pp.def_id], partWorldPos, partWorldRot, 
                                                          rocketBody, rocketNose, rocketBox,
                                                          (float)ws_d, false, false, 1.0f, 1);
                    }
                }
            }
            //=============火箭涂装==========================
            // ===== 发射台渲染 (已转移至 SpaceportManager) =====
            spaceportManager.render(r3d, rocket_state, current_soi_index, ws_d, ro_x, ro_y, ro_z, rocketBox, rw_3d, rh);
            //=================================================
            //发射台渲染逻辑
            // ===== 体积尾焰渲染 (已转移至 PlumeManager) =====
            plumeManager.render(rocket_state, rocket_config, control_input, assembly, r3d, rocketBox, rocketQuat, renderRocketBase, ws_d);
            r3d->endFrame();
        }
        // ================= 2D HUD 渲染通道 (叠加在 3D 之上) =================
        {
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
            hud_ctx.global_best_ang = &mnvManager.global_best_ang;
            hud.render(hud_ctx);
            renderer->endFrame();
        }
        // Update state for next frame ONLY at the very end
        lmb_prev = lmb;
    }
};