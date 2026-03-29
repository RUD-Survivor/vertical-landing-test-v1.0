#pragma once
#include "scene.h"
#include "game_context.h"
#include "menu_system.h"
#include "save_system.h"
#include "simulation/rocket_builder.h"

extern float g_scroll_y;

class WorkshopScene : public IScene {
public:
    bool done = false;

    void update(double dt) override {
        GameContext& ctx = GameContext::getInstance();
        GLFWwindow* window = ctx.window;
        Renderer* renderer = ctx.renderer2d;
        
        bool load_from_save = ctx.skip_builder;
        MenuSystem::MenuChoice menu_choice = load_from_save ? MenuSystem::MENU_CONTINUE : MenuSystem::MENU_VAB;

// Transition to Rocket Building/Flight
load_from_save = (menu_choice == MenuSystem::MENU_CONTINUE);
bool skip_builder = false; // Default behavior
bool factory_mode_active = false;

// =========================================================
// 初始化 3D 渲染器和网格 (Early instantiation for Workshop)
// =========================================================
Renderer3D* r3d = new Renderer3D();
Mesh earthMesh = MeshGen::sphere(256, 512, 1.0f);  // Extreme-res unit sphere for terrain detail
Mesh ringMesh = MeshGen::ring(128, 1.11f, 2.35f);  // NASA Real Ratios: D-ring start to F-ring end (1.11R to 2.35R)
Mesh rocketBody = MeshGen::cylinder(32, 1.0f, 1.0f);
Mesh rocketNose = MeshGen::cone(32, 1.0f, 1.0f);
Mesh rocketBox = MeshGen::box(1.0f, 1.0f, 1.0f);

// Try to load launch pad model
Mesh launchPadMesh = ModelLoader::loadOBJ("assets/launch_pad.obj");
bool has_launch_pad = (launchPadMesh.indexCount > 0);

// =========================================================
// 3. BUILD 阶段：KSP 式火箭组装
// 玩家在这里把零件（发动机、油箱等）拼凑在一起。
// =========================================================
BuilderState builder_state;
RocketState loaded_state;
ControlInput loaded_input;
// skip_builder is already set by Hub transition logic above

if (load_from_save) {
    // 从存档加载
    if (SaveSystem::LoadGame(builder_state.assembly, loaded_state, loaded_input)) {
        skip_builder = true;
        cout << ">> SAVE FILE LOADED SUCCESSFULLY!" << endl;
    }
    else {
        cout << ">> FAILED TO LOAD SAVE FILE, STARTING NEW GAME" << endl;
        // 加载失败,使用默认火箭
        builder_state.assembly.addPart(9);   // Raptor engine
        builder_state.assembly.addPart(6);   // Medium fuel tank 100t
        builder_state.assembly.addPart(0);   // Standard fairing nosecone
    }
}
else {
    // 新游戏,使用默认火箭
    builder_state.assembly.addPart(9);   // Raptor engine
    builder_state.assembly.addPart(6);   // Medium fuel tank 100t
    builder_state.assembly.addPart(0);   // Standard fairing nosecone
}

bool build_done = skip_builder;

BuilderKeyState bk_prev = {};

while (!build_done && !glfwWindowShouldClose(window)) {
    // 处理窗口事件（如按键、鼠标移动）
    glfwPollEvents();
    if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
        glfwSetWindowShouldClose(window, true);

    // Read builder inputs
    BuilderKeyState bk_now;
    bk_now.up = glfwGetKey(window, GLFW_KEY_UP) == GLFW_PRESS;
    bk_now.down = glfwGetKey(window, GLFW_KEY_DOWN) == GLFW_PRESS;
    bk_now.left = glfwGetKey(window, GLFW_KEY_LEFT) == GLFW_PRESS;
    bk_now.right = glfwGetKey(window, GLFW_KEY_RIGHT) == GLFW_PRESS;
    bk_now.enter = glfwGetKey(window, GLFW_KEY_ENTER) == GLFW_PRESS;
    bk_now.del = glfwGetKey(window, GLFW_KEY_DELETE) == GLFW_PRESS || glfwGetKey(window, GLFW_KEY_BACKSPACE) == GLFW_PRESS;
    bk_now.tab = glfwGetKey(window, GLFW_KEY_TAB) == GLFW_PRESS;
    bk_now.pgup = glfwGetKey(window, GLFW_KEY_PAGE_UP) == GLFW_PRESS;
    bk_now.pgdn = glfwGetKey(window, GLFW_KEY_PAGE_DOWN) == GLFW_PRESS;
    bk_now.space = glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_PRESS;
    bk_now.q = glfwGetKey(window, GLFW_KEY_Q) == GLFW_PRESS;
    bk_now.e = glfwGetKey(window, GLFW_KEY_E) == GLFW_PRESS;
    bk_now.a = glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS;
    bk_now.d = glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS;
    bk_now.w = glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS;
    bk_now.s = glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS;

    // Mouse for builder
    double bmx, bmy;
    glfwGetCursorPos(window, &bmx, &bmy);
    int bww, bwh;
    glfwGetWindowSize(window, &bww, &bwh);
    bk_now.mx = (float)(bmx / bww * 2.0 - 1.0);
    bk_now.my = (float)(1.0 - bmy / bwh * 2.0);
    bk_now.lmb = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS;
    bk_now.rmb = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS;

    // 核心逻辑：处理建造者的交互（如：零件点击、拖拽、吸附逻辑）
    build_done = builderHandleInput(builder_state, bk_now, bk_prev);
    bk_prev = bk_now;

    // Then process camera controls based on updated builder state
    static double workshop_prev_mx = 0, workshop_prev_my = 0;
    static bool workshop_is_dragging = false;
    static bool allowed_rotation = false;

    if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS) {
        double mx, my;
        glfwGetCursorPos(window, &mx, &my);

        if (!workshop_is_dragging) {
            workshop_prev_mx = mx;
            workshop_prev_my = my;
            workshop_is_dragging = true;
            // Only allow rotation if no part menu was JUST opened this frame
            allowed_rotation = !builder_state.show_part_menu;
        }
        else if (allowed_rotation) {
            float dx = (float)(mx - workshop_prev_mx) * 0.005f;
            float dy = (float)(my - workshop_prev_my) * 0.005f;
            builder_state.orbit_angle -= dx;
            builder_state.orbit_pitch = std::max(-0.5f, std::min(1.5f, builder_state.orbit_pitch + dy));
            workshop_prev_mx = mx;
            workshop_prev_my = my;
        }
    }
    else {
        workshop_is_dragging = false;
        allowed_rotation = false;
        if (!builder_state.show_part_menu) builder_state.orbit_angle += 0.001f;
    }

    // Zoom and Pan controls
    if (g_scroll_y != 0.0f) {
        if (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS || glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS) {
            builder_state.cam_dist *= powf(0.85f, g_scroll_y);
            builder_state.cam_dist = std::max(2.0f, std::min(100.0f, builder_state.cam_dist));
        }
        else {
            builder_state.cam_y_offset += g_scroll_y * (builder_state.cam_dist * 0.05f);
        }
        g_scroll_y = 0.0f;
    }

    // --- 3D WORKSHOP RENDER PASS ---
    int ww, wh;
    glfwGetFramebufferSize(window, &ww, &wh);
    r3d->lightDir = Vec3(0.5f, 1.0f, 0.3f).normalized(); // Angled floodlight

    // Dynamic camera based on rocket height and manual pan
    float current_height = std::max(5.0f, builder_state.assembly.total_height);
    float look_y = current_height * 0.4f + builder_state.cam_y_offset;

    Vec3 camTarget(0.0f, look_y, 0.0f);
    float dist = builder_state.cam_dist + current_height * 0.5f;
    float cy = sinf(builder_state.orbit_pitch) * dist;
    float cx = cosf(builder_state.orbit_pitch) * cosf(builder_state.orbit_angle) * dist;
    float cz = cosf(builder_state.orbit_pitch) * sinf(builder_state.orbit_angle) * dist;
    Vec3 camEye = camTarget + Vec3(cx, cy, cz);

    Mat4 viewMat = Mat4::lookAt(camEye, camTarget, Vec3(0.0f, 1.0f, 0.0f));
    Mat4 projMat = Mat4::perspective(1.0f, (float)ww / (float)wh, 0.1f, 1000.0f);

    // --- 3D 建造室渲染 (Workshop Render Pass) ---
    // 我们在这里把组装好的火箭画在屏幕上，让玩家看到 3D 模型。
    r3d->beginFrame(viewMat, projMat, camEye);

    // Dark foggy background for the massive VAB interior
    glClearColor(0.02f, 0.025f, 0.03f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // Draw Workshop Environment
    Mat4 padMat = Mat4::translate(Vec3(0, -0.1f, 0)) * Mat4::scale(Vec3(40.0f, 0.2f, 40.0f));
    r3d->drawMesh(rocketBox, padMat, 0.15f, 0.16f, 0.18f, 1.0f, 0.1f); // Concrete floor

    // Draw grid lines on the pad
    for (int i = -10; i <= 10; i++) {
        if (i == 0) continue;
        Mat4 lineX = Mat4::translate(Vec3(i * 2.0f, 0.02f, 0)) * Mat4::scale(Vec3(0.05f, 0.02f, 40.0f));
        Mat4 lineZ = Mat4::translate(Vec3(0, 0.02f, i * 2.0f)) * Mat4::scale(Vec3(40.0f, 0.02f, 0.05f));
        r3d->drawMesh(rocketBox, lineX, 0.3f, 0.3f, 0.1f, 1.0f, 0.2f); // Yellow hazard lines
        r3d->drawMesh(rocketBox, lineZ, 0.3f, 0.3f, 0.1f, 1.0f, 0.2f);
    }

    // Scaffold pillars
    Mat4 pillar1 = Mat4::translate(Vec3(-6, 15, -6)) * Mat4::scale(Vec3(1, 30, 1));
    Mat4 pillar2 = Mat4::translate(Vec3(-6, 15, 6)) * Mat4::scale(Vec3(1, 30, 1));
    r3d->drawMesh(rocketBox, pillar1, 0.2f, 0.2f, 0.2f, 1.0f, 0.1f);
    r3d->drawMesh(rocketBox, pillar2, 0.2f, 0.2f, 0.2f, 1.0f, 0.1f);

    // 这是一个 Lambda 函数：用于在 3D 空间里画出一个零件。
    // 它会考虑对称性 (Symmetry)，比如你装一个侧挂油箱，它会自动在另一侧也画一个。
    auto drawPartWithSymmetry = [&](const PartDef& def, Vec3 pos, Quat rot, bool highlight, bool selected, float alpha = 1.0f, int sym = 1, float rm = 1.0f, float gm = 1.0f, float bm = 1.0f) {
        float r = def.r * rm, g = def.g * gm, b = def.b * bm;
        if (highlight) {
            float blink = 0.5f + 0.5f * sinf((float)glfwGetTime() * 8.0f);
            r = std::min(1.0f, r + 0.4f * blink); g = std::min(1.0f, g + 0.6f * blink); b = std::min(1.0f, b + 0.3f * blink);
        }
        if (selected) {
            r = r * 0.4f; g = g * 0.6f; b = std::min(1.0f, b + 0.7f);
        }

        // --- Custom Model and Texture Loading ---
        Mesh* customMesh = nullptr;
        bool hasTexture = false;
        if (def.model_path) {
            std::string mp(def.model_path);
            if (r3d->meshCache.find(mp) == r3d->meshCache.end()) {
                r3d->meshCache[mp] = ModelLoader::loadOBJ(mp);
            }
            customMesh = &r3d->meshCache[mp];
        }
        if (def.texture_path) {
            std::string tp(def.texture_path);
            if (r3d->textureCache.find(tp) == r3d->textureCache.end()) {
                r3d->textureCache[tp] = Renderer3D::loadTGA(def.texture_path);
            }
            if (r3d->textureCache[tp].id != 0) {
                r3d->textureCache[tp].bind(0);
                hasTexture = true;
            }
        }
        glUniform1i(r3d->u_hasTexture, hasTexture ? 1 : 0);
        glUniform1i(r3d->u_sampler, 0);

        for (int s = 0; s < sym; s++) {
            float angle = (s * 2.0f * 3.14159f) / sym;
            Vec3 symPos = pos;
            if (sym > 1) {
                float dist = sqrt(pos.x * pos.x + pos.z * pos.z);
                if (dist > 0.01f) {
                    float curAngle = atan2(pos.z, pos.x);
                    symPos.x = cos(curAngle + angle) * dist;
                    symPos.z = sin(curAngle + angle) * dist;
                }
            }

            if (customMesh && customMesh->indexCount > 0) {
                // Use custom model
                Mat4 partMat = Mat4::translate(symPos) * Mat4::fromQuat(rot) * Mat4::fromQuat(Quat::fromAxisAngle(Vec3(0, 1, 0), angle));
                r3d->drawMesh(*customMesh, partMat, r, g, b, alpha, 0.2f);
            }
            else {
                // Procedural fallback
                float pd = def.diameter;
                if (def.category == CAT_NOSE_CONE) {
                    Mat4 partMat = Mat4::translate(symPos) * Mat4::fromQuat(rot) * Mat4::scale({ pd, def.height, pd });
                    r3d->drawMesh(rocketNose, partMat, r, g, b, alpha, 0.2f);
                }
                else if (def.category == CAT_ENGINE) {
                    float bf = 0.4f; float nf = 1.0f - bf;
                    Mat4 rotMat = Mat4::fromQuat(rot);
                    Mat4 bodyMat = Mat4::translate(symPos) * rotMat * Mat4::translate(Vec3(0, def.height * (1.0f - bf * 0.5f), 0)) * Mat4::scale({ pd * 0.6f, def.height * bf, pd * 0.6f });
                    r3d->drawMesh(rocketBody, bodyMat, 0.2f * rm, 0.2f * gm, 0.22f * bm, alpha, 0.4f);
                    Mat4 bellMat = Mat4::translate(symPos) * rotMat * Mat4::scale({ pd * 0.85f, def.height * nf, pd * 0.85f });
                    r3d->drawMesh(rocketNose, bellMat, r * 0.8f, g * 0.8f, b * 0.8f, alpha, 0.1f);
                }
                else if (def.category == CAT_STRUCTURAL) {
                    if (strstr(def.name, "Fin") || strstr(def.name, "Solar")) {
                        Mat4 finMat = Mat4::translate(symPos + Vec3(0, def.height * 0.5f, 0)) * Mat4::fromQuat(Quat::fromAxisAngle(Vec3(0, 1, 0), angle)) * Mat4::scale({ pd * 0.05f, def.height, pd * 0.5f });
                        r3d->drawMesh(rocketBox, finMat, r, g, b, alpha, 0.1f);
                    }
                    else if (strstr(def.name, "Leg")) {
                        Mat4 legMat = Mat4::translate(symPos + Vec3(0, def.height * 0.5f, 0)) * Mat4::fromQuat(Quat::fromAxisAngle(Vec3(0, 1, 0), angle)) * Mat4::scale({ pd * 0.15f, def.height, pd * 0.15f });
                        r3d->drawMesh(rocketBox, legMat, r, g, b, alpha, 0.1f);
                    }
                    else {
                        Mat4 partMat = Mat4::translate(symPos) * Mat4::fromQuat(rot) * Mat4::translate(Vec3(0, def.height * 0.5f, 0)) * Mat4::scale({ pd, def.height, pd });
                        r3d->drawMesh(rocketBody, partMat, r, g, b, alpha, 0.2f);
                    }
                }
                else {
                    Mat4 partMat = Mat4::translate(symPos) * Mat4::fromQuat(rot) * Mat4::translate(Vec3(0, def.height * 0.5f, 0)) * Mat4::scale({ pd, def.height, pd });
                    r3d->drawMesh(rocketBody, partMat, r, g, b, alpha, 0.2f);
                }
            }
        }
        // Cleanup texture binding
        glUniform1i(r3d->u_hasTexture, 0);
        };


    for (int i = 0; i < (int)builder_state.assembly.parts.size(); i++) {
        const PlacedPart& pp = builder_state.assembly.parts[i];
        bool is_selected = builder_state.show_part_menu && (builder_state.r_clicked_part_idx == i);
        drawPartWithSymmetry(PART_CATALOG[pp.def_id], pp.pos, pp.rot,
            (builder_state.in_assembly_mode && builder_state.assembly_cursor == i), is_selected, 1.0f, pp.symmetry);
    }

    // 处理火箭零件在建造阶段中的拖动交互逻辑，包括拖动的可视化效果和潜在连接点的显示
    if (builder_state.dragging_def_id != -1) {
        float pl = -0.98f, pw = 0.65f;
        bool over_catalog = (bk_now.mx < pl + pw);

        float rt = 1.0f, gt = 1.0f, bt = 1.0f;
        float alpha = 0.4f;
        if (!builder_state.is_placement_valid) {
            float pulse = 0.5f + 0.5f * sinf((float)glfwGetTime() * 10.0f);
            rt = 1.0f; gt = 0.1f * pulse; bt = 0.1f * pulse;
            alpha = 0.5f + 0.2f * pulse;
        }
        if (over_catalog) { rt = 1.0f; gt = 0.0f; bt = 0.0f; alpha = 0.3f; }

        drawPartWithSymmetry(PART_CATALOG[builder_state.dragging_def_id],
            builder_state.dragging_pos, builder_state.dragging_rot,
            false, false, alpha, builder_state.current_symmetry, rt, gt, bt);

        if (over_catalog) {
            renderer->addRect(pl + pw / 2.0f, 0.15f, pw, 1.40f, 0.5f, 0.0f, 0.0f, 0.3f);
            renderer->drawText(pl + 0.15f, 0.15f, "DROP TO DELETE", 0.015f, 1, 1, 1);
        }

        // Render potential snap nodes as glowy points
        for (const auto& p : builder_state.assembly.parts) {
            const auto& pdef = PART_CATALOG[p.def_id];
            for (const auto& node : pdef.snap_nodes) {
                Mat4 nodeMat = Mat4::translate(p.pos + node.pos) * Mat4::scale({ 0.3f, 0.3f, 0.3f });
                r3d->drawMesh(rocketBox, nodeMat, 0, 1, 0, 0.8f, 0); // Green glow
            }
        }
    }

    //检测火箭装配状态的变化，并在此基础上更新火箭的各个中心点（质心、升力中心、推力中心）的可视化信息。
    size_t current_hash = CenterCalculator::hashAssembly(builder_state.assembly);
    if (current_hash != builder_state.centerViz.lastAssemblyHash) {
        builder_state.centerViz.lastAssemblyHash = current_hash;

        // Recalculate all center positions
        builder_state.centerViz.comPos = CenterCalculator::calculateCenterOfMass(builder_state.assembly);
        builder_state.centerViz.colPos = CenterCalculator::calculateCenterOfLift(builder_state.assembly);
        builder_state.centerViz.cotPos = CenterCalculator::calculateCenterOfThrust(builder_state.assembly);

        // Update validity flags
        builder_state.centerViz.hasCoM = !builder_state.assembly.parts.empty();
        builder_state.centerViz.hasCoL = (builder_state.centerViz.colPos.y > 0.0f);
        builder_state.centerViz.hasCoT = builder_state.assembly.hasEngine();
    }

    // Render center point markers (if enabled)
    Mat4 rocketTransform; // Default constructor creates identity matrix

    if (builder_state.centerViz.showCoM && builder_state.centerViz.hasCoM) {
        std::cout << "Rendering CoM at y=" << builder_state.centerViz.comPos.y << std::endl;
        CenterVisualizer::renderMarker(r3d, builder_state.centerViz.comPos,
            CenterVisualizer::MARKER_COM, rocketTransform);
    }

    if (builder_state.centerViz.showCoL && builder_state.centerViz.hasCoL) {
        std::cout << "Rendering CoL at y=" << builder_state.centerViz.colPos.y << std::endl;
        CenterVisualizer::renderMarker(r3d, builder_state.centerViz.colPos,
            CenterVisualizer::MARKER_COL, rocketTransform);
    }

    if (builder_state.centerViz.showCoT && builder_state.centerViz.hasCoT) {
        std::cout << "Rendering CoT at y=" << builder_state.centerViz.cotPos.y << std::endl;
        CenterVisualizer::renderMarker(r3d, builder_state.centerViz.cotPos,
            CenterVisualizer::MARKER_COT, rocketTransform);
    }

    r3d->endFrame();
    //


    // Render builder UI OVERLAY (clear depth buffer so 2D renders on top)
    glClear(GL_DEPTH_BUFFER_BIT);
    //清空深度缓冲区是为了确保2D UI能够渲染在3D场景的顶部，而不是被3D场景中的物体遮挡。
    // 深度缓冲区用于确定3D场景中哪个物体应该在哪个物体的前面。
    renderer->beginFrame();

    // Render center point labels (2D overlay)
    if (builder_state.centerViz.showCoM && builder_state.centerViz.hasCoM) {
        CenterVisualizer::renderLabel(renderer, builder_state.centerViz.comPos,
            "CoM", viewMat * projMat);
    }

    if (builder_state.centerViz.showCoL && builder_state.centerViz.hasCoL) {
        CenterVisualizer::renderLabel(renderer, builder_state.centerViz.colPos,
            "CoL", viewMat * projMat);
    }

    if (builder_state.centerViz.showCoT && builder_state.centerViz.hasCoT) {
        CenterVisualizer::renderLabel(renderer, builder_state.centerViz.cotPos,
            "CoT", viewMat * projMat);
    }

    drawBuilderUI_KSP(renderer, builder_state, agency_state, (float)glfwGetTime());
    renderer->endFrame();

    glfwSwapBuffers(window);
    //将当前渲染的图像缓冲区显示到窗口上，并将后台缓冲区与前台缓冲区交换。
    // 这样玩家就能看到最新的渲染结果。

    std::this_thread::sleep_for(std::chrono::milliseconds(16));
    //控制帧率，程序会在每一帧渲染结束后休眠大约16毫秒。
}
        // Store models that we don't want to destroy into context or destroy them
        // FlightScene expects Renderer3D
        ctx.launch_assembly = builder_state.assembly;
        ctx.skip_builder = skip_builder;
        ctx.loaded_rocket_state = loaded_state;
        ctx.loaded_control_input = loaded_input;
        ctx.renderer3d = r3d;
        
        // Let flight scene destroy Earth and Ring meshes or initialize its own
        
        done = true;
    }

    void render() override {}
};
