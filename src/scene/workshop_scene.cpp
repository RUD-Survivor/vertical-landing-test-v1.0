#include "scene/workshop_scene.h"
#include "scene/flight_scene.h"
#include "scene/game_context.h"
#include "scene/scene_manager.h"
#include "save_system.h"
#include "render/part_renderer.h"

void WorkshopScene::onEnter() {
    auto& ctx = GameContext::getInstance();
    build_done = ctx.skip_builder;
    
    // Default rocket setup moved to MenuScene if load failed, but 
    // if skip_builder is false, we might just be starting.
}

void WorkshopScene::update(double dt) {
    if (build_done) {
        auto& ctx = GameContext::getInstance();
        ctx.launch_assembly = builder_state.assembly;
        SceneManager::getInstance().changeScene(std::make_unique<FlightScene>());
        return;
    }
    
    auto& ctx = GameContext::getInstance();
    auto window = ctx.window;
    

  while (!build_done && !glfwWindowShouldClose(window)) {
    // 处理窗口事件（如按键、鼠标移动）
    glfwPollEvents();
    if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
      glfwSetWindowShouldClose(window, true);

    // Read builder inputs
    BuilderKeyState bk_now;
    bk_now.up    = glfwGetKey(window, GLFW_KEY_UP) == GLFW_PRESS;
    bk_now.down  = glfwGetKey(window, GLFW_KEY_DOWN) == GLFW_PRESS;
    bk_now.left  = glfwGetKey(window, GLFW_KEY_LEFT) == GLFW_PRESS;
    bk_now.right = glfwGetKey(window, GLFW_KEY_RIGHT) == GLFW_PRESS;
    bk_now.enter = glfwGetKey(window, GLFW_KEY_ENTER) == GLFW_PRESS;
    bk_now.del   = glfwGetKey(window, GLFW_KEY_DELETE) == GLFW_PRESS || glfwGetKey(window, GLFW_KEY_BACKSPACE) == GLFW_PRESS;
    bk_now.tab   = glfwGetKey(window, GLFW_KEY_TAB) == GLFW_PRESS;
    bk_now.pgup  = glfwGetKey(window, GLFW_KEY_PAGE_UP) == GLFW_PRESS;
    bk_now.pgdn  = glfwGetKey(window, GLFW_KEY_PAGE_DOWN) == GLFW_PRESS;
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
        } else if (allowed_rotation) {
            float dx = (float)(mx - workshop_prev_mx) * 0.005f;
            float dy = (float)(my - workshop_prev_my) * 0.005f;
            builder_state.orbit_angle -= dx;
            builder_state.orbit_pitch = std::max(-0.5f, std::min(1.5f, builder_state.orbit_pitch + dy));
            workshop_prev_mx = mx;
            workshop_prev_my = my;
        }
    } else {
        workshop_is_dragging = false;
        allowed_rotation = false;
        if (!builder_state.show_part_menu) builder_state.orbit_angle += 0.001f;
    }
    
    // Zoom and Pan controls
    if (g_scroll_y != 0.0f) {
        if (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS || glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS) {
            builder_state.cam_dist *= powf(0.85f, g_scroll_y);
            builder_state.cam_dist = std::max(2.0f, std::min(100.0f, builder_state.cam_dist));
        } else {
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

    for (int i = 0; i < (int)builder_state.assembly.parts.size(); i++) {
        const PlacedPart& pp = builder_state.assembly.parts[i];
        bool is_selected = builder_state.show_part_menu && (builder_state.r_clicked_part_idx == i);
        
        PartRenderer::drawPartWithSymmetry(r3d, PART_CATALOG[pp.def_id], pp.pos, pp.rot, 
                                          rocketBody, rocketNose, rocketBox,
                                          1.0f, (builder_state.in_assembly_mode && builder_state.assembly_cursor == i), 
                                          is_selected, 1.0f, pp.symmetry);
    }

    // Draw Dragging Ghost
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

        PartRenderer::drawPartWithSymmetry(r3d, PART_CATALOG[builder_state.dragging_def_id], 
                                          builder_state.dragging_pos, builder_state.dragging_rot, 
                                          rocketBody, rocketNose, rocketBox,
                                          1.0f, false, false, alpha, builder_state.current_symmetry, 
                                          rt, gt, bt);
        
        if (over_catalog) {
            renderer->addRect(pl + pw/2.0f, 0.15f, pw, 1.40f, 0.5f, 0.0f, 0.0f, 0.3f);
            renderer->drawText(pl + 0.15f, 0.15f, "DROP TO DELETE", 0.015f, 1, 1, 1);
        }

        // Render potential snap nodes as glowy points
        for (const auto& p : builder_state.assembly.parts) {
            const auto& pdef = PART_CATALOG[p.def_id];
            for (const auto& node : pdef.snap_nodes) {
                Mat4 nodeMat = Mat4::translate(p.pos + node.pos) * Mat4::scale({0.3f, 0.3f, 0.3f});
                r3d->drawMesh(rocketBox, nodeMat, 0, 1, 0, 0.8f, 0); // Green glow
            }
        }
    }

    // Update center visualization state (detect assembly changes and recalculate)
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

    // Render builder UI OVERLAY (clear depth buffer so 2D renders on top)
    glClear(GL_DEPTH_BUFFER_BIT);
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

    std::this_thread::sleep_for(std::chrono::milliseconds(16));
  }

  // =========================================================
  // 4. 准备飞向太空：从组装零件构建物理配置
  // 此时我们将静态的 3D 模型转换为具有质量、推力和重心的物理实体。
  // 注意：物理引擎不需要知道 3D 模型长什么样，它只需要知道“多重”、“多快”。
  // =========================================================
  if (!skip_builder && !builder_state.assembly.parts.empty()) {
      // 重新对齐重心 (CoM)：让火箭的坐标系中心刚好在物理重心上，这样转弯才自然。
      Vec3 com = CenterCalculator::calculateCenterOfMass(builder_state.assembly);
      for (auto& p : builder_state.assembly.parts) {
          p.pos = p.pos - com;
      }
      builder_state.assembly.com = Vec3(0,0,0); // 重心归零
      builder_state.assembly.recalculate();
  }
  RocketConfig rocket_config = builder_state.assembly.buildRocketConfig();
  
  // Consume parts from agency inventory
  for (const auto& p : builder_state.assembly.parts) {
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

  RocketState rocket_state;
  ControlInput control_input;
  
  if (skip_builder) {
      // 使用加载的状态
      rocket_state = loaded_state;
      control_input = loaded_input;
      // Sync config to loaded stage
      StageManager::SyncActiveConfig(rocket_config, rocket_state.current_stage);
  } else {
      // 新游戏初始化
      rocket_state.fuel = builder_state.assembly.total_fuel;
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
      if (!builder_state.assembly.parts.empty()) {
          lowest_y = 1e10f;
          for (const auto& p : builder_state.assembly.parts) {
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

}

void WorkshopScene::render() {
}
