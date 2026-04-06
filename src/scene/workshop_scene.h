#pragma once
#include "scene/scene.h"
#include "scene/game_context.h"
#include "menu_system.h"
#include "save_system.h"
#include "simulation/rocket_builder.h"
#include "simulation/center_calculator.h"
#include "simulation/center_visualizer.h"
#include "render/part_renderer.h"
#include <thread>
#include <chrono>

extern float g_scroll_y;

class WorkshopScene : public IScene {
public:
    bool done = false;
    BuilderState builder_state;
    AgencyState agency_state;
    BuilderKeyState bk_prev = {};
    
    // Meshes for procedural fallback
    Mesh rocketBody, rocketNose, rocketBox;

    WorkshopScene() {
        rocketBody = MeshGen::cylinder(32, 1.0f, 1.0f);
        rocketNose = MeshGen::cone(32, 1.0f, 1.0f);
        rocketBox = MeshGen::box(1.0f, 1.0f, 1.0f);
    }

    void update(double dt) override {
        GameContext& ctx = GameContext::getInstance();
        GLFWwindow* window = ctx.window;
        Renderer* renderer = ctx.renderer2d;
        Renderer3D* r3d = ctx.renderer3d;
        
        // ==========================================
        // CRITICAL FIX: Instantiate Renderer3D if null
        // ==========================================
        if (!r3d) {
            r3d = new Renderer3D();
            ctx.renderer3d = r3d;
        }

        bool skip_builder = ctx.skip_builder;

        if (skip_builder) {
            done = true;
            return;
        }

        // Initialize default rocket (Nosecone + 100t Tank + Raptor) if assembly is empty
        if (builder_state.assembly.parts.empty()) {
            builder_state.assembly.addPart(9);   // Raptor engine
            builder_state.assembly.addPart(6);   // Medium tank 100t
            builder_state.assembly.addPart(0);   // Standard fairing
        }

        while (!done && !glfwWindowShouldClose(window)) {
            glfwPollEvents();
            if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
                glfwSetWindowShouldClose(window, true);

            // 1. Handle Input
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

            double bmx, bmy;
            glfwGetCursorPos(window, &bmx, &bmy);
            int bww, bwh;
            glfwGetWindowSize(window, &bww, &bwh);
            bk_now.mx = (float)(bmx / bww * 2.0 - 1.0);
            bk_now.my = (float)(1.0 - bmy / bwh * 2.0);
            bk_now.lmb = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS;
            bk_now.rmb = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS;

            done = builderHandleInput(builder_state, bk_now, bk_prev);
            bk_prev = bk_now;

            // 2. Camera Controls
            static double workshop_prev_mx = 0, workshop_prev_my = 0;
            static bool workshop_is_dragging = false;
            static bool allowed_rotation = false;

            if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS) {
                double mx, my;
                glfwGetCursorPos(window, &mx, &my);
                if (!workshop_is_dragging) {
                    workshop_prev_mx = mx; workshop_prev_my = my;
                    workshop_is_dragging = true;
                    allowed_rotation = !builder_state.show_part_menu;
                } else if (allowed_rotation) {
                    float dx = (float)(mx - workshop_prev_mx) * 0.005f;
                    float dy = (float)(my - workshop_prev_my) * 0.005f;
                    builder_state.orbit_angle -= dx;
                    builder_state.orbit_pitch = std::max(-0.5f, std::min(1.5f, builder_state.orbit_pitch + dy));
                    workshop_prev_mx = mx; workshop_prev_my = my;
                }
            } else {
                workshop_is_dragging = false; allowed_rotation = false;
                if (!builder_state.show_part_menu) builder_state.orbit_angle += 0.001f;
            }

            if (g_scroll_y != 0.0f) {
                if (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS) {
                    builder_state.cam_dist = std::max(2.0f, std::min(100.0f, builder_state.cam_dist * powf(0.85f, g_scroll_y)));
                } else {
                    builder_state.cam_y_offset += g_scroll_y * (builder_state.cam_dist * 0.05f);
                }
                g_scroll_y = 0.0f;
            }

            // 3. Render 3D
            int ww, wh;
            glfwGetFramebufferSize(window, &ww, &wh);
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

            r3d->beginFrame(viewMat, projMat, camEye);
            glClearColor(0.02f, 0.025f, 0.03f, 1.0f);
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

            // Pad & Environment
            r3d->drawMesh(rocketBox, Mat4::translate(Vec3(0, -0.1f, 0)) * Mat4::scale(Vec3(40, 0.2f, 40)), 0.15f, 0.16f, 0.18f, 1.0f, 0.1f);
            for (int i = -10; i <= 10; i++) {
                if (i == 0) continue;
                r3d->drawMesh(rocketBox, Mat4::translate(Vec3(i * 2.0f, 0.02f, 0)) * Mat4::scale(Vec3(0.05f, 0.02f, 40.0f)), 0.3f, 0.3f, 0.1f, 1.0f, 0.2f);
                r3d->drawMesh(rocketBox, Mat4::translate(Vec3(0, 0.02f, i * 2.0f)) * Mat4::scale(Vec3(40.0f, 0.02f, 0.05f)), 0.3f, 0.3f, 0.1f, 1.0f, 0.2f);
            }

            // Assembly Parts
            for (int i = 0; i < (int)builder_state.assembly.parts.size(); i++) {
                const PlacedPart& pp = builder_state.assembly.parts[i];
                bool is_selected = builder_state.show_part_menu && (builder_state.r_clicked_part_idx == i);
                PartRenderer::drawPartWithSymmetry(r3d, PART_CATALOG[pp.def_id], pp.pos, pp.rot, 
                                                  rocketBody, rocketNose, rocketBox,
                                                  1.0f, (builder_state.in_assembly_mode && builder_state.assembly_cursor == i), 
                                                  is_selected, 1.0f, pp.symmetry);
            }

            // Ghost Part
            if (builder_state.dragging_def_id != -1) {
                float alpha = builder_state.is_placement_valid ? 0.4f : 0.6f;
                PartRenderer::drawPartWithSymmetry(r3d, PART_CATALOG[builder_state.dragging_def_id], 
                                                  builder_state.dragging_pos, builder_state.dragging_rot, 
                                                  rocketBody, rocketNose, rocketBox,
                                                  1.0f, false, false, alpha, builder_state.current_symmetry);
            }

            // Center Visualization
            size_t current_hash = CenterCalculator::hashAssembly(builder_state.assembly);
            if (current_hash != builder_state.centerViz.lastAssemblyHash) {
                builder_state.centerViz.lastAssemblyHash = current_hash;
                builder_state.centerViz.comPos = CenterCalculator::calculateCenterOfMass(builder_state.assembly);
                builder_state.centerViz.colPos = CenterCalculator::calculateCenterOfLift(builder_state.assembly);
                builder_state.centerViz.cotPos = CenterCalculator::calculateCenterOfThrust(builder_state.assembly);
                builder_state.centerViz.hasCoM = !builder_state.assembly.parts.empty();
                builder_state.centerViz.hasCoL = (builder_state.centerViz.colPos.y > 0.0f);
                builder_state.centerViz.hasCoT = builder_state.assembly.hasEngine();
            }
            
            Mat4 rocketTransform;
            if (builder_state.centerViz.showCoM && builder_state.centerViz.hasCoM) CenterVisualizer::renderMarker(r3d, builder_state.centerViz.comPos, CenterVisualizer::MARKER_COM, rocketTransform);
            if (builder_state.centerViz.showCoL && builder_state.centerViz.hasCoL) CenterVisualizer::renderMarker(r3d, builder_state.centerViz.colPos, CenterVisualizer::MARKER_COL, rocketTransform);
            if (builder_state.centerViz.showCoT && builder_state.centerViz.hasCoT) CenterVisualizer::renderMarker(r3d, builder_state.centerViz.cotPos, CenterVisualizer::MARKER_COT, rocketTransform);

            r3d->endFrame();
            
            // UI Overlay
            glClear(GL_DEPTH_BUFFER_BIT);
            renderer->beginFrame();
            if (builder_state.centerViz.showCoM && builder_state.centerViz.hasCoM) CenterVisualizer::renderLabel(renderer, builder_state.centerViz.comPos, "CoM", viewMat * projMat);
            if (builder_state.centerViz.showCoL && builder_state.centerViz.hasCoL) CenterVisualizer::renderLabel(renderer, builder_state.centerViz.colPos, "CoL", viewMat * projMat);
            if (builder_state.centerViz.showCoT && builder_state.centerViz.hasCoT) CenterVisualizer::renderLabel(renderer, builder_state.centerViz.cotPos, "CoT", viewMat * projMat);
            drawBuilderUI_KSP(renderer, builder_state, agency_state, (float)glfwGetTime());
            renderer->endFrame();

            glfwSwapBuffers(window);
            std::this_thread::sleep_for(std::chrono::milliseconds(16));
        }
        
        // Prep for Flight
        if (!skip_builder && !builder_state.assembly.parts.empty()) {
            ctx.launch_assembly = builder_state.assembly;
        }
    }

    void render() override {}
};
