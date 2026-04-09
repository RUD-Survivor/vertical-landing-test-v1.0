#pragma once
#include <GLFW/glfw3.h>
#include <iostream>
#include "input/input_router.h"
#include "core/rocket_state.h"
#include "simulation/stage_manager.h"
#include "camera/camera_director.h"
#include "render/HUD_system.h"
#include "simulation/simulation_controller.h"

struct FlightInputSystem {
    // --- SVO State ---
    int svo_dig_mode = 0;
    bool svo_auto_activated = false;
    int svo_op_cooldown = 0;

    void setup(InputRouter& router, RocketConfig& rocket_config, 
               ControlInput& control_input, FlightHUD& hud, CameraDirector& cam, bool& show_clouds,
               entt::registry& world, entt::entity rocket_entity) {
        
        router.on_ignition = [&world, rocket_entity]() {
            auto& guid = world.get<GuidanceComponent>(rocket_entity);
            if (guid.status == PRE_LAUNCH) {
                guid.status = ASCEND;
                guid.mission_msg = "T-0: IGNITION! LIFTOFF!";
            }
        };

        router.registerKey(GLFW_KEY_G, [&world, rocket_entity]() {
            auto& guid = world.get<GuidanceComponent>(rocket_entity);
            if (guid.status == ASCEND || guid.status == DESCEND) {
                StageManager::SeparateStage(world, rocket_entity);
            }
        });

        router.registerKey(GLFW_KEY_N, [&world, rocket_entity, &hud, &control_input]() {
            auto& guid = world.get<GuidanceComponent>(rocket_entity);
            auto& mnv = world.get<ManeuverComponent>(rocket_entity);
            if (!mnv.maneuvers.empty()) {
                hud.auto_exec_mnv = !hud.auto_exec_mnv;
                guid.mission_msg = hud.auto_exec_mnv ? "MNV AUTO-EXEC: ARMED" : "MNV AUTO-EXEC: OFF";
                if (!hud.auto_exec_mnv) control_input.throttle = 0;
            }
        });

        router.registerKey(GLFW_KEY_C, [&cam]() { cam.cycleMode(); });
        router.registerKey(GLFW_KEY_H, [&hud]() { hud.show_hud = !hud.show_hud; });
        router.registerKey(GLFW_KEY_O, [&hud]() { hud.show_orbit = !hud.show_orbit; });

        router.registerKey(GLFW_KEY_T, [&world, rocket_entity]() {
            auto& guid = world.get<GuidanceComponent>(rocket_entity);
            guid.sas_active = !guid.sas_active;
            std::cout << "[SAS] " << (guid.sas_active ? "ON" : "OFF") << std::endl;
        });

        router.registerKey(GLFW_KEY_R, [&world, rocket_entity]() {
            auto& guid = world.get<GuidanceComponent>(rocket_entity);
            guid.rcs_active = !guid.rcs_active;
            std::cout << "[RCS] " << (guid.rcs_active ? "ON" : "OFF") << std::endl;
        });

        router.registerKey(GLFW_KEY_K, [&hud]() {
            hud.orbit_reference_sun = !hud.orbit_reference_sun;
            std::cout << "[REF FRAME] " << (hud.orbit_reference_sun ? "SUN" : "EARTH") << std::endl;
        });

        router.registerKey(GLFW_KEY_TAB, [&world, rocket_entity]() {
            auto& guid = world.get<GuidanceComponent>(rocket_entity);
            guid.auto_mode = !guid.auto_mode;
            if (guid.auto_mode) {
                guid.pid_vert.reset(); guid.pid_pos.reset(); 
                guid.pid_att.reset(); guid.pid_att_z.reset();
                guid.mission_msg = ">> AUTOPILOT ENGAGED";
            } else {
                guid.mission_msg = ">> MANUAL CONTROL ACTIVE";
            }
        });

        router.registerKey(GLFW_KEY_B, [this]() {
            Renderer3D* r3d = GameContext::getInstance().renderer3d;
            if (r3d->svoManager && r3d->svoManager->hasActiveChunks()) {
                svo_dig_mode = (svo_dig_mode + 1) % 3;
                const char* modeNames[] = { "OFF", "DIG", "BUILD" };
                std::cout << "[SVO] Mode: " << modeNames[svo_dig_mode] << std::endl;
            }
        });

        router.registerShiftKey(GLFW_KEY_P, [&show_clouds]() {
            show_clouds = !show_clouds;
            std::cout << "[CLOUDS] " << (show_clouds ? "ON" : "OFF") << std::endl;
        });

        router.registerShiftKey(GLFW_KEY_V, [this, &world, rocket_entity]() {
            Renderer3D* r3d = GameContext::getInstance().renderer3d;
            auto& trans = world.get<TransformComponent>(rocket_entity);
            if (r3d->svoManager) {
                if (r3d->svoManager->hasActiveChunks()) {
                    r3d->svoManager->deactivate(); 
                    svo_auto_activated = false; svo_dig_mode = 0;
                    std::cout << "[SVO] System Deactivated" << std::endl;
                } else {
                    Vec3 subNormal(trans.surf_px, trans.surf_py, trans.surf_pz);
                    subNormal = subNormal.normalized();
                    Quat unalign = Quat::fromAxisAngle(Vec3(1,0,0), (float)(3.14159/2.0));
                    r3d->svoManager->activate(unalign.rotate(subNormal), 6371.0 * 0.001, r3d->terrain);
                    std::cout << "[SVO] System Activated" << std::endl;
                    svo_auto_activated = true;
                }
            }
        });
    }

    void poll(GLFWwindow* window, InputRouter& router, entt::registry& world, entt::entity rocket_entity, Renderer3D* r3d) {
        auto& guid = world.get<GuidanceComponent>(rocket_entity);
        auto& tele = world.get<TelemetryComponent>(rocket_entity);
        auto& trans = world.get<TransformComponent>(rocket_entity);
        router.poll(window);

        // --- SVO Automation ---
        if (r3d->svoManager) {
            // Auto-activate on landing
            if (!r3d->svoManager->hasActiveChunks() && !svo_auto_activated) {
                if (guid.status == LANDED && tele.altitude < 1000.0) {
                    Vec3 subNormal = Vec3((float)trans.surf_px, (float)trans.surf_py, (float)trans.surf_pz).normalized();
                    Quat unalign = Quat::fromAxisAngle(Vec3(1,0,0), (float)(3.14159/2.0));
                    r3d->svoManager->activate(unalign.rotate(subNormal), 6371.0 * 0.001, r3d->terrain);
                    svo_auto_activated = true;
                    std::cout << "[SVO] Auto-activated on landing" << std::endl;
                }
            }
            // Auto-deactivate on ascent
            if (r3d->svoManager->hasActiveChunks() && guid.status == ASCEND && tele.altitude > 2000.0) {
                r3d->svoManager->deactivate();
                svo_auto_activated = false; svo_dig_mode = 0;
                std::cout << "[SVO] Auto-deactivated on ascent" << std::endl;
            }

            // --- SVO Execute Dig/Build ---
            if (r3d->svoManager->hasActiveChunks() && svo_dig_mode > 0) {
                if (svo_op_cooldown > 0) svo_op_cooldown--;
                if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS && svo_op_cooldown == 0) {
                    Vec3 rocketWorldPos((float)trans.px, (float)trans.py, (float)trans.pz);
                    Vec3 svoLocal = r3d->svoManager->planetLocalToSVOLocal(rocketWorldPos);
                    if (svo_dig_mode == 1) r3d->svoManager->dig(svoLocal, 0.005);
                    else if (svo_dig_mode == 2) r3d->svoManager->build(svoLocal, 0.005, SVO::Material::ROCK);
                    svo_op_cooldown = 5;
                }
            }
        }
    }
};
