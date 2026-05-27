#pragma once
// ==========================================================
// HUD_system.h — Vulkan stub (HUD rendered by ImGui/VkHUD)
// Data members retained for API compatibility
// ==========================================================
#include <vector>
#include <cmath>
#include <algorithm>
#include "core/rocket_state.h"
#include "math/math3d.h"
#include "simulation/transfer_calculator.h"  // PorkchopResult

class FlightHUD {
public:
    // Data members (retained for compatibility)
    bool show_galaxy_info = false;
    int selected_body_idx = -1;
    int expanded_planet_idx = -1;
    bool hlmb_prev_galaxy = false;
    bool mnv_popup_visible = false;
    int mnv_popup_index = -1;
    float mnv_popup_px = 0, mnv_popup_py = 0, mnv_popup_pw = 0, mnv_popup_ph = 0;
    float mnv_popup_node_scr_x = 0, mnv_popup_node_scr_y = 0;
    Vec3 mnv_popup_dv = Vec3(0,0,0);
    bool mnv_popup_close_hover = false, mnv_popup_del_hover = false;
    double mnv_popup_time_to_node = 0;
    double mnv_popup_burn_time = 0;
    int mnv_popup_ref_body = -1;
    int mnv_popup_slider_dragging = -1;
    float mnv_popup_slider_drag_x = 0;
    int mnv_popup_burn_mode = 0;
    bool mnv_popup_mode_hover = false;
    Vec3 adv_mnv_world_pos = Vec3(0,0,0);
    bool adv_orbit_enabled = false;
    bool adv_orbit_menu = false;
    float adv_orbit_pred_days = 30.0f;
    int adv_orbit_iters = 4000;
    int adv_orbit_ref_mode = 0;
    int adv_orbit_ref_body = 3;
    int adv_orbit_secondary_ref_body = 4;
    bool adv_warp_to_node = false;
    bool auto_exec_mnv = false;
    bool flight_assist_menu = false;
    bool adv_embed_mnv = false;
    bool adv_embed_mnv_mini = false;
    bool mnv_popup_mini_hover = false;
    bool show_hud = true;
    bool show_orbit = true;
    bool orbit_reference_sun = false;
    int global_best_ref_node = -1;
    bool transfer_window_menu = false;
    int transfer_target_body = 4;
    PorkchopResult transfer_result;
    bool transfer_result_valid = false;
    int transfer_hover_dep = -1;
    int transfer_hover_tof = -1;

    // No-op render methods (Vulkan: handled by ImGui/VkHUD)
    template<typename... Args> void render(Args&&...) {}
};
