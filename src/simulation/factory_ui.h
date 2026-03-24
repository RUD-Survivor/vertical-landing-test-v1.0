#pragma once
// ==========================================================
// factory_ui.h — 工厂建造界面 (Phase 4)
// 2D top-down grid view for placing buildings & conveyor belts
// ==========================================================

#include "simulation/factory_system.h"
#include "simulation/resource_system.h"
#include "core/agency_state.h"
#include <cmath>
#include <vector>

// ==========================================
// FactoryUI State
// ==========================================
struct FactoryUIState {
    // Camera / grid view
    float cam_x = 0.0f, cam_y = 0.0f; // center of view in grid coords
    float zoom = 1.0f;                  // zoom level

    // Grid
    static const int GRID_W = 30;
    static const int GRID_H = 20;
    float cell_size = 0.12f;           // screen size of one cell at zoom=1

    // Placement mode
    bool placing = false;
    FactoryNodeType place_type = NODE_MINER;
    int hover_gx = -1, hover_gy = -1;

    // Belt drawing mode
    bool belt_mode = false;
    int belt_from_id = -1;             // source node id for belt
    int belt_hover_id = -1;            // currently hovering over node

    // Selection
    int selected_node_id = -1;

    // Toolbar
    int toolbar_hover = -1;            // which toolbar button is hovered

    // Delete mode
    bool delete_mode = false;

    // Last mouse position (for hover in render)
    float last_mx = 0.0f, last_my = 0.0f;
};

// ==========================================
// Convert grid -> screen coordinates
// ==========================================
inline void gridToScreen(const FactoryUIState& ui, int gx, int gy, float& sx, float& sy) {
    float cs = ui.cell_size * ui.zoom;
    float origin_x = -((float)ui.GRID_W / 2.0f) * cs + ui.cam_x;
    float origin_y = -((float)ui.GRID_H / 2.0f) * cs + ui.cam_y;
    sx = origin_x + (gx + 0.5f) * cs;
    sy = origin_y + (gy + 0.5f) * cs;
}

inline void screenToGrid(const FactoryUIState& ui, float mx, float my, int& gx, int& gy) {
    float cs = ui.cell_size * ui.zoom;
    float origin_x = -((float)ui.GRID_W / 2.0f) * cs + ui.cam_x;
    float origin_y = -((float)ui.GRID_H / 2.0f) * cs + ui.cam_y;
    gx = (int)floorf((mx - origin_x) / cs);
    gy = (int)floorf((my - origin_y) / cs);
}

// Check if grid coords are in-bounds
inline bool gridValid(int gx, int gy) {
    return gx >= 0 && gx < FactoryUIState::GRID_W && gy >= 0 && gy < FactoryUIState::GRID_H;
}

// ==========================================
// Find node at grid position
// ==========================================
inline int findNodeAtGrid(const FactorySystem& factory, int gx, int gy) {
    for (auto& n : factory.nodes) {
        if (n.grid_x == gx && n.grid_y == gy) return n.id;
    }
    return -1;
}

// Node display colors
inline void getNodeColor(FactoryNodeType t, float& r, float& g, float& b) {
    switch (t) {
        case NODE_MINER:     r=0.6f; g=0.4f; b=0.2f; break;
        case NODE_SMELTER:   r=0.8f; g=0.5f; b=0.1f; break;
        case NODE_ASSEMBLER: r=0.2f; g=0.7f; b=0.3f; break;
        case NODE_STORAGE:   r=0.4f; g=0.4f; b=0.6f; break;
        case NODE_POWER_PLANT: r=0.9f; g=0.9f; b=0.2f; break;
        default:             r=0.5f; g=0.5f; b=0.5f; break;
    }
}

// ==========================================
// Draw the factory construction UI
// ==========================================
inline void DrawFactoryUI(Renderer* renderer, FactoryUIState& ui, const FactorySystem& factory, const AgencyState& agency, float time) {
    // Background
    renderer->addRect(0.0f, 0.0f, 2.0f, 2.0f, 0.06f, 0.07f, 0.09f, 1.0f);

    float cs = ui.cell_size * ui.zoom;
    float origin_x = -((float)ui.GRID_W / 2.0f) * cs + ui.cam_x;
    float origin_y = -((float)ui.GRID_H / 2.0f) * cs + ui.cam_y;

    // Draw grid lines
    for (int x = 0; x <= ui.GRID_W; x++) {
        float sx = origin_x + x * cs;
        renderer->addRect(sx, ui.cam_y, 0.002f, ui.GRID_H * cs, 0.15f, 0.18f, 0.22f, 0.5f);
    }
    for (int y = 0; y <= ui.GRID_H; y++) {
        float sy = origin_y + y * cs;
        renderer->addRect(ui.cam_x, sy, ui.GRID_W * cs, 0.002f, 0.15f, 0.18f, 0.22f, 0.5f);
    }

    // Draw conveyor belts from factory system
    for (auto& belt : factory.belts) {
        const FactoryNode* from = nullptr;
        const FactoryNode* to = nullptr;
        for (auto& n : factory.nodes) {
            if (n.id == belt.from_node_id) from = &n;
            if (n.id == belt.to_node_id) to = &n;
        }
        if (from && to) {
            float fx, fy, tx, ty;
            gridToScreen(ui, from->grid_x, from->grid_y, fx, fy);
            gridToScreen(ui, to->grid_x, to->grid_y, tx, ty);
            
            // Draw belt path (poly-line style)
            float mr = 0.3f, mg = 0.5f, mb = 0.4f;
            float pulse = 0.8f + 0.2f * sinf(time * 5.0f);
            
            float dx = tx - fx, dy = ty - fy;
            if (fabsf(dx) > 0.001f) {
                renderer->addRect((fx + tx) / 2.0f, fy, fabsf(dx) + 0.01f, 0.015f, mr * pulse, mg * pulse, mb * pulse, 0.8f);
            }
            if (fabsf(dy) > 0.001f) {
                renderer->addRect(tx, (fy + ty) / 2.0f, 0.015f, fabsf(dy) + 0.01f, mr * pulse, mg * pulse, mb * pulse, 0.8f);
            }
            // Render items on belt
            for (auto const& item : belt.items) {
                float ix, iy;
                // Calculate position based on progress (0.0 to 1.0)
                // Follow the L-shape path: first horizontal, then vertical
                if (item.progress < 0.5f) {
                    float t = item.progress * 2.0f;
                    ix = fx + (tx - fx) * t;
                    iy = fy;
                } else {
                    float t = (item.progress - 0.5f) * 2.0f;
                    ix = tx;
                    iy = fy + (ty - fy) * t;
                }

                const ItemInfo& info = GetItemInfo(item.type);
                float isize = cs * 0.18f;
                // Draw item icon (colored square)
                renderer->addRect(ix, iy, isize, isize, info.icon_r, info.icon_g, info.icon_b, 1.0f);
                // Glow effect
                renderer->addRect(ix, iy, isize * 1.2f, isize * 1.2f, info.icon_r, info.icon_g, info.icon_b, 0.3f);
            }
        }
    }

    // Draw belt being drawn
    if (ui.belt_mode && ui.belt_from_id >= 0) {
        const FactoryNode* from = nullptr;
        for (auto& n : factory.nodes) {
            if (n.id == ui.belt_from_id) { from = &n; break; }
        }
        if (from && ui.hover_gx >= 0) {
            float fx, fy, hx, hy;
            gridToScreen(ui, from->grid_x, from->grid_y, fx, fy);
            gridToScreen(ui, ui.hover_gx, ui.hover_gy, hx, hy);
            // Dashed preview line
            float pulse = 0.5f + 0.5f * sinf(time * 8.0f);
            if (fabsf(hx - fx) > 0.001f) {
                renderer->addRect((fx+hx)/2, fy, fabsf(hx-fx), 0.008f, 0.3f*pulse, 0.9f*pulse, 0.4f*pulse, 0.6f);
            }
            if (fabsf(hy - fy) > 0.001f) {
                renderer->addRect(hx, (fy+hy)/2, 0.008f, fabsf(hy-fy), 0.3f*pulse, 0.9f*pulse, 0.4f*pulse, 0.6f);
            }
        }
    }

    // Draw buildings on grid
    for (auto& node : factory.nodes) {
        float sx, sy;
        gridToScreen(ui, node.grid_x, node.grid_y, sx, sy);
        float nr, ng, nb;
        getNodeColor(node.type, nr, ng, nb);

        bool is_selected = (node.id == ui.selected_node_id);
        float bsize = cs * 0.42f;

        // Building body
        renderer->addRect(sx, sy, bsize * 2.0f, bsize * 2.0f, nr * 0.8f, ng * 0.8f, nb * 0.8f, 0.9f);

        // Selection highlight
        if (is_selected) {
            float pulse = 0.6f + 0.4f * sinf(time * 5.0f);
            // top border
            renderer->addRect(sx, sy + bsize, bsize * 2.0f, 0.005f, pulse, pulse, 1.0f, 1.0f);
            // bottom border
            renderer->addRect(sx, sy - bsize, bsize * 2.0f, 0.005f, pulse, pulse, 1.0f, 1.0f);
            // left border
            renderer->addRect(sx - bsize, sy, 0.005f, bsize * 2.0f, pulse, pulse, 1.0f, 1.0f);
            // right border
            renderer->addRect(sx + bsize, sy, 0.005f, bsize * 2.0f, pulse, pulse, 1.0f, 1.0f);
        }

        // Progress bar inside building
        if (node.progress > 0.0f && node.type != NODE_STORAGE) {
            float bar_w = bsize * 1.6f;
            float bar_h = bsize * 0.3f;
            renderer->addRect(sx, sy - bsize * 0.5f, bar_w, bar_h, 0.1f, 0.1f, 0.1f, 0.8f);
            float fill_w = bar_w * node.progress;
            renderer->addRect(sx - bar_w/2 + fill_w/2, sy - bsize * 0.5f, fill_w, bar_h * 0.7f,
                             0.2f, 0.9f, 0.3f, 0.9f);
        }

        // Label
        renderer->drawText(sx, sy + bsize * 0.3f, NodeTypeName(node.type), 0.008f, 1.0f, 1.0f, 1.0f, 1.0f, true, Renderer::CENTER);
    }

    // Ghost preview when placing
    if (ui.placing && gridValid(ui.hover_gx, ui.hover_gy)) {
        int existing = findNodeAtGrid(factory, ui.hover_gx, ui.hover_gy);
        float sx, sy;
        gridToScreen(ui, ui.hover_gx, ui.hover_gy, sx, sy);
        float nr, ng, nb;
        getNodeColor(ui.place_type, nr, ng, nb);
        float bsize = cs * 0.42f;
        float alpha = (existing >= 0) ? 0.3f : 0.6f;
        float pulse = 0.7f + 0.3f * sinf(time * 6.0f);
        renderer->addRect(sx, sy, bsize * 2.0f, bsize * 2.0f, nr * pulse, ng * pulse, nb * pulse, alpha);
        renderer->drawText(sx, sy, NodeTypeName(ui.place_type), 0.008f, 1.0f, 1.0f, 1.0f, alpha, true, Renderer::CENTER);
    }

    // ===== TOP TOOLBAR =====
    float tb_y = 0.9f;
    renderer->addRect(0.0f, tb_y, 2.0f, 0.18f, 0.03f, 0.04f, 0.06f, 0.95f);
    renderer->addRect(0.0f, 0.81f, 2.0f, 0.008f, 0.2f, 0.5f, 0.8f, 0.6f);

    renderer->drawText(-0.85f, tb_y + 0.02f, "FACTORY BUILDER", 0.025f, 0.9f, 0.7f, 0.3f, 1.0f, true, Renderer::LEFT);

    // Building buttons
    struct ToolBtn {
        const char* label;
        FactoryNodeType type;
        float r, g, b;
    };
    ToolBtn btns[] = {
        {"MINER",     NODE_MINER,     0.6f, 0.4f, 0.2f},
        {"SMELTER",   NODE_SMELTER,   0.8f, 0.5f, 0.1f},
        {"ASSEMBLER", NODE_ASSEMBLER, 0.2f, 0.7f, 0.3f},
        {"STORAGE",   NODE_STORAGE,   0.4f, 0.4f, 0.6f},
        {"POWER",     NODE_POWER_PLANT, 0.9f, 0.9f, 0.2f},
    };
    int num_btns = 5;
    float btn_x_start = -0.85f;
    float btn_spacing = 0.26f;
    float btn_w = 0.22f;
    float btn_h = 0.08f;

    for (int i = 0; i < num_btns; i++) {
        float bx = btn_x_start + i * btn_spacing;
        bool hov = (ui.toolbar_hover == i);
        bool active = (ui.placing && ui.place_type == btns[i].type);

        if (active) {
            float p = 0.7f + 0.3f * sinf(time * 5.0f);
            renderer->addRect(bx, tb_y, btn_w, btn_h, btns[i].r * p, btns[i].g * p, btns[i].b * p, 0.9f);
        } else if (hov) {
            renderer->addRect(bx, tb_y, btn_w, btn_h, btns[i].r * 0.6f, btns[i].g * 0.6f, btns[i].b * 0.6f, 0.8f);
        } else {
            renderer->addRect(bx, tb_y, btn_w, btn_h, 0.12f, 0.12f, 0.15f, 0.8f);
        }

        renderer->drawText(bx, tb_y, btns[i].label, 0.012f,
                          active ? 1.0f : 0.7f, active ? 1.0f : 0.7f, active ? 1.0f : 0.7f, 1.0f,
                          true, Renderer::CENTER);
    }
    
    // Draw Recipe Overlay on all nodes
    for (const auto& node : factory.nodes) {
        float nx, ny;
        gridToScreen(ui, node.grid_x, node.grid_y, nx, ny);
        if (nx > -1.0f && nx < 1.0f && ny > -1.0f && ny < 1.0f) {
            if (node.type == NODE_SMELTER || node.type == NODE_ASSEMBLER) {
                int count = 0;
                const Recipe* recipes = GetRecipes(count);
                if (node.recipe_index >= 0 && node.recipe_index < count) {
                    const char* label = recipes[node.recipe_index].name;
                    renderer->drawText(nx, ny + 0.08f * ui.zoom, label, 0.010f * ui.zoom, 1.0f, 0.9f, 0.5f, 0.8f, true, Renderer::CENTER);
                }
            } else if (node.type == NODE_MINER) {
                renderer->drawText(nx, ny + 0.08f * ui.zoom, GetItemInfo(node.mine_output).name, 0.010f * ui.zoom, 0.7f, 0.9f, 1.0f, 0.8f, true, Renderer::CENTER);
            }
        }
    }

    // Belt button
    float belt_bx = btn_x_start + num_btns * btn_spacing;
    bool belt_active = ui.belt_mode;
    bool belt_hov = (ui.toolbar_hover == num_btns);
    if (belt_active) {
        float p = 0.7f + 0.3f * sinf(time * 5.0f);
        renderer->addRect(belt_bx, tb_y, btn_w, btn_h, 0.3f*p, 0.8f*p, 0.4f*p, 0.9f);
    } else if (belt_hov) {
        renderer->addRect(belt_bx, tb_y, btn_w, btn_h, 0.2f, 0.5f, 0.3f, 0.8f);
    } else {
        renderer->addRect(belt_bx, tb_y, btn_w, btn_h, 0.12f, 0.12f, 0.15f, 0.8f);
    }
    renderer->drawText(belt_bx, tb_y, "BELT", 0.012f, belt_active ? 0.4f : 0.6f, belt_active ? 1.0f : 0.6f, belt_active ? 0.5f : 0.6f, 1.0f, true, Renderer::CENTER);

    // Delete button
    float del_bx = belt_bx + btn_spacing;
    bool del_active = ui.delete_mode;
    bool del_hov = (ui.toolbar_hover == num_btns + 1);
    if (del_active) {
        float p = 0.7f + 0.3f * sinf(time * 5.0f);
        renderer->addRect(del_bx, tb_y, btn_w, btn_h, 0.9f*p, 0.2f*p, 0.1f*p, 0.9f);
    } else if (del_hov) {
        renderer->addRect(del_bx, tb_y, btn_w, btn_h, 0.6f, 0.15f, 0.1f, 0.8f);
    } else {
        renderer->addRect(del_bx, tb_y, btn_w, btn_h, 0.12f, 0.12f, 0.15f, 0.8f);
    }
    renderer->drawText(del_bx, tb_y, "DELETE", 0.012f, del_active ? 1.0f : 0.6f, del_active ? 0.3f : 0.5f, del_active ? 0.3f : 0.5f, 1.0f, true, Renderer::CENTER);

    // ===== SIDE PANEL (SELECTED NODE INFO) =====
    if (ui.selected_node_id >= 0) {
        const FactoryNode* sel_node = nullptr;
        for (auto& n : factory.nodes) { if (n.id == ui.selected_node_id) sel_node = &n; }
        
        if (sel_node) {
            float sp_x = 0.75f;
            float sp_y = 0.0f;
            float sp_w = 0.45f;
            float sp_h = 1.2f;
            
            // Glassmorphism background
            renderer->addRect(sp_x, sp_y, sp_w, sp_h, 0.05f, 0.06f, 0.1f, 0.9f);
            renderer->addRect(sp_x - sp_w / 2.0f, sp_y, 0.005f, sp_h, 0.2f, 0.5f, 0.8f, 0.7f); // border
            
            renderer->drawText(sp_x, 0.5f, NodeTypeName(sel_node->type), 0.025f, 1.0f, 1.0f, 1.0f, 1.0f, true, Renderer::CENTER);
            renderer->drawText(sp_x, 0.43f, "STATUS: ACTIVE", 0.012f, 0.4f, 1.0f, 0.5f, 1.0f, true, Renderer::CENTER);
            
            float py = 0.3f;
            char buf[128];
            snprintf(buf, sizeof(buf), "PROGRESS: %.0f%%", sel_node->progress * 100.0f);
            renderer->drawText(sp_x - 0.18f, py, buf, 0.012f, 0.8f, 0.8f, 0.8f, 1.0f, true, Renderer::LEFT);
            
            // Recipe info
            if (sel_node->type == NODE_SMELTER || sel_node->type == NODE_ASSEMBLER) {
                py -= 0.1f;
                renderer->drawText(sp_x, py, "TARGET RECIPE:", 0.012f, 0.6f, 0.7f, 1.0f, 1.0f, true, Renderer::CENTER);
                py -= 0.05f;
                int count = 0;
                const Recipe* recipes = GetRecipes(count);
                if (sel_node->recipe_index >= 0 && sel_node->recipe_index < count) {
                    renderer->drawText(sp_x, py, recipes[sel_node->recipe_index].name, 0.018f, 1.0f, 0.8f, 0.3f, 1.0f, true, Renderer::CENTER);
                }
                
                // Recipe Switch Buttons
                py -= 0.08f;
                bool phov = (ui.last_mx >= sp_x - 0.12f - 0.15f/2 && ui.last_mx <= sp_x - 0.12f + 0.15f/2 && ui.last_my >= py - 0.06f/2 && ui.last_my <= py + 0.06f/2);
                renderer->addRect(sp_x - 0.12f, py, 0.15f, 0.06f, phov ? 0.3f : 0.2f, phov ? 0.4f : 0.3f, phov ? 0.6f : 0.5f, 0.8f);
                renderer->drawText(sp_x - 0.12f, py, "PREV", 0.010f, 1.0f, 1.0f, 1.0f, 1.0f, true, Renderer::CENTER);
                
                bool nhov = (ui.last_mx >= sp_x + 0.12f - 0.15f/2 && ui.last_mx <= sp_x + 0.12f + 0.15f/2 && ui.last_my >= py - 0.06f/2 && ui.last_my <= py + 0.06f/2);
                renderer->addRect(sp_x + 0.12f, py, 0.15f, 0.06f, nhov ? 0.3f : 0.2f, nhov ? 0.4f : 0.3f, nhov ? 0.6f : 0.5f, 0.8f);
                renderer->drawText(sp_x + 0.12f, py, "NEXT", 0.010f, 1.0f, 1.0f, 1.0f, 1.0f, true, Renderer::CENTER);
            }
            
            // Miner output info
            if (sel_node->type == NODE_MINER) {
                py -= 0.1f;
                renderer->drawText(sp_x, py, "MINING RESOURCE:", 0.012f, 0.6f, 0.7f, 1.0f, 1.0f, true, Renderer::CENTER);
                py -= 0.05f;
                renderer->drawText(sp_x, py, GetItemInfo(sel_node->mine_output).name, 0.018f, 0.9f, 0.6f, 0.4f, 1.0f, true, Renderer::CENTER);
            }
            
            // Stats
            py -= 0.2f;
            renderer->drawText(sp_x, py, "INTERNAL BUFFER:", 0.012f, 0.7f, 0.7f, 0.7f, 1.0f, true, Renderer::CENTER);
            for (auto const& pair : sel_node->input_buffer.items) {
                ItemType type = pair.first;
                int qty = pair.second;
                py -= 0.05f;
                snprintf(buf, sizeof(buf), "%s: %d", GetItemInfo(type).name, qty);
                renderer->drawText(sp_x, py, buf, 0.010f, 1.0f, 1.0f, 1.0f, 1.0f, true, Renderer::CENTER);
            }
        }
    }

    // ===== BOTTOM STATUS BAR =====
    float sb_y = -0.9f;
    renderer->addRect(0.0f, sb_y, 2.0f, 0.16f, 0.03f, 0.04f, 0.06f, 0.95f);
    renderer->addRect(0.0f, -0.82f, 2.0f, 0.008f, 0.2f, 0.5f, 0.8f, 0.6f);

    char info[256];
    snprintf(info, sizeof(info), "BUILDINGS: %d | BELTS: %d | POWER: %.0f/%.0f (%.0f%%) | FUNDS: $%.0f",
             (int)factory.nodes.size(), (int)factory.belts.size(), 
             factory.total_power_gen, factory.total_power_req, factory.power_efficiency * 100.0f,
             agency.funds);
    renderer->drawText(-0.95f, sb_y, info, 0.015f, 0.7f, 0.7f, 0.8f, 1.0f, true, Renderer::LEFT);

    // Mode indicator
    const char* mode_str = "SELECT";
    if (ui.placing) mode_str = "PLACE";
    if (ui.belt_mode) mode_str = "BELT";
    if (ui.delete_mode) mode_str = "DELETE";
    renderer->drawText(0.7f, sb_y, mode_str, 0.018f, 0.9f, 0.9f, 0.3f, 1.0f, true, Renderer::LEFT);

    // ESC hint
    renderer->drawText(0.0f, sb_y - 0.04f, "[ESC] RETURN TO HUB | [SCROLL] ZOOM",
                      0.010f, 0.4f, 0.4f, 0.5f, 0.7f, true, Renderer::CENTER);
}

// ==========================================
// Handle factory UI input
// Returns true if we should stay in factory mode, false to exit
// ==========================================
inline bool HandleFactoryInput(GLFWwindow* window, FactoryUIState& ui, FactorySystem& factory, AgencyState& agency, float mx, float my, bool lmb, float scroll) {
    ui.last_mx = mx;
    ui.last_my = my;

    // ESC -> return to hub
    if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS) {
        return false;
    }

    // Zoom
    if (scroll != 0.0f) {
        ui.zoom += scroll * 0.08f;
        if (ui.zoom < 0.4f) ui.zoom = 0.4f;
        if (ui.zoom > 2.5f) ui.zoom = 2.5f;
    }

    // Convert mouse to grid
    screenToGrid(ui, mx, my, ui.hover_gx, ui.hover_gy);

    // Toolbar hit detection
    float tb_y = 0.9f;
    float btn_x_start = -0.85f;
    float btn_spacing = 0.26f;
    float btn_w = 0.22f;
    float btn_h = 0.08f;
    int total_btns = 7; // 5 buildings + belt + delete

    bool over_side_panel = (ui.selected_node_id >= 0 && mx > 0.525f);

    ui.toolbar_hover = -1;
    if (!over_side_panel) {
        for (int i = 0; i < total_btns; i++) {
            float bx = btn_x_start + i * btn_spacing;
            if (mx >= bx - btn_w/2 && mx <= bx + btn_w/2 && my >= tb_y - btn_h/2 && my <= tb_y + btn_h/2) {
                ui.toolbar_hover = i;
            }
        }
    }

    // Click handling
    static bool lmb_prev = false;
    bool lmb_click = lmb && !lmb_prev;
    lmb_prev = lmb;

    if (lmb_click) {
        // Toolbar clicks
        if (ui.toolbar_hover >= 0 && ui.toolbar_hover < 5) {
            // Building button
            FactoryNodeType types[] = {NODE_MINER, NODE_SMELTER, NODE_ASSEMBLER, NODE_STORAGE, NODE_POWER_PLANT};
            if (ui.placing && ui.place_type == types[ui.toolbar_hover]) {
                ui.placing = false; // Toggle off
            } else {
                ui.placing = true;
                ui.place_type = types[ui.toolbar_hover];
                ui.belt_mode = false;
                ui.delete_mode = false;
            }
        } else if (ui.toolbar_hover == 5) {
            // Belt button
            ui.belt_mode = !ui.belt_mode;
            ui.placing = false;
            ui.delete_mode = false;
            ui.belt_from_id = -1;
        } else if (ui.toolbar_hover == 6) {
            // Delete button
            ui.delete_mode = !ui.delete_mode;
            ui.placing = false;
            ui.belt_mode = false;
        }
        // Grid clicks
        else if (!over_side_panel && gridValid(ui.hover_gx, ui.hover_gy)) {
            if (ui.placing) {
                // ... [Grid click handling remains same] ...
                int existing = findNodeAtGrid(factory, ui.hover_gx, ui.hover_gy);
                if (existing < 0) {
                    double cost = 10000.0;
                    if (ui.place_type == NODE_ASSEMBLER) cost = 25000.0;
                    if (ui.place_type == NODE_STORAGE) cost = 5000.0;
                    if (ui.place_type == NODE_POWER_PLANT) cost = 15000.0;
                    if (agency.funds >= cost) {
                        agency.funds -= cost;
                        factory.addNode(ui.place_type, ui.hover_gx, ui.hover_gy);
                    }
                }
            } else if (ui.belt_mode) {
                int node_id = findNodeAtGrid(factory, ui.hover_gx, ui.hover_gy);
                if (node_id >= 0) {
                    if (ui.belt_from_id < 0) { ui.belt_from_id = node_id; }
                    else {
                        if (node_id != ui.belt_from_id) {
                            ConveyorBelt belt; belt.from_node_id = ui.belt_from_id; belt.to_node_id = node_id;
                            factory.belts.push_back(belt);
                        }
                        ui.belt_from_id = -1;
                    }
                }
            } else if (ui.delete_mode) {
                int node_id = findNodeAtGrid(factory, ui.hover_gx, ui.hover_gy);
                if (node_id >= 0) {
                    factory.removeNode(node_id);
                    agency.funds += 5000.0;
                    for (int i = (int)factory.belts.size() - 1; i >= 0; i--) {
                        if (factory.belts[i].from_node_id == node_id || factory.belts[i].to_node_id == node_id)
                            factory.belts.erase(factory.belts.begin() + i);
                    }
                }
            } else {
                ui.selected_node_id = findNodeAtGrid(factory, ui.hover_gx, ui.hover_gy);
            }
        }
        
        // Side Panel Button Clicks
        if (ui.selected_node_id >= 0) {
            FactoryNode* sel_node = nullptr;
            for (auto& n : const_cast<FactorySystem&>(factory).nodes) { if (n.id == ui.selected_node_id) sel_node = &n; }
            
            if (sel_node && (sel_node->type == NODE_SMELTER || sel_node->type == NODE_ASSEMBLER)) {
                float sp_x = 0.75f;
                float py = 0.3f - 0.1f - 0.05f - 0.08f; // approx Y of buttons
                float btn_w = 0.15f, btn_h = 0.06f;
                
                int rcount = 0;
                GetRecipes(rcount);
                
                // PREV
                if (mx >= sp_x - 0.12f - btn_w/2 && mx <= sp_x - 0.12f + btn_w/2 && my >= py - btn_h/2 && my <= py + btn_h/2) {
                    sel_node->recipe_index = (sel_node->recipe_index - 1 + rcount) % rcount;
                    sel_node->progress = 0.0f; // reset progress on recipe change
                    sel_node->is_producing = false;
                }
                // NEXT
                if (mx >= sp_x + 0.12f - btn_w/2 && mx <= sp_x + 0.12f + btn_w/2 && my >= py - btn_h/2 && my <= py + btn_h/2) {
                    sel_node->recipe_index = (sel_node->recipe_index + 1) % rcount;
                    sel_node->progress = 0.0f;
                    sel_node->is_producing = false;
                }
            }
        }
    }

    return true; // Stay in factory mode
}
