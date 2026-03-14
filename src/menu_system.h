#pragma once
// ==========================================================
// menu_system.h — 启动菜单系统
// ==========================================================

#include "save_system.h"
#include "core/agency_state.h"
#include "simulation/resource_system.h"
#include "simulation/factory_system.h"
#include <GLFW/glfw3.h>

// 前置声明
class Renderer;

namespace MenuSystem {

enum MenuChoice {
    MENU_NONE = 0,
    MENU_CONTINUE = 1,
    MENU_NEW_GAME = 2,
    MENU_EXIT = 3,
    MENU_AGENCY_HUB = 4,
    MENU_VAB = 5,
    MENU_FACTORY = 6
};

struct MenuState {
    int selected_option = 0; // 0=继续/HUB, 1=新游戏, 2=退出...
    bool has_save = false;
    double save_time = 0.0;
    int save_parts = 0;
    float anim_time = 0.0f;
    
    // For Agency Hub
    int hub_selected_option = 0; // 0=VAB, 1=Launchpad, 2=Factory, 3=Back
};


// 绘制主菜单
void DrawMainMenu(Renderer* r, MenuState& menu, float time) {
    // 背景
    r->addRect(0.0f, 0.0f, 2.0f, 2.0f, 0.02f, 0.03f, 0.08f, 1.0f);
    
    // 星空背景
    for (int i = 0; i < 150; i++) {
        float sx = ::hash11(i * 3917) * 2.0f - 1.0f;
        float sy = ::hash11(i * 7121) * 2.0f - 1.0f;
        float ss = 0.001f + ::hash11(i * 2131) * 0.003f;
        float sa = 0.3f + ::hash11(i * 991) * 0.5f;
        sa *= 0.7f + 0.3f * sinf(time * (1.0f + ::hash11(i * 443) * 2.0f) + ::hash11(i * 661) * 6.28f);
        r->addRect(sx, sy, ss, ss, 0.8f, 0.85f, 1.0f, sa);
    }
    
    // 标题
    r->addRect(0.0f, 0.65f, 1.2f, 0.20f, 0.05f, 0.08f, 0.15f, 0.85f);
    float title_pulse = 0.8f + 0.2f * sinf(time * 2.0f);
    r->drawText(0.0f, 0.68f, "VERTICAL LANDING", 0.045f, 
                0.3f * title_pulse, 0.9f * title_pulse, 1.0f * title_pulse, 1.0f, true, Renderer::CENTER);
    r->drawText(0.0f, 0.60f, "SIMULATOR", 0.038f, 
                0.4f, 0.8f, 0.9f, 0.9f, true, Renderer::CENTER);
    
    // 菜单选项
    float option_y_start = 0.25f;
    float option_spacing = 0.15f;
    int num_options = menu.has_save ? 3 : 2;
    
    // 选项0: 继续游戏 (仅当有存档时显示)
    if (menu.has_save) {
        bool selected = (menu.selected_option == 0);
        float box_w = 0.50f;
        float box_h = 0.10f;
        float y = option_y_start;
        
        // 背景框
        if (selected) {
            float pulse = 0.7f + 0.3f * sinf(time * 5.0f);
            r->addRect(0.0f, y, box_w, box_h, 0.10f * pulse, 0.30f * pulse, 0.15f * pulse, 0.8f);
            r->addRect(-box_w/2.0f - 0.02f, y, 0.01f, box_h * 0.7f, 0.2f, 1.0f, 0.4f, pulse);
        } else {
            r->addRect(0.0f, y, box_w, box_h, 0.06f, 0.08f, 0.12f, 0.6f);
        }
        
        // 文字
        float text_r = selected ? 0.3f : 0.5f;
        float text_g = selected ? 1.0f : 0.6f;
        float text_b = selected ? 0.5f : 0.6f;
        r->drawText(0.0f, y + 0.02f, "CONTINUE GAME", 0.022f, text_r, text_g, text_b, 1.0f, true, Renderer::CENTER);
        
        // 存档信息
        char info[64];
        int hours = (int)(menu.save_time / 3600.0);
        int minutes = (int)((menu.save_time - hours * 3600) / 60.0);
        snprintf(info, sizeof(info), "TIME: %02d:%02d | PARTS: %d", hours, minutes, menu.save_parts);
        r->drawText(0.0f, y - 0.025f, info, 0.012f, 0.6f, 0.6f, 0.7f, 0.8f, true, Renderer::CENTER);
    }
    
    // 选项1: 新游戏
    {
        int option_idx = menu.has_save ? 1 : 0;
        bool selected = (menu.selected_option == option_idx);
        float box_w = 0.50f;
        float box_h = 0.10f;
        float y = option_y_start - option_spacing * (menu.has_save ? 1 : 0);
        
        if (selected) {
            float pulse = 0.7f + 0.3f * sinf(time * 5.0f);
            r->addRect(0.0f, y, box_w, box_h, 0.10f * pulse, 0.30f * pulse, 0.15f * pulse, 0.8f);
            r->addRect(-box_w/2.0f - 0.02f, y, 0.01f, box_h * 0.7f, 0.2f, 1.0f, 0.4f, pulse);
        } else {
            r->addRect(0.0f, y, box_w, box_h, 0.06f, 0.08f, 0.12f, 0.6f);
        }
        
        float text_r = selected ? 0.3f : 0.5f;
        float text_g = selected ? 1.0f : 0.6f;
        float text_b = selected ? 0.5f : 0.6f;
        r->drawText(0.0f, y + 0.01f, "NEW GAME", 0.022f, text_r, text_g, text_b, 1.0f, true, Renderer::CENTER);
        
        if (menu.has_save) {
            r->drawText(0.0f, y - 0.025f, "WARNING: WILL DELETE SAVE", 0.010f, 1.0f, 0.5f, 0.3f, 0.7f, true, Renderer::CENTER);
        }
    }
    
    // 选项2: 退出
    {
        int option_idx = menu.has_save ? 2 : 1;
        bool selected = (menu.selected_option == option_idx);
        float box_w = 0.50f;
        float box_h = 0.10f;
        float y = option_y_start - option_spacing * (menu.has_save ? 2 : 1);
        
        if (selected) {
            float pulse = 0.7f + 0.3f * sinf(time * 5.0f);
            r->addRect(0.0f, y, box_w, box_h, 0.30f * pulse, 0.10f * pulse, 0.10f * pulse, 0.8f);
            r->addRect(-box_w/2.0f - 0.02f, y, 0.01f, box_h * 0.7f, 1.0f, 0.2f, 0.2f, pulse);
        } else {
            r->addRect(0.0f, y, box_w, box_h, 0.08f, 0.06f, 0.06f, 0.6f);
        }
        
        float text_r = selected ? 1.0f : 0.6f;
        float text_g = selected ? 0.3f : 0.5f;
        float text_b = selected ? 0.3f : 0.5f;
        r->drawText(0.0f, y, "EXIT", 0.022f, text_r, text_g, text_b, 1.0f, true, Renderer::CENTER);
    }
    
    // 底部提示
    r->drawText(0.0f, -0.85f, "USE UP/DOWN ARROWS TO SELECT", 0.014f, 0.4f, 0.4f, 0.5f, 0.7f, true, Renderer::CENTER);
    r->drawText(0.0f, -0.90f, "PRESS ENTER TO CONFIRM", 0.014f, 0.4f, 0.4f, 0.5f, 0.7f, true, Renderer::CENTER);
}

// 绘制航天局总览界面
void DrawAgencyHub(Renderer* r, MenuState& menu, const AgencyState& agency, float time, const FactorySystem& factory) {
    // 背景
    r->addRect(0.0f, 0.0f, 2.0f, 2.0f, 0.05f, 0.08f, 0.12f, 1.0f);
    
    // 顶部状态栏
    r->addRect(0.0f, 0.9f, 2.0f, 0.2f, 0.02f, 0.03f, 0.05f, 0.9f);
    r->addRect(0.0f, 0.8f, 2.0f, 0.01f, 0.2f, 0.4f, 0.8f, 0.8f); // 蓝线
    
    char buf[128];
    snprintf(buf, sizeof(buf), "FUNDS: $%.0f", agency.funds);
    r->drawText(-0.9f, 0.9f, buf, 0.02f, 0.4f, 0.8f, 0.4f, 1.0f, true, Renderer::LEFT);
    
    snprintf(buf, sizeof(buf), "REP: %.1f", agency.reputation);
    r->drawText(-0.1f, 0.9f, buf, 0.02f, 0.2f, 0.6f, 1.0f, 1.0f, true, Renderer::LEFT);
    
    int days = (int)(agency.global_time / 86400.0);
    snprintf(buf, sizeof(buf), "DAY: %d", days);
    r->drawText(0.6f, 0.9f, buf, 0.02f, 0.9f, 0.9f, 0.9f, 1.0f, true, Renderer::LEFT);

    // 标题
    r->drawText(0.0f, 0.65f, "SPACE AGENCY HUB", 0.04f, 0.9f, 0.9f, 0.9f, 1.0f, true, Renderer::CENTER);

    // 工厂状态总览面板 (左侧)
    float panel_x = -0.5f;
    float panel_y = 0.1f;
    float panel_w = 0.7f;
    float panel_h = 0.8f;
    r->addRect(panel_x, panel_y, panel_w, panel_h, 0.1f, 0.12f, 0.15f, 0.8f);
    // Panel border (top, bottom, left, right)
    r->addRect(panel_x, panel_y + panel_h/2.0f, panel_w, 0.005f, 0.3f, 0.4f, 0.5f, 0.8f);
    r->addRect(panel_x, panel_y - panel_h/2.0f, panel_w, 0.005f, 0.3f, 0.4f, 0.5f, 0.8f);
    r->addRect(panel_x - panel_w/2.0f, panel_y, 0.005f, panel_h, 0.3f, 0.4f, 0.5f, 0.8f);
    r->addRect(panel_x + panel_w/2.0f, panel_y, 0.005f, panel_h, 0.3f, 0.4f, 0.5f, 0.8f);
    
    r->drawText(panel_x, panel_y + 0.3f, "FACTORY OVERVIEW", 0.02f, 0.8f, 0.6f, 0.2f, 1.0f, true, Renderer::CENTER);
    r->addRect(panel_x, panel_y + 0.25f, panel_w * 0.8f, 0.005f, 0.8f, 0.6f, 0.2f, 0.5f);
    
    int n_miners = factory.countType(NODE_MINER);
    int n_smelters = factory.countType(NODE_SMELTER);
    int n_assemblers = factory.countType(NODE_ASSEMBLER);
    bool has_factory = (n_miners + n_smelters + n_assemblers) > 0;
    
    if (has_factory) {
        r->drawText(panel_x - 0.25f, panel_y + 0.15f, "STATUS: ONLINE", 0.015f, 0.3f, 1.0f, 0.4f, 1.0f, true, Renderer::LEFT);
    } else {
        r->drawText(panel_x - 0.25f, panel_y + 0.15f, "STATUS: OFFLINE", 0.015f, 1.0f, 0.3f, 0.3f, 1.0f, true, Renderer::LEFT);
    }
    
    char stat_buf[64];
    snprintf(stat_buf, sizeof(stat_buf), "MINERS: %d", n_miners);
    r->drawText(panel_x - 0.25f, panel_y + 0.05f, stat_buf, 0.015f, 0.6f, 0.4f, 0.3f, 1.0f, true, Renderer::LEFT);
    snprintf(stat_buf, sizeof(stat_buf), "SMELTERS: %d", n_smelters);
    r->drawText(panel_x - 0.25f, panel_y - 0.01f, stat_buf, 0.015f, 0.7f, 0.5f, 0.2f, 1.0f, true, Renderer::LEFT);
    snprintf(stat_buf, sizeof(stat_buf), "ASSEMBLERS: %d", n_assemblers);
    r->drawText(panel_x - 0.25f, panel_y - 0.07f, stat_buf, 0.015f, 0.3f, 0.7f, 0.3f, 1.0f, true, Renderer::LEFT);
    
    // Production progress bars for active nodes
    float bar_y = panel_y - 0.14f;
    for (int i = 0; i < (int)factory.nodes.size() && i < 5; i++) {
        const FactoryNode& fn = factory.nodes[i];
        int recipe_count = 0;
        const Recipe* recipes = GetRecipes(recipe_count);
        const char* label = NodeTypeName(fn.type);
        if (fn.type == NODE_MINER) {
            label = GetItemInfo(fn.mine_output).name;
        } else if (fn.recipe_index >= 0 && fn.recipe_index < recipe_count) {
            label = recipes[fn.recipe_index].name;
        }
        // Label
        r->drawText(panel_x - 0.25f, bar_y, label, 0.010f, 0.8f, 0.8f, 0.8f, 1.0f, true, Renderer::LEFT);
        // Progress bar background
        float bar_x = panel_x + 0.05f;
        float bar_w = 0.2f;
        float bar_h = 0.018f;
        r->addRect(bar_x, bar_y, bar_w, bar_h, 0.15f, 0.15f, 0.15f, 0.8f);
        // Progress bar fill
        float fill = fn.progress;
        if (fill > 0.0f) {
            float fill_w = bar_w * fill;
            r->addRect(bar_x - bar_w/2.0f + fill_w/2.0f, bar_y, fill_w, bar_h * 0.8f, 0.2f, 0.8f, 0.3f, 0.9f);
        }
        bar_y -= 0.035f;
    }
    
    float wh_y = bar_y - 0.02f;
    r->drawText(panel_x - 0.25f, wh_y, "-- WAREHOUSE --", 0.015f, 0.5f, 0.8f, 1.0f, 1.0f, true, Renderer::LEFT);
    
    // Dynamic inventory display
    if (agency.inventory.empty()) {
        r->drawText(panel_x - 0.25f, wh_y - 0.05f, "NO ITEMS IN STORAGE", 0.012f, 0.5f, 0.5f, 0.5f, 1.0f, true, Renderer::LEFT);
    } else {
        float inv_y = wh_y - 0.04f;
    
    // Group 1: Raw & Processed
    r->drawText(panel_x - 0.25f, inv_y, "RESOURCES:", 0.009f, 0.4f, 0.6f, 0.8f, 0.8f, true, Renderer::LEFT);
    inv_y -= 0.03f;
    for (auto const& kv : agency.inventory) {
        const ItemInfo& info = GetItemInfo(kv.first);
        if (strcmp(info.category, "PART") != 0 && kv.second > 0) {
            char line[64];
            snprintf(line, sizeof(line), "- %s x%d", info.name, kv.second);
            r->drawText(panel_x - 0.25f, inv_y, line, 0.011f, info.icon_r, info.icon_g, info.icon_b, 1.0f, true, Renderer::LEFT);
            inv_y -= 0.03f;
        }
    }
    
    // Group 2: Rocket Parts
    inv_y -= 0.02f;
    r->drawText(panel_x - 0.25f, inv_y, "ROCKET PARTS:", 0.009f, 0.8f, 0.5f, 0.2f, 0.8f, true, Renderer::LEFT);
    inv_y -= 0.03f;
    bool has_parts = false;
    for (auto const& kv : agency.inventory) {
        const ItemInfo& info = GetItemInfo(kv.first);
        if (strcmp(info.category, "PART") == 0 && kv.second > 0) {
            has_parts = true;
            char line[64];
            snprintf(line, sizeof(line), "- %s x%d", info.name, kv.second);
            r->drawText(panel_x - 0.25f, inv_y, line, 0.011f, info.icon_r, info.icon_g, info.icon_b, 1.0f, true, Renderer::LEFT);
            inv_y -= 0.03f;
        }
    }
    if (!has_parts) {
        r->drawText(panel_x - 0.25f, inv_y, "(NONE)", 0.010f, 0.4f, 0.4f, 0.4f, 0.7f, true, Renderer::LEFT);
    }
}

    // 交互选项 (右侧)
    float opt_x = 0.4f;
    float opt_start_y = 0.3f;
    float opt_spacing = 0.18f;
    
    const char* labels[] = {
        "VEHICLE ASSEMBLY (VAB)",
        "LAUNCHPAD",
        "ENTER FACTORY MODE",
        "RETURN TO MAIN MENU"
    };
    
    for (int i = 0; i < 4; i++) {
        bool selected = (menu.hub_selected_option == i);
        float box_w = 0.6f;
        float box_h = 0.12f;
        float y = opt_start_y - i * opt_spacing;
        
        if (selected) {
            float pulse = 0.7f + 0.3f * sinf(time * 5.0f);
            if (i == 2) { // Factory button styling
                r->addRect(opt_x, y, box_w, box_h, 0.3f * pulse, 0.2f * pulse, 0.05f * pulse, 0.8f);
                r->addRect(opt_x - box_w/2.0f - 0.01f, y, 0.01f, box_h * 0.7f, 1.0f, 0.6f, 0.2f, pulse);
            } else if (i == 3) { // Back
                r->addRect(opt_x, y, box_w, box_h, 0.2f * pulse, 0.05f * pulse, 0.05f * pulse, 0.8f);
                r->addRect(opt_x - box_w/2.0f - 0.01f, y, 0.01f, box_h * 0.7f, 1.0f, 0.2f, 0.2f, pulse);
            } else {
                r->addRect(opt_x, y, box_w, box_h, 0.10f * pulse, 0.20f * pulse, 0.30f * pulse, 0.8f);
                r->addRect(opt_x - box_w/2.0f - 0.01f, y, 0.01f, box_h * 0.7f, 0.2f, 0.6f, 1.0f, pulse);
            }
        } else {
            r->addRect(opt_x, y, box_w, box_h, 0.08f, 0.08f, 0.10f, 0.8f);
            r->addRect(opt_x, y + box_h/2.0f, box_w, 0.003f, 0.2f, 0.2f, 0.25f, 0.5f);
            r->addRect(opt_x, y - box_h/2.0f, box_w, 0.003f, 0.2f, 0.2f, 0.25f, 0.5f);
        }
        
        float text_r = selected ? 1.0f : 0.6f;
        float text_g = selected ? 1.0f : 0.6f;
        float text_b = selected ? 1.0f : 0.6f;
        if (selected && i == 2) { text_r=1.0f; text_g=0.8f; text_b=0.4f; } // Orange for factory
        r->drawText(opt_x, y, labels[i], 0.018f, text_r, text_g, text_b, 1.0f, true, Renderer::CENTER);
    }
}

// 处理菜单选项点击检测
inline bool CheckRectHit(float mx, float my, float cx, float cy, float w, float h) {
    return (mx >= cx - w / 2.0f && mx <= cx + w / 2.0f &&
            my >= cy - h / 2.0f && my <= cy + h / 2.0f);
}

// 处理菜单输入
MenuChoice HandleMenuInput(GLFWwindow* window, MenuState& menu, bool& up_pressed, bool& down_pressed, bool& enter_pressed, float mx, float my, bool lmb) {
    static bool lmb_pressed = false;
    bool up_now = glfwGetKey(window, GLFW_KEY_UP) == GLFW_PRESS;
    bool down_now = glfwGetKey(window, GLFW_KEY_DOWN) == GLFW_PRESS;
    bool enter_now = glfwGetKey(window, GLFW_KEY_ENTER) == GLFW_PRESS || glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_PRESS;
    
    int max_option = menu.has_save ? 2 : 1;
    
    // 鼠标点击检测
    if (lmb && !lmb_pressed) {
        float option_y_start = 0.25f;
        float option_spacing = 0.15f;
        float box_w = 0.50f;
        float box_h = 0.10f;

        if (menu.has_save) {
            if (CheckRectHit(mx, my, 0.0f, option_y_start, box_w, box_h)) return MENU_AGENCY_HUB; // Continue -> Hub
            if (CheckRectHit(mx, my, 0.0f, option_y_start - option_spacing, box_w, box_h)) return MENU_AGENCY_HUB; // New Game -> Hub
            if (CheckRectHit(mx, my, 0.0f, option_y_start - option_spacing * 2.0f, box_w, box_h)) return MENU_EXIT;
        } else {
            if (CheckRectHit(mx, my, 0.0f, option_y_start, box_w, box_h)) return MENU_AGENCY_HUB; // New Game -> Hub
            if (CheckRectHit(mx, my, 0.0f, option_y_start - option_spacing, box_w, box_h)) return MENU_EXIT;
        }
    }
    lmb_pressed = lmb;

    // 鼠标悬停检测
    {
        float option_y_start = 0.25f;
        float option_spacing = 0.15f;
        float box_w = 0.50f;
        float box_h = 0.10f;
        
        if (menu.has_save) {
            if (CheckRectHit(mx, my, 0.0f, option_y_start, box_w, box_h)) menu.selected_option = 0;
            else if (CheckRectHit(mx, my, 0.0f, option_y_start - option_spacing, box_w, box_h)) menu.selected_option = 1;
            else if (CheckRectHit(mx, my, 0.0f, option_y_start - option_spacing * 2.0f, box_w, box_h)) menu.selected_option = 2;
        } else {
            if (CheckRectHit(mx, my, 0.0f, option_y_start, box_w, box_h)) menu.selected_option = 0;
            else if (CheckRectHit(mx, my, 0.0f, option_y_start - option_spacing, box_w, box_h)) menu.selected_option = 1;
        }
    }
    
    // 上下键导航
    if (up_now && !up_pressed) {
        menu.selected_option--;
        if (menu.selected_option < 0) menu.selected_option = max_option;
    }
    if (down_now && !down_pressed) {
        menu.selected_option++;
        if (menu.selected_option > max_option) menu.selected_option = 0;
    }
    
    up_pressed = up_now;
    down_pressed = down_now;
    
    // 回车确认
    if (enter_now && !enter_pressed) {
        enter_pressed = true;
        
        if (menu.has_save) {
            if (menu.selected_option == 0) return MENU_AGENCY_HUB; // Continue -> Hub
            if (menu.selected_option == 1) return MENU_AGENCY_HUB; // New Game -> Hub
            if (menu.selected_option == 2) return MENU_EXIT;
        } else {
            if (menu.selected_option == 0) return MENU_AGENCY_HUB; // New Game -> Hub
            if (menu.selected_option == 1) return MENU_EXIT;
        }
    }
    
    if (!enter_now) enter_pressed = false;
    
    return MENU_NONE;
}

// 处理航天局界面输入
MenuChoice HandleAgencyHubInput(GLFWwindow* window, MenuState& menu, bool& up_pressed, bool& down_pressed, bool& enter_pressed, float mx, float my, bool lmb) {
    static bool hub_lmb_pressed = false;
    bool up_now = glfwGetKey(window, GLFW_KEY_UP) == GLFW_PRESS;
    bool down_now = glfwGetKey(window, GLFW_KEY_DOWN) == GLFW_PRESS;
    bool enter_now = glfwGetKey(window, GLFW_KEY_ENTER) == GLFW_PRESS || glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_PRESS;
    
    // 鼠标点击
    if (lmb && !hub_lmb_pressed) {
        float opt_x = 0.4f;
        float opt_start_y = 0.3f;
        float opt_spacing = 0.18f;
        float box_w = 0.6f;
        float box_h = 0.12f;
        
        if (CheckRectHit(mx, my, opt_x, opt_start_y, box_w, box_h)) return MENU_VAB;
        if (CheckRectHit(mx, my, opt_x, opt_start_y - opt_spacing, box_w, box_h)) return MENU_CONTINUE; // Launchpad
        if (CheckRectHit(mx, my, opt_x, opt_start_y - opt_spacing * 2, box_w, box_h)) return MENU_FACTORY;
        if (CheckRectHit(mx, my, opt_x, opt_start_y - opt_spacing * 3, box_w, box_h)) return MENU_NONE; // Return to main (handled in main)
    }
    hub_lmb_pressed = lmb;

    // 鼠标悬停
    {
        float opt_x = 0.4f;
        float opt_start_y = 0.3f;
        float opt_spacing = 0.18f;
        float box_w = 0.6f;
        float box_h = 0.12f;
        if (CheckRectHit(mx, my, opt_x, opt_start_y, box_w, box_h)) menu.hub_selected_option = 0;
        else if (CheckRectHit(mx, my, opt_x, opt_start_y - opt_spacing, box_w, box_h)) menu.hub_selected_option = 1;
        else if (CheckRectHit(mx, my, opt_x, opt_start_y - opt_spacing * 2, box_w, box_h)) menu.hub_selected_option = 2;
        else if (CheckRectHit(mx, my, opt_x, opt_start_y - opt_spacing * 3, box_w, box_h)) menu.hub_selected_option = 3;
    }

    // 键盘导航
    if (up_now && !up_pressed) {
        menu.hub_selected_option--;
        if (menu.hub_selected_option < 0) menu.hub_selected_option = 3;
    }
    if (down_now && !down_pressed) {
        menu.hub_selected_option++;
        if (menu.hub_selected_option > 3) menu.hub_selected_option = 0;
    }
    
    up_pressed = up_now;
    down_pressed = down_now;
    
    // 回车确认
    if (enter_now && !enter_pressed) {
        enter_pressed = true;
        
        if (menu.hub_selected_option == 0) return MENU_VAB;
        if (menu.hub_selected_option == 1) return MENU_CONTINUE; // Launchpad
        if (menu.hub_selected_option == 2) return MENU_FACTORY;
        if (menu.hub_selected_option == 3) return MENU_NONE; // Signal main menu return
    }
    
    if (!enter_now) enter_pressed = false;
    
    // ESC 键返还主菜单
    static bool hub_esc_held = false;
    bool esc_now = glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS;
    if (esc_now && !hub_esc_held) {
        hub_esc_held = true;
        return MENU_NONE; // Return to main menu
    }
    if (!esc_now) hub_esc_held = false;
    
    return MENU_AGENCY_HUB; // Indicate we are still in hub
}

} // namespace MenuSystem
