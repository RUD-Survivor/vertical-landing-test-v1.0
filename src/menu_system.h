#pragma once
// ==========================================================
// menu_system.h — 启动菜单系统
// ==========================================================

#include "save_system.h"
#include <GLFW/glfw3.h>

// 前置声明
class Renderer;

namespace MenuSystem {

enum MenuChoice {
    MENU_NONE = 0,
    MENU_CONTINUE = 1,
    MENU_NEW_GAME = 2,
    MENU_EXIT = 3
};

struct MenuState {
    int selected_option = 0; // 0=继续游戏, 1=新游戏, 2=退出
    bool has_save = false;
    double save_time = 0.0;
    int save_parts = 0;
    float anim_time = 0.0f;
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

// 处理菜单输入
MenuChoice HandleMenuInput(GLFWwindow* window, MenuState& menu, bool& up_pressed, bool& down_pressed, bool& enter_pressed) {
    bool up_now = glfwGetKey(window, GLFW_KEY_UP) == GLFW_PRESS;
    bool down_now = glfwGetKey(window, GLFW_KEY_DOWN) == GLFW_PRESS;
    bool enter_now = glfwGetKey(window, GLFW_KEY_ENTER) == GLFW_PRESS || glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_PRESS;
    
    int max_option = menu.has_save ? 2 : 1;
    
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
            if (menu.selected_option == 0) return MENU_CONTINUE;
            if (menu.selected_option == 1) return MENU_NEW_GAME;
            if (menu.selected_option == 2) return MENU_EXIT;
        } else {
            if (menu.selected_option == 0) return MENU_NEW_GAME;
            if (menu.selected_option == 1) return MENU_EXIT;
        }
    }
    
    if (!enter_now) enter_pressed = false;
    
    return MENU_NONE;
}

} // namespace MenuSystem
