#pragma once
#include <GLFW/glfw3.h>
#include <vector>
#include <functional>

struct KeyAction {
    int key; // GLFW_KEY_*
    std::function<void()> on_press;
    bool was_pressed = false;
};

struct MouseAction {
    int button; // GLFW_MOUSE_BUTTON_*
    std::function<void()> on_press;
    bool was_pressed = false;
};

struct ShiftKeyAction {
    int key;
    std::function<void()> on_press;
    bool was_pressed = false;
};

class InputRouter {
private:
    std::vector<KeyAction> key_actions;
    std::vector<ShiftKeyAction> shift_actions;
    std::vector<MouseAction> mouse_actions;

    bool space_or_lmb_pressed = true; // Initialized to true to ignore initial editor press
    
public:
    void registerKey(int key, std::function<void()> callback) {
        key_actions.push_back({key, callback, false});
    }

    void registerShiftKey(int key, std::function<void()> callback) {
        shift_actions.push_back({key, callback, false});
    }

    void registerMouseButton(int button, std::function<void()> callback) {
        mouse_actions.push_back({button, callback, false});
    }

    // Special trigger for both SPACE and LMB (Ignition)
    std::function<void()> on_ignition;

    void poll(GLFWwindow* window) {
        // Track SHIFT modifier
        bool shift_down = glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS || 
                          glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS;

        // Process standard keys
        for (auto& action : key_actions) {
            bool is_pressed = glfwGetKey(window, action.key) == GLFW_PRESS;
            if (is_pressed && !action.was_pressed && !shift_down) {
                if (action.on_press) action.on_press();
            }
            action.was_pressed = is_pressed;
        }

        // Process shift + keys
        for (auto& action : shift_actions) {
            bool is_pressed = glfwGetKey(window, action.key) == GLFW_PRESS;
            if (is_pressed && shift_down && !action.was_pressed) {
                if (action.on_press) action.on_press();
            }
            action.was_pressed = (is_pressed && shift_down);
        }

        // Process Mouse Buttons
        for (auto& action : mouse_actions) {
            bool is_pressed = glfwGetMouseButton(window, action.button) == GLFW_PRESS;
            if (is_pressed && !action.was_pressed) {
                if (action.on_press) action.on_press();
            }
            action.was_pressed = is_pressed;
        }

        // Special Ignition Trigger (Space or LMB)
        bool lmb = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS;
        bool space = glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_PRESS;
        bool ignition_now = lmb || space;
        
        if (ignition_now && !space_or_lmb_pressed) {
            if (on_ignition) on_ignition();
        }
        space_or_lmb_pressed = ignition_now;
    }

    // Expose direct polling for continuous actions (like camera movement)
    bool isKeyPressed(GLFWwindow* window, int key) const {
        return glfwGetKey(window, key) == GLFW_PRESS;
    }
    
    bool isMouseButtonPressed(GLFWwindow* window, int button) const {
        return glfwGetMouseButton(window, button) == GLFW_PRESS;
    }
    
    bool isShiftPressed(GLFWwindow* window) const {
        return glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS || 
               glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS;
    }
};
