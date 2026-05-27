#pragma once
// ==========================================================
// workshop_scene.h — Vulkan-only stub (UI handled by vk_game_ui.h)
// ==========================================================
#include "scene.h"

class WorkshopScene : public IScene {
public:
    bool done = false;
    void onEnter() override {}
    void update(double dt) override { (void)dt; }
};
