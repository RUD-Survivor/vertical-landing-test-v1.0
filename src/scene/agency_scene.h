#pragma once
// ==========================================================
// agency_scene.h — Vulkan-only stub (UI handled by vk_game_ui.h)
// ==========================================================
#include "scene.h"

class AgencyScene : public IScene {
public:
    int next_scene_flag = 0;
    void onEnter() override {}
    void update(double dt) override { (void)dt; }
};
