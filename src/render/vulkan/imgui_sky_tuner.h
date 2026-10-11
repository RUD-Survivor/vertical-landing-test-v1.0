#pragma once
// ==========================================================================
// imgui_sky_tuner.h — 程序化星空 / 银河调参面板
// 用法: SkyTunerImGui(&open, g_vkR3D.sky.tune);
// F5 开关。
// ==========================================================================
#include "systems/vk_sky.h" // SkyTuneParams
#include "imgui.h"

inline void SkyTunerImGui(bool* p_open, SkyTuneParams& p) {
    if (ImGui::IsKeyPressed(ImGuiKey_F5, false)) *p_open = !*p_open;
    if (!*p_open) return;

    ImGui::SetNextWindowSize(ImVec2(400, 560), ImGuiCond_FirstUseEver);
    if (!ImGui::Begin("星空调参", p_open, ImGuiWindowFlags_None)) {
        ImGui::End();
        return;
    }

    ImGui::TextColored(ImVec4(0.75f, 0.85f, 1.0f, 1.0f), "曝光 / 背景");
    ImGui::Checkbox("银河观测（提高相机曝光）", &p.observationMode);
    ImGui::TextDisabled("观测曝光作用于整个画面，受光地球会更亮。");
    ImGui::SliderFloat("Radiance scale", &p.exposure, 0.0f, 4.0f, "%.2f");
    ImGui::SliderFloat("BgLevel", &p.bgLevel, 0.0f, 5.0f, "%.2f");
    ImGui::TextDisabled("Vibrancy 由场景昼夜自动控制，不在此调。");

    ImGui::Separator();
    ImGui::TextColored(ImVec4(1.0f, 0.95f, 0.7f, 1.0f), "恒星场");
    ImGui::SliderFloat("Catalog fraction", &p.starDensity, 0.0f, 1.0f, "%.2f");
    ImGui::SliderFloat("StarBrightness", &p.starBrightness, 0.0f, 3.0f, "%.2f");
    ImGui::SliderFloat("PSF sigma (px)", &p.starSize, 0.4f, 2.0f, "%.2f");
    ImGui::SliderFloat("Halo flux fraction", &p.glowStrength, 0.0f, 0.15f, "%.3f");
    ImGui::SliderFloat("ColorVariety", &p.colorVariety, 0.0f, 1.0f, "%.2f");
    ImGui::TextDisabled("HYG stars to magnitude 6.5; PSF size conserves flux.");

    ImGui::Separator();
    ImGui::TextColored(ImVec4(0.9f, 0.7f, 1.0f, 1.0f), "银河");
    ImGui::SliderFloat("MW Intensity", &p.milkyWayIntensity, 0.0f, 3.0f, "%.2f");
    ImGui::TextDisabled("NASA SVS / Gaia 银河背景；亮星独立渲染。");
    ImGui::SliderFloat("GalaxyYaw (rad)", &p.galaxyYaw, -3.1416f, 3.1416f, "%.2f");
    ImGui::SliderFloat("GalaxyPitch (rad)", &p.galaxyPitch, -1.57f, 1.57f, "%.2f");
    ImGui::TextDisabled("Yaw/Pitch 旋转银道相对世界坐标的取向。");

    ImGui::Separator();
    if (ImGui::Button("Reset Stars / Milky Way")) {
        p = SkyTuneParams{};
    }

    ImGui::Separator();
    ImGui::TextDisabled("F5: 开关面板");

    ImGui::End();
}
