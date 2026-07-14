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
    if (!ImGui::Begin("星空调参 (程序化)", p_open, ImGuiWindowFlags_None)) {
        ImGui::End();
        return;
    }

    ImGui::TextColored(ImVec4(0.75f, 0.85f, 1.0f, 1.0f), "曝光 / 背景");
    ImGui::SliderFloat("Exposure", &p.exposure, 0.1f, 5.0f, "%.2f");
    ImGui::SliderFloat("BgLevel", &p.bgLevel, 0.0f, 5.0f, "%.2f");
    ImGui::TextDisabled("Vibrancy 由场景昼夜自动控制，不在此调。");

    ImGui::Separator();
    ImGui::TextColored(ImVec4(1.0f, 0.95f, 0.7f, 1.0f), "恒星场");
    ImGui::SliderFloat("StarDensity", &p.starDensity, 0.0f, 3.0f, "%.2f");
    ImGui::SliderFloat("StarBrightness", &p.starBrightness, 0.0f, 3.0f, "%.2f");
    ImGui::SliderFloat("StarSize", &p.starSize, 0.25f, 3.0f, "%.2f");
    ImGui::SliderFloat("GlowStrength", &p.glowStrength, 0.0f, 3.0f, "%.2f");
    ImGui::SliderFloat("ColorVariety", &p.colorVariety, 0.0f, 1.0f, "%.2f");
    ImGui::TextDisabled("Density↑ = 更多暗星；Brightness/Size 管亮星与角径。");

    ImGui::Separator();
    ImGui::TextColored(ImVec4(0.9f, 0.7f, 1.0f, 1.0f), "银河");
    ImGui::SliderFloat("MW Intensity", &p.milkyWayIntensity, 0.0f, 3.0f, "%.2f");
    ImGui::SliderFloat("MW Width", &p.milkyWayWidth, 0.25f, 3.0f, "%.2f");
    ImGui::SliderFloat("DustStrength", &p.dustStrength, 0.0f, 3.0f, "%.2f");
    ImGui::SliderFloat("CoreBoost", &p.galacticCoreBoost, 0.0f, 4.0f, "%.2f");
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
