#pragma once
// ==========================================================================
// imgui_atmo_tuner.h — 大气渲染重构（壳内/壳外 LUT 路径）调参面板
// 用法: AtmoTunerImGui(&open, g_vkR3D.atmoTune, g_vkR3D.scene.atmoLutCache);
// F4 开关，跟随 imgui_cloud_tuner.h（F2/F3）的排版风格。
// ==========================================================================
#include "vk_atmosphere_lut.h" // AtmoTuneParams / VkAtmoLutCache
#include "imgui.h"

inline void AtmoTunerImGui(bool* p_open, AtmoTuneParams& p, VkAtmoLutCache& lutCache) {
    if (ImGui::IsKeyPressed(ImGuiKey_F4, false)) *p_open = !*p_open;
    if (!*p_open) return;

    ImGui::SetNextWindowSize(ImVec2(380, 300), ImGuiCond_FirstUseEver);
    if (!ImGui::Begin("大气渲染调参 (壳内/壳外)", p_open, ImGuiWindowFlags_None)) {
        ImGui::End();
        return;
    }

    ImGui::Text("大气壳");
    ImGui::SliderFloat("ShellThickness (km)", &p.shellThicknessKm, 20.0f, 500.0f, "%.0f");
    ImGui::TextDisabled("原来所有星球写死 +160km，现在可调；改动只影响新画的帧，不用重烤 LUT");

    ImGui::Separator();
    ImGui::Text("spaceVisibility（星空穿透，只影响壳内天空背景）");
    ImGui::SliderFloat("VisStart (altNorm)", &p.spaceVisStart, 0.0f, 1.0f, "%.2f");
    ImGui::SliderFloat("VisEnd (altNorm)",   &p.spaceVisEnd,   0.0f, 1.5f, "%.2f");
    ImGui::TextDisabled("altNorm=相机高度/壳厚度，0=贴地 1=大气顶。VisStart 开始透星空，VisEnd 完全透明");

    ImGui::Separator();
    ImGui::Text("壳外 limb 光晕（原来是黑的：raymarch 算出来的散射量级本来就很小，");
    ImGui::Text("壳内有 5~10x exposure，壳外原来完全没有对应倍率，先调 OuterExposure）");
    ImGui::SliderFloat("OuterExposure", &p.outerExposure, 1.0f, 5000.0f, "%.0f", ImGuiSliderFlags_Logarithmic);
    ImGui::SliderFloat("LimbBrightness (掠射边缘增强)", &p.limbBrightness, 0.5f, 30.0f, "%.1f");
    ImGui::TextDisabled("OuterExposure 是整体曝光倍率，LimbBrightness 只影响掠射角边缘的额外提亮");

    ImGui::Separator();
    ImGui::Text("LUT 缓存");
    ImGui::Text("已缓存星球数: %d", (int)lutCache.byBodyIdx.size());
    if (ImGui::Button("强制重烤全部 Transmittance/MultiScatter")) {
        lutCache.invalidateAll();
    }
    ImGui::TextDisabled("散射系数(getPlanetScatteringCoeffs)本身改了代码才需要点这个；半径/atmoIdx 变化会自动触发");

    ImGui::Separator();
    ImGui::TextDisabled("F4: 开关面板");

    ImGui::End();
}
