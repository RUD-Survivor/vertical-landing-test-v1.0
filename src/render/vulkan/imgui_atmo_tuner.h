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

    ImGui::SetNextWindowSize(ImVec2(420, 560), ImGuiCond_FirstUseEver);
    if (!ImGui::Begin("大气渲染调参 (壳内/壳外)", p_open, ImGuiWindowFlags_None)) {
        ImGui::End();
        return;
    }

    ImGui::Text("大气壳");
    ImGui::SliderFloat("ShellThickness (km)", &p.shellThicknessKm, 20.0f, 500.0f, "%.0f");
    ImGui::TextDisabled("原来所有星球写死 +160km，现在可调；改动只影响新画的帧，不用重烤 LUT");

    ImGui::Separator();
    ImGui::Text("spaceVisibility（旧壳内星空穿透；统一合成后影响较小）");
    ImGui::SliderFloat("VisStart (altNorm)", &p.spaceVisStart, 0.0f, 1.0f, "%.2f");
    ImGui::SliderFloat("VisEnd (altNorm)",   &p.spaceVisEnd,   0.0f, 1.5f, "%.2f");

    ImGui::Separator();
    ImGui::Text("曝光基准（建议 InnerFar ≈ Outer，减少穿壳跳变）");
    ImGui::SliderFloat("InnerExposureNear (贴地)", &p.innerExposureNear, 0.5f, 50.0f, "%.1f");
    ImGui::SliderFloat("InnerExposureFar (近大气顶)", &p.innerExposureFar, 0.5f, 50.0f, "%.1f");
    ImGui::SliderFloat("OuterExposure", &p.outerExposure, 1.0f, 5000.0f, "%.0f", ImGuiSliderFlags_Logarithmic);
    ImGui::TextDisabled("altNorm=0 用 Near，>=0.6 用 Far，再 smooth 接到 Outer");

    ImGui::Separator();
    ImGui::Text("Limb（仅边缘；高度从底渐变到顶）");
    ImGui::SliderFloat("LimbBrightnessBottom (大气底)", &p.limbBrightnessBottom, 0.5f, 30.0f, "%.1f");
    ImGui::SliderFloat("LimbBrightness (大气顶/默认)", &p.limbBrightness, 0.5f, 30.0f, "%.1f");
    ImGui::SliderFloat("LimbPower (边缘锐度)", &p.limbPower, 0.5f, 8.0f, "%.1f");
    ImGui::TextDisabled("amount = mix(Bottom, Top, smoothstep(0,1,altNorm))；壳外 altNorm>=1 用 Top。");
    ImGui::TextDisabled("E = Exposure * mix(1, amount, rim^Power)。");

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
