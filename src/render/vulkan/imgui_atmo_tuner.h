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

    ImGui::SetNextWindowSize(ImVec2(440, 640), ImGuiCond_FirstUseEver);
    if (!ImGui::Begin("大气渲染调参 (壳内/壳外)", p_open, ImGuiWindowFlags_None)) {
        ImGui::End();
        return;
    }

    bool profileEdited = false;

    ImGui::Text("大气壳");
    profileEdited |= ImGui::SliderFloat("ShellThickness (km)", &p.shellThicknessKm, 20.0f, 500.0f, "%.0f");
    ImGui::TextDisabled("改厚度会重烤 Transmittance LUT（topRadius 变了）。");

    ImGui::Separator();
    ImGui::Text("Transmittance LUT / Profile（改动自动重烤 T+MultiScatter）");
    profileEdited |= ImGui::SliderFloat("RayleighMul", &p.rayleighMul, 0.1f, 5.0f, "%.2f");
    profileEdited |= ImGui::SliderFloat("MieMul", &p.mieMul, 0.1f, 5.0f, "%.2f");
    profileEdited |= ImGui::SliderFloat("OzoneMul", &p.ozoneMul, 0.0f, 5.0f, "%.2f");
    profileEdited |= ImGui::SliderFloat("HRayleighMul", &p.hRayleighMul, 0.25f, 4.0f, "%.2f");
    profileEdited |= ImGui::SliderFloat("HMieMul", &p.hMieMul, 0.25f, 4.0f, "%.2f");
    profileEdited |= ImGui::SliderFloat("GMieOverride", &p.gMieOverride, -1.0f, 0.99f, "%.2f");
    ImGui::TextDisabled("GMie < 0 = 用星球默认；>=0 覆盖 Mie 相位 g。");
    profileEdited |= ImGui::SliderFloat("GroundAlbedo", &p.groundAlbedo, 0.0f, 1.0f, "%.2f");
    profileEdited |= ImGui::SliderFloat("MultiScatterFactor", &p.multipleScatteringFactor, 0.0f, 2.0f, "%.2f");
    profileEdited |= ImGui::SliderFloat("MieAbsFraction", &p.mieAbsFraction, 0.0f, 0.5f, "%.2f");
    ImGui::TextDisabled("Mie 吸收 ≈ mieCoeff × MieAbsFraction（默认 0.10）。");

    ImGui::SliderFloat("TransmittanceChroma", &p.transmittanceChromaStrength, 0.0f, 1.0f, "%.2f");
    ImGui::TextDisabled("scene×T：1=全 RGB LUT 透射；0=仅亮度（减轻地平线偏暖）。不重烤 LUT。");

    if (ImGui::Button("Reset Profile Multipliers")) {
        p.rayleighMul = 1.0f;
        p.mieMul = 1.0f;
        p.ozoneMul = 1.0f;
        p.hRayleighMul = 1.0f;
        p.hMieMul = 1.0f;
        p.gMieOverride = -1.0f;
        p.groundAlbedo = 0.3f;
        p.multipleScatteringFactor = 1.0f;
        p.mieAbsFraction = 0.10f;
        p.transmittanceChromaStrength = 1.0f;
        profileEdited = true;
    }

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
    ImGui::Text("壳内天空分支 (atmo_inside)");
    ImGui::SliderFloat("SkyDepthThreshold", &p.skyDepthThreshold, 0.9999f, 1.0f, "%.6f",
                       ImGuiSliderFlags_Logarithmic);
    ImGui::TextDisabled("sceneDepth >= 阈值 → Sky-View LUT；否则 raymarch+LUT T。默认 1.0。");

    ImGui::Separator();
    ImGui::Text("LUT 缓存");
    ImGui::Text("已缓存星球数: %d", (int)lutCache.byBodyIdx.size());
    if (ImGui::Button("强制重烤全部 Transmittance/MultiScatter"))
        lutCache.invalidateAll();
    // Profile 滑条变化时标脏；下一帧 getOrCreate 比对指纹也会 bakeFull（双保险）。
    if (profileEdited)
        lutCache.invalidateAll();
    ImGui::TextDisabled("Profile/厚度变化会自动重烤；按钮用于手动强制。");

    ImGui::Separator();
    ImGui::TextDisabled("F4: 开关面板");

    ImGui::End();
}
