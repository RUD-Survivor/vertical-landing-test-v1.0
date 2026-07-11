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

    ImGui::SetNextWindowSize(ImVec2(440, 720), ImGuiCond_FirstUseEver);
    if (!ImGui::Begin("大气渲染调参 (壳内/壳外)", p_open, ImGuiWindowFlags_None)) {
        ImGui::End();
        return;
    }

    ImGui::Text("相机（只读，壳内大气帧更新）");
    ImGui::Text("Alt: %.3f km  (%.0f m)", p.dbgCamAltKm, p.dbgCamAltKm * 1000.0f);
    ImGui::Text("InsideSurf: %s   InsideAtmo: %s",
                p.dbgCamInsideSurf > 0.5f ? "YES" : "no",
                p.dbgCamInsideAtmo > 0.5f ? "YES" : "no");

    ImGui::Separator();
    ImGui::Text("路径诊断（壳内 atmo_inside）");
    const char* pathModes[] = {
        "0 Normal",
        "1 ForceSky (永不裁 tFar)",
        "2 ForceGround (强制裁短)",
        "3 NoSphereClip",
        "4 NoDepthClip",
        "5 VizPath (蓝=天空红=地面)",
        "6 VizInsideWhite (球内白/品红)"
    };
    ImGui::Combo("PathDebug", &p.pathDebug, pathModes, 7);
    ImGui::SliderFloat("SkyDepthThresh", &p.skyDepthThresh, 0.99f, 1.0f, "%.5f");
    {
        bool sph = p.enableSphereClip > 0.5f;
        bool dep = p.enableDepthClip > 0.5f;
        if (ImGui::Checkbox("EnableSphereClip", &sph)) p.enableSphereClip = sph ? 1.0f : 0.0f;
        if (ImGui::Checkbox("EnableDepthClip", &dep)) p.enableDepthClip = dep ? 1.0f : 0.0f;
    }
    ImGui::SliderFloat("SphereClipMaxT (km)", &p.sphereClipMaxT, 1.0f, 1.0e6f, "%.1f", ImGuiSliderFlags_Logarithmic);
    ImGui::TextDisabled("ForceSky 棕带消失→地面路径；VizPath 看红/蓝边界；");
    ImGui::TextDisabled("VizInsideWhite: 钻地天空=白 地面=品红；球外正常。");
    ImGui::TextDisabled("NoSphereClip 棕带没了→解析球；NoDepthClip→深度。");

    ImGui::Separator();
    ImGui::Text("掠射天空 / 棕带诊断");
    ImGui::TextDisabled("地球默认 mie=5e-3 hMie=1 gMie=0.8；调倍率即时影响 raymarch。");
    ImGui::TextDisabled("sunT LUT 仍用烘焙系数；要一致请点下方强制重烤。");
    ImGui::SliderFloat("RayleighScale", &p.rayleighScale, 0.0f, 3.0f, "%.2f");
    ImGui::SliderFloat("MieScale", &p.mieScale, 0.0f, 3.0f, "%.2f");
    ImGui::SliderFloat("OzoneScale", &p.ozoneScale, 0.0f, 3.0f, "%.2f");
    ImGui::SliderFloat("HRayleighScale", &p.hRayleighScale, 0.25f, 4.0f, "%.2f");
    ImGui::SliderFloat("HMieScale", &p.hMieScale, 0.25f, 4.0f, "%.2f");
    {
        bool useG = p.useCustomGMie > 0.5f;
        if (ImGui::Checkbox("Override gMie", &useG)) p.useCustomGMie = useG ? 1.0f : 0.0f;
        ImGui::BeginDisabled(!useG);
        ImGui::SliderFloat("gMie", &p.gMie, 0.0f, 0.99f, "%.2f");
        ImGui::EndDisabled();
    }
    ImGui::SliderInt("MarchSteps", &p.marchSteps, 8, 128);
    ImGui::SliderFloat("LowMuScatterMul", &p.lowMuScatterMul, 0.0f, 2.0f, "%.2f");
    ImGui::SliderFloat("LowMuSoft", &p.lowMuSoft, 0.01f, 0.5f, "%.3f");
    ImGui::TextDisabled("LowMuSoft 同时控制掠射 densFloor 仰角宽度（地平→2km地板）。");
    ImGui::TextDisabled("LowMu Mul=0 可验证低仰角散射；球内已从表面出射再积。");
    {
        bool noAces = p.disableAces > 0.5f;
        if (ImGui::Checkbox("DisableACES (看线性散射)", &noAces)) p.disableAces = noAces ? 1.0f : 0.0f;
    }

    ImGui::Separator();
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
