#pragma once
// ==========================================================================
// imgui_cloud_tuner.h — Dear ImGui 体积云参数调试面板
// 用法: CloudTunerImGui(&open, params);
// ==========================================================================
#include "../cloud_system.h"  // CloudTuneParams
#include "vk_cloud_system.h"  // FlowerCloudTuneParams
#include "imgui.h"
#include <cmath>

inline void CloudTunerImGui(bool* p_open, CloudTuneParams& p) {
    // F2 toggles the window
    if (ImGui::IsKeyPressed(ImGuiKey_F2, false)) *p_open = !*p_open;

    if (!*p_open) return;

    ImGui::SetNextWindowSize(ImVec2(380, 480), ImGuiCond_FirstUseEver);
    if (!ImGui::Begin("体积云调参", p_open, ImGuiWindowFlags_NoResize)) {
        ImGui::End();
        return;
    }

    ImGui::Text("密度与消光");
    ImGui::SliderFloat("Density",      &p.density,    0.10f, 4.00f, "%.2f");
    ImGui::SliderFloat("Extinction",   &p.extinction, 0.02f, 0.40f, "%.3f");

    ImGui::Separator();
    ImGui::Text("覆盖率");
    ImGui::SliderFloat("CovLo",        &p.covLo,      0.10f, 0.70f, "%.3f");
    ImGui::SliderFloat("CovHi",        &p.covHi,      0.30f, 0.90f, "%.3f");

    ImGui::Separator();
    ImGui::Text("逾渗阈值（碎块程度）");
    ImGui::SliderFloat("ThreshLo",     &p.threshLo,   0.05f, 0.55f, "%.3f");
    ImGui::SliderFloat("ThreshHi",     &p.threshHi,   0.40f, 0.92f, "%.3f");

    ImGui::Separator();
    ImGui::Text("形状");
    ImGui::SliderFloat("Erosion",      &p.erosion,    0.00f, 0.50f, "%.3f");
    ImGui::SliderFloat("MinAlt (km)",  &p.minAlt,     0.50f, 6.00f, "%.2f");
    ImGui::SliderFloat("MaxAlt (km)",  &p.maxAlt,     8.00f, 22.00f, "%.2f");
    ImGui::SliderFloat("LocalRadius (km)", &p.localRadiusKm, 0.f, 500.f, "%.0f");
    ImGui::TextDisabled("LocalRadius: Flower-style tangent cloudMap near camera (0=off)");

    ImGui::Separator();
    ImGui::Text("调试");
    const char* debugItems[] = {
        "Off",
        "Force Density",
        "Density",
        "Coverage/Profile/Shape",
        "Alpha/Trans/Depth",
        "Sun/Sky/Ground",
        "Step Optical Depth"
    };
    ImGui::Combo("DebugMode", &p.debugMode, debugItems, IM_ARRAYSIZE(debugItems));

    ImGui::Separator();
    ImGui::TextDisabled("F2: 开关面板  |  Shift+P: 开关云");

    ImGui::End();
}

// ==========================================================================
// Flower 云管线（cloud_common.glsl 系，VkCloudSystem）调参面板。
// 用法: FlowerCloudTunerImGui(&open, cloudSystem.tuneParams);
// 和上面 CloudTunerImGui() 是两套独立参数——那个调的是 legacy cloud.frag，
// 这个调的是新的 1/4 分辨率 compute 云管线（cloudSystem.enabled=true 时生效）。
// ==========================================================================
inline void FlowerCloudTunerImGui(bool* p_open, FlowerCloudTuneParams& p) {
    if (ImGui::IsKeyPressed(ImGuiKey_F3, false)) *p_open = !*p_open;
    if (!*p_open) return;

    ImGui::SetNextWindowSize(ImVec2(400, 720), ImGuiCond_FirstUseEver);
    if (!ImGui::Begin("体积云调参 (Flower Phase1)", p_open, ImGuiWindowFlags_None)) {
        ImGui::End();
        return;
    }

    ImGui::Text("管线");
    ImGui::Checkbox("1/4 分辨率 + Bayer", &p.useQuarterRes);
    ImGui::TextDisabled("关=全分辨率 raymarch（×16 成本，减盐胡椒）");
    ImGui::SliderInt("MarchSteps (baseline)", &p.marchingStepNum, 8, 2048);
    ImGui::SliderInt("StepHardMax", &p.marchStepHardMax, 64, 2048);
    ImGui::SliderFloat("StepMin (km)", &p.stepTMinKm, 0.001f, 2.0f, "%.4f", ImGuiSliderFlags_Logarithmic);
    ImGui::SliderFloat("StepMax (km)", &p.stepTMaxKm, 0.01f, 20.0f, "%.3f", ImGuiSliderFlags_Logarithmic);
    ImGui::SliderFloat("MaxTraceDist (km)", &p.maxTracingDistanceKm, 10.f, 2000.f, "%.0f");

    ImGui::Separator();
    ImGui::Text("壳内真空跳步（第 2 层）");
    ImGui::Checkbox("EmptySkip 启用", &p.emptySkipEnabled);
    ImGui::SliderFloat("EmptyStep (km)", &p.emptyStepKm, 0.05f, 10.0f, "%.2f", ImGuiSliderFlags_Logarithmic);
    ImGui::TextDisabled("仅 cloudMap==0 时大步；薄云细步且全光照");
    ImGui::TextDisabled("EmptyStep 至少为 StepMin");

    if (p.marchingStepNum > 0) {
        const float segEst = p.maxTracingDistanceKm;
        const float stepBase = segEst / float(p.marchingStepNum);
        const float stepT = (stepBase < p.stepTMinKm) ? p.stepTMinKm
                          : (stepBase > p.stepTMaxKm) ? p.stepTMaxKm : stepBase;
        const int stepEff = (int)ceilf(segEst / stepT);
        const int stepClamped = (stepEff > p.marchStepHardMax) ? p.marchStepHardMax : stepEff;
        ImGui::TextDisabled("估算(MaxTraceDist): seg≈%.0fkm stepT≈%.3fkm (%.0fm)",
            segEst, stepT, stepT * 1000.f);
        ImGui::TextDisabled("估算实际步数≈%d (baseline=%d, hardMax=%d)",
            stepClamped, p.marchingStepNum, p.marchStepHardMax);
    }
    ImGui::SliderFloat("TraceStartMax (km)", &p.tracingStartMaxDistanceKm, 1e3f, 1e9f, "%.0e", ImGuiSliderFlags_Logarithmic);
    ImGui::TextDisabled("视线起点距相机超过此值则跳过云 march");

    ImGui::Separator();
    ImGui::Text("覆盖率与密度");
    ImGui::SliderFloat("Coverage",   &p.coverage, 0.0f, 1.0f, "%.3f");
    ImGui::SliderFloat("Density",    &p.density,  0.0f, 4.0f, "%.2f");

    ImGui::Separator();
    ImGui::Text("噪声频率（周期 ≈ 公里数倒数；越大纹理越密）");
    ImGui::SliderFloat("BasicNoiseScale",  &p.basicNoiseScale,  0.0001f, 4.0f, "%.4f", ImGuiSliderFlags_Logarithmic);
    ImGui::SliderFloat("DetailNoiseScale", &p.detailNoiseScale, 0.0001f,  8.0f, "%.3f", ImGuiSliderFlags_Logarithmic);
    ImGui::TextDisabled("近地 flower 参考: Basic≈0.4  Detail≈1.2");
    ImGui::TextDisabled("行星轨道参考: Basic≈0.003  Detail≈0.02");

    ImGui::Separator();
    ImGui::Text("体积阴影步进 (volumetricShadow)");
    ImGui::SliderInt("LightStepNum", &p.lightStepNum, 1, 16);
    ImGui::SliderFloat("LightBasicStep (km)", &p.lightBasicStepKm, 0.001f, 0.5f, "%.4f", ImGuiSliderFlags_Logarithmic);
    ImGui::SliderFloat("LightStepMul", &p.lightStepMul, 1.0f, 4.0f, "%.2f");
    ImGui::TextDisabled("沿太阳方向自阴影：步长 = basicStep × mul^i");

    ImGui::Separator();
    ImGui::Text("天气 / 风");
    ImGui::SliderFloat("WeatherUVScale", &p.weatherUVScale, 0.1f, 8.0f, "%.2f");
    ImGui::SliderFloat3("WindDir", &p.windDirX, -1.f, 1.f, "%.2f");
    ImGui::SliderFloat("Speed", &p.speed, 0.0f, 2.0f, "%.3f");

    ImGui::Separator();
    ImGui::Text("多重散射");
    ImGui::SliderFloat("MsExtinction", &p.multiScatterExtinction, 0.0f, 1.0f, "%.3f");
    ImGui::SliderFloat("MsScatter",    &p.multiScatterScatter,    0.0f, 1.0f, "%.3f");

    ImGui::Separator();
    ImGui::Text("相位 / 银边");
    ImGui::SliderFloat("PhaseForward",  &p.phaseForward,   0.0f, 0.99f, "%.3f");
    ImGui::SliderFloat("PhaseBackward", &p.phaseBackward, -0.99f, 0.0f, "%.3f");
    ImGui::SliderFloat("PhaseMix",      &p.phaseMixFactor, 0.0f, 1.0f, "%.3f");
    ImGui::SliderFloat("PowderScale",   &p.powderScale,    0.0f, 4.0f, "%.2f");
    ImGui::SliderFloat("PowderPow",     &p.powderPow,      0.0f, 4.0f, "%.2f");

    ImGui::Separator();
    ImGui::Text("光照与高度");
    ImGui::SliderFloat("SunIntensityMul", &p.sunIntensityMul, 0.1f, 200000.0f, "%.1f", ImGuiSliderFlags_Logarithmic);
    ImGui::TextDisabled("黑云先调这个：Bruneton 大气积分要的太阳辐照度量级和引擎其它地方不一样");
    ImGui::SliderFloat("SunLightScale", &p.sunLightScale, 0.0f, 4.0f, "%.2f");
    ImGui::SliderFloat("MinAlt (km)",   &p.minAltKm, 0.5f, 8.0f, "%.2f");
    ImGui::SliderFloat("MaxAlt (km)",   &p.maxAltKm, 8.0f, 30.0f, "%.2f");
    ImGui::SliderFloat("CloudAlbedo",   &p.cloudAlbedo, 0.0f, 4.0f, "%.2f");
    ImGui::SliderFloat("FogFade",       &p.fogFade, 0.0f, 2.0f, "%.2f");
    ImGui::SliderFloat("GroundNoiseScale", &p.groundNoiseScale, 0.0f, 4.0f, "%.2f");
    ImGui::Checkbox("GroundContribution", &p.enableGroundContribution);

    ImGui::Separator();
    ImGui::Text("调试可视化（排查黑云用）");
    const char* dbgItems[] = {
        "Off（正常渲染）",
        "1: 相机位置实时大气透射率",
        "2: 天空背景色（采样自 atmo pass）",
        "3: 天空环境色近似",
        "4: 太阳色×强度（不经过任何大气衰减）",
        "5: 不透明度 1-T（全黑=没找到云密度）",
        "6: 散射光原始值（未做后处理）",
        "7: tMin/tMax 中点原始密度（绿=正常 红=NaN 黑=求交区间无效）",
    };
    ImGui::Combo("DebugMode", &p.debugMode, dbgItems, IM_ARRAYSIZE(dbgItems));
    ImGui::TextDisabled("非 0 时直接输出对应中间量，全黑=那一级 LUT/参数有问题");

    ImGui::Separator();
    ImGui::TextDisabled("F3: 开关面板");

    ImGui::End();
}
