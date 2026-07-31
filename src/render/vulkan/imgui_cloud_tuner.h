#pragma once
// ==========================================================================
// imgui_cloud_tuner.h — Dear ImGui Flower 体积云参数调试面板
// 用法: FlowerCloudTunerImGui(&open, params, &weatherPreview);
// ==========================================================================
#include "vk_cloud_system.h"  // FlowerCloudTuneParams
#include "vk_shadow_cascade.h" // VkCascadeShadowSystem
#include "imgui.h"
#include "backends/imgui_impl_vulkan.h"
#include <cmath>
#include <cstdint>
#include <cstdio>

// Weather / Curl ImGui 预览（VkDescriptorSet == ImTextureID，经 ImGui_ImplVulkan_AddTexture 注册）
struct WeatherMapImGuiPreview {
    VkDescriptorSet sets[5] = {}; // 0=RGB 合成, 1=R coverage, 2=G type, 3=B humidity, 4=A land
    VkDescriptorSet curlSet = VK_NULL_HANDLE;
    bool ready = false;
    bool curlReady = false;

    bool registerTextures(VkImageView composite,
                          VkImageView r, VkImageView g, VkImageView b, VkImageView a) {
        unregisterWeather();
        VkImageView views[5] = { composite, r, g, b, a };
        for (int i = 0; i < 5; i++) {
            if (views[i] == VK_NULL_HANDLE) { unregisterWeather(); return false; }
            sets[i] = ImGui_ImplVulkan_AddTexture(views[i], VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
            if (sets[i] == VK_NULL_HANDLE) { unregisterWeather(); return false; }
        }
        ready = true;
        return true;
    }

    bool registerCurl(VkImageView curl) {
        unregisterCurl();
        if (curl == VK_NULL_HANDLE) return false;
        curlSet = ImGui_ImplVulkan_AddTexture(curl, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
        curlReady = (curlSet != VK_NULL_HANDLE);
        return curlReady;
    }

    void unregisterWeather() {
        for (int i = 0; i < 5; i++) {
            if (sets[i] != VK_NULL_HANDLE) {
                ImGui_ImplVulkan_RemoveTexture(sets[i]);
                sets[i] = VK_NULL_HANDLE;
            }
        }
        ready = false;
    }

    void unregisterCurl() {
        if (curlSet != VK_NULL_HANDLE) {
            ImGui_ImplVulkan_RemoveTexture(curlSet);
            curlSet = VK_NULL_HANDLE;
        }
        curlReady = false;
    }

    void unregister() {
        unregisterWeather();
        unregisterCurl();
    }

    ImTextureID texId(int channel) const {
        if (!ready || channel < 0 || channel > 4 || sets[channel] == VK_NULL_HANDLE)
            return ImTextureID_Invalid;
        return (ImTextureID)(uintptr_t)sets[channel];
    }

    ImTextureID curlTexId() const {
        if (!curlReady || curlSet == VK_NULL_HANDLE) return ImTextureID_Invalid;
        return (ImTextureID)(uintptr_t)curlSet;
    }
};

// ==========================================================================
// Flower 云管线（cloud_common.glsl 系，VkCloudSystem）调参面板。
// 用法: FlowerCloudTunerImGui(&open, cloudSystem.tuneParams, &weatherPreview);
// ==========================================================================
inline void FlowerCloudTunerImGui(bool* p_open, FlowerCloudTuneParams& p,
                                  WeatherMapImGuiPreview* weatherPreview = nullptr,
                                  VkCascadeShadowSystem* shadow = nullptr) {
    if (ImGui::IsKeyPressed(ImGuiKey_F3, false)) *p_open = !*p_open;
    if (!*p_open) return;

    ImGui::SetNextWindowSize(ImVec2(400, 720), ImGuiCond_FirstUseEver);
    if (!ImGui::Begin("体积云调参 (Flower Phase1)", p_open, ImGuiWindowFlags_None)) {
        ImGui::End();
        return;
    }

    ImGui::Text("管线");
    ImGui::Checkbox("flower体积云启用总开关", &p.flowerCloudEnabled);
    ImGui::TextDisabled("如果关闭，则不会渲染任何云");
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
    ImGui::SliderFloat("EmptyJitter ±", &p.emptyStepJitter, 0.0f, 0.5f, "%.2f");
    ImGui::TextDisabled("仅 density==0；中点+终点探测通过才跳");
    ImGui::TextDisabled("EmptyJitter: 蓝噪声缩放空步 (1±值)×EmptyStep");

    ImGui::Separator();
    ImGui::Text("云缘缩步（第 3 层）");
    ImGui::Checkbox("EdgeRefine 启用", &p.edgeRefineEnabled);
    ImGui::SliderFloat("EdgeScaleMin", &p.edgeScaleMin, 0.05f, 1.0f, "%.2f");
    ImGui::SliderFloat("EdgeGradLow", &p.edgeGradLow, 0.0001f, 0.05f, "%.4f", ImGuiSliderFlags_Logarithmic);
    ImGui::SliderFloat("EdgeGradHigh", &p.edgeGradHigh, 0.001f, 0.2f, "%.3f", ImGuiSliderFlags_Logarithmic);
    ImGui::TextDisabled("步长=stepT×mix(1,EdgeScaleMin,edgeSharpness)");
    ImGui::TextDisabled("edgeSharpness 来自 |Δ(σ·Δs)|/Δs");

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
    ImGui::Text("噪声频带阶梯（scale 单位 1/km；周期 = 1/scale ÷ 纹理频率）");
    ImGui::TextDisabled("形状 Basic/Meso/Fine 共用 128³ 噪声，只换 scale；侵蚀用 32³ Worley");
    if (ImGui::Button("按 Nyquist 重建阶梯")) rebuildCloudNoiseLadder(p);
    ImGui::SameLine();
    ImGui::TextDisabled("锚 Basic：scale ×%.0f/档，幅度 ×%.2f（侵蚀 ×%.2f）",
        CloudNoiseLadder::kBasicBandSpan, CloudNoiseLadder::kShapeAmpFalloff,
        CloudNoiseLadder::kErosionAmpFalloff);

    ImGui::SliderFloat("BasicNoiseScale",  &p.basicNoiseScale,  0.0001f,   4.0f, "%.4f", ImGuiSliderFlags_Logarithmic);
    ImGui::SliderFloat("MesoNoiseScale",   &p.mesoNoiseScale,   0.0001f,  16.0f, "%.4f", ImGuiSliderFlags_Logarithmic);
    ImGui::SliderFloat("MesoAmp",          &p.mesoAmplitude,    0.0f, 1.0f, "%.3f");
    ImGui::SliderFloat("FineNoiseScale",   &p.fineNoiseScale,   0.0001f,  64.0f, "%.4f", ImGuiSliderFlags_Logarithmic);
    ImGui::SliderFloat("FineAmp",          &p.fineAmplitude,    0.0f, 1.0f, "%.3f");
    ImGui::SliderFloat("DetailNoiseScale", &p.detailNoiseScale, 0.0001f, 256.0f, "%.3f", ImGuiSliderFlags_Logarithmic);
    ImGui::SliderFloat("DetailAmp",        &p.detailAmplitude,  0.0f, 1.0f, "%.3f");
    ImGui::TextDisabled("幅度=0 的档在 shader 里连采样一起跳过（频带 LOD 的关闭档）");

    // 频带读数：scale 换算成"这一档实际覆盖的世界周期区间"，并检查三个边界——
    // 与上一档的接缝是否连续、是否细过 march 步长（采样不到，只会闪）、是否细过
    // float 在行星尺度上的精度台阶。
    {
        const float stepBase = p.maxTracingDistanceKm / (float)(p.marchingStepNum > 0 ? p.marchingStepNum : 1);
        const float marchStepKm = fminf(fmaxf(stepBase, p.stepTMinKm), p.stepTMaxKm);
        const float precisionKm = CloudNoiseLadder::samplePrecisionKm();
        const ImVec4 warnCol(0.95f, 0.75f, 0.2f, 1.0f);

        auto fmtKm = [](char* buf, size_t n, float km) {
            if (km <= 0.f)            snprintf(buf, n, "n/a");
            else if (km < 1.0f)       snprintf(buf, n, "%.1f m", km * 1000.f);
            else                      snprintf(buf, n, "%.2f km", km);
        };

        float prevFinestKm = 0.f;
        auto bandRow = [&](const char* name, float scale, float amp, int texDim) {
            const float coarseKm = CloudNoiseLadder::coarsestPeriodKm(scale);
            const float finestKm = CloudNoiseLadder::finestPeriodKm(scale, texDim);
            char c[32], f[32];
            fmtKm(c, sizeof(c), coarseKm);
            fmtKm(f, sizeof(f), finestKm);
            ImGui::TextDisabled("%-6s %s → %s  (amp %.3f, %d³)", name, c, f, amp, texDim);

            if (amp <= 0.f || finestKm <= 0.f) { prevFinestKm = finestKm; return; }
            if (prevFinestKm > 0.f) {
                // 接缝：本档最粗特征应等于上一档最细特征，比值偏离即空洞（>1）或重复（<1）
                const float seam = coarseKm > 0.f ? prevFinestKm / coarseKm : 0.f;
                if (seam > 1.2f)       ImGui::TextColored(warnCol, "  ↑ 频带空洞 ×%.1f", seam);
                else if (seam < 0.83f) ImGui::TextColored(warnCol, "  ↑ 频带重复 ×%.1f", 1.f / seam);
            }
            if (finestKm < marchStepKm * 2.f)
                ImGui::TextColored(warnCol, "  ↑ 细过 march 步长×2 (%.0f m)：会闪不会更细", marchStepKm * 2000.f);
            if (finestKm < precisionKm * 4.f)
                ImGui::TextColored(warnCol, "  ↑ 接近 float 精度台阶 (%.1f m)", precisionKm * 1000.f);
            prevFinestKm = finestKm;
        };

        bandRow("Basic",  p.basicNoiseScale,  1.0f,               CloudNoiseLadder::kBasicTexDim);
        bandRow("Meso",   p.mesoNoiseScale,   p.mesoAmplitude,    CloudNoiseLadder::kBasicTexDim);
        bandRow("Fine",   p.fineNoiseScale,   p.fineAmplitude,    CloudNoiseLadder::kBasicTexDim);
        bandRow("Detail", p.detailNoiseScale, p.detailAmplitude,  CloudNoiseLadder::kDetailTexDim);
    }

    ImGui::Separator();
    ImGui::Text("噪声 LOD（屏幕足迹）");
    ImGui::TextDisabled("一档特征不足 nMin 像素就淡出；Basic 恒开，只影响 Meso/Fine/Detail");
    ImGui::Checkbox("NoiseLodEnabled", &p.noiseLodEnabled);
    ImGui::BeginDisabled(!p.noiseLodEnabled);
    ImGui::SliderFloat("NoiseLodNMin", &p.noiseLodNMin, 0.5f, 8.0f, "%.2f px");
    ImGui::SliderFloat("NoiseLodShadowDistScale", &p.noiseLodShadowDistScale, 1.0f, 8.0f, "%.2f×");
    ImGui::TextDisabled("阴影 march 按 %.1f× 距离判定，比主视线更早丢细档", p.noiseLodShadowDistScale);
    {
        // 各档大致从多远开始淡出。面板拿不到当帧相机参数，用固定参考视口给量级
        // （真实 fovy/分辨率不同只是整体等比缩放这些距离）。
        auto fadeRow = [&](const char* name, float scale, float amp, int texDim) {
            if (amp <= 0.f) return;
            const float finestKm = CloudNoiseLadder::finestPeriodKm(scale, texDim);
            const float dOff = CloudNoiseLadder::lodFadeStartKm(finestKm, p.noiseLodNMin);
            if (dOff <= 0.f) return;
            ImGui::TextDisabled("%-6s 约 %.1f km 起淡出，%.1f km 全关",
                name, dOff, dOff / CloudNoiseLadder::kLodFadeSpan);
        };
        fadeRow("Meso",   p.mesoNoiseScale,   p.mesoAmplitude,   CloudNoiseLadder::kBasicTexDim);
        fadeRow("Fine",   p.fineNoiseScale,   p.fineAmplitude,   CloudNoiseLadder::kBasicTexDim);
        fadeRow("Detail", p.detailNoiseScale, p.detailAmplitude, CloudNoiseLadder::kDetailTexDim);
        ImGui::TextDisabled("参考视口 %.0fp / %.0f°；实际按当帧 fovy 与渲染高度等比缩放",
            CloudNoiseLadder::kLodPreviewRenderHeight,
            CloudNoiseLadder::kLodPreviewFovYRad * 57.2957795f);
    }
    ImGui::EndDisabled();

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

    if (weatherPreview && weatherPreview->ready) {
        static int weatherVisChannel = 0;
        const char* channelItems[] = {
            "RGB 合成 (R=cov G=type B=hum)",
            "Coverage (R)",
            "CloudType (G)",
            "Humidity (B)",
            "LandMask (A)",
        };
        ImGui::Combo("WeatherMap 通道", &weatherVisChannel, channelItems, IM_ARRAYSIZE(channelItems));
        ImTextureID tid = weatherPreview->texId(weatherVisChannel);
        if (tid != ImTextureID_Invalid) {
            const float previewW = ImGui::GetContentRegionAvail().x;
            const float previewH = previewW * 0.5f; // 2048×1024 equirectangular
            ImGui::Image(ImTextureRef(tid), ImVec2(previewW, previewH));
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("经纬度 equirectangular · 左=-180° 右=+180° 下=南极 上=北极");
        }
        ImGui::TextDisabled("R=coverage  G=type  B=humidity  A=land（bake 自 vk_scene）");
    } else {
        ImGui::TextDisabled("WeatherMap 预览未注册");
    }

    ImGui::Separator();
    ImGui::Text("Curl / localCoverage（假 curl 纹理）");
    ImGui::SliderFloat("CurlUVScale", &p.curlUVScale, 0.00001f, 0.01f, "%.5f", ImGuiSliderFlags_Logarithmic);
    ImGui::SliderFloat("CurlStrength", &p.curlStrength, 0.0f, 1.0f, "%.3f");
    ImGui::SliderFloat("CurlContrast", &p.curlContrast, 0.5f, 8.0f, "%.2f");
    ImGui::TextDisabled("Strength=0 关闭碎点；轨道可试 UV≈0.0002 Strength≈0.1");
    ImGui::TextDisabled("旧硬编码: UV=0.001 Strength=0.2 Contrast=3");
    if (weatherPreview && weatherPreview->curlReady) {
        ImTextureID curlTid = weatherPreview->curlTexId();
        if (curlTid != ImTextureID_Invalid) {
            const float previewW = ImGui::GetContentRegionAvail().x;
            ImGui::Image(ImTextureRef(curlTid), ImVec2(previewW, previewW)); // 256² tile
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("bake 的 256² R8（Worley+Perlin），非真 curl 矢量场");
        }
    }

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
    ImGui::TextDisabled("全局曝光 L_sun=color×sunIntensity×本值；直射/环境/AP 同源");
    ImGui::SliderFloat("SunLightScale", &p.sunLightScale, 0.0f, 4.0f, "%.2f");
    ImGui::SliderFloat("MinAlt (km)",   &p.minAltKm, 0.5f, 8.0f, "%.2f");
    ImGui::SliderFloat("MaxAlt (km)",   &p.maxAltKm, 8.0f, 30.0f, "%.2f");
    ImGui::SliderFloat("CloudAlbedo",   &p.cloudAlbedo, 0.0f, 4.0f, "%.2f");
    ImGui::SliderFloat("FogFade",       &p.fogFade, 0.0f, 2.0f, "%.2f");
    ImGui::TextDisabled("云底地面反弹倍率（albedo×Ambient）；云顶 nearGround→0，不再灌 HDR");
    ImGui::SliderFloat("GroundNoiseScale", &p.groundNoiseScale, 0.0f, 4.0f, "%.2f");
    ImGui::TextDisabled("越大 → 地面反弹影响的高度带越厚（saturate(Scale−h)）");
    ImGui::Checkbox("GroundContribution", &p.enableGroundContribution);
    ImGui::TextDisabled("开=环境天光 Ambient + 云底 groundLit；关=只剩太阳直射");

    ImGui::Separator();
    ImGui::Text("空气透视 (Froxel)");
    ImGui::Checkbox("AirPerspective Enabled", &p.airPerspectiveEnabled);
    ImGui::SliderFloat("AirPerspectiveSunScale", &p.airPerspectiveSunScale, 0.001f, 30.0f, "%.3f", ImGuiSliderFlags_Logarithmic);
    ImGui::TextDisabled("相对 L_sun 的雾色倍率（默认 1；LUT 为单位太阳）");
    ImGui::SliderFloat("AirPerspectiveScale", &p.airPerspectiveScale, 0.0f, 1.0f, "%.3f", ImGuiSliderFlags_Logarithmic);
    ImGui::TextDisabled("混合权重 α'=α×Scale；L'=L×(1-α')+fog×α'（默认 0.01）");

    ImGui::Separator();
    ImGui::Text("环境天光 (μs LUT，与相机无关)");
    ImGui::SliderFloat("AmbientExposure", &p.ambientExposure, 0.1f, 50.0f, "%.2f", ImGuiSliderFlags_Logarithmic);
    ImGui::SliderFloat("AmbientAnalyticBlend", &p.ambientAnalyticBlend, 0.0f, 1.0f, "%.2f");
    ImGui::TextDisabled("单位太阳半球 LUT × L_sun × Exposure；夜半球由 μs 变暗");
    ImGui::TextDisabled("AnalyticBlend 混入 analyticSkyAmbient（同样乘 L_sun）");

    ImGui::Separator();
    ImGui::Text("God Ray（近距 400m 体积光柱）");
    ImGui::Checkbox("God Ray 启用", &p.godRayEnabled);
    ImGui::SliderFloat("God Ray 强度", &p.godRayScale, 0.0f, 8.0f, "%.2f");
    ImGui::TextDisabled("需要下面的级联阴影也开启，否则光柱不受地形遮挡");

    if (shadow) {
        ImGui::Separator();
        ImGui::Text("SDSM 级联阴影（地形，供 God Ray 用）");
        ImGui::Checkbox("级联阴影启用", &shadow->enabled);
        ImGui::Text("级联数 %d × %dpx（初始化时固定，改需重启）", shadow->cascadeCount, shadow->percascadeDimXY);
        ImGui::SliderFloat("SplitLambda", &shadow->cascadeSplitLambda, 0.0f, 1.0f, "%.2f");
        ImGui::TextDisabled("0=纯均匀拆分 1=纯对数拆分（近处更细）");
        ImGui::SliderFloat("近裁截 (km)", &shadow->nearDistanceKm, 0.001f, 1.0f, "%.3f", ImGuiSliderFlags_Logarithmic);
        ImGui::SliderFloat("远裁截 (km)", &shadow->maxDrawDepthDistanceKm, 0.5f, 50.0f, "%.2f", ImGuiSliderFlags_Logarithmic);
        ImGui::SliderFloat("深度余量 (km)", &shadow->depthMarginKm, 1.0f, 200.0f, "%.1f", ImGuiSliderFlags_Logarithmic);
        ImGui::TextDisabled("光源沿太阳方向后退的额外距离，兜住相机附近地形起伏");
        ImGui::SliderFloat("BiasConst", &shadow->shadowBiasConst, 0.0f, 0.02f, "%.4f");
        ImGui::SliderFloat("BiasSlope", &shadow->shadowBiasSlope, 0.0f, 0.02f, "%.4f");
        ImGui::SliderFloat("BorderAdopt", &shadow->cascadeBorderAdopt, 0.0f, 0.2f, "%.3f");
        ImGui::SliderFloat("EdgeLerpThreshold", &shadow->cascadeEdgeLerpThreshold, 0.5f, 1.0f, "%.3f");
    } else {
        ImGui::TextDisabled("级联阴影系统未接入（shadow==nullptr）");
    }

    ImGui::Separator();
    ImGui::Text("调试可视化（排查黑云用）");
    const char* dbgItems[] = {
        "Off（正常渲染）",
        "1: 相机位置实时大气透射率",
        "2: 天空背景色（采样自 atmo pass）",
        "3: 云点环境天光 (sampleCloudAmbient)",
        "4: 太阳色×强度（不经过任何大气衰减）",
        "5: 不透明度 1-T（全黑=没找到云密度）",
        "6: 散射光原始值（未做后处理）",
        "7: tMin/tMax 中点原始密度（绿=正常 红=NaN 黑=求交区间无效）",
        "8: 强制无云影 + raw L（对比6：厚区变亮=自阴影）",
        "9: 强制大气透射=1 + raw L（对比6：暖/暗来自T_atm）",
        "10: 8+9 同时 + raw L",
        "11: 平均 sunVis 灰阶（厚暗区近黑=直射被云影杀死）",
        "12: L_in=sunColor, powder/phase=1 + raw L（仍暗=Beer/积分）",
        "13: 平均 powderEffect 灰阶（云顶若<<1=粉末压暗）",
        "14: 平均云内高度（红=顶 绿=底）",
        "15: 物理 Beer（关 soft-T）+ raw L",
    };
    ImGui::Combo("DebugMode", &p.debugMode, dbgItems, IM_ARRAYSIZE(dbgItems));
    ImGui::TextDisabled("8/9仍暗→看11是否本就亮，再比6/12/13；非0时a=0替换背景");

    ImGui::Separator();
    ImGui::TextDisabled("F3: 开关面板");

    ImGui::End();
}
