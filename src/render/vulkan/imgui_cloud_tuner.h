#pragma once
// ==========================================================================
// imgui_cloud_tuner.h — Dear ImGui Flower 体积云参数调试面板
// 用法: FlowerCloudTunerImGui(&open, params, &weatherPreview);
// ==========================================================================
#include "vk_cloud_system.h"  // FlowerCloudTuneParams
#include "vk_shadow_cascade.h" // VkCascadeShadowSystem
#include "weather_map_bake.h"  // ProfileLutBake：廓形曲线预览与烘焙参数同源
#include "imgui.h"
#include "backends/imgui_impl_vulkan.h"
#include <cmath>
#include <cstdint>
#include <cstdio>

// Weather / Curl / 双层云纹理 ImGui 预览（VkDescriptorSet == ImTextureID，经 ImGui_ImplVulkan_AddTexture 注册）
struct WeatherMapImGuiPreview {
    VkDescriptorSet sets[5] = {}; // 0=RGB 合成, 1=R coverage, 2=G type, 3=B humidity, 4=A land
    VkDescriptorSet curlSet = VK_NULL_HANDLE;
    // 双层云：Profile LUT（256×64）+ 高空云 Hi weather map（2048×1024）
    VkDescriptorSet profileLutSet = VK_NULL_HANDLE;
    VkDescriptorSet hiWeatherSet = VK_NULL_HANDLE;
    bool ready = false;
    bool curlReady = false;
    bool profileLutReady = false;
    bool hiWeatherReady = false;

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

    // 双层云单图预览（Profile LUT / Hi weather map）：整图一个 descriptor。
    bool registerCloudTex(VkImageView view, VkDescriptorSet& set, bool& flag) {
        unregisterCloudTex(set, flag);
        if (view == VK_NULL_HANDLE) return false;
        set = ImGui_ImplVulkan_AddTexture(view, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
        flag = (set != VK_NULL_HANDLE);
        return flag;
    }
    bool registerProfileLut(VkImageView v) { return registerCloudTex(v, profileLutSet, profileLutReady); }
    bool registerHiWeather(VkImageView v)  { return registerCloudTex(v, hiWeatherSet, hiWeatherReady); }

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

    void unregisterCloudTex(VkDescriptorSet& set, bool& flag) {
        if (set != VK_NULL_HANDLE) {
            ImGui_ImplVulkan_RemoveTexture(set);
            set = VK_NULL_HANDLE;
        }
        flag = false;
    }

    void unregister() {
        unregisterWeather();
        unregisterCurl();
        unregisterCloudTex(profileLutSet, profileLutReady);
        unregisterCloudTex(hiWeatherSet, hiWeatherReady);
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

    ImTextureID profileLutTexId() const {
        if (!profileLutReady || profileLutSet == VK_NULL_HANDLE) return ImTextureID_Invalid;
        return (ImTextureID)(uintptr_t)profileLutSet;
    }

    ImTextureID hiWeatherTexId() const {
        if (!hiWeatherReady || hiWeatherSet == VK_NULL_HANDLE) return ImTextureID_Invalid;
        return (ImTextureID)(uintptr_t)hiWeatherSet;
    }
};

// 预览图小工具：等比缩放到一行可用宽度，超宽拉下滚动条也友好。
static void PreviewImageAspect(ImTextureID tid, float aspect) {
    if (tid == ImTextureID_Invalid) return;
    const float w = ImGui::GetContentRegionAvail().x;
    ImGui::Image(ImTextureRef(tid), ImVec2(w, w / aspect));
}

// Profile LUT 三条云类型廓形曲线预览（与 ProfileLutBake::profile 同参数烘焙，
// 所见即所采）。Cu=原 gradientShape 高度函数（积云预设），Tcu/Cb 为同族变体。
static void ProfileLutCurvePlot() {
    const int N = 64;
    float cu[N], tcu[N], cb[N];
    for (int i = 0; i < N; i++) {
        const float h = (float)i / (float)(N - 1);
        cu[i]  = ProfileLutBake::profile(h, 0.10f, 0.80f, 0.10f, 0.20f); // 积云（旧曲线）
        tcu[i] = ProfileLutBake::profile(h, 0.06f, 0.90f, 0.05f, 0.10f); // 塔状积云
        cb[i]  = ProfileLutBake::profile(h, 0.10f, 0.98f, 0.30f, 0.40f); // 积雨云
    }
    const ImVec2 size(ImGui::GetContentRegionAvail().x, 34.f);
    ImGui::PushStyleColor(ImGuiCol_PlotLines, ImVec4(0.35f, 0.65f, 1.00f, 1.f)); // Cu 蓝
    ImGui::PlotLines("Cu ", cu, N, 0, nullptr, 0.f, 1.f, size);
    ImGui::PopStyleColor();
    ImGui::PushStyleColor(ImGuiCol_PlotLines, ImVec4(0.35f, 1.00f, 0.55f, 1.f)); // Tcu 绿
    ImGui::PlotLines("Tcu", tcu, N, 0, nullptr, 0.f, 1.f, size);
    ImGui::PopStyleColor();
    ImGui::PushStyleColor(ImGuiCol_PlotLines, ImVec4(1.00f, 0.45f, 0.40f, 1.f)); // Cb 红
    ImGui::PlotLines("Cb ", cb, N, 0, nullptr, 0.f, 1.f, size);
    ImGui::PopStyleColor();
    ImGui::TextDisabled("横轴=归一化高度 0~1（云底→云顶），纵轴=廓形强度；蓝=Cu 绿=Tcu 红=Cb");
}

// ==========================================================================
// Flower 云管线（cloud_common.glsl 系，VkCloudSystem）调参面板。
// 用法: FlowerCloudTunerImGui(&open, cloudSystem.tuneParams, &weatherPreview);
// 面板按功能折叠分组（CollapsingHeader），默认展开双层云等常用组。
// ==========================================================================
inline void FlowerCloudTunerImGui(bool* p_open, FlowerCloudTuneParams& p,
                                  WeatherMapImGuiPreview* weatherPreview = nullptr,
                                  VkCascadeShadowSystem* shadow = nullptr) {
    if (ImGui::IsKeyPressed(ImGuiKey_F3, false)) *p_open = !*p_open;
    if (!*p_open) return;

    ImGui::SetNextWindowSize(ImVec2(420, 760), ImGuiCond_FirstUseEver);
    if (!ImGui::Begin("体积云调参 (Flower Phase1)", p_open, ImGuiWindowFlags_None)) {
        ImGui::End();
        return;
    }

    // 顶部常驻：总开关 + 分辨率（不折叠，一眼可关）
    ImGui::Text("体积云");
    ImGui::Checkbox("flower体积云启用总开关", &p.flowerCloudEnabled);
    ImGui::SameLine();
    ImGui::Checkbox("1/4 分辨率 + Bayer", &p.useQuarterRes);
    ImGui::TextDisabled("关=全分辨率 raymarch（×16 成本，减盐胡椒）");
    ImGui::Separator();

    // ── 1. 渲染管线 / March 步进（第 1 层） ────────────────────────────────
    if (ImGui::CollapsingHeader("渲染管线 / March 步进", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::SliderInt("MarchSteps (baseline)", &p.marchingStepNum, 8, 2048);
        ImGui::SliderInt("StepHardMax", &p.marchStepHardMax, 64, 2048);
        ImGui::SliderFloat("StepMin (km)", &p.stepTMinKm, 0.001f, 2.0f, "%.4f", ImGuiSliderFlags_Logarithmic);
        ImGui::SliderFloat("StepMax (km)", &p.stepTMaxKm, 0.01f, 20.0f, "%.3f", ImGuiSliderFlags_Logarithmic);
        ImGui::SliderFloat("MaxTraceDist (km)", &p.maxTracingDistanceKm, 10.f, 2000.f, "%.0f");
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
    }

    // ── 2. March 优化（空跳 / 边缘缩步，第 2/3 层） ────────────────────────
    if (ImGui::CollapsingHeader("March 优化（空跳 / 边缘缩步）")) {
        ImGui::Text("壳内真空跳步（第 2 层）");
        ImGui::Checkbox("EmptySkip 启用", &p.emptySkipEnabled);
        ImGui::SliderFloat("EmptyStep (km)", &p.emptyStepKm, 0.05f, 10.0f, "%.2f", ImGuiSliderFlags_Logarithmic);
        ImGui::SliderFloat("EmptyJitter ±", &p.emptyStepJitter, 0.0f, 0.5f, "%.2f");
        ImGui::TextDisabled("仅 density==0；中点+终点探测通过才跳；Jitter=蓝噪声缩放空步");
        ImGui::Separator();
        ImGui::Text("云缘缩步（第 3 层）");
        ImGui::Checkbox("EdgeRefine 启用", &p.edgeRefineEnabled);
        ImGui::SliderFloat("EdgeScaleMin", &p.edgeScaleMin, 0.05f, 1.0f, "%.2f");
        ImGui::SliderFloat("EdgeGradLow", &p.edgeGradLow, 0.0001f, 0.05f, "%.4f", ImGuiSliderFlags_Logarithmic);
        ImGui::SliderFloat("EdgeGradHigh", &p.edgeGradHigh, 0.001f, 0.2f, "%.3f", ImGuiSliderFlags_Logarithmic);
        ImGui::TextDisabled("步长=stepT×mix(1,EdgeScaleMin,edgeSharpness) | edgeSharpness 来自 |Δ(σ·Δs)|/Δs");
    }

    // ── 3. 覆盖率与密度 ───────────────────────────────────────────────────
    if (ImGui::CollapsingHeader("覆盖率与密度")) {
        ImGui::SliderFloat("Coverage", &p.coverage, 0.0f, 1.0f, "%.3f");
        ImGui::SliderFloat("Density", &p.density, 0.0f, 4.0f, "%.2f");
    }

    // ── 4. 噪声频带阶梯 ───────────────────────────────────────────────────
    if (ImGui::CollapsingHeader("噪声频带阶梯")) {
        ImGui::TextDisabled("形状 Basic/Meso/Fine 共用 128³ 噪声，只换 scale；侵蚀用 32³ Worley");
        ImGui::TextDisabled("scale 单位 1/km；周期 = 1/scale ÷ 纹理频率");
        if (ImGui::Button("按 Nyquist 重建阶梯")) rebuildCloudNoiseLadder(p);
        ImGui::SameLine();
        ImGui::TextDisabled("锚 Basic：scale ×%.0f/档，幅度 ×%.2f（侵蚀 ×%.2f）",
            CloudNoiseLadder::kBasicBandSpan, CloudNoiseLadder::kShapeAmpFalloff,
            CloudNoiseLadder::kErosionAmpFalloff);

        ImGui::SliderFloat("BasicNoiseScale", &p.basicNoiseScale, 0.0001f, 4.0f, "%.4f", ImGuiSliderFlags_Logarithmic);
        ImGui::SliderFloat("MesoNoiseScale", &p.mesoNoiseScale, 0.0001f, 16.0f, "%.4f", ImGuiSliderFlags_Logarithmic);
        ImGui::SliderFloat("MesoAmp", &p.mesoAmplitude, 0.0f, 1.0f, "%.3f");
        ImGui::SliderFloat("FineNoiseScale", &p.fineNoiseScale, 0.0001f, 64.0f, "%.4f", ImGuiSliderFlags_Logarithmic);
        ImGui::SliderFloat("FineAmp", &p.fineAmplitude, 0.0f, 1.0f, "%.3f");
        ImGui::SliderFloat("DetailNoiseScale", &p.detailNoiseScale, 0.0001f, 256.0f, "%.3f", ImGuiSliderFlags_Logarithmic);
        ImGui::SliderFloat("DetailAmp", &p.detailAmplitude, 0.0f, 1.0f, "%.3f");
        ImGui::TextDisabled("幅度=0 的档在 shader 里连采样一起跳过（频带 LOD 的关闭档）");

        // 频带读数：scale 换算成"这一档实际覆盖的世界周期区间"，检查接缝/步长/精度。
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
    }

    // ── 5. 噪声 LOD（屏幕足迹） ───────────────────────────────────────────
    if (ImGui::CollapsingHeader("噪声 LOD（屏幕足迹）")) {
        ImGui::TextDisabled("一档特征不足 nMin 像素就淡出；Basic 恒开，只影响 Meso/Fine/Detail");
        ImGui::Checkbox("NoiseLodEnabled", &p.noiseLodEnabled);
        ImGui::BeginDisabled(!p.noiseLodEnabled);
        ImGui::SliderFloat("NoiseLodNMin", &p.noiseLodNMin, 0.5f, 8.0f, "%.2f px");
        ImGui::SliderFloat("NoiseLodShadowDistScale", &p.noiseLodShadowDistScale, 1.0f, 8.0f, "%.2f×");
        ImGui::TextDisabled("阴影 march 按 %.1f× 距离判定，比主视线更早丢细档", p.noiseLodShadowDistScale);
        {
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
    }

    // ── 6. 双层云 · Profile LUT（云类型廓形） ─────────────────────────────
    if (ImGui::CollapsingHeader("双层云 · Profile LUT", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::TextDisabled("LUT: U=归一化高度 V=径向 RGB=Cu/Tcu/Cb；weather.g 提供云类型");
        ImGui::TextDisabled("类型轴 0=Cu 0.5=Tcu 1=Cb；Cu 曲线 = 原 gradientShape 高度函数（积云预设）");
        ImGui::SliderFloat("TypeBias", &p.profileTypeBias, -0.5f, 0.5f, "%.3f");
        ImGui::SliderFloat("TypeContrast", &p.profileTypeContrast, 0.1f, 4.0f, "%.2f");
        ImGui::TextDisabled(">1 拉开 Cu/Tcu/Cb 差异；<1 收敛到中部类型");
        ImGui::TextDisabled("廓形即 LUT 曲线本身，无运行时倍率（云量调 Coverage、浓淡调 Density）");
        ProfileLutCurvePlot();
        if (weatherPreview && weatherPreview->profileLutReady) {
            PreviewImageAspect(weatherPreview->profileLutTexId(), 256.f / 64.f);
            ImGui::TextDisabled("LUT 纹理（256×64，U=高度 V=径向；R=Cu G=Tcu B=Cb）");
        } else {
            ImGui::TextDisabled("Profile LUT 预览未注册");
        }
    }

    // ── 7. 双层云 · 高空云（第二层，纯 2D） ───────────────────────────────
    if (ImGui::CollapsingHeader("双层云 · 高空云（第二层）", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::Checkbox("HiCloud 启用", &p.hiCloudEnabled);
        ImGui::TextDisabled("关=hiCloudCoverage 置 0，回到单层低云");
        ImGui::BeginDisabled(!p.hiCloudEnabled);
        ImGui::SliderFloat("HiCoverage", &p.hiCloudCoverage, 0.0f, 2.0f, "%.2f");
        ImGui::SliderFloat("HiBottom (h)", &p.hiCloudBottom, 0.3f, 0.9f, "%.2f");
        ImGui::SliderFloat("HiTop (h)", &p.hiCloudTop, 0.6f, 1.0f, "%.2f");
        ImGui::SliderFloat("HiDensity", &p.hiCloudDensity, 0.0f, 2.0f, "%.3f");
        ImGui::TextDisabled("与低云同单位；max() 进 cloudMap，共用主 march（×1000 光学深度）");
        ImGui::SliderFloat("HiThreshold", &p.hiCloudThreshold, 0.0f, 0.5f, "%.3f");
        ImGui::SliderFloat("HiSoftness", &p.hiCloudSoftness, 0.01f, 0.6f, "%.3f");
        ImGui::SliderFloat("HiType", &p.hiCloudType, 0.0f, 1.0f, "%.2f");
        ImGui::TextDisabled("0=As 高层云（成层） 1=Ac 高积云（块状）；与天气图 G 各取一半");
        ImGui::TextDisabled("带底/带顶是 slab 归一化高度；默认 0.55~0.95 在低云之上");
        ImGui::TextDisabled("并进 cloudMap：低云 remap 后 density×densityShape，再与 hiDensity 取 max");
        ImGui::EndDisabled();
        if (weatherPreview && weatherPreview->hiWeatherReady) {
            PreviewImageAspect(weatherPreview->hiWeatherTexId(), 2.f);
            ImGui::TextDisabled("Hi weather map（2048×1024，R=HiMask G=HiType）");
        } else {
            ImGui::TextDisabled("Hi weather map 预览未注册");
        }
    }

    // ── 8. 天气 / 风 / Curl ───────────────────────────────────────────────
    if (ImGui::CollapsingHeader("天气 / 风 / Curl")) {
        ImGui::Text("天气 / 风");
        ImGui::SliderFloat("WeatherUVScale", &p.weatherUVScale, 0.1f, 8.0f, "%.2f");
        ImGui::SliderFloat3("WindDir", &p.windDirX, -1.f, 1.f, "%.2f");
        ImGui::SliderFloat("Speed", &p.speed, 0.0f, 2.0f, "%.3f");
        ImGui::Separator();
        ImGui::Text("Curl / localCoverage（假 curl 纹理）");
        ImGui::SliderFloat("CurlUVScale", &p.curlUVScale, 0.00001f, 0.01f, "%.5f", ImGuiSliderFlags_Logarithmic);
        ImGui::SliderFloat("CurlStrength", &p.curlStrength, 0.0f, 1.0f, "%.3f");
        ImGui::SliderFloat("CurlContrast", &p.curlContrast, 0.5f, 8.0f, "%.2f");
        ImGui::TextDisabled("Strength=0 关闭碎点；轨道可试 UV≈0.0002 Strength≈0.1");
        ImGui::TextDisabled("旧硬编码: UV=0.001 Strength=0.2 Contrast=3");
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
            PreviewImageAspect(weatherPreview->texId(weatherVisChannel), 2.f);
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("经纬度 equirectangular · 左=-180° 右=+180° 下=南极 上=北极");
            ImGui::TextDisabled("R=coverage  G=type  B=humidity  A=land（bake 自 vk_scene）");
        } else {
            ImGui::TextDisabled("WeatherMap 预览未注册");
        }
        if (weatherPreview && weatherPreview->curlReady) {
            PreviewImageAspect(weatherPreview->curlTexId(), 1.f);
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("bake 的 256² R8（Worley+Perlin），非真 curl 矢量场");
        }
    }

    // ── 9. 光照 · 体积阴影 / 多重散射 / PhiFwd ────────────────────────────
    if (ImGui::CollapsingHeader("光照 · 阴影 / 多重散射 / PhiFwd")) {
        ImGui::Text("体积阴影步进 (volumetricShadow)");
        ImGui::SliderInt("LightStepNum", &p.lightStepNum, 1, 16);
        ImGui::SliderFloat("LightBasicStep (km)", &p.lightBasicStepKm, 0.001f, 0.5f, "%.4f", ImGuiSliderFlags_Logarithmic);
        ImGui::SliderFloat("LightStepMul", &p.lightStepMul, 1.0f, 4.0f, "%.2f");
        ImGui::TextDisabled("沿太阳方向自阴影：步长 = basicStep × mul^i");
        ImGui::Separator();
        ImGui::Text("多重散射 (Frostbite)");
        ImGui::SliderFloat("MsExtinction", &p.multiScatterExtinction, 0.0f, 1.0f, "%.3f");
        ImGui::SliderFloat("MsScatter", &p.multiScatterScatter, 0.0f, 1.0f, "%.3f");
        ImGui::Separator();
        ImGui::Text("PhiFwd 漫射场（各向同性 MS，加性）");
        ImGui::TextDisabled("与上面 MsExtinction/MsScatter 正交：先固定假 MS，再调本组");
        // Intensity 吸收离散格林函数里缺的 1/(4πD) 等常数，再乘米制 1/r，
        // 不是 0~1 物理量；0~4 通常 ≪ 直射，厚云芯要用 10^1~10^3 量级才看得见。
        ImGui::SliderFloat("PhiFwdIntensity", &p.phiFwdIntensity, 0.0f, 10000.0f, "%.1f", ImGuiSliderFlags_Logarithmic);
        ImGui::TextDisabled("0=关；对数滑条。先 dbg16 确认非黑，再 0→10→100→1000 对厚云暗部");
        ImGui::SliderFloat("PhiFwdMsBuild", &p.phiFwdMsBuildScale, 0.0f, 8.0f, "%.2f");
        ImGui::TextDisabled("越大越早建立各向同性；0=几乎不积 φ");
        ImGui::SliderFloat("PhiFwdCompress", &p.phiFwdCompress, 0.0f, 4.0f, "%.2f");
        ImGui::TextDisabled("软饱和；0=线性。Intensity 很大时用 0.1~1 防云芯爆");
        ImGui::SliderFloat("PhiFwdDepthPow", &p.phiFwdDepthPow, 0.0f, 4.0f, "%.2f");
        ImGui::SliderFloat("PhiFwdDepthBias", &p.phiFwdDepthBias, -0.3f, 0.5f, "%.3f");
        ImGui::TextDisabled("底部置信度：Pow=0 禁用；Bias>0 让云底保留更多漫射光");
        ImGui::SliderFloat("PhiFwdBoundary", &p.phiFwdBoundaryConfidence, 0.0f, 1.0f, "%.2f");
        ImGui::TextDisabled("云顶受光置信度：0=不判边界；1=全量抑制背光侧假亮");
    }

    // ── 10. 相位 / 银边 ───────────────────────────────────────────────────
    if (ImGui::CollapsingHeader("相位 / 银边")) {
        ImGui::SliderFloat("PhaseForward", &p.phaseForward, 0.0f, 0.99f, "%.3f");
        ImGui::SliderFloat("PhaseBackward", &p.phaseBackward, -0.99f, 0.0f, "%.3f");
        ImGui::SliderFloat("PhaseMix", &p.phaseMixFactor, 0.0f, 1.0f, "%.3f");
        ImGui::Separator();
        const char* powderModes[] = { "Off", "Legacy", "Sugar" };
        ImGui::Combo("PowderMode", &p.powderMode, powderModes, 3);
        ImGui::TextDisabled("Off=无粉末; Legacy=density×高×VoL; Sugar=2·Beer·(1-e^{-2τ})");
        if (p.powderMode == 1) {
            ImGui::SliderFloat("PowderScale", &p.powderScale, 0.0f, 4.0f, "%.2f");
            ImGui::SliderFloat("PowderPow", &p.powderPow, 0.0f, 4.0f, "%.2f");
        } else if (p.powderMode == 2) {
            ImGui::Checkbox("SugarApplyToAllMs", &p.powderSugarAllMs);
            ImGui::TextDisabled("关=仅 ms0（推荐）; 开=每阶都乘（易与 Ms/phi_fwd 叠亮）");
            ImGui::SliderFloat("SugarGain", &p.powderSugarGain, 0.0f, 4.0f, "%.2f");
            ImGui::TextDisabled("原文 ×2；薄处 energy≈Beer，中等 τ 鼓包");
            ImGui::SliderFloat("SugarViewWeight", &p.powderSugarViewWeight, 0.0f, 1.0f, "%.2f");
            ImGui::TextDisabled("0=按 -VoL 门控（太阳在身后）; 1=全向套用原文");
        }
    }

    // ── 11. 光照与高度 ────────────────────────────────────────────────────
    if (ImGui::CollapsingHeader("光照与高度")) {
        ImGui::SliderFloat("SunIntensityMul", &p.sunIntensityMul, 0.1f, 200000.0f, "%.1f", ImGuiSliderFlags_Logarithmic);
        ImGui::TextDisabled("全局曝光 L_sun=color×sunIntensity×本值；直射/环境/AP 同源");
        ImGui::SliderFloat("SunLightScale", &p.sunLightScale, 0.0f, 4.0f, "%.2f");
        ImGui::SliderFloat("MinAlt (km)", &p.minAltKm, 0.5f, 8.0f, "%.2f");
        ImGui::SliderFloat("MaxAlt (km)", &p.maxAltKm, 8.0f, 30.0f, "%.2f");
        ImGui::SliderFloat("CloudAlbedo", &p.cloudAlbedo, 0.0f, 4.0f, "%.2f");
        ImGui::SliderFloat("FogFade", &p.fogFade, 0.0f, 2.0f, "%.2f");
        ImGui::TextDisabled("云底地面反弹倍率（albedo×Ambient）；云顶 nearGround→0，不再灌 HDR");
        ImGui::SliderFloat("GroundNoiseScale", &p.groundNoiseScale, 0.0f, 4.0f, "%.2f");
        ImGui::TextDisabled("越大 → 地面反弹影响的高度带越厚（saturate(Scale−h)）");
        ImGui::Checkbox("GroundContribution", &p.enableGroundContribution);
        ImGui::TextDisabled("开=环境天光 Ambient + 云底 groundLit；关=只剩太阳直射");
    }

    // ── 12. 空气透视 / 环境天光 ───────────────────────────────────────────
    if (ImGui::CollapsingHeader("空气透视 / 环境天光")) {
        ImGui::Text("空气透视 (Inscatter ToPoint，物理)");
        ImGui::Checkbox("AirPerspective Enabled", &p.airPerspectiveEnabled);
        ImGui::SliderFloat("AirPerspectiveSunScale", &p.airPerspectiveSunScale, 0.001f, 30.0f, "%.3f", ImGuiSliderFlags_Logarithmic);
        ImGui::SliderFloat("AirPerspectiveScale", &p.airPerspectiveScale, 0.0f, 1.0f, "%.3f");
        ImGui::TextDisabled("L'=L_air×Lsun×SunScale + T_air×L；composite=src×(T_air×T_cloud)+L'");
        ImGui::TextDisabled("Scale=AP 强度(1=全物理)；SunScale 只乘 L_air；T_air 为独立 RGB RT");
        ImGui::Separator();
        ImGui::Text("环境天光 (μs LUT，与相机无关)");
        ImGui::SliderFloat("AmbientExposure", &p.ambientExposure, 0.1f, 50.0f, "%.2f", ImGuiSliderFlags_Logarithmic);
        ImGui::SliderFloat("AmbientAnalyticBlend", &p.ambientAnalyticBlend, 0.0f, 1.0f, "%.2f");
        ImGui::TextDisabled("单位太阳半球 LUT × L_sun × Exposure；夜半球由 μs 变暗");
    }

    // ── 13. God Ray / 级联阴影 ────────────────────────────────────────────
    if (ImGui::CollapsingHeader("God Ray / 级联阴影")) {
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
            ImGui::SliderFloat("BiasConst", &shadow->shadowBiasConst, 0.0f, 0.02f, "%.4f");
            ImGui::SliderFloat("BiasSlope", &shadow->shadowBiasSlope, 0.0f, 0.02f, "%.4f");
            ImGui::SliderFloat("BorderAdopt", &shadow->cascadeBorderAdopt, 0.0f, 0.2f, "%.3f");
            ImGui::SliderFloat("EdgeLerpThreshold", &shadow->cascadeEdgeLerpThreshold, 0.5f, 1.0f, "%.3f");
        } else {
            ImGui::TextDisabled("级联阴影系统未接入（shadow==nullptr）");
        }
    }

    // ── 14. 调试可视化 ────────────────────────────────────────────────────
    if (ImGui::CollapsingHeader("调试可视化（排查黑云用）")) {
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
            "16: 仅 phi_fwd 漫射场贡献（Intensity=0 时全黑）",
        };
        ImGui::Combo("DebugMode", &p.debugMode, dbgItems, IM_ARRAYSIZE(dbgItems));
        ImGui::TextDisabled("8/9仍暗→看11是否本就亮，再比6/12/13；非0时a=0替换背景");
        ImGui::TextDisabled("低云（3D）与高空云（2D）为独立 march，调试 5/6 显示合成后结果");
    }

    ImGui::Separator();
    ImGui::TextDisabled("F3: 开关面板");

    ImGui::End();
}
