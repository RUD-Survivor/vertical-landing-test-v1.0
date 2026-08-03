#pragma once
// ==========================================================================
// vk_cloud_system.h — Flower 体积云管线 Phase 1（真实接入，替换早期 Phase0 占位）
//
// Pass 顺序（在 TAA geometry pass 之后、TAA resolve 之前）：
//   0. VkAtmosphereLut::bake()   — 每帧针对当前云所在行星重烘 Transmittance/
//                                  MultiScatter/SkyView LUT + Sky Irradiance cubemap
//   1. cloud_raymarching  (comp) — 1/4 分辨率 ray march，输出 L+T / 深度 / 雾
//   2. cloud_reconstruct  (comp) — 时域重建到全分辨率（history ping-pong）
//   3. cloud_composite    (comp) — 合成回 HDR 场景色
//
// 对应 shader: src/render/shaders/cloud/{cloud_common,cloud_raymarching,
// cloud_reconstruct,cloud_composite}.glsl，descriptor 布局逐字段对应
// cloud_common.glsl 顶部 binding 0-30 的声明（Set 0）+ shared_sampler.glsl
// 的 10 个采样器（Set 1）+ Heitz Sobol 蓝噪声 SSBO（Set 2，shared_blue_noise.glsl）。
//
// 30 个 binding 里，SDSM 级联阴影(28/29)、Hi-Z(30)、G-Buffer A(5) 目前绑定中性
// 占位资源（Phase 2 待接入，见 cloud_common.glsl / cloud_composite.glsl 对应
// 位置的注释），不影响主 march 路径。curl 噪声(9) 已接入真实纹理（vk_scene.h::
// cloudCurlTex，见 vk_cloud_bake.h::bakeCurlNoise）。空气透视(11) 已改为
// Inscatter ToPoint（与场景大气同源，见 cloud_air_perspective.glsl）。
// ==========================================================================

#include "vk_context.h"
#include "vk_frame.h"
#include "vk_mesh.h"
#include "vk_pipeline.h"
#include "vk_utils.h"
#include "vk_taa.h"
#include "vk_atmosphere_lut.h"
#include "cloud_noise_ladder.h"
#include "vk_blue_noise.h"
#include "vk_shadow_cascade.h"
#include "gpu_frame_data.h"
#include "../scene_snapshot.h"
#include "../../math/math3d.h"

#include <cstdio>
#include <cstring>
#include <algorithm>
#include <cmath>

// 把 float[16] 列主序矩阵求逆（复用 math3d.h::Mat4 的通用 4x4 逆，行列主序在这里
// 不影响逆矩阵本身的正确性，只要输入输出用同一种约定）。
inline void invertMat4(const float in[16], float out[16]) {
    Mat4 m;
    for (int i = 0; i < 16; i++) m.m[i] = (double)in[i];
    Mat4 inv = m.inverse();
    for (int i = 0; i < 16; i++) out[i] = (float)inv.m[i];
}

// Flower 云管线（cloud_common.glsl 系）的可调参数，供 imgui_cloud_tuner.h 的
// FlowerCloudTunerImGui() 实时调节。默认值对应 fillCloudFrameData() 里原先写死的常量。
struct FlowerCloudTuneParams {
    bool flowerCloudEnabled = true;
    float coverage = 1.0f;
    float density   = 1.00f;
    // 噪声频带阶梯（换算与规则见 cloud_noise_ladder.h）。形状链 Basic/Meso/Fine
    // 共用同一张 128³ 噪声，只换世界频率；侵蚀链是 32³ Worley，接在形状链末档之后。
    // 幅度以 Basic=1 为基准；把某档幅度调 0 就等于把该档摘掉（shader 侧连采样一起跳过）。
    float basicNoiseScale  = CloudNoiseLadder::kDefaultBasicScale;
    float mesoNoiseScale   = CloudNoiseLadder::kDefaultMesoScale;
    float fineNoiseScale   = CloudNoiseLadder::kDefaultFineScale;
    float detailNoiseScale = CloudNoiseLadder::kDefaultDetailScale;
    float mesoAmplitude    = CloudNoiseLadder::kDefaultMesoAmp;
    float fineAmplitude    = CloudNoiseLadder::kDefaultFineAmp;
    float detailAmplitude  = CloudNoiseLadder::kDefaultDetailAmp;

    // 频带 LOD：上面的幅度是"艺术上限"，这里按采样点屏幕足迹把高频档淡出。
    // 关掉即退回全档常开，可用来对照 LOD 是否吃掉了本该看见的细节。
    bool  noiseLodEnabled = true;
    float noiseLodNMin = CloudNoiseLadder::kDefaultLodNMin;
    float noiseLodShadowDistScale = CloudNoiseLadder::kDefaultLodShadowDistScale;
    float multiScatterExtinction = 0.175f;
    float multiScatterScatter    = 1.0f;

    // ── phi_fwd 物理漫射场（cloud_common.glsl::volumetricShadow / cloudColorCompute）──
    // 扩散近似（τ≫1、g_eff→0）下的各向同性多重散射：沿太阳光步用 1D 格林函数累加，
    // 视线侧作为「加性」项注入，与上面的 Frostbite 假 MS 正交——两套分开调，
    // 先固定 multiScatter*，再开 phiFwd。Intensity=0 时 shader 整条路径跳过。
    // 标定旋钮（非 0~1）：公式含 σ_t²·Δs·(1/r_m)，常用可见量级约 10~1000。
    float phiFwdIntensity = 1000.0f;         // 0=关闭；默认提到可见区，过曝再降或开 Compress
    float phiFwdMsBuildScale = 1.0f;        // 各向同性建立速度；越大越早从方向性转漫射
    float phiFwdCompress = 0.0f;            // 软饱和；0=线性不压缩
    float phiFwdDepthPow = 1.0f;            // 底部置信度恢复速度；0=禁用底部软化
    float phiFwdDepthBias = 0.0f;           // 底部锚点偏移；>0 云底保留更多漫射
    float phiFwdBoundaryConfidence = 1.0f;  // 云顶受光置信度强度（0=禁用背光抑制）

    float phaseForward  = 0.f;
    float phaseBackward = -0.5f;
    float phaseMixFactor = 0.5f;
    float powderScale = 1.0f;
    float powderPow   = 1.0f;
    // 0=Off 1=Legacy 2=Sugar；默认 Legacy
    int   powderMode = 1;
    bool  powderSugarAllMs = false;
    float powderSugarGain = 2.0f;
    float powderSugarViewWeight = 1.0f;
    float sunLightScale = 1.0f;
    float minAltKm = 1.0f;
    float maxAltKm = 20.0f;
    float speed = 0.15f;
    // Bruneton/Hillaire 大气积分对"太阳辐照度"的物理量级要求，和引擎其余部分（atmo.frag 等）
    // 手调出来的 sunIntensity(~1.0) 未必是同一套标定。Inscatter/Ambient 按单位太阳烘焙；
    // 本值是运行时全局曝光：L_sun = color × snap.sunIntensity × sunIntensityMul。
    float sunIntensityMul = 100.0f;
    // 0=正常；1–7 中间量；8–15 厚云发暗诊断（见 cloud_common.glsl dbgMode）。
    // 借用 cloudSunLitMapOctave 传给 shader。
    int debugMode = 0;

    // ── 管线 / cloud_common.glsl frameData.sky.atmosphereConfig ─────────────
    // useQuarterRes → updateFaceIndex（1=Bayer 1/4 + reconstruct，0=全分辨率 raymarch）
    bool useQuarterRes = true;
    int  marchingStepNum = 256;              // cloudMarchingStepNum（baseline）
    float maxTracingDistanceKm = 350.f;       // cloudMaxTraceingDistance
    float tracingStartMaxDistanceKm = 1e8f;   // cloudTracingStartMaxDistance
    float stepTMinKm = 0.05f;               // cloudMarchStepMinKm
    float stepTMaxKm = 0.5f;                // cloudMarchStepMaxKm
    int   marchStepHardMax = 2048;          // cloudMarchStepHardMax

    // 第二层：壳内真空跳步（cloudMarchEmptySkip / EmptyStepKm）
    bool  emptySkipEnabled = true;          // cloudMarchEmptySkip
    float emptyStepKm = 1.0f;               // cloudMarchEmptyStepKm
    float emptyStepJitter = 0.25f;            // cloudMarchEmptyTauMax：空步 ± 比例 (0~0.5)

    // 第三层：云缘密度梯度缩步（cloudMarchEdge*）
    bool  edgeRefineEnabled = true;           // cloudMarchEdgeRefine
    float edgeScaleMin = 0.25f;               // cloudMarchEdgeScaleMin：边缘步长 = stepT×此值
    float edgeGradLow = 0.002f;               // cloudMarchEdgeGradLow
    float edgeGradHigh = 0.018f;              // cloudMarchEdgeGradHigh

    // 沿太阳方向体积阴影步进（volumetricShadow）
    float lightStepMul = 1.5f;                // cloudLightStepMul
    float lightBasicStepKm = 0.03f;          // cloudLightBasicStep (km)
    int   lightStepNum = 12;                   // cloudLightStepNum

    // 天气 / 风
    float weatherUVScale = 1.f;               // cloudWeatherUVScale
    float windDirX = 1.f, windDirY = 0.f, windDirZ = 0.f; // cloudDirection

    // Curl 纹理 → localCoverage（cloud_common.glsl；假 curl = Worley/Perlin 2D）
    float curlUVScale = 0.001f;               // cloudCurlUVScale；轨道可试 0.0002
    float curlStrength = 0.0f;                // cloudCurlStrength；0=关闭碎点
    float curlContrast = 3.0f;                // cloudCurlContrast

    // 地面光 / 环境
    float fogFade = 1.0f;                       // cloudFogFade：云底 groundLit 倍率（云顶已归零）
    float groundNoiseScale = 4.0f;             // cloudNoiseScale
    bool  enableGroundContribution = true;    // cloudEnableGroundContribution
    float cloudAlbedo = 1.f;                  // cloudAlbedo.rgb

    // ── 双层云：Profile LUT 类型轴 ─────────────────────────────────────────
    // 云类型廓形 LUT（inCloudProfileLut）：U=归一化高度 V=径向 RGB=Cu/Tcu/Cb。
    // weather.g 提供类型，本组负责把 weather.g 映射到 LUT 类型轴（0=Cu,0.5=Tcu,1=Cb）。
    // 廓形即 LUT 曲线本身，无运行时倍率（云量调 Coverage、浓淡调 Density）。
    float profileTypeBias = 0.0f;             // 类型偏移
    float profileTypeContrast = 1.0f;         // 类型对比度（>1 拉开类型差异）

    // ── 双层云：高空云（第二层，inCloudHiWeather，纯 2D）──────────────────
    bool  hiCloudEnabled = true;
    float hiCloudCoverage = 0.4f;             // 高空云全局覆盖倍率（0=关闭第二层）
    float hiCloudBottom = 0.90f;              // 高空云带底（slab 归一化高度）
    float hiCloudTop = 0.95f;                 // 高空云带顶
    float hiCloudDensity = 1.0f;              // 高空云密度（与低云 cloudMap 同单位，主 march ×1000）
    float hiCloudThreshold = 0.05f;           // hi 覆盖阈值
    float hiCloudSoftness = 0.2f;             // 阈值过渡宽度
    float hiCloudType = 1.0f;                 // 高空云类型偏好（0=As 1=Ac）

    // Inscatter ToPoint 空气透视：单位太阳；运行时 × L_sun × airPerspectiveSunScale
    bool  airPerspectiveEnabled = true;       // cloudAirPerspectiveEnabled
    float airPerspectiveSunScale = 1.0f;      // 相对 L_sun 的雾色倍率（默认 1=与云直射同源）
    float airPerspectiveScale = 0.1f;        // 采样混合权重倍率（α'=α×此值；默认 0.01）

    // 环境天光：单位太阳 μs LUT × L_sun × ambientExposure
    float ambientExposure = 1.0f;
    float ambientAnalyticBlend = 0.f;

    // God Ray（400m 近距体积雾光柱 march，cloud_composite.glsl:108 起）。默认关闭——
    // 需要先在 VkCascadeShadowSystem（vk_shadow_cascade.h）里也打开 enabled，两者
    // 缺一都看不到效果：阴影关着 God Ray 全程 visibilityTerm=1（无遮挡但仍然有光轴）；
    // God Ray 关着阴影渲染是纯浪费（没人读图集）。
    bool  godRayEnabled = false;
    float godRayScale = 1.0f; // cloudGodRayScale：光轴强度倍率
};

// 以 basicNoiseScale 为锚点重算整条频带阶梯（Cloud Tuner 的"重建阶梯"按钮）。
// 手调各档后想回到"频率无空洞、幅度逐档递减"的基线时用这个，避免手算 ×16。
inline void rebuildCloudNoiseLadder(FlowerCloudTuneParams& p) {
    using namespace CloudNoiseLadder;
    if (!(p.basicNoiseScale > 0.f)) p.basicNoiseScale = kDefaultBasicScale;
    p.mesoNoiseScale   = nextScale(p.basicNoiseScale, kBasicBandSpan);
    p.fineNoiseScale   = nextScale(p.mesoNoiseScale,  kBasicBandSpan);
    p.detailNoiseScale = nextScale(p.fineNoiseScale,  kBasicBandSpan);
    p.mesoAmplitude   = kShapeAmpFalloff;
    p.fineAmplitude   = p.mesoAmplitude * kShapeAmpFalloff;
    p.detailAmplitude = p.fineAmplitude * kErosionAmpFalloff;
}

struct VkCloudSystem {
    static constexpr VkFormat kColorFmt = VK_FORMAT_R16G16B16A16_SFLOAT;
    static constexpr VkFormat kDepthFmt  = VK_FORMAT_R32_SFLOAT;

    bool enabled = true;  // Flower 体积云总开关（main.cpp 判断）
    bool debugLogOnce = true;
    // dispatch 成功和"跳过"两条路径分开各自的一次性日志开关（之前共用一个 debugLogOnce，
    // 一旦被"成功 dispatch"用掉，后面哪怕真的开始被跳过也不会再有任何提示，排查起来
    // 会误以为管线一直在正常跑）。
    bool skipLogOnce = true;
    FlowerCloudTuneParams tuneParams;

    VkExtent2D extent     = {};
    VkExtent2D quarterExt = {};

    // atmosphereLut 现在是"单星球 LUT 资源实例"（见 vk_atmosphere_lut.h 重构），
    // 不再自带 compute pipeline——云管线自己单独持有一份 VkAtmoLutPipelines
    // （不复用 vk_renderer3d.h 大气重构那边的全局 VkAtmoLutCache，两个子系统各自
    // 独立初始化/关闭，避免跨子系统生命周期耦合；反正 pipeline 本身很轻量）。
    VkAtmoLutPipelines cloudLutPipelines;
    VkAtmosphereLut atmosphereLut;
    // 云空气透视走 Inscatter ToPoint，需要 bakeFull 烤出的 Transmittance + Inscatter。
    // 云日照大气透射是实时算的（realTimeTransmittance），不需要每帧重烤 Sky-View；
    // 地球 profile 基本不变 → 只 bakeFull 一次，之后每帧只刷 Ambient μs LUT。
    // Inscatter 为单位太阳（shader 内不乘 intensity），太阳亮度变化不再触发 bakeFull。
    bool atmosphereLutBaked = false;

    // ── 1/4 分辨率 RT：颜色(L,T) / 深度 / 雾 ─────────────────────────────────
    VkImage quarterColorImg=VK_NULL_HANDLE, quarterDepthImg=VK_NULL_HANDLE, quarterFogImg=VK_NULL_HANDLE;
    VmaAllocation quarterColorAlloc=VK_NULL_HANDLE, quarterDepthAlloc=VK_NULL_HANDLE, quarterFogAlloc=VK_NULL_HANDLE;
    VkImageView quarterColorView=VK_NULL_HANDLE, quarterDepthView=VK_NULL_HANDLE, quarterFogView=VK_NULL_HANDLE;

    // ── 全分辨率 RT：颜色/深度/雾，ping-pong ×2（[frameSlot] 当前写，[1-frameSlot] 当帧的 history）
    VkImage fullColorImg[2]={}, fullDepthImg[2]={}, fullFogImg[2]={};
    VmaAllocation fullColorAlloc[2]={}, fullDepthAlloc[2]={}, fullFogAlloc[2]={};
    VkImageView fullColorView[2]={}, fullDepthView[2]={}, fullFogView[2]={};

    // ── Phase 2 占位资源（SDSM/Froxel/Hi-Z/GBufferA；curl 噪声已用真实纹理，见 curlNoiseView）──
    VkImage dummy2DImg=VK_NULL_HANDLE; VmaAllocation dummy2DAlloc=VK_NULL_HANDLE; VkImageView dummy2DView=VK_NULL_HANDLE;
    VkImage dummy3DImg=VK_NULL_HANDLE; VmaAllocation dummy3DAlloc=VK_NULL_HANDLE; VkImageView dummy3DView=VK_NULL_HANDLE;
    VkBuffer dummyCascadeBuf=VK_NULL_HANDLE; VmaAllocation dummyCascadeAlloc=VK_NULL_HANDLE;

    // ── 噪声/天气纹理（复用 vk_scene.h 已烘焙好的资源，不重复创建）─────────────
    VkImageView basicNoiseView=VK_NULL_HANDLE, detailNoiseView=VK_NULL_HANDLE, weatherView=VK_NULL_HANDLE;
    VkImageView curlNoiseView=VK_NULL_HANDLE; // Phase 2：真实 curl 湍流细节纹理（vk_scene.h::cloudCurlTex）
    // ── 双层云：Profile LUT + 高空云 Hi weather map（vk_scene.h 烘焙）─────
    VkImageView profileLutView=VK_NULL_HANDLE;
    VkImageView hiWeatherView=VK_NULL_HANDLE;

    VkSampler linearRepeatSampler=VK_NULL_HANDLE, linearClampSampler=VK_NULL_HANDLE, pointClampSampler=VK_NULL_HANDLE;

    // ── Set 1：shared_sampler.glsl 的 10 个采样器（一次性创建，静态） ─────────
    VkDescriptorSetLayout samplerSetLayout = VK_NULL_HANDLE;
    VkDescriptorSet       samplerSet       = VK_NULL_HANDLE;
    VkSampler samplers10[10] = {};

    // ── Set 0：cloud_common.glsl 31 个 binding ────────────────────────────────
    VkDescriptorSetLayout cloudSetLayout = VK_NULL_HANDLE;
    VkDescriptorSet       cloudSet[FRAMES_IN_FLIGHT] = {};
    VkPipelineLayout      cloudPipelineLayout = VK_NULL_HANDLE;
    VkDescriptorPool      pool = VK_NULL_HANDLE;

    VkPipeline raymarchPipe=VK_NULL_HANDLE, reconstructPipe=VK_NULL_HANDLE, compositePipe=VK_NULL_HANDLE;

    VkBlueNoiseBuffers blueNoise;

    bool cloudRtInitialized = false;

    // 上一次 fillCloudFrameData() 调用时算出的 camViewProj，供时域重投影用（见该函数内
    // camViewProjPrev 的赋值处）。首帧 hasPrevViewProj=false，退化成"prev=current"。
    float prevCamViewProj[16] = {};
    bool hasPrevViewProj = false;

    // 上一帧的相机模式（0=Orbit 1=Chase 2=Panorama 3=Free，见 scene_snapshot.h::cameraMode）。
    // 用于探测"切镜头"（C 键切换 Orbit/Chase/Panorama/Free）——这类切换本身就是相机位置
    // 的硬跳变（比如 camera_director.h 里进入 Free 模式会直接把相机搬到火箭当前位置），
    // 不是"prev/current 矩阵不一致"这种连续运动，caches 的 prevCamViewProj 在这一帧
    // 反而是错的参考系，需要靠 bCameraCut 让 cloud_reconstruct.glsl 那一帧直接放弃 history。
    // 注：全景模式内部切换聚焦天体（,/. 键）是同类型跳变，但 focus_target 目前没有
    // 传进 SceneSnapshot，暂不覆盖。
    int prevCameraMode = -1;

    // -----------------------------------------------------------------------
    bool init(VulkanContext& ctx, VkExtent2D ext,
              VkImageView basicNoise, VkImageView detailNoise, VkImageView weather, VkImageView curlNoise,
              VkImageView profileLut, VkImageView hiWeather) {
        extent = ext;
        quarterExt = { std::max(1u, (ext.width + 3) / 4), std::max(1u, (ext.height + 3) / 4) };
        basicNoiseView = basicNoise; detailNoiseView = detailNoise; weatherView = weather; curlNoiseView = curlNoise;
        profileLutView = profileLut; hiWeatherView = hiWeather;

        if (!cloudLutPipelines.init(ctx))       return false;
        if (!atmosphereLut.init(ctx, cloudLutPipelines)) return false;
        if (!createSamplers(ctx))     return false;
        if (!createImages(ctx))       return false;
        if (!blueNoise.init(ctx))     return false;
        if (!createDescriptors(ctx))  return false;
        if (!createPipelines(ctx))    return false;

        printf("[VkCloudSystem] Phase1 ready %ux%u (quarter %ux%u)\n",
               extent.width, extent.height, quarterExt.width, quarterExt.height);
        return true;
    }

    void recreate(VulkanContext& ctx, VkExtent2D ext) {
        vkDeviceWaitIdle(ctx.device);
        destroyPipelines(ctx.device);
        destroyDescriptorsDynamic(ctx.device);
        destroyImages(ctx);
        extent = ext;
        quarterExt = { std::max(1u, (ext.width + 3) / 4), std::max(1u, (ext.height + 3) / 4) };
        if (!createImages(ctx) || !createDescriptors(ctx) || !createPipelines(ctx)) {
            fprintf(stderr, "[VkCloudSystem] recreate failed\n");
        }
    }

    void shutdown(VulkanContext& ctx) {
        vkDeviceWaitIdle(ctx.device);
        destroyPipelines(ctx.device);
        destroyDescriptorsDynamic(ctx.device);
        blueNoise.shutdown(ctx);
        if (samplerSetLayout) { vkDestroyDescriptorSetLayout(ctx.device, samplerSetLayout, nullptr); samplerSetLayout=VK_NULL_HANDLE; }
        destroyImages(ctx);
        for (VkSampler& s : samplers10) if (s) { vkDestroySampler(ctx.device, s, nullptr); s = VK_NULL_HANDLE; }
        if (dummyCascadeBuf) { vmaDestroyBuffer(ctx.allocator, dummyCascadeBuf, dummyCascadeAlloc); dummyCascadeBuf=VK_NULL_HANDLE; }
        atmosphereLut.shutdown(ctx);
        cloudLutPipelines.shutdown(ctx);
    }

    // Called after TAA endGeometryPass — HDR + depth are SHADER_READ_ONLY.
    // 在 snap.celestials 里自己找地球类(atmoIdx==3)行星，抽取中心/半径；不需要调用方额外传参。
    //
    // shadowAtlasView/cascadeInfoBuf/cascadeCfg：来自 vk_renderer3d.h 持有的 VkCascadeShadowSystem，
    // 在本函数调用前已经渲染好本帧的级联阴影贴图（见 vk_shadow_cascade.h::render()）。三者
    // 全部为空（默认值）时退化成 Phase2 之前的占位行为（cascadeCount=0，God Ray 级联查找自动跳过）。
    void renderPhase0(VulkanContext& ctx, VkCommandBuffer cmd, VkTAA& taa,
                      const SceneSnapshot& snap, int frameSlot, bool drawClouds,
                      VkImageView shadowAtlasView = VK_NULL_HANDLE, VkBuffer cascadeInfoBuf = VK_NULL_HANDLE,
                      const GpuCascadeShadowConfig* cascadeCfg = nullptr) {
        if (!enabled) return;
        if (!tuneParams.flowerCloudEnabled) return;
        if (!drawClouds) {
            if (skipLogOnce) {
                fprintf(stderr, "[VkCloudSystem] Phase1 skip: drawClouds=false (need Earth atmoIdx=3 + showClouds)\n");
                skipLogOnce = false;
            }
            return;
        }
        skipLogOnce = true; // 下次真的又被跳过时，还能再打一次（不是一辈子只报一次）
        if (raymarchPipe == VK_NULL_HANDLE) return;

        const CelestialDraw* earth = nullptr;
        for (const auto& cd : snap.celestials) { if (cd.atmoIdx == 3 && cd.showClouds) { earth = &cd; break; } }
        if (!earth) {
            fprintf(stderr, "[VkCloudSystem] Phase1 skip: earth==nullptr despite drawClouds=true (snap 不一致?)\n");
            return;
        }

        if (debugLogOnce) {
            printf("[VkCloudSystem] Phase1 dispatch frame %u (%ux%u)\n", snap.frameIndex, extent.width, extent.height);
            debugLogOnce = false;
        }

        int slot = frameSlot % FRAMES_IN_FLIGHT;
        int histSlot = 1 - slot;

        GpuPerFrameData fd = fillCloudFrameData(snap, *earth, cascadeCfg);
        // 云日照透射实时算（realTimeTransmittance）；空气透视查 bakeFull 的 Inscatter ToPoint，
        // 不必每帧重烤 Transmittance/Inscatter/SkyView。binding 11/27 是活绑定：
        // 11=Inscatter（ToPoint AP），27=云环境 μs LUT（查 Inscatter 烘出）。
        //
        // atmosphereLut 的 UBO 同时还是 cloud_common.glsl binding 21 的数据来源（相机矩阵、
        // 太阳方向、DebugMode 开关都在里面），必须每帧上传，不能跟着 bake() 一起跳过。
        //
        // 单位太阳烘焙：integrateScatteredLuminance 只读 sky.color，不读 intensity。
        // 亮度旋钮只影响运行时采样，不再触发重烤。
        if (!atmosphereLutBaked) {
            atmosphereLut.bakeFull(ctx, cmd, frameSlot, cloudLutPipelines, fd);
            atmosphereLutBaked = true;
        } else {
            atmosphereLut.uploadFrameData(ctx, cmd, frameSlot, fd);
        }
        // 每帧刷新云环境 μs LUT（128×1，与相机无关，纯查单位太阳 Inscatter）
        atmosphereLut.bakeAmbientMusLut(ctx, cmd, frameSlot, cloudLutPipelines);

        updateCloudDescriptorSet(ctx, taa, slot, histSlot, fd, shadowAtlasView, cascadeInfoBuf);

        VkDescriptorSet sets[3] = { cloudSet[slot], samplerSet, blueNoise.set };

        VkImage hdrImage = taa.colorImages[taa.currentIdx];

        // Phase0 曾经每帧转换 quarter RT 的 layout；Phase1 里所有云自建 RT 都常驻 GENERAL
        // （创建时一次性转换，见 createImages），这里只需要处理场景 HDR 颜色的 layout 切换。
        //
        // 注意 dstAccessMask 必须同时包含 SHADER_WRITE 和 SHADER_READ：composite pass 最后要
        // 写 imageHdrSceneColor（WRITE），但现在 raymarch pass 一开始就要读 inHdrSceneColor
        // 当 skyBackgroundColor（放弃 Sky-View LUT 后新加的依赖，直接采样 atmo pass 已经
        // 渲染好的天空色）——只声明 WRITE 的话，raymarch 这次读取没有正确的内存依赖保证，
        // 可能读到未定稿的数据，云直接就没了。
        transitionImage(cmd, hdrImage, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, VK_IMAGE_LAYOUT_GENERAL,
            VK_PIPELINE_STAGE_2_FRAGMENT_SHADER_BIT, VK_ACCESS_2_SHADER_READ_BIT,
            VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT, VK_ACCESS_2_SHADER_WRITE_BIT | VK_ACCESS_2_SHADER_READ_BIT);

        vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_COMPUTE, cloudPipelineLayout, 0, 3, sets, 0, nullptr);

        vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_COMPUTE, raymarchPipe);
        const VkExtent2D marchExt = tuneParams.useQuarterRes ? quarterExt : extent;
        dispatch2D(cmd, marchExt);
        computeBarrier(cmd);

        if (tuneParams.useQuarterRes) {
            vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_COMPUTE, reconstructPipe);
            dispatch2D(cmd, extent);
            computeBarrier(cmd);
        }

        vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_COMPUTE, compositePipe);
        dispatch2D(cmd, extent);

        transitionImage(cmd, hdrImage, VK_IMAGE_LAYOUT_GENERAL, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
            VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT, VK_ACCESS_2_SHADER_WRITE_BIT,
            VK_PIPELINE_STAGE_2_FRAGMENT_SHADER_BIT, VK_ACCESS_2_SHADER_READ_BIT);

        cloudRtInitialized = true;
    }

private:
    static void dispatch2D(VkCommandBuffer cmd, VkExtent2D ext) {
        vkCmdDispatch(cmd, (ext.width + 7) / 8, (ext.height + 7) / 8, 1);
    }

    static void computeBarrier(VkCommandBuffer cmd) {
        VkMemoryBarrier2 b{ VK_STRUCTURE_TYPE_MEMORY_BARRIER_2 };
        b.srcStageMask = VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT; b.srcAccessMask = VK_ACCESS_2_SHADER_WRITE_BIT;
        b.dstStageMask = VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT; b.dstAccessMask = VK_ACCESS_2_SHADER_READ_BIT | VK_ACCESS_2_SHADER_WRITE_BIT;
        VkDependencyInfo dep{ VK_STRUCTURE_TYPE_DEPENDENCY_INFO }; dep.memoryBarrierCount=1; dep.pMemoryBarriers=&b;
        vkCmdPipelineBarrier2(cmd, &dep);
    }

    // 填充 cloud_common.glsl binding 21 用的每帧数据：相机矩阵（含逆矩阵）+ sky/atmosphere 配置。
    // 与 vk_atmosphere_lut.h 的 LUT bake 共用同一份数据（同一个 UBO 内容）。
    GpuPerFrameData fillCloudFrameData(const SceneSnapshot& snap, const CelestialDraw& earth, const GpuCascadeShadowConfig* cascadeCfg) {
        GpuPerFrameData fd{};

        fd.appTime[0] = snap.time; fd.appTime[1] = sinf(snap.time); fd.appTime[2] = cosf(snap.time);
        fd.frameIndex4[0] = (uint32_t)snap.frameIndex;
        fd.frameIndex4[1] = fd.frameIndex4[0] % 8; fd.frameIndex4[2] = fd.frameIndex4[0] % 16; fd.frameIndex4[3] = fd.frameIndex4[0] % 32;

        // 相机相对当前云所在行星中心的位置。snap.camPos / earth.center 和 earth.radius 一样，
        // 是 vk_renderer3d.h 里已经在用的公里单位（对照那边 camAltKm = length(camPos-center) -
        // surfaceRadius，没有额外换算）。但 shared_atmosphere.glsl::convertToAtmosphereUnit()
        // 是给"米"设计的（内部 *0.001 转公里），所以这里要先转成米再存进 frameData.camWorldPos，
        // 否则会被再缩小 1000 倍，相当于相机深埋地心，云 ray march 一开始就被判定为撞地提前退出。
        float camRelPlanetKm[3] = { snap.camPos[0]-earth.center[0], snap.camPos[1]-earth.center[1], snap.camPos[2]-earth.center[2] };
        fd.camWorldPos[0]=camRelPlanetKm[0]*1000.0f; fd.camWorldPos[1]=camRelPlanetKm[1]*1000.0f; fd.camWorldPos[2]=camRelPlanetKm[2]*1000.0f;

        // fovy/aspect：噪声屏幕足迹 LOD（cloud_noise_lod.glsl）要用竖直视场把世界周期换算成
        // 像素数。snap.proj 由 math3d.h::Mat4::perspective 生成，m[5] = 1/tan(fovy/2)，从矩阵
        // 反解比再传一份 fov 字段少一处会失同步的状态。
        const float projYY = snap.proj[5], projXX = snap.proj[0];
        fd.camInfo[0] = (projYY > 1e-6f) ? 2.0f * atanf(1.0f / projYY) : 0.8f;
        fd.camInfo[1] = (projYY > 1e-6f && fabsf(projXX) > 1e-6f) ? projYY / projXX : 1.0f;
        fd.camInfoPrev[0] = fd.camInfo[0]; fd.camInfoPrev[1] = fd.camInfo[1];
        fd.camInfo[2] = 0.1f; fd.camInfo[3] = 1e7f; // near/far，粗略值：目前 shader 未依赖精确值
        fd.camInfoPrev[2] = fd.camInfo[2]; fd.camInfoPrev[3] = fd.camInfo[3];

        memcpy(fd.camView, snap.view, 64);
        memcpy(fd.camProj, snap.proj, 64);
        Mat4 view, proj;
        for (int i=0;i<16;i++){ view.m[i]=(double)snap.view[i]; proj.m[i]=(double)snap.proj[i]; }
        Mat4 viewProj = proj * view;
        for (int i=0;i<16;i++) fd.camViewProj[i] = (float)viewProj.m[i];
        invertMat4(snap.view, fd.camInvertView);
        invertMat4(snap.proj, fd.camInvertProj);
        invertMat4(fd.camViewProj, fd.camInvertViewProj);
        // 没做单独的去抖动(NoJitter)矩阵，直接复用当前帧——云管线没有开抖动
        // （bEnableJitter=0），真正的 TAA 几何抖动由 vk_taa.h 单独处理，与云管线无关。
        memcpy(fd.camProjNoJitter, fd.camProj, 64);
        memcpy(fd.camViewProjNoJitter, fd.camViewProj, 64);
        memcpy(fd.camInvertProjNoJitter, fd.camInvertProj, 64);
        memcpy(fd.camInvertViewProjNoJitter, fd.camInvertViewProj, 64);

        // 已修复：camViewProjPrev 原先每帧都被 memcpy 成"当前帧"的 camViewProj，等于
        // 永远等于 current——cloud_reconstruct.glsl 用它把当前世界坐标反投影到"上一帧
        // 屏幕位置"时，prev==current 会让这一步变成恒等变换（uvPrev≈uv，等价于假装
        // 相机没动）。低速/静止时看不出问题，但高速飞行（本工程轨道速度常见 6~9km/s）
        // 时一帧内相机实际挪动的距离足以让同一世界点在屏幕上明显移位，"假装没动"的
        // 重投影会把 history 采样到错误位置，叠加 16 帧 Bayer 轮转就是稀疏噪点花屏、
        // 高速时永远收敛不了的根因之一。改成真正缓存"上一次调用时的 camViewProj"。
        if (hasPrevViewProj) {
            memcpy(fd.camViewProjPrev, prevCamViewProj, 64);
        } else {
            memcpy(fd.camViewProjPrev, fd.camViewProj, 64); // 首帧无历史，退化成恒等变换
        }
        memcpy(fd.camViewProjPrevNoJitter, fd.camViewProjPrev, 64);
        memcpy(prevCamViewProj, fd.camViewProj, 64);
        hasPrevViewProj = true;

        // 切镜头探测：cameraMode 变化（C 键切 Orbit/Chase/Panorama/Free）是相机位置的硬跳变，
        // 不是连续运动，prevCamViewProj 在这一帧不是有意义的参考系，标记 bCameraCut 让
        // cloud_reconstruct.glsl 那一帧强制放弃 history（该逻辑本就存在，见 39 行 bPrevUvValid，
        // 之前只是没有任何调用方真的把 bCameraCut 置过 1）。
        bool bModeCut = (prevCameraMode != -1) && (snap.cameraMode != prevCameraMode);
        prevCameraMode = snap.cameraMode;

        fd.jitterPeriod = 16; fd.bEnableJitter = 0; fd.bCameraCut = bModeCut ? 1u : 0u;
        fd.skyValid = 1; fd.skySDSMValid = 0; fd.fixExposure = 1.0f; fd.bAutoExposure = 0;
        fd.renderWidth = (float)extent.width; fd.renderHeight = (float)extent.height;
        fd.displayWidth = fd.renderWidth; fd.displayHeight = fd.renderHeight;

        // 太阳方向：snap.lightDir 是"行星指向太阳"的方向（updateLightDir 用 sunWorldPos-center 填的），
        // frameData.sky.direction 约定是"光线传播方向"（从太阳照向场景），互为相反数。
        fd.sky.direction[0] = -snap.lightDir[0]; fd.sky.direction[1] = -snap.lightDir[1]; fd.sky.direction[2] = -snap.lightDir[2];
        fd.sky.color[0]=fd.sky.color[1]=fd.sky.color[2]=1.0f;
        // 运行时 L_sun = color × intensity；LUT 烘焙端不乘 intensity（单位太阳）。
        fd.sky.intensity = snap.sunIntensity * tuneParams.sunIntensityMul;

        // cascadeCfg 来自 VkCascadeShadowSystem::render() 本帧渲染结果；为空（阴影关闭/无地形
        // 面片可画）时退化成 cascadeCount=0，cloud_composite.glsl 的级联查找/God Ray 自动跳过。
        if (cascadeCfg) fd.sky.cacsadeConfig = *cascadeCfg;
        else fd.sky.cacsadeConfig.cascadeCount = 0;

        const float kAtmosphereThicknessKm = 160.0f; // 与 vk_renderer3d.h 里 apc.outerRadius = radius+160 保持一致
        fillPlanetAtmosphereProfile(fd.sky.atmosphereConfig, earth.atmoIdx, earth.radius, kAtmosphereThicknessKm,
                                     tuneParams.minAltKm, tuneParams.maxAltKm, 0.0f);
        fd.sky.atmosphereConfig.camWorldPos[0]=camRelPlanetKm[0];
        fd.sky.atmosphereConfig.camWorldPos[1]=camRelPlanetKm[1];
        fd.sky.atmosphereConfig.camWorldPos[2]=camRelPlanetKm[2];

        // 用 tuneParams 覆盖 fillPlanetAtmosphereProfile() 里的云调参默认值（Cloud Tuner 面板实时调节）。
        auto& ac = fd.sky.atmosphereConfig;
        ac.cloudCoverage = tuneParams.coverage;
        ac.cloudDensity = tuneParams.density;
        ac.cloudBasicNoiseScale = tuneParams.basicNoiseScale;
        ac.cloudMesoNoiseScale = tuneParams.mesoNoiseScale;
        ac.cloudFineNoiseScale = tuneParams.fineNoiseScale;
        ac.cloudDetailNoiseScale = tuneParams.detailNoiseScale;
        ac.cloudMesoAmplitude = tuneParams.mesoAmplitude;
        ac.cloudFineAmplitude = tuneParams.fineAmplitude;
        ac.cloudDetailAmplitude = tuneParams.detailAmplitude;
        ac.cloudNoiseLodNMin = tuneParams.noiseLodNMin;
        ac.cloudNoiseLodShadowDistScale = tuneParams.noiseLodShadowDistScale;
        ac.cloudNoiseLodEnabled = tuneParams.noiseLodEnabled ? 1 : 0;
        ac.cloudMultiScatterExtinction = tuneParams.multiScatterExtinction;
        ac.cloudMultiScatterScatter = tuneParams.multiScatterScatter;
        ac.cloudPhiFwdIntensity = tuneParams.phiFwdIntensity;
        ac.cloudPhiFwdMsBuildScale = tuneParams.phiFwdMsBuildScale;
        ac.cloudPhiFwdCompress = tuneParams.phiFwdCompress;
        ac.cloudPhiFwdDepthPow = tuneParams.phiFwdDepthPow;
        ac.cloudPhiFwdDepthBias = tuneParams.phiFwdDepthBias;
        ac.cloudPhiFwdBoundaryConfidence = tuneParams.phiFwdBoundaryConfidence;
        ac.cloudPhaseForward = tuneParams.phaseForward;
        ac.cloudPhaseBackward = tuneParams.phaseBackward;
        ac.cloudPhaseMixFactor = tuneParams.phaseMixFactor;
        ac.cloudPowderScale = tuneParams.powderScale;
        ac.cloudPowderPow = tuneParams.powderPow;
        ac.cloudPowderMode = (tuneParams.powderMode < 0) ? 0 : (tuneParams.powderMode > 2 ? 2 : tuneParams.powderMode);
        ac.cloudPowderSugarAllMs = tuneParams.powderSugarAllMs ? 1 : 0;
        ac.cloudPowderSugarGain = tuneParams.powderSugarGain;
        ac.cloudPowderSugarViewWeight = tuneParams.powderSugarViewWeight;
        ac.cloudShadingSunLightScale = tuneParams.sunLightScale;
        ac.cloudSpeed = tuneParams.speed;
        ac.cloudSunLitMapOctave = tuneParams.debugMode;

        // RS3D：复用 updateFaceIndex 传递 1/4 Bayer 开关（flower 原字段未用）
        ac.updateFaceIndex = tuneParams.useQuarterRes ? 1u : 0u;
        ac.cloudMarchingStepNum = tuneParams.marchingStepNum;
        ac.cloudMaxTraceingDistance = tuneParams.maxTracingDistanceKm;
        ac.cloudTracingStartMaxDistance = tuneParams.tracingStartMaxDistanceKm;
        ac.cloudMarchStepMinKm = tuneParams.stepTMinKm;
        ac.cloudMarchStepMaxKm = tuneParams.stepTMaxKm;
        ac.cloudMarchStepHardMax = tuneParams.marchStepHardMax;
        ac.cloudMarchEmptySkip = tuneParams.emptySkipEnabled ? 1 : 0;
        ac.cloudMarchEmptyStepKm = tuneParams.emptyStepKm;
        ac.cloudMarchEmptyTauMax = tuneParams.emptyStepJitter;
        ac.cloudMarchEdgeRefine = tuneParams.edgeRefineEnabled ? 1 : 0;
        ac.cloudMarchEdgeScaleMin = tuneParams.edgeScaleMin;
        ac.cloudMarchEdgeGradLow = tuneParams.edgeGradLow;
        ac.cloudMarchEdgeGradHigh = tuneParams.edgeGradHigh;
        ac.cloudLightStepMul = tuneParams.lightStepMul;
        ac.cloudLightBasicStep = tuneParams.lightBasicStepKm;
        ac.cloudLightStepNum = tuneParams.lightStepNum;
        ac.cloudWeatherUVScale[0] = ac.cloudWeatherUVScale[1] = tuneParams.weatherUVScale;
        ac.cloudCurlUVScale = tuneParams.curlUVScale;
        ac.cloudCurlStrength = tuneParams.curlStrength;
        ac.cloudCurlContrast = tuneParams.curlContrast;
        ac.cloudDirection[0] = tuneParams.windDirX;
        ac.cloudDirection[1] = tuneParams.windDirY;
        ac.cloudDirection[2] = tuneParams.windDirZ;
        ac.cloudFogFade = tuneParams.fogFade;
        ac.cloudNoiseScale = tuneParams.groundNoiseScale;
        ac.cloudEnableGroundContribution = tuneParams.enableGroundContribution ? 1 : 0;
        ac.cloudAlbedo[0] = ac.cloudAlbedo[1] = ac.cloudAlbedo[2] = tuneParams.cloudAlbedo;
        ac.cloudAirPerspectiveEnabled = tuneParams.airPerspectiveEnabled ? 1 : 0;
        ac.cloudAirPerspectiveScale = tuneParams.airPerspectiveScale;
        ac.cloudAirPerspectiveSunScale = tuneParams.airPerspectiveSunScale;
        ac.cloudAmbientExposure = tuneParams.ambientExposure;
        ac.cloudAmbientAnalyticBlend = tuneParams.ambientAnalyticBlend;
        ac.cloudIrradianceBakeAtShell = 1;

        ac.cloudGodRay = tuneParams.godRayEnabled ? 1 : 0;
        ac.cloudGodRayScale = tuneParams.godRayScale;

        // ── 双层云：Profile LUT 类型轴 + 高空云 ──
        ac.cloudProfileTypeBias = tuneParams.profileTypeBias;
        ac.cloudProfileTypeContrast = tuneParams.profileTypeContrast;
        ac.hiCloudCoverage = tuneParams.hiCloudEnabled ? tuneParams.hiCloudCoverage : 0.0f;
        ac.hiCloudBottom = tuneParams.hiCloudBottom;
        ac.hiCloudTop = tuneParams.hiCloudTop;
        ac.hiCloudDensity = tuneParams.hiCloudDensity;
        ac.hiCloudThreshold = tuneParams.hiCloudThreshold;
        ac.hiCloudSoftness = tuneParams.hiCloudSoftness;
        ac.hiCloudType = tuneParams.hiCloudType;

        return fd;
    }

    bool createSamplers(VulkanContext& ctx) {
        auto make = [&](VkFilter filt, VkSamplerAddressMode addr, VkBorderColor border, bool mipLinear) -> VkSampler {
            VkSamplerCreateInfo sci{ VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO };
            sci.magFilter = filt; sci.minFilter = filt;
            sci.addressModeU = sci.addressModeV = sci.addressModeW = addr;
            sci.borderColor = border;
            sci.mipmapMode = mipLinear ? VK_SAMPLER_MIPMAP_MODE_LINEAR : VK_SAMPLER_MIPMAP_MODE_NEAREST;
            sci.maxLod = mipLinear ? VK_LOD_CLAMP_NONE : 0.25f;
            VkSampler s = VK_NULL_HANDLE;
            vkCreateSampler(ctx.device, &sci, nullptr, &s);
            return s;
        };
        // 顺序须与 shared_sampler.glsl 的 binding 0-9 完全一致。
        samplers10[0] = make(VK_FILTER_NEAREST, VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE, VK_BORDER_COLOR_FLOAT_TRANSPARENT_BLACK, false); // pointClampEdge
        samplers10[1] = make(VK_FILTER_NEAREST, VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_BORDER, VK_BORDER_COLOR_FLOAT_TRANSPARENT_BLACK, false); // pointClampBorder0000
        samplers10[2] = make(VK_FILTER_NEAREST, VK_SAMPLER_ADDRESS_MODE_REPEAT, VK_BORDER_COLOR_FLOAT_TRANSPARENT_BLACK, false); // pointRepeat
        samplers10[3] = make(VK_FILTER_LINEAR,  VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE, VK_BORDER_COLOR_FLOAT_TRANSPARENT_BLACK, false); // linearClampEdge
        samplers10[4] = make(VK_FILTER_LINEAR,  VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_BORDER, VK_BORDER_COLOR_FLOAT_TRANSPARENT_BLACK, false); // linearClampBorder0000
        samplers10[5] = make(VK_FILTER_LINEAR,  VK_SAMPLER_ADDRESS_MODE_REPEAT, VK_BORDER_COLOR_FLOAT_TRANSPARENT_BLACK, false); // linearRepeat
        samplers10[6] = make(VK_FILTER_LINEAR,  VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_BORDER, VK_BORDER_COLOR_FLOAT_OPAQUE_WHITE, false); // linearClampBorder1111
        samplers10[7] = make(VK_FILTER_NEAREST, VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_BORDER, VK_BORDER_COLOR_FLOAT_OPAQUE_WHITE, false); // pointClampBorder1111
        samplers10[8] = make(VK_FILTER_LINEAR,  VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE, VK_BORDER_COLOR_FLOAT_TRANSPARENT_BLACK, true);  // linearClampEdgeMipFilter
        samplers10[9] = make(VK_FILTER_LINEAR,  VK_SAMPLER_ADDRESS_MODE_REPEAT, VK_BORDER_COLOR_FLOAT_TRANSPARENT_BLACK, true);  // linearRepeatMipFilter
        for (VkSampler s : samplers10) if (!s) return false;

        linearRepeatSampler = samplers10[5];
        linearClampSampler  = samplers10[3];
        pointClampSampler   = samplers10[0];
        return true;
    }

    bool createStorage2D(VulkanContext& ctx, VkExtent2D ext, VkFormat fmt, VkImage& img, VmaAllocation& alloc, VkImageView& view) {
        VkImageCreateInfo ici{ VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO };
        ici.imageType = VK_IMAGE_TYPE_2D; ici.format = fmt; ici.extent = { ext.width, ext.height, 1 };
        ici.mipLevels = 1; ici.arrayLayers = 1; ici.samples = VK_SAMPLE_COUNT_1_BIT;
        ici.tiling = VK_IMAGE_TILING_OPTIMAL;
        ici.usage = VK_IMAGE_USAGE_STORAGE_BIT | VK_IMAGE_USAGE_SAMPLED_BIT;
        ici.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
        VmaAllocationCreateInfo aci{}; aci.usage = VMA_MEMORY_USAGE_GPU_ONLY;
        if (vmaCreateImage(ctx.allocator, &ici, &aci, &img, &alloc, nullptr) != VK_SUCCESS) return false;

        VkImageViewCreateInfo vci{ VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO };
        vci.image = img; vci.viewType = VK_IMAGE_VIEW_TYPE_2D; vci.format = fmt;
        vci.subresourceRange = { VK_IMAGE_ASPECT_COLOR_BIT, 0,1,0,1 };
        if (vkCreateImageView(ctx.device, &vci, nullptr, &view) != VK_SUCCESS) return false;

        VkCommandBuffer cmd = beginSingleTimeCommands(ctx);
        transitionImage(cmd, img, VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_GENERAL,
            VK_PIPELINE_STAGE_2_TOP_OF_PIPE_BIT, VK_ACCESS_2_NONE,
            VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT, VK_ACCESS_2_SHADER_READ_BIT | VK_ACCESS_2_SHADER_WRITE_BIT);
        endSingleTimeCommands(ctx, cmd);
        return true;
    }

    bool createImages(VulkanContext& ctx) {
        if (!createStorage2D(ctx, quarterExt, kColorFmt, quarterColorImg, quarterColorAlloc, quarterColorView)) return false;
        if (!createStorage2D(ctx, quarterExt, kDepthFmt, quarterDepthImg, quarterDepthAlloc, quarterDepthView)) return false;
        if (!createStorage2D(ctx, quarterExt, kColorFmt, quarterFogImg,   quarterFogAlloc,   quarterFogView))   return false;
        for (int i=0;i<2;i++) {
            if (!createStorage2D(ctx, extent, kColorFmt, fullColorImg[i], fullColorAlloc[i], fullColorView[i])) return false;
            if (!createStorage2D(ctx, extent, kDepthFmt, fullDepthImg[i], fullDepthAlloc[i], fullDepthView[i])) return false;
            if (!createStorage2D(ctx, extent, kColorFmt, fullFogImg[i],   fullFogAlloc[i],   fullFogView[i]))   return false;
        }
        if (!createStorage2D(ctx, {1,1}, VK_FORMAT_R16G16B16A16_SFLOAT, dummy2DImg, dummy2DAlloc, dummy2DView)) return false;

        // dummy 3D（Froxel 占位）
        {
            VkImageCreateInfo ici{ VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO };
            ici.imageType = VK_IMAGE_TYPE_3D; ici.format = VK_FORMAT_R16G16B16A16_SFLOAT; ici.extent = {1,1,1};
            ici.mipLevels = 1; ici.arrayLayers = 1; ici.samples = VK_SAMPLE_COUNT_1_BIT;
            ici.tiling = VK_IMAGE_TILING_OPTIMAL; ici.usage = VK_IMAGE_USAGE_SAMPLED_BIT;
            ici.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
            VmaAllocationCreateInfo aci{}; aci.usage = VMA_MEMORY_USAGE_GPU_ONLY;
            if (vmaCreateImage(ctx.allocator, &ici, &aci, &dummy3DImg, &dummy3DAlloc, nullptr) != VK_SUCCESS) return false;
            VkImageViewCreateInfo vci{ VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO };
            vci.image = dummy3DImg; vci.viewType = VK_IMAGE_VIEW_TYPE_3D; vci.format = VK_FORMAT_R16G16B16A16_SFLOAT;
            vci.subresourceRange = { VK_IMAGE_ASPECT_COLOR_BIT, 0,1,0,1 };
            if (vkCreateImageView(ctx.device, &vci, nullptr, &dummy3DView) != VK_SUCCESS) return false;
            VkCommandBuffer cmd = beginSingleTimeCommands(ctx);
            transitionImage(cmd, dummy3DImg, VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
                VK_PIPELINE_STAGE_2_TOP_OF_PIPE_BIT, VK_ACCESS_2_NONE,
                VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT, VK_ACCESS_2_SHADER_READ_BIT);
            endSingleTimeCommands(ctx, cmd);
        }

        // dummy cascade SSBO（1 个全零 GpuCascadeInfo 元素；cascadeCount=0 时永远不会被读到）
        {
            VkBufferCreateInfo bci{ VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO };
            bci.size = sizeof(GpuCascadeInfo); bci.usage = VK_BUFFER_USAGE_STORAGE_BUFFER_BIT;
            VmaAllocationCreateInfo aci{}; aci.usage = VMA_MEMORY_USAGE_GPU_ONLY;
            if (vmaCreateBuffer(ctx.allocator, &bci, &aci, &dummyCascadeBuf, &dummyCascadeAlloc, nullptr) != VK_SUCCESS) return false;
        }
        cloudRtInitialized = false;
        return true;
    }

    bool createDescriptors(VulkanContext& ctx) {
        // ── Set 1：采样器（10 个） ──
        {
            VkDescriptorSetLayoutBinding b[10]{};
            for (int i=0;i<10;i++) b[i] = { (uint32_t)i, VK_DESCRIPTOR_TYPE_SAMPLER, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr };
            VkDescriptorSetLayoutCreateInfo ci{ VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO };
            ci.bindingCount = 10; ci.pBindings = b;
            if (vkCreateDescriptorSetLayout(ctx.device, &ci, nullptr, &samplerSetLayout) != VK_SUCCESS) return false;
        }

        // ── Set 0：cloud_common.glsl binding 0-32 ──
        {
            VkDescriptorSetLayoutBinding b[33]{};
            auto img  = [&](int i, VkDescriptorType t){ b[i] = { (uint32_t)i, t, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr }; };
            img(0, VK_DESCRIPTOR_TYPE_STORAGE_IMAGE); img(1, VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE);
            img(2, VK_DESCRIPTOR_TYPE_STORAGE_IMAGE); img(3, VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE);
            img(4, VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE); img(5, VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE);
            img(6, VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE); img(7, VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE);
            img(8, VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE); img(9, VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE);
            img(10, VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE); img(11, VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE);
            img(12, VK_DESCRIPTOR_TYPE_STORAGE_IMAGE); img(13, VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE);
            img(14, VK_DESCRIPTOR_TYPE_STORAGE_IMAGE); img(15, VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE);
            img(16, VK_DESCRIPTOR_TYPE_STORAGE_IMAGE); img(17, VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE);
            img(18, VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE); img(19, VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE);
            img(20, VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE);
            b[21] = { 21, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr };
            img(22, VK_DESCRIPTOR_TYPE_STORAGE_IMAGE); img(23, VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE);
            img(24, VK_DESCRIPTOR_TYPE_STORAGE_IMAGE); img(25, VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE);
            img(26, VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE);
            img(27, VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE); // inCloudAmbientMusLut 2D
            img(28, VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE);
            b[29] = { 29, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr };
            img(30, VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE);
            img(31, VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE); // inCloudProfileLut 2D（双层云廓形）
            img(32, VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE); // inCloudHiWeather 2D（高空云天气图）

            VkDescriptorSetLayoutCreateInfo ci{ VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO };
            ci.bindingCount = 33; ci.pBindings = b;
            if (vkCreateDescriptorSetLayout(ctx.device, &ci, nullptr, &cloudSetLayout) != VK_SUCCESS) return false;
        }

        VkDescriptorSetLayout layouts[3] = { cloudSetLayout, samplerSetLayout, blueNoise.setLayout };
        VkPipelineLayoutCreateInfo plci{ VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO };
        plci.setLayoutCount = 3; plci.pSetLayouts = layouts;
        if (vkCreatePipelineLayout(ctx.device, &plci, nullptr, &cloudPipelineLayout) != VK_SUCCESS) return false;

        VkDescriptorPoolSize sizes[] = {
            { VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, 8 * FRAMES_IN_FLIGHT },
            { VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE, 25 * FRAMES_IN_FLIGHT },
            { VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1 * FRAMES_IN_FLIGHT },
            { VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1 * FRAMES_IN_FLIGHT },
            { VK_DESCRIPTOR_TYPE_SAMPLER, 10 },
        };
        VkDescriptorPoolCreateInfo pci{ VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO };
        pci.maxSets = FRAMES_IN_FLIGHT + 1; pci.poolSizeCount = 5; pci.pPoolSizes = sizes;
        if (vkCreateDescriptorPool(ctx.device, &pci, nullptr, &pool) != VK_SUCCESS) return false;

        for (int i=0;i<FRAMES_IN_FLIGHT;i++) {
            VkDescriptorSetAllocateInfo ai{ VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO };
            ai.descriptorPool = pool; ai.descriptorSetCount = 1; ai.pSetLayouts = &cloudSetLayout;
            if (vkAllocateDescriptorSets(ctx.device, &ai, &cloudSet[i]) != VK_SUCCESS) return false;
        }
        {
            VkDescriptorSetAllocateInfo ai{ VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO };
            ai.descriptorPool = pool; ai.descriptorSetCount = 1; ai.pSetLayouts = &samplerSetLayout;
            if (vkAllocateDescriptorSets(ctx.device, &ai, &samplerSet) != VK_SUCCESS) return false;

            VkDescriptorImageInfo si[10]{};
            VkWriteDescriptorSet w[10]{};
            for (int i=0;i<10;i++) {
                si[i].sampler = samplers10[i];
                w[i].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET; w[i].dstSet = samplerSet;
                w[i].dstBinding = (uint32_t)i; w[i].descriptorType = VK_DESCRIPTOR_TYPE_SAMPLER;
                w[i].descriptorCount = 1; w[i].pImageInfo = &si[i];
            }
            vkUpdateDescriptorSets(ctx.device, 10, w, 0, nullptr);
        }

        // 需要一个占位 UBO 满足首次 alloc 后 validation（真正内容在 renderPhase0 里每帧通过
        // updateCloudDescriptorSet 重写为 atmosphereLut 的 UBO，不使用这里的初值)。
        return true;
    }

    // 每帧重写 Set 0 的全部 31 个 binding（含没变化的静态资源）：简单、不容易漏绑，
    // 代价是每帧多几十次 vkUpdateDescriptorSets，对云这种低频 compute pass 可以接受。
    void updateCloudDescriptorSet(VulkanContext& ctx, VkTAA& taa, int slot, int histSlot, const GpuPerFrameData&,
                                   VkImageView shadowAtlasView, VkBuffer cascadeInfoBuf) {
        VkDescriptorSet set = cloudSet[slot];

        VkDescriptorImageInfo hdr{ VK_NULL_HANDLE, taa.colorViews[taa.currentIdx], VK_IMAGE_LAYOUT_GENERAL };
        VkDescriptorImageInfo hdrRO{ VK_NULL_HANDLE, taa.colorViews[taa.currentIdx], VK_IMAGE_LAYOUT_GENERAL };
        VkDescriptorImageInfo depth{ VK_NULL_HANDLE, taa.depthView, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL };
        VkDescriptorImageInfo gbufA{ VK_NULL_HANDLE, dummy2DView, VK_IMAGE_LAYOUT_GENERAL };
        VkDescriptorImageInfo basicN{ VK_NULL_HANDLE, basicNoiseView, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL };
        VkDescriptorImageInfo detailN{ VK_NULL_HANDLE, detailNoiseView, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL };
        VkDescriptorImageInfo weatherI{ VK_NULL_HANDLE, weatherView, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL };
        // Phase 2 已接入：真实 curl 湍流细节纹理（vk_scene.h::cloudCurlTex，见 vk_cloud_bake.h::bakeCurlNoise），
        // 之前这里绑定的是 1x1 全零占位纹理，cloudMap() 的 localCoverage 项恒为 0。
        VkDescriptorImageInfo curl{ VK_NULL_HANDLE, curlNoiseView, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL };
        VkDescriptorImageInfo transLut{ VK_NULL_HANDLE, atmosphereLut.transmittanceView(), VK_IMAGE_LAYOUT_GENERAL };
        // 空气透视：Bruneton Inscatter ToPoint（bakeFull 烤出；与场景大气同源）
        VkDescriptorImageInfo inscatter{ VK_NULL_HANDLE, atmosphereLut.inscatterLutView(), VK_IMAGE_LAYOUT_GENERAL };
        VkDescriptorImageInfo qColorRW{ VK_NULL_HANDLE, quarterColorView, VK_IMAGE_LAYOUT_GENERAL };
        VkDescriptorImageInfo qColorRO{ VK_NULL_HANDLE, quarterColorView, VK_IMAGE_LAYOUT_GENERAL };
        VkDescriptorImageInfo fColorRW{ VK_NULL_HANDLE, fullColorView[slot], VK_IMAGE_LAYOUT_GENERAL };
        VkDescriptorImageInfo fColorRO{ VK_NULL_HANDLE, fullColorView[slot], VK_IMAGE_LAYOUT_GENERAL };
        VkDescriptorImageInfo qDepthRW{ VK_NULL_HANDLE, quarterDepthView, VK_IMAGE_LAYOUT_GENERAL };
        VkDescriptorImageInfo qDepthRO{ VK_NULL_HANDLE, quarterDepthView, VK_IMAGE_LAYOUT_GENERAL };
        VkDescriptorImageInfo fDepthRW{ VK_NULL_HANDLE, fullDepthView[slot], VK_IMAGE_LAYOUT_GENERAL };
        VkDescriptorImageInfo fDepthRO{ VK_NULL_HANDLE, fullDepthView[slot], VK_IMAGE_LAYOUT_GENERAL };
        VkDescriptorImageInfo fColorHist{ VK_NULL_HANDLE, fullColorView[histSlot], VK_IMAGE_LAYOUT_GENERAL };
        VkDescriptorImageInfo fDepthHist{ VK_NULL_HANDLE, fullDepthView[histSlot], VK_IMAGE_LAYOUT_GENERAL };
        VkDescriptorImageInfo skyViewLut{ VK_NULL_HANDLE, atmosphereLut.skyViewLutView(), VK_IMAGE_LAYOUT_GENERAL };
        VkDescriptorImageInfo qFogRW{ VK_NULL_HANDLE, quarterFogView, VK_IMAGE_LAYOUT_GENERAL };
        VkDescriptorImageInfo qFogRO{ VK_NULL_HANDLE, quarterFogView, VK_IMAGE_LAYOUT_GENERAL };
        VkDescriptorImageInfo fFogRW{ VK_NULL_HANDLE, fullFogView[slot], VK_IMAGE_LAYOUT_GENERAL };
        VkDescriptorImageInfo fFogRO{ VK_NULL_HANDLE, fullFogView[slot], VK_IMAGE_LAYOUT_GENERAL };
        VkDescriptorImageInfo fFogHist{ VK_NULL_HANDLE, fullFogView[histSlot], VK_IMAGE_LAYOUT_GENERAL };
        VkDescriptorImageInfo skyIrr{ VK_NULL_HANDLE, atmosphereLut.cloudAmbientMusLutView(), VK_IMAGE_LAYOUT_GENERAL };
        // shadowAtlasView 是深度图（SHADER_READ_ONLY_OPTIMAL），采样时走 point-clamp 采样器
        // （cloud_composite.glsl 里 pointClampEdgeSampler），跟其它 SAMPLED_IMAGE 一致不需要
        // combined-image-sampler。为空（阴影系统未初始化/本帧被跳过）时退回旧的中性占位纹理。
        VkDescriptorImageInfo sdsm{ VK_NULL_HANDLE, shadowAtlasView ? shadowAtlasView : dummy2DView,
                                     shadowAtlasView ? VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL : VK_IMAGE_LAYOUT_GENERAL };
        VkDescriptorImageInfo hiz{ VK_NULL_HANDLE, dummy2DView, VK_IMAGE_LAYOUT_GENERAL };
        // ── 双层云：Profile LUT + 高空云 Hi weather map（vk_scene.h 烘焙，SHADER_READ_ONLY）──
        VkDescriptorImageInfo profileLut{ VK_NULL_HANDLE, profileLutView, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL };
        VkDescriptorImageInfo hiWeather{ VK_NULL_HANDLE, hiWeatherView, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL };
        VkDescriptorBufferInfo ubo{ atmosphereLut.frameUBO(slot), 0, sizeof(GpuPerFrameData) };
        VkDescriptorBufferInfo cascadeSsbo{ cascadeInfoBuf ? cascadeInfoBuf : dummyCascadeBuf, 0,
                                             cascadeInfoBuf ? sizeof(GpuCascadeInfo) * (size_t)VkCascadeShadowSystem::kMaxCascades
                                                            : sizeof(GpuCascadeInfo) };

        VkWriteDescriptorSet w[33]{};
        auto imgW = [&](int i, uint32_t binding, VkDescriptorType t, VkDescriptorImageInfo* info) {
            w[i].sType=VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET; w[i].dstSet=set; w[i].dstBinding=binding;
            w[i].descriptorType=t; w[i].descriptorCount=1; w[i].pImageInfo=info;
        };
        imgW(0,0,VK_DESCRIPTOR_TYPE_STORAGE_IMAGE,&hdr);         imgW(1,1,VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE,&hdrRO);
        imgW(2,2,VK_DESCRIPTOR_TYPE_STORAGE_IMAGE,&qColorRW);    imgW(3,3,VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE,&qColorRO);
        imgW(4,4,VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE,&depth);       imgW(5,5,VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE,&gbufA);
        imgW(6,6,VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE,&basicN);      imgW(7,7,VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE,&detailN);
        imgW(8,8,VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE,&weatherI);    imgW(9,9,VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE,&curl);
        imgW(10,10,VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE,&transLut);  imgW(11,11,VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE,&inscatter);
        imgW(12,12,VK_DESCRIPTOR_TYPE_STORAGE_IMAGE,&fColorRW);  imgW(13,13,VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE,&fColorRO);
        imgW(14,14,VK_DESCRIPTOR_TYPE_STORAGE_IMAGE,&qDepthRW);  imgW(15,15,VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE,&qDepthRO);
        imgW(16,16,VK_DESCRIPTOR_TYPE_STORAGE_IMAGE,&fDepthRW);  imgW(17,17,VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE,&fDepthRO);
        imgW(18,18,VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE,&fColorHist);imgW(19,19,VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE,&fDepthHist);
        imgW(20,20,VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE,&skyViewLut);
        w[21].sType=VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET; w[21].dstSet=set; w[21].dstBinding=21;
        w[21].descriptorType=VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER; w[21].descriptorCount=1; w[21].pBufferInfo=&ubo;
        imgW(22,22,VK_DESCRIPTOR_TYPE_STORAGE_IMAGE,&qFogRW);    imgW(23,23,VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE,&qFogRO);
        imgW(24,24,VK_DESCRIPTOR_TYPE_STORAGE_IMAGE,&fFogRW);    imgW(25,25,VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE,&fFogRO);
        imgW(26,26,VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE,&fFogHist);
        imgW(27,27,VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE,&skyIrr);
        imgW(28,28,VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE,&sdsm);
        w[29].sType=VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET; w[29].dstSet=set; w[29].dstBinding=29;
        w[29].descriptorType=VK_DESCRIPTOR_TYPE_STORAGE_BUFFER; w[29].descriptorCount=1; w[29].pBufferInfo=&cascadeSsbo;
        imgW(30,30,VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE,&hiz);
        imgW(31,31,VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE,&profileLut);
        imgW(32,32,VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE,&hiWeather);

        vkUpdateDescriptorSets(ctx.device, 33, w, 0, nullptr);
    }

    bool createPipelines(VulkanContext& ctx) {
        auto make = [&](const char* spv, VkPipeline& outPipe) -> bool {
            auto code = loadSPIRV(spv);
            if (code.empty()) { fprintf(stderr, "[VkCloudSystem] Missing spv: %s\n", spv); return false; }
            VkShaderModule mod = createShaderModule(ctx.device, code);
            if (mod == VK_NULL_HANDLE) return false;
            VkPipelineShaderStageCreateInfo stage{ VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO };
            stage.stage = VK_SHADER_STAGE_COMPUTE_BIT; stage.module = mod; stage.pName = "main";
            VkComputePipelineCreateInfo cpci{ VK_STRUCTURE_TYPE_COMPUTE_PIPELINE_CREATE_INFO };
            cpci.stage = stage; cpci.layout = cloudPipelineLayout;
            VkResult res = vkCreateComputePipelines(ctx.device, VK_NULL_HANDLE, 1, &cpci, nullptr, &outPipe);
            vkDestroyShaderModule(ctx.device, mod, nullptr);
            return res == VK_SUCCESS;
        };
        return make("src/render/shaders/spirv/cloud_raymarching.comp.spv", raymarchPipe)
            && make("src/render/shaders/spirv/cloud_reconstruct.comp.spv", reconstructPipe)
            && make("src/render/shaders/spirv/cloud_composite.comp.spv", compositePipe);
    }

    void destroyPipelines(VkDevice dev) {
        auto d=[&](VkPipeline& p){ if(p){vkDestroyPipeline(dev,p,nullptr);p=VK_NULL_HANDLE;} };
        d(raymarchPipe); d(reconstructPipe); d(compositePipe);
        if (cloudPipelineLayout) { vkDestroyPipelineLayout(dev, cloudPipelineLayout, nullptr); cloudPipelineLayout=VK_NULL_HANDLE; }
    }

    void destroyDescriptorsDynamic(VkDevice dev) {
        if (pool) { vkDestroyDescriptorPool(dev, pool, nullptr); pool=VK_NULL_HANDLE; }
        for (auto& s : cloudSet) s = VK_NULL_HANDLE;
        samplerSet = VK_NULL_HANDLE;
        if (cloudSetLayout) { vkDestroyDescriptorSetLayout(dev, cloudSetLayout, nullptr); cloudSetLayout=VK_NULL_HANDLE; }
    }

    void destroyImages(VulkanContext& ctx) {
        auto d=[&](VkImage& i, VmaAllocation& a, VkImageView& v){ if(v){vkDestroyImageView(ctx.device,v,nullptr);v=VK_NULL_HANDLE;} if(i){vmaDestroyImage(ctx.allocator,i,a);i=VK_NULL_HANDLE;} };
        d(quarterColorImg,quarterColorAlloc,quarterColorView);
        d(quarterDepthImg,quarterDepthAlloc,quarterDepthView);
        d(quarterFogImg,quarterFogAlloc,quarterFogView);
        for (int i=0;i<2;i++) { d(fullColorImg[i],fullColorAlloc[i],fullColorView[i]); d(fullDepthImg[i],fullDepthAlloc[i],fullDepthView[i]); d(fullFogImg[i],fullFogAlloc[i],fullFogView[i]); }
        d(dummy2DImg,dummy2DAlloc,dummy2DView);
        d(dummy3DImg,dummy3DAlloc,dummy3DView);
        cloudRtInitialized = false;
    }
};
