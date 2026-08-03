#pragma once
// ==========================================================================
// gpu_frame_data.h — C++ 镜像，逐字段对应 shared_struct.glsl 的
// PerFrameData / SkyInfo / CascadeShadowConfig / AtmosphereConfig / CascadeInfo。
//
// 供 cloud_common.glsl（binding 21）与 atmosphere/atmosphere_common.glsl
// （binding 8）的 UniformFrameData UBO 使用。
//
// 布局规则：所有字段都用 float/int32_t/uint32_t 平铺（不用 glm::vec3 等），
// 字段顺序、分组严格对应 GLSL 源码里作者手工排好的 16 字节分组，
// 因此自然满足 std140 对齐规则，不需要额外 pad。每一层都有 static_assert
// 校验总大小，一旦哪个字段漏掉/顺序错了会在编译期报错，而不是运行时错位。
// ==========================================================================

#include <cstdint>
#include <cstring>

// -----------------------------------------------------------------------
// CascadeShadowConfig — 48 bytes
// -----------------------------------------------------------------------
struct GpuCascadeShadowConfig {
    int32_t  cascadeCount           = 0;
    int32_t  percascadeDimXY        = 0;
    float    cascadeSplitLambda     = 0.f;
    float    maxDrawDepthDistance   = 0.f;

    float    shadowBiasConst        = 0.f;
    float    shadowBiasSlope        = 0.f;
    float    shadowFilterSize       = 0.f;
    float    maxFilterSize          = 0.f;

    float    cascadeBorderAdopt     = 0.f;
    float    cascadeEdgeLerpThreshold = 0.f;
    float    pad0                   = 0.f;
    float    pad1                   = 0.f;
};
static_assert(sizeof(GpuCascadeShadowConfig) == 48, "GpuCascadeShadowConfig must match std140 CascadeShadowConfig");

// -----------------------------------------------------------------------
// AtmosphereConfig — 704 bytes. Bruneton2017/UE SkyAtmosphere 参数化。
// 所有距离单位 km（与 shared_struct.glsl 注释一致）。
// -----------------------------------------------------------------------
struct GpuAtmosphereConfig {
    float atmospherePreExposure = 1.f;
    float pad0 = 0.f, pad1 = 0.f, pad2 = 0.f;

    float absorptionColor[3] = {0,0,0}; float absorptionLength = 0.f;
    float rayleighScatteringColor[3] = {0,0,0}; float rayleighScatterLength = 0.f;

    float multipleScatteringFactor = 1.f;
    float miePhaseFunctionG = 0.8f;
    float bottomRadius = 6360.f;
    float topRadius = 6460.f;

    float mieScatteringColor[3] = {0,0,0}; float mieScatteringLength = 0.f;
    float mieAbsColor[3] = {0,0,0}; float mieAbsLength = 0.f;

    float mieAbsorption[3] = {0,0,0}; int32_t viewRayMarchMinSPP = 4;
    float groundAlbedo[3] = {0,0,0}; int32_t viewRayMarchMaxSPP = 14;

    // 密度 profile：2 段分段线性 tent（Bruneton 原始参数化）。
    // 本引擎只用到 .w（指数衰减 scale，[1]）与线性 tent 系数（见下方注释），
    // 与 shared_atmosphere.glsl::getAtmosphereParameters()/sampleMediumRGB() 的读法一一对应。
    float rayleighDensity[3][4] = {};   // [1].w = -1/HRayleigh (指数衰减 scale)
    float mieDensity[3][4] = {};        // [1].w = -1/HMie
    float absorptionDensity[3][4] = {}; // [0]=(layerWidth,_,_,linearTerm0) [1]=(constantTerm0,_,_,_) [2]=(constantTerm1,linearTerm1,_,_)

    float cloudAreaStartHeight = 0.f;
    float cloudAreaThickness = 0.f;
    float cloudGodRayScale = 1.f;
    float cloudShadowExtent = 1.f;

    float camWorldPos[3] = {0,0,0}; uint32_t updateFaceIndex = 0;

    float cloudSpaceViewProject[16] = {};
    float cloudSpaceViewProjectInverse[16] = {};

    float cloudWeatherUVScale[2] = {1.f,1.f}; float cloudCoverage = 0.5f; float cloudDensity = 1.f;

    float cloudShadingSunLightScale = 1.f;
    float cloudFogFade = 1.f;
    float cloudMaxTraceingDistance = 300.f;
    float cloudTracingStartMaxDistance = 1e8f;

    float cloudDirection[3] = {1,0,0}; float cloudSpeed = 1.f;

    float cloudMultiScatterExtinction = 0.5f;
    float cloudMultiScatterScatter = 0.5f;
    float cloudBasicNoiseScale = 1.f;
    float cloudDetailNoiseScale = 1.f;

    float cloudAlbedo[3] = {1,1,1}; float cloudPhaseForward = 0.8f;

    float cloudPhaseBackward = -0.2f;
    float cloudPhaseMixFactor = 0.5f;
    float cloudPowderScale = 1.f;
    float cloudPowderPow = 1.f;

    // 0=Off 1=Legacy 2=Sugar；默认 Legacy，与接入堆糖前行为一致
    int32_t cloudPowderMode = 1;
    int32_t cloudPowderSugarAllMs = 0;
    float cloudPowderSugarGain = 2.f;
    float cloudPowderSugarViewWeight = 1.f;

    float cloudLightStepMul = 1.5f;
    float cloudLightBasicStep = 0.033f;
    int32_t cloudLightStepNum = 6;
    int32_t cloudEnableGroundContribution = 1;

    int32_t cloudMarchingStepNum = 64;
    int32_t cloudSunLitMapOctave = 1;
    float cloudNoiseScale = 1.f;
    int32_t cloudGodRay = 0;

    float cloudMarchStepMinKm = 0.05f;
    float cloudMarchStepMaxKm = 0.5f;
    int32_t cloudMarchStepHardMax = 2048;
    int32_t cloudMarchEmptySkip = 1;
    float cloudMarchEmptyStepKm = 1.0f;
    float cloudMarchEmptyTauMax = 0.25f;
    int32_t cloudMarchEdgeRefine = 1;
    float cloudMarchEdgeScaleMin = 0.25f;
    float cloudMarchEdgeGradLow = 0.002f;
    float cloudMarchEdgeGradHigh = 0.018f;

    int32_t cloudAirPerspectiveEnabled = 1;
    float cloudAirPerspectiveScale = 0.01f;
    float cloudAirPerspectiveSunScale = 1.f; // 相对 L_sun；默认 1=与云直射同源
    float cloudAmbientExposure = 5.f;
    float cloudAmbientAnalyticBlend = 0.f;
    int32_t cloudIrradianceBakeAtShell = 1;
    // Curl / localCoverage（原 padAmbient* 槽位；见 cloud_common.glsl::cloudMap）
    float cloudCurlUVScale = 0.001f;  // UV = km * scale；旧硬编码 0.001
    float cloudCurlStrength = 0.0f;   // 叠到 weather.x 的幅度；0=关闭
    float cloudCurlContrast = 3.0f;   // saturate(x*c - c*0.25)*strength；旧值 3 → bias -0.75

    // 噪声频带阶梯（见 shared_struct.glsl 同名字段与 cloud_common.glsl::sampleCloudShapeNoise）。
    // scale 单位 1/km；幅度以 Basic=1 为基准。全 0 = 单档 Basic + 无侵蚀。
    float cloudMesoNoiseScale = 0.f;
    float cloudMesoAmplitude = 0.f;
    float cloudFineNoiseScale = 0.f;
    float cloudFineAmplitude = 0.f;

    float cloudDetailAmplitude = 0.f;
    // 频带 LOD（见 shared_struct.glsl 同名字段与 cloud_noise_lod.glsl）。
    float cloudNoiseLodNMin = 0.f;
    float cloudNoiseLodShadowDistScale = 1.f;
    int32_t cloudNoiseLodEnabled = 0; // 默认关：未显式配置的路径保持全档常开

    // phi_fwd 漫射场（见 shared_struct.glsl 同名字段与 cloud_common.glsl::volumetricShadow）。
    // Intensity 默认 0：未显式开启的路径（LUT 烘焙 / 旧配置）行为与接入前完全一致。
    float cloudPhiFwdIntensity = 0.f;
    float cloudPhiFwdMsBuildScale = 1.f;
    float cloudPhiFwdCompress = 0.f;
    float cloudPhiFwdDepthPow = 1.f;
    float cloudPhiFwdDepthBias = 0.f;
    float cloudPhiFwdBoundaryConfidence = 1.f;

    // ── 双层云：Profile LUT 类型轴 + 高空云（见 shared_struct.glsl 同名字段）──
    // Profile LUT：U=归一化高度 V=径向 RGB=Cu/Tcu/Cb；廓形即 LUT 曲线本身。
    float cloudProfileTypeBias = 0.f;
    float cloudProfileTypeContrast = 1.f;
    float cloudProfilePad0 = 0.f;   // 原 cloudProfileScale（已移除语义，仅保 std140 布局）
    float cloudProfilePad1 = 0.f;
    // 高空云：R=HiMask G=HiType(As/Ac)，纯 2D 密度，带 [hiCloudBottom, hiCloudTop]。
    float hiCloudCoverage = 1.f;
    float hiCloudBottom = 0.55f;
    float hiCloudTop = 0.95f;
    float hiCloudDensity = 1.f;
    float hiCloudThreshold = 0.05f;
    float hiCloudSoftness = 0.2f;
    float hiCloudType = 1.f;
    // 原 hiCloudPad0 + cloudProfilePad2/3/4（16B）已挪到 cloudPowderMode 块，保 752B。
};
static_assert(sizeof(GpuAtmosphereConfig) == 752, "GpuAtmosphereConfig must match std140 AtmosphereConfig");

// -----------------------------------------------------------------------
// SkyInfo — 848 bytes (16+16+16+48+752)
// -----------------------------------------------------------------------
struct GpuSkyInfo {
    float color[3] = {1,1,1}; float intensity = 1.f;
    float direction[3] = {0,-1,0}; int32_t shadowType = 0;

    int32_t rayTraceShadow = 0;
    int32_t pad0 = 0, pad1 = 0, pad2 = 0;

    GpuCascadeShadowConfig cacsadeConfig;
    GpuAtmosphereConfig atmosphereConfig;
};
static_assert(sizeof(GpuSkyInfo) == 848, "GpuSkyInfo must match std140 SkyInfo");

// -----------------------------------------------------------------------
// PerFrameData — 1872 bytes. binding 21 (cloud_common.glsl) / binding 8 (atmosphere_common.glsl).
// -----------------------------------------------------------------------
struct GpuPerFrameData {
    float appTime[4] = {};       // .x runtime .y sin .z cos .w pad
    uint32_t frameIndex4[4] = {}; // .x frame count .y %8 .z %16 .w %32

    float camWorldPos[4] = {};   // 大气空间坐标（相对当前云所在行星中心，km）
    float camForward[4] = {};

    float camInfo[4] = {};       // .x fovy .y aspect .z nearZ .w farZ
    float camInfoPrev[4] = {};

    float camView[16] = {};
    float camProj[16] = {};
    float camViewProj[16] = {};

    float camInvertView[16] = {};
    float camInvertProj[16] = {};
    float camInvertViewProj[16] = {};

    float camProjNoJitter[16] = {};
    float camViewProjNoJitter[16] = {};

    float camInvertProjNoJitter[16] = {};
    float camInvertViewProjNoJitter[16] = {};

    float camViewProjPrev[16] = {};
    float camViewProjPrevNoJitter[16] = {};

    float frustumPlanes[6][4] = {};

    float jitterData[4] = {};

    uint32_t jitterPeriod = 1;
    uint32_t bEnableJitter = 0;
    float    basicTextureLODBias = 0.f;
    uint32_t bCameraCut = 0;

    uint32_t skyValid = 1;
    uint32_t skySDSMValid = 0;
    float    fixExposure = 1.f;
    uint32_t bAutoExposure = 0;

    float renderWidth = 0.f, renderHeight = 0.f, displayWidth = 0.f, displayHeight = 0.f;

    GpuSkyInfo sky;
};
static_assert(sizeof(GpuPerFrameData) == 1872, "GpuPerFrameData must match std140 PerFrameData");

// -----------------------------------------------------------------------
// CascadeInfo — 176 bytes. SDSM cascade SSBO 元素（binding 29）。
// Phase 2 待接入真正的级联阴影渲染；本次只绑定一个全零占位元素。
// -----------------------------------------------------------------------
struct GpuCascadeInfo {
    float viewProj[16] = {};
    float frustumPlanes[6][4] = {};
    float cascadeScale[4] = {};
};
static_assert(sizeof(GpuCascadeInfo) == 176, "GpuCascadeInfo must match std140 CascadeInfo");
