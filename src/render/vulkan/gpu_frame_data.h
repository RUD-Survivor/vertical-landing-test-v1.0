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
// AtmosphereConfig — 560 bytes. Bruneton2017/UE SkyAtmosphere 参数化。
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
    int32_t cloudMarchPad0 = 0;
};
static_assert(sizeof(GpuAtmosphereConfig) == 576, "GpuAtmosphereConfig must match std140 AtmosphereConfig");

// -----------------------------------------------------------------------
// SkyInfo — 656 bytes
// -----------------------------------------------------------------------
struct GpuSkyInfo {
    float color[3] = {1,1,1}; float intensity = 1.f;
    float direction[3] = {0,-1,0}; int32_t shadowType = 0;

    int32_t rayTraceShadow = 0;
    int32_t pad0 = 0, pad1 = 0, pad2 = 0;

    GpuCascadeShadowConfig cacsadeConfig;
    GpuAtmosphereConfig atmosphereConfig;
};
static_assert(sizeof(GpuSkyInfo) == 672, "GpuSkyInfo must match std140 SkyInfo");

// -----------------------------------------------------------------------
// PerFrameData — 1680 bytes. binding 21 (cloud_common.glsl) / binding 8 (atmosphere_common.glsl).
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
static_assert(sizeof(GpuPerFrameData) == 1696, "GpuPerFrameData must match std140 PerFrameData");

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
