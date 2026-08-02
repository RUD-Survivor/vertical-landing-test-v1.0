#pragma once
// ==========================================================================
// vk_atmosphere_lut.h — 实时大气 LUT 栈：Transmittance / MultiScatter / SkyView
// + Sky Irradiance cubemap。
//
// 对应 shader: src/render/shaders/atmosphere/{transmittance_lut,multi_scatter_lut,
// skyview_lut,sky_irradiance_capture}.glsl，四者共用同一套 descriptor set 布局
// （见 atmosphere/atmosphere_common.glsl）。
//
// ── 架构（按星球缓存，非"每帧四 pass"）──────────────────────────────────────
// 本引擎相机会在同一飞行任务里从行星地表飞到深空，一度让人以为"大气 profile
// 随相机位置变化"必须每帧对当前行星重新烘焙全部四个 pass——但实际上只有
// Sky-View LUT / Sky-Irradiance cubemap 的求值把"相机高度"当参数直接烤了
// 进去，需要逐帧刷新（这两张表分辨率很小，逐帧刷新很便宜）；Transmittance LUT
// 和 MultiScatter LUT 的参数化本身是 (高度, 天顶角)，只要大气 profile（Rayleigh/
// Mie/臭氧系数、行星半径）不变，相机飞到哪里这两张表依然有效，不需要每帧重烤。
// 所以这里改成仿 KSP Scatterer 插件的思路（仅借鉴架构，不照抄代码/许可证不同）：
// 每个星球一份 LUT 资源实例，按 bodyIdx 缓存（VkAtmoLutCache），Transmittance/
// MultiScatter 只在"profile 指纹"变化时重烤（首次遇到该星球，或调参改了系数），
// Sky-View/Irradiance 由调用方每帧对"这一帧实际要渲染"的星球单独刷新。
//
// 四个 compute pipeline（纯 shader 逻辑，和"哪个星球"无关）被拆到 VkAtmoLutPipelines，
// 全引擎只初始化一份，所有星球的 VkAtmosphereLut 实例共用；VkAtmosphereLut 本身瘦身
// 成"单星球的 LUT 资源实例"（images + 每帧 UBO + descriptor set），不再自带 pipeline。
// ==========================================================================

#include "vk_context.h"
#include "vk_pipeline.h"
#include "vk_utils.h"
#include "vk_frame.h"
#include "gpu_frame_data.h"
#include "cloud_noise_ladder.h"

#include <cstdio>
#include <cstring>
#include <cmath>
#include <unordered_map>

// 大气重构·可调参数（imgui_atmo_tuner.h 面板）。shellThicknessKm 只在 C++ 侧用于
// 算 apc.outerRadius（替代原来硬编码的 +160.0f）；其余三个直接塞进 AtmoPushConstants
// 尾部（vk_descriptors.h），壳内/壳外 shader 直接读，不再是 shader 里的魔法数字。
struct AtmoTuneParams {
    float shellThicknessKm = 160.0f;  // 大气壳厚度（曾经是所有星球统一硬编码 160km）
    float spaceVisStart    = 0.55f;   // altNorm 达到此值开始透出星空/深空色（旧路径；统一合成后保留）
    float spaceVisEnd      = 1.0f;    // altNorm 达到此值完全是深空（星空全透出）
    float limbBrightness   = 7.0f;    // 大气顶及以外：边缘增益默认值（rim=1）
    // 壳外整体曝光倍率：与壳内共用 atmoUnifiedExposure；建议 InnerFar≈Outer 作基准。
    float outerExposure    = 5.0f;
    // 壳内曝光：贴地 Near，近大气顶 Far；与 Outer 对齐可减少穿壳跳变。
    float innerExposureNear = 10.0f;
    float innerExposureFar  = 5.0f;
    float limbSpaceStart  = 1.3f;   // 保留布局，当前未用
    float limbSpaceEnd    = 3.0f;   // 保留布局，当前未用
    float limbBrightnessBottom = 1.0f; // 大气底（altNorm=0）边缘增益，smoothstep 收到顶
    float limbPower       = 2.0f;   // rim^power，越大边缘越尖
    // atmo_inside：sceneDepth >= 阈值走 Sky-View LUT，否则 raymarch 雾（1.0=仅 clear depth）
    float skyDepthThreshold = 1.0f;

    // ── Transmittance LUT / 大气 profile（改动会触发重烤 T+MultiScatter）──
    // 乘在 getPlanetScatteringCoeffs() 结果上；1=星球默认。
    float rayleighMul = 1.0f;
    float mieMul      = 1.0f;
    float ozoneMul    = 1.0f;
    float hRayleighMul = 1.0f;
    float hMieMul      = 1.0f;
    float gMieOverride = -1.0f;   // <0 用星球默认；否则覆盖 Mie g
    float groundAlbedo = 0.3f;
    float multipleScatteringFactor = 1.0f;
    float mieAbsFraction = 0.10f; // Mie 吸收 ≈ mieCoeff * 此值
    // scene×T：1=全 RGB LUT 透射；0=仅亮度（不偏暖）
    float transmittanceChromaStrength = 1.0f;
    // Phase A debug：>0 时 atmo_inside 用 Inscatter 天空采样×此值覆盖输出（0=关）
    float inscatterDebugExposure = 0.0f;

    // ── 渲染算法选择（F4 tuner；不触发 LUT 重烤）────────────────────────────
    // 段散射：壳外全部像素 + 壳内有场景深度。0=Inscatter ToPoint，1=Raymarch。
    int segmentScatterMode = 0;
    // 壳内天空像素（clear depth）。0=Sky-View LUT，1=Inscatter 天空查表。
    int skyScatterMode = 0;
    // Inscatter 单次 Mie：0=官方 combined A=Mie.r 外推；1=独立完整 RGB 3D LUT。
    int mieStorageMode = 0;
    // 壳外盘面：0=Disk AP（全深）；1=legacy discard（盘心无 AP）；
    // 2=出大气后 Depth↔解析球渐进混合（消远距同心纹，近地保留起伏）。
    int shellDiskMode = 0;
    // mode=2：地心距 camR 在 [outer+start, outer+end] 上 wDepth 从 1→0。
    float depthApBlendStartKm = 0.0f;    // 相对大气顶 outerRadius 的起点（km）
    float depthApBlendEndKm   = 1000.0f; // 相对大气顶的终点（km）；再远全解析球

    // ── ToPoint 内部量 debug（壳内/壳外；>0 覆盖合成输出）──────────────────
    // 0=关 1=S0 2=S1 3=T 4=T*S1 5=S0-T*S1(相位前) 6=最终phased 7=段长d 8=hitGround
    int   toPointDebugMode = 0;
    float toPointDebugExposure = 10.0f;
};

// ── 单一数据源：每类行星的 Rayleigh/Mie/臭氧散射系数 ─────────────────────────
// 曾经有两份手抄拷贝：这里（喂给 Bruneton LUT 烘焙）和 atmo.frag::setupPlanetProfile()
// （喂给旧版全屏 raymarch，已删除，改成直接从 AtmoPushConstants 读这里算出来的值，
// 见 vk_renderer3d.h 填充 apc.rayleighCoeff 等字段处）。现在两条消费路径
// （LUT 烘焙 / 壳内壳外新 shader 的 push constant）都只从这一个函数取数，不再重复。
struct PlanetScatteringCoeffs {
    float rayleighCoeff[3]; float mieCoeff;
    float hRayleigh, hMie, gMie;
    float ozoneCoeff[3]; float ozoneCenter, ozoneWidth;
};

// 把 F4 profile 倍率/覆盖叠到星球默认系数上（raymarch push constant 与 LUT 烘焙共用）。
inline void applyAtmoTuneToCoeffs(PlanetScatteringCoeffs& c, const AtmoTuneParams& t) {
    for (int i = 0; i < 3; i++) c.rayleighCoeff[i] *= t.rayleighMul;
    c.mieCoeff *= t.mieMul;
    for (int i = 0; i < 3; i++) c.ozoneCoeff[i] *= t.ozoneMul;
    c.hRayleigh *= t.hRayleighMul;
    c.hMie      *= t.hMieMul;
    if (t.gMieOverride >= 0.0f) c.gMie = t.gMieOverride;
}

// LUT 缓存指纹：半径/atmoIdx + profile 调参；任一变化 → bakeFull。
struct AtmoLutProfileKey {
    int32_t atmoIdx = -1;
    float radiusKm = 0.f;
    float thicknessKm = 0.f;
    float rayleighMul = 1.f;
    float mieMul = 1.f;
    float ozoneMul = 1.f;
    float hRayleighMul = 1.f;
    float hMieMul = 1.f;
    float gMieOverride = -1.f;
    float groundAlbedo = 0.3f;
    float multipleScatteringFactor = 1.f;
    float mieAbsFraction = 0.1f;
    // 烘焙语义修订号：改 Inscatter 烘焙公式时 bump，强制全量 bakeFull。
    float inscatterBakeRev = 4.f; // 4 = 单位太阳烘焙（globalL=sky.color，运行时乘 intensity）

    static AtmoLutProfileKey fromTune(int32_t atmoIdx, float radiusKm, float thicknessKm,
                                      const AtmoTuneParams& t) {
        AtmoLutProfileKey k{};
        k.atmoIdx = atmoIdx;
        k.radiusKm = radiusKm;
        k.thicknessKm = thicknessKm;
        k.rayleighMul = t.rayleighMul;
        k.mieMul = t.mieMul;
        k.ozoneMul = t.ozoneMul;
        k.hRayleighMul = t.hRayleighMul;
        k.hMieMul = t.hMieMul;
        k.gMieOverride = t.gMieOverride;
        k.groundAlbedo = t.groundAlbedo;
        k.multipleScatteringFactor = t.multipleScatteringFactor;
        k.mieAbsFraction = t.mieAbsFraction;
        k.inscatterBakeRev = 4.f;
        return k;
    }

    bool differsFrom(const AtmoLutProfileKey& o) const {
        auto ne = [](float a, float b) { return fabsf(a - b) > 1e-4f; };
        return atmoIdx != o.atmoIdx
            || ne(radiusKm, o.radiusKm) || ne(thicknessKm, o.thicknessKm)
            || ne(rayleighMul, o.rayleighMul) || ne(mieMul, o.mieMul) || ne(ozoneMul, o.ozoneMul)
            || ne(hRayleighMul, o.hRayleighMul) || ne(hMieMul, o.hMieMul)
            || ne(gMieOverride, o.gMieOverride)
            || ne(groundAlbedo, o.groundAlbedo)
            || ne(multipleScatteringFactor, o.multipleScatteringFactor)
            || ne(mieAbsFraction, o.mieAbsFraction)
            || ne(inscatterBakeRev, o.inscatterBakeRev);
    }
};

inline PlanetScatteringCoeffs getPlanetScatteringCoeffs(int32_t atmoIdx) {
    PlanetScatteringCoeffs c{};
    if (atmoIdx == 3) { // 地球类：强 Rayleigh、薄臭氧
        c.rayleighCoeff[0]=5.8e-3f*1.2f; c.rayleighCoeff[1]=13.5e-3f*1.2f; c.rayleighCoeff[2]=33.1e-3f*1.2f;
        c.mieCoeff = 5.0e-3f; c.hRayleigh = 8.0f; c.hMie = 1.0f; c.gMie = 0.8f;
        c.ozoneCoeff[0]=0.35e-3f; c.ozoneCoeff[1]=0.85e-3f; c.ozoneCoeff[2]=0.09e-3f;
        c.ozoneCenter = 25.0f; c.ozoneWidth = 15.0f;
    } else if (atmoIdx == 2) { // 厚大气类：强 Mie、厚臭氧
        c.rayleighCoeff[0]=5.0e-3f; c.rayleighCoeff[1]=12.0e-3f; c.rayleighCoeff[2]=28.0e-3f;
        c.mieCoeff = 0.04f; c.hRayleigh = 15.0f; c.hMie = 5.0f; c.gMie = 0.76f;
        c.ozoneCoeff[0]=0.5e-3f; c.ozoneCoeff[1]=5.0e-3f; c.ozoneCoeff[2]=20.0e-3f;
        c.ozoneCenter = 50.0f; c.ozoneWidth = 20.0f;
    } else if (atmoIdx == 5) { // 火星类：稀薄、偏红（弱 Rayleigh、无臭氧、中等 Mie 尘霾）
        c.rayleighCoeff[0]=10.0e-3f; c.rayleighCoeff[1]=5.0e-3f; c.rayleighCoeff[2]=1.0e-3f;
        c.mieCoeff = 0.015f; c.hRayleigh = 11.0f; c.hMie = 3.0f; c.gMie = 0.85f;
        c.ozoneCoeff[0]=c.ozoneCoeff[1]=c.ozoneCoeff[2]=0.0f;
        c.ozoneCenter = 1.0f; c.ozoneWidth = 1.0f;
    } else if (atmoIdx == 6) { // 木星类：厚气体巨行星
        c.rayleighCoeff[0]=18.0e-3f; c.rayleighCoeff[1]=15.0e-3f; c.rayleighCoeff[2]=12.0e-3f;
        c.mieCoeff = 0.01f; c.hRayleigh = 27.0f; c.hMie = 10.0f; c.gMie = 0.8f;
        c.ozoneCoeff[0]=c.ozoneCoeff[1]=c.ozoneCoeff[2]=0.0f;
        c.ozoneCenter = 1.0f; c.ozoneWidth = 1.0f;
    } else if (atmoIdx == 7) { // 土星类
        c.rayleighCoeff[0]=15.0e-3f; c.rayleighCoeff[1]=13.0e-3f; c.rayleighCoeff[2]=9.0e-3f;
        c.mieCoeff = 0.01f; c.hRayleigh = 60.0f; c.hMie = 15.0f; c.gMie = 0.8f;
        c.ozoneCoeff[0]=c.ozoneCoeff[1]=c.ozoneCoeff[2]=0.0f;
        c.ozoneCenter = 1.0f; c.ozoneWidth = 1.0f;
    } else if (atmoIdx == 8) { // 天王星类
        c.rayleighCoeff[0]=2.0e-3f; c.rayleighCoeff[1]=15.0e-3f; c.rayleighCoeff[2]=25.0e-3f;
        c.mieCoeff = 0.002f; c.hRayleigh = 28.0f; c.hMie = 5.0f; c.gMie = 0.8f;
        c.ozoneCoeff[0]=1.0e-3f; c.ozoneCoeff[1]=0.0f; c.ozoneCoeff[2]=0.0f;
        c.ozoneCenter = 20.0f; c.ozoneWidth = 10.0f;
    } else if (atmoIdx == 9) { // 海王星类
        c.rayleighCoeff[0]=1.0e-3f; c.rayleighCoeff[1]=8.0e-3f; c.rayleighCoeff[2]=35.0e-3f;
        c.mieCoeff = 0.001f; c.hRayleigh = 20.0f; c.hMie = 5.0f; c.gMie = 0.8f;
        c.ozoneCoeff[0]=1.0e-3f; c.ozoneCoeff[1]=0.0f; c.ozoneCoeff[2]=0.0f;
        c.ozoneCenter = 20.0f; c.ozoneWidth = 10.0f;
    } else { // 默认地球类，无臭氧
        c.rayleighCoeff[0]=5.8e-3f; c.rayleighCoeff[1]=13.5e-3f; c.rayleighCoeff[2]=33.1e-3f;
        c.mieCoeff = 0.005f; c.hRayleigh = 8.0f; c.hMie = 1.0f; c.gMie = 0.8f;
        c.ozoneCoeff[0]=c.ozoneCoeff[1]=c.ozoneCoeff[2]=0.0f;
        c.ozoneCenter = 1.0f; c.ozoneWidth = 1.0f;
    }
    return c;
}

// 按行星索引（AtmoPushConstants::planetIdx）填充 Bruneton 参数化的大气 profile。
// 系数来自上面唯一的 getPlanetScatteringCoeffs()，这里只做"翻译成 Bruneton LUT
// 需要的 密度 profile 参数"这一步。
inline void fillPlanetAtmosphereProfile(GpuAtmosphereConfig& cfg, int32_t atmoIdx,
                                        float surfaceRadiusKm, float atmosphereThicknessKm,
                                        float cloudMinAltKm, float cloudMaxAltKm, float cloudExtinction,
                                        const AtmoTuneParams* tune = nullptr) {
    PlanetScatteringCoeffs pc = getPlanetScatteringCoeffs(atmoIdx);
    if (tune) applyAtmoTuneToCoeffs(pc, *tune);
    const float (&rayleighCoeff)[3] = pc.rayleighCoeff;
    const float mieCoeff = pc.mieCoeff;
    const float hRayleigh = pc.hRayleigh, hMie = pc.hMie, gMie = pc.gMie;
    const float (&ozoneCoeff)[3] = pc.ozoneCoeff;
    const float ozoneCenter = pc.ozoneCenter, ozoneWidth = pc.ozoneWidth;

    cfg.atmospherePreExposure = 1.0f;
    cfg.bottomRadius = surfaceRadiusKm;
    cfg.topRadius    = surfaceRadiusKm + atmosphereThicknessKm;
    cfg.multipleScatteringFactor = tune ? tune->multipleScatteringFactor : 1.0f;
    cfg.miePhaseFunctionG = gMie;
    cfg.viewRayMarchMinSPP = 4;
    cfg.viewRayMarchMaxSPP = 14;
    const float albedo = tune ? tune->groundAlbedo : 0.3f;
    cfg.groundAlbedo[0] = cfg.groundAlbedo[1] = cfg.groundAlbedo[2] = albedo;

    // color*length 分解：直接把系数塞进 color，length 恒为 1，与
    // shared_atmosphere.glsl::getAtmosphereParameters() 的读法（color*length）对应。
    for (int i=0;i<3;i++) cfg.rayleighScatteringColor[i] = rayleighCoeff[i];
    cfg.rayleighScatterLength = 1.0f;
    for (int i=0;i<3;i++) cfg.mieScatteringColor[i] = mieCoeff;
    cfg.mieScatteringLength = 1.0f;
    // Mie 吸收取散射的比例（F4 mieAbsFraction，默认 10%）。
    const float mieAbsFrac = tune ? tune->mieAbsFraction : 0.10f;
    for (int i=0;i<3;i++) cfg.mieAbsColor[i] = mieCoeff * mieAbsFrac;
    cfg.mieAbsLength = 1.0f;
    for (int i=0;i<3;i++) cfg.absorptionColor[i] = ozoneCoeff[i];
    cfg.absorptionLength = 1.0f;

    // 指数衰减 scale：density = exp(-height/H) ⇒ expScale = -1/H。
    // 只用到 rayleighDensity[1].w / mieDensity[1].w（见 shared_atmosphere.glsl）。
    cfg.rayleighDensity[1][3] = -1.0f / hRayleigh;
    cfg.mieDensity[1][3]      = -1.0f / hMie;

    // 臭氧层用两段线性 tent 近似 rs3d 的高斯型 ozoneDensity()：
    // [center-width, center] 段从 0 线性升到 1，[center, center+width] 段从 1 降到 0。
    // absorptionDensity0LayerWidth 是分段断点（取在 center，见推导）。
    cfg.absorptionDensity[0][0] = ozoneCenter;                    // layerWidth（断点）
    cfg.absorptionDensity[0][3] = 1.0f / ozoneWidth;              // linearTerm0
    cfg.absorptionDensity[1][0] = 1.0f - ozoneCenter / ozoneWidth; // constantTerm0
    cfg.absorptionDensity[2][0] = -1.0f / ozoneWidth;             // linearTerm1
    cfg.absorptionDensity[2][1] = 1.0f + ozoneCenter / ozoneWidth; // constantTerm1

    cfg.cloudAreaStartHeight = surfaceRadiusKm + cloudMinAltKm;
    cfg.cloudAreaThickness   = cloudMaxAltKm - cloudMinAltKm;
    cfg.cloudGodRayScale = 1.0f;
    cfg.cloudShadowExtent = 1.0f;

    cfg.cloudWeatherUVScale[0] = cfg.cloudWeatherUVScale[1] = 1.0f;
    cfg.cloudCoverage = 0.45f;
    cfg.cloudDensity  = 1.0f;
    cfg.cloudShadingSunLightScale = 1.0f;
    cfg.cloudFogFade = 1.0f;
    cfg.cloudMaxTraceingDistance = 350.0f;
    cfg.cloudTracingStartMaxDistance = 1e8f;
    cfg.cloudDirection[0]=1.0f; cfg.cloudDirection[1]=0.0f; cfg.cloudDirection[2]=0.0f;
    cfg.cloudSpeed = 0.15f;
    cfg.cloudMultiScatterExtinction = 0.5f;
    cfg.cloudMultiScatterScatter = 0.5f;
    // phi_fwd 漫射场：这里给的是"未经 Cloud Tuner 覆盖"时的基线，Intensity=0 表示关闭，
    // 保证 LUT 烘焙路径与任何未显式开启的调用点都退化成接入前的行为。
    cfg.cloudPhiFwdIntensity = 0.0f;
    cfg.cloudPhiFwdMsBuildScale = 1.0f;
    cfg.cloudPhiFwdCompress = 0.0f;
    cfg.cloudPhiFwdDepthPow = 1.0f;
    cfg.cloudPhiFwdDepthBias = 0.0f;
    cfg.cloudPhiFwdBoundaryConfidence = 1.0f;
    // 行星尺度关键：cloudMap() 里这些 scale 直接乘在"公里"量级的 posKm 上
    // （posKm 现在是相对行星中心的真实公里坐标，不再是近地小范围坐标）。
    // flower 原版 0.4/1.2 是给"几公里量级"的近地场景调的，直接套用在几千公里量级的
    // 星球上会让 128³/32³ 噪声纹理在屏幕空间里重复几千次——compute shader 里 texture()
    // 没有隐式 mip LOD（噪声纹理本身也只建了 1 级 mip），会退化成极端欠采样，
    // 看起来就是满屏白色雪花噪点而不是云。首档降到"一个纹理周期对应几百公里"
    // 对应真实天气系统尺度，其余档由 Nyquist 跨度推出（见 cloud_noise_ladder.h）。
    CloudNoiseLadder::apply(cfg, CloudNoiseLadder::kDefaultBasicScale);
    cfg.cloudAlbedo[0]=cfg.cloudAlbedo[1]=cfg.cloudAlbedo[2]=1.0f;
    cfg.cloudPhaseForward = 0.8f;
    cfg.cloudPhaseBackward = -0.2f;
    cfg.cloudPhaseMixFactor = 0.5f;
    cfg.cloudPowderScale = 1.0f;
    cfg.cloudPowderPow = 1.0f;
    cfg.cloudLightStepMul = 1.5f;
    cfg.cloudLightBasicStep = 0.033f;
    cfg.cloudLightStepNum = 6;
    cfg.cloudEnableGroundContribution = 1;
    cfg.cloudMarchingStepNum = 64;
    cfg.cloudSunLitMapOctave = 1;
    cfg.cloudNoiseScale = 1.0f;
    cfg.cloudGodRay = 0; // Phase 2 待接入（依赖 SDSM 级联阴影）
    cfg.cloudCurlUVScale = 0.001f;
    cfg.cloudCurlStrength = 0.0f;
    cfg.cloudCurlContrast = 3.0f;
    (void)cloudExtinction; // 云消光由 FlowerCloudTuneParams / cloudMap Density 侧提供，这里不重复
}

// 填充"某星球的大气 LUT 烘焙"专用的 GpuPerFrameData。这个 UBO 结构体本来是给
// 云管线（vk_cloud_system.h::fillCloudFrameData，只服务当前有云的地球）设计的，
// 字段很多，但 LUT 烘焙的四个 compute pass（atmosphere_common.glsl）其实只读
// camWorldPos + sky.direction/color/intensity/atmosphereConfig 这几项——LUT 的
// 参数化本身是"高度+角度"，不需要相机的 view/proj 矩阵，所以这里只填这几项，
// 其余字段保持 GpuPerFrameData{} 的默认零值（未被 LUT 烘焙 shader 读取，无影响）。
// 供 vk_scene.h::drawAtmoInside()/drawAtmoShell() 在每帧刷新某星球的 Sky-View
// LUT（bakeSkyViewOnly）之前调用。
inline GpuPerFrameData fillAtmoLutFrameData(const float camPosKm[3], const float planetCenterKm[3],
                                             float radiusKm, float thicknessKm, int32_t atmoIdx,
                                             const float lightDirToSun[3],
                                             const AtmoTuneParams* tune = nullptr) {
    GpuPerFrameData fd{};
    float camRelKm[3] = { camPosKm[0]-planetCenterKm[0], camPosKm[1]-planetCenterKm[1], camPosKm[2]-planetCenterKm[2] };
    // camWorldPos 期望"米"为单位（shared_atmosphere.glsl::convertToAtmosphereUnit 内部
    // 会 *0.001 转回公里），这里输入已经是公里，所以要 *1000——和 vk_cloud_system.h::
    // fillCloudFrameData 对 camRelPlanetKm 做的换算完全一致（同一个坑，同一个修法）。
    fd.camWorldPos[0]=camRelKm[0]*1000.0f; fd.camWorldPos[1]=camRelKm[1]*1000.0f; fd.camWorldPos[2]=camRelKm[2]*1000.0f;

    // sky.direction 约定是"光线传播方向"（从太阳照向场景），lightDirToSun 传入的是
    // "指向太阳"的方向，互为相反数——与 vk_cloud_system.h::fillCloudFrameData 同一约定。
    fd.sky.direction[0]=-lightDirToSun[0]; fd.sky.direction[1]=-lightDirToSun[1]; fd.sky.direction[2]=-lightDirToSun[2];
    fd.sky.color[0]=fd.sky.color[1]=fd.sky.color[2]=1.0f;
    // 这里的 intensity 只是"给 Sky-View LUT 的相对形状"用的中性值 1.0——真正的曝光/
    // 强度换算在壳内/壳外 shader 自己的 exposure/nightFactor 里做（atmo_inside.frag/
    // atmo_shell.frag），不需要在这里重复一遍引擎级物理光照那套换算（另一个独立课题）。
    fd.sky.intensity = 1.0f;

    fillPlanetAtmosphereProfile(fd.sky.atmosphereConfig, atmoIdx, radiusKm, thicknessKm,
                                0.0f, 0.0f, 0.0f, tune);
    // 大气 Sky-View 必须在相机高度烘焙（与 atmo_inside 采样同观察点）。
    // 云管线 fillCloudFrameData 会置 1（云壳中点）；这里显式关，避免楔形/错位。
    fd.sky.atmosphereConfig.cloudIrradianceBakeAtShell = 0;
    return fd;
}

// ==========================================================================
// VkAtmoLutPipelines — 四个 LUT 烘焙 compute pipeline + descriptor set layout +
// pipeline layout + 采样器，全引擎共享一份（纯 shader 逻辑，和具体星球无关）。
// 每个 VkAtmosphereLut 实例（每星球一份）在自己的 descriptor set 里引用这里的
// setLayout/sampler，dispatch 时绑这里的 pipeline，实例之间不重复创建 pipeline。
// ==========================================================================
struct VkAtmoLutPipelines {
    VkDescriptorSetLayout setLayout      = VK_NULL_HANDLE;
    VkPipelineLayout      pipelineLayout = VK_NULL_HANDLE;

    VkPipeline transmittancePipe = VK_NULL_HANDLE;
    VkPipeline multiScatterPipe  = VK_NULL_HANDLE;
    VkPipeline skyViewPipe       = VK_NULL_HANDLE;
    VkPipeline skyIrradiancePipe = VK_NULL_HANDLE;
    VkPipeline froxelPipe        = VK_NULL_HANDLE; // aerial_perspective_lut.glsl，供云的空气透视用
    VkPipeline ambientMusPipe    = VK_NULL_HANDLE; // cloud_ambient_mus_lut.glsl，云环境天光 μs LUT
    VkPipeline inscatterPipe     = VK_NULL_HANDLE; // inscatter_lut.glsl，4D→3D 单次散射

    VkSampler linearClampSampler = VK_NULL_HANDLE;

    // ── 渲染侧采样布局（图形管线用，Set 1）───────────────────────────────────
    // atmo_inside.frag / atmo_shell.frag 用这个 Set（配合 Set 0 的 FrameUBO）采样
    // "某一个星球" 的 Transmittance/SkyView LUT + 场景深度，做壳内 haze / 壳外 limb。
    // 布局本身（4 个 binding）全局共享一份；每个 VkAtmosphereLut 实例各自的
    // renderSet[FRAMES_IN_FLIGHT] 引用同一个 renderSetLayout，只是内容（指向哪张
    // LUT）不同——见 VkAtmosphereLut::updateRenderSet()。
    VkDescriptorSetLayout renderSetLayout = VK_NULL_HANDLE;

    bool init(VulkanContext& ctx) {
        if (!createSampler(ctx))       return false;
        if (!createSetLayout(ctx))     return false;
        if (!createRenderSetLayout(ctx)) return false;
        if (!createPipelines(ctx))     return false;
        printf("[VkAtmoLutPipelines] Ready (shared compute pipelines for per-planet LUT cache)\n");
        return true;
    }

    void shutdown(VulkanContext& ctx) {
        vkDeviceWaitIdle(ctx.device);
        auto destroyPipe = [&](VkPipeline& p){ if(p){vkDestroyPipeline(ctx.device,p,nullptr);p=VK_NULL_HANDLE;} };
        destroyPipe(transmittancePipe); destroyPipe(multiScatterPipe);
        destroyPipe(skyViewPipe); destroyPipe(skyIrradiancePipe);
        destroyPipe(froxelPipe); destroyPipe(ambientMusPipe); destroyPipe(inscatterPipe);
        if (pipelineLayout) { vkDestroyPipelineLayout(ctx.device, pipelineLayout, nullptr); pipelineLayout = VK_NULL_HANDLE; }
        if (setLayout) { vkDestroyDescriptorSetLayout(ctx.device, setLayout, nullptr); setLayout = VK_NULL_HANDLE; }
        if (renderSetLayout) { vkDestroyDescriptorSetLayout(ctx.device, renderSetLayout, nullptr); renderSetLayout = VK_NULL_HANDLE; }
        if (linearClampSampler) { vkDestroySampler(ctx.device, linearClampSampler, nullptr); linearClampSampler = VK_NULL_HANDLE; }
    }

private:
    bool createSampler(VulkanContext& ctx) {
        VkSamplerCreateInfo sci{ VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO };
        sci.magFilter = VK_FILTER_LINEAR; sci.minFilter = VK_FILTER_LINEAR;
        sci.addressModeU = sci.addressModeV = sci.addressModeW = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
        return vkCreateSampler(ctx.device, &sci, nullptr, &linearClampSampler) == VK_SUCCESS;
    }

    // 16-binding 布局（12=combined Inscatter，13=完整 RGB Single Mie，
    // 14/15=同两张图的采样端，供 cloud_ambient_mus_lut 过滤查表）
    bool createSetLayout(VulkanContext& ctx) {
        VkDescriptorSetLayoutBinding b[16]{};
        b[0] = { 0, VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr };
        b[1] = { 1, VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr };
        b[2] = { 2, VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr };
        b[3] = { 3, VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr };
        b[4] = { 4, VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr };
        b[5] = { 5, VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr };
        b[6] = { 6, VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr };
        b[7] = { 7, VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr };
        b[8] = { 8, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr };
        b[9] = { 9, VK_DESCRIPTOR_TYPE_SAMPLER, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr };
        b[10] = { 10, VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr }; // Froxel
        b[11] = { 11, VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr }; // Ambient μs LUT
        b[12] = { 12, VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr }; // Inscatter 3D
        b[13] = { 13, VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr }; // Full RGB Mie 3D
        b[14] = { 14, VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr }; // Inscatter 3D（采样端）
        b[15] = { 15, VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr }; // Full RGB Mie 3D（采样端）
        VkDescriptorSetLayoutCreateInfo lci{ VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO };
        lci.bindingCount = 16; lci.pBindings = b;
        if (vkCreateDescriptorSetLayout(ctx.device, &lci, nullptr, &setLayout) != VK_SUCCESS) return false;

        VkPipelineLayoutCreateInfo plci{ VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO };
        plci.setLayoutCount = 1; plci.pSetLayouts = &setLayout;
        return vkCreatePipelineLayout(ctx.device, &plci, nullptr, &pipelineLayout) == VK_SUCCESS;
    }

    // 渲染侧 7-binding：0=depth 1=Trans 2=SkyView 3=sampler 4=HDR 5=Inscatter 6=Full Mie
    bool createRenderSetLayout(VulkanContext& ctx) {
        VkDescriptorSetLayoutBinding b[7]{};
        b[0] = { 0, VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE, 1, VK_SHADER_STAGE_FRAGMENT_BIT, nullptr };
        b[1] = { 1, VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE, 1, VK_SHADER_STAGE_FRAGMENT_BIT, nullptr };
        b[2] = { 2, VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE, 1, VK_SHADER_STAGE_FRAGMENT_BIT, nullptr };
        b[3] = { 3, VK_DESCRIPTOR_TYPE_SAMPLER, 1, VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT, nullptr };
        b[4] = { 4, VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE, 1, VK_SHADER_STAGE_FRAGMENT_BIT, nullptr };
        b[5] = { 5, VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE, 1, VK_SHADER_STAGE_FRAGMENT_BIT, nullptr }; // Inscatter
        b[6] = { 6, VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE, 1, VK_SHADER_STAGE_FRAGMENT_BIT, nullptr }; // Full RGB Mie
        VkDescriptorSetLayoutCreateInfo lci{ VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO };
        lci.bindingCount = 7; lci.pBindings = b;
        return vkCreateDescriptorSetLayout(ctx.device, &lci, nullptr, &renderSetLayout) == VK_SUCCESS;
    }

    bool createPipelines(VulkanContext& ctx) {
        auto make = [&](const char* spv, VkPipeline& outPipe) -> bool {
            auto code = loadSPIRV(spv);
            if (code.empty()) { fprintf(stderr, "[VkAtmoLutPipelines] Missing spv: %s\n", spv); return false; }
            VkShaderModule mod = createShaderModule(ctx.device, code);
            if (mod == VK_NULL_HANDLE) return false;
            VkPipelineShaderStageCreateInfo stage{ VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO };
            stage.stage = VK_SHADER_STAGE_COMPUTE_BIT; stage.module = mod; stage.pName = "main";
            VkComputePipelineCreateInfo cpci{ VK_STRUCTURE_TYPE_COMPUTE_PIPELINE_CREATE_INFO };
            cpci.stage = stage; cpci.layout = pipelineLayout;
            VkResult res = vkCreateComputePipelines(ctx.device, VK_NULL_HANDLE, 1, &cpci, nullptr, &outPipe);
            vkDestroyShaderModule(ctx.device, mod, nullptr);
            return res == VK_SUCCESS;
        };
        return make("src/render/shaders/spirv/transmittance_lut.comp.spv", transmittancePipe)
            && make("src/render/shaders/spirv/multi_scatter_lut.comp.spv", multiScatterPipe)
            && make("src/render/shaders/spirv/skyview_lut.comp.spv", skyViewPipe)
            && make("src/render/shaders/spirv/sky_irradiance_capture.comp.spv", skyIrradiancePipe)
            && make("src/render/shaders/spirv/aerial_perspective_lut.comp.spv", froxelPipe)
            && make("src/render/shaders/spirv/cloud_ambient_mus_lut.comp.spv", ambientMusPipe)
            && make("src/render/shaders/spirv/inscatter_lut.comp.spv", inscatterPipe);
    }
};

static void _atmoLutComputeBarrier(VkCommandBuffer cmd) {
    VkMemoryBarrier2 b{ VK_STRUCTURE_TYPE_MEMORY_BARRIER_2 };
    b.srcStageMask = VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT; b.srcAccessMask = VK_ACCESS_2_SHADER_WRITE_BIT;
    b.dstStageMask = VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT; b.dstAccessMask = VK_ACCESS_2_SHADER_READ_BIT | VK_ACCESS_2_SHADER_WRITE_BIT;
    VkDependencyInfo dep{ VK_STRUCTURE_TYPE_DEPENDENCY_INFO }; dep.memoryBarrierCount=1; dep.pMemoryBarriers=&b;
    vkCmdPipelineBarrier2(cmd, &dep);
}

// 已修复：compute→compute 的 _atmoLutComputeBarrier 只保证 dstStageMask=COMPUTE_SHADER
// 可见，不包含 FRAGMENT_SHADER——但 bakeFull()/bakeSkyViewOnly() 之后紧接着就是同一个
// command buffer 里的 atmo_inside.frag/atmo_shell.frag 图形绘制，会在片元着色器阶段
// 采样刚烤好的 Transmittance/Sky-View LUT。没有这道"compute 写 → fragment 读"的可见性
// 屏障，GPU 没有任何保证片元着色器读到的是烘焙完成后的数据。这是一个真实、独立的
// Vulkan 同步问题，无论如何都要修（不修的话是未定义行为，能不能观察到取决于具体
// GPU/驱动的调度巧合）；曾经怀疑过它和大气壳内那个白色三角形是同一个 bug，后来
// 确认三角形的真正原因是 atmosphere_common.glsl::getPosScatterLight() 的坐标系
// 不一致（已在那边修好），这道屏障单独修，不依赖那个结论。
// 放在每次烘焙链的最后一步，取代原来那个 compute-only 的收尾屏障。
static void _atmoLutComputeToFragmentBarrier(VkCommandBuffer cmd) {
    VkMemoryBarrier2 b{ VK_STRUCTURE_TYPE_MEMORY_BARRIER_2 };
    b.srcStageMask = VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT; b.srcAccessMask = VK_ACCESS_2_SHADER_WRITE_BIT;
    b.dstStageMask = VK_PIPELINE_STAGE_2_FRAGMENT_SHADER_BIT; b.dstAccessMask = VK_ACCESS_2_SHADER_READ_BIT;
    VkDependencyInfo dep{ VK_STRUCTURE_TYPE_DEPENDENCY_INFO }; dep.memoryBarrierCount=1; dep.pMemoryBarriers=&b;
    vkCmdPipelineBarrier2(cmd, &dep);
}

// ==========================================================================
// VkAtmosphereLut — 单个星球的 LUT 资源实例：images(trans/multi/skyView/cube) +
// 每帧 UBO + descriptor set。不再自带 pipeline——那些由 VkAtmoLutPipelines
// （全局共享一份，见上）提供，bake 相关函数都要求调用方传入共享的 pipelines 引用。
// 由 VkAtmoLutCache（本文件末尾）按 bodyIdx 持有/复用，调用方一般不直接构造。
// ==========================================================================
struct VkAtmosphereLut {
    static constexpr uint32_t kTransW = 256, kTransH = 64;
    static constexpr uint32_t kMultiScatterSize = 32;
    static constexpr uint32_t kSkyViewW = 192, kSkyViewH = 108;
    static constexpr uint32_t kCubeSize = 32;
    // Froxel 尺寸：屏幕 uv 方向用固定粗分辨率（不跟随实际窗口分辨率，靠双线性插值
    // 补细节），深度方向 32 片，配合 shared_atmosphere.glsl::kAirPerspectiveKmPerSlice=4
    // 正好是 32×4=128km 的最大空气透视距离，和 cloud_common.glsl 采样时的假设一致。
    static constexpr uint32_t kFroxelW = 160, kFroxelH = 90, kFroxelD = 32;
    // 云环境天光：μs∈[-1,1] → 云壳高度天空半球余弦加权辐照度/π（与相机无关）
    static constexpr uint32_t kAmbientMusW = 128, kAmbientMusH = 1;
    // Inscatter 4D→3D：MuS*Nu × Mu × R = 256×128×32
    static constexpr uint32_t kInscatterW = 32u * 8u;  // RES_MU_S * RES_NU
    static constexpr uint32_t kInscatterH = 128u;      // RES_MU
    static constexpr uint32_t kInscatterD = 32u;       // RES_R
    static constexpr VkFormat kFmt = VK_FORMAT_R16G16B16A16_SFLOAT;

    VkImage image_[3] = {}; VmaAllocation alloc_[3] = {}; VkImageView view_[3] = {}; // trans/multiscatter/skyview
    VkImage cubeImage = VK_NULL_HANDLE; VmaAllocation cubeAlloc = VK_NULL_HANDLE; VkImageView cubeView = VK_NULL_HANDLE;
    // Froxel 3D 体积散射（aerial_perspective_lut.glsl 写入，供 cloud_common.glsl::
    // inFroxelScatter 空气透视消费）。和相机视锥强相关，逐帧重烤（bakeFroxel()），
    // 不像 Transmittance/MultiScatter 那样能按 profile 指纹跨帧缓存。
    VkImage froxelImage = VK_NULL_HANDLE; VmaAllocation froxelAlloc = VK_NULL_HANDLE; VkImageView froxelView = VK_NULL_HANDLE;
    VkImage ambientMusImage = VK_NULL_HANDLE; VmaAllocation ambientMusAlloc = VK_NULL_HANDLE; VkImageView ambientMusView = VK_NULL_HANDLE;
    VkImage inscatterImage = VK_NULL_HANDLE; VmaAllocation inscatterAlloc = VK_NULL_HANDLE; VkImageView inscatterView = VK_NULL_HANDLE;
    VkImage singleMieImage = VK_NULL_HANDLE; VmaAllocation singleMieAlloc = VK_NULL_HANDLE; VkImageView singleMieView = VK_NULL_HANDLE;

    VkBuffer      uboBuf[FRAMES_IN_FLIGHT]   = {};
    VmaAllocation uboAlloc[FRAMES_IN_FLIGHT] = {};
    void*         uboMapped[FRAMES_IN_FLIGHT]= {};

    VkDescriptorPool      pool = VK_NULL_HANDLE;
    VkDescriptorSet       descSet[FRAMES_IN_FLIGHT] = {};

    // ── 渲染侧采样 Set（Set 1，图形管线用）──────────────────────────────────
    // 独立于上面 bake 用的 pool/descSet（那套是 compute 用的 STORAGE_IMAGE 写入端），
    // 这里是 atmo_inside.frag/atmo_shell.frag 采样读取端：场景深度 + 本星球的
    // Transmittance/Sky-View LUT + 采样器，引用 VkAtmoLutPipelines::renderSetLayout。
    VkDescriptorPool renderPool = VK_NULL_HANDLE;
    VkDescriptorSet  renderSet[FRAMES_IN_FLIGHT] = {};

    // ── Profile 指纹（按星球缓存的核心机制）───────────────────────────────
    // 记录上次 bakeFull 的 AtmoLutProfileKey；半径/atmoIdx/F4 profile 任一变 → 重烤。
    // atmoIdx=-1（默认）保证首次访问必定触发一次 bakeFull()。
    AtmoLutProfileKey bakedProfile{};

    bool init(VulkanContext& ctx, const VkAtmoLutPipelines& pipes) {
        if (!createImages(ctx))              return false;
        if (!createUBOs(ctx))                return false;
        if (!createDescriptors(ctx, pipes))  return false;
        if (!createRenderDescriptors(ctx, pipes)) return false;
        return true;
    }

    void shutdown(VulkanContext& ctx) {
        vkDeviceWaitIdle(ctx.device);
        if (pool) { vkDestroyDescriptorPool(ctx.device, pool, nullptr); pool = VK_NULL_HANDLE; }
        if (renderPool) { vkDestroyDescriptorPool(ctx.device, renderPool, nullptr); renderPool = VK_NULL_HANDLE; }
        for (int i=0;i<FRAMES_IN_FLIGHT;i++)
            if (uboBuf[i]) { vmaDestroyBuffer(ctx.allocator, uboBuf[i], uboAlloc[i]); uboBuf[i]=VK_NULL_HANDLE; }
        for (int i=0;i<3;i++) {
            if (view_[i])  { vkDestroyImageView(ctx.device, view_[i], nullptr); view_[i]=VK_NULL_HANDLE; }
            if (image_[i]) { vmaDestroyImage(ctx.allocator, image_[i], alloc_[i]); image_[i]=VK_NULL_HANDLE; }
        }
        if (cubeView)  { vkDestroyImageView(ctx.device, cubeView, nullptr); cubeView=VK_NULL_HANDLE; }
        if (cubeImage) { vmaDestroyImage(ctx.allocator, cubeImage, cubeAlloc); cubeImage=VK_NULL_HANDLE; }
        if (froxelView)  { vkDestroyImageView(ctx.device, froxelView, nullptr); froxelView=VK_NULL_HANDLE; }
        if (froxelImage) { vmaDestroyImage(ctx.allocator, froxelImage, froxelAlloc); froxelImage=VK_NULL_HANDLE; }
        if (ambientMusView)  { vkDestroyImageView(ctx.device, ambientMusView, nullptr); ambientMusView=VK_NULL_HANDLE; }
        if (ambientMusImage) { vmaDestroyImage(ctx.allocator, ambientMusImage, ambientMusAlloc); ambientMusImage=VK_NULL_HANDLE; }
        if (inscatterView)  { vkDestroyImageView(ctx.device, inscatterView, nullptr); inscatterView=VK_NULL_HANDLE; }
        if (inscatterImage) { vmaDestroyImage(ctx.allocator, inscatterImage, inscatterAlloc); inscatterImage=VK_NULL_HANDLE; }
        if (singleMieView)  { vkDestroyImageView(ctx.device, singleMieView, nullptr); singleMieView=VK_NULL_HANDLE; }
        if (singleMieImage) { vmaDestroyImage(ctx.allocator, singleMieImage, singleMieAlloc); singleMieImage=VK_NULL_HANDLE; }
    }

    // 每帧刷新渲染侧 Set 1 的 depth + HDR scene 视图（TAA ping-pong / 同图读写）。
    void updateRenderBindings(VulkanContext& ctx, int frameSlot, VkImageView depthView, VkImageView sceneColorView) {
        int slot = frameSlot % FRAMES_IN_FLIGHT;
        VkDescriptorImageInfo iDepth{ VK_NULL_HANDLE, depthView, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL };
        VkDescriptorImageInfo iScene{ VK_NULL_HANDLE, sceneColorView, VK_IMAGE_LAYOUT_GENERAL };
        VkWriteDescriptorSet w[2]{};
        w[0].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
        w[0].dstSet = renderSet[slot]; w[0].dstBinding = 0;
        w[0].descriptorType = VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE; w[0].descriptorCount = 1; w[0].pImageInfo = &iDepth;
        w[1].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
        w[1].dstSet = renderSet[slot]; w[1].dstBinding = 4;
        w[1].descriptorType = VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE; w[1].descriptorCount = 1; w[1].pImageInfo = &iScene;
        vkUpdateDescriptorSets(ctx.device, 2, w, 0, nullptr);
    }

    VkImageView transmittanceView() const { return view_[0]; }
    VkImageView skyViewLutView()    const { return view_[2]; }
    VkImageView skyIrradianceCubeView() const { return cubeView; }
    VkImageView cloudAmbientMusLutView() const { return ambientMusView; }
    VkImageView inscatterLutView()  const { return inscatterView; }
    VkImageView singleMieLutView()  const { return singleMieView; }
    VkBuffer    frameUBO(int frameSlot) const { return uboBuf[frameSlot % FRAMES_IN_FLIGHT]; }

    // 只做 UBO 上传（memcpy + host->compute 可见性 barrier）。cloud_common.glsl（binding 21）
    // 和这几个 LUT bake pass 共用同一个 GpuPerFrameData 结构（不是同一份 UBO——云管线读的是
    // vk_cloud_system.h 自己那份 fd；这里是"某星球的大气 LUT" 专用的一份），每次 bake 前都要上传。
    void uploadFrameData(VulkanContext& ctx, VkCommandBuffer cmd, int frameSlot, const GpuPerFrameData& fd) {
        int slot = frameSlot % FRAMES_IN_FLIGHT;
        memcpy(uboMapped[slot], &fd, sizeof(fd));

        // UBO 内容由 CPU 写，这里保证 host write 在 compute 读之前可见。
        VkMemoryBarrier2 hostBarrier{ VK_STRUCTURE_TYPE_MEMORY_BARRIER_2 };
        hostBarrier.srcStageMask = VK_PIPELINE_STAGE_2_HOST_BIT; hostBarrier.srcAccessMask = VK_ACCESS_2_HOST_WRITE_BIT;
        hostBarrier.dstStageMask = VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT; hostBarrier.dstAccessMask = VK_ACCESS_2_UNIFORM_READ_BIT;
        VkDependencyInfo hdep{ VK_STRUCTURE_TYPE_DEPENDENCY_INFO }; hdep.memoryBarrierCount=1; hdep.pMemoryBarriers=&hostBarrier;
        vkCmdPipelineBarrier2(cmd, &hdep);
    }

    // 全量重烤：Transmittance→MultiScatter→Inscatter→SkyView→SkyIrradiance。
    // Inscatter 依赖 Transmittance + MultiScatter（烘焙时折入多重散射）；
    // 只应在 profile 指纹变化时调用。
    void bakeFull(VulkanContext& ctx, VkCommandBuffer cmd, int frameSlot,
                  const VkAtmoLutPipelines& pipes, const GpuPerFrameData& fd) {
        int slot = frameSlot % FRAMES_IN_FLIGHT;
        uploadFrameData(ctx, cmd, frameSlot, fd);

        VkDescriptorSet set = descSet[slot];
        vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_COMPUTE, pipes.pipelineLayout, 0, 1, &set, 0, nullptr);

        vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_COMPUTE, pipes.transmittancePipe);
        vkCmdDispatch(cmd, (kTransW+7)/8, (kTransH+7)/8, 1);
        _atmoLutComputeBarrier(cmd);

        vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_COMPUTE, pipes.multiScatterPipe);
        vkCmdDispatch(cmd, kMultiScatterSize, kMultiScatterSize, 1); // local_size_z = 64 覆盖采样循环
        _atmoLutComputeBarrier(cmd);

        vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_COMPUTE, pipes.inscatterPipe);
        vkCmdDispatch(cmd, (kInscatterW+7)/8, (kInscatterH+7)/8, kInscatterD);
        _atmoLutComputeBarrier(cmd);

        vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_COMPUTE, pipes.skyViewPipe);
        vkCmdDispatch(cmd, (kSkyViewW+7)/8, (kSkyViewH+7)/8, 1);
        _atmoLutComputeBarrier(cmd);

        vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_COMPUTE, pipes.skyIrradiancePipe);
        vkCmdDispatch(cmd, (kCubeSize+7)/8, (kCubeSize+7)/8, 6);
        // 最后一步收尾：让片元着色器（紧接着的 atmo_inside.frag/atmo_shell.frag）
        // 能正确看到这一整条烘焙链（含前面 transmittance/multiScatter）的结果。
        _atmoLutComputeToFragmentBarrier(cmd);
    }

    // 只重烤 Sky-View LUT + Sky-Irradiance cubemap：这两张表把相机高度/太阳-视线角
    // 当参数直接烤了进去，理论上每帧都可能变（起飞/入轨/星际航行时高度持续变化），
    // 但分辨率很小（192×108、32×32×6），逐帧重烤远比 atmo.frag 原来的全屏 1920 步
    // raymarch 便宜。Transmittance/MultiScatter 维持上次 bakeFull() 的结果不动。
    // 调用方（vk_renderer3d.h）应该只对"这一帧实际要渲染"的星球调用这个，不是
    // 缓存里的每一个星球都刷一遍。
    void bakeSkyViewOnly(VulkanContext& ctx, VkCommandBuffer cmd, int frameSlot,
                          const VkAtmoLutPipelines& pipes, const GpuPerFrameData& fd) {
        int slot = frameSlot % FRAMES_IN_FLIGHT;
        uploadFrameData(ctx, cmd, frameSlot, fd);

        VkDescriptorSet set = descSet[slot];
        vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_COMPUTE, pipes.pipelineLayout, 0, 1, &set, 0, nullptr);

        vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_COMPUTE, pipes.skyViewPipe);
        vkCmdDispatch(cmd, (kSkyViewW+7)/8, (kSkyViewH+7)/8, 1);
        _atmoLutComputeBarrier(cmd);

        vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_COMPUTE, pipes.skyIrradiancePipe);
        vkCmdDispatch(cmd, (kCubeSize+7)/8, (kCubeSize+7)/8, 6);
        // 同上：收尾用 compute→fragment 可见性屏障，不是 compute→compute。
        _atmoLutComputeToFragmentBarrier(cmd);
    }

    // 烘焙 Froxel 3D 体积散射（供 cloud_common.glsl::inFroxelScatter 空气透视用）。
    // 和相机视锥强相关（每个 cell 都要重建屏幕方向的视线），必须逐帧重烤，不像
    // Transmittance/MultiScatter 那样能靠 profile 指纹跨帧复用。调用方传入的 fd
    // 必须已经填好 camInvertProj/camInvertView（LUT 烘焙用的 fillAtmoLutFrameData()
    // 默认不填这两项，因为其余四个 bake 不需要——云管线自己的 fillCloudFrameData()
    // 已经填了，直接复用云管线那份 fd 即可，见 vk_cloud_system.h 调用处）。
    // 依赖 Transmittance LUT（内部 integrateScatteredLuminance 查表用），所以要排在
    // bakeFull()/bakeSkyViewOnly() 之后调用，且不需要单独再传一次 UBO（同一个 fd，
    // uploadFrameData 已经在前面的 bake 里做过，这里不重复上传，避免同一帧多次
    // memcpy 同一份数据）。
    void bakeFroxel(VulkanContext& ctx, VkCommandBuffer cmd, int frameSlot,
                     const VkAtmoLutPipelines& pipes) {
        int slot = frameSlot % FRAMES_IN_FLIGHT;
        VkDescriptorSet set = descSet[slot];
        vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_COMPUTE, pipes.pipelineLayout, 0, 1, &set, 0, nullptr);

        vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_COMPUTE, pipes.froxelPipe);
        vkCmdDispatch(cmd, (kFroxelW+7)/8, (kFroxelH+7)/8, kFroxelD);
        // 收尾用 compute→fragment？不对——Froxel 是被 cloud_raymarching.glsl（同样是
        // compute shader）采样的，不是图形管线，用 compute→compute 屏障即可；但云的
        // raymarch dispatch 和这里的烘焙不在同一个 pipelineLayout/descriptor set 下，
        // 稳妥起见用一个全局内存屏障覆盖 compute→compute 的通用可见性。
        _atmoLutComputeBarrier(cmd);
    }

    // 烘焙云环境天光 μs LUT（128×1）。与相机无关，只依赖大气 profile（单位太阳 Inscatter）。
    // 依赖 Inscatter/SingleMie（半球每个方向查表补相位，不再逐方向 raymarch），
    // 所以必须排在 bakeFull 的 inscatter dispatch 之后、云 raymarch 之前调用。
    // 不重复 upload UBO（与 bakeFroxel 相同约定）。
    // 运行时 sampleCloudAmbient 再乘 L_sun = color×intensity。
    void bakeAmbientMusLut(VulkanContext& ctx, VkCommandBuffer cmd, int frameSlot,
                            const VkAtmoLutPipelines& pipes) {
        int slot = frameSlot % FRAMES_IN_FLIGHT;
        VkDescriptorSet set = descSet[slot];
        vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_COMPUTE, pipes.pipelineLayout, 0, 1, &set, 0, nullptr);

        vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_COMPUTE, pipes.ambientMusPipe);
        vkCmdDispatch(cmd, (kAmbientMusW + 63) / 64, 1, 1);
        _atmoLutComputeBarrier(cmd);
    }

    VkImageView froxelViewForRender() const { return froxelView; }

private:
    bool createImage2D(VulkanContext& ctx, uint32_t w, uint32_t h, VkImage& img, VmaAllocation& alloc, VkImageView& view) {
        VkImageCreateInfo ici{ VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO };
        ici.imageType = VK_IMAGE_TYPE_2D; ici.format = kFmt; ici.extent = {w,h,1};
        ici.mipLevels = 1; ici.arrayLayers = 1; ici.samples = VK_SAMPLE_COUNT_1_BIT;
        ici.tiling = VK_IMAGE_TILING_OPTIMAL;
        ici.usage = VK_IMAGE_USAGE_STORAGE_BIT | VK_IMAGE_USAGE_SAMPLED_BIT;
        ici.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
        VmaAllocationCreateInfo aci{}; aci.usage = VMA_MEMORY_USAGE_GPU_ONLY;
        if (vmaCreateImage(ctx.allocator, &ici, &aci, &img, &alloc, nullptr) != VK_SUCCESS) return false;

        VkImageViewCreateInfo vci{ VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO };
        vci.image = img; vci.viewType = VK_IMAGE_VIEW_TYPE_2D; vci.format = kFmt;
        vci.subresourceRange = { VK_IMAGE_ASPECT_COLOR_BIT, 0,1,0,1 };
        if (vkCreateImageView(ctx.device, &vci, nullptr, &view) != VK_SUCCESS) return false;

        // 常驻 GENERAL layout：同一 pass 内既作为 storage image 写入，也在后续 pass
        // 里作为 sampled image 读取，不做逐帧布局切换（LUT 只在 profile/相机高度变化时
        // 重烤，不需要保留跨帧只读态）。
        VkCommandBuffer cmd = beginSingleTimeCommands(ctx);
        transitionImage(cmd, img, VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_GENERAL,
            VK_PIPELINE_STAGE_2_TOP_OF_PIPE_BIT, VK_ACCESS_2_NONE,
            VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT, VK_ACCESS_2_SHADER_READ_BIT | VK_ACCESS_2_SHADER_WRITE_BIT);
        endSingleTimeCommands(ctx, cmd);
        return true;
    }

    bool createImages(VulkanContext& ctx) {
        if (!createImage2D(ctx, kTransW, kTransH, image_[0], alloc_[0], view_[0])) return false;
        if (!createImage2D(ctx, kMultiScatterSize, kMultiScatterSize, image_[1], alloc_[1], view_[1])) return false;
        if (!createImage2D(ctx, kSkyViewW, kSkyViewH, image_[2], alloc_[2], view_[2])) return false;

        VkImageCreateInfo ici{ VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO };
        ici.flags = VK_IMAGE_CREATE_CUBE_COMPATIBLE_BIT;
        ici.imageType = VK_IMAGE_TYPE_2D; ici.format = kFmt; ici.extent = {kCubeSize,kCubeSize,1};
        ici.mipLevels = 1; ici.arrayLayers = 6; ici.samples = VK_SAMPLE_COUNT_1_BIT;
        ici.tiling = VK_IMAGE_TILING_OPTIMAL;
        ici.usage = VK_IMAGE_USAGE_STORAGE_BIT | VK_IMAGE_USAGE_SAMPLED_BIT;
        ici.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
        VmaAllocationCreateInfo aci{}; aci.usage = VMA_MEMORY_USAGE_GPU_ONLY;
        if (vmaCreateImage(ctx.allocator, &ici, &aci, &cubeImage, &cubeAlloc, nullptr) != VK_SUCCESS) return false;

        VkImageViewCreateInfo vci{ VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO };
        vci.image = cubeImage; vci.viewType = VK_IMAGE_VIEW_TYPE_CUBE; vci.format = kFmt;
        vci.subresourceRange = { VK_IMAGE_ASPECT_COLOR_BIT, 0,1,0,6 };
        if (vkCreateImageView(ctx.device, &vci, nullptr, &cubeView) != VK_SUCCESS) return false;

        VkCommandBuffer cmd = beginSingleTimeCommands(ctx);
        transitionImage(cmd, cubeImage, VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_GENERAL,
            VK_PIPELINE_STAGE_2_TOP_OF_PIPE_BIT, VK_ACCESS_2_NONE,
            VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT, VK_ACCESS_2_SHADER_READ_BIT | VK_ACCESS_2_SHADER_WRITE_BIT);
        endSingleTimeCommands(ctx, cmd);

        if (!createFroxelImage3D(ctx)) return false;
        if (!createImage2D(ctx, kAmbientMusW, kAmbientMusH, ambientMusImage, ambientMusAlloc, ambientMusView)) return false;
        if (!createInscatterImage3D(ctx)) return false;
        return true;
    }

    bool createInscatterImage3D(VulkanContext& ctx) {
        auto createOne = [&](VkImage& image, VmaAllocation& alloc, VkImageView& view) {
            VkImageCreateInfo ici{ VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO };
            ici.imageType = VK_IMAGE_TYPE_3D; ici.format = kFmt; ici.extent = {kInscatterW,kInscatterH,kInscatterD};
            ici.mipLevels = 1; ici.arrayLayers = 1; ici.samples = VK_SAMPLE_COUNT_1_BIT;
            ici.tiling = VK_IMAGE_TILING_OPTIMAL;
            ici.usage = VK_IMAGE_USAGE_STORAGE_BIT | VK_IMAGE_USAGE_SAMPLED_BIT;
            ici.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
            VmaAllocationCreateInfo aci{}; aci.usage = VMA_MEMORY_USAGE_GPU_ONLY;
            if (vmaCreateImage(ctx.allocator, &ici, &aci, &image, &alloc, nullptr) != VK_SUCCESS) return false;

            VkImageViewCreateInfo vci{ VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO };
            vci.image = image; vci.viewType = VK_IMAGE_VIEW_TYPE_3D; vci.format = kFmt;
            vci.subresourceRange = { VK_IMAGE_ASPECT_COLOR_BIT, 0,1,0,1 };
            if (vkCreateImageView(ctx.device, &vci, nullptr, &view) != VK_SUCCESS) return false;

            VkCommandBuffer cmd = beginSingleTimeCommands(ctx);
            transitionImage(cmd, image, VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_GENERAL,
                VK_PIPELINE_STAGE_2_TOP_OF_PIPE_BIT, VK_ACCESS_2_NONE,
                VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT,
                VK_ACCESS_2_SHADER_READ_BIT | VK_ACCESS_2_SHADER_WRITE_BIT);
            endSingleTimeCommands(ctx, cmd);
            return true;
        };
        return createOne(inscatterImage, inscatterAlloc, inscatterView)
            && createOne(singleMieImage, singleMieAlloc, singleMieView);
    }

    bool createFroxelImage3D(VulkanContext& ctx) {
        VkImageCreateInfo ici{ VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO };
        ici.imageType = VK_IMAGE_TYPE_3D; ici.format = kFmt; ici.extent = {kFroxelW,kFroxelH,kFroxelD};
        ici.mipLevels = 1; ici.arrayLayers = 1; ici.samples = VK_SAMPLE_COUNT_1_BIT;
        ici.tiling = VK_IMAGE_TILING_OPTIMAL;
        ici.usage = VK_IMAGE_USAGE_STORAGE_BIT | VK_IMAGE_USAGE_SAMPLED_BIT;
        ici.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
        VmaAllocationCreateInfo aci{}; aci.usage = VMA_MEMORY_USAGE_GPU_ONLY;
        if (vmaCreateImage(ctx.allocator, &ici, &aci, &froxelImage, &froxelAlloc, nullptr) != VK_SUCCESS) return false;

        VkImageViewCreateInfo vci{ VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO };
        vci.image = froxelImage; vci.viewType = VK_IMAGE_VIEW_TYPE_3D; vci.format = kFmt;
        vci.subresourceRange = { VK_IMAGE_ASPECT_COLOR_BIT, 0,1,0,1 };
        if (vkCreateImageView(ctx.device, &vci, nullptr, &froxelView) != VK_SUCCESS) return false;

        VkCommandBuffer cmd2 = beginSingleTimeCommands(ctx);
        transitionImage(cmd2, froxelImage, VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_GENERAL,
            VK_PIPELINE_STAGE_2_TOP_OF_PIPE_BIT, VK_ACCESS_2_NONE,
            VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT, VK_ACCESS_2_SHADER_READ_BIT | VK_ACCESS_2_SHADER_WRITE_BIT);
        endSingleTimeCommands(ctx, cmd2);
        return true;
    }

    bool createUBOs(VulkanContext& ctx) {
        VkBufferCreateInfo bci{ VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO };
        bci.size = sizeof(GpuPerFrameData);
        bci.usage = VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT;
        VmaAllocationCreateInfo aci{};
        aci.usage = VMA_MEMORY_USAGE_CPU_TO_GPU;
        aci.flags = VMA_ALLOCATION_CREATE_MAPPED_BIT;
        for (int i=0;i<FRAMES_IN_FLIGHT;i++) {
            VmaAllocationInfo info{};
            if (vmaCreateBuffer(ctx.allocator, &bci, &aci, &uboBuf[i], &uboAlloc[i], &info) != VK_SUCCESS) return false;
            uboMapped[i] = info.pMappedData;
        }
        return true;
    }

    // 每个星球实例自带一份小 descriptor pool（FRAMES_IN_FLIGHT 个 set），引用
    // VkAtmoLutPipelines 共享的 setLayout/sampler——布局是共享的，内容（哪张图/哪个 UBO）
    // 是这个星球自己的。
    bool createDescriptors(VulkanContext& ctx, const VkAtmoLutPipelines& pipes) {
        VkDescriptorPoolSize sizes[] = {
            { VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, 8 * FRAMES_IN_FLIGHT }, // +inscatter + full Mie
            { VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE, 6 * FRAMES_IN_FLIGHT }, // +inscatter/fullMie 采样端
            { VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1 * FRAMES_IN_FLIGHT },
            { VK_DESCRIPTOR_TYPE_SAMPLER, 1 * FRAMES_IN_FLIGHT },
        };
        VkDescriptorPoolCreateInfo pci{ VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO };
        pci.maxSets = FRAMES_IN_FLIGHT; pci.poolSizeCount = 4; pci.pPoolSizes = sizes;
        if (vkCreateDescriptorPool(ctx.device, &pci, nullptr, &pool) != VK_SUCCESS) return false;

        for (int i=0;i<FRAMES_IN_FLIGHT;i++) {
            VkDescriptorSetAllocateInfo ai{ VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO };
            ai.descriptorPool = pool; ai.descriptorSetCount = 1; ai.pSetLayouts = &pipes.setLayout;
            if (vkAllocateDescriptorSets(ctx.device, &ai, &descSet[i]) != VK_SUCCESS) return false;

            VkDescriptorImageInfo iTrans{ VK_NULL_HANDLE, view_[0], VK_IMAGE_LAYOUT_GENERAL };
            VkDescriptorImageInfo iMulti{ VK_NULL_HANDLE, view_[1], VK_IMAGE_LAYOUT_GENERAL };
            VkDescriptorImageInfo iSky  { VK_NULL_HANDLE, view_[2], VK_IMAGE_LAYOUT_GENERAL };
            VkDescriptorImageInfo iCube { VK_NULL_HANDLE, cubeView, VK_IMAGE_LAYOUT_GENERAL };
            VkDescriptorImageInfo iSampler{ pipes.linearClampSampler, VK_NULL_HANDLE, VK_IMAGE_LAYOUT_UNDEFINED };
            VkDescriptorImageInfo iFroxel{ VK_NULL_HANDLE, froxelView, VK_IMAGE_LAYOUT_GENERAL };
            VkDescriptorImageInfo iAmbient{ VK_NULL_HANDLE, ambientMusView, VK_IMAGE_LAYOUT_GENERAL };
            VkDescriptorImageInfo iInscatter{ VK_NULL_HANDLE, inscatterView, VK_IMAGE_LAYOUT_GENERAL };
            VkDescriptorImageInfo iSingleMie{ VK_NULL_HANDLE, singleMieView, VK_IMAGE_LAYOUT_GENERAL };
            VkDescriptorBufferInfo iUbo{ uboBuf[i], 0, sizeof(GpuPerFrameData) };

            VkWriteDescriptorSet w[16]{};
            auto imgWrite = [&](int idx, uint32_t binding, VkDescriptorType type, VkDescriptorImageInfo* info) {
                w[idx].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET; w[idx].dstSet = descSet[i];
                w[idx].dstBinding = binding; w[idx].descriptorType = type; w[idx].descriptorCount = 1; w[idx].pImageInfo = info;
            };
            imgWrite(0, 0, VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, &iTrans);
            imgWrite(1, 1, VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE, &iTrans);
            imgWrite(2, 2, VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, &iSky);
            imgWrite(3, 3, VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE, &iSky);
            imgWrite(4, 4, VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, &iMulti);
            imgWrite(5, 5, VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE, &iMulti);
            imgWrite(6, 6, VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE, &iTrans);
            imgWrite(7, 7, VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, &iCube);
            w[8].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET; w[8].dstSet = descSet[i];
            w[8].dstBinding = 8; w[8].descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER; w[8].descriptorCount = 1; w[8].pBufferInfo = &iUbo;
            imgWrite(9, 9, VK_DESCRIPTOR_TYPE_SAMPLER, &iSampler);
            imgWrite(10, 10, VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, &iFroxel);
            imgWrite(11, 11, VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, &iAmbient);
            imgWrite(12, 12, VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, &iInscatter);
            imgWrite(13, 13, VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, &iSingleMie);
            imgWrite(14, 14, VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE, &iInscatter);
            imgWrite(15, 15, VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE, &iSingleMie);
            vkUpdateDescriptorSets(ctx.device, 16, w, 0, nullptr);
        }
        return true;
    }

    // 渲染侧采样 Set（Set 1）：0=depth 1=Trans 2=SkyView 3=sampler 4=HDR 5=Inscatter 6=Full Mie
    bool createRenderDescriptors(VulkanContext& ctx, const VkAtmoLutPipelines& pipes) {
        VkDescriptorPoolSize sizes[] = {
            { VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE, 6 * FRAMES_IN_FLIGHT },
            { VK_DESCRIPTOR_TYPE_SAMPLER, 1 * FRAMES_IN_FLIGHT },
        };
        VkDescriptorPoolCreateInfo pci{ VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO };
        pci.maxSets = FRAMES_IN_FLIGHT; pci.poolSizeCount = 2; pci.pPoolSizes = sizes;
        if (vkCreateDescriptorPool(ctx.device, &pci, nullptr, &renderPool) != VK_SUCCESS) return false;

        for (int i=0;i<FRAMES_IN_FLIGHT;i++) {
            VkDescriptorSetAllocateInfo ai{ VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO };
            ai.descriptorPool = renderPool; ai.descriptorSetCount = 1; ai.pSetLayouts = &pipes.renderSetLayout;
            if (vkAllocateDescriptorSets(ctx.device, &ai, &renderSet[i]) != VK_SUCCESS) return false;

            VkDescriptorImageInfo iDepthPlaceholder{ VK_NULL_HANDLE, view_[0], VK_IMAGE_LAYOUT_GENERAL };
            VkDescriptorImageInfo iTrans{ VK_NULL_HANDLE, view_[0], VK_IMAGE_LAYOUT_GENERAL };
            VkDescriptorImageInfo iSky  { VK_NULL_HANDLE, view_[2], VK_IMAGE_LAYOUT_GENERAL };
            VkDescriptorImageInfo iScenePlaceholder{ VK_NULL_HANDLE, view_[0], VK_IMAGE_LAYOUT_GENERAL };
            VkDescriptorImageInfo iInscatter{ VK_NULL_HANDLE, inscatterView, VK_IMAGE_LAYOUT_GENERAL };
            VkDescriptorImageInfo iSingleMie{ VK_NULL_HANDLE, singleMieView, VK_IMAGE_LAYOUT_GENERAL };
            VkDescriptorImageInfo iSampler{ pipes.linearClampSampler, VK_NULL_HANDLE, VK_IMAGE_LAYOUT_UNDEFINED };

            VkWriteDescriptorSet w[7]{};
            auto imgWrite = [&](int idx, uint32_t binding, VkDescriptorType type, VkDescriptorImageInfo* info) {
                w[idx].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET; w[idx].dstSet = renderSet[i];
                w[idx].dstBinding = binding; w[idx].descriptorType = type; w[idx].descriptorCount = 1; w[idx].pImageInfo = info;
            };
            imgWrite(0, 0, VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE, &iDepthPlaceholder);
            imgWrite(1, 1, VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE, &iTrans);
            imgWrite(2, 2, VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE, &iSky);
            imgWrite(3, 3, VK_DESCRIPTOR_TYPE_SAMPLER, &iSampler);
            imgWrite(4, 4, VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE, &iScenePlaceholder);
            imgWrite(5, 5, VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE, &iInscatter);
            imgWrite(6, 6, VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE, &iSingleMie);
            vkUpdateDescriptorSets(ctx.device, 7, w, 0, nullptr);
        }
        return true;
    }
};

// ==========================================================================
// VkAtmoLutCache — 按 bodyIdx（每个星球稳定的场景索引，见 CelestialDraw::bodyIdx）
// 缓存每颗星球的 VkAtmosphereLut 实例。首次遇到某星球时创建资源并做一次 bakeFull()；
// 之后每次 getOrCreate() 都会比较 profile 指纹，只有变化了（比如调参改了 Rayleigh
// 系数/半径）才重新 bakeFull()，否则直接复用已缓存的 Transmittance/MultiScatter。
//
// Sky-View/Irradiance 的逐帧刷新不在这里做（那是"每帧对哪些星球有意义"的调用方
// 决策，见 vk_renderer3d.h），调用方拿到返回的 VkAtmosphereLut* 后自行按需调用
// bakeSkyViewOnly()。
// ==========================================================================
struct VkAtmoLutCache {
    VkAtmoLutPipelines pipelines;
    std::unordered_map<int32_t, VkAtmosphereLut> byBodyIdx;

    bool init(VulkanContext& ctx) {
        return pipelines.init(ctx);
    }

    void shutdown(VulkanContext& ctx) {
        for (auto& kv : byBodyIdx) kv.second.shutdown(ctx);
        byBodyIdx.clear();
        pipelines.shutdown(ctx);
    }

    // 取得（必要时创建 + 首次 bakeFull）某星球的 LUT 实例；profile 指纹变了会
    // 自动重烤 Transmittance/MultiScatter。失败返回 nullptr（调用方应跳过该星球
    // 这一帧的大气渲染，而不是崩溃——LUT 创建失败通常意味着显存/句柄耗尽）。
    VkAtmosphereLut* getOrCreate(VulkanContext& ctx, VkCommandBuffer cmd, int frameSlot,
                                  int32_t bodyIdx, const AtmoLutProfileKey& key,
                                  const GpuPerFrameData& fd) {
        auto it = byBodyIdx.find(bodyIdx);
        if (it == byBodyIdx.end()) {
            VkAtmosphereLut lut{};
            if (!lut.init(ctx, pipelines)) {
                fprintf(stderr, "[VkAtmoLutCache] Failed to init LUT instance for bodyIdx=%d\n", bodyIdx);
                return nullptr;
            }
            it = byBodyIdx.emplace(bodyIdx, lut).first;
            printf("[VkAtmoLutCache] New cached LUT instance: bodyIdx=%d atmoIdx=%d radius=%.1fkm\n",
                   bodyIdx, key.atmoIdx, key.radiusKm);
        }

        VkAtmosphereLut& lut = it->second;
        if (lut.bakedProfile.differsFrom(key)) {
            lut.bakeFull(ctx, cmd, frameSlot, pipelines, fd);
            lut.bakedProfile = key;
        }
        return &lut;
    }

    // 强制下一次 getOrCreate() 对所有已缓存星球重烤 Transmittance/MultiScatter——
    // 供 imgui_atmo_tuner.h 的"强制重烤"按钮用。
    void invalidateAll() {
        for (auto& kv : byBodyIdx) kv.second.bakedProfile.atmoIdx = -1;
    }
};
