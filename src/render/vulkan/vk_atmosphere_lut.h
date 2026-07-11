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
                                        float cloudMinAltKm, float cloudMaxAltKm, float cloudExtinction) {
    const PlanetScatteringCoeffs pc = getPlanetScatteringCoeffs(atmoIdx);
    const float (&rayleighCoeff)[3] = pc.rayleighCoeff;
    const float mieCoeff = pc.mieCoeff;
    const float hRayleigh = pc.hRayleigh, hMie = pc.hMie, gMie = pc.gMie;
    const float (&ozoneCoeff)[3] = pc.ozoneCoeff;
    const float ozoneCenter = pc.ozoneCenter, ozoneWidth = pc.ozoneWidth;

    cfg.atmospherePreExposure = 1.0f;
    cfg.bottomRadius = surfaceRadiusKm;
    cfg.topRadius    = surfaceRadiusKm + atmosphereThicknessKm;
    cfg.multipleScatteringFactor = 1.0f;
    cfg.miePhaseFunctionG = gMie;
    cfg.viewRayMarchMinSPP = 4;
    cfg.viewRayMarchMaxSPP = 14;
    cfg.groundAlbedo[0] = cfg.groundAlbedo[1] = cfg.groundAlbedo[2] = 0.3f;

    // color*length 分解：直接把系数塞进 color，length 恒为 1，与
    // shared_atmosphere.glsl::getAtmosphereParameters() 的读法（color*length）对应。
    for (int i=0;i<3;i++) cfg.rayleighScatteringColor[i] = rayleighCoeff[i];
    cfg.rayleighScatterLength = 1.0f;
    for (int i=0;i<3;i++) cfg.mieScatteringColor[i] = mieCoeff;
    cfg.mieScatteringLength = 1.0f;
    // Mie 吸收取散射的 10% 作为近似（rs3d 原模型未拆分 mie 散射/吸收）。
    for (int i=0;i<3;i++) cfg.mieAbsColor[i] = mieCoeff * 0.10f;
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
    // 行星尺度关键：cloudMap() 里这个 scale 直接乘在"公里"量级的 posKm 上
    // （posKm 现在是相对行星中心的真实公里坐标，不再是近地小范围坐标）。
    // flower 原版 0.4/1.2 是给"几公里量级"的近地场景调的，直接套用在几千公里量级的
    // 星球上会让 128³/32³ 噪声纹理在屏幕空间里重复几千次——compute shader 里 texture()
    // 没有隐式 mip LOD（噪声纹理本身也只建了 1 级 mip），会退化成极端欠采样，
    // 看起来就是满屏白色雪花噪点而不是云。这里把频率降到"一个纹理周期对应几百公里"，
    // 对应真实天气系统的尺度。
    cfg.cloudBasicNoiseScale = 0.4f;
    cfg.cloudDetailNoiseScale = 1.2f;
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
    (void)cloudExtinction; // gCloudExtinction 由 cloud_common.glsl 侧的 CloudTuneParams 提供，这里不重复
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
                                             const float lightDirToSun[3]) {
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

    fillPlanetAtmosphereProfile(fd.sky.atmosphereConfig, atmoIdx, radiusKm, thicknessKm, 0.0f, 0.0f, 0.0f);
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
        destroyPipe(froxelPipe);
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

    // 11-binding 布局（0-7 图像 + 8 UBO + 9 采样器 + 10 Froxel 3D 图像）与具体星球
    // 无关，只建一次。binding 10 只有 aerial_perspective_lut.glsl 真正用，其余四个
    // bake shader 通过 atmosphere_common.glsl 的 #include 也会看到这个声明，但
    // shader 体内不引用它，不影响各自的 SPIR-V——复用同一个 pipelineLayout，不用
    // 为这一个 bake 单独开一套 descriptor set。
    bool createSetLayout(VulkanContext& ctx) {
        VkDescriptorSetLayoutBinding b[11]{};
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
        b[10] = { 10, VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr }; // Froxel（aerial_perspective_lut.glsl）
        VkDescriptorSetLayoutCreateInfo lci{ VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO };
        lci.bindingCount = 11; lci.pBindings = b;
        if (vkCreateDescriptorSetLayout(ctx.device, &lci, nullptr, &setLayout) != VK_SUCCESS) return false;

        VkPipelineLayoutCreateInfo plci{ VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO };
        plci.setLayoutCount = 1; plci.pSetLayouts = &setLayout;
        return vkCreatePipelineLayout(ctx.device, &plci, nullptr, &pipelineLayout) == VK_SUCCESS;
    }

    // 渲染侧 4-binding 布局：0=场景深度(只读) 1=Transmittance LUT 2=Sky-View LUT 3=采样器。
    // 只给 atmo_inside.frag / atmo_shell.frag 用，VERTEX+FRAGMENT 都声明可见性方便
    // 以后壳网格顶点着色器如果也要查表（比如按海拔做顶点位移）不用再改布局。
    bool createRenderSetLayout(VulkanContext& ctx) {
        VkDescriptorSetLayoutBinding b[4]{};
        b[0] = { 0, VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE, 1, VK_SHADER_STAGE_FRAGMENT_BIT, nullptr }; // 场景深度
        b[1] = { 1, VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE, 1, VK_SHADER_STAGE_FRAGMENT_BIT, nullptr }; // Transmittance LUT
        b[2] = { 2, VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE, 1, VK_SHADER_STAGE_FRAGMENT_BIT, nullptr }; // Sky-View LUT
        b[3] = { 3, VK_DESCRIPTOR_TYPE_SAMPLER, 1, VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT, nullptr };
        VkDescriptorSetLayoutCreateInfo lci{ VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO };
        lci.bindingCount = 4; lci.pBindings = b;
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
            && make("src/render/shaders/spirv/aerial_perspective_lut.comp.spv", froxelPipe);
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
    static constexpr VkFormat kFmt = VK_FORMAT_R16G16B16A16_SFLOAT;

    VkImage image_[3] = {}; VmaAllocation alloc_[3] = {}; VkImageView view_[3] = {}; // trans/multiscatter/skyview
    VkImage cubeImage = VK_NULL_HANDLE; VmaAllocation cubeAlloc = VK_NULL_HANDLE; VkImageView cubeView = VK_NULL_HANDLE;
    // Froxel 3D 体积散射（aerial_perspective_lut.glsl 写入，供 cloud_common.glsl::
    // inFroxelScatter 空气透视消费）。和相机视锥强相关，逐帧重烤（bakeFroxel()），
    // 不像 Transmittance/MultiScatter 那样能按 profile 指纹跨帧缓存。
    VkImage froxelImage = VK_NULL_HANDLE; VmaAllocation froxelAlloc = VK_NULL_HANDLE; VkImageView froxelView = VK_NULL_HANDLE;

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
    // 记录"上次真正 dispatch Transmittance/MultiScatter 时"用的大气 profile 输入。
    // VkAtmoLutCache::getOrCreate() 每次被访问都会比较当前 profile 与这三个字段，
    // 不同才触发 bakeFull()（重烤 Transmittance+MultiScatter，贵）；相同则跳过，
    // 只由调用方另外调 bakeSkyViewOnly()（逐帧、便宜）刷新相机高度/太阳角相关的部分。
    // bakedAtmoIdx=-1（未烤过）保证首次访问必定触发一次 bakeFull()。
    int32_t bakedAtmoIdx    = -1;
    float   bakedRadiusKm   = 0.0f;
    float   bakedThicknessKm= 0.0f;

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
    }

    // 每帧（对"这一帧实际要渲染"的星球）调用一次，把当前场景深度视图写进渲染侧
    // 采样 Set——Transmittance/Sky-View LUT 的 view 句柄本身不变（同一批 image，
    // 只是内容被 compute 重烤），不需要重复写；只有 depthView 会随 TAA ping-pong
    // 逐帧变化，所以每帧都要刷新这一项。
    void updateDepthBinding(VulkanContext& ctx, int frameSlot, VkImageView depthView) {
        int slot = frameSlot % FRAMES_IN_FLIGHT;
        VkDescriptorImageInfo iDepth{ VK_NULL_HANDLE, depthView, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL };
        VkWriteDescriptorSet w{ VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET };
        w.dstSet = renderSet[slot]; w.dstBinding = 0;
        w.descriptorType = VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE; w.descriptorCount = 1; w.pImageInfo = &iDepth;
        vkUpdateDescriptorSets(ctx.device, 1, &w, 0, nullptr);
    }

    VkImageView transmittanceView() const { return view_[0]; }
    VkImageView skyViewLutView()    const { return view_[2]; }
    VkImageView skyIrradianceCubeView() const { return cubeView; }
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

    // 全量重烤：Transmittance→MultiScatter→SkyView→SkyIrradiance 依次 dispatch，pass 间
    // 用 compute->compute 内存屏障同步（后者依赖前者的结果）。只应在 profile 指纹变化时调用
    // （见 VkAtmoLutCache::getOrCreate）——这是四者中真正"贵"的一次性开销。
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
        return true;
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
            { VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, 5 * FRAMES_IN_FLIGHT }, // trans/sky/multi/cube/froxel
            { VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE, 4 * FRAMES_IN_FLIGHT },
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
            VkDescriptorBufferInfo iUbo{ uboBuf[i], 0, sizeof(GpuPerFrameData) };

            VkWriteDescriptorSet w[11]{};
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
            // binding 6 (inDepth) 只在 depthBufferValue>=0 分支被采样，本管线始终传 -1，绑同一张
            // transmittance view 充数即可（未被实际读取，只是满足 layout 合法性）。
            imgWrite(6, 6, VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE, &iTrans);
            imgWrite(7, 7, VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, &iCube);
            w[8].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET; w[8].dstSet = descSet[i];
            w[8].dstBinding = 8; w[8].descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER; w[8].descriptorCount = 1; w[8].pBufferInfo = &iUbo;
            imgWrite(9, 9, VK_DESCRIPTOR_TYPE_SAMPLER, &iSampler);
            imgWrite(10, 10, VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, &iFroxel);
            vkUpdateDescriptorSets(ctx.device, 11, w, 0, nullptr);
        }
        return true;
    }

    // 渲染侧采样 Set（Set 1）：binding 0=场景深度(占位，真正视图由每帧 updateDepthBinding
    // 写入) 1=Transmittance LUT 2=Sky-View LUT 3=采样器。1/2 号绑定的 view 句柄创建后
    // 就固定不变（compute 只重写内容，不重建 image），所以这里写一次就够，不用像
    // binding 0 那样每帧刷新。
    bool createRenderDescriptors(VulkanContext& ctx, const VkAtmoLutPipelines& pipes) {
        VkDescriptorPoolSize sizes[] = {
            { VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE, 3 * FRAMES_IN_FLIGHT },
            { VK_DESCRIPTOR_TYPE_SAMPLER, 1 * FRAMES_IN_FLIGHT },
        };
        VkDescriptorPoolCreateInfo pci{ VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO };
        pci.maxSets = FRAMES_IN_FLIGHT; pci.poolSizeCount = 2; pci.pPoolSizes = sizes;
        if (vkCreateDescriptorPool(ctx.device, &pci, nullptr, &renderPool) != VK_SUCCESS) return false;

        for (int i=0;i<FRAMES_IN_FLIGHT;i++) {
            VkDescriptorSetAllocateInfo ai{ VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO };
            ai.descriptorPool = renderPool; ai.descriptorSetCount = 1; ai.pSetLayouts = &pipes.renderSetLayout;
            if (vkAllocateDescriptorSets(ctx.device, &ai, &renderSet[i]) != VK_SUCCESS) return false;

            // binding 0 占位（真正的场景深度视图在几何 pass 结束、TAA ping-pong 确定后
            // 由 updateDepthBinding() 每帧写入），先随便指一张已有的图满足 layout 合法性。
            VkDescriptorImageInfo iDepthPlaceholder{ VK_NULL_HANDLE, view_[0], VK_IMAGE_LAYOUT_GENERAL };
            VkDescriptorImageInfo iTrans{ VK_NULL_HANDLE, view_[0], VK_IMAGE_LAYOUT_GENERAL };
            VkDescriptorImageInfo iSky  { VK_NULL_HANDLE, view_[2], VK_IMAGE_LAYOUT_GENERAL };
            VkDescriptorImageInfo iSampler{ pipes.linearClampSampler, VK_NULL_HANDLE, VK_IMAGE_LAYOUT_UNDEFINED };

            VkWriteDescriptorSet w[4]{};
            auto imgWrite = [&](int idx, uint32_t binding, VkDescriptorType type, VkDescriptorImageInfo* info) {
                w[idx].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET; w[idx].dstSet = renderSet[i];
                w[idx].dstBinding = binding; w[idx].descriptorType = type; w[idx].descriptorCount = 1; w[idx].pImageInfo = info;
            };
            imgWrite(0, 0, VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE, &iDepthPlaceholder);
            imgWrite(1, 1, VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE, &iTrans);
            imgWrite(2, 2, VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE, &iSky);
            imgWrite(3, 3, VK_DESCRIPTOR_TYPE_SAMPLER, &iSampler);
            vkUpdateDescriptorSets(ctx.device, 4, w, 0, nullptr);
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
                                  int32_t bodyIdx, int32_t atmoIdx, float radiusKm, float thicknessKm,
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
                   bodyIdx, atmoIdx, radiusKm);
        }

        VkAtmosphereLut& lut = it->second;
        const bool profileChanged =
            (lut.bakedAtmoIdx != atmoIdx) ||
            (fabsf(lut.bakedRadiusKm - radiusKm) > 0.01f) ||
            (fabsf(lut.bakedThicknessKm - thicknessKm) > 0.01f);
        if (profileChanged) {
            lut.bakeFull(ctx, cmd, frameSlot, pipelines, fd);
            lut.bakedAtmoIdx     = atmoIdx;
            lut.bakedRadiusKm    = radiusKm;
            lut.bakedThicknessKm = thicknessKm;
        }
        return &lut;
    }

    // 强制下一次 getOrCreate() 对所有已缓存星球重烤 Transmittance/MultiScatter——
    // 供 imgui_atmo_tuner.h 的"强制重烤"按钮用，比如手动改了 getPlanetScatteringCoeffs()
    // 之后不想等自然的 profile 指纹比较（那只在半径/atmoIdx 变化时触发，散射系数
    // 本身改了不会被现有指纹感知到）。
    void invalidateAll() {
        for (auto& kv : byBodyIdx) kv.second.bakedAtmoIdx = -1;
    }
};
