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
// 的 10 个采样器（Set 1）。
//
// 30 个 binding 里，SDSM 级联阴影(28/29)、Froxel 空气透视(11)、Hi-Z(30)、
// G-Buffer A(5)、curl 噪声(9) 目前绑定中性占位资源（Phase 2 待接入，见
// cloud_common.glsl / cloud_composite.glsl 对应位置的注释），不影响主 march 路径。
// ==========================================================================

#include "vk_context.h"
#include "vk_frame.h"
#include "vk_mesh.h"
#include "vk_pipeline.h"
#include "vk_utils.h"
#include "vk_taa.h"
#include "vk_atmosphere_lut.h"
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
    float coverage = 0.45f;
    float density   = 1.00f;
    float basicNoiseScale  = 0.003f;
    float detailNoiseScale = 0.02f;
    float multiScatterExtinction = 0.5f;
    float multiScatterScatter    = 0.5f;
    float phaseForward  = 0.8f;
    float phaseBackward = -0.2f;
    float phaseMixFactor = 0.5f;
    float powderScale = 1.0f;
    float powderPow   = 1.0f;
    float sunLightScale = 1.0f;
    float minAltKm = 2.0f;
    float maxAltKm = 14.0f;
    float speed = 0.15f;
    // Bruneton/Hillaire 大气积分对"太阳辐照度"的物理量级要求，和引擎其余部分（atmo.frag 等）
    // 手调出来的 sunIntensity(~1.0) 未必是同一套标定；这里单独给一个宽范围倍率，
    // 用来在不改引擎全局曝光的前提下，把云的整体亮度调到合理范围（黑云多半是这里太低）。
    float sunIntensityMul = 50.0f;
    // 0=正常渲染；1/2/3/4=调试可视化（借用 cloudSunLitMapOctave 字段传给 shader，
    // 见 cloud_common.glsl::cloudColorCompute 开头的 dbgMode 分支），排查"黑云"用。
    int debugMode = 0;
};

struct VkCloudSystem {
    static constexpr VkFormat kColorFmt = VK_FORMAT_R16G16B16A16_SFLOAT;
    static constexpr VkFormat kDepthFmt  = VK_FORMAT_R32_SFLOAT;

    bool enabled = true;  // true = 新管线；false = legacy cloud.frag（vk_renderer3d.h 里判断）
    bool debugLogOnce = true;
    FlowerCloudTuneParams tuneParams;

    VkExtent2D extent     = {};
    VkExtent2D quarterExt = {};

    VkAtmosphereLut atmosphereLut;

    // ── 1/4 分辨率 RT：颜色(L,T) / 深度 / 雾 ─────────────────────────────────
    VkImage quarterColorImg=VK_NULL_HANDLE, quarterDepthImg=VK_NULL_HANDLE, quarterFogImg=VK_NULL_HANDLE;
    VmaAllocation quarterColorAlloc=VK_NULL_HANDLE, quarterDepthAlloc=VK_NULL_HANDLE, quarterFogAlloc=VK_NULL_HANDLE;
    VkImageView quarterColorView=VK_NULL_HANDLE, quarterDepthView=VK_NULL_HANDLE, quarterFogView=VK_NULL_HANDLE;

    // ── 全分辨率 RT：颜色/深度/雾，ping-pong ×2（[frameSlot] 当前写，[1-frameSlot] 当帧的 history）
    VkImage fullColorImg[2]={}, fullDepthImg[2]={}, fullFogImg[2]={};
    VmaAllocation fullColorAlloc[2]={}, fullDepthAlloc[2]={}, fullFogAlloc[2]={};
    VkImageView fullColorView[2]={}, fullDepthView[2]={}, fullFogView[2]={};

    // ── Phase 2 占位资源（SDSM/Froxel/Hi-Z/GBufferA/curl 噪声）────────────────
    VkImage dummy2DImg=VK_NULL_HANDLE; VmaAllocation dummy2DAlloc=VK_NULL_HANDLE; VkImageView dummy2DView=VK_NULL_HANDLE;
    VkImage dummy3DImg=VK_NULL_HANDLE; VmaAllocation dummy3DAlloc=VK_NULL_HANDLE; VkImageView dummy3DView=VK_NULL_HANDLE;
    VkBuffer dummyCascadeBuf=VK_NULL_HANDLE; VmaAllocation dummyCascadeAlloc=VK_NULL_HANDLE;

    // ── 噪声/天气纹理（复用 vk_scene.h 已烘焙好的资源，不重复创建）─────────────
    VkImageView basicNoiseView=VK_NULL_HANDLE, detailNoiseView=VK_NULL_HANDLE, weatherView=VK_NULL_HANDLE;

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

    bool cloudRtInitialized = false;

    // -----------------------------------------------------------------------
    bool init(VulkanContext& ctx, VkExtent2D ext,
              VkImageView basicNoise, VkImageView detailNoise, VkImageView weather) {
        extent = ext;
        quarterExt = { std::max(1u, (ext.width + 3) / 4), std::max(1u, (ext.height + 3) / 4) };
        basicNoiseView = basicNoise; detailNoiseView = detailNoise; weatherView = weather;

        if (!atmosphereLut.init(ctx)) return false;
        if (!createSamplers(ctx))     return false;
        if (!createImages(ctx))       return false;
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
        if (samplerSetLayout) { vkDestroyDescriptorSetLayout(ctx.device, samplerSetLayout, nullptr); samplerSetLayout=VK_NULL_HANDLE; }
        destroyImages(ctx);
        for (VkSampler& s : samplers10) if (s) { vkDestroySampler(ctx.device, s, nullptr); s = VK_NULL_HANDLE; }
        if (dummyCascadeBuf) { vmaDestroyBuffer(ctx.allocator, dummyCascadeBuf, dummyCascadeAlloc); dummyCascadeBuf=VK_NULL_HANDLE; }
        atmosphereLut.shutdown(ctx);
    }

    // Called after TAA endGeometryPass — HDR + depth are SHADER_READ_ONLY.
    // 在 snap.celestials 里自己找地球类(atmoIdx==3)行星，抽取中心/半径；不需要调用方额外传参。
    void renderPhase0(VulkanContext& ctx, VkCommandBuffer cmd, VkTAA& taa,
                      const SceneSnapshot& snap, int frameSlot, bool drawClouds) {
        if (!enabled) return;
        if (!drawClouds) {
            if (debugLogOnce) {
                fprintf(stderr, "[VkCloudSystem] Phase1 skip: drawClouds=false (need Earth atmoIdx=3 + showClouds)\n");
                debugLogOnce = false;
            }
            return;
        }
        if (raymarchPipe == VK_NULL_HANDLE) return;

        const CelestialDraw* earth = nullptr;
        for (const auto& cd : snap.celestials) { if (cd.atmoIdx == 3 && cd.showClouds) { earth = &cd; break; } }
        if (!earth) return;

        if (debugLogOnce) {
            printf("[VkCloudSystem] Phase1 dispatch frame %u (%ux%u)\n", snap.frameIndex, extent.width, extent.height);
            debugLogOnce = false;
        }

        int slot = frameSlot % FRAMES_IN_FLIGHT;
        int histSlot = 1 - slot;

        GpuPerFrameData fd = fillCloudFrameData(snap, *earth);
        atmosphereLut.bake(ctx, cmd, frameSlot, fd);
        updateCloudDescriptorSet(ctx, taa, slot, histSlot, fd);

        VkDescriptorSet sets[2] = { cloudSet[slot], samplerSet };

        VkImage hdrImage = taa.colorImages[taa.currentIdx];

        // Phase0 曾经每帧转换 quarter RT 的 layout；Phase1 里所有云自建 RT 都常驻 GENERAL
        // （创建时一次性转换，见 createImages），这里只需要处理场景 HDR 颜色的 layout 切换。
        transitionImage(cmd, hdrImage, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, VK_IMAGE_LAYOUT_GENERAL,
            VK_PIPELINE_STAGE_2_FRAGMENT_SHADER_BIT, VK_ACCESS_2_SHADER_READ_BIT,
            VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT, VK_ACCESS_2_SHADER_WRITE_BIT);

        vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_COMPUTE, cloudPipelineLayout, 0, 2, sets, 0, nullptr);

        vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_COMPUTE, raymarchPipe);
        dispatch2D(cmd, quarterExt);
        computeBarrier(cmd);

        vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_COMPUTE, reconstructPipe);
        dispatch2D(cmd, extent);
        computeBarrier(cmd);

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
    GpuPerFrameData fillCloudFrameData(const SceneSnapshot& snap, const CelestialDraw& earth) {
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
        // 没做单独的去抖动(NoJitter)矩阵/上一帧矩阵，直接复用当前帧——云的时域重建对此不敏感
        // （真正的 TAA 几何抖动由 vk_taa.h 单独处理，与云管线无关）。
        memcpy(fd.camProjNoJitter, fd.camProj, 64);
        memcpy(fd.camViewProjNoJitter, fd.camViewProj, 64);
        memcpy(fd.camInvertProjNoJitter, fd.camInvertProj, 64);
        memcpy(fd.camInvertViewProjNoJitter, fd.camInvertViewProj, 64);
        memcpy(fd.camViewProjPrev, fd.camViewProj, 64);
        memcpy(fd.camViewProjPrevNoJitter, fd.camViewProj, 64);

        fd.jitterPeriod = 1; fd.bEnableJitter = 0; fd.bCameraCut = 0;
        fd.skyValid = 1; fd.skySDSMValid = 0; fd.fixExposure = 1.0f; fd.bAutoExposure = 0;
        fd.renderWidth = (float)extent.width; fd.renderHeight = (float)extent.height;
        fd.displayWidth = fd.renderWidth; fd.displayHeight = fd.renderHeight;

        // 太阳方向：snap.lightDir 是"行星指向太阳"的方向（updateLightDir 用 sunWorldPos-center 填的），
        // frameData.sky.direction 约定是"光线传播方向"（从太阳照向场景），互为相反数。
        fd.sky.direction[0] = -snap.lightDir[0]; fd.sky.direction[1] = -snap.lightDir[1]; fd.sky.direction[2] = -snap.lightDir[2];
        fd.sky.color[0]=fd.sky.color[1]=fd.sky.color[2]=1.0f;
        fd.sky.intensity = snap.sunIntensity * tuneParams.sunIntensityMul;

        // Phase 2 占位：SDSM 级联数量置 0，getShadow()/God Ray 级联查找自动跳过（见
        // cloud_composite.glsl 里的说明），不需要真的渲染级联阴影贴图。
        fd.sky.cacsadeConfig.cascadeCount = 0;

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
        ac.cloudDetailNoiseScale = tuneParams.detailNoiseScale;
        ac.cloudMultiScatterExtinction = tuneParams.multiScatterExtinction;
        ac.cloudMultiScatterScatter = tuneParams.multiScatterScatter;
        ac.cloudPhaseForward = tuneParams.phaseForward;
        ac.cloudPhaseBackward = tuneParams.phaseBackward;
        ac.cloudPhaseMixFactor = tuneParams.phaseMixFactor;
        ac.cloudPowderScale = tuneParams.powderScale;
        ac.cloudPowderPow = tuneParams.powderPow;
        ac.cloudShadingSunLightScale = tuneParams.sunLightScale;
        ac.cloudSpeed = tuneParams.speed;
        ac.cloudSunLitMapOctave = tuneParams.debugMode; // 调试可视化模式开关（见 cloud_common.glsl）

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

        // ── Set 0：cloud_common.glsl binding 0-30 ──
        {
            VkDescriptorSetLayoutBinding b[31]{};
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
            img(27, VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE); // textureCube 仍是 SAMPLED_IMAGE 描述符类型
            img(28, VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE);
            b[29] = { 29, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr };
            img(30, VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE);

            VkDescriptorSetLayoutCreateInfo ci{ VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO };
            ci.bindingCount = 31; ci.pBindings = b;
            if (vkCreateDescriptorSetLayout(ctx.device, &ci, nullptr, &cloudSetLayout) != VK_SUCCESS) return false;
        }

        VkDescriptorSetLayout layouts[2] = { cloudSetLayout, samplerSetLayout };
        VkPipelineLayoutCreateInfo plci{ VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO };
        plci.setLayoutCount = 2; plci.pSetLayouts = layouts;
        if (vkCreatePipelineLayout(ctx.device, &plci, nullptr, &cloudPipelineLayout) != VK_SUCCESS) return false;

        VkDescriptorPoolSize sizes[] = {
            { VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, 8 * FRAMES_IN_FLIGHT },
            { VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE, 23 * FRAMES_IN_FLIGHT },
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
    void updateCloudDescriptorSet(VulkanContext& ctx, VkTAA& taa, int slot, int histSlot, const GpuPerFrameData&) {
        VkDescriptorSet set = cloudSet[slot];

        VkDescriptorImageInfo hdr{ VK_NULL_HANDLE, taa.colorViews[taa.currentIdx], VK_IMAGE_LAYOUT_GENERAL };
        VkDescriptorImageInfo hdrRO{ VK_NULL_HANDLE, taa.colorViews[taa.currentIdx], VK_IMAGE_LAYOUT_GENERAL };
        VkDescriptorImageInfo depth{ VK_NULL_HANDLE, taa.depthView, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL };
        VkDescriptorImageInfo gbufA{ VK_NULL_HANDLE, dummy2DView, VK_IMAGE_LAYOUT_GENERAL };
        VkDescriptorImageInfo basicN{ VK_NULL_HANDLE, basicNoiseView, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL };
        VkDescriptorImageInfo detailN{ VK_NULL_HANDLE, detailNoiseView, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL };
        VkDescriptorImageInfo weatherI{ VK_NULL_HANDLE, weatherView, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL };
        VkDescriptorImageInfo curl{ VK_NULL_HANDLE, dummy2DView, VK_IMAGE_LAYOUT_GENERAL };
        VkDescriptorImageInfo transLut{ VK_NULL_HANDLE, atmosphereLut.transmittanceView(), VK_IMAGE_LAYOUT_GENERAL };
        VkDescriptorImageInfo froxel{ VK_NULL_HANDLE, dummy3DView, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL };
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
        VkDescriptorImageInfo skyIrr{ VK_NULL_HANDLE, atmosphereLut.skyIrradianceCubeView(), VK_IMAGE_LAYOUT_GENERAL };
        VkDescriptorImageInfo sdsm{ VK_NULL_HANDLE, dummy2DView, VK_IMAGE_LAYOUT_GENERAL };
        VkDescriptorImageInfo hiz{ VK_NULL_HANDLE, dummy2DView, VK_IMAGE_LAYOUT_GENERAL };
        VkDescriptorBufferInfo ubo{ atmosphereLut.frameUBO(slot), 0, sizeof(GpuPerFrameData) };
        VkDescriptorBufferInfo cascadeSsbo{ dummyCascadeBuf, 0, sizeof(GpuCascadeInfo) };

        VkWriteDescriptorSet w[31]{};
        auto imgW = [&](int i, uint32_t binding, VkDescriptorType t, VkDescriptorImageInfo* info) {
            w[i].sType=VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET; w[i].dstSet=set; w[i].dstBinding=binding;
            w[i].descriptorType=t; w[i].descriptorCount=1; w[i].pImageInfo=info;
        };
        imgW(0,0,VK_DESCRIPTOR_TYPE_STORAGE_IMAGE,&hdr);         imgW(1,1,VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE,&hdrRO);
        imgW(2,2,VK_DESCRIPTOR_TYPE_STORAGE_IMAGE,&qColorRW);    imgW(3,3,VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE,&qColorRO);
        imgW(4,4,VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE,&depth);       imgW(5,5,VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE,&gbufA);
        imgW(6,6,VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE,&basicN);      imgW(7,7,VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE,&detailN);
        imgW(8,8,VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE,&weatherI);    imgW(9,9,VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE,&curl);
        imgW(10,10,VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE,&transLut);  imgW(11,11,VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE,&froxel);
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

        vkUpdateDescriptorSets(ctx.device, 31, w, 0, nullptr);
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
