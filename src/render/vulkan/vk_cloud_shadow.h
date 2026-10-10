#pragma once
// ==========================================================================
// vk_cloud_shadow.h — 云→地表阴影级联系统（Cloud Cascaded Shadow Maps）
//
// 单一职责：维护「太阳方向的云透过率级联图集」这一份资源。每帧
//   1) 按相机高度 / 太阳方向算出各级正交盒矩阵；
//   2) dispatch 一次 compute，把 Beer-Lambert 透过率 T=exp(-τ) 积分进图集；
//   3) 把级联描述（矩阵 + 图集切片）上传成 UBO，供任意地表着色器采样。
//
// 与外界的接口全部是显式结构 / 访问器，本模块不反向依赖任何调用方：
//   输入   CloudShadowFrameInput   — 相机、太阳、行星与云层几何（纯数据，无 Vulkan 类型）
//   输出   atlasView() / atlasSampler() / terrainUbo() — 给地表管线绑定
//   协作   render(...)             — 借用云管线的 descriptor set 跑 compute
//                                    （阴影 compute 要读云噪声/天气图，与主云管线同一套）
//
// GLSL 侧契约见 shaders/common/shared_cloud_shadow.glsl，结构体逐字段对应。
//
// ── 几何/物理设计 ──────────────────────────────────────────────────────────
// 阴影落在地面上，所以级联按「地面圆盘半径」分级，而不是按视锥深度分级
// （后者是给几何 CSM 用的，对贴地效果不自然）。第 c 级覆盖以相机地面天底点
// 为心、半径 R_c 的圆盘，R_c 从 nearRadiusKm 按实用拆分方案递增到地平线距离。
//
// 每级的正交盒取「以锚点为心、半径 = R_c + 云顶高度 + 地形起伏余量」的包围球
// 沿太阳方向的切片。这样盒内必然同时容纳：
//   (a) 半径 R_c 的地面圆盘（接收面）
//   (b) 该圆盘正上方的整层云壳（投射体）
// 且对任意太阳仰角都成立 —— 低太阳角的长斜程不会把投射云柱漏在盒外，不需要
// 按仰角特判。横向半宽取 R_c 即可：地面圆盘在垂直于太阳的投影面上是一个短半轴
// R_c·sin(仰角) 的椭圆，永远被 R_c 包住。
// ==========================================================================

#include "vk_context.h"
#include "vk_frame.h"
#include "vk_mesh.h"      // beginSingleTimeCommands / endSingleTimeCommands
#include "vk_pipeline.h"  // loadSPIRV / createShaderModule
#include "vk_utils.h"     // transitionImage
#include "../../math/math3d.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>

// --------------------------------------------------------------------------
// Tuner 可调参数（imgui_cloud_tuner.h 直接编辑本结构）
// --------------------------------------------------------------------------
struct CloudShadowTuneParams {
    bool  enabled       = true;   // 总开关；关闭时地表拿到的 config.x=0，采样直接返回 1
    float strength      = 1.0f;   // 阴影强度：1=纯 Beer-Lambert 物理值，<1 为美术减弱

    // 下面两项决定图集显存与 dispatch 规模，只在 initResources() 时生效，
    // 运行时改需要重建（窗口 resize 会顺带重建）。
    int   cascadeCount  = 4;      // 级数 [1, kMaxCascades]
    int   perCascadeDim = 256;    // 中文说明：边长减半使图集像素数降至四分之一，降低每帧阴影预算。

    float nearRadiusKm  = 4.0f;   // 第 0 级的地面圆盘半径
    float maxRadiusKm   = 3000.0f;// 最远级半径上限（地平线更近时以地平线为准）
    float minFarRadiusKm= 60.0f;  // 最远级半径下限：贴地时地平线只有几 km，但远处高山
                                  // 仍在视野里，需要一个保底覆盖范围
    float splitLambda   = 0.85f;  // 0=均匀拆分，1=对数拆分（近级更密）
    float extentMul     = 1.0f;   // 整体半径倍率（调试用）
    float blendStart    = 0.85f;  // 跨级混合起点（归一化偏心度）
    int   marchSteps    = 24;     // 中文说明：配合 256 分辨率，默认最大采样量约为原来的八分之一。
};

// --------------------------------------------------------------------------
// GPU 镜像（std140），逐字段对应 shared_cloud_shadow.glsl
// --------------------------------------------------------------------------
struct GpuCloudShadowCascade {
    float viewProj[16];    // 相对行星中心 km → 级联内 [0,1] UV + 深度（已含 bias）
    float viewProjInv[16];
    float params[4];       // x=半径 km  y=texel 边长 km  z=图集 U 起点  w=图集 U 宽度
};
static_assert(sizeof(GpuCloudShadowCascade) == 144, "must match GLSL CloudShadowCascade std140");

struct GpuCloudShadowData {
    static constexpr int kMaxCascades = 4;
    GpuCloudShadowCascade cascades[kMaxCascades]{};
    float config[4]{};      // x=级数 y=强度 z=云底高度 km w=混合起点
    float marchConfig[4]{}; // x=积分步数 y/z/w=预留
};
static_assert(sizeof(GpuCloudShadowData) == 608, "must match GLSL CloudShadowData std140");

// --------------------------------------------------------------------------
// 每帧输入（纯数据：不含 Vulkan 类型，便于单测与复用）
// --------------------------------------------------------------------------
struct CloudShadowFrameInput {
    float camRelPlanetKm[3]   = {0,0,0}; // 相机相对行星中心（km）
    float sunDirToPlanet[3]   = {0,1,0}; // 行星→太阳的方向（不必归一化）
    float planetRadiusKm      = 6371.0f; // 地表半径
    float cloudBottomRadiusKm = 6372.0f; // 云层底（从行星中心量）
    float cloudTopRadiusKm    = 6391.0f; // 云层顶（从行星中心量）
    float terrainReliefKm     = 10.0f;   // 地形起伏余量（山峰高出基准面的量级）
};

// --------------------------------------------------------------------------
// 级联半径拆分：从 nearKm 到 farKm 的实用拆分方案（practical split scheme）。
// lambda=0 纯均匀，=1 纯对数（近级更密）。与 vk_shadow_cascade.h 的
// computeCascadeSplits 同源，但这里切的是「地面圆盘半径」而不是视锥深度，
// 两者语义不同，故各自独立一份，避免把不相关的模块耦合起来。
// --------------------------------------------------------------------------
inline void computeCloudShadowRadii(float nearKm, float farKm, int count, float lambda, float* outRadii) {
    nearKm = std::max(nearKm, 0.01f);
    farKm  = std::max(farKm, nearKm * 1.01f);
    for (int i = 0; i < count; i++) {
        float p = (float)(i + 1) / (float)count;
        float logR     = nearKm * powf(farKm / nearKm, p);
        float uniformR = nearKm + (farKm - nearKm) * p;
        outRadii[i] = lambda * logR + (1.0f - lambda) * uniformR;
    }
}

// ==========================================================================
// VkCloudShadowSystem
// ==========================================================================
struct VkCloudShadowSystem {
    static constexpr int      kMaxCascades = GpuCloudShadowData::kMaxCascades;
    static constexpr VkFormat kAtlasFormat = VK_FORMAT_R32_SFLOAT;

    CloudShadowTuneParams tune;

    // ---------------------------------------------------------------------
    // 生命周期
    //
    // 分两阶段是因为存在环形依赖：compute pipeline 需要云管线的 descriptor
    // set layout，而那个 layout 又要先绑定本模块的图集与 UBO。于是
    //   initResources() → 云侧建 descriptor → initPipeline()
    // ---------------------------------------------------------------------
    bool initResources(VulkanContext& ctx) {
        _cascadeCount  = std::min(std::max(tune.cascadeCount, 1), kMaxCascades);
        _perCascadeDim = std::max(tune.perCascadeDim, 16);
        if (!createAtlas(ctx))   return false;
        if (!createUniforms(ctx))return false;
        if (!createSampler(ctx)) return false;
        printf("[VkCloudShadow] Atlas ready: %d cascades x %dpx (R32F)\n", _cascadeCount, _perCascadeDim);
        return true;
    }

    bool initPipeline(VulkanContext& ctx, VkPipelineLayout cloudPipelineLayout) {
        auto code = loadSPIRV("src/render/shaders/spirv/cloud_shadow.comp.spv");
        if (code.empty()) { fprintf(stderr, "[VkCloudShadow] Missing cloud_shadow.comp.spv\n"); return false; }
        VkShaderModule mod = createShaderModule(ctx.device, code);
        if (mod == VK_NULL_HANDLE) return false;

        VkPipelineShaderStageCreateInfo stage{ VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO };
        stage.stage = VK_SHADER_STAGE_COMPUTE_BIT; stage.module = mod; stage.pName = "main";
        VkComputePipelineCreateInfo cpci{ VK_STRUCTURE_TYPE_COMPUTE_PIPELINE_CREATE_INFO };
        cpci.stage = stage; cpci.layout = cloudPipelineLayout;
        VkResult res = vkCreateComputePipelines(ctx.device, VK_NULL_HANDLE, 1, &cpci, nullptr, &_pipeline);
        vkDestroyShaderModule(ctx.device, mod, nullptr);
        return res == VK_SUCCESS;
    }

    void destroyPipeline(VkDevice dev) {
        if (_pipeline) { vkDestroyPipeline(dev, _pipeline, nullptr); _pipeline = VK_NULL_HANDLE; }
    }

    void destroyResources(VulkanContext& ctx) {
        for (int i = 0; i < FRAMES_IN_FLIGHT; i++) {
            if (_computeUbo[i]) { vmaDestroyBuffer(ctx.allocator, _computeUbo[i], _computeUboAlloc[i]); _computeUbo[i] = VK_NULL_HANDLE; }
            if (_terrainUbo[i]) { vmaDestroyBuffer(ctx.allocator, _terrainUbo[i], _terrainUboAlloc[i]); _terrainUbo[i] = VK_NULL_HANDLE; }
        }
        if (_sampler)   { vkDestroySampler(ctx.device, _sampler, nullptr); _sampler = VK_NULL_HANDLE; }
        if (_atlasView) { vkDestroyImageView(ctx.device, _atlasView, nullptr); _atlasView = VK_NULL_HANDLE; }
        if (_atlasImage){ vmaDestroyImage(ctx.allocator, _atlasImage, _atlasAlloc); _atlasImage = VK_NULL_HANDLE; }
        _atlasReadOnly = true;
        _rendered = false;
    }

    void shutdown(VulkanContext& ctx) {
        destroyPipeline(ctx.device);
        destroyResources(ctx);
    }

    // ---------------------------------------------------------------------
    // 每帧：算级联 → 上传 compute UBO → dispatch → 转回只读
    //
    // cloudPipelineLayout / cloudSet：阴影 compute 复用主云管线的 Set0/1/2
    // （它要读同一套噪声、天气图和 PerFrameData），由云系统在调用前绑好。
    // 调用方需保证 cloudSet 里 binding 38 = atlasView、binding 39 = computeUbo(slot)。
    // ---------------------------------------------------------------------
    void render(VkCommandBuffer cmd, const CloudShadowFrameInput& in, int slot) {
        if (!tune.enabled || _pipeline == VK_NULL_HANDLE || _atlasImage == VK_NULL_HANDLE) {
            markInactive();
            // Composite statically declares binding 38 even when config.x is zero.
            if (_atlasImage && _atlasReadOnly) {
                transitionImage(cmd, _atlasImage,
                    VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, VK_IMAGE_LAYOUT_GENERAL,
                    VK_PIPELINE_STAGE_2_FRAGMENT_SHADER_BIT, VK_ACCESS_2_SHADER_READ_BIT,
                    VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT, VK_ACCESS_2_SHADER_READ_BIT);
                _atlasReadOnly = false;
            }
            GpuCloudShadowData off{};
            if (_computeUboMapped[slot % FRAMES_IN_FLIGHT])
                memcpy(_computeUboMapped[slot % FRAMES_IN_FLIGHT], &off, sizeof(off));
            return;
        }
        slot %= FRAMES_IN_FLIGHT;

        buildCascades(in);
        memcpy(_computeUboMapped[slot], &_gpuData, sizeof(GpuCloudShadowData));

        transitionImage(cmd, _atlasImage,
            _atlasReadOnly ? VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL : VK_IMAGE_LAYOUT_GENERAL,
            VK_IMAGE_LAYOUT_GENERAL,
            VK_PIPELINE_STAGE_2_FRAGMENT_SHADER_BIT, VK_ACCESS_2_SHADER_READ_BIT,
            VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT, VK_ACCESS_2_SHADER_WRITE_BIT | VK_ACCESS_2_SHADER_READ_BIT);
        _atlasReadOnly = false;

        vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_COMPUTE, _pipeline);
        vkCmdDispatch(cmd, (atlasWidth() + 7) / 8, (atlasHeight() + 7) / 8, 1);

        // Remain GENERAL for same-frame God Ray storage-image reads.
        _rendered = true;
    }

    void finishComputeReads(VkCommandBuffer cmd) {
        if (!_atlasImage || _atlasReadOnly) return;
        transitionImage(cmd, _atlasImage,
            VK_IMAGE_LAYOUT_GENERAL, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
            VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT, VK_ACCESS_2_SHADER_WRITE_BIT | VK_ACCESS_2_SHADER_READ_BIT,
            VK_PIPELINE_STAGE_2_FRAGMENT_SHADER_BIT, VK_ACCESS_2_SHADER_READ_BIT);
        _atlasReadOnly = true;
    }

    // 云本帧没渲染（关闭 / 不在地球）：让地表侧立刻退化成无阴影，
    // 否则会拿旧图集按新相机位置采样，出现错位的残留云影。
    void markInactive() {
        _rendered = false;
        _gpuData.config[0] = 0.0f;
    }

    // ---------------------------------------------------------------------
    // 地表消费端：把「上一次成功渲染的那批级联」写进 terrain UBO。
    //
    // 为什么单独一份 UBO：地形几何 pass 跑在云 compute pass 之前，因此它采样到的
    // 图集内容来自上一帧，必须配上一帧的矩阵才对得上。compute 侧的 UBO 存的是
    // 本帧矩阵，两者内容不同、写入时机也不同，共用一个 buffer 会有竞态。
    // 返回值 = 本帧地表是否真的有阴影可用。
    // ---------------------------------------------------------------------
    bool uploadTerrainData(int slot) {
        slot %= FRAMES_IN_FLIGHT;
        if (!_terrainUboMapped[slot]) return false;
        if (!_rendered) {
            GpuCloudShadowData off{};   // config.x = 0 → 采样函数直接返回 1
            memcpy(_terrainUboMapped[slot], &off, sizeof(off));
            return false;
        }
        memcpy(_terrainUboMapped[slot], &_gpuData, sizeof(_gpuData));
        return true;
    }

    // ---------------------------------------------------------------------
    // 访问器
    // ---------------------------------------------------------------------
    VkImageView atlasView()    const { return _atlasView; }
    VkSampler   atlasSampler() const { return _sampler; }
    VkBuffer    computeUbo(int slot) const { return _computeUbo[slot % FRAMES_IN_FLIGHT]; }
    VkBuffer    terrainUbo(int slot) const { return _terrainUbo[slot % FRAMES_IN_FLIGHT]; }
    bool        active()       const { return _rendered; }
    int         cascadeCount() const { return _cascadeCount; }
    int         perCascadeDim()const { return _perCascadeDim; }
    uint32_t    atlasWidth()   const { return (uint32_t)(_perCascadeDim * _cascadeCount); }
    uint32_t    atlasHeight()  const { return (uint32_t)_perCascadeDim; }
    // 最远级半径，供 UI 显示实际覆盖范围
    float       farRadiusKm()  const { return _gpuData.cascades[_cascadeCount - 1].params[0]; }

private:
    // ---------------------------------------------------------------------
    // 级联矩阵构建（本模块的核心几何逻辑，见文件头「几何/物理设计」）
    // ---------------------------------------------------------------------
    void buildCascades(const CloudShadowFrameInput& in) {
        Vec3 camRel((double)in.camRelPlanetKm[0], (double)in.camRelPlanetKm[1], (double)in.camRelPlanetKm[2]);
        double camDist = camRel.length();

        // 径向单位向量：相机在行星中心（退化输入）时取任意固定轴，避免 NaN 扩散。
        Vec3 up = (camDist > 1e-6) ? (camRel * (1.0 / camDist)) : Vec3(0.0, 1.0, 0.0);

        Vec3 sunDir((double)in.sunDirToPlanet[0], (double)in.sunDirToPlanet[1], (double)in.sunDirToPlanet[2]);
        double sunLen = sunDir.length();
        sunDir = (sunLen > 1e-6) ? (sunDir * (1.0 / sunLen)) : up;

        // 锚点 = 相机天底点（相机径向在地表的投影）。阴影落在地面，锚在地面比锚在
        // 相机更稳定：相机拉高时级联不会整体飘走，地面覆盖区始终在视野正下方。
        const double planetR = (double)in.planetRadiusKm;
        Vec3 anchor = up * planetR;

        const double camAltKm  = std::max(camDist - planetR, 0.0);
        const double cloudTopAlt = std::max((double)in.cloudTopRadiusKm - planetR, 0.1);
        const double cloudBotAlt = std::max((double)in.cloudBottomRadiusKm - planetR, 0.0);

        // 最远级半径：以地平线距离为物理上界（再远的地面本来就看不见），
        // 但设一个下限兜住「贴地时地平线只有几 km、远处高山却仍在视野里」的情况。
        const double horizonKm = std::sqrt(camAltKm * (2.0 * planetR + camAltKm));
        float farRadius = (float)std::max(horizonKm, (double)tune.minFarRadiusKm);
        farRadius = std::min(farRadius, tune.maxRadiusKm) * std::max(tune.extentMul, 0.01f);

        float nearRadius = std::min(std::max(tune.nearRadiusKm, 0.5f), farRadius * 0.5f);

        float radii[kMaxCascades] = {};
        computeCloudShadowRadii(nearRadius, farRadius, _cascadeCount, tune.splitLambda, radii);

        // lookAt 的 worldUp 只用来定侧向；太阳方向与径向近乎平行时（太阳接近天顶）
        // 换一个不平行的轴，否则 f×worldUp 退化成零向量。
        Vec3 refUp = (std::abs(sunDir.dot(up)) < 0.99) ? up : Vec3(1.0, 0.0, 0.0);
        if (std::abs(sunDir.dot(refUp)) > 0.999) refUp = Vec3(0.0, 0.0, 1.0);

        // 盒的横向基底（与 lookAt 内部算法一致）。用来做纹素吸附。
        Vec3 fwd   = -sunDir;                        // 视线前向 = 太阳→行星
        Vec3 axisR = fwd.cross(refUp).normalized();
        Vec3 axisU = axisR.cross(fwd);

        // [-1,1] → [0,1]：UV 与深度都在 shader 里直接用，消费端不必再换算。
        Mat4 bias;
        bias.m[0] = 0.5; bias.m[5] = 0.5; bias.m[10] = 0.5;
        bias.m[12] = 0.5; bias.m[13] = 0.5; bias.m[14] = 0.5;

        const float uScale = 1.0f / (float)_cascadeCount;

        for (int c = 0; c < _cascadeCount; c++) {
            const double R = (double)radii[c];

            // 包围球半径：覆盖地面圆盘 + 其上方整层云壳 + 地形起伏。沿任意太阳方向
            // 的深度跨度都是 2R_bound，于是低太阳角也不会漏掉投射云柱。
            const double boundR = R + cloudTopAlt + (double)in.terrainReliefKm;

            // 纹素吸附：把盒心在横向两轴上的坐标量化到整纹素格。相机连续移动时盒子
            // 按整纹素跳，同一块地面每帧落在同一个 texel 里，阴影边缘不会逐帧抖动
            //（texel swimming）。只消得掉平移抖动；相机绕行星移动导致 refUp 转动的那
            // 部分残留很小，而云影本身是低频软边，肉眼看不出来。
            const double texelKm = 2.0 * R / (double)_perCascadeDim;
            const double projR = anchor.dot(axisR);
            const double projU = anchor.dot(axisU);
            Vec3 center = anchor
                        + axisR * (std::round(projR / texelKm) * texelKm - projR)
                        + axisU * (std::round(projU / texelKm) * texelKm - projU);

            Vec3 eye = center + sunDir * boundR;
            Mat4 view = Mat4::lookAt(eye, center, refUp);
            Mat4 proj = Mat4::ortho(-R, R, -R, R, 0.01, 2.0 * boundR);
            Mat4 vpBias = bias * (proj * view);

            GpuCloudShadowCascade& gc = _gpuData.cascades[c];
            vpBias.toFloatArray(gc.viewProj);
            vpBias.inverse().toFloatArray(gc.viewProjInv);
            gc.params[0] = (float)R;                                  // 半径 km
            gc.params[1] = (float)(2.0 * R / (double)_perCascadeDim); // texel 边长 km
            gc.params[2] = (float)c * uScale;                         // 图集 U 起点
            gc.params[3] = uScale;                                    // 图集 U 宽度
        }
        // 未使用的级填成单位量，避免 shader 里读到未初始化数据（config.x 已经挡住了，
        // 这里是第二道保险）。
        for (int c = _cascadeCount; c < kMaxCascades; c++) _gpuData.cascades[c] = GpuCloudShadowCascade{};

        _gpuData.config[0] = (float)_cascadeCount;
        _gpuData.config[1] = std::min(std::max(tune.strength, 0.0f), 1.0f);
        _gpuData.config[2] = (float)cloudBotAlt;                        // 半影宽度用
        _gpuData.config[3] = std::min(std::max(tune.blendStart, 0.0f), 0.99f);
        _gpuData.marchConfig[0] = (float)std::min(std::max(tune.marchSteps, 4), 256);
    }

    // ---------------------------------------------------------------------
    // 资源创建
    // ---------------------------------------------------------------------
    bool createAtlas(VulkanContext& ctx) {
        VkImageCreateInfo ici{ VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO };
        ici.imageType = VK_IMAGE_TYPE_2D;
        ici.format    = kAtlasFormat;
        ici.extent    = { atlasWidth(), atlasHeight(), 1 };
        ici.mipLevels = 1; ici.arrayLayers = 1; ici.samples = VK_SAMPLE_COUNT_1_BIT;
        ici.tiling    = VK_IMAGE_TILING_OPTIMAL;
        ici.usage     = VK_IMAGE_USAGE_STORAGE_BIT | VK_IMAGE_USAGE_SAMPLED_BIT;
        ici.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
        VmaAllocationCreateInfo aci{}; aci.usage = VMA_MEMORY_USAGE_GPU_ONLY;
        if (vmaCreateImage(ctx.allocator, &ici, &aci, &_atlasImage, &_atlasAlloc, nullptr) != VK_SUCCESS) {
            fprintf(stderr, "[VkCloudShadow] Failed to create atlas image\n");
            return false;
        }

        VkImageViewCreateInfo vci{ VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO };
        vci.image = _atlasImage; vci.viewType = VK_IMAGE_VIEW_TYPE_2D; vci.format = kAtlasFormat;
        vci.subresourceRange = { VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1 };
        if (vkCreateImageView(ctx.device, &vci, nullptr, &_atlasView) != VK_SUCCESS) return false;

        // 初始就转成只读：地形几何 pass 每帧先跑，首帧会采样到这张还没写过的图。
        // 内容未定义但 config.x=0 会让采样函数短路，不会真的读进去。
        VkCommandBuffer cmd = beginSingleTimeCommands(ctx);
        transitionImage(cmd, _atlasImage, VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
            VK_PIPELINE_STAGE_2_TOP_OF_PIPE_BIT, VK_ACCESS_2_NONE,
            VK_PIPELINE_STAGE_2_FRAGMENT_SHADER_BIT, VK_ACCESS_2_SHADER_READ_BIT);
        endSingleTimeCommands(ctx, cmd);
        _atlasReadOnly = true;
        _rendered = false;
        return true;
    }

    bool createUniforms(VulkanContext& ctx) {
        VkBufferCreateInfo bci{ VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO };
        bci.size = sizeof(GpuCloudShadowData);
        bci.usage = VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT;
        VmaAllocationCreateInfo aci{};
        aci.usage = VMA_MEMORY_USAGE_CPU_TO_GPU; /* 中文说明：持久映射写入要求一致缓存，避免遗漏 flush。 */ aci.requiredFlags |= VK_MEMORY_PROPERTY_HOST_COHERENT_BIT;
        aci.flags = VMA_ALLOCATION_CREATE_MAPPED_BIT;

        GpuCloudShadowData zero{}; // config.x = 0 → 无阴影，作为所有 UBO 的安全初值
        for (int i = 0; i < FRAMES_IN_FLIGHT; i++) {
            VmaAllocationInfo info{};
            if (vmaCreateBuffer(ctx.allocator, &bci, &aci, &_computeUbo[i], &_computeUboAlloc[i], &info) != VK_SUCCESS) return false;
            _computeUboMapped[i] = info.pMappedData;
            memcpy(_computeUboMapped[i], &zero, sizeof(zero));

            if (vmaCreateBuffer(ctx.allocator, &bci, &aci, &_terrainUbo[i], &_terrainUboAlloc[i], &info) != VK_SUCCESS) return false;
            _terrainUboMapped[i] = info.pMappedData;
            memcpy(_terrainUboMapped[i], &zero, sizeof(zero));
        }
        return true;
    }

    bool createSampler(VulkanContext& ctx) {
        // 线性过滤（PCF taps 之间还要再插值），clamp-to-edge。跨切片越界由
        // shared_cloud_shadow.glsl 里的 UV 夹取负责，采样器只兜住图集外边界。
        VkSamplerCreateInfo sci{ VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO };
        sci.magFilter = VK_FILTER_LINEAR; sci.minFilter = VK_FILTER_LINEAR;
        sci.addressModeU = sci.addressModeV = sci.addressModeW = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
        sci.borderColor = VK_BORDER_COLOR_FLOAT_OPAQUE_WHITE;
        sci.mipmapMode = VK_SAMPLER_MIPMAP_MODE_NEAREST;
        sci.maxLod = 0.0f;  // 图集只有 1 级 mip
        return vkCreateSampler(ctx.device, &sci, nullptr, &_sampler) == VK_SUCCESS;
    }

    // ---------------------------------------------------------------------
    int _cascadeCount  = 4;
    int _perCascadeDim = 512;

    VkImage       _atlasImage = VK_NULL_HANDLE;
    VmaAllocation _atlasAlloc = VK_NULL_HANDLE;
    VkImageView   _atlasView  = VK_NULL_HANDLE;
    VkSampler     _sampler    = VK_NULL_HANDLE;
    VkPipeline    _pipeline   = VK_NULL_HANDLE;

    VkBuffer      _computeUbo[FRAMES_IN_FLIGHT]{};
    VmaAllocation _computeUboAlloc[FRAMES_IN_FLIGHT]{};
    void*         _computeUboMapped[FRAMES_IN_FLIGHT]{};

    VkBuffer      _terrainUbo[FRAMES_IN_FLIGHT]{};
    VmaAllocation _terrainUboAlloc[FRAMES_IN_FLIGHT]{};
    void*         _terrainUboMapped[FRAMES_IN_FLIGHT]{};

    GpuCloudShadowData _gpuData{};  // 最近一次 buildCascades 的结果（= 图集当前内容对应的矩阵）
    bool _atlasReadOnly = true;     // 图集当前 layout
    bool _rendered      = false;    // 图集里已有可用内容
};
