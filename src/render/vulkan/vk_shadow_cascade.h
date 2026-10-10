#pragma once
// ==========================================================================
// vk_shadow_cascade.h — 地形级联阴影贴图（云 Phase2：SDSM + God Ray 的生产端）
//
// 命名沿用 cloud_composite.glsl 里已经写好的消费端叫法"SDSM"，但这里做的是
// 经典 CSM 的"实用拆分方案"（cascadeSplitLambda 在均匀/对数拆分间插值），
// 不是真正逐帧读回深度直方图紧密拟合视锥的那种严格 SDSM（Sample Distribution
// Shadow Maps 原论文的做法）——GpuCascadeShadowConfig 里现成的 cascadeSplitLambda
// 字段就是照实用拆分方案设计的，说明这本来就是预期实现方式。见计划
// fuzzy-toasting-flurry 的取舍说明。
//
// 进一步简化：每级级联用"以相机为球心"的正交包围盒（半径=该级远裁截距离），
// 不做视锥 8 角点反投影再取 AABB 那种严格贴合——God Ray 只关心相机附近
// 400m 内的太阳遮挡，贴合精度不是瓶颈，省掉这一步换来实现简单很多。
//
// 阴影投射体只有地形（quadtree 面片）：God Ray 是近相机效果，地形是唯一真正
// 影响这个范围太阳可见性的几何；火箭/其它网格不投射阴影。
// ==========================================================================

#include "vk_context.h"
#include "vk_mesh.h"
#include "vk_utils.h"
#include "vk_frame.h"
#include "gpu_frame_data.h"
#include "../scene_snapshot.h"
#include "../../math/math3d.h"

#include <vector>
#include <algorithm>
#include <cstdio>
#include <cstring>
#include <cmath>

// 与 terrain_shadow.vert 的 push_constant 块逐字段对应。前 13 个字段和
// TerrainPushConstants（vk_pipeline.h）完全一致，只是尾部多一个 lightViewProj——
// 阴影 pass 不需要相机的 view/proj，直接用光源的正交 viewProj 代替。
struct TerrainShadowPushConstants {
    float model[16];       // offset   0
    float planetRadius;    // offset  64
    float maxElevation;    // offset  68
    int   nodeLevel;       // offset  72
    int   hasLocalHydro;   // offset  76
    float nodePos[3];      // offset  80
    float _pad0;           // offset  92
    float nodeSide[3];     // offset  96
    float _pad1;           // offset 108
    float nodeUp[3];       // offset 112
    float _pad2;           // offset 124
    float lightViewProj[16]; // offset 128 — 192 bytes total
};
static_assert(sizeof(TerrainShadowPushConstants) == 192, "must match terrain_shadow.vert PC block byte layout");

// 经典 practical split scheme：cascadeSplitLambda=0 时纯均匀拆分，=1 时纯对数
// 拆分（近处级联更细）。splits[0]=nearZ，splits[cascadeCount]=farZ。
inline std::vector<float> computeCascadeSplits(float nearZ, float farZ, int cascadeCount, float lambda) {
    std::vector<float> splits((size_t)cascadeCount + 1);
    splits[0] = nearZ;
    for (int i = 1; i <= cascadeCount; i++) {
        float p = (float)i / (float)cascadeCount;
        float logSplit     = nearZ * powf(std::max(farZ / nearZ, 1.0001f), p);
        float uniformSplit = nearZ + (farZ - nearZ) * p;
        splits[(size_t)i] = lambda * logSplit + (1.0f - lambda) * uniformSplit;
    }
    return splits;
}

// 单个级联的光源正交 viewProj：以相机（camPosAbsKm，绝对浮动原点世界坐标）为
// 球心，半径=splitFarKm，光源沿"指向太阳"方向的反方向后退 (splitFar+depthMargin)
// 再看向相机，正交投影半宽=splitFar，深度范围额外留 depthMargin 兜住相机附近
// 的地形起伏（山峰/山谷）。sunDirToPlanetKm 是"当前星球→太阳"方向（不是光线
// 传播方向），与 vk_renderer3d.h::renderAtmoAfterGeometry 给大气用的同一个量。
inline Mat4 buildCascadeViewProj(float splitFarKm, const Vec3& camPosAbsKm, const Vec3& sunDirToPlanetKm, float depthMarginKm) {
    Vec3 lightDir = sunDirToPlanetKm.normalized();
    double back = (double)splitFarKm + (double)depthMarginKm;
    Vec3 eye = camPosAbsKm + lightDir * back;
    // lookAt 的 worldUp 只是用来定側方向，光源方向接近世界 Y 轴时换一个不平行的轴。
    Vec3 worldUp = (std::abs(lightDir.y) < 0.99) ? Vec3(0.0, 1.0, 0.0) : Vec3(1.0, 0.0, 0.0);
    Mat4 lightView = Mat4::lookAt(eye, camPosAbsKm, worldUp);

    double halfExtent = (double)splitFarKm;
    double zNear = 0.01;
    double zFar  = 2.0 * back + 0.1;
    Mat4 lightProj = Mat4::ortho(-halfExtent, halfExtent, -halfExtent, halfExtent, zNear, zFar);
    return lightProj * lightView;
}

struct VkCascadeShadowSystem {
    static constexpr int kMaxCascades = 4;
    static constexpr VkFormat kDepthFmt = VK_FORMAT_D32_SFLOAT;

    // ── Tuner 可调（imgui_cloud_tuner.h）──────────────────────────────────
    // enabled 呼应 cloudGodRay：两者一起在面板里开，级联阴影本身不依赖 God Ray
    // （getShadow 之类的次要用途也能用），但目前唯一的消费者就是 God Ray。
    bool  enabled               = false;
    float cascadeSplitLambda    = 0.6f;
    float maxDrawDepthDistanceKm= 8.0f;   // 最远一级级联的远裁截距离
    float nearDistanceKm        = 0.05f;  // 第一级级联的近裁截距离
    float depthMarginKm         = 50.0f;  // 每级正交盒沿光源方向额外留出的深度余量
    float shadowBiasConst       = 0.0015f;
    float shadowBiasSlope       = 0.003f;
    float cascadeBorderAdopt    = 0.02f;
    float cascadeEdgeLerpThreshold = 0.85f;

    // cascadeCount/percascadeDimXY 只在 init() 时决定图集尺寸，之后不支持热改
    // （改了不生效，要重启）——运行时改变图集尺寸需要重建 atlas image，为了不
    // 把这套本来就是"实用简化版"的系统搞得更复杂，先不支持。
    int cascadeCount    = 4;
    int percascadeDimXY = 1024;

    VkImage       atlasImage = VK_NULL_HANDLE;
    VmaAllocation atlasAlloc = VK_NULL_HANDLE;
    VkImageView   atlasView  = VK_NULL_HANDLE; // 深度纹理视图，cloud_common.glsl::inSDSMShadowDepth 直接采样

    VkBuffer      cascadeSsbo[FRAMES_IN_FLIGHT]       = {};
    VmaAllocation cascadeSsboAlloc[FRAMES_IN_FLIGHT]  = {};
    void*         cascadeSsboMapped[FRAMES_IN_FLIGHT] = {};

    VkPipelineLayout shadowLayout   = VK_NULL_HANDLE;
    VkPipeline       shadowPipeline = VK_NULL_HANDLE;

    // true 表示 atlasView 里已经有本帧渲染好的内容，可以安全采样；刚 init()
    // 完、或这一帧被 enabled=false 跳过时是 false（此时 cloud 侧也不会真的去
    // 采样它，因为 outConfig.cascadeCount 会跟着是 0）。
    bool atlasReady = false;

    bool init(VulkanContext& ctx, VkDescriptorSetLayout terrainTexSetLayout) {
        cascadeCount = std::min(std::max(cascadeCount, 1), kMaxCascades);
        if (!createAtlas(ctx))                      return false;
        if (!createSsbo(ctx))                        return false;
        if (!createPipeline(ctx, terrainTexSetLayout)) return false;
        printf("[VkCascadeShadowSystem] Ready: %d cascades x %dpx\n", cascadeCount, percascadeDimXY);
        return true;
    }

    void shutdown(VulkanContext& ctx) {
        vkDeviceWaitIdle(ctx.device);
        if (shadowPipeline) { vkDestroyPipeline(ctx.device, shadowPipeline, nullptr); shadowPipeline = VK_NULL_HANDLE; }
        if (shadowLayout)   { vkDestroyPipelineLayout(ctx.device, shadowLayout, nullptr); shadowLayout = VK_NULL_HANDLE; }
        for (int i = 0; i < FRAMES_IN_FLIGHT; i++)
            if (cascadeSsbo[i]) { vmaDestroyBuffer(ctx.allocator, cascadeSsbo[i], cascadeSsboAlloc[i]); cascadeSsbo[i] = VK_NULL_HANDLE; }
        if (atlasView)  { vkDestroyImageView(ctx.device, atlasView, nullptr); atlasView = VK_NULL_HANDLE; }
        if (atlasImage) { vmaDestroyImage(ctx.allocator, atlasImage, atlasAlloc); atlasImage = VK_NULL_HANDLE; }
    }

    VkImageView shadowAtlasView() const { return atlasView; }
    VkBuffer    cascadeInfoBuffer(int frameSlot) const { return cascadeSsbo[frameSlot % FRAMES_IN_FLIGHT]; }

    // 每帧调用：算级联拆分+光源正交 viewProj，把地形面片画进阴影图集，填
    // cascadeSsbo（binding 29 的真实数据）；outConfig 供调用方抄进
    // fd.sky.cacsadeConfig（vk_cloud_system.h::fillCloudFrameData）。
    // sunDirToPlanetKm/camPosAbsKm：调用方已经算好的"星球→太阳"方向 + 绝对
    // 浮动原点相机世界坐标（都是 vk_renderer3d.h 已有的量，直接传进来）。
    void render(VulkanContext& ctx, VkCommandBuffer cmd, int frameSlot,
                const SceneSnapshot& snap,
                VkBuffer terrainVbo, VkBuffer terrainIbo, uint32_t terrainIcount,
                VkDescriptorSet terrainTexSet,
                const float sunDirToPlanetKm[3], const float camPosAbsKm[3],
                GpuCascadeShadowConfig& outConfig) {
        outConfig = GpuCascadeShadowConfig{};

        if (!enabled || shadowPipeline == VK_NULL_HANDLE || snap.terrainPatches.empty() || terrainTexSet == VK_NULL_HANDLE) {
            atlasReady = false;
            return;
        }

        int slot = frameSlot % FRAMES_IN_FLIGHT;
        Vec3 camPos((double)camPosAbsKm[0], (double)camPosAbsKm[1], (double)camPosAbsKm[2]);
        Vec3 sunDir((double)sunDirToPlanetKm[0], (double)sunDirToPlanetKm[1], (double)sunDirToPlanetKm[2]);

        std::vector<float> splits = computeCascadeSplits(nearDistanceKm, maxDrawDepthDistanceKm, cascadeCount, cascadeSplitLambda);

        std::vector<GpuCascadeInfo> infos((size_t)cascadeCount);

        transitionImage(cmd, atlasImage,
            atlasReady ? VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL : VK_IMAGE_LAYOUT_UNDEFINED,
            VK_IMAGE_LAYOUT_DEPTH_ATTACHMENT_OPTIMAL,
            VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT, VK_ACCESS_2_SHADER_READ_BIT,
            VK_PIPELINE_STAGE_2_EARLY_FRAGMENT_TESTS_BIT | VK_PIPELINE_STAGE_2_LATE_FRAGMENT_TESTS_BIT,
            VK_ACCESS_2_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT);

        for (int c = 0; c < cascadeCount; c++) {
            Mat4 vp = buildCascadeViewProj(splits[(size_t)c + 1], camPos, sunDir, depthMarginKm);
            vp.toFloatArray(infos[(size_t)c].viewProj);

            VkRenderingAttachmentInfo depthAI{ VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO };
            depthAI.imageView   = atlasView;
            depthAI.imageLayout = VK_IMAGE_LAYOUT_DEPTH_ATTACHMENT_OPTIMAL;
            depthAI.loadOp      = VK_ATTACHMENT_LOAD_OP_CLEAR; // 每级只清自己那条切片，不影响其它级联
            depthAI.storeOp     = VK_ATTACHMENT_STORE_OP_STORE;
            depthAI.clearValue.depthStencil = { 1.0f, 0 };

            VkRenderingInfo ri{ VK_STRUCTURE_TYPE_RENDERING_INFO };
            ri.renderArea = { { percascadeDimXY * c, 0 }, { (uint32_t)percascadeDimXY, (uint32_t)percascadeDimXY } };
            ri.layerCount = 1;
            ri.colorAttachmentCount = 0;
            ri.pDepthAttachment = &depthAI;

            vkCmdBeginRendering(cmd, &ri);

            VkViewport vpRect{ (float)(percascadeDimXY * c), 0.0f, (float)percascadeDimXY, (float)percascadeDimXY, 0.0f, 1.0f };
            vkCmdSetViewport(cmd, 0, 1, &vpRect);
            VkRect2D scissor{ { percascadeDimXY * c, 0 }, { (uint32_t)percascadeDimXY, (uint32_t)percascadeDimXY } };
            vkCmdSetScissor(cmd, 0, 1, &scissor);

            vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, shadowPipeline);
            vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, shadowLayout, 0, 1, &terrainTexSet, 0, nullptr);

            VkDeviceSize off = 0;
            vkCmdBindVertexBuffers(cmd, 0, 1, &terrainVbo, &off);
            vkCmdBindIndexBuffer(cmd, terrainIbo, 0, VK_INDEX_TYPE_UINT32);

            for (const auto& tp : snap.terrainPatches) {
                TerrainShadowPushConstants spc{};
                memcpy(spc.model, tp.model, 64);
                spc.planetRadius  = tp.planetRadius;
                spc.maxElevation  = tp.maxElev;
                spc.nodeLevel     = tp.nodeLevel;
                spc.hasLocalHydro = 0;
                spc.nodePos[0] = tp.nodeCenter[0]; spc.nodePos[1] = tp.nodeCenter[1]; spc.nodePos[2] = tp.nodeCenter[2];
                spc.nodeSide[0] = tp.nodeSideA[0]; spc.nodeSide[1] = tp.nodeSideA[1]; spc.nodeSide[2] = tp.nodeSideA[2];
                spc.nodeUp[0]   = tp.nodeSideB[0]; spc.nodeUp[1]   = tp.nodeSideB[1]; spc.nodeUp[2]   = tp.nodeSideB[2];
                memcpy(spc.lightViewProj, infos[(size_t)c].viewProj, 64);

                vkCmdPushConstants(cmd, shadowLayout, VK_SHADER_STAGE_VERTEX_BIT,
                    0, sizeof(TerrainShadowPushConstants), &spc);
                vkCmdDrawIndexed(cmd, terrainIcount, 1, 0, 0, 0);
            }

            vkCmdEndRendering(cmd);
        }

        transitionImage(cmd, atlasImage,
            VK_IMAGE_LAYOUT_DEPTH_ATTACHMENT_OPTIMAL, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
            VK_PIPELINE_STAGE_2_EARLY_FRAGMENT_TESTS_BIT | VK_PIPELINE_STAGE_2_LATE_FRAGMENT_TESTS_BIT,
            VK_ACCESS_2_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT,
            VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT, VK_ACCESS_2_SHADER_READ_BIT);
        atlasReady = true;

        memcpy(cascadeSsboMapped[slot], infos.data(), sizeof(GpuCascadeInfo) * infos.size());

        outConfig.cascadeCount             = cascadeCount;
        outConfig.percascadeDimXY          = percascadeDimXY;
        outConfig.cascadeSplitLambda       = cascadeSplitLambda;
        outConfig.maxDrawDepthDistance     = maxDrawDepthDistanceKm;
        outConfig.shadowBiasConst          = shadowBiasConst;
        outConfig.shadowBiasSlope          = shadowBiasSlope;
        outConfig.shadowFilterSize         = 1.0f;
        outConfig.maxFilterSize            = 3.0f;
        outConfig.cascadeBorderAdopt       = cascadeBorderAdopt;
        outConfig.cascadeEdgeLerpThreshold = cascadeEdgeLerpThreshold;
    }

private:
    bool createAtlas(VulkanContext& ctx) {
        uint32_t w = (uint32_t)(percascadeDimXY * cascadeCount);
        uint32_t h = (uint32_t)percascadeDimXY;
        VkImageCreateInfo ici{ VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO };
        ici.imageType = VK_IMAGE_TYPE_2D; ici.format = kDepthFmt; ici.extent = { w, h, 1 };
        ici.mipLevels = 1; ici.arrayLayers = 1; ici.samples = VK_SAMPLE_COUNT_1_BIT;
        ici.tiling = VK_IMAGE_TILING_OPTIMAL;
        ici.usage = VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT;
        ici.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
        VmaAllocationCreateInfo aci{}; aci.usage = VMA_MEMORY_USAGE_GPU_ONLY;
        if (vmaCreateImage(ctx.allocator, &ici, &aci, &atlasImage, &atlasAlloc, nullptr) != VK_SUCCESS) return false;

        VkImageViewCreateInfo vci{ VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO };
        vci.image = atlasImage; vci.viewType = VK_IMAGE_VIEW_TYPE_2D; vci.format = kDepthFmt;
        vci.subresourceRange = { VK_IMAGE_ASPECT_DEPTH_BIT, 0, 1, 0, 1 };
        return vkCreateImageView(ctx.device, &vci, nullptr, &atlasView) == VK_SUCCESS;
    }

    bool createSsbo(VulkanContext& ctx) {
        VkBufferCreateInfo bci{ VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO };
        bci.size = sizeof(GpuCascadeInfo) * (size_t)kMaxCascades;
        bci.usage = VK_BUFFER_USAGE_STORAGE_BUFFER_BIT;
        VmaAllocationCreateInfo aci{};
        aci.usage = VMA_MEMORY_USAGE_CPU_TO_GPU; /* 中文说明：持久映射写入要求一致缓存，避免遗漏 flush。 */ aci.requiredFlags |= VK_MEMORY_PROPERTY_HOST_COHERENT_BIT;
        aci.flags = VMA_ALLOCATION_CREATE_MAPPED_BIT;
        for (int i = 0; i < FRAMES_IN_FLIGHT; i++) {
            VmaAllocationInfo info{};
            if (vmaCreateBuffer(ctx.allocator, &bci, &aci, &cascadeSsbo[i], &cascadeSsboAlloc[i], &info) != VK_SUCCESS) return false;
            cascadeSsboMapped[i] = info.pMappedData;
            memset(cascadeSsboMapped[i], 0, sizeof(GpuCascadeInfo) * (size_t)kMaxCascades);
        }
        return true;
    }

    bool createPipeline(VulkanContext& ctx, VkDescriptorSetLayout terrainTexSetLayout) {
        auto vc = loadSPIRV("src/render/shaders/spirv/terrain_shadow.vert.spv");
        if (vc.empty()) { fprintf(stderr, "[VkCascadeShadowSystem] Missing terrain_shadow.vert.spv\n"); return false; }
        VkShaderModule vm = createShaderModule(ctx.device, vc);
        if (!vm) return false;

        VkPushConstantRange pcRange{ VK_SHADER_STAGE_VERTEX_BIT, 0, sizeof(TerrainShadowPushConstants) };
        VkDescriptorSetLayout sets[] = { terrainTexSetLayout };
        VkPipelineLayoutCreateInfo plci{ VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO };
        plci.setLayoutCount = 1; plci.pSetLayouts = sets;
        plci.pushConstantRangeCount = 1; plci.pPushConstantRanges = &pcRange;
        if (vkCreatePipelineLayout(ctx.device, &plci, nullptr, &shadowLayout) != VK_SUCCESS) {
            vkDestroyShaderModule(ctx.device, vm, nullptr);
            return false;
        }

        VkPipelineShaderStageCreateInfo stage{ VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO };
        stage.stage = VK_SHADER_STAGE_VERTEX_BIT; stage.module = vm; stage.pName = "main";
        // 只有顶点阶段——纯深度 pass 没有颜色 attachment，不需要片元着色器
        // （Vulkan 动态渲染下合法：rasterizerDiscardEnable=false 且颜色
        // attachment 数为 0 时，管线可以只有 vertex stage）。

        VkVertexInputBindingDescription vb{ 0, 48, VK_VERTEX_INPUT_RATE_VERTEX };
        VkVertexInputAttributeDescription va[3] = {
            { 0, 0, VK_FORMAT_R32G32B32_SFLOAT, 0 },
            { 1, 0, VK_FORMAT_R32G32B32_SFLOAT, 12 },
            { 2, 0, VK_FORMAT_R32G32_SFLOAT, 24 },
        };
        VkPipelineVertexInputStateCreateInfo vi{ VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO };
        vi.vertexBindingDescriptionCount = 1; vi.pVertexBindingDescriptions = &vb;
        vi.vertexAttributeDescriptionCount = 3; vi.pVertexAttributeDescriptions = va;

        VkPipelineInputAssemblyStateCreateInfo ia{ VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO };
        ia.topology = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;

        VkPipelineViewportStateCreateInfo vpState{ VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO };
        vpState.viewportCount = 1; vpState.scissorCount = 1;

        VkPipelineRasterizationStateCreateInfo rs{ VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO };
        rs.polygonMode = VK_POLYGON_MODE_FILL;
        rs.cullMode = VK_CULL_MODE_NONE; // 地形深度不裁背面，避免薄结构（悬崖/山脊背面）自遮挡穿帮
        rs.frontFace = VK_FRONT_FACE_COUNTER_CLOCKWISE;
        rs.lineWidth = 1.0f;
        // 固定管线深度偏移，替代 shader 里手调 bias——和 shadowBiasConst/Slope
        // 是两条独立的防阴影痤疮手段，光栅化阶段这个管硬件深度精度带来的自遮挡。
        rs.depthBiasEnable = VK_TRUE;
        rs.depthBiasConstantFactor = 1.25f;
        rs.depthBiasSlopeFactor = 1.75f;

        VkPipelineMultisampleStateCreateInfo ms{ VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO };
        ms.rasterizationSamples = VK_SAMPLE_COUNT_1_BIT;

        VkPipelineDepthStencilStateCreateInfo ds{ VK_STRUCTURE_TYPE_PIPELINE_DEPTH_STENCIL_STATE_CREATE_INFO };
        ds.depthTestEnable = VK_TRUE; ds.depthWriteEnable = VK_TRUE; ds.depthCompareOp = VK_COMPARE_OP_LESS;

        VkPipelineColorBlendStateCreateInfo bs{ VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO };
        bs.attachmentCount = 0; // 没有颜色 attachment

        VkDynamicState dyn[] = { VK_DYNAMIC_STATE_VIEWPORT, VK_DYNAMIC_STATE_SCISSOR };
        VkPipelineDynamicStateCreateInfo dsi{ VK_STRUCTURE_TYPE_PIPELINE_DYNAMIC_STATE_CREATE_INFO };
        dsi.dynamicStateCount = 2; dsi.pDynamicStates = dyn;

        VkPipelineRenderingCreateInfo rci{ VK_STRUCTURE_TYPE_PIPELINE_RENDERING_CREATE_INFO };
        rci.colorAttachmentCount = 0;
        rci.depthAttachmentFormat = kDepthFmt;

        VkGraphicsPipelineCreateInfo pci{ VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO };
        pci.pNext = &rci;
        pci.stageCount = 1; pci.pStages = &stage;
        pci.pVertexInputState = &vi;
        pci.pInputAssemblyState = &ia;
        pci.pViewportState = &vpState;
        pci.pRasterizationState = &rs;
        pci.pMultisampleState = &ms;
        pci.pDepthStencilState = &ds;
        pci.pColorBlendState = &bs;
        pci.pDynamicState = &dsi;
        pci.layout = shadowLayout;

        VkResult res = vkCreateGraphicsPipelines(ctx.device, VK_NULL_HANDLE, 1, &pci, nullptr, &shadowPipeline);
        vkDestroyShaderModule(ctx.device, vm, nullptr);
        if (res != VK_SUCCESS) { fprintf(stderr, "[VkCascadeShadowSystem] pipeline failed (%d)\n", (int)res); return false; }
        return true;
    }
};
