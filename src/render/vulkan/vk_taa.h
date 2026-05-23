#pragma once
// ==========================================================================
// vk_taa.h — TAA ping-pong render targets + resolve pipeline (Phase 4)
//
// 两帧渲染流程：
//   Pass 1 (Geometry): 场景渲染 → taaColorImages[currentIdx] (RGBA16F)
//                      深度写入 → depthImage (D32, StoreOp=STORE，保留供重投影)
//   Pass 2 (Resolve):  读取 current + history + depth → 混合输出到 Swapchain Image
//
// 使用 Vulkan 1.3 Dynamic Rendering，无 VkRenderPass 对象。
// ==========================================================================

#include "vk_context.h"
#include "vk_utils.h"
#include "vk_mesh.h"      // beginSingleTimeCommands / endSingleTimeCommands
#include "vk_pipeline.h"  // loadSPIRV, createShaderModule

#include <cstdio>

// Push constants for TAA resolve pass
struct TAAParams {
    float blendFactor = 0.1f;  // 0.1 = smooth, 1.0 = disable TAA (debug)
    float _pad[3]     = {};
};
static_assert(sizeof(TAAParams) <= 128, "TAAParams exceeds push constant limit");

// UBO: reprojection matrices (std140, 128 bytes)
// Must match layout(set=0, binding=3) in taa.frag
struct TAAMatrices {
    float invViewProj[16];   // current frame inverse(proj * view)
    float prevViewProj[16];  // previous frame proj * view
};

// -----------------------------------------------------------------------
// VkTAA — ping-pong TAA render targets + resolve pipeline
// -----------------------------------------------------------------------
struct VkTAA {
    static constexpr VkFormat kColorFmt = VK_FORMAT_R16G16B16A16_SFLOAT;
    static constexpr VkFormat kDepthFmt = VK_FORMAT_D32_SFLOAT;

    // Ping-pong RGBA16F color targets (index 0 and 1)
    VkImage       colorImages[2] = {};
    VmaAllocation colorAllocs[2] = {};
    VkImageView   colorViews[2]  = {};
    VkSampler     colorSampler   = VK_NULL_HANDLE;  // LINEAR, CLAMP_TO_EDGE

    // D32 depth (StoreOp=STORE, readable as sampler2D for reprojection)
    VkImage       depthImage  = VK_NULL_HANDLE;
    VmaAllocation depthAlloc  = VK_NULL_HANDLE;
    VkImageView   depthView   = VK_NULL_HANDLE;
    VkSampler     depthSampler = VK_NULL_HANDLE;   // NEAREST, CLAMP_TO_EDGE

    // TAA resolve pipeline (full-screen triangle → swapchain image)
    VkDescriptorSetLayout resolveSetLayout = VK_NULL_HANDLE;
    VkPipelineLayout      resolveLayout    = VK_NULL_HANDLE;
    VkPipeline            resolvePipeline  = VK_NULL_HANDLE;
    VkDescriptorPool      resolvePool      = VK_NULL_HANDLE;

    // resolveSets[i]: binding0=colorViews[i] (current), binding1=colorViews[1-i] (history)
    VkDescriptorSet resolveSets[2] = {};

    // Per-frame matrix UBO (binding 3: invViewProj + prevViewProj)
    VkBuffer      matrixUBO[2]   = {};
    VmaAllocation matrixAlloc[2] = {};
    void*         matrixMapped[2]= {};

    // 存储上一帧和当前帧的 view/proj，用于重投影
    float _prevViewProj[16]    = {};
    float _currViewProj[16]    = {};
    float _currInvViewProj[16] = {};
    bool  _matricesReady       = false;

    int  currentIdx   = 0;     // ping-pong, flipped by swap() each frame
    bool historyReady = false; // false = history 图像需在帧内 cmd buffer 里初始化

    // -----------------------------------------------------------------------
    // init — 创建全部资源（图像、采样器、管线、描述符集）
    // swapFormat: swapchain 颜色格式（用于 resolve pipeline 的 pNext rendering info）
    // vertSpv/fragSpv: taa.vert.spv / taa.frag.spv 路径
    // -----------------------------------------------------------------------
    bool init(VulkanContext& ctx,
              VkExtent2D    extent,
              VkFormat      swapFormat,
              const char*   vertSpv,
              const char*   fragSpv) {
        if (!createImages(ctx, extent))                                    return false;
        if (!createSamplers(ctx))                                          return false;
        // 不再用独立 submit 做初始化——改为在第一帧 cmd buffer 内做，
        // 避免跨 submit 的验证层 layout 追踪失效。
        historyReady = false;
        if (!createResolvePipeline(ctx, swapFormat, vertSpv, fragSpv))    return false;
        if (!createDescriptors(ctx))                                       return false;
        printf("[VkTAA] Initialized %ux%u RGBA16F ping-pong\n",
               extent.width, extent.height);
        return true;
    }

    // -----------------------------------------------------------------------
    // recreate — swapchain resize 后重建（管线因 Dynamic Viewport 无需重建）
    // -----------------------------------------------------------------------
    void recreate(VulkanContext& ctx,
                  VkExtent2D    extent,
                  VkFormat      swapFormat,
                  const char*   vertSpv,
                  const char*   fragSpv) {
        vkDeviceWaitIdle(ctx.device);
        // 描述符集随 pool 销毁一同释放
        if (resolvePool         != VK_NULL_HANDLE) { vkDestroyDescriptorPool    (ctx.device, resolvePool,         nullptr); resolvePool         = VK_NULL_HANDLE; }
        if (resolvePipeline     != VK_NULL_HANDLE) { vkDestroyPipeline          (ctx.device, resolvePipeline,     nullptr); resolvePipeline     = VK_NULL_HANDLE; }
        if (resolveLayout       != VK_NULL_HANDLE) { vkDestroyPipelineLayout    (ctx.device, resolveLayout,       nullptr); resolveLayout       = VK_NULL_HANDLE; }
        if (resolveSetLayout    != VK_NULL_HANDLE) { vkDestroyDescriptorSetLayout(ctx.device, resolveSetLayout,   nullptr); resolveSetLayout    = VK_NULL_HANDLE; }
        destroyImages(ctx);
        currentIdx = 0;

        createImages(ctx, extent);
        historyReady = false;  // 新图像是 UNDEFINED，在帧内 cmd buffer 里初始化
        createResolvePipeline(ctx, swapFormat, vertSpv, fragSpv);
        createDescriptors(ctx);
    }

    // -----------------------------------------------------------------------
    // Pass 1 — Geometry pass begin/end
    //
    // beginGeometryPass: 过渡 colorImages[currentIdx] → COLOR_ATTACHMENT_OPTIMAL
    //                    过渡 depthImage → DEPTH_ATTACHMENT_OPTIMAL
    //                    调用 vkCmdBeginRendering
    // endGeometryPass:   vkCmdEndRendering
    //                    过渡 colorImages[currentIdx] → SHADER_READ_ONLY_OPTIMAL
    //                    过渡 depthImage → SHADER_READ_ONLY_OPTIMAL
    // -----------------------------------------------------------------------
    void beginGeometryPass(VkCommandBuffer cmd, VkExtent2D extent) {
        // history 图像的初始化：首帧或 recreate 后在帧内 cmd buffer 里做，
        // 避免独立 submit 的跨提交 layout 追踪问题（验证层跟踪失效）。
        if (!historyReady) {
            int histIdx = 1 - currentIdx;
            // 清除 history 到与几何 pass 相同的背景色，避免第一帧采样 UNDEFINED 垃圾内存
            transitionImage(cmd, colorImages[histIdx],
                VK_IMAGE_LAYOUT_UNDEFINED,
                VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
                VK_PIPELINE_STAGE_2_TOP_OF_PIPE_BIT, VK_ACCESS_2_NONE,
                VK_PIPELINE_STAGE_2_TRANSFER_BIT,    VK_ACCESS_2_TRANSFER_WRITE_BIT);
            VkClearColorValue clearVal = { {0.05f, 0.05f, 0.1f, 1.0f} };
            VkImageSubresourceRange range = { VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1 };
            vkCmdClearColorImage(cmd, colorImages[histIdx],
                VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, &clearVal, 1, &range);
            transitionImage(cmd, colorImages[histIdx],
                VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
                VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
                VK_PIPELINE_STAGE_2_TRANSFER_BIT,    VK_ACCESS_2_TRANSFER_WRITE_BIT,
                VK_PIPELINE_STAGE_2_FRAGMENT_SHADER_BIT, VK_ACCESS_2_SHADER_READ_BIT);
            historyReady = true;
        }

        transitionImage(cmd, colorImages[currentIdx],
            VK_IMAGE_LAYOUT_UNDEFINED,
            VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL,
            VK_PIPELINE_STAGE_2_TOP_OF_PIPE_BIT,             VK_ACCESS_2_NONE,
            VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT, VK_ACCESS_2_COLOR_ATTACHMENT_WRITE_BIT);

        transitionToDepthAttachment(cmd, depthImage);

        VkRenderingAttachmentInfo colorAI{ VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO };
        colorAI.imageView   = colorViews[currentIdx];
        colorAI.imageLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
        colorAI.loadOp      = VK_ATTACHMENT_LOAD_OP_CLEAR;
        colorAI.storeOp     = VK_ATTACHMENT_STORE_OP_STORE;
        colorAI.clearValue  = {{ 0.05f, 0.05f, 0.1f, 1.0f }};

        VkRenderingAttachmentInfo depthAI{ VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO };
        depthAI.imageView   = depthView;
        depthAI.imageLayout = VK_IMAGE_LAYOUT_DEPTH_ATTACHMENT_OPTIMAL;
        depthAI.loadOp      = VK_ATTACHMENT_LOAD_OP_CLEAR;
        depthAI.storeOp     = VK_ATTACHMENT_STORE_OP_STORE;  // 保留深度供重投影采样
        depthAI.clearValue.depthStencil = { 1.0f, 0 };

        VkRenderingInfo ri{ VK_STRUCTURE_TYPE_RENDERING_INFO };
        ri.renderArea           = { {0, 0}, extent };
        ri.layerCount           = 1;
        ri.colorAttachmentCount = 1;
        ri.pColorAttachments    = &colorAI;
        ri.pDepthAttachment     = &depthAI;

        vkCmdBeginRendering(cmd, &ri);
    }

    void endGeometryPass(VkCommandBuffer cmd) const {
        vkCmdEndRendering(cmd);

        // 颜色目标: COLOR_ATTACHMENT → SHADER_READ_ONLY（供 TAA resolve 采样）
        transitionImage(cmd, colorImages[currentIdx],
            VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL,
            VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
            VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT, VK_ACCESS_2_COLOR_ATTACHMENT_WRITE_BIT,
            VK_PIPELINE_STAGE_2_FRAGMENT_SHADER_BIT,         VK_ACCESS_2_SHADER_READ_BIT);

        // 深度: DEPTH_ATTACHMENT → SHADER_READ_ONLY（供重投影采样）
        transitionImage(cmd, depthImage,
            VK_IMAGE_LAYOUT_DEPTH_ATTACHMENT_OPTIMAL,
            VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
            VK_PIPELINE_STAGE_2_LATE_FRAGMENT_TESTS_BIT, VK_ACCESS_2_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT,
            VK_PIPELINE_STAGE_2_FRAGMENT_SHADER_BIT,     VK_ACCESS_2_SHADER_READ_BIT);
    }

    // -----------------------------------------------------------------------
    // 每帧调一次：更新重投影矩阵（view/proj 均为列主序 float[16]）
    // 内部保存上一帧 VP 并计算当前帧 invViewProj
    // -----------------------------------------------------------------------
    void updateMatrices(const float view[16], const float proj[16]) {
        // 保存上一帧的 VP
        memcpy(_prevViewProj, _currViewProj, 64);

        // 计算当前 VP = proj * view（列主序）
        float* vp = _currViewProj;
        for (int col = 0; col < 4; col++) {
            for (int row = 0; row < 4; row++) {
                vp[col * 4 + row] =
                    proj[row + 0]  * view[col * 4 + 0] +
                    proj[row + 4]  * view[col * 4 + 1] +
                    proj[row + 8]  * view[col * 4 + 2] +
                    proj[row + 12] * view[col * 4 + 3];
            }
        }

        // 计算逆 VP（简化：对 orthographic-like 投影用近似）
        // 着色器真正需要的是 inverse(VP)，这里做数值逆
        // 对于大多数相机矩阵，invViewProj ≈ viewProj 的逆
        // 完整实现见下方 computeInverse 辅助
        computeInverseVP(vp, _currInvViewProj);

        if (!_matricesReady) {
            memcpy(_prevViewProj, _currViewProj, 64);
            _matricesReady = true;
        }
    }

    // 上传矩阵到 UBO（在 drawResolve 前调用，由调用方指定 frameSlot）
    void uploadMatrices(int frameSlot) {
        TAAMatrices m;
        memcpy(m.invViewProj,  _currInvViewProj, 64);
        memcpy(m.prevViewProj, _prevViewProj,     64);
        if (matrixMapped[frameSlot])
            memcpy(matrixMapped[frameSlot], &m, sizeof(TAAMatrices));
    }

    // -----------------------------------------------------------------------
    // Pass 2 — TAA resolve begin/draw/end
    //
    // 输出到 swapchain image（由 vkAcquireNextImageKHR 获取）。
    // -----------------------------------------------------------------------
    void beginResolvePass(VkCommandBuffer cmd,
                          VkImage         swapImage,
                          VkImageView     swapView,
                          VkExtent2D      extent) const {
        // Swapchain: UNDEFINED → COLOR_ATTACHMENT_OPTIMAL
        transitionToColorAttachment(cmd, swapImage);

        VkRenderingAttachmentInfo colorAI{ VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO };
        colorAI.imageView   = swapView;
        colorAI.imageLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
        colorAI.loadOp      = VK_ATTACHMENT_LOAD_OP_DONT_CARE;  // resolve 覆盖全屏
        colorAI.storeOp     = VK_ATTACHMENT_STORE_OP_STORE;

        VkRenderingInfo ri{ VK_STRUCTURE_TYPE_RENDERING_INFO };
        ri.renderArea           = { {0, 0}, extent };
        ri.layerCount           = 1;
        ri.colorAttachmentCount = 1;
        ri.pColorAttachments    = &colorAI;

        vkCmdBeginRendering(cmd, &ri);
        vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, resolvePipeline);
        vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS,
            resolveLayout, 0, 1, &resolveSets[currentIdx], 0, nullptr);
    }

    // blendFactor: 0.1 = 90% history + 10% current (smooth TAA)，1.0 = 仅当前帧
    void drawResolve(VkCommandBuffer cmd, float blendFactor = 0.1f) const {
        TAAParams params{};
        params.blendFactor = blendFactor;
        vkCmdPushConstants(cmd, resolveLayout,
            VK_SHADER_STAGE_FRAGMENT_BIT, 0, sizeof(TAAParams), &params);
        vkCmdDraw(cmd, 3, 1, 0, 0);  // 全屏三角，无顶点缓冲区
    }

    void endResolvePass(VkCommandBuffer cmd, VkImage swapImage) const {
        vkCmdEndRendering(cmd);
        // COLOR_ATTACHMENT_OPTIMAL → PRESENT_SRC_KHR
        transitionToPresent(cmd, swapImage);
    }

    // 翻转 ping-pong 索引，每帧帧尾调用
    void swap() { currentIdx ^= 1; }

    // -----------------------------------------------------------------------
    void shutdown(VulkanContext& ctx) {
        vkDeviceWaitIdle(ctx.device);
        if (resolvePool      != VK_NULL_HANDLE) { vkDestroyDescriptorPool    (ctx.device, resolvePool,      nullptr); resolvePool      = VK_NULL_HANDLE; }
        if (resolvePipeline  != VK_NULL_HANDLE) { vkDestroyPipeline          (ctx.device, resolvePipeline,  nullptr); resolvePipeline  = VK_NULL_HANDLE; }
        if (resolveLayout    != VK_NULL_HANDLE) { vkDestroyPipelineLayout    (ctx.device, resolveLayout,    nullptr); resolveLayout    = VK_NULL_HANDLE; }
        if (resolveSetLayout != VK_NULL_HANDLE) { vkDestroyDescriptorSetLayout(ctx.device, resolveSetLayout, nullptr); resolveSetLayout = VK_NULL_HANDLE; }
        if (colorSampler     != VK_NULL_HANDLE) { vkDestroySampler           (ctx.device, colorSampler,     nullptr); colorSampler     = VK_NULL_HANDLE; }
        if (depthSampler     != VK_NULL_HANDLE) { vkDestroySampler           (ctx.device, depthSampler,     nullptr); depthSampler     = VK_NULL_HANDLE; }
        for (int i = 0; i < 2; ++i) {
            if (matrixUBO[i] != VK_NULL_HANDLE) { vmaDestroyBuffer(ctx.allocator, matrixUBO[i], matrixAlloc[i]); matrixUBO[i] = VK_NULL_HANDLE; }
        }
        destroyImages(ctx);
    }

private:
    bool createImages(VulkanContext& ctx, VkExtent2D extent) {
        for (int i = 0; i < 2; ++i) {
            VkImageCreateInfo ici{ VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO };
            ici.imageType     = VK_IMAGE_TYPE_2D;
            ici.format        = kColorFmt;
            ici.extent        = { extent.width, extent.height, 1 };
            ici.mipLevels     = 1;
            ici.arrayLayers   = 1;
            ici.samples       = VK_SAMPLE_COUNT_1_BIT;
            ici.tiling        = VK_IMAGE_TILING_OPTIMAL;
            ici.usage         = VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT
                            | VK_IMAGE_USAGE_TRANSFER_DST_BIT;
            ici.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
            VmaAllocationCreateInfo aci{};
            aci.usage = VMA_MEMORY_USAGE_GPU_ONLY;
            if (vmaCreateImage(ctx.allocator, &ici, &aci, &colorImages[i], &colorAllocs[i], nullptr) != VK_SUCCESS) {
                fprintf(stderr, "[VkTAA] Failed to create color image %d\n", i);
                return false;
            }
            VkImageViewCreateInfo vci{ VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO };
            vci.image                       = colorImages[i];
            vci.viewType                    = VK_IMAGE_VIEW_TYPE_2D;
            vci.format                      = kColorFmt;
            vci.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
            vci.subresourceRange.levelCount = 1;
            vci.subresourceRange.layerCount = 1;
            if (vkCreateImageView(ctx.device, &vci, nullptr, &colorViews[i]) != VK_SUCCESS) {
                fprintf(stderr, "[VkTAA] Failed to create color view %d\n", i);
                return false;
            }
        }
        {
            VkImageCreateInfo ici{ VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO };
            ici.imageType     = VK_IMAGE_TYPE_2D;
            ici.format        = kDepthFmt;
            ici.extent        = { extent.width, extent.height, 1 };
            ici.mipLevels     = 1;
            ici.arrayLayers   = 1;
            ici.samples       = VK_SAMPLE_COUNT_1_BIT;
            ici.tiling        = VK_IMAGE_TILING_OPTIMAL;
            ici.usage         = VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT;
            ici.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
            VmaAllocationCreateInfo aci{};
            aci.usage = VMA_MEMORY_USAGE_GPU_ONLY;
            if (vmaCreateImage(ctx.allocator, &ici, &aci, &depthImage, &depthAlloc, nullptr) != VK_SUCCESS) {
                fprintf(stderr, "[VkTAA] Failed to create depth image\n");
                return false;
            }
            VkImageViewCreateInfo vci{ VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO };
            vci.image                       = depthImage;
            vci.viewType                    = VK_IMAGE_VIEW_TYPE_2D;
            vci.format                      = kDepthFmt;
            vci.subresourceRange.aspectMask = VK_IMAGE_ASPECT_DEPTH_BIT;
            vci.subresourceRange.levelCount = 1;
            vci.subresourceRange.layerCount = 1;
            if (vkCreateImageView(ctx.device, &vci, nullptr, &depthView) != VK_SUCCESS) {
                fprintf(stderr, "[VkTAA] Failed to create depth view\n");
                return false;
            }
        }
        return true;
    }

    bool createSamplers(VulkanContext& ctx) {
        {
            VkSamplerCreateInfo sci{ VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO };
            sci.magFilter    = VK_FILTER_LINEAR;
            sci.minFilter    = VK_FILTER_LINEAR;
            sci.mipmapMode   = VK_SAMPLER_MIPMAP_MODE_NEAREST;
            sci.addressModeU = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
            sci.addressModeV = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
            sci.maxLod       = VK_LOD_CLAMP_NONE;
            if (vkCreateSampler(ctx.device, &sci, nullptr, &colorSampler) != VK_SUCCESS) {
                fprintf(stderr, "[VkTAA] Failed to create color sampler\n");
                return false;
            }
        }
        {
            VkSamplerCreateInfo sci{ VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO };
            sci.magFilter    = VK_FILTER_NEAREST;
            sci.minFilter    = VK_FILTER_NEAREST;
            sci.mipmapMode   = VK_SAMPLER_MIPMAP_MODE_NEAREST;
            sci.addressModeU = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
            sci.addressModeV = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
            sci.maxLod       = VK_LOD_CLAMP_NONE;
            if (vkCreateSampler(ctx.device, &sci, nullptr, &depthSampler) != VK_SUCCESS) {
                fprintf(stderr, "[VkTAA] Failed to create depth sampler\n");
                return false;
            }
        }
        return true;
    }

    bool createResolvePipeline(VulkanContext& ctx, VkFormat swapFormat,
                               const char* vertSpv, const char* fragSpv) {
        auto vertCode = loadSPIRV(vertSpv);
        auto fragCode = loadSPIRV(fragSpv);
        if (vertCode.empty() || fragCode.empty()) return false;

        VkShaderModule vertMod = createShaderModule(ctx.device, vertCode);
        VkShaderModule fragMod = createShaderModule(ctx.device, fragCode);
        if (vertMod == VK_NULL_HANDLE || fragMod == VK_NULL_HANDLE) {
            if (vertMod) vkDestroyShaderModule(ctx.device, vertMod, nullptr);
            if (fragMod) vkDestroyShaderModule(ctx.device, fragMod, nullptr);
            return false;
        }

        // Descriptor set layout: 0=current, 1=history, 2=depth, 3=matrices UBO
        VkDescriptorSetLayoutBinding bindings[4]{};
        for (uint32_t i = 0; i < 3; ++i) {
            bindings[i].binding         = i;
            bindings[i].descriptorType  = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
            bindings[i].descriptorCount = 1;
            bindings[i].stageFlags      = VK_SHADER_STAGE_FRAGMENT_BIT;
        }
        // binding 3: TAAMatrices UBO (std140)
        bindings[3].binding         = 3;
        bindings[3].descriptorType  = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
        bindings[3].descriptorCount = 1;
        bindings[3].stageFlags      = VK_SHADER_STAGE_FRAGMENT_BIT;
        VkDescriptorSetLayoutCreateInfo setCI{ VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO };
        setCI.bindingCount = 4;
        setCI.pBindings    = bindings;
        if (vkCreateDescriptorSetLayout(ctx.device, &setCI, nullptr, &resolveSetLayout) != VK_SUCCESS) {
            fprintf(stderr, "[VkTAA] Failed to create descriptor set layout\n");
            return false;
        }

        VkPushConstantRange pcRange{};
        pcRange.stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;
        pcRange.size       = sizeof(TAAParams);

        VkPipelineLayoutCreateInfo layoutCI{ VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO };
        layoutCI.setLayoutCount         = 1;
        layoutCI.pSetLayouts            = &resolveSetLayout;
        layoutCI.pushConstantRangeCount = 1;
        layoutCI.pPushConstantRanges    = &pcRange;
        if (vkCreatePipelineLayout(ctx.device, &layoutCI, nullptr, &resolveLayout) != VK_SUCCESS) {
            fprintf(stderr, "[VkTAA] Failed to create pipeline layout\n");
            return false;
        }

        VkPipelineShaderStageCreateInfo stages[2]{};
        stages[0].sType  = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
        stages[0].stage  = VK_SHADER_STAGE_VERTEX_BIT;
        stages[0].module = vertMod;
        stages[0].pName  = "main";
        stages[1].sType  = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
        stages[1].stage  = VK_SHADER_STAGE_FRAGMENT_BIT;
        stages[1].module = fragMod;
        stages[1].pName  = "main";

        VkPipelineVertexInputStateCreateInfo   viState{ VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO };
        VkPipelineInputAssemblyStateCreateInfo iaState{ VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO };
        iaState.topology = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;

        VkPipelineViewportStateCreateInfo vpState{ VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO };
        vpState.viewportCount = 1;
        vpState.scissorCount  = 1;

        VkPipelineRasterizationStateCreateInfo rasState{ VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO };
        rasState.polygonMode = VK_POLYGON_MODE_FILL;
        rasState.cullMode    = VK_CULL_MODE_NONE;
        rasState.frontFace   = VK_FRONT_FACE_COUNTER_CLOCKWISE;
        rasState.lineWidth   = 1.0f;

        VkPipelineMultisampleStateCreateInfo msState{ VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO };
        msState.rasterizationSamples = VK_SAMPLE_COUNT_1_BIT;

        VkPipelineDepthStencilStateCreateInfo dsState{ VK_STRUCTURE_TYPE_PIPELINE_DEPTH_STENCIL_STATE_CREATE_INFO };
        // 全屏 resolve 不写深度

        VkPipelineColorBlendAttachmentState blendAttach{};
        blendAttach.colorWriteMask = VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT
                                   | VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT;
        blendAttach.blendEnable = VK_FALSE;

        VkPipelineColorBlendStateCreateInfo blendState{ VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO };
        blendState.attachmentCount = 1;
        blendState.pAttachments    = &blendAttach;

        VkDynamicState dynStates[] = { VK_DYNAMIC_STATE_VIEWPORT, VK_DYNAMIC_STATE_SCISSOR };
        VkPipelineDynamicStateCreateInfo dynState{ VK_STRUCTURE_TYPE_PIPELINE_DYNAMIC_STATE_CREATE_INFO };
        dynState.dynamicStateCount = 2;
        dynState.pDynamicStates    = dynStates;

        VkPipelineRenderingCreateInfo renderingCI{ VK_STRUCTURE_TYPE_PIPELINE_RENDERING_CREATE_INFO };
        renderingCI.colorAttachmentCount    = 1;
        renderingCI.pColorAttachmentFormats = &swapFormat;
        // 无 depthAttachmentFormat：resolve pass 不使用深度附件

        VkGraphicsPipelineCreateInfo pipeCI{ VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO };
        pipeCI.pNext               = &renderingCI;
        pipeCI.stageCount          = 2;
        pipeCI.pStages             = stages;
        pipeCI.pVertexInputState   = &viState;
        pipeCI.pInputAssemblyState = &iaState;
        pipeCI.pViewportState      = &vpState;
        pipeCI.pRasterizationState = &rasState;
        pipeCI.pMultisampleState   = &msState;
        pipeCI.pDepthStencilState  = &dsState;
        pipeCI.pColorBlendState    = &blendState;
        pipeCI.pDynamicState       = &dynState;
        pipeCI.layout              = resolveLayout;
        pipeCI.renderPass          = VK_NULL_HANDLE;

        VkResult result = vkCreateGraphicsPipelines(
            ctx.device, VK_NULL_HANDLE, 1, &pipeCI, nullptr, &resolvePipeline);
        vkDestroyShaderModule(ctx.device, vertMod, nullptr);
        vkDestroyShaderModule(ctx.device, fragMod, nullptr);

        if (result != VK_SUCCESS) {
            fprintf(stderr, "[VkTAA] Failed to create resolve pipeline\n");
            return false;
        }
        printf("[VkTAA] Resolve pipeline created\n");
        return true;
    }

    bool createDescriptors(VulkanContext& ctx) {
        VkDescriptorPoolSize poolSizes[2]{};
        poolSizes[0].type            = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
        poolSizes[0].descriptorCount = 6;  // 2 sets × 3 samplers
        poolSizes[1].type            = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
        poolSizes[1].descriptorCount = 2;  // 2 sets × 1 UBO
        VkDescriptorPoolCreateInfo poolCI{ VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO };
        poolCI.maxSets       = 2;
        poolCI.poolSizeCount = 2;
        poolCI.pPoolSizes    = poolSizes;
        if (vkCreateDescriptorPool(ctx.device, &poolCI, nullptr, &resolvePool) != VK_SUCCESS) {
            fprintf(stderr, "[VkTAA] Failed to create descriptor pool\n");
            return false;
        }

        VkDescriptorSetLayout layouts[2] = { resolveSetLayout, resolveSetLayout };
        VkDescriptorSetAllocateInfo allocInfo{ VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO };
        allocInfo.descriptorPool     = resolvePool;
        allocInfo.descriptorSetCount = 2;
        allocInfo.pSetLayouts        = layouts;
        if (vkAllocateDescriptorSets(ctx.device, &allocInfo, resolveSets) != VK_SUCCESS) {
            fprintf(stderr, "[VkTAA] Failed to allocate descriptor sets\n");
            return false;
        }

        // 创建矩阵 UBO (每帧一份, CPU_TO_GPU persistent mapped)
        for (int i = 0; i < 2; ++i) {
            VkBufferCreateInfo bci{ VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO };
            bci.size  = sizeof(TAAMatrices);
            bci.usage = VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT;
            VmaAllocationCreateInfo aci{};
            aci.usage = VMA_MEMORY_USAGE_CPU_TO_GPU;
            aci.flags = VMA_ALLOCATION_CREATE_MAPPED_BIT;
            VmaAllocationInfo info{};
            if (vmaCreateBuffer(ctx.allocator, &bci, &aci,
                    &matrixUBO[i], &matrixAlloc[i], &info) != VK_SUCCESS) {
                fprintf(stderr, "[VkTAA] Failed to create matrix UBO %d\n", i);
                return false;
            }
            matrixMapped[i] = info.pMappedData;
            memset(matrixMapped[i], 0, sizeof(TAAMatrices));
        }

        // resolveSets[i]: current=colorViews[i], history=colorViews[1-i], depth=depthView, UBO=matrixUBO[i]
        for (int i = 0; i < 2; ++i) {
            VkDescriptorImageInfo imgs[3]{};
            imgs[0] = { colorSampler, colorViews[i],   VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL };
            imgs[1] = { colorSampler, colorViews[1-i], VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL };
            imgs[2] = { depthSampler, depthView,       VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL };

            VkWriteDescriptorSet writes[4]{};
            for (uint32_t b = 0; b < 3; ++b) {
                writes[b].sType           = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
                writes[b].dstSet          = resolveSets[i];
                writes[b].dstBinding      = b;
                writes[b].descriptorCount = 1;
                writes[b].descriptorType  = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
                writes[b].pImageInfo      = &imgs[b];
            }
            VkDescriptorBufferInfo bufInfo{};
            bufInfo.buffer = matrixUBO[i];
            bufInfo.offset = 0;
            bufInfo.range  = sizeof(TAAMatrices);
            writes[3].sType           = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
            writes[3].dstSet          = resolveSets[i];
            writes[3].dstBinding      = 3;
            writes[3].descriptorCount = 1;
            writes[3].descriptorType  = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
            writes[3].pBufferInfo     = &bufInfo;
            vkUpdateDescriptorSets(ctx.device, 4, writes, 0, nullptr);
        }
        return true;
    }

    // 初始化后立即调用：将两张颜色图转到 SHADER_READ_ONLY_OPTIMAL
    // 这样第 0 帧 TAA resolve 采样 history 图像时 layout 正确
    // beginGeometryPass 用 oldLayout=UNDEFINED（丢弃内容），LOAD_OP_CLEAR 无需保留
    void initialLayoutTransition(VulkanContext& ctx) {
        VkCommandBuffer cmd = beginSingleTimeCommands(ctx);
        for (int i = 0; i < 2; ++i) {
            transitionImage(cmd, colorImages[i],
                VK_IMAGE_LAYOUT_UNDEFINED,
                VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
                VK_PIPELINE_STAGE_2_TOP_OF_PIPE_BIT,     VK_ACCESS_2_NONE,
                VK_PIPELINE_STAGE_2_FRAGMENT_SHADER_BIT, VK_ACCESS_2_SHADER_READ_BIT);
        }
        endSingleTimeCommands(ctx, cmd);
    }

    // 4×4 矩阵 inverse（纯数值，精度足够用于 TAA 重投影）
    static void computeInverseVP(const float* m, float* inv) {
        float a00=m[0], a01=m[4], a02=m[8],  a03=m[12];
        float a10=m[1], a11=m[5], a12=m[9],  a13=m[13];
        float a20=m[2], a21=m[6], a22=m[10], a23=m[14];
        float a30=m[3], a31=m[7], a32=m[11], a33=m[15];

        float b00 = a00*a11 - a01*a10, b01 = a00*a12 - a02*a10;
        float b02 = a00*a13 - a03*a10, b03 = a01*a12 - a02*a11;
        float b04 = a01*a13 - a03*a11, b05 = a02*a13 - a03*a12;
        float b06 = a20*a31 - a21*a30, b07 = a20*a32 - a22*a30;
        float b08 = a20*a33 - a23*a30, b09 = a21*a32 - a22*a31;
        float b10 = a21*a33 - a23*a31, b11 = a22*a33 - a23*a32;

        float det = b00*b11 - b01*b10 + b02*b09 + b03*b08 - b04*b07 + b05*b06;
        if (fabsf(det) < 1e-10f) { memset(inv, 0, 64); inv[0]=inv[5]=inv[10]=inv[15]=1.f; return; }
        det = 1.0f / det;

        inv[0] = ( a11*b11 - a12*b10 + a13*b09) * det;
        inv[1] = (-a10*b11 + a12*b08 - a13*b07) * det;
        inv[2] = ( a10*b10 - a11*b08 + a13*b06) * det;
        inv[3] = (-a10*b09 + a11*b07 - a12*b06) * det;
        inv[4] = (-a01*b11 + a02*b10 - a03*b09) * det;
        inv[5] = ( a00*b11 - a02*b08 + a03*b07) * det;
        inv[6] = (-a00*b10 + a01*b08 - a03*b06) * det;
        inv[7] = ( a00*b09 - a01*b07 + a02*b06) * det;
        inv[8] = ( a31*b05 - a32*b04 + a33*b03) * det;
        inv[9] = (-a30*b05 + a32*b02 - a33*b01) * det;
        inv[10]= ( a30*b04 - a31*b02 + a33*b00) * det;
        inv[11]= (-a30*b03 + a31*b01 - a32*b00) * det;
        inv[12]= (-a21*b05 + a22*b04 - a23*b03) * det;
        inv[13]= ( a20*b05 - a22*b02 + a23*b01) * det;
        inv[14]= (-a20*b04 + a21*b02 - a23*b00) * det;
        inv[15]= ( a20*b03 - a21*b01 + a22*b00) * det;
    }

    void destroyImages(VulkanContext& ctx) {
        for (int i = 0; i < 2; ++i) {
            if (colorViews[i]  != VK_NULL_HANDLE) { vkDestroyImageView(ctx.device, colorViews[i], nullptr);               colorViews[i]  = VK_NULL_HANDLE; }
            if (colorImages[i] != VK_NULL_HANDLE) { vmaDestroyImage(ctx.allocator, colorImages[i], colorAllocs[i]);        colorImages[i] = VK_NULL_HANDLE; }
        }
        if (depthView  != VK_NULL_HANDLE) { vkDestroyImageView(ctx.device, depthView, nullptr);               depthView  = VK_NULL_HANDLE; }
        if (depthImage != VK_NULL_HANDLE) { vmaDestroyImage(ctx.allocator, depthImage, depthAlloc);            depthImage = VK_NULL_HANDLE; }
    }
};
