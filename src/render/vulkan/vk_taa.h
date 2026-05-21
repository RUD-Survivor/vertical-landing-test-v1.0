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
#include "vk_pipeline.h"  // loadSPIRV, createShaderModule

#include <cstdio>

// Push constants for TAA resolve pass
struct TAAParams {
    float blendFactor = 0.1f;  // 0.1 = smooth, 1.0 = disable TAA (debug)
    float _pad[3]     = {};
};
static_assert(sizeof(TAAParams) <= 128, "TAAParams exceeds push constant limit");

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

    int currentIdx = 0;  // ping-pong, flipped by swap() each frame

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
        if (!createImages(ctx, extent))                         return false;
        if (!createSamplers(ctx))                               return false;
        if (!createResolvePipeline(ctx, swapFormat, vertSpv, fragSpv)) return false;
        if (!createDescriptors(ctx))                            return false;
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
        if (resolvePool != VK_NULL_HANDLE) {
            vkDestroyDescriptorPool(ctx.device, resolvePool, nullptr);
            resolvePool = VK_NULL_HANDLE;
        }
        destroyImages(ctx);
        currentIdx = 0;

        // resolve pipeline 只有 swapFormat 变化才需重建（极少见）
        if (resolvePipeline != VK_NULL_HANDLE) {
            vkDestroyPipeline(ctx.device, resolvePipeline, nullptr);
            resolvePipeline = VK_NULL_HANDLE;
        }
        if (resolveLayout != VK_NULL_HANDLE) {
            vkDestroyPipelineLayout(ctx.device, resolveLayout, nullptr);
            resolveLayout = VK_NULL_HANDLE;
        }

        createImages(ctx, extent);
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
    void beginGeometryPass(VkCommandBuffer cmd, VkExtent2D extent) const {
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
            ici.usage         = VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT;
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

        // Descriptor set layout: binding 0=current, 1=history, 2=depth (all sampler2D)
        VkDescriptorSetLayoutBinding bindings[3]{};
        for (uint32_t i = 0; i < 3; ++i) {
            bindings[i].binding         = i;
            bindings[i].descriptorType  = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
            bindings[i].descriptorCount = 1;
            bindings[i].stageFlags      = VK_SHADER_STAGE_FRAGMENT_BIT;
        }
        VkDescriptorSetLayoutCreateInfo setCI{ VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO };
        setCI.bindingCount = 3;
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
        VkDescriptorPoolSize poolSize{ VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 6 };
        VkDescriptorPoolCreateInfo poolCI{ VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO };
        poolCI.maxSets       = 2;
        poolCI.poolSizeCount = 1;
        poolCI.pPoolSizes    = &poolSize;
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

        // resolveSets[i]: current=colorViews[i], history=colorViews[1-i], depth=depthView
        for (int i = 0; i < 2; ++i) {
            VkDescriptorImageInfo imgs[3]{};
            imgs[0] = { colorSampler, colorViews[i],   VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL };
            imgs[1] = { colorSampler, colorViews[1-i], VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL };
            imgs[2] = { depthSampler, depthView,       VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL };

            VkWriteDescriptorSet writes[3]{};
            for (uint32_t b = 0; b < 3; ++b) {
                writes[b].sType           = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
                writes[b].dstSet          = resolveSets[i];
                writes[b].dstBinding      = b;
                writes[b].descriptorCount = 1;
                writes[b].descriptorType  = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
                writes[b].pImageInfo      = &imgs[b];
            }
            vkUpdateDescriptorSets(ctx.device, 3, writes, 0, nullptr);
        }
        return true;
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
