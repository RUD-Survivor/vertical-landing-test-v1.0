#pragma once
// ==========================================================================
// vk_pipeline.h — SPIR-V 加载 + 网格渲染管线创建
//
// 使用 VkPipelineRenderingCreateInfo (Vulkan 1.3 dynamic rendering)，
// 无需 VkRenderPass 对象：格式通过 pNext 链在管线创建时指定。
// ==========================================================================

#include "vk_context.h"
#include "vk_descriptors.h"
#include "vk_renderpass.h"  // VkFrameRenderer (for colorFormat/depthFormat)

#include <vector>
#include <cstdio>

// -----------------------------------------------------------------------
// SPIR-V 文件加载
// -----------------------------------------------------------------------
inline std::vector<uint32_t> loadSPIRV(const char* path) {
    FILE* f = nullptr;
    fopen_s(&f, path, "rb");
    if (!f) {
        fprintf(stderr, "[VkPipeline] Cannot open shader: %s\n", path);
        return {};
    }
    fseek(f, 0, SEEK_END);
    long size = ftell(f);
    fseek(f, 0, SEEK_SET);
    std::vector<uint32_t> code((size_t)size / sizeof(uint32_t));
    fread(code.data(), 1, (size_t)size, f);
    fclose(f);
    return code;
}

inline VkShaderModule createShaderModule(VkDevice device, const std::vector<uint32_t>& code) {
    VkShaderModuleCreateInfo ci{ VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO };
    ci.codeSize = code.size() * sizeof(uint32_t);
    ci.pCode    = code.data();
    VkShaderModule mod = VK_NULL_HANDLE;
    if (vkCreateShaderModule(device, &ci, nullptr, &mod) != VK_SUCCESS)
        fprintf(stderr, "[VkPipeline] Failed to create shader module\n");
    return mod;
}

// -----------------------------------------------------------------------
// VkMeshPipeline — 不透明网格管线（等价于 program3d）
//
// Vertex layout: Vertex3D (48 bytes)
//   location 0: pos    (R32G32B32_SFLOAT,    offset  0)
//   location 1: normal (R32G32B32_SFLOAT,    offset 12)
//   location 2: uv     (R32G32_SFLOAT,       offset 24)
//   location 3: color  (R32G32B32A32_SFLOAT, offset 32)
// -----------------------------------------------------------------------
struct VkMeshPipeline {
    VkPipelineLayout layout   = VK_NULL_HANDLE;
    VkPipeline       pipeline = VK_NULL_HANDLE;

    // colorFormat/depthFormat: from VkFrameRenderer (swapchain + depth format)
    bool init(VulkanContext&        ctx,
              VkDescriptorManager&  desc,
              VkFormat              colorFormat,
              VkFormat              depthFormat,
              const char*           vertSpv,
              const char*           fragSpv) {
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

        // --- Shader stages ---
        VkPipelineShaderStageCreateInfo stages[2]{};
        stages[0].sType  = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
        stages[0].stage  = VK_SHADER_STAGE_VERTEX_BIT;
        stages[0].module = vertMod;
        stages[0].pName  = "main";
        stages[1].sType  = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
        stages[1].stage  = VK_SHADER_STAGE_FRAGMENT_BIT;
        stages[1].module = fragMod;
        stages[1].pName  = "main";

        // --- Vertex input: Vertex3D (48 bytes) ---
        VkVertexInputBindingDescription binding{};
        binding.binding   = 0;
        binding.stride    = 48;  // sizeof(Vertex3D)
        binding.inputRate = VK_VERTEX_INPUT_RATE_VERTEX;

        VkVertexInputAttributeDescription attrs[4]{};
        attrs[0].location = 0; attrs[0].binding = 0; attrs[0].format = VK_FORMAT_R32G32B32_SFLOAT;    attrs[0].offset = 0;
        attrs[1].location = 1; attrs[1].binding = 0; attrs[1].format = VK_FORMAT_R32G32B32_SFLOAT;    attrs[1].offset = 12;
        attrs[2].location = 2; attrs[2].binding = 0; attrs[2].format = VK_FORMAT_R32G32_SFLOAT;       attrs[2].offset = 24;
        attrs[3].location = 3; attrs[3].binding = 0; attrs[3].format = VK_FORMAT_R32G32B32A32_SFLOAT; attrs[3].offset = 32;

        VkPipelineVertexInputStateCreateInfo viState{ VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO };
        viState.vertexBindingDescriptionCount   = 1;
        viState.pVertexBindingDescriptions      = &binding;
        viState.vertexAttributeDescriptionCount = 4;
        viState.pVertexAttributeDescriptions    = attrs;

        // --- Input assembly ---
        VkPipelineInputAssemblyStateCreateInfo iaState{ VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO };
        iaState.topology = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;

        // --- Viewport/scissor (dynamic — no rebuild on resize) ---
        VkPipelineViewportStateCreateInfo vpState{ VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO };
        vpState.viewportCount = 1;
        vpState.scissorCount  = 1;

        // --- Rasterizer ---
        VkPipelineRasterizationStateCreateInfo rasState{ VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO };
        rasState.polygonMode = VK_POLYGON_MODE_FILL;
        rasState.cullMode    = VK_CULL_MODE_NONE;  // match OpenGL default (cull disabled)
        rasState.frontFace   = VK_FRONT_FACE_COUNTER_CLOCKWISE;
        rasState.lineWidth   = 1.0f;

        // --- Multisampling (MSAA disabled; Phase 4 may enable 4x) ---
        VkPipelineMultisampleStateCreateInfo msState{ VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO };
        msState.rasterizationSamples = VK_SAMPLE_COUNT_1_BIT;

        // --- Depth/stencil ---
        VkPipelineDepthStencilStateCreateInfo dsState{ VK_STRUCTURE_TYPE_PIPELINE_DEPTH_STENCIL_STATE_CREATE_INFO };
        dsState.depthTestEnable  = VK_TRUE;
        dsState.depthWriteEnable = VK_TRUE;
        dsState.depthCompareOp   = VK_COMPARE_OP_LESS;

        // --- Color blend (opaque — no blending) ---
        VkPipelineColorBlendAttachmentState blendAttach{};
        blendAttach.colorWriteMask = VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT
                                   | VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT;
        blendAttach.blendEnable = VK_FALSE;

        VkPipelineColorBlendStateCreateInfo blendState{ VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO };
        blendState.attachmentCount = 1;
        blendState.pAttachments    = &blendAttach;

        // --- Dynamic state: viewport + scissor ---
        VkDynamicState dynStates[] = { VK_DYNAMIC_STATE_VIEWPORT, VK_DYNAMIC_STATE_SCISSOR };
        VkPipelineDynamicStateCreateInfo dynState{ VK_STRUCTURE_TYPE_PIPELINE_DYNAMIC_STATE_CREATE_INFO };
        dynState.dynamicStateCount = 2;
        dynState.pDynamicStates    = dynStates;

        // --- Pipeline layout: Set 0 + Set 1 + push constants ---
        VkPushConstantRange pcRange{};
        pcRange.stageFlags = VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT;
        pcRange.offset     = 0;
        pcRange.size       = sizeof(MeshPushConstants);  // 88 bytes

        VkDescriptorSetLayout setLayouts[] = { desc.set0Layout, desc.set1Layout };
        VkPipelineLayoutCreateInfo layoutCI{ VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO };
        layoutCI.setLayoutCount         = 2;
        layoutCI.pSetLayouts            = setLayouts;
        layoutCI.pushConstantRangeCount = 1;
        layoutCI.pPushConstantRanges    = &pcRange;

        if (vkCreatePipelineLayout(ctx.device, &layoutCI, nullptr, &layout) != VK_SUCCESS) {
            fprintf(stderr, "[VkPipeline] Failed to create pipeline layout\n");
            vkDestroyShaderModule(ctx.device, vertMod, nullptr);
            vkDestroyShaderModule(ctx.device, fragMod, nullptr);
            return false;
        }

        // --- Dynamic rendering: 通过 pNext 指定 attachment 格式，无需 VkRenderPass ---
        VkPipelineRenderingCreateInfo renderingCI{ VK_STRUCTURE_TYPE_PIPELINE_RENDERING_CREATE_INFO };
        renderingCI.colorAttachmentCount    = 1;
        renderingCI.pColorAttachmentFormats = &colorFormat;
        renderingCI.depthAttachmentFormat   = depthFormat;

        // --- Graphics pipeline ---
        VkGraphicsPipelineCreateInfo pipeCI{ VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO };
        pipeCI.pNext               = &renderingCI;  // dynamic rendering via pNext
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
        pipeCI.layout              = layout;
        pipeCI.renderPass          = VK_NULL_HANDLE;  // dynamic rendering 不需要 renderPass

        VkResult result = vkCreateGraphicsPipelines(
            ctx.device, VK_NULL_HANDLE, 1, &pipeCI, nullptr, &pipeline);

        vkDestroyShaderModule(ctx.device, vertMod, nullptr);
        vkDestroyShaderModule(ctx.device, fragMod, nullptr);

        if (result != VK_SUCCESS) {
            fprintf(stderr, "[VkPipeline] Failed to create mesh pipeline\n");
            return false;
        }
        printf("[Vulkan] Mesh opaque pipeline created\n");
        return true;
    }

    // Bind this pipeline for subsequent draw calls
    void bind(VkCommandBuffer cmd) const {
        vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, pipeline);
    }

    void shutdown(VkDevice device) {
        if (pipeline != VK_NULL_HANDLE) { vkDestroyPipeline      (device, pipeline, nullptr); pipeline = VK_NULL_HANDLE; }
        if (layout   != VK_NULL_HANDLE) { vkDestroyPipelineLayout(device, layout,   nullptr); layout   = VK_NULL_HANDLE; }
    }
};
