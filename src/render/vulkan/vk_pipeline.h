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
        rasState.frontFace   = VK_FRONT_FACE_CLOCKWISE;  // CW=正面（MeshGen 生成 CW 外面）
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
        // 使用 PlanetPushConstants 的大小（104 bytes），使 mesh pipeline 与
        // planet pipeline 的 layout 兼容（Vulkan 要求 pcRange 一致才能在同
        // 一 cmd buffer 内跨 pipeline 复用 descriptor set 绑定）。
        VkPushConstantRange pcRange{};
        pcRange.stageFlags = VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT;
        pcRange.offset     = 0;
        pcRange.size       = sizeof(PlanetPushConstants);  // 104 bytes

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

// -----------------------------------------------------------------------
// VkPlanetPipeline — 行星表面管线（复用 mesh.vert，专用 frag 着色器）
//
// 与 VkMeshPipeline 共享相同的 descriptor set layout（Set 0 FrameUBO +
// Set 1 sampler），push constant range 同为 104 bytes（PlanetPushConstants），
// 因此两者 layout 完全兼容，descriptor set 绑定可跨 pipeline 复用。
// -----------------------------------------------------------------------
struct VkPlanetPipeline {
    VkPipelineLayout layout   = VK_NULL_HANDLE;
    VkPipeline       pipeline = VK_NULL_HANDLE;

    bool init(VulkanContext&       ctx,
              VkDescriptorManager& desc,
              VkFormat             colorFormat,
              VkFormat             depthFormat,
              const char*          vertSpv,
              const char*          fragSpv) {
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

        VkPipelineShaderStageCreateInfo stages[2]{};
        stages[0].sType  = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
        stages[0].stage  = VK_SHADER_STAGE_VERTEX_BIT;
        stages[0].module = vertMod;  stages[0].pName = "main";
        stages[1].sType  = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
        stages[1].stage  = VK_SHADER_STAGE_FRAGMENT_BIT;
        stages[1].module = fragMod;  stages[1].pName = "main";

        // 顶点输入与 VkMeshPipeline 相同（Vertex3D 48 bytes）
        VkVertexInputBindingDescription binding{ 0, 48, VK_VERTEX_INPUT_RATE_VERTEX };
        VkVertexInputAttributeDescription attrs[4]{};
        attrs[0] = { 0, 0, VK_FORMAT_R32G32B32_SFLOAT,    0  };
        attrs[1] = { 1, 0, VK_FORMAT_R32G32B32_SFLOAT,    12 };
        attrs[2] = { 2, 0, VK_FORMAT_R32G32_SFLOAT,       24 };
        attrs[3] = { 3, 0, VK_FORMAT_R32G32B32A32_SFLOAT, 32 };
        VkPipelineVertexInputStateCreateInfo viState{ VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO };
        viState.vertexBindingDescriptionCount   = 1;   viState.pVertexBindingDescriptions      = &binding;
        viState.vertexAttributeDescriptionCount = 4;   viState.pVertexAttributeDescriptions    = attrs;

        VkPipelineInputAssemblyStateCreateInfo iaState{ VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO };
        iaState.topology = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;

        VkPipelineViewportStateCreateInfo vpState{ VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO };
        vpState.viewportCount = 1;  vpState.scissorCount = 1;

        VkPipelineRasterizationStateCreateInfo rasState{ VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO };
        rasState.polygonMode = VK_POLYGON_MODE_FILL;
        rasState.cullMode    = VK_CULL_MODE_BACK_BIT;
        rasState.frontFace   = VK_FRONT_FACE_CLOCKWISE;  // CW=正面（MeshGen::sphere 是 CW 外面）
        rasState.lineWidth   = 1.0f;

        VkPipelineMultisampleStateCreateInfo msState{ VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO };
        msState.rasterizationSamples = VK_SAMPLE_COUNT_1_BIT;

        VkPipelineDepthStencilStateCreateInfo dsState{ VK_STRUCTURE_TYPE_PIPELINE_DEPTH_STENCIL_STATE_CREATE_INFO };
        dsState.depthTestEnable  = VK_TRUE;
        dsState.depthWriteEnable = VK_TRUE;
        dsState.depthCompareOp   = VK_COMPARE_OP_LESS;

        VkPipelineColorBlendAttachmentState blendAttach{};
        blendAttach.colorWriteMask = VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT
                                   | VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT;
        blendAttach.blendEnable = VK_FALSE;
        VkPipelineColorBlendStateCreateInfo blendState{ VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO };
        blendState.attachmentCount = 1;  blendState.pAttachments = &blendAttach;

        VkDynamicState dynStates[] = { VK_DYNAMIC_STATE_VIEWPORT, VK_DYNAMIC_STATE_SCISSOR };
        VkPipelineDynamicStateCreateInfo dynState{ VK_STRUCTURE_TYPE_PIPELINE_DYNAMIC_STATE_CREATE_INFO };
        dynState.dynamicStateCount = 2;  dynState.pDynamicStates = dynStates;

        VkPushConstantRange pcRange{};
        pcRange.stageFlags = VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT;
        pcRange.size       = sizeof(PlanetPushConstants);  // 104 bytes

        VkDescriptorSetLayout setLayouts[] = { desc.set0Layout, desc.set1Layout };
        VkPipelineLayoutCreateInfo layoutCI{ VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO };
        layoutCI.setLayoutCount         = 2;  layoutCI.pSetLayouts            = setLayouts;
        layoutCI.pushConstantRangeCount = 1;  layoutCI.pPushConstantRanges    = &pcRange;
        if (vkCreatePipelineLayout(ctx.device, &layoutCI, nullptr, &layout) != VK_SUCCESS) {
            vkDestroyShaderModule(ctx.device, vertMod, nullptr);
            vkDestroyShaderModule(ctx.device, fragMod, nullptr);
            return false;
        }

        VkPipelineRenderingCreateInfo renderingCI{ VK_STRUCTURE_TYPE_PIPELINE_RENDERING_CREATE_INFO };
        renderingCI.colorAttachmentCount    = 1;
        renderingCI.pColorAttachmentFormats = &colorFormat;
        renderingCI.depthAttachmentFormat   = depthFormat;

        VkGraphicsPipelineCreateInfo pipeCI{ VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO };
        pipeCI.pNext               = &renderingCI;
        pipeCI.stageCount          = 2;  pipeCI.pStages             = stages;
        pipeCI.pVertexInputState   = &viState;  pipeCI.pInputAssemblyState = &iaState;
        pipeCI.pViewportState      = &vpState;  pipeCI.pRasterizationState = &rasState;
        pipeCI.pMultisampleState   = &msState;  pipeCI.pDepthStencilState  = &dsState;
        pipeCI.pColorBlendState    = &blendState;  pipeCI.pDynamicState    = &dynState;
        pipeCI.layout              = layout;
        pipeCI.renderPass          = VK_NULL_HANDLE;

        VkResult result = vkCreateGraphicsPipelines(ctx.device, VK_NULL_HANDLE, 1, &pipeCI, nullptr, &pipeline);
        vkDestroyShaderModule(ctx.device, vertMod, nullptr);
        vkDestroyShaderModule(ctx.device, fragMod, nullptr);

        if (result != VK_SUCCESS) {
            fprintf(stderr, "[VkPipeline] Failed to create planet pipeline\n");
            return false;
        }
        printf("[Vulkan] Planet pipeline created\n");
        return true;
    }

    void bind(VkCommandBuffer cmd) const {
        vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, pipeline);
    }

    void shutdown(VkDevice device) {
        if (pipeline != VK_NULL_HANDLE) { vkDestroyPipeline      (device, pipeline, nullptr); pipeline = VK_NULL_HANDLE; }
        if (layout   != VK_NULL_HANDLE) { vkDestroyPipelineLayout(device, layout,   nullptr); layout   = VK_NULL_HANDLE; }
    }
};

// -----------------------------------------------------------------------
// 通用管线构建辅助 — 减少以下各管线的重复代码
// -----------------------------------------------------------------------
struct PipelineInitParams {
    const VkVertexInputBindingDescription*   bindings     = nullptr;
    uint32_t                                 bindingCount = 0;
    const VkVertexInputAttributeDescription* attrs        = nullptr;
    uint32_t                                 attrCount    = 0;
    bool          blendEnable = false;
    VkBlendFactor srcColor    = VK_BLEND_FACTOR_SRC_ALPHA;
    VkBlendFactor dstColor    = VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA;
    VkBlendFactor srcAlpha    = VK_BLEND_FACTOR_ONE;
    VkBlendFactor dstAlpha    = VK_BLEND_FACTOR_ZERO;
    VkBlendOp     blendOp     = VK_BLEND_OP_ADD;
    VkBool32        depthTest  = VK_TRUE;
    VkBool32        depthWrite = VK_TRUE;
    VkCompareOp     depthOp    = VK_COMPARE_OP_LESS;
    VkCullModeFlags cullMode   = VK_CULL_MODE_NONE;
    uint32_t pcSize = 0;
    const VkDescriptorSetLayout* setLayouts = nullptr;
    uint32_t                     setCount   = 0;
    VkFormat colorFmt = VK_FORMAT_UNDEFINED;
    VkFormat depthFmt = VK_FORMAT_UNDEFINED;
    VkPrimitiveTopology topology = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;
    const char* name = "Pipeline";
};

inline bool buildPipeline(VkDevice device,
                           VkShaderModule vertMod, VkShaderModule fragMod,
                           const PipelineInitParams& p,
                           VkPipelineLayout& outLayout, VkPipeline& outPipeline) {
    VkPipelineShaderStageCreateInfo stages[2]{};
    stages[0].sType=VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
    stages[0].stage=VK_SHADER_STAGE_VERTEX_BIT;   stages[0].module=vertMod; stages[0].pName="main";
    stages[1].sType=VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
    stages[1].stage=VK_SHADER_STAGE_FRAGMENT_BIT; stages[1].module=fragMod; stages[1].pName="main";

    VkPipelineVertexInputStateCreateInfo viState{VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO};
    viState.vertexBindingDescriptionCount=p.bindingCount; viState.pVertexBindingDescriptions=p.bindings;
    viState.vertexAttributeDescriptionCount=p.attrCount;  viState.pVertexAttributeDescriptions=p.attrs;

    VkPipelineInputAssemblyStateCreateInfo iaState{VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO};
    iaState.topology=p.topology;

    VkPipelineViewportStateCreateInfo vpState{VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO};
    vpState.viewportCount=1; vpState.scissorCount=1;

    VkPipelineRasterizationStateCreateInfo rasState{VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO};
    rasState.polygonMode=VK_POLYGON_MODE_FILL; rasState.cullMode=p.cullMode;
    rasState.frontFace=VK_FRONT_FACE_CLOCKWISE; rasState.lineWidth=1.0f;

    VkPipelineMultisampleStateCreateInfo msState{VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO};
    msState.rasterizationSamples=VK_SAMPLE_COUNT_1_BIT;

    VkPipelineDepthStencilStateCreateInfo dsState{VK_STRUCTURE_TYPE_PIPELINE_DEPTH_STENCIL_STATE_CREATE_INFO};
    dsState.depthTestEnable=p.depthTest; dsState.depthWriteEnable=p.depthWrite; dsState.depthCompareOp=p.depthOp;

    VkPipelineColorBlendAttachmentState blendAttach{};
    blendAttach.colorWriteMask=VK_COLOR_COMPONENT_R_BIT|VK_COLOR_COMPONENT_G_BIT|VK_COLOR_COMPONENT_B_BIT|VK_COLOR_COMPONENT_A_BIT;
    blendAttach.blendEnable=p.blendEnable?VK_TRUE:VK_FALSE;
    blendAttach.srcColorBlendFactor=p.srcColor; blendAttach.dstColorBlendFactor=p.dstColor;
    blendAttach.srcAlphaBlendFactor=p.srcAlpha; blendAttach.dstAlphaBlendFactor=p.dstAlpha;
    blendAttach.colorBlendOp=p.blendOp;         blendAttach.alphaBlendOp=VK_BLEND_OP_ADD;

    VkPipelineColorBlendStateCreateInfo blendState{VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO};
    blendState.attachmentCount=1; blendState.pAttachments=&blendAttach;

    VkDynamicState dynArr[]={VK_DYNAMIC_STATE_VIEWPORT,VK_DYNAMIC_STATE_SCISSOR};
    VkPipelineDynamicStateCreateInfo dynState{VK_STRUCTURE_TYPE_PIPELINE_DYNAMIC_STATE_CREATE_INFO};
    dynState.dynamicStateCount=2; dynState.pDynamicStates=dynArr;

    VkPushConstantRange pcRange{};
    pcRange.stageFlags=VK_SHADER_STAGE_VERTEX_BIT|VK_SHADER_STAGE_FRAGMENT_BIT; pcRange.size=p.pcSize;

    VkPipelineLayoutCreateInfo layoutCI{VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO};
    layoutCI.setLayoutCount=p.setCount; layoutCI.pSetLayouts=p.setLayouts;
    layoutCI.pushConstantRangeCount=(p.pcSize>0)?1u:0u; layoutCI.pPushConstantRanges=(p.pcSize>0)?&pcRange:nullptr;
    if (vkCreatePipelineLayout(device,&layoutCI,nullptr,&outLayout)!=VK_SUCCESS) {
        fprintf(stderr,"[VkPipeline] Layout create failed: %s\n",p.name); return false;
    }

    VkPipelineRenderingCreateInfo renderingCI{VK_STRUCTURE_TYPE_PIPELINE_RENDERING_CREATE_INFO};
    renderingCI.colorAttachmentCount=1; renderingCI.pColorAttachmentFormats=&p.colorFmt;
    renderingCI.depthAttachmentFormat=p.depthFmt;

    VkGraphicsPipelineCreateInfo pipeCI{VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO};
    pipeCI.pNext=&renderingCI; pipeCI.stageCount=2; pipeCI.pStages=stages;
    pipeCI.pVertexInputState=&viState; pipeCI.pInputAssemblyState=&iaState;
    pipeCI.pViewportState=&vpState;    pipeCI.pRasterizationState=&rasState;
    pipeCI.pMultisampleState=&msState; pipeCI.pDepthStencilState=&dsState;
    pipeCI.pColorBlendState=&blendState; pipeCI.pDynamicState=&dynState;
    pipeCI.layout=outLayout; pipeCI.renderPass=VK_NULL_HANDLE;

    if (vkCreateGraphicsPipelines(device,VK_NULL_HANDLE,1,&pipeCI,nullptr,&outPipeline)!=VK_SUCCESS) {
        fprintf(stderr,"[VkPipeline] Pipeline create failed: %s\n",p.name); return false;
    }
    printf("[Vulkan] %s pipeline created\n",p.name);
    return true;
}

// -----------------------------------------------------------------------
// VkRingPipeline — Saturn ring (alpha blend, depth test, no depth write)
// Layout matches VkPlanetPipeline (PlanetPushConstants 104 bytes, Set 0+1).
// pc.ambientStr = planet radius in render units.
// -----------------------------------------------------------------------
struct VkRingPipeline {
    VkPipelineLayout layout=VK_NULL_HANDLE; VkPipeline pipeline=VK_NULL_HANDLE;
    bool init(VulkanContext& ctx, VkDescriptorManager& desc,
              VkFormat colorFmt, VkFormat depthFmt, const char* vertSpv, const char* fragSpv) {
        auto vc=loadSPIRV(vertSpv); auto fc=loadSPIRV(fragSpv);
        if(vc.empty()||fc.empty()) return false;
        VkShaderModule vm=createShaderModule(ctx.device,vc), fm=createShaderModule(ctx.device,fc);
        if(!vm||!fm){if(vm)vkDestroyShaderModule(ctx.device,vm,nullptr);if(fm)vkDestroyShaderModule(ctx.device,fm,nullptr);return false;}
        VkVertexInputBindingDescription   bind{0,48,VK_VERTEX_INPUT_RATE_VERTEX};
        VkVertexInputAttributeDescription attr[4]{{0,0,VK_FORMAT_R32G32B32_SFLOAT,0},{1,0,VK_FORMAT_R32G32B32_SFLOAT,12},{2,0,VK_FORMAT_R32G32_SFLOAT,24},{3,0,VK_FORMAT_R32G32B32A32_SFLOAT,32}};
        VkDescriptorSetLayout sets[]={desc.set0Layout,desc.set1Layout};
        PipelineInitParams p{}; p.bindings=&bind; p.bindingCount=1; p.attrs=attr; p.attrCount=4;
        p.blendEnable=true; p.srcColor=VK_BLEND_FACTOR_SRC_ALPHA; p.dstColor=VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA;
        p.srcAlpha=VK_BLEND_FACTOR_ONE; p.dstAlpha=VK_BLEND_FACTOR_ZERO;
        p.depthTest=VK_TRUE; p.depthWrite=VK_FALSE; p.depthOp=VK_COMPARE_OP_LESS; p.cullMode=VK_CULL_MODE_NONE;
        p.pcSize=sizeof(PlanetPushConstants); p.setLayouts=sets; p.setCount=2;
        p.colorFmt=colorFmt; p.depthFmt=depthFmt; p.name="Ring";
        bool ok=buildPipeline(ctx.device,vm,fm,p,layout,pipeline);
        vkDestroyShaderModule(ctx.device,vm,nullptr); vkDestroyShaderModule(ctx.device,fm,nullptr); return ok;
    }
    void bind(VkCommandBuffer cmd) const{vkCmdBindPipeline(cmd,VK_PIPELINE_BIND_POINT_GRAPHICS,pipeline);}
    void shutdown(VkDevice d){if(pipeline){vkDestroyPipeline(d,pipeline,nullptr);pipeline=VK_NULL_HANDLE;}if(layout){vkDestroyPipelineLayout(d,layout,nullptr);layout=VK_NULL_HANDLE;}}
};

// -----------------------------------------------------------------------
// VkSkyboxPipeline — Fullscreen triangle (no vertex buffer, no depth write)
// Set 0 only. Push: SkyboxPushConstants (80 bytes).
// Draw: vkCmdDraw(cmd, 3, 1, 0, 0)
// -----------------------------------------------------------------------
struct VkSkyboxPipeline {
    VkPipelineLayout layout=VK_NULL_HANDLE; VkPipeline pipeline=VK_NULL_HANDLE;
    bool init(VulkanContext& ctx, VkDescriptorManager& desc,
              VkFormat colorFmt, VkFormat depthFmt, const char* vertSpv, const char* fragSpv) {
        auto vc=loadSPIRV(vertSpv); auto fc=loadSPIRV(fragSpv);
        if(vc.empty()||fc.empty()) return false;
        VkShaderModule vm=createShaderModule(ctx.device,vc), fm=createShaderModule(ctx.device,fc);
        if(!vm||!fm){if(vm)vkDestroyShaderModule(ctx.device,vm,nullptr);if(fm)vkDestroyShaderModule(ctx.device,fm,nullptr);return false;}
        PipelineInitParams p{}; // No vertex input
        p.blendEnable=false;
        p.depthTest=VK_TRUE; p.depthWrite=VK_FALSE; p.depthOp=VK_COMPARE_OP_LESS_OR_EQUAL;
        p.cullMode=VK_CULL_MODE_NONE;
        p.pcSize=sizeof(SkyboxPushConstants); p.setLayouts=&desc.set0Layout; p.setCount=1;
        p.colorFmt=colorFmt; p.depthFmt=depthFmt; p.name="Skybox";
        bool ok=buildPipeline(ctx.device,vm,fm,p,layout,pipeline);
        vkDestroyShaderModule(ctx.device,vm,nullptr); vkDestroyShaderModule(ctx.device,fm,nullptr); return ok;
    }
    void bind(VkCommandBuffer cmd) const{vkCmdBindPipeline(cmd,VK_PIPELINE_BIND_POINT_GRAPHICS,pipeline);}
    void shutdown(VkDevice d){if(pipeline){vkDestroyPipeline(d,pipeline,nullptr);pipeline=VK_NULL_HANDLE;}if(layout){vkDestroyPipelineLayout(d,layout,nullptr);layout=VK_NULL_HANDLE;}}
};

// -----------------------------------------------------------------------
// VkExhaustPipeline — Additive volumetric plume, pos-only vertex (stride=12)
// Set 0 only. Push: ExhaustPushConstants (80 bytes).
// -----------------------------------------------------------------------
struct VkExhaustPipeline {
    VkPipelineLayout layout=VK_NULL_HANDLE; VkPipeline pipeline=VK_NULL_HANDLE;
    bool init(VulkanContext& ctx, VkDescriptorManager& desc,
              VkFormat colorFmt, VkFormat depthFmt, const char* vertSpv, const char* fragSpv) {
        auto vc=loadSPIRV(vertSpv); auto fc=loadSPIRV(fragSpv);
        if(vc.empty()||fc.empty()) return false;
        VkShaderModule vm=createShaderModule(ctx.device,vc), fm=createShaderModule(ctx.device,fc);
        if(!vm||!fm){if(vm)vkDestroyShaderModule(ctx.device,vm,nullptr);if(fm)vkDestroyShaderModule(ctx.device,fm,nullptr);return false;}
        VkVertexInputBindingDescription   bind{0,12,VK_VERTEX_INPUT_RATE_VERTEX};
        VkVertexInputAttributeDescription attr{0,0,VK_FORMAT_R32G32B32_SFLOAT,0};
        PipelineInitParams p{}; p.bindings=&bind; p.bindingCount=1; p.attrs=&attr; p.attrCount=1;
        p.blendEnable=true; p.srcColor=VK_BLEND_FACTOR_ONE; p.dstColor=VK_BLEND_FACTOR_ONE; // Additive
        p.srcAlpha=VK_BLEND_FACTOR_ONE; p.dstAlpha=VK_BLEND_FACTOR_ONE;
        p.depthTest=VK_TRUE; p.depthWrite=VK_FALSE; p.depthOp=VK_COMPARE_OP_LESS; p.cullMode=VK_CULL_MODE_NONE;
        p.pcSize=sizeof(ExhaustPushConstants); p.setLayouts=&desc.set0Layout; p.setCount=1;
        p.colorFmt=colorFmt; p.depthFmt=depthFmt; p.name="Exhaust";
        bool ok=buildPipeline(ctx.device,vm,fm,p,layout,pipeline);
        vkDestroyShaderModule(ctx.device,vm,nullptr); vkDestroyShaderModule(ctx.device,fm,nullptr); return ok;
    }
    void bind(VkCommandBuffer cmd) const{vkCmdBindPipeline(cmd,VK_PIPELINE_BIND_POINT_GRAPHICS,pipeline);}
    void shutdown(VkDevice d){if(pipeline){vkDestroyPipeline(d,pipeline,nullptr);pipeline=VK_NULL_HANDLE;}if(layout){vkDestroyPipelineLayout(d,layout,nullptr);layout=VK_NULL_HANDLE;}}
};

// -----------------------------------------------------------------------
// VkRibbonPipeline — Alpha-blended ribbon trail
// Vertex: vec3 pos(0) + vec4 color(1) + float qSide(2), stride=32.
// Set 0 only. Push: RibbonPushConstants (64 bytes).
// -----------------------------------------------------------------------
struct VkRibbonPipeline {
    VkPipelineLayout layout=VK_NULL_HANDLE; VkPipeline pipeline=VK_NULL_HANDLE;
    bool init(VulkanContext& ctx, VkDescriptorManager& desc,
              VkFormat colorFmt, VkFormat depthFmt, const char* vertSpv, const char* fragSpv) {
        auto vc=loadSPIRV(vertSpv); auto fc=loadSPIRV(fragSpv);
        if(vc.empty()||fc.empty()) return false;
        VkShaderModule vm=createShaderModule(ctx.device,vc), fm=createShaderModule(ctx.device,fc);
        if(!vm||!fm){if(vm)vkDestroyShaderModule(ctx.device,vm,nullptr);if(fm)vkDestroyShaderModule(ctx.device,fm,nullptr);return false;}
        VkVertexInputBindingDescription   bind{0,32,VK_VERTEX_INPUT_RATE_VERTEX};
        VkVertexInputAttributeDescription attr[3]{{0,0,VK_FORMAT_R32G32B32_SFLOAT,0},{1,0,VK_FORMAT_R32G32B32A32_SFLOAT,12},{2,0,VK_FORMAT_R32_SFLOAT,28}};
        PipelineInitParams p{}; p.bindings=&bind; p.bindingCount=1; p.attrs=attr; p.attrCount=3;
        p.blendEnable=true; p.srcColor=VK_BLEND_FACTOR_SRC_ALPHA; p.dstColor=VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA;
        p.srcAlpha=VK_BLEND_FACTOR_ONE; p.dstAlpha=VK_BLEND_FACTOR_ZERO;
        p.depthTest=VK_TRUE; p.depthWrite=VK_FALSE; p.depthOp=VK_COMPARE_OP_LESS; p.cullMode=VK_CULL_MODE_NONE;
        p.topology=VK_PRIMITIVE_TOPOLOGY_TRIANGLE_STRIP;
        p.pcSize=sizeof(RibbonPushConstants); p.setLayouts=&desc.set0Layout; p.setCount=1;
        p.colorFmt=colorFmt; p.depthFmt=depthFmt; p.name="Ribbon";
        bool ok=buildPipeline(ctx.device,vm,fm,p,layout,pipeline);
        vkDestroyShaderModule(ctx.device,vm,nullptr); vkDestroyShaderModule(ctx.device,fm,nullptr); return ok;
    }
    void bind(VkCommandBuffer cmd) const{vkCmdBindPipeline(cmd,VK_PIPELINE_BIND_POINT_GRAPHICS,pipeline);}
    void shutdown(VkDevice d){if(pipeline){vkDestroyPipeline(d,pipeline,nullptr);pipeline=VK_NULL_HANDLE;}if(layout){vkDestroyPipelineLayout(d,layout,nullptr);layout=VK_NULL_HANDLE;}}
};

// -----------------------------------------------------------------------
// VkBillboardPipeline — Screen-aligned sprite (alpha blend)
// Vertex: vec2 aPos (stride=8). Set 0 only. Push: BillboardPushConstants (48 bytes).
// -----------------------------------------------------------------------
struct VkBillboardPipeline {
    VkPipelineLayout layout=VK_NULL_HANDLE; VkPipeline pipeline=VK_NULL_HANDLE;
    bool init(VulkanContext& ctx, VkDescriptorManager& desc,
              VkFormat colorFmt, VkFormat depthFmt, const char* vertSpv, const char* fragSpv) {
        auto vc=loadSPIRV(vertSpv); auto fc=loadSPIRV(fragSpv);
        if(vc.empty()||fc.empty()) return false;
        VkShaderModule vm=createShaderModule(ctx.device,vc), fm=createShaderModule(ctx.device,fc);
        if(!vm||!fm){if(vm)vkDestroyShaderModule(ctx.device,vm,nullptr);if(fm)vkDestroyShaderModule(ctx.device,fm,nullptr);return false;}
        VkVertexInputBindingDescription   bind{0,8,VK_VERTEX_INPUT_RATE_VERTEX};
        VkVertexInputAttributeDescription attr{0,0,VK_FORMAT_R32G32_SFLOAT,0};
        PipelineInitParams p{}; p.bindings=&bind; p.bindingCount=1; p.attrs=&attr; p.attrCount=1;
        p.blendEnable=true; p.srcColor=VK_BLEND_FACTOR_SRC_ALPHA; p.dstColor=VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA;
        p.srcAlpha=VK_BLEND_FACTOR_ONE; p.dstAlpha=VK_BLEND_FACTOR_ZERO;
        p.depthTest=VK_TRUE; p.depthWrite=VK_FALSE; p.depthOp=VK_COMPARE_OP_LESS; p.cullMode=VK_CULL_MODE_NONE;
        p.topology=VK_PRIMITIVE_TOPOLOGY_TRIANGLE_STRIP;
        p.pcSize=sizeof(BillboardPushConstants); p.setLayouts=&desc.set0Layout; p.setCount=1;
        p.colorFmt=colorFmt; p.depthFmt=depthFmt; p.name="Billboard";
        bool ok=buildPipeline(ctx.device,vm,fm,p,layout,pipeline);
        vkDestroyShaderModule(ctx.device,vm,nullptr); vkDestroyShaderModule(ctx.device,fm,nullptr); return ok;
    }
    void bind(VkCommandBuffer cmd) const{vkCmdBindPipeline(cmd,VK_PIPELINE_BIND_POINT_GRAPHICS,pipeline);}
    void shutdown(VkDevice d){if(pipeline){vkDestroyPipeline(d,pipeline,nullptr);pipeline=VK_NULL_HANDLE;}if(layout){vkDestroyPipelineLayout(d,layout,nullptr);layout=VK_NULL_HANDLE;}}
};

// -----------------------------------------------------------------------
// VkLensFlarePipeline — Additive 2D lens flare (no depth test/write)
// Vertex: vec2 aPos (stride=8). No descriptor sets. Push: LensFlarePushConstants (64 bytes).
// -----------------------------------------------------------------------
struct VkLensFlarePipeline {
    VkPipelineLayout layout=VK_NULL_HANDLE; VkPipeline pipeline=VK_NULL_HANDLE;
    bool init(VulkanContext& ctx, VkFormat colorFmt, VkFormat depthFmt, const char* vertSpv, const char* fragSpv) {
        auto vc=loadSPIRV(vertSpv); auto fc=loadSPIRV(fragSpv);
        if(vc.empty()||fc.empty()) return false;
        VkShaderModule vm=createShaderModule(ctx.device,vc), fm=createShaderModule(ctx.device,fc);
        if(!vm||!fm){if(vm)vkDestroyShaderModule(ctx.device,vm,nullptr);if(fm)vkDestroyShaderModule(ctx.device,fm,nullptr);return false;}
        VkVertexInputBindingDescription   bind{0,8,VK_VERTEX_INPUT_RATE_VERTEX};
        VkVertexInputAttributeDescription attr{0,0,VK_FORMAT_R32G32_SFLOAT,0};
        PipelineInitParams p{}; p.bindings=&bind; p.bindingCount=1; p.attrs=&attr; p.attrCount=1;
        p.blendEnable=true; p.srcColor=VK_BLEND_FACTOR_ONE; p.dstColor=VK_BLEND_FACTOR_ONE; // Additive
        p.srcAlpha=VK_BLEND_FACTOR_ONE; p.dstAlpha=VK_BLEND_FACTOR_ONE;
        p.depthTest=VK_TRUE; p.depthWrite=VK_FALSE; p.depthOp=VK_COMPARE_OP_LESS_OR_EQUAL; p.cullMode=VK_CULL_MODE_NONE;
        p.topology=VK_PRIMITIVE_TOPOLOGY_TRIANGLE_STRIP;
        p.pcSize=sizeof(LensFlarePushConstants); p.setLayouts=nullptr; p.setCount=0;
        p.colorFmt=colorFmt; p.depthFmt=depthFmt; p.name="LensFlare";
        bool ok=buildPipeline(ctx.device,vm,fm,p,layout,pipeline);
        vkDestroyShaderModule(ctx.device,vm,nullptr); vkDestroyShaderModule(ctx.device,fm,nullptr); return ok;
    }
    void bind(VkCommandBuffer cmd) const{vkCmdBindPipeline(cmd,VK_PIPELINE_BIND_POINT_GRAPHICS,pipeline);}
    void shutdown(VkDevice d){if(pipeline){vkDestroyPipeline(d,pipeline,nullptr);pipeline=VK_NULL_HANDLE;}if(layout){vkDestroyPipelineLayout(d,layout,nullptr);layout=VK_NULL_HANDLE;}}
};

// -----------------------------------------------------------------------
// VkSVOPipeline — SVO 体素网格（与 VkMeshPipeline 顶点格式相同 stride=48，
// 片段使用逐顶点颜色 + 三平面噪声 Phong 光照）
// Set 0+1, MeshPushConstants (88 bytes).
// -----------------------------------------------------------------------
struct VkSVOPipeline {
    VkPipelineLayout layout=VK_NULL_HANDLE; VkPipeline pipeline=VK_NULL_HANDLE;
    bool init(VulkanContext& ctx, VkDescriptorManager& desc,
              VkFormat colorFmt, VkFormat depthFmt, const char* vertSpv, const char* fragSpv) {
        auto vc=loadSPIRV(vertSpv); auto fc=loadSPIRV(fragSpv);
        if(vc.empty()||fc.empty()) return false;
        VkShaderModule vm=createShaderModule(ctx.device,vc), fm=createShaderModule(ctx.device,fc);
        if(!vm||!fm){if(vm)vkDestroyShaderModule(ctx.device,vm,nullptr);if(fm)vkDestroyShaderModule(ctx.device,fm,nullptr);return false;}
        VkVertexInputBindingDescription bind{0,48,VK_VERTEX_INPUT_RATE_VERTEX};
        VkVertexInputAttributeDescription attrs[4]={
            {0,0,VK_FORMAT_R32G32B32_SFLOAT,0},
            {1,0,VK_FORMAT_R32G32B32_SFLOAT,12},
            {2,0,VK_FORMAT_R32G32_SFLOAT,24},
            {3,0,VK_FORMAT_R32G32B32A32_SFLOAT,32}};
        VkDescriptorSetLayout sets[2]={desc.set0Layout,desc.set1Layout};
        PipelineInitParams p{}; p.bindings=&bind; p.bindingCount=1; p.attrs=attrs; p.attrCount=4;
        p.depthTest=VK_TRUE; p.depthWrite=VK_TRUE; p.depthOp=VK_COMPARE_OP_LESS;
        p.cullMode=VK_CULL_MODE_NONE;
        p.pcSize=sizeof(MeshPushConstants); p.setLayouts=sets; p.setCount=2;
        p.colorFmt=colorFmt; p.depthFmt=depthFmt; p.name="SVO";
        bool ok=buildPipeline(ctx.device,vm,fm,p,layout,pipeline);
        vkDestroyShaderModule(ctx.device,vm,nullptr); vkDestroyShaderModule(ctx.device,fm,nullptr); return ok;
    }
    void bind(VkCommandBuffer cmd) const{vkCmdBindPipeline(cmd,VK_PIPELINE_BIND_POINT_GRAPHICS,pipeline);}
    void shutdown(VkDevice d){if(pipeline){vkDestroyPipeline(d,pipeline,nullptr);pipeline=VK_NULL_HANDLE;}if(layout){vkDestroyPipelineLayout(d,layout,nullptr);layout=VK_NULL_HANDLE;}}
};

// -----------------------------------------------------------------------
// VkAtmoPipeline — 大气散射球体（alpha 混合，深度测试不写入）
// Vertex: vec3 aPos (stride=12). Set 0 only. Push: AtmoPushConstants (64 bytes).
// -----------------------------------------------------------------------
struct VkAtmoPipeline {
    VkPipelineLayout layout=VK_NULL_HANDLE; VkPipeline pipeline=VK_NULL_HANDLE;
    bool init(VulkanContext& ctx, VkDescriptorManager& desc,
              VkFormat colorFmt, VkFormat depthFmt, const char* vertSpv, const char* fragSpv) {
        auto vc=loadSPIRV(vertSpv); auto fc=loadSPIRV(fragSpv);
        if(vc.empty()||fc.empty()) return false;
        VkShaderModule vm=createShaderModule(ctx.device,vc), fm=createShaderModule(ctx.device,fc);
        if(!vm||!fm){if(vm)vkDestroyShaderModule(ctx.device,vm,nullptr);if(fm)vkDestroyShaderModule(ctx.device,fm,nullptr);return false;}
        VkVertexInputBindingDescription bind{0,12,VK_VERTEX_INPUT_RATE_VERTEX};
        VkVertexInputAttributeDescription attr{0,0,VK_FORMAT_R32G32B32_SFLOAT,0};
        PipelineInitParams p{}; p.bindings=&bind; p.bindingCount=1; p.attrs=&attr; p.attrCount=1;
        p.blendEnable=true;
        // Premultiplied alpha: matches OpenGL glBlendFunc(GL_ONE, GL_ONE_MINUS_SRC_ALPHA)
        // Result = scatter_color * 1 + background * (1 - opacity)  [physically correct]
        p.srcColor=VK_BLEND_FACTOR_ONE;  p.dstColor=VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA;
        p.srcAlpha=VK_BLEND_FACTOR_ONE;  p.dstAlpha=VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA;
        // No depth test: matches OpenGL glDisable(GL_DEPTH_TEST).
        // Shader handles ray termination at surface via intersectSphere internally.
        p.depthTest=VK_FALSE; p.depthWrite=VK_FALSE; p.depthOp=VK_COMPARE_OP_LESS_OR_EQUAL;
        // Hemisphere selection is done in the fragment shader via tFrag test.
        // Hardware culling is disabled: Y-flip + VK_FRONT_FACE_CLOCKWISE inverts the
        // effective cull sense relative to OpenGL, making BACK_BIT keep the wrong hemisphere.
        // The fragment shader discards near-hemisphere fragments (tFrag < midpoint).
        p.cullMode=VK_CULL_MODE_NONE;
        p.pcSize=sizeof(AtmoPushConstants); p.setLayouts=&desc.set0Layout; p.setCount=1;
        p.colorFmt=colorFmt; p.depthFmt=depthFmt; p.name="Atmosphere";
        bool ok=buildPipeline(ctx.device,vm,fm,p,layout,pipeline);
        vkDestroyShaderModule(ctx.device,vm,nullptr); vkDestroyShaderModule(ctx.device,fm,nullptr); return ok;
    }
    void bind(VkCommandBuffer cmd) const{vkCmdBindPipeline(cmd,VK_PIPELINE_BIND_POINT_GRAPHICS,pipeline);}
    void shutdown(VkDevice d){if(pipeline){vkDestroyPipeline(d,pipeline,nullptr);pipeline=VK_NULL_HANDLE;}if(layout){vkDestroyPipelineLayout(d,layout,nullptr);layout=VK_NULL_HANDLE;}}
};

// -----------------------------------------------------------------------
// VkVegetationPipeline — 植被实例化渲染
// Binding 0 (per-vertex,   stride=24): vec3 pos(loc0) + vec3 normal(loc1)
// Binding 1 (per-instance, stride=20): vec3 iPos(loc2) + float iScale(loc3) + float iRot(loc4)
// Set 0 only. No push constants.
// -----------------------------------------------------------------------
struct VkVegetationPipeline {
    VkPipelineLayout layout=VK_NULL_HANDLE; VkPipeline pipeline=VK_NULL_HANDLE;
    bool init(VulkanContext& ctx, VkDescriptorManager& desc,
              VkFormat colorFmt, VkFormat depthFmt, const char* vertSpv, const char* fragSpv) {
        auto vc=loadSPIRV(vertSpv); auto fc=loadSPIRV(fragSpv);
        if(vc.empty()||fc.empty()) return false;
        VkShaderModule vm=createShaderModule(ctx.device,vc), fm=createShaderModule(ctx.device,fc);
        if(!vm||!fm){if(vm)vkDestroyShaderModule(ctx.device,vm,nullptr);if(fm)vkDestroyShaderModule(ctx.device,fm,nullptr);return false;}
        VkVertexInputBindingDescription binds[2]={
            {0,24,VK_VERTEX_INPUT_RATE_VERTEX},
            {1,20,VK_VERTEX_INPUT_RATE_INSTANCE}};
        VkVertexInputAttributeDescription attrs[5]={
            {0,0,VK_FORMAT_R32G32B32_SFLOAT,0},
            {1,0,VK_FORMAT_R32G32B32_SFLOAT,12},
            {2,1,VK_FORMAT_R32G32B32_SFLOAT,0},
            {3,1,VK_FORMAT_R32_SFLOAT,12},
            {4,1,VK_FORMAT_R32_SFLOAT,16}};
        PipelineInitParams p{}; p.bindings=binds; p.bindingCount=2; p.attrs=attrs; p.attrCount=5;
        p.depthTest=VK_TRUE; p.depthWrite=VK_TRUE; p.depthOp=VK_COMPARE_OP_LESS;
        p.cullMode=VK_CULL_MODE_BACK_BIT;
        p.pcSize=0; p.setLayouts=&desc.set0Layout; p.setCount=1;
        p.colorFmt=colorFmt; p.depthFmt=depthFmt; p.name="Vegetation";
        bool ok=buildPipeline(ctx.device,vm,fm,p,layout,pipeline);
        vkDestroyShaderModule(ctx.device,vm,nullptr); vkDestroyShaderModule(ctx.device,fm,nullptr); return ok;
    }
    void bind(VkCommandBuffer cmd) const{vkCmdBindPipeline(cmd,VK_PIPELINE_BIND_POINT_GRAPHICS,pipeline);}
    void shutdown(VkDevice d){if(pipeline){vkDestroyPipeline(d,pipeline,nullptr);pipeline=VK_NULL_HANDLE;}if(layout){vkDestroyPipelineLayout(d,layout,nullptr);layout=VK_NULL_HANDLE;}}
};

// ==========================================================================
// VkTerrainPipeline — Quadtree 地形渲染
// Set 0: FrameUBO(b0) + TerrainData UBO(b1)
// Set 1: 4 textures (tectonic/hydro/climate/localHydro)
// Push: TerrainPushConstants (80 bytes)
// ==========================================================================
struct TerrainPushConstants {
    float model[16]; float planetRadius; float maxElevation; int nodeLevel; int hasLocalHydro;
};
struct VkTerrainPipeline {
    VkPipelineLayout layout=VK_NULL_HANDLE; VkPipeline pipeline=VK_NULL_HANDLE;
    VkDescriptorSetLayout set0Layout=VK_NULL_HANDLE, set1Layout=VK_NULL_HANDLE;
    bool init(VulkanContext& ctx, VkFormat cf, VkFormat df, const char* vs, const char* fs) {
        auto vc=loadSPIRV(vs), fc=loadSPIRV(fs);
        if(vc.empty()||fc.empty()) return false;
        VkShaderModule vm=createShaderModule(ctx.device,vc), fm=createShaderModule(ctx.device,fc);
        if(!vm||!fm){if(vm)vkDestroyShaderModule(ctx.device,vm,nullptr);if(fm)vkDestroyShaderModule(ctx.device,fm,nullptr);return false;}
        auto destroySMs=[&]{vkDestroyShaderModule(ctx.device,vm,nullptr);vkDestroyShaderModule(ctx.device,fm,nullptr);};

        // lambda 封装管线创建，避免 goto 跳过初始化引起的 C2362
        bool ok = [&]() -> bool {
            VkDescriptorSetLayoutBinding b0[2]={
                {0,VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER,1,VK_SHADER_STAGE_VERTEX_BIT|VK_SHADER_STAGE_FRAGMENT_BIT,nullptr},
                {1,VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER,1,VK_SHADER_STAGE_VERTEX_BIT|VK_SHADER_STAGE_FRAGMENT_BIT,nullptr}};
            VkDescriptorSetLayoutCreateInfo s0{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO,nullptr,0,2,b0};
            if(vkCreateDescriptorSetLayout(ctx.device,&s0,nullptr,&set0Layout)!=VK_SUCCESS) return false;
            VkDescriptorSetLayoutBinding b1[4];
            for(int i=0;i<4;i++) b1[i]={(uint32_t)i,VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,1,VK_SHADER_STAGE_VERTEX_BIT|VK_SHADER_STAGE_FRAGMENT_BIT,nullptr};
            VkDescriptorSetLayoutCreateInfo s1{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO,nullptr,0,4,b1};
            if(vkCreateDescriptorSetLayout(ctx.device,&s1,nullptr,&set1Layout)!=VK_SUCCESS) return false;
            VkPushConstantRange pc{VK_SHADER_STAGE_VERTEX_BIT|VK_SHADER_STAGE_FRAGMENT_BIT,0,sizeof(TerrainPushConstants)};
            VkDescriptorSetLayout sl[]={set0Layout,set1Layout};
            VkPipelineLayoutCreateInfo pl{VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO,nullptr,0,2,sl,1,&pc};
            if(vkCreatePipelineLayout(ctx.device,&pl,nullptr,&layout)!=VK_SUCCESS) return false;
            VkPipelineShaderStageCreateInfo ss[2]={
                {VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO,nullptr,0,VK_SHADER_STAGE_VERTEX_BIT,  vm,"main",nullptr},
                {VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO,nullptr,0,VK_SHADER_STAGE_FRAGMENT_BIT,fm,"main",nullptr}};
            VkVertexInputBindingDescription vb{0,48,VK_VERTEX_INPUT_RATE_VERTEX};
            VkVertexInputAttributeDescription va[4]={{0,0,VK_FORMAT_R32G32B32_SFLOAT,0},{1,0,VK_FORMAT_R32G32B32_SFLOAT,12},{2,0,VK_FORMAT_R32G32_SFLOAT,24},{3,0,VK_FORMAT_R32G32B32A32_SFLOAT,32}};
            VkPipelineVertexInputStateCreateInfo vi{VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO,nullptr,0,1,&vb,4,va};
            VkPipelineInputAssemblyStateCreateInfo ia{VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO,nullptr,0,VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST,VK_FALSE};
            VkPipelineViewportStateCreateInfo vp{VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO,nullptr,0,1,nullptr,1,nullptr};
            VkPipelineRasterizationStateCreateInfo rs{VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO,nullptr,0,VK_FALSE,VK_FALSE,VK_POLYGON_MODE_FILL,VK_CULL_MODE_NONE,VK_FRONT_FACE_COUNTER_CLOCKWISE,VK_FALSE,0,0,0,1.f};
            VkPipelineMultisampleStateCreateInfo ms{VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO,nullptr,0,VK_SAMPLE_COUNT_1_BIT,VK_FALSE,0.f,nullptr,VK_FALSE,VK_FALSE};
            VkPipelineDepthStencilStateCreateInfo ds{VK_STRUCTURE_TYPE_PIPELINE_DEPTH_STENCIL_STATE_CREATE_INFO,nullptr,0,VK_TRUE,VK_TRUE,VK_COMPARE_OP_LESS,VK_FALSE,VK_FALSE,{},{},0.f,0.f};
            VkPipelineColorBlendAttachmentState ba{VK_FALSE,VK_BLEND_FACTOR_ZERO,VK_BLEND_FACTOR_ZERO,VK_BLEND_OP_ADD,VK_BLEND_FACTOR_ZERO,VK_BLEND_FACTOR_ZERO,VK_BLEND_OP_ADD,0xF};
            VkPipelineColorBlendStateCreateInfo bs{VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO,nullptr,0,VK_FALSE,VK_LOGIC_OP_COPY,1,&ba,{}};
            VkDynamicState dyn[]={VK_DYNAMIC_STATE_VIEWPORT,VK_DYNAMIC_STATE_SCISSOR};
            VkPipelineDynamicStateCreateInfo dsi{VK_STRUCTURE_TYPE_PIPELINE_DYNAMIC_STATE_CREATE_INFO,nullptr,0,2,dyn};
            VkPipelineRenderingCreateInfo rci{VK_STRUCTURE_TYPE_PIPELINE_RENDERING_CREATE_INFO,nullptr,0,1,&cf,df,VK_FORMAT_UNDEFINED};
            VkGraphicsPipelineCreateInfo pci{VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO,&rci,0,2,ss,&vi,&ia,nullptr,&vp,&rs,&ms,&ds,&bs,&dsi,layout,VK_NULL_HANDLE,0,VK_NULL_HANDLE,-1};
            VkResult r=vkCreateGraphicsPipelines(ctx.device,VK_NULL_HANDLE,1,&pci,nullptr,&pipeline);
            if(r!=VK_SUCCESS){fprintf(stderr,"[Terrain] pipeline failed\n");return false;}
            return true;
        }();
        destroySMs();
        if(!ok) return false;
        printf("[Terrain] Pipeline created\n"); return true;
    }
    void bind(VkCommandBuffer cmd) const{vkCmdBindPipeline(cmd,VK_PIPELINE_BIND_POINT_GRAPHICS,pipeline);}
    void shutdown(VkDevice d){
        if(pipeline){vkDestroyPipeline(d,pipeline,nullptr);pipeline=VK_NULL_HANDLE;}
        if(layout){vkDestroyPipelineLayout(d,layout,nullptr);layout=VK_NULL_HANDLE;}
        if(set0Layout){vkDestroyDescriptorSetLayout(d,set0Layout,nullptr);set0Layout=VK_NULL_HANDLE;}
        if(set1Layout){vkDestroyDescriptorSetLayout(d,set1Layout,nullptr);set1Layout=VK_NULL_HANDLE;}
    }
};
