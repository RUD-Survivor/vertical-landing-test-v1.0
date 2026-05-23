#pragma once
// ==========================================================================
// vk_hud.h — 2D HUD 覆盖层（纯色矩形 + 纹理 quad）
// Swapchain 之上叠加，alpha 混合，无深度
// ==========================================================================
#include "../vk_context.h"
#include "../vk_pipeline.h"
#include "../vk_texture.h"
#include <vector>
#include <cstring>

struct HudVertex { float x,y,u,v; };

struct VkHUDSystem {
    static constexpr uint32_t MAX_VERTS = 4096;
    VkDescriptorSetLayout setLayout=VK_NULL_HANDLE;
    VkPipelineLayout layout=VK_NULL_HANDLE;
    VkPipeline pipeline=VK_NULL_HANDLE;
    VkBuffer vbo=VK_NULL_HANDLE; VmaAllocation vboAlloc=VK_NULL_HANDLE;
    void* vboMapped=nullptr;
    VkTexture2D nullTex;
    VkDescriptorPool descPool=VK_NULL_HANDLE;
    VkDescriptorSet descSet=VK_NULL_HANDLE;
    std::vector<HudVertex> batch{};

    bool init(VulkanContext& ctx, VkFormat swapFmt) {
        auto vc=loadSPIRV("src/render/shaders/spirv/hud2d.vert.spv");
        auto fc=loadSPIRV("src/render/shaders/spirv/hud2d.frag.spv");
        if(vc.empty()||fc.empty())return false;
        VkShaderModule vm=createShaderModule(ctx.device,vc), fm=createShaderModule(ctx.device,fc);
        if(!vm||!fm){if(vm)vkDestroyShaderModule(ctx.device,vm,nullptr);if(fm)vkDestroyShaderModule(ctx.device,fm,nullptr);return false;}
        VkDescriptorSetLayoutBinding b{VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,1,VK_SHADER_STAGE_FRAGMENT_BIT,nullptr};
        VkDescriptorSetLayoutCreateInfo sl{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO,nullptr,0,1,&b};
        if(vkCreateDescriptorSetLayout(ctx.device,&sl,nullptr,&setLayout)!=VK_SUCCESS)goto fail;
        VkPushConstantRange pc{VK_SHADER_STAGE_VERTEX_BIT,0,9*4};
        VkPipelineLayoutCreateInfo pl{VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO,nullptr,0,1,&setLayout,1,&pc};
        if(vkCreatePipelineLayout(ctx.device,&pl,nullptr,&layout)!=VK_SUCCESS)goto fail;
        VkPipelineShaderStageCreateInfo ss[2]{{VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO,nullptr,0,VK_SHADER_STAGE_VERTEX_BIT,vm,"main",nullptr},{VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO,nullptr,0,VK_SHADER_STAGE_FRAGMENT_BIT,fm,"main",nullptr}};
        VkVertexInputBindingDescription vb{0,sizeof(HudVertex),VK_VERTEX_INPUT_RATE_VERTEX};
        VkVertexInputAttributeDescription va[2]{{0,0,VK_FORMAT_R32G32_SFLOAT,0},{1,0,VK_FORMAT_R32G32_SFLOAT,8}};
        VkPipelineVertexInputStateCreateInfo vi{VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO,nullptr,0,1,&vb,2,va};
        VkPipelineInputAssemblyStateCreateInfo ia{VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO,nullptr,0,VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST,VK_FALSE};
        VkPipelineViewportStateCreateInfo vp{VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO,nullptr,0,1,nullptr,1,nullptr};
        VkPipelineRasterizationStateCreateInfo rs{VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO,nullptr,0,VK_FALSE,VK_FALSE,VK_POLYGON_MODE_FILL,VK_CULL_MODE_NONE,VK_FRONT_FACE_COUNTER_CLOCKWISE,VK_FALSE,0,0,0,1.f};
        VkPipelineMultisampleStateCreateInfo ms{VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO,nullptr,0,VK_SAMPLE_COUNT_1_BIT,VK_FALSE,0,nullptr,nullptr,nullptr};
        VkPipelineColorBlendAttachmentState ba{VK_TRUE,VK_BLEND_FACTOR_SRC_ALPHA,VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA,VK_BLEND_OP_ADD,VK_BLEND_FACTOR_ONE,VK_BLEND_FACTOR_ZERO,VK_BLEND_OP_ADD,0xF};
        VkPipelineColorBlendStateCreateInfo bs{VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO,nullptr,0,VK_FALSE,VK_LOGIC_OP_COPY,1,&ba,{}};
        VkDynamicState dyn[]={VK_DYNAMIC_STATE_VIEWPORT,VK_DYNAMIC_STATE_SCISSOR};
        VkPipelineDynamicStateCreateInfo ds{VK_STRUCTURE_TYPE_PIPELINE_DYNAMIC_STATE_CREATE_INFO,nullptr,0,2,dyn};
        VkPipelineRenderingCreateInfo rci{VK_STRUCTURE_TYPE_PIPELINE_RENDERING_CREATE_INFO,nullptr,0,1,&swapFmt,VK_FORMAT_UNDEFINED,VK_FORMAT_UNDEFINED};
        VkGraphicsPipelineCreateInfo pci{VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO,&rci,0,2,ss,&vi,&ia,nullptr,&vp,&rs,&ms,nullptr,&bs,&ds,layout,VK_NULL_HANDLE,0,VK_NULL_HANDLE,-1};
        if(vkCreateGraphicsPipelines(ctx.device,VK_NULL_HANDLE,1,&pci,nullptr,&pipeline)!=VK_SUCCESS)goto fail;
        vkDestroyShaderModule(ctx.device,vm,nullptr);vkDestroyShaderModule(ctx.device,fm,nullptr);
        VkBufferCreateInfo bci{VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO};bci.size=MAX_VERTS*sizeof(HudVertex);bci.usage=VK_BUFFER_USAGE_VERTEX_BUFFER_BIT;
        VmaAllocationCreateInfo aci{};aci.usage=VMA_MEMORY_USAGE_CPU_TO_GPU;aci.flags=VMA_ALLOCATION_CREATE_MAPPED_BIT;VmaAllocationInfo ai;
        if(vmaCreateBuffer(ctx.allocator,&bci,&aci,&vbo,&vboAlloc,&ai)!=VK_SUCCESS)return false;
        vboMapped=ai.pMappedData;batch.reserve(MAX_VERTS);
        static uint8_t w[4]={255,255,255,255};nullTex.upload(ctx,w,1,1);
        VkDescriptorPoolSize ps{VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,1};
        VkDescriptorPoolCreateInfo dpi{VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO,nullptr,0,1,1,&ps};
        if(vkCreateDescriptorPool(ctx.device,&dpi,nullptr,&descPool)!=VK_SUCCESS)return false;
        VkDescriptorSetAllocateInfo ai2{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO,descPool,1,&setLayout};
        if(vkAllocateDescriptorSets(ctx.device,&ai2,&descSet)!=VK_SUCCESS)return false;
        VkDescriptorImageInfo ii{nullTex.sampler,nullTex.view,VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL};
        VkWriteDescriptorSet w2{VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET,nullptr,descSet,0,0,1,VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,&ii,nullptr,nullptr};
        vkUpdateDescriptorSets(ctx.device,1,&w2,0,nullptr);
        printf("[HUD] Initialized\n");return true;
    fail:vkDestroyShaderModule(ctx.device,vm,nullptr);vkDestroyShaderModule(ctx.device,fm,nullptr);return false;
    }

    void shutdown(VulkanContext& ctx){
        if(pipeline)vkDestroyPipeline(ctx.device,pipeline,nullptr);
        if(layout)vkDestroyPipelineLayout(ctx.device,layout,nullptr);
        if(setLayout)vkDestroyDescriptorSetLayout(ctx.device,setLayout,nullptr);
        if(vbo)vmaDestroyBuffer(ctx.allocator,vbo,vboAlloc);
        if(descPool)vkDestroyDescriptorPool(ctx.device,descPool,nullptr);
        nullTex.destroy(ctx);
    }

    void pushQuad(float x, float y, float w, float h, float r=1,float g=1,float b=1,float a=1){
        if(batch.size()+6>MAX_VERTS) return;
        HudVertex v[6]={{x,y,0,0},{x+w,y,1,0},{x+w,y+h,1,1},{x,y,0,0},{x+w,y+h,1,1},{x,y+h,0,1}};
        for(int i=0;i<6;i++)batch.push_back(v[i]);
    }

    void flush(VkCommandBuffer cmd){
        if(batch.empty())return;
        memcpy(vboMapped,batch.data(),batch.size()*sizeof(HudVertex));
        VkDeviceSize off=0; vkCmdBindVertexBuffers(cmd,0,1,&vbo,&off);
        vkCmdBindPipeline(cmd,VK_PIPELINE_BIND_POINT_GRAPHICS,pipeline);
        vkCmdBindDescriptorSets(cmd,VK_PIPELINE_BIND_POINT_GRAPHICS,layout,0,1,&descSet,0,nullptr);
        float pc[9]={0,0,2,2, 1,1,1,1, 0};
        vkCmdPushConstants(cmd,layout,VK_SHADER_STAGE_VERTEX_BIT,0,36,pc);
        vkCmdDraw(cmd,(uint32_t)batch.size(),1,0,0);
        batch.clear();
    }
};
