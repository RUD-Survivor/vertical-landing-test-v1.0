#pragma once
// ==========================================================================
// vk_cloud_bake.h — GPU compute bake for cloud 3D noise textures
//
// Runs once at init:
//   cloud_basicnoise.comp  → 128³ R8  (Perlin-Worley composite)
//   cloud_detailnoise.comp → 32³  R8  (Worley detail FBM)
// ==========================================================================

#include "vk_context.h"
#include "vk_mesh.h"
#include "vk_pipeline.h"
#include "vk_texture.h"
#include "vk_utils.h"

struct VkCloudBake {
    VkDescriptorSetLayout setLayout       = VK_NULL_HANDLE;
    VkPipelineLayout      pipelineLayout  = VK_NULL_HANDLE;
    VkPipeline            basicPipeline   = VK_NULL_HANDLE;
    VkPipeline            detailPipeline  = VK_NULL_HANDLE;
    VkDescriptorPool      pool            = VK_NULL_HANDLE;
    VkDescriptorSet       bakeSet         = VK_NULL_HANDLE;

    bool init(VulkanContext& ctx) {
        VkDescriptorSetLayoutBinding b{};
        b.binding         = 0;
        b.descriptorType  = VK_DESCRIPTOR_TYPE_STORAGE_IMAGE;
        b.descriptorCount = 1;
        b.stageFlags      = VK_SHADER_STAGE_COMPUTE_BIT;

        VkDescriptorSetLayoutCreateInfo lci{ VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO };
        lci.bindingCount = 1;
        lci.pBindings    = &b;
        if (vkCreateDescriptorSetLayout(ctx.device, &lci, nullptr, &setLayout) != VK_SUCCESS) {
            fprintf(stderr, "[VkCloudBake] Failed to create set layout\n");
            return false;
        }

        VkPipelineLayoutCreateInfo plci{ VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO };
        plci.setLayoutCount = 1;
        plci.pSetLayouts    = &setLayout;
        if (vkCreatePipelineLayout(ctx.device, &plci, nullptr, &pipelineLayout) != VK_SUCCESS) {
            fprintf(stderr, "[VkCloudBake] Failed to create pipeline layout\n");
            return false;
        }

        VkDescriptorPoolSize poolSize{};
        poolSize.type            = VK_DESCRIPTOR_TYPE_STORAGE_IMAGE;
        poolSize.descriptorCount = 1;
        VkDescriptorPoolCreateInfo pci{ VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO };
        pci.maxSets       = 1;
        pci.poolSizeCount = 1;
        pci.pPoolSizes    = &poolSize;
        if (vkCreateDescriptorPool(ctx.device, &pci, nullptr, &pool) != VK_SUCCESS) {
            fprintf(stderr, "[VkCloudBake] Failed to create descriptor pool\n");
            return false;
        }

        VkDescriptorSetAllocateInfo ai{ VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO };
        ai.descriptorPool     = pool;
        ai.descriptorSetCount = 1;
        ai.pSetLayouts        = &setLayout;
        if (vkAllocateDescriptorSets(ctx.device, &ai, &bakeSet) != VK_SUCCESS) {
            fprintf(stderr, "[VkCloudBake] Failed to allocate descriptor set\n");
            return false;
        }

        if (!createComputePipeline(ctx, "src/render/shaders/spirv/cloud_basicnoise.comp.spv", basicPipeline) ||
            !createComputePipeline(ctx, "src/render/shaders/spirv/cloud_detailnoise.comp.spv", detailPipeline)) {
            return false;
        }

        printf("[VkCloudBake] Compute bake pipelines ready\n");
        return true;
    }

    bool bakeBasicNoise(VulkanContext& ctx, VkTexture3D& tex) {
        return dispatchBake(ctx, tex, basicPipeline, 128, 128, 128);
    }

    bool bakeDetailNoise(VulkanContext& ctx, VkTexture3D& tex) {
        return dispatchBake(ctx, tex, detailPipeline, 32, 32, 32);
    }

    void shutdown(VkDevice device) {
        if (basicPipeline  != VK_NULL_HANDLE) { vkDestroyPipeline(device, basicPipeline,  nullptr); basicPipeline  = VK_NULL_HANDLE; }
        if (detailPipeline != VK_NULL_HANDLE) { vkDestroyPipeline(device, detailPipeline, nullptr); detailPipeline = VK_NULL_HANDLE; }
        if (pipelineLayout != VK_NULL_HANDLE) { vkDestroyPipelineLayout(device, pipelineLayout, nullptr); pipelineLayout = VK_NULL_HANDLE; }
        if (pool           != VK_NULL_HANDLE) { vkDestroyDescriptorPool(device, pool, nullptr); pool = VK_NULL_HANDLE; }
        if (setLayout      != VK_NULL_HANDLE) { vkDestroyDescriptorSetLayout(device, setLayout, nullptr); setLayout = VK_NULL_HANDLE; }
        bakeSet = VK_NULL_HANDLE;
    }

private:
    bool createComputePipeline(VulkanContext& ctx, const char* spvPath, VkPipeline& outPipeline) {
        auto code = loadSPIRV(spvPath);
        if (code.empty()) return false;

        VkShaderModule mod = createShaderModule(ctx.device, code);
        if (mod == VK_NULL_HANDLE) return false;

        VkPipelineShaderStageCreateInfo stage{ VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO };
        stage.stage  = VK_SHADER_STAGE_COMPUTE_BIT;
        stage.module = mod;
        stage.pName  = "main";

        VkComputePipelineCreateInfo cpci{ VK_STRUCTURE_TYPE_COMPUTE_PIPELINE_CREATE_INFO };
        cpci.stage  = stage;
        cpci.layout = pipelineLayout;

        VkResult res = vkCreateComputePipelines(ctx.device, VK_NULL_HANDLE, 1, &cpci, nullptr, &outPipeline);
        vkDestroyShaderModule(ctx.device, mod, nullptr);

        if (res != VK_SUCCESS) {
            fprintf(stderr, "[VkCloudBake] Failed compute pipeline: %s\n", spvPath);
            return false;
        }
        return true;
    }

    void updateStorageBinding(VulkanContext& ctx, VkTexture3D& tex) {
        VkDescriptorImageInfo imgInfo{};
        imgInfo.imageView   = tex.view;
        imgInfo.imageLayout = VK_IMAGE_LAYOUT_GENERAL;

        VkWriteDescriptorSet w{ VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET };
        w.dstSet          = bakeSet;
        w.dstBinding      = 0;
        w.descriptorType  = VK_DESCRIPTOR_TYPE_STORAGE_IMAGE;
        w.descriptorCount = 1;
        w.pImageInfo      = &imgInfo;
        vkUpdateDescriptorSets(ctx.device, 1, &w, 0, nullptr);
    }

    bool dispatchBake(VulkanContext& ctx, VkTexture3D& tex, VkPipeline pipeline,
                      uint32_t w, uint32_t h, uint32_t d) {
        if (pipeline == VK_NULL_HANDLE || tex.image == VK_NULL_HANDLE) return false;

        updateStorageBinding(ctx, tex);

        const uint32_t gx = (w + 7u) / 8u;
        const uint32_t gy = (h + 7u) / 8u;
        const uint32_t gz = d;

        VkCommandBuffer cmd = beginSingleTimeCommands(ctx);

        transitionImage(cmd, tex.image,
            VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_GENERAL,
            VK_PIPELINE_STAGE_2_TOP_OF_PIPE_BIT, VK_ACCESS_2_NONE,
            VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT, VK_ACCESS_2_SHADER_WRITE_BIT);

        vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_COMPUTE, pipeline);
        vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_COMPUTE,
            pipelineLayout, 0, 1, &bakeSet, 0, nullptr);
        vkCmdDispatch(cmd, gx, gy, gz);

        transitionImage(cmd, tex.image,
            VK_IMAGE_LAYOUT_GENERAL, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
            VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT, VK_ACCESS_2_SHADER_WRITE_BIT,
            VK_PIPELINE_STAGE_2_FRAGMENT_SHADER_BIT, VK_ACCESS_2_SHADER_READ_BIT);

        endSingleTimeCommands(ctx, cmd);
        return true;
    }
};
