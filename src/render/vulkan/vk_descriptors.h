#pragma once
// ==========================================================================
// vk_descriptors.h — DescriptorSetLayout / Pool / FrameUBO 管理
//
// 三层设计：
//   Set 0 (per-frame)  : FrameUBO — view/proj/lighting，每帧 memcpy 更新
//   Set 1 (per-material): sampler2D — 主纹理，每材质一个 DescriptorSet
//   Push Constants      : per-object mat4 model + baseColor + params
// ==========================================================================

#include "vk_context.h"
#include "vk_frame.h"
#include <cstring>

// -----------------------------------------------------------------------
// FrameUBO — GLSL std140 layout (matches mesh.vert / mesh.frag Set 0 Binding 0)
// -----------------------------------------------------------------------
struct FrameUBO {
    float view[16];     // 64 bytes
    float proj[16];     // 64 bytes
    float lightDir[3];  // 12 bytes
    float _pad0;        //  4 bytes (vec3 std140 padding)
    float viewPos[3];   // 12 bytes
    float time;         //  4 bytes
};                      // total: 160 bytes

// -----------------------------------------------------------------------
// MeshPushConstants — must match push_constant block in mesh.vert / mesh.frag
// std430: mat4(64) + vec4(16) + float(4) + int(4) = 88 bytes
// -----------------------------------------------------------------------
struct MeshPushConstants {
    float model[16];    // 64 bytes — world transform
    float baseColor[4]; // 16 bytes — tint color
    float ambientStr;   //  4 bytes
    int   hasTexture;   //  4 bytes — 0 = no texture, 1 = sample uSampler
};

// -----------------------------------------------------------------------
// VkDescriptorManager — owns layouts, pool, and per-frame UBO resources
// -----------------------------------------------------------------------
struct VkDescriptorManager {
    VkDescriptorSetLayout set0Layout = VK_NULL_HANDLE;  // FrameUBO
    VkDescriptorSetLayout set1Layout = VK_NULL_HANDLE;  // sampler2D

    VkDescriptorPool pool = VK_NULL_HANDLE;

    // Per-frame UBO buffers (persistently mapped — no map/unmap per frame)
    VkBuffer      uboBuffers[FRAMES_IN_FLIGHT]{};
    VmaAllocation uboAllocs [FRAMES_IN_FLIGHT]{};
    void*         uboMapped [FRAMES_IN_FLIGHT]{};

    // Pre-allocated Set 0 descriptor sets (one per frame in flight)
    VkDescriptorSet set0[FRAMES_IN_FLIGHT]{};

    bool init(VulkanContext& ctx) {
        if (!createLayouts(ctx))    return false;
        if (!createPool(ctx))       return false;
        if (!createUBOBuffers(ctx)) return false;
        if (!createSet0(ctx))       return false;
        return true;
    }

    void shutdown(VulkanContext& ctx) {
        for (int i = 0; i < FRAMES_IN_FLIGHT; i++) {
            if (uboBuffers[i] != VK_NULL_HANDLE) {
                vmaDestroyBuffer(ctx.allocator, uboBuffers[i], uboAllocs[i]);
                uboBuffers[i] = VK_NULL_HANDLE;
            }
        }
        if (pool       != VK_NULL_HANDLE) { vkDestroyDescriptorPool      (ctx.device, pool,       nullptr); pool       = VK_NULL_HANDLE; }
        if (set0Layout != VK_NULL_HANDLE) { vkDestroyDescriptorSetLayout (ctx.device, set0Layout, nullptr); set0Layout = VK_NULL_HANDLE; }
        if (set1Layout != VK_NULL_HANDLE) { vkDestroyDescriptorSetLayout (ctx.device, set1Layout, nullptr); set1Layout = VK_NULL_HANDLE; }
    }

    // Call once per frame before recording draw commands
    void updateFrameUBO(int frameIndex, const FrameUBO& data) {
        memcpy(uboMapped[frameIndex], &data, sizeof(FrameUBO));
    }

    // Allocate a Set 1 descriptor set for one texture (caller keeps the VkDescriptorSet)
    VkDescriptorSet allocateTextureSet(VkDevice device, VkImageView view, VkSampler sampler) {
        VkDescriptorSetAllocateInfo ai{ VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO };
        ai.descriptorPool     = pool;
        ai.descriptorSetCount = 1;
        ai.pSetLayouts        = &set1Layout;

        VkDescriptorSet ds = VK_NULL_HANDLE;
        if (vkAllocateDescriptorSets(device, &ai, &ds) != VK_SUCCESS) {
            fprintf(stderr, "[VkDesc] Failed to allocate texture descriptor set\n");
            return VK_NULL_HANDLE;
        }

        VkDescriptorImageInfo imgInfo{};
        imgInfo.imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
        imgInfo.imageView   = view;
        imgInfo.sampler     = sampler;

        VkWriteDescriptorSet write{ VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET };
        write.dstSet          = ds;
        write.dstBinding      = 0;
        write.descriptorType  = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
        write.descriptorCount = 1;
        write.pImageInfo      = &imgInfo;
        vkUpdateDescriptorSets(device, 1, &write, 0, nullptr);
        return ds;
    }

private:
    bool createLayouts(VulkanContext& ctx) {
        // Set 0: uniform buffer, vertex + fragment stages
        VkDescriptorSetLayoutBinding b0{};
        b0.binding         = 0;
        b0.descriptorType  = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
        b0.descriptorCount = 1;
        b0.stageFlags      = VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT;

        VkDescriptorSetLayoutCreateInfo ci0{ VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO };
        ci0.bindingCount = 1;
        ci0.pBindings    = &b0;
        if (vkCreateDescriptorSetLayout(ctx.device, &ci0, nullptr, &set0Layout) != VK_SUCCESS) {
            fprintf(stderr, "[VkDesc] Failed to create Set 0 layout\n");
            return false;
        }

        // Set 1: combined image sampler, fragment stage only
        VkDescriptorSetLayoutBinding b1{};
        b1.binding         = 0;
        b1.descriptorType  = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
        b1.descriptorCount = 1;
        b1.stageFlags      = VK_SHADER_STAGE_FRAGMENT_BIT;

        VkDescriptorSetLayoutCreateInfo ci1{ VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO };
        ci1.bindingCount = 1;
        ci1.pBindings    = &b1;
        if (vkCreateDescriptorSetLayout(ctx.device, &ci1, nullptr, &set1Layout) != VK_SUCCESS) {
            fprintf(stderr, "[VkDesc] Failed to create Set 1 layout\n");
            return false;
        }
        return true;
    }

    bool createPool(VulkanContext& ctx) {
        VkDescriptorPoolSize sizes[2] = {
            { VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER,         FRAMES_IN_FLIGHT },
            { VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 64 },
        };
        VkDescriptorPoolCreateInfo ci{ VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO };
        ci.maxSets       = FRAMES_IN_FLIGHT + 64;
        ci.poolSizeCount = 2;
        ci.pPoolSizes    = sizes;
        if (vkCreateDescriptorPool(ctx.device, &ci, nullptr, &pool) != VK_SUCCESS) {
            fprintf(stderr, "[VkDesc] Failed to create descriptor pool\n");
            return false;
        }
        return true;
    }

    bool createUBOBuffers(VulkanContext& ctx) {
        VkBufferCreateInfo bci{ VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO };
        bci.size  = sizeof(FrameUBO);
        bci.usage = VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT;

        VmaAllocationCreateInfo aci{};
        aci.usage = VMA_MEMORY_USAGE_CPU_TO_GPU;
        aci.flags = VMA_ALLOCATION_CREATE_MAPPED_BIT;

        for (int i = 0; i < FRAMES_IN_FLIGHT; i++) {
            VmaAllocationInfo info;
            if (vmaCreateBuffer(ctx.allocator, &bci, &aci,
                                &uboBuffers[i], &uboAllocs[i], &info) != VK_SUCCESS) {
                fprintf(stderr, "[VkDesc] Failed to create UBO buffer %d\n", i);
                return false;
            }
            uboMapped[i] = info.pMappedData;
        }
        return true;
    }

    bool createSet0(VulkanContext& ctx) {
        for (int i = 0; i < FRAMES_IN_FLIGHT; i++) {
            VkDescriptorSetAllocateInfo ai{ VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO };
            ai.descriptorPool     = pool;
            ai.descriptorSetCount = 1;
            ai.pSetLayouts        = &set0Layout;
            if (vkAllocateDescriptorSets(ctx.device, &ai, &set0[i]) != VK_SUCCESS) {
                fprintf(stderr, "[VkDesc] Failed to allocate Set 0[%d]\n", i);
                return false;
            }

            VkDescriptorBufferInfo bufInfo{};
            bufInfo.buffer = uboBuffers[i];
            bufInfo.offset = 0;
            bufInfo.range  = sizeof(FrameUBO);

            VkWriteDescriptorSet write{ VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET };
            write.dstSet          = set0[i];
            write.dstBinding      = 0;
            write.descriptorType  = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
            write.descriptorCount = 1;
            write.pBufferInfo     = &bufInfo;
            vkUpdateDescriptorSets(ctx.device, 1, &write, 0, nullptr);
        }
        return true;
    }
};
