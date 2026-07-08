#pragma once
// Heitz / flower Sobol blue-noise SSBOs for shared_blue_noise.glsl (descriptor set 2).

#include "vk_context.h"
#include "../data/blue_noise_sampler_data.h"

#include <cstdio>
#include <cstring>

struct VkBlueNoiseBuffers {
    static constexpr uint32_t kSobolCount      = 256u * 256u;
    static constexpr uint32_t kScramblingCount = 128u * 128u * 8u;
    static constexpr uint32_t kRankingCount    = 128u * 128u * 8u;

    VkBuffer sobolBuf = VK_NULL_HANDLE;
    VkBuffer rankingBuf = VK_NULL_HANDLE;
    VkBuffer scramblingBuf = VK_NULL_HANDLE;
    VmaAllocation sobolAlloc = VK_NULL_HANDLE;
    VmaAllocation rankingAlloc = VK_NULL_HANDLE;
    VmaAllocation scramblingAlloc = VK_NULL_HANDLE;

    VkDescriptorSetLayout setLayout = VK_NULL_HANDLE;
    VkDescriptorSet       set       = VK_NULL_HANDLE;
    VkDescriptorPool      pool      = VK_NULL_HANDLE;

    bool init(VulkanContext& ctx) {
        auto upload = [&](const uint32_t* src, VkDeviceSize bytes, VkBuffer& buf, VmaAllocation& alloc) -> bool {
            VkBufferCreateInfo bci{ VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO };
            bci.size = bytes;
            bci.usage = VK_BUFFER_USAGE_STORAGE_BUFFER_BIT;
            VmaAllocationCreateInfo aci{};
            aci.usage = VMA_MEMORY_USAGE_CPU_TO_GPU;
            if (vmaCreateBuffer(ctx.allocator, &bci, &aci, &buf, &alloc, nullptr) != VK_SUCCESS)
                return false;
            void* mapped = nullptr;
            if (vmaMapMemory(ctx.allocator, alloc, &mapped) != VK_SUCCESS)
                return false;
            memcpy(mapped, src, (size_t)bytes);
            vmaUnmapMemory(ctx.allocator, alloc);
            vmaFlushAllocation(ctx.allocator, alloc, 0, bytes);
            return true;
        };

        if (!upload(rs3d::blue_noise::kSobol256spp256d, kSobolCount * sizeof(uint32_t), sobolBuf, sobolAlloc) ||
            !upload(rs3d::blue_noise::kRankingTile, kRankingCount * sizeof(uint32_t), rankingBuf, rankingAlloc) ||
            !upload(rs3d::blue_noise::kScramblingTile, kScramblingCount * sizeof(uint32_t), scramblingBuf, scramblingAlloc)) {
            fprintf(stderr, "[VkBlueNoise] buffer upload failed\n");
            shutdown(ctx);
            return false;
        }

        VkDescriptorSetLayoutBinding bindings[3]{};
        for (uint32_t i = 0; i < 3; ++i) {
            bindings[i].binding = i;
            bindings[i].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
            bindings[i].descriptorCount = 1;
            bindings[i].stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
        }
        VkDescriptorSetLayoutCreateInfo lci{ VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO };
        lci.bindingCount = 3;
        lci.pBindings = bindings;
        if (vkCreateDescriptorSetLayout(ctx.device, &lci, nullptr, &setLayout) != VK_SUCCESS) {
            shutdown(ctx);
            return false;
        }

        VkDescriptorPoolSize ps{ VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 3 };
        VkDescriptorPoolCreateInfo pci{ VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO };
        pci.maxSets = 1;
        pci.poolSizeCount = 1;
        pci.pPoolSizes = &ps;
        if (vkCreateDescriptorPool(ctx.device, &pci, nullptr, &pool) != VK_SUCCESS) {
            shutdown(ctx);
            return false;
        }

        VkDescriptorSetAllocateInfo ai{ VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO };
        ai.descriptorPool = pool;
        ai.descriptorSetCount = 1;
        ai.pSetLayouts = &setLayout;
        if (vkAllocateDescriptorSets(ctx.device, &ai, &set) != VK_SUCCESS) {
            shutdown(ctx);
            return false;
        }

        VkDescriptorBufferInfo sobolInfo{ sobolBuf, 0, VK_WHOLE_SIZE };
        VkDescriptorBufferInfo rankingInfo{ rankingBuf, 0, VK_WHOLE_SIZE };
        VkDescriptorBufferInfo scramblingInfo{ scramblingBuf, 0, VK_WHOLE_SIZE };
        VkWriteDescriptorSet writes[3]{};
        auto bufW = [&](uint32_t i, uint32_t binding, VkDescriptorBufferInfo* info) {
            writes[i].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
            writes[i].dstSet = set;
            writes[i].dstBinding = binding;
            writes[i].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
            writes[i].descriptorCount = 1;
            writes[i].pBufferInfo = info;
        };
        bufW(0, 0, &sobolInfo);
        bufW(1, 1, &rankingInfo);
        bufW(2, 2, &scramblingInfo);
        vkUpdateDescriptorSets(ctx.device, 3, writes, 0, nullptr);

        printf("[VkBlueNoise] Sobol tile ready (set 2, %u + %u + %u uints)\n",
               kSobolCount, kRankingCount, kScramblingCount);
        return true;
    }

    void shutdown(VulkanContext& ctx) {
        if (pool) {
            vkDestroyDescriptorPool(ctx.device, pool, nullptr);
            pool = VK_NULL_HANDLE;
        }
        if (setLayout) {
            vkDestroyDescriptorSetLayout(ctx.device, setLayout, nullptr);
            setLayout = VK_NULL_HANDLE;
        }
        set = VK_NULL_HANDLE;
        auto destroyBuf = [&](VkBuffer& b, VmaAllocation& a) {
            if (b) { vmaDestroyBuffer(ctx.allocator, b, a); b = VK_NULL_HANDLE; }
        };
        destroyBuf(sobolBuf, sobolAlloc);
        destroyBuf(rankingBuf, rankingAlloc);
        destroyBuf(scramblingBuf, scramblingAlloc);
    }
};
