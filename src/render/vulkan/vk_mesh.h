#pragma once
// ==========================================================================
// vk_mesh.h — VkBuffer 替换 OpenGL VAO/VBO/EBO
//
// 使用前提：vk_context.h 已 include（提供 VulkanContext + VMA）
// ==========================================================================

#include "vk_context.h"
#include <cstring>

// -----------------------------------------------------------------------
// 一次性命令缓冲 helpers（用于 staging buffer 上传）
// -----------------------------------------------------------------------
inline VkCommandBuffer beginSingleTimeCommands(VulkanContext& ctx) {
    VkCommandBufferAllocateInfo ai{ VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO };
    ai.commandPool        = ctx.commandPool;
    ai.level              = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
    ai.commandBufferCount = 1;

    VkCommandBuffer cmd;
    vkAllocateCommandBuffers(ctx.device, &ai, &cmd);

    VkCommandBufferBeginInfo bi{ VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO };
    bi.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
    vkBeginCommandBuffer(cmd, &bi);
    return cmd;
}

inline void endSingleTimeCommands(VulkanContext& ctx, VkCommandBuffer cmd) {
    vkEndCommandBuffer(cmd);

    VkSubmitInfo si{ VK_STRUCTURE_TYPE_SUBMIT_INFO };
    si.commandBufferCount = 1;
    si.pCommandBuffers    = &cmd;
    vkQueueSubmit(ctx.graphicsQueue, 1, &si, VK_NULL_HANDLE);
    vkQueueWaitIdle(ctx.graphicsQueue);
    vkFreeCommandBuffers(ctx.device, ctx.commandPool, 1, &cmd);
}

// -----------------------------------------------------------------------
// VkMesh — GPU 顶点/索引缓冲封装
// -----------------------------------------------------------------------
struct VkMesh {
    VkBuffer      vertexBuffer = VK_NULL_HANDLE;
    VkBuffer      indexBuffer  = VK_NULL_HANDLE;
    VmaAllocation vertAlloc    = VK_NULL_HANDLE;
    VmaAllocation idxAlloc     = VK_NULL_HANDLE;
    uint32_t      indexCount   = 0;

    // Upload to GPU-only memory via staging buffers.
    // Requires ctx.commandPool to be valid (Phase 5+).
    bool upload(VulkanContext& ctx,
                const void*     vertexData, VkDeviceSize vertexSize,
                const uint32_t* indexData,  uint32_t     idxCount) {
        indexCount = idxCount;
        VkDeviceSize indexSize = (VkDeviceSize)idxCount * sizeof(uint32_t);

        if (!uploadBuffer(ctx, vertexData, vertexSize,
                          VK_BUFFER_USAGE_VERTEX_BUFFER_BIT,
                          vertexBuffer, vertAlloc)) return false;
        if (!uploadBuffer(ctx, indexData, indexSize,
                          VK_BUFFER_USAGE_INDEX_BUFFER_BIT,
                          indexBuffer, idxAlloc))  return false;
        return true;
    }

    // Dynamic mesh variant: CPU_TO_GPU, no staging, direct memcpy each frame.
    // Used for SVO and other frequently-rebuilt meshes.
    bool uploadDynamic(VulkanContext& ctx,
                       const void*     vertexData, VkDeviceSize vertexSize,
                       const uint32_t* indexData,  uint32_t     idxCount) {
        indexCount = idxCount;

        auto makeBuffer = [&](const void* src, VkDeviceSize sz,
                              VkBufferUsageFlags usage,
                              VkBuffer& buf, VmaAllocation& alloc) -> bool {
            VkBufferCreateInfo bci{ VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO };
            bci.size  = sz;
            bci.usage = usage;

            VmaAllocationCreateInfo aci{};
            aci.usage = VMA_MEMORY_USAGE_CPU_TO_GPU;
            aci.flags = VMA_ALLOCATION_CREATE_MAPPED_BIT;

            VmaAllocationInfo info;
            if (vmaCreateBuffer(ctx.allocator, &bci, &aci, &buf, &alloc, &info) != VK_SUCCESS)
                return false;
            memcpy(info.pMappedData, src, sz);
            return true;
        };

        VkDeviceSize indexSize = (VkDeviceSize)idxCount * sizeof(uint32_t);
        if (!makeBuffer(vertexData, vertexSize, VK_BUFFER_USAGE_VERTEX_BUFFER_BIT, vertexBuffer, vertAlloc)) return false;
        if (!makeBuffer(indexData,  indexSize,  VK_BUFFER_USAGE_INDEX_BUFFER_BIT,  indexBuffer,  idxAlloc))  return false;
        return true;
    }

    void bind(VkCommandBuffer cmd) const {
        VkDeviceSize offset = 0;
        vkCmdBindVertexBuffers(cmd, 0, 1, &vertexBuffer, &offset);
        vkCmdBindIndexBuffer(cmd, indexBuffer, 0, VK_INDEX_TYPE_UINT32);
    }

    void draw(VkCommandBuffer cmd) const {
        vkCmdDrawIndexed(cmd, indexCount, 1, 0, 0, 0);
    }

    void destroy(VulkanContext& ctx) {
        if (vertexBuffer != VK_NULL_HANDLE) {
            vmaDestroyBuffer(ctx.allocator, vertexBuffer, vertAlloc);
            vertexBuffer = VK_NULL_HANDLE;
            vertAlloc    = VK_NULL_HANDLE;
        }
        if (indexBuffer != VK_NULL_HANDLE) {
            vmaDestroyBuffer(ctx.allocator, indexBuffer, idxAlloc);
            indexBuffer = VK_NULL_HANDLE;
            idxAlloc    = VK_NULL_HANDLE;
        }
    }

private:
    static bool uploadBuffer(VulkanContext& ctx,
                             const void* data, VkDeviceSize size,
                             VkBufferUsageFlags usage,
                             VkBuffer& outBuf, VmaAllocation& outAlloc) {
        // Staging buffer — CPU writable, GPU readable
        VkBufferCreateInfo stagCI{ VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO };
        stagCI.size  = size;
        stagCI.usage = VK_BUFFER_USAGE_TRANSFER_SRC_BIT;

        VmaAllocationCreateInfo stagACI{};
        stagACI.usage = VMA_MEMORY_USAGE_CPU_ONLY;

        VkBuffer stagBuf; VmaAllocation stagAlloc;
        if (vmaCreateBuffer(ctx.allocator, &stagCI, &stagACI, &stagBuf, &stagAlloc, nullptr) != VK_SUCCESS) {
            fprintf(stderr, "[VkMesh] Failed to create staging buffer\n");
            return false;
        }

        void* mapped;
        vmaMapMemory(ctx.allocator, stagAlloc, &mapped);
        memcpy(mapped, data, size);
        vmaUnmapMemory(ctx.allocator, stagAlloc);

        // Device-local buffer — GPU only
        VkBufferCreateInfo devCI{ VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO };
        devCI.size  = size;
        devCI.usage = usage | VK_BUFFER_USAGE_TRANSFER_DST_BIT;

        VmaAllocationCreateInfo devACI{};
        devACI.usage = VMA_MEMORY_USAGE_GPU_ONLY;

        if (vmaCreateBuffer(ctx.allocator, &devCI, &devACI, &outBuf, &outAlloc, nullptr) != VK_SUCCESS) {
            fprintf(stderr, "[VkMesh] Failed to create device buffer\n");
            vmaDestroyBuffer(ctx.allocator, stagBuf, stagAlloc);
            return false;
        }

        // Copy staging → device
        VkCommandBuffer cmd = beginSingleTimeCommands(ctx);
        VkBufferCopy copy{ 0, 0, size };
        vkCmdCopyBuffer(cmd, stagBuf, outBuf, 1, &copy);
        endSingleTimeCommands(ctx, cmd);

        vmaDestroyBuffer(ctx.allocator, stagBuf, stagAlloc);
        return true;
    }
};
