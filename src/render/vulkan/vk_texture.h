#pragma once
// ==========================================================================
// vk_texture.h — 2D/3D 纹理封装 (VMA + Vulkan 1.3 synchronization2)
//
// 上传流程：
//   1. CPU 像素数据 → Staging Buffer (CPU_ONLY)
//   2. vkCmdCopyBufferToImage (UNDEFINED → TRANSFER_DST_OPTIMAL)
//   3. transitionImage (TRANSFER_DST_OPTIMAL → SHADER_READ_ONLY_OPTIMAL)
//   4. VkSampler (LINEAR + REPEAT + Anisotropy 16x)
// ==========================================================================

#include "vk_context.h"
#include "vk_utils.h"
#include "vk_mesh.h"   // beginSingleTimeCommands / endSingleTimeCommands

#include <cstdint>
#include <cstdio>

// -----------------------------------------------------------------------
// VkTexture2D — 单张 2D 纹理（替换 OpenGL glGenTextures/glTexImage2D）
// -----------------------------------------------------------------------
struct VkTexture2D {
    VkImage       image   = VK_NULL_HANDLE;
    VmaAllocation alloc   = VK_NULL_HANDLE;
    VkImageView   view    = VK_NULL_HANDLE;
    VkSampler     sampler = VK_NULL_HANDLE;

    uint32_t width  = 0;
    uint32_t height = 0;

    // 从 CPU 内存上传 RGBA8 像素（4 字节/像素）
    bool upload(VulkanContext& ctx,
                const uint8_t* pixels,
                uint32_t       w,
                uint32_t       h,
                VkFormat       format = VK_FORMAT_R8G8B8A8_UNORM) {
        width  = w;
        height = h;
        VkDeviceSize size = (VkDeviceSize)w * h * 4;

        // --- Staging buffer ---
        VkBuffer      stagingBuf  = VK_NULL_HANDLE;
        VmaAllocation stagingAlloc = VK_NULL_HANDLE;
        {
            VkBufferCreateInfo bci{ VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO };
            bci.size        = size;
            bci.usage       = VK_BUFFER_USAGE_TRANSFER_SRC_BIT;
            VmaAllocationCreateInfo aci{};
            aci.usage = VMA_MEMORY_USAGE_CPU_ONLY;
            aci.flags = VMA_ALLOCATION_CREATE_MAPPED_BIT;
            VmaAllocationInfo info{};
            if (vmaCreateBuffer(ctx.allocator, &bci, &aci, &stagingBuf, &stagingAlloc, &info) != VK_SUCCESS) {
                fprintf(stderr, "[VkTexture2D] Failed to create staging buffer\n");
                return false;
            }
            memcpy(info.pMappedData, pixels, (size_t)size);
        }

        // --- GPU image ---
        {
            VkImageCreateInfo ici{ VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO };
            ici.imageType     = VK_IMAGE_TYPE_2D;
            ici.format        = format;
            ici.extent        = { w, h, 1 };
            ici.mipLevels     = 1;
            ici.arrayLayers   = 1;
            ici.samples       = VK_SAMPLE_COUNT_1_BIT;
            ici.tiling        = VK_IMAGE_TILING_OPTIMAL;
            ici.usage         = VK_IMAGE_USAGE_TRANSFER_DST_BIT | VK_IMAGE_USAGE_SAMPLED_BIT;
            ici.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
            VmaAllocationCreateInfo aci{};
            aci.usage = VMA_MEMORY_USAGE_GPU_ONLY;
            if (vmaCreateImage(ctx.allocator, &ici, &aci, &image, &alloc, nullptr) != VK_SUCCESS) {
                fprintf(stderr, "[VkTexture2D] Failed to create GPU image\n");
                vmaDestroyBuffer(ctx.allocator, stagingBuf, stagingAlloc);
                return false;
            }
        }

        // --- Upload via one-shot command buffer ---
        VkCommandBuffer cmd = beginSingleTimeCommands(ctx);

        // UNDEFINED → TRANSFER_DST_OPTIMAL
        transitionImage(cmd, image,
            VK_IMAGE_LAYOUT_UNDEFINED,
            VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
            VK_PIPELINE_STAGE_2_TOP_OF_PIPE_BIT,    VK_ACCESS_2_NONE,
            VK_PIPELINE_STAGE_2_TRANSFER_BIT,        VK_ACCESS_2_TRANSFER_WRITE_BIT);

        VkBufferImageCopy region{};
        region.imageSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
        region.imageSubresource.layerCount = 1;
        region.imageExtent                 = { w, h, 1 };
        vkCmdCopyBufferToImage(cmd, stagingBuf, image,
            VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, 1, &region);

        // TRANSFER_DST_OPTIMAL → SHADER_READ_ONLY_OPTIMAL
        transitionToShaderRead(cmd, image, 1);

        endSingleTimeCommands(ctx, cmd);
        vmaDestroyBuffer(ctx.allocator, stagingBuf, stagingAlloc);

        // --- Image view ---
        VkImageViewCreateInfo vci{ VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO };
        vci.image                           = image;
        vci.viewType                        = VK_IMAGE_VIEW_TYPE_2D;
        vci.format                          = format;
        vci.subresourceRange.aspectMask     = VK_IMAGE_ASPECT_COLOR_BIT;
        vci.subresourceRange.levelCount     = 1;
        vci.subresourceRange.layerCount     = 1;
        if (vkCreateImageView(ctx.device, &vci, nullptr, &view) != VK_SUCCESS) {
            fprintf(stderr, "[VkTexture2D] Failed to create image view\n");
            return false;
        }

        // --- Sampler ---
        if (!createSampler(ctx)) return false;

        printf("[VkTexture2D] Uploaded %ux%u\n", w, h);
        return true;
    }

    // Upload RGBA32F float texture (16 bytes/pixel: 4×float).
    // Used for hydro/climate maps that need raw float values in the shader.
    bool uploadFloat4(VulkanContext& ctx, const float* data, uint32_t w, uint32_t h) {
        width = w; height = h;
        VkDeviceSize size = (VkDeviceSize)w * h * 4 * sizeof(float);

        VkBuffer      stagingBuf  = VK_NULL_HANDLE;
        VmaAllocation stagingAlloc = VK_NULL_HANDLE;
        {
            VkBufferCreateInfo bci{ VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO };
            bci.size  = size;
            bci.usage = VK_BUFFER_USAGE_TRANSFER_SRC_BIT;
            VmaAllocationCreateInfo aci{};
            aci.usage = VMA_MEMORY_USAGE_CPU_ONLY;
            aci.flags = VMA_ALLOCATION_CREATE_MAPPED_BIT;
            VmaAllocationInfo info{};
            if (vmaCreateBuffer(ctx.allocator, &bci, &aci, &stagingBuf, &stagingAlloc, &info) != VK_SUCCESS) {
                fprintf(stderr, "[VkTexture2D] uploadFloat4: staging failed\n"); return false;
            }
            memcpy(info.pMappedData, data, (size_t)size);
        }
        VkFormat fmt = VK_FORMAT_R32G32B32A32_SFLOAT;
        {
            VkImageCreateInfo ici{ VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO };
            ici.imageType  = VK_IMAGE_TYPE_2D; ici.format = fmt;
            ici.extent     = { w, h, 1 };
            ici.mipLevels  = 1; ici.arrayLayers = 1;
            ici.samples    = VK_SAMPLE_COUNT_1_BIT;
            ici.tiling     = VK_IMAGE_TILING_OPTIMAL;
            ici.usage      = VK_IMAGE_USAGE_TRANSFER_DST_BIT | VK_IMAGE_USAGE_SAMPLED_BIT;
            ici.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
            VmaAllocationCreateInfo aci{}; aci.usage = VMA_MEMORY_USAGE_GPU_ONLY;
            if (vmaCreateImage(ctx.allocator, &ici, &aci, &image, &alloc, nullptr) != VK_SUCCESS) {
                fprintf(stderr, "[VkTexture2D] uploadFloat4: image create failed\n");
                vmaDestroyBuffer(ctx.allocator, stagingBuf, stagingAlloc); return false;
            }
        }
        VkCommandBuffer cmd = beginSingleTimeCommands(ctx);
        transitionImage(cmd, image, VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
            VK_PIPELINE_STAGE_2_TOP_OF_PIPE_BIT, VK_ACCESS_2_NONE,
            VK_PIPELINE_STAGE_2_TRANSFER_BIT,   VK_ACCESS_2_TRANSFER_WRITE_BIT);
        VkBufferImageCopy region{};
        region.imageSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
        region.imageSubresource.layerCount = 1;
        region.imageExtent = { w, h, 1 };
        vkCmdCopyBufferToImage(cmd, stagingBuf, image, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, 1, &region);
        transitionToShaderRead(cmd, image, 1);
        endSingleTimeCommands(ctx, cmd);
        vmaDestroyBuffer(ctx.allocator, stagingBuf, stagingAlloc);

        VkImageViewCreateInfo vci{ VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO };
        vci.image = image; vci.viewType = VK_IMAGE_VIEW_TYPE_2D; vci.format = fmt;
        vci.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
        vci.subresourceRange.levelCount = 1; vci.subresourceRange.layerCount = 1;
        if (vkCreateImageView(ctx.device, &vci, nullptr, &view) != VK_SUCCESS) {
            fprintf(stderr, "[VkTexture2D] uploadFloat4: view failed\n"); return false;
        }
        if (!createSampler(ctx)) return false;
        printf("[VkTexture2D] uploadFloat4 %ux%u RGBA32F\n", w, h);
        return true;
    }

    // Compute bake target: STORAGE + SAMPLED, no CPU upload. 镜像 VkTexture3D::createStorage3D，
    // 用于 GPU compute 直接 imageStore 写入的 2D 烘焙纹理（如 cloud curl noise）。
    bool createStorage2D(VulkanContext& ctx,
                         uint32_t       w,
                         uint32_t       h,
                         VkFormat       fmt = VK_FORMAT_R8_UNORM) {
        width = w; height = h;

        VkImageCreateInfo ici{ VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO };
        ici.imageType     = VK_IMAGE_TYPE_2D;
        ici.format        = fmt;
        ici.extent        = { w, h, 1 };
        ici.mipLevels     = 1;
        ici.arrayLayers   = 1;
        ici.samples       = VK_SAMPLE_COUNT_1_BIT;
        ici.tiling        = VK_IMAGE_TILING_OPTIMAL;
        ici.usage         = VK_IMAGE_USAGE_STORAGE_BIT | VK_IMAGE_USAGE_SAMPLED_BIT;
        ici.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
        VmaAllocationCreateInfo aci{};
        aci.usage = VMA_MEMORY_USAGE_GPU_ONLY;
        if (vmaCreateImage(ctx.allocator, &ici, &aci, &image, &alloc, nullptr) != VK_SUCCESS) {
            fprintf(stderr, "[VkTexture2D] Failed to create storage 2D image\n");
            return false;
        }

        VkImageViewCreateInfo vci{ VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO };
        vci.image                       = image;
        vci.viewType                    = VK_IMAGE_VIEW_TYPE_2D;
        vci.format                      = fmt;
        vci.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
        vci.subresourceRange.levelCount = 1;
        vci.subresourceRange.layerCount = 1;
        if (vkCreateImageView(ctx.device, &vci, nullptr, &view) != VK_SUCCESS) {
            fprintf(stderr, "[VkTexture2D] Failed to create storage 2D view\n");
            destroy(ctx);
            return false;
        }

        if (!createSampler(ctx)) { destroy(ctx); return false; }

        printf("[VkTexture2D] Created storage %ux%u (fmt=%d)\n", w, h, (int)fmt);
        return true;
    }

    void destroy(VulkanContext& ctx) {
        if (sampler != VK_NULL_HANDLE) { vkDestroySampler   (ctx.device, sampler, nullptr); sampler = VK_NULL_HANDLE; }
        if (view    != VK_NULL_HANDLE) { vkDestroyImageView (ctx.device, view,    nullptr); view    = VK_NULL_HANDLE; }
        if (image   != VK_NULL_HANDLE) { vmaDestroyImage    (ctx.allocator, image, alloc);  image   = VK_NULL_HANDLE; }
    }

private:
    bool createSampler(VulkanContext& ctx) {
        VkSamplerCreateInfo sci{ VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO };
        sci.magFilter        = VK_FILTER_LINEAR;
        sci.minFilter        = VK_FILTER_LINEAR;
        sci.mipmapMode       = VK_SAMPLER_MIPMAP_MODE_LINEAR;
        sci.addressModeU     = VK_SAMPLER_ADDRESS_MODE_REPEAT;
        sci.addressModeV     = VK_SAMPLER_ADDRESS_MODE_REPEAT;
        sci.addressModeW     = VK_SAMPLER_ADDRESS_MODE_REPEAT;
        sci.anisotropyEnable = VK_TRUE;
        sci.maxAnisotropy    = 16.0f;
        sci.maxLod           = VK_LOD_CLAMP_NONE;
        if (vkCreateSampler(ctx.device, &sci, nullptr, &sampler) != VK_SUCCESS) {
            fprintf(stderr, "[VkTexture2D] Failed to create sampler\n");
            return false;
        }
        return true;
    }
};

// -----------------------------------------------------------------------
// VkTexture3D — 3D 纹理（云系统 sampler3D 用，如 128³ 噪声体积）
// -----------------------------------------------------------------------
struct VkTexture3D {
    VkImage       image   = VK_NULL_HANDLE;
    VmaAllocation alloc   = VK_NULL_HANDLE;
    VkImageView   view    = VK_NULL_HANDLE;
    VkSampler     sampler = VK_NULL_HANDLE;

    uint32_t width  = 0;
    uint32_t height = 0;
    uint32_t depth  = 0;
    VkFormat  format = VK_FORMAT_R8G8B8A8_UNORM;

    // Compute bake target: STORAGE + SAMPLED, no CPU upload.
    bool createStorage3D(VulkanContext& ctx,
                         uint32_t       w,
                         uint32_t       h,
                         uint32_t       d,
                         VkFormat       fmt = VK_FORMAT_R8_UNORM) {
        width = w; height = h; depth = d; format = fmt;

        VkImageCreateInfo ici{ VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO };
        ici.imageType     = VK_IMAGE_TYPE_3D;
        ici.format        = fmt;
        ici.extent        = { w, h, d };
        ici.mipLevels     = 1;
        ici.arrayLayers   = 1;
        ici.samples       = VK_SAMPLE_COUNT_1_BIT;
        ici.tiling        = VK_IMAGE_TILING_OPTIMAL;
        ici.usage         = VK_IMAGE_USAGE_STORAGE_BIT | VK_IMAGE_USAGE_SAMPLED_BIT;
        ici.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
        VmaAllocationCreateInfo aci{};
        aci.usage = VMA_MEMORY_USAGE_GPU_ONLY;
        if (vmaCreateImage(ctx.allocator, &ici, &aci, &image, &alloc, nullptr) != VK_SUCCESS) {
            fprintf(stderr, "[VkTexture3D] Failed to create storage 3D image\n");
            return false;
        }

        VkImageViewCreateInfo vci{ VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO };
        vci.image                           = image;
        vci.viewType                        = VK_IMAGE_VIEW_TYPE_3D;
        vci.format                          = fmt;
        vci.subresourceRange.aspectMask     = VK_IMAGE_ASPECT_COLOR_BIT;
        vci.subresourceRange.levelCount     = 1;
        vci.subresourceRange.layerCount     = 1;
        if (vkCreateImageView(ctx.device, &vci, nullptr, &view) != VK_SUCCESS) {
            fprintf(stderr, "[VkTexture3D] Failed to create storage 3D view\n");
            destroy(ctx);
            return false;
        }

        VkSamplerCreateInfo sci{ VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO };
        sci.magFilter        = VK_FILTER_LINEAR;
        sci.minFilter        = VK_FILTER_LINEAR;
        sci.mipmapMode       = VK_SAMPLER_MIPMAP_MODE_LINEAR;
        sci.addressModeU     = VK_SAMPLER_ADDRESS_MODE_REPEAT;
        sci.addressModeV     = VK_SAMPLER_ADDRESS_MODE_REPEAT;
        sci.addressModeW     = VK_SAMPLER_ADDRESS_MODE_REPEAT;
        sci.anisotropyEnable = VK_FALSE;
        sci.maxLod           = VK_LOD_CLAMP_NONE;
        if (vkCreateSampler(ctx.device, &sci, nullptr, &sampler) != VK_SUCCESS) {
            fprintf(stderr, "[VkTexture3D] Failed to create storage 3D sampler\n");
            destroy(ctx);
            return false;
        }

        printf("[VkTexture3D] Created storage %ux%ux%u (fmt=%d)\n", w, h, d, (int)fmt);
        return true;
    }

    // pixels: width*height*depth*4 bytes (RGBA8)
    bool upload(VulkanContext& ctx,
                const uint8_t* pixels,
                uint32_t       w,
                uint32_t       h,
                uint32_t       d,
                VkFormat       format = VK_FORMAT_R8G8B8A8_UNORM) {
        width = w; height = h; depth = d; this->format = format;
        VkDeviceSize size = (VkDeviceSize)w * h * d * 4;

        // --- Staging buffer ---
        VkBuffer      stagingBuf   = VK_NULL_HANDLE;
        VmaAllocation stagingAlloc = VK_NULL_HANDLE;
        {
            VkBufferCreateInfo bci{ VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO };
            bci.size  = size;
            bci.usage = VK_BUFFER_USAGE_TRANSFER_SRC_BIT;
            VmaAllocationCreateInfo aci{};
            aci.usage = VMA_MEMORY_USAGE_CPU_ONLY;
            aci.flags = VMA_ALLOCATION_CREATE_MAPPED_BIT;
            VmaAllocationInfo info{};
            if (vmaCreateBuffer(ctx.allocator, &bci, &aci, &stagingBuf, &stagingAlloc, &info) != VK_SUCCESS) {
                fprintf(stderr, "[VkTexture3D] Failed to create staging buffer\n");
                return false;
            }
            memcpy(info.pMappedData, pixels, (size_t)size);
        }

        // --- GPU image (3D) ---
        {
            VkImageCreateInfo ici{ VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO };
            ici.imageType     = VK_IMAGE_TYPE_3D;
            ici.format        = format;
            ici.extent        = { w, h, d };
            ici.mipLevels     = 1;
            ici.arrayLayers   = 1;
            ici.samples       = VK_SAMPLE_COUNT_1_BIT;
            ici.tiling        = VK_IMAGE_TILING_OPTIMAL;
            ici.usage         = VK_IMAGE_USAGE_TRANSFER_DST_BIT | VK_IMAGE_USAGE_SAMPLED_BIT;
            ici.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
            VmaAllocationCreateInfo aci{};
            aci.usage = VMA_MEMORY_USAGE_GPU_ONLY;
            if (vmaCreateImage(ctx.allocator, &ici, &aci, &image, &alloc, nullptr) != VK_SUCCESS) {
                fprintf(stderr, "[VkTexture3D] Failed to create GPU image\n");
                vmaDestroyBuffer(ctx.allocator, stagingBuf, stagingAlloc);
                return false;
            }
        }

        // --- Upload ---
        VkCommandBuffer cmd = beginSingleTimeCommands(ctx);

        transitionImage(cmd, image,
            VK_IMAGE_LAYOUT_UNDEFINED,
            VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
            VK_PIPELINE_STAGE_2_TOP_OF_PIPE_BIT, VK_ACCESS_2_NONE,
            VK_PIPELINE_STAGE_2_TRANSFER_BIT,     VK_ACCESS_2_TRANSFER_WRITE_BIT);

        VkBufferImageCopy region{};
        region.imageSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
        region.imageSubresource.layerCount = 1;
        region.imageExtent                 = { w, h, d };
        vkCmdCopyBufferToImage(cmd, stagingBuf, image,
            VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, 1, &region);

        transitionToShaderRead(cmd, image, 1);

        endSingleTimeCommands(ctx, cmd);
        vmaDestroyBuffer(ctx.allocator, stagingBuf, stagingAlloc);

        // --- Image view (3D) ---
        VkImageViewCreateInfo vci{ VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO };
        vci.image                           = image;
        vci.viewType                        = VK_IMAGE_VIEW_TYPE_3D;
        vci.format                          = format;
        vci.subresourceRange.aspectMask     = VK_IMAGE_ASPECT_COLOR_BIT;
        vci.subresourceRange.levelCount     = 1;
        vci.subresourceRange.layerCount     = 1;
        if (vkCreateImageView(ctx.device, &vci, nullptr, &view) != VK_SUCCESS) {
            fprintf(stderr, "[VkTexture3D] Failed to create image view\n");
            return false;
        }

        // --- Sampler (REPEAT on all 3 axes) ---
        VkSamplerCreateInfo sci{ VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO };
        sci.magFilter        = VK_FILTER_LINEAR;
        sci.minFilter        = VK_FILTER_LINEAR;
        sci.mipmapMode       = VK_SAMPLER_MIPMAP_MODE_LINEAR;
        sci.addressModeU     = VK_SAMPLER_ADDRESS_MODE_REPEAT;
        sci.addressModeV     = VK_SAMPLER_ADDRESS_MODE_REPEAT;
        sci.addressModeW     = VK_SAMPLER_ADDRESS_MODE_REPEAT;
        sci.anisotropyEnable = VK_FALSE;  // 各向异性对体积纹理无意义
        sci.maxLod           = VK_LOD_CLAMP_NONE;
        if (vkCreateSampler(ctx.device, &sci, nullptr, &sampler) != VK_SUCCESS) {
            fprintf(stderr, "[VkTexture3D] Failed to create sampler\n");
            return false;
        }

        printf("[VkTexture3D] Uploaded %ux%ux%u\n", w, h, d);
        return true;
    }

    void destroy(VulkanContext& ctx) {
        if (sampler != VK_NULL_HANDLE) { vkDestroySampler   (ctx.device, sampler, nullptr); sampler = VK_NULL_HANDLE; }
        if (view    != VK_NULL_HANDLE) { vkDestroyImageView (ctx.device, view,    nullptr); view    = VK_NULL_HANDLE; }
        if (image   != VK_NULL_HANDLE) { vmaDestroyImage    (ctx.allocator, image, alloc);  image   = VK_NULL_HANDLE; }
    }
};
