#pragma once
// ==========================================================================
// vk_renderpass.h — 动态渲染管理 (Vulkan 1.3 Dynamic Rendering)
//
// 用 vkCmdBeginRendering / vkCmdEndRendering 完全替代传统的
// VkRenderPass + VkFramebuffer 模式：
//   - 无需提前创建 VkRenderPass 对象
//   - 无需为每个 swapchain image 创建 VkFramebuffer
//   - 颜色/深度 attachment 在录制命令时直接指定 ImageView
//
// 前提：device 创建时已开启 VkPhysicalDeviceVulkan13Features::dynamicRendering
// ==========================================================================

#include "vk_context.h"
#include "vk_utils.h"
#include <vector>

struct VkFrameRenderer {
    // 深度缓冲（共享，整个 swapchain 生命周期复用）
    VkImage       depthImage  = VK_NULL_HANDLE;
    VmaAllocation depthAlloc  = VK_NULL_HANDLE;
    VkImageView   depthView   = VK_NULL_HANDLE;

    // swapchain image 当前帧的格式（pipeline 创建时需要）
    VkFormat colorFormat = VK_FORMAT_UNDEFINED;
    VkFormat depthFormat = VK_FORMAT_D32_SFLOAT;

    bool init(VulkanContext& ctx) {
        colorFormat = ctx.swapFormat;
        return createDepthBuffer(ctx);
    }

    void shutdown(VulkanContext& ctx) {
        destroyDepthBuffer(ctx);
    }

    // swapchain resize 后调用：重建深度缓冲（colorFormat 由外部更新）
    void recreate(VulkanContext& ctx) {
        destroyDepthBuffer(ctx);
        colorFormat = ctx.swapFormat;
        createDepthBuffer(ctx);
    }

    // -----------------------------------------------------------------------
    // 帧渲染开始：设置 dynamic rendering + 转换 image layout
    //
    // colorImage : 当前 swapchain image（用于 layout transition）
    // colorView  : 当前 swapchain image view（绑定为颜色附件）
    // extent     : 渲染区域
    // clearColor : 是否清除颜色（false = LOAD，保留上帧内容）
    // -----------------------------------------------------------------------
    void beginFrame(VkCommandBuffer cmd,
                    VkImage         colorImage,
                    VkImageView     colorView,
                    VkExtent2D      extent,
                    bool            clearColor = true) const
    {
        // swapchain image: UNDEFINED/PRESENT → COLOR_ATTACHMENT_OPTIMAL
        transitionToColorAttachment(cmd, colorImage);

        VkRenderingAttachmentInfo colorAI{ VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO };
        colorAI.imageView   = colorView;
        colorAI.imageLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
        colorAI.loadOp      = clearColor ? VK_ATTACHMENT_LOAD_OP_CLEAR : VK_ATTACHMENT_LOAD_OP_LOAD;
        colorAI.storeOp     = VK_ATTACHMENT_STORE_OP_STORE;
        colorAI.clearValue  = {{ 0.05f, 0.05f, 0.1f, 1.0f }};

        VkRenderingAttachmentInfo depthAI{ VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO };
        depthAI.imageView   = depthView;
        depthAI.imageLayout = VK_IMAGE_LAYOUT_DEPTH_ATTACHMENT_OPTIMAL;
        depthAI.loadOp      = VK_ATTACHMENT_LOAD_OP_CLEAR;
        depthAI.storeOp     = VK_ATTACHMENT_STORE_OP_DONT_CARE;
        depthAI.clearValue.depthStencil = { 1.0f, 0 };

        VkRenderingInfo ri{ VK_STRUCTURE_TYPE_RENDERING_INFO };
        ri.renderArea           = { {0, 0}, extent };
        ri.layerCount           = 1;
        ri.colorAttachmentCount = 1;
        ri.pColorAttachments    = &colorAI;
        ri.pDepthAttachment     = &depthAI;

        vkCmdBeginRendering(cmd, &ri);
    }

    // 帧渲染结束：结束 dynamic rendering + 转换到 PRESENT
    void endFrame(VkCommandBuffer cmd, VkImage colorImage) const {
        vkCmdEndRendering(cmd);
        // COLOR_ATTACHMENT_OPTIMAL → PRESENT_SRC_KHR
        transitionToPresent(cmd, colorImage);
    }

    // -----------------------------------------------------------------------
    // 设置动态 viewport 和 scissor（在 beginFrame 之后调用一次）
    // -----------------------------------------------------------------------
    static void setViewportScissor(VkCommandBuffer cmd, VkExtent2D extent) {
        VkViewport vp{};
        vp.x        = 0.0f;
        vp.y        = 0.0f;
        vp.width    = (float)extent.width;
        vp.height   = (float)extent.height;
        vp.minDepth = 0.0f;
        vp.maxDepth = 1.0f;
        vkCmdSetViewport(cmd, 0, 1, &vp);

        VkRect2D scissor{ {0, 0}, extent };
        vkCmdSetScissor(cmd, 0, 1, &scissor);
    }

private:
    bool createDepthBuffer(VulkanContext& ctx) {
        VkImageCreateInfo ici{ VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO };
        ici.imageType     = VK_IMAGE_TYPE_2D;
        ici.format        = depthFormat;
        ici.extent        = { ctx.swapExtent.width, ctx.swapExtent.height, 1 };
        ici.mipLevels     = 1;
        ici.arrayLayers   = 1;
        ici.samples       = VK_SAMPLE_COUNT_1_BIT;
        ici.tiling        = VK_IMAGE_TILING_OPTIMAL;
        ici.usage         = VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT;
        ici.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;

        VmaAllocationCreateInfo aci{};
        aci.usage = VMA_MEMORY_USAGE_GPU_ONLY;
        if (vmaCreateImage(ctx.allocator, &ici, &aci, &depthImage, &depthAlloc, nullptr) != VK_SUCCESS) {
            fprintf(stderr, "[VkFrameRenderer] Failed to create depth image\n");
            return false;
        }

        VkImageViewCreateInfo vci{ VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO };
        vci.image                           = depthImage;
        vci.viewType                        = VK_IMAGE_VIEW_TYPE_2D;
        vci.format                          = depthFormat;
        vci.subresourceRange.aspectMask     = VK_IMAGE_ASPECT_DEPTH_BIT;
        vci.subresourceRange.baseMipLevel   = 0;
        vci.subresourceRange.levelCount     = 1;
        vci.subresourceRange.baseArrayLayer = 0;
        vci.subresourceRange.layerCount     = 1;
        if (vkCreateImageView(ctx.device, &vci, nullptr, &depthView) != VK_SUCCESS) {
            fprintf(stderr, "[VkFrameRenderer] Failed to create depth image view\n");
            return false;
        }
        return true;
    }

    void destroyDepthBuffer(VulkanContext& ctx) {
        if (depthView  != VK_NULL_HANDLE) { vkDestroyImageView(ctx.device, depthView, nullptr); depthView  = VK_NULL_HANDLE; }
        if (depthImage != VK_NULL_HANDLE) { vmaDestroyImage(ctx.allocator, depthImage, depthAlloc);  depthImage = VK_NULL_HANDLE; }
    }
};
