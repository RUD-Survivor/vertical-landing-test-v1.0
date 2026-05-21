#pragma once
// ==========================================================================
// vk_utils.h — Vulkan 1.3 图像布局转换工具
//
// 使用 VkImageMemoryBarrier2 + vkCmdPipelineBarrier2 (synchronization2 core in 1.3)
// 比旧版 vkCmdPipelineBarrier 更精确：每个 barrier 独立指定 stage/access mask。
// ==========================================================================

#include "vk_context.h"

// -----------------------------------------------------------------------
// 判断 layout 对应的 aspect (color / depth / depth+stencil)
// -----------------------------------------------------------------------
inline VkImageAspectFlags aspectMaskForLayout(VkImageLayout layout) {
    return (layout == VK_IMAGE_LAYOUT_DEPTH_ATTACHMENT_OPTIMAL
         || layout == VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL
         || layout == VK_IMAGE_LAYOUT_DEPTH_READ_ONLY_OPTIMAL)
        ? VK_IMAGE_ASPECT_DEPTH_BIT
        : VK_IMAGE_ASPECT_COLOR_BIT;
}

// -----------------------------------------------------------------------
// 通用图像布局转换 (Vulkan 1.3 synchronization2)
//
// srcStage/Access 描述"谁刚写完"，dstStage/Access 描述"谁接下来要读/写"。
// -----------------------------------------------------------------------
inline void transitionImage(VkCommandBuffer    cmd,
                            VkImage            image,
                            VkImageLayout      oldLayout,
                            VkImageLayout      newLayout,
                            VkPipelineStageFlags2 srcStage  = VK_PIPELINE_STAGE_2_ALL_COMMANDS_BIT,
                            VkAccessFlags2     srcAccess = VK_ACCESS_2_MEMORY_WRITE_BIT,
                            VkPipelineStageFlags2 dstStage  = VK_PIPELINE_STAGE_2_ALL_COMMANDS_BIT,
                            VkAccessFlags2     dstAccess = VK_ACCESS_2_MEMORY_WRITE_BIT | VK_ACCESS_2_MEMORY_READ_BIT)
{
    VkImageAspectFlags aspect = VK_IMAGE_ASPECT_COLOR_BIT;
    if (newLayout == VK_IMAGE_LAYOUT_DEPTH_ATTACHMENT_OPTIMAL
     || newLayout == VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL
     || newLayout == VK_IMAGE_LAYOUT_DEPTH_READ_ONLY_OPTIMAL
     || oldLayout == VK_IMAGE_LAYOUT_DEPTH_ATTACHMENT_OPTIMAL
     || oldLayout == VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL)
        aspect = VK_IMAGE_ASPECT_DEPTH_BIT;

    VkImageMemoryBarrier2 barrier{ VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER_2 };
    barrier.srcStageMask        = srcStage;
    barrier.srcAccessMask       = srcAccess;
    barrier.dstStageMask        = dstStage;
    barrier.dstAccessMask       = dstAccess;
    barrier.oldLayout           = oldLayout;
    barrier.newLayout           = newLayout;
    barrier.image               = image;
    barrier.subresourceRange    = { aspect, 0, VK_REMAINING_MIP_LEVELS, 0, VK_REMAINING_ARRAY_LAYERS };

    VkDependencyInfo dep{ VK_STRUCTURE_TYPE_DEPENDENCY_INFO };
    dep.imageMemoryBarrierCount = 1;
    dep.pImageMemoryBarriers    = &barrier;
    vkCmdPipelineBarrier2(cmd, &dep);
}

// -----------------------------------------------------------------------
// 常用快捷转换
// -----------------------------------------------------------------------

// 未定义 → 颜色附件（渲染开始前）
inline void transitionToColorAttachment(VkCommandBuffer cmd, VkImage image) {
    transitionImage(cmd, image,
        VK_IMAGE_LAYOUT_UNDEFINED,
        VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL,
        VK_PIPELINE_STAGE_2_TOP_OF_PIPE_BIT,    VK_ACCESS_2_NONE,
        VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT,
        VK_ACCESS_2_COLOR_ATTACHMENT_WRITE_BIT);
}

// 颜色附件 → 呈现（渲染结束后）
inline void transitionToPresent(VkCommandBuffer cmd, VkImage image) {
    transitionImage(cmd, image,
        VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL,
        VK_IMAGE_LAYOUT_PRESENT_SRC_KHR,
        VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT,
        VK_ACCESS_2_COLOR_ATTACHMENT_WRITE_BIT,
        VK_PIPELINE_STAGE_2_BOTTOM_OF_PIPE_BIT, VK_ACCESS_2_NONE);
}

// 未定义 → 深度附件（渲染开始前）
inline void transitionToDepthAttachment(VkCommandBuffer cmd, VkImage image) {
    transitionImage(cmd, image,
        VK_IMAGE_LAYOUT_UNDEFINED,
        VK_IMAGE_LAYOUT_DEPTH_ATTACHMENT_OPTIMAL,
        VK_PIPELINE_STAGE_2_TOP_OF_PIPE_BIT,    VK_ACCESS_2_NONE,
        VK_PIPELINE_STAGE_2_EARLY_FRAGMENT_TESTS_BIT,
        VK_ACCESS_2_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT);
}

// 未定义 → 着色器采样（纹理上传完成后）
inline void transitionToShaderRead(VkCommandBuffer cmd, VkImage image,
                                   uint32_t mipLevels = VK_REMAINING_MIP_LEVELS) {
    VkImageMemoryBarrier2 barrier{ VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER_2 };
    barrier.srcStageMask     = VK_PIPELINE_STAGE_2_TRANSFER_BIT;
    barrier.srcAccessMask    = VK_ACCESS_2_TRANSFER_WRITE_BIT;
    barrier.dstStageMask     = VK_PIPELINE_STAGE_2_FRAGMENT_SHADER_BIT;
    barrier.dstAccessMask    = VK_ACCESS_2_SHADER_READ_BIT;
    barrier.oldLayout        = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
    barrier.newLayout        = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
    barrier.image            = image;
    barrier.subresourceRange = { VK_IMAGE_ASPECT_COLOR_BIT, 0, mipLevels, 0, VK_REMAINING_ARRAY_LAYERS };

    VkDependencyInfo dep{ VK_STRUCTURE_TYPE_DEPENDENCY_INFO };
    dep.imageMemoryBarrierCount = 1;
    dep.pImageMemoryBarriers    = &barrier;
    vkCmdPipelineBarrier2(cmd, &dep);
}
