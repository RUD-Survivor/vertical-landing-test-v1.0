#pragma once
// ==========================================================================
// vk_frame.h — Per-frame 同步对象与 Command Buffer
//
// 使用前提：已包含 <GLFW/glfw3.h>（带 GLFW_INCLUDE_VULKAN）
// ==========================================================================

#include <cstdio>

// 同时在 GPU 上飞行的最大帧数（CPU 领先 GPU 的帧数）
static constexpr int FRAMES_IN_FLIGHT = 2;

// -----------------------------------------------------------------------
// 每帧的同步资源
// -----------------------------------------------------------------------
struct FrameData {
    VkCommandBuffer commandBuffer  = VK_NULL_HANDLE;
    VkSemaphore     imageAvailable = VK_NULL_HANDLE;  // vkAcquireNextImageKHR 信号
    VkSemaphore     renderFinished = VK_NULL_HANDLE;  // vkQueuePresentKHR 等待
    VkFence         inFlightFence  = VK_NULL_HANDLE;  // CPU 等待 GPU 完成
};

// -----------------------------------------------------------------------
// FrameSync — 管理所有帧的同步对象
// -----------------------------------------------------------------------
struct FrameSync {
    FrameData frames[FRAMES_IN_FLIGHT]{};
    int       currentFrame = 0;

    bool init(VkDevice device, VkCommandPool commandPool) {
        VkCommandBufferAllocateInfo allocInfo{ VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO };
        allocInfo.commandPool        = commandPool;
        allocInfo.level              = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
        allocInfo.commandBufferCount = 1;

        VkSemaphoreCreateInfo semCI{ VK_STRUCTURE_TYPE_SEMAPHORE_CREATE_INFO };

        VkFenceCreateInfo fenceCI{ VK_STRUCTURE_TYPE_FENCE_CREATE_INFO };
        fenceCI.flags = VK_FENCE_CREATE_SIGNALED_BIT;  // 初始已信号，第一帧不会永久等待

        for (int i = 0; i < FRAMES_IN_FLIGHT; i++) {
            if (vkAllocateCommandBuffers(device, &allocInfo, &frames[i].commandBuffer) != VK_SUCCESS
             || vkCreateSemaphore(device, &semCI, nullptr, &frames[i].imageAvailable) != VK_SUCCESS
             || vkCreateSemaphore(device, &semCI, nullptr, &frames[i].renderFinished) != VK_SUCCESS
             || vkCreateFence(device, &fenceCI, nullptr, &frames[i].inFlightFence) != VK_SUCCESS)
            {
                fprintf(stderr, "[Vulkan] Failed to create frame sync objects for frame %d\n", i);
                return false;
            }
        }
        return true;
    }

    void shutdown(VkDevice device) {
        for (int i = 0; i < FRAMES_IN_FLIGHT; i++) {
            if (frames[i].imageAvailable != VK_NULL_HANDLE)
                vkDestroySemaphore(device, frames[i].imageAvailable, nullptr);
            if (frames[i].renderFinished != VK_NULL_HANDLE)
                vkDestroySemaphore(device, frames[i].renderFinished, nullptr);
            if (frames[i].inFlightFence != VK_NULL_HANDLE)
                vkDestroyFence(device, frames[i].inFlightFence, nullptr);
            // commandBuffer 在 commandPool 销毁时自动释放，无需单独 vkFreeCommandBuffers
        }
    }

    FrameData& current() { return frames[currentFrame]; }
    const FrameData& current() const { return frames[currentFrame]; }

    void advance() { currentFrame = (currentFrame + 1) % FRAMES_IN_FLIGHT; }
};
