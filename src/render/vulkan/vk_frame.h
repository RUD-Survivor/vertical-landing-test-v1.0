#pragma once
// ==========================================================================
// vk_frame.h — Per-frame 同步对象与 Command Buffer
//
// Semaphore 设计（修复 MAILBOX 模式下的 semaphore 重用警告）：
//   - renderFinished: 每张 swapchain image 独立一个，避免 present 未消费时被再次发射
//   - acquireSems:    独立于 frame slot 的获取信号量池（MAX_ACQUIRE_SEMS 个）
// ==========================================================================

#include <cstdio>

static constexpr int FRAMES_IN_FLIGHT  = 2;
static constexpr int MAX_SWAPCHAIN_IMG = 4;  // 典型 swapchain 最多 3~4 张

// -----------------------------------------------------------------------
// 每帧的同步资源（fence + command buffer 仍按 frame slot 分配）
// -----------------------------------------------------------------------
struct FrameData {
    VkCommandBuffer commandBuffer = VK_NULL_HANDLE;
    VkFence         inFlightFence = VK_NULL_HANDLE;
};

// -----------------------------------------------------------------------
// FrameSync
// -----------------------------------------------------------------------
struct FrameSync {
    FrameData   frames[FRAMES_IN_FLIGHT]{};
    int         currentFrame = 0;

    // 独立的 acquire 信号量池（不与 frame slot 绑定）
    VkSemaphore acquireSems[MAX_SWAPCHAIN_IMG]{};
    int         acquireIdx  = 0;

    // 每张 swapchain image 独立的 renderFinished 信号量
    VkSemaphore renderSems[MAX_SWAPCHAIN_IMG]{};

    bool init(VkDevice device, VkCommandPool commandPool) {
        VkCommandBufferAllocateInfo allocInfo{ VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO };
        allocInfo.commandPool        = commandPool;
        allocInfo.level              = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
        allocInfo.commandBufferCount = 1;

        VkSemaphoreCreateInfo semCI{ VK_STRUCTURE_TYPE_SEMAPHORE_CREATE_INFO };
        VkFenceCreateInfo     fenceCI{ VK_STRUCTURE_TYPE_FENCE_CREATE_INFO };
        fenceCI.flags = VK_FENCE_CREATE_SIGNALED_BIT;

        for (int i = 0; i < FRAMES_IN_FLIGHT; i++) {
            if (vkAllocateCommandBuffers(device, &allocInfo, &frames[i].commandBuffer) != VK_SUCCESS
             || vkCreateFence(device, &fenceCI, nullptr, &frames[i].inFlightFence) != VK_SUCCESS)
            {
                fprintf(stderr, "[Vulkan] Failed to create frame sync objects for frame %d\n", i);
                return false;
            }
        }
        for (int i = 0; i < MAX_SWAPCHAIN_IMG; i++) {
            if (vkCreateSemaphore(device, &semCI, nullptr, &acquireSems[i]) != VK_SUCCESS
             || vkCreateSemaphore(device, &semCI, nullptr, &renderSems[i])  != VK_SUCCESS)
            {
                fprintf(stderr, "[Vulkan] Failed to create semaphore %d\n", i);
                return false;
            }
        }
        return true;
    }

    void shutdown(VkDevice device) {
        for (int i = 0; i < MAX_SWAPCHAIN_IMG; i++) {
            if (acquireSems[i] != VK_NULL_HANDLE) vkDestroySemaphore(device, acquireSems[i], nullptr);
            if (renderSems[i]  != VK_NULL_HANDLE) vkDestroySemaphore(device, renderSems[i],  nullptr);
        }
        for (int i = 0; i < FRAMES_IN_FLIGHT; i++) {
            if (frames[i].inFlightFence != VK_NULL_HANDLE)
                vkDestroyFence(device, frames[i].inFlightFence, nullptr);
        }
    }

    FrameData& current() { return frames[currentFrame]; }

    // 每次 acquire 取一个新信号量（循环）
    VkSemaphore nextAcquireSem() {
        VkSemaphore s = acquireSems[acquireIdx];
        acquireIdx = (acquireIdx + 1) % MAX_SWAPCHAIN_IMG;
        return s;
    }

    void advance() { currentFrame = (currentFrame + 1) % FRAMES_IN_FLIGHT; }
};
