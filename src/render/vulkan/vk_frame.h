#pragma once
#include <vector>
static constexpr int FRAMES_IN_FLIGHT = 2;
struct FrameData {
    VkCommandBuffer commandBuffer = VK_NULL_HANDLE;
    VkFence inFlightFence = VK_NULL_HANDLE;
    VkSemaphore acquireSemaphore = VK_NULL_HANDLE;
};
struct FrameSync {
    FrameData frames[FRAMES_IN_FLIGHT]{};
    int currentFrame = 0;
    // 按实际交换链图像数量分配，避免固定容量造成数组越界。
    std::vector<VkSemaphore> renderSems;
    std::vector<VkFence> presentFences;
    std::vector<bool> presentPending;
    bool init(VkDevice device, VkCommandPool pool, size_t imageCount) {
        VkCommandBufferAllocateInfo alloc{ VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO };
        alloc.commandPool = pool; alloc.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY; alloc.commandBufferCount = 1;
        VkFenceCreateInfo fence{ VK_STRUCTURE_TYPE_FENCE_CREATE_INFO };
        fence.flags = VK_FENCE_CREATE_SIGNALED_BIT;
        VkSemaphoreCreateInfo sem{ VK_STRUCTURE_TYPE_SEMAPHORE_CREATE_INFO };
        for (auto& frame : frames) {
            if (vkAllocateCommandBuffers(device, &alloc, &frame.commandBuffer) != VK_SUCCESS
                || vkCreateFence(device, &fence, nullptr, &frame.inFlightFence) != VK_SUCCESS
                || vkCreateSemaphore(device, &sem, nullptr, &frame.acquireSemaphore) != VK_SUCCESS) return false;
        }
        return createPresentObjects(device, imageCount);
    }
    bool createPresentObjects(VkDevice device, size_t imageCount) {
        renderSems.assign(imageCount, VK_NULL_HANDLE);
        presentFences.assign(imageCount, VK_NULL_HANDLE);
        presentPending.assign(imageCount, false);
        VkSemaphoreCreateInfo sem{ VK_STRUCTURE_TYPE_SEMAPHORE_CREATE_INFO };
        VkFenceCreateInfo fence{ VK_STRUCTURE_TYPE_FENCE_CREATE_INFO };
        fence.flags = VK_FENCE_CREATE_SIGNALED_BIT;
        for (size_t i = 0; i < imageCount; ++i) {
            if (vkCreateSemaphore(device, &sem, nullptr, &renderSems[i]) != VK_SUCCESS
                || vkCreateFence(device, &fence, nullptr, &presentFences[i]) != VK_SUCCESS) return false;
        }
        return true;
    }
    VkResult waitForPresent(VkDevice device) const {
        // 渲染完成不等于呈现完成；呈现 fence 才能证明交换链资源可安全退役。
        for (size_t i = 0; i < presentFences.size(); ++i) {
            // 呈现入队失败时 fence 不会被置位，不能等待从未提交的操作。
            if (!presentPending[i]) continue;
            VkFence fence = presentFences[i];
            if (!fence) continue;
            VkResult result = vkWaitForFences(device, 1, &fence, VK_TRUE, UINT64_MAX);
            if (result != VK_SUCCESS) return result;
        }
        return VK_SUCCESS;
    }
    void destroyPresentObjects(VkDevice device) {
        // 调用前必须已等待呈现完成，或已确认设备丢失。
        for (VkSemaphore sem : renderSems) if (sem) vkDestroySemaphore(device, sem, nullptr);
        for (VkFence fence : presentFences) if (fence) vkDestroyFence(device, fence, nullptr);
        renderSems.clear(); presentFences.clear(); presentPending.clear();
    }
    void shutdown(VkDevice device) {
        destroyPresentObjects(device);
        for (auto& frame : frames) {
            if (frame.acquireSemaphore) vkDestroySemaphore(device, frame.acquireSemaphore, nullptr);
            if (frame.inFlightFence) vkDestroyFence(device, frame.inFlightFence, nullptr);
            frame = {};
        }
    }
    FrameData& current() { return frames[currentFrame]; }
    // 本帧提交 fence 完成后，上一轮对 acquire 信号量的等待也已消费。
    VkSemaphore nextAcquireSem() const { return frames[currentFrame].acquireSemaphore; }
    void advance() { currentFrame = (currentFrame + 1) % FRAMES_IN_FLIGHT; }
};
