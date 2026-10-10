// 中文说明：只提交空白画面，不加载地形、云或计算着色器，用于验证基础设施。
#define VK_USE_PLATFORM_WIN32_KHR
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#define GLFW_INCLUDE_VULKAN
#include <windows.h>
#include <GLFW/glfw3.h>
#include "render/vulkan/vk_context.h"
#include "render/vulkan/vk_frame.h"
#include "render/vulkan/vk_utils.h"
#include "render/vulkan/vk_mesh.h"
#include <cassert>

int main() {
    if (!glfwInit()) return 1;
    glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API);
    glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
    GLFWwindow* window = glfwCreateWindow(96, 96, "Vulkan infrastructure smoke", nullptr, nullptr);
    if (!window) return 2;
    VulkanContext ctx;
    FrameSync sync;
    try {
        if (!ctx.initCore(window) || !ctx.initSwapchain(window)) return 3;
        if (!sync.init(ctx.device, ctx.commandPool, 7)) return 4;
        // 人工使用七张图像的容量验证动态分配，旧实现只有四个槽位。
        assert(sync.renderSems.size() == 7 && sync.presentFences.size() == 7);
        vkRequireSuccess(sync.waitForPresent(ctx.device), "initial present wait");
        sync.destroyPresentObjects(ctx.device);
        if (!sync.createPresentObjects(ctx.device, ctx.swapImages.size())) return 5;

        for (int generation = 0; generation < 3; ++generation) {
            for (int i = 0; i < 6; ++i) {
                auto& frame = sync.current();
                vkRequireSuccess(vkWaitForFences(ctx.device, 1, &frame.inFlightFence, VK_TRUE, UINT64_MAX), "frame fence");
                uint32_t image = 0;
                VkResult acquired = vkAcquireNextImageKHR(ctx.device, ctx.swapchain, 5000000000ULL, frame.acquireSemaphore, VK_NULL_HANDLE, &image);
                if (acquired != VK_SUBOPTIMAL_KHR) vkRequireSuccess(acquired, "acquire");
                vkRequireSuccess(vkResetCommandBuffer(frame.commandBuffer, 0), "reset command");
                VkCommandBufferBeginInfo begin{ VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO };
                vkRequireSuccess(vkBeginCommandBuffer(frame.commandBuffer, &begin), "begin command");
                transitionImage(frame.commandBuffer, ctx.swapImages[image], VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_PRESENT_SRC_KHR,
                    VK_PIPELINE_STAGE_2_NONE, 0, VK_PIPELINE_STAGE_2_NONE, 0);
                vkRequireSuccess(vkEndCommandBuffer(frame.commandBuffer), "end command");
                vkRequireSuccess(vkResetFences(ctx.device, 1, &frame.inFlightFence), "reset frame fence");
                VkPipelineStageFlags stage = VK_PIPELINE_STAGE_ALL_COMMANDS_BIT;
                VkSubmitInfo submit{ VK_STRUCTURE_TYPE_SUBMIT_INFO };
                submit.waitSemaphoreCount = 1; submit.pWaitSemaphores = &frame.acquireSemaphore;
                submit.pWaitDstStageMask = &stage;
                submit.commandBufferCount = 1; submit.pCommandBuffers = &frame.commandBuffer;
                submit.signalSemaphoreCount = 1; submit.pSignalSemaphores = &sync.renderSems[image];
                vkRequireSuccess(vkQueueSubmit(ctx.graphicsQueue, 1, &submit, frame.inFlightFence), "submit");
                VkFence fence = sync.presentFences[image];
                vkRequireSuccess(vkWaitForFences(ctx.device, 1, &fence, VK_TRUE, UINT64_MAX), "previous present");
                vkRequireSuccess(vkResetFences(ctx.device, 1, &fence), "reset present fence");
                VkSwapchainPresentFenceInfoEXT fenceInfo{ VK_STRUCTURE_TYPE_SWAPCHAIN_PRESENT_FENCE_INFO_EXT };
                fenceInfo.swapchainCount = 1; fenceInfo.pFences = &fence;
                VkPresentInfoKHR present{ VK_STRUCTURE_TYPE_PRESENT_INFO_KHR };
                present.pNext = &fenceInfo;
                present.waitSemaphoreCount = 1; present.pWaitSemaphores = &sync.renderSems[image];
                present.swapchainCount = 1; present.pSwapchains = &ctx.swapchain; present.pImageIndices = &image;
                VkResult result = vkQueuePresentKHR(ctx.presentQueue, &present);
                sync.presentPending[image] = result == VK_SUCCESS || result == VK_SUBOPTIMAL_KHR;
                if (result != VK_SUBOPTIMAL_KHR) vkRequireSuccess(result, "present");
                sync.advance();
            }
            // 在实际呈现之后重建，验证旧代信号量及交换链的安全退役顺序。
            vkRequireSuccess(sync.waitForPresent(ctx.device), "retire present");
            glfwSetWindowSize(window, 200 + generation * 40, 140 + generation * 30);
            glfwPollEvents();
            if (!ctx.recreateSwapchain(window)) return 6;
            sync.destroyPresentObjects(ctx.device);
            if (!sync.createPresentObjects(ctx.device, ctx.swapImages.size())) return 7;
        }
        // 上传辅助函数的成功与失败路径：错误必须向上抛出，不能继续录制命令。
        const float vertices[] = { 0, 0, 0, 1, 0, 0, 0, 1, 0 };
        const uint32_t indices[] = { 0, 1, 2 };
        VkMesh mesh;
        assert(mesh.upload(ctx, vertices, sizeof(vertices), indices, 3));
        mesh.destroy(ctx);
        assert(mesh.uploadDynamic(ctx, vertices, sizeof(vertices), indices, 3));
        mesh.destroy(ctx);
        bool caught = false;
        try { vkRequireSuccess(VK_ERROR_OUT_OF_HOST_MEMORY, "injected smoke failure"); }
        catch (const std::runtime_error&) { caught = true; }
        assert(caught);
        vkRequireSuccess(vkDeviceWaitIdle(ctx.device), "device idle");
        sync.shutdown(ctx.device);
        ctx.shutdown();
        assert(g_vkValidationErrorCount == 0);
        glfwDestroyWindow(window);
        glfwTerminate();
        puts("PASS: 7-image capacity, 18 presents, 3 recreations, uploads, failure propagation");
        return 0;
    } catch (const std::exception& error) {
        fprintf(stderr, "FAIL: %s\n", error.what());
        return 8;
    }
}
