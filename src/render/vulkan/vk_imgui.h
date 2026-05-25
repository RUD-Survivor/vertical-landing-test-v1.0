#pragma once
// ==========================================================================
// vk_imgui.h — Dear ImGui Vulkan 1.3 动态渲染集成
// 使用方式（main.cpp）：
//   g_vkImGui.init(g_vkCtx, window, g_vkCtx.swapFormat)
//   每帧: g_vkImGui.newFrame()  → ImGui::...  → ImGui::Render()
//         g_vkImGui.renderToSwapchain(cmd, swapImage, swapView, extent)
//   退出: g_vkImGui.shutdown(g_vkCtx.device)
// ==========================================================================
#include "vk_context.h"
#include "vk_renderpass.h"  // transitionImage / transitionToPresent

// ImGui headers（vendor/imgui 已在 include 路径中）
#include "imgui.h"
#include "backends/imgui_impl_glfw.h"
#include "backends/imgui_impl_vulkan.h"
#include <GLFW/glfw3.h>
#include <cstdio>

struct VkImGuiSystem {
    VkDescriptorPool descPool = VK_NULL_HANDLE;
    bool             initialized = false;

    // ----------------------------------------------------------------
    bool init(VulkanContext& ctx, GLFWwindow* window, VkFormat swapFmt) {
        // 为 ImGui 创建专用 descriptor pool
        VkDescriptorPoolSize poolSizes[] = {
            { VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 16 }
        };
        VkDescriptorPoolCreateInfo pi{VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO};
        pi.flags         = VK_DESCRIPTOR_POOL_CREATE_FREE_DESCRIPTOR_SET_BIT;
        pi.maxSets       = 16;
        pi.poolSizeCount = 1;
        pi.pPoolSizes    = poolSizes;
        if (vkCreateDescriptorPool(ctx.device, &pi, nullptr, &descPool) != VK_SUCCESS) {
            fprintf(stderr, "[ImGui] Failed to create descriptor pool\n");
            return false;
        }

        IMGUI_CHECKVERSION();
        ImGui::CreateContext();
        ImGuiIO& io = ImGui::GetIO();
        io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;

        // 中文字体（SourceHanSansCN）
        ImFontConfig cfg;
        cfg.OversampleH = 2; cfg.OversampleV = 2;
        static const ImWchar ranges[] = {
            0x0020, 0x007E,  // ASCII
            0x4E00, 0x9FFF,  // CJK 常用汉字
            0
        };
        const char* fontPath = "assets/fonts/SourceHanSansCN-Regular.otf";
        ImFont* fnt = io.Fonts->AddFontFromFileTTF(fontPath, 18.0f, &cfg, ranges);
        if (!fnt) {
            // 回退：使用内置字体
            io.Fonts->AddFontDefault();
            fprintf(stderr, "[ImGui] Font not found at %s, using default\n", fontPath);
        }

        // 深色主题 + 半透明 HUD 风格
        ImGui::StyleColorsDark();
        ImGuiStyle& style = ImGui::GetStyle();
        style.WindowRounding    = 6.0f;
        style.FrameRounding     = 4.0f;
        style.WindowBorderSize  = 1.0f;
        style.Alpha             = 0.92f;
        style.Colors[ImGuiCol_WindowBg]      = ImVec4(0.06f, 0.06f, 0.10f, 0.90f);
        style.Colors[ImGuiCol_TitleBg]       = ImVec4(0.05f, 0.20f, 0.35f, 1.00f);
        style.Colors[ImGuiCol_TitleBgActive] = ImVec4(0.08f, 0.30f, 0.50f, 1.00f);
        style.Colors[ImGuiCol_Button]        = ImVec4(0.15f, 0.45f, 0.70f, 0.85f);
        style.Colors[ImGuiCol_ButtonHovered] = ImVec4(0.25f, 0.60f, 0.85f, 1.00f);
        style.Colors[ImGuiCol_ButtonActive]  = ImVec4(0.10f, 0.35f, 0.60f, 1.00f);
        style.Colors[ImGuiCol_Header]        = ImVec4(0.15f, 0.40f, 0.65f, 0.80f);
        style.Colors[ImGuiCol_HeaderHovered] = ImVec4(0.20f, 0.55f, 0.80f, 0.90f);

        // GLFW 后端
        ImGui_ImplGlfw_InitForVulkan(window, true);

        // Vulkan 后端（Vulkan 1.3 动态渲染，ImGui post-2025/09/26 API）
        ImGui_ImplVulkan_InitInfo vi{};
        vi.ApiVersion      = VK_API_VERSION_1_3;
        vi.Instance        = ctx.instance;
        vi.PhysicalDevice  = ctx.physDevice;
        vi.Device          = ctx.device;
        vi.QueueFamily     = ctx.graphicsFamily;
        vi.Queue           = ctx.graphicsQueue;
        vi.DescriptorPool  = descPool;
        vi.MinImageCount   = 2;
        vi.ImageCount      = (uint32_t)ctx.swapImages.size();
        vi.UseDynamicRendering = true;

        // PipelineInfoMain 替代了旧的平铺字段
        vi.PipelineInfoMain.MSAASamples = VK_SAMPLE_COUNT_1_BIT;
#ifdef IMGUI_IMPL_VULKAN_HAS_DYNAMIC_RENDERING
        vi.PipelineInfoMain.PipelineRenderingCreateInfo.sType =
            VK_STRUCTURE_TYPE_PIPELINE_RENDERING_CREATE_INFO;
        vi.PipelineInfoMain.PipelineRenderingCreateInfo.colorAttachmentCount = 1;
        vi.PipelineInfoMain.PipelineRenderingCreateInfo.pColorAttachmentFormats = &swapFmt;
#endif

        if (!ImGui_ImplVulkan_Init(&vi)) {
            fprintf(stderr, "[ImGui] ImGui_ImplVulkan_Init failed\n");
            return false;
        }
        // 字体纹理由 ImGui_ImplVulkan_Init 内部自动上传（post-2025/09/26）

        initialized = true;
        printf("[ImGui] Initialized (Vulkan dynamic rendering)\n");
        return true;
    }

    // ----------------------------------------------------------------
    void shutdown(VkDevice device) {
        if (!initialized) return;
        ImGui_ImplVulkan_Shutdown();
        ImGui_ImplGlfw_Shutdown();
        ImGui::DestroyContext();
        if (descPool != VK_NULL_HANDLE)
            vkDestroyDescriptorPool(device, descPool, nullptr);
        initialized = false;
    }

    // ----------------------------------------------------------------
    // 每帧开始前调用（在提交任何 ImGui draw call 之前）
    void newFrame() {
        ImGui_ImplVulkan_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();
    }

    // ----------------------------------------------------------------
    // ImGui::Render() 之后调用，将绘制数据渲染到 swapchain image
    // swapchain image 进入时应处于 PRESENT_SRC_KHR（由 TAA resolve 留下的状态）
    void renderToSwapchain(VkCommandBuffer cmd,
                           VkImage swapImage, VkImageView swapView,
                           VkExtent2D extent)
    {
        if (!initialized) return;

        // PRESENT_SRC_KHR → COLOR_ATTACHMENT_OPTIMAL
        transitionImage(cmd, swapImage,
            VK_IMAGE_LAYOUT_PRESENT_SRC_KHR,
            VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL,
            VK_PIPELINE_STAGE_2_BOTTOM_OF_PIPE_BIT, VK_ACCESS_2_NONE,
            VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT,
            VK_ACCESS_2_COLOR_ATTACHMENT_WRITE_BIT | VK_ACCESS_2_COLOR_ATTACHMENT_READ_BIT);

        VkRenderingAttachmentInfo ai{VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO};
        ai.imageView   = swapView;
        ai.imageLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
        ai.loadOp      = VK_ATTACHMENT_LOAD_OP_LOAD;   // 保留 TAA/HUD 已渲染内容
        ai.storeOp     = VK_ATTACHMENT_STORE_OP_STORE;

        VkRenderingInfo ri{VK_STRUCTURE_TYPE_RENDERING_INFO};
        ri.renderArea             = {{0, 0}, extent};
        ri.layerCount             = 1;
        ri.colorAttachmentCount   = 1;
        ri.pColorAttachments      = &ai;
        vkCmdBeginRendering(cmd, &ri);

        ImGui_ImplVulkan_RenderDrawData(ImGui::GetDrawData(), cmd);

        vkCmdEndRendering(cmd);

        // COLOR_ATTACHMENT_OPTIMAL → PRESENT_SRC_KHR（供 vkQueuePresentKHR）
        transitionToPresent(cmd, swapImage);
    }
};
