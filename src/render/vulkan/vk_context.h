
#pragma once
// ==========================================================================
// vk_context.h — Vulkan 基础设施
// Instance / Physical Device / Logical Device / Swapchain / VMA
//
// 使用前提（调用方必须已定义/包含）：
//   #define WIN32_LEAN_AND_MEAN
//   #define NOMINMAX
//   #define GLFW_INCLUDE_VULKAN     ← 必须在第一次 include glfw3.h 之前
//   #include <GLFW/glfw3.h>         ← glfw 会自动 include vulkan/vulkan.h
//   #include <vk_mem_alloc.h>       ← VMA 单头文件
// ==========================================================================

#include <vector>
#include <string>
#include <cstdio>
#include <cstring>
#include <algorithm>

// -----------------------------------------------------------------------
// Validation layer 控制
// -----------------------------------------------------------------------
#ifdef _DEBUG
static constexpr bool kVkEnableValidation = true;
#else
static constexpr bool kVkEnableValidation = false;
#endif
static constexpr const char* kVkValidationLayer = "VK_LAYER_KHRONOS_validation";

// -----------------------------------------------------------------------
// Debug messenger 回调
// -----------------------------------------------------------------------
static VKAPI_ATTR VkBool32 VKAPI_CALL vkDebugMsgCallback(
    VkDebugUtilsMessageSeverityFlagBitsEXT severity,
    VkDebugUtilsMessageTypeFlagsEXT       /*type*/,
    const VkDebugUtilsMessengerCallbackDataEXT* data,
    void* /*user*/)
{
    if (severity >= VK_DEBUG_UTILS_MESSAGE_SEVERITY_WARNING_BIT_EXT)
        fprintf(stderr, "[Vulkan] %s\n", data->pMessage);
    return VK_FALSE;
}

// -----------------------------------------------------------------------
// 辅助结构
// -----------------------------------------------------------------------
struct VkQueueFamilyIndices {
    uint32_t graphics = UINT32_MAX;
    uint32_t present  = UINT32_MAX;
    bool isComplete() const { return graphics != UINT32_MAX && present != UINT32_MAX; }
};

struct VkSwapchainSupport {
    VkSurfaceCapabilitiesKHR        caps{};
    std::vector<VkSurfaceFormatKHR> formats;
    std::vector<VkPresentModeKHR>   presentModes;
};

// -----------------------------------------------------------------------
// VulkanContext — 主上下文
// -----------------------------------------------------------------------
struct VulkanContext {
    // --- Core handles ---
    VkInstance               instance       = VK_NULL_HANDLE;
    VkDebugUtilsMessengerEXT debugMessenger = VK_NULL_HANDLE;
    VkSurfaceKHR             surface        = VK_NULL_HANDLE;
    VkPhysicalDevice         physDevice     = VK_NULL_HANDLE;
    VkDevice                 device         = VK_NULL_HANDLE;

    // --- Queues ---
    VkQueue  graphicsQueue  = VK_NULL_HANDLE;
    VkQueue  presentQueue   = VK_NULL_HANDLE;
    uint32_t graphicsFamily = UINT32_MAX;
    uint32_t presentFamily  = UINT32_MAX;

    // --- VMA allocator ---
    VmaAllocator allocator = VK_NULL_HANDLE;

    // --- Swapchain ---
    VkSwapchainKHR           swapchain = VK_NULL_HANDLE;
    VkFormat                 swapFormat{};
    VkExtent2D               swapExtent{};
    std::vector<VkImage>     swapImages;
    std::vector<VkImageView> swapImageViews;

    // --- Command pool (graphics family) ---
    VkCommandPool commandPool = VK_NULL_HANDLE;

    // --- Info ---
    char deviceName[VK_MAX_PHYSICAL_DEVICE_NAME_SIZE]{};

    // ---------------------------------------------------------------
    // 公开接口
    // ---------------------------------------------------------------

    // Phase 1 用：只初始化 Instance + Device + VMA，不创建 Surface/Swapchain。
    // 原因：Windows 上 WGL (OpenGL) 会锁定窗口 DC，与 Vulkan WSI 冲突，
    //       无法在同一窗口上同时创建 Vulkan surface 和 OpenGL context。
    //       Surface/Swapchain 在 Phase 5 切换为 GLFW_NO_API 窗口后再初始化。
    bool initCore() {
        if (!createInstance())     return false;
        setupDebugMessenger();     // 非致命，失败继续
        if (!pickPhysicalDeviceNoSurface()) return false;
        if (!createLogicalDevice()) return false;
        if (!initVMA())            return false;
        printf("[Vulkan] Core initialized: %s (surface/swapchain deferred to Phase 5)\n", deviceName);
        return true;
    }

    // Phase 5 用：切换 GLFW_NO_API 窗口后调用，创建 Surface + Swapchain + CommandPool
    bool initSwapchain(GLFWwindow* window) {
        if (!createSurface(window))   return false;
        if (!createCommandPool())     return false;
        if (!createSwapchain(window)) return false;
        return true;
    }

    // call before program exit (after vkDeviceWaitIdle is guaranteed)
    void shutdown() {
        if (device != VK_NULL_HANDLE) vkDeviceWaitIdle(device);
        destroySwapchainObjects();
        if (commandPool != VK_NULL_HANDLE)
            vkDestroyCommandPool(device, commandPool, nullptr);
        if (allocator != VK_NULL_HANDLE)
            vmaDestroyAllocator(allocator);
        if (device != VK_NULL_HANDLE)
            vkDestroyDevice(device, nullptr);
        if (kVkEnableValidation && debugMessenger != VK_NULL_HANDLE) {
            auto fn = (PFN_vkDestroyDebugUtilsMessengerEXT)
                vkGetInstanceProcAddr(instance, "vkDestroyDebugUtilsMessengerEXT");
            if (fn) fn(instance, debugMessenger, nullptr);
        }
        if (surface  != VK_NULL_HANDLE) vkDestroySurfaceKHR(instance, surface, nullptr);
        if (instance != VK_NULL_HANDLE) vkDestroyInstance(instance, nullptr);
    }

    // call on window resize — recreates swapchain + image views
    void recreateSwapchain(GLFWwindow* window) {
        // Handle minimized window
        int w = 0, h = 0;
        glfwGetFramebufferSize(window, &w, &h);
        while (w == 0 || h == 0) {
            glfwGetFramebufferSize(window, &w, &h);
            glfwWaitEvents();
        }
        vkDeviceWaitIdle(device);
        destroySwapchainObjects();
        createSwapchain(window);
    }

    bool isValid() const { return device != VK_NULL_HANDLE; }

private:
    // ---------------------------------------------------------------
    // 1. Instance
    // ---------------------------------------------------------------
    bool createInstance() {
        // Validation layer check
        if (kVkEnableValidation) {
            uint32_t layerCount = 0;
            vkEnumerateInstanceLayerProperties(&layerCount, nullptr);
            std::vector<VkLayerProperties> layers(layerCount);
            vkEnumerateInstanceLayerProperties(&layerCount, layers.data());
            bool found = false;
            for (auto& l : layers)
                if (strcmp(l.layerName, kVkValidationLayer) == 0) { found = true; break; }
            if (!found)
                fprintf(stderr, "[Vulkan] Warning: validation layer not found (install Vulkan SDK)\n");
        }

        VkApplicationInfo appInfo{ VK_STRUCTURE_TYPE_APPLICATION_INFO };
        appInfo.pApplicationName   = "RocketSim3D";
        appInfo.applicationVersion = VK_MAKE_VERSION(1, 0, 0);
        appInfo.apiVersion         = VK_API_VERSION_1_3;

        // GLFW 提供必要的 surface extension
        uint32_t glfwExtCount = 0;
        const char** glfwExts = glfwGetRequiredInstanceExtensions(&glfwExtCount);
        std::vector<const char*> extensions(glfwExts, glfwExts + glfwExtCount);
        if (kVkEnableValidation)
            extensions.push_back(VK_EXT_DEBUG_UTILS_EXTENSION_NAME);

        VkInstanceCreateInfo ci{ VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO };
        ci.pApplicationInfo        = &appInfo;
        ci.enabledExtensionCount   = (uint32_t)extensions.size();
        ci.ppEnabledExtensionNames = extensions.data();

        const char* valLayer = kVkValidationLayer;
        if (kVkEnableValidation) {
            ci.enabledLayerCount   = 1;
            ci.ppEnabledLayerNames = &valLayer;
        }

        if (vkCreateInstance(&ci, nullptr, &instance) != VK_SUCCESS) {
            fprintf(stderr, "[Vulkan] Failed to create instance\n");
            return false;
        }
        return true;
    }

    // ---------------------------------------------------------------
    // 2. Debug messenger
    // ---------------------------------------------------------------
    bool setupDebugMessenger() {
        if (!kVkEnableValidation) return true;

        auto fn = (PFN_vkCreateDebugUtilsMessengerEXT)
            vkGetInstanceProcAddr(instance, "vkCreateDebugUtilsMessengerEXT");
        if (!fn) return true;  // 不致命

        VkDebugUtilsMessengerCreateInfoEXT ci{ VK_STRUCTURE_TYPE_DEBUG_UTILS_MESSENGER_CREATE_INFO_EXT };
        ci.messageSeverity = VK_DEBUG_UTILS_MESSAGE_SEVERITY_WARNING_BIT_EXT
                           | VK_DEBUG_UTILS_MESSAGE_SEVERITY_ERROR_BIT_EXT;
        ci.messageType     = VK_DEBUG_UTILS_MESSAGE_TYPE_GENERAL_BIT_EXT
                           | VK_DEBUG_UTILS_MESSAGE_TYPE_VALIDATION_BIT_EXT
                           | VK_DEBUG_UTILS_MESSAGE_TYPE_PERFORMANCE_BIT_EXT;
        ci.pfnUserCallback = vkDebugMsgCallback;
        fn(instance, &ci, nullptr, &debugMessenger);
        return true;
    }

    // ---------------------------------------------------------------
    // 3. Surface（通过 GLFW 创建）
    // ---------------------------------------------------------------
    bool createSurface(GLFWwindow* window) {
        if (glfwCreateWindowSurface(instance, window, nullptr, &surface) != VK_SUCCESS) {
            fprintf(stderr, "[Vulkan] Failed to create window surface\n");
            return false;
        }
        return true;
    }

    // ---------------------------------------------------------------
    // 4a. Physical device 选择（不依赖 surface，Phase 1 用）
    //     只检查 swapchain extension + graphics queue，present queue
    //     留到有 surface 后（Phase 5）再确认。
    // ---------------------------------------------------------------
    bool pickPhysicalDeviceNoSurface() {
        uint32_t count = 0;
        vkEnumeratePhysicalDevices(instance, &count, nullptr);
        if (count == 0) {
            fprintf(stderr, "[Vulkan] No Vulkan-capable GPU found\n");
            return false;
        }
        std::vector<VkPhysicalDevice> devices(count);
        vkEnumeratePhysicalDevices(instance, &count, devices.data());

        int bestScore = -1;
        for (auto& dev : devices) {
            if (!hasSwapchainExtension(dev)) continue;
            uint32_t gfxFamily = findGraphicsFamily(dev);
            if (gfxFamily == UINT32_MAX) continue;

            VkPhysicalDeviceProperties props;
            vkGetPhysicalDeviceProperties(dev, &props);
            int score = 0;
            if (props.deviceType == VK_PHYSICAL_DEVICE_TYPE_DISCRETE_GPU)   score = 1000;
            else if (props.deviceType == VK_PHYSICAL_DEVICE_TYPE_INTEGRATED_GPU) score = 100;
            if (score > bestScore) {
                bestScore      = score;
                physDevice     = dev;
                graphicsFamily = gfxFamily;
                presentFamily  = gfxFamily;  // 临时用 graphics family，Phase 5 会更新
                strncpy_s(deviceName, sizeof(deviceName), props.deviceName, _TRUNCATE);
            }
        }
        if (physDevice == VK_NULL_HANDLE) {
            fprintf(stderr, "[Vulkan] No suitable GPU found\n");
            return false;
        }
        printf("[Vulkan] Selected GPU: %s\n", deviceName);
        return true;
    }

    // 4b. Physical device 选择（有 surface 后调用，Phase 5 用）
    bool pickPhysicalDevice() {
        uint32_t count = 0;
        vkEnumeratePhysicalDevices(instance, &count, nullptr);
        std::vector<VkPhysicalDevice> devices(count);
        vkEnumeratePhysicalDevices(instance, &count, devices.data());

        int bestScore = -1;
        for (auto& dev : devices) {
            if (!isDeviceSuitable(dev)) continue;
            VkPhysicalDeviceProperties props;
            vkGetPhysicalDeviceProperties(dev, &props);
            int score = (props.deviceType == VK_PHYSICAL_DEVICE_TYPE_DISCRETE_GPU) ? 1000 : 100;
            if (score > bestScore) {
                bestScore  = score;
                physDevice = dev;
                strncpy_s(deviceName, sizeof(deviceName), props.deviceName, _TRUNCATE);
            }
        }
        if (physDevice == VK_NULL_HANDLE) { fprintf(stderr, "[Vulkan] No suitable GPU\n"); return false; }
        auto qfi      = findQueueFamilies(physDevice);
        graphicsFamily = qfi.graphics;
        presentFamily  = qfi.present;
        return true;
    }

    bool hasSwapchainExtension(VkPhysicalDevice dev) {
        uint32_t n = 0;
        vkEnumerateDeviceExtensionProperties(dev, nullptr, &n, nullptr);
        std::vector<VkExtensionProperties> exts(n);
        vkEnumerateDeviceExtensionProperties(dev, nullptr, &n, exts.data());
        for (auto& e : exts)
            if (strcmp(e.extensionName, VK_KHR_SWAPCHAIN_EXTENSION_NAME) == 0) return true;
        return false;
    }

    uint32_t findGraphicsFamily(VkPhysicalDevice dev) {
        uint32_t count = 0;
        vkGetPhysicalDeviceQueueFamilyProperties(dev, &count, nullptr);
        std::vector<VkQueueFamilyProperties> families(count);
        vkGetPhysicalDeviceQueueFamilyProperties(dev, &count, families.data());
        for (uint32_t i = 0; i < count; i++)
            if (families[i].queueFlags & VK_QUEUE_GRAPHICS_BIT) return i;
        return UINT32_MAX;
    }

    bool isDeviceSuitable(VkPhysicalDevice dev) {
        if (!hasSwapchainExtension(dev)) return false;
        if (!findQueueFamilies(dev).isComplete()) return false;
        auto sc = querySwapchainSupport(dev);
        return !sc.formats.empty() && !sc.presentModes.empty();
    }

    VkQueueFamilyIndices findQueueFamilies(VkPhysicalDevice dev) {
        VkQueueFamilyIndices qfi;
        uint32_t count = 0;
        vkGetPhysicalDeviceQueueFamilyProperties(dev, &count, nullptr);
        std::vector<VkQueueFamilyProperties> families(count);
        vkGetPhysicalDeviceQueueFamilyProperties(dev, &count, families.data());
        for (uint32_t i = 0; i < count; i++) {
            if (families[i].queueFlags & VK_QUEUE_GRAPHICS_BIT)
                qfi.graphics = i;
            VkBool32 presentSupport = VK_FALSE;
            vkGetPhysicalDeviceSurfaceSupportKHR(dev, i, surface, &presentSupport);
            if (presentSupport) qfi.present = i;
            if (qfi.isComplete()) break;
        }
        return qfi;
    }

    VkSwapchainSupport querySwapchainSupport(VkPhysicalDevice dev) {
        VkSwapchainSupport sc;
        vkGetPhysicalDeviceSurfaceCapabilitiesKHR(dev, surface, &sc.caps);
        uint32_t n = 0;
        vkGetPhysicalDeviceSurfaceFormatsKHR(dev, surface, &n, nullptr);
        if (n) { sc.formats.resize(n); vkGetPhysicalDeviceSurfaceFormatsKHR(dev, surface, &n, sc.formats.data()); }
        vkGetPhysicalDeviceSurfacePresentModesKHR(dev, surface, &n, nullptr);
        if (n) { sc.presentModes.resize(n); vkGetPhysicalDeviceSurfacePresentModesKHR(dev, surface, &n, sc.presentModes.data()); }
        return sc;
    }

    // ---------------------------------------------------------------
    // 5. Logical device
    // ---------------------------------------------------------------
    bool createLogicalDevice() {
        std::vector<uint32_t> uniqueFamilies = { graphicsFamily };
        if (presentFamily != graphicsFamily) uniqueFamilies.push_back(presentFamily);

        float priority = 1.0f;
        std::vector<VkDeviceQueueCreateInfo> queueCIs;
        for (uint32_t family : uniqueFamilies) {
            VkDeviceQueueCreateInfo qci{ VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO };
            qci.queueFamilyIndex = family;
            qci.queueCount       = 1;
            qci.pQueuePriorities = &priority;
            queueCIs.push_back(qci);
        }

        VkPhysicalDeviceFeatures features{};
        features.samplerAnisotropy = VK_TRUE;

        const char* swapExt = VK_KHR_SWAPCHAIN_EXTENSION_NAME;
        VkDeviceCreateInfo ci{ VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO };
        ci.queueCreateInfoCount    = (uint32_t)queueCIs.size();
        ci.pQueueCreateInfos       = queueCIs.data();
        ci.enabledExtensionCount   = 1;
        ci.ppEnabledExtensionNames = &swapExt;
        ci.pEnabledFeatures        = &features;

        if (vkCreateDevice(physDevice, &ci, nullptr, &device) != VK_SUCCESS) {
            fprintf(stderr, "[Vulkan] Failed to create logical device\n");
            return false;
        }
        vkGetDeviceQueue(device, graphicsFamily, 0, &graphicsQueue);
        vkGetDeviceQueue(device, presentFamily,  0, &presentQueue);
        return true;
    }

    // ---------------------------------------------------------------
    // 6. VMA 内存分配器
    // ---------------------------------------------------------------
    bool initVMA() {
        VmaAllocatorCreateInfo vmaCI{};
        vmaCI.physicalDevice   = physDevice;
        vmaCI.device           = device;
        vmaCI.instance         = instance;
        vmaCI.vulkanApiVersion = VK_API_VERSION_1_3;
        if (vmaCreateAllocator(&vmaCI, &allocator) != VK_SUCCESS) {
            fprintf(stderr, "[Vulkan] Failed to create VMA allocator\n");
            return false;
        }
        return true;
    }

    // ---------------------------------------------------------------
    // 7. Swapchain + Image Views
    // ---------------------------------------------------------------
    bool createSwapchain(GLFWwindow* window) {
        auto sc = querySwapchainSupport(physDevice);

        // 选 surface format：优先 BGRA8_UNORM（Windows 最通用）
        VkSurfaceFormatKHR chosen = sc.formats[0];
        for (auto& f : sc.formats)
            if (f.format == VK_FORMAT_B8G8R8A8_UNORM
             && f.colorSpace == VK_COLOR_SPACE_SRGB_NONLINEAR_KHR)
                { chosen = f; break; }
        swapFormat = chosen.format;

        // 选 present mode：MAILBOX（三重缓冲）> FIFO
        VkPresentModeKHR pm = VK_PRESENT_MODE_FIFO_KHR;
        for (auto& p : sc.presentModes)
            if (p == VK_PRESENT_MODE_MAILBOX_KHR) { pm = p; break; }

        // 计算 extent
        if (sc.caps.currentExtent.width != UINT32_MAX) {
            swapExtent = sc.caps.currentExtent;
        } else {
            int w = 0, h = 0;
            glfwGetFramebufferSize(window, &w, &h);
            swapExtent.width  = std::clamp((uint32_t)w,
                sc.caps.minImageExtent.width,  sc.caps.maxImageExtent.width);
            swapExtent.height = std::clamp((uint32_t)h,
                sc.caps.minImageExtent.height, sc.caps.maxImageExtent.height);
        }

        // Triple buffer：minImageCount + 1
        uint32_t imageCount = sc.caps.minImageCount + 1;
        if (sc.caps.maxImageCount > 0 && imageCount > sc.caps.maxImageCount)
            imageCount = sc.caps.maxImageCount;

        VkSwapchainCreateInfoKHR ci{ VK_STRUCTURE_TYPE_SWAPCHAIN_CREATE_INFO_KHR };
        ci.surface          = surface;
        ci.minImageCount    = imageCount;
        ci.imageFormat      = chosen.format;
        ci.imageColorSpace  = chosen.colorSpace;
        ci.imageExtent      = swapExtent;
        ci.imageArrayLayers = 1;
        ci.imageUsage       = VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT;
        ci.preTransform     = sc.caps.currentTransform;
        ci.compositeAlpha   = VK_COMPOSITE_ALPHA_OPAQUE_BIT_KHR;
        ci.presentMode      = pm;
        ci.clipped          = VK_TRUE;

        uint32_t queueFamilyIndices[] = { graphicsFamily, presentFamily };
        if (graphicsFamily != presentFamily) {
            ci.imageSharingMode      = VK_SHARING_MODE_CONCURRENT;
            ci.queueFamilyIndexCount = 2;
            ci.pQueueFamilyIndices   = queueFamilyIndices;
        } else {
            ci.imageSharingMode = VK_SHARING_MODE_EXCLUSIVE;
        }

        if (vkCreateSwapchainKHR(device, &ci, nullptr, &swapchain) != VK_SUCCESS) {
            fprintf(stderr, "[Vulkan] Failed to create swapchain\n");
            return false;
        }

        // 获取 swapchain images
        vkGetSwapchainImagesKHR(device, swapchain, &imageCount, nullptr);
        swapImages.resize(imageCount);
        vkGetSwapchainImagesKHR(device, swapchain, &imageCount, swapImages.data());

        // 创建 image views
        swapImageViews.resize(imageCount);
        for (size_t i = 0; i < imageCount; i++) {
            VkImageViewCreateInfo viewCI{ VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO };
            viewCI.image    = swapImages[i];
            viewCI.viewType = VK_IMAGE_VIEW_TYPE_2D;
            viewCI.format   = swapFormat;
            viewCI.components = {
                VK_COMPONENT_SWIZZLE_IDENTITY, VK_COMPONENT_SWIZZLE_IDENTITY,
                VK_COMPONENT_SWIZZLE_IDENTITY, VK_COMPONENT_SWIZZLE_IDENTITY
            };
            viewCI.subresourceRange.aspectMask     = VK_IMAGE_ASPECT_COLOR_BIT;
            viewCI.subresourceRange.baseMipLevel   = 0;
            viewCI.subresourceRange.levelCount     = 1;
            viewCI.subresourceRange.baseArrayLayer = 0;
            viewCI.subresourceRange.layerCount     = 1;
            if (vkCreateImageView(device, &viewCI, nullptr, &swapImageViews[i]) != VK_SUCCESS) {
                fprintf(stderr, "[Vulkan] Failed to create swapchain image view %zu\n", i);
                return false;
            }
        }
        printf("[Vulkan] Swapchain created: %ux%u, %u images, format=%d, mode=%s\n",
            swapExtent.width, swapExtent.height, imageCount,
            (int)swapFormat, (pm == VK_PRESENT_MODE_MAILBOX_KHR) ? "MAILBOX" : "FIFO");
        return true;
    }

    void destroySwapchainObjects() {
        for (auto& iv : swapImageViews)
            if (iv != VK_NULL_HANDLE) vkDestroyImageView(device, iv, nullptr);
        swapImageViews.clear();
        swapImages.clear();
        if (swapchain != VK_NULL_HANDLE) {
            vkDestroySwapchainKHR(device, swapchain, nullptr);
            swapchain = VK_NULL_HANDLE;
        }
    }

    // ---------------------------------------------------------------
    // 8. Command pool
    // ---------------------------------------------------------------
    bool createCommandPool() {
        VkCommandPoolCreateInfo ci{ VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO };
        ci.queueFamilyIndex = graphicsFamily;
        ci.flags            = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;
        if (vkCreateCommandPool(device, &ci, nullptr, &commandPool) != VK_SUCCESS) {
            fprintf(stderr, "[Vulkan] Failed to create command pool\n");
            return false;
        }
        return true;
    }
};
