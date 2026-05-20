// ==========================================================================
// vk_allocator.cpp — VMA 实现编译单元
//
// VMA（Vulkan Memory Allocator）是单头文件库，但需要在且只能在一个
// .cpp 文件中定义 VMA_IMPLEMENTATION 后 include。将它放在独立编译单元
// 可以避免每次修改其他文件时重新展开 ~6000 行 VMA 代码。
//
// 前置条件：vendor/vma/vk_mem_alloc.h 必须已下载
// 下载地址：https://github.com/GPUOpen-LibrariesAndSDKs/VulkanMemoryAllocator/releases
// ==========================================================================

#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>

// Vulkan headers（来自 Vulkan SDK）
#include <vulkan/vulkan.h>

// VMA 实现
#define VMA_IMPLEMENTATION
#include "vma/vk_mem_alloc.h"
