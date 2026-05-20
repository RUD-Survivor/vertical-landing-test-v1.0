# 计划：RocketSim3D 图形 API 从 OpenGL 迁移到 Vulkan

## Context

当前项目使用 OpenGL 3.3（通过 GLAD 加载器）+ GLFW 进行所有 3D/2D 渲染，CPU 端每帧有大量 `glUseProgram` / `glUniform*` / 状态切换调用，存在 CPU 开销瓶颈。

迁移目标：用 Vulkan 的 Command Buffer 机制消除这些 CPU 瓶颈，同时保持分阶段可运行，降低风险。辅助库：VMA（内存管理），不引入 vk-bootstrap。

**项目规模提示（工程量巨大）：**
- `src/render/renderer3d.h`：**4544 行全内联代码**，含 14+ 个着色器程序，是核心渲染引擎
- 渲染子系统共 24 个文件（云、地形、植被、SVO、HUD 等）
- 所有 GLSL 着色器以 C++ 字符串字面量形式嵌入，需全部提取并改写为 Vulkan SPIR-V 兼容版本

---

## 分阶段实现计划

每个阶段结束后程序必须可编译运行。

---

### Phase 1：Vulkan 基础设施（双轨并存）

**目标：** Vulkan 初始化成功，但所有像素仍由 OpenGL 渲染。

#### 1.1 新增文件

```
vendor/vma/vk_mem_alloc.h           ← 下载 VMA 单头文件
src/render/vulkan/vk_context.h      ← Instance/Device/Swapchain/Queue 封装
src/render/vulkan/vk_frame.h        ← per-frame 同步对象（Semaphore + Fence）
src/render/vulkan/vk_allocator.cpp  ← VMA 实现（#define VMA_IMPLEMENTATION + include）
```

#### 1.2 vk_context.h 初始化顺序

```
1. glfwGetRequiredInstanceExtensions() → vkCreateInstance (API 1.3)
2. VK_EXT_debug_utils 验证层（仅 _DEBUG 构建）
3. glfwCreateWindowSurface() → VkSurfaceKHR
4. 枚举 Physical Device，优先 DISCRETE_GPU
5. 找 Graphics + Present Queue Family
6. vkCreateDevice（扩展：VK_KHR_SWAPCHAIN）
7. vmaCreateAllocator（VMA 集成）
8. vkCreateSwapchainKHR：
   - 格式：VK_FORMAT_B8G8R8A8_UNORM（Windows 最通用）
   - Present Mode：MAILBOX → FIFO 降级
   - Triple Buffer（minImageCount+1）
9. Command Pool（RESET_COMMAND_BUFFER_BIT）
10. FRAMES_IN_FLIGHT=2 的同步对象：imageAvailableSemaphore / renderFinishedSemaphore / inFlightFence
```

#### 1.3 main.cpp 改动（最小化）

在 `gladLoadGLLoader` 之后添加（`#ifdef USE_VULKAN` 包裹）：
```cpp
VulkanContext vkCtx;
vkCtx.init(window);  // 仅初始化，不参与渲染循环
```

**GLFW 窗口不需要改动**：`glfwCreateWindowSurface` 在已有 OpenGL context 的窗口上可正常调用。

#### 1.4 构建系统（.vcxproj）

- 新增 include 路径：`C:\VulkanSDK\1.3.xxx\Include`
- 新增 lib 路径：`C:\VulkanSDK\1.3.xxx\Lib`
- 新增链接：`vulkan-1.lib`
- 新增编译单元：`src\render\vulkan\vk_allocator.cpp`

---

### Phase 2：基础渲染管线（普通网格 Vulkan 化）

**目标：** `program3d` 对应的不透明网格（火箭组件）改用 Vulkan 渲染。

#### 2.1 GLSL → SPIR-V 修改

**先将所有嵌入 C++ 字符串的 GLSL 提取为独立文件**（放入 `src/render/shaders/`）。

以 `mesh.vert/frag` 为例（对应原 `vertSrc`/`fragSrc`）：

| OpenGL GLSL | Vulkan SPIR-V 修改 |
|---|---|
| `#version 330 core` | 改为 `#version 450` |
| `uniform mat4 uMVP;` | 移入 UBO 或 Push Constants，加 `layout(set=X, binding=Y)` |
| `uniform sampler2D tex;` | 加 `layout(set=1, binding=0)` |
| `bool uHasTexture` uniform | 改为 `int`（SPIR-V 不支持 bool uniform）|
| `gl_Position.y` | proj 矩阵 `[1][1]` 取反（处理 Vulkan Y 轴翻转）|

共享噪声函数（`noise3d`/`fbm`/`hash` 在多个行星着色器中复用）提取到 `shaders/common/noise.glsl`，glslc 用 `-I` 参数 include。

编译脚本写入 `build_fast.bat`：
```bat
%GLSLC% src/render/shaders/mesh.vert -o src/render/shaders/spirv/mesh.vert.spv
```

#### 2.2 Descriptor Set 策略（3 层设计）

```
Set 0 — Per-Frame UBO（每帧一次 memcpy）
  Binding 0: FrameUBO { mat4 view, proj; vec3 lightDir, viewPos; float time; }

Set 1 — Per-Material 纹理
  Binding 0: sampler2D（主纹理）
  Binding 1~4: 云系统 sampler3D（Phase 3 补齐）

Push Constants — Per-Object（< 128 字节，直接嵌入 Command Buffer）
  mat4 model + vec4 baseColor + float ambientStr + padding
  ↑ 替代每次 drawMesh 的 glUniform* 调用，开销最低
```

FrameUBO 用 `VMA_MEMORY_USAGE_CPU_TO_GPU` + 持久映射，避免每帧 map/unmap。

#### 2.3 新增文件

```
src/render/vulkan/vk_mesh.h         ← VkBuffer 替换 VAO/VBO/EBO
src/render/vulkan/vk_pipeline.h     ← VkPipeline 工厂
src/render/vulkan/vk_descriptors.h  ← DescriptorPool/Set 管理
src/render/vulkan/vk_renderpass.h   ← RenderPass + Framebuffer
src/render/shaders/mesh.vert/frag   ← 提取的 Vulkan 版 mesh 着色器
src/render/shaders/common/noise.glsl ← 公共噪声函数库
```

#### 2.4 VkMesh::upload 关键流程

```
1. 创建 Staging Buffer（VMA_MEMORY_USAGE_CPU_ONLY）
2. vmaMapMemory → memcpy 顶点/索引数据 → vmaUnmapMemory
3. 创建 Device Buffer（VMA_MEMORY_USAGE_GPU_ONLY，VERTEX_BUFFER_BIT | TRANSFER_DST_BIT）
4. vkCmdCopyBuffer（录入 Command Buffer）
5. 等待 fence 后销毁 Staging Buffer
```

SVO 动态网格（`svo_meshing.h` 每帧重建）改用 `VMA_MEMORY_USAGE_CPU_TO_GPU`（HOST_VISIBLE，跳过 staging）。

#### 2.5 Vertex Input（直接对应 Vertex3D，56 字节）

```cpp
// binding 0，stride=56，VERTEX_INPUT_RATE_VERTEX
attrs[0] = {0, 0, VK_FORMAT_R32G32B32_FLOAT,    offsetof(Vertex3D, px)};  // pos
attrs[1] = {1, 0, VK_FORMAT_R32G32B32_FLOAT,    offsetof(Vertex3D, nx)};  // normal
attrs[2] = {2, 0, VK_FORMAT_R32G32_FLOAT,       offsetof(Vertex3D, u)};   // uv
attrs[3] = {3, 0, VK_FORMAT_R32G32B32A32_FLOAT, offsetof(Vertex3D, r)};   // color
```

#### 2.6 管线状态（等价于 OpenGL 默认状态）

- Depth Test：`depthTestEnable=TRUE, depthWriteEnable=TRUE, compareOp=LESS`
- Blend：`blendEnable=FALSE`（不透明网格）
- Cull Mode：`VK_CULL_MODE_NONE`（初始，与项目 GL 用法一致）
- **Dynamic State：`VIEWPORT + SCISSOR`**（支持 resize 时无需重建 Pipeline）

**每种混合状态 = 一个 VkPipeline**（Vulkan 与 OpenGL 运行时状态切换不同）：

| 原 OpenGL | Vulkan Pipeline |
|---|---|
| program3d，不透明 | `meshOpaquePipeline` |
| exhaustProg，加法混合 | `exhaustPipeline` |
| atmoProg，预乘 alpha | `atmospherePipeline` |
| ribbonProg，加法 | `ribbonPipeline` |

---

### Phase 3：纹理和全部专用管线（所有星球 Vulkan 渲染）

**目标：** VkImage 替换 glGenTextures，8 个行星着色器 + 地形/植被/SVO/大气/云 全部 Vulkan 化。

#### 3.1 VkTexture2D（替换 Texture::loadTGA）

```
1. Staging Buffer 上传 CPU 像素数据
2. vmaCreateImage（GPU_ONLY，TRANSFER_DST | SAMPLED）
3. transitionLayout: UNDEFINED → TRANSFER_DST_OPTIMAL
4. vkCmdCopyBufferToImage
5. transitionLayout: TRANSFER_DST_OPTIMAL → SHADER_READ_ONLY_OPTIMAL
6. vkCreateImageView + vkCreateSampler（LINEAR + REPEAT + Anisotropy 16x）
```

Image Layout Transition 封装为独立辅助函数（这是 Vulkan 最易错点）。

#### 3.2 3D 纹理（云系统，sampler3D）

与 2D 相同流程，仅 `imageType = VK_IMAGE_TYPE_3D`，`viewType = VK_IMAGE_VIEW_TYPE_3D`，`addressModeW = REPEAT`。

CloudSystem 的 4 个纹理（noiseT=128³，coverT=128³，detailT=32³，weatherT=2048×1024）CPU 烘焙时间可能数秒，建议 Phase 3 中添加后台线程烘焙 + 上传完成前用 1×1×1 占位纹理。

#### 3.3 新增文件

```
src/render/vulkan/vk_texture.h                        ← VkImage/VkImageView/VkSampler 封装
src/render/shaders/terrain.vert/frag
src/render/shaders/atmosphere.vert/frag（含云系统）
src/render/shaders/skybox.vert/frag
src/render/shaders/[earth/mercury/venus/moon/mars/
                    jupiter/saturn/uranus/neptune].frag ← 9 个行星 frag，共享 noise.glsl
```

植被 Pipeline 需第二 Binding（`VK_VERTEX_INPUT_RATE_INSTANCE`）传入 per-instance position/scale/rot，对应原 `treeInstanceVBO`。

使用 `VkPipelineCache` 跨帧缓存 Pipeline 创建（9 个行星 Pipeline 创建开销较大）。

---

### Phase 4：TAA 和后处理（完整 Vulkan 帧循环）

**目标：** 实现等价于 OpenGL TAA 的 Vulkan ping-pong 方案，包含深度重投影。完成后 OpenGL 渲染路径可选择性关闭。

#### 4.1 TAA 两 Pass 方案

**Pass 1 — Geometry Pass（等价于 `beginTAAPass`）：**

```
Color Attachment 0: taaColorImages[currentIdx]
  Format: VK_FORMAT_R16G16B16A16_SFLOAT（等价 GL_RGBA16F）
  LoadOp: CLEAR, StoreOp: STORE
  Layout: UNDEFINED → COLOR_ATTACHMENT_OPTIMAL

Depth Attachment: 共享 D32_SFLOAT（精度高于原 D24，且支持着色器采样）
  LoadOp: CLEAR, StoreOp: STORE（保留，用于重投影）
  Layout: UNDEFINED → DEPTH_STENCIL_ATTACHMENT_OPTIMAL
```

**Pass 2 — TAA Resolve（输出到 Swapchain Image）：**

```
采样 taaColorImages[currentIdx]（当前帧）
采样 taaColorImages[historyIdx]（历史帧，ping-pong 自动保留，无需 glCopyTexSubImage2D）
采样 depthImage（重投影）
混合后写入 Swapchain Image → PRESENT_SRC_KHR
```

两 Pass 间需要 Image Barrier：`COLOR_ATTACHMENT_OPTIMAL → SHADER_READ_ONLY_OPTIMAL`。

#### 4.2 完整帧循环

```
vkWaitForFences → vkAcquireNextImageKHR → vkResetCommandBuffer
→ vkBeginCommandBuffer
  → FrameUBO memcpy
  → [Geometry Pass] 地形/行星/大气/云/天空/耀斑 DrawCall
  → [Image Barriers]
  → [TAA Resolve Pass] vkCmdDraw(3, 1, 0, 0)（全屏三角）
→ vkEndCommandBuffer
→ vkQueueSubmit → vkQueuePresentKHR
→ currentFrame = (currentFrame+1) % FRAMES_IN_FLIGHT
```

#### 4.3 Resize 处理

监听 `VK_ERROR_OUT_OF_DATE_KHR`，触发 `vkDeviceWaitIdle` → 销毁 Swapchain/Framebuffer/TAA Attachments → 重建。Pipeline 因 Dynamic Viewport 无需重建。

#### 4.4 新增文件

```
src/render/vulkan/vk_taa.h  ← TAA ping-pong render targets 管理
src/render/shaders/taa.vert/frag  ← TAA resolve 着色器（Vulkan 版）
```

---

### Phase 5：清理（删除 OpenGL 依赖）

#### 5.1 main.cpp 改动

```cpp
// 删除：
// glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3); ...
// glfwWindowHint(GLFW_SAMPLES, 4);
// glfwMakeContextCurrent(window);
// gladLoadGLLoader(...)
// glEnable(GL_BLEND/GL_MULTISAMPLE)

// 新增：
glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API);

// framebuffer_size_callback 中：删除 glViewport，改为通知 Vulkan resize
```

#### 5.2 构建系统

```xml
<!-- 删除 -->
<ClCompile Include="..\vendor\glad\glad.c" />
<AdditionalDependencies>opengl32.lib;...</AdditionalDependencies>

<!-- 保留 glfw3.lib，添加 vulkan-1.lib -->
```

#### 5.3 renderer_2d.h 处理

保持 API（`Renderer::drawRect/drawText` 等）不变，底层从 GL 替换为 Vulkan 专用 2D RenderPass（在 Swapchain Image 上叠加绘制）。

---

## 关键风险与缓解

| 风险 | 缓解措施 |
|---|---|
| GLSL Y 轴/深度约定差异（OpenGL NDC Y 向上，深度 -1~1 vs Vulkan 相反）| proj 矩阵 `[1][1]` 取反；viewport `minDepth=0, maxDepth=1` |
| 14 个着色器程序需逐一改写 | Phase 2 先提取为独立 `.glsl` 文件 + `#ifdef VULKAN_BACKEND` 保留双版本 |
| 云系统 3D 纹理 CPU 烘焙耗时 | 后台线程烘焙 + 占位纹理，完成后异步 Vulkan upload |
| SVO 动态网格频繁上传 | `VMA_MEMORY_USAGE_CPU_TO_GPU` 跳过 staging，直接 map 写入 |
| renderer3d.h 全内联导致编译时间极长 | Vulkan 新代码放独立 `.cpp` 编译单元，不全内联 |

---

## 关键文件

- [src/main.cpp](src/main.cpp) — Phase 1/5 主要改动
- [src/render/renderer3d.h](src/render/renderer3d.h) — Phase 2~5 逐步替换，Phase 5 清空
- [src/render/renderer_2d.h](src/render/renderer_2d.h) — Phase 5 底层替换
- [src/render/cloud_system.h](src/render/cloud_system.h) — Phase 3 纹理/Uniform 迁移
- [src/render/svo_meshing.h](src/render/svo_meshing.h) — Phase 2 动态 Buffer 迁移
- [vertical-landing-test v1.0/vertical-landing-test v1.0.vcxproj](vertical-landing-test%20v1.0/vertical-landing-test%20v1.0.vcxproj) — Phase 1/5 构建系统

---

## 验证方式

- **Phase 1**：启动后控制台打印 Vulkan 设备名称，验证层无 ERROR 输出，程序渲染正常（仍 OpenGL）
- **Phase 2**：火箭组件（Mesh）可见，切换 `#define USE_VULKAN` 两路对比无视觉差异
- **Phase 3**：所有星球、地形、大气正确渲染，帧率相比 OpenGL 有所提升（CPU 占用降低）
- **Phase 4**：TAA 效果（边缘平滑）与 OpenGL 版主观一致，resize 窗口不崩溃
- **Phase 5**：删除 GLAD 后编译通过，运行正常，RenderDoc/Nsight 可捕获纯 Vulkan 帧
