#pragma once
// ==========================================================================
// vk_descriptors.h — DescriptorSetLayout / Pool / FrameUBO 管理
//
// 三层设计：
//   Set 0 (per-frame)  : FrameUBO — view/proj/lighting，每帧 memcpy 更新
//   Set 1 (per-material): sampler2D — 主纹理，每材质一个 DescriptorSet
//   Push Constants      : per-object mat4 model + baseColor + params
// ==========================================================================

#include "vk_context.h"
#include "vk_frame.h"
#include <cstring>

// -----------------------------------------------------------------------
// FrameUBO — GLSL std140 layout (matches mesh.vert / mesh.frag Set 0 Binding 0)
// -----------------------------------------------------------------------
struct FrameUBO {
    float view[16];     // 64 bytes
    float proj[16];     // 64 bytes
    float lightDir[3];  // 12 bytes
    float _pad0;        //  4 bytes (vec3 std140 padding)
    float viewPos[3];   // 12 bytes
    float time;         //  4 bytes
};                      // total: 160 bytes

// -----------------------------------------------------------------------
// MeshPushConstants — must match push_constant block in mesh.vert / mesh.frag
// std430: mat4(64) + vec4(16) + vec4(16) + float(4) + int(4) + vec4(16) = 120
struct MeshPushConstants {
    float model[16];       // 64 bytes — offset   0
    float baseColor[4];    // 16 bytes — offset  64
    float planetCenter[4]; // 16 bytes — offset  80（mesh 火箭绘制未使用；星球管线填 center）
    float ambientStr;      //  4 bytes — offset  96
    int   hasTexture;      //  4 bytes — offset 100
    float clipPlane[4];    // 16 bytes — offset 104 (.xyz=world normal, .w=dot(n,p) keep threshold)
};                         //           total: 120 bytes

// -----------------------------------------------------------------------
// PlanetPushConstants — 行星着色器扩展（在 MeshPushConstants 基础上追加
// planetCenter.xyz，供 earth.frag 重建 vLocalPos）
// 104 bytes total (< 128 byte minimum guarantee)
// -----------------------------------------------------------------------
// 字段顺序保证 GLSL std430 与 C++ sizeof 一致（均 104 字节，无隐式 padding）：
//   mat4(64) + vec4 baseColor(16) + vec4 planetCenter(16) + float(4) + int(4)
// 注意：vec4 planetCenter 必须在 float/int 标量之前，否则 GLSL std430 会在
// offset 88 处插入 8 字节 padding 使块大小变为 112 字节（与 C++ 不一致）。
struct PlanetPushConstants {
    float model[16];       // 64 bytes — offset   0
    float baseColor[4];    // 16 bytes — offset  64
    float planetCenter[4]; // 16 bytes — offset  80 (.xyz = center world pos)
    float ambientStr;      //  4 bytes — offset  96
    int   hasTexture;      //  4 bytes — offset 100
};                         //           total: 104 bytes

// -----------------------------------------------------------------------
// SkyboxPushConstants — fullscreen triangle skybox (no vertex buffer)
// 80 bytes: mat4 invViewProj(64) + float skyVibrancy(4) + pad(12)
// -----------------------------------------------------------------------
struct SkyboxPushConstants {
    float invViewProj[16]; // 64 bytes — offset  0
    float skyVibrancy;     //  4 bytes — offset 64
    float _pad[3];         // 12 bytes — offset 68
};                         //            total: 80 bytes

// -----------------------------------------------------------------------
// ExhaustPushConstants — volumetric plume (additive blend)
// 80 bytes: mat4 model(64) + 4×float(16)
// -----------------------------------------------------------------------
struct ExhaustPushConstants {
    float model[16];   // 64 bytes — offset  0
    float throttle;    //  4 bytes — offset 64
    float expansion;   //  4 bytes — offset 68
    float groundDist;  //  4 bytes — offset 72
    float plumeLen;    //  4 bytes — offset 76
};                     //            total: 80 bytes

// -----------------------------------------------------------------------
// RibbonPushConstants — alpha-blended trail ribbon
// 64 bytes: mat4 model only
// -----------------------------------------------------------------------
struct RibbonPushConstants {
    float model[16]; // 64 bytes
};

// -----------------------------------------------------------------------
// BillboardPushConstants — screen-aligned sprite (alpha blend)
// 48 bytes: vec4 center(16) + vec2 size(8) + pad(8) + vec4 color(16)
// -----------------------------------------------------------------------
struct BillboardPushConstants {
    float center[4]; // 16 bytes — offset  0 (vec4 alignment)
    float size[2];   //  8 bytes — offset 16
    float _pad[2];   //  8 bytes — offset 24
    float color[4];  // 16 bytes — offset 32
};                   //            total: 48 bytes

// -----------------------------------------------------------------------
// LensFlarePushConstants — additive 2D lens flare element
// 64 bytes: see layout below
// -----------------------------------------------------------------------
struct LensFlarePushConstants {
    float sunScreenPos[2]; //  8 bytes — offset  0
    float aspect;          //  4 bytes — offset  8
    float intensity;       //  4 bytes — offset 12
    float scale[2];        //  8 bytes — offset 16
    float offset[2];       //  8 bytes — offset 24
    float color[4];        // 16 bytes — offset 32
    int   shapeType;       //  4 bytes — offset 48
    float _pad[3];         // 12 bytes — offset 52
};                         //            total: 64 bytes

// -----------------------------------------------------------------------
// AtmoPushConstants — 大气散射球体（152 bytes，超过 128 字节保证下限——本工程
// 目标硬件是桌面独显（实测 NVIDIA RTX 4060），desktop NVIDIA/AMD 早就普遍支持
// 到 256 字节，这里就不再费事查询 VkPhysicalDeviceLimits::maxPushConstantsSize
// 了，直接用）。
//
// 尾部新增的字段（rayleighCoeff..sunDir）是大气重构（壳内/壳外 LUT 路径）
// 之后唯一的散射系数数据源：C++ 侧从 vk_atmosphere_lut.h::
// getPlanetScatteringCoeffs() 取值填入（见 vk_renderer3d.h），shader 侧
// （atmo_inside.frag / atmo_shell.frag）直接读这些字段，不再各自维护一份
// setupPlanetProfile() 硬编码表。
//
// 已修复：GLSL push_constant 块默认按 std430 布局，vec3 成员的对齐要求是
// 16 字节（4N，和 std140 同一条规则，std430 并没有放松这一点，只放松了数组
// 跨距/结构体对齐）——rayleighCoeff 凑巧落在 offset 64（16 的倍数）所以没
// 出事，但后面的 ozoneCoeff 本该落在 offset 92（不是 16 的倍数），GLSL 编译器
// 会在它前面插 4 字节 padding 补齐到 96，而这个 C++ 结构体（打包时按
// "全部紧密排列、无 padding" 假设手算的偏移）完全不知道这个 padding，导致
// ozoneCoeff 往后的所有字段在 GLSL 侧读到的其实是 C++ 侧偏移 +4 处的数据——
// 相当于 ozoneCoeff/ozoneCenter/ozoneWidth/spaceVisStart/spaceVisEnd/
// limbBrightness 全部错位读串了。改法：C++/GLSL 两侧的 push constant 块都
// 只用标量 float/int（不再用 vec3——vec4 在偏移 0 处天然 16 对齐，不受影响，
// 可以保留），彻底避免这一整类对齐陷阱，不需要每次改字段都重新心算 padding。
// -----------------------------------------------------------------------
struct AtmoPushConstants {
    float planetCenter[4]; // 16 bytes — offset  0 (.xyz = center, .w unused)
    float innerRadius;     //  4 bytes — offset 16 (行星表面半径)
    float outerRadius;     //  4 bytes — offset 20 (大气层外缘)
    float surfaceRadius;   //  4 bytes — offset 24 (同 innerRadius, 用于不同计算)
    int   planetIdx;       //  4 bytes — offset 28 (行星索引: 2=Venus 3=Earth 5=Mars …)
    float sunVisibility;   //  4 bytes — offset 32 (0=夜 1=昼)
    float ringInner;       //  4 bytes — offset 36 (土星环内径, 0=无环)
    float ringOuter;       //  4 bytes — offset 40
    int   frameIndex;      //  4 bytes — offset 44 (TAA 抖动帧号)
    float tuneMinAlt;      //  4 bytes — offset 48 (云层最低高度 km)
    float tuneMaxAlt;      //  4 bytes — offset 52
    float tuneExtinction;  //  4 bytes — offset 56 (云层消光系数)
    float showClouds;      //  4 bytes — offset 60 (0.0=隐藏云, 1.0=显示云)
    float rayleighCoeff[3];//  12 bytes — offset 64 (km^-1，标量数组，非 vec3)
    float mieCoeff;        //  4 bytes  — offset 76 (km^-1)
    float hRayleigh;       //  4 bytes  — offset 80 (标高 km)
    float hMie;             //  4 bytes  — offset 84
    float gMie;             //  4 bytes  — offset 88 (Mie 相位不对称因子)
    float ozoneCoeff[3];    // 12 bytes  — offset 92 (km^-1，标量数组，非 vec3)
    float ozoneCenter;      //  4 bytes  — offset 104 (臭氧层中心高度 km)
    float ozoneWidth;       //  4 bytes  — offset 108
    // 大气重构 tuner 可调参数（imgui_atmo_tuner.h）：
    float spaceVisStart;    //  4 bytes  — offset 112 (altNorm 达到此值开始透出星空)
    float spaceVisEnd;      //  4 bytes  — offset 116 (altNorm 达到此值完全是深空)
    float limbBrightness;   //  4 bytes  — offset 120 (壳外 limb 边缘增强系数，乘在 outerExposure 之上)
    float outerExposure;    //  4 bytes  — offset 124 (壳外整体曝光倍率——壳内有同类的
                             //             exposure=5~10 倍，壳外原来完全没有，raymarch
                             //             算出来的散射量级本来就很小，不乘一个大倍率
                             //             ACES 压完基本看不见，见 imgui_atmo_tuner.h)
    // 已修复：太阳方向不再靠 shader 里现读 frame.lightDir（几何 pass 结束后那个全局
    // UBO 已经被恢复成"火箭→太阳"方向，全景模式相机离火箭很远时和"当前行星→太阳"
    // 方向偏差很大，会让 Transmittance 查表和相位函数都算错，散射偏弱更容易发黑）。
    // 改成 C++ 侧按每颗星球各自算好的"行星→太阳"方向传进来（vk_renderer3d.h::
    // renderAtmoAfterGeometry 里本来就已经算了这个值，只是之前没接上 shader）。
    float sunDir[3];        // 12 bytes  — offset 128 (已归一化，"指向太阳"的方向)
    float innerExposureNear;//  4 bytes  — offset 140 (壳内路径，贴地 altNorm=0 时的曝光倍率)
    float innerExposureFar; //  4 bytes  — offset 144 (壳内路径，接近大气顶 altNorm≥0.6 时的曝光倍率)
    float limbSpaceStart;   //  4 bytes  — offset 148 (壳外 limb 淡入起点 altNorm，默认 1.3)
    float limbSpaceEnd;     //  4 bytes  — offset 152 (壳外 limb 满增益 altNorm，默认 3.0)
    float limbInsideScale;  //  4 bytes  — offset 156 (壳内 limb 相对满增益比例，默认 0.4)
    float limbPower;        //  4 bytes  — offset 160 (rim 指数，默认 2.0)
};                         //            total: 164 bytes

// Terrain patch 专属 UBO（binding 1 of Set 0）
// nodePos/nodeSide/nodeUp 已移至 push constants（每 draw call 更新），此处只保留每帧常量
struct TerrainDataUBO {
    float planetCenterRel[4]; // 16 bytes — planet center relative to camera (per-frame constant)
};

// -----------------------------------------------------------------------
// CloudParamsUBO — GLSL std140 layout (matches cloud.frag Set 0 Binding 1)
// -----------------------------------------------------------------------
struct CloudParamsUBO {
    float uTime;
    float uPhaseSin, uPhaseCos;
    float uCovLo, uCovHi, uThreshLo, uThreshHi;
    float uErosion, uDensity, uExtinction;
    float uMinAlt, uMaxAlt;
    int   uCloudSteps, uLightSteps, uDebug;
    float uLocalRadius;
};

// -----------------------------------------------------------------------
// VkDescriptorManager — owns layouts, pool, and per-frame UBO resources
// -----------------------------------------------------------------------
struct VkDescriptorManager {
    VkDescriptorSetLayout set0Layout = VK_NULL_HANDLE;
    VkDescriptorSetLayout set1Layout = VK_NULL_HANDLE;
    VkDescriptorSetLayout cloudSet0Layout = VK_NULL_HANDLE; // FrameUBO+CloudParams
    VkDescriptorSetLayout cloudTexLayout  = VK_NULL_HANDLE; // 4 cloud textures

    VkDescriptorPool pool = VK_NULL_HANDLE;

    // Per-frame UBO buffers (persistently mapped — no map/unmap per frame)
    VkBuffer      uboBuffers[FRAMES_IN_FLIGHT]{};
    VmaAllocation uboAllocs [FRAMES_IN_FLIGHT]{};
    void*         uboMapped [FRAMES_IN_FLIGHT]{};

    // Pre-allocated Set 0 descriptor sets (one per frame in flight)
    VkDescriptorSet set0[FRAMES_IN_FLIGHT]{};

    // Cloud UBO buffers + descriptor sets
    VkBuffer      cloudUbo[FRAMES_IN_FLIGHT]{};
    VmaAllocation cloudUboAlloc[FRAMES_IN_FLIGHT]{};
    void*         cloudUboMapped[FRAMES_IN_FLIGHT]{};
    VkDescriptorSet cloudSet0[FRAMES_IN_FLIGHT]{};
    VkDescriptorSet cloudTexSet = VK_NULL_HANDLE;

    bool init(VulkanContext& ctx) {
        if (!createLayouts(ctx))      return false;
        if (!createPool(ctx))         return false;
        if (!createUBOBuffers(ctx))   return false;
        if (!createSet0(ctx))         return false;
        if (!createCloudLayouts(ctx)) return false;
        if (!createCloudUBOs(ctx))    return false;
        if (!createCloudSet0(ctx))    return false;
        return true;
    }

    void shutdown(VulkanContext& ctx) {
        for (int i = 0; i < FRAMES_IN_FLIGHT; i++) {
            if (uboBuffers[i] != VK_NULL_HANDLE) {
                vmaDestroyBuffer(ctx.allocator, uboBuffers[i], uboAllocs[i]);
                uboBuffers[i] = VK_NULL_HANDLE;
            }
            if (cloudUbo[i] != VK_NULL_HANDLE) {
                vmaDestroyBuffer(ctx.allocator, cloudUbo[i], cloudUboAlloc[i]);
                cloudUbo[i] = VK_NULL_HANDLE;
            }
        }
        if (pool != VK_NULL_HANDLE) { vkDestroyDescriptorPool(ctx.device, pool, nullptr); pool = VK_NULL_HANDLE; }
        if (set0Layout != VK_NULL_HANDLE) { vkDestroyDescriptorSetLayout(ctx.device, set0Layout, nullptr); set0Layout = VK_NULL_HANDLE; }
        if (set1Layout != VK_NULL_HANDLE) { vkDestroyDescriptorSetLayout(ctx.device, set1Layout, nullptr); set1Layout = VK_NULL_HANDLE; }
        if (cloudSet0Layout != VK_NULL_HANDLE) { vkDestroyDescriptorSetLayout(ctx.device, cloudSet0Layout, nullptr); cloudSet0Layout = VK_NULL_HANDLE; }
        if (cloudTexLayout != VK_NULL_HANDLE) { vkDestroyDescriptorSetLayout(ctx.device, cloudTexLayout, nullptr); cloudTexLayout = VK_NULL_HANDLE; }
    }

    // Call once per frame before recording draw commands
    void updateFrameUBO(int frameIndex, const FrameUBO& data) {
        memcpy(uboMapped[frameIndex], &data, sizeof(FrameUBO));
    }

    // Allocate a Set 1 descriptor set for one texture (caller keeps the VkDescriptorSet)
    VkDescriptorSet allocateTextureSet(VkDevice device, VkImageView view, VkSampler sampler) {
        VkDescriptorSetAllocateInfo ai{ VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO };
        ai.descriptorPool     = pool;
        ai.descriptorSetCount = 1;
        ai.pSetLayouts        = &set1Layout;

        VkDescriptorSet ds = VK_NULL_HANDLE;
        if (vkAllocateDescriptorSets(device, &ai, &ds) != VK_SUCCESS) {
            fprintf(stderr, "[VkDesc] Failed to allocate texture descriptor set\n");
            return VK_NULL_HANDLE;
        }

        VkDescriptorImageInfo imgInfo{};
        imgInfo.imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
        imgInfo.imageView   = view;
        imgInfo.sampler     = sampler;

        VkWriteDescriptorSet write{ VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET };
        write.dstSet          = ds;
        write.dstBinding      = 0;
        write.descriptorType  = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
        write.descriptorCount = 1;
        write.pImageInfo      = &imgInfo;
        vkUpdateDescriptorSets(device, 1, &write, 0, nullptr);
        return ds;
    }

    void updateCloudUBO(int frameIndex, const CloudParamsUBO& data) {
        memcpy(cloudUboMapped[frameIndex], &data, sizeof(CloudParamsUBO));
    }

    VkDescriptorSet allocateCloudTexSet(VkDevice device) {
        VkDescriptorSetAllocateInfo ai{ VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO };
        ai.descriptorPool = pool; ai.descriptorSetCount = 1;
        ai.pSetLayouts = &cloudTexLayout;
        VkDescriptorSet ds = VK_NULL_HANDLE;
        if (vkAllocateDescriptorSets(device, &ai, &ds) != VK_SUCCESS) {
            fprintf(stderr, "[VkDesc] Failed cloud tex set\n"); return VK_NULL_HANDLE;
        }
        return ds;
    }

private:
    bool createLayouts(VulkanContext& ctx) {
        // Set 0: uniform buffer, vertex + fragment stages
        VkDescriptorSetLayoutBinding b0{};
        b0.binding         = 0;
        b0.descriptorType  = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
        b0.descriptorCount = 1;
        b0.stageFlags      = VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT;

        VkDescriptorSetLayoutCreateInfo ci0{ VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO };
        ci0.bindingCount = 1;
        ci0.pBindings    = &b0;
        if (vkCreateDescriptorSetLayout(ctx.device, &ci0, nullptr, &set0Layout) != VK_SUCCESS) {
            fprintf(stderr, "[VkDesc] Failed to create Set 0 layout\n");
            return false;
        }

        // Set 1: combined image sampler, fragment stage only
        VkDescriptorSetLayoutBinding b1{};
        b1.binding         = 0;
        b1.descriptorType  = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
        b1.descriptorCount = 1;
        b1.stageFlags      = VK_SHADER_STAGE_FRAGMENT_BIT;

        VkDescriptorSetLayoutCreateInfo ci1{ VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO };
        ci1.bindingCount = 1;
        ci1.pBindings    = &b1;
        if (vkCreateDescriptorSetLayout(ctx.device, &ci1, nullptr, &set1Layout) != VK_SUCCESS) {
            fprintf(stderr, "[VkDesc] Failed to create Set 1 layout\n");
            return false;
        }
        return true;
    }

    bool createPool(VulkanContext& ctx) {
        // UBO count: FRAMES_IN_FLIGHT (FrameUBO sets) + FRAMES_IN_FLIGHT*2 (terrain set0: 2 bindings each)
        // Image sampler: 64 (textures) + extra headroom
        VkDescriptorPoolSize sizes[2] = {
            { VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER,         FRAMES_IN_FLIGHT * 4 },
            { VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 64 },
        };
        VkDescriptorPoolCreateInfo ci{ VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO };
        ci.maxSets       = FRAMES_IN_FLIGHT * 4 + 64;
        ci.poolSizeCount = 2;
        ci.pPoolSizes    = sizes;
        if (vkCreateDescriptorPool(ctx.device, &ci, nullptr, &pool) != VK_SUCCESS) {
            fprintf(stderr, "[VkDesc] Failed to create descriptor pool\n");
            return false;
        }
        return true;
    }

    bool createUBOBuffers(VulkanContext& ctx) {
        VkBufferCreateInfo bci{ VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO };
        bci.size  = sizeof(FrameUBO);
        bci.usage = VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT;

        VmaAllocationCreateInfo aci{};
        aci.usage = VMA_MEMORY_USAGE_CPU_TO_GPU;
        aci.flags = VMA_ALLOCATION_CREATE_MAPPED_BIT;

        for (int i = 0; i < FRAMES_IN_FLIGHT; i++) {
            VmaAllocationInfo info;
            if (vmaCreateBuffer(ctx.allocator, &bci, &aci,
                                &uboBuffers[i], &uboAllocs[i], &info) != VK_SUCCESS) {
                fprintf(stderr, "[VkDesc] Failed to create UBO buffer %d\n", i);
                return false;
            }
            uboMapped[i] = info.pMappedData;
        }
        return true;
    }

    bool createSet0(VulkanContext& ctx) {
        for (int i = 0; i < FRAMES_IN_FLIGHT; i++) {
            VkDescriptorSetAllocateInfo ai{ VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO };
            ai.descriptorPool     = pool;
            ai.descriptorSetCount = 1;
            ai.pSetLayouts        = &set0Layout;
            if (vkAllocateDescriptorSets(ctx.device, &ai, &set0[i]) != VK_SUCCESS) {
                fprintf(stderr, "[VkDesc] Failed to allocate Set 0[%d]\n", i);
                return false;
            }

            VkDescriptorBufferInfo bufInfo{};
            bufInfo.buffer = uboBuffers[i];
            bufInfo.offset = 0;
            bufInfo.range  = sizeof(FrameUBO);

            VkWriteDescriptorSet write{ VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET };
            write.dstSet          = set0[i];
            write.dstBinding      = 0;
            write.descriptorType  = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
            write.descriptorCount = 1;
            write.pBufferInfo     = &bufInfo;
            vkUpdateDescriptorSets(ctx.device, 1, &write, 0, nullptr);
        }
        return true;
    }

    // ── Cloud-specific ───────────────────────────────────────────────────────
    bool createCloudLayouts(VulkanContext& ctx) {
        // Cloud Set 0: binding 0=FrameUBO, binding 1=CloudParamsUBO
        VkDescriptorSetLayoutBinding cb[2]{};
        cb[0].binding = 0; cb[0].descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
        cb[0].descriptorCount = 1; cb[0].stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;
        cb[1].binding = 1; cb[1].descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
        cb[1].descriptorCount = 1; cb[1].stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;
        VkDescriptorSetLayoutCreateInfo ci0{ VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO };
        ci0.bindingCount = 2; ci0.pBindings = cb;
        if (vkCreateDescriptorSetLayout(ctx.device, &ci0, nullptr, &cloudSet0Layout) != VK_SUCCESS) {
            fprintf(stderr, "[VkDesc] Failed cloud Set0 layout\n"); return false;
        }
        // Cloud Texture Set: 3×sampler3D + 1×sampler2D
        VkDescriptorSetLayoutBinding tb[4]{};
        for (int j = 0; j < 3; j++) {
            tb[j].binding = (uint32_t)j; tb[j].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
            tb[j].descriptorCount = 1; tb[j].stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;
        }
        tb[3].binding = 3; tb[3].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
        tb[3].descriptorCount = 1; tb[3].stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;
        VkDescriptorSetLayoutCreateInfo ci1{ VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO };
        ci1.bindingCount = 4; ci1.pBindings = tb;
        if (vkCreateDescriptorSetLayout(ctx.device, &ci1, nullptr, &cloudTexLayout) != VK_SUCCESS) {
            fprintf(stderr, "[VkDesc] Failed cloud tex layout\n"); return false;
        }
        return true;
    }

    bool createCloudUBOs(VulkanContext& ctx) {
        VkBufferCreateInfo bci{ VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO };
        bci.size = sizeof(CloudParamsUBO); bci.usage = VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT;
        VmaAllocationCreateInfo aci{}; aci.usage = VMA_MEMORY_USAGE_CPU_TO_GPU;
        aci.flags = VMA_ALLOCATION_CREATE_MAPPED_BIT;
        for (int i = 0; i < FRAMES_IN_FLIGHT; i++) {
            VmaAllocationInfo info;
            if (vmaCreateBuffer(ctx.allocator, &bci, &aci, &cloudUbo[i], &cloudUboAlloc[i], &info) != VK_SUCCESS) {
                fprintf(stderr, "[VkDesc] Failed cloud UBO %d\n", i); return false;
            }
            cloudUboMapped[i] = info.pMappedData;
        }
        return true;
    }

    bool createCloudSet0(VulkanContext& ctx) {
        for (int i = 0; i < FRAMES_IN_FLIGHT; i++) {
            VkDescriptorSetAllocateInfo ai{ VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO };
            ai.descriptorPool = pool; ai.descriptorSetCount = 1;
            ai.pSetLayouts = &cloudSet0Layout;
            if (vkAllocateDescriptorSets(ctx.device, &ai, &cloudSet0[i]) != VK_SUCCESS) {
                fprintf(stderr, "[VkDesc] Failed cloud Set0[%d]\n", i); return false;
            }
            VkDescriptorBufferInfo bi[2]{};
            bi[0].buffer = uboBuffers[i]; bi[0].offset = 0; bi[0].range = sizeof(FrameUBO);
            bi[1].buffer = cloudUbo[i];   bi[1].offset = 0; bi[1].range = sizeof(CloudParamsUBO);
            VkWriteDescriptorSet w[2]{};
            w[0].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET; w[0].dstSet = cloudSet0[i];
            w[0].dstBinding = 0; w[0].descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
            w[0].descriptorCount = 1; w[0].pBufferInfo = &bi[0];
            w[1].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET; w[1].dstSet = cloudSet0[i];
            w[1].dstBinding = 1; w[1].descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
            w[1].descriptorCount = 1; w[1].pBufferInfo = &bi[1];
            vkUpdateDescriptorSets(ctx.device, 2, w, 0, nullptr);
        }
        return true;
    }
};
