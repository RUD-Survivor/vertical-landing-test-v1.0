#pragma once
// ==========================================================================
// vk_effects.h — 透明/加法混合特效子系统
//
// 包含管线：
//   ExhaustPipeline   — 体积排气羽流（加法混合，pos-only 顶点 stride=12）
//   RibbonPipeline    — 尾迹带（alpha 混合，stride=32: pos+color+qSide）
//   BillboardPipeline — 广告牌精灵（alpha 混合，vec2 顶点 stride=8）
//   LensFlarePipeline — 镜头光晕（加法混合，无深度，vec2 顶点 stride=8）
//
// 所有特效在不透明几何和天空盒之后绘制。
// ==========================================================================

#include "../vk_context.h"
#include "../vk_descriptors.h"
#include "../vk_pipeline.h"

#include <cstring>
#include <cstdio>

struct VkEffectsSystem {
    VkExhaustPipeline   exhaustPipe;
    VkRibbonPipeline    ribbonPipe;
    VkBillboardPipeline billboardPipe;
    VkLensFlarePipeline lensFlarePipe;

    VkDescriptorManager* desc = nullptr;

    // -----------------------------------------------------------------------
    bool init(VulkanContext& ctx, VkDescriptorManager& d,
              VkFormat colorFmt, VkFormat depthFmt) {
        desc = &d;
        if (!exhaustPipe.init(ctx, d, colorFmt, depthFmt,
                              "src/render/shaders/spirv/exhaust.vert.spv",
                              "src/render/shaders/spirv/exhaust.frag.spv")) {
            fprintf(stderr, "[VkEffects] Failed: exhaust\n"); return false;
        }
        if (!ribbonPipe.init(ctx, d, colorFmt, depthFmt,
                             "src/render/shaders/spirv/ribbon.vert.spv",
                             "src/render/shaders/spirv/ribbon.frag.spv")) {
            fprintf(stderr, "[VkEffects] Failed: ribbon\n"); return false;
        }
        if (!billboardPipe.init(ctx, d, colorFmt, depthFmt,
                                "src/render/shaders/spirv/billboard.vert.spv",
                                "src/render/shaders/spirv/billboard.frag.spv")) {
            fprintf(stderr, "[VkEffects] Failed: billboard\n"); return false;
        }
        if (!lensFlarePipe.init(ctx, colorFmt,
                                "src/render/shaders/spirv/lens_flare.vert.spv",
                                "src/render/shaders/spirv/lens_flare.frag.spv")) {
            fprintf(stderr, "[VkEffects] Failed: lens_flare\n"); return false;
        }
        printf("[VkEffects] Initialized\n");
        return true;
    }

    void shutdown(VkDevice d) {
        exhaustPipe.shutdown(d);
        ribbonPipe.shutdown(d);
        billboardPipe.shutdown(d);
        lensFlarePipe.shutdown(d);
    }

    // -----------------------------------------------------------------------
    // 体积排气羽流。vbo 为 vec3（stride=12）顶点缓冲，带索引缓冲。
    void drawExhaust(VkCommandBuffer cmd, int frameIdx,
                     VkBuffer vbo, VkBuffer ibo, uint32_t icount,
                     const float model[16],
                     float throttle, float expansion, float groundDist, float plumeLen) {
        if (exhaustPipe.pipeline == VK_NULL_HANDLE) return;
        exhaustPipe.bind(cmd);
        vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS,
            exhaustPipe.layout, 0, 1, &desc->set0[frameIdx], 0, nullptr);
        ExhaustPushConstants pc{};
        memcpy(pc.model, model, 64);
        pc.throttle=throttle; pc.expansion=expansion;
        pc.groundDist=groundDist; pc.plumeLen=plumeLen;
        vkCmdPushConstants(cmd, exhaustPipe.layout,
            VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT,
            0, sizeof(ExhaustPushConstants), &pc);
        VkDeviceSize off = 0;
        vkCmdBindVertexBuffers(cmd, 0, 1, &vbo, &off);
        vkCmdBindIndexBuffer(cmd, ibo, 0, VK_INDEX_TYPE_UINT32);
        vkCmdDrawIndexed(cmd, icount, 1, 0, 0, 0);
    }

    // -----------------------------------------------------------------------
    // 尾迹带。vbo 顶点格式: vec3 pos + vec4 color + float qSide（stride=32）。
    void drawRibbon(VkCommandBuffer cmd, int frameIdx,
                    VkBuffer vbo, uint32_t vertCount,
                    const float model[16]) {
        if (ribbonPipe.pipeline == VK_NULL_HANDLE) return;
        ribbonPipe.bind(cmd);
        vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS,
            ribbonPipe.layout, 0, 1, &desc->set0[frameIdx], 0, nullptr);
        RibbonPushConstants pc{};
        memcpy(pc.model, model, 64);
        vkCmdPushConstants(cmd, ribbonPipe.layout,
            VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT,
            0, sizeof(RibbonPushConstants), &pc);
        VkDeviceSize off = 0;
        vkCmdBindVertexBuffers(cmd, 0, 1, &vbo, &off);
        vkCmdDraw(cmd, vertCount, 1, 0, 0);
    }

    // 便捷接口：从 RibbonSegment 数据直接构建 VBO 并绘制
    // 使用预分配的环形 VBO（每次覆盖写入，不创建新 buffer）
    bool initRibbonRing(VulkanContext& ctx, size_t maxVerts = 65536) {
        VkDeviceSize size = maxVerts * 32; // pos(vec3)+color(vec4)+side(float)=32 bytes
        VkBufferCreateInfo bci{VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO};
        bci.size=size; bci.usage=VK_BUFFER_USAGE_VERTEX_BUFFER_BIT|VK_BUFFER_USAGE_TRANSFER_DST_BIT;
        VmaAllocationCreateInfo aci{};
        aci.usage=VMA_MEMORY_USAGE_AUTO;
        aci.flags=VMA_ALLOCATION_CREATE_HOST_ACCESS_SEQUENTIAL_WRITE_BIT;
        if (vmaCreateBuffer(ctx.allocator, &bci, &aci, &ribbonRingVbo, &ribbonRingAlloc, nullptr)!=VK_SUCCESS)
            return false;
        ribbonRingMax = maxVerts;
        ribbonAllocator = ctx.allocator;
        return true;
    }

    void drawRibbonData(VkCommandBuffer cmd, int frameIdx,
                        const std::vector<Vec3>& pts, const std::vector<Vec4>& cols,
                        float width, const Vec3& camPos) {
        if (pts.size()<2 || ribbonRingVbo==VK_NULL_HANDLE) return;
        struct RibV { float px,py,pz, cr,cg,cb,ca, side; };
        size_t n = pts.size();
        size_t total = n*2;
        if (total > ribbonRingMax) return; // 超过环形缓冲容量

        void* data;
        vmaMapMemory(ribbonAllocator, ribbonRingAlloc, &data);
        auto* v = (RibV*)data;
        for (size_t i=0; i<n; i++) {
            Vec4 col = (i<cols.size())?cols[i]:Vec4(1,1,1,1);
            Vec3 fwd;
            if (i<n-1) fwd=(pts[i+1]-pts[i]).normalized();
            else fwd=(pts[i]-pts[i-1]).normalized();
            Vec3 toCam = camPos - pts[i];
            Vec3 right = fwd.cross(toCam).normalized();
            if (right.length()<0.001f) right=Vec3(1,0,0);
            float hw=width*0.5f;
            v[i*2]  ={pts[i].x+right.x*hw,pts[i].y+right.y*hw,pts[i].z+right.z*hw, col.x,col.y,col.z,col.w, 1.f};
            v[i*2+1]={pts[i].x-right.x*hw,pts[i].y-right.y*hw,pts[i].z-right.z*hw, col.x,col.y,col.z,col.w,-1.f};
        }
        vmaUnmapMemory(ribbonAllocator, ribbonRingAlloc);

        float idMat[16]={1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1};
        drawRibbon(cmd, frameIdx, ribbonRingVbo, (uint32_t)total, idMat);
    }

    void shutdownRibbonRing() {
        if (ribbonRingVbo!=VK_NULL_HANDLE && ribbonAllocator!=VK_NULL_HANDLE) {
            vmaDestroyBuffer(ribbonAllocator, ribbonRingVbo, ribbonRingAlloc);
            ribbonRingVbo=VK_NULL_HANDLE;
        }
    }

    VkBuffer      ribbonRingVbo   = VK_NULL_HANDLE;
    VmaAllocation ribbonRingAlloc = VK_NULL_HANDLE;
    size_t        ribbonRingMax   = 0;

    // -----------------------------------------------------------------------
    // 广告牌精灵。vbo 为 4 个 vec2 顶点（quad，stride=8）。
    //   cx/cy/cz   : 世界空间中心点
    //   w/h        : 尺寸（render units）
    void drawBillboard(VkCommandBuffer cmd, int frameIdx,
                       VkBuffer vbo,
                       float cx, float cy, float cz, float w, float h,
                       float r, float g, float b, float a) {
        if (billboardPipe.pipeline == VK_NULL_HANDLE) return;
        billboardPipe.bind(cmd);
        vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS,
            billboardPipe.layout, 0, 1, &desc->set0[frameIdx], 0, nullptr);
        BillboardPushConstants pc{};
        pc.center[0]=cx; pc.center[1]=cy; pc.center[2]=cz; pc.center[3]=1.f;
        pc.size[0]=w; pc.size[1]=h;
        pc.color[0]=r; pc.color[1]=g; pc.color[2]=b; pc.color[3]=a;
        vkCmdPushConstants(cmd, billboardPipe.layout,
            VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT,
            0, sizeof(BillboardPushConstants), &pc);
        VkDeviceSize off = 0;
        vkCmdBindVertexBuffers(cmd, 0, 1, &vbo, &off);
        vkCmdDraw(cmd, 4, 1, 0, 0);
    }

    // -----------------------------------------------------------------------
    // 单个镜头光晕元素（无深度，加法混合）。vbo 为 4 个 vec2 顶点（quad）。
    //   sunSX/SY   : 太阳的屏幕空间位置（NDC -1~1）
    //   scaleX/Y   : 元素尺寸缩放
    //   offX/Y     : 沿光晕轴的位置偏移（NDC）
    //   shapeType  : 0=光晕 1=衍射条纹 2=幽灵圆盘 3=星芒
    void drawLensFlare(VkCommandBuffer cmd,
                       VkBuffer vbo,
                       float sunSX, float sunSY, float aspect, float intensity,
                       float scaleX, float scaleY, float offX, float offY,
                       float r, float g, float b, float a, int shapeType) {
        if (lensFlarePipe.pipeline == VK_NULL_HANDLE) return;
        lensFlarePipe.bind(cmd);
        LensFlarePushConstants pc{};
        pc.sunScreenPos[0]=sunSX; pc.sunScreenPos[1]=sunSY;
        pc.aspect=aspect; pc.intensity=intensity;
        pc.scale[0]=scaleX; pc.scale[1]=scaleY;
        pc.offset[0]=offX; pc.offset[1]=offY;
        pc.color[0]=r; pc.color[1]=g; pc.color[2]=b; pc.color[3]=a;
        pc.shapeType=shapeType;
        vkCmdPushConstants(cmd, lensFlarePipe.layout,
            VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT,
            0, sizeof(LensFlarePushConstants), &pc);
        VkDeviceSize off = 0;
        vkCmdBindVertexBuffers(cmd, 0, 1, &vbo, &off);
        vkCmdDraw(cmd, 4, 1, 0, 0);
    }
};
