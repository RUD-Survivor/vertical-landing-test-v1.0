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
#include "../../math/math3d.h"  // Vec3, Vec4
#include <cmath>
#include <cstring>
#include <cstdio>

struct VkEffectsSystem {
    VkExhaustPipeline   exhaustPipe;
    VkRibbonPipeline    ribbonPipe;
    VkBillboardPipeline billboardPipe;
    VkBillboardPipeline billboardOverlayPipe;
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
                                "src/render/shaders/spirv/billboard.frag.spv",
                                true)) {
            fprintf(stderr, "[VkEffects] Failed: billboard\n"); return false;
        }
        if (!billboardOverlayPipe.init(ctx, d, colorFmt, depthFmt,
                                       "src/render/shaders/spirv/billboard.vert.spv",
                                       "src/render/shaders/spirv/billboard.frag.spv",
                                       false)) {
            fprintf(stderr, "[VkEffects] Failed: billboard overlay\n"); return false;
        }
        if (!lensFlarePipe.init(ctx, colorFmt, depthFmt,
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
        billboardOverlayPipe.shutdown(d);
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
    // coreEdge: 视觉核心在 |qSide| 中的外缘（几何含 AA 余量时 < 1）。
    void drawRibbon(VkCommandBuffer cmd, int frameIdx,
                    VkBuffer vbo, uint32_t vertCount,
                    const float model[16], VkDeviceSize vboOffset = 0,
                    float coreEdge = 1.0f) {
        if (ribbonPipe.pipeline == VK_NULL_HANDLE) return;
        ribbonPipe.bind(cmd);
        vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS,
            ribbonPipe.layout, 0, 1, &desc->set0[frameIdx], 0, nullptr);
        RibbonPushConstants pc{};
        memcpy(pc.model, model, 64);
        pc.coreEdge = coreEdge;
        vkCmdPushConstants(cmd, ribbonPipe.layout,
            VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT,
            0, sizeof(RibbonPushConstants), &pc);
        vkCmdBindVertexBuffers(cmd, 0, 1, &vbo, &vboOffset);
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

    // widthPixels: 目标屏幕像素宽度。几何额外加宽 aaPad 供外侧羽化（细线 AA 必需）。
    // 逐点按相机距离换算世界半宽，使轨道线屏上近似定宽。
    void drawRibbonData(VkCommandBuffer cmd, int frameIdx,//cmd:命令缓存，frameIdx:帧索引
                        const std::vector<Vec3>& pts, const std::vector<Vec4>& cols,//pts:轨道线中心点，cols:逐点颜色
                        float widthPixels, const Vec3& camPos,//widthPixels:ribbon 在屏幕上的目标宽度，camPos:相机位置
                        float viewportHeight, float fovY = 0.8f)//viewportHeight:视口高度，fovY:竖直视场角
    {
        if (pts.size()<2 || ribbonRingVbo==VK_NULL_HANDLE) return;//至少两点才能成带；环形 VBO 未初始化则跳过
        if (viewportHeight < 1.f) viewportHeight = 1.f;//防止除零
        if (widthPixels < 0.5f) widthPixels = 0.5f;//过细没有覆盖率，钳到半像素
        struct RibV { float px,py,pz, cr,cg,cb,ca, side; };//pos + color + qSide，与 stride=32 对齐
        size_t n = pts.size();
        size_t total = n*2;//每个中心点挤出左右两个顶点
        if (ringOffset + total > ribbonRingMax) return; // 超出环形缓冲

        // 两侧合计约 2px 的 AA 余量；视觉宽度仍为 widthPixels
        constexpr float kAaPadPx = 2.0f;//几何比视觉多出的屏上像素宽（外侧羽化带）
        const float geomPixels = widthPixels + kAaPadPx;//实际挤出用的几何宽度
        const float coreEdge = widthPixels / geomPixels;//|qSide|<coreEdge 为不透明核心，之外为 AA 区

        const float tanHalf = tanf(fovY * 0.5f);//半角 tan，用于透视下“单位距离竖直可见高度”
        // (geomPixels/viewportHeight)：几何宽占屏高比例；×2*tanHalf：单位距离上的世界宽度系数
        const float pxScale = (geomPixels / viewportHeight) * 2.f * tanHalf;

        void* data;
        vmaMapMemory(ribbonAllocator, ribbonRingAlloc, &data);//映射环形 VBO，CPU 可写
        auto* v = (RibV*)data + ringOffset;//当前帧写入游标
        for (size_t i=0; i<n; i++) {
            Vec4 col = (i<cols.size())?cols[i]:Vec4(1,1,1,1);//缺色时用白
            Vec3 fwd;//轨道局部切线
            if (i<n-1) fwd=(pts[i+1]-pts[i]).normalized();//前向差分
            else fwd=(pts[i]-pts[i-1]).normalized();//末点用后向差分
            Vec3 toCam = camPos - pts[i];//点指向相机
            float dist = toCam.length();//到相机距离，决定该点世界线宽
            if (dist < 1e-6f) dist = 1e-6f;
            Vec3 right = fwd.cross(toCam).normalized();//带宽方向：垂直于切线且朝向相机侧
            if (right.length()<0.001f) right=Vec3(1,0,0);//切线几乎朝向相机时叉积退化，兜底
            // 屏上定宽：worldW ∝ dist；相对 dist 的比例 clamp 防贴相机炸开 / 远距过粗
            float worldW = pxScale * dist;//该点处几何丝带的世界全宽
            float minW = dist * 1e-5f;//最小宽度
            float maxW = dist * 0.05f;//最大宽度
            if (worldW < minW) worldW = minW;
            if (worldW > maxW) worldW = maxW;
            float hw = worldW * 0.5f;//半宽，左右各挤 hw
            v[i*2]  ={(float)(pts[i].x+right.x*hw),(float)(pts[i].y+right.y*hw),(float)(pts[i].z+right.z*hw), (float)col.x,(float)col.y,(float)col.z,(float)col.w, 1.f};//左侧，qSide=+1
            v[i*2+1]={(float)(pts[i].x-right.x*hw),(float)(pts[i].y-right.y*hw),(float)(pts[i].z-right.z*hw), (float)col.x,(float)col.y,(float)col.z,(float)col.w,-1.f};//右侧，qSide=-1
        }
        vmaUnmapMemory(ribbonAllocator, ribbonRingAlloc);//写完解映射，交 GPU 读

        float idMat[16]={1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1};//单位矩阵：顶点已是世界坐标
        VkDeviceSize voff = ringOffset * 32; // 当前段在 VBO 中的字节偏移，每顶点 32 字节
        drawRibbon(cmd, frameIdx, ribbonRingVbo, (uint32_t)total, idMat, voff, coreEdge);//提交绘制，传入核心边缘
        ringOffset += total; // 推进偏移，下一条 ribbon 写到后面
    }

    void beginRibbonFrame() { ringOffset = 0; } // 每帧开始时重置

    void shutdownRibbonRing() {
        if (ribbonRingVbo!=VK_NULL_HANDLE && ribbonAllocator!=VK_NULL_HANDLE) {
            vmaDestroyBuffer(ribbonAllocator, ribbonRingVbo, ribbonRingAlloc);
            ribbonRingVbo=VK_NULL_HANDLE;
        }
    }

    VkBuffer      ribbonRingVbo   = VK_NULL_HANDLE;
    VmaAllocation ribbonRingAlloc = VK_NULL_HANDLE;
    VmaAllocator  ribbonAllocator = VK_NULL_HANDLE;
    size_t        ribbonRingMax   = 0;
    size_t        ringOffset      = 0;  // 当前帧写入偏移

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

    void drawBillboardOverlay(VkCommandBuffer cmd, int frameIdx,
                              VkBuffer vbo,
                              float cx, float cy, float cz, float w, float h,
                              float r, float g, float b, float a) {
        if (billboardOverlayPipe.pipeline == VK_NULL_HANDLE) return;
        billboardOverlayPipe.bind(cmd);
        vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS,
            billboardOverlayPipe.layout, 0, 1, &desc->set0[frameIdx], 0, nullptr);
        BillboardPushConstants pc{};
        pc.center[0]=cx; pc.center[1]=cy; pc.center[2]=cz; pc.center[3]=1.f;
        pc.size[0]=w; pc.size[1]=h;
        pc.color[0]=r; pc.color[1]=g; pc.color[2]=b; pc.color[3]=a;
        vkCmdPushConstants(cmd, billboardOverlayPipe.layout,
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
//ribbon,billboard,lens_flare,exhaust全部挤在一起