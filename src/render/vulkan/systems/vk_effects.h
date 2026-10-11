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
#include "../../solar_optics.h"
#include "../vk_descriptors.h"
#include "../vk_pipeline.h"
#include "../vk_texture.h"
#include <vector>
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
    VkLensFlarePipeline solarDiskPipe;
    VkDescriptorSetLayout solarInputLayout=VK_NULL_HANDLE;
    VkDescriptorPool solarInputPool=VK_NULL_HANDLE;
    VkDescriptorSet solarInputSets[2]={};
    VkDescriptorSet solarDiskSet=VK_NULL_HANDLE;
    VkTexture2D solarResponse;
    VkTexture2D solarFieldResponse[2];
    bool solarFieldInitialized[2]={};
    VkPipeline solarResponsePipeline=VK_NULL_HANDLE;
    VulkanContext* solarContext=nullptr;
    VkBuffer solarSourceBuffers[2]={};
    VmaAllocation solarSourceAllocations[2]={};
    VkPipeline solarSourcePipeline=VK_NULL_HANDLE;
    VkPipelineLayout solarSourcePipelineLayout=VK_NULL_HANDLE;

    VkDescriptorManager* desc = nullptr;

    // -----------------------------------------------------------------------
    bool init(VulkanContext& ctx, VkDescriptorManager& d,
              VkFormat colorFmt, VkFormat depthFmt) {
        desc = &d;
        solarContext=&ctx;
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
        VkDescriptorSetLayoutBinding bindings[]={{0,VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,1,VK_SHADER_STAGE_COMPUTE_BIT,nullptr},
            {1,VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,1,VK_SHADER_STAGE_COMPUTE_BIT,nullptr},
            {2,VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,1,VK_SHADER_STAGE_COMPUTE_BIT|VK_SHADER_STAGE_FRAGMENT_BIT,nullptr},
            {3,VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,1,VK_SHADER_STAGE_COMPUTE_BIT|VK_SHADER_STAGE_VERTEX_BIT|VK_SHADER_STAGE_FRAGMENT_BIT,nullptr},
            {4,VK_DESCRIPTOR_TYPE_STORAGE_IMAGE,1,VK_SHADER_STAGE_COMPUTE_BIT,nullptr},
            {5,VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,1,VK_SHADER_STAGE_FRAGMENT_BIT,nullptr}};
        // Generated immutable linear-HDR optical response, not an sRGB sprite.
        FILE* responseFile=fopen("assets/data/solar/aperture-response.rgba32f","rb");
        if(!responseFile){fprintf(stderr,"[VkEffects] Missing solar response; run tools/build_solar_response.py\n");return false;}
        uint32_t header[3]={};
        if(fread(header,sizeof(header),1,responseFile)!=1 || header[0]!=0x53505347 || header[1]!=1024 || header[2]!=3072){
            fclose(responseFile);fprintf(stderr,"[VkEffects] Invalid solar response header\n");return false;}
        float halfExtent=0;
        if(fread(&halfExtent,sizeof(halfExtent),1,responseFile)!=1||!std::isfinite(halfExtent)||
            std::abs(halfExtent-SolarOptics::responseHalfExtent)>1e-7){
            fclose(responseFile);fprintf(stderr,"[VkEffects] Stale solar response extent; run tools/build_solar_response.py\n");return false;}
        std::vector<float> responseData(size_t(header[1])*header[2]*4);
        bool complete=fread(responseData.data(),sizeof(float),responseData.size(),responseFile)==responseData.size();
        complete=complete&&fgetc(responseFile)==EOF;fclose(responseFile);
        for(float value:responseData)if(!std::isfinite(value)||value<0)complete=false;
        if(!complete || !solarResponse.uploadFloat4(ctx,responseData.data(),header[1],header[2]))return false;
        VkDescriptorSetLayoutCreateInfo li{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO};li.bindingCount=6;li.pBindings=bindings;
        vkRequireSuccess(vkCreateDescriptorSetLayout(ctx.device,&li,nullptr,&solarInputLayout),"solar input layout");
        VkDescriptorPoolSize ps[]={{VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,12},{VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,3},{VK_DESCRIPTOR_TYPE_STORAGE_IMAGE,3}};
        VkDescriptorPoolCreateInfo pi{VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO};pi.maxSets=3;pi.poolSizeCount=3;pi.pPoolSizes=ps;
        vkRequireSuccess(vkCreateDescriptorPool(ctx.device,&pi,nullptr,&solarInputPool),"solar input pool");
        VkDescriptorSetLayout layouts[]={solarInputLayout,solarInputLayout};
        VkDescriptorSetAllocateInfo ai{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO};ai.descriptorPool=solarInputPool;ai.descriptorSetCount=2;ai.pSetLayouts=layouts;
        vkRequireSuccess(vkAllocateDescriptorSets(ctx.device,&ai,solarInputSets),"solar inputs");
        ai.descriptorSetCount=1;
        vkRequireSuccess(vkAllocateDescriptorSets(ctx.device,&ai,&solarDiskSet),"solar disk input");
        VkDescriptorImageInfo kernelImage{solarResponse.sampler,solarResponse.view,VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL};
        for(auto set:{solarInputSets[0],solarInputSets[1],solarDiskSet})for(int binding=0;binding<3;++binding){
            VkWriteDescriptorSet write{VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
            write.dstSet=set;write.dstBinding=binding;write.descriptorCount=1;write.descriptorType=VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
            write.pImageInfo=&kernelImage;vkUpdateDescriptorSets(ctx.device,1,&write,0,nullptr);}
        for(int slot=0;slot<2;++slot){
            // RGBA32F is required: normalized diffraction densities can exceed
            // FP16 even though the final radiometric optical image does not.
            if(!solarFieldResponse[slot].createStorage2D(ctx,1024,1024,VK_FORMAT_R32G32B32A32_SFLOAT))return false;
            VkDescriptorImageInfo fieldImages[]={{VK_NULL_HANDLE,solarFieldResponse[slot].view,VK_IMAGE_LAYOUT_GENERAL},
                {solarFieldResponse[slot].sampler,solarFieldResponse[slot].view,VK_IMAGE_LAYOUT_GENERAL}};
            for(int binding=4;binding<=5;++binding){VkWriteDescriptorSet fieldWrite{VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
                fieldWrite.dstSet=solarInputSets[slot];fieldWrite.dstBinding=binding;fieldWrite.descriptorCount=1;
                fieldWrite.descriptorType=binding==4?VK_DESCRIPTOR_TYPE_STORAGE_IMAGE:VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
                fieldWrite.pImageInfo=&fieldImages[binding-4];vkUpdateDescriptorSets(ctx.device,1,&fieldWrite,0,nullptr);
                // The photosphere is drawn before field computation. Its
                // statically declared sampler must reference a valid image
                // even though the disk branch does not sample that binding.
                if(slot==0){fieldWrite.dstSet=solarDiskSet;
                    if(binding==5)fieldWrite.pImageInfo=&kernelImage;
                    vkUpdateDescriptorSets(ctx.device,1,&fieldWrite,0,nullptr);}}
            VkBufferCreateInfo buffer{VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO};buffer.size=258*16;buffer.usage=VK_BUFFER_USAGE_STORAGE_BUFFER_BIT;
            VmaAllocationCreateInfo allocation{};allocation.usage=VMA_MEMORY_USAGE_GPU_ONLY;
            vkRequireSuccess(vmaCreateBuffer(ctx.allocator,&buffer,&allocation,&solarSourceBuffers[slot],&solarSourceAllocations[slot],nullptr),"solar source cache");
            VkDescriptorBufferInfo info{solarSourceBuffers[slot],0,VK_WHOLE_SIZE};
            VkWriteDescriptorSet write{VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};write.dstSet=solarInputSets[slot];write.dstBinding=3;
            write.descriptorCount=1;write.descriptorType=VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;write.pBufferInfo=&info;
            vkUpdateDescriptorSets(ctx.device,1,&write,0,nullptr);
            if(slot==0){write.dstSet=solarDiskSet;vkUpdateDescriptorSets(ctx.device,1,&write,0,nullptr);}
        }
        VkPushConstantRange computeRange{VK_SHADER_STAGE_COMPUTE_BIT,0,sizeof(LensFlarePushConstants)};
        VkPipelineLayoutCreateInfo computeLayout{VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO};computeLayout.setLayoutCount=1;
        computeLayout.pSetLayouts=&solarInputLayout;computeLayout.pushConstantRangeCount=1;computeLayout.pPushConstantRanges=&computeRange;
        vkRequireSuccess(vkCreatePipelineLayout(ctx.device,&computeLayout,nullptr,&solarSourcePipelineLayout),"solar cache layout");
        auto computeCode=loadSPIRV("src/render/shaders/spirv/solar_source.comp.spv");
        if(computeCode.empty())return false;
        VkShaderModule computeModule=createShaderModule(ctx.device,computeCode);
        if(!computeModule)return false;
        VkComputePipelineCreateInfo compute{VK_STRUCTURE_TYPE_COMPUTE_PIPELINE_CREATE_INFO};compute.layout=solarSourcePipelineLayout;
        compute.stage={VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO};compute.stage.stage=VK_SHADER_STAGE_COMPUTE_BIT;
        compute.stage.module=computeModule;compute.stage.pName="main";
        VkResult computeResult=vkCreateComputePipelines(ctx.device,VK_NULL_HANDLE,1,&compute,nullptr,&solarSourcePipeline);
        vkDestroyShaderModule(ctx.device,computeModule,nullptr);vkRequireSuccess(computeResult,"solar source pipeline");
        computeCode=loadSPIRV("src/render/shaders/spirv/solar_response.comp.spv");if(computeCode.empty())return false;
        computeModule=createShaderModule(ctx.device,computeCode);if(!computeModule)return false;
        compute.stage.module=computeModule;
        computeResult=vkCreateComputePipelines(ctx.device,VK_NULL_HANDLE,1,&compute,nullptr,&solarResponsePipeline);
        vkDestroyShaderModule(ctx.device,computeModule,nullptr);vkRequireSuccess(computeResult,"solar field response pipeline");
        if (!solarDiskPipe.init(ctx,colorFmt,depthFmt,"src/render/shaders/spirv/solar_disk.vert.spv","src/render/shaders/spirv/lens_flare.frag.spv",solarInputLayout))return false;
        if (!lensFlarePipe.init(ctx, colorFmt, depthFmt,
                                "src/render/shaders/spirv/lens_flare.vert.spv",
                                "src/render/shaders/spirv/lens_flare.frag.spv",solarInputLayout)) {
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
        solarDiskPipe.shutdown(d);
        if(solarSourcePipeline)vkDestroyPipeline(d,solarSourcePipeline,nullptr);
        if(solarResponsePipeline)vkDestroyPipeline(d,solarResponsePipeline,nullptr);
        solarResponsePipeline=VK_NULL_HANDLE;
        if(solarSourcePipelineLayout)vkDestroyPipelineLayout(d,solarSourcePipelineLayout,nullptr);
        solarSourcePipeline=VK_NULL_HANDLE;solarSourcePipelineLayout=VK_NULL_HANDLE;
        if(solarContext)for(int i=0;i<2;++i){
            solarFieldResponse[i].destroy(*solarContext);solarFieldInitialized[i]=false;
            if(solarSourceBuffers[i])vmaDestroyBuffer(solarContext->allocator,solarSourceBuffers[i],solarSourceAllocations[i]);
            solarSourceBuffers[i]=VK_NULL_HANDLE;solarSourceAllocations[i]=VK_NULL_HANDLE;
        }
        if(solarContext)solarResponse.destroy(*solarContext);
        solarContext=nullptr;
        if(solarInputPool)vkDestroyDescriptorPool(d,solarInputPool,nullptr);
        if(solarInputLayout)vkDestroyDescriptorSetLayout(d,solarInputLayout,nullptr);
        solarInputPool=VK_NULL_HANDLE;solarInputLayout=VK_NULL_HANDLE;
        solarDiskSet=VK_NULL_HANDLE;solarInputSets[0]=solarInputSets[1]=VK_NULL_HANDLE;
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
    //   shapeType  : 0=归一化散射，4=太阳圆盘，6=光圈星芒（a 传叶片数）
    void drawLensFlare(VkCommandBuffer cmd,
                       VkBuffer vbo,
                       float sunSX, float sunSY, float aspect, float intensity,
                       float scaleX, float scaleY, float offX, float offY,
                       float r, float g, float b, float a, int shapeType,
                       float focalY=0,float sinRadius=0,float sourceFlux=0) {
        auto& pipeline=shapeType==4?solarDiskPipe:lensFlarePipe;
        if (pipeline.pipeline == VK_NULL_HANDLE) return;
        pipeline.bind(cmd);
        if(shapeType==4)vkCmdBindDescriptorSets(cmd,VK_PIPELINE_BIND_POINT_GRAPHICS,
            pipeline.layout,0,1,&solarDiskSet,0,nullptr);
        LensFlarePushConstants pc{};
        pc.sunScreenPos[0]=sunSX; pc.sunScreenPos[1]=sunSY;
        pc.aspect=aspect; pc.intensity=intensity;
        pc.scale[0]=scaleX; pc.scale[1]=scaleY;
        pc.offset[0]=offX; pc.offset[1]=offY;
        pc.color[0]=r; pc.color[1]=g; pc.color[2]=b; pc.color[3]=a;
        pc.shapeType=shapeType;
        pc._pad[0]=focalY;pc._pad[1]=sinRadius;pc._pad[2]=sourceFlux;
        vkCmdPushConstants(cmd, pipeline.layout,
            VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT,
            0, sizeof(LensFlarePushConstants), &pc);
        VkDeviceSize off = 0;
        vkCmdBindVertexBuffers(cmd, 0, 1, &vbo, &off);
        vkCmdDraw(cmd, 4, 1, 0, 0);
    }

    // Only update the frame slot whose fence has completed. Input is a distinct
    // HDR snapshot, never the color attachment being written by the optics pass.
    void bindSolarInput(VulkanContext& ctx,VkCommandBuffer cmd,int slot,VkImageView color,VkSampler colorSampler,VkImageView depth,VkSampler depthSampler) {
        VkDescriptorImageInfo images[]={{colorSampler,color,VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL},
            {depthSampler,depth,VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL},
            {solarResponse.sampler,solarResponse.view,VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL}};
        VkWriteDescriptorSet writes[3]={};
        for(int i=0;i<3;++i){writes[i].sType=VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;writes[i].dstSet=solarInputSets[slot%2];
            writes[i].dstBinding=i;writes[i].descriptorCount=1;writes[i].descriptorType=VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;writes[i].pImageInfo=&images[i];}
        vkUpdateDescriptorSets(ctx.device,3,writes,0,nullptr);
        vkCmdBindDescriptorSets(cmd,VK_PIPELINE_BIND_POINT_GRAPHICS,lensFlarePipe.layout,0,1,&solarInputSets[slot%2],0,nullptr);
    }
    void prepareSolarSource(VkCommandBuffer cmd,int slot,float sx,float sy,float aspect,float fy,float sinRadius,float flux,int blades=7){
        LensFlarePushConstants pc{};pc.sunScreenPos[0]=sx;pc.sunScreenPos[1]=sy;pc.aspect=aspect;
        pc._pad[0]=fy;pc._pad[1]=sinRadius;pc._pad[2]=flux;
        vkCmdBindPipeline(cmd,VK_PIPELINE_BIND_POINT_COMPUTE,solarSourcePipeline);
        vkCmdBindDescriptorSets(cmd,VK_PIPELINE_BIND_POINT_COMPUTE,solarSourcePipelineLayout,0,1,&solarInputSets[slot%2],0,nullptr);
        vkCmdPushConstants(cmd,solarSourcePipelineLayout,VK_SHADER_STAGE_COMPUTE_BIT,0,sizeof(pc),&pc);
        vkCmdDispatch(cmd,1,1,1);
        int frame=slot%2;
        transitionImage(cmd,solarFieldResponse[frame].image,
            solarFieldInitialized[frame]?VK_IMAGE_LAYOUT_GENERAL:VK_IMAGE_LAYOUT_UNDEFINED,VK_IMAGE_LAYOUT_GENERAL,
            VK_PIPELINE_STAGE_2_ALL_COMMANDS_BIT,solarFieldInitialized[frame]?VK_ACCESS_2_MEMORY_READ_BIT|VK_ACCESS_2_MEMORY_WRITE_BIT:0,
            VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT,VK_ACCESS_2_SHADER_STORAGE_WRITE_BIT);
        solarFieldInitialized[frame]=true;pc.color[3]=float(blades>=7&&blades<=9?blades:7);
        vkCmdBindPipeline(cmd,VK_PIPELINE_BIND_POINT_COMPUTE,solarResponsePipeline);
        vkCmdPushConstants(cmd,solarSourcePipelineLayout,VK_SHADER_STAGE_COMPUTE_BIT,0,sizeof(pc),&pc);
        vkCmdDispatch(cmd,128,128,1);
        VkMemoryBarrier2 visible{VK_STRUCTURE_TYPE_MEMORY_BARRIER_2};
        visible.srcStageMask=VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT;visible.srcAccessMask=VK_ACCESS_2_SHADER_STORAGE_WRITE_BIT;
        visible.dstStageMask=VK_PIPELINE_STAGE_2_VERTEX_SHADER_BIT|VK_PIPELINE_STAGE_2_FRAGMENT_SHADER_BIT;
        visible.dstAccessMask=VK_ACCESS_2_SHADER_STORAGE_READ_BIT|VK_ACCESS_2_SHADER_SAMPLED_READ_BIT;
        VkDependencyInfo dependency{VK_STRUCTURE_TYPE_DEPENDENCY_INFO};dependency.memoryBarrierCount=1;dependency.pMemoryBarriers=&visible;
        vkCmdPipelineBarrier2(cmd,&dependency);
    }
};
//ribbon,billboard,lens_flare,exhaust全部挤在一起
