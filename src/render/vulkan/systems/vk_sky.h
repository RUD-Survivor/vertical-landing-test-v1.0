#pragma once
// ==========================================================================
// vk_sky.h — 线性 HDR 银河背景与星表恒星渲染
// 全屏三角形（无顶点缓冲），几何 pass 早期绘制。
// ==========================================================================

#include "../vk_context.h"
#include "../vk_descriptors.h"
#include "../vk_pipeline.h"
#include "vk_milky_way.h"

#include <cstring>
#include <cmath>
#include <fstream>
#include <vector>
#include <cstdint>

// 星空 / 银河调参（imgui_sky_tuner.h，F5）
struct SkyTuneParams {
    bool observationMode    = false;
    float exposure          = 1.0f; // Linear source multiplier; display exposure is global.
    float starDensity       = 1.0f;
    float starBrightness    = 1.0f;
    float starSize          = 0.65f; // Camera PSF sigma in pixels, not stellar angular radius.
    float glowStrength      = 0.03f; // Fraction of conserved flux in the wide PSF.
    float milkyWayIntensity = 1.0f;
    float colorVariety      = 1.0f;
    float bgLevel           = 1.0f;
    float galaxyYaw         = 0.0f; // Artistic offset from catalog Galactic frame.
    float galaxyPitch       = 0.0f;
};

struct VkSkySystem {
    VkSkyboxPipeline     pipe;
    VkDescriptorManager* desc = nullptr;
    SkyTuneParams        tune;
    struct Star {float directionFlux[4],color[4];};
    static_assert(sizeof(Star)==32,"star SSBO layout");
    VkSkyboxPipeline stars;
    VkMilkyWay milkyWay;
    VkDescriptorSetLayout starLayout=VK_NULL_HANDLE;
    VkDescriptorPool starPool=VK_NULL_HANDLE;VkDescriptorSet starSet=VK_NULL_HANDLE;
    VkBuffer starBuffer=VK_NULL_HANDLE;VmaAllocation starAllocation{};void* starMapped=nullptr;
    std::vector<Star> catalog;uint32_t starCount=0;

    void uploadStars(VulkanContext& ctx,const std::vector<Star>& data) {
        if(data.size()>catalog.size())throw std::runtime_error("sky catalog capacity exceeded");
        starCount=uint32_t(data.size());
        if(!data.empty())memcpy(starMapped,data.data(),data.size()*sizeof(Star));
        vkRequireSuccess(vmaFlushAllocation(ctx.allocator,starAllocation,0,VK_WHOLE_SIZE),"flush star catalog");
    }

    // -----------------------------------------------------------------------
    bool init(VulkanContext& ctx, VkDescriptorManager& d,
              VkFormat colorFmt, VkFormat depthFmt) {
        desc = &d;
        std::ifstream file("assets/data/sky-stars.bin",std::ios::binary);uint32_t header[2]={};
        file.read(reinterpret_cast<char*>(header),sizeof(header));
        if(!file || header[0]!=0x53544152 || !header[1] || header[1]>25000)return false;
        catalog.resize(header[1]);file.read(reinterpret_cast<char*>(catalog.data()),catalog.size()*sizeof(Star));
        if(!file)return false;
        for(const auto& s:catalog){double length=0;
            for(int i=0;i<3;++i){if(!std::isfinite(s.directionFlux[i]) || !std::isfinite(s.color[i]) || s.color[i]<0)return false;length+=s.directionFlux[i]*s.directionFlux[i];}
            if(std::abs(length-1)>1e-4 || !std::isfinite(s.directionFlux[3]) || s.directionFlux[3]<=0)return false;}
        if(!milkyWay.load(ctx))return false;
        VkDescriptorSetLayoutBinding bindings[]={{0,VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,1,VK_SHADER_STAGE_VERTEX_BIT,nullptr},
            {1,VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,1,VK_SHADER_STAGE_FRAGMENT_BIT,nullptr}};
        VkDescriptorSetLayoutCreateInfo li{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO};li.bindingCount=2;li.pBindings=bindings;
        vkRequireSuccess(vkCreateDescriptorSetLayout(ctx.device,&li,nullptr,&starLayout),"star layout");
        VkDescriptorPoolSize ps[]={{VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,1},{VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,1}};VkDescriptorPoolCreateInfo pi{VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO};
        pi.maxSets=1;pi.poolSizeCount=2;pi.pPoolSizes=ps;vkRequireSuccess(vkCreateDescriptorPool(ctx.device,&pi,nullptr,&starPool),"star pool");
        VkDescriptorSetAllocateInfo ai{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO};ai.descriptorPool=starPool;ai.descriptorSetCount=1;ai.pSetLayouts=&starLayout;
        vkRequireSuccess(vkAllocateDescriptorSets(ctx.device,&ai,&starSet),"star set");
        VkBufferCreateInfo bi{VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO};bi.size=catalog.size()*sizeof(Star);bi.usage=VK_BUFFER_USAGE_STORAGE_BUFFER_BIT;
        VmaAllocationCreateInfo ac{};ac.usage=VMA_MEMORY_USAGE_CPU_TO_GPU;ac.flags=VMA_ALLOCATION_CREATE_MAPPED_BIT;VmaAllocationInfo info{};
        vkRequireSuccess(vmaCreateBuffer(ctx.allocator,&bi,&ac,&starBuffer,&starAllocation,&info),"star buffer");starMapped=info.pMappedData;uploadStars(ctx,catalog);
        VkDescriptorBufferInfo buffer{starBuffer,0,bi.size};VkWriteDescriptorSet write{VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
        write.dstSet=starSet;write.dstBinding=0;write.descriptorCount=1;write.descriptorType=VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;write.pBufferInfo=&buffer;vkUpdateDescriptorSets(ctx.device,1,&write,0,nullptr);
        VkDescriptorImageInfo image{milkyWay.sampler,milkyWay.view,VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL};
        write.dstBinding=1;write.descriptorType=VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;write.pBufferInfo=nullptr;write.pImageInfo=&image;
        vkUpdateDescriptorSets(ctx.device,1,&write,0,nullptr);
        if(!stars.init(ctx,d,colorFmt,depthFmt,"src/render/shaders/spirv/stars.vert.spv","src/render/shaders/spirv/stars.frag.spv",starLayout,true))return false;
        if (!pipe.init(ctx, d, colorFmt, depthFmt,
                       "src/render/shaders/spirv/skybox.vert.spv",
                       "src/render/shaders/spirv/skybox.frag.spv",starLayout)) {
            fprintf(stderr, "[VkSky] Failed: skybox pipeline\n"); return false;
        }
        printf("[VkSky] Initialized\n");
        return true;
    }

    void shutdown(VulkanContext& ctx) {
        pipe.shutdown(ctx.device);stars.shutdown(ctx.device);
        if(starPool)vkDestroyDescriptorPool(ctx.device,starPool,nullptr);
        if(starLayout)vkDestroyDescriptorSetLayout(ctx.device,starLayout,nullptr);
        if(starBuffer)vmaDestroyBuffer(ctx.allocator,starBuffer,starAllocation);
        milkyWay.destroy(ctx);
        starPool=VK_NULL_HANDLE;starLayout=VK_NULL_HANDLE;starBuffer=VK_NULL_HANDLE;starMapped=nullptr;starCount=0;
    }

    // -----------------------------------------------------------------------
    // 绘制天空盒
    //   view/proj : 列主序 float[16]
    //   vibrancy  : 场景昼夜衰减（1.0 = 正常；白天由 flight_scene 压低）
    void draw(VkCommandBuffer cmd, int frameIdx,
              const float view[16], const float proj[16],
              float vibrancy = 1.0f,float width=1280,float height=720) {
        if (pipe.pipeline == VK_NULL_HANDLE) return;
        float vp[16], invVP[16];
        mul4x4(proj, view, vp);
        if (!inv4x4(vp, invVP)) return;

        pipe.bind(cmd);
        VkDescriptorSet backgroundSets[]={desc->set0[frameIdx],starSet};
        vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS,
            pipe.layout, 0, 2, backgroundSets, 0, nullptr);

        SkyboxPushConstants pc{};
        memcpy(pc.invViewProj, invVP, 64);
        pc.skyVibrancy       = vibrancy;
        pc.exposure          = tune.exposure;
        pc.starDensity       = tune.starDensity;
        pc.starBrightness    = tune.starBrightness;
        pc.starSize          = tune.starSize;
        pc.glowStrength      = tune.glowStrength;
        pc.milkyWayIntensity = tune.milkyWayIntensity;
        // Former procedural shape parameters remain reserved in the 128-byte ABI.
        pc.colorVariety      = tune.colorVariety;
        pc.bgLevel           = tune.bgLevel;
        pc.galaxyYaw         = tune.galaxyYaw;
        pc.galaxyPitch       = tune.galaxyPitch;
        vkCmdPushConstants(cmd, pipe.layout,
            VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT,
            0, sizeof(SkyboxPushConstants), &pc);
        vkCmdDraw(cmd, 3, 1, 0, 0);
        // A catalog star is an unresolved light source. Rasterize only its PSF
        // footprint instead of searching 108 random cells for every sky pixel.
        stars.bind(cmd);VkDescriptorSet sets[]={desc->set0[frameIdx],starSet};
        vkCmdBindDescriptorSets(cmd,VK_PIPELINE_BIND_POINT_GRAPHICS,stars.layout,0,2,sets,0,nullptr);
        memcpy(pc.invViewProj,vp,64);pc._pad[0]=width;pc._pad[1]=height;
        vkCmdPushConstants(cmd,stars.layout,VK_SHADER_STAGE_VERTEX_BIT|VK_SHADER_STAGE_FRAGMENT_BIT,0,sizeof(pc),&pc);
        uint32_t count=uint32_t(starCount*std::clamp(tune.starDensity,0.f,1.f));
        if(count && vibrancy>0)vkCmdDraw(cmd,6,count,0,0);
    }

private:
    static void mul4x4(const float A[16], const float B[16], float C[16]) {
        for (int r = 0; r < 4; r++)
            for (int c = 0; c < 4; c++) {
                float s = 0.f;
                for (int k = 0; k < 4; k++) s += A[r + k*4] * B[k + c*4];
                C[r + c*4] = s;
            }
    }

    static bool inv4x4(const float M[16], float inv[16]) {
        float s0=M[0]*M[5]-M[4]*M[1], s1=M[0]*M[9]-M[8]*M[1], s2=M[0]*M[13]-M[12]*M[1];
        float s3=M[4]*M[9]-M[8]*M[5], s4=M[4]*M[13]-M[12]*M[5], s5=M[8]*M[13]-M[12]*M[9];
        float c5=M[10]*M[15]-M[14]*M[11], c4=M[6]*M[15]-M[14]*M[7], c3=M[6]*M[11]-M[10]*M[7];
        float c2=M[2]*M[15]-M[14]*M[3], c1=M[2]*M[11]-M[10]*M[3], c0=M[2]*M[7]-M[6]*M[3];
        float det = s0*c5-s1*c4+s2*c3+s3*c2-s4*c1+s5*c0;
        if (fabsf(det) < 1e-10f) return false;
        float id = 1.f / det;
        inv[0] =( M[5]*c5-M[9]*c4+M[13]*c3)*id; inv[4] =(-M[4]*c5+M[8]*c4-M[12]*c3)*id;
        inv[8] =( M[7]*s5-M[11]*s4+M[15]*s3)*id; inv[12]=(-M[6]*s5+M[10]*s4-M[14]*s3)*id;
        inv[1] =(-M[1]*c5+M[9]*c2-M[13]*c1)*id; inv[5] =( M[0]*c5-M[8]*c2+M[12]*c1)*id;
        inv[9] =(-M[3]*s5+M[11]*s2-M[15]*s1)*id; inv[13]=( M[2]*s5-M[10]*s2+M[14]*s1)*id;
        inv[2] =( M[1]*c4-M[5]*c2+M[13]*c0)*id; inv[6] =(-M[0]*c4+M[4]*c2-M[12]*c0)*id;
        inv[10]=( M[3]*s4-M[7]*s2+M[15]*s0)*id; inv[14]=(-M[2]*s4+M[6]*s2-M[14]*s0)*id;
        inv[3] =(-M[1]*c3+M[5]*c1-M[9]*c0 )*id; inv[7] =( M[0]*c3-M[4]*c1+M[8]*c0 )*id;
        inv[11]=(-M[3]*s3+M[7]*s1-M[11]*s0)*id; inv[15]=( M[2]*s3-M[6]*s1+M[10]*s0)*id;
        return true;
    }
};
