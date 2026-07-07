#pragma once
// ==========================================================================
// vk_cloud_system.h — Flower cloud pipeline (Phase 0: stub compute + composite)
//
// Pass order (after TAA geometry pass, before TAA resolve):
//   1. raymarch stub  → quarter cloud L+T + depth
//   2. reconstruct    → full-res cloud L+T
//   3. composite      → HDR scene * T + L  (RS3D depth: >= 0.9995 = sky)
//
// Phase 1+ will replace stub comp shaders with cloud_raymarching / reconstruct /
// cloud_composite and fill CloudFrameUBO → PerFrameData.
// ==========================================================================

#include "vk_context.h"
#include "vk_frame.h"
#include "vk_mesh.h"
#include "vk_pipeline.h"
#include "vk_utils.h"
#include "vk_taa.h"
#include "../scene_snapshot.h"

#include <cstdio>
#include <cstring>
#include <algorithm>

// Matches cloud_phase0_*.comp.glsl — will grow into PerFrameData in Phase 1.
struct CloudPhase0UBO {
    float renderWidth   = 0.f;
    float renderHeight  = 0.f;
    float quarterWidth  = 0.f;
    float quarterHeight = 0.f;
    uint32_t frameIndex = 0;
    uint32_t bCameraCut = 0;
    float _pad0 = 0.f;
    float _pad1 = 0.f;
};
static_assert(sizeof(CloudPhase0UBO) == 32, "CloudPhase0UBO size must match GLSL std140");

struct VkCloudSystem {
    static constexpr VkFormat kCloudFmt  = VK_FORMAT_R16G16B16A16_SFLOAT;
    static constexpr VkFormat kDepthFmt  = VK_FORMAT_R32_SFLOAT;

    bool enabled = true;  // true = flower path; false = legacy cloud.frag
    bool debugLogOnce = true;

    VkExtent2D extent     = {};
    VkExtent2D quarterExt = {};

    // ── Internal cloud RTs ───────────────────────────────────────────────────
    VkImage       cloudQuarterImg   = VK_NULL_HANDLE;
    VmaAllocation cloudQuarterAlloc = VK_NULL_HANDLE;
    VkImageView   cloudQuarterView  = VK_NULL_HANDLE;

    VkImage       cloudDepthQuarterImg   = VK_NULL_HANDLE;
    VmaAllocation cloudDepthQuarterAlloc = VK_NULL_HANDLE;
    VkImageView   cloudDepthQuarterView  = VK_NULL_HANDLE;

    VkImage       cloudFullImg   = VK_NULL_HANDLE;
    VmaAllocation cloudFullAlloc = VK_NULL_HANDLE;
    VkImageView   cloudFullView  = VK_NULL_HANDLE;

    VkSampler linearClampSampler = VK_NULL_HANDLE;

    // ── UBO (per frame in flight) ────────────────────────────────────────────
    VkBuffer      uboBuf[FRAMES_IN_FLIGHT]   = {};
    VmaAllocation uboAlloc[FRAMES_IN_FLIGHT] = {};
    void*         uboMapped[FRAMES_IN_FLIGHT]= {};

    // ── Compute pipelines ────────────────────────────────────────────────────
    VkDescriptorSetLayout raymarchLayout    = VK_NULL_HANDLE;
    VkDescriptorSetLayout reconstructLayout = VK_NULL_HANDLE;
    VkDescriptorSetLayout compositeLayout   = VK_NULL_HANDLE;

    VkDescriptorSetLayout samplerLayout     = VK_NULL_HANDLE;
    VkPipelineLayout      raymarchPL        = VK_NULL_HANDLE;
    VkPipelineLayout      reconstructPL     = VK_NULL_HANDLE;
    VkPipelineLayout      compositePL       = VK_NULL_HANDLE;

    VkPipeline raymarchPipe    = VK_NULL_HANDLE;
    VkPipeline reconstructPipe = VK_NULL_HANDLE;
    VkPipeline compositePipe   = VK_NULL_HANDLE;

    VkDescriptorPool pool           = VK_NULL_HANDLE;
    VkDescriptorSet  raymarchSet    = VK_NULL_HANDLE;
    VkDescriptorSet  reconstructSet = VK_NULL_HANDLE;
    VkDescriptorSet  compositeSet   = VK_NULL_HANDLE;
    VkDescriptorSet  samplerSet     = VK_NULL_HANDLE;

    bool cloudRtInitialized = false;

    // -----------------------------------------------------------------------
    bool init(VulkanContext& ctx, VkExtent2D ext) {
        extent = ext;
        quarterExt = { std::max(1u, (ext.width  + 3) / 4),
                       std::max(1u, (ext.height + 3) / 4) };

        if (!createSamplers(ctx)) return false;
        if (!createImages(ctx)) return false;
        if (!createUBOs(ctx)) return false;
        if (!createDescriptors(ctx)) return false;
        if (!createPipelines(ctx)) return false;

        printf("[VkCloudSystem] Phase0 ready %ux%u (quarter %ux%u)\n",
               extent.width, extent.height, quarterExt.width, quarterExt.height);
        return true;
    }

    void recreate(VulkanContext& ctx, VkExtent2D ext) {
        vkDeviceWaitIdle(ctx.device);
        destroyPipelines(ctx.device);
        destroyDescriptors(ctx.device);
        destroyImages(ctx);
        destroyUBOs(ctx);
        extent = ext;
        quarterExt = { std::max(1u, (ext.width  + 3) / 4),
                       std::max(1u, (ext.height + 3) / 4) };
        if (!createImages(ctx) || !createUBOs(ctx) ||
            !createDescriptors(ctx) || !createPipelines(ctx)) {
            fprintf(stderr, "[VkCloudSystem] recreate failed\n");
        }
    }

    void shutdown(VulkanContext& ctx) {
        vkDeviceWaitIdle(ctx.device);
        destroyPipelines(ctx.device);
        destroyDescriptors(ctx.device);
        destroyImages(ctx);
        destroyUBOs(ctx);
        if (linearClampSampler != VK_NULL_HANDLE) {
            vkDestroySampler(ctx.device, linearClampSampler, nullptr);
            linearClampSampler = VK_NULL_HANDLE;
        }
    }

    // Called after TAA endGeometryPass — HDR + depth are SHADER_READ_ONLY.
    void renderPhase0(VulkanContext& ctx, VkCommandBuffer cmd, VkTAA& taa,
                      const SceneSnapshot& snap, int frameSlot,
                      bool drawClouds) {
        if (!enabled) return;
        if (!drawClouds) {
            if (debugLogOnce) {
                fprintf(stderr, "[VkCloudSystem] Phase0 skip: drawClouds=false (need Earth atmoIdx=3 + showClouds)\n");
                debugLogOnce = false;
            }
            return;
        }
        if (raymarchPipe == VK_NULL_HANDLE) return;

        if (debugLogOnce) {
            printf("[VkCloudSystem] Phase0 dispatch frame %u (%ux%u)\n",
                   snap.frameIndex, extent.width, extent.height);
            debugLogOnce = false;
        }

        updatePerFrameBindings(ctx, taa, frameSlot);

        CloudPhase0UBO ubo{};
        ubo.renderWidth  = (float)extent.width;
        ubo.renderHeight = (float)extent.height;
        ubo.quarterWidth = (float)quarterExt.width;
        ubo.quarterHeight= (float)quarterExt.height;
        ubo.frameIndex   = (uint32_t)snap.frameIndex;
        ubo.bCameraCut   = 0;
        memcpy(uboMapped[frameSlot % FRAMES_IN_FLIGHT], &ubo, sizeof(ubo));

        VkImage hdrImage  = taa.colorImages[taa.currentIdx];

        const VkImageLayout qOld = cloudRtInitialized
            ? VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL : VK_IMAGE_LAYOUT_UNDEFINED;
        const VkImageLayout dOld = cloudRtInitialized
            ? VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL : VK_IMAGE_LAYOUT_UNDEFINED;
        const VkImageLayout fOld = cloudRtInitialized
            ? VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL : VK_IMAGE_LAYOUT_UNDEFINED;

        // ── 1) Raymarch stub ───────────────────────────────────────────────
        transitionImage(cmd, cloudQuarterImg, qOld, VK_IMAGE_LAYOUT_GENERAL,
            VK_PIPELINE_STAGE_2_ALL_COMMANDS_BIT,
            qOld == VK_IMAGE_LAYOUT_UNDEFINED ? VK_ACCESS_2_NONE : VK_ACCESS_2_SHADER_READ_BIT,
            VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT, VK_ACCESS_2_SHADER_WRITE_BIT);
        transitionImage(cmd, cloudDepthQuarterImg, dOld, VK_IMAGE_LAYOUT_GENERAL,
            VK_PIPELINE_STAGE_2_ALL_COMMANDS_BIT,
            dOld == VK_IMAGE_LAYOUT_UNDEFINED ? VK_ACCESS_2_NONE : VK_ACCESS_2_SHADER_READ_BIT,
            VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT, VK_ACCESS_2_SHADER_WRITE_BIT);

        vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_COMPUTE, raymarchPipe);
        VkDescriptorSet rmSets[] = { raymarchSet };
        vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_COMPUTE,
            raymarchPL, 0, 1, rmSets, 0, nullptr);
        dispatch2D(cmd, raymarchPipe, quarterExt);

        transitionImage(cmd, cloudQuarterImg,
            VK_IMAGE_LAYOUT_GENERAL, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
            VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT, VK_ACCESS_2_SHADER_WRITE_BIT,
            VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT, VK_ACCESS_2_SHADER_READ_BIT);

        // ── 2) Reconstruct stub ────────────────────────────────────────────
        transitionImage(cmd, cloudFullImg, fOld, VK_IMAGE_LAYOUT_GENERAL,
            VK_PIPELINE_STAGE_2_ALL_COMMANDS_BIT,
            fOld == VK_IMAGE_LAYOUT_UNDEFINED ? VK_ACCESS_2_NONE : VK_ACCESS_2_SHADER_READ_BIT,
            VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT, VK_ACCESS_2_SHADER_WRITE_BIT);

        vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_COMPUTE, reconstructPipe);
        VkDescriptorSet rcSets[] = { reconstructSet, samplerSet };
        vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_COMPUTE,
            reconstructPL, 0, 2, rcSets, 0, nullptr);
        dispatch2D(cmd, reconstructPipe, extent);

        transitionImage(cmd, cloudFullImg,
            VK_IMAGE_LAYOUT_GENERAL, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
            VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT, VK_ACCESS_2_SHADER_WRITE_BIT,
            VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT, VK_ACCESS_2_SHADER_READ_BIT);

        // ── 3) Composite stub ──────────────────────────────────────────────
        transitionImage(cmd, hdrImage,
            VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, VK_IMAGE_LAYOUT_GENERAL,
            VK_PIPELINE_STAGE_2_FRAGMENT_SHADER_BIT, VK_ACCESS_2_SHADER_READ_BIT,
            VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT, VK_ACCESS_2_SHADER_WRITE_BIT);

        vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_COMPUTE, compositePipe);
        VkDescriptorSet cpSets[] = { compositeSet, samplerSet };
        vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_COMPUTE,
            compositePL, 0, 2, cpSets, 0, nullptr);
        dispatch2D(cmd, compositePipe, extent);

        transitionImage(cmd, hdrImage,
            VK_IMAGE_LAYOUT_GENERAL, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
            VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT, VK_ACCESS_2_SHADER_WRITE_BIT,
            VK_PIPELINE_STAGE_2_FRAGMENT_SHADER_BIT, VK_ACCESS_2_SHADER_READ_BIT);

        cloudRtInitialized = true;
    }

private:
    bool createSamplers(VulkanContext& ctx) {
        VkSamplerCreateInfo sci{ VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO };
        sci.magFilter    = VK_FILTER_LINEAR;
        sci.minFilter    = VK_FILTER_LINEAR;
        sci.addressModeU = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
        sci.addressModeV = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
        return vkCreateSampler(ctx.device, &sci, nullptr, &linearClampSampler) == VK_SUCCESS;
    }

    bool createImages(VulkanContext& ctx) {
        return createStorage2D(ctx, quarterExt, kCloudFmt,
                   VK_IMAGE_USAGE_STORAGE_BIT | VK_IMAGE_USAGE_SAMPLED_BIT,
                   cloudQuarterImg, cloudQuarterAlloc, cloudQuarterView)
            && createStorage2D(ctx, quarterExt, kDepthFmt,
                   VK_IMAGE_USAGE_STORAGE_BIT | VK_IMAGE_USAGE_SAMPLED_BIT,
                   cloudDepthQuarterImg, cloudDepthQuarterAlloc, cloudDepthQuarterView)
            && createStorage2D(ctx, extent, kCloudFmt,
                   VK_IMAGE_USAGE_STORAGE_BIT | VK_IMAGE_USAGE_SAMPLED_BIT,
                   cloudFullImg, cloudFullAlloc, cloudFullView);
    }

    bool createStorage2D(VulkanContext& ctx, VkExtent2D ext, VkFormat fmt,
                         VkImageUsageFlags usage,
                         VkImage& img, VmaAllocation& alloc, VkImageView& view) {
        VkImageCreateInfo ici{ VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO };
        ici.imageType   = VK_IMAGE_TYPE_2D;
        ici.format      = fmt;
        ici.extent      = { ext.width, ext.height, 1 };
        ici.mipLevels   = 1;
        ici.arrayLayers = 1;
        ici.samples     = VK_SAMPLE_COUNT_1_BIT;
        ici.tiling      = VK_IMAGE_TILING_OPTIMAL;
        ici.usage       = usage;
        ici.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
        VmaAllocationCreateInfo aci{}; aci.usage = VMA_MEMORY_USAGE_GPU_ONLY;
        if (vmaCreateImage(ctx.allocator, &ici, &aci, &img, &alloc, nullptr) != VK_SUCCESS)
            return false;

        VkImageViewCreateInfo vci{ VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO };
        vci.image = img; vci.viewType = VK_IMAGE_VIEW_TYPE_2D; vci.format = fmt;
        vci.subresourceRange = { VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1 };
        return vkCreateImageView(ctx.device, &vci, nullptr, &view) == VK_SUCCESS;
    }

    bool createUBOs(VulkanContext& ctx) {
        VkBufferCreateInfo bci{ VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO };
        bci.size  = sizeof(CloudPhase0UBO);
        bci.usage = VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT;
        VmaAllocationCreateInfo aci{};
        aci.usage = VMA_MEMORY_USAGE_CPU_TO_GPU;
        aci.flags = VMA_ALLOCATION_CREATE_MAPPED_BIT;
        for (int i = 0; i < FRAMES_IN_FLIGHT; i++) {
            VmaAllocationInfo info{};
            if (vmaCreateBuffer(ctx.allocator, &bci, &aci,
                    &uboBuf[i], &uboAlloc[i], &info) != VK_SUCCESS)
                return false;
            uboMapped[i] = info.pMappedData;
        }
        return true;
    }

    bool createDescriptors(VulkanContext& ctx) {
        // Raymarch: storage quarter + storage depth + UBO
        {
            VkDescriptorSetLayoutBinding b[3]{};
            b[0] = { 0, VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr };
            b[1] = { 1, VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr };
            b[2] = { 2, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr };
            VkDescriptorSetLayoutCreateInfo ci{ VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO };
            ci.bindingCount = 3; ci.pBindings = b;
            if (vkCreateDescriptorSetLayout(ctx.device, &ci, nullptr, &raymarchLayout) != VK_SUCCESS)
                return false;
        }
        // Reconstruct: texture quarter + storage full
        {
            VkDescriptorSetLayoutBinding b[2]{};
            b[0] = { 0, VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr };
            b[1] = { 1, VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr };
            VkDescriptorSetLayoutCreateInfo ci{ VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO };
            ci.bindingCount = 2; ci.pBindings = b;
            if (vkCreateDescriptorSetLayout(ctx.device, &ci, nullptr, &reconstructLayout) != VK_SUCCESS)
                return false;
        }
        // Composite: storage HDR + sampled cloud + sampled depth + UBO
        {
            VkDescriptorSetLayoutBinding b[4]{};
            b[0] = { 0, VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr };
            b[1] = { 1, VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr };
            b[2] = { 2, VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr };
            b[3] = { 3, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr };
            VkDescriptorSetLayoutCreateInfo ci{ VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO };
            ci.bindingCount = 4; ci.pBindings = b;
            if (vkCreateDescriptorSetLayout(ctx.device, &ci, nullptr, &compositeLayout) != VK_SUCCESS)
                return false;
        }
        // Sampler set 1
        {
            VkDescriptorSetLayoutBinding b{};
            b.binding = 0; b.descriptorType = VK_DESCRIPTOR_TYPE_SAMPLER;
            b.descriptorCount = 1; b.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
            VkDescriptorSetLayoutCreateInfo ci{ VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO };
            ci.bindingCount = 1; ci.pBindings = &b;
            if (vkCreateDescriptorSetLayout(ctx.device, &ci, nullptr, &samplerLayout) != VK_SUCCESS)
                return false;
        }

        VkDescriptorPoolSize sizes[] = {
            { VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, 5 },
            { VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE, 4 },
            { VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 2 },
            { VK_DESCRIPTOR_TYPE_SAMPLER, 1 },
        };
        VkDescriptorPoolCreateInfo pci{ VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO };
        pci.maxSets       = 4;
        pci.poolSizeCount = 4;
        pci.pPoolSizes    = sizes;
        if (vkCreateDescriptorPool(ctx.device, &pci, nullptr, &pool) != VK_SUCCESS)
            return false;

        auto allocSet = [&](VkDescriptorSetLayout layout, VkDescriptorSet& out) -> bool {
            VkDescriptorSetAllocateInfo ai{ VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO };
            ai.descriptorPool = pool; ai.descriptorSetCount = 1; ai.pSetLayouts = &layout;
            return vkAllocateDescriptorSets(ctx.device, &ai, &out) == VK_SUCCESS;
        };
        if (!allocSet(raymarchLayout, raymarchSet) ||
            !allocSet(reconstructLayout, reconstructSet) ||
            !allocSet(compositeLayout, compositeSet) ||
            !allocSet(samplerLayout, samplerSet))
            return false;

        // Static bindings (except HDR which is updated per frame in updateCompositeHdrBinding)
        {
            VkDescriptorImageInfo i0{ VK_NULL_HANDLE, cloudQuarterView, VK_IMAGE_LAYOUT_GENERAL };
            VkDescriptorImageInfo i1{ VK_NULL_HANDLE, cloudDepthQuarterView, VK_IMAGE_LAYOUT_GENERAL };
            VkDescriptorBufferInfo bi{ uboBuf[0], 0, sizeof(CloudPhase0UBO) };
            VkWriteDescriptorSet w[3]{};
            w[0].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET; w[0].dstSet = raymarchSet;
            w[0].dstBinding = 0; w[0].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_IMAGE;
            w[0].descriptorCount = 1; w[0].pImageInfo = &i0;
            w[1].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET; w[1].dstSet = raymarchSet;
            w[1].dstBinding = 1; w[1].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_IMAGE;
            w[1].descriptorCount = 1; w[1].pImageInfo = &i1;
            w[2].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET; w[2].dstSet = raymarchSet;
            w[2].dstBinding = 2; w[2].descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
            w[2].descriptorCount = 1; w[2].pBufferInfo = &bi;
            vkUpdateDescriptorSets(ctx.device, 3, w, 0, nullptr);
        }
        {
            VkDescriptorImageInfo i0{ VK_NULL_HANDLE, cloudQuarterView, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL };
            VkDescriptorImageInfo i1{ VK_NULL_HANDLE, cloudFullView, VK_IMAGE_LAYOUT_GENERAL };
            VkWriteDescriptorSet w[2]{};
            w[0].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET; w[0].dstSet = reconstructSet;
            w[0].dstBinding = 0; w[0].descriptorType = VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE;
            w[0].descriptorCount = 1; w[0].pImageInfo = &i0;
            w[1].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET; w[1].dstSet = reconstructSet;
            w[1].dstBinding = 1; w[1].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_IMAGE;
            w[1].descriptorCount = 1; w[1].pImageInfo = &i1;
            vkUpdateDescriptorSets(ctx.device, 2, w, 0, nullptr);
        }
        {
            VkDescriptorImageInfo i1{ VK_NULL_HANDLE, cloudFullView, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL };
            VkWriteDescriptorSet w{};
            w.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET; w.dstSet = compositeSet;
            w.dstBinding = 1; w.descriptorType = VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE;
            w.descriptorCount = 1; w.pImageInfo = &i1;
            vkUpdateDescriptorSets(ctx.device, 1, &w, 0, nullptr);
        }
        {
            VkDescriptorImageInfo si{};
            si.sampler = linearClampSampler;
            VkWriteDescriptorSet w{};
            w.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET; w.dstSet = samplerSet;
            w.dstBinding = 0; w.descriptorType = VK_DESCRIPTOR_TYPE_SAMPLER;
            w.descriptorCount = 1; w.pImageInfo = &si;
            vkUpdateDescriptorSets(ctx.device, 1, &w, 0, nullptr);
        }

        // UBO on both raymarch + composite sets (same buffer slot 0)
        for (int i = 0; i < FRAMES_IN_FLIGHT; i++) {
            (void)i; // all sets point to uboBuf[0] initially; updated per frame below
        }
        updateCompositeStaticBindings(ctx);
        return true;
    }

    void updateCompositeStaticBindings(VulkanContext& ctx) {
        VkDescriptorBufferInfo bi{ uboBuf[0], 0, sizeof(CloudPhase0UBO) };
        VkWriteDescriptorSet w{};
        w.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET; w.dstSet = compositeSet;
        w.dstBinding = 3; w.descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
        w.descriptorCount = 1; w.pBufferInfo = &bi;
        vkUpdateDescriptorSets(ctx.device, 1, &w, 0, nullptr);

        w.dstSet = raymarchSet; w.dstBinding = 2;
        vkUpdateDescriptorSets(ctx.device, 1, &w, 0, nullptr);
    }

    void updatePerFrameBindings(VulkanContext& ctx, VkTAA& taa, int frameSlot) {
        VkDescriptorBufferInfo bi{ uboBuf[frameSlot % FRAMES_IN_FLIGHT], 0, sizeof(CloudPhase0UBO) };
        VkWriteDescriptorSet w[2]{};
        w[0].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
        w[0].dstSet = raymarchSet; w[0].dstBinding = 2;
        w[0].descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
        w[0].descriptorCount = 1; w[0].pBufferInfo = &bi;
        w[1] = w[0]; w[1].dstSet = compositeSet; w[1].dstBinding = 3;
        vkUpdateDescriptorSets(ctx.device, 2, w, 0, nullptr);

        VkDescriptorImageInfo hdr{};
        hdr.imageView   = taa.colorViews[taa.currentIdx];
        hdr.imageLayout = VK_IMAGE_LAYOUT_GENERAL;

        VkDescriptorImageInfo cloudFull{};
        cloudFull.imageView   = cloudFullView;
        cloudFull.imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;

        VkDescriptorImageInfo depth{};
        depth.imageView   = taa.depthView;
        depth.imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;

        VkWriteDescriptorSet iw[3]{};
        iw[0].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET; iw[0].dstSet = compositeSet;
        iw[0].dstBinding = 0; iw[0].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_IMAGE;
        iw[0].descriptorCount = 1; iw[0].pImageInfo = &hdr;
        iw[1].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET; iw[1].dstSet = compositeSet;
        iw[1].dstBinding = 1; iw[1].descriptorType = VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE;
        iw[1].descriptorCount = 1; iw[1].pImageInfo = &cloudFull;
        iw[2].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET; iw[2].dstSet = compositeSet;
        iw[2].dstBinding = 2; iw[2].descriptorType = VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE;
        iw[2].descriptorCount = 1; iw[2].pImageInfo = &depth;
        vkUpdateDescriptorSets(ctx.device, 3, iw, 0, nullptr);
    }

    bool createPipelines(VulkanContext& ctx) {
        if (!createComputePL(ctx, raymarchLayout, &samplerLayout, 1,
                "src/render/shaders/spirv/cloud_phase0_raymarch.comp.spv",
                raymarchPL, raymarchPipe))
            return false;
        if (!createComputePL(ctx, reconstructLayout, &samplerLayout, 1,
                "src/render/shaders/spirv/cloud_phase0_reconstruct.comp.spv",
                reconstructPL, reconstructPipe))
            return false;
        if (!createComputePL(ctx, compositeLayout, &samplerLayout, 1,
                "src/render/shaders/spirv/cloud_phase0_composite.comp.spv",
                compositePL, compositePipe))
            return false;
        return true;
    }

    bool createComputePL(VulkanContext& ctx,
                         VkDescriptorSetLayout set0,
                         VkDescriptorSetLayout* set1, uint32_t set1Count,
                         const char* spvPath,
                         VkPipelineLayout& outPL, VkPipeline& outPipe) {
        auto code = loadSPIRV(spvPath);
        if (code.empty()) return false;
        VkShaderModule mod = createShaderModule(ctx.device, code);
        if (mod == VK_NULL_HANDLE) return false;

        VkDescriptorSetLayout layouts[2] = { set0, set1 ? set1[0] : VK_NULL_HANDLE };
        uint32_t layoutCount = set1Count > 0 ? 2u : 1u;

        VkPipelineLayoutCreateInfo plci{ VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO };
        plci.setLayoutCount = layoutCount;
        plci.pSetLayouts    = layouts;
        if (vkCreatePipelineLayout(ctx.device, &plci, nullptr, &outPL) != VK_SUCCESS) {
            vkDestroyShaderModule(ctx.device, mod, nullptr);
            return false;
        }

        VkPipelineShaderStageCreateInfo stage{ VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO };
        stage.stage = VK_SHADER_STAGE_COMPUTE_BIT; stage.module = mod; stage.pName = "main";
        VkComputePipelineCreateInfo cpci{ VK_STRUCTURE_TYPE_COMPUTE_PIPELINE_CREATE_INFO };
        cpci.stage = stage; cpci.layout = outPL;
        VkResult res = vkCreateComputePipelines(ctx.device, VK_NULL_HANDLE, 1, &cpci, nullptr, &outPipe);
        vkDestroyShaderModule(ctx.device, mod, nullptr);
        return res == VK_SUCCESS;
    }

    void dispatch2D(VkCommandBuffer cmd, VkPipeline pipe, VkExtent2D ext) {
        (void)pipe;
        const uint32_t gx = (ext.width  + 7) / 8;
        const uint32_t gy = (ext.height + 7) / 8;
        vkCmdDispatch(cmd, gx, gy, 1);
    }

    void destroyPipelines(VkDevice dev) {
        if (raymarchPipe)    { vkDestroyPipeline(dev, raymarchPipe, nullptr);    raymarchPipe = VK_NULL_HANDLE; }
        if (reconstructPipe) { vkDestroyPipeline(dev, reconstructPipe, nullptr); reconstructPipe = VK_NULL_HANDLE; }
        if (compositePipe)   { vkDestroyPipeline(dev, compositePipe, nullptr);   compositePipe = VK_NULL_HANDLE; }
        if (raymarchPL)      { vkDestroyPipelineLayout(dev, raymarchPL, nullptr);      raymarchPL = VK_NULL_HANDLE; }
        if (reconstructPL)   { vkDestroyPipelineLayout(dev, reconstructPL, nullptr);   reconstructPL = VK_NULL_HANDLE; }
        if (compositePL)     { vkDestroyPipelineLayout(dev, compositePL, nullptr);     compositePL = VK_NULL_HANDLE; }
    }

    void destroyDescriptors(VkDevice dev) {
        if (pool) { vkDestroyDescriptorPool(dev, pool, nullptr); pool = VK_NULL_HANDLE; }
        raymarchSet = reconstructSet = compositeSet = samplerSet = VK_NULL_HANDLE;
        if (raymarchLayout)    { vkDestroyDescriptorSetLayout(dev, raymarchLayout, nullptr);    raymarchLayout = VK_NULL_HANDLE; }
        if (reconstructLayout) { vkDestroyDescriptorSetLayout(dev, reconstructLayout, nullptr); reconstructLayout = VK_NULL_HANDLE; }
        if (compositeLayout)   { vkDestroyDescriptorSetLayout(dev, compositeLayout, nullptr);   compositeLayout = VK_NULL_HANDLE; }
        if (samplerLayout)     { vkDestroyDescriptorSetLayout(dev, samplerLayout, nullptr);     samplerLayout = VK_NULL_HANDLE; }
    }

    void destroyImages(VulkanContext& ctx) {
        auto destroyOne = [&](VkImage& img, VmaAllocation& a, VkImageView& v) {
            if (v)  { vkDestroyImageView(ctx.device, v, nullptr); v = VK_NULL_HANDLE; }
            if (img){ vmaDestroyImage(ctx.allocator, img, a); img = VK_NULL_HANDLE; }
        };
        destroyOne(cloudQuarterImg, cloudQuarterAlloc, cloudQuarterView);
        destroyOne(cloudDepthQuarterImg, cloudDepthQuarterAlloc, cloudDepthQuarterView);
        destroyOne(cloudFullImg, cloudFullAlloc, cloudFullView);
        cloudRtInitialized = false;
    }

    void destroyUBOs(VulkanContext& ctx) {
        for (int i = 0; i < FRAMES_IN_FLIGHT; i++) {
            if (uboBuf[i]) { vmaDestroyBuffer(ctx.allocator, uboBuf[i], uboAlloc[i]); uboBuf[i] = VK_NULL_HANDLE; }
        }
    }
};
