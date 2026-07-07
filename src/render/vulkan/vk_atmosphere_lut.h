#pragma once
// ==========================================================================
// vk_atmosphere_lut.h — 实时大气 LUT 栈：Transmittance / MultiScatter / SkyView
// + Sky Irradiance cubemap。
//
// 对应 shader: src/render/shaders/atmosphere/{transmittance_lut,multi_scatter_lut,
// skyview_lut,sky_irradiance_capture}.glsl，四者共用同一套 descriptor set 布局
// （见 atmosphere/atmosphere_common.glsl）。
//
// 与传统离线烘焙不同：本引擎相机会在同一飞行任务里从行星地表飞到深空，
// 大气 profile（bottomRadius/topRadius/散射系数...）随"当前云所在行星"变化，
// 所以这四个 pass 每帧都对当前行星重新烘焙一次，而不是启动时烘一次静态资源。
//
// 调用顺序（每帧，在 TAA geometry pass 之后、cloud 管线 dispatch 之前）：
//   bake() 内部依次 dispatch: transmittance -> multiScatter -> skyView -> skyIrradiance
//   （后者依赖前者，pass 间用 compute->compute 内存屏障同步）
// ==========================================================================

#include "vk_context.h"
#include "vk_pipeline.h"
#include "vk_utils.h"
#include "vk_frame.h"
#include "gpu_frame_data.h"

#include <cstdio>
#include <cstring>
#include <cmath>

// 按行星索引（AtmoPushConstants::planetIdx）填充 Bruneton 参数化的大气 profile。
// 对应 cloud_rs3d_common.glsl::setupPlanetProfile() 的三个分支，把 rs3d 的
// "指数衰减系数 + 标高" 写法翻译成 Bruneton LUT 需要的 密度 profile 参数。
inline void fillPlanetAtmosphereProfile(GpuAtmosphereConfig& cfg, int32_t atmoIdx,
                                        float surfaceRadiusKm, float atmosphereThicknessKm,
                                        float cloudMinAltKm, float cloudMaxAltKm, float cloudExtinction) {
    float rayleighCoeff[3];   float mieCoeff;
    float hRayleigh, hMie, gMie;
    float ozoneCoeff[3]; float ozoneCenter, ozoneWidth;

    if (atmoIdx == 3) { // 地球类：强 Rayleigh、薄臭氧
        rayleighCoeff[0]=5.8e-3f*1.2f; rayleighCoeff[1]=13.5e-3f*1.2f; rayleighCoeff[2]=33.1e-3f*1.2f;
        mieCoeff = 5.0e-3f; hRayleigh = 8.0f; hMie = 1.0f; gMie = 0.8f;
        ozoneCoeff[0]=0.35e-3f; ozoneCoeff[1]=0.85e-3f; ozoneCoeff[2]=0.09e-3f;
        ozoneCenter = 25.0f; ozoneWidth = 15.0f;
    } else if (atmoIdx == 2) { // 厚大气类：强 Mie、厚臭氧
        rayleighCoeff[0]=5.0e-3f; rayleighCoeff[1]=12.0e-3f; rayleighCoeff[2]=28.0e-3f;
        mieCoeff = 0.04f; hRayleigh = 15.0f; hMie = 5.0f; gMie = 0.76f;
        ozoneCoeff[0]=0.5e-3f; ozoneCoeff[1]=5.0e-3f; ozoneCoeff[2]=20.0e-3f;
        ozoneCenter = 50.0f; ozoneWidth = 20.0f;
    } else { // 默认地球类，无臭氧
        rayleighCoeff[0]=5.8e-3f; rayleighCoeff[1]=13.5e-3f; rayleighCoeff[2]=33.1e-3f;
        mieCoeff = 0.005f; hRayleigh = 8.0f; hMie = 1.0f; gMie = 0.8f;
        ozoneCoeff[0]=ozoneCoeff[1]=ozoneCoeff[2]=0.0f;
        ozoneCenter = 1.0f; ozoneWidth = 1.0f;
    }

    cfg.atmospherePreExposure = 1.0f;
    cfg.bottomRadius = surfaceRadiusKm;
    cfg.topRadius    = surfaceRadiusKm + atmosphereThicknessKm;
    cfg.multipleScatteringFactor = 1.0f;
    cfg.miePhaseFunctionG = gMie;
    cfg.viewRayMarchMinSPP = 4;
    cfg.viewRayMarchMaxSPP = 14;
    cfg.groundAlbedo[0] = cfg.groundAlbedo[1] = cfg.groundAlbedo[2] = 0.3f;

    // color*length 分解：直接把系数塞进 color，length 恒为 1，与
    // shared_atmosphere.glsl::getAtmosphereParameters() 的读法（color*length）对应。
    for (int i=0;i<3;i++) cfg.rayleighScatteringColor[i] = rayleighCoeff[i];
    cfg.rayleighScatterLength = 1.0f;
    for (int i=0;i<3;i++) cfg.mieScatteringColor[i] = mieCoeff;
    cfg.mieScatteringLength = 1.0f;
    // Mie 吸收取散射的 10% 作为近似（rs3d 原模型未拆分 mie 散射/吸收）。
    for (int i=0;i<3;i++) cfg.mieAbsColor[i] = mieCoeff * 0.10f;
    cfg.mieAbsLength = 1.0f;
    for (int i=0;i<3;i++) cfg.absorptionColor[i] = ozoneCoeff[i];
    cfg.absorptionLength = 1.0f;

    // 指数衰减 scale：density = exp(-height/H) ⇒ expScale = -1/H。
    // 只用到 rayleighDensity[1].w / mieDensity[1].w（见 shared_atmosphere.glsl）。
    cfg.rayleighDensity[1][3] = -1.0f / hRayleigh;
    cfg.mieDensity[1][3]      = -1.0f / hMie;

    // 臭氧层用两段线性 tent 近似 rs3d 的高斯型 ozoneDensity()：
    // [center-width, center] 段从 0 线性升到 1，[center, center+width] 段从 1 降到 0。
    // absorptionDensity0LayerWidth 是分段断点（取在 center，见推导）。
    cfg.absorptionDensity[0][0] = ozoneCenter;                    // layerWidth（断点）
    cfg.absorptionDensity[0][3] = 1.0f / ozoneWidth;              // linearTerm0
    cfg.absorptionDensity[1][0] = 1.0f - ozoneCenter / ozoneWidth; // constantTerm0
    cfg.absorptionDensity[2][0] = -1.0f / ozoneWidth;             // linearTerm1
    cfg.absorptionDensity[2][1] = 1.0f + ozoneCenter / ozoneWidth; // constantTerm1

    cfg.cloudAreaStartHeight = surfaceRadiusKm + cloudMinAltKm;
    cfg.cloudAreaThickness   = cloudMaxAltKm - cloudMinAltKm;
    cfg.cloudGodRayScale = 1.0f;
    cfg.cloudShadowExtent = 1.0f;

    cfg.cloudWeatherUVScale[0] = cfg.cloudWeatherUVScale[1] = 1.0f;
    cfg.cloudCoverage = 0.45f;
    cfg.cloudDensity  = 1.0f;
    cfg.cloudShadingSunLightScale = 1.0f;
    cfg.cloudFogFade = 1.0f;
    cfg.cloudMaxTraceingDistance = 350.0f;
    cfg.cloudTracingStartMaxDistance = 1e8f;
    cfg.cloudDirection[0]=1.0f; cfg.cloudDirection[1]=0.0f; cfg.cloudDirection[2]=0.0f;
    cfg.cloudSpeed = 0.15f;
    cfg.cloudMultiScatterExtinction = 0.5f;
    cfg.cloudMultiScatterScatter = 0.5f;
    // 行星尺度关键：cloudMap() 里这个 scale 直接乘在"公里"量级的 posKm 上
    // （posKm 现在是相对行星中心的真实公里坐标，不再是近地小范围坐标）。
    // flower 原版 0.4/1.2 是给"几公里量级"的近地场景调的，直接套用在几千公里量级的
    // 星球上会让 128³/32³ 噪声纹理在屏幕空间里重复几千次——compute shader 里 texture()
    // 没有隐式 mip LOD（噪声纹理本身也只建了 1 级 mip），会退化成极端欠采样，
    // 看起来就是满屏白色雪花噪点而不是云。这里把频率降到"一个纹理周期对应几百公里"，
    // 对应真实天气系统的尺度。
    cfg.cloudBasicNoiseScale = 0.4f;
    cfg.cloudDetailNoiseScale = 1.2f;
    cfg.cloudAlbedo[0]=cfg.cloudAlbedo[1]=cfg.cloudAlbedo[2]=1.0f;
    cfg.cloudPhaseForward = 0.8f;
    cfg.cloudPhaseBackward = -0.2f;
    cfg.cloudPhaseMixFactor = 0.5f;
    cfg.cloudPowderScale = 1.0f;
    cfg.cloudPowderPow = 1.0f;
    cfg.cloudLightStepMul = 1.5f;
    cfg.cloudLightBasicStep = 0.033f;
    cfg.cloudLightStepNum = 6;
    cfg.cloudEnableGroundContribution = 1;
    cfg.cloudMarchingStepNum = 64;
    cfg.cloudSunLitMapOctave = 1;
    cfg.cloudNoiseScale = 1.0f;
    cfg.cloudGodRay = 0; // Phase 2 待接入（依赖 SDSM 级联阴影）
    (void)cloudExtinction; // gCloudExtinction 由 cloud_common.glsl 侧的 CloudTuneParams 提供，这里不重复
}

struct VkAtmosphereLut {
    static constexpr uint32_t kTransW = 256, kTransH = 64;
    static constexpr uint32_t kMultiScatterSize = 32;
    static constexpr uint32_t kSkyViewW = 192, kSkyViewH = 108;
    static constexpr uint32_t kCubeSize = 32;
    static constexpr VkFormat kFmt = VK_FORMAT_R16G16B16A16_SFLOAT;

    VkImage image_[3] = {}; VmaAllocation alloc_[3] = {}; VkImageView view_[3] = {}; // trans/multiscatter/skyview
    VkImage cubeImage = VK_NULL_HANDLE; VmaAllocation cubeAlloc = VK_NULL_HANDLE; VkImageView cubeView = VK_NULL_HANDLE;

    VkBuffer      uboBuf[FRAMES_IN_FLIGHT]   = {};
    VmaAllocation uboAlloc[FRAMES_IN_FLIGHT] = {};
    void*         uboMapped[FRAMES_IN_FLIGHT]= {};

    VkDescriptorSetLayout setLayout = VK_NULL_HANDLE;
    VkPipelineLayout      pipelineLayout = VK_NULL_HANDLE;
    VkDescriptorPool      pool = VK_NULL_HANDLE;
    VkDescriptorSet       descSet[FRAMES_IN_FLIGHT] = {};

    VkPipeline transmittancePipe = VK_NULL_HANDLE;
    VkPipeline multiScatterPipe  = VK_NULL_HANDLE;
    VkPipeline skyViewPipe       = VK_NULL_HANDLE;
    VkPipeline skyIrradiancePipe = VK_NULL_HANDLE;

    VkSampler linearClampSampler = VK_NULL_HANDLE;

    bool init(VulkanContext& ctx) {
        if (!createSampler(ctx))    return false;
        if (!createImages(ctx))     return false;
        if (!createUBOs(ctx))       return false;
        if (!createDescriptors(ctx))return false;
        if (!createPipelines(ctx))  return false;
        printf("[VkAtmosphereLut] Ready: trans %ux%u, multiScatter %ux%u, skyView %ux%u, cube %ux%ux6\n",
               kTransW, kTransH, kMultiScatterSize, kMultiScatterSize, kSkyViewW, kSkyViewH, kCubeSize, kCubeSize);
        return true;
    }

    void shutdown(VulkanContext& ctx) {
        vkDeviceWaitIdle(ctx.device);
        auto destroyPipe = [&](VkPipeline& p){ if(p){vkDestroyPipeline(ctx.device,p,nullptr);p=VK_NULL_HANDLE;} };
        destroyPipe(transmittancePipe); destroyPipe(multiScatterPipe);
        destroyPipe(skyViewPipe); destroyPipe(skyIrradiancePipe);
        if (pipelineLayout) { vkDestroyPipelineLayout(ctx.device, pipelineLayout, nullptr); pipelineLayout = VK_NULL_HANDLE; }
        if (pool) { vkDestroyDescriptorPool(ctx.device, pool, nullptr); pool = VK_NULL_HANDLE; }
        if (setLayout) { vkDestroyDescriptorSetLayout(ctx.device, setLayout, nullptr); setLayout = VK_NULL_HANDLE; }
        for (int i=0;i<FRAMES_IN_FLIGHT;i++)
            if (uboBuf[i]) { vmaDestroyBuffer(ctx.allocator, uboBuf[i], uboAlloc[i]); uboBuf[i]=VK_NULL_HANDLE; }
        for (int i=0;i<3;i++) {
            if (view_[i])  { vkDestroyImageView(ctx.device, view_[i], nullptr); view_[i]=VK_NULL_HANDLE; }
            if (image_[i]) { vmaDestroyImage(ctx.allocator, image_[i], alloc_[i]); image_[i]=VK_NULL_HANDLE; }
        }
        if (cubeView)  { vkDestroyImageView(ctx.device, cubeView, nullptr); cubeView=VK_NULL_HANDLE; }
        if (cubeImage) { vmaDestroyImage(ctx.allocator, cubeImage, cubeAlloc); cubeImage=VK_NULL_HANDLE; }
        if (linearClampSampler) { vkDestroySampler(ctx.device, linearClampSampler, nullptr); linearClampSampler=VK_NULL_HANDLE; }
    }

    VkImageView transmittanceView() const { return view_[0]; }
    VkImageView skyViewLutView()    const { return view_[2]; }
    VkImageView skyIrradianceCubeView() const { return cubeView; }
    VkBuffer    frameUBO(int frameSlot) const { return uboBuf[frameSlot % FRAMES_IN_FLIGHT]; }

    // 每帧调用一次：把调用方已经填好的 GpuPerFrameData（相机矩阵 + sky/atmosphere 配置，
    // 见 vk_cloud_system.h::fillCloudFrameData）写入 UBO，并依次 dispatch 四个 bake pass。
    // 与 cloud_common.glsl（binding 21）共用同一份每帧数据，天空/大气参数只需算一次。
    void bake(VulkanContext& ctx, VkCommandBuffer cmd, int frameSlot, const GpuPerFrameData& fd) {
        int slot = frameSlot % FRAMES_IN_FLIGHT;
        memcpy(uboMapped[slot], &fd, sizeof(fd));

        // UBO 内容由 CPU 写，这里保证 host write 在 compute 读之前可见。
        VkMemoryBarrier2 hostBarrier{ VK_STRUCTURE_TYPE_MEMORY_BARRIER_2 };
        hostBarrier.srcStageMask = VK_PIPELINE_STAGE_2_HOST_BIT; hostBarrier.srcAccessMask = VK_ACCESS_2_HOST_WRITE_BIT;
        hostBarrier.dstStageMask = VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT; hostBarrier.dstAccessMask = VK_ACCESS_2_UNIFORM_READ_BIT;
        VkDependencyInfo hdep{ VK_STRUCTURE_TYPE_DEPENDENCY_INFO }; hdep.memoryBarrierCount=1; hdep.pMemoryBarriers=&hostBarrier;
        vkCmdPipelineBarrier2(cmd, &hdep);

        VkDescriptorSet set = descSet[slot];
        vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_COMPUTE, pipelineLayout, 0, 1, &set, 0, nullptr);

        vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_COMPUTE, transmittancePipe);
        vkCmdDispatch(cmd, (kTransW+7)/8, (kTransH+7)/8, 1);
        computeBarrier(cmd);

        vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_COMPUTE, multiScatterPipe);
        vkCmdDispatch(cmd, kMultiScatterSize, kMultiScatterSize, 1); // local_size_z = 64 覆盖采样循环
        computeBarrier(cmd);

        vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_COMPUTE, skyViewPipe);
        vkCmdDispatch(cmd, (kSkyViewW+7)/8, (kSkyViewH+7)/8, 1);
        computeBarrier(cmd);

        vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_COMPUTE, skyIrradiancePipe);
        vkCmdDispatch(cmd, (kCubeSize+7)/8, (kCubeSize+7)/8, 6);
        computeBarrier(cmd);
    }

private:
    static void computeBarrier(VkCommandBuffer cmd) {
        VkMemoryBarrier2 b{ VK_STRUCTURE_TYPE_MEMORY_BARRIER_2 };
        b.srcStageMask = VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT; b.srcAccessMask = VK_ACCESS_2_SHADER_WRITE_BIT;
        b.dstStageMask = VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT; b.dstAccessMask = VK_ACCESS_2_SHADER_READ_BIT | VK_ACCESS_2_SHADER_WRITE_BIT;
        VkDependencyInfo dep{ VK_STRUCTURE_TYPE_DEPENDENCY_INFO }; dep.memoryBarrierCount=1; dep.pMemoryBarriers=&b;
        vkCmdPipelineBarrier2(cmd, &dep);
    }

    bool createSampler(VulkanContext& ctx) {
        VkSamplerCreateInfo sci{ VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO };
        sci.magFilter = VK_FILTER_LINEAR; sci.minFilter = VK_FILTER_LINEAR;
        sci.addressModeU = sci.addressModeV = sci.addressModeW = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
        return vkCreateSampler(ctx.device, &sci, nullptr, &linearClampSampler) == VK_SUCCESS;
    }

    bool createImage2D(VulkanContext& ctx, uint32_t w, uint32_t h, VkImage& img, VmaAllocation& alloc, VkImageView& view) {
        VkImageCreateInfo ici{ VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO };
        ici.imageType = VK_IMAGE_TYPE_2D; ici.format = kFmt; ici.extent = {w,h,1};
        ici.mipLevels = 1; ici.arrayLayers = 1; ici.samples = VK_SAMPLE_COUNT_1_BIT;
        ici.tiling = VK_IMAGE_TILING_OPTIMAL;
        ici.usage = VK_IMAGE_USAGE_STORAGE_BIT | VK_IMAGE_USAGE_SAMPLED_BIT;
        ici.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
        VmaAllocationCreateInfo aci{}; aci.usage = VMA_MEMORY_USAGE_GPU_ONLY;
        if (vmaCreateImage(ctx.allocator, &ici, &aci, &img, &alloc, nullptr) != VK_SUCCESS) return false;

        VkImageViewCreateInfo vci{ VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO };
        vci.image = img; vci.viewType = VK_IMAGE_VIEW_TYPE_2D; vci.format = kFmt;
        vci.subresourceRange = { VK_IMAGE_ASPECT_COLOR_BIT, 0,1,0,1 };
        if (vkCreateImageView(ctx.device, &vci, nullptr, &view) != VK_SUCCESS) return false;

        // 常驻 GENERAL layout：同一 pass 内既作为 storage image 写入，也在后续 pass
        // 里作为 sampled image 读取，不做逐帧布局切换（LUT 是每帧全量重算，无需保留跨帧只读态）。
        VkCommandBuffer cmd = beginSingleTimeCommands(ctx);
        transitionImage(cmd, img, VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_GENERAL,
            VK_PIPELINE_STAGE_2_TOP_OF_PIPE_BIT, VK_ACCESS_2_NONE,
            VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT, VK_ACCESS_2_SHADER_READ_BIT | VK_ACCESS_2_SHADER_WRITE_BIT);
        endSingleTimeCommands(ctx, cmd);
        return true;
    }

    bool createImages(VulkanContext& ctx) {
        if (!createImage2D(ctx, kTransW, kTransH, image_[0], alloc_[0], view_[0])) return false;
        if (!createImage2D(ctx, kMultiScatterSize, kMultiScatterSize, image_[1], alloc_[1], view_[1])) return false;
        if (!createImage2D(ctx, kSkyViewW, kSkyViewH, image_[2], alloc_[2], view_[2])) return false;

        VkImageCreateInfo ici{ VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO };
        ici.flags = VK_IMAGE_CREATE_CUBE_COMPATIBLE_BIT;
        ici.imageType = VK_IMAGE_TYPE_2D; ici.format = kFmt; ici.extent = {kCubeSize,kCubeSize,1};
        ici.mipLevels = 1; ici.arrayLayers = 6; ici.samples = VK_SAMPLE_COUNT_1_BIT;
        ici.tiling = VK_IMAGE_TILING_OPTIMAL;
        ici.usage = VK_IMAGE_USAGE_STORAGE_BIT | VK_IMAGE_USAGE_SAMPLED_BIT;
        ici.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
        VmaAllocationCreateInfo aci{}; aci.usage = VMA_MEMORY_USAGE_GPU_ONLY;
        if (vmaCreateImage(ctx.allocator, &ici, &aci, &cubeImage, &cubeAlloc, nullptr) != VK_SUCCESS) return false;

        VkImageViewCreateInfo vci{ VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO };
        vci.image = cubeImage; vci.viewType = VK_IMAGE_VIEW_TYPE_CUBE; vci.format = kFmt;
        vci.subresourceRange = { VK_IMAGE_ASPECT_COLOR_BIT, 0,1,0,6 };
        if (vkCreateImageView(ctx.device, &vci, nullptr, &cubeView) != VK_SUCCESS) return false;

        VkCommandBuffer cmd = beginSingleTimeCommands(ctx);
        transitionImage(cmd, cubeImage, VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_GENERAL,
            VK_PIPELINE_STAGE_2_TOP_OF_PIPE_BIT, VK_ACCESS_2_NONE,
            VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT, VK_ACCESS_2_SHADER_READ_BIT | VK_ACCESS_2_SHADER_WRITE_BIT);
        endSingleTimeCommands(ctx, cmd);
        return true;
    }

    bool createUBOs(VulkanContext& ctx) {
        VkBufferCreateInfo bci{ VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO };
        bci.size = sizeof(GpuPerFrameData);
        bci.usage = VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT;
        VmaAllocationCreateInfo aci{};
        aci.usage = VMA_MEMORY_USAGE_CPU_TO_GPU;
        aci.flags = VMA_ALLOCATION_CREATE_MAPPED_BIT;
        for (int i=0;i<FRAMES_IN_FLIGHT;i++) {
            VmaAllocationInfo info{};
            if (vmaCreateBuffer(ctx.allocator, &bci, &aci, &uboBuf[i], &uboAlloc[i], &info) != VK_SUCCESS) return false;
            uboMapped[i] = info.pMappedData;
        }
        return true;
    }

    bool createDescriptors(VulkanContext& ctx) {
        VkDescriptorSetLayoutBinding b[10]{};
        b[0] = { 0, VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr };
        b[1] = { 1, VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr };
        b[2] = { 2, VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr };
        b[3] = { 3, VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr };
        b[4] = { 4, VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr };
        b[5] = { 5, VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr };
        b[6] = { 6, VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr };
        b[7] = { 7, VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr };
        b[8] = { 8, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr };
        b[9] = { 9, VK_DESCRIPTOR_TYPE_SAMPLER, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr };
        VkDescriptorSetLayoutCreateInfo lci{ VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO };
        lci.bindingCount = 10; lci.pBindings = b;
        if (vkCreateDescriptorSetLayout(ctx.device, &lci, nullptr, &setLayout) != VK_SUCCESS) return false;

        VkPipelineLayoutCreateInfo plci{ VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO };
        plci.setLayoutCount = 1; plci.pSetLayouts = &setLayout;
        if (vkCreatePipelineLayout(ctx.device, &plci, nullptr, &pipelineLayout) != VK_SUCCESS) return false;

        VkDescriptorPoolSize sizes[] = {
            { VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, 4 * FRAMES_IN_FLIGHT },
            { VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE, 4 * FRAMES_IN_FLIGHT },
            { VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1 * FRAMES_IN_FLIGHT },
            { VK_DESCRIPTOR_TYPE_SAMPLER, 1 * FRAMES_IN_FLIGHT },
        };
        VkDescriptorPoolCreateInfo pci{ VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO };
        pci.maxSets = FRAMES_IN_FLIGHT; pci.poolSizeCount = 4; pci.pPoolSizes = sizes;
        if (vkCreateDescriptorPool(ctx.device, &pci, nullptr, &pool) != VK_SUCCESS) return false;

        for (int i=0;i<FRAMES_IN_FLIGHT;i++) {
            VkDescriptorSetAllocateInfo ai{ VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO };
            ai.descriptorPool = pool; ai.descriptorSetCount = 1; ai.pSetLayouts = &setLayout;
            if (vkAllocateDescriptorSets(ctx.device, &ai, &descSet[i]) != VK_SUCCESS) return false;

            VkDescriptorImageInfo iTrans{ VK_NULL_HANDLE, view_[0], VK_IMAGE_LAYOUT_GENERAL };
            VkDescriptorImageInfo iMulti{ VK_NULL_HANDLE, view_[1], VK_IMAGE_LAYOUT_GENERAL };
            VkDescriptorImageInfo iSky  { VK_NULL_HANDLE, view_[2], VK_IMAGE_LAYOUT_GENERAL };
            VkDescriptorImageInfo iCube { VK_NULL_HANDLE, cubeView, VK_IMAGE_LAYOUT_GENERAL };
            VkDescriptorImageInfo iSampler{ linearClampSampler, VK_NULL_HANDLE, VK_IMAGE_LAYOUT_UNDEFINED };
            VkDescriptorBufferInfo iUbo{ uboBuf[i], 0, sizeof(GpuPerFrameData) };

            VkWriteDescriptorSet w[10]{};
            auto imgWrite = [&](int idx, uint32_t binding, VkDescriptorType type, VkDescriptorImageInfo* info) {
                w[idx].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET; w[idx].dstSet = descSet[i];
                w[idx].dstBinding = binding; w[idx].descriptorType = type; w[idx].descriptorCount = 1; w[idx].pImageInfo = info;
            };
            imgWrite(0, 0, VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, &iTrans);
            imgWrite(1, 1, VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE, &iTrans);
            imgWrite(2, 2, VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, &iSky);
            imgWrite(3, 3, VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE, &iSky);
            imgWrite(4, 4, VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, &iMulti);
            imgWrite(5, 5, VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE, &iMulti);
            // binding 6 (inDepth) 只在 depthBufferValue>=0 分支被采样，本管线始终传 -1，绑同一张
            // transmittance view 充数即可（未被实际读取，只是满足 layout 合法性）。
            imgWrite(6, 6, VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE, &iTrans);
            imgWrite(7, 7, VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, &iCube);
            w[8].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET; w[8].dstSet = descSet[i];
            w[8].dstBinding = 8; w[8].descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER; w[8].descriptorCount = 1; w[8].pBufferInfo = &iUbo;
            imgWrite(9, 9, VK_DESCRIPTOR_TYPE_SAMPLER, &iSampler);
            vkUpdateDescriptorSets(ctx.device, 10, w, 0, nullptr);
        }
        return true;
    }

    bool createPipelines(VulkanContext& ctx) {
        auto make = [&](const char* spv, VkPipeline& outPipe) -> bool {
            auto code = loadSPIRV(spv);
            if (code.empty()) { fprintf(stderr, "[VkAtmosphereLut] Missing spv: %s\n", spv); return false; }
            VkShaderModule mod = createShaderModule(ctx.device, code);
            if (mod == VK_NULL_HANDLE) return false;
            VkPipelineShaderStageCreateInfo stage{ VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO };
            stage.stage = VK_SHADER_STAGE_COMPUTE_BIT; stage.module = mod; stage.pName = "main";
            VkComputePipelineCreateInfo cpci{ VK_STRUCTURE_TYPE_COMPUTE_PIPELINE_CREATE_INFO };
            cpci.stage = stage; cpci.layout = pipelineLayout;
            VkResult res = vkCreateComputePipelines(ctx.device, VK_NULL_HANDLE, 1, &cpci, nullptr, &outPipe);
            vkDestroyShaderModule(ctx.device, mod, nullptr);
            return res == VK_SUCCESS;
        };
        return make("src/render/shaders/spirv/transmittance_lut.comp.spv", transmittancePipe)
            && make("src/render/shaders/spirv/multi_scatter_lut.comp.spv", multiScatterPipe)
            && make("src/render/shaders/spirv/skyview_lut.comp.spv", skyViewPipe)
            && make("src/render/shaders/spirv/sky_irradiance_capture.comp.spv", skyIrradiancePipe);
    }
};
