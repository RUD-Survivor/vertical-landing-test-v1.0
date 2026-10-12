#pragma once
#include "../terrain_anchor.h"
#include "systems/vk_scene.h"
#include "vk_taa.h"
#include "../ocean_model.h"
#include "../radiometry.h"
#include "../scene_snapshot.h"

// Single-layer water: independent geometry, opaque color/depth snapshots, surface
// atmosphere reflection, cloud shadows and a dedicated TAA reactive attachment.
// Own only water resources. Terrain topology, descriptors and atmosphere LUTs are reused.
struct OceanUniforms {
    float invViewProj[16];
    float anchorRadius[4];
    float phases[12];
    float sunRadius[4];
    float rayleigh[4];
    float mie[4];
    float absorption[4];
    float scattering[4];
    float params[4];
    float ozone[4];
    float ozoneShape[4];
};
static_assert(sizeof(OceanUniforms)==256, "OceanData std140 layout");

struct VkOceanSystem {
    VkTerrainPipeline pipeline;
    VkDescriptorSetLayout resourcesLayout=VK_NULL_HANDLE;
    VkDescriptorPool pool=VK_NULL_HANDLE;
    VkDescriptorSet sets[2]={};
    VkBuffer ubos[2]={}; VmaAllocation uboAllocs[2]={}; void* mapped[2]={};
    VkImage bedDepth=VK_NULL_HANDLE, reactive=VK_NULL_HANDLE;
    VmaAllocation bedAlloc={}, reactiveAlloc={};
    VkImageView bedView=VK_NULL_HANDLE, reactiveView=VK_NULL_HANDLE;
    VkExtent2D extent={};
    bool imagesReady=false, bedReady=false;
    VkPipeline skyBake=VK_NULL_HANDLE;
    VkPipeline irradianceBake=VK_NULL_HANDLE;
    VkTexture3D skyLut;
    VkTexture2D irradianceLut;
    float skyKey[18]={}; bool skyReady=false;
    int debugView=0; // Radiometry probes; zero is the normal water path.

    bool init(VulkanContext& ctx, VkFormat colorFmt, VkFormat depthFmt) {
        VkDescriptorSetLayoutBinding b[8]={};
        for(uint32_t i=0;i<4;++i) b[i]={i,i==3?VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER:VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,
            1,VkShaderStageFlags(i==3?VK_SHADER_STAGE_VERTEX_BIT|VK_SHADER_STAGE_FRAGMENT_BIT:VK_SHADER_STAGE_FRAGMENT_BIT),nullptr};
        b[2].stageFlags|=VK_SHADER_STAGE_COMPUTE_BIT;b[3].stageFlags|=VK_SHADER_STAGE_COMPUTE_BIT;
        b[4]={4,VK_DESCRIPTOR_TYPE_STORAGE_IMAGE,1,VK_SHADER_STAGE_COMPUTE_BIT,nullptr};
        b[5]={5,VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,1,VK_SHADER_STAGE_FRAGMENT_BIT|VK_SHADER_STAGE_COMPUTE_BIT,nullptr};
        b[6]={6,VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,1,VK_SHADER_STAGE_FRAGMENT_BIT,nullptr};
        b[7]={7,VK_DESCRIPTOR_TYPE_STORAGE_IMAGE,1,VK_SHADER_STAGE_COMPUTE_BIT,nullptr};
        VkDescriptorSetLayoutCreateInfo ci{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO};
        ci.bindingCount=8;ci.pBindings=b;
        if(vkCreateDescriptorSetLayout(ctx.device,&ci,nullptr,&resourcesLayout)!=VK_SUCCESS)return false;
        if(!pipeline.init(ctx,colorFmt,depthFmt,"src/render/shaders/spirv/ocean.vert.spv",
            "src/render/shaders/spirv/ocean.frag.spv",resourcesLayout))return false;
        if(!skyLut.createStorage3D(ctx,64,32,32,VK_FORMAT_R16G16B16A16_SFLOAT))return false;
        if(!irradianceLut.createStorage2D(ctx,32,2,VK_FORMAT_R16G16B16A16_SFLOAT,VK_IMAGE_USAGE_TRANSFER_SRC_BIT))return false;
        // Sky table coordinates include boundary samples; repeat would wrap
        // zenith to horizon or daylight to night during trilinear filtering.
        vkDestroySampler(ctx.device,skyLut.sampler,nullptr);skyLut.sampler=VK_NULL_HANDLE;
        VkSamplerCreateInfo sampler{VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO};sampler.magFilter=sampler.minFilter=VK_FILTER_LINEAR;
        sampler.addressModeU=sampler.addressModeV=sampler.addressModeW=VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
        vkRequireSuccess(vkCreateSampler(ctx.device,&sampler,nullptr,&skyLut.sampler),"ocean sky sampler");
        vkDestroySampler(ctx.device,irradianceLut.sampler,nullptr);irradianceLut.sampler=VK_NULL_HANDLE;
        vkRequireSuccess(vkCreateSampler(ctx.device,&sampler,nullptr,&irradianceLut.sampler),"ocean irradiance sampler");
        auto code=loadSPIRV("src/render/shaders/spirv/ocean_sky.comp.spv");if(code.empty())return false;
        VkShaderModule module=createShaderModule(ctx.device,code);if(!module)return false;
        VkComputePipelineCreateInfo compute{VK_STRUCTURE_TYPE_COMPUTE_PIPELINE_CREATE_INFO};compute.layout=pipeline.layout;
        compute.stage={VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO,nullptr,0,VK_SHADER_STAGE_COMPUTE_BIT,module,"main",nullptr};
        VkResult result=vkCreateComputePipelines(ctx.device,VK_NULL_HANDLE,1,&compute,nullptr,&skyBake);
        vkDestroyShaderModule(ctx.device,module,nullptr);if(result!=VK_SUCCESS)return false;
        code=loadSPIRV("src/render/shaders/spirv/ocean_irradiance.comp.spv");if(code.empty())return false;
        module=createShaderModule(ctx.device,code);if(!module)return false;
        compute.stage.module=module;
        result=vkCreateComputePipelines(ctx.device,VK_NULL_HANDLE,1,&compute,nullptr,&irradianceBake);
        vkDestroyShaderModule(ctx.device,module,nullptr);if(result!=VK_SUCCESS)return false;
        VkDescriptorPoolSize sizes[]={{VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,10},{VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER,2},{VK_DESCRIPTOR_TYPE_STORAGE_IMAGE,4}};
        VkDescriptorPoolCreateInfo pi{VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO};
        pi.maxSets=2;pi.poolSizeCount=3;pi.pPoolSizes=sizes;
        if(vkCreateDescriptorPool(ctx.device,&pi,nullptr,&pool)!=VK_SUCCESS)return false;
        VkDescriptorSetLayout layouts[]={resourcesLayout,resourcesLayout};
        VkDescriptorSetAllocateInfo ai{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO};
        ai.descriptorPool=pool;ai.descriptorSetCount=2;ai.pSetLayouts=layouts;
        if(vkAllocateDescriptorSets(ctx.device,&ai,sets)!=VK_SUCCESS)return false;
        for(int i=0;i<2;++i) {
            VkBufferCreateInfo bi{VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO};bi.size=sizeof(OceanUniforms);bi.usage=VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT;
            VmaAllocationCreateInfo ac{};ac.usage=VMA_MEMORY_USAGE_CPU_TO_GPU;ac.flags=VMA_ALLOCATION_CREATE_MAPPED_BIT;
            VmaAllocationInfo info{};
            if(vmaCreateBuffer(ctx.allocator,&bi,&ac,&ubos[i],&uboAllocs[i],&info)!=VK_SUCCESS)return false;
            mapped[i]=info.pMappedData;
        }
        return true;
    }
    void destroyImages(VulkanContext& ctx) {
        if(bedView)vkDestroyImageView(ctx.device,bedView,nullptr);
        if(reactiveView)vkDestroyImageView(ctx.device,reactiveView,nullptr);
        if(bedDepth)vmaDestroyImage(ctx.allocator,bedDepth,bedAlloc);
        if(reactive)vmaDestroyImage(ctx.allocator,reactive,reactiveAlloc);
        bedView=reactiveView=VK_NULL_HANDLE;bedDepth=reactive=VK_NULL_HANDLE;
        imagesReady=bedReady=false;extent={};
    }
    void shutdown(VulkanContext& ctx) {
        destroyImages(ctx);
        if(skyBake)vkDestroyPipeline(ctx.device,skyBake,nullptr);skyBake=VK_NULL_HANDLE;
        if(irradianceBake)vkDestroyPipeline(ctx.device,irradianceBake,nullptr);irradianceBake=VK_NULL_HANDLE;
        skyLut.destroy(ctx);skyReady=false;
        irradianceLut.destroy(ctx);
        pipeline.shutdown(ctx.device);
        if(pool)vkDestroyDescriptorPool(ctx.device,pool,nullptr);
        pool=VK_NULL_HANDLE;
        for(int i=0;i<2;++i)if(ubos[i]){vmaDestroyBuffer(ctx.allocator,ubos[i],uboAllocs[i]);ubos[i]=VK_NULL_HANDLE;mapped[i]=nullptr;}
        if(resourcesLayout)vkDestroyDescriptorSetLayout(ctx.device,resourcesLayout,nullptr);
        resourcesLayout=VK_NULL_HANDLE;
    }
    void ensureImages(VulkanContext& ctx, VkExtent2D size) {
        if(reactive && extent.width==size.width && extent.height==size.height)return;
        // Only on resize: descriptors/images from both flight slots must retire.
        vkRequireSuccess(vkDeviceWaitIdle(ctx.device),"ocean resize idle");
        destroyImages(ctx);
        for(int i=0;i<2;++i) {
            VkImage& image=i==0?bedDepth:reactive; VmaAllocation& alloc=i==0?bedAlloc:reactiveAlloc;
            VkImageView& view=i==0?bedView:reactiveView;
            VkFormat fmt=i==0?VK_FORMAT_D32_SFLOAT:VK_FORMAT_R8_UNORM;
            VkImageCreateInfo ci{VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO};ci.imageType=VK_IMAGE_TYPE_2D;ci.format=fmt;
            ci.extent={size.width,size.height,1};ci.mipLevels=ci.arrayLayers=1;ci.samples=VK_SAMPLE_COUNT_1_BIT;
            ci.tiling=VK_IMAGE_TILING_OPTIMAL;
            ci.usage=VK_IMAGE_USAGE_SAMPLED_BIT|VK_IMAGE_USAGE_TRANSFER_DST_BIT|VK_IMAGE_USAGE_TRANSFER_SRC_BIT;
            if(i==1)ci.usage|=VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT;
            VmaAllocationCreateInfo ac{};ac.usage=VMA_MEMORY_USAGE_GPU_ONLY;
            vkRequireSuccess(vmaCreateImage(ctx.allocator,&ci,&ac,&image,&alloc,nullptr),"ocean image");
            VkImageViewCreateInfo vi{VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO};vi.image=image;vi.viewType=VK_IMAGE_VIEW_TYPE_2D;vi.format=fmt;
            vi.subresourceRange={VkImageAspectFlags(i==0?VK_IMAGE_ASPECT_DEPTH_BIT:VK_IMAGE_ASPECT_COLOR_BIT),0,1,0,1};
            vkRequireSuccess(vkCreateImageView(ctx.device,&vi,nullptr,&view),"ocean image view");
        }
        extent=size;
    }
    // D32 transfer barriers need an explicit depth aspect; layout names alone
    // cannot distinguish a sampled depth snapshot from a color image.
    static void depthBarrier(VkCommandBuffer cmd, VkImage image, VkImageLayout oldLayout, VkImageLayout newLayout) {
        VkImageMemoryBarrier2 b{VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER_2};
        b.srcStageMask=VK_PIPELINE_STAGE_2_ALL_COMMANDS_BIT;b.srcAccessMask=VK_ACCESS_2_MEMORY_READ_BIT|VK_ACCESS_2_MEMORY_WRITE_BIT;
        b.dstStageMask=VK_PIPELINE_STAGE_2_ALL_COMMANDS_BIT;b.dstAccessMask=VK_ACCESS_2_MEMORY_READ_BIT|VK_ACCESS_2_MEMORY_WRITE_BIT;
        b.oldLayout=oldLayout;b.newLayout=newLayout;b.image=image;
        b.srcQueueFamilyIndex=b.dstQueueFamilyIndex=VK_QUEUE_FAMILY_IGNORED;
        b.subresourceRange={VK_IMAGE_ASPECT_DEPTH_BIT,0,1,0,1};
        VkDependencyInfo d{VK_STRUCTURE_TYPE_DEPENDENCY_INFO};d.imageMemoryBarrierCount=1;d.pImageMemoryBarriers=&b;
        vkCmdPipelineBarrier2(cmd,&d);
    }
    void render(VulkanContext& ctx,VkCommandBuffer cmd,VkTAA& taa,VkSceneSystem& scene,
                const SceneSnapshot& snap,int frameSlot,VkExtent2D size,const AtmoTuneParams& tune) {
        ensureImages(ctx,size);
        transitionImage(cmd,reactive,imagesReady?VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL:VK_IMAGE_LAYOUT_UNDEFINED,
            VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL);
        VkClearColorValue zero{};VkImageSubresourceRange range{VK_IMAGE_ASPECT_COLOR_BIT,0,1,0,1};
        vkCmdClearColorImage(cmd,reactive,VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,&zero,1,&range);
        imagesReady=true;
        taa.updateWaterMask(ctx,reactiveView);
        const CelestialDraw* earth=nullptr;
        for(const auto& cd:snap.celestials)if(cd.bodyIdx==3){earth=&cd;break;}
        if(!earth || earth->radius<=0.001f || snap.terrainPatches.empty() || !scene.terrainGlobalTexSet) {
            transitionImage(cmd,reactive,VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
            return;
        }
        const int slot=frameSlot%2;
        OceanUniforms u{};
        memcpy(u.invViewProj,taa._currInvViewProj,64);
        double local[3]={}, len2=0;
        for(int j=0;j<3;++j) {
            for(int k=0;k<3;++k)local[j]+=double(earth->model[j*4+k])/earth->radius*(double(snap.camPos[k])-earth->center[k]);
            len2+=local[j]*local[j];
        }
        double len=std::sqrt(len2);
        for(int j=0;j<3;++j)u.anchorRadius[j]=len>1e-9?float(local[j]/len):(j==1?1.f:0.f);
        u.anchorRadius[3]=earth->radius*1000;
        OceanModel::phases(u.anchorRadius,u.anchorRadius[3],snap.time,u.phases);
        float light[3]={snap.sunWorldPos[0]-earth->center[0],snap.sunWorldPos[1]-earth->center[1],snap.sunWorldPos[2]-earth->center[2]};
        float ll=std::sqrt(light[0]*light[0]+light[1]*light[1]+light[2]*light[2]);
        for(int j=0;j<3;++j){light[j]=ll>1e-6f?light[j]/ll:snap.lightDir[j];u.sunRadius[j]=light[j];}
        u.sunRadius[3]=earth->radius+tune.shellThicknessKm;
        PlanetScatteringCoeffs sc=getPlanetScatteringCoeffs(earth->atmoIdx);applyAtmoTuneToCoeffs(sc,tune);
        for(int j=0;j<3;++j){u.rayleigh[j]=sc.rayleighCoeff[j];u.mie[j]=sc.mieCoeff;}
        for(int j=0;j<3;++j)u.ozone[j]=sc.ozoneCoeff[j];u.ozone[3]=sc.ozoneCenter;u.ozoneShape[0]=sc.ozoneWidth;
        u.ozoneShape[1]=float(debugView);
        u.ozoneShape[2]=tune.innerExposureNear; // surface-sky display reference, probes only
        u.rayleigh[3]=sc.hRayleigh;u.mie[3]=sc.hMie;
        u.absorption[0]=.18f;u.absorption[1]=.025f;u.absorption[2]=.007f;
        u.scattering[0]=.001f;u.scattering[1]=.002f;u.scattering[2]=.003f;
        // Match terrain's unit-sun illumination, rather than a separate water exposure.
        u.params[0]=Radiometry::solarIrradiance(snap.sunIntensity);u.params[1]=1.f;
        u.params[2]=float(std::max(0.0,(len-earth->radius)*1000));
        u.params[3]=sc.gMie;
        memcpy(mapped[slot],&u,sizeof(u));
        vkRequireSuccess(vmaFlushAllocation(ctx.allocator,uboAllocs[slot],0,sizeof(u)),"flush ocean uniforms");
        // Profile-only lookup: surface reflections must not depend on camera SkyView.
        GpuPerFrameData fd=fillAtmoLutFrameData(snap.camPos,earth->center,earth->radius,
            tune.shellThicknessKm,earth->atmoIdx,light,&tune);
        auto key=AtmoLutProfileKey::fromTune(earth->atmoIdx,earth->radius,tune.shellThicknessKm,tune);
        VkAtmosphereLut* lut=scene.atmoLutCache.getOrCreate(ctx,cmd,frameSlot,earth->bodyIdx,key,fd);
        if(!lut)throw std::runtime_error("ocean atmosphere LUT unavailable");
        VkDescriptorImageInfo skyImages[]={
            {taa.colorSampler,taa.atmosphereInputView,VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL},
            {taa.depthSampler,bedView,VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL},
            {taa.colorSampler,lut->transmittanceView(),VK_IMAGE_LAYOUT_GENERAL},
            {VK_NULL_HANDLE,skyLut.view,VK_IMAGE_LAYOUT_GENERAL},
            {skyLut.sampler,skyLut.view,VK_IMAGE_LAYOUT_GENERAL},
            {irradianceLut.sampler,irradianceLut.view,VK_IMAGE_LAYOUT_GENERAL},
            {VK_NULL_HANDLE,irradianceLut.view,VK_IMAGE_LAYOUT_GENERAL}};
        VkDescriptorBufferInfo skyBuffer{ubos[slot],0,sizeof(u)};
        VkWriteDescriptorSet sw[8]={};
        for(uint32_t i=0;i<8;++i){sw[i].sType=VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;sw[i].dstSet=sets[slot];sw[i].dstBinding=i;sw[i].descriptorCount=1;
            sw[i].descriptorType=i==3?VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER:(i==4 || i==7)?VK_DESCRIPTOR_TYPE_STORAGE_IMAGE:VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
            if(i==3)sw[i].pBufferInfo=&skyBuffer;else sw[i].pImageInfo=&skyImages[i<3?i:i-1];}
        // Update once, BEFORE the set is bound by either compute or graphics.
        // Updating even unused bindings after a dispatch invalidates its command buffer.
        vkUpdateDescriptorSets(ctx.device,8,sw,0,nullptr);
        float nextKey[18]={earth->radius,u.sunRadius[3],u.params[3]};
        memcpy(nextKey+3,u.rayleigh,16);memcpy(nextKey+7,u.mie,16);memcpy(nextKey+11,u.ozone,16);nextKey[15]=u.ozoneShape[0];
        if(!skyReady || memcmp(nextKey,skyKey,sizeof(skyKey))!=0) {
            transitionImage(cmd,skyLut.image,skyReady?VK_IMAGE_LAYOUT_GENERAL:VK_IMAGE_LAYOUT_UNDEFINED,VK_IMAGE_LAYOUT_GENERAL,
                VK_PIPELINE_STAGE_2_ALL_COMMANDS_BIT,VK_ACCESS_2_MEMORY_READ_BIT|VK_ACCESS_2_MEMORY_WRITE_BIT,
                VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT,VK_ACCESS_2_SHADER_WRITE_BIT);
            vkCmdBindPipeline(cmd,VK_PIPELINE_BIND_POINT_COMPUTE,skyBake);
            vkCmdBindDescriptorSets(cmd,VK_PIPELINE_BIND_POINT_COMPUTE,pipeline.layout,2,1,&sets[slot],0,nullptr);
            vkCmdDispatch(cmd,16,8,8);
            transitionImage(cmd,skyLut.image,VK_IMAGE_LAYOUT_GENERAL,VK_IMAGE_LAYOUT_GENERAL,
                VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT,VK_ACCESS_2_SHADER_WRITE_BIT,
                VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT|VK_PIPELINE_STAGE_2_FRAGMENT_SHADER_BIT,VK_ACCESS_2_SHADER_READ_BIT);
            transitionImage(cmd,irradianceLut.image,skyReady?VK_IMAGE_LAYOUT_GENERAL:VK_IMAGE_LAYOUT_UNDEFINED,VK_IMAGE_LAYOUT_GENERAL,
                VK_PIPELINE_STAGE_2_ALL_COMMANDS_BIT,VK_ACCESS_2_MEMORY_READ_BIT|VK_ACCESS_2_MEMORY_WRITE_BIT,
                VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT,VK_ACCESS_2_SHADER_WRITE_BIT);
            vkCmdBindPipeline(cmd,VK_PIPELINE_BIND_POINT_COMPUTE,irradianceBake);
            vkCmdDispatch(cmd,4,2,1);
            transitionImage(cmd,irradianceLut.image,VK_IMAGE_LAYOUT_GENERAL,VK_IMAGE_LAYOUT_GENERAL,
                VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT,VK_ACCESS_2_SHADER_WRITE_BIT,
                VK_PIPELINE_STAGE_2_FRAGMENT_SHADER_BIT,VK_ACCESS_2_SHADER_READ_BIT);
            memcpy(skyKey,nextKey,sizeof(skyKey));skyReady=true;
        }
        depthBarrier(cmd,taa.depthImage,VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL);
        depthBarrier(cmd,bedDepth,bedReady?VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL:VK_IMAGE_LAYOUT_UNDEFINED,VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL);
        VkImageCopy copy{};copy.srcSubresource={VK_IMAGE_ASPECT_DEPTH_BIT,0,0,1};copy.dstSubresource=copy.srcSubresource;
        copy.extent={size.width,size.height,1};
        vkCmdCopyImage(cmd,taa.depthImage,VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,bedDepth,VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,1,&copy);
        depthBarrier(cmd,bedDepth,VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);bedReady=true;
        depthBarrier(cmd,taa.depthImage,VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,VK_IMAGE_LAYOUT_DEPTH_ATTACHMENT_OPTIMAL);
        taa.copyAtmosphereInput(cmd,size); // snapshot opaque HDR, transitions output to attachment
        transitionImage(cmd,reactive,VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL);
        VkRenderingAttachmentInfo colors[2]={};
        for(int i=0;i<2;++i){colors[i].sType=VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO;colors[i].imageView=i==0?taa.colorViews[taa.currentIdx]:reactiveView;
            colors[i].imageLayout=VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;colors[i].loadOp=VK_ATTACHMENT_LOAD_OP_LOAD;colors[i].storeOp=VK_ATTACHMENT_STORE_OP_STORE;}
        VkRenderingAttachmentInfo depth{VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO};depth.imageView=taa.depthView;
        depth.imageLayout=VK_IMAGE_LAYOUT_DEPTH_ATTACHMENT_OPTIMAL;depth.loadOp=VK_ATTACHMENT_LOAD_OP_LOAD;depth.storeOp=VK_ATTACHMENT_STORE_OP_STORE;
        VkRenderingInfo ri{VK_STRUCTURE_TYPE_RENDERING_INFO};ri.renderArea={{0,0},size};ri.layerCount=1;ri.colorAttachmentCount=2;ri.pColorAttachments=colors;ri.pDepthAttachment=&depth;
        vkCmdBeginRendering(cmd,&ri);
        VkViewport vp{0,0,float(size.width),float(size.height),0,1};VkRect2D rect{{0,0},size};
        vkCmdSetViewport(cmd,0,1,&vp);vkCmdSetScissor(cmd,0,1,&rect);
        pipeline.bind(cmd);
        VkDescriptorSet bound[]={scene.terrainSet0[slot],scene.terrainGlobalTexSet,sets[slot]};
        vkCmdBindDescriptorSets(cmd,VK_PIPELINE_BIND_POINT_GRAPHICS,pipeline.layout,0,3,bound,0,nullptr);
        VkDeviceSize offset=0;vkCmdBindVertexBuffers(cmd,0,1,&scene.terrainPatchVbo,&offset);
        vkCmdBindIndexBuffer(cmd,scene.terrainPatchIbo,0,VK_INDEX_TYPE_UINT32);
        for(const auto& tp:snap.terrainPatches){
            TerrainPushConstants p{};memcpy(p.model,tp.model,64);p.planetRadius=tp.planetRadius;p.maxElevation=tp.maxElev;p.nodeLevel=tp.nodeLevel;
            p.lodMorph=tp.lodMorph;p.skirtDepthKm=tp.skirtDepthKm;p.detailWeight=tp.detailWeight;
            memcpy(p.nodePos,tp.nodeCenter,12);memcpy(p.nodeSide,tp.nodeSideA,12);memcpy(p.nodeUp,tp.nodeSideB,12);
            p.anchorGeometry=int(1u|tp.edgeState);packTerrainAnchor(p.model,p.nodePos,p.planetRadius,snap.camPos);
            vkCmdPushConstants(cmd,pipeline.layout,VK_SHADER_STAGE_VERTEX_BIT|VK_SHADER_STAGE_FRAGMENT_BIT,0,sizeof(p),&p);
            vkCmdDrawIndexed(cmd,scene.terrainPatchIcount,1,0,0,0);
        }
        vkCmdEndRendering(cmd);
        depthBarrier(cmd,taa.depthImage,VK_IMAGE_LAYOUT_DEPTH_ATTACHMENT_OPTIMAL,VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
        transitionImage(cmd,taa.colorImages[taa.currentIdx],VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL,VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
        transitionImage(cmd,reactive,VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL,VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
    }
};
