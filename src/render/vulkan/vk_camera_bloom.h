#pragma once
#include "vk_texture.h"
#include "vk_pipeline.h"
#include "vk_taa.h"
#include "../scene_snapshot.h"
#include "../solar_optics.h"

// Scene-wide scatter complement to the dedicated solar optical response.
// Source partition prevents double bloom. All resources are per waited frame
// slot; extent changes recreate both slots after the existing resize fence.
struct VkCameraBloom {
    static constexpr float scatterFraction=.005f;
    struct Params {
        float sun[2]={},aspect=1,focalY=1,sinRadius=0;
        int removeSolar=0,operation=0;float sigma=1;
        int axis[2]={};float fraction=scatterFraction,pad=0;
    };
    static_assert(sizeof(Params)==48,"Bloom push ABI");
    struct Frame {
        VkTexture2D a[5],b[5];VkExtent2D sizes[5]={};
        VkDescriptorSet down[5]={},horizontal[5]={},vertical[5]={},compose={};
        bool initialized=false;
    } frames[2];
    VkExtent2D extent{};
    VkDescriptorSetLayout computeSet=VK_NULL_HANDLE,composeSet=VK_NULL_HANDLE;
    VkDescriptorPool pool=VK_NULL_HANDLE;VkPipelineLayout computeLayout=VK_NULL_HANDLE;
    VkPipeline computePipeline=VK_NULL_HANDLE;VkLensFlarePipeline composePipeline;

    bool init(VulkanContext& ctx,VkFormat color,VkFormat depth){
        VkDescriptorSetLayoutBinding computeBindings[]={
            {0,VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,1,VK_SHADER_STAGE_COMPUTE_BIT,nullptr},
            {1,VK_DESCRIPTOR_TYPE_STORAGE_IMAGE,1,VK_SHADER_STAGE_COMPUTE_BIT,nullptr},
            {2,VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,1,VK_SHADER_STAGE_COMPUTE_BIT,nullptr}};
        VkDescriptorSetLayoutCreateInfo set{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO};set.bindingCount=3;set.pBindings=computeBindings;
        vkRequireSuccess(vkCreateDescriptorSetLayout(ctx.device,&set,nullptr,&computeSet),"bloom compute set");
        VkDescriptorSetLayoutBinding composeBindings[]={
            {0,VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,1,VK_SHADER_STAGE_FRAGMENT_BIT,nullptr},
            {1,VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,5,VK_SHADER_STAGE_FRAGMENT_BIT,nullptr},
            {2,VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,1,VK_SHADER_STAGE_FRAGMENT_BIT,nullptr}};
        set.pBindings=composeBindings;
        vkRequireSuccess(vkCreateDescriptorSetLayout(ctx.device,&set,nullptr,&composeSet),"bloom compose set");
        VkPushConstantRange range{VK_SHADER_STAGE_COMPUTE_BIT,0,sizeof(Params)};
        VkPipelineLayoutCreateInfo layout{VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO};layout.setLayoutCount=1;layout.pSetLayouts=&computeSet;
        layout.pushConstantRangeCount=1;layout.pPushConstantRanges=&range;
        vkRequireSuccess(vkCreatePipelineLayout(ctx.device,&layout,nullptr,&computeLayout),"bloom compute layout");
        auto code=loadSPIRV("src/render/shaders/spirv/camera_bloom.comp.spv");if(code.empty())return false;
        VkShaderModule module=createShaderModule(ctx.device,code);if(!module)return false;
        VkComputePipelineCreateInfo pipeline{VK_STRUCTURE_TYPE_COMPUTE_PIPELINE_CREATE_INFO};pipeline.layout=computeLayout;
        pipeline.stage={VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO};pipeline.stage.stage=VK_SHADER_STAGE_COMPUTE_BIT;pipeline.stage.module=module;pipeline.stage.pName="main";
        VkResult result=vkCreateComputePipelines(ctx.device,VK_NULL_HANDLE,1,&pipeline,nullptr,&computePipeline);
        vkDestroyShaderModule(ctx.device,module,nullptr);vkRequireSuccess(result,"bloom compute pipeline");
        return composePipeline.init(ctx,color,depth,"src/render/shaders/spirv/camera_bloom.vert.spv","src/render/shaders/spirv/camera_bloom.frag.spv",composeSet);
    }
    void releaseImages(VulkanContext& ctx){
        for(auto& frame:frames){for(int i=0;i<5;++i){frame.a[i].destroy(ctx);frame.b[i].destroy(ctx);}frame.initialized=false;}
        if(pool)vkDestroyDescriptorPool(ctx.device,pool,nullptr);pool=VK_NULL_HANDLE;extent={};
    }
    bool ensure(VulkanContext& ctx,VkExtent2D size){
        if(size.width==extent.width&&size.height==extent.height)return true;
        vkRequireSuccess(vkDeviceWaitIdle(ctx.device),"bloom resize wait");releaseImages(ctx);
        VkDescriptorPoolSize counts[]={{VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,256},{VK_DESCRIPTOR_TYPE_STORAGE_IMAGE,64}};
        VkDescriptorPoolCreateInfo poolInfo{VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO};poolInfo.maxSets=32;poolInfo.poolSizeCount=2;poolInfo.pPoolSizes=counts;
        vkRequireSuccess(vkCreateDescriptorPool(ctx.device,&poolInfo,nullptr,&pool),"bloom pool");
        auto allocate=[&](VkDescriptorSetLayout layout,VkDescriptorSet& set){
            VkDescriptorSetAllocateInfo info{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO};info.descriptorPool=pool;info.descriptorSetCount=1;info.pSetLayouts=&layout;
            vkRequireSuccess(vkAllocateDescriptorSets(ctx.device,&info,&set),"bloom set");};
        for(auto& frame:frames){VkExtent2D level=size;
            for(int i=0;i<5;++i){level={(level.width+1)/2,(level.height+1)/2};frame.sizes[i]=level;
                if(!frame.a[i].createStorage2D(ctx,level.width,level.height,VK_FORMAT_R16G16B16A16_SFLOAT)||
                   !frame.b[i].createStorage2D(ctx,level.width,level.height,VK_FORMAT_R16G16B16A16_SFLOAT))return false;
                for(auto* texture:{&frame.a[i],&frame.b[i]}){
                    vkDestroySampler(ctx.device,texture->sampler,nullptr);
                    VkSamplerCreateInfo sampler{VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO};sampler.magFilter=sampler.minFilter=VK_FILTER_LINEAR;
                    sampler.mipmapMode=VK_SAMPLER_MIPMAP_MODE_NEAREST;sampler.addressModeU=sampler.addressModeV=sampler.addressModeW=VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
                    vkRequireSuccess(vkCreateSampler(ctx.device,&sampler,nullptr,&texture->sampler),"bloom clamp sampler");
                }
                allocate(computeSet,frame.down[i]);allocate(computeSet,frame.horizontal[i]);allocate(computeSet,frame.vertical[i]);
            }allocate(composeSet,frame.compose);
        }extent=size;return true;
    }
    static void visible(VkCommandBuffer cmd,VkPipelineStageFlags2 stages){
        VkMemoryBarrier2 barrier{VK_STRUCTURE_TYPE_MEMORY_BARRIER_2};barrier.srcStageMask=VK_PIPELINE_STAGE_2_ALL_COMMANDS_BIT;
        barrier.srcAccessMask=VK_ACCESS_2_MEMORY_WRITE_BIT|VK_ACCESS_2_MEMORY_READ_BIT;barrier.dstStageMask=stages;
        barrier.dstAccessMask=VK_ACCESS_2_SHADER_READ_BIT|VK_ACCESS_2_SHADER_WRITE_BIT;
        VkDependencyInfo dependency{VK_STRUCTURE_TYPE_DEPENDENCY_INFO};dependency.memoryBarrierCount=1;dependency.pMemoryBarriers=&barrier;
        vkCmdPipelineBarrier2(cmd,&dependency);
    }
    void render(VulkanContext& ctx,VkCommandBuffer cmd,VkTAA& taa,const SceneSnapshot& snap,int slot,VkExtent2D size,VkBuffer quad){
        if(!size.width||!size.height||!quad)return;
        if(!ensure(ctx,size))throw std::runtime_error("bloom extent allocation");
        auto& frame=frames[slot%2];taa.copyAtmosphereInput(cmd,size);
        for(int i=0;i<5;++i)for(auto* texture:{&frame.a[i],&frame.b[i]})transitionImage(cmd,texture->image,
            frame.initialized?VK_IMAGE_LAYOUT_GENERAL:VK_IMAGE_LAYOUT_UNDEFINED,VK_IMAGE_LAYOUT_GENERAL,
            VK_PIPELINE_STAGE_2_ALL_COMMANDS_BIT,frame.initialized?VK_ACCESS_2_MEMORY_READ_BIT|VK_ACCESS_2_MEMORY_WRITE_BIT:0,
            VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT,VK_ACCESS_2_SHADER_WRITE_BIT);
        frame.initialized=true;visible(cmd,VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT);
        Params pc{};pc.sun[0]=snap.sunScreen[0];pc.sun[1]=snap.sunScreen[1];pc.aspect=float(size.width)/size.height;
        pc.focalY=std::abs(snap.proj[5]);auto sun=SolarOptics::source(snap.sunDistanceKm,snap.sunRadiusKm);
        pc.removeSolar=sun.valid&&snap.sunInFront&&std::isfinite(pc.focalY)&&pc.focalY>0&&std::isfinite(pc.sun[0])&&std::isfinite(pc.sun[1]);pc.sinRadius=float(sun.sinRadius);
        if(!std::isfinite(pc.focalY)||pc.focalY<=0)pc.focalY=1;
        auto dispatch=[&](VkDescriptorSet set,VkImageView input,VkSampler sampler,VkImageLayout inputLayout,VkTexture2D& output,VkExtent2D level){
            VkDescriptorImageInfo images[]={{sampler,input,inputLayout},{VK_NULL_HANDLE,output.view,VK_IMAGE_LAYOUT_GENERAL},
                {taa.depthSampler,taa.depthView,VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL}};
            VkWriteDescriptorSet writes[3]={};for(int j=0;j<3;++j){writes[j].sType=VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;writes[j].dstSet=set;
                writes[j].dstBinding=j;writes[j].descriptorCount=1;writes[j].descriptorType=j==1?VK_DESCRIPTOR_TYPE_STORAGE_IMAGE:VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;writes[j].pImageInfo=&images[j];}
            vkUpdateDescriptorSets(ctx.device,3,writes,0,nullptr);
            vkCmdBindPipeline(cmd,VK_PIPELINE_BIND_POINT_COMPUTE,computePipeline);
            vkCmdBindDescriptorSets(cmd,VK_PIPELINE_BIND_POINT_COMPUTE,computeLayout,0,1,&set,0,nullptr);
            vkCmdPushConstants(cmd,computeLayout,VK_SHADER_STAGE_COMPUTE_BIT,0,sizeof(pc),&pc);
            vkCmdDispatch(cmd,(level.width+7)/8,(level.height+7)/8,1);visible(cmd,VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT);
        };
        const float angles[]={.0015f,.006f,.020f,.060f,.15f};float previousVariance=0;
        for(int i=0;i<5;++i){pc.operation=i==0?0:1;
            dispatch(frame.down[i],i?frame.a[i-1].view:taa.atmosphereInputView,i?frame.a[i-1].sampler:taa.colorSampler,
                i?VK_IMAGE_LAYOUT_GENERAL:VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,frame.a[i],frame.sizes[i]);
            float scale=float(1<<(i+1));float target=pc.focalY*angles[i]*size.height/2;
            pc.sigma=std::sqrt(std::max(0.f,target*target-previousVariance-scale*scale/16))/scale;
            previousVariance=std::max(previousVariance,target*target);
            pc.operation=2;pc.axis[0]=1;pc.axis[1]=0;
            dispatch(frame.horizontal[i],frame.a[i].view,frame.a[i].sampler,VK_IMAGE_LAYOUT_GENERAL,frame.b[i],frame.sizes[i]);
            pc.axis[0]=0;pc.axis[1]=1;
            dispatch(frame.vertical[i],frame.b[i].view,frame.b[i].sampler,VK_IMAGE_LAYOUT_GENERAL,frame.a[i],frame.sizes[i]);
        }
        visible(cmd,VK_PIPELINE_STAGE_2_FRAGMENT_SHADER_BIT);
        VkDescriptorImageInfo original{taa.colorSampler,taa.atmosphereInputView,VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL};
        VkDescriptorImageInfo depth{taa.depthSampler,taa.depthView,VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL},pyramid[5];
        for(int i=0;i<5;++i)pyramid[i]={frame.a[i].sampler,frame.a[i].view,VK_IMAGE_LAYOUT_GENERAL};
        VkWriteDescriptorSet writes[3]={};for(int i=0;i<3;++i){writes[i].sType=VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;writes[i].dstSet=frame.compose;
            writes[i].dstBinding=i;writes[i].descriptorType=VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;writes[i].descriptorCount=i==1?5:1;writes[i].pImageInfo=i==0?&original:i==1?pyramid:&depth;}
        vkUpdateDescriptorSets(ctx.device,3,writes,0,nullptr);
        VkRenderingAttachmentInfo color{VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO};color.imageView=taa.colorViews[taa.currentIdx];
        color.imageLayout=VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;color.loadOp=VK_ATTACHMENT_LOAD_OP_LOAD;color.storeOp=VK_ATTACHMENT_STORE_OP_STORE;
        VkRenderingInfo rendering{VK_STRUCTURE_TYPE_RENDERING_INFO};rendering.renderArea={{0,0},size};rendering.layerCount=1;rendering.colorAttachmentCount=1;rendering.pColorAttachments=&color;
        vkCmdBeginRendering(cmd,&rendering);VkViewport viewport{0,0,float(size.width),float(size.height),0,1};VkRect2D scissor{{0,0},size};
        vkCmdSetViewport(cmd,0,1,&viewport);vkCmdSetScissor(cmd,0,1,&scissor);composePipeline.bind(cmd);
        vkCmdBindDescriptorSets(cmd,VK_PIPELINE_BIND_POINT_GRAPHICS,composePipeline.layout,0,1,&frame.compose,0,nullptr);
        vkCmdPushConstants(cmd,composePipeline.layout,VK_SHADER_STAGE_VERTEX_BIT|VK_SHADER_STAGE_FRAGMENT_BIT,0,sizeof(pc),&pc);
        VkDeviceSize offset=0;vkCmdBindVertexBuffers(cmd,0,1,&quad,&offset);vkCmdDraw(cmd,4,1,0,0);vkCmdEndRendering(cmd);
        transitionImage(cmd,taa.colorImages[taa.currentIdx],VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL,VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
            VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT,VK_ACCESS_2_COLOR_ATTACHMENT_WRITE_BIT,VK_PIPELINE_STAGE_2_ALL_COMMANDS_BIT,VK_ACCESS_2_SHADER_READ_BIT);
    }
    void shutdown(VulkanContext& ctx){releaseImages(ctx);composePipeline.shutdown(ctx.device);
        if(computePipeline)vkDestroyPipeline(ctx.device,computePipeline,nullptr);if(computeLayout)vkDestroyPipelineLayout(ctx.device,computeLayout,nullptr);
        if(computeSet)vkDestroyDescriptorSetLayout(ctx.device,computeSet,nullptr);if(composeSet)vkDestroyDescriptorSetLayout(ctx.device,composeSet,nullptr);
        computePipeline=VK_NULL_HANDLE;computeLayout=VK_NULL_HANDLE;computeSet=composeSet=VK_NULL_HANDLE;
    }
};
