#pragma once
#include "../vk_texture.h"
#include <fstream>
#include <vector>
#include <algorithm>
#include <cstring>
#include <cstdint>

// Offline linear half-float map; no EXR decoder or runtime mip generation.
struct VkMilkyWay : VkTexture2D {
    bool load(VulkanContext& ctx) {
        std::ifstream file("assets/data/milky-way.bin",std::ios::binary);
        uint32_t header[4]={};file.read(reinterpret_cast<char*>(header),sizeof(header));
        if(!file || header[0]!=0x594B574D || header[1]!=2048 || header[2]!=1024 || header[3]!=12)return false;
        width=header[1];height=header[2];std::vector<VkBufferImageCopy> regions;
        size_t bytes=0;uint32_t w=width,h=height;
        for(uint32_t level=0;level<header[3];++level){
            VkBufferImageCopy r{};r.bufferOffset=bytes;r.imageSubresource={VK_IMAGE_ASPECT_COLOR_BIT,level,0,1};
            r.imageExtent={w,h,1};regions.push_back(r);bytes+=size_t(w)*h*8;
            w=std::max(1u,w/2);h=std::max(1u,h/2);
        }
        std::vector<uint16_t> data(bytes/2);file.read(reinterpret_cast<char*>(data.data()),bytes);
        if(!file || file.peek()!=std::char_traits<char>::eof())return false;
        // Reject NaN/Inf, negative channels and malformed alpha before upload.
        for(size_t i=0;i<data.size();++i)
            if((data[i]&0x8000) || (data[i]&0x7c00)==0x7c00 || (i%4==3 && data[i]!=0x3c00))return false;
        VkFormatProperties properties{};vkGetPhysicalDeviceFormatProperties(ctx.physDevice,VK_FORMAT_R16G16B16A16_SFLOAT,&properties);
        if(!(properties.optimalTilingFeatures&VK_FORMAT_FEATURE_SAMPLED_IMAGE_FILTER_LINEAR_BIT))return false;
        VkBuffer staging;VmaAllocation stagingAllocation;VmaAllocationInfo info{};
        VkBufferCreateInfo bi{VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO};bi.size=bytes;bi.usage=VK_BUFFER_USAGE_TRANSFER_SRC_BIT;
        VmaAllocationCreateInfo ac{};ac.usage=VMA_MEMORY_USAGE_CPU_ONLY;ac.flags=VMA_ALLOCATION_CREATE_MAPPED_BIT;
        vkRequireSuccess(vmaCreateBuffer(ctx.allocator,&bi,&ac,&staging,&stagingAllocation,&info),"Milky Way staging");
        memcpy(info.pMappedData,data.data(),bytes);vkRequireSuccess(vmaFlushAllocation(ctx.allocator,stagingAllocation,0,bytes),"Milky Way flush");
        VkImageCreateInfo ii{VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO};ii.imageType=VK_IMAGE_TYPE_2D;ii.format=VK_FORMAT_R16G16B16A16_SFLOAT;
        ii.extent={width,height,1};ii.mipLevels=header[3];ii.arrayLayers=1;ii.samples=VK_SAMPLE_COUNT_1_BIT;
        ii.tiling=VK_IMAGE_TILING_OPTIMAL;ii.usage=VK_IMAGE_USAGE_TRANSFER_DST_BIT|VK_IMAGE_USAGE_SAMPLED_BIT;
        ac={};ac.usage=VMA_MEMORY_USAGE_GPU_ONLY;
        VkResult result=vmaCreateImage(ctx.allocator,&ii,&ac,&image,&alloc,nullptr);
        if(result!=VK_SUCCESS){vmaDestroyBuffer(ctx.allocator,staging,stagingAllocation);return false;}
        VkCommandBuffer cmd=beginSingleTimeCommands(ctx);
        transitionImage(cmd,image,VK_IMAGE_LAYOUT_UNDEFINED,VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
            VK_PIPELINE_STAGE_2_TOP_OF_PIPE_BIT,VK_ACCESS_2_NONE,VK_PIPELINE_STAGE_2_TRANSFER_BIT,VK_ACCESS_2_TRANSFER_WRITE_BIT);
        vkCmdCopyBufferToImage(cmd,staging,image,VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,uint32_t(regions.size()),regions.data());
        transitionToShaderRead(cmd,image,header[3]);endSingleTimeCommands(ctx,cmd);
        vmaDestroyBuffer(ctx.allocator,staging,stagingAllocation);
        VkImageViewCreateInfo vi{VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO};vi.image=image;vi.viewType=VK_IMAGE_VIEW_TYPE_2D;vi.format=ii.format;
        vi.subresourceRange={VK_IMAGE_ASPECT_COLOR_BIT,0,header[3],0,1};
        if(vkCreateImageView(ctx.device,&vi,nullptr,&view)!=VK_SUCCESS){destroy(ctx);return false;}
        VkSamplerCreateInfo si{VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO};si.magFilter=si.minFilter=VK_FILTER_LINEAR;si.mipmapMode=VK_SAMPLER_MIPMAP_MODE_LINEAR;
        // Equirectangular footprints stretch strongly near celestial poles.
        VkPhysicalDeviceProperties deviceProperties{};vkGetPhysicalDeviceProperties(ctx.physDevice,&deviceProperties);
        si.anisotropyEnable=VK_TRUE;si.maxAnisotropy=std::min(16.f,deviceProperties.limits.maxSamplerAnisotropy);
        si.addressModeU=VK_SAMPLER_ADDRESS_MODE_REPEAT;si.addressModeV=si.addressModeW=VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;si.maxLod=float(header[3]-1);
        if(vkCreateSampler(ctx.device,&si,nullptr,&sampler)!=VK_SUCCESS){destroy(ctx);return false;}
        printf("[VkSky] NASA Milky Way %ux%u, %u spherical mips\n",width,height,header[3]);return true;
    }
};
