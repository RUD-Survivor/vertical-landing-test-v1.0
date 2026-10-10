#pragma once
#include <vector>

// Multi-frame orbit regression using the production geometry, ocean, atmosphere
// and TAA passes. Constant synthetic sea isolates depth/BRDF instability from
// legitimate shoreline motion and weather evolution.
static void validateOrbitOcean(VulkanContext& ctx,VkRenderer3D& renderer,VkTAA& taa,GLFWwindow* window,bool enforce) {
    auto& scene=renderer.scene;auto& ocean=renderer.oceanSystem;
    VkExtent2D size{1280,720};size_t pixels=size_t(size.width)*size.height;
    glfwSetWindowSize(window,int(size.width),int(size.height));glfwPollEvents();
    require(ctx.recreateSwapchain(window),"orbit swapchain resize");
    require(taa.recreate(ctx,size,ctx.swapFormat,"src/render/shaders/spirv/taa.vert.spv","src/render/shaders/spirv/taa.frag.spv"),"orbit HDR resize");
    require(renderer.cloudSystem.init(ctx,size,scene.cloudNoiseTex.view,scene.cloudDetailTex.view,
        scene.cloudWeatherTex.view,scene.cloudCurlTex.view,scene.cloudProfileLutTex.view,scene.cloudHiWeatherTex.view),"orbit cloud init");
    renderer.cloudSystem.tuneParams.density=0;renderer.cloudSystem.tuneParams.hiCloudEnabled=false;
    float vertices[4][12]={{-.25f,0,-.25f,0,1,0,0,0,1,1,1,1},{.25f,0,-.25f,0,1,0,1,0,1,1,1,1},
        {.25f,0,.25f,0,1,0,1,1,1,1,1,1},{-.25f,0,.25f,0,1,0,0,1,1,1,1,1}};
    uint32_t indices[]={0,1,2,0,2,3};
    require(scene.registerMesh(ctx,"orbit-occluder",vertices,sizeof(vertices),indices,6),"orbit foreground mesh");
    struct Rig {const char* name;double altitude,nearPlane;float bed;bool moving;bool occluder=false,glint=false;};
    const Rig rigs[]={{"orbit-400km-deep",400,.001,.3f,false},
        // .449 triggers procedural coastal cliffs that create genuine dry
        // islands. .44 keeps every generated shelf vertex below sea level.
        {"orbit-400km-moving",400,.001,.3f,true},{"orbit-400km-shallow",400,.001,.44f,true},
        {"orbit-400km-near1",400,1,.3f,true},{"orbit-2000km",2000,.001,.3f,true},
        {"orbit-10000km",10000,1,.3f,true},{"orbit-foreground",400,.001,.3f,true,true},
        {"orbit-sunglint",400,.001,.3f,true,false,true}};
    for(const auto& rig:rigs) {
        scene.terrainTectonicTex.destroy(ctx);scene.terrainHydroTex.destroy(ctx);
        float tectonic[8*4*4],hydro[8*4*4];
        for(int i=0;i<32;++i){tectonic[4*i]=rig.bed;tectonic[4*i+1]=tectonic[4*i+2]=0;tectonic[4*i+3]=1;
            hydro[4*i]=rig.bed;hydro[4*i+1]=hydro[4*i+2]=0;hydro[4*i+3]=-1;}
        require(scene.uploadTerrainTextures(ctx,tectonic,8,4,hydro,8,4),"orbit maps");
        std::vector<uint8_t> previousMask;std::vector<double> previousY,previousDisplayY,previousCompositeY;
        for(int frame=0;frame<8;++frame) {
            int slot=frame%2;double x=rig.moving?frame*.1:0;
            SceneSnapshot snap{};snap.camPos[0]=float(x);snap.camPos[1]=float(rig.altitude);snap.aspect=float(size.width)/size.height;
            snap.time=3+frame/60.;snap.frameIndex=frame;snap.sunWorldPos[0]=100000;snap.sunWorldPos[1]=100000;
            snap.lightDir[0]=snap.lightDir[1]=.7071f;
            if(rig.glint){snap.sunWorldPos[0]=0;snap.lightDir[0]=0;snap.lightDir[1]=1;}
            Mat4::lookAt(Vec3(x,rig.altitude,0),Vec3(x,0,0),Vec3(0,0,1)).toFloatArray(snap.view);
            Mat4::perspective(1.0,snap.aspect,rig.nearPlane,1e10).toFloatArray(snap.proj);
            CelestialDraw earth{};earth.bodyIdx=earth.atmoIdx=3;earth.radius=6371;earth.center[1]=-6371;
            // Production terrain includes a fixed KSC land pad even on constant
            // sea maps. Rotate that pad onto the hidden hemisphere so this
            // fixture's visible disk is actually all water.
            (Mat4::translate(Vec3(0,-6371,0))*Mat4::scale(Vec3(6371,-6371,-6371))).toFloatArray(earth.model);
            snap.celestials.push_back(earth);
            TerrainPatchDraw tp{};memcpy(tp.model,earth.model,64);tp.planetRadius=6371;tp.maxElev=25;tp.nodeCenter[1]=1;
            tp.nodeSideA[0]=2;tp.nodeSideB[2]=2;tp.nodeLevel=0;
            tp.planetCenterRel[0]=-snap.camPos[0];tp.planetCenterRel[1]=-6371-snap.camPos[1];snap.terrainPatches.push_back(tp);
            const float centers[5][3]={{0,-1,0},{1,0,0},{-1,0,0},{0,0,1},{0,0,-1}};
            for(const auto& center:centers){TerrainPatchDraw face=tp;
                memcpy(face.nodeCenter,center,12);memset(face.nodeSideA,0,12);memset(face.nodeSideB,0,12);
                if(center[1]!=0){face.nodeSideA[0]=2;face.nodeSideB[2]=2;}
                else if(center[0]!=0){face.nodeSideA[1]=2;face.nodeSideB[2]=2;}
                else {face.nodeSideA[0]=2;face.nodeSideB[1]=2;}
                snap.terrainPatches.push_back(face);}
            taa.updateMatrices(snap.view,snap.proj,snap.camPos,earth.center,earth.radius);taa.uploadMatrices(ctx,slot);
            VkFence acquireFence;VkFenceCreateInfo fci{VK_STRUCTURE_TYPE_FENCE_CREATE_INFO};
            vkRequireSuccess(vkCreateFence(ctx.device,&fci,nullptr,&acquireFence),"orbit acquire fence");uint32_t swapIndex;
            vkRequireSuccess(vkAcquireNextImageKHR(ctx.device,ctx.swapchain,5000000000ULL,VK_NULL_HANDLE,acquireFence,&swapIndex),"orbit acquire");
            vkRequireSuccess(vkWaitForFences(ctx.device,1,&acquireFence,VK_TRUE,5000000000ULL),"orbit acquire wait");
            VkBuffer readback;VmaAllocation allocation;VmaAllocationInfo info{};
            VkBufferCreateInfo bi{VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO};bi.size=pixels*13;bi.usage=VK_BUFFER_USAGE_TRANSFER_DST_BIT;
            VmaAllocationCreateInfo ac{};ac.usage=VMA_MEMORY_USAGE_GPU_TO_CPU;ac.flags=VMA_ALLOCATION_CREATE_MAPPED_BIT;
            vkRequireSuccess(vmaCreateBuffer(ctx.allocator,&bi,&ac,&readback,&allocation,&info),"orbit water readback");
            VkCommandBuffer cmd=beginSingleTimeCommands(ctx);taa.beginGeometryPass(cmd,size);
            VkFrameRenderer::setViewportScissor(cmd,size);
            renderer.render(cmd,slot,snap,size);
            if(rig.occluder){float model[16];Mat4::translate(Vec3(x,rig.altitude-1,0)).toFloatArray(model);
                scene.drawMesh(cmd,"orbit-occluder",model,1,0,0,1,1);}
            taa.endGeometryPass(cmd);
            ocean.render(ctx,cmd,taa,scene,snap,slot,size,renderer.atmoTune);
            VkBufferImageCopy copy{};copy.imageSubresource={VK_IMAGE_ASPECT_COLOR_BIT,0,0,1};copy.imageExtent={size.width,size.height,1};
            transitionImage(cmd,taa.colorImages[taa.currentIdx],VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL);
            vkCmdCopyImageToBuffer(cmd,taa.colorImages[taa.currentIdx],VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,readback,1,&copy);
            transitionImage(cmd,taa.colorImages[taa.currentIdx],VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
            copy.bufferOffset=pixels*8;
            transitionImage(cmd,ocean.reactive,VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL);
            vkCmdCopyImageToBuffer(cmd,ocean.reactive,VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,readback,1,&copy);
            transitionImage(cmd,ocean.reactive,VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
            copy.bufferOffset=pixels*9;copy.imageSubresource.aspectMask=VK_IMAGE_ASPECT_DEPTH_BIT;
            VkOceanSystem::depthBarrier(cmd,ocean.bedDepth,VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL);
            vkCmdCopyImageToBuffer(cmd,ocean.bedDepth,VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,readback,1,&copy);
            VkOceanSystem::depthBarrier(cmd,ocean.bedDepth,VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
            renderer.renderAtmoAfterGeometry(ctx,cmd,taa,snap,slot);
            renderer.renderFlowerCloudAfterGeometry(ctx,cmd,taa,snap,slot);
            HdrProbe reference;reference.capture(ctx,cmd,taa.colorImages[taa.currentIdx],size);
            taa.beginResolvePass(cmd,ctx.swapImages[swapIndex],ctx.swapImageViews[swapIndex],size);
            VkViewport vp{0,0,float(size.width),float(size.height),0,1};VkRect2D rect{{0,0},size};
            vkCmdSetViewport(cmd,0,1,&vp);vkCmdSetScissor(cmd,0,1,&rect);taa.drawResolve(cmd,frame==0?1.f:.1f);
            taa.endResolvePass(cmd,ctx.swapImages[swapIndex]);DisplayProbe display;display.capture(ctx,cmd,ctx.swapImages[swapIndex],size);
            endSingleTimeCommands(ctx,cmd);
            vkRequireSuccess(vmaInvalidateAllocation(ctx.allocator,reference.allocation,0,VK_WHOLE_SIZE),"orbit composite invalidate");
            auto composite=static_cast<const uint16_t*>(reference.mapped);std::vector<double> compositeY(pixels);
            for(size_t i=0;i<pixels;++i){const double weights[3]={.2126,.7152,.0722};
                for(int c=0;c<3;++c)compositeY[i]+=weights[c]*Radiometry::aces(halfFloat(composite[4*i+c]),taa.displayExposure);}
            vkRequireSuccess(vmaInvalidateAllocation(ctx.allocator,display.allocation,0,VK_WHOLE_SIZE),"orbit display invalidate");
            auto displayed=static_cast<const uint8_t*>(display.mapped);std::vector<double> displayY(pixels);
            bool bgra=ctx.swapFormat==VK_FORMAT_B8G8R8A8_UNORM || ctx.swapFormat==VK_FORMAT_B8G8R8A8_SRGB;
            for(size_t i=0;i<pixels;++i){const double weights[3]={.2126,.7152,.0722};
                for(int c=0;c<3;++c){double value=displayed[4*i+(bgra?2-c:c)]/255.;
                    displayY[i]+=weights[c]*(value<=.04045?value/12.92:std::pow((value+.055)/1.055,2.4));}}
            display.report(ctx,size,rig.name,frame==0?&reference:nullptr,false);
            if(frame==0)reference.report(ctx,size,rig.name,"first-hdr");
            else vmaDestroyBuffer(ctx.allocator,reference.buffer,reference.allocation);
            vkRequireSuccess(vmaInvalidateAllocation(ctx.allocator,allocation,0,VK_WHOLE_SIZE),"orbit invalidate");
            auto color=static_cast<const uint16_t*>(info.pMappedData);
            auto mask=static_cast<const uint8_t*>(info.pMappedData)+pixels*8;auto depth=reinterpret_cast<const float*>(mask+pixels);
            size_t wet=0,bedPixels=0,holes=0,toggles=0,common=0,foreground=0,leaks=0;double sq=0,maxDelta=0,resolveSq=0,resolveMax=0,compositeMax=0;
            std::vector<double> currentY(pixels);
            for(size_t i=0;i<pixels;++i){
                bool isForeground=rig.occluder && depth[i]<1-rig.nearPlane/(rig.altitude*.5);
                if(isForeground){++foreground;if(mask[i])++leaks;}
                if(mask[i])++wet;if(depth[i]<1 && !isForeground){++bedPixels;
                    // Rasterization differences at the moving planetary limb
                    // are coverage, not interior depth flicker. Exclude its
                    // one-pixel border from the missing-water assertion.
                    size_t row=i/size.width,column=i%size.width;
                    bool interior=row>0 && row+1<size.height && column>0 && column+1<size.width;
                    if(interior)for(int dy=-1;dy<=1;++dy)for(int dx=-1;dx<=1;++dx)
                        if(depth[size_t(int(row)+dy)*size.width+int(column)+dx]>=1)interior=false;
                    if(interior && !mask[i])++holes;}
                double linearY=0;const double weights[3]={.2126,.7152,.0722};
                for(int c=0;c<3;++c){double value=halfFloat(color[i*4+c]);require(std::isfinite(value)&&value>=0,"orbit finite HDR");
                    linearY+=weights[c]*Radiometry::aces(value,taa.displayExposure);}
                currentY[i]=linearY;
                if(!previousMask.empty()){
                    if(depth[i]<1 && bool(mask[i])!=bool(previousMask[i]))++toggles;
                    if(mask[i]&&previousMask[i]){double d=std::abs(linearY-previousY[i]);sq+=d*d;maxDelta=std::max(maxDelta,d);++common;
                        double displayedDelta=std::abs(displayY[i]-previousDisplayY[i]);resolveSq+=displayedDelta*displayedDelta;resolveMax=std::max(resolveMax,displayedDelta);}}
                if(!previousCompositeY.empty() && mask[i] && previousMask[i])compositeMax=std::max(compositeMax,std::abs(compositeY[i]-previousCompositeY[i]));
            }
            double holeFraction=double(holes)/std::max(size_t(1),bedPixels),rms=std::sqrt(sq/std::max(size_t(1),common));
            printf("ORBIT %s frame=%d wet=%zu bed=%zu holes=%.6f%% toggles=%zu displayLinearRMS=%.8f maxDelta=%.6f\n",
                rig.name,frame,wet,bedPixels,holeFraction*100,toggles,rms,maxDelta);
            double resolveRms=std::sqrt(resolveSq/std::max(size_t(1),common));
            printf("ORBIT_RESOLVE %s frame=%d RMS=%.8f maxDelta=%.6f compositeMax=%.6f foreground=%zu leaks=%zu\n",rig.name,frame,resolveRms,resolveMax,compositeMax,foreground,leaks);
            if(enforce){require(wet>pixels/20,"orbit ocean coverage missing");require(holeFraction<1e-5,"orbit water loses depth ties to bed");
                if(rig.occluder)require(foreground>pixels/100 && leaks==0,"water overwrites orbital foreground");
                if(frame>0 && !rig.moving)require(rms<1e-5&&toggles==0,"static orbit ocean flickers");
                if(frame>0 && rig.moving)require(rms<.002,"moving orbit radiance has temporal sparkle");
                if(frame>0)require(resolveRms<.005 && resolveMax<.03,"orbital presentation flickers after TAA");}
            previousMask.assign(mask,mask+pixels);previousY.swap(currentY);previousDisplayY.swap(displayY);previousCompositeY.swap(compositeY);vmaDestroyBuffer(ctx.allocator,readback,allocation);
            vkResetFences(ctx.device,1,&acquireFence);
            VkSwapchainPresentFenceInfoEXT fi{VK_STRUCTURE_TYPE_SWAPCHAIN_PRESENT_FENCE_INFO_EXT};fi.swapchainCount=1;fi.pFences=&acquireFence;
            VkPresentInfoKHR present{VK_STRUCTURE_TYPE_PRESENT_INFO_KHR};present.pNext=&fi;present.swapchainCount=1;present.pSwapchains=&ctx.swapchain;present.pImageIndices=&swapIndex;
            vkRequireSuccess(vkQueuePresentKHR(ctx.presentQueue,&present),"orbit present");
            vkRequireSuccess(vkWaitForFences(ctx.device,1,&acquireFence,VK_TRUE,5000000000ULL),"orbit present complete");
            vkDestroyFence(ctx.device,acquireFence,nullptr);taa.swap();require(g_vkValidationErrorCount==0,"orbit Vulkan errors");
        }
    }
}
