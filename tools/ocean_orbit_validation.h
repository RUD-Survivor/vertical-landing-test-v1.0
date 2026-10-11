#pragma once
#include <vector>

// Multi-frame orbit regression using the production geometry, ocean, atmosphere
// and TAA passes. Constant synthetic sea isolates depth/BRDF instability from
// legitimate shoreline motion and weather evolution.
static void validateOrbitOcean(VulkanContext& ctx,VkRenderer3D& renderer,VkTAA& taa,GLFWwindow* window,bool enforce,bool cloudValidation=false,bool oddSize=false,bool cameraChain=false,bool cloudResponse=false,bool cloudDynamic=false) {
    auto& scene=renderer.scene;auto& ocean=renderer.oceanSystem;
    VkExtent2D size=cloudValidation?VkExtent2D{640,360}:VkExtent2D{1280,720};size_t pixels=size_t(size.width)*size.height;
    if(oddSize){size={641,363};pixels=size_t(size.width)*size.height;}
    size_t depthOffset=(pixels*9+3)&~size_t(3);
    VkQueryPool cloudQueries=VK_NULL_HANDLE;
    VkPhysicalDeviceProperties cloudProps{};vkGetPhysicalDeviceProperties(ctx.physDevice,&cloudProps);
    if(cloudValidation){VkQueryPoolCreateInfo qi{VK_STRUCTURE_TYPE_QUERY_POOL_CREATE_INFO};qi.queryType=VK_QUERY_TYPE_TIMESTAMP;qi.queryCount=2;
        vkRequireSuccess(vkCreateQueryPool(ctx.device,&qi,nullptr,&cloudQueries),"cloud timing queries");}
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
    struct Rig {const char* name;double altitude,nearPlane;float bed;bool moving;bool occluder=false,glint=false,cloud=false,full=false;double fov=1.;bool wind=false,timeJump=false;};
    const std::vector<Rig> rigs=cloudDynamic?std::vector<Rig>{
        {"cloud-dynamic-camera-quarter",400,.001,.3f,true,false,false,true,false,2.6},
        {"cloud-dynamic-camera-full",400,.001,.3f,true,false,false,true,true,2.6},
        {"cloud-dynamic-wind-quarter",400,.001,.3f,false,false,false,true,false,2.6,true},
        {"cloud-dynamic-wind-full",400,.001,.3f,false,false,false,true,true,2.6,true},
        {"cloud-dynamic-time-jump",10000,1,.3f,false,false,false,true,false,1.,false,true}
    }:cloudResponse?std::vector<Rig>{
        {"cloud-response-quarter",10000,1,.3f,false,false,false,true},
        {"cloud-response-full",10000,1,.3f,false,false,false,true,true}
    }:cloudValidation?std::vector<Rig>{
        {"cloud-off",10000,1,.3f,false},
        {"cloud-empty",10000,1,.3f,false,false,false,true},
        {"cloud-quarter-static",10000,1,.3f,false,false,false,true},
        {"cloud-full-static",10000,1,.3f,false,false,false,true,true},
        {"cloud-quarter-moving",10000,1,.3f,true,false,false,true},
        {"cloud-quarter-depth",10000,.001,.3f,false,false,false,true},
        {"cloud-quarter-cut",10000,1,.3f,false,false,false,true},
        {"cloud-quarter-foreground",10000,.001,.3f,false,true,false,true},
        {"cloud-quarter-2000km",2000,.001,.3f,true,false,false,true,false,2.0},
        {"cloud-quarter-400km",400,.001,.3f,true,false,false,true,false,2.6}
    }:std::vector<Rig>{{"orbit-400km-deep",400,.001,.3f,false},
        // .449 triggers procedural coastal cliffs that create genuine dry
        // islands. .44 keeps every generated shelf vertex below sea level.
        {"orbit-400km-moving",400,.001,.3f,true},{"orbit-400km-shallow",400,.001,.44f,true},
        {"orbit-400km-near1",400,1,.3f,true},{"orbit-2000km",2000,.001,.3f,true},
        {"orbit-10000km",10000,1,.3f,true},{"orbit-foreground",400,.001,.3f,true,true},
        {"orbit-sunglint",400,.001,.3f,true,false,true}};
    std::vector<float> dynamicReference[2][4];
    for(const auto& rig:rigs) {
        if(cloudValidation){require(renderer.cloudSystem.recreate(ctx,size),"cloud rig history reset");
            renderer.cloudSystem.tuneParams=FlowerCloudTuneParams{};
            renderer.cloudSystem.tuneParams.speed=rig.wind?.15f:0.f;
            if(cloudResponse)renderer.cloudSystem.tuneParams.hiCloudEnabled=false;
            if(std::string(rig.name)=="cloud-empty"){
                renderer.cloudSystem.tuneParams.density=0;
                renderer.cloudSystem.tuneParams.hiCloudEnabled=false;}
            renderer.cloudSystem.tuneParams.useQuarterRes=!rig.full;}
        scene.terrainTectonicTex.destroy(ctx);scene.terrainHydroTex.destroy(ctx);
        float tectonic[8*4*4],hydro[8*4*4];
        for(int i=0;i<32;++i){tectonic[4*i]=rig.bed;tectonic[4*i+1]=tectonic[4*i+2]=0;tectonic[4*i+3]=1;
            hydro[4*i]=rig.bed;hydro[4*i+1]=hydro[4*i+2]=0;hydro[4*i+3]=-1;}
        require(scene.uploadTerrainTextures(ctx,tectonic,8,4,hydro,8,4),"orbit maps");
        std::vector<uint8_t> previousMask;std::vector<double> previousY,previousDisplayY,previousCompositeY,previousCloudT;
        double responseBaseline=0;
        for(int frame=0;frame<(cloudResponse?96:cloudDynamic?64:cloudValidation?32:8);++frame) {
            // Keep history intact: empty physical medium must stop contributing.
            // Resetting the history here would conceal the persistence defect.
            if(cloudResponse && frame==48)renderer.cloudSystem.tuneParams.density=0;
            int slot=frame%2;double x=rig.moving?(cloudDynamic?std::min(frame,31)*2.:frame*.1):0;
            bool cameraCut=cloudValidation && std::string(rig.name)=="cloud-quarter-cut" && frame==16;
            if(cameraCut)renderer.cloudSystem.hasPrevViewProj=false;
            SceneSnapshot snap{};snap.camPos[0]=float(x);snap.camPos[1]=float(rig.altitude);snap.aspect=float(size.width)/size.height;
            snap.time=cloudDynamic?3+frame/60.:cloudValidation?3:3+frame/60.;
            if(rig.timeJump && frame==16)snap.time=2;
            if(rig.timeJump && frame==32)snap.time=100;
            snap.frameIndex=frame;snap.sunWorldPos[0]=100000;snap.sunWorldPos[1]=100000;
            snap.lightDir[0]=snap.lightDir[1]=.7071f;
            if(rig.glint){snap.sunWorldPos[0]=0;snap.lightDir[0]=0;snap.lightDir[1]=1;}
            Mat4::lookAt(Vec3(x,rig.altitude,0),Vec3(x,0,0),Vec3(0,0,1)).toFloatArray(snap.view);
            Mat4::perspective(rig.fov,snap.aspect,rig.nearPlane,1e10).toFloatArray(snap.proj);
            CelestialDraw earth{};earth.bodyIdx=earth.atmoIdx=3;earth.radius=6371;earth.center[1]=-6371;
            earth.showClouds=rig.cloud;
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
            VkBufferCreateInfo bi{VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO};bi.size=depthOffset+pixels*4;bi.usage=VK_BUFFER_USAGE_TRANSFER_DST_BIT;
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
            copy.bufferOffset=depthOffset;copy.imageSubresource.aspectMask=VK_IMAGE_ASPECT_DEPTH_BIT;
            VkOceanSystem::depthBarrier(cmd,ocean.bedDepth,VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL);
            vkCmdCopyImageToBuffer(cmd,ocean.bedDepth,VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,readback,1,&copy);
            VkOceanSystem::depthBarrier(cmd,ocean.bedDepth,VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
            renderer.renderAtmoAfterGeometry(ctx,cmd,taa,snap,slot);
            HdrProbe preCloud;
            bool emptyCloud=std::string(rig.name)=="cloud-empty";
            bool responseClear=cloudResponse && frame>=48;
            if(cloudValidation && (rig.occluder || emptyCloud || responseClear))preCloud.capture(ctx,cmd,taa.colorImages[taa.currentIdx],size);
            if(cloudValidation){vkCmdResetQueryPool(cmd,cloudQueries,0,2);vkCmdWriteTimestamp(cmd,VK_PIPELINE_STAGE_ALL_COMMANDS_BIT,cloudQueries,0);}
            renderer.renderFlowerCloudAfterGeometry(ctx,cmd,taa,snap,slot);
            if(enforce && rig.timeJump && (frame==16 || frame==17 || frame==32 || frame==33))
                require(renderer.cloudSystem.cameraCutThisFrame,"time discontinuity reused cloud history");
            if(cloudValidation)vkCmdWriteTimestamp(cmd,VK_PIPELINE_STAGE_ALL_COMMANDS_BIT,cloudQueries,1);
            HdrProbe cloudProbe;
            if(cloudValidation && rig.cloud){
                VkImage image=renderer.cloudSystem.fullColorImg[slot];
                transitionImage(cmd,image,VK_IMAGE_LAYOUT_GENERAL,VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
                cloudProbe.capture(ctx,cmd,image,size);
                transitionImage(cmd,image,VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,VK_IMAGE_LAYOUT_GENERAL);
            }
            if(cameraChain){renderer.cameraBloom.render(ctx,cmd,taa,snap,slot,size,scene.billboardQuadVbo);
                renderer.renderSolarOpticsAfterGeometry(ctx,cmd,taa,snap,slot,size);}
            HdrProbe reference;reference.capture(ctx,cmd,taa.colorImages[taa.currentIdx],size);
            taa.beginResolvePass(cmd,ctx.swapImages[swapIndex],ctx.swapImageViews[swapIndex],size);
            VkViewport vp{0,0,float(size.width),float(size.height),0,1};VkRect2D rect{{0,0},size};
            vkCmdSetViewport(cmd,0,1,&vp);vkCmdSetScissor(cmd,0,1,&rect);
            taa.drawResolve(cmd,frame==0 || (rig.cloud && renderer.cloudSystem.cameraCutThisFrame)?1.f:.1f);
            taa.endResolvePass(cmd,ctx.swapImages[swapIndex]);DisplayProbe display;display.capture(ctx,cmd,ctx.swapImages[swapIndex],size);
            endSingleTimeCommands(ctx,cmd);
            if(cloudValidation && rig.cloud){uint64_t ticks[2];vkRequireSuccess(vkGetQueryPoolResults(ctx.device,cloudQueries,0,2,sizeof(ticks),ticks,sizeof(uint64_t),VK_QUERY_RESULT_64_BIT|VK_QUERY_RESULT_WAIT_BIT),"cloud GPU timing");
                printf("CLOUD_GPU %s frame=%d ms=%.4f\n",rig.name,frame,double(ticks[1]-ticks[0])*cloudProps.limits.timestampPeriod/1e6);}
            vkRequireSuccess(vmaInvalidateAllocation(ctx.allocator,reference.allocation,0,VK_WHOLE_SIZE),"orbit composite invalidate");
            auto composite=static_cast<const uint16_t*>(reference.mapped);std::vector<double> compositeY(pixels);
            for(size_t i=0;i<pixels;++i){const double weights[3]={.2126,.7152,.0722};
                for(int c=0;c<3;++c)compositeY[i]+=weights[c]*Radiometry::aces(halfFloat(composite[4*i+c]),taa.displayExposure);}
            if(emptyCloud){vkRequireSuccess(vmaInvalidateAllocation(ctx.allocator,preCloud.allocation,0,VK_WHOLE_SIZE),"clear-cloud invalidate");
                auto before=static_cast<const uint16_t*>(preCloud.mapped);size_t changed=0;
                for(size_t i=0;i<pixels;++i)for(int c=0;c<3;++c)if(before[4*i+c]!=composite[4*i+c]){++changed;break;}
                printf("CLOUD_IDENTITY frame=%d changed=%zu\n",frame,changed);
                if(enforce)require(changed==0,"empty cloud reapplies atmospheric transport");
                vmaDestroyBuffer(ctx.allocator,preCloud.buffer,preCloud.allocation);}
            vkRequireSuccess(vmaInvalidateAllocation(ctx.allocator,display.allocation,0,VK_WHOLE_SIZE),"orbit display invalidate");
            auto displayed=static_cast<const uint8_t*>(display.mapped);std::vector<double> displayY(pixels);
            bool bgra=ctx.swapFormat==VK_FORMAT_B8G8R8A8_UNORM || ctx.swapFormat==VK_FORMAT_B8G8R8A8_SRGB;
            for(size_t i=0;i<pixels;++i){const double weights[3]={.2126,.7152,.0722};
                for(int c=0;c<3;++c){double value=displayed[4*i+(bgra?2-c:c)]/255.;
                    displayY[i]+=weights[c]*(value<=.04045?value/12.92:std::pow((value+.055)/1.055,2.4));}}
            if(cloudValidation){
                if(rig.cloud)vkRequireSuccess(vmaInvalidateAllocation(ctx.allocator,cloudProbe.allocation,0,VK_WHOLE_SIZE),"cloud invalidate");
                auto cloud=static_cast<const uint16_t*>(cloudProbe.mapped);
                if(cloudResponse){double opacity=0,maxOpacity=0,maxRadiance=0;size_t retained=0,changed=0;
                    for(size_t i=0;i<pixels;++i){double a=1-halfFloat(cloud[4*i+3]);opacity+=a;maxOpacity=std::max(maxOpacity,a);
                        if(a>.001)++retained;
                        for(int c=0;c<3;++c)maxRadiance=std::max(maxRadiance,double(halfFloat(cloud[4*i+c])));}
                    if(frame==47)responseBaseline=opacity;
                    if(responseClear){vkRequireSuccess(vmaInvalidateAllocation(ctx.allocator,preCloud.allocation,0,VK_WHOLE_SIZE),"response clear invalidate");
                        auto before=static_cast<const uint16_t*>(preCloud.mapped);
                        for(size_t i=0;i<pixels;++i)for(int c=0;c<3;++c)if(before[4*i+c]!=composite[4*i+c]){++changed;break;}
                        vmaDestroyBuffer(ctx.allocator,preCloud.buffer,preCloud.allocation);}
                    printf("CLOUD_RESPONSE %s frame=%d elapsed=%d residual=%.8f retained=%zu maxOpacity=%.6f maxRadiance=%.6f compositeChanged=%zu\n",
                        rig.name,frame,frame-48,frame>=47?opacity/std::max(responseBaseline,1e-12):-1.,retained,maxOpacity,maxRadiance,changed);
                    if(frame==95)printf("CLOUD_RESPONSE_DIAGNOSIS %s residualAfter48Frames=%.8f; diagnostic measurement, not temporal quality acceptance\n",
                        rig.name,opacity/std::max(responseBaseline,1e-12));
                    if(enforce && responseClear){
                        require(opacity/std::max(responseBaseline,1e-12)<.01,"cloud response retains over one percent opacity");
                        if(frame>=63 || rig.full)require(retained==0 && changed==0 && maxRadiance==0,"empty cloud response fails background identity");
                    }
                }
                std::vector<double> cloudT(pixels,1);size_t ring=0,outside=0,ghost=0,maxPixel=0;double tSq=0,tMax=0,hdrSq=0,hdrMax=0,displaySq=0,displayMax=0;
                double camR=6371+rig.altitude,halfFov=std::tan(rig.fov*.5);
                double pixelKm=std::sqrt(2.)*camR*std::pow(1-6371.*6371./(camR*camR),1.5)*2*halfFov/size.height;
                for(size_t i=0;i<pixels;++i){
                    double rx=(2*(i%size.width+.5)/size.width-1)*snap.aspect*halfFov;
                    double rz=(1-2*(i/size.width+.5)/size.height)*halfFov;
                    // lookAt(down Y, up Z) has camera-right = negative world X.
                    double norm=std::sqrt(1+rx*rx+rz*rz),along=(camR+x*rx)/norm;
                    double impact=std::sqrt(std::max(0.,camR*camR+x*x-along*along));
                    if(cloud){for(int c=0;c<4;++c)require(std::isfinite(halfFloat(cloud[4*i+c])),"nonfinite cloud RT");
                        cloudT[i]=halfFloat(cloud[4*i+3]);require(cloudT[i]>=0 && cloudT[i]<=1,"cloud T outside [0,1]");}
                    if(impact>6391+pixelKm){++outside;if(cloudT[i]<.999)++ghost;}
                    // Include both the opaque disk and the outer cloud shell.
                    if(std::min(std::abs(impact-6371),std::abs(impact-6391))<3*pixelKm && frame>0){++ring;
                        double d=std::abs(cloudT[i]-previousCloudT[i]);tSq+=d*d;tMax=std::max(tMax,d);
                        d=std::abs(compositeY[i]-previousCompositeY[i]);hdrSq+=d*d;hdrMax=std::max(hdrMax,d);
                        d=std::abs(displayY[i]-previousDisplayY[i]);displaySq+=d*d;if(d>displayMax){displayMax=d;maxPixel=i;}}
                }
                printf("CLOUD_LIMB %s frame=%d ring=%zu outside=%zu ghost=%zu T_RMS=%.8f T_max=%.6f HDR_RMS=%.8f HDR_max=%.6f display_RMS=%.8f display_max=%.6f\n",
                    rig.name,frame,ring,outside,ghost,std::sqrt(tSq/std::max(size_t(1),ring)),tMax,std::sqrt(hdrSq/std::max(size_t(1),ring)),hdrMax,std::sqrt(displaySq/std::max(size_t(1),ring)),displayMax);
                if(displayMax>.1 && !rig.full){size_t i=maxPixel;
                    double rx=(2*(i%size.width+.5)/size.width-1)*snap.aspect*halfFov;
                    double rz=(1-2*(i/size.width+.5)/size.height)*halfFov;
                    auto impactAt=[&](double a,double b){double along=(camR+x*a)/std::sqrt(1+a*a+b*b);return std::sqrt(std::max(0.,camR*camR+x*x-along*along));};
                    double impact=impactAt(rx,rz),foot=std::abs(impactAt(rx+2*snap.aspect*halfFov/size.width,rz)-impact)+std::abs(impactAt(rx,rz+2*halfFov/size.height)-impact);
                    printf("CLOUD_OUTLIER pixel=(%zu,%zu) impact=%.3f footprint=%.3f limb=%d T=%.6f prevT=%.6f\n",i%size.width,i/size.width,impact,foot,int(std::min(std::abs(impact-6391),std::abs(impact-6372))<4*foot),cloudT[i],previousCloudT[i]);}
                previousCloudT.swap(cloudT);
                if(cloudDynamic && !rig.timeJump){
                    int sample=frame==31?0:frame==32?1:frame==48?2:frame==63?3:-1;
                    if(sample>=0){auto& expected=dynamicReference[rig.wind?1:0][sample];
                        if(!rig.full){expected.resize(pixels*4);for(size_t i=0;i<pixels*4;++i)expected[i]=halfFloat(cloud[i]);}
                        else {double tError=0,lError=0;size_t count=0;
                            require(expected.size()==pixels*4,"missing dynamic paired reference");
                            for(size_t i=0;i<pixels;++i){
                                double rx=(2*(i%size.width+.5)/size.width-1)*snap.aspect*halfFov;
                                double rz=(1-2*(i/size.width+.5)/size.height)*halfFov;
                                double along=(camR+x*rx)/std::sqrt(1+rx*rx+rz*rz);
                                double impact=std::sqrt(std::max(0.,camR*camR+x*x-along*along));
                                if(impact>=6371-4*pixelKm)continue; // full reference has no silhouette supersampling
                                double d=expected[4*i+3]-halfFloat(cloud[4*i+3]);tError+=d*d;
                                for(int c=0;c<3;++c){double a=expected[4*i+c],b=halfFloat(cloud[4*i+c]);d=(a-b)/(1+std::max(a,b));lError+=d*d;}
                                ++count;
                            }
                            double tr=std::sqrt(tError/std::max(size_t(1),count)),lr=std::sqrt(lError/(3*std::max(size_t(1),count)));
                            printf("CLOUD_DYNAMIC_REFERENCE %s frame=%d interior=%zu opacityRMS=%.8f radianceRelativeRMS=%.8f\n",rig.name,frame,count,tr,lr);
                            if(enforce){require(count>0 && tr<.05 && lr<.05,"dynamic cloud differs from current full-ray reference");}
                        }
                    }
                }
                if(rig.cloud)vmaDestroyBuffer(ctx.allocator,cloudProbe.buffer,cloudProbe.allocation);
                if(enforce){require(ghost==0,"cloud reconstruction leaks beyond analytic shell");
                    if(frame>=16 && !cameraCut && !rig.full && !cloudResponse && !cloudDynamic){
                        // Fixed-camera noise and actual moving coverage have
                        // different bounds. Allow at most the camera's pixel
                        // travel in addition to the stationary RMS budget.
                        double motionPixels=rig.moving?.1/pixelKm:0.;
                        require(std::sqrt(tSq/std::max(size_t(1),ring))<.005+motionPixels,"cloud limb transmittance flickers beyond camera motion");
                        // The cut frame disables scene TAA. Its next frame
                        // turns variance clipping back on, so that transition
                        // is not a comparison of two identical resolve modes.
                        bool resolveTransition=std::string(rig.name)=="cloud-quarter-cut" && frame==17;
                        if(resolveTransition)require(std::sqrt(hdrSq/std::max(size_t(1),ring))<.005 && hdrMax<.1,"cloud cold-start HDR flickers");
                        else require(std::sqrt(displaySq/std::max(size_t(1),ring))<.005 && displayMax<.12,"cloud limb presentation flickers");}}
            }
            display.report(ctx,size,rig.name,frame==0?&reference:nullptr,false);
            if(frame==0)reference.report(ctx,size,rig.name,"first-hdr");
            else vmaDestroyBuffer(ctx.allocator,reference.buffer,reference.allocation);
            vkRequireSuccess(vmaInvalidateAllocation(ctx.allocator,allocation,0,VK_WHOLE_SIZE),"orbit invalidate");
            auto color=static_cast<const uint16_t*>(info.pMappedData);
            auto mask=static_cast<const uint8_t*>(info.pMappedData)+pixels*8;
            auto depth=reinterpret_cast<const float*>(static_cast<const uint8_t*>(info.pMappedData)+depthOffset);
            if(cloudValidation && rig.occluder){
                vkRequireSuccess(vmaInvalidateAllocation(ctx.allocator,preCloud.allocation,0,VK_WHOLE_SIZE),"pre-cloud invalidate");
                auto before=static_cast<const uint16_t*>(preCloud.mapped);size_t foregroundPixels=0,changed=0;
                for(size_t i=0;i<pixels;++i)if(depth[i]<1-rig.nearPlane/(rig.altitude*.5)){
                    ++foregroundPixels;for(int c=0;c<3;++c)if(before[4*i+c]!=composite[4*i+c]){++changed;break;}}
                printf("CLOUD_FOREGROUND frame=%d pixels=%zu changed=%zu\n",frame,foregroundPixels,changed);
                if(enforce)require(foregroundPixels>pixels/100 && changed==0,"cloud overwrites orbital foreground");
                vmaDestroyBuffer(ctx.allocator,preCloud.buffer,preCloud.allocation);
            }
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
            if(enforce && !cloudValidation){require(wet>pixels/20,"orbit ocean coverage missing");require(holeFraction<1e-5,"orbit water loses depth ties to bed");
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
    if(cloudQueries)vkDestroyQueryPool(ctx.device,cloudQueries,nullptr);
}
