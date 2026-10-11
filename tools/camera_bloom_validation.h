#pragma once
// Synthetic linear-HDR fixtures isolate the scene-wide operator from solar PSF.
static void validateCameraBloom(VulkanContext& ctx,VkRenderer3D& renderer,VkTAA& taa) {
    double repeatFlux=0;
    for(int test=0;test<11;++test){
        VkExtent2D size=test==6?VkExtent2D{257,145}:test==7?VkExtent2D{17,9}:test==8?VkExtent2D{1,1}:VkExtent2D{256,144};
        require(taa.recreate(ctx,size,ctx.swapFormat,"src/render/shaders/spirv/taa.vert.spv","src/render/shaders/spirv/taa.frag.spv"),"bloom fixture resize");
        SceneSnapshot snap{};snap.aspect=float(size.width)/size.height;
        Mat4().toFloatArray(snap.view);Mat4::perspective(.8,snap.aspect,.1,10).toFloatArray(snap.proj);
        snap.lightDir[2]=1;
        if(test==4){snap.sunInFront=true;snap.sunScreen[0]=snap.sunScreen[1]=0;snap.sunRadiusKm=696340;snap.sunDistanceKm=SolarOptics::auKm;}
        if(test==9)snap.proj[5]=std::numeric_limits<float>::quiet_NaN();
        if(test==10){snap.sunInFront=true;snap.sunRadiusKm=696340;snap.sunDistanceKm=SolarOptics::auKm;
            snap.sunScreen[0]=std::numeric_limits<float>::infinity();}
        VkCommandBuffer cmd=beginSingleTimeCommands(ctx);
        taa.beginGeometryPass(cmd,size);VkFrameRenderer::setViewportScissor(cmd,size);
        VkClearAttachment color{};color.aspectMask=VK_IMAGE_ASPECT_COLOR_BIT;color.clearValue.color={{0,0,0,1}};
        if(test==1||test>=7)color.clearValue.color={{.18f,.18f,.18f,1}};
        VkClearRect rect{{{0,0},size},0,1};vkCmdClearAttachments(cmd,1,&color,1,&rect);
        if(test==2||test==3||test==5||test==6){
            color.clearValue.color={{16,16,16,1}};rect.rect={{test==3?0:int(size.width/2-8),int(size.height/2-8)},{16,16}};
            vkCmdClearAttachments(cmd,1,&color,1,&rect);
        }
        if(test==4){renderer.beginFrame(cmd,0,snap.view,snap.proj,snap.lightDir,snap.camPos,0);renderer.drawSolarDisk(cmd,snap,size);}
        taa.endGeometryPass(cmd);HdrProbe before,after;before.capture(ctx,cmd,taa.colorImages[taa.currentIdx],size);
        renderer.cameraBloom.render(ctx,cmd,taa,snap,test%2,size,renderer.scene.billboardQuadVbo);
        after.capture(ctx,cmd,taa.colorImages[taa.currentIdx],size);endSingleTimeCommands(ctx,cmd);
        vkRequireSuccess(vmaInvalidateAllocation(ctx.allocator,before.allocation,0,VK_WHOLE_SIZE),"bloom input read");
        vkRequireSuccess(vmaInvalidateAllocation(ctx.allocator,after.allocation,0,VK_WHOLE_SIZE),"bloom output read");
        auto a=static_cast<const uint16_t*>(after.mapped),b=static_cast<const uint16_t*>(before.mapped);
        double input=0,output=0,outside=0,opposite=0,peak=0,maxChange=0,centroid=0,momentX=0,momentY=0;
        std::ofstream raw("build/tests/camera-bloom-"+std::to_string(test)+".bin",std::ios::binary);
        raw.write(reinterpret_cast<const char*>(a),size_t(size.width)*size.height*8);
        for(size_t i=0;i<size_t(size.width)*size.height;++i){
            for(int c=0;c<3;++c)require(std::isfinite(halfFloat(a[4*i+c]))&&halfFloat(a[4*i+c])>=0,"bloom invalid HDR");
            double v=halfFloat(a[4*i]),prior=halfFloat(b[4*i]);input+=prior;output+=v;peak=std::max(peak,v);
            maxChange=std::max(maxChange,std::abs(v-prior));centroid+=v*(i%size.width+.5);
            double dx=double(i%size.width)+.5-int(size.width/2),dy=double(i/size.width)+.5-int(size.height/2);
            momentX+=v*dx*dx;momentY+=v*dy*dy;
            if(prior==0)outside+=v;if(i%size.width>=size.width-16)opposite+=v;
        }
        double ratio=input?output/input:0;
        printf("BLOOM_FIXTURE %d %ux%u input=%.9g output=%.9g ratio=%.9g outside=%.9g peak=%.9g maxChange=%.9g centroidX=%.9g\n",test,size.width,size.height,input,output,ratio,outside,peak,maxChange,output?centroid/output:0);
        if(test==0)require(output==0,"black bloom must stay black");
        if(test==1)require(ratio>.997&&ratio<=1.001&&std::abs(halfFloat(a[4*(size.width*(size.height/2)+size.width/2)])-.18)<.0003,"gray bloom DC preservation");
        if(test==2||test==5||test==6){require(ratio>.997&&ratio<1.001&&outside>1&&peak<16,"bloom scatter/conservation");
            require(std::abs(centroid/output-size.width/2)<.6,"bloom interior centroid");}
        if(test==2){
            // Physical angular mixture, with explicit finite pyramid/sensor
            // sampling envelope. Cropped vertical tails reduce its second moment.
            const double weights[]={.15,.25,.30,.20,.10},angles[]={.0015,.006,.020,.060,.15};
            double nominal=0,samplingBound=0,focalPixels=std::abs(snap.proj[5])*size.height/2;
            for(int i=0;i<5;++i){double scale=1<<(i+1);nominal+=.005*weights[i]*std::pow(focalPixels*angles[i],2);
                samplingBound+=.005*weights[i]*scale*scale*(1./6+.15);}
            double addedX=momentX/output-21.25,addedY=momentY/output-21.25;
            printf("BLOOM_WIDTH addedVariance=(%.9g,%.9g) nominal=%.9g samplingBound=%.9g\n",addedX,addedY,nominal,samplingBound);
            require(addedX>nominal-samplingBound&&addedX<nominal+samplingBound&&addedY>nominal-samplingBound-.05&&addedY<nominal+samplingBound,"bloom angular width outside sampling/crop envelope");
        }
        if(test==2)repeatFlux=output;if(test==5)require(std::abs(output-repeatFlux)<repeatFlux*1e-5,"bloom frame-slot repeatability");
        if(test==3)require(output<=input*1.001&&ratio>.994&&opposite<input*1e-6,"bloom cropped field/no wrapping");
        if(test==4)require(input>1&&maxChange==0,"nonempty solar source must not be bloomed twice");
        if(test>=7)require(output<=input*1.001&&ratio>.99,"tiny bloom extent stability");
        vmaDestroyBuffer(ctx.allocator,before.buffer,before.allocation);vmaDestroyBuffer(ctx.allocator,after.buffer,after.allocation);
        require(g_vkValidationErrorCount==0,"bloom Vulkan validation");puts("PASS CAMERA_BLOOM");
    }
}
