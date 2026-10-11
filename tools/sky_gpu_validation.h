#pragma once
// Production sky -> linear HDR -> production TAA/display, not a preview shader.
static void validateSky(VulkanContext& ctx,VkRenderer3D& renderer,VkTAA& taa,GLFWwindow* window,int stage) {
    VkExtent2D size{1280,720};size_t pixels=size_t(size.width)*size.height;
    glfwSetWindowSize(window,size.width,size.height);glfwPollEvents();
    require(ctx.recreateSwapchain(window),"sky resize");
    require(taa.recreate(ctx,size,ctx.swapFormat,"src/render/shaders/spirv/taa.vert.spv","src/render/shaders/spirv/taa.frag.spv"),"sky HDR resize");
    auto original=renderer.sky.tune;double baseMean=0;std::vector<double> baseline(pixels*3);
    std::vector<unsigned> overlaps(pixels);
    for(const auto& star:renderer.sky.catalog){double z=-star.directionFlux[2];if(z<=0)continue;
        double fy=1/std::tan(.4),fx=fy*size.height/size.width;
        double sx=(fx*star.directionFlux[0]/z*.5+.5)*size.width,sy=(-fy*star.directionFlux[1]/z*.5+.5)*size.height;
        int radius=int(std::ceil(8*original.starSize+1));
        if(sx+radius<0||sy+radius<0||sx-radius>=size.width||sy-radius>=size.height)continue;
        for(int y=std::max(0,int(std::floor(sy-radius)));y<std::min(int(size.height),int(std::ceil(sy+radius)));++y)
            for(int x=std::max(0,int(std::floor(sx-radius)));x<std::min(int(size.width),int(std::ceil(sx+radius)));++x)++overlaps[size_t(y)*size.width+x];}
    VkQueryPool timing;VkQueryPoolCreateInfo qi{VK_STRUCTURE_TYPE_QUERY_POOL_CREATE_INFO};qi.queryType=VK_QUERY_TYPE_TIMESTAMP;qi.queryCount=2;
    vkRequireSuccess(vkCreateQueryPool(ctx.device,&qi,nullptr,&timing),"sky timer");double seamMean=0;std::vector<double> rotationDisplay;
    for(int test=0;test<(stage>=5?35:stage>=4?27:stage>=3?18:stage>=2?11:4);++test) {
        VkExtent2D nextSize=test>=33?VkExtent2D{1279,719}:test==7?VkExtent2D{640,360}:VkExtent2D{1280,720};
        if(nextSize.width!=size.width){size=nextSize;pixels=size_t(size.width)*size.height;
            glfwSetWindowSize(window,size.width,size.height);glfwPollEvents();require(ctx.recreateSwapchain(window),"PSF swapchain resize");
            require(taa.recreate(ctx,size,ctx.swapFormat,"src/render/shaders/spirv/taa.vert.spv","src/render/shaders/spirv/taa.frag.spv"),"PSF HDR resize");}
        renderer.sky.tune=original;renderer.sky.tune.exposure=test==1?2.f:test==2?0.f:1.f;
        if(test>=4 && test<=10){renderer.sky.tune.milkyWayIntensity=renderer.sky.tune.bgLevel=0;
            renderer.sky.tune.starSize=test==6?1.3f:.65f;
            VkSkySystem::Star point{{0,0,-1,1},{1,1,1,1}};
            if(test==5){point.directionFlux[0]=.00044f;point.directionFlux[1]=.00031f;point.directionFlux[2]=-std::sqrt(1-.00044f*.00044f-.00031f*.00031f);}
            if(test==9){point.directionFlux[0]=.25f;point.directionFlux[2]=-std::sqrt(1-.25f*.25f);}
            if(test==10)point.directionFlux[2]=1;
            renderer.sky.uploadStars(ctx,std::vector<VkSkySystem::Star>{point});}
        taa.displayExposure=test==3?128.f:Radiometry::defaultDisplayExposure;
        float view[16],proj[16];Mat4().toFloatArray(view);Mat4::perspective(test==8?1.3:.8,double(size.width)/size.height,.1,10).toFloatArray(proj);
        if(test>=11 && test<18){renderer.sky.uploadStars(ctx,renderer.sky.catalog);taa.displayExposure=128;
            Vec3 direction(-.0548755604,-.4838350155,-.8734370902);
            if(test==12 || test==15 || test==16)direction=direction*-1;
            if(test==13 || test==14)direction=Vec3(-.8676661490,.4559837762,-.1980763734)*(test==14?-1:1);
            if(test==15 || test==16){direction.x+=(test==15?-1:1)*1e-5;renderer.sky.tune.starDensity=0;}
            if(test==17){renderer.sky.tune.starDensity=renderer.sky.tune.milkyWayIntensity=renderer.sky.tune.bgLevel=0;}
            Mat4::lookAt(Vec3(0,0,0),direction,Vec3(0,1,0)).toFloatArray(view);}
        if(test>=18 && test<=26){renderer.sky.tune.milkyWayIntensity=renderer.sky.tune.bgLevel=0;
            renderer.sky.uploadStars(ctx,std::vector<VkSkySystem::Star>{{{0,0,-1,1},{1,1,1,1}}});
            Mat4::lookAt(Vec3(0,0,0),Vec3((test-18)*.00044,0,-1),Vec3(0,1,0)).toFloatArray(view);taa.displayExposure=128;}
        if(test>=27){renderer.sky.uploadStars(ctx,renderer.sky.catalog);renderer.sky.tune.starDensity=0;taa.displayExposure=128;
            Vec3 direction(-1,0,test==27?1e-5:-1e-5),up(0,1,0);
            if(test>=29){direction=Vec3(test<31?1e-6:0,(test==29||test==31||test==33)?1:-1,0);up=Vec3(0,0,1);}
            Mat4::lookAt(Vec3(0,0,0),direction,up).toFloatArray(view);
            Mat4::perspective(1.3,double(size.width)/size.height,.1,10).toFloatArray(proj);}
        taa.updateMatrices(view,proj);taa.uploadMatrices(ctx,0);
        VkFence fence;VkFenceCreateInfo fi{VK_STRUCTURE_TYPE_FENCE_CREATE_INFO};
        vkRequireSuccess(vkCreateFence(ctx.device,&fi,nullptr,&fence),"sky acquire fence");uint32_t index;
        vkRequireSuccess(vkAcquireNextImageKHR(ctx.device,ctx.swapchain,5000000000ULL,VK_NULL_HANDLE,fence,&index),"sky acquire");
        vkRequireSuccess(vkWaitForFences(ctx.device,1,&fence,VK_TRUE,5000000000ULL),"sky wait");
        VkCommandBuffer cmd=beginSingleTimeCommands(ctx);vkCmdResetQueryPool(cmd,timing,0,2);
        vkCmdWriteTimestamp(cmd,VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT,timing,0);
        taa.beginGeometryPass(cmd,size);VkFrameRenderer::setViewportScissor(cmd,size);
        renderer.sky.draw(cmd,0,view,proj,1.f,float(size.width),float(size.height));taa.endGeometryPass(cmd);
        HdrProbe cameraInput;
        if(stage>=6){cameraInput.capture(ctx,cmd,taa.colorImages[taa.currentIdx],size);SceneSnapshot camera{};memcpy(camera.proj,proj,sizeof(proj));
            renderer.cameraBloom.render(ctx,cmd,taa,camera,0,size,renderer.scene.billboardQuadVbo);}
        vkCmdWriteTimestamp(cmd,VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT,timing,1);
        HdrProbe hdr;hdr.capture(ctx,cmd,taa.colorImages[taa.currentIdx],size);
        taa.beginResolvePass(cmd,ctx.swapImages[index],ctx.swapImageViews[index],size);VkFrameRenderer::setViewportScissor(cmd,size);
        taa.drawResolve(cmd,test>18?.1f:1.f);taa.endResolvePass(cmd,ctx.swapImages[index]);DisplayProbe display;display.capture(ctx,cmd,ctx.swapImages[index],size);
        endSingleTimeCommands(ctx,cmd);
        uint64_t times[2]={};vkRequireSuccess(vkGetQueryPoolResults(ctx.device,timing,0,2,sizeof(times),times,sizeof(uint64_t),VK_QUERY_RESULT_64_BIT|VK_QUERY_RESULT_WAIT_BIT),"sky timing result");
        VkPhysicalDeviceProperties properties{};vkGetPhysicalDeviceProperties(ctx.physDevice,&properties);
        double milliseconds=(times[1]-times[0])*properties.limits.timestampPeriod/1e6;
        vkRequireSuccess(vmaInvalidateAllocation(ctx.allocator,hdr.allocation,0,VK_WHOLE_SIZE),"sky HDR invalidate");
        if(stage>=6)vkRequireSuccess(vmaInvalidateAllocation(ctx.allocator,cameraInput.allocation,0,VK_WHOLE_SIZE),"sky pre-camera invalidate");
        vkRequireSuccess(vmaInvalidateAllocation(ctx.allocator,display.allocation,0,VK_WHOLE_SIZE),"sky display invalidate");
        auto h=static_cast<const uint16_t*>(hdr.mapped);auto d=static_cast<const uint8_t*>(display.mapped);
        auto input=stage>=6?static_cast<const uint16_t*>(cameraInput.mapped):nullptr;
        bool bgra=ctx.swapFormat==VK_FORMAT_B8G8R8A8_UNORM||ctx.swapFormat==VK_FORMAT_B8G8R8A8_SRGB;
        double mean=0,maxValue=0,energy=0,inputEnergy=0,quantizationBound=0,displayLuminance=0;size_t clipped=0;int error=0;
        std::string name="build/tests/sky-stage"+std::to_string(stage)+"-"+(test>=4?"psf"+std::to_string(test):test==3?"observation":test==0?"daylight":test==1?"double":"zero");
        std::ofstream image(name+".ppm",std::ios::binary);image<<"P6\n"<<size.width<<" "<<size.height<<"\n255\n";
        for(size_t i=0;i<pixels;++i)for(int c=0;c<3;++c) {
            double value=halfFloat(h[4*i+c]);require(std::isfinite(value)&&value>=0,"invalid sky radiance");mean+=value;maxValue=std::max(maxValue,value);
            if(input){double prior=halfFloat(input[4*i+c]);double ulp=halfFloat(uint16_t(input[4*i+c]+1))-prior;
                require(value>=prior*(1-VkCameraBloom::scatterFraction)-2*ulp,"camera scatter removed more than its source/rounding budget");}
            if(test==0)baseline[3*i+c]=value;
            // RGBA16F subnormal spacing is 2^-24. Bound absolute quantization
            // as well as normal-value relative error; a mean-only threshold
            // falsely fails dark pixels near zero.
            if(test==1 && std::abs(value-2*baseline[3*i+c])>=.003*value+(4+4*overlaps[i])*std::ldexp(1.0,-24)) {
                fprintf(stderr,"SCALE pixel=%zu channel=%d base=%.10g double=%.10g\n",i,c,baseline[3*i+c],value);
                require(false,"sky source scaling exceeds FP16 error bound");}
            if(c==0){double x=(2*(double(i%size.width)+.5)/size.width-1)/proj[0];double y=(2*(double(i/size.width)+.5)/size.height-1)/proj[5];
                double omega=4.0/(size.width*size.height*proj[0]*proj[5]*std::pow(1+x*x+y*y,1.5));energy+=value*omega;
                if(input){double prior=halfFloat(input[4*i]);inputEnergy+=prior*omega;
                    if(prior>0)quantizationBound+=2*(halfFloat(uint16_t(input[4*i]+1))-prior)*omega;}}
            uint8_t actual=d[4*i+(bgra?2-c:c)];image.write(reinterpret_cast<char*>(&actual),1);clipped+=actual==255;
            double s=actual/255.;double linear=s<=.04045?s/12.92:std::pow((s+.055)/1.055,2.4);
            const double weights[]={.2126,.7152,.0722};displayLuminance+=weights[c]*linear;
            int expected=int(std::round(255*Radiometry::srgb(Radiometry::aces(value,taa.displayExposure))));error=std::max(error,std::abs(expected-int(actual)));
        }
        mean/=pixels*3;require(error<=2,"sky presentation transfer mismatch");
        if(test==0)baseMean=mean;
        if(test==1)printf("SKY_SCALE ratio=%.8f (FP16 absolute/relative bound checked per channel)\n",mean/baseMean);
        if(test==2)require(maxValue==0,"zero sky source must be black");
        if((test>=4 && test<10)||(test>=18 && test<=26)){printf("PSF case=%d flux=%.10g relativeError=%.6g\n",test,energy,std::abs(energy/2e-7-1));
            double psfEnergy=input?inputEnergy:energy;
            require(std::abs(psfEnergy/2e-7-1)<.003,"PSF energy changes with size, subpixel location, resolution or FOV");
            if(input){printf("CAMERA_POINT_ENERGY input=%.10g output=%.10g loss=%.6g roundingBound=%.10g\n",inputEnergy,energy,1-energy/inputEnergy,quantizationBound);
                require(energy>=inputEnergy*(1-VkCameraBloom::scatterFraction)-quantizationBound&&energy<=inputEnergy*(1+VkCameraBloom::scatterFraction)+quantizationBound,"post-camera point energy exceeds explicit scatter/FP16 budget");}}
        if(test==10 || test==17)require(maxValue==0,"disabled or back-facing sky sources must be black");
        if(test==15)seamMean=mean;
        if(test==16)require(std::abs(mean-seamMean)<seamMean*.002+1e-9,"Galactic longitude seam discontinuity");
        if(test==27)seamMean=mean;
        if(test==28)require(std::abs(mean-seamMean)<seamMean*.002+1e-9,"celestial texture seam discontinuity");
        if(test>=18 && test<=26)rotationDisplay.push_back(displayLuminance);
        printf("PASS SKY stage=%d case=%d mean=%.9g max=%.7g clipped=%.5f%% displayError=%d GPU=%.3fms\n",stage,test,mean,maxValue,100.*clipped/(pixels*3),error,milliseconds);
        vmaDestroyBuffer(ctx.allocator,hdr.buffer,hdr.allocation);vmaDestroyBuffer(ctx.allocator,display.buffer,display.allocation);
        if(input)vmaDestroyBuffer(ctx.allocator,cameraInput.buffer,cameraInput.allocation);
        vkRequireSuccess(vkResetFences(ctx.device,1,&fence),"sky reset fence");
        VkSwapchainPresentFenceInfoEXT completion{VK_STRUCTURE_TYPE_SWAPCHAIN_PRESENT_FENCE_INFO_EXT};completion.swapchainCount=1;completion.pFences=&fence;
        VkPresentInfoKHR present{VK_STRUCTURE_TYPE_PRESENT_INFO_KHR};present.pNext=&completion;present.swapchainCount=1;present.pSwapchains=&ctx.swapchain;present.pImageIndices=&index;
        vkRequireSuccess(vkQueuePresentKHR(ctx.presentQueue,&present),"sky present");
        vkRequireSuccess(vkWaitForFences(ctx.device,1,&fence,VK_TRUE,5000000000ULL),"sky present complete");vkDestroyFence(ctx.device,fence,nullptr);taa.swap();
        require(g_vkValidationErrorCount==0,"sky Vulkan validation errors");
    }
    renderer.sky.uploadStars(ctx,renderer.sky.catalog);renderer.sky.tune=original;taa.displayExposure=Radiometry::defaultDisplayExposure;
    if(!rotationDisplay.empty()){double sum=0;for(double value:rotationDisplay)sum+=value;
        auto range=std::minmax_element(rotationDisplay.begin(),rotationDisplay.end());
        double variation=(*range.second-*range.first)/(sum/rotationDisplay.size());
        printf("SKY_ROTATION display-integrated linear Y phase range=%.4f%% (includes ACES and 8-bit quantization)\n",100*variation);
        require(variation<.05,"subpixel motion produces excessive display brightness variation");}
    // Continuous GPU work, without per-frame CPU image analysis, avoids
    // mistaking laptop idle-clock timestamps for real-time rendering cost.
    if(stage>=4){float view[16],proj[16];Mat4().toFloatArray(view);Mat4::perspective(.8,double(size.width)/size.height,.1,10).toFloatArray(proj);
        VkCommandBuffer cmd=beginSingleTimeCommands(ctx);vkCmdResetQueryPool(cmd,timing,0,2);
        for(int frame=0;frame<40;++frame){
            if(frame==10)vkCmdWriteTimestamp(cmd,VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT,timing,0);
            taa.beginGeometryPass(cmd,size);VkFrameRenderer::setViewportScissor(cmd,size);
            renderer.sky.draw(cmd,0,view,proj,1.f,float(size.width),float(size.height));taa.endGeometryPass(cmd);}
        vkCmdWriteTimestamp(cmd,VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT,timing,1);endSingleTimeCommands(ctx,cmd);
        uint64_t times[2]={};vkRequireSuccess(vkGetQueryPoolResults(ctx.device,timing,0,2,sizeof(times),times,sizeof(uint64_t),VK_QUERY_RESULT_64_BIT|VK_QUERY_RESULT_WAIT_BIT),"steady sky timing");
        VkPhysicalDeviceProperties properties{};vkGetPhysicalDeviceProperties(ctx.physDevice,&properties);
        printf("SKY_STEADY %ux%u catalog=%u 10 warmup + 30 frames meanGPU=%.3fms (sky draw and attachment barriers only)\n",size.width,size.height,renderer.sky.starCount,(times[1]-times[0])*properties.limits.timestampPeriod/30e6);
        require(g_vkValidationErrorCount==0,"sky benchmark validation errors");}
    vkDestroyQueryPool(ctx.device,timing,nullptr);
}
