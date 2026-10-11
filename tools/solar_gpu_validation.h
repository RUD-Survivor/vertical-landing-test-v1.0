#pragma once
static void validateSolar(VulkanContext& ctx,VkRenderer3D& renderer,VkTAA& taa,GLFWwindow* window,int cameraStage=3,bool shapeOnly=false) {
    setvbuf(stdout,nullptr,_IONBF,0);
    // Black opaque quad masks half/all the solar image using production depth.
    const float vertices[]={-1,-1,-1,0,0,1,0,0,1,1,1,1, 1,-1,-1,0,0,1,0,0,1,1,1,1,
                             1,1,-1,0,0,1,0,0,1,1,1,1, -1,1,-1,0,0,1,0,0,1,1,1,1};
    const uint32_t indices[]={0,1,2,0,2,3};
    require(renderer.registerMesh(ctx,"solar-occluder",vertices,sizeof(vertices),indices,6),"solar occluder mesh");
    // Same unit sphere topology and vertex format as production MeshGen::sphere,
    // without importing the OpenGL upload path into this Vulkan-only validator.
    std::vector<float> sphereVertices;std::vector<uint32_t> sphereIndices;
    const int latitudes=256,longitudes=512;
    for(int lat=0;lat<=latitudes;++lat)for(int lon=0;lon<=longitudes;++lon){
        double theta=double(lat)/latitudes*Radiometry::pi,phi=double(lon)/longitudes*2*Radiometry::pi;
        float x=float(std::cos(phi)*std::sin(theta)),y=float(std::cos(theta)),z=float(std::sin(phi)*std::sin(theta));
        const float vertex[]={x,y,z,x,y,z,float(lon)/longitudes,float(lat)/latitudes,1,1,1,1};
        sphereVertices.insert(sphereVertices.end(),vertex,vertex+12);
    }
    for(int lat=0;lat<latitudes;++lat)for(int lon=0;lon<longitudes;++lon){
        uint32_t a=lat*(longitudes+1)+lon,b=a+longitudes+1;
        const uint32_t triangle[]={a,b,a+1,b,b+1,a+1};sphereIndices.insert(sphereIndices.end(),triangle,triangle+6);
    }
    require(renderer.registerMesh(ctx,"earth",sphereVertices.data(),sphereVertices.size()*sizeof(float),sphereIndices.data(),uint32_t(sphereIndices.size())),"solar environment sphere");
    VkExtent2D size{};double baselineBody=0,baselineOptics=0;
    const int cases=cameraStage>=8?63:53;
    for(int test=0;test<cases;++test) {
        if(shapeOnly&&test!=0&&test!=9&&test!=14&&test!=17&&test!=38&&test!=39&&test!=41&&test!=44&&test!=45&&test!=46&&test!=47&&!(test>=53&&test<=60))continue;
        printf("SOLAR_BEGIN case=%d\n",test);
        const bool isolated=(test>=29&&test<=33)||(test>=41&&test<=43)||(test>=45&&test<=52);
        const bool environment=test>=38&&test<=40;
        VkExtent2D next=test==13?VkExtent2D{640,360}:test==44?VkExtent2D{1200,800}:test==47?VkExtent2D{1281,721}:VkExtent2D{1280,720};
        if(next.width!=size.width||next.height!=size.height){size=next;glfwSetWindowSize(window,size.width,size.height);glfwPollEvents();
            require(ctx.recreateSwapchain(window),"solar swapchain");
            require(taa.recreate(ctx,size,ctx.swapFormat,"src/render/shaders/spirv/taa.vert.spv","src/render/shaders/spirv/taa.frag.spv"),"solar HDR");}
        const double factors[]={1,.5,2,5,10,100,.1};double factor=test<7?factors[test]:1;
        if(test>=29&&test<=33){const double isolated[]={1,.5,2,10,100};factor=isolated[test-29];}
        if(test==35)factor=.1; // Resolved low-exposure photosphere observation.
        if(test>=48&&test<=52){const double transitionDistances[]={.1,.14,.18,.24,.32};factor=transitionDistances[test-48];}
        SceneSnapshot snap{};snap.sunRadiusKm=696340;snap.sunDistanceKm=SolarOptics::auKm*factor;snap.sunInFront=true;
        snap.sunScreen[0]=snap.sunScreen[1]=0;snap.aspect=float(size.width)/size.height;snap.lightDir[2]=1;
        if(test==46)snap.sunScreen[0]=float(.5*2/size.width);
        if(test>=53)snap.sunScreen[0]=float(.2+(test-53)*.5*2/size.width);
        if(test==56)snap.sunScreen[0]=-.4f; // Camera cut, clear-depth optics.
        Mat4().toFloatArray(snap.view);Mat4::perspective(test==12?1.3:.8,snap.aspect,.1,10).toFloatArray(snap.proj);
        if(test==44){Mat4::perspective(2*std::atan(12./35),snap.aspect,.1,10).toFloatArray(snap.proj);
            snap.sunScreen[0]=.51f;snap.sunScreen[1]=-.21f;}
        if(environment){
            const double altitude=test==38?400:.02,elevation=test==38?-.30:.10;
            Vec3 eye(0,6371+altitude,0),direction(0,std::sin(elevation),-std::cos(elevation));
            snap.camPos[1]=float(eye.y);snap.lightDir[1]=float(direction.y);snap.lightDir[2]=float(direction.z);
            snap.sunWorldPos[1]=float(eye.y+direction.y*snap.sunDistanceKm);
            snap.sunWorldPos[2]=float(direction.z*snap.sunDistanceKm);
            Mat4::lookAt(eye,eye+direction,Vec3(0,1,0)).toFloatArray(snap.view);
            Mat4::perspective(.8,snap.aspect,.001,30000).toFloatArray(snap.proj);
            CelestialDraw earth{};earth.bodyIdx=earth.atmoIdx=3;earth.radius=6371;earth.showClouds=test==40;
            earth.shaderKey="earth";earth.r=earth.g=earth.b=1;
            Mat4::scale(Vec3(6371,6371,6371)).toFloatArray(earth.model);snap.celestials.push_back(earth);
            snap.sunIntensity=1;snap.time=3;snap.dayBlend=1;
            if(test==40)require(renderer.cloudSystem.init(ctx,size,renderer.scene.cloudNoiseTex.view,renderer.scene.cloudDetailTex.view,
                renderer.scene.cloudWeatherTex.view,renderer.scene.cloudCurlTex.view,renderer.scene.cloudProfileLutTex.view,
                renderer.scene.cloudHiWeatherTex.view),"solar environment clouds init");
        }
        if(test==10)snap.sunDistanceKm=snap.sunRadiusKm;
        if(test==11)snap.sunInFront=false;
        if(test==14){snap.sunScreen[0]=.7f;snap.sunScreen[1]=.45f;}
        if(test==17){snap.sunScreen[0]=-1.05f;snap.sunScreen[1]=.2f;}
        if(test>=18 && test<=26)snap.sunScreen[0]=float((test-18)*.17*2/size.width);
        if(test==27)snap.sunDistanceKm=snap.sunRadiusKm*(1+1e-5);
        if(test==28){factor=10000;snap.sunDistanceKm=SolarOptics::auKm*factor;}
        if(test==61)snap.proj[5]=std::numeric_limits<float>::quiet_NaN();
        if(test==62)snap.sunScreen[0]=std::numeric_limits<float>::infinity();
        auto sun=SolarOptics::source(snap.sunDistanceKm,snap.sunRadiusKm);
        taa.updateMatrices(snap.view,snap.proj);taa.uploadMatrices(ctx,0);taa.displayExposure=Radiometry::defaultDisplayExposure;
        if(test>=34&&test<=36){const float exposures[]={0,.00002f,.25f};taa.displayExposure=exposures[test-34];}
        VkFence fence;VkFenceCreateInfo fi{VK_STRUCTURE_TYPE_FENCE_CREATE_INFO};
        vkRequireSuccess(vkCreateFence(ctx.device,&fi,nullptr,&fence),"solar fence");uint32_t index;
        vkRequireSuccess(vkAcquireNextImageKHR(ctx.device,ctx.swapchain,5000000000ULL,VK_NULL_HANDLE,fence,&index),"solar acquire");
        vkRequireSuccess(vkWaitForFences(ctx.device,1,&fence,VK_TRUE,5000000000ULL),"solar acquire wait");
        VkCommandBuffer cmd=beginSingleTimeCommands(ctx);
        taa.beginGeometryPass(cmd,size);VkFrameRenderer::setViewportScissor(cmd,size);
        VkClearAttachment black{};black.aspectMask=VK_IMAGE_ASPECT_COLOR_BIT;black.colorAttachment=0;black.clearValue.color={{0,0,0,1}};
        VkClearRect clear{{{0,0},size},0,1};vkCmdClearAttachments(cmd,1,&black,1,&clear);
        renderer.beginFrame(cmd,0,snap.view,snap.proj,snap.lightDir,snap.camPos,0);
        if(test==37)renderer.sky.draw(cmd,0,snap.view,snap.proj,1,float(size.width),float(size.height));
        if(environment){printf("SOLAR_CONTEXT geometry\n");renderer.render(cmd,0,snap,size);printf("SOLAR_CONTEXT geometry recorded\n");}
        else if(test==15 || test==16){float transmission=test==15?.25f:0.f;
            renderer.effects.drawLensFlare(cmd,renderer.scene.billboardQuadVbo,0,0,snap.aspect,float(sun.radiance)*transmission,
                snap.aspect,1,0,0,SolarOptics::sourceSpectrum[0],SolarOptics::sourceSpectrum[1],SolarOptics::sourceSpectrum[2],float(1-SolarOptics::scatteredFraction),4,std::abs(snap.proj[5]),float(sun.sinRadius));
        } else renderer.drawSolarDisk(cmd,snap,size);
        if(test==8||test==9||test==45||test==57){Mat4 model=Mat4::translate(Vec3(test==9||test==45?-1.:0,0,-1))*Mat4::scale(Vec3(1,1,1));
            model.m[14]=0; // vertices are already at z=-1.
            float matrix[16];model.toFloatArray(matrix);renderer.drawMesh(cmd,"solar-occluder",matrix,0,0,0,1,0);}
        taa.endGeometryPass(cmd);
        if(environment){printf("SOLAR_CONTEXT atmosphere\n");renderer.renderAtmoAfterGeometry(ctx,cmd,taa,snap,0);printf("SOLAR_CONTEXT atmosphere recorded\n");
            if(test==40)renderer.renderFlowerCloudAfterGeometry(ctx,cmd,taa,snap,0);}
        if(cameraStage>=4&&!isolated)renderer.cameraBloom.render(ctx,cmd,taa,snap,0,size,renderer.scene.billboardQuadVbo);
        HdrProbe before;before.capture(ctx,cmd,taa.colorImages[taa.currentIdx],size);
        if(test!=7)renderer.renderSolarOpticsAfterGeometry(ctx,cmd,taa,snap,test%2,size,isolated,
            test==42?8:test==43?9:SolarOptics::apertureBlades,cameraStage);
        HdrProbe after;after.capture(ctx,cmd,taa.colorImages[taa.currentIdx],size);
        taa.beginResolvePass(cmd,ctx.swapImages[index],ctx.swapImageViews[index],size);VkFrameRenderer::setViewportScissor(cmd,size);
        taa.drawResolve(cmd,test>=53?.1f:1.f);taa.endResolvePass(cmd,ctx.swapImages[index]);DisplayProbe display;display.capture(ctx,cmd,ctx.swapImages[index],size);
        if(environment)printf("SOLAR_CONTEXT submit\n");
        endSingleTimeCommands(ctx,cmd);
        if(environment)printf("SOLAR_CONTEXT complete\n");
        vkRequireSuccess(vmaInvalidateAllocation(ctx.allocator,before.allocation,0,VK_WHOLE_SIZE),"solar before invalidate");
        vkRequireSuccess(vmaInvalidateAllocation(ctx.allocator,after.allocation,0,VK_WHOLE_SIZE),"solar after invalidate");
        vkRequireSuccess(vmaInvalidateAllocation(ctx.allocator,display.allocation,0,VK_WHOLE_SIZE),"solar display invalidate");
        auto b=static_cast<const uint16_t*>(before.mapped),a=static_cast<const uint16_t*>(after.mapped);
        auto d=static_cast<const uint8_t*>(display.mapped);bool bgra=ctx.swapFormat==VK_FORMAT_B8G8R8A8_UNORM||ctx.swapFormat==VK_FORMAT_B8G8R8A8_SRGB;
        std::ofstream image("build/tests/solar-optics-"+std::to_string(test)+".ppm",std::ios::binary);image<<"P6\n"<<size.width<<" "<<size.height<<"\n255\n";
        double body=0,optics=0,maximum=0,spectralOptics[3]={};int error=0;size_t pixels=size_t(size.width)*size.height;
        bool validProjection=std::isfinite(snap.proj[0])&&std::isfinite(snap.proj[5])&&snap.proj[0]>0&&snap.proj[5]>0&&std::isfinite(snap.sunScreen[0])&&std::isfinite(snap.sunScreen[1]);
        Vec3 source=validProjection?Vec3(snap.sunScreen[0]/snap.proj[0],-snap.sunScreen[1]/snap.proj[5],-1):Vec3(0,0,-1);source=source.normalized();
        for(size_t i=0;i<pixels;++i){double x=(2*(double(i%size.width)+.5)/size.width-1)/snap.proj[0];
            double y=-(2*(double(i/size.width)+.5)/size.height-1)/snap.proj[5];
            double length=std::sqrt(1+x*x+y*y),cosine=(source.x*x+source.y*y-source.z)/length;
            double omega=validProjection?4.0/(size.width*size.height*snap.proj[0]*snap.proj[5]*length*length*length):0;
            if(validProjection)body+=halfFloat(b[4*i])*omega*cosine;
            optics+=(isolated?halfFloat(a[4*i]):std::max(0.f,halfFloat(a[4*i])-halfFloat(b[4*i])))*4.0/(size.width*size.height);
            for(int c=0;c<3;++c){double value=halfFloat(a[4*i+c]);require(std::isfinite(value)&&value>=0,"invalid solar HDR");maximum=std::max(maximum,value);
                if(isolated)spectralOptics[c]+=value*4.0/(size.width*size.height);
                uint8_t actual=d[4*i+(bgra?2-c:c)];image.write(reinterpret_cast<char*>(&actual),1);
                int expected=int(std::round(255*Radiometry::srgb(Radiometry::aces(value,taa.displayExposure))));
                if(std::abs(expected-int(actual))>error){error=std::abs(expected-int(actual));if(error>2)fprintf(stderr,"SOLAR_DISPLAY case=%d pixel=%zu c=%d HDR=%.10g actual=%d expected=%d\n",test,i,c,value,int(actual),expected);}}}
        std::ofstream raw("build/tests/solar-body-"+std::to_string(test)+".bin",std::ios::binary);raw.write(reinterpret_cast<const char*>(b),pixels*8);
        if(isolated){std::ofstream rawOptics("build/tests/solar-response-"+std::to_string(test)+".bin",std::ios::binary);rawOptics.write(reinterpret_cast<const char*>(a),pixels*8);}
        if(test==44||shapeOnly){std::ofstream rawFinal("build/tests/solar-final-"+std::to_string(test)+".bin",std::ios::binary);rawFinal.write(reinterpret_cast<const char*>(a),pixels*8);}
        printf("SOLAR_MEASURE case=%d body=%.10g optics=%.10g displayError=%d\n",test,body,optics,error);
        require(error<=2,"solar output transfer mismatch");
        double relative=sun.valid?std::abs(body/std::max(1e-20,sun.irradiance*(1-SolarOptics::scatteredFraction))-1):0;
        if(!shapeOnly){
        if(test==0){baselineBody=body;baselineOptics=optics;}
        if(test<7||test==12||test==13||test==14||(test>=18&&test<=26)||test==28)require(relative<.01,"solar irradiance differs from inverse-square reference");
        if(test==27)require(maximum<65504,"near-surface photosphere overflows FP16");
        if(isolated){double budget=SolarOptics::starFraction;
            for(const auto& kernel:SolarOptics::kernels)budget+=kernel.fraction*(cameraStage>=2?kernel.r:1);
            if(cameraStage>=3)for(const auto& ghost:SolarOptics::ghosts)budget+=ghost.fraction*ghost.r;
            double expected=sun.radiance*sun.solidAngle*snap.proj[0]*snap.proj[5]*budget;
            double opticalError=std::abs(optics/expected-1);
            if(test==45){expected*=.5;opticalError=std::abs(optics/expected-1);}
            double area=0,projectedRadius=std::abs(snap.proj[5])*std::tan(sun.angularRadius);
            for(const auto& kernel:SolarOptics::kernels){
                double sx=SolarOptics::kernelSigma(projectedRadius,std::abs(snap.proj[5])*kernel.sigmaX);
                double sy=SolarOptics::kernelSigma(projectedRadius,std::abs(snap.proj[5])*kernel.sigmaY);
                area+=std::min(4.,64*sx*sy/snap.aspect);}
            double starLong=SolarOptics::kernelSigma(projectedRadius,std::abs(snap.proj[5])*SolarOptics::starLongSigma);
            area+=std::min(4.,64*starLong*starLong/snap.aspect);
            // FP16 blend operations: one subnormal ULP per covered pixel
            // (covers truncation as well as nearest rounding), integrated over
            // each quad's NDC area, plus source quadrature. No brightness floor.
            double absoluteBound=area*std::ldexp(1.0,-24)+expected*.02;
            printf("SOLAR_OPTICAL_BUDGET case=%d expected=%.10g measured=%.10g relative=%.6g absoluteBound=%.10g\n",test,expected,optics,opticalError,absoluteBound);
            require(std::abs(optics-expected)<absoluteBound,"isolated optical energy exceeds FP16 absolute/relative camera budget");
            for(int c=1;c<3;++c){double channelBudget=SolarOptics::starFraction;
                for(const auto& k:SolarOptics::kernels)channelBudget+=k.fraction*(cameraStage>=2?(c==1?k.g:k.b):(c==1?.98:.95));
                if(cameraStage>=3)for(const auto& g:SolarOptics::ghosts)channelBudget+=g.fraction*(c==1?g.g:g.b);
                double target=sun.radiance*sun.solidAngle*snap.proj[0]*snap.proj[5]*channelBudget*SolarOptics::sourceSpectrum[c]*(test==45?.5:1);
                double bound=area*std::ldexp(1.0,-24)+target*.02;
                printf("SOLAR_RGB_BUDGET case=%d c=%d expected=%.10g measured=%.10g relative=%.6g\n",test,c,target,spectralOptics[c],std::abs(spectralOptics[c]/target-1));
                require(std::abs(spectralOptics[c]-target)<bound,"isolated RGB camera energy exceeds source-spectrum/FP16 budget");}}
        if(test==7)require(optics==0,"disabled camera optics adds energy");
        if(test==8||test==10||test==11||test==16||test==57||test>=61)require(maximum==0,"blocked/invalid solar source must be black");
        if(test==9){require(std::abs(body/baselineBody-.5)<.02,"partial eclipse solar visibility");require(optics>0&&optics<baselineOptics*.7,"partial eclipse flare visibility");}
        if(test==15){require(std::abs(body/baselineBody-.25)<.01,"solar attenuation body response");require(std::abs(optics/baselineOptics-.25)<.05,"solar attenuation optics response");}
        }
        printf("PASS SOLAR_GPU case=%d distance=%.5gAU bodyE=%.9g relative=%.5g lensNDCFlux=%.9g max=%.7g displayError=%d\n",test,snap.sunDistanceKm/SolarOptics::auKm,body,relative,optics,maximum,error);
        vmaDestroyBuffer(ctx.allocator,before.buffer,before.allocation);vmaDestroyBuffer(ctx.allocator,after.buffer,after.allocation);vmaDestroyBuffer(ctx.allocator,display.buffer,display.allocation);
        vkRequireSuccess(vkResetFences(ctx.device,1,&fence),"solar fence reset");
        VkSwapchainPresentFenceInfoEXT completion{VK_STRUCTURE_TYPE_SWAPCHAIN_PRESENT_FENCE_INFO_EXT};completion.swapchainCount=1;completion.pFences=&fence;
        VkPresentInfoKHR present{VK_STRUCTURE_TYPE_PRESENT_INFO_KHR};present.pNext=&completion;present.swapchainCount=1;present.pSwapchains=&ctx.swapchain;present.pImageIndices=&index;
        vkRequireSuccess(vkQueuePresentKHR(ctx.presentQueue,&present),"solar present");vkRequireSuccess(vkWaitForFences(ctx.device,1,&fence,VK_TRUE,5000000000ULL),"solar present wait");
        vkDestroyFence(ctx.device,fence,nullptr);taa.swap();require(g_vkValidationErrorCount==0,"solar Vulkan errors");
    }
    if(shapeOnly){printf("SOLAR_SHAPE: captures complete; finite values and output conversion checked; energy acceptance deferred\n");return;}
    SceneSnapshot bench{};bench.sunRadiusKm=696340;bench.sunDistanceKm=SolarOptics::auKm;bench.sunInFront=true;
    bench.sunScreen[0]=.4f;bench.sunScreen[1]=-.2f;bench.aspect=float(size.width)/size.height;bench.lightDir[2]=1;
    Mat4().toFloatArray(bench.view);Mat4::perspective(.8,bench.aspect,.1,10).toFloatArray(bench.proj);
    VkQueryPool timer;VkQueryPoolCreateInfo qi{VK_STRUCTURE_TYPE_QUERY_POOL_CREATE_INFO};qi.queryType=VK_QUERY_TYPE_TIMESTAMP;qi.queryCount=2;
    vkRequireSuccess(vkCreateQueryPool(ctx.device,&qi,nullptr,&timer),"solar timer");
    VkPhysicalDeviceProperties properties{};vkGetPhysicalDeviceProperties(ctx.physDevice,&properties);double milliseconds=0;
    for(int frame=0;frame<40;++frame){VkCommandBuffer cmd=beginSingleTimeCommands(ctx);vkCmdResetQueryPool(cmd,timer,0,2);
        vkCmdWriteTimestamp(cmd,VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT,timer,0);
        taa.beginGeometryPass(cmd,size);VkFrameRenderer::setViewportScissor(cmd,size);
        renderer.beginFrame(cmd,0,bench.view,bench.proj,bench.lightDir,bench.camPos,0);renderer.drawSolarDisk(cmd,bench,size);taa.endGeometryPass(cmd);
        if(cameraStage>=4)renderer.cameraBloom.render(ctx,cmd,taa,bench,0,size,renderer.scene.billboardQuadVbo);
        renderer.renderSolarOpticsAfterGeometry(ctx,cmd,taa,bench,0,size,false,7,cameraStage);
        vkCmdWriteTimestamp(cmd,VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT,timer,1);endSingleTimeCommands(ctx,cmd);
        uint64_t stamps[2]={};vkRequireSuccess(vkGetQueryPoolResults(ctx.device,timer,0,2,sizeof(stamps),stamps,sizeof(uint64_t),VK_QUERY_RESULT_64_BIT|VK_QUERY_RESULT_WAIT_BIT),"solar timer result");
        if(frame>=10)milliseconds+=(stamps[1]-stamps[0])*properties.limits.timestampPeriod/1e6;
    }
    printf("SOLAR_STEADY %ux%u 10 warmup + 30 frames meanGPU=%.3fms (disk, %soptics, HDR copy and barriers; no image readback; clocks not locked)\n",size.width,size.height,milliseconds/30,cameraStage>=4?"scene Bloom, ":"");
    vkDestroyQueryPool(ctx.device,timer,nullptr);require(g_vkValidationErrorCount==0,"solar benchmark Vulkan errors");
}
