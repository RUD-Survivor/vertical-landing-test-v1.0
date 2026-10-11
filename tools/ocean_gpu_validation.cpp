#define VK_USE_PLATFORM_WIN32_KHR
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#define GLFW_INCLUDE_VULKAN
#include <windows.h>
#include <GLFW/glfw3.h>
#include "render/vulkan/vk_renderer3d.h"
#include "math/math3d.h"
#include <cassert>
#include <chrono>
#include <fstream>

static float halfFloat(uint16_t h) {
    int exponent=(h>>10)&31, mantissa=h&1023;
    float x=exponent==0?std::ldexp(float(mantissa),-24):
        exponent==31?(mantissa?std::numeric_limits<float>::quiet_NaN():std::numeric_limits<float>::infinity()):
        std::ldexp(float(1024+mantissa),exponent-25);
    return h&32768?-x:x;
}
static void require(bool condition,const char* message) {if(!condition)throw std::runtime_error(message);}
// Stage probes validate daytime color before and after atmosphere/cloud composition.
struct HdrProbe {
    VkBuffer buffer=VK_NULL_HANDLE; VmaAllocation allocation{}; void* mapped=nullptr;
    void capture(VulkanContext& ctx,VkCommandBuffer cmd,VkImage image,VkExtent2D size) {
        VkBufferCreateInfo bi{VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO};
        bi.size=size_t(size.width)*size.height*8;bi.usage=VK_BUFFER_USAGE_TRANSFER_DST_BIT;
        VmaAllocationCreateInfo ac{};ac.usage=VMA_MEMORY_USAGE_GPU_TO_CPU;ac.flags=VMA_ALLOCATION_CREATE_MAPPED_BIT;
        VmaAllocationInfo info{};
        vkRequireSuccess(vmaCreateBuffer(ctx.allocator,&bi,&ac,&buffer,&allocation,&info),"HDR stage probe");mapped=info.pMappedData;
        transitionImage(cmd,image,VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL);
        VkBufferImageCopy copy{};copy.imageSubresource={VK_IMAGE_ASPECT_COLOR_BIT,0,0,1};copy.imageExtent={size.width,size.height,1};
        vkCmdCopyImageToBuffer(cmd,image,VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,buffer,1,&copy);
        transitionImage(cmd,image,VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
    }
    double report(VulkanContext& ctx,VkExtent2D size,const char* name,const char* stage) {
        vkRequireSuccess(vmaInvalidateAllocation(ctx.allocator,allocation,0,VK_WHOLE_SIZE),"HDR probe invalidate");
        auto data=static_cast<const uint16_t*>(mapped);size_t pixels=size_t(size.width)*size.height,black=0;
        double rgb[3]={},lum=0;
        std::ofstream ppm(std::string("build/tests/ocean-")+name+"-"+stage+".ppm",std::ios::binary);
        ppm<<"P6\n"<<size.width<<" "<<size.height<<"\n255\n";
        for(size_t i=0;i<pixels;++i) {
            float c[3];for(int k=0;k<3;++k){c[k]=halfFloat(data[i*4+k]);require(std::isfinite(c[k])&&c[k]>=0,"stage HDR invalid");rgb[k]+=c[k];
                unsigned char v=static_cast<unsigned char>(255*std::pow(c[k]/(1+c[k]),1/2.2f));ppm.write(reinterpret_cast<char*>(&v),1);}
            double y=.2126*c[0]+.7152*c[1]+.0722*c[2];lum+=y;if(y<1e-6)++black;
        }
        printf("COLOR %s %s RGBmean=(%.6f,%.6f,%.6f) luminance=%.6f black=%.2f%%\n",name,stage,rgb[0]/pixels,rgb[1]/pixels,rgb[2]/pixels,lum/pixels,100.0*black/pixels);
        vmaDestroyBuffer(ctx.allocator,buffer,allocation);buffer=VK_NULL_HANDLE;return lum/pixels;
    }
};
struct DisplayProbe {
    VkBuffer buffer=VK_NULL_HANDLE;VmaAllocation allocation{};void* mapped=nullptr;
    void capture(VulkanContext& ctx,VkCommandBuffer cmd,VkImage image,VkExtent2D size) {
        VkSurfaceCapabilitiesKHR caps{};
        vkRequireSuccess(vkGetPhysicalDeviceSurfaceCapabilitiesKHR(ctx.physDevice,ctx.surface,&caps),"surface readback capability");
        require((caps.supportedUsageFlags&VK_IMAGE_USAGE_TRANSFER_SRC_BIT)!=0,"presentation readback unsupported");
        VkBufferCreateInfo bi{VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO};bi.size=size_t(size.width)*size.height*4;bi.usage=VK_BUFFER_USAGE_TRANSFER_DST_BIT;
        VmaAllocationCreateInfo ac{};ac.usage=VMA_MEMORY_USAGE_GPU_TO_CPU;ac.flags=VMA_ALLOCATION_CREATE_MAPPED_BIT;
        VmaAllocationInfo info{};
        vkRequireSuccess(vmaCreateBuffer(ctx.allocator,&bi,&ac,&buffer,&allocation,&info),"display probe");mapped=info.pMappedData;
        transitionImage(cmd,image,VK_IMAGE_LAYOUT_PRESENT_SRC_KHR,VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL);
        VkBufferImageCopy copy{};copy.imageSubresource={VK_IMAGE_ASPECT_COLOR_BIT,0,0,1};copy.imageExtent={size.width,size.height,1};
        vkCmdCopyImageToBuffer(cmd,image,VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,buffer,1,&copy);
        transitionImage(cmd,image,VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,VK_IMAGE_LAYOUT_PRESENT_SRC_KHR);
    }
    void report(VulkanContext& ctx,VkExtent2D size,const char* name,HdrProbe* firstFrameReference,bool requireFullDaylight=true) {
        vkRequireSuccess(vmaInvalidateAllocation(ctx.allocator,allocation,0,VK_WHOLE_SIZE),"display invalidate");
        if(firstFrameReference)vkRequireSuccess(vmaInvalidateAllocation(ctx.allocator,firstFrameReference->allocation,0,VK_WHOLE_SIZE),"display reference invalidate");
        auto data=static_cast<const uint8_t*>(mapped);
        auto ref=firstFrameReference?static_cast<const uint16_t*>(firstFrameReference->mapped):nullptr;
        bool bgra=ctx.swapFormat==VK_FORMAT_B8G8R8A8_UNORM || ctx.swapFormat==VK_FORMAT_B8G8R8A8_SRGB;
        require(bgra || ctx.swapFormat==VK_FORMAT_R8G8B8A8_UNORM || ctx.swapFormat==VK_FORMAT_R8G8B8A8_SRGB,"display format");
        double sum[3]={};size_t pixels=size_t(size.width)*size.height,black=0;int maxError=0;
        std::ofstream ppm(std::string("build/tests/ocean-")+name+"-display.ppm",std::ios::binary);
        ppm<<"P6\n"<<size.width<<" "<<size.height<<"\n255\n";
        for(size_t i=0;i<pixels;++i) {
            int total=0;
            for(int k=0;k<3;++k) {
                uint8_t v=data[4*i+(bgra?2-k:k)];sum[k]+=v;total+=v;ppm.write(reinterpret_cast<char*>(&v),1);
                if(ref){double x=std::max(double(halfFloat(ref[4*i+k])),0.0);
                    double encoded=Radiometry::srgb(Radiometry::aces(x,Radiometry::defaultDisplayExposure));
                    maxError=std::max(maxError,std::abs(int(v)-int(std::round(encoded*255))));}
            }
            if(total<=3)++black;
        }
        printf("DISPLAY %s RGB8mean=(%.2f,%.2f,%.2f) black=%.2f%% sRGBmaxError=%d\n",name,sum[0]/pixels,sum[1]/pixels,sum[2]/pixels,100.0*black/pixels,ref?maxError:-1);
        if(requireFullDaylight)require(black<pixels/100,"daytime presentation contains black holes");
        if(ref)require(maxError<=2,"presentation does not match linear-to-sRGB reference");
        vmaDestroyBuffer(ctx.allocator,buffer,allocation);
    }
};
// Independent hemisphere quadrature of the GPU-baked sky. A zenith sample
// alone is not irradiance, especially with a bright horizon.
static void reportSkyIrradiance(VulkanContext& ctx,VkOceanSystem& ocean,const double grayReference[3]) {
    constexpr int nx=64,ny=32,nz=32;
    VkBuffer buffer;VmaAllocation allocation;VmaAllocationInfo info{};
    VkBufferCreateInfo bi{VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO};bi.size=nx*ny*nz*8;bi.usage=VK_BUFFER_USAGE_TRANSFER_DST_BIT;
    VmaAllocationCreateInfo ac{};ac.usage=VMA_MEMORY_USAGE_GPU_TO_CPU;ac.flags=VMA_ALLOCATION_CREATE_MAPPED_BIT;
    vkRequireSuccess(vmaCreateBuffer(ctx.allocator,&bi,&ac,&buffer,&allocation,&info),"sky radiometry readback");
    VkCommandBuffer cmd=beginSingleTimeCommands(ctx);
    transitionImage(cmd,ocean.skyLut.image,VK_IMAGE_LAYOUT_GENERAL,VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL);
    VkBufferImageCopy copy{};copy.imageSubresource={VK_IMAGE_ASPECT_COLOR_BIT,0,0,1};copy.imageExtent={nx,ny,nz};
    vkCmdCopyImageToBuffer(cmd,ocean.skyLut.image,VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,buffer,1,&copy);
    transitionImage(cmd,ocean.skyLut.image,VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,VK_IMAGE_LAYOUT_GENERAL);
    endSingleTimeCommands(ctx,cmd);
    vkRequireSuccess(vmaInvalidateAllocation(ctx.allocator,allocation,0,VK_WHOLE_SIZE),"sky radiometry invalidate");
    auto data=static_cast<const uint16_t*>(info.pMappedData);auto& u=*static_cast<const OceanUniforms*>(ocean.mapped[0]);
    double mus=u.sunRadius[1],irradiance[3]={},zenith[3]={};
    auto sample=[&](double mu,double phi,int channel) {
        double p[3]={std::sqrt(mu)*(nx-1),std::clamp((mus+.2)/1.2,0.0,1.0)*(ny-1),(std::cos(phi)*.5+.5)*(nz-1)};
        int a[3]={int(p[0]),int(p[1]),int(p[2])};double f[3]={p[0]-a[0],p[1]-a[1],p[2]-a[2]},result=0;
        for(int z=0;z<2;++z)for(int y=0;y<2;++y)for(int x=0;x<2;++x){int ix=std::min(a[0]+x,nx-1),iy=std::min(a[1]+y,ny-1),iz=std::min(a[2]+z,nz-1);
            double weight=(x?f[0]:1-f[0])*(y?f[1]:1-f[1])*(z?f[2]:1-f[2]);
            result+=weight*halfFloat(data[((iz*ny+iy)*nx+ix)*4+channel]);}
        return result*u.params[0];
    };
    for(int c=0;c<3;++c){zenith[c]=sample(1,0,c);
        for(int i=0;i<32;++i)for(int j=0;j<64;++j){double mu=(i+.5)/32,phi=(j+.5)*2*OceanModel::pi/64;
            irradiance[c]+=sample(mu,phi,c)*mu*2*OceanModel::pi/(32*64);}}
    printf("SKY_IRRADIANCE RGB=(%.8f,%.8f,%.8f) zenithPi=(%.8f,%.8f,%.8f) ratio=(%.3f,%.3f,%.3f)\n",
        irradiance[0],irradiance[1],irradiance[2],zenith[0]*OceanModel::pi,zenith[1]*OceanModel::pi,zenith[2]*OceanModel::pi,
        irradiance[0]/(zenith[0]*OceanModel::pi),irradiance[1]/(zenith[1]*OceanModel::pi),irradiance[2]/(zenith[2]*OceanModel::pi));
    double maxQuadratureError=0;
    for(int c=0;c<3;++c){double refined=0;
        for(int i=0;i<128;++i)for(int j=0;j<256;++j){double mu=(i+.5)/128,phi=(j+.5)*2*OceanModel::pi/256;
            refined+=sample(mu,phi,c)*mu*2*OceanModel::pi/(128*256);}
        double error=std::abs(refined-irradiance[c])/std::max(refined,1e-8);
        maxQuadratureError=std::max(maxQuadratureError,error);require(error<.02,"sky cosine quadrature fails refinement check");}
    printf("PASS SKY_QUADRATURE 32x64 versus 128x256 maxRelativeError=%.6f\n",maxQuadratureError);
    double corrected[3],grayY=0;
    const double weights[3]={.2126,.7152,.0722};
    for(int c=0;c<3;++c){corrected[c]=grayReference[c];grayY+=weights[c]*corrected[c];}
    double referenceExposure=.18/grayY;
    printf("GRAY_REFERENCE hemisphericRGB=(%.8f,%.8f,%.8f) Y=%.8f exposureToLinear18=%.4f EV=%.4f\n",
        corrected[0],corrected[1],corrected[2],grayY,referenceExposure,std::log2(referenceExposure));
    require(std::isfinite(referenceExposure)&&grayY>0,"invalid gray reference");
    double displayY=0;for(int c=0;c<3;++c)displayY+=weights[c]*Radiometry::aces(grayReference[c],Radiometry::defaultDisplayExposure);
    printf("GRAY_DISPLAY linearY=%.8f target=0.18 exposure=%.3f\n",displayY,Radiometry::defaultDisplayExposure);
    require(std::abs(displayY-.18)<.03,"daylight gray calibration outside tolerance");
    double expected[2][32][3]={};
    for(int e=0;e<32;++e){mus=-.2+1.2*e/31;
        for(int i=0;i<32;++i)for(int j=0;j<64;++j){double mu=(i+.5)/32,phi=(j+.5)*2*OceanModel::pi/64;
            double entry=1-(.020373+.979627*std::pow(1-mu,5));
            for(int c=0;c<3;++c){double value=sample(mu,phi,c)/u.params[0]*mu*2*OceanModel::pi/(32*64);
                expected[0][e][c]+=value;expected[1][e][c]+=value*entry;}}}
    VkCommandBuffer icmd=beginSingleTimeCommands(ctx);
    transitionImage(icmd,ocean.irradianceLut.image,VK_IMAGE_LAYOUT_GENERAL,VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL);
    copy.imageExtent={32,2,1};
    vkCmdCopyImageToBuffer(icmd,ocean.irradianceLut.image,VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,buffer,1,&copy);
    transitionImage(icmd,ocean.irradianceLut.image,VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,VK_IMAGE_LAYOUT_GENERAL);
    endSingleTimeCommands(ctx,icmd);
    vkRequireSuccess(vmaInvalidateAllocation(ctx.allocator,allocation,0,VK_WHOLE_SIZE),"irradiance invalidate");
    auto lut=static_cast<const uint16_t*>(info.pMappedData);
    double maxRelative=0;
    for(int row=0;row<2;++row)for(int e=0;e<32;++e)for(int c=0;c<3;++c){double actual=halfFloat(lut[((row*32+e)*4)+c]);
        require(std::isfinite(actual)&&actual>=0,"irradiance finite/nonnegative");
        double relative=std::abs(actual-expected[row][e][c])/std::max(1e-5,expected[row][e][c]);
        maxRelative=std::max(maxRelative,relative);require(relative<.005,"GPU irradiance differs from hemisphere integral");
        if(row==1)require(actual<=halfFloat(lut[e*4+c])+1e-6,"water entry creates energy");}
    printf("PASS SKY_IRRADIANCE 32 solar elevations, both rows; maxRelativeError=%.6f\n",maxRelative);
    vmaDestroyBuffer(ctx.allocator,buffer,allocation);
}
#include "ocean_orbit_validation.h"
#include "sky_gpu_validation.h"
#include "solar_gpu_validation.h"
#include "camera_bloom_validation.h"
int main(int argc,char** argv) {
    int skyStage=argc>1 && std::string(argv[1]).find("--sky-stage")==0?std::atoi(argv[1]+11):0;
    bool cloudResponse=argc>1 && std::string(argv[1])=="--cloud-response";
    bool cloudDynamic=argc>1 && std::string(argv[1])=="--cloud-dynamic";
    bool cloudOnly=cloudResponse || cloudDynamic || (argc>1 && (std::string(argv[1])=="--cloud-limb" || std::string(argv[1])=="--cloud-limb-baseline" || std::string(argv[1])=="--cloud-limb-odd"));
    int solarStage=argc>1&&std::string(argv[1]).find("--solar-stage")==0?std::atoi(argv[1]+13):3;
    bool solarShape=argc>1&&std::string(argv[1])=="--solar-shape";
    if(solarShape)solarStage=10;
    bool solarOnly=solarShape||(argc>1 && (std::string(argv[1])=="--solar-optics"||std::string(argv[1]).find("--solar-stage")==0));
    bool bloomOnly=argc>1&&std::string(argv[1])=="--camera-bloom";
    bool cameraChain=argc>1&&(std::string(argv[1])=="--camera-chain"||std::string(argv[1])=="--orbit-camera-chain");
    bool orbitOnly=bloomOnly || solarOnly || skyStage>0 || cloudOnly || (argc>1 && (std::string(argv[1])=="--orbit" || std::string(argv[1])=="--orbit-camera-chain" || std::string(argv[1])=="--orbit-baseline"));
    if(!glfwInit())return 1;
    glfwWindowHint(GLFW_CLIENT_API,GLFW_NO_API);glfwWindowHint(GLFW_VISIBLE,GLFW_FALSE);
    GLFWwindow* window=glfwCreateWindow(256,144,"Ocean GPU validation",nullptr,nullptr);
    VulkanContext ctx;VkDescriptorManager desc;VkRenderer3D renderer;VkTAA taa;
    auto& scene=renderer.scene;auto& ocean=renderer.oceanSystem;
    VkQueryPool queries=VK_NULL_HANDLE;
    try {
        require(window && ctx.initCore(window) && ctx.initSwapchain(window),"Vulkan init");
        require(desc.init(ctx),"descriptors");
        require(renderer.init(ctx,desc,VK_FORMAT_R16G16B16A16_SFLOAT,VK_FORMAT_D32_SFLOAT),"renderer init");
        VkExtent2D size{256,144};
        require(taa.init(ctx,size,ctx.swapFormat,"src/render/shaders/spirv/taa.vert.spv","src/render/shaders/spirv/taa.frag.spv"),"TAA init");
        VkQueryPoolCreateInfo qi{VK_STRUCTURE_TYPE_QUERY_POOL_CREATE_INFO};qi.queryType=VK_QUERY_TYPE_TIMESTAMP;qi.queryCount=2;
        vkRequireSuccess(vkCreateQueryPool(ctx.device,&qi,nullptr,&queries),"query pool");
        AtmoTuneParams tune;
        if(bloomOnly)validateCameraBloom(ctx,renderer,taa);
        else if(solarOnly)validateSolar(ctx,renderer,taa,window,solarStage,solarShape);
        else if(skyStage)validateSky(ctx,renderer,taa,window,skyStage);
        else if(orbitOnly)validateOrbitOcean(ctx,renderer,taa,window,std::string(argv[1])!="--orbit-baseline" && std::string(argv[1])!="--cloud-limb-baseline",cloudOnly,std::string(argv[1])=="--cloud-limb-odd",cameraChain,cloudResponse,cloudDynamic);
        const char* names[]={"deep-day","deep-night","dry","shallow","orbit","disabled","resize","shifted-origin",
            "shallow-lake","coast","sunset","underwater-surface","720p","full-chain","full-chain-history",
            "full-chain-steady1","full-chain-steady2","zero-scattering","south-pole","long-clock","cloud-resize","cloud-resume",
            "radiometry-beauty","radiometry-reflection","radiometry-body","radiometry-direct","radiometry-foam",
            "radiometry-sky","radiometry-zenith","radiometry-below","radiometry-fresnel","radiometry-noSSR",
            "radiometry-flat","radiometry-gray","radiometry-cloudvis","radiometry-shadow","zero-cloud-build","radiometry-clearvis","radiometry-visible-sky",
            "clear-day-display","solar-double","solar-zero","solar-ten","gray-display"};
        const int debugViews[]={0,1,2,3,4,5,6,7,8,9,10,11,12,0,0,12,13,0,0,0,0,11};
        double dayMean=0;
        double beautyY=0,componentY=0;
        double grayReference[3]={};
        double clearY=0;
        for(int test=0;test<(orbitOnly?0:44);++test) {
            bool diagnostic=test>=22;
            ocean.debugView=diagnostic?debugViews[test-22]:0;
            bool fullChain=(test>=13 && test<=16) || test==20 || test==21 || test==36 || test==39 || test==43;
            if(test==36){renderer.cloudSystem.tuneParams.density=0;renderer.cloudSystem.tuneParams.hiCloudEnabled=false;}
            int slot=fullChain?(test-13)%2:0;
            if(test==17){tune.rayleighMul=0;tune.mieMul=0;tune.ozoneMul=0;}
            if(test==20) {
                tune=AtmoTuneParams{};
                require(renderer.cloudSystem.recreate(ctx,size),"cloud history resize");
            }
            if(test==6 || test==12){size=test==6?VkExtent2D{384,216}:VkExtent2D{1280,720};require(taa.recreate(ctx,size,ctx.swapFormat,"src/render/shaders/spirv/taa.vert.spv","src/render/shaders/spirv/taa.frag.spv"),"resize");}
            const float bed=test==2?.8f:test==3?.449f:test==8?.5f:.30f;
            float tectonic[8*4*4],hydro[8*4*4];
            for(int i=0;i<32;++i){tectonic[4*i]=test==9?((i%8)<4?.3f:.8f):bed;tectonic[4*i+1]=tectonic[4*i+2]=0;tectonic[4*i+3]=1;
                hydro[4*i]=test==8?.5001f:tectonic[4*i];hydro[4*i+1]=hydro[4*i+2]=0;hydro[4*i+3]=-1;}
            // Each previous submit/readback has completed. Retire synthetic maps
            // before replacing them (upload is an initial-upload API).
            if(test<14) {
                scene.terrainTectonicTex.destroy(ctx);scene.terrainHydroTex.destroy(ctx);
                require(scene.uploadTerrainTextures(ctx,tectonic,8,4,hydro,8,4),"maps");
            }
            if(test==13) {
                glfwSetWindowSize(window,int(size.width),int(size.height));glfwPollEvents();
                require(ctx.recreateSwapchain(window),"present resize");
                require(renderer.cloudSystem.init(ctx,size,scene.cloudNoiseTex.view,scene.cloudDetailTex.view,
                    scene.cloudWeatherTex.view,scene.cloudCurlTex.view,scene.cloudProfileLutTex.view,scene.cloudHiWeatherTex.view),"cloud init");
            }
            SceneSnapshot snap{};
            if(test==40)snap.sunIntensity=2;
            if(test==41)snap.sunIntensity=0;
            if(test==42)snap.sunIntensity=10;
            const float offset=test==7?100.f:0.f, altitude=(test==4 || test==9)?400.f:test==8?float(OceanModel::bedKm(.5001,25))+.02f:test==11?-.01f:.02f;
            float cameraX=offset+(test==16?.0001f:0.f); // 10 cm camera motion with valid TAA history
            snap.camPos[0]=cameraX;snap.camPos[1]=altitude;
            snap.time=test==19?1e7+.016:test>=39?3.0:fullChain?3.0+.05*(test-13):3.0;
            snap.frameIndex=fullChain?test-13:0;
            snap.sunWorldPos[0]=offset+100000;snap.sunWorldPos[1]=test==1?-100000:100000;
            snap.lightDir[0]=.7071f;snap.lightDir[1]=test==1?-.7071f:.7071f;
            if(test==10){snap.sunWorldPos[1]=-6371+300; snap.lightDir[1]=.003f;}
            snap.aspect=float(size.width)/size.height;
            Mat4 view=Mat4::lookAt(Vec3(cameraX,altitude,0),Vec3(cameraX,(test==4 || test==9)?0:test==11?.04:altitude-.06,(test==4 || test==9)?0:.1),Vec3(0,0,1));
            Mat4 proj=Mat4::perspective(1.0,double(size.width)/size.height,.001,30000);
            view.toFloatArray(snap.view);proj.toFloatArray(snap.proj);
            CelestialDraw earth{};earth.bodyIdx=earth.atmoIdx=3;earth.radius=6371;
            earth.showClouds=fullChain;
            earth.center[0]=offset;earth.center[1]=-6371;
            Mat4 model=Mat4::translate(Vec3(offset,-6371,0))*Mat4::scale(Vec3(6371,6371,6371));model.toFloatArray(earth.model);
            if(test==18){earth.model[5]=-6371;earth.model[10]=-6371;} // proper 180 degree rotation around X
            snap.celestials.push_back(earth);
            TerrainPatchDraw tp{};memcpy(tp.model,earth.model,64);tp.planetRadius=6371;tp.maxElev=25;
            tp.nodeCenter[1]=1;float span=(test==4 || test==9)?.6f:.0003f;tp.nodeSideA[0]=span;tp.nodeSideB[2]=span;tp.nodeLevel=12;
            if(test==18)tp.nodeCenter[1]=-1;
            tp.planetCenterRel[0]=earth.center[0]-snap.camPos[0];tp.planetCenterRel[1]=earth.center[1]-snap.camPos[1];
            if(test!=5)snap.terrainPatches.push_back(tp);
            taa.updateMatrices(snap.view,snap.proj,snap.camPos,earth.center,earth.radius);taa.uploadMatrices(ctx,slot);
            uint32_t swapIndex=0;VkFence acquireFence=VK_NULL_HANDLE;
            if(fullChain) {
                VkFenceCreateInfo fci{VK_STRUCTURE_TYPE_FENCE_CREATE_INFO};
                vkRequireSuccess(vkCreateFence(ctx.device,&fci,nullptr,&acquireFence),"acquire fence");
                vkRequireSuccess(vkAcquireNextImageKHR(ctx.device,ctx.swapchain,5000000000ULL,VK_NULL_HANDLE,acquireFence,&swapIndex),"acquire for resolve");
                vkRequireSuccess(vkWaitForFences(ctx.device,1,&acquireFence,VK_TRUE,5000000000ULL),"acquire wait");
            }
            VkCommandBuffer cmd=beginSingleTimeCommands(ctx);
            HdrProbe probes[3];
            DisplayProbe display;
            taa.beginGeometryPass(cmd,size);
            VkViewport vp{0,0,float(size.width),float(size.height),0,1};VkRect2D rect{{0,0},size};
            vkCmdSetViewport(cmd,0,1,&vp);vkCmdSetScissor(cmd,0,1,&rect);
            TerrainPushConstants pc{};memcpy(pc.model,tp.model,64);pc.planetRadius=6371;pc.maxElevation=25;pc.nodeLevel=12;
            memcpy(pc.nodePos,tp.nodeCenter,12);memcpy(pc.nodeSide,tp.nodeSideA,12);memcpy(pc.nodeUp,tp.nodeSideB,12);
            TerrainDataUBO td{};
            if(fullChain) renderer.render(cmd,slot,snap,size);
            else {
                scene.beginFrame(cmd,slot,snap.view,snap.proj,snap.lightDir,snap.camPos,snap.time,snap.sunIntensity);
                scene.updateTerrainCloudShadow(VK_NULL_HANDLE,VK_NULL_HANDLE,VK_NULL_HANDLE,0);
                if(test>=34) {
                    auto& shadow=renderer.cloudSystem.cloudShadow;
                    require(shadow.uploadTerrainData(slot),"radiometry shadow unavailable");
                    scene.updateTerrainCloudShadow(shadow.atlasView(),shadow.atlasSampler(),shadow.terrainUbo(slot),sizeof(GpuCloudShadowData));
                }
                scene.drawTerrainPatch(cmd,scene.terrainPatchVbo,scene.terrainPatchIbo,scene.terrainPatchIcount,pc,td,scene.terrainGlobalTexSet);
            }
            taa.endGeometryPass(cmd);
            vkCmdResetQueryPool(cmd,queries,0,2);
            vkCmdWriteTimestamp2(cmd,VK_PIPELINE_STAGE_2_ALL_COMMANDS_BIT,queries,0);
            float previousSkyKey[18];memcpy(previousSkyKey,ocean.skyKey,sizeof(previousSkyKey));
            ocean.render(ctx,cmd,taa,scene,snap,slot,size,tune);
            if(test>=40)require(memcmp(previousSkyKey,ocean.skyKey,sizeof(previousSkyKey))==0,"solar intensity should not invalidate unit-sun sky cache");
            vkCmdWriteTimestamp2(cmd,VK_PIPELINE_STAGE_2_ALL_COMMANDS_BIT,queries,1);
            if(fullChain) {
                probes[0].capture(ctx,cmd,taa.colorImages[taa.currentIdx],size);
                renderer.renderAtmoAfterGeometry(ctx,cmd,taa,snap,slot);
                probes[1].capture(ctx,cmd,taa.colorImages[taa.currentIdx],size);
                if(test==21)renderer.cloudSystem.renderPhase0(ctx,cmd,taa,snap,slot,false);
                renderer.renderFlowerCloudAfterGeometry(ctx,cmd,taa,snap,slot);
                if(cameraChain){renderer.cameraBloom.render(ctx,cmd,taa,snap,slot,size,scene.billboardQuadVbo);
                    renderer.renderSolarOpticsAfterGeometry(ctx,cmd,taa,snap,slot,size);}
                probes[2].capture(ctx,cmd,taa.colorImages[taa.currentIdx],size);
                taa.beginResolvePass(cmd,ctx.swapImages[swapIndex],ctx.swapImageViews[swapIndex],size);
                vkCmdSetViewport(cmd,0,1,&vp);vkCmdSetScissor(cmd,0,1,&rect);
                taa.drawResolve(cmd,(test==13 || test>=39)?1.f:.1f);
                taa.endResolvePass(cmd,ctx.swapImages[swapIndex]);
                display.capture(ctx,cmd,ctx.swapImages[swapIndex],size);
            }
            endSingleTimeCommands(ctx,cmd);
            if(fullChain) {
                display.report(ctx,size,names[test],(test==13 || test>=39)?&probes[2]:nullptr);
                double water=probes[0].report(ctx,size,names[test],"water");
                if(test==39)clearY=water;
                probes[1].report(ctx,size,names[test],"atmosphere");
                double composite=probes[2].report(ctx,size,names[test],"clouds");
                // All camera rays hit water within the sub-cloud region in this scene.
                // Clouds behind opaque water cannot erase its daytime radiance.
                require(composite>water*.5,"daytime cloud composition erased water color");
            }
            if(fullChain) {
                vkResetFences(ctx.device,1,&acquireFence);
                VkSwapchainPresentFenceInfoEXT fi{VK_STRUCTURE_TYPE_SWAPCHAIN_PRESENT_FENCE_INFO_EXT};fi.swapchainCount=1;fi.pFences=&acquireFence;
                VkPresentInfoKHR present{VK_STRUCTURE_TYPE_PRESENT_INFO_KHR};present.pNext=&fi;present.swapchainCount=1;present.pSwapchains=&ctx.swapchain;present.pImageIndices=&swapIndex;
                vkRequireSuccess(vkQueuePresentKHR(ctx.presentQueue,&present),"present water resolve");
                vkRequireSuccess(vkWaitForFences(ctx.device,1,&acquireFence,VK_TRUE,5000000000ULL),"present complete");
                vkDestroyFence(ctx.device,acquireFence,nullptr);
            }
            uint64_t times[2]={};vkRequireSuccess(vkGetQueryPoolResults(ctx.device,queries,0,2,sizeof(times),times,sizeof(uint64_t),VK_QUERY_RESULT_64_BIT|VK_QUERY_RESULT_WAIT_BIT),"timestamps");
            // Read back actual GPU HDR and reactive coverage. CPU tests alone cannot catch binding/layout failures.
            size_t pixels=size_t(size.width)*size.height, colorBytes=pixels*8;
            VkBuffer staging;VmaAllocation alloc;VmaAllocationInfo info{};
            VkBufferCreateInfo bi{VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO};bi.size=colorBytes+pixels+pixels*8;bi.usage=VK_BUFFER_USAGE_TRANSFER_DST_BIT;
            VmaAllocationCreateInfo ac{};ac.usage=VMA_MEMORY_USAGE_GPU_TO_CPU;ac.flags=VMA_ALLOCATION_CREATE_MAPPED_BIT;
            vkRequireSuccess(vmaCreateBuffer(ctx.allocator,&bi,&ac,&staging,&alloc,&info),"readback buffer");
            cmd=beginSingleTimeCommands(ctx);
            transitionImage(cmd,taa.colorImages[taa.currentIdx],VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL);
            transitionImage(cmd,ocean.reactive,VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL);
            VkBufferImageCopy region{};region.imageSubresource={VK_IMAGE_ASPECT_COLOR_BIT,0,0,1};region.imageExtent={size.width,size.height,1};
            vkCmdCopyImageToBuffer(cmd,taa.colorImages[taa.currentIdx],VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,staging,1,&region);
            region.bufferOffset=colorBytes;vkCmdCopyImageToBuffer(cmd,ocean.reactive,VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,staging,1,&region);
            // D32 surface depth must be in front of the opaque bed wherever water drew.
            region.imageSubresource.aspectMask=VK_IMAGE_ASPECT_DEPTH_BIT;
            region.bufferOffset=colorBytes+pixels;
            VkOceanSystem::depthBarrier(cmd,taa.depthImage,VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL);
            vkCmdCopyImageToBuffer(cmd,taa.depthImage,VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,staging,1,&region);
            VkOceanSystem::depthBarrier(cmd,taa.depthImage,VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
            region.bufferOffset=colorBytes+pixels+pixels*4;
            VkOceanSystem::depthBarrier(cmd,ocean.bedDepth,VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL);
            vkCmdCopyImageToBuffer(cmd,ocean.bedDepth,VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,staging,1,&region);
            VkOceanSystem::depthBarrier(cmd,ocean.bedDepth,VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
            transitionImage(cmd,taa.colorImages[taa.currentIdx],VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
            transitionImage(cmd,ocean.reactive,VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
            endSingleTimeCommands(ctx,cmd);
            vkRequireSuccess(vmaInvalidateAllocation(ctx.allocator,alloc,0,VK_WHOLE_SIZE),"invalidate readback");
            auto color=static_cast<const uint16_t*>(info.pMappedData);auto mask=static_cast<const uint8_t*>(info.pMappedData)+colorBytes;
            auto surfaceDepth=reinterpret_cast<const float*>(mask+pixels);
            auto opaqueDepth=surfaceDepth+pixels;
            double sum=0,maxValue=0,rgbSum[3]={};size_t wet=0;
            for(size_t i=0;i<pixels;++i){
                require(std::isfinite(surfaceDepth[i]) && surfaceDepth[i]>=0 && surfaceDepth[i]<=1,"invalid surface depth");
                if(mask[i]){++wet;require(surfaceDepth[i]<=opaqueDepth[i]+1e-6f,"water must occlude the bed");}
                for(int c=0;c<3;++c){float value=halfFloat(color[i*4+c]);require(std::isfinite(value)&&value>=0,"non-finite/negative GPU HDR");sum+=value;rgbSum[c]+=value;maxValue=std::max(maxValue,double(value));}}
            VkPhysicalDeviceProperties props{};vkGetPhysicalDeviceProperties(ctx.physDevice,&props);
            printf("PASS GPU %-14s %ux%u water=%zu HDRmean=%.5f max=%.3f waterGPU=%.3f ms\n",names[test],size.width,size.height,wet,sum/(pixels*3),maxValue,(times[1]-times[0])*props.limits.timestampPeriod/1e6);
            if(diagnostic) {
                double y=(rgbSum[0]*.2126+rgbSum[1]*.7152+rgbSum[2]*.0722)/pixels;
                printf("RADIOMETRY %s RGB=(%.8f,%.8f,%.8f) Y=%.8f\n",names[test],rgbSum[0]/pixels,rgbSum[1]/pixels,rgbSum[2]/pixels,y);
                if(test==22)beautyY=y;
                if(test>=23 && test<=26)componentY+=y;
                if(test==26)require(std::abs(beautyY-componentY)<beautyY*.005,"water radiometry decomposition mismatch");
                if(test==37)require(y>.999,"zero cloud extinction must transmit sunlight");
                if(test==33)for(int c=0;c<3;++c)grayReference[c]=rgbSum[c]/pixels;
                if(test==40 || test==42) {
                    double factor=test==40?2:10;
                    require(std::abs(y-clearY*factor)<clearY*factor*.005,"water solar radiance scaling is nonlinear");
                    printf("PASS SOLAR_SCALE factor=%.0f measured=%.6f\n",factor,y/clearY);
                }
                if(test==41)require(y<1e-7,"zero sunlight must produce zero water radiance");
            }
            if(test==0)dayMean=sum/(pixels*3);
            if(test==2 || test==5)require(wet==0,"dry/disabled mask must be empty");
            else {
                require(wet>pixels/20,"water pass must actually draw pixels");
                if(test==1)require(sum/(pixels*3)<dayMean*.1,"night must suppress direct sunlight");
                else if(!diagnostic)require(maxValue>0.001,"daytime water must be lit");
                if(test==9)require(wet<pixels,"coast must preserve dry pixels");
            }
            // PPM preview is tone mapped for inspection; tests use untouched HDR above.
            std::string path=std::string("build/tests/ocean-")+names[test]+".ppm";
            std::ofstream image(path,std::ios::binary);image<<"P6\n"<<size.width<<" "<<size.height<<"\n255\n";
            for(size_t i=0;i<pixels;++i)for(int c=0;c<3;++c){float v=halfFloat(color[i*4+c]);unsigned char x=static_cast<unsigned char>(255*std::pow(v/(1+v),1/2.2f));image.write(reinterpret_cast<char*>(&x),1);}
            vmaDestroyBuffer(ctx.allocator,staging,alloc);
            require(g_vkValidationErrorCount==0,"Vulkan validation errors");
            if(fullChain)taa.swap();
        }
        if(!orbitOnly)reportSkyIrradiance(ctx,ocean,grayReference);
        vkDestroyQueryPool(ctx.device,queries,nullptr);renderer.shutdown(ctx);taa.shutdown(ctx);desc.shutdown(ctx);ctx.shutdown();
        require(g_vkValidationErrorCount==0,"shutdown validation errors");
        glfwDestroyWindow(window);glfwTerminate();
        if(cloudResponse)puts("PASS: cloud response assertions and direct empty-medium identity; zero Vulkan errors.");
        else puts("PASS: GPU optical water, masks, HDR, resize, night, poles, orbit, floating origin; zero validation errors");
        return 0;
    } catch(const std::exception& error) {fprintf(stderr,"FAIL: %s\n",error.what());return 9;}
}
