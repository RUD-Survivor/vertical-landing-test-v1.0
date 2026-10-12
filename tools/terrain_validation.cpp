#define VK_USE_PLATFORM_WIN32_KHR
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#define GLFW_INCLUDE_VULKAN
#include <windows.h>
#include <GLFW/glfw3.h>
#include "render/vulkan/vk_renderer3d.h"
#include "render/terrain_system.h"
#include <fstream>
#include <random>
#include <chrono>
#include <stdexcept>
#include <iomanip>

static std::ofstream report("build/tests/terrain-validation.txt");
static void check(bool ok,const char* why){if(!ok)throw std::runtime_error(why);}
static void log(const std::string& s){printf("%s\n",s.c_str());report<<s<<'\n';report.flush();}
static float halfFloat(uint16_t h){
    int e=(h>>10)&31,m=h&1023;
    float v=e==0?std::ldexp(float(m),-24):e==31?std::numeric_limits<float>::infinity():std::ldexp(float(1024+m),e-25);
    return h&32768?-v:v;
}
static std::vector<float> pack(const std::vector<float>& a){
    std::vector<float> r(a.size()*4,0);for(size_t i=0;i<a.size();++i){r[i*4]=a[i];r[i*4+3]=1;}return r;
}
static void upload(VulkanContext& ctx,VkSceneSystem& scene,Terrain::QuadtreeTerrain& t){
    scene.terrainTectonicTex.destroy(ctx);scene.terrainHydroTex.destroy(ctx);
    auto p=pack(t.sim->gridHeight),h=pack(t.hydroSim->data.filledHeight);
    for(size_t i=0;i<t.hydroSim->data.filledHeight.size();++i){
        h[4*i+1]=t.hydroSurfaceBed[i];h[4*i+2]=float(t.hydroSim->data.strahler[i]);h[4*i+3]=float(t.hydroSim->data.flowDir[i]);
    }
    check(scene.uploadTerrainTextures(ctx,p.data(),t.sim->width,t.sim->height,h.data(),t.hydroSim->width,t.hydroSim->height),"height textures");
}
struct Buffer {
    VkBuffer handle{};VmaAllocation allocation{};void* mapped{};
    void init(VulkanContext& ctx,size_t bytes,VkBufferUsageFlags usage){
        VkBufferCreateInfo bi{VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO};bi.size=bytes;bi.usage=usage;
        VmaAllocationCreateInfo ai{};ai.usage=(usage&VK_BUFFER_USAGE_TRANSFER_DST_BIT)?VMA_MEMORY_USAGE_GPU_TO_CPU:VMA_MEMORY_USAGE_CPU_TO_GPU;
        ai.flags=VMA_ALLOCATION_CREATE_MAPPED_BIT|VMA_ALLOCATION_CREATE_HOST_ACCESS_RANDOM_BIT;
        VmaAllocationInfo info{};vkRequireSuccess(vmaCreateBuffer(ctx.allocator,&bi,&ai,&handle,&allocation,&info),"terrain buffer");mapped=info.pMappedData;
    }
    void destroy(VulkanContext& ctx){vmaDestroyBuffer(ctx.allocator,handle,allocation);}
};

static void numerical(VulkanContext& ctx,VkSceneSystem& scene,Terrain::QuadtreeTerrain& terrain,const char* name){
    terrain.refreshBounds();
    upload(ctx,scene,terrain);
    std::vector<float> input;
    auto add=[&](Vec3 p){p=p.normalized();input.insert(input.end(),{float(p.x),float(p.y),float(p.z),0});};
    // Fixed random seed plus seam/pole/launchpad/cube edge samples.
    std::mt19937 rng(7919);std::uniform_real_distribution<double> d(-1,1);
    for(int i=0;i<4096;++i)add(Vec3(d(rng),d(rng),d(rng)));
    for(int i=0;i<129;++i){double v=-1.0+i/64.0;add(Vec3(1,v,1));add(Vec3(-1,v,1));add(Vec3(v,1,1));}
    for(double y:{-.999,-.5,0.,.5,.999}){add(Vec3(-1,y,1e-7));add(Vec3(-1,y,-1e-7));}
    add(Vec3(0,1,0));add(Vec3(0,-1,0));add(Vec3(.1436,.478,.866));
    for(float morph:{0.0f,0.5f,1.0f})for(int i=0;i<65;++i){
        float x=float(i%33)/32-.5f,z=float((i*7)%33)/32-.5f;
        input.insert(input.end(),{x,i%7==0?-1.0f:0.0f,z,10.0f+morph});
    }
    for(int phase=0;phase<3;++phase)for(int difference=1;difference<=3;++difference)for(int edge=0;edge<4;++edge)for(int i=1;i<32;++i){
        float t=float(i)/32-.5f,x=edge<2?(edge==0?-.5f:.5f):t,z=edge>=2?(edge==2?-.5f:.5f):t;
        float code=float(20+edge+4*(difference-1)+16*phase);
        input.insert(input.end(),{x,0,z,code});
    }
    size_t count=input.size()/4;
    Buffer in,out;in.init(ctx,input.size()*4,VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
    out.init(ctx,count*32,VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);memcpy(in.mapped,input.data(),input.size()*4);
    vmaFlushAllocation(ctx.allocator,in.allocation,0,VK_WHOLE_SIZE);
    VkDescriptorSetLayoutBinding b[4]={{0,VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,1,VK_SHADER_STAGE_COMPUTE_BIT,nullptr},
        {1,VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,1,VK_SHADER_STAGE_COMPUTE_BIT,nullptr},
        {2,VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,1,VK_SHADER_STAGE_COMPUTE_BIT,nullptr},{3,VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,1,VK_SHADER_STAGE_COMPUTE_BIT,nullptr}};
    VkDescriptorSetLayoutCreateInfo li{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO};li.bindingCount=4;li.pBindings=b;
    VkDescriptorSetLayout layout;vkRequireSuccess(vkCreateDescriptorSetLayout(ctx.device,&li,nullptr,&layout),"numeric layout");
    VkDescriptorPoolSize sizes[]={{VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,2},{VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,2}};
    VkDescriptorPoolCreateInfo pi{VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO};pi.maxSets=1;pi.poolSizeCount=2;pi.pPoolSizes=sizes;
    VkDescriptorPool pool;vkRequireSuccess(vkCreateDescriptorPool(ctx.device,&pi,nullptr,&pool),"numeric pool");
    VkDescriptorSetAllocateInfo si{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO};si.descriptorPool=pool;si.descriptorSetCount=1;si.pSetLayouts=&layout;
    VkDescriptorSet set;vkRequireSuccess(vkAllocateDescriptorSets(ctx.device,&si,&set),"numeric set");
    VkDescriptorImageInfo images[]={{scene.terrainTectonicTex.sampler,scene.terrainTectonicTex.view,VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL},
        {scene.terrainHydroTex.sampler,scene.terrainHydroTex.view,VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL}};
    VkDescriptorBufferInfo buffers[]={{in.handle,0,VK_WHOLE_SIZE},{out.handle,0,VK_WHOLE_SIZE}};
    VkWriteDescriptorSet writes[4]{};
    for(int i=0;i<4;++i){writes[i].sType=VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;writes[i].dstSet=set;writes[i].dstBinding=i;writes[i].descriptorCount=1;
        writes[i].descriptorType=b[i].descriptorType;if(i<2)writes[i].pImageInfo=&images[i];else writes[i].pBufferInfo=&buffers[i-2];}
    vkUpdateDescriptorSets(ctx.device,4,writes,0,nullptr);
    struct Params{float radius,maxElevation;uint32_t count;float detail;} params{terrain.planetRadius,terrain.maxElevation,uint32_t(count),1};
    VkPushConstantRange range{VK_SHADER_STAGE_COMPUTE_BIT,0,sizeof(params)};
    VkPipelineLayoutCreateInfo pli{VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO};pli.setLayoutCount=1;pli.pSetLayouts=&layout;pli.pushConstantRangeCount=1;pli.pPushConstantRanges=&range;
    VkPipelineLayout pipelineLayout;vkRequireSuccess(vkCreatePipelineLayout(ctx.device,&pli,nullptr,&pipelineLayout),"numeric pipeline layout");
    auto code=loadSPIRV("src/render/shaders/spirv/terrain_validate.comp.spv");VkShaderModule shader=createShaderModule(ctx.device,code);
    VkComputePipelineCreateInfo ci{VK_STRUCTURE_TYPE_COMPUTE_PIPELINE_CREATE_INFO};ci.layout=pipelineLayout;
    ci.stage={VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO,nullptr,0,VK_SHADER_STAGE_COMPUTE_BIT,shader,"main",nullptr};
    VkPipeline pipeline;vkRequireSuccess(vkCreateComputePipelines(ctx.device,VK_NULL_HANDLE,1,&ci,nullptr,&pipeline),"numeric pipeline");
    auto cmd=beginSingleTimeCommands(ctx);vkCmdBindPipeline(cmd,VK_PIPELINE_BIND_POINT_COMPUTE,pipeline);
    vkCmdBindDescriptorSets(cmd,VK_PIPELINE_BIND_POINT_COMPUTE,pipelineLayout,0,1,&set,0,nullptr);
    vkCmdPushConstants(cmd,pipelineLayout,VK_SHADER_STAGE_COMPUTE_BIT,0,sizeof(params),&params);
    vkCmdDispatch(cmd,uint32_t((count+63)/64),1,1);
    VkMemoryBarrier barrier{VK_STRUCTURE_TYPE_MEMORY_BARRIER};barrier.srcAccessMask=VK_ACCESS_SHADER_WRITE_BIT;barrier.dstAccessMask=VK_ACCESS_HOST_READ_BIT;
    vkCmdPipelineBarrier(cmd,VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,VK_PIPELINE_STAGE_HOST_BIT,0,1,&barrier,0,nullptr,0,nullptr);
    endSingleTimeCommands(ctx,cmd);vmaInvalidateAllocation(ctx.allocator,out.allocation,0,VK_WHOLE_SIZE);
    auto m=terrain.heightModel();auto values=static_cast<float*>(out.mapped);
    double maxBed=0,maxWater=0,maxNormal=0,maxDetail=0,maxGeometry=0;size_t land=0,wet=0;
    for(size_t i=0;i<count;++i){
        if(input[4*i+3]>=10){
            double span=2.0/8192.0,step=span/16;
            Vec3 cube(input[4*i]*span,1,input[4*i+2]*span),anchor(0,1,0);
            auto point=[&](Vec3 c){Vec3 n=c.normalized();float h=m.thHeight(TerrainHeight::normalize(TerrainHeight::vec3(n)),params.radius,params.maxElevation,1);
                return (n-anchor)*params.radius+n*h;};
            Vec3 fine=point(cube);
            double ux=(cube.x+1)/step,uz=(cube.z+1)/step,ix=std::floor(ux+1e-5),iz=std::floor(uz+1e-5);
            double fx=std::clamp(ux-ix,0.0,1.0),fz=std::clamp(uz-iz,0.0,1.0);
            Vec3 origin=cube+Vec3((ix-ux)*step,0,(iz-uz)*step);
            Vec3 coarse=fx+fz<=1?point(origin)*(1-fx-fz)+point(origin+Vec3(step,0,0))*fx+point(origin+Vec3(0,0,step))*fz:
                point(origin+Vec3(step,0,step))*(fx+fz-1)+point(origin+Vec3(step,0,0))*(1-fz)+point(origin+Vec3(0,0,step))*(1-fx);
            double morph=input[4*i+3]-10;Vec3 ref=coarse*(1-morph)+fine*morph;
            if(input[4*i+3]>=20){
                int code=int(input[4*i+3])-20,edge=code%4,difference=1+(code/4)%4,phase=code/16;
                Vec3 along=edge<2?Vec3(0,0,1):Vec3(1,0,0);
                auto aligned=[&](double step){double g=(cube.dot(along)+1)/step,cell=std::floor(g+1e-5),f=std::clamp(g-cell,0.0,1.0);
                    Vec3 first=cube+along*((cell-g)*step);return point(first)*(1-f)+point(first+along*step)*f;};
                double step=span/32*std::pow(2.,difference);double weight=phase*.5;
                ref=aligned(step*2)*(1-weight)+aligned(step)*weight;
            }
            if(input[4*i+1]<0)ref-=cube.normalized()*.05;
            for(int k=0;k<3;++k)check(std::isfinite(values[8*i+k]),"non-finite geometry");
            Vec3 gpu(values[8*i],values[8*i+1],values[8*i+2]);
            maxGeometry=std::max(maxGeometry,(ref-gpu).length()*1000);
            continue;
        }
        TerrainHeight::vec3 p=TerrainHeight::normalize({input[4*i],input[4*i+1],input[4*i+2]});
        float bed=m.thHeight(p,params.radius,params.maxElevation,1),water=m.thWater(p,params.radius,params.maxElevation,bed);
        auto n=m.thNormal(p,params.radius,params.maxElevation,.032f);
        for(int k=0;k<8;++k)check(std::isfinite(values[8*i+k]),"non-finite GPU surface");
        maxBed=std::max(maxBed,double(std::abs(values[8*i]-bed))*1000);
        maxWater=std::max(maxWater,double(std::abs(values[8*i+1]-water))*1000);
        Vec3 gpu(values[8*i+4],values[8*i+5],values[8*i+6]),cpu(n.x,n.y,n.z);
        maxNormal=std::max(maxNormal,std::acos(std::clamp(gpu.normalized().dot(cpu.normalized()),-1.0,1.0))*180/TerrainHeight::pi);
        maxDetail=std::max(maxDetail,double(std::abs(bed-m.thHeight(p,params.radius,params.maxElevation,0)))*1000);
        check(values[8*i+1]+1e-6f>=values[8*i],"water below bed");
        if(bed>0)++land;if(values[8*i+2]>.02f)++wet;
    }
    char line[512];snprintf(line,sizeof(line),"NUMERIC %s samples=%zu bedMaxError=%.6fm waterMaxError=%.6fm normalMaxError=%.6fdeg detailBound=%.6fm land=%zu wet=%zu",name,count,maxBed,maxWater,maxNormal,maxDetail,land,wet);log(line);
    check(maxBed<.5 && maxWater<.5,"CPU/GPU height mismatch above 0.5m");
    log(std::string("GEOMETRY ")+name+" anchored+morph+skirts+stitched-edges independent-double-reference maxError="+std::to_string(maxGeometry)+"m");
    check(maxGeometry<.02,"anchored/morph geometry exceeds 2cm");
    check(maxNormal<3,"CPU/GPU normal mismatch above 3 degrees");check(maxDetail<.151,"visual detail LOD changes collision by >15cm");
    check(std::abs(terrain.getHeight(Vec3(.1436,.478,.866))-.005f)<1e-6f,"launch pad must be 5m");
    for(double y:{-.9,-.5,0.,.5,.9})check(std::abs(terrain.getHeight(Vec3(-1,y,1e-7))-terrain.getHeight(Vec3(-1,y,-1e-7)))<.001f,"longitude seam >1m");
    check(g_vkValidationErrorCount==0,"Vulkan numerical validation errors");
    vkDestroyPipeline(ctx.device,pipeline,nullptr);vkDestroyShaderModule(ctx.device,shader,nullptr);vkDestroyPipelineLayout(ctx.device,pipelineLayout,nullptr);
    vkDestroyDescriptorPool(ctx.device,pool,nullptr);vkDestroyDescriptorSetLayout(ctx.device,layout,nullptr);in.destroy(ctx);out.destroy(ctx);
}

static void leaves(Terrain::QuadtreeTerrain& t,Terrain::TerrainNode* n,SceneSnapshot& snap,Vec3 localCamera,const CelestialDraw& earth,bool cull=true){
    if(cull && (!t.horizonVisible(n,localCamera,t.planetRadius)||!t.frustumVisible(n,earth.model,snap.view,snap.proj)))return;
    if(!cull&&!t.shadowVisible(n,localCamera))return;
    if(!n->isLeaf){for(auto& c:n->children)leaves(t,c.get(),snap,localCamera,earth,cull);return;}
    TerrainPatchDraw p{};memcpy(p.model,earth.model,64);p.planetRadius=t.planetRadius;p.maxElev=t.maxElevation;p.nodeLevel=n->level;
    p.nodeCenter[0]=float(n->center.x);p.nodeCenter[1]=float(n->center.y);p.nodeCenter[2]=float(n->center.z);
    p.nodeSideA[0]=float(n->sideA.x);p.nodeSideA[1]=float(n->sideA.y);p.nodeSideA[2]=float(n->sideA.z);
    p.nodeSideB[0]=float(n->sideB.x);p.nodeSideB[1]=float(n->sideB.y);p.nodeSideB[2]=float(n->sideB.z);
    for(int i=0;i<3;++i)p.planetCenterRel[i]=earth.center[i]-snap.camPos[i];
    p.lodMorph=n->morph;p.edgeState=t.edgeState(n);p.skirtDepthKm=t.skirtDepth(n);
    double altitude=localCamera.length()-t.planetRadius-t.getHeight(localCamera.normalized());
    p.detailWeight=1-Terrain::QuadtreeTerrain::smoothstep_local(.1f,1.5f,float(altitude));
    snap.terrainPatches.push_back(p);
}
static size_t leafCount(Terrain::TerrainNode* n){if(n->isLeaf)return 1;size_t c=0;for(auto& ch:n->children)c+=leafCount(ch.get());return c;}
static void lodTests(Terrain::QuadtreeTerrain& t){
    Vec3 p=Vec3(.1436,.478,.866).normalized(),camera=p*(t.planetRadius+.005+.05);
    for(int i=0;i<160;++i)for(auto& root:t.roots)t.updateSubdivision(root.get(),camera,t.planetRadius);
    size_t stable=0;for(auto& r:t.roots)stable+=leafCount(r.get());
    size_t boundSamples=0;
    auto boundsCheck=[&](auto&& self,Terrain::TerrainNode* n)->void{
        if(!n->isLeaf){for(auto& c:n->children)self(self,c.get());return;}
        auto bound=t.surfaceBounds(n);
        for(int y=0;y<5;++y)for(int x=0;x<5;++x){Vec3 p=(n->center+n->sideA*(x*.25-.5)+n->sideB*(y*.25-.5)).normalized();
            check((p*(t.planetRadius+t.getHeight(p))-bound.center).length()<=bound.radius,"terrain outside culling bounds");
            check((p*(t.planetRadius+t.getWaterHeight(p))-bound.center).length()<=bound.radius,"water outside culling bounds");++boundSamples;}
    };
    for(auto& r:t.roots)boundsCheck(boundsCheck,r.get());
    log("CULL_BOUNDS terrain+water samples="+std::to_string(boundSamples)+" covered=100%");
    size_t splits=t.splits,merges=t.merges;
    for(int i=0;i<64;++i){Vec3 jitter=camera+p*((i&1)?1e-7:-1e-7);for(auto& r:t.roots)t.updateSubdivision(r.get(),jitter,t.planetRadius);}
    check(t.splits==splits && t.merges==merges,"stationary sub-mm camera jitter churns LOD");
    size_t visible=0;for(auto& r:t.roots)if(t.horizonVisible(r.get(),p*(t.planetRadius+10000),t.planetRadius))++visible;
    check(!t.horizonVisible(t.roots[3].get(),Vec3(0,t.planetRadius+10000,0),t.planetRadius),"back hemisphere not culled");
    for(int i=0;i<160;++i)for(auto& r:t.roots)t.updateSubdivision(r.get(),p*(t.planetRadius+1e6),t.planetRadius);
    size_t farLeaves=0;for(auto& r:t.roots)farLeaves+=leafCount(r.get());check(farLeaves==6,"far camera cannot merge to roots");
    // Independent known-plane flood regression: boundaries are max(bed,level), no erosion.
    std::vector<float> bed(64,.4f),boundary(32,.6f);Hydro::HydroSimulator::fillDepressionsLocal(8,bed,boundary);
    for(int y=1;y<7;++y)for(int x=1;x<7;++x)check(std::abs(bed[y*8+x]-.6f)<1e-6f,"local priority flood height");
    char line[256];snprintf(line,sizeof(line),"LOD stationaryLeaves=%zu jitterSplits=0 jitterMerges=0 farLeaves=%zu horizonVisibleRoots=%zu; floodPlane=PASS",stable,farLeaves,visible);log(line);
}

static void runtimeUnits(Terrain::QuadtreeTerrain& t){
    Terrain::TerrainNode tile(Vec3(0,0,1),Vec3(0.00002,0,0),Vec3(0,0.00002,0),0.00001f,17);
    double h=t.getHeight(tile.center);
    tile.cachedCenterHeight=float(h);tile.cachedWaterHeight=t.getWaterHeight(tile.center);
    float proj[16],view[16],model[16];
    Mat4::perspective(.8,16.0/9,.0001,1e8).toFloatArray(proj);
    for(Vec3 offset:{Vec3(0,0,0),Vec3(10000,-20000,30000)}){
        Vec3 center=offset+Vec3(0,0,-6371);
        (Mat4::translate(center)*Mat4::scale(Vec3(6371,6371,6371))).toFloatArray(model);
        Mat4::lookAt(offset+Vec3(0,0,h+.08),offset+Vec3(0,0,h),Vec3(0,1,0)).toFloatArray(view);
        check(t.frustumVisible(&tile,model,view,proj),"runtime fine tile incorrectly culled");
        // Reproduce the old bootstrap: physics meters mistaken for render km.
        t.planetRadius=6371000;
        bool missing=!t.frustumVisible(&tile,model,view,proj);
        t.planetRadius=6371;
        check(missing,"meters/km disappearance reproduction did not trigger");
    }
    log("RUNTIME_UNITS old bootstrap reproduces missing tile; corrected factory visible at two floating origins: PASS");
}

static void visual(VulkanContext& ctx,VkRenderer3D& renderer,VkTAA& taa,Terrain::QuadtreeTerrain& t){
    auto& scene=renderer.scene;upload(ctx,scene,t);
    Vec3 mountain=Vec3(.2,.8,.4).normalized(),coast=Vec3(0,0,1);float maxH=-1e9,minCoast=1e9;
    for(int y=1;y<t.sim->height-1;++y)for(int x=0;x<t.sim->width;++x){
        Vec3 p=TerrainHeight::direction(x,y,t.sim->width,t.sim->height);float h=t.getHeight(p);
        if((p-Vec3(.1436,.478,.866).normalized()).length()<.15)continue;
        if(h>5 && h<15){
            auto n=t.heightModel().thNormal(TerrainHeight::vec3(p),t.planetRadius,t.maxElevation,.25f);
            double score=1.0-Vec3(n.x,n.y,n.z).dot(p);
            if(score>maxH){maxH=float(score);mountain=p;}
        }
        if(std::abs(h)<minCoast){minCoast=std::abs(h);coast=p;}
    }
    struct Rig{const char* name;Vec3 up;double altitude;bool orbit;};
    std::vector<Rig> rigs={{"launchpad",Vec3(.1436,.478,.866).normalized(),.08,false},
        {"mountain",mountain,2.0,false},{"coast",coast,.20,false},{"orbit",mountain,400,true},{"lod-transition",mountain,.15,false},
        {"startup",Vec3(.1436,.478,.866).normalized(),.08,false}};
    VkExtent2D size{960,540};size_t pixels=size_t(size.width)*size.height;
    VkQueryPool queries;VkQueryPoolCreateInfo qi{VK_STRUCTURE_TYPE_QUERY_POOL_CREATE_INFO};qi.queryType=VK_QUERY_TYPE_TIMESTAMP;qi.queryCount=2;
    vkRequireSuccess(vkCreateQueryPool(ctx.device,&qi,nullptr,&queries),"visual queries");
    VkPhysicalDeviceProperties props{};vkGetPhysicalDeviceProperties(ctx.physDevice,&props);
    for(const auto& rig:rigs){
        Vec3 up=rig.up.normalized(),east=Vec3(0,1,0).cross(up).normalized();if(east.length()<.1)east=Vec3(1,0,0);
        Vec3 north=up.cross(east).normalized();double ground=t.getHeight(up);
        double height=rig.orbit?rig.altitude:std::max(ground,0.0)+rig.altitude;
        std::vector<float> previous;
        size_t total=0,submitted=0;double maxFrameMs=0,maxMotion=0;std::vector<double> timings;
        size_t startSplits=0,startMerges=0;bool transition=std::string(rig.name)=="lod-transition";
        bool startup=std::string(rig.name)=="startup";
        if(startup)for(auto& root:t.roots){root->collapse();root->morph=1;}
        int frames=startup?144:transition?64:12;
        for(int frame=0;frame<frames;++frame){
            SceneSnapshot snap{};snap.sunIntensity=1;snap.skyVibrancy=1;snap.time=3;snap.aspect=float(size.width)/size.height;
            snap.lightDir[0]=.45f;snap.lightDir[1]=.8f;snap.lightDir[2]=.4f;
            double rigHeight=height;
            if(transition)rigHeight=ground+.15+1.2*std::max(0.0,1.0-std::abs(frame-24.0)/24.0);
            Vec3 camera((frame-6)*.002,rigHeight,0);snap.camPos[0]=float(camera.x);snap.camPos[1]=float(camera.y);
            snap.sunWorldPos[0]=500000;snap.sunWorldPos[1]=1000000;snap.sunWorldPos[2]=500000;
            Vec3 target=rig.orbit?Vec3(0,-6371,0):Vec3(camera.x,std::max(ground,0.0)+(std::string(rig.name)=="mountain"?-2.0:.015),std::string(rig.name)=="mountain"?-20.0:-1.0);
            Mat4::lookAt(camera,target,rig.orbit?Vec3(0,0,1):Vec3(0,1,0)).toFloatArray(snap.view);
            Mat4::perspective(rig.orbit?2.6:1.15,snap.aspect,.001,1e8).toFloatArray(snap.proj);
            CelestialDraw earth{};earth.bodyIdx=earth.atmoIdx=3;earth.radius=t.planetRadius;earth.center[1]=-t.planetRadius;
            // Rotation rows are east/up/north; local rig direction maps to world +Y.
            for(int j=0;j<3;++j){double e=j==0?east.x:j==1?east.y:east.z,u=j==0?up.x:j==1?up.y:up.z,n=j==0?north.x:j==1?north.y:north.z;
                earth.model[j*4]=float(e*t.planetRadius);earth.model[j*4+1]=float(u*t.planetRadius);earth.model[j*4+2]=float(n*t.planetRadius);}
            earth.model[13]=-t.planetRadius;earth.model[15]=1;earth.r=earth.g=earth.b=1;earth.shaderKey="earth";
            snap.celestials.push_back(earth);
            Vec3 localCamera=east*camera.x+up*(camera.y+t.planetRadius)+north*camera.z;
            // Settle initial LOD; subsequent frames preserve the tree/history.
            int settle=frame==0&&!startup?160:1;
            for(int k=0;k<settle;++k)for(auto& r:t.roots)t.updateSubdivision(r.get(),localCamera,t.planetRadius);
            if(frame==0){startSplits=t.splits;startMerges=t.merges;}
            total=0;for(auto& r:t.roots){total+=leafCount(r.get());leaves(t,r.get(),snap,localCamera,earth);}
            SceneSnapshot shadowSnap=snap;shadowSnap.terrainPatches.clear();
            for(auto& root:t.roots)leaves(t,root.get(),shadowSnap,localCamera,earth,false);
            snap.terrainShadowPatches=std::move(shadowSnap.terrainPatches);
            submitted=snap.terrainPatches.size();check(submitted>0,"frustum culled all terrain");check(submitted<total,"visible draw count not culled");
            Buffer readback;readback.init(ctx,pixels*20,VK_BUFFER_USAGE_TRANSFER_DST_BIT);
            int slot=frame%2;taa.updateMatrices(snap.view,snap.proj,snap.camPos,earth.center,earth.radius);taa.uploadMatrices(ctx,slot);
            Buffer shadowReadback;size_t shadowPixels=0;
            auto cmd=beginSingleTimeCommands(ctx);
            if(frame==0 && !rig.orbit){renderer.shadowSystem.enabled=true;GpuCascadeShadowConfig cfg{};
                renderer.shadowSystem.render(ctx,cmd,slot,snap,scene.terrainPatchVbo,scene.terrainPatchIbo,scene.terrainPatchIcount,scene.terrainGlobalTexSet,snap.lightDir,snap.camPos,cfg);
                check(renderer.shadowSystem.atlasReady,"terrain shadow pass not rendered");
                auto& shadow=renderer.shadowSystem;shadowPixels=size_t(shadow.percascadeDimXY)*shadow.percascadeDimXY*shadow.cascadeCount;
                shadowReadback.init(ctx,shadowPixels*4,VK_BUFFER_USAGE_TRANSFER_DST_BIT);
                VkBufferImageCopy region{};region.imageSubresource={VK_IMAGE_ASPECT_DEPTH_BIT,0,0,1};region.imageExtent={uint32_t(shadow.percascadeDimXY*shadow.cascadeCount),uint32_t(shadow.percascadeDimXY),1};
                VkOceanSystem::depthBarrier(cmd,shadow.atlasImage,VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL);
                vkCmdCopyImageToBuffer(cmd,shadow.atlasImage,VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,shadowReadback.handle,1,&region);
                VkOceanSystem::depthBarrier(cmd,shadow.atlasImage,VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
            }
            if(frame==0)log(std::string("RENDER_BEGIN ")+rig.name);
            vkCmdResetQueryPool(cmd,queries,0,2);
            taa.beginGeometryPass(cmd,size);VkFrameRenderer::setViewportScissor(cmd,size);vkCmdWriteTimestamp(cmd,VK_PIPELINE_STAGE_ALL_COMMANDS_BIT,queries,0);
            renderer.render(cmd,slot,snap,size);taa.endGeometryPass(cmd);
            if(frame==0){VkBufferImageCopy probe{};probe.bufferOffset=pixels*12;probe.imageSubresource={VK_IMAGE_ASPECT_COLOR_BIT,0,0,1};probe.imageExtent={size.width,size.height,1};
                transitionImage(cmd,taa.colorImages[taa.currentIdx],VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL);
                vkCmdCopyImageToBuffer(cmd,taa.colorImages[taa.currentIdx],VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,readback.handle,1,&probe);
                transitionImage(cmd,taa.colorImages[taa.currentIdx],VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);}

            renderer.oceanSystem.render(ctx,cmd,taa,scene,snap,slot,size,renderer.atmoTune);
            vkCmdWriteTimestamp(cmd,VK_PIPELINE_STAGE_ALL_COMMANDS_BIT,queries,1);
            renderer.renderAtmoAfterGeometry(ctx,cmd,taa,snap,slot);
            VkBufferImageCopy region{};region.imageSubresource={VK_IMAGE_ASPECT_COLOR_BIT,0,0,1};region.imageExtent={size.width,size.height,1};
            transitionImage(cmd,taa.colorImages[taa.currentIdx],VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL);
            vkCmdCopyImageToBuffer(cmd,taa.colorImages[taa.currentIdx],VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,readback.handle,1,&region);
            transitionImage(cmd,taa.colorImages[taa.currentIdx],VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
            region.bufferOffset=pixels*8;region.imageSubresource.aspectMask=VK_IMAGE_ASPECT_DEPTH_BIT;
            VkOceanSystem::depthBarrier(cmd,taa.depthImage,VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL);
            vkCmdCopyImageToBuffer(cmd,taa.depthImage,VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,readback.handle,1,&region);
            VkOceanSystem::depthBarrier(cmd,taa.depthImage,VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
            endSingleTimeCommands(ctx,cmd);vmaInvalidateAllocation(ctx.allocator,readback.allocation,0,VK_WHOLE_SIZE);
            if(shadowPixels){vmaInvalidateAllocation(ctx.allocator,shadowReadback.allocation,0,VK_WHOLE_SIZE);
                auto depths=static_cast<float*>(shadowReadback.mapped);size_t covered=0;
                for(size_t i=0;i<shadowPixels;++i){check(std::isfinite(depths[i])&&depths[i]>=0&&depths[i]<=1,"non-finite shadow depth");if(depths[i]<1)++covered;}
                check(covered>0,"shadow atlas has no terrain");log(std::string("SHADOW ")+rig.name+" covered="+std::to_string(covered)+" casters="+std::to_string(snap.terrainShadowPatches.size()));shadowReadback.destroy(ctx);}
            uint64_t times[2];vkRequireSuccess(vkGetQueryPoolResults(ctx.device,queries,0,2,sizeof(times),times,8,VK_QUERY_RESULT_64_BIT|VK_QUERY_RESULT_WAIT_BIT),"visual timestamps");
            double ms=double(times[1]-times[0])*props.limits.timestampPeriod/1e6;
            maxFrameMs=std::max(maxFrameMs,ms);if(frame>0)timings.push_back(ms);
            auto data=static_cast<uint16_t*>(readback.mapped);auto depth=reinterpret_cast<float*>(data+pixels*4);
            if(frame==0){auto pre=reinterpret_cast<uint16_t*>(static_cast<uint8_t*>(readback.mapped)+pixels*12);
                std::ofstream image(std::string("build/tests/terrain-")+rig.name+"-before-water.ppm",std::ios::binary);image<<"P6\n"<<size.width<<" "<<size.height<<"\n255\n";
                for(size_t i=0;i<pixels;++i)for(int c=0;c<3;++c){uint8_t v=uint8_t(std::round(255*Radiometry::srgb(Radiometry::aces(halfFloat(pre[4*i+c]),Radiometry::defaultDisplayExposure))));image.write(reinterpret_cast<char*>(&v),1);}}
            size_t drawn=0;double sum=0,motion=0;std::vector<float> next(pixels*3);
            std::ofstream image;if(frame==0||frame==frames-1){image.open(std::string("build/tests/terrain-")+rig.name+"-"+std::to_string(frame)+".ppm",std::ios::binary);image<<"P6\n"<<size.width<<" "<<size.height<<"\n255\n";}
            for(size_t i=0;i<pixels;++i){check(std::isfinite(depth[i])&&depth[i]>=0&&depth[i]<=1,"invalid visual depth");if(depth[i]<1)++drawn;
                for(int c=0;c<3;++c){float v=halfFloat(data[4*i+c]);check(std::isfinite(v)&&v>=0,"invalid visual HDR");sum+=v;next[i*3+c]=v;
                    if(!previous.empty())motion+=std::abs(v-previous[i*3+c]);
                    if(image){unsigned char q=uint8_t(std::round(255*Radiometry::srgb(Radiometry::aces(v,Radiometry::defaultDisplayExposure))));image.write(reinterpret_cast<char*>(&q),1);}}
            }
            size_t holes=0;
            for(size_t y=1;y+1<size.height;++y)for(size_t x=1;x+1<size.width;++x){size_t i=y*size.width+x;
                if(depth[i]<1)continue;bool enclosed=true;
                for(int dy=-1;dy<=1;++dy)for(int dx=-1;dx<=1;++dx)if(dx||dy)enclosed=enclosed&&depth[size_t(int64_t(i)+int64_t(dy)*int64_t(size.width)+dx)]<1;
                if(enclosed)++holes;
            }
            if(holes)log(std::string("DEPTH_HOLES ")+rig.name+" frame="+std::to_string(frame)+" enclosed="+std::to_string(holes));
            check(holes==0,"enclosed depth holes in terrain/water image");
            check(drawn>pixels/20,"terrain image empty");check(sum>1,"terrain image unlit");
            maxMotion=std::max(maxMotion,motion/(pixels*3));previous=std::move(next);
            readback.destroy(ctx);check(g_vkValidationErrorCount==0,"visual Vulkan validation errors");taa.swap();
        }
        std::sort(timings.begin(),timings.end());
        log(std::string("TIMING ")+rig.name+" warmGeometryWaterMedian="+std::to_string(timings[timings.size()/2])+"ms");
        if(transition){check(t.splits>startSplits&&t.merges>startMerges,"transition rig did not split and merge");
            log("TRANSITIONS splits="+std::to_string(t.splits-startSplits)+" merges="+std::to_string(t.merges-startMerges)+" zero enclosed depth holes");}
        char line[512];snprintf(line,sizeof(line),"VISUAL %s ground=%.6fkm leaves=%zu submitted=%zu cull=%.2f%% maxGeometryWaterGPU=%.3fms maxMotionHDR=%.7f frames=%d",rig.name,ground,total,submitted,100.0*(1.-double(submitted)/total),maxFrameMs,maxMotion,frames);log(line);
    }
    vkDestroyQueryPool(ctx.device,queries,nullptr);
}

int main(int argc,char** argv){
    setvbuf(stdout,nullptr,_IONBF,0);
    try{
        check(glfwInit()!=0,"GLFW init");glfwWindowHint(GLFW_CLIENT_API,GLFW_NO_API);glfwWindowHint(GLFW_VISIBLE,GLFW_FALSE);
        GLFWwindow* window=glfwCreateWindow(960,540,"Terrain verification",nullptr,nullptr);
        VulkanContext ctx;VkDescriptorManager desc;VkRenderer3D renderer;VkTAA taa;
        check(window&&ctx.initCore(window)&&ctx.initSwapchain(window),"Vulkan init");check(desc.init(ctx),"descriptors");
        check(renderer.init(ctx,desc,VK_FORMAT_R16G16B16A16_SFLOAT,VK_FORMAT_D32_SFLOAT),"renderer init");
        check(taa.init(ctx,{960,540},ctx.swapFormat,"src/render/shaders/spirv/taa.vert.spv","src/render/shaders/spirv/taa.frag.spv"),"TAA init");
        Terrain::QuadtreeTerrain fixture(6371,false);
        for(int test=0;test<3;++test){
            int w=test==0?8:test==1?64:256,h=w/2;
            fixture.sim->width=w;fixture.sim->height=h;fixture.sim->gridHeight.resize(w*h);
            fixture.hydroSim->width=w;fixture.hydroSim->height=h;
            fixture.hydroSim->data.filledHeight.resize(w*h);fixture.hydroSurfaceBed.resize(w*h);fixture.hydroSim->data.accumulation.assign(w*h,0);
            fixture.hydroSim->data.strahler.assign(w*h,0);fixture.hydroSim->data.flowDir.assign(w*h,-1);
            for(int y=0;y<h;++y)for(int x=0;x<w;++x){Vec3 p=TerrainHeight::direction(x,y,w,h);
                float value=test==0?.3f:float(.48+.3*p.x+.08*p.z);
                fixture.hydroSurfaceBed[y*w+x]=value;fixture.sim->gridHeight[y*w+x]=value;fixture.hydroSim->data.filledHeight[y*w+x]=value+.001f;}
            Terrain::QuadtreeTerrain::canonicalPoles(fixture.sim->gridHeight,w,h);
            Terrain::QuadtreeTerrain::canonicalPoles(fixture.hydroSurfaceBed,w,h);
            Terrain::QuadtreeTerrain::canonicalPoles(fixture.hydroSim->data.filledHeight,w,h);
            numerical(ctx,renderer.scene,fixture,test==0?"sea8x4":test==1?"coast64x32":"mountains256x128");
        }
        lodTests(fixture);
        auto production=Terrain::QuadtreeTerrain::fromMeters(6371000.0);
        check(std::abs(production.planetRadius-6371.0f)<0.001f,"runtime meters/km initialization mismatch");
        runtimeUnits(production);
        numerical(ctx,renderer.scene,production,"production256x128-hydro512x256");
        visual(ctx,renderer,taa,production);
        renderer.shutdown(ctx);taa.shutdown(ctx);desc.shutdown(ctx);ctx.shutdown();glfwDestroyWindow(window);glfwTerminate();
        check(g_vkValidationErrorCount==0,"shutdown validation errors");
        log("PASS: runtime meter/km bootstrap; CPU/GPU surface, water and normals; seams; pad; LOD stability/merges; six production rigs (256 frames), including cold startup; zero Vulkan errors.");return 0;
    }catch(const std::exception& e){log(std::string("FAIL: ")+e.what());return 9;}
}
