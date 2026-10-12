#pragma once
#include "math/math3d.h"
#include "tectonic_sim.h"
#include "climate_sim.h"
#include "hydro_sim.h"
#include "terrain_height.h"
#include <cstdio>
#include <limits>
#include <vector>
#include <memory>
#include <cmath>
#include <algorithm>

namespace Terrain {

// Rendering LOD owns geometry only. Hydrology is planet-fixed, never rebuilt
// when a leaf splits, merges, or crosses the camera's horizon.
struct TerrainNode {
    Vec3 center,sideA,sideB;
    float size;
    int level;
    bool isLeaf=true;
    float morph=1.0f;
    mutable float cachedSkirt=-1.0f;
    mutable float cachedCenterHeight=std::numeric_limits<float>::quiet_NaN();
    mutable float cachedWaterHeight=std::numeric_limits<float>::quiet_NaN();
    std::unique_ptr<TerrainNode> children[4];
    TerrainNode(Vec3 c,Vec3 a,Vec3 b,float s,int l):center(c),sideA(a),sideB(b),size(s),level(l){}
    void subdivide() {
        if(!isLeaf)return;
        Vec3 a=sideA*0.5,b=sideB*0.5;
        for(int i=0;i<4;++i){
            children[i]=std::make_unique<TerrainNode>(center+a*((i&1)?0.5:-0.5)+b*((i&2)?-0.5:0.5),a,b,size*0.5f,level+1);
            children[i]->morph=0.0f;
        }
        isLeaf=false;
    }
    void collapse(){for(auto& c:children)c.reset();isLeaf=true;}
};

class QuadtreeTerrain {
public:
    float planetRadius,maxElevation=25.0f;
    Tectonic::TectonicSimulator* sim=nullptr;
    Climate::ClimateSimulator* climateSim=nullptr;
    Hydro::HydroSimulator* hydroSim=nullptr;
    std::unique_ptr<TerrainNode> roots[6];
    std::vector<float> hydroSurfaceBed;
    double plateLongitudeSlope=0,plateLatitudeSlope=0;
    size_t visitedNodes=0,splits=0,merges=0;
    // Physics stores meters; all terrain geometry, bounds and heights use km.
    // Use this factory at the physics/render boundary, including validation.
    static QuadtreeTerrain fromMeters(double radiusMeters,bool generate=true){
        return QuadtreeTerrain(float(radiusMeters*0.001),generate);
    }
    // The constructor flag is used by deterministic numerical/GPU fixtures.
    explicit QuadtreeTerrain(float radiusKm,bool generate=true):planetRadius(radiusKm) {
        sim=new Tectonic::TectonicSimulator(256,128);
        climateSim=new Climate::ClimateSimulator(256,128);
        hydroSim=new Hydro::HydroSimulator(512,256);
        if(generate){
            fprintf(stderr,"[Terrain] Tectonic + climate...\n");
            sim->simulate(40);
            for(float& height:sim->gridHeight)height=std::clamp(height,0.0f,1.0f);
            canonicalPoles(sim->gridHeight,sim->width,sim->height);
            climateSim->simulate(sim->gridHeight);
            rebuildHydrology();
        }
        roots[0]=std::make_unique<TerrainNode>(Vec3(1,0,0),Vec3(0,0,-2),Vec3(0,2,0),1.0f,0);
        roots[1]=std::make_unique<TerrainNode>(Vec3(-1,0,0),Vec3(0,0,2),Vec3(0,2,0),1.0f,0);
        roots[2]=std::make_unique<TerrainNode>(Vec3(0,1,0),Vec3(2,0,0),Vec3(0,0,-2),1.0f,0);
        roots[3]=std::make_unique<TerrainNode>(Vec3(0,-1,0),Vec3(2,0,0),Vec3(0,0,2),1.0f,0);
        roots[4]=std::make_unique<TerrainNode>(Vec3(0,0,1),Vec3(2,0,0),Vec3(0,2,0),1.0f,0);
        roots[5]=std::make_unique<TerrainNode>(Vec3(0,0,-1),Vec3(-2,0,0),Vec3(0,2,0),1.0f,0);
    }
    ~QuadtreeTerrain(){delete hydroSim;delete climateSim;delete sim;}
    QuadtreeTerrain(const QuadtreeTerrain&)=delete;
    QuadtreeTerrain& operator=(const QuadtreeTerrain&)=delete;

    TerrainHeight::Model heightModel()const {
        TerrainHeight::Model m;
        m.plate=&sim->gridHeight;m.pw=sim->width;m.ph=sim->height;
        m.filled=&hydroSim->data.filledHeight;m.hydroBed=&hydroSurfaceBed;m.hw=hydroSim->width;m.hh=hydroSim->height;
        return m;
    }
    static void canonicalPoles(std::vector<float>& grid,int w,int h){
        for(int y:{0,h-1}){double sum=0;for(int x=0;x<w;++x)sum+=grid[y*w+x];
            for(int x=0;x<w;++x)grid[y*w+x]=float(sum/w);}
    }
    void rebuildHydrology(){
        refreshBounds();
        const int w=hydroSim->width,h=hydroSim->height;
        std::vector<float> bed(w*h),rain(w*h),temp(w*h);
        auto model=heightModel();
        for(int y=0;y<h;++y)for(int x=0;x<w;++x){
            Vec3 p=TerrainHeight::direction(x,y,w,h);
            auto f=TerrainHeight::normalize(TerrainHeight::vec3(p));
            bed[y*w+x]=model.thElevation(model.thHeight(f,planetRadius,maxElevation,0),maxElevation);
            float u=float(x)/w,v=float(y)/(h-1);
            rain[y*w+x]=TerrainHeight::sample(climateSim->data.precipitation,climateSim->width,climateSim->height,u,v);
            temp[y*w+x]=TerrainHeight::sample(climateSim->data.temperature,climateSim->width,climateSim->height,u,v);
        }
        fprintf(stderr,"[Terrain] Fixed %dx%d refined surface hydrology...\n",w,h);
        canonicalPoles(bed,w,h);
        hydroSurfaceBed=bed;
        hydroSim->simulate(bed,rain,temp,maxElevation);
        canonicalPoles(hydroSim->data.filledHeight,w,h);
        // Preserve climate moisture. River order belongs in the hydro texture.
    }
    void refreshBounds(){
        double dx=0,dy=0;
        for(int y=0;y<sim->height;++y)for(int x=0;x<sim->width;++x){
            float h=sim->gridHeight[y*sim->width+x];
            dx=std::max(dx,double(std::abs(h-sim->gridHeight[y*sim->width+(x+1)%sim->width])));
            if(y+1<sim->height)dy=std::max(dy,double(std::abs(h-sim->gridHeight[(y+1)*sim->width+x])));
        }
        plateLongitudeSlope=dx*sim->width;plateLatitudeSlope=dy*(sim->height-1);
    }
    float sampleTectonic(float u,float v)const{return heightModel().thPlate(u,v);}
    static float smoothstep_local(float a,float b,float x){return TerrainHeight::Model{}.thSmooth(a,b,x);}
    float getHeightRaw(const Vec3& p)const{return getRefinedHeight(p);}
    float getRefinedHeight(const Vec3& p)const{
        auto m=heightModel();auto f=TerrainHeight::normalize(TerrainHeight::vec3(p));
        return m.thElevation(m.thHeight(f,planetRadius,maxElevation,0),maxElevation);
    }
    float getHeight(const Vec3& p)const{
        return heightModel().thHeight(TerrainHeight::normalize(TerrainHeight::vec3(p)),planetRadius,maxElevation,1);
    }
    float getWaterHeight(const Vec3& p)const{
        auto m=heightModel();auto f=TerrainHeight::normalize(TerrainHeight::vec3(p));
        return m.thWater(f,planetRadius,maxElevation,m.thHeight(f,planetRadius,maxElevation,1));
    }
    Vec3 getPosition(const Vec3& p)const{return p.normalized()*(planetRadius+getHeight(p));}

    static double angularRadius(const TerrainNode* n){
        // Cube projection is Lipschitz <=1 on a unit face. Half the diagonal
        // bounds the geodesic radius, including cube corners.
        return std::min(TerrainHeight::pi,0.5*std::sqrt(n->sideA.lengthSq()+n->sideB.lengthSq()));
    }
    bool horizonVisible(const TerrainNode* n,const Vec3& camera,float radius)const{
        double d=camera.length();
        double occluder=radius-6.0;
        if(d<=occluder)return true;
        double angle=std::acos(std::clamp(n->center.normalized().dot(camera/d),-1.0,1.0));
        double horizon=std::acos(std::clamp(occluder/d,0.0,1.0));
        double relief=std::acos(occluder/(radius+maxElevation));
        return angle<=horizon+relief+angularRadius(n);
    }
    const TerrainNode* leafAt(Vec3 p)const {
        double ax=std::abs(p.x),ay=std::abs(p.y),az=std::abs(p.z),scale=std::max({ax,ay,az});
        int face=ax>=ay&&ax>=az?(p.x>=0?0:1):ay>=az?(p.y>=0?2:3):(p.z>=0?4:5);
        p=p/scale;
        const TerrainNode* n=roots[face].get();
        while(!n->isLeaf){
            Vec3 d=p-n->center;
            int child=(d.dot(n->sideA)>0?1:0)+(d.dot(n->sideB)<0?2:0);
            n=n->children[child].get();
        }
        return n;
    }
    // Bit 30 enables stitching. Four 3-bit neighbour level differences and
    // four 4-bit morph steps (0..8, exactly 1/8) fit the existing 128-byte PC.
    uint32_t edgeState(const TerrainNode* n)const {
        uint32_t state=1u<<30;
        for(int edge=0;edge<4;++edge){
            Vec3 axis=edge<2?n->sideA:n->sideB;
            const TerrainNode* other=leafAt(n->center+axis*((edge&1)?0.500001:-0.500001));
            if(n->level-other->level>7)return 0; // skirt fallback for transient extreme gaps
            int difference=std::clamp(n->level-other->level,0,7);
            float morph=other->level<n->level?other->morph:
                other->level==n->level?std::min(n->morph,other->morph):n->morph;
            uint32_t step=uint32_t(std::clamp(int(std::round(morph*8)),0,8));
            state|=uint32_t(difference)<<(1+edge*3);
            state|=step<<(13+edge*4);
        }
        return state;
    }
    struct Bounds{Vec3 center;double radius;};
    Bounds surfaceBounds(const TerrainNode* n)const {
        Vec3 dir=n->center.normalized();double angle=angularRadius(n);
        if(n->level<5)return {dir*planetRadius,planetRadius*angle+maxElevation+6.0};
        if(!std::isfinite(n->cachedCenterHeight)){
            n->cachedCenterHeight=getHeight(dir);n->cachedWaterHeight=getWaterHeight(dir);
        }
        // Bounds for the analytic field, not an arbitrary 25km allowance on
        // every meter-sized tile. Bilinear derivatives plus conservative noise
        // derivative limits bound all vertices, morphs and water in the tile.
        double phi=std::acos(std::clamp(dir.y,-1.0,1.0));
        double minSin=std::min(std::sin(std::max(0.0,phi-angle)),std::sin(std::min(TerrainHeight::pi,phi+angle)));
        double poleBound=sim->width*plateLatitudeSlope/(2*TerrainHeight::pi);
        double lonBound=minSin>1e-9?std::min(poleBound,plateLongitudeSlope/(2*TerrainHeight::pi*minSin)):poleBound;
        double plateGradient=lonBound+plateLatitudeSlope/TerrainHeight::pi;
        // Cubic value noise |gradient|<=1.5*sqrt(3). Six ridge octaves and
        // domain warp yield <270 + 7*plateGradient after coastal/shelf shaping.
        double heightGradient=std::max(6.0/.45,double(maxElevation)/.55)*(270+7*plateGradient)+(maxElevation+6)*25;
        double bed=n->cachedCenterHeight,water=n->cachedWaterHeight;
        double radius=angle*(planetRadius+maxElevation+6+heightGradient)+std::abs(water-bed)*.5+.075;
        double middle=(bed+water)*.5;
        double globalBound=(planetRadius+std::abs(middle))*angle+std::max(std::abs(maxElevation-middle),std::abs(-6.0-middle))+.075;
        radius=std::min(radius,globalBound);
        if(n->isLeaf){uint32_t edges=edgeState(n);bool extreme=(edges&(1u<<30))==0;
            for(int i=0;i<4;++i)extreme=extreme||((edges>>(1+i*3))&7u)>5;
            if(extreme)radius=std::max(radius,planetRadius*angle+maxElevation+6.0+skirtDepth(n));
        }
        return {dir*(planetRadius+(bed+water)*.5),radius};
    }
    bool shadowVisible(const TerrainNode* n,const Vec3& camera)const {
        auto b=surfaceBounds(n);
        return (b.center-camera).length()<=b.radius+60.0;
    }
    // Conservative sphere/frustum test in floating-origin world km.
    bool frustumVisible(const TerrainNode* n,const float model[16],const float view[16],const float proj[16])const{
        auto bounds=surfaceBounds(n);Vec3 p=bounds.center;
        Vec3 x=Vec3(model[0],model[1],model[2]).normalized(),y=Vec3(model[4],model[5],model[6]).normalized(),z=Vec3(model[8],model[9],model[10]).normalized();
        Vec3 world=Vec3(model[12],model[13],model[14])+x*p.x+y*p.y+z*p.z;
        double vx=view[0]*world.x+view[4]*world.y+view[8]*world.z+view[12];
        double vy=view[1]*world.x+view[5]*world.y+view[9]*world.z+view[13];
        double vz=view[2]*world.x+view[6]*world.y+view[10]*world.z+view[14];
        double bound=bounds.radius;
        if(-vz+bound<0)return false;
        double fx=std::abs(proj[0]),fy=std::abs(proj[5]);
        return std::abs(vx)*fx+vz<=bound*std::sqrt(fx*fx+1) &&
               std::abs(vy)*fy+vz<=bound*std::sqrt(fy*fy+1);
    }
    // Skirts cover sphere chord error plus bounded regional relief between
    // coarse/fine tessellation. Fine leaves use measured local height range.
    float skirtDepth(const TerrainNode* n)const{
        if(n->cachedSkirt>=0)return n->cachedSkirt;
        double chord=planetRadius*(1.0-std::cos(angularRadius(n)/16.0));
        if(n->level<6)return n->cachedSkirt=float(chord+maxElevation+6.0);
        float lo=1e9f,hi=-1e9f;
        for(int y=0;y<3;++y)for(int x=0;x<3;++x){
            float h=getHeight((n->center+n->sideA*(x*0.5-0.5)+n->sideB*(y*0.5-0.5)).normalized());
            lo=std::min(lo,h);hi=std::max(hi,h);
        }
        return n->cachedSkirt=float(chord)+std::max(0.003f,(hi-lo)*2.0f+0.001f);
    }
    bool mergeSubtree(TerrainNode* n){
        if(n->isLeaf){n->morph=std::max(0.0f,n->morph-0.125f);return n->morph==0;}
        bool ready=true;
        for(auto& c:n->children){bool r=mergeSubtree(c.get());ready=ready&&r;}
        if(ready){n->collapse();++merges;}
        return false;
    }
    void updateSubdivision(TerrainNode* n,const Vec3& camera,float radius){
        ++visitedNodes;
        bool visible=horizonVisible(n,camera,radius);
        // Center altitude is part of the distance; a camera standing on a
        // 5km mountain must refine as much as one standing at sea level.
        Vec3 dir=n->center.normalized();
        if(n->level>=5 && !std::isfinite(n->cachedCenterHeight)){n->cachedCenterHeight=getHeight(dir);n->cachedWaterHeight=getWaterHeight(dir);}
        double elev=n->level>=5?n->cachedCenterHeight:0.0;
        double dist=(dir*(radius+elev)-camera).length();
        if(n->level>=5)dist=std::min(dist,(dir*(radius+n->cachedWaterHeight)-camera).length());
        double span=n->size*radius*2;
        bool split=visible && dist<span*(n->isLeaf?2.5:3.25) && n->level<18;
        if(split){
            n->morph=std::min(1.0f,n->morph+0.125f);
            // Finish parent morph before subdividing, so nested transitions
            // cannot lose their parent interpolation surface.
            if(n->isLeaf && n->morph>=1){n->subdivide();++splits;}
            if(!n->isLeaf)for(auto& c:n->children)updateSubdivision(c.get(),camera,radius);
        }else if(!n->isLeaf){
            bool ready=true;
            for(auto& c:n->children){bool r=mergeSubtree(c.get());ready=ready&&r;}
            if(ready){n->collapse();n->morph=1;++merges;}
        }else n->morph=std::min(1.0f,n->morph+0.125f);
    }
};
} // namespace Terrain
