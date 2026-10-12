#pragma once
#include "math/math3d.h"
#include <cstdint>
#include <vector>

namespace TerrainHeight {
inline constexpr double pi=3.14159265358979323846;
// Float arithmetic deliberately matches the shader; world transforms remain double.
struct vec3 {
    float x,y,z;
    vec3(float a,float b,float c):x(a),y(b),z(c){}
    explicit vec3(const Vec3& p):x(float(p.x)),y(float(p.y)),z(float(p.z)){}
    vec3 operator+(vec3 b)const{return {x+b.x,y+b.y,z+b.z};}
    vec3 operator-(vec3 b)const{return {x-b.x,y-b.y,z-b.z};}
    vec3 operator*(float f)const{return {x*f,y*f,z*f};}
};
using uint=uint32_t;
inline float clamp(float x,float a,float b){return std::clamp(x,a,b);}
inline float mix(float a,float b,float t){return a+(b-a)*t;}
inline float abs(float x){return std::abs(x);}
inline float floor(float x){return std::floor(x);}
inline float max(float a,float b){return std::max(a,b);}
inline float min(float a,float b){return std::min(a,b);}
inline float acos(float x){return std::acos(x);}
inline float atan(float y,float x){return std::atan2(y,x);}
inline float length(vec3 p){return std::sqrt(p.x*p.x+p.y*p.y+p.z*p.z);}
inline vec3 normalize(vec3 p){return p*(1.0f/max(length(p),1e-20f));}
inline vec3 cross(vec3 a,vec3 b){return {a.y*b.z-a.z*b.y,a.z*b.x-a.x*b.z,a.x*b.y-a.y*b.x};}

// Unique periodic longitude columns (x/width); latitude includes both poles
// (y/(height-1)). Explicit texel interpolation, no sampler-dependent offsets.
inline float sample(const std::vector<float>& data,int width,int height,float u,float v) {
    if(data.empty() || width<1 || height<1)return 0.0f;
    u-=std::floor(u); v=clamp(v,0.0f,1.0f);
    float fx=u*width,fy=v*(height-1);
    int ix=int(floor(fx)),iy=int(floor(fy));
    int nx=(ix+1)%width,ny=std::min(iy+1,height-1);
    ix=std::min(ix,width-1);
    return mix(mix(data[iy*width+ix],data[iy*width+nx],fx-ix),
               mix(data[ny*width+ix],data[ny*width+nx],fx-ix),fy-iy);
}
inline Vec3 direction(int x,int y,int width,int height) {
    double theta=(double(x)/width-0.5)*2.0*pi,phi=double(y)/(height-1)*pi;
    return {std::cos(theta)*std::sin(phi),std::cos(phi),std::sin(theta)*std::sin(phi)};
}
struct Model {
    const std::vector<float>* plate=nullptr;
    const std::vector<float>* filled=nullptr;
    const std::vector<float>* hydroBed=nullptr;
    int pw=0,ph=0,hw=0,hh=0;
    float thPlate(float u,float v)const{return plate?sample(*plate,pw,ph,u,v):0.5f;}
    float thFilled(float u,float v)const{return filled?sample(*filled,hw,hh,u,v):0.45f;}
float thHydroBed(float u,float v)const{return hydroBed&&!hydroBed->empty()?sample(*hydroBed,hw,hh,u,v):thFilled(u,v);}
#define TH_CONST const
#include "shaders/common/terrain_height_shared.glsl"
#undef TH_CONST
};
}
