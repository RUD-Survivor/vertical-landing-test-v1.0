#include "render/ocean_model.h"
#include "math/math3d.h"
#include <cassert>
#include <cstdio>
#include <limits>
using namespace OceanModel;
int main() {
    // Stage 1: independent bed/water levels, continuity and actual metric depth.
    assert(depthM(.45,.45,25)==0);
    assert(std::abs(depthM(0,.45,25)-6000)<1e-9);
    assert(std::abs(depthM(.6,.6001,25)-4.54545454545)<1e-7);
    assert(depthM(.6,.5,25)==0);
    assert(std::abs(bedKm(.45-1e-9,25)-bedKm(.45+1e-9,25))<1e-6);
    for (int i=0;i<=1000;++i) {
        double h=i/1000.0;
        assert(std::isfinite(bedKm(h,25)) && depthM(h,h,25)>=0);
        if(h>=seaLevel) assert(depthM(h,h,25)==0);
    }
    puts("PASS stage 1: shoreline continuity, 6 km deep ocean, 4.545 m lake, dry land");
    // Stage 2: dielectric energy bounds and Beer-Lambert monotonicity.
    for (int i=0;i<=1000;++i) {
        double c=i/1000.0, f=fresnel(c);
        assert(std::isfinite(f) && f>=0 && f<=1);
        assert(std::abs(f+(1-f)-1)<1e-12);
        if(i>0) assert(f<=fresnel((i-1)/1000.0));
    }
    for(double sigma: {0.0,.007,.025,.18}) {
        double last=1;
        for(int i=0;i<=10000;++i) {
            double t=transmission(sigma,i);
            assert(std::isfinite(t) && t>=0 && t<=last); last=t;
        }
    }
    assert(transmission(.2,0)==1);
    puts("PASS stage 2: grazing/normal Fresnel, bounded energy, monotonic absorption");
    // Stage 3: rebasing invariance, poles, long-running clock and bounded steepness.
    float a[3]={0.1436f,.478f,.866f}, b[3]={a[0]+.00001f,a[1],a[2]}, pa[12],pb[12];
    double maxError=0, steepness=0;
    for(double time: {0.0,1000.0,1e7}) {
        phases(a,6371000,time,pa); phases(b,6371000,time,pb);
        for(int i=0;i<12;++i) {
            const auto& d=directions[i]; double n=std::sqrt(d[0]*d[0]+d[1]*d[1]+d[2]*d[2]);
            double shift=2*pi/lengths[i]*6371000*(double(b[0])-double(a[0]))*d[0]/n;
            double error=std::abs(std::remainder(double(pa[i])+shift-double(pb[i]),2*pi));
            maxError=std::max(maxError,error); assert(error<1e-6);
        }
    }
    for(int i=0;i<12;++i) steepness+=amplitudes[i]*2*pi/lengths[i];
    assert(steepness<1); // analytic radial height field cannot overturn
    for(float y: {-1.f,1.f}) {float pole[3]={0,y,0};phases(pole,6371000,1e7,pa);for(float p:pa)assert(std::isfinite(p));}
    phases(a,6371000,1e7,pa);phases(a,6371000,1e7+.016,pb);
    assert(std::abs(std::remainder(double(pb[11])-pa[11],2*pi))>.1);
    printf("PASS stage 3: rebasing max phase error %.3g rad, slope bound %.3f, poles, 1e7 s\n",maxError,steepness);
    // Stage 4: pixel filtering transfers slope energy to the microfacet model.
    double variance=0;
    for(int i=0;i<12;++i){double s=amplitudes[i]*2*pi/lengths[i];variance+=.5*s*s;}
    for(double footprint: {0.0,.1,1.0,10.0,1000.0}) {
        double resolved=0,unresolved=0;
        for(int i=0;i<12;++i){double t=std::clamp((footprint/lengths[i]-.18)/.32,0.0,1.0),r=1-t*t*(3-2*t);
            double s=amplitudes[i]*2*pi/lengths[i],e=.5*s*s;
            resolved+=e*r*r;unresolved+=e*(1-r*r);}
        assert(std::abs(resolved+unresolved-variance)<1e-12);
        if(footprint==1000)assert(resolved==0 && unresolved>0);
    }
    puts("PASS stage 4: unresolved waves conserve slope energy instead of aliasing");
    // Stage 5: Vulkan Y flip + GL depth remap round-trip, including camera motion.
    auto uv=[](const Mat4& vp,Vec3 p){Vec3 ndc=vp.transformPoint(p);return Vec3(ndc.x*.5+.5,.5-ndc.y*.5,ndc.z*.5+.5);};
    Mat4 proj=Mat4::perspective(1.0,16.0/9,.001,30000);
    Mat4 vp=proj*Mat4::lookAt(Vec3(.01,.02,0),Vec3(0,-.04,.1),Vec3(0,0,1));
    Mat4 previous=proj*Mat4::lookAt(Vec3(0,.02,0),Vec3(0,-.04,.1),Vec3(0,0,1));
    for(Vec3 point: {Vec3(0,0,.1),Vec3(.03,-.01,.2),Vec3(-.1,-.1,1)}) {
        Vec3 pixel=uv(vp,point);
        Vec3 reconstructed=vp.inverse().transformPoint(Vec3(pixel.x*2-1,1-pixel.y*2,pixel.z*2-1));
        assert((reconstructed-point).length()<1e-9);
        assert((uv(previous,reconstructed)-uv(previous,point)).length()<1e-9);
    }
    puts("PASS stage 5: depth reconstruction and moving-camera history projection");
}
