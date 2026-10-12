// Compiled by BOTH C++ and GLSL. Sampling adapters live in terrain_height.h
// and terrain_sampling.glsl. Heights are dimensionless until converted to km.
// Do not duplicate this formula in collision, water or shadow consumers.
#ifndef TH_CONST
#define TH_CONST
#endif

float thSmooth(float a, float b, float x) TH_CONST {
    float t = clamp((x-a)/(b-a), 0.0f, 1.0f);
    return t*t*(3.0f-2.0f*t);
}
float thU(vec3 p) TH_CONST { return abs(p.x)+abs(p.z)<1e-12f ? 0.0f : atan(p.z,p.x)/(2.0f*3.141592653589793f)+0.5f; }
float thV(vec3 p) TH_CONST { return atan(length(vec3(p.x,0.0f,p.z)),p.y)/3.141592653589793f; }
// Integer lattice hash avoids the divergent float fract hash on CPU/GPU.
float thHash(int x, int y, int z) TH_CONST {
    uint h = uint(x)*1597334677u ^ uint(y)*3812015801u ^ uint(z)*2798796415u;
    h = (h ^ (h >> 16u))*2246822519u;
    h = (h ^ (h >> 13u))*3266489917u;
    h ^= h >> 16u;
    return float(h >> 8u)*(1.0f/16777216.0f);
}
float thNoise(vec3 p) TH_CONST {
    int x=int(floor(p.x)), y=int(floor(p.y)), z=int(floor(p.z));
    vec3 f=p-vec3(float(x),float(y),float(z));
    float a=thSmooth(0.0f,1.0f,f.x), b=thSmooth(0.0f,1.0f,f.y), c=thSmooth(0.0f,1.0f,f.z);
    return mix(mix(mix(thHash(x,y,z),thHash(x+1,y,z),a),
                   mix(thHash(x,y+1,z),thHash(x+1,y+1,z),a),b),
               mix(mix(thHash(x,y,z+1),thHash(x+1,y,z+1),a),
                   mix(thHash(x,y+1,z+1),thHash(x+1,y+1,z+1),a),b),c);
}
float thFbm(vec3 p, int octaves) TH_CONST {
    float v=0.0f, a=0.5f;
    for(int i=0;i<octaves;++i) {
        v+=thNoise(p)*a;
        p=p*2.15f+vec3(0.131f,-0.217f,0.344f); a*=0.47f;
    }
    return v;
}
float thMountains(vec3 p) TH_CONST {
    vec3 off=vec3(thFbm(p*2.0f,3),thFbm(p*2.0f+vec3(5.2f,1.3f,0.7f),3),
                  thFbm(p*2.0f+vec3(2.7f,8.4f,3.1f),3));
    p=p+off*0.15f;
    float v=0.0f,a=0.5f,f=1.05f;
    for(int i=0;i<6;++i) {
        float n=1.0f-abs(thNoise(p*f)*2.0f-1.0f);
        v+=n*n*a; f*=2.07f; a*=0.48f;
    }
    return v;
}
float thBedKm(float h, float maxElevation) TH_CONST {
    return h<0.45f ? (h-0.45f)/0.45f*6.0f : (h-0.45f)/0.55f*maxElevation;
}
float thElevation(float km, float maxElevation) TH_CONST {
    return km<0.0f ? 0.45f+km/6.0f*0.45f : 0.45f+km/maxElevation*0.55f;
}
float thRegional(vec3 p) TH_CONST {
    float u=thU(p),v=thV(p),base=thPlate(u,v),h=base;
    float mountain=thSmooth(0.55f,0.75f,base);
    if(mountain>0.0f) {
        // Fixed sphere-space anisotropy is continuous at seams and poles.
        // Rotating noise domains by a noisy gradient angle creates discontinuities.
        h+=thMountains(vec3(p.x*0.72f,p.y,p.z*1.28f)*6.5f)*0.18f*mountain;
        h-=thSmooth(0.78f,0.96f,h)*0.045f;
    }
    float coast=thSmooth(0.435f,0.445f,base)*(1.0f-thSmooth(0.455f,0.465f,base));
    float cliff=1.0f-abs(thNoise(p*180.0f)*1.5f-0.5f);
    h+=thSmooth(0.6f,0.9f,cliff)*0.007f*coast;
    if(h<0.45f && h>0.38f) h=mix(0.38f,0.45f,thSmooth(0.38f,0.45f,h));
    return clamp(h,0.0f,1.0f);
}
float thHeight(vec3 p, float radius, float maxElevation, float detailWeight) TH_CONST {
    float h=thRegional(p),km=thBedKm(h,maxElevation);
    if(h>=0.45f && detailWeight>0.0f) {
        // <= 15 cm, >= 32 m wavelength. Micro detail belongs in shading;
        // meter-high noise at sub-meter frequency aliases on every coarse mesh.
        km+=(thNoise(p*(radius/0.032f))-0.5f)*0.0003f*detailWeight;
    }
    vec3 ksc=normalize(vec3(0.1436f,0.478f,0.866f));
    float pad=1.0f-thSmooth(0.02f,0.08f,length(p-ksc));
    return mix(km,0.005f,pad);
}
float thWater(vec3 p, float radius, float maxElevation, float bedKm) TH_CONST {
    if(bedKm<0.0f) return 0.0f;
    float u=thU(p),v=thV(p),filled=thFilled(u,v),reference=thHydroBed(u,v);
    float depth=max(0.0f,thBedKm(filled,maxElevation)-thBedKm(reference,maxElevation));
    // Filled absolute heights alone turn unresolved hills into false lakes.
    // Dry cells remain dry, and unresolved water cannot exceed physical depth.
    if(depth<=0.00002f)return bedKm;
    return max(bedKm,min(thBedKm(filled,maxElevation),bedKm+depth));
}
vec3 thNormal(vec3 p, float radius, float maxElevation, float stepKm) TH_CONST {
    vec3 ref=abs(p.y)>0.95f?vec3(1.0f,0.0f,0.0f):vec3(0.0f,1.0f,0.0f);
    vec3 t=normalize(cross(ref,p)),b=cross(p,t);
    float eps=max(stepKm,0.032f)/radius;
    vec3 p0=normalize(p-t*eps),p1=normalize(p+t*eps);
    vec3 q0=normalize(p-b*eps),q1=normalize(p+b*eps);
    vec3 dt=(p1-p0)*radius+p1*thHeight(p1,radius,maxElevation,0.0f)-p0*thHeight(p0,radius,maxElevation,0.0f);
    vec3 db=(q1-q0)*radius+q1*thHeight(q1,radius,maxElevation,0.0f)-q0*thHeight(q0,radius,maxElevation,0.0f);
    return normalize(cross(dt,db));
}
