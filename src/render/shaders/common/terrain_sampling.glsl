#ifndef TERRAIN_SAMPLING_GLSL
#define TERRAIN_SAMPLING_GLSL
vec4 thMap(sampler2D map, float u, float v) {
    ivec2 size=textureSize(map,0);
    vec2 st=vec2(fract(u)*float(size.x),clamp(v,0.0,1.0)*float(size.y-1));
    ivec2 a=ivec2(floor(st)); a=clamp(a,ivec2(0),size-1);
    ivec2 b=ivec2((a.x+1)%size.x,min(a.y+1,size.y-1));
    vec2 f=st-vec2(a);
    return mix(mix(texelFetch(map,a,0),texelFetch(map,ivec2(b.x,a.y),0),f.x),
               mix(texelFetch(map,ivec2(a.x,b.y),0),texelFetch(map,b,0),f.x),f.y);
}
float thPlate(float u,float v){return thMap(uTectonicMap,u,v).r;}
float thFilled(float u,float v){return thMap(uHydroMap,u,v).r;}
float thHydroBed(float u,float v){return thMap(uHydroMap,u,v).g;}
#endif
