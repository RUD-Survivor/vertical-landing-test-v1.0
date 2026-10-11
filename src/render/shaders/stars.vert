#version 450
#extension GL_GOOGLE_include_directive : enable
#include "common/sky_params.glsl"
struct Star {vec4 directionFlux;vec4 color;};
layout(set=1,binding=0,std430) readonly buffer Stars {Star catalog[];};
layout(location=0) noperspective out vec2 offsetPixels;
layout(location=1) flat out vec3 radiance;
layout(location=2) flat out float sigma;
layout(location=3) flat out float haloFraction;
void main() {
    Star star=catalog[gl_InstanceIndex];
    // This draw pushes translation-free projection*view into the matrix slot.
    vec4 center=pc.invViewProj*vec4(star.directionFlux.xyz,0.0);
    vec2 corners[6]=vec2[](vec2(-1,-1),vec2(1,-1),vec2(1,1),vec2(-1,-1),vec2(1,1),vec2(-1,1));
    sigma=clamp(pc.starSize,0.4,2.0);haloFraction=clamp(pc.glowStrength,0.0,0.15);
    float radius=ceil(8.0*sigma+1.0);offsetPixels=corners[gl_VertexIndex]*radius;
    vec2 viewport=max(vec2(pc._pad0,pc._pad1),vec2(1.0));
    // Solid angle of a screen pixel at this direction. Clip w is -view z.
    float cosine=max(center.w,0.00001);
    vec3 rowX=vec3(pc.invViewProj[0][0],pc.invViewProj[1][0],pc.invViewProj[2][0]);
    vec3 rowY=vec3(pc.invViewProj[0][1],pc.invViewProj[1][1],pc.invViewProj[2][1]);
    float solidAngle=4.0*pow(cosine,3.0)/(viewport.x*viewport.y*length(rowX)*length(rowY));
    vec3 color=mix(vec3(1.0),star.color.rgb,clamp(pc.colorVariety,0.0,1.0));
    radiance=color*(2e-7*star.directionFlux.w*max(pc.starBrightness,0.0)*max(pc.exposure,0.0)*pc.skyVibrancy)/max(solidAngle,1e-15);
    center.y=-center.y;
    center.xy+=offsetPixels*2.0/viewport*center.w;
    // Reject sources behind the camera, avoiding giant near-plane quads.
    gl_Position=center.w>0.00001?vec4(center.xy,0.9999*center.w,center.w):vec4(2,2,1,1);
}
