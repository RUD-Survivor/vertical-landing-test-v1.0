#version 450
#extension GL_GOOGLE_include_directive : enable
#include "common/sky_params.glsl"
layout(set=1,binding=1) uniform sampler2D milkyWay;
layout(location=0) in vec3 vRayDir;
layout(location=0) out vec4 FragColor;
void main() {
    vec3 rd=normalize(vRayDir);
    float cy=cos(pc.galaxyYaw),sy=sin(pc.galaxyYaw),cp=cos(pc.galaxyPitch),sp=sin(pc.galaxyPitch);
    rd=vec3(cy*rd.x+sy*rd.z,rd.y,-sy*rd.x+cy*rd.z);
    rd=vec3(rd.x,cp*rd.y-sp*rd.z,sp*rd.y+cp*rd.z);
    // NASA celestial map: RA zero at center, increases to the left.
    const float PI=3.141592653589793;
    float rho2=max(dot(rd.xz,rd.xz),1e-10);
    float ra=dot(rd.xz,rd.xz)>1e-10?atan(rd.z,rd.x):0.0; // atan(0,0) is undefined.
    vec2 uv=vec2(.5-ra/(2.0*PI),.5-asin(clamp(rd.y,-1.0,1.0))/PI);
    // Differentiate continuous directions: wrapped UVs break seam filtering.
    vec3 dx=dFdx(rd),dy=dFdy(rd);
    vec3 raGradient=vec3(-rd.z,0,rd.x)/rho2;
    vec2 gx=vec2(-dot(raGradient,dx)/(2.0*PI),-dx.y/(sqrt(rho2)*PI));
    vec2 gy=vec2(-dot(raGradient,dy)/(2.0*PI),-dy.y/(sqrt(rho2)*PI));
    vec3 col=textureGrad(milkyWay,uv,gx,gy).rgb*max(pc.milkyWayIntensity,0.0);
    col+=vec3(1.5e-7,1.8e-7,2.3e-7)*max(pc.bgLevel,0.0);
    FragColor=vec4(col*max(pc.skyVibrancy,0.0)*max(pc.exposure,0.0),1.0);
}
