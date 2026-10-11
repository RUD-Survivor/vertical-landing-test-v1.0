#version 450
#extension GL_GOOGLE_include_directive : enable
#include "common/solar_optics.glsl"
#include "common/solar_response.glsl"
layout(location=0) in vec2 vUV;
layout(location=1) in vec2 vNdc;
layout(location=2) flat in vec3 vTransmission;
layout(location=3) flat in vec4 vSourceBounds;
layout(set=0,binding=2) uniform sampler2D apertureResponse;
layout(set=0,binding=5) uniform sampler2D fieldResponse;
layout(std430,set=0,binding=3) readonly buffer SourceCache {
 vec4 bounds;vec4 transmission;vec4 weights[256];
} cache;
layout(location=0) out vec4 FragColor;
void main() {
 vec2 pixel=vec2(abs(dFdx(vNdc.x)),abs(dFdy(vNdc.y)));
 float response=0.0;
 if(pc.shapeType==4) {
   vec2 focal=vec2(pc.focalY/pc.aspect,pc.focalY);
   vec3 source=normalize(vec3(pc.sunScreenPos/focal,1.0));
   vec2 relativePixels=(vNdc-pc.sunScreenPos)/pixel;
   float radiusPixels=pc.focalY*pc.sinRadius/(pixel.y*max(source.z,1e-6)*max(source.z,1e-6));
   if(radiusPixels<1.0) {
     float sigma=sqrt(.65*.65+radiusPixels*radiusPixels*.225);
     if(max(abs(relativePixels.x),abs(relativePixels.y))>8.0*sigma)discard;
     float mass=solarPixelMass(relativePixels.x,sigma)*solarPixelMass(relativePixels.y,sigma);
     float cosAlpha=sqrt(max(0.0,1.0-pc.sinRadius*pc.sinRadius));
     float omega=6.28318530718*pc.sinRadius*pc.sinRadius/(1.0+cosAlpha);
     float pixelOmega=pixel.x*pixel.y*pow(source.z,3.0)/(focal.x*focal.y);
     response=mass*omega/max(pixelOmega,1e-20);
   } else {
     vec3 ray=normalize(vec3(vNdc/focal,1.0));
     float r=length(cross(ray,source))/pc.sinRadius;
     float edgeWidth=length(pixel/focal)/pc.sinRadius;
     if(dot(ray,source)<=0.0||r>1.0+edgeWidth)discard;
     // Pixel integration of the photosphere and normalized limb darkening.
     for(int y=0;y<8;++y)for(int x=0;x<8;++x) {
       vec2 ndc=vNdc+pixel*((vec2(x,y)+.5)/8.0-.5);
       vec3 sampleRay=normalize(vec3(ndc/focal,1.0));
       float q=length(cross(sampleRay,source))/pc.sinRadius;
       if(q<1.0)response+=(.4+.6*sqrt(max(0.0,1.0-q*q)))/.8;
     }
     response/=64.0;
   }
   FragColor=vec4(pc.color.rgb*pc.intensity*pc.color.a*response,0);
 } else if(pc.shapeType==6) {
   // Stable linear-HDR pupil response, convolved with observed source patches.
   // FFT is offline; runtime uses a sparse extended-source quadrature, not
   // repeated Gaussian rays. RGB wavelengths were normalized independently.
   int blades=int(pc.color.a+.5);
   if(blades<7||blades>9)discard;
   // Positive low-order off-axis response approximation. Keep the aperture
   // fixed in lens coordinates; only the aberration axes follow incidence.
   // Two asymmetric radial taps approximate coma; symmetric tangential taps
   // approximate astigmatic spreading. Zero mean avoids a false source drift.
   vec2 radial,tangent;float coma,astig;solarField(radial,tangent,coma,astig);
   float halfExtent=.20+1.5*coma+astig;
   vec3 value=vec3(0);
   for(int cell=0;cell<int(cache.transmission.a);++cell) {
     vec4 sampleData=cache.weights[cell];
     int index=int(sampleData.a+.5),sx=index%16,sy=index/16;
     vec2 source=mix(vSourceBounds.xy,vSourceBounds.zw,(vec2(sx,sy)+.5)/16.0);
     vec3 observed=sampleData.rgb;
     if(max(max(observed.r,observed.g),observed.b)<=0)continue;
     // Integrate the sensor pixel; kernel support cannot wrap into another
     // aperture plane even though the shared texture wrapper uses REPEAT.
     for(int y=0;y<2;++y)for(int x=0;x<2;++x){
       vec2 ndc=vNdc+pixel*((vec2(x,y)+.5)/2.0-.5);
       vec2 angle=(ndc-source)*vec2(pc.aspect,1)/pc.focalY;
       vec2 uv=angle/(2*halfExtent)+.5;
       if(any(lessThan(uv,vec2(.5/1024.0)))||any(greaterThan(uv,vec2(1.0-.5/1024.0))))continue;
       value+=textureLod(fieldResponse,uv,0).rgb*observed;
     }
   }
   FragColor=vec4(pc.color.rgb*pc.intensity*value/4.0,0);
 } else if(pc.shapeType==7) {
   // Aperture-image convolution with fixed defocus and chromatic magnification.
   // Positive normalized taps soften the boundary while keeping a flat interior.
   // The quad includes a 20% support guard; this is an approximate lens profile.
   vec2 local=(vNdc-pc.offset)*vec2(pc.aspect,1)/pc.scale*1.2;
   vec2 footprint=pixel*vec2(pc.aspect,1)/pc.scale*1.2;
   vec3 coverage=vec3(0);
   for(int channel=0;channel<3;++channel){
     float dispersion=channel==0?1.015:(channel==1?1.0:.985);
     for(int y=0;y<4;++y)for(int x=0;x<4;++x)
       for(int by=-1;by<=1;++by)for(int bx=-1;bx<=1;++bx){
         vec2 p=(local+footprint*((vec2(x,y)+.5)/4.0-.5)+vec2(bx,by)*.07)/dispersion;
         float a=(dot(p,p)>0?atan(p.y,p.x):0)-.1745329252;
         float sector=mod(a+3.14159265359/7,6.28318530718/7)-3.14159265359/7;
         float boundary=length(p)*cos(sector);
         float weight=float((bx==0?2:1)*(by==0?2:1))/16;
         coverage[channel]+=(boundary<=cos(3.14159265359/7)?1:0)*weight/(dispersion*dispersion);
       }
   }
   FragColor=vec4(pc.color.rgb*pc.intensity*coverage/16*vTransmission,0);
 } else {
   // Unit-energy Gaussian camera response, integrated per pixel. Quad = +/-4 sigma.
   vec2 sigma=pc.scale/vec2(pc.aspect,1.0)/4.0/pixel;
   vec2 p=(vNdc-pc.offset)/pixel;
   // Slight wavelength-dependent width, independently normalized per channel.
   // Blue is narrower; no chromatic energy gain or time-varying noise.
   vec3 spectralResponse=vec3(0);
   for(int channel=0;channel<3;++channel) {
     float dispersion=channel==0?1.0:(channel==1?.97:.94);
     vec2 width=sigma*dispersion;
     float mass=solarPixelMass(p.x,width.x)*solarPixelMass(p.y,width.y);
     spectralResponse[channel]=mass*6.28318530718*sigma.x*sigma.y;
   }
   FragColor=vec4(pc.color.rgb*pc.intensity*spectralResponse*vTransmission,0);
 }
}
