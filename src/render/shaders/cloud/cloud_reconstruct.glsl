#version 460
#extension GL_GOOGLE_include_directive : enable
#extension GL_EXT_samplerless_texture_functions : enable
#include "cloud_common.glsl"
#define BLUE_NOISE_BUFFER_SET 2
#include "../common/shared_blue_noise.glsl"
layout(local_size_x=8,local_size_y=8) in;
vec3 cloudViewRay(vec2 uv) {
    vec4 p=frameData.camInvertProj*vec4(uv.x*2.0-1.0,1.0-uv.y*2.0,0.0,1.0);
    return normalize((frameData.camInvertView*vec4(p.xyz,0.0)).xyz);
}
void main() {
    ivec2 size=imageSize(imageCloudReconstructionTexture);
    ivec2 pixel=ivec2(gl_GlobalInvocationID.xy);
    if(any(greaterThanEqual(pixel,size))) return;
    vec2 uv=(vec2(pixel)+0.5)/vec2(size);
    vec3 ray=cloudViewRay(uv);
    vec3 origin=convertToAtmosphereUnit(frameData.camWorldPos.xyz,frameData);
    AtmosphereParameters atmosphere=getAtmosphereParameters(frameData);
    float outer=atmosphere.cloudAreaStartHeight+atmosphere.cloudAreaThickness;
    float inner=atmosphere.cloudAreaStartHeight;
    float impact=length(cross(origin,ray));
    // Measure impact-distance derivatives rather than using camera radius:
    // a wide-angle low-orbit view otherwise makes most of the disk a "limb".
    float impactX=length(cross(origin,cloudViewRay(uv+vec2(1.0/size.x,0))));
    float impactY=length(cross(origin,cloudViewRay(uv+vec2(0,1.0/size.y))));
    float footprint=max(abs(impactX-impact)+abs(impactY-impact),0.001);
    bool orbital=length(origin)>outer;
    bool outside=orbital && (dot(origin,ray)>=0.0 || impact>outer+footprint);
    // Both shell tangencies matter: crossing the inner sphere changes the
    // trace interval too, and is almost coincident with the Earth silhouette.
    bool limb=orbital && min(abs(impact-outer),abs(impact-inner))<4.0*footprint;
    vec4 color=vec4(0,0,0,1),fog=vec4(0,0,0,-1),airT=vec4(1);
    float distanceKm=1e20;
    uint bayer=frameData.frameIndex.x%16u;
    ivec2 offset=ivec2(kBayerMatrix16[bayer]%4,kBayerMatrix16[bayer]/4);
    bool evaluated=all(equal(pixel%4,offset));
    bool geometricCoverage=true;
    if(!outside) {
        color=texelFetch(inCloudRenderTexture,pixel/4,0);
        fog=texelFetch(inCloudFogRenderTexture,pixel/4,0);
        airT=texelFetch(inCloudAirTRenderTexture,pixel/4,0);
        distanceKm=texelFetch(inCloudDepthTexture,pixel/4,0).r;
        if(limb) {
            // Opposite sides of the shell cannot share a Bayer sample.
            // Four subpixel rays are needed only at the actual silhouette;
            // the surrounding Bayer-discontinuity band needs one full ray.
            color=vec4(0);fog=vec4(0);airT=vec4(0);
            float distanceSum=0.0,hitCount=0.0;
            geometricCoverage=false;
            int samples=min(abs(impact-outer),abs(impact-inner))<footprint?4:1;
            float sampleWeight=1.0/float(samples);
            for(int s=0;s<samples;++s) {
                vec2 sub=samples==4?(vec2(s%2,s/2)+0.5)*0.5:vec2(0.5);
                vec2 sampleUv=(vec2(pixel)+sub)/vec2(size);
                vec3 subRay=cloudViewRay(sampleUv);
                geometricCoverage=geometricCoverage || (dot(origin,subRay)<0.0 && length(cross(origin,subRay))<=outer);
                float sampleDistance=1e20;
                vec4 sampleFog=vec4(0);vec3 sampleAir=vec3(1);
                float noise=samplerBlueNoiseErrorDistribution_128x128_OptimizedFor_2d2d2d2d(
                    uint(pixel.x),uint(pixel.y),0u,uint(s));
                color+=sampleWeight*cloudColorCompute(atmosphere,sampleUv,noise,sampleDistance,pixel,subRay,
                    frameData.sky.atmosphereConfig.cloudGodRay!=0,sampleFog,noise,sampleAir);
                fog+=sampleWeight*sampleFog;airT+=sampleWeight*vec4(sampleAir,1);
                if(sampleDistance<1e19){distanceSum+=sampleDistance;hitCount+=1.0;}
            }
            distanceKm=hitCount>0.0?distanceSum/hitCount:1e20;
            evaluated=true;
        }
        // Finite shell anchor avoids inversion of quantized far depth (w=0).
        // Depth RTs contain ray distances in km, including a no-hit sentinel.
        float anchor=distanceKm<1e19?distanceKm:max(-dot(origin,ray),0.0);
        vec3 camera=(frameData.camInvertView*vec4(0,0,0,1)).xyz;
        // Shape samples use p+wind*time: the same material point in the
        // previous frame is p+wind*dt (the positive sign is intentional).
        vec3 wind=(frameData.sky.atmosphereConfig.cloudDirection+vec3(0,0.1,0))
            *frameData.sky.atmosphereConfig.cloudSpeed;
        vec3 previousPoint=camera+ray*anchor+wind*frameData.appTime.w;
        vec4 previous=frameData.camViewProjPrev*vec4(previousPoint,1);
        vec2 previousUv=previous.xy/previous.w*0.5+0.5;
        previousUv.y=1.0-previousUv.y;
        bool valid=frameData.bCameraCut==0 && previous.w>0.0
            && geometricCoverage && onRange(previousUv,vec2(0),vec2(1));
        if(valid) {
            vec4 history=texture(sampler2D(inCloudReconstructionTextureHistory,linearClampEdgeSampler),previousUv);
            vec4 oldFog=texture(sampler2D(inCloudFogReconstructionTextureHistory,linearClampEdgeSampler),previousUv);
            vec4 oldAir=texture(sampler2D(inCloudAirTReconstructionTextureHistory,linearClampEdgeSampler),previousUv);
            float oldDistance=texture(sampler2D(inCloudDepthReconstructionTextureHistory,pointClampEdgeSampler),previousUv).r;
            float motionPixels=length((previousUv-uv)*vec2(size));
            float expectedDistance=length(previousPoint-frameData.cloudHistoryCameraPrev.xyz);
            float depthTolerance=max(2.0,2.0*anchor/float(size.y));
            if(evaluated && !limb && ((distanceKm<1e19)!=(oldDistance<1e19)
                || (distanceKm<1e19 && abs(oldDistance-expectedDistance)>depthTolerance)))valid=false;
            if(limb && frameData.sky.atmosphereConfig.cloudDensity<=0.0
                && frameData.sky.atmosphereConfig.hiCloudCoverage<=0.0)valid=false;
            if(!evaluated) {
                // The current sparse neighborhood is evidence, not a substitute
                // for this pixel's ray. Reject an unsupported history sample
                // and trace it, instead of spreading another cloud column.
                vec4 lo=vec4(1e20),hi=vec4(-1e20);
                float nearDepth=1e20,farDepth=0.0;
                ivec2 coarseSize=textureSize(inCloudRenderTexture,0);
                for(int y=-1;y<=1;++y)for(int x=-1;x<=1;++x){
                    ivec2 q=clamp(pixel/4+ivec2(x,y),ivec2(0),coarseSize-1);
                    vec4 sampleColor=texelFetch(inCloudRenderTexture,q,0);
                    lo=min(lo,sampleColor);hi=max(hi,sampleColor);
                    float sampleDepth=texelFetch(inCloudDepthTexture,q,0).r;
                    if(sampleDepth<1e19){nearDepth=min(nearDepth,sampleDepth);farDepth=max(farDepth,sampleDepth);}
                }
                vec4 tolerance=vec4(max(vec3(0.02),(hi.rgb-lo.rgb)*0.05),0.005);
                if(any(lessThan(history,lo-tolerance)) || any(greaterThan(history,hi+tolerance))
                    || motionPixels>0.25)valid=false;
                if(oldDistance<1e19 && (nearDepth>=1e19
                    || oldDistance<nearDepth-depthTolerance || oldDistance>farDepth+depthTolerance))valid=false;
            }
            // Clear evaluated rays and depth disocclusions must not retain old
            // extinction. Limb rays are already fully evaluated above.
            if(evaluated) {
                // Bayer is a scheduling period, not a temporal filter weight.
                // A fresh sparse sample replaces history. At the fully traced
                // limb, constrain spatial lag instead of using sparse point
                // bounds to clip a subpixel-integrated coverage estimate.
                if(valid && limb){
                    // For constant motion m, EMA lag=m*(1-a)/a. Choosing
                    // a>=4*m bounds lag below 0.25 pixel; >=0.25 px/frame is
                    // current-only. The minimum is a noise floor, not Bayer.
                    float currentWeight=clamp(max(0.0625,4.0*motionPixels),0.0,1.0);
                    color=mix(history,color,currentWeight);
                    fog=mix(oldFog,fog,currentWeight);
                    airT=mix(oldAir,airT,currentWeight);
                }
            } else if(valid) {
                color=history;fog=oldFog;airT=oldAir;
                distanceKm=oldDistance;
            }
        }
        if(!valid && !limb && !evaluated) {
            // Cold/disoccluded pixels need their own ray. Bilinear bootstrap
            // mixes distinct cloud columns and creates a new 4x4 pattern as
            // Bayer replaces those estimates over the next sixteen frames.
            float noise=samplerBlueNoiseErrorDistribution_128x128_OptimizedFor_2d2d2d2d(
                uint(pixel.x),uint(pixel.y),0u,0u);
            vec3 currentAir=vec3(1);distanceKm=1e20;
            color=cloudColorCompute(atmosphere,uv,noise,distanceKm,pixel,ray,
                frameData.sky.atmosphereConfig.cloudGodRay!=0,fog,noise,currentAir);
            airT=vec4(currentAir,1);
        }
    }
    // Bootstrap/filtered coverage can contain a cloud even when the point
    // sample has a no-hit sentinel. Keep visibility consistent with (L,T),
    // using the front shell as a conservative representative anchor.
    if(distanceKm>=1e19 && color.a<0.999 && dot(origin,ray)<0.0 && impact<outer)
        distanceKm=max(-dot(origin,ray)-sqrt(max(outer*outer-impact*impact,0.0)),0.0);
    imageStore(imageCloudReconstructionTexture,pixel,color);
    imageStore(imageCloudFogReconstructionTexture,pixel,fog);
    imageStore(imageCloudAirTReconstructionTexture,pixel,airT);
    imageStore(imageCloudDepthReconstructionTexture,pixel,vec4(distanceKm));
}
