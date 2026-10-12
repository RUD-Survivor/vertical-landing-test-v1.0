#version 450
#extension GL_GOOGLE_include_directive : enable
#include "common/shared_cloud_shadow.glsl"
#include "common/shared_atmosphere.glsl"
#include "common/ocean_waves.glsl"
#include "common/water_surface.glsl"
layout(set=0,binding=0) uniform FrameUBO {
    mat4 view; mat4 proj; vec3 lightDir; float _pad0; vec3 viewPos; float time;
} frame;
layout(set=0,binding=2) uniform sampler2D uCloudShadowAtlas;
layout(set=0,binding=3,std140) uniform CloudShadowUBO { CloudShadowData data; } uCloudShadow;
layout(push_constant) uniform PC {
    mat4 model; float planetRadius; float maxElevation; int nodeLevel; int anchorGeometry;
    vec3 nodePos; float lodMorph; vec3 nodeSide; float skirtDepthKm; vec3 nodeUp; float detailWeight;
} pc;
layout(set=2,binding=0) uniform sampler2D sceneColor;
layout(set=2,binding=1) uniform sampler2D sceneDepth; // opaque snapshot, never output depth
layout(set=2,binding=2) uniform sampler2D transmittanceLut;
layout(set=2,binding=5) uniform sampler3D oceanSkyLut;
layout(set=2,binding=6) uniform sampler2D oceanIrradianceLut;
layout(location=0) in vec3 vRelViewPos;
layout(location=1) in vec3 vNormal;
layout(location=2) in vec2 vUV;
layout(location=3) in vec2 vLocalUV;
layout(location=4) in vec3 vLocalPos;
layout(location=5) in float vElevation;
layout(location=6) in float vWaterDepth; // meters, computed before flood fill
layout(location=7) in float vSlope;
layout(location=0) out vec4 FragColor;
layout(location=1) out float Reactive;

vec3 sunTransmission(float r, float mu) {
    // A ground-intersecting solar ray receives no direct light.
    float horizon = -sqrt(max(0.0, 1.0 - pc.planetRadius*pc.planetRadius/(r*r)));
    if (mu < horizon) return vec3(0);
    AtmosphereParameters a;
    a.bottomRadius=pc.planetRadius; a.topRadius=ocean.sunRadius.w;
    vec2 uv; lutTransmittanceParamsToUv(a, r, mu, uv);
    return clamp(textureLod(transmittanceLut, uv, 0).rgb,0.0,1.0);
}

// Profile-baked surface sky (mu, solar elevation, relative azimuth). It is not
// camera SkyView: orbit pixels use their own local up and sun elevation.
vec3 skyReflection(vec3 up, vec3 ray, vec3 L) {
    float mu=dot(up,ray);
    float mus=clamp(dot(up,L),-1.0,1.0);
    if(mus < -0.2) return vec3(0);
    if(mu < 0.0) return vec3(0); // off-screen ground has no radiance source
    float den=sqrt(max(0.0,1-mu*mu))*sqrt(max(0.0,1-mus*mus));
    float cosAz=den>1e-5?clamp((dot(ray,L)-mu*mus)/den,-1.0,1.0):0.0;
    vec3 coord=vec3(sqrt(clamp(mu,0,1)),clamp((mus+0.2)/1.2,0,1),cosAz*0.5+0.5);
    vec3 size=vec3(textureSize(oceanSkyLut,0));
    coord=(coord*(size-1)+0.5)/size;
    return textureLod(oceanSkyLut,coord,0).rgb*ocean.params.x*ocean.params.y;
}
vec3 skyIrradiance(vec3 up,vec3 L,bool enteredWater) {
    float mus=dot(up,L);
    if(mus < -0.2)return vec3(0);
    float x=clamp((mus+0.2)/1.2,0,1);
    float width=float(textureSize(oceanIrradianceLut,0).x);
    return textureLod(oceanIrradianceLut,vec2((x*(width-1)+0.5)/width,enteredWater?0.75:0.25),0).rgb*ocean.params.x;
}

bool projectPoint(vec3 p, out vec2 uv) {
    vec4 c=frame.proj*frame.view*vec4(p,1);
    if(c.w<=0.00001) return false;
    uv=vec2(c.x,-c.y)/c.w*0.5+0.5;
    return all(greaterThan(uv,vec2(0.002))) && all(lessThan(uv,vec2(0.998)));
}
vec3 reconstruct(vec2 uv, float depth) {
    vec4 p=ocean.invViewProj*vec4(uv.x*2.0-1.0,1.0-uv.y*2.0,depth*2.0-1.0,1);
    return p.xyz / (abs(p.w)>1e-8 ? p.w : 1e-8);
}
vec3 reflectionScene(vec3 P, vec3 R, vec3 fallback, float roughness) {
    // SSR uses opaque input only. At orbit distances or rough surfaces use sky.
    if(length(vRelViewPos)>20.0 || roughness>0.45) return fallback;
    float previousDelta=1e6, previousT=0.001;
    for(int i=0;i<16;++i) {
        float t=0.002*pow(1.65,float(i)); // km; bounded to ~3 m..3.7 km
        vec3 Q=P+R*t; vec2 uv;
        if(!projectPoint(Q,uv)) break;
        float z=textureLod(sceneDepth,uv,0).r;
        if(z>=0.999999) {previousDelta=1e6;previousT=t;continue;}
        vec3 hit=reconstruct(uv,z);
        float delta=-(frame.view*vec4(hit,1)).z+(frame.view*vec4(Q,1)).z;
        if(delta<=0.0 && previousDelta>0.0) {
            float lo=previousT,hi=t;
            for(int j=0;j<4;++j) {
                float mid=(lo+hi)*0.5; vec2 mUV;
                if(!projectPoint(P+R*mid,mUV)) break;
                vec3 mHit=reconstruct(mUV,textureLod(sceneDepth,mUV,0).r);
                float dz=-(frame.view*vec4(mHit,1)).z+(frame.view*vec4(P+R*mid,1)).z;
                if(dz>0.0)lo=mid;else hi=mid;
            }
            vec2 hitUV; if(!projectPoint(P+R*hi,hitUV)) break;
            vec3 actual=reconstruct(hitUV,textureLod(sceneDepth,hitUV,0).r);
            // Reject a crossing caused by silhouette discontinuity / near occluder.
            if(length(actual-(P+R*hi))>0.03+hi*0.015) break;
            float edge=min(min(hitUV.x,hitUV.y),min(1-hitUV.x,1-hitUV.y));
            float weight=smoothstep(0.0,0.08,edge)*(1.0-smoothstep(0.15,0.45,roughness));
            return mix(fallback,textureLod(sceneColor,hitUV,0).rgb,weight);
        }
        previousDelta=delta; previousT=t;
    }
    return fallback;
}

void main() {
    if(vWaterDepth<=0.02) discard;
    vec3 up=normalize(vNormal), local=normalize(vLocalPos);
    float footprint=max(length(dFdx(local)),length(dFdy(local)))*ocean.anchorRadius.w;
    OceanWaves waves=oceanWaves(local,ocean.anchorRadius.w,frame.time,footprint,vWaterDepth);
    int debugView=int(ocean.ozoneShape.y);
    if(debugView==10){waves.slope=vec3(0);waves.crest=0;}
    vec3 worldSlope=mat3(pc.model)/pc.planetRadius*waves.slope;
    vec3 N=normalize(up-worldSlope), V=normalize(-vRelViewPos), L=normalize(ocean.sunRadius.xyz);
    bool farWater=length(pc.model[3].xyz-frame.viewPos)-pc.planetRadius>20.0;
    // Analytic near-sphere depth belongs to the visible hemisphere. Backside
    // patches must not reuse it and overwrite the front with backside lighting.
    if(farWater) {
        // Use the triangle's geometric facing at the limb. Interpolated radial
        // normals can cross the horizon inside an otherwise visible triangle.
        vec3 faceNormal=cross(dFdx(vRelViewPos),dFdy(vRelViewPos));
        faceNormal*=dot(faceNormal,up)<0.0?-1.0:1.0;
        if(dot(faceNormal,V)<=0.0)discard;
    }
    // Back-facing underwater path: reciprocal IOR, bounded total internal reflection.
    bool underwater=dot(up,V)<0;
    if(underwater)N=-N;
    float NoV=max(dot(N,V),0.0001), NoL=max(dot(N,L),0.0);
    // GGX alpha=perceptualRoughness^2. Add unresolved slope variance
    // to alpha^2, not perceptualRoughness^2 (which would under-filter glints).
    float roughness=clamp(pow(pow(0.08,4.0)+waves.variance,0.25),0.08,0.65);
    float alpha=roughness*roughness;
    // Solar angular radius prevents singular point-light highlights.
    alpha=sqrt(alpha*alpha+0.00465*0.00465);
    vec3 hv=L+V; vec3 H=hv/max(length(hv),0.00001);
    float NoH=max(dot(N,H),0), VoH=max(dot(V,H),0);
    float a2=alpha*alpha, den=NoH*NoH*(a2-1.0)+1.0;
    float D=a2/(OCEAN_PI*max(den*den,1e-12));
    float gv=2*NoV/(NoV+sqrt(a2+(1-a2)*NoV*NoV));
    float gl=2*NoL/(NoL+sqrt(a2+(1-a2)*NoL*NoL)+1e-6);
    const float F0=0.020373;
    float F=F0+(1-F0)*pow(1-NoV,5);
    float Fsun=F0+(1-F0)*pow(1-VoH,5);
    vec3 P=frame.viewPos+vRelViewPos;
    float sunVis=cloudShadowSunVisibility(uCloudShadow.data,uCloudShadowAtlas,P-pc.model[3].xyz);
    vec3 sunT=sunTransmission(pc.planetRadius+0.005,dot(up,L));
    vec3 sunColor=vec3(ocean.params.x)*sunT;
    vec3 direct=sunColor*(D*gv*gl*Fsun/max(4*NoV,1e-5))*sunVis;
    vec3 R=reflect(-V,N);
    vec3 reflectedSky=skyReflection(up,R,L);
    vec3 environment=debugView==9?reflectedSky:reflectionScene(P,R,reflectedSky,roughness);
    vec2 uv=gl_FragCoord.xy/vec2(textureSize(sceneDepth,0));
    vec3 sigma=max(ocean.absorption.rgb+ocean.scattering.rgb,vec3(1e-6));
    vec3 refracted=refract(-V,N,underwater?1.333:1.0/1.333);
    if(dot(refracted,refracted)<0.01) {F=1.0;refracted=-V;}
    float pathM=vWaterDepth/max(abs(dot(refracted,up)),0.15);
    vec3 bottom=vec3(0);
    // Deep water is optically opaque; skip background reads and refraction work.
    if(pathM<100.0) {
        vec2 refrUV;
        if(farWater)refrUV=uv; // orbital depth cannot resolve a meter-scale water column
        else if(!projectPoint(P+refracted*min(pathM,30.0)*0.001,refrUV))refrUV=uv;
        float depth=textureLod(sceneDepth,refrUV,0).r;
        vec3 bed=reconstruct(refrUV,depth);
        // Never refract foreground geometry into the water.
        if(farWater || depth>=0.999999 || dot(bed-P,up)>0.002)refrUV=uv;
        else pathM=min(pathM,max(0.0,length(bed-P)*1000.0));
        bottom=textureLod(sceneColor,refrUV,0).rgb;
    }
    vec3 T=exp(-sigma*max(pathM,0.0));
    vec3 skyDown=skyReflection(up,up,L);
    // Isotropic in-water source uses actual cosine-integrated irradiance.
    // Dielectric entry transmission precedes scattering; view Fresnel follows it.
    vec3 incidentSky=skyIrradiance(up,L,false);
    float sunCos=max(dot(up,L),0.0);
    float sunEntry=1-(F0+(1-F0)*pow(1-sunCos,5));
    vec3 bodyLight=(skyIrradiance(up,L,true)+sunColor*sunCos*sunVis*sunEntry)/(4.0*OCEAN_PI);
    vec3 body=bottom*T+ocean.scattering.rgb/sigma*(1-T)*bodyLight;
    float coastal=(1.0-smoothstep(0.3,2.5,vWaterDepth))*smoothstep(0.02,0.3,vWaterDepth);
    float foam=clamp(smoothstep(0.16,0.30,waves.crest)*0.3+coastal*0.45,0,0.6);
    vec3 result=F*environment+direct+(1-F)*body;
    vec3 foamRadiance=0.65*(incidentSky+sunColor*NoL*sunVis)/OCEAN_PI;
    result=mix(result,foamRadiance,foam);
    // Isolate additive energy and inputs without changing beauty exposure.
    if(debugView==1)result=F*environment*(1-foam);
    if(debugView==2)result=(1-F)*body*(1-foam);
    if(debugView==3)result=direct*(1-foam);
    if(debugView==4)result=foamRadiance*foam;
    if(debugView==5)result=reflectedSky;
    if(debugView==6)result=skyDown;
    if(debugView==7)result=vec3(dot(up,R)<0?1:0);
    if(debugView==8)result=vec3(F);
    if(debugView==11)result=0.18*(incidentSky+sunColor*sunCos*sunVis)/OCEAN_PI;
    if(debugView==12)result=vec3(sunVis);
    if(debugView==13){
        // Existing near-surface atmosphere display transform, for a same-sky
        // comparison. This is NOT applied to normal reflection radiometry.
        vec3 x=skyDown*ocean.ozoneShape.z;
        result=(x*(2.51*x+0.03))/(x*(2.43*x+0.59)+0.14);
    }
    if(any(isnan(result)) || any(isinf(result)))result=vec3(0);
    FragColor=vec4(max(result,vec3(0)),1);
    float viewDistance=max(-(mat3(frame.view)*vRelViewPos).z,1e-8);
    if(farWater) {
        // Orbital water is a sphere (geometry waves have already faded out).
        // Its analytic front intersection lies ahead of the triangulated bed,
        // including shallow columns smaller than one D32 quantization cell.
        vec3 ray=normalize(vRelViewPos),center=pc.model[3].xyz-frame.viewPos;
        float along=dot(ray,center);
        vec3 perpendicular=center-ray*along;
        float radius=pc.planetRadius+waterBedKm(vElevation,pc.maxElevation)+vWaterDepth*0.001;
        float discriminant=radius*radius-dot(perpendicular,perpendicular);
        if(discriminant>=0.0 && along>0.0)
            viewDistance=max((along-sqrt(discriminant))*(-(mat3(frame.view)*ray).z),1e-8);
    }
    gl_FragDepth=clamp(0.5-0.5*frame.proj[2][2]+0.5*frame.proj[3][2]/viewDistance,0.0,1.0);
    Reactive=clamp(0.35+foam*0.3+length(waves.slope)*0.5,0.35,0.85);
}
