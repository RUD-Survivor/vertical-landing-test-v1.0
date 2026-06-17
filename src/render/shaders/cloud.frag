#version 450
// Volumetric cloud fragment shader — fullscreen ray march through cloud band.
// Ported from cloud.glsl + cloud_system.cpp for Vulkan standalone pass.
// Includes atmospheric optical depth for physically-based cloud lighting.

#define PI 3.14159265359
#define CLOUD_STEPS_MAX 128

// ── Set 0: Frame UBO (binding 0) + Cloud params UBO (binding 1) ──────────────
layout(set = 0, binding = 0) uniform FrameUBO {
    mat4  view;
    mat4  proj;
    vec3  lightDir;
    float _pad0;
    vec3  viewPos;
    float time;
} frame;

// CloudParamsUBO — std140, 64 bytes
layout(set = 0, binding = 1) uniform CloudParams {
    float uTime;           // offset 0
    float uPhaseSin;       // offset 4
    float uPhaseCos;       // offset 8
    float uCovLo;          // offset 12
    float uCovHi;          // offset 16
    float uThreshLo;       // offset 20
    float uThreshHi;       // offset 24
    float uErosion;        // offset 28
    float uDensity;        // offset 32
    float uExtinction;     // offset 36
    float uMinAlt;         // offset 40
    float uMaxAlt;         // offset 44
    int   uCloudSteps;     // offset 48
    int   uLightSteps;     // offset 52
    int   uDebug;          // offset 56
    float _pad1;           // offset 60
} cloud;

// ── Set 1: Cloud textures ────────────────────────────────────────────────────
layout(set = 1, binding = 0) uniform sampler3D uNoiseTex3D;
layout(set = 1, binding = 1) uniform sampler3D uCoverTex3D;
layout(set = 1, binding = 2) uniform sampler3D uDetailTex3D;
layout(set = 1, binding = 3) uniform sampler2D uWeatherMap;

// ── Push constants (same layout as atmo) ─────────────────────────────────────
layout(push_constant) uniform PC {
    vec4  planetCenter;
    float innerRadius;
    float outerRadius;
    float surfaceRadius;
    int   planetIdx;
    float sunVisibility;
    float ringInner;
    float ringOuter;
    int   frameIndex;
    float tuneMinAlt;
    float tuneMaxAlt;
    float tuneExtinction;
    float showClouds;
} pc;

layout(location = 0) in  vec2 vNDC;
layout(location = 0) out vec4 FragColor;

// ── Atmosphere globals ───────────────────────────────────────────────────────
vec3  gRayleighCoeff;
float gMieCoeff;
float gHRayleigh, gHMie, gGMie;
vec3  gOzoneCoeff;
float gHOzoneCenter, gHOzoneWidth;
float gCloudMinAlt, gCloudMaxAlt;
vec3  gCloudExtinction;

void setupPlanetProfile() {
    int idx = pc.planetIdx;
    if (idx == 3) {
        gRayleighCoeff = vec3(5.8, 13.5, 33.1) * 1.2e-3;
        gMieCoeff = 5.0e-3; gHRayleigh = 8.0; gHMie = 1.0; gGMie = 0.8;
        gOzoneCoeff = vec3(0.35, 0.85, 0.09) * 1e-3; gHOzoneCenter = 25.0; gHOzoneWidth = 15.0;
    } else if (idx == 2) {
        gRayleighCoeff = vec3(5.0, 12.0, 28.0) * 1e-3;
        gMieCoeff = 0.04; gHRayleigh = 15.0; gHMie = 5.0; gGMie = 0.76;
        gOzoneCoeff = vec3(0.5, 5.0, 20.0) * 1e-3; gHOzoneCenter = 50.0; gHOzoneWidth = 20.0;
    } else {
        gRayleighCoeff = vec3(5.8, 13.5, 33.1) * 1e-3;
        gMieCoeff = 0.005; gHRayleigh = 8.0; gHMie = 1.0; gGMie = 0.8;
        gOzoneCoeff = vec3(0.0); gHOzoneCenter = 1.0; gHOzoneWidth = 1.0;
    }
    gCloudMinAlt = cloud.uMinAlt;
    gCloudMaxAlt = cloud.uMaxAlt;
    gCloudExtinction = vec3(cloud.uExtinction);
}

// ── Sphere intersection ──────────────────────────────────────────────────────
bool intersectSphere(vec3 ro, vec3 rd, float radius, out float t0, out float t1) {
    vec3  L = ro - pc.planetCenter.xyz;
    float b = dot(L, rd), disc = b*b - dot(L,L) + radius*radius;
    if (disc < 0.0) return false;
    float sq = sqrt(disc); t0 = -b - sq; t1 = -b + sq;
    return true;
}

float ozoneDensity(float h) {
    float d = (h - gHOzoneCenter) / gHOzoneWidth;
    return exp(-0.5 * d * d);
}

// ── Baked noise helpers (HZD-style: no runtime FBM in density path) ─────────
const float INV8 = 0.125;

float sampleBakedN(vec3 tc) {
    vec3 uvw = tc * INV8;
    float n  = texture(uNoiseTex3D, uvw).r;
    float wInv = 1.0 - texture(uNoiseTex3D, uvw * 2.0).g;
    return n + (wInv - 0.5) * (1.0 - n) * 0.50;
}

float cheapCloudNoise(vec3 sph, float t) {
    return texture(uNoiseTex3D, (sph * 6.0 + vec3(t * 0.001)) * INV8).r;
}

// ── Atmospheric optical depth (includes cloud contribution via dC) ───────────
void getOpticalDepth(vec3 p, vec3 lDir, out float dR, out float dM, out float dO, out float dC) {
    dR = 0.0; dM = 0.0; dO = 0.0; dC = 0.0;
    float tS0, tS1;
    if (intersectSphere(p, lDir, pc.innerRadius, tS0, tS1) && tS0 > 0.0) {
        dR = dM = dO = 1e6; return;
    }
    float t0, t1;
    if (!intersectSphere(p, lDir, pc.outerRadius, t0, t1) || t1 <= 0.0) return;
    float start = max(t0, 0.0);
    float stepSz = (t1 - start) / float(max(cloud.uLightSteps, 4));
    for (int i = 0; i < 64; i++) {
        if (i >= cloud.uLightSteps) break;
        vec3  pos = p + lDir * (start + (float(i) + 0.5) * stepSz);
        float h = length(pos - pc.planetCenter.xyz) - pc.surfaceRadius;
        if (h < -1.0) { dR = dM = dO = 1e6; return; }
        dR += exp(-h / gHRayleigh) * stepSz;
        dM += exp(-h / gHMie)      * stepSz;
        dO += ozoneDensity(h)       * stepSz;
    }

    // ── Cloud self-shadow: cheap procedural density along light path ─────
    float cloudInnerR = pc.surfaceRadius + gCloudMinAlt;
    float cloudOuterR = pc.surfaceRadius + gCloudMaxAlt;
    float ct0, ct1;
    if (intersectSphere(p, lDir, cloudOuterR, ct0, ct1) && ct1 > 0.0) {
        float cStart = max(ct0, 0.0);
        float cStep = max((ct1 - cStart) / 6.0, 0.05);
        for (int j = 0; j < 6; j++) {
            vec3 cp = p + lDir * (cStart + (float(j) + 0.5) * cStep);
            float ch = length(cp - pc.planetCenter.xyz) - pc.surfaceRadius;
            if (ch >= gCloudMinAlt && ch <= gCloudMaxAlt) {
                vec3 cs = normalize(cp - pc.planetCenter.xyz);
                float altN = clamp((ch - gCloudMinAlt) / max(gCloudMaxAlt - gCloudMinAlt, 0.1), 0.0, 1.0);
                float profile = smoothstep(0.0, 0.05, altN) * smoothstep(1.0, 0.85, altN);
                float cheapDensity = cheapCloudNoise(cs, cloud.uTime) * profile * cloud.uDensity * 0.01;
                dC += cheapDensity * cStep;
            }
        }
    }
}

// ── Screen-space dither ─────────────────────────────────────────────────────
float screenHash(vec2 uv) {
    float ign = fract(52.9829189 * fract(dot(uv, vec2(0.06711056, 0.00583715))));
    return fract(ign + float(pc.frameIndex % 32) * 0.6180339887);
}

// ── Cloud density (HZD baked-noise path, no runtime FBM) ────────────────────
float cloudDensity(vec3 p, float h, bool highDetail) {
    if (pc.showClouds < 0.5) return 0.0;
    if (h < gCloudMinAlt || h > gCloudMaxAlt) return 0.0;

    // Debug: force uniform density to isolate shape from lighting
    if (cloud.uDebug == 1) return 2.0;

    vec3 sph = normalize(p - pc.planetCenter.xyz);
    float t = cloud.uTime * 0.004;
    float density = 0.0;

    if (pc.planetIdx == 3) {
        if (h >= gCloudMinAlt && h <= 7.0) {
            float altNorm = (h - gCloudMinAlt) / (7.0 - gCloudMinAlt);
            float altBase = smoothstep(0.0, 0.02, altNorm);  // sharp cloud base (~110m)

            float latWarp = sph.z*sph.z*sph.z*0.3;
            float sB=sin(latWarp), cB=cos(latWarp);
            float s1=cloud.uPhaseSin*cB+cloud.uPhaseCos*sB;
            float c1=cloud.uPhaseCos*cB-cloud.uPhaseSin*sB;
            vec3 windSph1 = vec3(sph.x*c1-sph.y*s1, sph.x*s1+sph.y*c1, sph.z);

            float cycAngle = t * -0.1;
            vec3 cycCenter = normalize(vec3(0.6*cos(cycAngle),0.6*sin(cycAngle),0.35));
            float cycDist = distance(sph, cycCenter);
            float cycInfluence = smoothstep(0.22, 0.0, cycDist);
            if (cycInfluence > 0.001) {
                float twist = cycInfluence*1.8;
                float ct=cos(twist), st=sin(twist);
                vec3 cycCenterW = normalize(vec3(cycCenter.x*c1-cycCenter.y*s1,
                                                 cycCenter.x*s1+cycCenter.y*c1,cycCenter.z));
                vec3 warped = windSph1*ct+cross(cycCenterW,windSph1)*st
                            + cycCenterW*dot(cycCenterW,windSph1)*(1.0-ct);
                windSph1 = mix(windSph1, warped, cycInfluence);
            }

            vec3 coverageSph = vec3(sph.x*c1-sph.y*s1, sph.x*s1+sph.y*c1, sph.z);
            vec3 wAxis = normalize(cross(coverageSph,vec3(0,0,1)+coverageSph*0.01));
            vec3 vAxis = cross(coverageSph,wAxis);
            const float EPS=0.06;

            vec3 seedLo = vec3(t*0.003,0,t*0.002);
            float pLo_pu=texture(uCoverTex3D,(coverageSph*1.5+wAxis*EPS+seedLo)*INV8).r;
            float pLo_mu=texture(uCoverTex3D,(coverageSph*1.5-wAxis*EPS+seedLo)*INV8).r;
            float pLo_pv=texture(uCoverTex3D,(coverageSph*1.5+vAxis*EPS+seedLo)*INV8).r;
            float pLo_mv=texture(uCoverTex3D,(coverageSph*1.5-vAxis*EPS+seedLo)*INV8).r;
            float curlU_lo=-(pLo_pv-pLo_mv), curlV_lo=(pLo_pu-pLo_mu);

            vec3 seedHi=vec3(91.2+t*0.005,5.1,22.0);
            float pHi_pu=texture(uCoverTex3D,(coverageSph*6.0+wAxis*EPS+seedHi)*INV8).g;
            float pHi_mu=texture(uCoverTex3D,(coverageSph*6.0-wAxis*EPS+seedHi)*INV8).g;
            float pHi_pv=texture(uCoverTex3D,(coverageSph*6.0+vAxis*EPS+seedHi)*INV8).g;
            float pHi_mv=texture(uCoverTex3D,(coverageSph*6.0-vAxis*EPS+seedHi)*INV8).g;
            float curlU_hi=-(pHi_pv-pHi_mv), curlV_hi=(pHi_pu-pHi_mu);

            float warpU=curlU_lo*2.5+curlU_hi*1.0, warpV=curlV_lo*2.5+curlV_hi*1.0;
            vec3 warpedSph=normalize(coverageSph+wAxis*warpU+vAxis*warpV);
            float cov_large=texture(uNoiseTex3D,(warpedSph*0.8+vec3(t*0.004))*INV8).a;
            float cov_medium=texture(uCoverTex3D,(warpedSph*6.0+vec3(t*0.011,-t*0.008,t*0.013))*INV8).a;
            float coverage=cov_large*0.65+cov_medium*0.35;
            vec2 wUV=vec2(atan(warpedSph.y,warpedSph.x)*(0.5/PI)+0.5,
                         asin(clamp(warpedSph.z,-1.0,1.0))/PI+0.5);
            vec4 weather=(pc.planetIdx==3)?texture(uWeatherMap,wUV):vec4(0.65,0.5,0.6,0.0);
            coverage=clamp(coverage*weather.r,0.0,1.0);
            float covRange=max(cloud.uCovHi-cloud.uCovLo,0.001);
            coverage=clamp((coverage-cloud.uCovLo)/covRange,0.0,1.0);

            if (cycInfluence>0.001) {
                coverage=mix(coverage,clamp(coverage*1.9,0.0,1.0),cycInfluence*0.7);
                float eyeNoise=texture(uCoverTex3D,(sph*4.0+vec3(t*0.02))*INV8).r*0.009;
                coverage*=smoothstep((0.017+eyeNoise)*0.35,0.017+eyeNoise,cycDist);
            }

            float cloudType=mix(texture(uCoverTex3D,(warpedSph*2.5+vec3(31.4,17.8,42.1))*INV8).r,weather.g,0.55);
            float stratusProfile=altBase*(1.0-smoothstep(0.82,1.0,altNorm));
            float cumulusCore=exp(-((altNorm-0.35)*(altNorm-0.35))/(0.25*0.25));
            float topNoise=texture(uCoverTex3D,(windSph1*8.0+vec3(53.2,29.7,71.4+t*0.01))*INV8).r;
            float topHeight=mix(0.55,1.00,topNoise);
            float cumulusTop=1.0-smoothstep(topHeight-0.08,topHeight+0.08,altNorm);
            float cumulusProfile=altBase*cumulusCore*cumulusTop;
            float cbTower=smoothstep(0.0,0.50,altNorm)*(1.0-smoothstep(0.50,0.72,altNorm));
            float cbAnvil=smoothstep(0.68,0.80,altNorm)*(1.0-smoothstep(0.88,1.00,altNorm))*0.35;
            float cumulonimbusProfile=altBase*max(cbTower,cbAnvil);
            float heightGradient=mix(mix(stratusProfile,cumulusProfile,smoothstep(0.0,0.5,cloudType)),
                                    cumulonimbusProfile,smoothstep(0.5,1.0,cloudType));
            float altShape=heightGradient*1.5;

            vec3 tc=windSph1*(80.0+h*0.45)+vec3(t*0.6,t*0.4,t*0.15);
            float viewDist=length(frame.viewPos-p);

            float warp=texture(uNoiseTex3D,tc*0.0375).b;
            vec3 warpedTc=tc+vec3(warp*0.4,warp*0.4,warp*0.2)+vec3(t*0.02);
            float n=sampleBakedN(warpedTc);

            float ero=texture(uNoiseTex3D,tc*0.2875+vec3(-t*0.04,t*0.03,0.0)).g;
            float topFraction=smoothstep(topHeight*0.5,topHeight*0.95,altNorm);
            n-=mix(ero,1.0-ero,topFraction)*cloud.uErosion;

            {
                vec3 pW=p+vec3(warp-0.5,warp*1.4-0.7,0.5-warp*0.9)*18.0;
                float c1=1.0-texture(uNoiseTex3D,pW*0.005+vec3(0.914+t*0.00225,0.521,0.341+t*0.00150)).g;
                float c2=1.0-texture(uNoiseTex3D,pW*0.00975+vec3(0.039,0.440+t*0.00275,0.091)).g;
                n+=(c1*(0.4+0.6*c2))*0.30-0.16;
            }

            if (viewDist<8.0) {
                float dW1=1.0-smoothstep(0.5,8.0,viewDist);
                vec3 uvA=fract(p/0.200+vec3(0.137,0.421,0.073));
                float da=texture(uDetailTex3D,uvA).a;
                float tFr1=smoothstep(0.5,0.85,altNorm);
                float maskLoVal=min(cloud.uThreshLo,cloud.uThreshHi);
                float surfW=1.0-smoothstep(maskLoVal,maskLoVal+0.10,n);
                n-=mix(da,1.0-da,tFr1)*(0.10+0.30*surfW)*dW1;
            }

            float maskLo=min(cloud.uThreshLo,cloud.uThreshHi);
            float maskHi=max(cloud.uThreshLo,cloud.uThreshHi);
            float formPotential=smoothstep(0.18,0.42,coverage);
            float localMaskLo=mix(0.58,maskLo,formPotential);
            float localMaskHi=mix(0.76,maskHi,formPotential);
            float mesoField=texture(uCoverTex3D,(windSph1*1.2+vec3(71.3,39.7,14.2+t*0.002))*INV8).r;
            float thickVar=(mesoField-0.45)*0.28;
            float mesoLo=localMaskLo+thickVar, mesoHi=localMaskHi+thickVar*0.5;
            float mask=smoothstep(mesoLo,mesoHi,n)*altShape;
            density=max(density,mask*cloud.uDensity*(0.30+0.70*coverage));
        }

        if (h>=8.0 && h<=gCloudMaxAlt) {
            float altNorm=(h-8.0)/(gCloudMaxAlt-8.0);
            float cirrusBase=smoothstep(0.0,0.08,altNorm);
            float cirrusWisp=exp(-altNorm*3.2);
            float altShape=cirrusBase*cirrusWisp;
            float angle2=t*-0.5;
            float latWarp2=sph.z*sph.z*sph.z*0.15;
            float s2=sin(angle2+latWarp2),c2=cos(angle2+latWarp2);
            vec3 windSph2=vec3(sph.x*c2-sph.y*s2,sph.x*s2+sph.y*c2,sph.z);
            vec3 tc2=windSph2*50.0+vec3(t*0.04);
            float n2=texture(uNoiseTex3D,tc2*INV8).b;
            n2-=texture(uNoiseTex3D,tc2*0.3125).g*0.18;
            float mask2=smoothstep(0.42+altNorm*0.10,0.82,n2)*altShape;
            density=max(density,mask2*0.30);
        }
    } else {
        float altNorm=(h-gCloudMinAlt)/(gCloudMaxAlt-gCloudMinAlt);
        float altShape=4.0*altNorm*(1.0-altNorm);
        float angle=cloud.uTime*0.004*-0.1;
        float s1s=sin(angle),c1s=cos(angle);
        vec3 windSph=vec3(sph.x*c1s-sph.y*s1s,sph.x*s1s+sph.y*c1s,sph.z);
        float n=texture(uNoiseTex3D,(windSph*8.0+vec3(cloud.uTime*0.0002))*INV8).r;
        density=(0.3+0.7*n)*altShape*0.8;
    }
    return density;
}

// ── Cloud phase ─────────────────────────────────────────────────────────────
float cloudPhase(float cosTheta) {
    float g1=0.85, num1=1.0-g1*g1, denom1=pow(max(1.0+g1*g1-2.0*g1*cosTheta,1e-5),1.5);
    float phase1=(1.0/(4.0*PI))*(num1/denom1);
    float g2=-0.3, num2=1.0-g2*g2, denom2=pow(max(1.0+g2*g2-2.0*g2*cosTheta,1e-5),1.5);
    float phase2=(1.0/(4.0*PI))*(num2/denom2);
    return mix(phase2,phase1,0.7);
}

// ── Cone-traced AO ──────────────────────────────────────────────────────────
float coneAO(vec3 p, vec3 sph, float h) {
    if (h<gCloudMinAlt+0.3) return 0.08;
    vec3 down=-sph, tan1=normalize(cross(sph,vec3(0.371,0.629,0.683)));
    vec3 tan2=cross(sph,tan1);
    float occ=0.0, wSum=0.0;
    float dists[3]=float[3](0.12,0.28,0.50), wgts[3]=float[3](0.50,0.30,0.20);
    for (int d=0; d<3; d++) {
        { vec3 pp=p+down*dists[d]; float ph=length(pp-pc.planetCenter.xyz)-pc.surfaceRadius;
          if (ph>=gCloudMinAlt&&ph<=gCloudMaxAlt) {
              vec3 ps=normalize(pp-pc.planetCenter.xyz);
              float dC=1.0-texture(uNoiseTex3D,ps*INV8).g; occ+=dC*wgts[d]*0.50; }
          wSum+=wgts[d]*0.50; }
        { vec3 pp=p+tan1*dists[d]*1.5; float ph=length(pp-pc.planetCenter.xyz)-pc.surfaceRadius;
          if (ph>=gCloudMinAlt&&ph<=gCloudMaxAlt) {
              vec3 ps=normalize(pp-pc.planetCenter.xyz);
              float dC=1.0-texture(uNoiseTex3D,ps*INV8).g; occ+=dC*wgts[d]*0.25; }
          wSum+=wgts[d]*0.25; }
        { vec3 pp=p+tan2*dists[d]*1.5; float ph=length(pp-pc.planetCenter.xyz)-pc.surfaceRadius;
          if (ph>=gCloudMinAlt&&ph<=gCloudMaxAlt) {
              vec3 ps=normalize(pp-pc.planetCenter.xyz);
              float dC=1.0-texture(uNoiseTex3D,ps*INV8).g; occ+=dC*wgts[d]*0.25; }
          wSum+=wgts[d]*0.25; }
    }
    float avgOc=(wSum>0.001)?occ/wSum:0.0;
    return 1.0-avgOc*1.3;
}

// ── Main ────────────────────────────────────────────────────────────────────
void main() {
    if (pc.showClouds<0.5 || pc.planetIdx!=3) discard;

    setupPlanetProfile();

    vec3 viewDir = vec3(vNDC.x/frame.proj[0][0], -vNDC.y/frame.proj[1][1], -1.0);
    vec3 rayDir  = normalize(transpose(mat3(frame.view))*viewDir);
    vec3 camPos  = frame.viewPos;
    vec3 lightDir= normalize(frame.lightDir);
    vec3 ctr     = pc.planetCenter.xyz;
    float cloudInnerR=pc.surfaceRadius+gCloudMinAlt;
    float cloudOuterR=pc.surfaceRadius+gCloudMaxAlt;

    float t0i,t1i,t0o,t1o;
    bool hitInner=intersectSphere(camPos,rayDir,cloudInnerR,t0i,t1i);
    bool hitOuter=intersectSphere(camPos,rayDir,cloudOuterR,t0o,t1o);
    if (!hitOuter||t1o<=0.0) discard;

    float camDist=length(camPos-ctr);
    bool inOuter=camDist<cloudOuterR, inInner=camDist<cloudInnerR;

    float tStart,tEnd;
    if (inInner)     { tStart=max(t1i,0.001); tEnd=t1o; }
    else if (inOuter){ tStart=0.001; tEnd=t1o; if(hitInner&&t0i>0.0) tEnd=min(tEnd,t0i); }
    else             { tStart=max(t0o,0.0); tEnd=t1o; if(hitInner&&t0i>0.0) tEnd=min(tEnd,t0i); }
    if (tEnd-tStart<0.1) discard;

    // ── Clip ray at planet surface (prevents seeing clouds through ground) ──
    float tSurf0, tSurf1;
    if (intersectSphere(camPos, rayDir, pc.surfaceRadius, tSurf0, tSurf1) && tSurf0 > 0.0)
        tEnd = min(tEnd, tSurf0);
    if (tEnd - tStart < 0.1) discard;

    float camAltCloud=length(camPos-ctr)-pc.surfaceRadius;
    bool inCloudLayer=(camAltCloud<gCloudMaxAlt+5.0);
    float fineStep=0.30, bigStep=inCloudLayer?2.50:4.0;

    // Adaptive step: horizon rays traverse hundreds of km through cloud layer.
    // Use max(fineStep, segmentLen/128) to guarantee full coverage in ≤128 iters.
    float segLen = tEnd - tStart;
    float adaptStep = min(max(fineStep, segLen / 128.0), 0.50);

    float cJitter=screenHash(gl_FragCoord.xy);
    float tCur=tStart+cJitter*adaptStep;
    float cosTheta=dot(rayDir,lightDir);

    vec3 sumCloud=vec3(0.0);
    float optDepthC=0.0, prevDC=0.0;
    float viewTrans=1.0;  // view-ray transmittance for cloud self-occlusion

    float entryFrac=clamp(tStart/max(tEnd,0.001),0.0,1.0);
    float dummyR,dummyM,dummyO,dummyC;
    getOpticalDepth(camPos+rayDir*tStart,lightDir,dummyR,dummyM,dummyO,dummyC);
    vec3 atmTransAtCloud=exp(-(gRayleighCoeff*dummyR+gMieCoeff*dummyM+gOzoneCoeff*dummyO)*entryFrac);

    // March with fineStep granularity; CLOUD_STEPS_MAX is safety cap only.
    // cloud.uCloudSteps controls getOpticalDepth light-march resolution, not loop bound.
    for (int ci=0; ci<CLOUD_STEPS_MAX; ci++) {
        if (tCur>=tEnd) break;
        float dt=min(adaptStep,tEnd-tCur);
        vec3 cPos=camPos+rayDir*(tCur+dt*0.5);
        float ch=max(length(cPos-ctr)-pc.surfaceRadius,0.0);

        if (ch<gCloudMinAlt||ch>gCloudMaxAlt) { tCur+=bigStep; continue; }

        float dC=cloudDensity(cPos,ch,true)*dt;
        if (dC<0.0001) { tCur+=adaptStep; continue; }

        optDepthC+=dC;

        float cLR,cLM,cLO,cLC;
        getOpticalDepth(cPos,lightDir,cLR,cLM,cLO,cLC);
        vec3 cloudShadow=exp(-gCloudExtinction*cLC*0.45);
        vec3 sunTint=exp(-(gRayleighCoeff*cLR+gMieCoeff*1.1*cLM+gOzoneCoeff*cLO))*cloudShadow;

        vec3 tauRgb=gCloudExtinction.x*(optDepthC+cLC)*vec3(1.0,1.04,1.10);
        vec3 msRgb=(exp(-tauRgb)+exp(-tauRgb*0.5)*0.5+exp(-tauRgb*0.25)*0.25+exp(-tauRgb*0.125)*0.125)/6.0;

        float tauLight=gCloudExtinction.x*cLC;
        float powder=1.0-exp(-tauLight*2.0);
        float edgeSharpness=smoothstep(0.002,0.018,abs(dC-prevDC)/max(adaptStep,0.0001));
        float powderEdge=powder*(1.0+edgeSharpness*0.8);
        float silverMask=max(0.0,cosTheta);
        vec3 cloudAlbedo=vec3(0.97,0.98,1.00);
        vec3 directAtt=msRgb*(0.55+powderEdge*silverMask*0.55)*cloudAlbedo;
        float coreDark=1.0-powder*0.55;
        prevDC=dC;

        float layerBase=(ch<7.5)?gCloudMinAlt:8.0;
        float layerTop=(ch<7.5)?7.0:gCloudMaxAlt;
        float heightInBand=clamp((ch-layerBase)/max(layerTop-layerBase,0.1),0.0,1.0);
        float aoHeight=pow(heightInBand,0.55);
        float aoCone=aoHeight;
        float cloudDist=length(camPos-cPos);
        if (cloudDist<60.0&&(ci&7)==0) {
            float aoConeS=coneAO(cPos,normalize(cPos-ctr),ch);
            aoCone=aoHeight*0.35+aoConeS*0.65;
        }
        float aoDirect=mix(0.40,1.0,aoCone);
        float aoAmbient=mix(0.40,1.0,aoCone);

        vec3 cSph=normalize(cPos-ctr);
        float sunElev=dot(cSph,lightDir);
        float skyVis=smoothstep(-0.5,0.15,sunElev);
        float nightGlow=0.015*(1.0-skyVis);

        // Terminator reddening: per-sample sun elevation + global dusk (matches atmo sky band)
        float twilightLocal=smoothstep(0.22,-0.06,sunElev);
        float twilightGlobal=smoothstep(0.50,0.12,pc.sunVisibility);
        float twilight=clamp(max(twilightLocal,twilightGlobal*0.75),0.0,1.0);

        vec3 sunColorDay=vec3(1.08,1.02,0.94);
        vec3 sunColorDusk=vec3(1.50,0.45,0.08);
        vec3 sunColor=mix(sunColorDay,sunColorDusk,twilight)*mix(0.04,1.0,pc.sunVisibility);

        float phaseNorm=cloudPhase(cosTheta)/(cloudPhase(1.0)+1e-5);
        float VoLPowder=pow(clamp(cosTheta*0.5+0.5,0.0,1.0),1.4);
        float nightFactor=mix(0.05,1.0,pc.sunVisibility);
        float exposure=mix(8.0,5.0,clamp(camAltCloud/120.0,0.0,1.0))*nightFactor;

        // View-path chromatic tint at twilight only (red on entry, not daytime double-extinction)
        float atmLum=dot(atmTransAtCloud,vec3(0.299,0.587,0.114));
        vec3 viewChrom=atmTransAtCloud/max(atmLum,0.04);
        vec3 viewTint=mix(vec3(1.0),viewChrom,twilight*0.85);

        vec3 directContrib=sunColor*sunTint*directAtt*aoDirect*phaseNorm*VoLPowder*exposure*viewTint;

        vec3 skyDay=vec3(0.62,0.70,0.92);
        vec3 skyDusk=vec3(1.05,0.40,0.12);
        vec3 skyNight=vec3(0.02,0.025,0.04);
        vec3 skyAmbColor=mix(mix(skyNight,skyDay,skyVis),skyDusk,twilightLocal*skyVis)+vec3(nightGlow);
        float shadowStrength=1.0-dot(cloudShadow,vec3(0.333));
        vec3 ambAtmTint=mix(vec3(atmLum),atmTransAtCloud,twilight*0.80);
        vec3 ambientContrib=ambAtmTint*skyAmbColor*cloudAlbedo*aoAmbient*coreDark
                            *(1.0-shadowStrength*0.40)*mix(0.45,1.0,phaseNorm)*0.65*exposure;

        sumCloud+=dC*(directContrib+ambientContrib)*viewTrans;
        viewTrans*=exp(-gCloudExtinction.x*dC);

        if (viewTrans<0.005) break;
        tCur+=adaptStep;
    }

    float nightFade=smoothstep(0.0,0.12,pc.sunVisibility);
    sumCloud*=nightFade;

    // Match atmo.frag ACES tonemap — prevents linear muddy brown on composite
    {
        vec3 x=max(sumCloud,vec3(0.0));
        sumCloud=(x*(2.51*x+0.03))/(x*(2.43*x+0.59)+0.14);
    }

    float alpha=clamp(1.0-viewTrans,0.0,1.0);
    alpha*=mix(0.08,1.0,nightFade);
    if (alpha<0.003) discard;

    FragColor=vec4(sumCloud,alpha);  // sumCloud already pre-multiplied by viewTrans
}
