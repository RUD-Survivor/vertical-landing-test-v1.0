#version 450
// =============================================================================
// cloud.frag — RocketSim3D 体积云主入口（全屏 fragment shader，单 pass ray march）
// -----------------------------------------------------------------------------
// 用途：
//   全屏四边形 ray march 体积云。每个像素从相机出发穿过云带，逐步累加散射光与
//   透射率，最终输出 tone-mapped + premultiplied 的 RGBA，用于与场景 HDR 做混合。
//
// 依赖（本文件提供 binding/uniform 声明）：
//   Set 0 binding 0: FrameUBO     — view/proj/lightDir/viewPos/time
//   Set 0 binding 1: CloudParams  — 云调参（覆盖率/阈值/密度/消光/高度/步数/调试）
//   Set 1 binding 0~3: 3D 噪声纹理 + weather map
//   push_constant: 行星几何/轨道/调参
//
// 逻辑库（无 binding，纯函数）：
//   #include "cloud/cloud_rs3d_common.glsl"    — 坐标/球壳/profile/常量
//   #include "cloud/cloud_rs3d_noise.glsl"     — 烘焙噪声/remap/相位
//   #include "cloud/cloud_rs3d_density.glsl"   — cloudDensity / cloudDensityLow / cloudMapLocalCore
//   #include "cloud/cloud_rs3d_lighting.glsl"  — getOpticalDepth / cloudPhase / powder / coneAO / screenHash
//
// 后续迁移 1/4 compute pass 时，本文件 main() 的 march 循环将整体搬到
// cloud_raymarching.comp，库文件原样复用。
// =============================================================================

#extension GL_GOOGLE_include_directive : enable

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
    float uLocalRadius;    // offset 60 — local meter cloudMap radius (km), 0 = off
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

// ── 逻辑库（顺序重要：common 先于 noise/density/lighting）──────────────────
#include "cloud/cloud_rs3d_common.glsl"
#include "cloud/cloud_rs3d_noise.glsl"
#include "cloud/cloud_rs3d_density.glsl"
#include "cloud/cloud_rs3d_lighting.glsl"

// ── Main ────────────────────────────────────────────────────────────────────
void main() {
    // 仅在 idx=3（地球类）且开启云时绘制；其余行星走各自 fallback。
    if (pc.showClouds<0.5 || pc.planetIdx!=3) discard;

    setupPlanetProfile();

    // ── 重建相机射线 ────────────────────────────────────────────────────────
    // vNDC → view space → world space 方向。
    vec3 viewDir = vec3(vNDC.x/frame.proj[0][0], -vNDC.y/frame.proj[1][1], -1.0);
    vec3 rayDir  = normalize(transpose(mat3(frame.view))*viewDir);
    vec3 camPos  = frame.viewPos;
    vec3 lightDir= normalize(frame.lightDir);
    vec3 ctr     = pc.planetCenter.xyz;
    float cloudInnerR=pc.surfaceRadius+gCloudMinAlt;
    float cloudOuterR=pc.surfaceRadius+gCloudMaxAlt;

    // ── 与云带内外球壳求交，确定 march 区间 [tStart, tEnd] ───────────────────
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

    // ── 截断到行星表面，避免云画到地下 ──────────────────────────────────────
    float tSurf0, tSurf1;
    if (intersectSphere(camPos, rayDir, pc.surfaceRadius, tSurf0, tSurf1) && tSurf0 > 0.0)
        tEnd = min(tEnd, tSurf0);
    if (tEnd - tStart < 0.1) discard;

    // ── 自适应步长 ──────────────────────────────────────────────────────────
    // 地平线射线穿过云带可达数百 km；用 max(fineStep, segLen/128) 保证 ≤128 步覆盖。
    // inCloudLayer 时用更细的 fineStep（2.5 vs 4.0）。
    float camAltCloud=length(camPos-ctr)-pc.surfaceRadius;
    bool inCloudLayer=(camAltCloud<gCloudMaxAlt+5.0);
    float fineStep=0.30, bigStep=inCloudLayer?2.50:4.0;
    float segLen = tEnd - tStart;
    float adaptStep = min(max(fineStep, segLen / 128.0), 0.50);

    // ── march 起点抖动 + 相位角 ──────────────────────────────────────────────
    float cJitter=screenHash(gl_FragCoord.xy);
    float tCur=tStart+cJitter*adaptStep;
    float cosTheta=dot(rayDir,lightDir);

    // ── 累加器 ──────────────────────────────────────────────────────────────
    // sumCloud: 累加的散射光 L (premultiplied 已乘 viewTrans)
    // viewTrans: 视线方向剩余透射率 T (Beer-Lambert 累乘)
    // optDepthC: 累积云光学深度（调试用）
    vec3 sumCloud=vec3(0.0);
    float optDepthC=0.0, prevDC=0.0;
    float viewTrans=1.0;
    float dbgDensity=0.0, dbgCoverage=0.0, dbgProfile=0.0, dbgShape=0.0;
    float dbgStepTau=0.0;
    vec3 dbgLighting=vec3(0.0);

    // ── 入口处大气透射（用于 twilight 视线红化）──────────────────────────────
    // entryFrac: 入口点到地表的比例（粗略），用于把整段大气透射按比例缩放。
    float entryFrac=clamp(tStart/max(tEnd,0.001),0.0,1.0);
    float dummyR,dummyM,dummyO,dummyC;
    getOpticalDepth(camPos+rayDir*tStart,lightDir,dummyR,dummyM,dummyO,dummyC);
    vec3 atmTransAtCloud=exp(-(gRayleighCoeff*dummyR+gMieCoeff*dummyM+gOzoneCoeff*dummyO)*entryFrac);

    // ── 主 march 循环 ────────────────────────────────────────────────────────
    // CLOUD_STEPS_MAX=128 是安全上限；步长由 adaptStep 决定，循环内 break 提前退出。
    // cloud.uCloudSteps 控制 getOpticalDepth 的 light march 步数，不是本循环上限。
    for (int ci=0; ci<CLOUD_STEPS_MAX; ci++) {
        if (tCur>=tEnd) break;
        float dt=min(adaptStep,tEnd-tCur);
        vec3 cPos=camPos+rayDir*(tCur+dt*0.5);                 // 步中点
        float ch=max(length(cPos-ctr)-pc.surfaceRadius,0.0);   // 海拔 km

        // 云带外快速跳过（bigStep）
        if (ch<gCloudMinAlt||ch>gCloudMaxAlt) { tCur+=bigStep; continue; }

        // 主密度（highDetail=true 启用近距侵蚀）
        float densitySample=cloudDensity(cPos,ch,true);
        float dC=densitySample*dt;                            // 该步光学深度增量
        if (dC<0.0001) { tCur+=adaptStep; continue; }         // 几乎透明，跳过
        dbgDensity=max(dbgDensity,densitySample);
        dbgCoverage=max(dbgCoverage,gDbgCoverage);
        dbgProfile=max(dbgProfile,gDbgProfile);
        dbgShape=max(dbgShape,remapClamped(gDbgEroded,0.0,1.0,0.0,1.0));

        optDepthC+=dC;

        // ── 太阳光照：大气透射 + 云自阴影 ────────────────────────────────────
        // 沿太阳方向积分得到 dR/dM/dO/dC，再 Beer-Lambert 得到 sunTint / cloudShadow。
        float cLR,cLM,cLO,cLC;
        getOpticalDepth(cPos,lightDir,cLR,cLM,cLO,cLC);
        vec3 airTint=exp(-(gRayleighCoeff*cLR+gMieCoeff*1.1*cLM+gOzoneCoeff*cLO));
        vec3 cloudShadow=exp(-gCloudExtinction*cLC*0.45);     // 单次散射云影
        vec3 cloudShadowMs=exp(-gCloudExtinction*cLC*0.14);   // 多次散射云影（更弱）
        vec3 sunTint=airTint*cloudShadow;
        vec3 sunTintMs=airTint*cloudShadowMs;

        // ── powder / 银边 / 核心暗化 ─────────────────────────────────────────
        float tauLight=gCloudExtinction.x*cLC;
        float powderDepth=clamp(densitySample*8.0+tauLight*0.25,0.0,1.5)+0.05;
        float edgeSharpness=smoothstep(0.002,0.018,abs(dC-prevDC)/max(adaptStep,0.0001));
        float powderEdge=edgeSharpness*0.35;                   // 边缘银边强化
        float silverMask=max(0.0,cosTheta);
        vec3 cloudAlbedo=vec3(0.97,0.98,1.00);
        float coreDark=1.0-clamp(tauLight*0.18,0.0,0.55);      // 厚云核心变暗
        prevDC=dC;

        // ── coneAO（近距每 8 步算一次）──────────────────────────────────────
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

        // ── 太阳高度/昼夜/晨昏调色 ───────────────────────────────────────────
        vec3 cSph=normalize(cPos-ctr);
        float sunElev=dot(cSph,lightDir);
        float skyVis=smoothstep(-0.5,0.15,sunElev);
        float nightGlow=0.015*(1.0-skyVis);

        // 晨昏红化：局部太阳高度 + 全局 dusk（与 atmo 天空带一致）
        float twilightLocal=smoothstep(0.22,-0.06,sunElev);
        float twilightGlobal=smoothstep(0.50,0.12,pc.sunVisibility);
        float twilight=clamp(max(twilightLocal,twilightGlobal*0.75),0.0,1.0);

        vec3 sunColorDay=vec3(1.08,1.02,0.94);
        vec3 sunColorDusk=vec3(1.50,0.45,0.08);
        vec3 sunColor=mix(sunColorDay,sunColorDusk,twilight)*mix(0.04,1.0,pc.sunVisibility);

        // ── 相位 / powder / 曝光 ─────────────────────────────────────────────
        // phaseNorm: 当前相位相对最大相位的归一化（朝太阳时最强）
        // phaseMs: 多次散射相位（更各向同性）
        float phaseNorm=cloudPhase(cosTheta)/(cloudPhase(1.0)+1e-5);
        float phaseMs=mix(uniformPhase(),cloudPhase(cosTheta),0.35)/(cloudPhase(1.0)+1e-5);
        float verticalProbability=pow(remapClamped(heightInBand,0.07,0.22,0.10,1.0),0.8);
        float powder=powderEffectNew(powderDepth,verticalProbability,cosTheta);
        float powderMs=mix(1.0,powder,0.45);
        float forwardEdge=1.0+powderEdge*silverMask;
        float nightFactor=mix(0.05,1.0,pc.sunVisibility);
        float exposure=mix(8.0,5.0,clamp(camAltCloud/120.0,0.0,1.0))*nightFactor;

        // 视线方向大气红化（仅晨昏，避免白天双重消光）
        float atmLum=dot(atmTransAtCloud,vec3(0.299,0.587,0.114));
        vec3 viewChrom=atmTransAtCloud/max(atmLum,0.04);
        vec3 viewTint=mix(vec3(1.0),viewChrom,twilight*0.85);

        // ── 三光照分量：直射 / 多次散射 / 天空 / 地面 ─────────────────────────
        vec3 sunLit=sunColor*sunTint*aoDirect*phaseNorm*powder*forwardEdge*exposure*viewTint;
        vec3 sunLitMs=sunColor*sunTintMs*aoDirect*phaseMs*powderMs*0.55*exposure*viewTint;

        vec3 skyDay=vec3(0.62,0.70,0.92);
        vec3 skyDusk=vec3(1.05,0.40,0.12);
        vec3 skyNight=vec3(0.02,0.025,0.04);
        vec3 skyAmbColor=mix(mix(skyNight,skyDay,skyVis),skyDusk,twilightLocal*skyVis)+vec3(nightGlow);
        float shadowStrength=1.0-dot(cloudShadow,vec3(0.333));
        vec3 ambAtmTint=mix(vec3(atmLum),atmTransAtCloud,twilight*0.80);
        vec3 skyLit=ambAtmTint*skyAmbColor*aoAmbient*coreDark
                   *(1.0-shadowStrength*0.35)*mix(0.45,1.0,phaseNorm)*0.60*exposure;
        float groundWeight=(1.0-heightInBand)*smoothstep(-0.05,0.30,sunElev)*0.22;
        vec3 groundLit=vec3(0.26,0.22,0.16)*ambAtmTint*groundWeight*exposure;

        // ── 散射累加（Beer-Lambert + premultiplied transmittance）────────────
        // 双散射阶：sigmaE0/S0 主阶，sigmaE1/S1 多次散射阶（更弱消光、更弱散射）。
        vec3 luminance0=sunLit+skyLit+groundLit;
        vec3 luminance1=sunLitMs+skyLit*0.75+groundLit*0.55;
        float sigmaE0=max(gCloudExtinction.x*densitySample,1e-4);
        float sigmaE1=max(sigmaE0*0.35,1e-4);
        vec3 sigmaS0=sigmaE0*cloudAlbedo;
        vec3 sigmaS1=sigmaE0*cloudAlbedo*0.45;
        float stepTrans0=exp(-sigmaE0*dt);
        float stepTrans1=exp(-sigmaE1*dt);
        dbgStepTau=max(dbgStepTau,sigmaE0*dt);
        dbgLighting=max(dbgLighting,vec3(dot(sunLit,vec3(0.333)),dot(skyLit,vec3(0.333)),dot(groundLit,vec3(0.333))));

        // L += T * (Lum * sigmaS) * (1 - exp(-sigmaE*dt)) / sigmaE
        sumCloud+=viewTrans*(luminance0*sigmaS0)*(1.0-stepTrans0)/sigmaE0;
        sumCloud+=viewTrans*(luminance1*sigmaS1)*(1.0-stepTrans1)/sigmaE1;
        viewTrans*=stepTrans0;   // 透射率累乘

        if (viewTrans<0.005) break;  // 几乎不透明，提前退出
        tCur+=adaptStep;
    }

    // ── 夜晚整体衰减 ────────────────────────────────────────────────────────
    float nightFade=smoothstep(0.0,0.12,pc.sunVisibility);
    sumCloud*=nightFade;

    // ── alpha = 1 - T ────────────────────────────────────────────────────────
    float alpha=clamp(1.0-viewTrans,0.0,1.0);
    alpha*=mix(0.08,1.0,nightFade);
    if (alpha<0.003) discard;

    // ── 调试输出 ────────────────────────────────────────────────────────────
    if (cloud.uDebug == 2) FragColor=vec4(vec3(clamp(dbgDensity*0.5,0.0,1.0)),1.0);
    else if (cloud.uDebug == 3) FragColor=vec4(clamp(vec3(dbgCoverage,dbgProfile,dbgShape),0.0,1.0),1.0);
    else if (cloud.uDebug == 4) FragColor=vec4(clamp(vec3(alpha,viewTrans,optDepthC*0.08),0.0,1.0),1.0);
    else if (cloud.uDebug == 5) FragColor=vec4(clamp(dbgLighting*0.15,0.0,1.0),1.0);
    else if (cloud.uDebug == 6) FragColor=vec4(vec3(clamp(dbgStepTau,0.0,1.0)),1.0);
    else {
        // ── 正式输出：tone-map → premultiply ────────────────────────────────
        // 先除 alpha 得 unpremultiplied radiance，ACES tone-map，再乘回 alpha。
        // Vulkan blend: ONE / ONE_MINUS_SRC_ALPHA（premultiplied）。
        vec3 cloudColor=sumCloud/max(alpha,1e-4);
        vec3 x=max(cloudColor,vec3(0.0));
        cloudColor=(x*(2.51*x+0.03))/(x*(2.43*x+0.59)+0.14);   // ACES filmic

        FragColor=vec4(cloudColor*alpha,alpha);
    }
}
