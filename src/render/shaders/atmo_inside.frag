#version 450
#extension GL_GOOGLE_include_directive : enable
#extension GL_EXT_samplerless_texture_functions : enable

// ==========================================================================
// atmo_inside.frag — 大气壳内路径（相机在大气层以内）
//
// 第一版统一合成（与 atmo_shell.frag / 云 composite 同公式）：
//   L_out = L_scatter * E + L_scene * T_rgb
// 天空像素（depth=clear）：Sky-View LUT 散射 + skybox/光晕 * T_rgb。
// 有场景深度：GetSkyRadianceToPoint 段 Inscatter LUT + Transmittance LUT 段透射。
// ==========================================================================

#define PI 3.14159265359
#define PRIMARY_STEPS 48

#include "atmo_scatter_common.glsl"

layout(set = 0, binding = 0) uniform FrameUBO {
    mat4  view;
    mat4  proj;
    vec3  lightDir;
    float _pad0;
    vec3  viewPos;
    float time;
} frame;

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
    float rayleighCoeffX, rayleighCoeffY, rayleighCoeffZ;
    float mieCoeff;
    float hRayleigh;
    float hMie;
    float gMie;
    float ozoneCoeffX, ozoneCoeffY, ozoneCoeffZ;
    float ozoneCenter;
    float ozoneWidth;
    float spaceVisStart;   // 保留布局；第一版合成不再用
    float spaceVisEnd;
    float limbBrightness;
    float outerExposure;
    float sunDirX, sunDirY, sunDirZ;
    float innerExposureNear;
    float innerExposureFar;
    float limbSpaceStart;
    float limbSpaceEnd;
    float limbBrightnessBottom;
    float limbPower;
    float skyDepthThreshold;
    float transmittanceChromaStrength;
    float inscatterDebugExposure; // 0=关；>0=Inscatter 天空 debug 曝光
    int   segmentScatterMode;     // 0=Inscatter ToPoint，1=Raymarch
    int   skyScatterMode;         // 0=Sky-View LUT，1=Inscatter 天空
} pc;

#define PC_RAYLEIGH vec3(pc.rayleighCoeffX, pc.rayleighCoeffY, pc.rayleighCoeffZ)
#define PC_OZONE    vec3(pc.ozoneCoeffX, pc.ozoneCoeffY, pc.ozoneCoeffZ)
#define PC_SUNDIR   vec3(pc.sunDirX, pc.sunDirY, pc.sunDirZ)

layout(location = 0) in  vec2 vNDC;
layout(location = 0) out vec4 FragColor;

void main() {
    vec3 camPos = frame.viewPos;
    vec3 sunDir = PC_SUNDIR;

    vec3 viewDir = vec3(
         vNDC.x / frame.proj[0][0],
        -vNDC.y / frame.proj[1][1],
        -1.0
    );
    vec3 rayDir = normalize(transpose(mat3(frame.view)) * viewDir);

    float tNear, tFar;
    if (!intersectSphereAtCenter(camPos, rayDir, pc.planetCenter.xyz, pc.outerRadius, tNear, tFar)) discard;
    tNear = max(tNear, 0.0);
    if (tFar <= tNear) discard;

    float camDist = length(camPos - pc.planetCenter.xyz);

    // 解析球面交点
    float tS0 = -1.0, tS1 = -1.0;
    bool hitSurf = intersectSphereAtCenter(camPos, rayDir, pc.planetCenter.xyz, pc.surfaceRadius, tS0, tS1);

    if (camDist < pc.surfaceRadius) {
        // 球内：禁止地下积分（否则从地下 h=0 积到大气顶 → 整片棕天）。
        // 只从表面出射点往外积。
        if (!hitSurf) discard;
        float tExit = (tS0 > 0.0) ? tS0 : tS1; // 球内一正一负根，取朝前出射
        if (tExit <= 0.0) discard;
        tNear = max(tNear, tExit);
    } else if (hitSurf && tS0 > 0.0) {
        // 球外：撞到行星表面则截断
        tFar = min(tFar, tS0);
    }

    vec2  screenUv      = vNDC * 0.5 + 0.5;
    float sceneDepthNdc = texture(sampler2D(inSceneDepth, samp), screenUv).r;
    float skyDepthThresh = clamp(pc.skyDepthThreshold, 0.0, 1.0);
    if (sceneDepthNdc < skyDepthThresh) {
        vec4 clip  = vec4(vNDC.x, -vNDC.y, sceneDepthNdc * 2.0 - 1.0, 1.0);
        vec4 viewP = inverse(frame.proj) * clip;
        viewP     /= viewP.w;
        vec4 worldP = inverse(frame.view) * viewP;
        float tScene = length(worldP.xyz - camPos);
        tFar = min(tFar, tScene);
    }

    if (tFar - tNear <= 0.0) discard;

    vec3 camRel = camPos - pc.planetCenter.xyz;
    bool isSkyPixel = sceneDepthNdc >= skyDepthThresh;

    AtmoMarchResult march;
    if (isSkyPixel) {
        if (pc.skyScatterMode == 1) {
            march.scattered = atmoSampleInscatterSky(camRel, rayDir, sunDir,
                pc.surfaceRadius, pc.outerRadius, vec3(pc.mieCoeff), pc.gMie);
        } else {
            march.scattered = atmoSampleSkyViewLut(camRel, rayDir, sunDir,
                pc.surfaceRadius, pc.outerRadius);
        }
        march.viewT = atmoSegmentViewTransmittance(camRel, rayDir, tNear, tFar,
            pc.surfaceRadius, pc.outerRadius);
        march.opacity = clamp(1.0 - dot(march.viewT, vec3(0.299, 0.587, 0.114)), 0.0, 1.0);
    } else if (pc.segmentScatterMode == 1) {
        float cosTheta = dot(rayDir, sunDir);
        march = raymarchAtmoSegment(
            camPos, rayDir, pc.planetCenter.xyz, tNear, tFar, PRIMARY_STEPS,
            PC_RAYLEIGH, pc.mieCoeff, pc.hRayleigh, pc.hMie, pc.gMie,
            PC_OZONE, pc.ozoneCenter, pc.ozoneWidth,
            pc.surfaceRadius, pc.outerRadius, sunDir, cosTheta);
    } else {
        march = atmoSegmentInscatterMarch(camRel, rayDir, tNear, tFar, sunDir,
            pc.surfaceRadius, pc.outerRadius, vec3(pc.mieCoeff), pc.gMie);
    }

    float thickness = max(pc.outerRadius - pc.surfaceRadius, 1e-3);
    float altNorm = max(camDist - pc.surfaceRadius, 0.0) / thickness;
    float nightFactor = mix(0.01, 1.0, pc.sunVisibility);
    float baseE = atmoUnifiedExposure(
        camDist, pc.surfaceRadius, pc.outerRadius,
        pc.innerExposureNear, pc.innerExposureFar, pc.outerExposure);
    float rim = atmoInsideRim(rayDir, camPos - pc.planetCenter.xyz);
    float limbAmt = atmoLimbAmount(altNorm, pc.limbBrightnessBottom, pc.limbBrightness);
    float exposure = baseE * atmoLimbMul(limbAmt, rim, pc.limbPower) * nightFactor;

    vec3 sceneColor = texture(sampler2D(inSceneColor, samp), screenUv).rgb;

    // Phase A debug：整屏显示 Inscatter 天空采样（验证烘焙/查表通路）
    if (pc.inscatterDebugExposure > 0.0) {
        vec3 Ldbg = atmoSampleInscatterSky(camRel, rayDir, sunDir,
            pc.surfaceRadius, pc.outerRadius, vec3(pc.mieCoeff), pc.gMie);
        vec3 x = max(Ldbg * pc.inscatterDebugExposure, vec3(0.0));
        vec3 toned = (x * (2.51 * x + 0.03)) / (x * (2.43 * x + 0.59) + 0.14);
        FragColor = vec4(toned, 1.0);
        return;
    }

    FragColor = atmoCompositeOut(march.scattered, march.viewT, exposure, sceneColor,
        pc.transmittanceChromaStrength);
}
