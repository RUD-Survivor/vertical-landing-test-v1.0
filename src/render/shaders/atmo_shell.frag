#version 450
#extension GL_GOOGLE_include_directive : enable
#extension GL_EXT_samplerless_texture_functions : enable

// ==========================================================================
// atmo_shell.frag — 大气壳外路径（相机在大气层以外看行星）
//
// 合成与 atmo_inside.frag 同式：
//   L_out = L_scatter * E + L_scene * T
// 散射：GetSkyRadianceToPoint 段 Inscatter LUT；曝光用 atmoUnifiedExposure()。
//
// 壳外盘面（F4 shellDiskMode）：
//   0 Disk AP：场景深度截 [tNear,tFar]
//   1 Legacy ：场景 NDC 比壳片近则 discard（盘心无 AP）
//   2 Blend  ：出大气后 Depth↔解析球渐进混合（消远距同心纹）
// ==========================================================================

#define PRIMARY_STEPS 32

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
    float spaceVisStart;
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
    float inscatterDebugExposure;
    int   segmentScatterMode;     // 0=Inscatter ToPoint，1=Raymarch
    int   skyScatterMode;         // 壳外未用；与 inside 对齐布局
    int   toPointDebugMode;       // 0=关；1..8 ToPoint 中间量
    float toPointDebugExposure;
    int   shellDiskMode;          // 0=Disk AP；1=legacy discard；2=Depth↔解析球混合
    float depthApBlendStartKm;    // mode2：相对 outer 的混合起点 km
    float depthApBlendEndKm;      // mode2：相对 outer 的混合终点 km
    int   mieStorageMode;         // 0=combined 外推；1=完整 RGB Mie LUT
} pc;

#define PC_RAYLEIGH vec3(pc.rayleighCoeffX, pc.rayleighCoeffY, pc.rayleighCoeffZ)
#define PC_OZONE    vec3(pc.ozoneCoeffX, pc.ozoneCoeffY, pc.ozoneCoeffZ)
#define PC_SUNDIR   vec3(pc.sunDirX, pc.sunDirY, pc.sunDirZ)

layout(location = 0) in  vec3 vWorldPos;
layout(location = 1) in  vec3 vLocalNormal;
layout(location = 0) out vec4 FragColor;

void main() {
    vec3 camPos = frame.viewPos;
    vec3 sunDir = PC_SUNDIR;

    vec3  toCam          = camPos - vWorldPos;
    float distToCam       = length(toCam);
    vec3  viewDirFromFrag = toCam / max(distToCam, 1e-4);

    if (dot(vLocalNormal, viewDirFromFrag) < 0.0) discard;

    vec4  clip = frame.proj * frame.view * vec4(vWorldPos, 1.0);
    float myNdcDepth = (clip.z / clip.w) * 0.5 + 0.5;
    vec2  ndcXY    = clip.xy / clip.w;
    vec2  screenUv = vec2(ndcXY.x, -ndcXY.y) * 0.5 + 0.5;
    float sceneDepthNdc = texture(sampler2D(inSceneDepth, samp), screenUv).r;

    // legacy：场景比壳片近则整像素丢弃（盘心无空气透视）
    // ToPoint debug 始终走 Disk AP，才能看太空→地表弦
    bool useLegacyDiscard = (pc.shellDiskMode == 1) && (pc.toPointDebugMode == 0);
    if (useLegacyDiscard && sceneDepthNdc < myNdcDepth) discard;

    vec3 planetCenter = pc.planetCenter.xyz;
    vec3 rayDir = -viewDirFromFrag;

    float tNear = distToCam;
    float tFar  = tNear;
    {
        float t0, t1;
        if (intersectSphereAtCenter(camPos, rayDir, planetCenter, pc.outerRadius, t0, t1)) {
            tNear = max(min(t0, t1), 0.0);
            tFar  = max(t0, t1);
        }
        // 对齐壳内：撞行星表面则截断（太空盘心弦）
        float tS0, tS1;
        if (intersectSphereAtCenter(camPos, rayDir, planetCenter, pc.surfaceRadius, tS0, tS1)
            && tS0 > 0.0) {
            tFar = min(tFar, tS0);
        }
    }

    // Disk AP / Blend：用场景深度截断；遮挡物在大气入口前则丢弃
    // mode 0：全深；mode 2：出大气后混到解析球；mode 1 走上方 legacy discard
    float camDist = length(camPos - planetCenter);
    if (!useLegacyDiscard) {
        float skyDepthThresh = clamp(pc.skyDepthThreshold, 0.0, 1.0);
        if (sceneDepthNdc < skyDepthThresh) {
            vec2 vulkanNdc = screenUv * 2.0 - 1.0;
            vec4 depthClip = vec4(vulkanNdc.x, -vulkanNdc.y, sceneDepthNdc * 2.0 - 1.0, 1.0);
            vec4 viewP = inverse(frame.proj) * depthClip;
            viewP /= viewP.w;
            vec4 worldP = inverse(frame.view) * viewP;
            float tScene = max(dot(worldP.xyz - camPos, rayDir), 0.0);
            int diskMode = (pc.shellDiskMode == 2) ? 2 : 0;
            if (!atmoApplyDepthFarMix(tFar, tNear, tScene, camDist, pc.outerRadius,
                    diskMode, pc.depthApBlendStartKm, pc.depthApBlendEndKm))
                discard;
        }
    }

    if (tFar <= tNear) discard;

    vec3 camRel = camPos - planetCenter;
    AtmoMarchResult march;
    if (pc.toPointDebugMode > 0) {
        march = atmoSegmentInscatterMarch(
            camRel, rayDir, tNear, tFar, sunDir,
            pc.surfaceRadius, pc.outerRadius, PC_RAYLEIGH, vec3(pc.mieCoeff),
            pc.gMie, pc.mieStorageMode, pc.toPointDebugMode);
        vec3 x = march.scattered;
        if (pc.toPointDebugMode != 3 && pc.toPointDebugMode != 7 && pc.toPointDebugMode != 8)
            x = max(x * pc.toPointDebugExposure, vec3(0.0));
        if (pc.toPointDebugMode == 3 || pc.toPointDebugMode == 7 || pc.toPointDebugMode == 8)
            FragColor = vec4(clamp(x, vec3(0.0), vec3(1.0)), 1.0);
        else {
            vec3 toned = (x * (2.51 * x + 0.03)) / (x * (2.43 * x + 0.59) + 0.14);
            FragColor = vec4(toned, 1.0);
        }
        return;
    }

    if (pc.segmentScatterMode == 1) {
        float cosTheta = dot(rayDir, sunDir);
        march = raymarchAtmoSegment(
            camPos, rayDir, planetCenter, tNear, tFar, PRIMARY_STEPS,
            PC_RAYLEIGH, pc.mieCoeff, pc.hRayleigh, pc.hMie, pc.gMie,
            PC_OZONE, pc.ozoneCenter, pc.ozoneWidth,
            pc.surfaceRadius, pc.outerRadius, sunDir, cosTheta);
    } else {
        march = atmoSegmentInscatterMarch(
            camRel, rayDir, tNear, tFar, sunDir,
            pc.surfaceRadius, pc.outerRadius, PC_RAYLEIGH, vec3(pc.mieCoeff),
            pc.gMie, pc.mieStorageMode, 0);
    }

    float thickness = max(pc.outerRadius - pc.surfaceRadius, 1e-3);
    float altNorm = max(camDist - pc.surfaceRadius, 0.0) / thickness;
    float nightFactor = mix(0.05, 1.0, pc.sunVisibility);
    float baseE = atmoUnifiedExposure(
        camDist, pc.surfaceRadius, pc.outerRadius,
        pc.innerExposureNear, pc.innerExposureFar, pc.outerExposure);
    // 壳法线掠射：正对盘面 rim≈0，轮廓 rim≈1
    float rim = clamp(1.0 - abs(dot(vLocalNormal, viewDirFromFrag)), 0.0, 1.0);
    float limbAmt = atmoLimbAmount(altNorm, pc.limbBrightnessBottom, pc.limbBrightness);
    float exposure = baseE * atmoLimbMul(limbAmt, rim, pc.limbPower) * nightFactor;

    vec3 sceneColor = texture(sampler2D(inSceneColor, samp), screenUv).rgb;
    FragColor = atmoCompositeOut(march.scattered, march.viewT, exposure, sceneColor,
        pc.transmittanceChromaStrength);
}
