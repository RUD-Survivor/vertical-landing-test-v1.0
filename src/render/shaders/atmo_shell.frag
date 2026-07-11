#version 450
#extension GL_GOOGLE_include_directive : enable
#extension GL_EXT_samplerless_texture_functions : enable

// ==========================================================================
// atmo_shell.frag — 大气壳外路径（相机在大气层以外看行星）
//
// 第一版统一合成（与 atmo_inside.frag 同公式）：
//   L_out = L_scatter * E + L_scene * T
// 曝光用 atmoUnifiedExposure() 与壳内连续；limbBoost 只做轻微掠射增强。
// 几何仍是壳 mesh（像素覆盖优化），合成不再与壳内分叉。
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
    float _pad3;
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

    {
        vec4  clip = frame.proj * frame.view * vec4(vWorldPos, 1.0);
        float myNdcDepth = (clip.z / clip.w) * 0.5 + 0.5;
        vec2  ndcXY    = clip.xy / clip.w;
        vec2  screenUv = vec2(ndcXY.x, -ndcXY.y) * 0.5 + 0.5;
        float sceneDepthNdc = texture(sampler2D(inSceneDepth, samp), screenUv).r;
        if (sceneDepthNdc < myNdcDepth) discard;
    }

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
        float tS0, tS1;
        if (intersectSphereAtCenter(camPos, rayDir, planetCenter, pc.surfaceRadius, tS0, tS1) && tS0 > tNear) {
            tFar = min(tFar, tS0);
        }
    }

    if (tFar <= tNear) discard;

    float cosTheta = dot(rayDir, sunDir);
    AtmoMarchResult march = raymarchAtmoSegment(
        camPos, rayDir, planetCenter, tNear, tFar, PRIMARY_STEPS,
        PC_RAYLEIGH, pc.mieCoeff, pc.hRayleigh, pc.hMie, pc.gMie,
        PC_OZONE, pc.ozoneCenter, pc.ozoneWidth,
        pc.surfaceRadius, pc.outerRadius, sunDir, cosTheta);

    float camDist = length(camPos - planetCenter);
    float thickness = max(pc.outerRadius - pc.surfaceRadius, 1e-3);
    float altNorm = max(camDist - pc.surfaceRadius, 0.0) / thickness;

    float nightFactor = mix(0.05, 1.0, pc.sunVisibility);
    float baseE = atmoUnifiedExposure(
        camDist, pc.surfaceRadius, pc.outerRadius,
        pc.innerExposureNear, pc.innerExposureFar, pc.outerExposure);

    // 掠射增强：边界正面(grazing≈0)保持 baseE，与壳内连续；越出壳后 limb 才拉高
    float grazing   = 1.0 - abs(dot(vLocalNormal, viewDirFromFrag));
    float limb      = mix(1.0, max(pc.limbBrightness, 1.0), pow(clamp(grazing, 0.0, 1.0), 2.0));
    float outsideW  = smoothstep(0.9, 1.2, altNorm);
    float exposure  = baseE * mix(1.0, limb, outsideW) * nightFactor;

    FragColor = atmoCompositeOut(march.scattered, march.viewT, exposure);
}
