#version 450
#extension GL_GOOGLE_include_directive : enable
#extension GL_EXT_samplerless_texture_functions : enable

// ==========================================================================
// atmo_inside.frag — 大气壳内路径（相机在大气层以内）
//
// 第一版统一合成（与 atmo_shell.frag / 云 composite 同公式）：
//   L_out = L_scatter * E + L_scene * T
// 其中 T = luma(viewT)，靠管线 ONE / ONE_MINUS_SRC_ALPHA + LOAD 已有场景色实现，
// 不读 HDR（色度透射留第二版）。全程 raymarch，不再用 Sky-View LUT 天空硬分支。
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

    float tS0 = -1.0, tS1;
    bool rayHitsSurface = intersectSphereAtCenter(camPos, rayDir, pc.planetCenter.xyz, pc.surfaceRadius, tS0, tS1) && tS0 > 0.0;
    if (rayHitsSurface)
        tFar = min(tFar, tS0);

    vec2  screenUv      = vNDC * 0.5 + 0.5;
    float sceneDepthNdc = texture(sampler2D(inSceneDepth, samp), screenUv).r;
    if (sceneDepthNdc < 0.9999) {
        vec4 clip  = vec4(vNDC.x, -vNDC.y, sceneDepthNdc * 2.0 - 1.0, 1.0);
        vec4 viewP = inverse(frame.proj) * clip;
        viewP     /= viewP.w;
        vec4 worldP = inverse(frame.view) * viewP;
        float tScene = length(worldP.xyz - camPos);
        tFar = min(tFar, tScene);
    }

    if (tFar - tNear <= 0.0) discard;

    float cosTheta = dot(rayDir, sunDir);
    AtmoMarchResult march = raymarchAtmoSegment(
        camPos, rayDir, pc.planetCenter.xyz, tNear, tFar, PRIMARY_STEPS,
        PC_RAYLEIGH, pc.mieCoeff, pc.hRayleigh, pc.hMie, pc.gMie,
        PC_OZONE, pc.ozoneCenter, pc.ozoneWidth,
        pc.surfaceRadius, pc.outerRadius, sunDir, cosTheta);

    float camDist = length(camPos - pc.planetCenter.xyz);
    float thickness = max(pc.outerRadius - pc.surfaceRadius, 1e-3);
    float altNorm = max(camDist - pc.surfaceRadius, 0.0) / thickness;
    float nightFactor = mix(0.01, 1.0, pc.sunVisibility);
    float baseE = atmoUnifiedExposure(
        camDist, pc.surfaceRadius, pc.outerRadius,
        pc.innerExposureNear, pc.innerExposureFar, pc.outerExposure);
    float rim = atmoInsideRim(rayDir, camPos - pc.planetCenter.xyz);
    float limbAmt = atmoLimbAmount(altNorm, pc.limbBrightnessBottom, pc.limbBrightness);
    float exposure = baseE * atmoLimbMul(limbAmt, rim, pc.limbPower) * nightFactor;

    FragColor = atmoCompositeOut(march.scattered, march.viewT, exposure);
}
