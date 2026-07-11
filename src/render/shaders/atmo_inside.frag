#version 450
#extension GL_GOOGLE_include_directive : enable
#extension GL_EXT_samplerless_texture_functions : enable

// ==========================================================================
// atmo_inside.frag — 大气壳内路径（相机在大气层以内）
//
// L_out = L_scatter * E + L_scene * T（ONE / ONE_MINUS_SRC_ALPHA）
// 掠射棕带：天空路径按几何地平线抬密度地板；球内从表面出射再积。
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
    int   pathDebug;
    float skyDepthThresh;
    float enableSphereClip;
    float enableDepthClip;
    float sphereClipMaxT;
    int   marchSteps;
    float lowMuScatterMul;
    float lowMuSoft;
    float disableAces;
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

    float camDist = length(camPos - pc.planetCenter.xyz);

    float tNear, tFar;
    if (!intersectSphereAtCenter(camPos, rayDir, pc.planetCenter.xyz, pc.outerRadius, tNear, tFar)) discard;
    tNear = max(tNear, 0.0);
    if (tFar <= tNear) discard;

    float tS0 = -1.0, tS1 = -1.0;
    bool hitSurfRaw = intersectSphereAtCenter(camPos, rayDir, pc.planetCenter.xyz, pc.surfaceRadius, tS0, tS1);

    // 球内：禁止地下积分，从表面出射点往外积（钻地不再整屏掠射棕带）
    if (camDist < pc.surfaceRadius) {
        if (!hitSurfRaw) discard;
        float tExit = (tS0 > 0.0) ? tS0 : tS1;
        if (tExit <= 0.0) discard;
        tNear = max(tNear, tExit);
    }

    bool rayHitsSurface = (camDist >= pc.surfaceRadius) && hitSurfRaw && (tS0 > 0.0);

    // pathDebug: 0 Normal 1 ForceSky 2 ForceGround 3 NoSphere 4 NoDepth 5 VizPath 6 VizInsideWhite
    int mode = pc.pathDebug;
    bool doSphere = (pc.enableSphereClip > 0.5) && (mode != 1) && (mode != 3);
    bool doDepth  = (pc.enableDepthClip  > 0.5) && (mode != 1) && (mode != 4);
    if (mode == 2) { doSphere = true; doDepth = true; }

    float tFarSky = tFar;
    if (doSphere && rayHitsSurface && tS0 < pc.sphereClipMaxT)
        tFar = min(tFar, tS0);

    vec2  screenUv      = vNDC * 0.5 + 0.5;
    float sceneDepthNdc = texture(sampler2D(inSceneDepth, samp), screenUv).r;
    float depthThresh   = pc.skyDepthThresh;
    if (doDepth && sceneDepthNdc < depthThresh) {
        vec4 clip  = vec4(vNDC.x, -vNDC.y, sceneDepthNdc * 2.0 - 1.0, 1.0);
        vec4 viewP = inverse(frame.proj) * clip;
        viewP     /= viewP.w;
        vec4 worldP = inverse(frame.view) * viewP;
        float tScene = length(worldP.xyz - camPos);
        tFar = min(tFar, tScene);
    }

    if (mode == 1)
        tFar = tFarSky;

    bool usedGroundPath = (tFar < tFarSky - 1e-4);

    if (tFar - tNear <= 0.0) discard;

    if (mode == 5) {
        FragColor = usedGroundPath ? vec4(1.0, 0.05, 0.05, 1.0) : vec4(0.1, 0.35, 1.0, 1.0);
        return;
    }

    // 球内染白诊断：InsideSurf 时天空路径=白、地面路径=品红；球外走正常着色
    if (mode == 6) {
        if (camDist < pc.surfaceRadius) {
            FragColor = usedGroundPath ? vec4(1.0, 0.0, 1.0, 1.0) : vec4(1.0, 1.0, 1.0, 1.0);
            return;
        }
    }

    // 天空掠射密度地板：相对几何地平线，带越近 floor 越高，削弱饱和棕带。
    // 不再按 usedGroundPath 二选一（那样在地平线处，命中远处地形的像素和
    // 命中天空的像素会分别拿到 floor=0 / floor=max 两种截然不同的结果，
    // 边界正好卡在地形轮廓线上，就是画面里那条锯齿状硬边的成因）。
    // 改成关于几何地平线对称的钟形权重：只在贴着地平线（|above| 很小）时
    // 抬 floor，无论这条光线最终是打在地面还是穿到天空；仰角大幅偏离地平线
    // （不管朝上看天还是朝下看近处地面）floor 都回落到 0，避免误伤俯视地形。
    float densFloorKm = 0.0;
    float rSafe = max(camDist, pc.surfaceRadius + 1e-3);
    vec3  up = (camPos - pc.planetCenter.xyz) / rSafe;
    float mu = dot(rayDir, up);
    float cosHoriz = -sqrt(max(0.0, 1.0 - (pc.surfaceRadius / rSafe) * (pc.surfaceRadius / rSafe)));
    {
        float above = mu - cosHoriz; // 0=几何地平，>0 仰天，<0 俯视地面
        float softAng = max(pc.lowMuSoft, 0.02);
        float horizWeight = 1.0 - smoothstep(0.0, softAng, abs(above));
        densFloorKm = mix(0.0, 2.0, horizWeight);
    }

    float cosTheta = dot(rayDir, sunDir);
    int steps = clamp(pc.marchSteps, 4, 128);
    AtmoMarchResult march = raymarchAtmoSegment(
        camPos, rayDir, pc.planetCenter.xyz, tNear, tFar, steps,
        PC_RAYLEIGH, pc.mieCoeff, pc.hRayleigh, pc.hMie, pc.gMie,
        PC_OZONE, pc.ozoneCenter, pc.ozoneWidth,
        pc.surfaceRadius, pc.outerRadius, sunDir, cosTheta,
        densFloorKm);

    // 诊断用低仰角倍率（默认 1=关闭）
    {
        float soft = max(pc.lowMuSoft, 1e-4);
        float w = smoothstep(0.0, soft, mu);
        march.scattered *= mix(pc.lowMuScatterMul, 1.0, w);
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

    FragColor = atmoCompositeOut(march.scattered, march.viewT, exposure, pc.disableAces);
}
