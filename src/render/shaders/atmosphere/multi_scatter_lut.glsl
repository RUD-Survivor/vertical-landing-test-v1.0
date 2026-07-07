#version 460
#extension GL_GOOGLE_include_directive : enable

// Multi-Scatter LUT：对每个 (太阳天顶角 cos, 视高度) texel，在整个球面方向上
// 采样 64 条射线求平均多次散射贡献（幂级数求和近似，见文末推导）。依赖
// Transmittance LUT，须在其之后烘焙；每帧重新烘焙，理由同 transmittance_lut.glsl。
#define NO_MULTISCATAPPROX_ENABLED
#include "atmosphere_common.glsl"

const uint kSqrtSampleCount = 8;
const uint kSampleCount = kSqrtSampleCount * kSqrtSampleCount;

shared vec3 sharedMultiScatAs1[kSampleCount];
shared vec3 sharedScatterLight[kSampleCount];

layout (local_size_x = 1, local_size_y = 1, local_size_z = kSampleCount) in;
void main()
{
    ivec2 lutSize = imageSize(imageMultiScatterLut);
    ivec2 workPos = ivec2(gl_GlobalInvocationID.xy);
    const uint flattenId = gl_GlobalInvocationID.z;

    AtmosphereParameters atmosphere = getAtmosphereParameters();

    const vec2 pixPos = vec2(workPos) + vec2(0.5f);
    vec2 uv = pixPos / vec2(lutSize);
    uv = vec2(fromSubUvsToUnit(uv.x, lutSize.x), fromSubUvsToUnit(uv.y, lutSize.y));

    float cosSunZenithAngle = uv.x * 2.0 - 1.0;
    vec3 sunDir = vec3(0.0, cosSunZenithAngle, sqrt(saturate(1.0 - cosSunZenithAngle * cosSunZenithAngle)));

    // 按 kPlanetRadiusOffset 修正，确保 viewHeight 落在有效范围内。
    float viewHeight = atmosphere.bottomRadius + saturate(uv.y + kPlanetRadiusOffset) * (atmosphere.topRadius - atmosphere.bottomRadius - kPlanetRadiusOffset);

    vec3 worldPos = vec3(0.0f, viewHeight, 0.0f);
    vec3 worldDir = vec3(0.0f, 1.0f, 0.0f);

    const bool bGround = true;
    const float sampleCountIni = 20;
    const float depthBufferValue = -1.0;
    const bool bMieRayPhase = false;
    const float tMaxMax = kDefaultMaxT;
    const bool bVariableSampleCount = false;
    const float sphereSolidAngle = 4.0 * kPI;
    const float isotropicPhase = 1.0 / sphereSolidAngle;
    const float sqrtSample = float(kSqrtSampleCount);

    float i = 0.5 + float(flattenId / kSqrtSampleCount);
    float j = 0.5 + float(flattenId - float((flattenId / kSqrtSampleCount) * kSqrtSampleCount));

    {
        float randA = i / sqrtSample;
        float randB = j / sqrtSample;
        float theta = 2.0f * kPI * randA;
        // 球面均匀取点 https://mathworld.wolfram.com/SpherePointPicking.html
        float phi = acos(1.0f - 2.0f * randB);

        float cosPhi = cos(phi);
        float sinPhi = sin(phi);
        float cosTheta = cos(theta);
        float sinTheta = sin(theta);

        worldDir.x = cosTheta * sinPhi;
        worldDir.y = cosPhi;
        worldDir.z = sinTheta * sinPhi;

        SingleScatteringResult result = integrateScatteredLuminance(
            pixPos, worldPos, worldDir, sunDir, atmosphere,
            bGround, sampleCountIni, depthBufferValue, bMieRayPhase, tMaxMax, bVariableSampleCount
        );

        sharedMultiScatAs1[flattenId] = result.multiScatAs1 * sphereSolidAngle / (sqrtSample * sqrtSample);
        sharedScatterLight[flattenId] = result.scatteredLight * sphereSolidAngle / (sqrtSample * sqrtSample);
    }

    groupMemoryBarrier();
    barrier();

    // 并行规约求和（64 -> 1）。
    uint loopIndex = kSampleCount / 2;
    while (loopIndex > 0)
    {
        if (flattenId < loopIndex)
        {
            sharedMultiScatAs1[flattenId] += sharedMultiScatAs1[flattenId + loopIndex];
            sharedScatterLight[flattenId] += sharedScatterLight[flattenId + loopIndex];
        }
        loopIndex /= 2;
        groupMemoryBarrier();
        barrier();
    }
    if (flattenId > 0) return;

    // MultiScatAs1 是"假设整个球面入射辐射度积分为 1"时，单次弹射能散射出的能量比例 r。
    // 把所有高阶散射看成等比级数 sum r^n = 1/(1-r)，见 Frostbite/UE 的推导。
    const vec3 r = sharedMultiScatAs1[flattenId] * isotropicPhase;
    vec3 inScatteredLuminance = sharedScatterLight[flattenId] * isotropicPhase;
    const vec3 sumOfAllMultiScatteringEventsContribution = 1.0f / (1.0 - r);
    vec3 L = inScatteredLuminance * sumOfAllMultiScatteringEventsContribution;

    vec3 result = min(L * atmosphere.multipleScatteringFactor, vec3(kMaxHalfFloat));
    imageStore(imageMultiScatterLut, workPos, vec4(result, 1.0f));
}
