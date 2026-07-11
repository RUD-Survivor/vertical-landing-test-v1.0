#version 460
#extension GL_GOOGLE_include_directive : enable

// ==========================================================================
// aerial_perspective_lut.glsl — Froxel 3D 体积散射 LUT 烘焙。
//
// 对齐 flower air_perspective.glsl：
//   https://github.com/qiutang98/flower/blob/main/source/shader/atmosphere/air_perspective.glsl
//
// 差异（行星尺度）：flower 用 convertToAtmosphereUnit + (0, bottomRadius, 0) 的
// Y-up 近地假设；RS3D 的 camWorldPos 已是相对行星中心的偏移，convert 后 length
// 即为地心距，不再加 bottomRadius。
//
// 供 cloud_common.glsl::inFroxelScatter 空气透视消费。
// ==========================================================================

#include "atmosphere_common.glsl"

// 32 x 32 x 32 Dimension（与 flower / VkAtmosphereLut::kFroxel* 一致）
layout(local_size_x=8, local_size_y=8, local_size_z=1) in;

void main()
{
    ivec3 lutSize = imageSize(imageFroxelScatter);
    ivec3 workPos = ivec3(gl_GlobalInvocationID.xyz);

    if (workPos.x >= lutSize.x || workPos.y >= lutSize.y || workPos.z >= lutSize.z)
    {
        return;
    }

    AtmosphereParameters atmosphere = getAtmosphereParameters();

    const vec2 pixPos = vec2(workPos.xy) + vec2(0.5f);
    const vec2 uv = pixPos / vec2(lutSize.xy);

    vec4 clipSpace = vec4(uv.x * 2.0f - 1.0f, 1.0f - uv.y * 2.0f, 0.0, 1.0);
    vec4 viewPosH = frameData.camInvertProj * clipSpace;
    vec3 viewDir = viewPosH.xyz / viewPosH.w;
    vec3 worldDir = normalize((frameData.camInvertView * vec4(viewDir, 0.0)).xyz);

    // 行星中心坐标系：不要照抄 flower 的 +vec3(0, bottomRadius, 0)
    vec3 worldPos = convertToAtmosphereUnit(frameData.camWorldPos.xyz);
    vec3 sunDir = -normalize(frameData.sky.direction);

    // [0, 1) → 平方分布 → 线性 slice 索引（与 cloud_common 采样 w=sqrt(slice/N) 互逆）
    float slice = ((float(workPos.z) + 0.5f) / float(lutSize.z));
    slice *= slice;
    slice *= float(lutSize.z);

    float viewHeight;

    // Compute position from froxel information
    float tMax = aerialPerspectiveSliceToDepth(slice);
    vec3 newWorldPos = worldPos + tMax * worldDir;

    // If the voxel is under the ground, offset it out onto the ground.
    viewHeight = length(newWorldPos);
    if (viewHeight <= (atmosphere.bottomRadius + kPlanetRadiusOffset))
    {
        newWorldPos = normalize(newWorldPos) * (atmosphere.bottomRadius + kPlanetRadiusOffset + 0.001f);
        worldDir = normalize(newWorldPos - worldPos);
        tMax = length(newWorldPos - worldPos);
    }
    float tMaxMax = tMax;

    // Move ray marching start up to top atmosphere (camera in deep space / orbit).
    viewHeight = length(worldPos);
    if (viewHeight >= atmosphere.topRadius)
    {
        vec3 prevWorldPos = worldPos;
        if (!moveToTopAtmosphere(worldPos, worldDir, atmosphere.topRadius))
        {
            imageStore(imageFroxelScatter, workPos, vec4(0.0, 0.0, 0.0, 1.0));
            return;
        }

        float lengthToAtmosphere = length(prevWorldPos - worldPos);
        if (tMaxMax < lengthToAtmosphere)
        {
            imageStore(imageFroxelScatter, workPos, vec4(0.0, 0.0, 0.0, 1.0));
            return;
        }

        tMaxMax = max(0.0, tMaxMax - lengthToAtmosphere);
    }

    // flower：深 slice 用更多步数；近处 slice 少步即可
    const bool bGround = false;
    const float sampleCountIni = max(1.0, float(workPos.z + 1.0) * 2.0f);
    const float depthBufferValue = -1.0;
    const bool bVariableSampleCount = false;
    const bool bMieRayPhase = true;

    SingleScatteringResult ss = integrateScatteredLuminance(
        pixPos,
        worldPos,
        worldDir,
        sunDir,
        atmosphere,
        bGround,
        sampleCountIni,
        depthBufferValue,
        bMieRayPhase,
        tMaxMax,
        bVariableSampleCount
    );

    ss.scatteredLight = min(ss.scatteredLight, vec3(kMaxHalfFloat));

    // flower：α = 1 - mean(transmittance)
    imageStore(imageFroxelScatter, workPos, vec4(ss.scatteredLight, 1.0 - mean(ss.transmittance)));
}
