#version 460
#extension GL_GOOGLE_include_directive : enable

// ==========================================================================
// inscatter_lut.glsl — Phase C：Bruneton reduced single scattering 烘焙。
//
// 每个 texel 存无相位 Rayleigh（RGB）+ Mie 亮度（A）；多重散射与相位在
// 运行时由 atmo_scatter_common.glsl 处理（对齐 GetCombinedScattering）。
// UV 映射见 shared_atmosphere.glsl::inscatterTexelToParams。
//
// 生成顺序：transmittance → **inscatter** → multiScatter → skyView → ...
// ==========================================================================

#define NO_MULTISCATAPPROX_ENABLED

#include "atmosphere_common.glsl"

layout(local_size_x = 8, local_size_y = 8, local_size_z = 1) in;

void main()
{
    ivec3 lutSize = imageSize(imageInscatterLut);
    ivec3 workPos = ivec3(gl_GlobalInvocationID.xyz);
    if (workPos.x >= lutSize.x || workPos.y >= lutSize.y || workPos.z >= lutSize.z)
        return;

    AtmosphereParameters atmosphere = getAtmosphereParameters();

    float r, mu, mu_s, nu;
    bool rayIntersectsGround;
    inscatterTexelToParams(atmosphere, workPos, lutSize, r, mu, mu_s, nu, rayIntersectsGround);

    vec3 viewDir, sunDir;
    inscatterDirsFromParams(mu, mu_s, nu, viewDir, sunDir);

    // 行星中心坐标系：观察点在 +Y 轴上（与 Transmittance bake 同约定）
    vec3 worldPos = vec3(0.0, r, 0.0);
    vec2 pixPos = vec2(workPos.xy) + vec2(0.5);

    // 略随高度增加步数；击地射线积分到地面，不击地积到大气顶
    float sampleCount = mix(16.0, 48.0, clamp(workPos.z / float(max(lutSize.z - 1, 1)), 0.0, 1.0));

    SingleScatteringResult ss = integrateScatteredLuminance(
        pixPos,
        worldPos,
        viewDir,
        sunDir,
        atmosphere,
        /*bGround*/ false,
        sampleCount,
        /*depthBufferValue*/ -1.0,
        /*bMieRayPhase*/ false,
        /*tMaxMax*/ kDefaultMaxT,
        /*bVariableSampleCount*/ false,
        /*bBrunetonReducedBake*/ true);

    vec3 rayleigh = min(ss.rayleighLight, vec3(kMaxHalfFloat));
    float mieLuma = min(dot(ss.mieLight, vec3(0.299, 0.587, 0.114)), kMaxHalfFloat);
    imageStore(imageInscatterLut, workPos, vec4(rayleigh, mieLuma));
}
