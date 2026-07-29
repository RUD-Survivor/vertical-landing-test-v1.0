#version 460
#extension GL_GOOGLE_include_directive : enable

// ==========================================================================
// inscatter_lut.glsl — Bruneton reduced scattering 烘焙（单次 + 多重）。
//
// 每个 texel：RGB = Rayleigh_reduced + Multi / RayleighPhase(ν)
//           A   = 单次 Mie.r（无相位，供 Bruneton combined 外推）
// 独立 SingleMie LUT 同时保存完整 RGB，供 tuner 精确模式选择。
// 运行时 atmoApplyBrunetonPhases：RGB×phaseR + Mie×phaseM，与
// Bruneton COMBINED_SCATTERING_TEXTURES 同构（多重已折进 RGB）。
// UV 映射见 shared_atmosphere.glsl::inscatterTexelToParams。
//
// 生成顺序：transmittance → multiScatter → **inscatter** → skyView → ...
// 下游除了 atmo_inside/atmo_shell，还有 cloud_ambient_mus_lut（查这两张表算云环境天光）。
// ==========================================================================

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

    // 行星中心坐标系：观察点在 +Y 轴上（与 Transmittance bake 同约定）。
    // 严格夹在 top 内侧，避免 r==topRadius 时 raySphere 最近根 t=0 导致整 texel 烘黑。
    float rTrace = min(r, atmosphere.topRadius - max(kPlanetRadiusOffset, 0.01));
    rTrace = max(rTrace, atmosphere.bottomRadius + kPlanetRadiusOffset);
    vec3 worldPos = vec3(0.0, rTrace, 0.0);
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

    // Bruneton：多重进 RGB 时除以 Rayleigh 相位，运行时再乘回。
    float phaseR = max(rayleighPhase(nu), 1e-4);
    vec3 rayleigh = min(ss.rayleighLight + ss.multiLight / phaseR, vec3(kMaxHalfFloat));
    vec3 singleMie = min(ss.mieLight, vec3(kMaxHalfFloat));
    imageStore(imageInscatterLut, workPos, vec4(rayleigh, singleMie.r));
    imageStore(imageSingleMieLut, workPos, vec4(singleMie, 1.0));
}
