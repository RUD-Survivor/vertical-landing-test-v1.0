#version 460
#extension GL_GOOGLE_include_directive : enable

// ==========================================================================
// inscatter_lut.glsl — Phase A：Inscatter 4D（打包 3D）单次散射烘焙。
//
// 每个 texel 对应 Bruneton 参数 (r, mu, mu_s, nu)，用已有
// integrateScatteredLuminance 沿该视线积分（依赖 Transmittance LUT）。
// 多次散射阶迭代留到 Phase D；本 pass 只验证「能烤 / 能采」。
//
// 生成顺序：transmittance → **inscatter** → multiScatter → skyView → ...
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
    inscatterTexelToParams(atmosphere, workPos, lutSize, r, mu, mu_s, nu);

    vec3 viewDir, sunDir;
    inscatterDirsFromParams(mu, mu_s, nu, viewDir, sunDir);

    // 行星中心坐标系：观察点在 +Y 轴上（与 Transmittance bake 同约定）
    vec3 worldPos = vec3(0.0, r, 0.0);
    vec2 pixPos = vec2(workPos.xy) + vec2(0.5);

    // 略随高度增加步数；Phase A 用固定积分足够看出天空形状
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
        /*bMieRayPhase*/ true,
        /*tMaxMax*/ kDefaultMaxT,
        /*bVariableSampleCount*/ false);

    vec3 L = min(ss.scatteredLight, vec3(kMaxHalfFloat));
    imageStore(imageInscatterLut, workPos, vec4(L, 1.0));
}
