#version 460
#extension GL_GOOGLE_include_directive : enable

// 云环境天光 1D LUT：ambient(μs)，μs=dot(localUp,sun)∈[-1,1]。
// 观察点固定在云壳中层高度，对上半球天空做余弦加权积分；太阳方向相对 up 扫过 μs。
// 不依赖相机位置/朝向——夜半球与昼半球由 μs 自然编码，无需额外昼夜曲线。
//
// 天空辐亮度不再逐方向重新 raymarch，而是查 Inscatter/SingleMie（Bruneton reduced
// scattering）再补相位——一次 LUT 采样代替一整条大气积分，把这张表的成本从
// 128×16 条积分降到纯查表，方向数因此可以给到 8×8。
// 生成顺序必须排在 inscatter_lut 之后（见 bakeFull / bakeAmbientMusLut）。
#include "atmosphere_common.glsl"

const int kAmbientRadialSamples = 8;
const int kAmbientAzimuthSamples = 8;
const float kTwoPi = 6.28318530717958647692;

// Inscatter 的 μs 轴下界是 kInscatterMuSMin(-0.2)，再往下整个夜半球会全部钳到
// 同一层、颜色锁死。用一段 smoothstep 把 μs 低于下界后的环境光继续压到 0：
// 端点导数为 0，在 μs=kInscatterMuSMin 处与查表值 C1 连续；-0.5 大致对应
// 天文昏影终了，此时无月光模型的夜空环境光本就该趋近 0。
const float kAmbientNightFadeEnd = -0.5;

// 天空某一方向的辐亮度：查 Bruneton reduced LUT + 运行时补相位。
// 与 integrateScatteredLuminance(bMieRayPhase=true) 等价：
//   combined RGB × RayleighPhase(ν) = 单次 Rayleigh×phaseR + 多重散射（各向同性）
//   SingleMie RGB × hgPhase(g,-ν)   = 单次 Mie
vec3 sampleSkyRadiance(
    in const AtmosphereParameters atmosphere,
    float rObs, vec3 up, vec3 worldDir, vec3 sunDir, float muS)
{
    const float mu = dot(worldDir, up);
    const float muSLut = max(muS, kInscatterMuSMin);

    // ν 与 (μ, μs) 的几何相容上界。查表侧本来也会做同样的钳制，这里提前算出来，
    // 保证喂给相位函数的 ν 和实际查到的那个 texel 是同一个角度。
    const float muBound = mu * muSLut
        + sqrt(max(0.0, (1.0 - mu * mu) * (1.0 - muSLut * muSLut)));
    const float nu = clamp(dot(worldDir, sunDir), -muBound, muBound);

    // 半球采样方向恒有 μ>0，从 r>bottomRadius 出发不可能与地面相交。
    const bool hitGround = false;

    BrunetonReducedScattering reduced = sampleBrunetonReducedInscatterLut(
        inInscatterLut, linearClampSampler, atmosphere,
        rObs, mu, muSLut, nu, hitGround);
    const vec3 singleMie = sampleInscatterLut(
        inSingleMieLut, linearClampSampler, atmosphere,
        rObs, mu, muSLut, nu, hitGround);

    return reduced.scattering * rayleighPhase(nu)
        + singleMie * hgPhase(atmosphere.miePhaseG, -nu);
}

layout (local_size_x = 64, local_size_y = 1, local_size_z = 1) in;
void main()
{
    const ivec2 lutSize = imageSize(imageCloudAmbientMusLut);
    const int x = int(gl_GlobalInvocationID.x);
    if (x >= lutSize.x) return;

    // 纹素中心 → μs ∈ [-1, 1]（-1=太阳在天底，+1=太阳在天顶）
    const float u = (float(x) + 0.5) / float(lutSize.x);
    const float muS = clamp(u * 2.0 - 1.0, -1.0, 1.0);

    AtmosphereParameters atmosphere = getAtmosphereParameters();
    const float cloudMid = atmosphere.cloudAreaStartHeight
        + 0.5 * atmosphere.cloudAreaThickness;
    const float rObs = max(cloudMid, atmosphere.bottomRadius + 0.01);

    // 规范系：up=+Y，太阳落在 XY 平面，dot(up,sun)=μs
    const vec3 up = vec3(0.0, 1.0, 0.0);
    const float sinTheta = sqrt(max(0.0, 1.0 - muS * muS));
    const vec3 sunDir = normalize(vec3(sinTheta, muS, 0.0));

    vec3 ambientRadiance = vec3(0.0);

    // 余弦加权半球采样，概率密度 p(ω)=cosθ/π。
    // 因此样本均值直接估计 E_sky/π：
    //   E_sky/π = (1/π)∫ L_sky(ω) cosθ dω = E_p[L_sky(ω)]。
    // 每个分层取纹素中心，并让相邻径向环错开半个方位格，减轻规则采样条纹。
    for (int radial = 0; radial < kAmbientRadialSamples; ++radial)
    {
        const float u1 = (float(radial) + 0.5) / float(kAmbientRadialSamples);
        const float horizontalRadius = sqrt(u1);
        const float cosTheta = sqrt(1.0 - u1);
        const float azimuthOffset = 0.5 * float(radial & 1);

        for (int azimuth = 0; azimuth < kAmbientAzimuthSamples; ++azimuth)
        {
            const float u2 = (float(azimuth) + 0.5 + azimuthOffset)
                / float(kAmbientAzimuthSamples);
            const float phi = kTwoPi * u2;
            const vec3 worldDir = vec3(
                horizontalRadius * cos(phi),
                cosTheta,
                horizontalRadius * sin(phi));

            ambientRadiance += sampleSkyRadiance(
                atmosphere, rObs, up, worldDir, sunDir, muS);
        }
    }

    const float sampleCount = float(kAmbientRadialSamples * kAmbientAzimuthSamples);
    const float nightFade = smoothstep(kAmbientNightFadeEnd, kInscatterMuSMin, muS);
    ambientRadiance = clamp(ambientRadiance * (nightFade / sampleCount),
        vec3(0.0), vec3(kMaxHalfFloat));

    imageStore(imageCloudAmbientMusLut, ivec2(x, 0), vec4(ambientRadiance, 1.0));
}
