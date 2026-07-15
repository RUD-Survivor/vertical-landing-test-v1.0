#version 460
#extension GL_GOOGLE_include_directive : enable

// 云环境天光 1D LUT：ambient(μs)，μs=dot(localUp,sun)∈[-1,1]。
// 观察点固定在云壳中层高度，对上半球天空做余弦加权积分；太阳方向相对 up 扫过 μs。
// 不依赖相机位置/朝向——夜半球与昼半球由 μs 自然编码，无需额外昼夜曲线。
// 依赖 Transmittance + MultiScatter，排在两者之后；分辨率极小，可每帧重烤。
#include "atmosphere_common.glsl"

const int kAmbientRadialSamples = 4;
const int kAmbientAzimuthSamples = 4;
const float kTwoPi = 6.28318530717958647692;

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
    const vec3 worldPos = up * rObs;
    const float sinTheta = sqrt(max(0.0, 1.0 - muS * muS));
    const vec3 sunDir = normalize(vec3(sinTheta, muS, 0.0));

    const vec2 pixPos = vec2(float(x) + 0.5, 0.5);
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

            SingleScatteringResult ss = integrateScatteredLuminance(
                pixPos, worldPos, worldDir, sunDir, atmosphere,
                false, /*sampleCountIni*/ 16.0, /*depth*/ -1.0,
                true, kDefaultMaxT, true, false);
            ambientRadiance += ss.scatteredLight;
        }
    }

    const float sampleCount = float(kAmbientRadialSamples * kAmbientAzimuthSamples);
    ambientRadiance /= sampleCount;
    imageStore(imageCloudAmbientMusLut, ivec2(x, 0), vec4(ambientRadiance, 1.0));
}
