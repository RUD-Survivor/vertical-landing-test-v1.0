#version 460
#extension GL_GOOGLE_include_directive : enable

// 云环境天光 1D LUT：ambient(μs)，μs=dot(localUp,sun)∈[-1,1]。
// 观察点固定在云壳中层高度、天顶方向积分；太阳方向相对 up 扫过 μs。
// 不依赖相机位置/朝向——夜半球与昼半球由 μs 自然编码，无需额外昼夜曲线。
// 依赖 Transmittance + MultiScatter，排在两者之后；分辨率极小，可每帧重烤。
#include "atmosphere_common.glsl"

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
    const vec3 worldDir = up; // 沿当地天顶看向大气顶

    const vec2 pixPos = vec2(float(x) + 0.5, 0.5);
    SingleScatteringResult ss = integrateScatteredLuminance(
        pixPos, worldPos, worldDir, sunDir, atmosphere,
        false, /*sampleCountIni*/ 16.0, /*depth*/ -1.0,
        true, kDefaultMaxT, true);

    imageStore(imageCloudAmbientMusLut, ivec2(x, 0), vec4(ss.scatteredLight, 1.0));
}
