#version 460
#extension GL_GOOGLE_include_directive : enable

// Transmittance LUT：给定 (视高度, 太阳天顶角 cos) 查大气到该点的透射率。
// 每帧对"当前云所在行星"重新烘焙一次（frameData.sky.atmosphereConfig 已按最近
// 行星刷新），解决飞船从地表起飞到深空时行星尺度突变的问题——不能像 flower
// 原版那样只在加载时静态烘焙一次。
//
// 生成顺序（必须先于 multi_scatter_lut / skyview_lut / sky_irradiance_capture）：
//   transmittance_lut → multi_scatter_lut → skyview_lut → sky_irradiance_capture
#define NO_MULTISCATAPPROX_ENABLED
#include "atmosphere_common.glsl"

layout (local_size_x = 8, local_size_y = 8) in;
void main()
{
    ivec2 lutSize = imageSize(imageTransmittanceLut);
    ivec2 workPos = ivec2(gl_GlobalInvocationID.xy);
    if (workPos.x >= lutSize.x || workPos.y >= lutSize.y) return;

    AtmosphereParameters atmosphere = getAtmosphereParameters();

    const vec2 pixPos = vec2(workPos) + vec2(0.5f);
    const vec2 uv = pixPos / vec2(lutSize);

    float viewHeight;
    float viewZenithCosAngle;
    uvToLutTransmittanceParams(atmosphere, viewHeight, viewZenithCosAngle, uv);

    const vec3 worldPos = vec3(0.0f, viewHeight, 0.0f);
    const vec3 worldDir = vec3(0.0f, viewZenithCosAngle, sqrt(1.0 - viewZenithCosAngle * viewZenithCosAngle));
    const vec3 sunDir = -normalize(frameData.sky.direction);

    vec3 opticalDepth = integrateScatteredLuminance(
        pixPos, worldPos, worldDir, sunDir, atmosphere,
        /*bGround*/ false, /*sampleCountIni*/ 40.0, /*depthBufferValue*/ -1.0,
        /*bMieRayPhase*/ false, /*tMaxMax*/ kDefaultMaxT, /*bVariableSampleCount*/ false
    ).opticalDepth;

    vec3 transmittance = min(exp(-opticalDepth), vec3(kMaxHalfFloat));
    imageStore(imageTransmittanceLut, workPos, vec4(transmittance, 1.0f));
}
