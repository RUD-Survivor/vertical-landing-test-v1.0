#version 460
#extension GL_GOOGLE_include_directive : enable

// Sky-View LUT：以相机当前高度为准，把 (视天顶角, 太阳-视线方位角) 编码到一张
// 2D 贴图里，云管线的 lookupSkylight() 用它代替逐像素实时大气积分。
// 依赖 Transmittance LUT + Multi-Scatter LUT，须在两者之后烘焙；每帧重新烘焙
// （相机高度每帧都可能不同，尤其是起飞/入轨/星际航行阶段）。
#include "atmosphere_common.glsl"

layout (local_size_x = 8, local_size_y = 8) in;
void main()
{
    ivec2 lutSize = imageSize(imageSkyViewLut);
    ivec2 workPos = ivec2(gl_GlobalInvocationID.xy);
    if (workPos.x >= lutSize.x || workPos.y >= lutSize.y) return;

    const vec2 pixPos = vec2(workPos) + vec2(0.5f);
    const vec2 uv = pixPos / vec2(lutSize);

    AtmosphereParameters atmosphere = getAtmosphereParameters();

    // RocketSim3D: camWorldPos 已经是"相机相对当前行星中心"的偏移（C++ 侧 bake() 里减去了
    // planetCenter），不需要 flower 原版那个假设固定地面参考点的 +bottomRadius 沿 Y 偏移。
    vec3 worldPos = convertToAtmosphereUnit(frameData.camWorldPos.xyz);
    vec3 scatteredLight = getPosScatterLight(atmosphere, worldPos, uv, true, pixPos);
    imageStore(imageSkyViewLut, workPos, vec4(scatteredLight, 1.0f));
}
