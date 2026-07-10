#version 450
#extension GL_GOOGLE_include_directive : enable
#extension GL_EXT_samplerless_texture_functions : enable

// ==========================================================================
// atmo_shell.frag — 大气渲染重构·壳外路径（相机在大气层以外看行星）
//
// 不再整屏画：这个 frag 只在 atmo_shell.vert 输出的壳网格实际光栅化覆盖的屏幕
// 像素上跑（一个行星在宽镜头里通常只占一小圈），深度测试开启（vk_pipeline.h::
// VkAtmoShellPipeline），不会画到更近的物体前面，天然比旧版整屏 raymarch 便宜。
// 采样这个星球缓存好的 Sky-View LUT（vk_atmosphere_lut.h::VkAtmoLutCache）取
// limb 光晕颜色，透明度按掠射角（法线 vs 视线夹角）做边缘增强——法线正对相机
// （从正上方往下看）时大气路径最短、幅亮度低；掠射（贴着边缘看）时大气路径最
// 长、最亮，这就是"行星边缘发光"的物理直觉来源。
//
// 详见计划 fuzzy-toasting-flurry：大气渲染重构 / 方案 3。
// ==========================================================================

#include "common/shared_atmosphere.glsl"

layout(set = 0, binding = 0) uniform FrameUBO {
    mat4  view;
    mat4  proj;
    vec3  lightDir;
    float _pad0;
    vec3  viewPos;
    float time;
} frame;

layout(push_constant) uniform PC {
    vec4  planetCenter;
    float innerRadius;
    float outerRadius;
    float surfaceRadius;
    int   planetIdx;
    float sunVisibility;
    float ringInner;
    float ringOuter;
    int   frameIndex;
    float tuneMinAlt;
    float tuneMaxAlt;
    float tuneExtinction;
    float showClouds;
    vec3  rayleighCoeff;
    float mieCoeff;
    float hRayleigh;
    float hMie;
    float gMie;
    vec3  ozoneCoeff;
    float ozoneCenter;
    float ozoneWidth;
    float spaceVisStart;   // 本文件不用（壳内专用），保留字段对齐 push constant 布局
    float spaceVisEnd;
    float limbBrightness;  // imgui_atmo_tuner.h 可调，默认 6.0
    float _padTune;
} pc;

// Set 1：与 atmo_inside.frag 完全同一套渲染侧采样布局（同一个 renderSetLayout，
// 同一个每星球 renderSet 实例）。壳外路径不需要场景深度（深度测试已经处理了
// "被更近物体挡住"这件事），inSceneDepth/inTransmittanceLut 这里不采样，
// 保留绑定只是为了复用同一个 Set 1 布局，不用为壳外单独建一份。
layout(set = 1, binding = 0) uniform texture2D inSceneDepth;
layout(set = 1, binding = 1) uniform texture2D inTransmittanceLut;
layout(set = 1, binding = 2) uniform texture2D inSkyViewLut;
layout(set = 1, binding = 3) uniform sampler   samp;

layout(location = 0) in  vec3 vWorldPos;
layout(location = 1) in  vec3 vLocalNormal;
layout(location = 0) out vec4 FragColor;

vec3 skyViewLUT(vec3 worldDirFromPlanet, vec3 camRelPlanet, vec3 sunDir) {
    AtmosphereParameters atmo;
    atmo.bottomRadius = pc.surfaceRadius;
    atmo.topRadius    = pc.outerRadius;

    float viewHeight = length(camRelPlanet);
    vec3  upVector   = camRelPlanet / max(viewHeight, 1e-4);
    float viewZenithCosAngle = dot(worldDirFromPlanet, upVector);

    vec3 sideVector    = normalize(cross(upVector, worldDirFromPlanet));
    vec3 forwardVector = normalize(cross(sideVector, upVector));
    vec2 lightOnPlane  = vec2(dot(sunDir, forwardVector), dot(sunDir, sideVector));
    lightOnPlane = normalize(lightOnPlane);
    float lightViewCosAngle = lightOnPlane.x;

    bool bIntersectGround = raySphereIntersectNearest(camRelPlanet, worldDirFromPlanet, vec3(0.0), atmo.bottomRadius) >= 0.0;

    vec2 lutSize = vec2(textureSize(inSkyViewLut, 0));
    vec2 uv;
    skyViewLutParamsToUv(atmo, bIntersectGround, viewZenithCosAngle, lightViewCosAngle, viewHeight, lutSize, uv);
    return texture(sampler2D(inSkyViewLut, samp), uv).rgb;
}

void main() {
    vec3 camPos = frame.viewPos;
    vec3 sunDir = normalize(frame.lightDir);

    vec3  toCam        = camPos - vWorldPos;
    float distToCam     = length(toCam);
    vec3  viewDirFromFrag = toCam / max(distToCam, 1e-4); // 片元指向相机
    vec3  rayDirFromCam   = -viewDirFromFrag;             // 相机看向片元（Sky-View LUT 期望的方向）

    vec3 camRelPlanet = camPos - pc.planetCenter.xyz;
    vec3 skyColor = skyViewLUT(rayDirFromCam, camRelPlanet, sunDir);

    // 掠射角增强：法线和"看向相机"方向越垂直（掠射/limb），大气路径越长、越亮；
    // 法线正对相机（俯视行星正中）时大气路径最短，limb 接近 0（该处主要看地表本身）。
    float grazing = 1.0 - abs(dot(vLocalNormal, viewDirFromFrag));
    float limb = pow(clamp(grazing, 0.0, 1.0), 2.0);

    float nightFactor = mix(0.05, 1.0, pc.sunVisibility);
    vec3 finalColor = skyColor * limb * nightFactor * pc.limbBrightness;

    // ACES filmic tone mapping（与壳内路径一致）
    vec3 x = max(finalColor, vec3(0.0));
    finalColor = (x * (2.51 * x + 0.03)) / (x * (2.43 * x + 0.59) + 0.14);

    FragColor = vec4(finalColor, limb);
}
