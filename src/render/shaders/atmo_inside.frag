#version 450
#extension GL_GOOGLE_include_directive : enable
#extension GL_EXT_samplerless_texture_functions : enable

// ==========================================================================
// atmo_inside.frag — 大气渲染重构·壳内路径（相机在大气层以内）
//
// 替换旧版 atmo.frag 的全屏嵌套 raymarch（96 外层 × 20 内层 = ~1920 步/像素）：
//   · 天空背景像素（没有场景几何挡住）：直接查 Sky-View LUT，O(1)，不再 raymarch
//     （这里合法——观察者高度 < topRadius，Sky-View LUT 的有效范围，见
//     atmo_scatter_common.glsl 顶部注释）。
//   · 有场景深度的像素（地形/行星表面，见 vk_renderer3d.h 绘制顺序注释——大气
//     排在火箭/发射台之前，深度缓冲此时只有地形+行星表面+环，够用）：单趟
//     （非嵌套）raymarch 算空气透视(haze)，太阳方向的光学深度查 Transmittance
//     LUT（vk_atmosphere_lut.h 的 VkAtmoLutCache，profile 没变就直接复用，
//     不重烤），不再是每外层步一次 20 步内层 march。raymarch 本体和
//     atmo_shell.frag 共用 atmo_scatter_common.glsl::raymarchAtmoSegment()。
//
// 详见计划 fuzzy-toasting-flurry：大气渲染重构 / 方案 2、4。
// ==========================================================================

#define PI 3.14159265359
#define PRIMARY_STEPS 48

#include "atmo_scatter_common.glsl"

layout(set = 0, binding = 0) uniform FrameUBO {
    mat4  view;
    mat4  proj;
    vec3  lightDir;
    float _pad0;
    vec3  viewPos;
    float time;
} frame;

// AtmoPushConstants（vk_descriptors.h）：尾部 12 个 float（rayleighCoeff..ozoneWidth）
// 是本次重构后唯一的大气散射系数数据源，C++ 侧从 getPlanetScatteringCoeffs() 填，
// 不再像旧 atmo.frag::setupPlanetProfile() 那样在 shader 里手抄一份 7 组行星系数。
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
    float spaceVisStart;
    float spaceVisEnd;
    float limbBrightness; // 本文件不用（壳外 limb 专用），保留字段对齐 push constant 布局
    float _padTune;
} pc;

layout(location = 0) in  vec2 vNDC;
layout(location = 0) out vec4 FragColor;

// 天空背景 O(1) 查表：不做任何 raymarch，直接从 Sky-View LUT 里按视线方向取色。
// 合法条件：viewHeight（这里是相机高度）< topRadius——本文件只在"相机在壳内"
// 时才会被调度到（见 vk_renderer3d.h::renderAtmoAfterGeometry 的 cameraInside
// 分支），恒成立。
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
    vec3 camPos   = frame.viewPos;
    vec3 lightDir = normalize(frame.lightDir);
    // frame.lightDir 约定同旧 atmo.frag："指向太阳"的方向就是 -lightDir？
    // 保留旧版原始写法（旧代码直接用 lightDir 当 sunDir 用于相位/光学深度），
    // 这里同样不做额外取反，避免引入和旧版不一致的方向翻转 bug。
    vec3 sunDir = lightDir;

    // ── 重建世界空间视线方向（与旧 atmo.frag 完全相同的写法，未改动）──
    // vNDC 是 Vulkan NDC；frame.proj 是 OpenGL Y-up 习惯的投影矩阵，所以要negate Y。
    vec3 viewDir = vec3(
         vNDC.x / frame.proj[0][0],
        -vNDC.y / frame.proj[1][1],
        -1.0
    );
    vec3 rayDir = normalize(transpose(mat3(frame.view)) * viewDir);

    vec3  camRelPlanet  = camPos - pc.planetCenter.xyz;
    float camDist       = length(camRelPlanet);
    float camAlt        = max(camDist - pc.surfaceRadius, 0.0);
    float atmoThickness = pc.outerRadius - pc.surfaceRadius;
    float altNorm       = clamp(camAlt / max(atmoThickness, 1e-3), 0.0, 1.0);

    float tNear, tFar;
    if (!intersectSphereAtCenter(camPos, rayDir, pc.planetCenter.xyz, pc.outerRadius, tNear, tFar)) discard;
    tNear = max(tNear, 0.0);
    if (tFar <= tNear) discard;

    // Clip ray at planet surface（行星本体挡住的部分不用再往后传播）
    float tS0, tS1;
    if (intersectSphereAtCenter(camPos, rayDir, pc.planetCenter.xyz, pc.surfaceRadius, tS0, tS1) && tS0 > 0.0)
        tFar = min(tFar, tS0);

    // ── 深度缓冲空气透视（haze）：把 tFar 进一步 clip 到场景深度对应的距离 ──
    // vNDC 是 Vulkan 原生 NDC（顶部=-1），转 UV 不需要翻转 Y（和 Vulkan UV 方向
    // 一致），这里没问题。
    vec2  screenUv      = vNDC * 0.5 + 0.5;
    float sceneDepthNdc = texture(sampler2D(inSceneDepth, samp), screenUv).r;
    bool  bHasScene     = sceneDepthNdc < 0.9999;
    if (bHasScene) {
        // 已修复：frame.proj 是 OpenGL 习惯的矩阵（math3d.h::Mat4::perspective），
        // 它的逆矩阵要喂 OpenGL 约定的 NDC 才能正确反投影——① Y 轴要取反
        // （vNDC 是 Vulkan 约定顶部=-1，OpenGL 约定顶部=+1，和上面 154 行左右
        // 重建视线方向时的 -vNDC.y 是同一个道理，这里之前漏了）；② z 分量要从
        // Vulkan 深度缓冲的 [0,1] 换算回 OpenGL NDC 的 [-1,1]（*2-1），之前直接
        // 把 [0,1] 的深度值当 [-1,1] 用，反投影出来的世界坐标是错的，tScene
        // 距离算错，haze 裁剪的位置也就跟着错。
        vec4 clip  = vec4(vNDC.x, -vNDC.y, sceneDepthNdc * 2.0 - 1.0, 1.0);
        vec4 viewP = inverse(frame.proj) * clip;
        viewP     /= viewP.w;
        vec4 worldP = inverse(frame.view) * viewP;
        float tScene = length(worldP.xyz - camPos);
        tFar = min(tFar, tScene);
    }

    if (tFar - tNear <= 0.0) discard;

    float cosTheta = dot(rayDir, sunDir);
    AtmoMarchResult march = raymarchAtmoSegment(
        camPos, rayDir, pc.planetCenter.xyz, tNear, tFar, PRIMARY_STEPS,
        pc.rayleighCoeff, pc.mieCoeff, pc.hRayleigh, pc.hMie, pc.gMie,
        pc.ozoneCoeff, pc.ozoneCenter, pc.ozoneWidth,
        pc.surfaceRadius, pc.outerRadius, sunDir, cosTheta);

    float nightFactor = mix(0.01, 1.0, pc.sunVisibility);
    float exposure     = mix(10.0, 5.0, smoothstep(0.0, 0.6, altNorm)) * nightFactor;

    // ── spaceVisibility / hazeOpacity 拆分（不再用 1-transLuma 当唯一 alpha）──
    // spaceVisibility：纯粹由高度驱动，只用在"天空背景该不该透出星空"这件事上；
    // altNorm 越大（越接近/超出大气顶）越该让星空透出来，和这条视线本身的
    // 光学厚度无关——这正是旧版把两者耦合在一起时出问题的地方。
    // pc.spaceVisStart/spaceVisEnd 可在 imgui_atmo_tuner.h 里实时调（默认 0.55/1.0）。
    float spaceVisibility = (1.0 - smoothstep(pc.spaceVisStart, pc.spaceVisEnd, altNorm)) * nightFactor;

    vec3  finalColor;
    float alpha;
    if (!bHasScene) {
        // 天空背景：O(1) 查 Sky-View LUT（不用上面 raymarch 的结果）。
        vec3 skyColor = skyViewLUT(rayDir, camRelPlanet, sunDir);
        finalColor = skyColor * exposure;
        alpha = spaceVisibility;
    } else {
        // haze：叠加在地表颜色上方，透明度只由这段路径本身的光学深度决定
        // （march.opacity，即 hazeOpacity），和"天该不该透星星"完全独立。
        finalColor = march.scattered * exposure;
        alpha = march.opacity;
    }

    // ACES filmic tone mapping（与旧版一致）
    vec3 x = max(finalColor, vec3(0.0));
    finalColor = (x * (2.51 * x + 0.03)) / (x * (2.43 * x + 0.59) + 0.14);

    FragColor = vec4(finalColor, alpha);
}
