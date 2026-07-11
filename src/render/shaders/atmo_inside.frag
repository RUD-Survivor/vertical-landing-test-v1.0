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

// AtmoPushConstants（vk_descriptors.h）：尾部字段（rayleighCoeff..sunDir）是本次
// 重构后唯一的大气散射系数数据源，C++ 侧从 getPlanetScatteringCoeffs() 填，
// 不再像旧 atmo.frag::setupPlanetProfile() 那样在 shader 里手抄一份 7 组行星系数。
//
// 已修复：这里全部用标量 float（不用 vec3）——GLSL push_constant 块默认走
// std430 布局，vec3 成员对齐要求是 16 字节，一旦某个 vec3 字段没有落在 16 的
// 倍数偏移上，编译器会静默插入 padding，而 C++ 那边手算的紧凑偏移完全不知道
// 这回事，会导致后面所有字段错位读串（这个 bug 真实发生过一次，ozoneCoeff
// 之后的字段全部读到了相邻字段的值）。标量 float 没有这个陷阱，永远和 C++
// 结构体的紧凑偏移一一对应。
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
    float rayleighCoeffX, rayleighCoeffY, rayleighCoeffZ;
    float mieCoeff;
    float hRayleigh;
    float hMie;
    float gMie;
    float ozoneCoeffX, ozoneCoeffY, ozoneCoeffZ;
    float ozoneCenter;
    float ozoneWidth;
    float spaceVisStart;
    float spaceVisEnd;
    float limbBrightness; // 本文件不用（壳外 limb 专用），保留字段对齐 push constant 布局
    float outerExposure;  // 本文件不用（壳外专用）
    float sunDirX, sunDirY, sunDirZ; // "这颗星球→太阳"方向，已归一化
    float innerExposureNear; // imgui_atmo_tuner.h 可调，贴地(altNorm=0)曝光倍率，默认 10
    float innerExposureFar;  // imgui_atmo_tuner.h 可调，近大气顶(altNorm≥0.6)曝光倍率，默认 5
    float _pad3;
} pc;

// 拼回 vec3，减少下面调用点的改动量。
#define PC_RAYLEIGH vec3(pc.rayleighCoeffX, pc.rayleighCoeffY, pc.rayleighCoeffZ)
#define PC_OZONE    vec3(pc.ozoneCoeffX, pc.ozoneCoeffY, pc.ozoneCoeffZ)
#define PC_SUNDIR   vec3(pc.sunDirX, pc.sunDirY, pc.sunDirZ)

layout(location = 0) in  vec2 vNDC;
layout(location = 0) out vec4 FragColor;

// 构造一个和 v 不平行的辅助轴，用来在 v≈0 的退化情形下兜底（见 skyViewLUT()
// 里两处 normalize(cross(...))）。选 |v.x| 更小的那个分量对应的世界轴，
// 保证辅助轴和 v 之间的夹角不会太小。
vec3 safeOrthoAxis(vec3 v) {
    return (abs(v.x) < 0.9) ? vec3(1.0, 0.0, 0.0) : vec3(0.0, 1.0, 0.0);
}

// 天空背景 O(1) 查表：不做任何 raymarch，直接从 Sky-View LUT 里按视线方向取色。
// 合法条件：viewHeight（这里是相机高度）< topRadius——本文件只在"相机在壳内"
// 时才会被调度到（见 vk_renderer3d.h::renderAtmoAfterGeometry 的 cameraInside
// 分支），恒成立。
//
// 注：大气壳内出现的那个顶到天顶的巨大白色三角形，真正原因是烘焙侧
// （atmosphere_common.glsl::getPosScatterLight）用合成的"规范坐标系"和这里
// 采样用的真实局部系不一致，已经在烘焙侧改成真实局部系修好（该函数注释里
// 有完整说明）。下面的天顶奇点兜底是另一个独立、范围小得多的问题：正对当地
// 天顶/天底看时 cross(upVector, worldDirFromPlanet) 趋近零向量，normalize(0)
// = NaN，是这套参数化本身的数学奇点（那个方向上"朝向太阳的方位角"本来就没
// 有定义），和上面那个大范围的坐标系错位是两回事，仍然需要保留。
vec3 skyViewLUT(vec3 worldDirFromPlanet, vec3 camRelPlanet, vec3 sunDir) {
    AtmosphereParameters atmo;
    atmo.bottomRadius = pc.surfaceRadius;
    atmo.topRadius    = pc.outerRadius;

    float viewHeight = length(camRelPlanet);
    vec3  upVector   = camRelPlanet / max(viewHeight, 1e-4);
    float viewZenithCosAngle = dot(worldDirFromPlanet, upVector);

    vec3  sideCross = cross(upVector, worldDirFromPlanet);
    float sideLen   = length(sideCross);
    vec3  sideVector = (sideLen > 1e-5)
        ? sideCross / sideLen
        : normalize(cross(upVector, safeOrthoAxis(upVector)));
    vec3 forwardVector = normalize(cross(sideVector, upVector));

    vec2  lightOnPlane = vec2(dot(sunDir, forwardVector), dot(sunDir, sideVector));
    float lightOnPlaneLen = length(lightOnPlane);
    lightOnPlane = (lightOnPlaneLen > 1e-5) ? (lightOnPlane / lightOnPlaneLen) : vec2(1.0, 0.0);
    float lightViewCosAngle = lightOnPlane.x;

    bool bIntersectGround = raySphereIntersectNearest(camRelPlanet, worldDirFromPlanet, vec3(0.0), atmo.bottomRadius) >= 0.0;

    vec2 lutSize = vec2(textureSize(inSkyViewLut, 0));
    vec2 uv;
    skyViewLutParamsToUv(atmo, bIntersectGround, viewZenithCosAngle, lightViewCosAngle, viewHeight, lutSize, uv);
    return texture(sampler2D(inSkyViewLut, samp), uv).rgb;
}

void main() {
    vec3 camPos = frame.viewPos;
    // 已修复：不再用 frame.lightDir（几何 pass 那个全局 FrameUBO，循环画完所有
    // 星球后被重置成"火箭→太阳"方向）。改用 C++ 侧按这颗星球单独算好、随
    // push constant 传进来的方向（vk_renderer3d.h::renderAtmoAfterGeometry），
    // 全景模式相机离火箭很远时也不会算错。
    vec3 sunDir = PC_SUNDIR;

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
    float tS0 = -1.0, tS1;
    bool rayHitsSurface = intersectSphereAtCenter(camPos, rayDir, pc.planetCenter.xyz, pc.surfaceRadius, tS0, tS1) && tS0 > 0.0;
    if (rayHitsSurface)
        tFar = min(tFar, tS0);

    // ── 深度缓冲空气透视（haze）：把 tFar 进一步 clip 到场景深度对应的距离 ──
    // vNDC 是 Vulkan 原生 NDC（顶部=-1），转 UV 不需要翻转 Y（和 Vulkan UV 方向
    // 一致），这里没问题。
    vec2  screenUv      = vNDC * 0.5 + 0.5;
    float sceneDepthNdc = texture(sampler2D(inSceneDepth, samp), screenUv).r;

    // 射线是否会在深度缓冲未覆盖处（地形 LOD 空洞）碰到行星球面。
    // 此时 depth 仍为清除值 1.0，不能按天空 LUT 路径处理，应走 haze（tFar 已在
    // 上面 clip 到 tS0）。
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
        PC_RAYLEIGH, pc.mieCoeff, pc.hRayleigh, pc.hMie, pc.gMie,
        PC_OZONE, pc.ozoneCenter, pc.ozoneWidth,
        pc.surfaceRadius, pc.outerRadius, sunDir, cosTheta);

    float nightFactor = mix(0.01, 1.0, pc.sunVisibility);
    // pc.innerExposureNear/Far 可在 imgui_atmo_tuner.h 里实时调（默认 10/5，
    // 和原来硬编码的 mix(10.0,5.0,...) 数值一致，只是现在能调了）。
    float exposure     = mix(pc.innerExposureNear, pc.innerExposureFar, smoothstep(0.0, 0.6, altNorm)) * nightFactor;

    // ── spaceVisibility / hazeOpacity 拆分（不再用 1-transLuma 当唯一 alpha）──
    // spaceVisibility：纯粹由高度驱动，只用在"天空背景该不该透出星空"这件事上；
    // altNorm 越大（越接近/超出大气顶）越该让星空透出来，和这条视线本身的
    // 光学厚度无关——这正是旧版把两者耦合在一起时出问题的地方。
    // pc.spaceVisStart/spaceVisEnd 可在 imgui_atmo_tuner.h 里实时调（默认 0.55/1.0）。
    float spaceVisibility = (1.0 - smoothstep(pc.spaceVisStart, pc.spaceVisEnd, altNorm)) * nightFactor;

    // ── P0：预乘 alpha + 天空/场景路径选择（颜色不混 LUT+haze，避免洗白）──
    // 真天空：depth≥0.9999 且射线未碰到球面（朝上的天空）。
    // LOD 空洞：depth 仍为 1.0 但射线会打到球面（地形没画到）→ 走 haze 而非白 LUT。
    // 其余（depth<0.9999）：场景 haze。不在颜色上做 smoothstep——把过曝 LUT
    // 和米色 haze 线性混在一起会把整条地平线洗成一片白。
    bool isSkyPixel = (sceneDepthNdc >= 0.9999) && !rayHitsSurface;

    vec3 hazeRadiance = march.scattered * exposure;
    vec3 skyRadiance  = skyViewLUT(rayDir, camRelPlanet, sunDir) * exposure;
    vec3 inscatter    = isSkyPixel ? skyRadiance : hazeRadiance;
    float alpha       = isSkyPixel ? spaceVisibility : march.opacity;

    // ACES filmic tone mapping（与旧版一致）
    vec3 x = max(inscatter, vec3(0.0));
    vec3 finalColor = (x * (2.51 * x + 0.03)) / (x * (2.43 * x + 0.59) + 0.14);

    // 预乘 alpha：混合模式 ONE / ONE_MINUS_SRC_ALPHA（与 cloud.frag 一致）。
    // 正确合成：L_atmo * alpha + L_scene * (1 - alpha)，不再把未衰减的 haze
    // 全亮度叠在地表/火箭上。
    FragColor = vec4(finalColor * alpha, alpha);
}
