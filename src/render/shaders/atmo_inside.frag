#version 450
#extension GL_GOOGLE_include_directive : enable
#extension GL_EXT_samplerless_texture_functions : enable

// ==========================================================================
// atmo_inside.frag — 大气渲染重构·壳内路径（相机在大气层以内）
//
// 替换旧版 atmo.frag 的全屏嵌套 raymarch（96 外层 × 20 内层 = ~1920 步/像素）：
//   · 天空背景像素（没有场景几何挡住）：直接查 Sky-View LUT，O(1)，不再 raymarch。
//   · 有场景深度的像素（地形/行星表面，见 vk_renderer3d.h 绘制顺序注释——大气
//     排在火箭/发射台之前，深度缓冲此时只有地形+行星表面+环，够用）：做一次
//     单趟（非嵌套）raymarch 算空气透视(haze)，太阳方向的光学深度不再现场 20 步
//     内层 march，改成查这个星球缓存好的 Transmittance LUT（vk_atmosphere_lut.h
//     的 VkAtmoLutCache，profile 没变就直接复用，不重烤）。
//
// 详见计划 fuzzy-toasting-flurry：大气渲染重构 / 方案 2、4。
// ==========================================================================

#define PI 3.14159265359
#define PRIMARY_STEPS 48

#include "common/shared_atmosphere.glsl"

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

// Set 1：渲染侧采样（VkAtmoLutPipelines::renderSetLayout，每个星球一份实例，
// 见 vk_atmosphere_lut.h::VkAtmosphereLut::renderSet / updateDepthBinding）。
layout(set = 1, binding = 0) uniform texture2D inSceneDepth;
layout(set = 1, binding = 1) uniform texture2D inTransmittanceLut;
layout(set = 1, binding = 2) uniform texture2D inSkyViewLut;
layout(set = 1, binding = 3) uniform sampler   samp;

layout(location = 0) in  vec2 vNDC;
layout(location = 0) out vec4 FragColor;

float ozoneDensity(float h) {
    float d = (h - pc.ozoneCenter) / pc.ozoneWidth;
    return exp(-0.5 * d * d);
}

bool intersectSphere(vec3 ro, vec3 rd, float radius, out float t0, out float t1) {
    vec3  L    = ro - pc.planetCenter.xyz;
    float tca  = -dot(L, rd);
    vec3  perp = L + tca * rd;
    float d2   = dot(perp, perp);
    float r2   = radius * radius;
    if (d2 > r2) return false;
    float thc = sqrt(r2 - d2);
    t0 = tca - thc; t1 = tca + thc;
    return true;
}

float rayleighPhaseFn(float cosTheta) {
    return 3.0 / (16.0 * PI) * (1.0 + cosTheta * cosTheta);
}

float miePhaseFn(float cosTheta) {
    float g2    = pc.gMie * pc.gMie;
    float num   = 3.0 * (1.0 - g2) * (1.0 + cosTheta * cosTheta);
    float denom = 8.0 * PI * (2.0 + g2) * pow(1.0 + g2 - 2.0 * pc.gMie * cosTheta, 1.5);
    return num / max(denom, 1e-6);
}

// 用 Transmittance LUT 代替嵌套 raymarch 里的太阳方向内层 march：给定采样点
// （相对行星中心）和太阳方向，直接查表得到"这点到大气顶之间，太阳光被
// Rayleigh/Mie/臭氧吃掉多少"。AtmosphereParameters 这里只需要 bottomRadius/
// topRadius 两个字段（lutTransmittanceParamsToUv 内部只读这两个），不需要
// 整套 Bruneton 参数化——避免把云管线那个大 UBO(GpuPerFrameData) 也搬进图形管线。
vec3 sunTransmittanceLUT(vec3 posRelPlanet, vec3 sunDir) {
    AtmosphereParameters atmo;
    atmo.bottomRadius = pc.surfaceRadius;
    atmo.topRadius    = pc.outerRadius;

    float viewHeight = length(posRelPlanet);
    float sunZenithCosAngle = dot(sunDir, posRelPlanet / max(viewHeight, 1e-4));
    vec2 uv;
    lutTransmittanceParamsToUv(atmo, viewHeight, sunZenithCosAngle, uv);
    return texture(sampler2D(inTransmittanceLut, samp), uv).rgb;
}

// 天空背景 O(1) 查表：不做任何 raymarch，直接从 Sky-View LUT 里按视线方向取色。
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
    if (!intersectSphere(camPos, rayDir, pc.outerRadius, tNear, tFar)) discard;
    tNear = max(tNear, 0.0);
    if (tFar <= tNear) discard;

    // Clip ray at planet surface（行星本体挡住的部分不用再往后传播）
    float tS0, tS1;
    if (intersectSphere(camPos, rayDir, pc.surfaceRadius, tS0, tS1) && tS0 > 0.0)
        tFar = min(tFar, tS0);

    // ── 深度缓冲空气透视（haze）：把 tFar 进一步 clip 到场景深度对应的距离 ──
    vec2  screenUv      = vNDC * 0.5 + 0.5;
    float sceneDepthNdc = texture(sampler2D(inSceneDepth, samp), screenUv).r;
    bool  bHasScene     = sceneDepthNdc < 0.9999;
    if (bHasScene) {
        // 标准深度（近0远1），从 NDC 反投影出世界坐标，取相机到该点的距离。
        vec4 clip  = vec4(vNDC, sceneDepthNdc, 1.0);
        vec4 viewP = inverse(frame.proj) * clip;
        viewP     /= viewP.w;
        vec4 worldP = inverse(frame.view) * viewP;
        float tScene = length(worldP.xyz - camPos);
        tFar = min(tFar, tScene);
    }

    if (tFar - tNear <= 0.0) discard;

    float STEP   = (tFar - tNear) / float(PRIMARY_STEPS);
    float ign    = fract(52.9829189 * fract(dot(gl_FragCoord.xy, vec2(0.06711056, 0.00583715))));
    float jitter = fract(ign + float(pc.frameIndex % 64) * 0.6180339887);
    float tCur   = tNear + jitter * STEP;

    vec3  sumRayleigh = vec3(0.0);
    vec3  sumMie      = vec3(0.0);
    float optDepthR = 0.0, optDepthM = 0.0, optDepthO = 0.0;
    float cosTheta  = dot(rayDir, sunDir);

    for (int i = 0; i < PRIMARY_STEPS; i++) {
        if (tCur >= tFar) break;
        float dt  = min(STEP, tFar - tCur);
        vec3  pos = camPos + rayDir * (tCur + dt * 0.5);
        vec3  posRelPlanet = pos - pc.planetCenter.xyz;
        float h   = max(length(posRelPlanet) - pc.surfaceRadius, 0.0);

        float dR = exp(-h / pc.hRayleigh) * dt;
        float dM = exp(-h / pc.hMie)      * dt;
        float dO = ozoneDensity(h)         * dt;
        optDepthR += dR; optDepthM += dM; optDepthO += dO;

        // 已修复：不再是嵌套 raymarch——太阳方向的光学深度不再在每个外层步里
        // 现算 20 步，而是查这个星球缓存好的 Transmittance LUT（profile 不变
        // 就不用重烤，逐帧成本只有这一次纹理采样，而不是 20 次密度求值）。
        vec3 sunT = sunTransmittanceLUT(posRelPlanet, sunDir);

        vec3 tau        = pc.rayleighCoeff * optDepthR + pc.mieCoeff * 1.1 * optDepthM + pc.ozoneCoeff * optDepthO;
        vec3 selfAtten  = exp(-tau);

        sumRayleigh += dR * selfAtten * sunT;
        sumMie      += dM * selfAtten * sunT;
        tCur        += STEP;

        if (max(selfAtten.x, max(selfAtten.y, selfAtten.z)) < 0.01) break;
    }

    float phaseR = rayleighPhaseFn(cosTheta);
    float phaseM = miePhaseFn(cosTheta);
    vec3  hazeColor = sumRayleigh * pc.rayleighCoeff * phaseR + sumMie * pc.mieCoeff * phaseM;

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
        // 天空背景：O(1) 查 Sky-View LUT。
        vec3 skyColor = skyViewLUT(rayDir, camRelPlanet, sunDir);
        finalColor = skyColor * exposure;
        alpha = spaceVisibility;
    } else {
        // haze：叠加在地表颜色上方，透明度只由这段路径本身的光学深度决定
        // （hazeOpacity），和"天该不该透星星"完全独立。
        finalColor = hazeColor * exposure;
        vec3  tauView   = pc.rayleighCoeff * optDepthR + pc.mieCoeff * optDepthM + pc.ozoneCoeff * optDepthO;
        vec3  viewT     = exp(-tauView);
        float transLuma = dot(viewT, vec3(0.299, 0.587, 0.114));
        float hazeOpacity = clamp(1.0 - transLuma, 0.0, 1.0);
        alpha = hazeOpacity;
    }

    // ACES filmic tone mapping（与旧版一致）
    vec3 x = max(finalColor, vec3(0.0));
    finalColor = (x * (2.51 * x + 0.03)) / (x * (2.43 * x + 0.59) + 0.14);

    FragColor = vec4(finalColor, alpha);
}
