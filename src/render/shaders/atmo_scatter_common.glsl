#ifndef ATMO_SCATTER_COMMON_GLSL
#define ATMO_SCATTER_COMMON_GLSL
#extension GL_EXT_samplerless_texture_functions : enable

// ==========================================================================
// atmo_scatter_common.glsl — 大气重构·壳内(atmo_inside.frag)/壳外(atmo_shell.frag)
// 共用 GetSkyRadianceToPoint 段散射（Inscatter LUT 双查表相减 + Transmittance LUT）。
//
// 壳外曾直接查 Sky-View LUT 导致参数越界（观察者高度须 < topRadius），
// 现改为段散射 LUT，对壳内/壳外任意相机位置成立。
// ==========================================================================

#include "common/shared_atmosphere.glsl"

// Set 1：渲染侧采样（VkAtmoLutPipelines::renderSetLayout，每个星球一份实例，
// 见 vk_atmosphere_lut.h::VkAtmosphereLut::renderSet / updateRenderBindings）。
// atmo_inside.frag / atmo_shell.frag 共用同一份声明，不用各自重复。
layout(set = 1, binding = 0) uniform texture2D inSceneDepth;
layout(set = 1, binding = 1) uniform texture2D inTransmittanceLut;
layout(set = 1, binding = 2) uniform texture2D inSkyViewLut; // 壳内天空像素采样；须与 skyview_lut 烘焙同基
layout(set = 1, binding = 3) uniform sampler   samp;
layout(set = 1, binding = 4) uniform texture2D inSceneColor; // 几何 pass HDR，与 atmo 同帧同图（GENERAL 布局读写）
layout(set = 1, binding = 5) uniform texture3D inInscatterLut; // Phase B：4D inscatter（单次散射，含相位）

// scattered：路径积分散射光 L_inscatter
// viewT：视线 RGB 透射
struct AtmoMarchResult { vec3 scattered; vec3 viewT; float opacity; };

float ozoneDensityFn(float h, float ozoneCenter, float ozoneWidth) {
    float d = (h - ozoneCenter) / ozoneWidth;
    return exp(-0.5 * d * d);
}

bool intersectSphereAtCenter(vec3 ro, vec3 rd, vec3 center, float radius, out float t0, out float t1) {
    vec3  L    = ro - center;
    float tca  = -dot(L, rd);
    vec3  perp = L + tca * rd;
    float d2   = dot(perp, perp);
    float r2   = radius * radius;
    if (d2 > r2) return false;
    float thc = sqrt(r2 - d2);
    t0 = tca - thc; t1 = tca + thc;
    return true;
}

float rayleighPhaseAtmo(float cosTheta) {
    return 3.0 / (16.0 * 3.14159265359) * (1.0 + cosTheta * cosTheta);
}

float miePhaseAtmo(float cosTheta, float gMie) {
    float g2    = gMie * gMie;
    float num   = 3.0 * (1.0 - g2) * (1.0 + cosTheta * cosTheta);
    float denom = 8.0 * 3.14159265359 * (2.0 + g2) * pow(1.0 + g2 - 2.0 * gMie * cosTheta, 1.5);
    return num / max(denom, 1e-6);
}

// 与 convertToAtmosphereUnit / getSkyLutObserverPosKm 一致：沿 camRel 径向抬高 0.5 km。
vec3 atmoObserverPosKm(vec3 camRelPlanetKm)
{
    return cameraPosToAtmosphereObserverKm(camRelPlanetKm);
}

// 与 getPosScatterLight() 烘焙完全同构的 {up, side, forward}（太阳水平前向基）。
void atmoSkyLutBasis(vec3 observerAtmoKm, vec3 sunDirToPlanet,
                      out vec3 up, out vec3 side, out vec3 forward)
{
    up = normalize(observerAtmoKm);
    vec3 sideCross = cross(up, sunDirToPlanet);
    float sideLen  = length(sideCross);
    if (sideLen > 1e-5)
        side = sideCross / sideLen;
    else
        side = normalize(cross(up, (abs(up.x) < 0.9) ? vec3(1.0, 0.0, 0.0) : vec3(0.0, 1.0, 0.0)));
    forward = normalize(cross(side, up));
}

// 视线 → Sky-View LUT 参数（须与烘焙侧 worldDir 分解同式）。
void atmoRayToSkyViewParams(vec3 rayDir, vec3 up, vec3 forward,
                             out float viewZenithCosAngle, out float lightViewCosAngle)
{
    viewZenithCosAngle = dot(rayDir, up);
    float sinV = sqrt(max(0.0, 1.0 - viewZenithCosAngle * viewZenithCosAngle));
    lightViewCosAngle = (sinV > 1e-5) ? clamp(dot(rayDir, forward) / sinV, -1.0, 1.0) : 1.0;
}

// 查 Sky-View LUT（壳内天空像素）。坐标系与 skyview_lut / getPosScatterLight 一致。
vec3 atmoSampleSkyViewLut(vec3 camRelPlanetKm, vec3 rayDir, vec3 sunDirToPlanet,
                           float bottomRadius, float topRadius)
{
    vec3 observer = atmoObserverPosKm(camRelPlanetKm);
    float viewHeight = length(observer);
    if (viewHeight >= topRadius)
        return vec3(0.0);

    AtmosphereParameters atmo;
    atmo.bottomRadius = bottomRadius;
    atmo.topRadius    = topRadius;

    vec3 up, side, fwd;
    atmoSkyLutBasis(observer, sunDirToPlanet, up, side, fwd);

    float viewZenithCosAngle, lightViewCosAngle;
    atmoRayToSkyViewParams(rayDir, up, fwd, viewZenithCosAngle, lightViewCosAngle);

    // LUT V 轴分两半：上半=地平以上，下半=地平以下/撞地。须与 skyview_lut 烘焙及
    // sky_irradiance_capture.glsl 一致，勿写死 false（贴地时地下楔形白块）。
    bool bIntersectGround = raySphereIntersectNearest(observer, rayDir, vec3(0.0), bottomRadius) >= 0.0;

    vec2 sampleUv;
    skyViewLutParamsToUv(atmo, bIntersectGround, viewZenithCosAngle, lightViewCosAngle,
                         viewHeight, vec2(textureSize(inSkyViewLut, 0)), sampleUv);
    return texture(sampler2D(inSkyViewLut, samp), sampleUv).rgb;
}

// 沿视线方向的 Transmittance LUT（mu = dot(view, up)）。
vec3 viewTransmittanceLUT(vec3 posRelAtmoKm, vec3 viewDir, float bottomRadius, float topRadius)
{
    AtmosphereParameters atmo;
    atmo.bottomRadius = bottomRadius;
    atmo.topRadius    = topRadius;

    float viewHeight = length(posRelAtmoKm);
    float mu = dot(viewDir, posRelAtmoKm / max(viewHeight, 1e-4));
    vec2 uv;
    lutTransmittanceParamsToUv(atmo, viewHeight, mu, uv);
    return texture(sampler2D(inTransmittanceLut, samp), uv).rgb;
}

// 段 [tNear,tFar] 视线透射（LUT 比值）。Bruneton 约定 T(r,μ) 为沿视线到大气边界的
// 透射率，段内透射 = T(p0)/T(p1)（p1 沿射线更远），勿写成 T1/T0。
vec3 atmoPointSegmentTransmittance(vec3 p0Rel, vec3 p1Rel, vec3 viewDir,
                                    float bottomRadius, float topRadius)
{
    vec3 T0 = viewTransmittanceLUT(p0Rel, viewDir, bottomRadius, topRadius);
    vec3 T1 = viewTransmittanceLUT(p1Rel, viewDir, bottomRadius, topRadius);
    return clamp(T0 / max(T1, vec3(1e-4)), vec3(0.0), vec3(1.0));
}

vec3 atmoSegmentViewTransmittance(vec3 camRelKm, vec3 rayDir, float tNear, float tFar,
                                  float bottomRadius, float topRadius)
{
    return atmoPointSegmentTransmittance(
        camRelKm + rayDir * tNear, camRelKm + rayDir * tFar, rayDir, bottomRadius, topRadius);
}

// 相对行星中心位置 → Inscatter 查表参数 (r,μ,μ_s,ν)。
void atmoInscatterParamsAtRelPos(vec3 posRel, vec3 viewDir, vec3 sunDir,
                                  float bottomRadius, float topRadius,
                                  out float r, out float mu, out float mu_s, out float nu,
                                  out bool hitGround)
{
    AtmosphereParameters atmo;
    atmo.bottomRadius = bottomRadius;
    atmo.topRadius    = topRadius;

    float len = length(posRel);
    r = clamp(len, bottomRadius, topRadius);
    vec3 up = posRel / max(len, 1e-4);
    mu   = clamp(dot(viewDir, up), -1.0, 1.0);
    mu_s = clamp(dot(sunDir, up), kInscatterMuSMin, 1.0);
    nu   = clamp(dot(viewDir, sunDir), -1.0, 1.0);
    hitGround = inscatterRayIntersectsGround(atmo, r, mu);
    float muBound = mu * mu_s + sqrt(max(0.0, (1.0 - mu * mu) * (1.0 - mu_s * mu_s)));
    nu = clamp(nu, -muBound, muBound);
}

// 从 posRel 沿 viewDir 积到大气边界/地面的预烘焙散射（Inscatter LUT 一次查表）。
vec3 atmoInscatterToBoundary(vec3 posRel, vec3 viewDir, vec3 sunDir,
                              float bottomRadius, float topRadius)
{
    AtmosphereParameters atmo;
    atmo.bottomRadius = bottomRadius;
    atmo.topRadius    = topRadius;

    float r, mu, mu_s, nu;
    bool hitGround;
    atmoInscatterParamsAtRelPos(posRel, viewDir, sunDir, bottomRadius, topRadius,
                                r, mu, mu_s, nu, hitGround);
    return sampleInscatterLut(inInscatterLut, samp, atmo, r, mu, mu_s, nu);
}

// Phase B：查 Inscatter LUT 的天空散射（单点→边界；太空相机先投影到大气顶）。
vec3 atmoSampleInscatterSky(vec3 camRelPlanetKm, vec3 rayDir, vec3 sunDirToPlanet,
                             float bottomRadius, float topRadius)
{
    vec3 observer = atmoObserverPosKm(camRelPlanetKm);
    if (length(observer) > topRadius) {
        if (!moveToTopAtmosphere(observer, rayDir, topRadius))
            return vec3(0.0);
    }
    return atmoInscatterToBoundary(observer, rayDir, sunDirToPlanet, bottomRadius, topRadius);
}

// Bruneton GetSkyRadianceToPoint（无 shadow_length）：
//   S(p0→p1) = S_lut(p0→边界) − T(p0,p1) · S_lut(p1→边界)
// p0 若在大气外，先沿 viewDir 投影到大气顶。
vec3 atmoGetSkyRadianceToPoint(vec3 p0Rel, vec3 p1Rel, vec3 viewDir, vec3 sunDir,
                                float bottomRadius, float topRadius)
{
    vec3 p0 = p0Rel;
    if (length(p0) > topRadius) {
        if (!moveToTopAtmosphere(p0, viewDir, topRadius))
            return vec3(0.0);
    }

    vec3 S0   = atmoInscatterToBoundary(p0, viewDir, sunDir, bottomRadius, topRadius);
    vec3 S1   = atmoInscatterToBoundary(p1Rel, viewDir, sunDir, bottomRadius, topRadius);
    vec3 Tseg = atmoPointSegmentTransmittance(p0, p1Rel, viewDir, bottomRadius, topRadius);
    return max(S0 - Tseg * S1, vec3(0.0));
}

// [tNear,tFar] 段散射 + 段透射（壳内/壳外共用；GetSkyRadianceToPoint + T_LUT）。
AtmoMarchResult atmoSegmentInscatterMarch(vec3 camRelKm, vec3 rayDir,
                                         float tNear, float tFar, vec3 sunDir,
                                         float bottomRadius, float topRadius)
{
    AtmoMarchResult result;
    result.scattered = vec3(0.0);
    result.viewT     = vec3(1.0);
    result.opacity   = 0.0;
    if (tFar <= tNear)
        return result;

    vec3 p0 = camRelKm + rayDir * tNear;
    vec3 p1 = camRelKm + rayDir * tFar;
    if (length(p0) > topRadius) {
        if (!moveToTopAtmosphere(p0, rayDir, topRadius))
            return result;
    }

    vec3 S0   = atmoInscatterToBoundary(p0, rayDir, sunDir, bottomRadius, topRadius);
    vec3 S1   = atmoInscatterToBoundary(p1, rayDir, sunDir, bottomRadius, topRadius);
    vec3 Tseg = atmoPointSegmentTransmittance(p0, p1, rayDir, bottomRadius, topRadius);
    result.scattered = max(S0 - Tseg * S1, vec3(0.0));
    result.viewT     = Tseg;
    result.opacity   = clamp(1.0 - dot(result.viewT, vec3(0.299, 0.587, 0.114)), 0.0, 1.0);
    return result;
}

// 用 Transmittance LUT 代替嵌套 raymarch 里的太阳方向内层 march：给定采样点
// （相对行星中心）和太阳方向，直接查表得到"这点到大气顶之间，太阳光被
// Rayleigh/Mie/臭氧吃掉多少"。AtmosphereParameters 这里只需要 bottomRadius/
// topRadius 两个字段（lutTransmittanceParamsToUv 内部只读这两个）。
vec3 sunTransmittanceLUT(vec3 posRelPlanet, vec3 sunDir, float bottomRadius, float topRadius) {
    AtmosphereParameters atmo;
    atmo.bottomRadius = bottomRadius;
    atmo.topRadius    = topRadius;

    float viewHeight = length(posRelPlanet);
    float sunZenithCosAngle = dot(sunDir, posRelPlanet / max(viewHeight, 1e-4));
    vec2 uv;
    lutTransmittanceParamsToUv(atmo, viewHeight, sunZenithCosAngle, uv);
    return texture(sampler2D(inTransmittanceLut, samp), uv).rgb;
}

// 射线段透射覆盖（raymarch 路径遗留；ToPoint 段散射已自带 viewT）。
void atmoOverrideViewTFromLut(inout AtmoMarchResult march, vec3 camRelKm, vec3 rayDir,
                              float tNear, float tFar, float bottomRadius, float topRadius)
{
    march.viewT   = atmoSegmentViewTransmittance(camRelKm, rayDir, tNear, tFar, bottomRadius, topRadius);
    march.opacity = clamp(1.0 - dot(march.viewT, vec3(0.299, 0.587, 0.114)), 0.0, 1.0);
}

// 壳内/壳外共用曝光：按相对大气厚度的高度连续插值，避免 camDist==outerRadius
// 时 innerExposureFar ↔ outerExposure 硬切闪一下。
float atmoUnifiedExposure(float camDist, float surfaceRadius, float outerRadius,
                          float innerNear, float innerFar, float outerExposure)
{
    float thickness = max(outerRadius - surfaceRadius, 1e-3);
    float altNorm   = max(camDist - surfaceRadius, 0.0) / thickness; // 壳外可 >1
    float insideExp = mix(innerNear, innerFar, smoothstep(0.0, 0.6, clamp(altNorm, 0.0, 1.0)));
    // 0.85→1.15 跨边界平滑接到 outerExposure
    return mix(insideExp, outerExposure, smoothstep(0.85, 1.15, altNorm));
}

// 仅边缘增益：rim=0 →×1；rim=1 →×limbAmount。
// limbAmount 由高度决定：大气底→顶用 smoothstep(0,1,altNorm) 从 bottom 收到 top。
float atmoLimbAmount(float altNorm, float limbBottom, float limbTop)
{
    // Hermite smoothstep：贴地=bottom，大气顶及以外=top，中间 S 形过渡
    float t = smoothstep(0.0, 1.0, clamp(altNorm, 0.0, 1.0));
    return mix(limbBottom, limbTop, t);
}

float atmoLimbMul(float limbAmount, float rim, float limbPower)
{
    float r = pow(clamp(rim, 0.0, 1.0), max(limbPower, 0.01));
    return mix(1.0, max(limbAmount, 0.01), r);
}

// 壳内 rim：视线相对当地天顶，地平 |mu|≈0 → rim≈1。
float atmoInsideRim(vec3 rayDir, vec3 camRelPlanet)
{
    float len = length(camRelPlanet);
    vec3  up  = camRelPlanet / max(len, 1e-4);
    return clamp(1.0 - abs(dot(rayDir, up)), 0.0, 1.0);
}

// scene 乘 T：mix(亮度, RGB, chromaStrength)，减轻地平线偏暖（1=全 RGB LUT T）。
vec3 atmoSceneTransmittance(vec3 viewT, float chromaStrength)
{
    vec3  T      = clamp(viewT, vec3(0.0), vec3(1.0));
    float T_luma = dot(T, vec3(0.299, 0.587, 0.114));
    float w      = clamp(chromaStrength, 0.0, 1.0);
    return mix(vec3(T_luma), T, w);
}

// 统一合成输出：L_out = ACES(scatter*E) + scene * T_scene
vec4 atmoCompositeOut(vec3 scattered, vec3 viewT, float exposure, vec3 sceneColor,
                      float chromaStrength)
{
    vec3  T_scene = atmoSceneTransmittance(viewT, chromaStrength);
    vec3  color  = max(scattered * exposure, vec3(0.0));
    vec3  x      = color;
    vec3  toned  = (x * (2.51 * x + 0.03)) / (x * (2.43 * x + 0.59) + 0.14);
    return vec4(toned + sceneColor * T_scene, 1.0);
}

// 单趟（非嵌套）raymarch：从 rayOrigin 沿 rayDir，在 [tNear,tFar] 区间内积分
// Rayleigh+Mie+臭氧散射，太阳方向遮蔽查 Transmittance LUT（不再是每外层步
// 再做一次 20 步内层 march）。物理上这段积分对"观察者在大气层内还是外"
// 完全不敏感——只关心这条路径本身穿过了多少大气——所以壳内 haze 和壳外 limb
// 可以共用同一个函数，不需要 Sky-View LUT 那种"观察者高度"参数化。
AtmoMarchResult raymarchAtmoSegment(
    vec3 rayOrigin, vec3 rayDir, vec3 planetCenter, float tNear, float tFar, int steps,
    vec3 rayleighCoeff, float mieCoeff, float hRayleigh, float hMie, float gMie,
    vec3 ozoneCoeff, float ozoneCenter, float ozoneWidth,
    float bottomRadius, float topRadius, vec3 sunDir, float cosTheta)
{
    AtmoMarchResult result;
    result.scattered = vec3(0.0);
    result.viewT     = vec3(1.0);
    result.opacity   = 0.0;
    if (tFar <= tNear || steps <= 0) return result;

    float STEP = (tFar - tNear) / float(steps);
    float tCur = tNear + 0.5 * STEP;

    vec3  sumRayleigh = vec3(0.0);
    vec3  sumMie      = vec3(0.0);
    float optDepthR = 0.0, optDepthM = 0.0, optDepthO = 0.0;

    for (int i = 0; i < steps; i++) {
        vec3  pos          = rayOrigin + rayDir * tCur;
        vec3  posRelPlanet  = pos - planetCenter;
        float h             = max(length(posRelPlanet) - bottomRadius, 0.0);

        float dR = exp(-h / hRayleigh) * STEP;
        float dM = exp(-h / hMie)      * STEP;
        float dO = ozoneDensityFn(h, ozoneCenter, ozoneWidth) * STEP;
        optDepthR += dR; optDepthM += dM; optDepthO += dO;

        vec3 sunT = sunTransmittanceLUT(posRelPlanet, sunDir, bottomRadius, topRadius);

        vec3 tau       = rayleighCoeff * optDepthR + mieCoeff * 1.1 * optDepthM + ozoneCoeff * optDepthO;
        vec3 selfAtten = exp(-tau);

        sumRayleigh += dR * selfAtten * sunT;
        sumMie      += dM * selfAtten * sunT;
        tCur        += STEP;

        if (max(selfAtten.x, max(selfAtten.y, selfAtten.z)) < 0.01) break;
    }

    float phaseR = rayleighPhaseAtmo(cosTheta);
    float phaseM = miePhaseAtmo(cosTheta, gMie);
    result.scattered = sumRayleigh * rayleighCoeff * phaseR + sumMie * mieCoeff * phaseM;

    vec3  tauView   = rayleighCoeff * optDepthR + mieCoeff * optDepthM + ozoneCoeff * optDepthO;
    result.viewT    = exp(-tauView);
    float transLuma = dot(result.viewT, vec3(0.299, 0.587, 0.114));
    result.opacity  = clamp(1.0 - transLuma, 0.0, 1.0);

    return result;
}

#endif
