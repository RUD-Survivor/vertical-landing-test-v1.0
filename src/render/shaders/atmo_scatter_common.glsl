#ifndef ATMO_SCATTER_COMMON_GLSL
#define ATMO_SCATTER_COMMON_GLSL
#extension GL_EXT_samplerless_texture_functions : enable

// ==========================================================================
// atmo_scatter_common.glsl — 大气重构·壳内(atmo_inside.frag)/壳外(atmo_shell.frag)
// 共用 GetSkyRadianceToPoint 段散射（Bruneton reduced LUT：单次+多重已折入 RGB，
// 运行时乘相位）。
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
layout(set = 1, binding = 4) uniform texture2D inSceneColor; // Read-only HDR snapshot taken before this atmosphere draw
layout(set = 1, binding = 5) uniform texture3D inInscatterLut; // RGB=Rayleigh+Multi/phaseR，A=Mie.r
layout(set = 1, binding = 6) uniform texture3D inSingleMieLut; // 可选完整 RGB 单次 Mie

// scattered：路径积分散射光 L_inscatter
// viewT：视线 RGB 透射
struct AtmoMarchResult { vec3 scattered; vec3 viewT; float opacity; };

// Bruneton COMBINED_SCATTERING_TEXTURES：由 combined RGB 与 A=Mie.r 外推完整 Mie。
vec3 atmoExtrapolateSingleMie(vec3 scattering, float mieRed,
                              vec3 rayleighScattering, vec3 mieScattering)
{
    if (scattering.r <= 0.0 || mieScattering.r <= 0.0)
        return vec3(0.0);
    vec3 safeRayleigh = max(rayleighScattering, vec3(1e-8));
    return scattering * (mieRed / scattering.r)
        * (rayleighScattering.r / mieScattering.r)
        * (mieScattering / safeRayleigh);
}

// Transmittance LUT：沿 (r,μ) 射线到大气顶的透射率
vec3 atmoTransmittanceToTop(float r, float mu, float bottomRadius, float topRadius)
{
    AtmosphereParameters atmo;
    atmo.bottomRadius = bottomRadius;
    atmo.topRadius    = topRadius;
    vec2 uv;
    lutTransmittanceParamsToUv(atmo, r, mu, uv);
    return texture(sampler2D(inTransmittanceLut, samp), uv).rgb;
}

// Bruneton GetTransmittance(r,μ,d,hitGround)
vec3 atmoGetTransmittanceBruneton(float r, float mu, float d, bool hitGround,
                                   float bottomRadius, float topRadius)
{
    AtmosphereParameters atmo;
    atmo.bottomRadius = bottomRadius;
    atmo.topRadius    = topRadius;

    float r_d, mu_d, mu_s_d;
    inscatterParamsAfterDistance(atmo, r, mu, 0.0, 0.0, d, r_d, mu_d, mu_s_d);

    if (hitGround) {
        vec3 Td = atmoTransmittanceToTop(r_d, -mu_d, bottomRadius, topRadius);
        vec3 T0 = atmoTransmittanceToTop(r, -mu, bottomRadius, topRadius);
        return clamp(Td / max(T0, vec3(1e-4)), vec3(0.0), vec3(1.0));
    }
    vec3 T0 = atmoTransmittanceToTop(r, mu, bottomRadius, topRadius);
    vec3 Td = atmoTransmittanceToTop(r_d, mu_d, bottomRadius, topRadius);
    return clamp(T0 / max(Td, vec3(1e-4)), vec3(0.0), vec3(1.0));
}

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

// Perspective depth reconstruction without inverse-matrix/world-position cancellation.
float atmoDepthRayDistance(float depth, mat4 proj, float viewCos)
{
    float denominator = 2.0 * (1.0 - depth) - (1.0 + proj[2][2]);
    return -proj[3][2] / max(denominator * viewCos, 1e-30);
}

void atmoSceneRayDepth(float depth, mat4 proj, mat4 view, vec3 rayDir,
                      out float distance, out float uncertainty)
{
    float viewCos = max(-(mat3(view) * rayDir).z, 1e-6);
    uint bits = floatBitsToUint(clamp(depth, 0.0, 1.0));
    // Two D32 ULPs also cover rounding in the vertex projection and depth mapping.
    float lower = uintBitsToFloat(bits > 2u ? bits - 2u : 0u);
    float upper = uintBitsToFloat(min(bits + 2u, floatBitsToUint(1.0)));
    distance = atmoDepthRayDistance(depth, proj, viewCos);
    float lo = atmoDepthRayDistance(lower, proj, viewCos);
    float hi = atmoDepthRayDistance(upper, proj, viewCos);
    uncertainty = max(max(distance - lo, hi - distance), 1e-4);
}

// Reject only a foreground depth interval entirely before the atmosphere.
// Unresolved planetary depths fall back to the analytic surface; a resolved foreground
// inside the atmosphere still clips the segment even when distant depth blending is off.
bool atmoApplyDepthFarMix(
    inout float tFar, float tNear, float tScene, float depthUncertainty,
    float camDist, float outerRadius, int shellDiskMode,
    float blendStartKm, float blendEndKm)
{
    if (tScene + depthUncertainty < tNear)
        return false;
    if (tScene < tNear)
        return true; // Entry lies in the depth uncertainty interval, not a proven occluder.

    float tAnalytic = tFar;
    float tDepth = min(tAnalytic, tScene);
    float wDepth = 1.0;
    bool resolvedForeground = tScene + depthUncertainty + 1.0 < tAnalytic;
    if (shellDiskMode == 2 && !resolvedForeground) {
        float r0 = outerRadius + max(blendStartKm, 0.0);
        float r1 = outerRadius + max(blendEndKm, blendStartKm + 1.0);
        wDepth = 1.0 - smoothstep(r0, r1, camDist);
    }
    if (!resolvedForeground) {
        float segmentLength = max(tAnalytic - tNear, 1e-3);
        wDepth *= 1.0 - smoothstep(0.1 * segmentLength, 0.5 * segmentLength, depthUncertainty);
    }
    tFar = mix(tAnalytic, tDepth, wDepth);
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

// 段 [tNear,tFar] 视线透射（Bruneton GetTransmittance）。
vec3 atmoPointSegmentTransmittance(vec3 p0Rel, vec3 p1Rel, vec3 viewDir,
                                    float bottomRadius, float topRadius, bool hitGround)
{
    float r = length(p0Rel);
    float mu = dot(viewDir, p0Rel / max(r, 1e-4));
    float d = max(dot(p1Rel - p0Rel, viewDir), 0.0);
    return atmoGetTransmittanceBruneton(r, mu, d, hitGround, bottomRadius, topRadius);
}

vec3 atmoSegmentViewTransmittance(vec3 camRelKm, vec3 rayDir, float tNear, float tFar,
                                  float bottomRadius, float topRadius)
{
    vec3 p0 = camRelKm + rayDir * tNear;
    AtmosphereParameters atmo;
    atmo.bottomRadius = bottomRadius;
    atmo.topRadius    = topRadius;
    float r = length(p0);
    float mu = dot(rayDir, p0 / max(r, 1e-4));
    bool hitGround = inscatterRayIntersectsGround(atmo, r, mu);
    return atmoPointSegmentTransmittance(
        p0, camRelKm + rayDir * tFar, rayDir, bottomRadius, topRadius, hitGround);
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
    nu = inscatterClampNu(nu, mu, mu_s);
}

struct AtmoReducedScattering {
    vec3 scattering;
    vec3 singleMie;
    float singleMieRed;
};

// mieStorageMode：0=Bruneton combined 外推；1=独立完整 RGB Mie LUT。
AtmoReducedScattering atmoGetCombinedReducedScattering(
    float r, float mu, float mu_s, float nu, bool hitGround,
    float bottomRadius, float topRadius,
    vec3 rayleighScattering, vec3 mieScattering, int mieStorageMode)
{
    AtmosphereParameters atmo;
    atmo.bottomRadius = bottomRadius;
    atmo.topRadius    = topRadius;

    BrunetonReducedScattering s = sampleBrunetonReducedInscatterLut(
        inInscatterLut, samp, atmo, r, mu, mu_s, nu, hitGround);
    AtmoReducedScattering result;
    result.scattering = s.scattering;
    result.singleMieRed = s.mieRed;
    result.singleMie = (mieStorageMode == 1)
        ? sampleInscatterLut(inSingleMieLut, samp, atmo, r, mu, mu_s, nu, hitGround)
        : atmoExtrapolateSingleMie(s.scattering, s.mieRed,
                                  rayleighScattering, mieScattering);
    return result;
}

vec3 atmoApplyBrunetonPhases(vec3 scattering, vec3 singleMie, float nu, float mu_s, float gMie)
{
    singleMie *= smoothstep(0.0, 0.01, mu_s);
    return scattering * rayleighPhaseAtmo(nu) + singleMie * miePhaseAtmo(nu, gMie);
}

// 从 posRel 沿 viewDir 积到大气边界的预烘焙散射（单次查表 + 运行时相位）
vec3 atmoInscatterToBoundary(vec3 posRel, vec3 viewDir, vec3 sunDir,
                              float bottomRadius, float topRadius,
                              bool hitGround, vec3 rayleighScattering,
                              vec3 mieScattering, float gMie, int mieStorageMode)
{
    float r = length(posRel);
    vec3 up = posRel / max(r, 1e-4);
    float mu   = dot(viewDir, up);
    float mu_s = clamp(dot(sunDir, up), kInscatterMuSMin, 1.0);
    float nu   = dot(viewDir, sunDir);
    nu = inscatterClampNu(nu, mu, mu_s);

    AtmoReducedScattering s = atmoGetCombinedReducedScattering(
        r, mu, mu_s, nu, hitGround, bottomRadius, topRadius,
        rayleighScattering, mieScattering, mieStorageMode);
    return atmoApplyBrunetonPhases(s.scattering, s.singleMie, nu, mu_s, gMie);
}

// GetSkyRadiance：太空相机先投影到大气顶，再单次查表 + 相位
vec3 atmoSampleInscatterSky(vec3 camRelPlanetKm, vec3 rayDir, vec3 sunDirToPlanet,
                             float bottomRadius, float topRadius,
                             vec3 rayleighScattering, vec3 mieScattering,
                             float gMie, int mieStorageMode)
{
    vec3 observer = atmoObserverPosKm(camRelPlanetKm);
    if (length(observer) > topRadius) {
        if (!moveToTopAtmosphere(observer, rayDir, topRadius))
            return vec3(0.0);
    }

    AtmosphereParameters atmo;
    atmo.bottomRadius = bottomRadius;
    atmo.topRadius    = topRadius;

    float r = length(observer);
    float mu = dot(rayDir, observer / max(r, 1e-4));
    bool hitGround = inscatterRayIntersectsGround(atmo, r, mu);
    return atmoInscatterToBoundary(observer, rayDir, sunDirToPlanet,
        bottomRadius, topRadius, hitGround, rayleighScattering,
        mieScattering, gMie, mieStorageMode);
}

// ToPoint 中间量（F4 debug：区分 S0≈0 vs 相减归零）
struct AtmoToPointDebug {
    vec3  S0;          // LUT Rayleigh at p0（无相位）
    vec3  S1;          // LUT Rayleigh at p1'
    vec3  mie0;
    vec3  mie1;
    vec3  T;           // 段透射
    vec3  T_S1;        // T * S1
    vec3  diffR;       // max(S0 - T*S1, 0) 相位前
    vec3  diffMie;     // max(mie0 - T*mie1, 0)
    vec3  finalScattered;
    float d;           // 段长 km
    float hitGround;   // 0/1
    float ok;          // 0=射线未碰大气
};

AtmoToPointDebug atmoEvalSkyRadianceToPointDebug(vec3 p0Rel, vec3 p1Rel, vec3 viewDir, vec3 sunDir,
                                                  float bottomRadius, float topRadius,
                                                  vec3 rayleighScattering, vec3 mieScattering,
                                                  float gMie, int mieStorageMode)
{
    AtmoToPointDebug dbg;
    dbg.S0 = dbg.S1 = dbg.mie0 = dbg.mie1 = dbg.T = dbg.T_S1 =
        dbg.diffR = dbg.diffMie = dbg.finalScattered = vec3(0.0);
    dbg.d = 0.0;
    dbg.hitGround = 0.0;
    dbg.ok = 0.0;

    vec3 p0 = p0Rel;
    if (length(p0) > topRadius) {
        if (!moveToTopAtmosphere(p0, viewDir, topRadius))
            return dbg;
    }
    dbg.ok = 1.0;

    AtmosphereParameters atmo;
    atmo.bottomRadius = bottomRadius;
    atmo.topRadius    = topRadius;

    float r = length(p0);
    vec3 up = p0 / max(r, 1e-4);
    float mu   = dot(viewDir, up);
    float mu_s = clamp(dot(sunDir, up), kInscatterMuSMin, 1.0);
    float nu   = dot(viewDir, sunDir);
    nu = inscatterClampNu(nu, mu, mu_s);

    bool hitGround = inscatterRayIntersectsGround(atmo, r, mu);
    dbg.hitGround = hitGround ? 1.0 : 0.0;
    dbg.d = max(dot(p1Rel - p0, viewDir), 0.0);

    dbg.T = atmoGetTransmittanceBruneton(r, mu, dbg.d, hitGround, bottomRadius, topRadius);

    AtmoReducedScattering s0 = atmoGetCombinedReducedScattering(
        r, mu, mu_s, nu, hitGround, bottomRadius, topRadius,
        rayleighScattering, mieScattering, mieStorageMode);
    dbg.S0 = s0.scattering;
    dbg.mie0 = s0.singleMie;

    float r_p, mu_p, mu_s_p;
    inscatterParamsAfterDistance(atmo, r, mu, mu_s, nu, dbg.d, r_p, mu_p, mu_s_p);
    AtmoReducedScattering s1 = atmoGetCombinedReducedScattering(
        r_p, mu_p, mu_s_p, nu, hitGround, bottomRadius, topRadius,
        rayleighScattering, mieScattering, mieStorageMode);
    dbg.S1 = s1.scattering;
    dbg.mie1 = s1.singleMie;

    dbg.T_S1  = dbg.T * dbg.S1;
    dbg.diffR = max(dbg.S0 - dbg.T_S1, vec3(0.0));
    if (mieStorageMode == 1) {
        dbg.diffMie = max(dbg.mie0 - dbg.T * dbg.mie1, vec3(0.0));
    } else {
        // 官方 combined 路径：先对可靠的 Mie.r 做段差，再用段 scattering RGB 外推。
        float diffMieRed = max(s0.singleMieRed - dbg.T.r * s1.singleMieRed, 0.0);
        dbg.diffMie = atmoExtrapolateSingleMie(
            dbg.diffR, diffMieRed, rayleighScattering, mieScattering);
    }
    dbg.finalScattered = atmoApplyBrunetonPhases(dbg.diffR, dbg.diffMie, nu, mu_s, gMie);
    return dbg;
}

// 0=关 1=S0 2=S1 3=T 4=T*S1 5=diffR 6=final 7=d 8=hitGround
vec3 atmoToPointDebugColor(int mode, AtmoToPointDebug dbg)
{
    if (dbg.ok < 0.5)
        return vec3(0.2, 0.0, 0.2); // 紫：未进大气
    if (mode == 1) return dbg.S0;
    if (mode == 2) return dbg.S1;
    if (mode == 3) return dbg.T;
    if (mode == 4) return dbg.T_S1;
    if (mode == 5) return dbg.diffR;
    if (mode == 6) return dbg.finalScattered;
    if (mode == 7) {
        // 段长：绿=短(<50km) 红=很长(>=2000km)
        float t = clamp(dbg.d / 2000.0, 0.0, 1.0);
        return vec3(t, 1.0 - t, 0.05);
    }
    if (mode == 8)
        return dbg.hitGround > 0.5 ? vec3(1.0, 0.2, 0.1) : vec3(0.1, 0.8, 1.0);
    return dbg.finalScattered;
}

// Bruneton GetSkyRadianceToPoint：hitGround 仅在 p0 算一次；p1 用 (r_p,μ_p,μ_s_p) 同 ν
vec3 atmoGetSkyRadianceToPoint(vec3 p0Rel, vec3 p1Rel, vec3 viewDir, vec3 sunDir,
                                float bottomRadius, float topRadius,
                                vec3 rayleighScattering, vec3 mieScattering,
                                float gMie, int mieStorageMode)
{
    return atmoEvalSkyRadianceToPointDebug(p0Rel, p1Rel, viewDir, sunDir,
        bottomRadius, topRadius, rayleighScattering, mieScattering,
        gMie, mieStorageMode).finalScattered;
}

// [tNear,tFar] 段散射 + 段透射（Bruneton ToPoint）
// debugMode>0 时 scattered 为中间量（由 frag 单独曝光可视化，不走 scene 合成）。
AtmoMarchResult atmoSegmentInscatterMarch(vec3 camRelKm, vec3 rayDir,
                                         float tNear, float tFar, vec3 sunDir,
                                         float bottomRadius, float topRadius,
                                         vec3 rayleighScattering, vec3 mieScattering,
                                         float gMie, int mieStorageMode, int debugMode)
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

    AtmoToPointDebug dbg = atmoEvalSkyRadianceToPointDebug(
        p0, p1, rayDir, sunDir, bottomRadius, topRadius,
        rayleighScattering, mieScattering, gMie, mieStorageMode);

    if (debugMode > 0)
        result.scattered = atmoToPointDebugColor(debugMode, dbg);
    else
        result.scattered = dbg.finalScattered;

    AtmosphereParameters atmo;
    atmo.bottomRadius = bottomRadius;
    atmo.topRadius    = topRadius;
    float r = length(p0);
    float mu = dot(rayDir, p0 / max(r, 1e-4));
    bool hitGround = inscatterRayIntersectsGround(atmo, r, mu);
    result.viewT = atmoPointSegmentTransmittance(p0, p1, rayDir, bottomRadius, topRadius, hitGround);
    result.opacity = clamp(1.0 - dot(result.viewT, vec3(0.299, 0.587, 0.114)), 0.0, 1.0);
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
    vec3 p0 = camRelKm + rayDir * tNear;
    AtmosphereParameters atmo;
    atmo.bottomRadius = bottomRadius;
    atmo.topRadius    = topRadius;
    float r = length(p0);
    float mu = dot(rayDir, p0 / max(r, 1e-4));
    bool hitGround = inscatterRayIntersectsGround(atmo, r, mu);
    march.viewT = atmoPointSegmentTransmittance(
        p0, camRelKm + rayDir * tFar, rayDir, bottomRadius, topRadius, hitGround);
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

// Linear HDR composition. Exposure and tone mapping occur in final presentation.
vec4 atmoCompositeOut(vec3 scattered, vec3 viewT, float solarIrradiance, vec3 sceneColor,
                      float chromaStrength)
{
    vec3  T_scene = atmoSceneTransmittance(viewT, chromaStrength);
    vec3  color  = max(scattered * solarIrradiance, vec3(0.0));
    return vec4(color + sceneColor * T_scene, 1.0);
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
        float sunGroundNear, sunGroundFar;
        if (intersectSphereAtCenter(posRelPlanet, sunDir, vec3(0.0), bottomRadius,
                sunGroundNear, sunGroundFar) && sunGroundNear > 0.0)
            sunT = vec3(0.0);

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
