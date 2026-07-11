#ifndef ATMO_SCATTER_COMMON_GLSL
#define ATMO_SCATTER_COMMON_GLSL
#extension GL_EXT_samplerless_texture_functions : enable

// ==========================================================================
// atmo_scatter_common.glsl — 大气重构·壳内(atmo_inside.frag)/壳外(atmo_shell.frag)
// 共用的单趟（非嵌套）raymarch + Transmittance LUT 查表。
//
// 抽出来一是避免两份重复代码，二是修了壳外路径最初的一个真实 bug：壳外最早
// 直接照抄 Sky-View LUT 查表（和壳内一样），但 Sky-View LUT 的参数化只在
// "观察者高度 < 大气顶(topRadius)" 时有效——这是 Bruneton/UE Sky Atmosphere
// 的标准限制，本工程自己的 sky_irradiance_capture.glsl 也有同样的
// `bCanUseSkyViewLut = viewHeight < atmosphere.topRadius` 判断，超出这个范围
// 时它自己都会退回真正的散射积分，不再查表。壳外路径的相机按定义就在
// topRadius 以外，所以直接查 Sky-View LUT 参数会越界（对应到 acos/sqrt 的
// 无效输入域），产出 NaN，NaN 喂给纹理采样在很多实现下直接读回 0——这就是
// "大气壳是黑的"的根因。改成这里的物理积分，对任意相机位置（壳内/壳外）都
// 成立，不再需要区分。
// ==========================================================================

#include "common/shared_atmosphere.glsl"

// Set 1：渲染侧采样（VkAtmoLutPipelines::renderSetLayout，每个星球一份实例，
// 见 vk_atmosphere_lut.h::VkAtmosphereLut::renderSet / updateDepthBinding）。
// atmo_inside.frag / atmo_shell.frag 共用同一份声明，不用各自重复。
layout(set = 1, binding = 0) uniform texture2D inSceneDepth;
layout(set = 1, binding = 1) uniform texture2D inTransmittanceLut;
layout(set = 1, binding = 2) uniform texture2D inSkyViewLut; // 第一版壳内/壳外合成不再采样；保留绑定兼容 layout
layout(set = 1, binding = 3) uniform sampler   samp;

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

// scattered：路径积分散射光 L_inscatter
// viewT：视线 RGB 透射率 exp(-τ)（第一版合成用 luma(viewT) 当标量 T，经
//   ONE/ONE_MINUS_SRC_ALPHA 实现 L_out = scatter*E + scene*T；色度透射留到
//   读 HDR 的第二版）
// opacity：1 - luma(viewT)，兼容旧调用点
struct AtmoMarchResult { vec3 scattered; vec3 viewT; float opacity; };

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

// 仅边缘增益：rim=0（正对/天顶）→×1；rim=1（掠射轮廓/地平）→×limbBrightness。
// 无高度/壳内外渐变，Exposure 管整体，Limb 只管边缘。
float atmoLimbMul(float limbBrightness, float rim, float limbPower)
{
    float r = pow(clamp(rim, 0.0, 1.0), max(limbPower, 0.01));
    return mix(1.0, max(limbBrightness, 1.0), r);
}

// 壳内 rim：视线相对当地天顶，地平 |mu|≈0 → rim≈1。
float atmoInsideRim(vec3 rayDir, vec3 camRelPlanet)
{
    float len = length(camRelPlanet);
    vec3  up  = camRelPlanet / max(len, 1e-4);
    return clamp(1.0 - abs(dot(rayDir, up)), 0.0, 1.0);
}

// 统一合成输出：L_out = scatter*E + scene*T（靠 blend 乘 framebuffer 里的 scene）
// FragColor=(rgb, a)=(scatter*E, 1-T)，管线 ONE / ONE_MINUS_SRC_ALPHA。
vec4 atmoCompositeOut(vec3 scattered, vec3 viewT, float exposure)
{
    float T     = clamp(dot(viewT, vec3(0.299, 0.587, 0.114)), 0.0, 1.0);
    vec3  color = max(scattered * exposure, vec3(0.0));
    // ACES filmic（壳内/壳外一致）
    vec3 x = color;
    vec3 toned = (x * (2.51 * x + 0.03)) / (x * (2.43 * x + 0.59) + 0.14);
    return vec4(toned, 1.0 - T);
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
