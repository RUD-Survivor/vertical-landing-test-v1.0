#ifndef CLOUD_LIGHTING_GLSL
#define CLOUD_LIGHTING_GLSL

// ==========================================================================
// cloud_lighting.glsl — 云光照公共组装（低云 / 高空云共用）
//
// 低云 / 高空云并进 cloudMap 后共用主 march，以下函数为光照组装：
//   • sampleCloudAmbient       环境天光（μs LUT × L_sun × Exposure）
//   • powderEffectNew          Legacy 粉末/银边（density×height 随 VoL）
//   • powderSugarSunVisibility 堆糖：2·Beer·(1-e^{-2τ})，可按 VoL 门控
//   • cloudInScatterLuminance  组装某散射阶入射光 L_in（太阳路+可选环境/地面路）
//   • cloudScatterStep         RTE 前向积分的一步散射增量
//
// cloudPowderMode：0=Off 1=Legacy 2=Sugar（互斥；默认 Legacy）。
//
// 依赖（由 cloud_common.glsl 在 include 前提供）：
//   frameData（binding 21）、inCloudAmbientMusLut（binding 27）、
//   linearClampEdgeSampler（Set1）、analyticSkyAmbient（shared_atmosphere.glsl）。
// ==========================================================================

#define CLOUD_POWDER_MODE_OFF     0
#define CLOUD_POWDER_MODE_LEGACY  1
#define CLOUD_POWDER_MODE_SUGAR   2

// 环境天光：单位太阳 μs LUT × L_sun × cloudAmbientExposure（与相机无关）。
vec3 sampleCloudAmbient(const in AtmosphereParameters atmosphere, vec3 posKm, vec3 sunDirection)
{
    const vec3 localUp = normalize(posKm);
    const float muS = dot(localUp, sunDirection);
    const float expos = frameData.sky.atmosphereConfig.cloudAmbientExposure;
    const float analBlend = saturate(frameData.sky.atmosphereConfig.cloudAmbientAnalyticBlend);
    const vec3 Lsun = frameData.sky.color * frameData.sky.intensity;

    const float u = clamp(muS * 0.5 + 0.5, 0.0, 1.0);
    vec3 lutColor = texture(sampler2D(inCloudAmbientMusLut, linearClampEdgeSampler), vec2(u, 0.5)).rgb;
    vec3 analytic = analyticSkyAmbient(muS);
    return mix(lutColor, analytic, analBlend) * expos * Lsun;
}

// Legacy 粉末/银边：让云在背光、厚、云底时不要太黑，并随朝向太阳变化。
// depth=厚度因子；height=云底概率；VoL=dot(视线,光向)（背光为负）。
float powderEffectNew(float depth, float height, float VoL)
{
    float r = VoL * 0.5 + 0.5;
    r = r * r;
    height = height * (1.0 - r) + r;
    return depth * height;
}

// 堆糖（Powdered Sugar）：用光路光学深度 τ 修正 Beer，折痕/凸起对比。
// beer = exp(-τ)；sugar = 1-exp(-2τ)；energy = gain·beer·sugar（gain 默认 2）。
// viewWeight：1=全向套用原文；0=仅当太阳在身后（-VoL）时混入。
float powderSugarSunVisibility(float beer, float tau, float VoL, float gain, float viewWeight)
{
    float sugar = 1.0 - exp(-tau * 2.0);
    float lightEnergy = max(gain, 0.0) * beer * sugar;
    float applyW = mix(saturate(-VoL), 1.0, saturate(viewWeight));
    return mix(beer, lightEnergy, applyW);
}

// 组装某散射阶的入射光 L_in（单位：亮度）：
//   太阳路 = 云影 T_sun × 直射 sunlightTerm × 相位 × 粉末
//   环境路（仅 ms==0 或薄层高云）= 天空遮挡 × 环境天光 + 云底地面反弹
vec3 cloudInScatterLuminance(vec3 sunlightTerm, float sunVisibilityTerm,
                             float phaseTerm, float powderTerm,
                             vec3 ambientLit, float skyVisibilityTerm,
                             vec3 groundLit, bool addGroundContribution)
{
    vec3 L = sunlightTerm * sunVisibilityTerm * phaseTerm * powderTerm;
    if (addGroundContribution)
    {
        L += skyVisibilityTerm * ambientLit;
        L += groundLit;
    }
    return L;
}

// RTE 前向积分的散射增量：ΔL = T_view × (L_in×σ_s) × (1−T_step) / σ_t
vec3 cloudScatterStep(vec3 inScatterL, float viewTransmittance,
                      vec3 sigmaS, float sigmaE, float stepTransmittance)
{
    vec3 scatteredLitStep = inScatterL * sigmaS;
    return viewTransmittance * (scatteredLitStep - scatteredLitStep * stepTransmittance)
           / max(1e-4f, sigmaE);
}

#endif
