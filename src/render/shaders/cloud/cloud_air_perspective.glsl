#ifndef CLOUD_AIR_PERSPECTIVE_GLSL
#define CLOUD_AIR_PERSPECTIVE_GLSL

// 云空气透视：Bruneton GetSkyRadianceToPoint（与 atmo_scatter_common / 场景大气同源）。
// 依赖 cloud_common.glsl 已声明：
//   inTransmittanceLut(10) inInscatterLut(11) linearClampEdgeSampler
// Inscatter 为单位太阳；Mie 走 combined A=Mie.r 外推（场景 mieStorageMode=0）。
// 调用方：L' = L_air×L_sun + T_air×L_cloud；T_air 写入独立 RGB RT。

struct CloudAirPerspective
{
    vec3 scattered;     // 相机→云段 in-scatter（单位太阳）
    vec3 transmittance; // 同段 RGB 透射 T_air
};

vec3 cloudApExtrapolateSingleMie(vec3 scattering, float mieRed,
                                 vec3 rayleighScattering, vec3 mieScattering)
{
    if (scattering.r <= 0.0 || mieScattering.r <= 0.0)
        return vec3(0.0);
    vec3 safeRayleigh = max(rayleighScattering, vec3(1e-8));
    return scattering * (mieRed / scattering.r)
        * (rayleighScattering.r / mieScattering.r)
        * (mieScattering / safeRayleigh);
}

vec3 cloudApTransmittanceToTop(float r, float mu, float bottomRadius, float topRadius)
{
    AtmosphereParameters atmo;
    atmo.bottomRadius = bottomRadius;
    atmo.topRadius = topRadius;
    vec2 uv;
    lutTransmittanceParamsToUv(atmo, r, mu, uv);
    return texture(sampler2D(inTransmittanceLut, linearClampEdgeSampler), uv).rgb;
}

vec3 cloudApSegmentTransmittance(float r, float mu, float d, bool hitGround,
                                 float bottomRadius, float topRadius)
{
    AtmosphereParameters atmo;
    atmo.bottomRadius = bottomRadius;
    atmo.topRadius = topRadius;

    float r_d, mu_d, mu_s_d;
    inscatterParamsAfterDistance(atmo, r, mu, 0.0, 0.0, d, r_d, mu_d, mu_s_d);

    if (hitGround) {
        vec3 Td = cloudApTransmittanceToTop(r_d, -mu_d, bottomRadius, topRadius);
        vec3 T0 = cloudApTransmittanceToTop(r, -mu, bottomRadius, topRadius);
        return clamp(Td / max(T0, vec3(1e-4)), vec3(0.0), vec3(1.0));
    }
    vec3 T0 = cloudApTransmittanceToTop(r, mu, bottomRadius, topRadius);
    vec3 Td = cloudApTransmittanceToTop(r_d, mu_d, bottomRadius, topRadius);
    return clamp(T0 / max(Td, vec3(1e-4)), vec3(0.0), vec3(1.0));
}

// 与 atmo_scatter_common::rayleighPhaseAtmo / miePhaseAtmo 同构（场景 ToPoint 用这套）
float cloudApRayleighPhase(float cosTheta)
{
    return 3.0 / (16.0 * 3.14159265359) * (1.0 + cosTheta * cosTheta);
}

float cloudApMiePhase(float cosTheta, float gMie)
{
    float g2 = gMie * gMie;
    float num = 3.0 * (1.0 - g2) * (1.0 + cosTheta * cosTheta);
    float denom = 8.0 * 3.14159265359 * (2.0 + g2)
        * pow(1.0 + g2 - 2.0 * gMie * cosTheta, 1.5);
    return num / max(denom, 1e-6);
}

vec3 cloudApApplyPhases(vec3 scattering, vec3 singleMie, float nu, float mu_s, float gMie)
{
    singleMie *= smoothstep(0.0, 0.01, mu_s);
    return scattering * cloudApRayleighPhase(nu) + singleMie * cloudApMiePhase(nu, gMie);
}

CloudAirPerspective cloudAirPerspectiveToPoint(
    vec3 camRelKm,
    vec3 cloudRelKm,
    vec3 viewDir,
    vec3 sunDir,
    const in AtmosphereParameters atmosphere)
{
    CloudAirPerspective ap;
    ap.scattered = vec3(0.0);
    ap.transmittance = vec3(1.0);

    vec3 p0 = camRelKm;
    if (length(p0) > atmosphere.topRadius) {
        if (!moveToTopAtmosphere(p0, viewDir, atmosphere.topRadius))
            return ap;
    }

    float r = length(p0);
    vec3 up = p0 / max(r, 1e-4);
    float mu = dot(viewDir, up);
    float mu_s = clamp(dot(sunDir, up), kInscatterMuSMin, 1.0);
    float nu = dot(viewDir, sunDir);
    float muBound = mu * mu_s + sqrt(max(0.0, (1.0 - mu * mu) * (1.0 - mu_s * mu_s)));
    nu = clamp(nu, -muBound, muBound);

    bool hitGround = inscatterRayIntersectsGround(atmosphere, r, mu);
    float d = max(dot(cloudRelKm - p0, viewDir), 0.0);
    if (d <= 1e-4)
        return ap;

    vec3 T = cloudApSegmentTransmittance(
        r, mu, d, hitGround, atmosphere.bottomRadius, atmosphere.topRadius);

    BrunetonReducedScattering s0 = sampleBrunetonReducedInscatterLut(
        inInscatterLut, linearClampEdgeSampler, atmosphere, r, mu, mu_s, nu, hitGround);

    float r_p, mu_p, mu_s_p;
    inscatterParamsAfterDistance(atmosphere, r, mu, mu_s, nu, d, r_p, mu_p, mu_s_p);
    BrunetonReducedScattering s1 = sampleBrunetonReducedInscatterLut(
        inInscatterLut, linearClampEdgeSampler, atmosphere, r_p, mu_p, mu_s_p, nu, hitGround);

    vec3 diffR = max(s0.scattering - T * s1.scattering, vec3(0.0));
    float diffMieRed = max(s0.mieRed - T.r * s1.mieRed, 0.0);
    vec3 diffMie = cloudApExtrapolateSingleMie(
        diffR, diffMieRed, atmosphere.rayleighScattering, atmosphere.mieScattering);

    ap.scattered = cloudApApplyPhases(diffR, diffMie, nu, mu_s, atmosphere.miePhaseG);
    ap.transmittance = T;
    return ap;
}

#endif
