// cloud.glsl — Volumetric cloud GLSL functions.
// Injected into atmoFragSrc by CloudSystem::glslSource().
// Injection point: after ozoneDensity(), before getOpticalDepth().
//
// Depends on atmosphere globals already declared in the preceding shader source:
//   float gCloudMinAlt, gCloudMaxAlt; vec3 gCloudExtinction, gCloudScattering;
//   vec3 gRayleighCoeff; float gMieCoeff; float gHRayleigh, gHMie, gGMie;
//   vec3 gOzoneCoeff; float gHOzoneCenter, gHOzoneWidth;
//   uniform vec3 uCamPos, uLightDir, uPlanetCenter;
//   uniform float uInnerRadius, uOuterRadius, uSurfaceRadius;
//   uniform int uPlanetIdx; uniform float uRingInner, uRingOuter;
//   #define PI

// ── Forward declaration (getOpticalDepth is defined after this block) ─────────
void getOpticalDepth(vec3 p, vec3 lightDir,
    out float dR, out float dM, out float dO, out float dC);

// ── Cloud uniforms ────────────────────────────────────────────────────────────
uniform float uShowClouds;
uniform int   uFrameIndex;
uniform float uCloudTime;
uniform float uCloudPhaseSin;
uniform float uCloudPhaseCos;
uniform int   uCloudSteps;
uniform int   uLightSteps;
uniform float uTuneCovLo;
uniform float uTuneCovHi;
uniform float uTuneThreshLo;
uniform float uTuneThreshHi;
uniform float uTuneErosion;
uniform float uTuneDensity;
uniform float uDebugCoverage;
uniform sampler3D uNoiseTex3D;  // R=PerlinWorley G=Worley B=fbm(3oct) A=fbm(5oct)
uniform sampler3D uCoverTex3D;  // R=curl_lo G=curl_hi B=latNoise A=cov_med
uniform sampler3D uDetailTex3D; // 32^3: R/G/B=Worley, A=fBm blend
uniform sampler2D uWeatherMap;  // 2048x1024: R=coverage_bias G=cloud_type B=humidity A=land

// ── Screen-space dither (IGN + TAA) — also used by atmosphere ray jitter ─────
float screenHash(vec2 uv) {
    float ign = fract(52.9829189 * fract(dot(uv, vec2(0.06711056, 0.00583715))));
    float animated = fract(ign + float(uFrameIndex % 32) * 0.6180339887);
    return animated;
}

// ── Noise helpers ─────────────────────────────────────────────────────────────
vec3 hashGrad(vec3 p) {
    vec3 q = vec3(dot(p, vec3(127.1, 311.7, 74.7)),
                  dot(p, vec3(269.5, 183.3, 246.1)),
                  dot(p, vec3(113.5, 271.9, 124.6)));
    return normalize(-1.0 + 2.0 * fract(q * fract(q * 0.3183099)));
}
float noise3d(vec3 p) {
    vec3 i = floor(p); vec3 f = fract(p);
    vec3 u = f * f * f * (f * (f * 6.0 - 15.0) + 10.0);
    return mix(mix(mix(dot(hashGrad(i + vec3(0,0,0)), f - vec3(0,0,0)),
                       dot(hashGrad(i + vec3(1,0,0)), f - vec3(1,0,0)), u.x),
                   mix(dot(hashGrad(i + vec3(0,1,0)), f - vec3(0,1,0)),
                       dot(hashGrad(i + vec3(1,1,0)), f - vec3(1,1,0)), u.x), u.y),
               mix(mix(dot(hashGrad(i + vec3(0,0,1)), f - vec3(0,0,1)),
                       dot(hashGrad(i + vec3(1,0,1)), f - vec3(1,0,1)), u.x),
                   mix(dot(hashGrad(i + vec3(0,1,1)), f - vec3(0,1,1)),
                       dot(hashGrad(i + vec3(1,1,1)), f - vec3(1,1,1)), u.x), u.y), u.z)
           * 0.5 + 0.5;
}
float fbm(vec3 p, int octaves) {
    float v = 0.0, amp = 0.5;
    for (int i = 0; i < octaves; i++) {
        v += noise3d(p) * amp;
        p = p * 2.07 + vec3(0.131, -0.217, 0.344);
        amp *= 0.48;
    }
    return v;
}
float worley3d(vec3 p) {
    vec3 i = floor(p); vec3 f = fract(p);
    float md = 1.0;
    for (int x = -1; x <= 1; x++)
    for (int y = -1; y <= 1; y++)
    for (int z = -1; z <= 1; z++) {
        vec3 nb = vec3(float(x), float(y), float(z));
        vec3 h = fract(sin(vec3(dot(i+nb, vec3(127.1,311.7,74.7)),
                               dot(i+nb, vec3(269.5,183.3,246.1)),
                               dot(i+nb, vec3(113.5,271.9,124.6)))) * 43758.5453);
        vec3 d = nb + h - f;
        md = min(md, dot(d, d));
    }
    return sqrt(md);
}
float fbm_pw(vec3 p, int octaves) {
    float pn = fbm(p, octaves);
    float wn = 1.0 - worley3d(p * 1.8);
    return clamp(pn - (1.0 - wn) * 0.32, 0.0, 1.0);
}

// ── Cloud sphere utilities ────────────────────────────────────────────────────
vec3 curlSph(vec3 p, float scale, float toff) {
    vec3 n = p * scale + vec3(toff, toff * 0.7, toff * 1.3);
    const float eps = 0.015;
    float dx = noise3d(n + vec3(eps,0,0)) - noise3d(n - vec3(eps,0,0));
    float dy = noise3d(n + vec3(0,eps,0)) - noise3d(n - vec3(0,eps,0));
    float dz = noise3d(n + vec3(0,0,eps)) - noise3d(n - vec3(0,0,eps));
    vec3 grad = vec3(dx, dy, dz) * (0.5 / eps);
    grad -= p * dot(p, grad);
    return cross(p, grad);
}

float rawCoverage(vec3 sph, float t) {
    float latWarp = sph.z * sph.z * sph.z * 0.3;
    float sB = sin(latWarp); float cB = cos(latWarp);
    float s1 = uCloudPhaseSin * cB + uCloudPhaseCos * sB;
    float c1 = uCloudPhaseCos * cB - uCloudPhaseSin * sB;
    vec3 cs = vec3(sph.x*c1 - sph.y*s1, sph.x*s1 + sph.y*c1, sph.z);
    vec3 ax = normalize(cross(cs, vec3(0.0, 0.0, 1.0) + cs * 0.01));
    vec3 va = cross(cs, ax);
    const float EPS = 0.06;
    vec3 slLo = vec3(t * 0.003, 0.0, t * 0.002);
    float rLo_pu = fbm(cs * 1.5 + ax * EPS + slLo, 2);
    float rLo_mu = fbm(cs * 1.5 - ax * EPS + slLo, 2);
    float rLo_pv = fbm(cs * 1.5 + va * EPS + slLo, 2);
    float rLo_mv = fbm(cs * 1.5 - va * EPS + slLo, 2);
    float cUlo = -(rLo_pv - rLo_mv);
    float cVlo =  (rLo_pu - rLo_mu);
    vec3 slHi = vec3(91.2 + t * 0.005, 5.1, 22.0);
    float rHi_pu = fbm(cs * 6.0 + ax * EPS + slHi, 1);
    float rHi_mu = fbm(cs * 6.0 - ax * EPS + slHi, 1);
    float rHi_pv = fbm(cs * 6.0 + va * EPS + slHi, 1);
    float rHi_mv = fbm(cs * 6.0 - va * EPS + slHi, 1);
    float cUhi = -(rHi_pv - rHi_mv);
    float cVhi =  (rHi_pu - rHi_mu);
    float wU = cUlo * 2.5 + cUhi * 1.0;
    float wV = cVlo * 2.5 + cVhi * 1.0;
    vec3 ws = normalize(cs + ax * wU + va * wV);
    float latNoise = fbm(cs*4.0 + vec3(127.3, 88.1, 9.6+t*0.006), 3) - 0.5;
    float warpedLat = sph.z + latNoise * 0.40;
    float cov = fbm(ws * 0.8 + vec3(t*0.004), 5) * 0.65
              + fbm(ws * 6.0 + vec3(t*0.011, -t*0.008, t*0.013), 3) * 0.35;
    vec2 rwUV = vec2(atan(ws.y, ws.x) * (0.5 / PI) + 0.5,
                    asin(clamp(ws.z, -1.0, 1.0)) / PI + 0.5);
    float wxCov = (uPlanetIdx == 3) ? texture(uWeatherMap, rwUV).r : 0.65;
    return clamp(cov * wxCov, 0.0, 1.0);
}

// ── Cloud density ─────────────────────────────────────────────────────────────
float cloudDensity(vec3 p, float h, bool highDetail) {
    if (uShowClouds < 0.5) return 0.0;
    if (h < gCloudMinAlt || h > gCloudMaxAlt) return 0.0;

    vec3 sph = normalize(p - uPlanetCenter);
    float t = uCloudTime * 0.004;
    float density = 0.0;

    if (uPlanetIdx == 3) {
        if (h >= gCloudMinAlt && h <= 7.0) {
            float altNorm = (h - gCloudMinAlt) / (7.0 - gCloudMinAlt);
            float altBase = smoothstep(0.0, 0.02, altNorm);

            float latWarp = sph.z * sph.z * sph.z * 0.3;
            float sB = sin(latWarp); float cB = cos(latWarp);
            float s1 = uCloudPhaseSin * cB + uCloudPhaseCos * sB;
            float c1 = uCloudPhaseCos * cB - uCloudPhaseSin * sB;
            vec3 windSph1 = vec3(sph.x * c1 - sph.y * s1, sph.x * s1 + sph.y * c1, sph.z);

            float cycAngle = t * -0.1;
            vec3 cycCenter = normalize(vec3(0.6*cos(cycAngle), 0.6*sin(cycAngle), 0.35));
            float cycDist = distance(sph, cycCenter);
            float cycRadius = 0.22;
            float cycInfluence = smoothstep(cycRadius, 0.0, cycDist);
            if (cycInfluence > 0.001) {
                float twist = cycInfluence * 1.8;
                float ct = cos(twist); float st = sin(twist);
                vec3 cycCenterW = normalize(vec3(cycCenter.x*c1 - cycCenter.y*s1,
                                                 cycCenter.x*s1 + cycCenter.y*c1, cycCenter.z));
                vec3 warped = windSph1*ct + cross(cycCenterW, windSph1)*st + cycCenterW*dot(cycCenterW, windSph1)*(1.0-ct);
                windSph1 = mix(windSph1, warped, cycInfluence);
            }

            vec3 coverageSph = vec3(sph.x * c1 - sph.y * s1, sph.x * s1 + sph.y * c1, sph.z);
            vec3 wAxis = normalize(cross(coverageSph, vec3(0.0, 0.0, 1.0) + coverageSph * 0.01));
            vec3 vAxis = cross(coverageSph, wAxis);
            const float EPS = 0.06;
            const float INV8 = 0.125;
            vec3 seedLo = vec3(t * 0.003, 0.0, t * 0.002);
            float pLo_pu = texture(uCoverTex3D,(coverageSph*1.5+wAxis*EPS+seedLo)*INV8).r;
            float pLo_mu = texture(uCoverTex3D,(coverageSph*1.5-wAxis*EPS+seedLo)*INV8).r;
            float pLo_pv = texture(uCoverTex3D,(coverageSph*1.5+vAxis*EPS+seedLo)*INV8).r;
            float pLo_mv = texture(uCoverTex3D,(coverageSph*1.5-vAxis*EPS+seedLo)*INV8).r;
            float curlU_lo = -(pLo_pv - pLo_mv);
            float curlV_lo =  (pLo_pu - pLo_mu);
            vec3 seedHi = vec3(91.2 + t * 0.005, 5.1, 22.0);
            float pHi_pu = texture(uCoverTex3D,(coverageSph*6.0+wAxis*EPS+seedHi)*INV8).g;
            float pHi_mu = texture(uCoverTex3D,(coverageSph*6.0-wAxis*EPS+seedHi)*INV8).g;
            float pHi_pv = texture(uCoverTex3D,(coverageSph*6.0+vAxis*EPS+seedHi)*INV8).g;
            float pHi_mv = texture(uCoverTex3D,(coverageSph*6.0-vAxis*EPS+seedHi)*INV8).g;
            float curlU_hi = -(pHi_pv - pHi_mv);
            float curlV_hi =  (pHi_pu - pHi_mu);
            float warpU = curlU_lo * 2.5 + curlU_hi * 1.0;
            float warpV = curlV_lo * 2.5 + curlV_hi * 1.0;
            vec3 warpedSph = normalize(coverageSph + wAxis * warpU + vAxis * warpV);
            float latNoise = texture(uCoverTex3D,(coverageSph*4.0+vec3(127.3,88.1,9.6+t*0.006))*INV8).b - 0.5;
            float warpedLat = sph.z + latNoise * 0.40;
            float cov_large  = texture(uNoiseTex3D,(warpedSph*0.8+vec3(t*0.004))*INV8).a;
            float cov_medium = texture(uCoverTex3D,(warpedSph*6.0+vec3(t*0.011,-t*0.008,t*0.013))*INV8).a;
            float coverage = cov_large * 0.65 + cov_medium * 0.35;
            vec2 wUV = vec2(
                atan(warpedSph.y, warpedSph.x) * (0.5 / PI) + 0.5,
                asin(clamp(warpedSph.z, -1.0, 1.0)) / PI + 0.5
            );
            vec4 weather = (uPlanetIdx == 3) ? texture(uWeatherMap, wUV) : vec4(0.65, 0.5, 0.6, 0.0);
            coverage = clamp(coverage * weather.r, 0.0, 1.0);
            float covRange = max(uTuneCovHi - uTuneCovLo, 0.001);
            coverage = clamp((coverage - uTuneCovLo) / covRange, 0.0, 1.0);

            if (cycInfluence > 0.001) {
                coverage = mix(coverage, clamp(coverage * 1.9, 0.0, 1.0), cycInfluence * 0.7);
                float eyeNoise = fbm(sph * 38.0 + vec3(t * 0.02), 2) * 0.009;
                float eyeRadius = 0.017 + eyeNoise;
                coverage *= smoothstep(eyeRadius * 0.35, eyeRadius, cycDist);
            }

            // ── Physics-driven vertical profiles per cloud type ──────────────────
            // Stratus:  uniform layer, sharp base, gradual top
            // Cumulus:  flat base, dense core at 0.3-0.6h, cauliflower top
            // Cumulonimbus: tower + anvil shelf at 0.7-0.95h
            float cloudType = mix(texture(uCoverTex3D, (warpedSph * 2.5 + vec3(31.4, 17.8, 42.1)) * INV8).r, weather.g, 0.55);
            float stratusProfile      = altBase * (1.0 - smoothstep(0.82, 1.0, altNorm));
            float cumulusCore         = exp(-((altNorm - 0.35) * (altNorm - 0.35)) / (0.25 * 0.25));
            float topNoise  = texture(uCoverTex3D, (windSph1 * 8.0 + vec3(53.2, 29.7, 71.4 + t * 0.01)) * INV8).r;
            float topHeight = mix(0.55, 1.00, topNoise);
            float cumulusTop          = 1.0 - smoothstep(topHeight - 0.08, topHeight + 0.08, altNorm);
            float cumulusProfile      = altBase * cumulusCore * cumulusTop;
            float cbTower             = smoothstep(0.0, 0.50, altNorm) * (1.0 - smoothstep(0.50, 0.72, altNorm));
            float cbAnvil             = smoothstep(0.68, 0.80, altNorm) * (1.0 - smoothstep(0.88, 1.00, altNorm)) * 0.35;
            float cumulonimbusProfile = altBase * max(cbTower, cbAnvil);
            float heightGradient = mix(
                mix(stratusProfile, cumulusProfile, smoothstep(0.0, 0.5, cloudType)),
                cumulonimbusProfile,
                smoothstep(0.5, 1.0, cloudType)
            );
            float altShape = heightGradient * 1.5;

            vec3 tc = windSph1 * (80.0 + h * 0.45) + vec3(t * 0.6, t * 0.4, t * 0.15);
            float n = 0.0;
            float viewDist = length(uCamPos - p);  // hoisted: used by LOD details + turbulence gate
            if (highDetail) {
                float warp = texture(uNoiseTex3D, tc * 0.0375).b;
                vec3 warpedTc = tc + vec3(warp * 0.4, warp * 0.4, warp * 0.2) + vec3(t * 0.02);
                // 3 octaves: avoids 10km-wavelength 4th-octave boiling at close range.
                // The mesoscale modulation + LOD detail levels already provide
                // sufficient frequency diversity without the 4th FBM octave.
                n = fbm(warpedTc, 3);
                float wInv = 1.0 - texture(uNoiseTex3D, warpedTc * 0.125).g;
                n = n + (wInv - 0.5) * (1.0 - n) * 0.50;
                float ero = texture(uNoiseTex3D, tc * 0.2875 + vec3(-t * 0.04, t * 0.03, 0.0)).g;
                float topFraction = smoothstep(topHeight * 0.5, topHeight * 0.95, altNorm);
                float eroDir = mix(ero, 1.0 - ero, topFraction);
                n -= eroDir * uTuneErosion;
                {
                    vec3 pW = p + vec3(warp - 0.5, warp * 1.4 - 0.7, 0.5 - warp * 0.9) * 18.0;
                    float c1 = 1.0 - texture(uNoiseTex3D, pW * 0.005   + vec3(0.914 + t * 0.00225, 0.521, 0.341 + t * 0.00150)).g;
                    float c2 = 1.0 - texture(uNoiseTex3D, pW * 0.00975 + vec3(0.039, 0.440 + t * 0.00275, 0.091)).g;
                    n += (c1 * (0.4 + 0.6 * c2)) * 0.30 - 0.16;
                }
                float maskLoVal = min(uTuneThreshLo, uTuneThreshHi);
                float cloudExists = smoothstep(maskLoVal + 0.02, maskLoVal + 0.10, n);
                if (viewDist < 250.0) {
                    float wm = 1.0 - smoothstep(80.0, 250.0, viewDist);
                    float cellVal = 1.0 - texture(uNoiseTex3D, p * 0.005 + vec3(t * 0.00375, 0.0, 0.0)).g;
                    float surfaceProx = smoothstep(0.20, 0.50, n) * (1.0 - smoothstep(0.50, 0.70, n));
                    n += (cellVal - 0.5) * 0.22 * wm * (0.3 + 0.7 * surfaceProx) * cloudExists;
                }
                if (viewDist < 80.0) {
                    float lm = 1.0 - smoothstep(40.0, 80.0, viewDist);
                    float largeW = 1.0 - texture(uNoiseTex3D, p * 0.015625 + vec3(t * 0.003125, t * 0.001875, 0.0)).g;
                    float largeSurf = smoothstep(0.15, 0.45, n) * (1.0 - smoothstep(0.45, 0.65, n));
                    n += (largeW - 0.5) * 0.14 * lm * (0.30 + 0.70 * largeSurf) * cloudExists;
                }
                if (viewDist < 30.0) {
                    float mm = 1.0 - smoothstep(15.0, 30.0, viewDist);
                    float mesoW = 1.0 - texture(uNoiseTex3D, p * 0.04750 + vec3(t * 0.0075, t * 0.005, 0.0)).g;
                    float mesoSurf = smoothstep(0.15, 0.45, n) * (1.0 - smoothstep(0.45, 0.65, n));
                    n += (mesoW - 0.5) * 0.18 * mm * (0.35 + 0.65 * mesoSurf) * cloudExists;
                }
                if (viewDist < 8.0) {
                    float dW1  = 1.0 - smoothstep(0.5, 8.0, viewDist);
                    vec3  uvA  = fract(p / 0.200 + vec3(0.137, 0.421, 0.073));
                    float da   = texture(uDetailTex3D, uvA).a;
                    float tFr1 = smoothstep(0.5, 0.85, altNorm);
                    float dirDa = mix(da, 1.0 - da, tFr1);
                    float surfW = 1.0 - smoothstep(maskLoVal, maskLoVal + 0.10, n);
                    n -= dirDa * (0.10 + 0.30 * surfW) * dW1;
                }
                if (viewDist < 0.5) {
                    float dW2   = 1.0 - smoothstep(0.05, 0.5, viewDist);
                    vec3  uvG   = fract(p / 0.040 + vec3(0.251, 0.637, 0.184));
                    float dg    = texture(uDetailTex3D, uvG).g;
                    float tFr2  = smoothstep(0.5, 0.85, altNorm);
                    float dirDg = mix(dg, 1.0 - dg, tFr2);
                    float surfW2 = 1.0 - smoothstep(maskLoVal, maskLoVal + 0.08, n);
                    n -= dirDg * (0.06 + 0.18 * surfW2) * dW2;
                }
                if (viewDist < 0.10) {
                    float dW3 = 1.0 - smoothstep(0.01, 0.10, viewDist);
                    vec3  uvB = fract(p / 0.008 + vec3(0.712, 0.053, 0.835));
                    float db  = texture(uDetailTex3D, uvB).b;
                    float tFr3 = smoothstep(0.5, 0.85, altNorm);
                    n -= mix(db, 1.0 - db, tFr3) * 0.08 * dW3;
                }
            } else {
                n = fbm(tc, 2);
                { float wInvS = 1.0 - texture(uNoiseTex3D, tc * 0.125).g;
                  n = n + (wInvS - 0.5) * (1.0 - n) * 0.50; }
                {
                    float pw = texture(uNoiseTex3D, tc * 0.0375).b;
                    vec3 pW = p + vec3(pw - 0.5, pw * 1.4 - 0.7, 0.5 - pw * 0.9) * 18.0;
                    float c1 = 1.0 - texture(uNoiseTex3D, pW * 0.005   + vec3(0.914 + t * 0.00225, 0.521, 0.341 + t * 0.00150)).g;
                    float c2 = 1.0 - texture(uNoiseTex3D, pW * 0.00975 + vec3(0.039, 0.440 + t * 0.00275, 0.091)).g;
                    n += (c1 * (0.4 + 0.6 * c2)) * 0.30 - 0.16;
                }
            }

            // ── Turbulent updraft cores (close-range only: invisible from orbit) ─
            // curlSph + worley3d is ~35% of per-sample cost. Only activate
            // within 40 km where individual updraft columns are resolvable.
            if (viewDist < 40.0) {
                float edgeMaskLo = min(uTuneThreshLo, uTuneThreshHi);
                vec3 curlVec = curlSph(windSph1 * 4.0, 3.0, t * 0.3);
                float tke = abs(dot(curlVec, windSph1)) * 2.5;
                float tkeThreshold = smoothstep(0.35, 0.75, tke);
                float distToCore = 1.0 - worley3d(windSph1 * 14.0 + t * 0.12);
                float coreStrength = exp(-distToCore * distToCore * 18.0) * tkeThreshold;
                float surfaceZone = smoothstep(edgeMaskLo - 0.06, edgeMaskLo + 0.02, n)
                                  * (1.0 - smoothstep(edgeMaskLo + 0.18, edgeMaskLo + 0.35, n));
                n += coreStrength * 0.12 * surfaceZone * (0.4 + 0.6 * altNorm);

                // ── Entrainment mixing fingers (close-range, additive only) ──
                if (viewDist < 25.0) {
                    float edgeZone = smoothstep(edgeMaskLo - 0.04, edgeMaskLo + 0.04, n)
                                   * (1.0 - smoothstep(edgeMaskLo + 0.08, edgeMaskLo + 0.20, n));
                    float fingers = fbm(p * 24.0 + vec3(t * 0.09, t * 0.06, 0.0), 3);
                    n += edgeZone * fingers * 0.10 * tke;
                }
            }

            // ── Coverage-driven percolation threshold ────────────────────────
            // Architecture fix: instead of coverage being a scalar density
            // multiplier (which just dims clouds without changing their shape),
            // coverage modulates the noise threshold itself — creating a
            // percolation transition:
            //   high coverage → low threshold → most FBM passes → connected sheets
            //   low coverage → high threshold → only FBM peaks pass → sparse fragments
            //   below critical coverage → no cloud can form (percolation cutoff)
            float maskLo = min(uTuneThreshLo, uTuneThreshHi);
            float maskHi = max(uTuneThreshLo, uTuneThreshHi);
            float formPotential = smoothstep(0.18, 0.42, coverage);
            float localMaskLo = mix(0.58, maskLo, formPotential);
            float localMaskHi = mix(0.76, maskHi, formPotential);

            // ── Mesoscale thickness modulation (~100-300km) ─────────────────
            // Without this layer, the FBM's ~20-80km dominant frequency creates
            // regularly-spaced "Swiss cheese" holes within cloud sheets.
            // A separate low-frequency FBM varies the effective threshold across
            // the cloud sheet, producing natural thick/thin mesoscale regions.
            float mesoField = texture(uCoverTex3D, 
                (windSph1 * 1.2 + vec3(71.3, 39.7, 14.2 + t * 0.002)) * INV8).r;
            float thickVar = (mesoField - 0.45) * 0.28;
            float mesoLo = localMaskLo + thickVar;
            float mesoHi = localMaskHi + thickVar * 0.5;

            float mask = smoothstep(mesoLo, mesoHi, n) * altShape;
            // Percolation threshold (above) controls WHERE cloud exists.
            // Coverage multiplier (below) controls HOW DENSE the existing cloud is —
            // producing natural density gradation from cloud-core to cloud-periphery.
            density = max(density, mask * uTuneDensity * (0.30 + 0.70 * coverage));
        }

        if (h >= 8.0 && h <= gCloudMaxAlt) {
            float altNorm = (h - 8.0) / (gCloudMaxAlt - 8.0);
            // Asymmetric cirrus profile: sharp ice-crystal formation at base,
            // exponential wisp decay at top (wind-shear streak effect)
            float cirrusBase   = smoothstep(0.0, 0.08, altNorm);
            float cirrusWisp   = exp(-altNorm * 3.2);
            float altShape = cirrusBase * cirrusWisp;
            float angle2 = t * -0.5;
            float latWarp2 = sph.z * sph.z * sph.z * 0.15;
            float s2 = sin(angle2 + latWarp2); float c2 = cos(angle2 + latWarp2);
            vec3 windSph2 = vec3(sph.x * c2 - sph.y * s2, sph.x * s2 + sph.y * c2, sph.z);
            vec3 tc2 = windSph2 * 50.0;
            float n2 = 0.0;
            if (highDetail) {
                float warp2 = fbm(tc2 * 0.4, 2);
                n2 = fbm(tc2 + vec3(warp2 * 0.7, warp2 * 0.9, 0.0) + vec3(t * 0.04), 4);
                float detail2 = fbm(tc2 * 2.5 + vec3(t * 0.08), 2);
                n2 = n2 - detail2 * 0.18;
            } else {
                n2 = fbm(tc2 + vec3(t * 0.04), 1);
            }
            float mask2 = smoothstep(0.42 + altNorm * 0.10, 0.82, n2) * altShape;
            density = max(density, mask2 * 0.30);
        }

    } else if (uPlanetIdx == 2) {
        float altNorm = (h - gCloudMinAlt) / (gCloudMaxAlt - gCloudMinAlt);
        float altShape = 4.0 * altNorm * (1.0 - altNorm);
        float angle = t * -0.1;
        float s1 = sin(angle); float c1 = cos(angle);
        vec3 windSph = vec3(sph.x * c1 - sph.y * s1, sph.x * s1 + sph.y * c1, sph.z);
        float n = highDetail ? fbm(windSph * 8.0 + vec3(t * 0.05), 3) : fbm(windSph * 8.0 + vec3(t * 0.05), 1);
        density = (0.3 + 0.7 * n) * altShape * 0.8;
    }
    return density;
}

// ── Cloud phase function (dual-lobe HG) ───────────────────────────────────────
float cloudPhase(float cosTheta) {
    float g1 = 0.85;
    float num1 = 1.0 - g1 * g1;
    float denom1 = pow(1.0 + g1 * g1 - 2.0 * g1 * cosTheta, 1.5);
    float phase1 = (1.0 / (4.0 * PI)) * (num1 / denom1);
    float g2 = -0.3;
    float num2 = 1.0 - g2 * g2;
    float denom2 = pow(1.0 + g2 * g2 - 2.0 * g2 * cosTheta, 1.5);
    float phase2 = (1.0 / (4.0 * PI)) * (num2 / denom2);
    return mix(phase2, phase1, 0.7);
}

// ── Cone-traced ambient occlusion (lightweight, ~3-probe) ─────────────────────
// Estimates how much neighbouring cloud mass surrounds a point by probing
// downward and sideways with cheap 2-oct FBM density samples.
// Returns [0,1]: 0 = fully buried in cloud, 1 = exposed to open sky.
float coneAO(vec3 p, vec3 sph, float h) {
    if (h < gCloudMinAlt + 0.3) return 0.08;  // cloud base is always dark

    // 3 probe directions: straight down (radial inward) + 2 orthogonal tangents
    vec3 down = -sph;
    vec3 tan1 = normalize(cross(sph, vec3(0.371, 0.629, 0.683)));
    vec3 tan2 = cross(sph, tan1);

    float occlusion = 0.0;
    float weightSum  = 0.0;

    // 3 probe distances: 0.12, 0.28, 0.50 km (non-linear spacing)
    float dists[3] = float[3](0.12, 0.28, 0.50);
    float wgts[3]  = float[3](0.50, 0.30, 0.20);

    for (int d = 0; d < 3; d++) {
        // Downward probe (heaviest weight — most meaningful occlusion)
        {
            vec3 probePos = p + down * dists[d];
            float probeH = length(probePos - uPlanetCenter) - uSurfaceRadius;
            if (probeH >= gCloudMinAlt && probeH <= gCloudMaxAlt) {
                vec3 probeSph = normalize(probePos - uPlanetCenter);
                float dC = fbm_pw(probeSph * 3.0, 2);  // cheap 2-oct estimate
                occlusion += dC * wgts[d] * 0.50;
            }
            weightSum += wgts[d] * 0.50;
        }
        // Tangent probe 1
        {
            vec3 probePos = p + tan1 * dists[d] * 1.5;
            float probeH = length(probePos - uPlanetCenter) - uSurfaceRadius;
            if (probeH >= gCloudMinAlt && probeH <= gCloudMaxAlt) {
                vec3 probeSph = normalize(probePos - uPlanetCenter);
                float dC = fbm_pw(probeSph * 3.0, 2);
                occlusion += dC * wgts[d] * 0.25;
            }
            weightSum += wgts[d] * 0.25;
        }
        // Tangent probe 2
        {
            vec3 probePos = p + tan2 * dists[d] * 1.5;
            float probeH = length(probePos - uPlanetCenter) - uSurfaceRadius;
            if (probeH >= gCloudMinAlt && probeH <= gCloudMaxAlt) {
                vec3 probeSph = normalize(probePos - uPlanetCenter);
                float dC = fbm_pw(probeSph * 3.0, 2);
                occlusion += dC * wgts[d] * 0.25;
            }
            weightSum += wgts[d] * 0.25;
        }
    }

    float avgOcclusion = (weightSum > 0.001) ? occlusion / weightSum : 0.0;
    // Remap: even moderate occlusion should darken noticeably
    return 1.0 - avgOcclusion * 1.3;
}

// ── Cloud ray march: shadow + inscatter ───────────────────────────────────────
// Encapsulates the full volumetric cloud pass (adaptive big/fine step + probe).
// Uses gCloudMinAlt/MaxAlt/Extinction/Scattering from atmosphere globals.
void marchClouds(
    vec3  rayDir,
    float tFar,
    float cosTheta,
    float optDepthR, float optDepthM, float optDepthO,
    out vec3  sumCloud,
    out float optDepthC
) {
    sumCloud  = vec3(0.0);
    optDepthC = 0.0;

    float cloudInnerR = uSurfaceRadius + gCloudMinAlt;
    float cloudOuterR = uSurfaceRadius + gCloudMaxAlt;

    float cNear, cFar;
    // intersectSphere is declared in the atmosphere shader (part1), before this block
    if (!intersectSphere(uCamPos, rayDir, cloudOuterR, cNear, cFar)) return;

    float cStart = (cNear < 0.0) ? 0.0 : cNear;
    float cEnd   = cFar;

    float iNear, iFar;
    if (intersectSphere(uCamPos, rayDir, cloudInnerR, iNear, iFar) && iNear > 0.0)
        cEnd = min(cEnd, iNear);
    cEnd = min(cEnd, tFar);

    float cloudSegLen = cEnd - cStart;
    if (cloudSegLen <= 0.0) return;

    float camAlt_cloud = length(uCamPos - uPlanetCenter) - uSurfaceRadius;
    bool inCloudLayer  = (camAlt_cloud < gCloudMaxAlt + 5.0);
    // Step sizes: 300 m balances detail capture (~47 samples / 14 km) with
    // performance. 200 m gave ~70 samples but was too expensive after the
    // percolation + mesoscale additions to cloudDensity().
    float fineStep = 0.30;
    float bigStep  = inCloudLayer ? 2.50 : 4.0;

    // Per-row golden-ratio jitter, same rationale as atmosphere primary march:
    // IGN (~3.2-row period) causes systematic cloud density differences between rows
    // → horizontal bands when viewing cloud layer at a grazing angle from above.
    // Golden ratio is irrational → no period → every row is independent.
    // Same 2D IGN fix as atmosphere primary march: y-only jitter creates concentric rings
    // when looking straight down (same y = same ring radius = same jitter = same cloud density).
    float cIgn = fract(52.9829189 * fract(dot(gl_FragCoord.xy, vec2(0.06711056, 0.00583715))));
    float cJitter = fract(cIgn + float(uFrameIndex % 64) * 0.6180339887 + 0.38196601125)
                  * (inCloudLayer ? 0.20 : 1.0);
    float entryFrac = clamp(cStart / max(tFar, 0.001), 0.0, 1.0);
    vec3 atmTransAtCloud = exp(-(gRayleighCoeff * optDepthR + gMieCoeff * optDepthM + gOzoneCoeff * optDepthO) * entryFrac);

    float tPos = cStart + cJitter * fineStep;
    float prevDC = 0.0;  // for edge-detection silver lining
    const int MAX_CLOUD_ITER = 128;

    for (int ci = 0; ci < MAX_CLOUD_ITER; ci++) {
        if (tPos >= cEnd) break;

        vec3 cPos = uCamPos + rayDir * tPos;
        float ch = max(length(cPos - uPlanetCenter) - uSurfaceRadius, 0.0);

        // Altitude-based step decision: no texture probe.
        // Any probe sampled near the cloud entry point is nearly constant for all rays
        // at the same elevation angle (same ring) → probe threshold creates concentric rings.
        // Altitude is smooth and spherically symmetric → no ring artifacts.
        if (ch < gCloudMinAlt || ch > gCloudMaxAlt) { tPos += bigStep; continue; }

        vec3 probeSph = normalize(cPos - uPlanetCenter);
        float dC = cloudDensity(cPos, ch, true) * fineStep;
        if (dC < 0.0001) { tPos += fineStep; continue; }

        optDepthC += dC;

        float cLightR, cLightM, cLightO, cLightC;
        getOpticalDepth(cPos, uLightDir, cLightR, cLightM, cLightO, cLightC);

        vec3 cloudShadow = exp(-gCloudExtinction * cLightC);
        vec3 sunTint = exp(-(gRayleighCoeff * cLightR + gMieCoeff * 1.1 * cLightM + gOzoneCoeff * cLightO))
                     * cloudShadow;

        // ── Spectral multi-scatter (RGB wavelength-dependent) ─────────────────
        // Blue scatters more in Rayleigh regime → longer path → redder clouds at depth.
        // Simple vectorised Jimenez 4-term with wavelength bias.
        vec3 tauRgb = gCloudExtinction.x * (optDepthC + cLightC)
                    * vec3(1.0, 1.04, 1.10);  // R preserved, G slightly, B most attenuated
        // Jimenez 4-term: higher divisor → less brightening of cloud tops.
        vec3 msRgb = (exp(-tauRgb) + exp(-tauRgb * 0.5) * 0.5
                   + exp(-tauRgb * 0.25) * 0.25 + exp(-tauRgb * 0.125) * 0.125) / 6.0;

        // ── Core darkening + silver-lining ────────────────────────────────────
        // powder: 0=edge(near sun), 1=deep inside cloud (matches cloud.frag logic)
        float tauLight  = gCloudExtinction.x * cLightC;
        float powder = 1.0 - exp(-tauLight * 2.0);
        float coreDark = 1.0 - powder * 0.72;  // dense interior → darker core
        // Edge gradient: previous-step density drop → cloud boundary
        float edgeSharpness = smoothstep(0.002, 0.018, abs(dC - prevDC) / max(fineStep, 0.0001));
        float powderEdge = powder * (1.0 + edgeSharpness * 0.7);
        float silverMask = max(0.0, cosTheta);
        vec3 cloudAtt = msRgb * (coreDark + powderEdge * silverMask * 0.65);
        prevDC = dC;

        // ── Ambient occlusion: height-based + cone-traced ──────────────────
        // Height AO: free baseline bottom→top gradient.
        // Cone-traced AO: spatial surrounding-occlusion, only at close range
        // (<60 km, every 8th step) where the effect is visually resolvable.
        float layerBase = (ch < 7.5) ? gCloudMinAlt : 8.0;
        float layerTop  = (ch < 7.5) ? 7.0 : gCloudMaxAlt;
        float heightInBand = clamp((ch - layerBase) / max(layerTop - layerBase, 0.1), 0.0, 1.0);
        float aoHeight = pow(heightInBand, 0.55);
        float aoCone   = aoHeight;
        float cloudDist = length(uCamPos - cPos);
        if (cloudDist < 60.0 && (ci & 7) == 0) {
            float aoConeSample = coneAO(cPos, probeSph, ch);
            aoCone = aoHeight * 0.35 + aoConeSample * 0.65;
        }
        float aoDirect  = mix(0.30, 1.0, aoCone);
        float aoAmbient = mix(0.30, 1.0, aoCone);

        float sunElev = dot(probeSph, uLightDir);
        float skyPathKm = min(8.0 / max(sunElev, 0.02), 60.0);
        float skyVis = smoothstep(-0.5, 0.15, sunElev);
        float nightGlow = 0.012 * (1.0 - skyVis);
        vec3 skyAmbColor = exp(-gRayleighCoeff * skyPathKm) * skyVis + vec3(nightGlow);

        float atmLum = dot(atmTransAtCloud, vec3(0.299, 0.587, 0.114));
        vec3 directContrib  = atmLum * sunTint  * cloudAtt * aoDirect;
        // Ambient darkens where cloud is self-shadowed (matches cloud.frag logic)
        float shadowStrength = 1.0 - dot(cloudShadow, vec3(0.333));  // 0=lit, 1=shadowed
        vec3 ambientContrib = atmLum * skyAmbColor * 1.2 * aoAmbient;
                            * (1.0 - shadowStrength * 0.5);
        sumCloud += dC * (directContrib + ambientContrib);

        // ── Early termination ─────────────────────────────────────────────────
        // Stop marching when the cumulative transmittance is < 2% —
        // further samples would contribute negligibly to the pixel.
        float totalTrans = exp(-gCloudExtinction.x * optDepthC);
        if (totalTrans < 0.02) break;
        if (dot(cloudAtt, vec3(0.333)) < 0.006) break;
        tPos += fineStep;
    }

    // ── Terminator fade: dim clouds at night (matches cloud.frag nightFade) ──
    // uSunVisibility is declared in atmosphere_preamble.frag, available here.
    float nightFade = smoothstep(0.0, 0.12, uSunVisibility);
    sumCloud *= nightFade;
}
