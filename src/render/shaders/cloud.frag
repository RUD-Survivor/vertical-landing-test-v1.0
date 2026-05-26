#version 450
// Volumetric cloud fragment shader — fullscreen ray march through cloud band.
// Procedural FBM density, Henyey-Greenstein phase, 6-step cone self-shadow.
// Reuses AtmoPushConstants layout (no additional descriptor needed).
// Only renders for Earth (planetIdx == 3) when showClouds >= 0.5.

#define PI 3.14159265359
#define CLOUD_STEPS 48
#define SHADOW_STEPS 6

layout(set = 0, binding = 0) uniform FrameUBO {
    mat4  view;
    mat4  proj;
    vec3  lightDir;
    float _pad0;
    vec3  viewPos;
    float time;
} frame;

// Same layout as AtmoPushConstants — no new descriptor set needed.
layout(push_constant) uniform PC {
    vec4  planetCenter;   // .xyz = center world pos
    float innerRadius;    // planet surface radius (km)
    float outerRadius;    // atmosphere outer radius
    float surfaceRadius;  // same as innerRadius
    int   planetIdx;
    float sunVisibility;
    float ringInner;
    float ringOuter;
    int   frameIndex;
    float tuneMinAlt;     // cloud base altitude (km)
    float tuneMaxAlt;     // cloud top  altitude (km)
    float tuneExtinction; // density/opacity scale
    float showClouds;     // 0 = off, 1 = on
} pc;

layout(location = 0) in  vec2 vNDC;
layout(location = 0) out vec4 FragColor;

// ── Noise ────────────────────────────────────────────────────────────────────
float hash3(vec3 p) {
    p  = fract(p * vec3(443.897, 441.423, 437.195));
    p += dot(p, p.yzx + 19.19);
    return fract((p.x + p.y) * p.z);
}

float vnoise(vec3 p) {
    vec3 i = floor(p), f = fract(p);
    f = f * f * (3.0 - 2.0 * f);
    return mix(mix(mix(hash3(i),              hash3(i+vec3(1,0,0)), f.x),
                   mix(hash3(i+vec3(0,1,0)),  hash3(i+vec3(1,1,0)), f.x), f.y),
               mix(mix(hash3(i+vec3(0,0,1)),  hash3(i+vec3(1,0,1)), f.x),
                   mix(hash3(i+vec3(0,1,1)),  hash3(i+vec3(1,1,1)), f.x), f.y), f.z);
}

float fbm4(vec3 p) {
    float v = 0.0, a = 0.5;
    for (int i = 0; i < 4; i++) { v += vnoise(p)*a; p = p*2.1 + vec3(1.7,-0.8,2.3); a *= 0.5; }
    return v;
}

float fbm2(vec3 p) {
    float v = 0.0, a = 0.5;
    for (int i = 0; i < 2; i++) { v += vnoise(p)*a; p = p*2.2 + vec3(-1.3,0.9,-2.1); a *= 0.5; }
    return v;
}

// ── Sphere intersection ──────────────────────────────────────────────────────
bool isectSphere(vec3 ro, vec3 rd, vec3 c, float r, out float t0, out float t1) {
    vec3  L = ro - c;
    float b = dot(L, rd), disc = b*b - dot(L,L) + r*r;
    if (disc < 0.0) return false;
    float sq = sqrt(disc); t0 = -b - sq; t1 = -b + sq; return true;
}

// ── Height gradient — cumulus profile ────────────────────────────────────────
float heightGrad(float hFrac) {
    return smoothstep(0.0, 0.08, hFrac) * smoothstep(1.0, 0.15, hFrac);
}

// ── Cloud density ─────────────────────────────────────────────────────────────
float cloudDensity(vec3 pos) {
    vec3  ctr   = pc.planetCenter.xyz;
    float dist  = length(pos - ctr);
    float alt   = dist - pc.surfaceRadius;
    float bandH = max(pc.tuneMaxAlt - pc.tuneMinAlt, 1.0);
    float hFrac = clamp((alt - pc.tuneMinAlt) / bandH, 0.0, 1.0);
    if (hFrac <= 0.0 || hFrac >= 1.0) return 0.0;

    float hg      = heightGrad(hFrac);
    vec3  normPos = (pos - ctr) / dist;

    // Weather advection (~50 km/s animation gives visible motion)
    float t    = frame.time * 0.010;
    vec3  drft = vec3(t, 0.0, t * 0.4);

    // Coverage: planetary-scale pattern (feature size ~400 km → scale ≈ 4.0 on unit sphere)
    float cov  = fbm4(normPos * 4.5 + drft * 0.08);
    // Suppress clouds near poles
    cov *= smoothstep(0.90, 0.55, abs(normPos.y));
    cov  = smoothstep(0.38, 0.70, cov);
    if (cov < 0.01) return 0.0;

    // Shape: cloud-scale FBM in world-km space (1 unit ≈ 33 km → 0.030)
    float shape = fbm4(pos * 0.030 + drft);
    shape = smoothstep(0.32, 0.72, shape);

    // Fine erosion (~8 km features)
    float erode = fbm2(pos * 0.12 + drft * 1.6) * 0.28;

    return max(0.0, cov * shape - erode) * hg * pc.tuneExtinction;
}

// ── Cone shadow march toward sun ─────────────────────────────────────────────
float cloudShadow(vec3 pos, vec3 ldir) {
    float bandH   = max(pc.tuneMaxAlt - pc.tuneMinAlt, 1.0);
    float stepLen = bandH / float(SHADOW_STEPS);
    float shadow  = 0.0;
    for (int i = 0; i < SHADOW_STEPS; i++) {
        shadow += cloudDensity(pos + ldir * (float(i) + 0.5) * stepLen);
    }
    return exp(-shadow * stepLen * 2.0);
}

// ── Henyey-Greenstein phase + two-lobe cloud phase ───────────────────────────
float HG(float ct, float g) {
    float g2 = g * g;
    return (1.0 - g2) / (4.0 * PI * pow(max(1.0 + g2 - 2.0*g*ct, 1e-5), 1.5));
}
float cloudPhase(float ct) {
    // Forward scatter g=0.75 (pillar/silver-lining) + back g=-0.15 (dark core)
    return mix(HG(ct, 0.75), HG(ct, -0.15), 0.25) * 4.0 * PI;
}

// ── Main ──────────────────────────────────────────────────────────────────────
void main() {
    if (pc.showClouds < 0.5 || pc.planetIdx != 3) discard;

    // Reconstruct world-space ray direction from Vulkan NDC (same as atmo.frag)
    vec3 viewDir = vec3(
         vNDC.x / frame.proj[0][0],
        -vNDC.y / frame.proj[1][1],
        -1.0
    );
    vec3 rayDir = normalize(transpose(mat3(frame.view)) * viewDir);
    vec3 camPos = frame.viewPos;

    vec3  ctr      = pc.planetCenter.xyz;
    float cloudMinR = pc.surfaceRadius + pc.tuneMinAlt;
    float cloudMaxR = pc.surfaceRadius + pc.tuneMaxAlt;

    float t0i, t1i, t0o, t1o;
    bool  hitInner = isectSphere(camPos, rayDir, ctr, cloudMinR, t0i, t1i);
    bool  hitOuter = isectSphere(camPos, rayDir, ctr, cloudMaxR, t0o, t1o);

    if (!hitOuter || t1o <= 0.0) discard;

    float camDist     = length(camPos - ctr);
    bool  inOuter     = camDist < cloudMaxR;
    bool  inInner     = camDist < cloudMinR;

    float tStart, tEnd;
    if (inInner) {
        // Camera at/below surface — march from cloud base outward
        tStart = max(t1i, 0.001);
        tEnd   = t1o;
    } else if (inOuter) {
        // Camera inside cloud layer — start immediately
        tStart = 0.001;
        tEnd   = t1o;
        if (hitInner && t0i > 0.0) tEnd = min(tEnd, t0i);
    } else {
        // Camera outside atmosphere — enter at outer sphere
        tStart = max(t0o, 0.0);
        tEnd   = t1o;
        if (hitInner && t0i > 0.0) tEnd = min(tEnd, t0i);
    }

    if (tEnd - tStart < 0.1) discard;

    float stepSize = (tEnd - tStart) / float(CLOUD_STEPS);

    // IGN dither jitter for TAA compatibility
    float ign    = fract(52.9829189 * fract(dot(gl_FragCoord.xy, vec2(0.06711056, 0.00583715))));
    float jitter = fract(ign + float(pc.frameIndex % 64) * 0.6180339887);
    float tCur   = tStart + jitter * stepSize;

    vec3  lightDir = normalize(frame.lightDir);
    float cosTheta = dot(rayDir, lightDir);
    float phase    = cloudPhase(cosTheta);

    // Sun and ambient colors
    vec3 sunCol = vec3(1.00, 0.97, 0.90) * clamp(pc.sunVisibility, 0.0, 1.0);
    vec3 ambCol = vec3(0.38, 0.55, 0.85) * 0.25 * clamp(pc.sunVisibility + 0.15, 0.0, 1.0);

    vec3  cloudColor    = vec3(0.0);
    float transmittance = 1.0;

    for (int i = 0; i < CLOUD_STEPS; i++) {
        if (tCur >= tEnd || transmittance < 0.005) break;

        float dt      = min(stepSize, tEnd - tCur);
        vec3  pos     = camPos + rayDir * (tCur + dt * 0.5);
        float density = cloudDensity(pos);

        if (density > 0.0005) {
            float dtau   = density * dt;
            float extinc = exp(-dtau);

            // Self-shadowing + phase
            float shadow = cloudShadow(pos, lightDir);
            // Powder darkens thick cloud interiors (fake multiple-scattering)
            float powder = 1.0 - exp(-dtau * 2.5);

            vec3 Li = sunCol * shadow * phase * powder
                    + ambCol * (1.0 - shadow * 0.6);

            cloudColor    += Li * (1.0 - extinc) * transmittance;
            transmittance *= extinc;
        }

        tCur += stepSize;
    }

    float alpha = clamp(1.0 - transmittance, 0.0, 1.0);

    // Night fade: dim but keep silhouette visible
    float nightFade = smoothstep(0.0, 0.12, pc.sunVisibility);
    cloudColor     *= nightFade;
    alpha          *= mix(0.08, 1.0, nightFade);

    if (alpha < 0.003) discard;

    // Pre-multiplied alpha (matches VkAtmoPipeline blend mode: src=ONE dst=1-SrcA)
    FragColor = vec4(cloudColor * alpha, alpha);
}
