#version 450
// Atmosphere fragment shader
// Full Rayleigh/Mie/Ozone scattering, 512-step fixed march.
// Cloud and depth-buffer sampling deferred (requires 3D textures and depth pass).

#define PI 3.14159265359
#define PRIMARY_STEPS 96

layout(set = 0, binding = 0) uniform FrameUBO {
    mat4  view;
    mat4  proj;
    vec3  lightDir;
    float _pad0;
    vec3  viewPos;
    float time;
} frame;

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
    float _pad;
} pc;

layout(location = 0) in vec3 vWorldPos;
layout(location = 0) out vec4 FragColor;

// ── Per-planet scattering coefficients (set by setupPlanetProfile) ──────────
vec3  gRayleighCoeff;
float gMieCoeff;
float gHRayleigh;
float gHMie;
float gGMie;
vec3  gOzoneCoeff;
float gHOzoneCenter;
float gHOzoneWidth;

// All spatial constants below are in render units (1 rU = 1000 km).
// Scale heights: physical km ÷ 1000.  Scattering coefficients: physical km⁻¹ × 1000.
void setupPlanetProfile() {
    int idx = pc.planetIdx;
    if (idx == 3) {         // Earth
        gRayleighCoeff = vec3(5.8, 13.5, 33.1) * 1.2;
        gMieCoeff = 5.0; gHRayleigh = 0.008; gHMie = 0.001; gGMie = 0.8;
        gOzoneCoeff = vec3(0.35, 0.85, 0.09); gHOzoneCenter = 0.025; gHOzoneWidth = 0.015;
    } else if (idx == 2) {  // Venus
        gRayleighCoeff = vec3(5.0, 12.0, 28.0);
        gMieCoeff = 40.0; gHRayleigh = 0.015; gHMie = 0.005; gGMie = 0.76;
        gOzoneCoeff = vec3(0.5, 5.0, 20.0); gHOzoneCenter = 0.05; gHOzoneWidth = 0.02;
    } else if (idx == 5) {  // Mars
        gRayleighCoeff = vec3(10.0, 5.0, 1.0);
        gMieCoeff = 15.0; gHRayleigh = 0.011; gHMie = 0.003; gGMie = 0.85;
        gOzoneCoeff = vec3(0.0); gHOzoneCenter = 0.001; gHOzoneWidth = 0.001;
    } else if (idx == 6) {  // Jupiter
        gRayleighCoeff = vec3(18.0, 15.0, 12.0);
        gMieCoeff = 10.0; gHRayleigh = 0.027; gHMie = 0.01; gGMie = 0.8;
        gOzoneCoeff = vec3(0.0); gHOzoneCenter = 0.001; gHOzoneWidth = 0.001;
    } else if (idx == 7) {  // Saturn
        gRayleighCoeff = vec3(15.0, 13.0, 9.0);
        gMieCoeff = 10.0; gHRayleigh = 0.06; gHMie = 0.015; gGMie = 0.8;
        gOzoneCoeff = vec3(0.0); gHOzoneCenter = 0.001; gHOzoneWidth = 0.001;
    } else if (idx == 8) {  // Uranus
        gRayleighCoeff = vec3(2.0, 15.0, 25.0);
        gMieCoeff = 2.0; gHRayleigh = 0.028; gHMie = 0.005; gGMie = 0.8;
        gOzoneCoeff = vec3(1.0, 0.0, 0.0); gHOzoneCenter = 0.02; gHOzoneWidth = 0.01;
    } else if (idx == 9) {  // Neptune
        gRayleighCoeff = vec3(1.0, 8.0, 35.0);
        gMieCoeff = 1.0; gHRayleigh = 0.02; gHMie = 0.005; gGMie = 0.8;
        gOzoneCoeff = vec3(1.0, 0.0, 0.0); gHOzoneCenter = 0.02; gHOzoneWidth = 0.01;
    } else {                // Default (fallback)
        gRayleighCoeff = vec3(5.8, 13.5, 33.1);
        gMieCoeff = 5.0; gHRayleigh = 0.008; gHMie = 0.001; gGMie = 0.8;
        gOzoneCoeff = vec3(0.0); gHOzoneCenter = 0.001; gHOzoneWidth = 0.001;
    }
}

bool intersectSphere(vec3 ro, vec3 rd, float radius, out float t0, out float t1) {
    vec3 L = ro - pc.planetCenter.xyz;
    float tca = -dot(L, rd);
    vec3 perp = L + tca * rd;
    float d2 = dot(perp, perp);
    float r2 = radius * radius;
    if (d2 > r2) return false;
    float thc = sqrt(r2 - d2);
    t0 = tca - thc; t1 = tca + thc;
    return true;
}

float rayleighPhase(float cosTheta) {
    return 3.0 / (16.0 * PI) * (1.0 + cosTheta * cosTheta);
}

float miePhase(float cosTheta) {
    float g2 = gGMie * gGMie;
    float num = 3.0 * (1.0 - g2) * (1.0 + cosTheta * cosTheta);
    float denom = 8.0 * PI * (2.0 + g2) * pow(1.0 + g2 - 2.0 * gGMie * cosTheta, 1.5);
    return num / max(denom, 1e-6);
}

float ozoneDensity(float h) {
    float d = (h - gHOzoneCenter) / gHOzoneWidth;
    return exp(-0.5 * d * d);
}

// Light-ray optical depth integration (20 uniform steps toward sun)
void getOpticalDepth(vec3 p, vec3 lightDir,
                     out float dR, out float dM, out float dO) {
    dR = 0.0; dM = 0.0; dO = 0.0;
    // Ground shadow
    float tS0, tS1;
    if (intersectSphere(p, lightDir, pc.innerRadius, tS0, tS1) && tS0 > 0.0) {
        dR = dM = dO = 1e6; return;
    }
    // Ring shadow (Saturn)
    if (pc.ringOuter > 0.0 && lightDir.y != 0.0) {
        float tR = -(p - pc.planetCenter.xyz).y / lightDir.y;
        if (tR > 0.0) {
            vec3 hit = (p - pc.planetCenter.xyz) + tR * lightDir;
            float d = length(hit.xz);
            if (d >= pc.ringInner && d <= pc.ringOuter) {
                float tau = 0.0;
                if      (d < 1.24) tau = 0.005;
                else if (d < 1.53) tau = 0.1;
                else if (d < 1.95) tau = 1.8;
                else if (d < 2.03) tau = 0.05;
                else if (d < 2.27) tau = 0.6;
                else if (d < 2.35) tau = 0.1;
                float block = (1.0 - exp(-tau / abs(lightDir.y))) * 8.0;
                dR += block; dM += block; dO += block;
            }
        }
    }
    float t0, t1;
    if (!intersectSphere(p, lightDir, pc.outerRadius, t0, t1) || t1 <= 0.0) return;
    float start = max(t0, 0.0);
    float stepSize = (t1 - start) / 20.0;
    for (int i = 0; i < 20; i++) {
        vec3 pos = p + lightDir * (start + (float(i) + 0.5) * stepSize);
        float h = length(pos - pc.planetCenter.xyz) - pc.surfaceRadius;
        if (h < -0.1) { dR = dM = dO = 1e6; return; }
        dR += exp(-h / gHRayleigh) * stepSize;
        dM += exp(-h / gHMie)      * stepSize;
        dO += ozoneDensity(h)       * stepSize;
    }
}

void main() {
    setupPlanetProfile();

    vec3 camPos  = frame.viewPos;
    vec3 rayDir  = normalize(vWorldPos - camPos);
    vec3 lightDir= normalize(frame.lightDir);

    float camDist = length(camPos - pc.planetCenter.xyz);
    float camAlt  = max(camDist - pc.surfaceRadius, 0.0);
    float atmoThickness = pc.outerRadius - pc.surfaceRadius;
    bool  cameraInside  = camDist < pc.outerRadius;

    float tNear, tFar;
    if (!intersectSphere(camPos, rayDir, pc.outerRadius, tNear, tFar)) discard;

    // Keep far hemisphere only (matches OpenGL GL_BACK cull on a CW-outside sphere).
    // Near hemisphere: vWorldPos is at the FIRST intersection → tFrag ≈ tNear (small).
    // Far  hemisphere: vWorldPos is at the SECOND intersection → tFrag ≈ tFar  (large).
    // From inside: tNear < 0, so midpoint < tFar/2 < tFrag → condition never fires, all kept.
    // Y-flip + VK_FRONT_FACE_CLOCKWISE inverts the cull sense, so hardware culling can't
    // replicate OpenGL GL_BACK here — the shader tFrag test is the reliable equivalent.
    {
        float tFrag = length(vWorldPos - camPos);
        if (tFrag < (tNear + tFar) * 0.5) discard;
    }

    tNear = max(tNear, 0.0);

    // Clip at planet surface (analytical sphere — no LOD banding)
    float tS0, tS1;
    if (intersectSphere(camPos, rayDir, pc.surfaceRadius, tS0, tS1) && tS0 > 0.0)
        tFar = min(tFar, tS0);

    if (tFar - tNear <= 0.0) discard;

    // Adaptive step: divides the FULL ray segment into PRIMARY_STEPS equal parts.
    // Fixed 2 km step capped coverage at ~1024 km, which is far too short for
    // atmosphere limb rays (~2870 km) — the dense lower layers were never reached,
    // making the limb appear transparent and the disc visually smaller than the planet.
    float STEP  = (tFar - tNear) / float(PRIMARY_STEPS);
    float ign    = fract(52.9829189 * fract(dot(gl_FragCoord.xy, vec2(0.06711056, 0.00583715))));
    float jitter = fract(ign + float(pc.frameIndex % 64) * 0.6180339887);
    float tCur   = tNear + jitter * STEP;

    vec3  sumRayleigh = vec3(0.0);
    vec3  sumMie      = vec3(0.0);
    float optDepthR = 0.0, optDepthM = 0.0, optDepthO = 0.0;
    float cosTheta  = dot(rayDir, lightDir);

    for (int i = 0; i < PRIMARY_STEPS; i++) {
        if (tCur >= tFar) break;
        float dt  = min(STEP, tFar - tCur);
        vec3  pos = camPos + rayDir * (tCur + dt * 0.5);
        float h   = max(length(pos - pc.planetCenter.xyz) - pc.surfaceRadius, 0.0);

        float dR = exp(-h / gHRayleigh) * dt;
        float dM = exp(-h / gHMie)      * dt;
        float dO = ozoneDensity(h)       * dt;
        optDepthR += dR; optDepthM += dM; optDepthO += dO;

        float lightR, lightM, lightO;
        getOpticalDepth(pos, lightDir, lightR, lightM, lightO);

        vec3 tau   = gRayleighCoeff * (optDepthR + lightR)
                   + gMieCoeff * 1.1 * (optDepthM + lightM)
                   + gOzoneCoeff * (optDepthO + lightO);
        vec3 atten = exp(-tau);

        sumRayleigh += dR * atten;
        sumMie      += dM * atten;
        tCur        += STEP;

        if (max(atten.x, max(atten.y, atten.z)) < 0.01) break;
    }

    float phaseR = rayleighPhase(cosTheta);
    float phaseM = miePhase(cosTheta);

    vec3 color = sumRayleigh * gRayleighCoeff * phaseR
               + sumMie      * gMieCoeff      * phaseM;

    // Altitude-adaptive exposure
    float altNorm    = clamp(camAlt / atmoThickness, 0.0, 1.0);
    float nightFactor = mix(0.01, 1.0, pc.sunVisibility);
    float exposure   = mix(10.0, 5.0, smoothstep(0.0, 0.6, altNorm)) * nightFactor;

    if (cameraInside) {
        vec3  localUp   = normalize(camPos - pc.planetCenter.xyz);
        float upDot     = dot(rayDir, localUp);
        float horizBoost = 1.0 + 0.25 * exp(-upDot * upDot * 8.0) * (1.0 - altNorm);
        exposure *= horizBoost;
    }

    color *= exposure;

    // Physical transmittance → opacity
    vec3  tau_view        = gRayleighCoeff * optDepthR + gMieCoeff * optDepthM + gOzoneCoeff * optDepthO;
    vec3  viewTransmittance = exp(-tau_view);
    float transLuma       = dot(viewTransmittance, vec3(0.299, 0.587, 0.114));
    float opacity         = clamp(1.0 - transLuma, 0.0, 1.0);

    if (pc.sunVisibility < 0.1)
        opacity *= smoothstep(0.0, 0.1, pc.sunVisibility + 0.05);
    if (cameraInside)
        opacity = clamp(opacity * mix(1.2, 1.0, smoothstep(0.0, 0.5, altNorm)), 0.0, 1.0);

    // ACES filmic tone mapping
    vec3 x = max(color, vec3(0.0));
    color = (x * (2.51 * x + 0.03)) / (x * (2.43 * x + 0.59) + 0.14);

    FragColor = vec4(color, opacity);
}
