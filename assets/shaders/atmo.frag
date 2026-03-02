#version 330 core
in vec3 vWorldPos;

uniform vec3 uCamPos;
uniform vec3 uLightDir;
uniform vec3 uPlanetCenter;
uniform float uInnerRadius;
uniform float uOuterRadius;

out vec4 FragColor;

#define PI 3.14159265359

const int PRIMARY_STEPS = 16;
const int LIGHT_STEPS = 6;

// === Cinematic Parameters (Meter Units) ===
const vec3 RAYLEIGH_COEFF = (vec3(0.0058, 0.0135, 0.0331) * 3.5) / 1000.0;
const float MIE_COEFF = (0.003 * 3.0) / 1000.0;
const float H_RAYLEIGH = 8000.0;  // 8 km (meters)
const float H_MIE = 1200.0;       // 1.2 km (meters)
const float G_MIE = 0.85;

bool intersectSphere(vec3 ro, vec3 rd, float radius, out float t0, out float t1) {
    vec3 L = ro - uPlanetCenter;
    float a = dot(rd, rd);
    float b = 2.0 * dot(rd, L);
    float c = dot(L, L) - radius * radius;
    float delta = b * b - 4.0 * a * c;
    if (delta < 0.0) return false;
    float sqrtDelta = sqrt(delta);
    t0 = (-b - sqrtDelta) / (2.0 * a);
    t1 = (-b + sqrtDelta) / (2.0 * a);
    return true;
}

float rayleighPhase(float cosTheta) {
    return 3.0 / (16.0 * PI) * (1.0 + cosTheta * cosTheta);
}

float miePhase(float cosTheta) {
    float g2 = G_MIE * G_MIE;
    return (1.0 - g2) / (4.0 * PI * pow(1.0 + g2 - 2.0 * G_MIE * cosTheta, 1.5));
}

void getOpticalDepth(vec3 p, vec3 lightDir, out float dRayleigh, out float dMie) {
    dRayleigh = 0.0; dMie = 0.0;

    float tS0, tS1;
    if (intersectSphere(p, lightDir, uInnerRadius, tS0, tS1) && tS0 > 0.0) {
        dRayleigh = 1e6; dMie = 1e6;
        return; 
    }

    float t0, t1;
    if (!intersectSphere(p, lightDir, uOuterRadius, t0, t1) || t1 <= 0.0) return;
    float stepSize = t1 / float(LIGHT_STEPS);
    for (int i = 0; i < LIGHT_STEPS; i++) {
        vec3 pos = p + lightDir * (float(i) + 0.5) * stepSize;
        float h = length(pos - uPlanetCenter) - uInnerRadius;
        dRayleigh += exp(-max(h, 0.0) / H_RAYLEIGH) * stepSize;
        dMie += exp(-max(h, 0.0) / H_MIE) * stepSize;
    }
}

void main() {
    vec3 rayDir = normalize(vWorldPos - uCamPos);
    float tNear, tFar;
    if (!intersectSphere(uCamPos, rayDir, uOuterRadius, tNear, tFar)) discard;

    float tSurf0, tSurf1;
    if (intersectSphere(uCamPos, rayDir, uInnerRadius, tSurf0, tSurf1)) {
        if (tSurf0 > 0.0) tFar = min(tFar, tSurf0);
    }

    tNear = max(tNear, 0.0);
    float segmentLength = tFar - tNear;
    if (segmentLength <= 0.0) discard;

    float stepSize = segmentLength / float(PRIMARY_STEPS);

    vec3 sumRayleigh = vec3(0.0);
    vec3 sumMie = vec3(0.0);
    float optDepthRayleigh = 0.0;
    float optDepthMie = 0.0;
    float cosTheta = dot(rayDir, uLightDir);

    for (int i = 0; i < PRIMARY_STEPS; i++) {
        vec3 pos = uCamPos + rayDir * (tNear + (float(i) + 0.5) * stepSize);
        float h = length(pos - uPlanetCenter) - uInnerRadius;
        if (h < 0.0) h = 0.0;

        float dR = exp(-h / H_RAYLEIGH) * stepSize;
        float dM = exp(-h / H_MIE) * stepSize;
        optDepthRayleigh += dR;
        optDepthMie += dM;

        float dRayleighLight, dMieLight;
        getOpticalDepth(pos, uLightDir, dRayleighLight, dMieLight);

        vec3 tau = RAYLEIGH_COEFF * (optDepthRayleigh + dRayleighLight) + MIE_COEFF * 1.1 * (optDepthMie + dMieLight);
        vec3 attenuation = exp(-tau);

        sumRayleigh += dR * attenuation;
        sumMie += dM * attenuation;
    }

    vec3 color = sumRayleigh * RAYLEIGH_COEFF * rayleighPhase(cosTheta) +
                 sumMie * MIE_COEFF * miePhase(cosTheta);

    color *= 18.0;

    // Physical Alpha Calculation
    vec3 tau_view = RAYLEIGH_COEFF * optDepthRayleigh + MIE_COEFF * optDepthMie;
    vec3 viewTransmittance = exp(-tau_view);
    float transLuma = dot(viewTransmittance, vec3(0.299, 0.587, 0.114));
    float opacity = clamp(1.0 - transLuma, 0.0, 1.0);
    float finalAlpha = opacity; 

    color = 1.0 - exp(-color); // Tone mapping
    FragColor = vec4(color, finalAlpha);
}