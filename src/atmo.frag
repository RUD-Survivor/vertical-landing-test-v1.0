#version 330 core
in vec3 vWorldPos;

uniform vec3 uCamPos;
uniform vec3 uLightDir;
uniform vec3 uPlanetCenter;
uniform float uInnerRadius;
uniform float uOuterRadius;

out vec4 FragColor;

#define PI 3.14159265359

// Constants
const int PRIMARY_STEPS = 12; // 10-16
const int LIGHT_STEPS = 4;

// Coefficients (km^-1, world units = km)
// Scale heights enlarged from physical (8/1.2) to artistic (40/15) because
// the atmosphere shell is ~255km thick (4% of R). With physical H=8km,
// density at 100km = exp(-12.5) ~ 0, making the scattering invisible.
const vec3 RAYLEIGH_COEFF = vec3(5.5e-3, 13.0e-3, 22.4e-3);
const float MIE_COEFF = 21e-3;
const float H_RAYLEIGH = 40.0;
const float H_MIE = 15.0;
const float G_MIE = 0.758;    // Henyey-Greenstein asym factor

// Ray-Sphere intersection
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

// Rayleigh phase function
float rayleighPhase(float cosTheta) {
    return 3.0 / (16.0 * PI) * (1.0 + cosTheta * cosTheta);
}

// Mie phase function (Henyey-Greenstein)
float miePhase(float cosTheta) {
    float g = G_MIE;
    float g2 = g * g;
    return (1.0 - g2) / (4.0 * PI * pow(1.0 + g2 - 2.0 * g * cosTheta, 1.5));
}

// Optical depth
void getOpticalDepth(vec3 p, vec3 lightDir, out float dRayleigh, out float dMie) {
    dRayleigh = 0.0;
    dMie = 0.0;
    float t0, t1;
    if (!intersectSphere(p, lightDir, uOuterRadius, t0, t1) || t1 <= 0.0) return;
    
    float stepSize = t1 / float(LIGHT_STEPS);
    for (int i = 0; i < LIGHT_STEPS; i++) {
        vec3 pos = p + lightDir * (float(i) + 0.5) * stepSize;
        float h = length(pos - uPlanetCenter) - uInnerRadius;
        if (h < 0.0) { // Occluded by planet
            dRayleigh = 1e20;
            dMie = 1e20;
            return;
        }
        dRayleigh += exp(-h / H_RAYLEIGH) * stepSize;
        dMie += exp(-h / H_MIE) * stepSize;
    }
}

void main() {
    vec3 rayDir = normalize(vWorldPos - uCamPos);
    float tNear, tFar;
    
    if (!intersectSphere(uCamPos, rayDir, uOuterRadius, tNear, tFar)) discard;
    
    // Clamp to planet surface if needed
    float tSurf0, tSurf1;
    if (intersectSphere(uCamPos, rayDir, uInnerRadius, tSurf0, tSurf1)) {
        if (tSurf0 > 0.0) tFar = min(tFar, tSurf0);
    }
    
    tNear = max(tNear, 0.0);
    float segmentLength = tFar - tNear;
    float stepSize = segmentLength / float(PRIMARY_STEPS);
    
    vec3 sumRayleigh = vec3(0.0);
    vec3 sumMie = vec3(0.0);
    float optDepthRayleigh = 0.0;
    float optDepthMie = 0.0;
    
    float cosTheta = dot(rayDir, uLightDir);
    
    for (int i = 0; i < PRIMARY_STEPS; i++) {
        vec3 pos = uCamPos + rayDir * (tNear + (float(i) + 0.5) * stepSize);
        float h = length(pos - uPlanetCenter) - uInnerRadius;
        
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
    
    color *= 2.0; // High intensity for cinematic look

    FragColor = vec4(color, 1.0);
}
