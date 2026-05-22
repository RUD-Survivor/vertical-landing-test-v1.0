
#version 330 core
in vec3 vWorldPos;

uniform vec3 uCamPos;       // 摄像机位置
uniform vec3 uLightDir;     // 太阳光方向
uniform vec3 uPlanetCenter; // 星球中心点
uniform float uInnerRadius; // 星球表面半径 (km)
uniform float uOuterRadius; // 大气层外缘半径 (km)
uniform float uSurfaceRadius;
uniform float uTime;
uniform int uPlanetIdx;     // 星球索引（用于加载不同的物理参数）

uniform float uSunVisibility;
uniform float uRingInner;
uniform float uRingOuter;
uniform sampler2D uDepthTex;
uniform vec2 uResolution;
uniform mat4 uInvProj;
uniform float uTuneMinAlt;
uniform float uTuneMaxAlt;
uniform float uTuneExtinction;

out vec4 FragColor;

#define PI 3.14159265359

const int PRIMARY_STEPS = 512; // covers 1024 km at 2 km/step (handles horizon rays from low altitude)

// --- Dynamic Atmosphere Parameters ---
vec3 gRayleighCoeff;
float gMieCoeff;
float gHRayleigh;
float gHMie;
float gGMie;
vec3 gOzoneCoeff;
float gHOzoneCenter;
float gHOzoneWidth;

float gCloudMinAlt;
float gCloudMaxAlt;
vec3 gCloudExtinction;
vec3 gCloudScattering;

// 设置星球的大气物理属性
void setupPlanetProfile() {
    if (uPlanetIdx == 3) {
        // --- 地球 (Earth) ---
        // 瑞利散射系数 (RGB)：蓝色分量最大，所以天空是蓝色的。
        gRayleighCoeff = vec3(0.0058, 0.0135, 0.0331) * 1.2;
        // 米氏散射系数 (尘埃/气溶胶)
        gMieCoeff = 0.005; gHRayleigh = 8.0; gHMie = 1.0; gGMie = 0.8;
        // 臭氧吸收系数 (吸收特定红色波段，增强蓝天的饱和度)
        gOzoneCoeff = vec3(0.00035, 0.00085, 0.00009); gHOzoneCenter = 25.0; gHOzoneWidth = 15.0;
        gCloudMinAlt = uTuneMinAlt; gCloudMaxAlt = uTuneMaxAlt;
        gCloudExtinction = vec3(uTuneExtinction); gCloudScattering = vec3(0.70);
    } else if (uPlanetIdx == 2) {
        // --- 金星 (Venus) ---
        // 极厚的大气层和硫酸云，天空呈现土黄色。
        gRayleighCoeff = vec3(0.005, 0.012, 0.028);
        gMieCoeff = 0.04; gHRayleigh = 15.0; gHMie = 5.0; gGMie = 0.76;
        gOzoneCoeff = vec3(0.0005, 0.005, 0.02); gHOzoneCenter = 50.0; gHOzoneWidth = 20.0;
        gCloudMinAlt = 45.0; gCloudMaxAlt = 65.0;
        gCloudExtinction = vec3(0.08); gCloudScattering = vec3(0.15);
    } else if (uPlanetIdx == 5) {
        // --- 火星 (Mars) ---
        // 稀薄的大气，红色尘埃导致傍晚的天空变蓝，白天是红色。
        gRayleighCoeff = vec3(0.01, 0.005, 0.001);
        gMieCoeff = 0.015; gHRayleigh = 11.0; gHMie = 3.0; gGMie = 0.85;
        gOzoneCoeff = vec3(0.0); gHOzoneCenter = 1.0; gHOzoneWidth = 1.0;
        gCloudMinAlt = 100.0; gCloudMaxAlt = 101.0;
        gCloudExtinction = vec3(0.01); gCloudScattering = vec3(0.01);
    } else if (uPlanetIdx == 6) {
        // JUPITER
        gRayleighCoeff = vec3(0.018, 0.015, 0.012);
        gMieCoeff = 0.01; gHRayleigh = 27.0; gHMie = 10.0; gGMie = 0.8;
        gOzoneCoeff = vec3(0.0); gHOzoneCenter = 1.0; gHOzoneWidth = 1.0;
        gCloudMinAlt = 10.0; gCloudMaxAlt = 50.0;
        gCloudExtinction = vec3(0.1); gCloudScattering = vec3(0.6);
    } else if (uPlanetIdx == 7) {
        // SATURN
        gRayleighCoeff = vec3(0.015, 0.013, 0.009);
        gMieCoeff = 0.01; gHRayleigh = 60.0; gHMie = 15.0; gGMie = 0.8;
        gOzoneCoeff = vec3(0.0); gHOzoneCenter = 1.0; gHOzoneWidth = 1.0;
        gCloudMinAlt = 10.0; gCloudMaxAlt = 50.0;
        gCloudExtinction = vec3(0.1); gCloudScattering = vec3(0.6);
    } else if (uPlanetIdx == 8) {
        // URANUS
        gRayleighCoeff = vec3(0.002, 0.015, 0.025);
        gMieCoeff = 0.002; gHRayleigh = 28.0; gHMie = 5.0; gGMie = 0.8;
        gOzoneCoeff = vec3(0.001, 0.0, 0.0); gHOzoneCenter = 20.0; gHOzoneWidth = 10.0;
        gCloudMinAlt = 10.0; gCloudMaxAlt = 50.0;
        gCloudExtinction = vec3(0.1); gCloudScattering = vec3(0.6);
    } else if (uPlanetIdx == 9) {
        // NEPTUNE
        gRayleighCoeff = vec3(0.001, 0.008, 0.035);
        gMieCoeff = 0.001; gHRayleigh = 20.0; gHMie = 5.0; gGMie = 0.8;
        gOzoneCoeff = vec3(0.001, 0.0, 0.0); gHOzoneCenter = 20.0; gHOzoneWidth = 10.0;
        gCloudMinAlt = 10.0; gCloudMaxAlt = 50.0;
        gCloudExtinction = vec3(0.1); gCloudScattering = vec3(0.6);
    } else {
        // DEFAULT (fallback)
        gRayleighCoeff = vec3(0.0058, 0.0135, 0.0331);
        gMieCoeff = 0.005; gHRayleigh = 8.0; gHMie = 1.0; gGMie = 0.8;
        gOzoneCoeff = vec3(0.0); gHOzoneCenter = 1.0; gHOzoneWidth = 1.0;
        gCloudMinAlt = 100.0; gCloudMaxAlt = 101.0;
        gCloudExtinction = vec3(0.0); gCloudScattering = vec3(0.0);
    }
}

bool intersectSphere(vec3 ro, vec3 rd, float radius, out float t0, out float t1) {
    // Stable geometric intersection to prevent precision loss and catastrophic cancellation
    // (which causes concentric rings and missing atmosphere at >10,000 km)
    vec3 L = ro - uPlanetCenter;
    float tca = -dot(L, rd);
    vec3 perp = L + tca * rd;
    float d2 = dot(perp, perp);
    float radius2 = radius * radius;
    if (d2 > radius2) return false;
    float thc = sqrt(radius2 - d2);
    t0 = tca - thc;
    t1 = tca + thc;
    return true;
}

float rayleighPhase(float cosTheta) {
    return 3.0 / (16.0 * PI) * (1.0 + cosTheta * cosTheta);
}

float miePhase(float cosTheta) {
    // Cornette-Shanks phase function (improved Henyey-Greenstein)
    float g2 = gGMie * gGMie;
    float num = 3.0 * (1.0 - g2) * (1.0 + cosTheta * cosTheta);
    float denom = 8.0 * PI * (2.0 + g2) * pow(1.0 + g2 - 2.0 * gGMie * cosTheta, 1.5);
    return num / max(denom, 1e-6);
}

// Ozone density at altitude h (Gaussian profile around 25km)
float ozoneDensity(float h) {
    float d = (h - gHOzoneCenter) / gHOzoneWidth;
    return exp(-0.5 * d * d);
}


