#version 450
// Saturn ring shader — physically based scattering + planet shadow.
// pc.ambientStr = planet radius in render units (for shadow intersection & ring radius normalization).
// Ring uses alpha blending; depth write disabled.

layout(set = 0, binding = 0) uniform FrameUBO {
    mat4  view;
    mat4  proj;
    vec3  lightDir;
    float _pad0;
    vec3  viewPos;
    float time;
} frame;

layout(push_constant) uniform PC {
    mat4  model;
    vec4  baseColor;
    vec4  planetCenter;
    float ambientStr;   // planet radius in render units
    int   hasTexture;
} pc;

layout(location = 0) in vec3 vWorldPos;
layout(location = 1) in vec3 vNormal;
layout(location = 2) in vec2 vUV;
layout(location = 3) in vec4 vColor;

layout(location = 0) out vec4 FragColor;

float henyeyGreenstein(float cosTheta, float g) {
    float g2 = g * g;
    return (1.0 - g2) / (4.0 * 3.14159 * pow(1.0 + g2 - 2.0 * g * cosTheta, 1.5));
}

float getRingOpticalDepth(float r) {
    if (r < 1.11) return 0.0;
    if (r < 1.24) return 0.005;
    if (r < 1.53) return mix(0.05, 0.15, smoothstep(1.24, 1.53, r));
    if (r < 1.95) return mix(1.2, 2.5, sin(r * 40.0) * 0.2 + 0.8);
    if (r < 2.03) return 0.02;
    if (r < 2.27) {
        float base  = 0.5;
        float encke = smoothstep(2.21, 2.22, r) * smoothstep(2.23, 2.22, r);
        return mix(base, 0.01, encke);
    }
    if (r < 2.31) return 0.0;
    if (r < 2.34) return 0.1;
    return 0.0;
}

void main() {
    vec3 N = normalize(vNormal);
    vec3 L = normalize(frame.lightDir);
    vec3 V = normalize(frame.viewPos - vWorldPos);
    float cosTheta = dot(V, -L);
    float NdotV = abs(dot(N, V));

    float phaseForward = henyeyGreenstein(cosTheta, 0.85);
    float phaseBack    = henyeyGreenstein(cosTheta, -0.3);
    float phase        = mix(phaseForward, phaseBack, 0.4);

    // Planet shadow: ray from ring point in direction L intersects planet sphere
    float pr = pc.ambientStr; // planet radius in render units
    vec3 localPos = vWorldPos - pc.planetCenter.xyz;
    float b = 2.0 * dot(localPos, L);
    float c = dot(localPos, localPos) - pr * pr;
    float disc = b * b - 4.0 * c;
    float shadow = 1.0;
    if (disc > 0.0) {
        float t1 = (-b + sqrt(disc)) / 2.0;
        if (t1 > 0.0) shadow = 0.0;
    }

    // Normalized ring radius
    float r = length(localPos.xz) / pr;
    float tau = getRingOpticalDepth(r);

    float transmittance = exp(-tau / max(NdotV, 0.001));
    float alpha = 1.0 - transmittance;

    float fineS = sin(r * 300.0) * 0.1 + sin(r * 800.0) * 0.05;
    alpha *= (1.0 + fineS);
    alpha = clamp(alpha, 0.0, 0.95);

    vec3 iceWhite = vec3(0.92, 0.90, 0.88);
    vec3 albedo = mix(vec3(0.8, 0.7, 0.5), iceWhite, smoothstep(0.1, 1.0, tau));

    vec3 result = albedo * (0.05 + 1.5 * phase * shadow);
    if (cosTheta > 0.95) {
        result += iceWhite * pow(cosTheta, 64.0) * 4.0 * shadow;
    }

    FragColor = vec4(result, alpha);
}
