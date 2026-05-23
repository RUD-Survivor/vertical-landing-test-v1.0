#version 450
// Terrain fragment shader — Vulkan port, logic identical to OpenGL version

// Textures (Set 1, same as vertex shader)
layout(set = 1, binding = 1) uniform sampler2D uHydroMap;
layout(set = 1, binding = 3) uniform sampler2D uLocalHydroMap;
layout(set = 1, binding = 0) uniform sampler2D uTectonicMap;
layout(set = 1, binding = 2) uniform sampler2D uClimateMap;

// FrameUBO (Set 0, binding 0)
layout(set = 0, binding = 0) uniform FrameUBO {
    mat4  view;
    mat4  proj;
    vec3  lightDir;
    float _pad0;
    vec3  viewPos;
    float time;
} frame;

// Push constants
layout(push_constant) uniform PC {
    mat4  model;
    float planetRadius;
    float maxElevation;
    int   nodeLevel;
    int   hasLocalHydro;
} pc;

layout(location = 0) in vec3 vRelViewPos;
layout(location = 1) in vec3 vNormal;
layout(location = 2) in vec2 vUV;
layout(location = 3) in vec2 vLocalUV;
layout(location = 4) in vec3 vLocalPos;
layout(location = 5) in float vElevation;
layout(location = 6) in float vWaterDepth;
layout(location = 7) in float vSlope;

layout(location = 0) out vec4 FragColor;

// --- BICUBIC FILTERING ---
float w0(float a) { return (1.0/6.0)*(a*(a*(-a + 3.0) - 3.0) + 1.0); }
float w1(float a) { return (1.0/6.0)*(a*a*(3.0*a - 6.0) + 4.0); }
float w2(float a) { return (1.0/6.0)*(a*a*(-3.0*a + 3.0) + 3.0*a + 1.0); }
float w3(float a) { return (1.0/6.0)*(a*a*a); }
float g0(float a) { return w0(a) + w1(a); }
float g1(float a) { return w2(a) + w3(a); }
float h0(float a) { return -1.0 + w1(a) / (w0(a) + w1(a)); }
float h1(float a) { return 1.0 + w3(a) / (w2(a) + w3(a)); }

vec4 textureBicubic(sampler2D tex, vec2 uv) {
    vec2 res = vec2(textureSize(tex, 0));
    vec2 p = uv * res - 0.5;
    vec2 f = fract(p); p -= f;
    float x_h0 = h0(f.x), x_h1 = h1(f.x);
    float y_h0 = h0(f.y), y_h1 = h1(f.y);
    float x_g0 = g0(f.x), x_g1 = g1(f.x);
    float y_g0 = g0(f.y), y_g1 = g1(f.y);
    return (texture(tex, (p + vec2(x_h0, y_h0) + 0.5) / res) * x_g0 * y_g0 +
            texture(tex, (p + vec2(x_h1, y_h0) + 0.5) / res) * x_g1 * y_g0 +
            texture(tex, (p + vec2(x_h0, y_h1) + 0.5) / res) * x_g0 * y_g1 +
            texture(tex, (p + vec2(x_h1, y_h1) + 0.5) / res) * x_g1 * y_g1);
}

vec4 sampleHydroSmooth(vec2 uv) {
    return textureBicubic(uHydroMap, uv);
}

float hash(vec3 p) {
    p = fract(p * vec3(443.897, 441.423, 437.195));
    p += dot(p, p.yzx + 19.19);
    return fract((p.x + p.y) * p.z);
}

float noise3d(vec3 p) {
    vec3 i = floor(p); vec3 f = fract(p);
    f = f * f * f * (f * (f * 6.0 - 15.0) + 10.0);
    return mix(mix(mix(hash(i), hash(i+vec3(1,0,0)), f.x), 
                   mix(hash(i+vec3(0,1,0)), hash(i+vec3(1,1,0)), f.x), f.y),
               mix(mix(hash(i+vec3(0,0,1)), hash(i+vec3(1,0,1)), f.x), 
                   mix(hash(i+vec3(0,1,1)), hash(i+vec3(1,1,1)), f.x), f.y), f.z);
}

float getMicroHeight(vec3 p, float distFade) {
    float h = noise3d(p * 800.0) * 0.5;
    h += noise3d(p * 8000.0) * 0.15 * clamp(1.0 - distFade * 0.2, 0.0, 1.0);
    return h;
}

float warpedNoise(vec3 p) {
    vec3 q = vec3(noise3d(p + vec3(0.0, 0.0, 0.0)),
                  noise3d(p + vec3(5.2, 1.3, 0.7)),
                  noise3d(p + vec3(2.7, 8.4, 3.1)));
    return noise3d(p + q * 0.5);
}

void main() {
    vec3 L = normalize(frame.lightDir);
    vec3 V = normalize(-vRelViewPos);
    float distToCam = length(vRelViewPos);
    float detailFade = distToCam / 1000.0;

    vec3 fV_normPos = normalize(vLocalPos);
    float fV_phi = acos(clamp(fV_normPos.y, -1.0, 1.0));
    float fV_theta = atan(fV_normPos.z, fV_normPos.x);
    vec2 fV_geoUV = vec2(fV_theta / (2.0 * 3.14159) + 0.5, fV_phi / 3.14159);

    float hRefined = vElevation;
    float mMask = smoothstep(0.55, 0.75, hRefined);

    vec3 N = normalize(vNormal);
    if (vWaterDepth < 0.0001 && distToCam < 8.0) {
        float nScale = 1500.0 * (1.0 + mMask * 2.0);
        float nAmp = 0.05 * (1.0 - smoothstep(1.0, 8.0, distToCam));
        float d10 = noise3d(vLocalPos * 3000000.0) * 0.4;
        vec3 pMic = vLocalPos * nScale;
        float h0 = noise3d(pMic) + d10;
        float h1 = noise3d(pMic + vec3(0.005, 0, 0)) + d10;
        float h2 = noise3d(pMic + vec3(0, 0.005, 0)) + d10;
        vec3 microN = normalize(vec3(h0 - h1, h0 - h2, 1.0));
        vec3 TN = normalize(cross(N, vec3(0,1,0)));
        if (abs(N.y) > 0.99) TN = normalize(cross(N, vec3(0,0,1)));
        vec3 BN = cross(N, TN);
        N = normalize(N + (TN * microN.x + BN * microN.y) * nAmp * 2.0);
    }

    vec3 deepWater = vec3(0.01, 0.06, 0.15);
    vec3 shallowWater = vec3(0.05, 0.35, 0.45);
    vec3 beach = vec3(0.72, 0.68, 0.52);
    vec3 lowland = vec3(0.12, 0.32, 0.10);
    vec3 forest = vec3(0.06, 0.22, 0.05);
    vec3 mountainColor = vec3(0.42, 0.38, 0.35);
    vec3 snow = vec3(0.92, 0.94, 1.0);
    vec3 concrete = vec3(0.45, 0.45, 0.48);

    vec3 surfColor;
    float sLevel = 0.45;

    if (hRefined < sLevel || vWaterDepth > 0.000001) {
        float depth = (hRefined < sLevel) ? (sLevel - hRefined) : vWaterDepth * 2.0;
        surfColor = mix(shallowWater, deepWater, smoothstep(0.0, 0.15, depth));
    } else {
        float landH = (hRefined - sLevel) / (1.0 - sLevel);
        float albedoVar = noise3d(vLocalPos * 120.0) * 0.1 + 0.95;
        vec3 cMid = mix(beach, lowland, smoothstep(0.0, 0.1, landH));
        cMid = mix(cMid, forest, smoothstep(0.1, 0.45, landH));
        cMid = mix(cMid, mountainColor, smoothstep(0.45, 0.8, landH));
        float snowMask = smoothstep(0.8, 0.95, landH);
        snowMask *= (1.0 - smoothstep(0.08, 0.22, vSlope));
        surfColor = mix(cMid, snow, snowMask) * albedoVar;
        vec3 kscPos = vec3(0.1436, 0.478, 0.866);
        float distToKSC = length(fV_normPos - kscPos);
        float concreteMask = smoothstep(0.015, 0.008, distToKSC);
        surfColor = mix(surfColor, concrete, concreteMask);
    }

    float diff = max(dot(N, L), 0.0);
    float ambient = 0.18;
    vec3 result = surfColor * (ambient + diff * (1.1 - ambient));

    // uViewMode disabled in Vulkan path (no climate overlay yet)

    // --- River Overlay (Segment-based Distance Field) ---
    {
        vec2 res = vec2(512.0, 256.0);
        vec2 pixelPos = fV_geoUV * res;
        vec2 cellCoords = floor(pixelPos);
        vec2 localPos = fract(pixelPos);

        float minDist = 1.0;
        float maxStrahler = 0.0;

        const vec2 deltas[8] = vec2[](
            vec2(1,0), vec2(1,1), vec2(0,1), vec2(-1,1),
            vec2(-1,0), vec2(-1,-1), vec2(0,-1), vec2(1,-1)
        );

        for (int dy = -1; dy <= 1; dy++) {
            for (int dx = -1; dx <= 1; dx++) {
                vec2 nCoord = cellCoords + vec2(dx, dy);
                vec2 nUV = (nCoord + 0.5) / res;
                nUV.x = fract(nUV.x);
                vec4 nData = texture(uHydroMap, nUV);

                if (nData.b >= 4.0) {
                    int dir = int(nData.a);
                    if (dir >= 0 && dir < 8) {
                        vec2 p1 = nCoord + 0.5;
                        vec2 p2 = nCoord + 0.5 + deltas[dir];
                        vec2 pa = pixelPos - p1, ba = p2 - p1;
                        float h = clamp(dot(pa, ba) / dot(ba, ba), 0.0, 1.0);
                        float d = length(pa - ba * h);
                        if (d < minDist) {
                            minDist = d;
                            maxStrahler = nData.b;
                        }
                    }
                }
            }
        }

        if (minDist < 2.0) {
            float simPixelScale = length(fwidth(pixelPos));
            float screenRadius = minDist / max(simPixelScale, 0.0001);
            float targetWidth = (maxStrahler >= 5.0) ? 1.5 : 0.8;
            if (screenRadius < targetWidth) {
                float edge = smoothstep(targetWidth, targetWidth - 0.5, screenRadius);
                result = mix(result, vec3(0.01, 0.08, 0.25), 0.7 * edge);
            }
        }
    }

    FragColor = vec4(result, 1.0);
}
