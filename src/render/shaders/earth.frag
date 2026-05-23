#version 450

// Set 0, Binding 0: per-frame data
layout(set = 0, binding = 0) uniform FrameUBO {
    mat4  view;
    mat4  proj;
    vec3  lightDir;
    float _pad0;
    vec3  viewPos;
    float time;
} frame;

// Push constants (PlanetPushConstants — 104 bytes, std430 无 padding)
// 字段顺序与 C++ struct 对齐：vec4 planetCenter 必须在 float/int 之前
layout(push_constant) uniform PC {
    mat4  model;
    vec4  baseColor;
    vec4  planetCenter;  // .xyz = world-space planet center (offset 80)
    float ambientStr;    // offset 96
    int   hasTexture;    // offset 100
} pc;

layout(location = 0) in vec3 vWorldPos;
layout(location = 1) in vec3 vNormal;
layout(location = 2) in vec2 vUV;
layout(location = 3) in vec4 vColor;

layout(location = 0) out vec4 FragColor;

// ---- 噪声原语 ----
float hash(vec3 p) {
    p = fract(p * vec3(443.897, 441.423, 437.195));
    p += dot(p, p.yzx + 19.19);
    return fract((p.x + p.y) * p.z);
}

float noise3d(vec3 p) {
    vec3 i = floor(p);
    vec3 f = fract(p);
    f = f * f * f * (f * (f * 6.0 - 15.0) + 10.0);
    float n000 = hash(i);              float n100 = hash(i + vec3(1,0,0));
    float n010 = hash(i + vec3(0,1,0)); float n110 = hash(i + vec3(1,1,0));
    float n001 = hash(i + vec3(0,0,1)); float n101 = hash(i + vec3(1,0,1));
    float n011 = hash(i + vec3(0,1,1)); float n111 = hash(i + vec3(1,1,1));
    float nx00 = mix(n000, n100, f.x); float nx10 = mix(n010, n110, f.x);
    float nx01 = mix(n001, n101, f.x); float nx11 = mix(n011, n111, f.x);
    float nxy0 = mix(nx00, nx10, f.y); float nxy1 = mix(nx01, nx11, f.y);
    return mix(nxy0, nxy1, f.z);
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

float warpedFbm(vec3 p) {
    vec3 q = vec3(fbm(p, 4), fbm(p + vec3(5.2, 1.3, 2.8), 4), fbm(p + vec3(9.1, 4.7, 3.1), 4));
    return fbm(p + q * 1.6, 8);
}

float ridgeNoise(vec3 p, int octaves) {
    float v = 0.0, amp = 0.5, prev = 1.0;
    for (int i = 0; i < octaves; i++) {
        float n = abs(noise3d(p));
        n = 1.0 - n;
        n = n * n;
        v += n * amp * prev;
        prev = n;
        p = p * 2.07 + vec3(0.131, -0.217, 0.344);
        amp *= 0.48;
    }
    return v;
}

void main() {
    vec3 N = normalize(vNormal);
    vec3 L = normalize(frame.lightDir);
    vec3 V = normalize(frame.viewPos - vWorldPos);
    vec3 H = normalize(L + V);

    // 从世界空间位置重建单位球面坐标（等价于 GL 版的 vLocalPos）
    vec3 sph = normalize(vWorldPos - pc.planetCenter.xyz);
    vec3 texCoord = sph * 3.5;
    float lat    = sph.y;
    float absLat = abs(lat);

    float continent = warpedFbm(texCoord * 1.2);
    float seaLevel  = 0.45;
    bool  isLand    = continent > seaLevel;
    float landHeight = isLand ? (continent - seaLevel) / (1.0 - seaLevel) : 0.0;

    // ---- 颜色调色板 ----
    vec3 deepOcean  = vec3(0.012, 0.045, 0.14);
    vec3 midOcean   = vec3(0.03,  0.10,  0.30);
    vec3 shallowSea = vec3(0.06,  0.22,  0.42);
    vec3 coastWater = vec3(0.08,  0.35,  0.50);
    vec3 beach      = vec3(0.76,  0.70,  0.50);
    vec3 lowland    = vec3(0.18,  0.42,  0.10);
    vec3 forest     = vec3(0.08,  0.30,  0.06);
    vec3 highland   = vec3(0.45,  0.38,  0.22);
    vec3 mountain   = vec3(0.52,  0.46,  0.40);
    vec3 peak       = vec3(0.78,  0.76,  0.74);
    vec3 snow       = vec3(0.92,  0.94,  0.96);
    vec3 desert     = vec3(0.72,  0.58,  0.32);
    vec3 tundra     = vec3(0.55,  0.58,  0.50);

    vec3  surfColor = vec3(0.0);
    float specMask  = 0.0;

    if (!isLand) {
        float oceanDepth = (seaLevel - continent) / seaLevel;
        if      (oceanDepth < 0.05) surfColor = mix(coastWater, shallowSea, oceanDepth / 0.05);
        else if (oceanDepth < 0.3)  surfColor = mix(shallowSea, midOcean,  (oceanDepth - 0.05) / 0.25);
        else                        surfColor = mix(midOcean,   deepOcean,  min(1.0, (oceanDepth - 0.3) / 0.7));
        specMask = 1.0;
    } else {
        float biomeNoise = noise3d(texCoord * 6.0);
        float moisture   = fbm(texCoord * 3.0 + vec3(42.0), 4);
        float ridgeH     = ridgeNoise(texCoord * 2.5, 6);
        float combinedH  = landHeight * 0.7 + ridgeH * 0.3;

        if (combinedH < 0.02) {
            surfColor = beach;
        } else if (absLat < 0.25 && moisture < 0.4 && combinedH < 0.3) {
            vec3 sandDark = vec3(0.60, 0.45, 0.25);
            surfColor = mix(desert, sandDark, biomeNoise * 0.5);
        } else if (absLat < 0.20 && moisture > 0.45 && combinedH < 0.35) {
            vec3 jungle = vec3(0.05, 0.25, 0.04);
            surfColor = mix(forest, jungle, moisture);
        } else if (absLat < 0.55 && combinedH < 0.4) {
            float forestBlend = smoothstep(0.02, 0.15, combinedH);
            vec3 grassland = vec3(0.25, 0.48, 0.12);
            surfColor = mix(grassland, forest, forestBlend * moisture);
            surfColor = mix(surfColor, lowland, biomeNoise * 0.3);
        } else if (absLat > 0.65 && combinedH < 0.5) {
            vec3 taiga = vec3(0.20, 0.32, 0.18);
            surfColor = mix(tundra, taiga, moisture * (1.0 - absLat));
        } else {
            if      (combinedH < 0.3)  surfColor = mix(lowland,  highland, combinedH / 0.3);
            else if (combinedH < 0.55) surfColor = mix(highland, mountain, (combinedH - 0.3) / 0.25);
            else                       surfColor = mix(mountain, peak,     (combinedH - 0.55) / 0.45);
        }

        float snowLine = 0.65 - absLat * 0.45;
        if (combinedH > snowLine) {
            float ridgeHLocal = ridgeNoise(texCoord * 2.5, 6);
            float snowBlend = smoothstep(snowLine, snowLine + 0.12, combinedH);
            snowBlend *= (0.7 + 0.3 * ridgeHLocal);
            surfColor = mix(surfColor, snow, snowBlend);
        }
    }

    // ---- 极地冰盖 ----
    float iceNoise = fbm(texCoord * 4.0 + vec3(100.0), 4);
    float iceEdge  = 0.82 - iceNoise * 0.08;
    if (absLat > iceEdge) {
        float ice = smoothstep(iceEdge, iceEdge + 0.06, absLat);
        surfColor = mix(surfColor, snow, ice * 0.95);
        specMask  = mix(specMask, 0.3, ice);
    }

    // ---- 光照 ----
    float NdotL     = dot(N, L);
    float diff      = max(NdotL, 0.0);
    float ambient   = 0.008;
    float NdotH     = max(dot(N, H), 0.0);
    float spec      = pow(NdotH, 256.0) * specMask * 2.5;
    float fresnel   = pow(1.0 - max(dot(N, V), 0.0), 4.0);
    spec *= (1.0 + fresnel * 3.0);

    // ---- 大气晕边 ----
    float rim     = 1.0 - max(dot(N, V), 0.0);
    float rimPow  = pow(rim, 4.0);
    vec3 dayRim    = vec3(0.25, 0.50, 1.0);
    vec3 sunsetRim = vec3(1.0,  0.35, 0.08);
    vec3 atmosColor = mix(sunsetRim, dayRim, smoothstep(0.0, 0.3, NdotL));
    atmosColor = atmosColor * rimPow * 0.65 * smoothstep(-0.15, 0.1, NdotL);

    vec3 result = surfColor * (ambient + diff * 0.92) + atmosColor;
    result += vec3(1.0, 0.95, 0.85) * spec * diff;

    // ---- 夜间城市灯光 ----
    if (NdotL < 0.03 && isLand) {
        float nightFade = smoothstep(0.03, -0.08, NdotL);
        float city1 = noise3d(texCoord * 12.0);
        float city2 = noise3d(texCoord * 25.0 + vec3(7.7));
        float city3 = noise3d(texCoord * 50.0 + vec3(13.1));
        float habitability  = smoothstep(0.5, 0.0, landHeight) * smoothstep(0.0, 0.1, landHeight);
        habitability *= (1.0 - smoothstep(0.6, 0.85, absLat));
        float cityBrightness = 0.0;
        if (city1 > 0.55) cityBrightness += (city1 - 0.55) * 4.0;
        if (city2 > 0.6)  cityBrightness += (city2 - 0.6)  * 2.0;
        if (city3 > 0.65) cityBrightness += (city3 - 0.65) * 1.0;
        cityBrightness = min(cityBrightness * habitability * nightFade, 1.2);
        vec3 cityColor = mix(vec3(1.0, 0.7, 0.2), vec3(1.0, 0.9, 0.6), city2);
        result += cityColor * cityBrightness * 0.8;
    }

    FragColor = vec4(result, 1.0);
}
