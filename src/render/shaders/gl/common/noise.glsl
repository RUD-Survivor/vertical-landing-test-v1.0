// ============================================================================
// noise.glsl — 公共噪声函数库
// 被所有程序化行星/地形/大气/SVO 着色器共享
// ============================================================================

#ifndef NOISE_GLSL
#define NOISE_GLSL

// ---- 哈希原语 ----

// 基础 3D 哈希（最通用，用于所有程序化行星着色器）
float hash(vec3 p) {
    p = fract(p * vec3(443.897, 441.423, 437.195));
    p += dot(p, p.yzx + 19.19);
    return fract((p.x + p.y) * p.z);
}

// 1D 哈希（用于地球着色器、尾焰着色器等的细节扰动）
float hash1(float n) { return fract(sin(n) * 43758.5453123); }

// 带别名的 3D 哈希（用于 SVO / 地形着色器的不同命名约定）
float hash3(vec3 p) {
    p = fract(p * vec3(443.897, 441.423, 437.195));
    p += dot(p, p.yzx + 19.19);
    return fract((p.x + p.y) * p.z);
}

// 天空盒用 3D 哈希（与 hash/hash3 相同逻辑，独立的符号以避免冲突）
float hash13(vec3 p) {
    p = fract(p * vec3(443.897, 441.423, 437.195));
    p += dot(p, p.yzx + 19.19);
    return fract((p.x + p.y) * p.z);
}

// 地形着色器用 3D 哈希（返回 vec3 用于 Worley 噪声）
vec3 hash33(vec3 p) {
    p = fract(p * vec3(443.897, 441.423, 437.195));
    p += dot(p, p.yzx + 19.19);
    return fract((p.xxy + p.yzz) * p.zyx);
}

// ---- 3D 值噪声 ----

// 五次平滑插值版本（地球、气态巨行星、荒芜星体、所有 8 个行星着色器）
float noise3d(vec3 p) {
    vec3 i = floor(p);
    vec3 f = fract(p);
    f = f * f * f * (f * (f * 6.0 - 15.0) + 10.0); // 五次平滑插值 (Quintic)
    float n000 = hash(i); float n100 = hash(i + vec3(1,0,0));
    float n010 = hash(i + vec3(0,1,0)); float n110 = hash(i + vec3(1,1,0));
    float n001 = hash(i + vec3(0,0,1)); float n101 = hash(i + vec3(1,0,1));
    float n011 = hash(i + vec3(0,1,1)); float n111 = hash(i + vec3(1,1,1));
    float nx00 = mix(n000, n100, f.x); float nx10 = mix(n010, n110, f.x);
    float nx01 = mix(n001, n101, f.x); float nx11 = mix(n011, n111, f.x);
    float nxy0 = mix(nx00, nx10, f.y); float nxy1 = mix(nx01, nx11, f.y);
    return mix(nxy0, nxy1, f.z);
}

// ---- 分形布朗运动 (FBM) ----

// 通过叠加多个不同频率和振幅的噪声层来模拟自然界的自相似性（如山脉）
float fbm(vec3 p, int octaves) {
    float v = 0.0, amp = 0.5;
    for (int i = 0; i < octaves; i++) {
        v += noise3d(p) * amp;
        p = p * 2.07 + vec3(0.131, -0.217, 0.344);
        amp *= 0.48;
    }
    return v;
}

// ---- 领域扭曲 FBM（地球着色器） ----

// 用一个噪声去扭曲另一个噪声的输入，生成更自然的海岸线
float warpedFbm(vec3 p) {
    vec3 q = vec3(fbm(p, 4), fbm(p + vec3(5.2, 1.3, 2.8), 4), fbm(p + vec3(9.1, 4.7, 3.1), 4));
    return fbm(p + q * 1.6, 8);
}

// ---- 脊状噪声 (Ridge Noise) — 地球/火星/地形着色器 ----

// 用于生成尖锐的山脉脊线
float ridgeNoise(vec3 p, int octaves) {
    float v = 0.0, amp = 0.5, prev = 1.0;
    for (int i = 0; i < octaves; i++) {
        float n = abs(noise3d(p));
        n = 1.0 - n;  // 翻转以创建山脊
        n = n * n;     // 加剧山脊的尖锐度
        v += n * amp * prev;
        prev = n;
        p = p * 2.07 + vec3(0.131, -0.217, 0.344);
        amp *= 0.48;
    }
    return v;
}

// ---- 地形着色器用 FBM 变体（带 ridgedFbm 别名） ----

float ridgedFbm(vec3 p, int oct) {
    float v = 0.0, a = 0.5, f = 1.05;
    for (int i = 0; i < oct; ++i) {
        float n = 1.0 - abs(noise3d(p * f) * 2.0 - 1.0);
        v += n * n * a;
        f *= 2.07; a *= 0.48;
    }
    return v;
}

// ---- 地形着色器用 mountainNoise ----

float mountainNoise(vec3 p) {
    vec3 off = vec3(fbm(p * 2.0, 3), fbm(p * 2.0 + 5.2, 3), fbm(p * 2.0 + 2.7, 3));
    return ridgedFbm(p + off * 0.15, 6);
}

// ---- Worley 噪声（地形着色器用，模拟岩石裂纹） ----

float worley(vec3 p) {
    vec3 i = floor(p); vec3 f = fract(p);
    float minDist = 1.0;
    for (int z = -1; z <= 1; z++) {
        for (int y = -1; y <= 1; y++) {
            for (int x = -1; x <= 1; x++) {
                vec3 neighbor = vec3(float(x), float(y), float(z));
                vec3 point = hash33(i + neighbor);
                vec3 diff = neighbor + point - f;
                minDist = min(minDist, length(diff));
            }
        }
    }
    return minDist;
}

#endif // NOISE_GLSL
