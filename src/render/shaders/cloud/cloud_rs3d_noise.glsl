// =============================================================================
// cloud_rs3d_noise.glsl
// -----------------------------------------------------------------------------
// 用途：RocketSim3D 体积云噪声采样与 remap 工具库（binding 无关）。
//
// 提供内容：
//   • remap / remapClamped   — 线性区间映射（无 saturate / 带 clamp）
//   • basicNoiseComposite()  — Perlin-Worley 两阶段合成（GPU Pro 7 / HZD）
//   • sampleBakedN()         — 从烘焙 3D 纹理恢复 Perlin-Worley 形状
//   • cheapCloudNoise()      — 低成本球面采样（远景 fallback）
//   • uniformPhase()         — 各向同性相位 1/(4π)
//
// 设计原则：
//   1. 不声明任何 binding —— uNoiseTex3D / uCoverTex3D / uDetailTex3D / uWeatherMap
//      由调用方 .frag / .comp 提供（与 cloud.frag 原版一致）。
//   2. 通道约定：R=Perlin-Worley 合成 / G,B,A=Worley octaves w1,w2,w3
//      （与 vk_scene.h CPU 烘焙一致，见 cloud_basicnoise.glsl）。
//   3. INV8 = 1/8：3D 纹素 8 次平铺，避免高频走样。
//
// 来源：cloud.frag 第 139–178、180–182 行抽取。
// =============================================================================

#ifndef CLOUD_RS3D_NOISE_GLSL
#define CLOUD_RS3D_NOISE_GLSL

// 3D 纹理采样时的统一缩放：texSize 8 → INV8 使采样坐标 *8 在纹理内平铺多次。
const float INV8 = 0.125;

// ── remap ─────────────────────────────────────────────────────────────────────
// 线性区间映射：把 x 从 [a,b] 映射到 [c,d]，不做 clamp。
// 与 cloud_basicnoise.glsl 的 remap 一致（注意：与 cloud_common.glsl 的 remap 不同——
// 后者带 saturate，是 flower 版本）。
float remap(float x, float a, float b, float c, float d) {
    return (((x - a) / (b - a)) * (d - c)) + c;
}

// 带 clamp 的 remap：先归一化到 [0,1] 再映射到 [c,d]。
// 用于 coverage / 形状 / 高度阈值等需要安全区间的场合。
float remapClamped(float value, float oldMin, float oldMax, float newMin, float newMax) {
    float t = clamp((value - oldMin) / max(oldMax - oldMin, 1e-5), 0.0, 1.0);
    return mix(newMin, newMax, t);
}

// ── Perlin-Worley 两阶段合成 ─────────────────────────────────────────────────
// GPU Pro 7 / HZD 经典做法：
//   v = (perlin, w1, w2, w3)
//   wfbm = w1*0.625 + w2*0.25 + w3*0.125   （Worley 分形布朗运动）
//   stage2 = remap(perlin, wfbm-1, 1, 0, 1) （用 Worley 包络裁剪 Perlin）
// 输入 v 的通道布局见 sampleBakedN。
float basicNoiseComposite(vec4 v) {
    float wfbm = v.y * 0.625 + v.z * 0.25 + v.w * 0.125;
    return remap(v.x, wfbm - 1.0, 1.0, 0.0, 1.0);
}

// ── 从烘焙 3D 纹理恢复 Perlin-Worley ─────────────────────────────────────────
// vk_scene.h CPU 烘焙布局（RGBA）：
//   R = (perlin + 1 - wfbm) / (2 - wfbm)   ← 合成后的 Perlin-Worley（便于 R8-only 退化）
//   G/B/A = Worley octaves w1/w2/w3
//
// 本函数做两件事：
//   1. 若 G+B+A < 0.01（R8-only 烘焙），直接返回 R（退化路径）。
//   2. 否则反解 Perlin，再做 stage1 remap(perlin, 0, 1, w1, 1) + basicNoiseComposite。
//      stage1 用 w1 作 Worley 地板裁剪 Perlin，stage2 用 wfbm 作整体包络。
//
// tc：采样坐标（已 *INV8 由调用方决定平铺密度，本函数再 *INV8）。
float sampleBakedN(vec3 tc) {
    vec4 col = texture(uNoiseTex3D, tc * INV8);
    float worleySum = col.g + col.b + col.a;
    if (worleySum < 0.01)
        return col.r;  // R8-only 退化：直接用合成值

    float wfbm = col.g * 0.625 + col.b * 0.25 + col.a * 0.125;
    // 反解 CPU 烘焙公式：R = (perlin + 1 - wfbm) / (2 - wfbm)
    float perlin = col.r * (2.0 - wfbm) - (1.0 - wfbm);
    perlin = clamp(perlin, 0.0, 1.0);
    // stage1：用 w1 作 Worley 地板裁剪 Perlin
    col.r = remap(perlin, 0.0, 1.0, col.g, 1.0);
    return clamp(basicNoiseComposite(col), 0.0, 1.0);
}

// 低成本球面噪声：用于远景 fallback / 低频形状。
// 球面方向 *6 + 时间漂移，再走 sampleBakedN。
float cheapCloudNoise(vec3 sph, float t) {
    return sampleBakedN(sph * 6.0 + vec3(t * 0.001));
}

// ── 各向同性相位 ─────────────────────────────────────────────────────────────
// 1/(4π)，用于多散射高阶项（与 cloudPhase 形成对比）。
float uniformPhase() {
    return 1.0 / (4.0 * PI);
}

#endif // CLOUD_RS3D_NOISE_GLSL
