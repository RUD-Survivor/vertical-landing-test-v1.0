#pragma once
// ==========================================================================
// weather_map_bake.h — Flower 体积云 weather map 的 CPU 烘焙（纯像素生成）
//
// 职责边界：
//   • 本文件：气候带 × 陆块 × 球面 domain-warp FBM → RGBA8 像素缓冲
//   • 不含 Vulkan：不建纹理、不 upload、不建 ImageView
//   • VkScene::initCloudTextures() 负责 upload + ImGui 预览 swizzle view
//
// 通道布局（与 cloud_common.glsl / Cloud Tuner 预览约定一致）：
//   R = coverage    G = cloudType    B = humidity    A = land
//
// 默认分辨率 2048×1024：赤道周长 ≈4e4 km → ≈19.5 km/px。
// ==========================================================================

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <vector>

namespace WeatherMapBake {

constexpr int   kWidth    = 2048;
constexpr int   kHeight   = 1024;
constexpr int   kChannels = 4;
constexpr float kPi       = 3.14159265358979323846f;

namespace detail {

inline float fract(float x) { return x - floorf(x); }

inline float hash(float px, float py, float pz) {
    px = fract(px * 443.897f);
    py = fract(py * 441.423f);
    pz = fract(pz * 437.195f);
    float d = px * (py + 19.19f) + py * (pz + 19.19f) + pz * (px + 19.19f);
    px += d;
    py += d;
    pz += d;
    return fract((px + py) * pz);
}

inline float noise3d(float px, float py, float pz) {
    float ix = floorf(px), iy = floorf(py), iz = floorf(pz);
    float fx = px - ix, fy = py - iy, fz = pz - iz;
    fx = fx * fx * fx * (fx * (fx * 6.f - 15.f) + 10.f);
    fy = fy * fy * fy * (fy * (fy * 6.f - 15.f) + 10.f);
    fz = fz * fz * fz * (fz * (fz * 6.f - 15.f) + 10.f);
    float n000 = hash(ix, iy, iz),         n100 = hash(ix + 1.f, iy, iz);
    float n010 = hash(ix, iy + 1.f, iz),   n110 = hash(ix + 1.f, iy + 1.f, iz);
    float n001 = hash(ix, iy, iz + 1.f),   n101 = hash(ix + 1.f, iy, iz + 1.f);
    float n011 = hash(ix, iy + 1.f, iz + 1.f), n111 = hash(ix + 1.f, iy + 1.f, iz + 1.f);
    float nx00 = n000 + fx * (n100 - n000), nx10 = n010 + fx * (n110 - n010);
    float nx01 = n001 + fx * (n101 - n001), nx11 = n011 + fx * (n111 - n011);
    return (nx00 + fy * (nx10 - nx00)) +
           fz * ((nx01 + fy * (nx11 - nx01)) - (nx00 + fy * (nx10 - nx00)));
}

inline float fbm(float px, float py, float pz, int oct) {
    float v = 0.f, amp = 0.5f;
    for (int i = 0; i < oct; i++) {
        v += noise3d(px, py, pz) * amp;
        px = px * 2.07f + 0.131f;
        py = py * 2.07f - 0.217f;
        pz = pz * 2.07f + 0.344f;
        amp *= 0.48f;
    }
    return v;
}

inline uint8_t clampU8(float v) {
    int i = (int)(v * 255.f + 0.5f);
    return (uint8_t)(i < 0 ? 0 : i > 255 ? 255 : i);
}

inline float sat01(float v) { return v < 0.f ? 0.f : (v > 1.f ? 1.f : v); }

inline float ellipse(float lon, float lat, float cLon, float cLat, float hW, float hH) {
    float dlon = lon - cLon;
    if (dlon > 180.f) dlon -= 360.f;
    if (dlon < -180.f) dlon += 360.f;
    float dlat = lat - cLat;
    float r = sqrtf((dlon / hW) * (dlon / hW) + (dlat / hH) * (dlat / hH));
    if (r >= 1.3f) return 0.f;
    if (r <= 0.7f) return 1.f;
    float tt = (r - 0.7f) / 0.6f;
    return 1.f - tt * tt * (3.f - 2.f * tt);
}

inline float gauss(float x, float c, float s) {
    float d = (x - c) / s;
    return expf(-0.5f * d * d);
}

inline float landMask(float lon, float lat) {
    float land = 0.f;
    land = fmaxf(land, ellipse(lon, lat, -98.f, 52.f, 42.f, 30.f));
    land = fmaxf(land, ellipse(lon, lat, -90.f, 18.f, 13.f, 12.f));
    land = fmaxf(land, ellipse(lon, lat, -60.f, -14.f, 22.f, 27.f));
    land = fmaxf(land, ellipse(lon, lat, 12.f, 52.f, 22.f, 18.f));
    land = fmaxf(land, ellipse(lon, lat, 17.f, 20.f, 38.f, 22.f));
    land = fmaxf(land, ellipse(lon, lat, 22.f, -10.f, 28.f, 26.f));
    land = fmaxf(land, ellipse(lon, lat, 45.f, 23.f, 18.f, 13.f));
    land = fmaxf(land, ellipse(lon, lat, 73.f, 28.f, 18.f, 14.f));
    land = fmaxf(land, ellipse(lon, lat, 108.f, 36.f, 30.f, 28.f));
    land = fmaxf(land, ellipse(lon, lat, 80.f, 62.f, 65.f, 17.f));
    land = fmaxf(land, ellipse(lon, lat, 102.f, 15.f, 15.f, 12.f));
    land = fmaxf(land, ellipse(lon, lat, 118.f, -3.f, 20.f, 7.f));
    land = fmaxf(land, ellipse(lon, lat, 134.f, -26.f, 22.f, 16.f));
    land = fmaxf(land, ellipse(lon, lat, -42.f, 72.f, 22.f, 12.f));
    {
        float tt = fminf(1.f, fmaxf(0.f, (-lat - 70.f) / 14.f));
        land = fmaxf(land, tt * tt * (3.f - 2.f * tt));
    }
    return fminf(land, 1.f);
}

} // namespace detail

// 气候带均值 × 球面 domain-warped 天气场 → 锋面/斑块/破碎 ITCZ。
// out 会被 resize 为 kWidth * kHeight * 4。
inline bool generate(std::vector<uint8_t>& out) {
    using namespace detail;

    out.assign((size_t)kWidth * (size_t)kHeight * (size_t)kChannels, 0);
    constexpr float kDeg2Rad = kPi / 180.f;

    printf("[WeatherMapBake] Baking weather map %dx%d (climate × domain-warp)...\n",
           kWidth, kHeight);

    for (int y = 0; y < kHeight; y++) {
        float lat = ((float)y / (float)kHeight - 0.5f) * 180.f;
        for (int x = 0; x < kWidth; x++) {
            float lon = ((float)x / (float)kWidth - 0.5f) * 360.f;
            float land = landMask(lon, lat);

            float absLat = fabsf(lat);
            float wITCZ = gauss(absLat, 2.f, 6.f), wSubDry = gauss(absLat, 27.f, 10.f);
            float wTemperate = gauss(absLat, 52.f, 13.f), wPolar = gauss(absLat, 75.f, 9.f);
            float wSum = wITCZ + wSubDry + wTemperate + wPolar + 1e-6f;
            wITCZ /= wSum; wSubDry /= wSum; wTemperate /= wSum; wPolar /= wSum;

            float cov_oc = wITCZ * 0.82f + wSubDry * 0.35f + wTemperate * 0.68f + wPolar * 0.52f;
            float type_oc = wITCZ * 0.84f + wSubDry * 0.15f + wTemperate * 0.48f + wPolar * 0.12f;
            float hum_oc = wITCZ * 0.88f + wSubDry * 0.52f + wTemperate * 0.72f + wPolar * 0.58f;
            float cov_la = wITCZ * 0.55f + wSubDry * 0.28f + wTemperate * 0.48f + wPolar * 0.38f;
            float type_la = wITCZ * 0.65f + wSubDry * 0.30f + wTemperate * 0.38f + wPolar * 0.08f;
            float hum_la = wITCZ * 0.62f + wSubDry * 0.28f + wTemperate * 0.52f + wPolar * 0.42f;
            float climateCov = cov_oc + (cov_la - cov_oc) * land;
            float climateType = type_oc + (type_la - type_oc) * land;
            float climateHum = hum_oc + (hum_la - hum_oc) * land;

            // 球面坐标（无缝），再做两级 domain warp 打破纬线圈
            float lonR = lon * kDeg2Rad, latR = lat * kDeg2Rad;
            float cl = cosf(latR);
            float sx = cl * cosf(lonR), sy = cl * sinf(lonR), sz = sinf(latR);

            float wx = fbm(sx * 2.1f + 17.1f, sy * 2.1f - 3.4f, sz * 2.1f + 9.2f, 3);
            float wy = fbm(sx * 2.1f - 8.7f, sy * 2.1f + 22.3f, sz * 2.1f - 4.1f, 3);
            float wz = fbm(sx * 2.1f + 5.5f, sy * 2.1f + 11.0f, sz * 2.1f + 30.6f, 3);
            const float warp1 = 0.55f;
            float qx = sx + (wx * 2.f - 1.f) * warp1;
            float qy = sy + (wy * 2.f - 1.f) * warp1;
            float qz = sz + (wz * 2.f - 1.f) * warp1;

            float w2x = fbm(qx * 3.3f + 41.f, qy * 3.3f, qz * 3.3f - 19.f, 2);
            float w2y = fbm(qx * 3.3f - 27.f, qy * 3.3f + 8.f, qz * 3.3f, 2);
            float w2z = fbm(qx * 3.3f, qy * 3.3f + 13.f, qz * 3.3f + 7.f, 2);
            const float warp2 = 0.35f;
            float rx = qx + (w2x * 2.f - 1.f) * warp2;
            float ry = qy + (w2y * 2.f - 1.f) * warp2;
            float rz = qz + (w2z * 2.f - 1.f) * warp2;

            // 天气尺度场 + 温带 storm-track（略沿经向拉长）
            float synoptic = fbm(rx * 4.5f, ry * 4.5f, rz * 4.5f, 4);
            float midLat = gauss(absLat, 48.f, 14.f);
            float track = fbm(sx * 1.2f + sy * 3.8f, sz * 2.0f, sx * 0.5f - sy * 0.3f + 40.f, 3);
            float band = sat01(synoptic * 0.72f + track * 0.48f * midLat);

            // 破碎 ITCZ（不是一整圈均匀云带）
            float itczBreak = fbm(sx * 6.f + 3.f, sy * 6.f, sz * 1.5f + 50.f, 3);
            float itczMask = gauss(absLat, 2.f, 7.f) * sat01(itczBreak * 1.65f - 0.22f);

            float weatherField = sat01(band * 0.85f + itczMask * 0.55f);
            weatherField = sat01((weatherField - 0.28f) / 0.42f); // 高对比：大片晴空 vs 浓云
            weatherField = weatherField * weatherField * (3.f - 2.f * weatherField);
            weatherField = powf(weatherField, 1.12f);

            // 把 Weather FBM 内容贴满 2048 纹理带宽（赤道 ≈19.5 km/px）。
            // 球频 f≈R/λ（R≈6370）：原 synoptic 最细约 150km；下列三层补 250→~19.5km。
            // 分档加权（而非单条长 FBM），避免细层被 amp*=0.48 埋掉。
            float meso = fbm(rx * 25.f + 11.f, ry * 25.f - 4.f, rz * 25.f + 19.f, 3);   // ~255→~60 km
            float fine = fbm(rx * 100.f + 55.f, ry * 100.f - 12.f, rz * 100.f + 3.f, 3); // ~64→~15 km
            float micro = fbm(rx * 327.f + 7.f, ry * 327.f + 21.f, rz * 327.f - 9.f, 2); // ~19.5→~9 km
            float detailFill = sat01(meso * 0.40f + fine * 0.35f + micro * 0.30f);
            weatherField = sat01(weatherField * (0.45f + 0.55f * detailFill) + detailFill * 0.22f);

            float coverage = sat01(climateCov * (0.20f + 0.98f * weatherField));
            float coast = land * (1.f - land) * 4.f; // 海岸抬升一点点
            coverage = sat01(coverage + coast * 0.08f * weatherField);

            float cloudTypeV = sat01(climateType * (0.35f + 0.75f * weatherField) + (1.f - land) * 0.04f);
            float humidity = sat01(climateHum * (0.30f + 0.80f * (0.45f * weatherField + 0.55f * synoptic)));

            size_t idx = ((size_t)y * (size_t)kWidth + (size_t)x) * (size_t)kChannels;
            out[idx]     = clampU8(coverage);
            out[idx + 1] = clampU8(cloudTypeV);
            out[idx + 2] = clampU8(humidity);
            out[idx + 3] = clampU8(land);
        }
    }

    return true;
}

} // namespace WeatherMapBake
