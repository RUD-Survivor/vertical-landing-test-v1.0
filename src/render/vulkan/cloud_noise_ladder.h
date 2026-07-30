#pragma once
// ==========================================================================
// cloud_noise_ladder.h — 体积云噪声频带阶梯的唯一真源（Nyquist 换算 + 幅度规则）
//
// 一张烘焙好的噪声纹理能表达的频率上限受 Nyquist 限制：边长 N 的纹理最多分辨
// f_max ≈ N/2 个周期，再高的层只会写入混叠噪点。纹理内部最低频 f_base 由烘焙
// shader 的基频决定，于是单张纹理只能覆盖 f_max/f_base 倍的世界尺度跨度：
//
//   平铺周期 tile(km) = 1 / scale          scale 单位 1/km（cloudMap 按 km 采样）
//   最粗特征 = tile / f_base
//   最细特征 = tile / f_max
//
// 想覆盖"行星尺度 → 米级"这么宽的频带，必须多档接力：把下一档的 scale 乘上
// bandSpan，接缝处两档的世界周期正好相等，既不留空洞也不重复同一频率。
//
// 数值来源（改动任一处都要回到这里同步）：
//   纹理边长   vk_scene.h::initCloudTextures()      Basic 128³ / Detail 32³
//   纹理基频   cloud_basicnoise.glsl kBasicFrequency / cloud_detailnoise.glsl
//              kDetailFrequency（两者都是 4）
//   采样与混合 cloud_common.glsl::sampleCloudShapeNoise() / cloudMap()
// ==========================================================================

#include "gpu_frame_data.h"

namespace CloudNoiseLadder {

// 两张噪声纹理的最低频（每个纹理平铺周期内的特征数）
constexpr float kBandBaseFreq = 4.f;

// 烘焙分辨率
constexpr int kBasicTexDim  = 128;
constexpr int kDetailTexDim = 32;

// 单档能覆盖的世界尺度跨度 = Nyquist 频率 / 基频
constexpr float kBasicBandSpan  = (kBasicTexDim  * 0.5f) / kBandBaseFreq; // 16
constexpr float kDetailBandSpan = (kDetailTexDim * 0.5f) / kBandBaseFreq; // 4

// 幅度规则：形状链逐档减半；侵蚀链要明显低于形状链末档，否则细碎 Worley 会把
// 大形状打成雪花噪点（cloudMap 里侵蚀是 remap 的下限阈值，不是加性细节）。
constexpr float kShapeAmpFalloff   = 0.5f;
constexpr float kErosionAmpFalloff = 0.2f;

// 形状链首档的世界频率：一个纹理周期 ≈ 333 km，对应真实天气系统尺度。
constexpr float kDefaultBasicScale = 0.003f;

// 由首档推出的其余档位（各差一个 bandSpan，接缝处周期相等）
constexpr float kDefaultMesoScale   = kDefaultBasicScale * kBasicBandSpan;  // 0.048
constexpr float kDefaultFineScale   = kDefaultMesoScale  * kBasicBandSpan;  // 0.768
constexpr float kDefaultDetailScale = kDefaultFineScale  * kBasicBandSpan;  // 12.288

constexpr float kDefaultMesoAmp   = kShapeAmpFalloff;                      // 0.5
constexpr float kDefaultFineAmp   = kDefaultMesoAmp * kShapeAmpFalloff;    // 0.25
constexpr float kDefaultDetailAmp = kDefaultFineAmp * kErosionAmpFalloff;  // 0.05

// ── 频带 LOD（屏幕足迹）─────────────────────────────────────────────────────
// 一档噪声在屏幕上占不到一两个像素时就不再是"细节"，只是每帧乱跳的噪点，可以整档关掉。
// 特征周期 P(km) 在距离 d(km) 处横跨的像素数：
//
//   nPx ≈ P·H / (d · fovy)          H=渲染高度(px)，fovy=竖直视场(rad)
//
// 于是"开始淡出"的距离可以直接解出来（UI 用它显示各档大致作用范围）：
//
//   dOff ≈ P·H / (nMin · fovy)
//
// 判据只依赖距离/视场/分辨率，和 raymarch 的自适应步长解耦：步长由 empty skip 与
// edge refine 负责，两者若互相挂钩会形成"缩步→开细档→更碎→更缩步"的反馈。
constexpr float kDefaultLodNMin = 1.1f;            // 低于 1.1px 的档开始淡出
constexpr float kDefaultLodShadowDistScale = 1.0f; // 阴影侧按 1.0 倍距离算，比主视线更早关
constexpr float kLodFadeSpan = 0.25f;             // 完全关闭点 = nMin × 此值（smoothstep 下界）

// UI 读数用的参考视口：面板不持有真实相机参数，用常见 1080p/60° 给一个量级参考。
constexpr float kLodPreviewRenderHeight = 1080.f;
constexpr float kLodPreviewFovYRad = 1.0472f; // 60°

// 该档从多远开始淡出（km）。P 为该档最细特征周期，见 finestPeriodKm()。
inline float lodFadeStartKm(float finestKm, float nMin,
                            float renderHeight = kLodPreviewRenderHeight,
                            float fovYRad = kLodPreviewFovYRad) {
    if (!(finestKm > 0.f) || !(nMin > 0.f) || !(fovYRad > 0.f)) return 0.f;
    return finestKm * renderHeight / (nMin * fovYRad);
}

// cloudMap 的 posKm 是"相对行星中心"的坐标，量级 ≈ 行星半径。float32 在这个量级上的
// 相邻可表示值间隔就是世界空间采样网格的硬下限（地球量级 ≈ 0.8 m）：频带细端接近它时
// 噪声不会更细，只会被量化成台阶，必须先把 march 改成相机相对坐标才能继续往下做。
constexpr float kPlanetScaleSampleMagnitudeKm = 6400.f;
constexpr float kFloat32Epsilon = 1.19209290e-7f;
inline float samplePrecisionKm() { return kPlanetScaleSampleMagnitudeKm * kFloat32Epsilon; }

// scale ≤ 0 视为"该档无效"，统一返回 0，让 UI 读数和越界保护走同一条路径。
inline float tileKm(float scale) { return scale > 0.f ? 1.f / scale : 0.f; }
inline float coarsestPeriodKm(float scale) { return tileKm(scale) / kBandBaseFreq; }
inline float finestPeriodKm(float scale, int texDim) {
    return texDim > 1 ? tileKm(scale) / (texDim * 0.5f) : 0.f;
}
// 下一档接在本档细端之后。bandSpan 取"正在结束的那一档"所用纹理的跨度：
// 三档形状都用 128³，所以形状链内部以及形状→侵蚀的这一步都乘 kBasicBandSpan；
// kDetailBandSpan 只在侵蚀链之后再接一档（Micro）时才用得上。
inline float nextScale(float scale, float bandSpan) { return scale * bandSpan; }

// 把整条阶梯写进 UBO 镜像。基频/分辨率变了只需改上面的常量，所有调用方自动跟随。
inline void apply(GpuAtmosphereConfig& cfg, float basicScale) {
    if (!(basicScale > 0.f)) basicScale = kDefaultBasicScale;
    cfg.cloudBasicNoiseScale  = basicScale;
    cfg.cloudMesoNoiseScale   = nextScale(basicScale, kBasicBandSpan);
    cfg.cloudFineNoiseScale   = nextScale(cfg.cloudMesoNoiseScale, kBasicBandSpan);
    cfg.cloudDetailNoiseScale = nextScale(cfg.cloudFineNoiseScale, kBasicBandSpan);
    cfg.cloudMesoAmplitude   = kDefaultMesoAmp;
    cfg.cloudFineAmplitude   = kDefaultFineAmp;
    cfg.cloudDetailAmplitude = kDefaultDetailAmp;
    cfg.cloudNoiseLodNMin = kDefaultLodNMin;
    cfg.cloudNoiseLodShadowDistScale = kDefaultLodShadowDistScale;
    cfg.cloudNoiseLodEnabled = 1;
}

} // namespace CloudNoiseLadder
