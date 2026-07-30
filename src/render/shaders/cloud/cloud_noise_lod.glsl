#ifndef CLOUD_NOISE_LOD_GLSL
#define CLOUD_NOISE_LOD_GLSL

// ==========================================================================
// cloud_noise_lod.glsl — 噪声频带的屏幕足迹 LOD（纯数学，不碰纹理与业务字段）
//
// 公式与 CPU 侧 cloud_noise_ladder.h 的同名注释一致：一档噪声的特征周期 P(km)
// 在距离 d(km) 处横跨的像素数
//
//   nPx = P·H / (d · fovy · distScale)
//
// 低于 nMin 像素时该档已经无法被屏幕采样分辨，继续叠加只会得到逐帧闪烁的噪点，
// 于是按 smoothstep 把它的幅度收到 0（硬切会在视距推进时"啪"地跳出一层细节）。
//
// 判据只依赖 距离 / 视场 / 分辨率 / 该档 P，刻意不挂钩 raymarch 的自适应步长：
// 步长由 empty skip 与 edge refine 决定，若反过来驱动 LOD 会形成
// "缩步 → 开细档 → 边缘更碎 → 更缩步" 的正反馈。
//
// 改 f_max（纹理分辨率）或各档 scale：只动 cloud_noise_ladder.h 与调用方传进来的
// scale 字段，本文件不需要改。
// ==========================================================================

// 完全关闭点相对 nMin 的比例（smoothstep 下界）。与 CloudNoiseLadder::kLodFadeSpan 同值。
#define kCloudLodFadeSpan 0.25

// 一个像素在单位距离上张开的角度（rad/px）。fovy/H 退化时返回 0，调用方据此保持全档常开。
float cloudLodPixelTheta(float fovYRad,float renderHeight)
{
    return (fovYRad>0.0&&renderHeight>1.0)?fovYRad/renderHeight:0.0;
}

// 该档在此采样点的幅度权重 [0,1]。
//   dKm       采样点到相机距离（km，与 cloudMap 的大气空间同单位）
//   periodKm  该档最细特征周期 = 1/(scale·f_max)
//   distScale 有效距离倍率：主视线 1，阴影用更大值以更早丢弃细档
float cloudBandScreenWeight(float dKm,float periodKm,float pixelTheta,float nMin,float distScale)
{
    // 任一参数无效（未填 fovy、周期为 0、LOD 关闭时传入的 theta=0）→ 不做裁剪
    if(pixelTheta<=0.0||periodKm<=0.0||nMin<=0.0) return 1.0;

    float d=max(dKm,1e-4)*max(distScale,1e-4);
    float nPx=periodKm/(d*pixelTheta);
    return smoothstep(nMin*kCloudLodFadeSpan,nMin,nPx);
}

#endif
