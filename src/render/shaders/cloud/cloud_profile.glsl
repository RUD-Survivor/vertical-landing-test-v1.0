#ifndef CLOUD_PROFILE_GLSL
#define CLOUD_PROFILE_GLSL

// ==========================================================================
// cloud_profile.glsl — 云类型垂直廓形 LUT（Profile LUT）采样模块
//
// 布局（ProfileLutBake::generate 烘焙，与 HanPi 同族）：
//   U = 归一化高度 ∈[0,1]（0=云底，1=云顶）
//   V = 径向距离 ∈[0,1]（0=中心，1=边缘；当前按 kEdgeThin=1.0 烘焙为均匀，
//       无径向衰减——径向衰减曾导致低云覆盖率整体下降）
//   R = 积云 Cu 廓形   G = 塔状积云 Tcu 廓形   B = 积雨云 Cb 廓形
//
// 类型轴映射：weather.g ∈[0,1] 经对比度/偏移重映射到 LUT 类型轴
// （0=Cu, 0.5=Tcu, 1=Cb），再按三段线性插值取 Cu/Tcu/Cb 三条曲线。
// 廓形为近似 LWC 的钟形（峰值中上部、顶部归零），云顶边界由此定义。
// ==========================================================================

// weather.g → LUT 类型轴。>1 拉开类型差异，<1 收敛到中部类型。
float cloudProfileTypeAxis(float weatherType)
{
    const float contrast=frameData.sky.atmosphereConfig.cloudProfileTypeContrast;
    const float bias    =frameData.sky.atmosphereConfig.cloudProfileTypeBias;
    return saturate((weatherType-0.5)*max(contrast,1e-3)+0.5+bias);
}

// 按云类型取垂直廓形（U=高度 V=radial，RGB=Cu/Tcu/Cb 三段插值）。
float cloudProfileHeightGradient(float cloudType, float normalizedHeight, vec2 weatherUv)
{
    const float radialDist=saturate(length(weatherUv-vec2(0.5))*2.0);
    const vec3 profiles=texture(sampler2D(inCloudProfileLut,linearClampEdgeSampler),
                                vec2(normalizedHeight,radialDist)).rgb;
    float g;
    if(cloudType<0.5)
        g=mix(profiles.r,profiles.g,cloudType*2.0);
    else
        g=mix(profiles.g,profiles.b,(cloudType-0.5)*2.0);
    return g;
}

#endif
