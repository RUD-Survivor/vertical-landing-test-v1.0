#version 460
#extension GL_GOOGLE_include_directive : enable

#include "cloud_common_noise.glsl"

// 真正的 curl 噪声：对矢量势场 F 求旋度 ∇×F，得到无散（divergence-free）矢量场。
// 用途是"位移"而不是"覆盖度"：cloud_common.glsl::cloudMap() 里把 3D 噪声采样位置
// 加上这个矢量，让云的形状/边缘产生湍流卷曲（wispy / swirling），并随风的平流
// 自然演化。之前的"假 curl"（固定 z 切片的 Worley/Perlin 标量混合）不是矢量场，
// 只能当 coverage 噪声用，方向和幅度信息全部丢失，故整个重写。
//
// 烘焙到 64³ RGBA8（storage image）：
//   RGB = 归一化方向（[-1,1] 编码到 [0,1]，shader 采样后 *2-1 还原）
//   A   = 幅度（curl 大小压缩到 [0,1]，shader 乘 curlStrength 得 km 位移）
// 势场用 tileable Perlin FBM（cloud_common_noise.glsl，mod(p,freq) 逐轴平铺），
// 旋度由中心差分得到，自动继承无缝平铺，输出可直接 REPEAT 采样。

#define kCurlFreq 8.0     // 每 tile 的势场特征数
#define kCurlMagScale 0.5 // 幅度压缩：mag*scale → A（超 1 的部分 clamp）
#define kCurlEps 0.05     // 中心差分步长（势场频率空间，一个特征约 1/kCurlFreq）

layout (set=0,binding=0,rgba8) uniform image3D imageCurlNoise;

layout(local_size_x=8,local_size_y=8,local_size_z=1) in;

// 矢量势的三个分量：同一 tileable Perlin FBM，用不同偏移错开相位，避免三分量相关。
vec3 curlPotential(vec3 p)
{
    return vec3(
        perlinFbm(p,                    kCurlFreq, 3),
        perlinFbm(p+vec3(37.1,11.7,61.3),  kCurlFreq, 3),
        perlinFbm(p+vec3(-83.4,53.1,-29.8),kCurlFreq, 3));
}

void main()
{
    ivec3 texSize=imageSize(imageCurlNoise);
    ivec3 workPos=ivec3(gl_GlobalInvocationID.xyz);

    if(workPos.x>=texSize.x || workPos.y>=texSize.y || workPos.z>=texSize.z)
    {
        return;
    }

    const vec3 uv=(vec3(workPos)+vec3(0.5))/vec3(texSize);

    // 中心差分求旋度：curl = ∇×F
    const float e=kCurlEps;
    vec3 Fpx0=curlPotential(uv-vec3(e,0.0,0.0));
    vec3 Fpx1=curlPotential(uv+vec3(e,0.0,0.0));
    vec3 Fpy0=curlPotential(uv-vec3(0.0,e,0.0));
    vec3 Fpy1=curlPotential(uv+vec3(0.0,e,0.0));
    vec3 Fpz0=curlPotential(uv-vec3(0.0,0.0,e));
    vec3 Fpz1=curlPotential(uv+vec3(0.0,0.0,e));

    vec3 curlV;
    curlV.x=(Fpy1.z-Fpy0.z)-(Fpz1.y-Fpz0.y);
    curlV.y=(Fpz1.x-Fpz0.x)-(Fpx1.z-Fpx0.z);
    curlV.z=(Fpx1.y-Fpx0.y)-(Fpy1.x-Fpy0.x);
    curlV/=(2.0*e);

    // 编码：方向进 RGB，幅度进 A（幅度为 0 的点给一个无害的向上方向）。
    float mag=length(curlV);
    vec3 dir=(mag>1e-5)?(curlV/mag):vec3(0.0,1.0,0.0);
    vec4 encoded=vec4(dir*0.5+0.5,clamp(mag*kCurlMagScale,0.0,1.0));

    imageStore(imageCurlNoise,workPos,encoded);
}
