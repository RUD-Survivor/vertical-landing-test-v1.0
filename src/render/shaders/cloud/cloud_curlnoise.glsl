#version 460
#extension GL_GOOGLE_include_directive : enable

#include "cloud_common_noise.glsl"

// 2D 湍流细节噪声烘焙，供 cloud_common.glsl::cloudMap() 里 inWeatherTexture 之外
// 那层"局部碎片/不均匀"(localCoverage)采样用。flower 原版这里用的是一整套专用
// curl noise 数据资产（本工程没有），也不属于"接入体积云算法"本身——复用已有的
// tileable Perlin/Worley 基元（cloud_common_noise.glsl，与 basicnoise/detailnoise
// 同一套函数），在固定 z 切片上采样：这些噪声函数按 mod(p,freq) 逐轴平铺，固定 z
// 不影响 xy 方向的无缝平铺，输出仍可以 REPEAT 采样。
layout (set=0,binding=0,r8) uniform image2D imageCurlNoise;

layout(local_size_x=8,local_size_y=8,local_size_z=1) in;

void main()
{
    ivec2 texSize=imageSize(imageCurlNoise);
    ivec2 workPos=ivec2(gl_GlobalInvocationID.xy);

    if(workPos.x>=texSize.x || workPos.y>=texSize.y)
    {
        return;
    }

    const vec2 uv=(vec2(workPos)+vec2(0.5))/vec2(texSize);
    const vec3 uvw=vec3(uv,0.5);// 固定 z 切片，freq 平铺只依赖 xy 分量

    float w=worleyFbm(uvw,8.0);// 高频 Worley：碎块感
    float p=perlinFbm(uvw,4.0,4);// 低频 Perlin：柔和大尺度起伏
    p=abs(p*2.0-1.0);// billowy，避免大片负值被浪费

    float turbulence=clamp(mix(w,p,0.35),0.0,1.0);

    imageStore(imageCurlNoise,workPos,vec4(turbulence));
}
