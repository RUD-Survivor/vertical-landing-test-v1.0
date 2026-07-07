#version 460
#extension GL_GOOGLE_include_directive : enable
#extension GL_EXT_samplerless_texture_functions : enable
//允许写成 texture(sampler2D(tex, sampler), uv)，把 纹理对象 和 采样器状态（repeat/clamp、filter 等）分开传

#include "cloud_common.glsl"

//1/4 分辨率云渲染 + Bayer 时域抖动：每个低分辨率线程算云时，对应到 全分辨率屏幕上的哪一个像素
layout(local_size_x=8,local_size_y=8) in;

void main()
{
    ivec2 texSize=imageSize(imageCloudRenderTexture);//纹理大小，imageCloudRenderTexture来自cloud_common.glsl的set = 0, binding = 2
    ivec2 workPos=ivec2(gl_GlobalInvocationID);//全局线程ID,也是1/4分辨率像素坐标

    if(workPos.x>=texSize.x||workPos.y>=texSize.y)
    {
        return;
    }//超出纹理大小就退出

    uint bayerIndex=frameData.frameIndex.x%16;//当前帧在 16 帧循环里的相位（frameIndex 是 uvec4，取 .x）
    ivec2 bayerOffset=ivec2(kBayerMatrix16[bayerIndex]%4,kBayerMatrix16[bayerIndex]/4);//kBayerMatrix16来自shared_functions.glsl
    //从 kBayerMatrix16 取出本帧在 4×4 块内的偏移 (0~3, 0~3)

    ivec2 fullResSize=texSize*4;//全分辨率宽高
    ivec2 fullResWorkPos=workPos*4+bayerOffset;//本帧该线程实际追踪的 全分辨率像素坐标
    
    const vec2 uv=(vec2(fullResWorkPos)+vec2(0.5))/vec2(fullResSize);//该像素的屏幕uv坐标



    float blueNoise = fract(bayer64(vec2(workPos.xy)) + frameData.frameIndex.x / float(frameData.jitterPeriod));//值为0~1
    //blueNoise，bayer64来自shared_functions.glsl
    //bayer64(vec2(workPos.xy))，按 1/4 分辨率像素坐标 查 64×64 Bayer 矩阵，得到 [0,1) 的空间抖动
    //每帧加一个 时间偏移，周期 jitterPeriod 帧后循环

    // Offset retarget for new seeds each frame每帧重映射的采样坐标
    uvec2 offset = uvec2(vec2(0.754877669, 0.569840296) * (frameData.frameIndex.x) * uvec2(texSize));
    uvec2 offsetId = workPos.xy + offset;
    offsetId.x = offsetId.x % texSize.x;
    offsetId.y = offsetId.y % texSize.y;
    float blueNoise2 = whangHashNoise(offsetId.x, offsetId.y, frameData.frameIndex.x);//仅x,y二维
    //本没有 flower 那套预烘焙蓝噪声数据表，改用 whangHashNoise（shared_functions.glsl）代替，
    //效果是普通白噪声抖动而非蓝噪声，时域/空间分布没那么均匀，但足够打散 banding，见 cloud_common.glsl 顶部说明

    // We are revert z.flower 用 Reversed-Z（近=1、远=0），和 OpenGL 默认相反
    //——————————从屏幕UV反推出该像素对应的相机视线方向worldDir————————————————————————note:vulkan的uv坐标以左上为原点，opengl以左下为原点
    vec4 clipSpace = vec4(uv.x * 2.0f - 1.0f, 1.0f - uv.y * 2.0f, 0.0, 1.0);//分量x从[0,1]映射到[-1,1],分量y做同样映射但反转方向，z=0这里 z=0 配合他们的camInvertProj约定,UV->裁剪空间(NDC)
    vec4 viewPosH = frameData.camInvertProj * clipSpace;//camInvertProj是投影矩阵的逆，把NDC上的点反投影到视空间(相机在原点)
    vec3 viewSpaceDir = viewPosH.xyz / viewPosH.w;//除以 w 得到视空间 3D 点，相机在 (0,0,0)，从原点指向该点的向量就是视空间射线方向
    vec3 worldDir = normalize((frameData.camInvertView * vec4(viewSpaceDir, 0.0)).xyz);//camInvertView是视图矩阵的逆，w=0.0表示只做旋转不加平移，normalize单位化
    //比cloud.frag更加完整，适用所有投影，值得保留

    AtmosphereParameters atmosphere=getAtmosphereParameters(frameData);//AtmosphereParameters和getAtmosphereParameters来自shared_atmosphere.glsl

    float depth=0.0f;//传给 cloudColorCompute 的 cloudZ。march 结束后写入该像素云的代表深度（clip z/w，Reversed-Z）。用于重建、TAA、与场景深度比较
    vec4 fogLighting=vec4(0.0);//God Ray / 低层体积雾。.xyz = 散射 RGB，.w = 有效标记（函数内会先设 -1 表示未算）。


    vec4 cloudColor=cloudColorCompute(atmosphere, uv, blueNoise2, depth, workPos, worldDir, frameData.sky.atmosphereConfig.cloudGodRay != 0, fogLighting, blueNoise2);
    // cloudColor.rgb = scatteredLight：整条视线 cloud march（+ 789 行空气透视）累积的云散射光 RGB，HDR 辐射度。
    // cloudColor.a   = transmittance：视线剩余透射 T_view∈[0,1]；1=全透明（无云），0=全挡背景。

    imageStore(imageCloudRenderTexture,workPos,cloudColor);//RGB散射+A透射，binding=2
    imageStore(imageCloudDepthTexture,workPos,vec4(depth));//云代表深度,bingding=14
    imageStore(imageCloudFogRenderTexture,workPos,fogLighting);//God Ray/雾,bingding=22
    //C++侧需绑定
}