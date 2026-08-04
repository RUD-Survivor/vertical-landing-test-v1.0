#ifndef VOLUMETRIC_CLOUD_COMMON_GLSL
#define VOLUMETRIC_CLOUD_COMMON_GLSL

#extension GL_EXT_samplerless_texture_functions : enable

// Reference implement from https://www.slideshare.net/guerrillagames/the-realtime-volumetric-cloudscapes-of-horizon-zero-dawn.
// Reference from https://github.com/qiutang98/flower/blob/main/source/shader/cloud/cloud_common.glsl

#include "../common/shared_functions.glsl"
#include "../common/shared_atmosphere.glsl"
#include "cloud_noise_lod.glsl"

//// #define SkyMsExtintion 0.5
#define SkyMsExtinction 0.5//天空方向多次散射的消光衰减系数,后续可以修改硬编码参数
#define GroundOcc 0.5//地面遮挡/地面光贡献的固定系数,算 groundLit（云底收到的来自地面/下方天空的照明）时会用到

layout (set = 0, binding = 0, rgba16f) uniform image2D imageHdrSceneColor; // HDR 场景颜色（可写）；合成 pass 将云叠回最终 HDR 缓冲
layout (set = 0, binding = 1) uniform texture2D inHdrSceneColor; // HDR 场景颜色（只读）；合成时读取已有场景色并与云混合

layout (set = 0, binding = 2, rgba16f) uniform image2D imageCloudRenderTexture; // 1/4 分辨率云渲染 RT（可写）；compute 写入散射光 + 透射率
layout (set = 0, binding = 3) uniform texture2D inCloudRenderTexture; // 1/4 分辨率云渲染 RT（只读）；上采样/重建 pass 采样

layout (set = 0, binding = 4) uniform texture2D inDepth; // 场景深度；限制云 ray march 终点、避免穿过不透明几何
layout (set = 0, binding = 5) uniform texture2D inGBufferA; // G-Buffer A（如法线/材质）；与场景几何正确合成云

layout (set = 0, binding = 6) uniform texture3D inBasicNoise; // 3D 基础噪声（Perlin-Worley）；云块大形状与覆盖率
layout (set = 0, binding = 7) uniform texture3D inDetailNoise; // 3D 细节噪声（Worley）；云边缘侵蚀与内部细节
layout (set = 0, binding = 8) uniform texture2D inWeatherTexture; // 2D 天气图（coverage/type/humidity）；区域云量与类型变化
layout (set = 0, binding = 9) uniform texture2D inCloudCurlNoise; // 2D curl 噪声；局部扰动形状与 coverage 变化

layout (set = 0, binding = 10) uniform texture2D inTransmittanceLut; // 大气透射率 LUT；ToPoint 段透射 +（日照路径已改实时）
layout (set = 0, binding = 11) uniform texture3D inInscatterLut; // Bruneton Inscatter；云空气透视 GetSkyRadianceToPoint

layout (set = 0, binding = 12, rgba16f) uniform image2D imageCloudReconstructionTexture; // 全分辨率云重建 RT（可写）；上采样/滤波后输出
layout (set = 0, binding = 13) uniform texture2D inCloudReconstructionTexture; // 全分辨率云重建 RT（只读）；合成 pass 与 temporal 采样
layout (set = 0, binding = 14, r32f) uniform image2D imageCloudDepthTexture; // 1/4 分辨率云深度 RT（可写）；记录云平均命中深度
layout (set = 0, binding = 15) uniform texture2D inCloudDepthTexture; // 1/4 分辨率云深度（只读）；重建与重投影用
layout (set = 0, binding = 16, r32f) uniform image2D imageCloudDepthReconstructionTexture; // 全分辨率云深度 RT（可写）；上采样后的云深度
layout (set = 0, binding = 17) uniform texture2D inCloudDepthReconstructionTexture; // 全分辨率云深度（只读）；与场景深度合成、TAA
layout (set = 0, binding = 18) uniform texture2D inCloudReconstructionTextureHistory; // 上一帧全分辨率云颜色；时域重投影/TAA 稳定
layout (set = 0, binding = 19) uniform texture2D inCloudDepthReconstructionTextureHistory; // 上一帧全分辨率云深度；时域深度验证

layout (set = 0, binding = 20) uniform texture2D inSkyViewLut; // 天空视图 LUT；按方向/高度查环境天光（lookupSkylight）

layout (set = 0, binding = 21) uniform UniformFrameData { PerFrameData frameData; }; // 每帧 UBO：相机矩阵、时间、天空/大气/云参数
//块内只有一个只读成员：名为 frameData 的 PerFrameData 结构体,该结构体在 shared_struct.glsl定义

layout (set = 0, binding = 22, rgba16f) uniform image2D imageCloudFogRenderTexture; // 1/4 分辨率云雾/God Ray RT（可写）；低分辨率体积雾
layout (set = 0, binding = 23) uniform texture2D inCloudFogRenderTexture; // 1/4 分辨率云雾（只读）；雾重建 pass 采样
layout (set = 0, binding = 24, rgba16f) uniform image2D imageCloudFogReconstructionTexture; // 全分辨率云雾重建 RT（可写）；上采样后雾效
layout (set = 0, binding = 25) uniform texture2D inCloudFogReconstructionTexture; // 全分辨率云雾（只读）；与云/场景合成
layout (set = 0, binding = 26) uniform texture2D inCloudFogReconstructionTextureHistory; // 上一帧全分辨率云雾；雾效时域滤波

layout (set = 0, binding = 27) uniform texture2D inCloudAmbientMusLut; // 云环境天光 μs→半球辐照度/π 1D LUT（与相机无关）

layout (set = 0, binding = 28) uniform texture2D inSDSMShadowDepth; // SDSM 级联阴影深度；云受太阳阴影影响
layout (set = 0, binding = 29) buffer SSBOCascadeInfoBuffer{ CascadeInfo cascadeInfos[]; }; // 阴影级联 SSBO；各 cascade 视锥与矩阵
//可读写的 storage 缓冲（SSBO），不是uniform
//块内有一个cascadeInfos[]可变长度数组成员，c++侧定义CascadeInfo结构体

layout (set = 0, binding = 30) uniform texture2D inHiz; // 层次化 Z（Hi-Z）；快速估算场景深度，提前终止云 ray march
//image2D和texture2D成对出现，image2D 用于 compute 的 imageStore 写入，texture2D 用于后续 pass 的 texture() 采样
//大括号{}定义interface block接口块的成员，大括号列出这块GPU缓存里有哪些成员

layout (set = 0, binding = 31) uniform texture2D inCloudProfileLut; // 双层云：云类型垂直廓形 LUT（U=类型轴 0=Cu/0.5=Tcu/1=Cb，V=归一化高度，R=廓形）
layout (set = 0, binding = 32) uniform texture2D inCloudHiWeather;  // 双层云：高空云第二套天气图（R=HiMask G=HiType As/Ac）

// 相机→云大气段 RGB 透射 T_air（AP）；composite：src × (T_air × T_cloud) + L'
layout (set = 0, binding = 33, rgba16f) uniform image2D imageCloudAirTRenderTexture;
layout (set = 0, binding = 34) uniform texture2D inCloudAirTRenderTexture;
layout (set = 0, binding = 35, rgba16f) uniform image2D imageCloudAirTReconstructionTexture;
layout (set = 0, binding = 36) uniform texture2D inCloudAirTReconstructionTexture;
layout (set = 0, binding = 37) uniform texture2D inCloudAirTReconstructionTextureHistory;

//用宏指定 Descriptor Set 编号，再引入公共采样器定义
#define SHARED_SAMPLER_SET 1
#include "../common/shared_sampler.glsl"

// 双层云模块：Profile LUT + 高空云密度（并进 cloudMap 共用主 march）+ 公共光照。
#include "cloud_profile.glsl"
#include "cloud_high_layer.glsl" // highCloudDensity：与低云同 σ 单位，max() 进 cloudMap
#include "cloud_lighting.glsl"   // 公共光照组装（sampleCloudAmbient/powder/散射公式）
#include "cloud_air_perspective.glsl" // Inscatter ToPoint 空气透视
//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//note:现在是Set1 的 sampler3D uNoiseTex3D 等，layout 完全不同。替换时要 重排 descriptor，或改 cloud_common.glsl 的 binding 对齐现有 vk_descriptors.h


// Set 2：Heitz Sobol 蓝噪声 SSBO（shared_blue_noise.glsl），由 vk_blue_noise.h 上传，
// cloud_raymarching.glsl / cloud_composite.glsl 在 include 前 #define BLUE_NOISE_BUFFER_SET 2。

// RS3D Cloud Tuner：updateFaceIndex 复用为 1/4 Bayer 开关（1=1/4+Bayer+reconstruct，0=全分辨率直写重建 RT）
bool cloudUseQuarterRes()
{
    return frameData.sky.atmosphereConfig.updateFaceIndex != 0u;
}

float getDensity(float heightMeter)//单位为米
{
    return exp(-heightMeter * 0.001) * 0.001 * 0.001 * frameData.sky.atmosphereConfig.cloudGodRayScale;//可调强度来自 AtmosphereConfig，目前缺frameData与调用方法
}//高度相关的指数衰减密度，给 God Ray / 体积雾 pass 用，不是主云体 cloudMap() 的密度

#define MsCount 2//多重散射阶数

struct ParticipatingMedia//参与介质（Participating Media）的多次散射近似数据结构
{
    float extinctionCoefficients[MsCount];//各阶消光系数 σ_t（单位长度上的衰减强度）第 0 阶：来自 cloudMap() 的真实云密度第 1 阶：× cloudMultiScatterExtinction（逐阶衰减）
    float transmittanceToLight[MsCount];//从采样点沿光方向到光源的透射率 T（0~1）T=e−∫σt​ds 第0阶：直射光还能剩多少（云影深浅） 第1阶：高阶路径的“可见度”，通常更亮、更 diffuse
    float extinctionAcc[MsCount];//沿光路径累积的消光（光学深度 τ 的积分），再换算成透射率
    float phiFwd;//phi_fwd 各向同性漫射场：沿同一条光路累积的标量辐照度（无量纲，已含 Intensity）。
    //与上面三项正交——那三项是「还剩多少直射光」，这项是「多次散射后填进来多少漫射光」。
    //只有太阳方向的 shadow march 会填它；天空方向恒为 0。

};//第 1 阶让云 背光面/云体内 不会死黑，更接近真实多次散射，代价只是多算一条 shadow march 和一个散射阶循环

float remap(float value,float originMin,float originMax,float newMin,float newMax)
{
    return newMin+(saturate((value-originMin)/(originMax-originMin))*(newMax-newMin));
}//saturate来自shared_functions.glsl用于把值钳制至[0,1]和cloud_basicnoise.glsl的remap()并不相同


//texture(sampler(纹理，采样器规则)，uv)是GLSL内置函数：用UV坐标从纹理中取色,返回值类型vec4
//采样规则：采样规则（怎么插值、怎么重复）
//uv:读哪一点，通常[0,1]可以超出

// 幅度低于此值就整档跳过采样：既省一次 texture()，也是频带 LOD 的关闭档
// （把某档幅度调 0 等价于把该档从频带阶梯里摘掉，画面退化成上一档的结果）。
#define kCloudBandAmpEpsilon 1e-4

// 三档形状噪声共用同一张 inBasicNoise，只换世界频率（scale 单位 1/km）。同一张纹理
// 在不同尺度上会把噪声特征点排在同一批世界坐标上，产生网格状对齐痕迹，所以每档加一个
// 固定世界偏移（km）打散相位；偏移取无理数量级的杂值，避免和纹理平铺周期成整数比。
const vec3 kCloudMesoOffsetKm = vec3(37.19, 11.73, 61.31);
const vec3 kCloudFineOffsetKm = vec3(-83.47, 53.09, -29.83);

// 各噪声纹理能分辨的最高频（一个平铺周期内的特征数）= N/2。与 cloud_noise_ladder.h
// 的 kBasicTexDim / kDetailTexDim 同源，换烘焙分辨率时两边一起改。
#define kCloudBasicNyquistFreq 64.0  // 128³ inBasicNoise
#define kCloudDetailNyquistFreq 16.0 // 32³  inDetailNoise

// 该档最细特征的世界周期（km）：scale 单位 1/km，平铺周期 = 1/scale。
float cloudBandFinestPeriodKm(float scale,float nyquistFreq)
{
    return scale>0.0?1.0/(scale*nyquistFreq):0.0;
}

// LOD 关闭时返回 0：cloudBandScreenWeight 对 theta<=0 恒返回 1，等价于全档常开（回归开关）。
float cloudLodPixelThetaFromFrame()
{
    if(frameData.sky.atmosphereConfig.cloudNoiseLodEnabled==0) return 0.0;
    return cloudLodPixelTheta(frameData.camInfo.x,frameData.renderHeight);
}

// 阴影 march 的细节最终只汇成一个标量透射率，比主视线更早丢高频档看不出差别，
// 却能省下成倍的 texture()。倍率下限 1，避免误配成"阴影比主视线还细"。
float cloudLodShadowDistScale()
{
    return max(frameData.sky.atmosphereConfig.cloudNoiseLodShadowDistScale,1.0);
}
#define kCloudLodDistScaleView 1.0

// 返回值与"只采一档 Basic"同量纲（[0,1]）：幅度和被归一化，因此 meso/fine 幅度为 0 时
// 结果与单档采样逐位一致，方便按频带做 LOD 或回退。
// dKm/pixelTheta/lodDistScale 用于屏幕足迹 LOD：小到看不见的档幅度收 0，走 amp>ε 整档跳过。
float sampleCloudShapeNoise(vec3 posKm,vec3 windOffset,float dKm,float pixelTheta,float lodDistScale)
{
    const float lodNMin=frameData.sky.atmosphereConfig.cloudNoiseLodNMin;
    // Basic 不参与淡出：它是云的形状锚点，淡掉不是"变粗"而是整团消失。
    float mesoAmp=max(frameData.sky.atmosphereConfig.cloudMesoAmplitude,0.0)
        *cloudBandScreenWeight(dKm,
            cloudBandFinestPeriodKm(frameData.sky.atmosphereConfig.cloudMesoNoiseScale,kCloudBasicNyquistFreq),
            pixelTheta,lodNMin,lodDistScale);
    float fineAmp=max(frameData.sky.atmosphereConfig.cloudFineAmplitude,0.0)
        *cloudBandScreenWeight(dKm,
            cloudBandFinestPeriodKm(frameData.sky.atmosphereConfig.cloudFineNoiseScale,kCloudBasicNyquistFreq),
            pixelTheta,lodNMin,lodDistScale);

    float noise=texture(sampler3D(inBasicNoise,linearRepeatSampler),
        (posKm+windOffset)*frameData.sky.atmosphereConfig.cloudBasicNoiseScale).r;
    float ampSum=1.0;

    if(mesoAmp>kCloudBandAmpEpsilon)
    {
        noise+=mesoAmp*texture(sampler3D(inBasicNoise,linearRepeatSampler),
            (posKm+windOffset+kCloudMesoOffsetKm)*frameData.sky.atmosphereConfig.cloudMesoNoiseScale).r;
        ampSum+=mesoAmp;
    }
    if(fineAmp>kCloudBandAmpEpsilon)
    {
        noise+=fineAmp*texture(sampler3D(inBasicNoise,linearRepeatSampler),
            (posKm+windOffset+kCloudFineOffsetKm)*frameData.sky.atmosphereConfig.cloudFineNoiseScale).r;
        ampSum+=fineAmp;
    }

    return noise/ampSum;
}

// lodDistScale：屏幕足迹 LOD 的有效距离倍率。主视线/空跳探测/debug 传 kCloudLodDistScaleView，
// 各条 shadow march 传 cloudLodShadowDistScale()——不给默认值是刻意的，新增调用点必须
// 显式表态走哪条策略，避免阴影和主视线悄悄用上两套不同的细节。
float cloudMap(vec3 posMeter,float normalizedHeight,float lodDistScale)//单位是米，根据位置和归一化高度计算云密度
{
    const float Coverage=frameData.sky.atmosphereConfig.cloudCoverage;
    const float Density=frameData.sky.atmosphereConfig.cloudDensity;
    const vec3 windDirection=frameData.sky.atmosphereConfig.cloudDirection;
    const float cloudSpeed =frameData.sky.atmosphereConfig.cloudSpeed;

    posMeter+=windDirection*normalizedHeight*500.0f;//按云内高度，沿风向“错开”采样位置，让云在风里呈现倾斜/错层，而不是竖直一柱

    vec3 posKm=posMeter*0.001;//单位转换成km，后面 weather / 3D noise 都按 km 尺度采样

    vec3 windOffset=(windDirection+vec3(0.0,0.1,0.0))*frameData.appTime.x*cloudSpeed;//随时间的风动画偏移，用于后面采样 inBasicNoise / inDetailNoise，windDirection主导风向，+vec3(0.0,0.1,0.0)给风一点垂直分量，云不止在水平位移，appTime.x * cloudSpeed：时间 × 速度 → 位移量

    // 行星尺度修正：posMeter 现在是"相对当前行星中心"的位置（C++ 侧每帧减去 planetCenter 后
    // 写入 frameData.camWorldPos，见 vk_atmosphere_lut.h::bake()），所以 normalize(posMeter) 就是
    // 该点的行星径向单位向量，可以直接转经纬度。不再用 posKm.xz 平面 UV（相机绕到行星背面/两极时
    // 会跟着乱飘），改成球面经纬度 UV——但 inWeatherTexture 本身、天气图的内容完全不变，
    // 仍是 vk_scene.h 里生成的那张 2048×1024 地理天气图（保留原算法的 weather map）。
    vec3 sphDir=normalize(posMeter);
    float lonRad=atan(sphDir.y,sphDir.x);
    float latRad=asin(clamp(sphDir.z,-1.0,1.0));
    vec2 sampleUv=vec2(lonRad*(1.0/(2.0*kPI))+0.5, latRad*(1.0/kPI)+0.5);
    sampleUv*=frameData.sky.atmosphereConfig.cloudWeatherUVScale;//保留原有的可调 UV 缩放（云图案缩放/重复）

    vec4 weatherValue=texture(sampler2D(inWeatherTexture,linearRepeatSampler),sampleUv);//查2D天气图，.x区域coverage bias .g/.b/.a云类型、湿度等

    // Curl（假资产：bake 的 Worley/Perlin 2D）→ localCoverage。
    // UV 用局部东/北向 km（不随浮点原点漂移）；频率/强度/对比均由 Cloud Tuner 控制。
    float cosLat=max(cos(latRad),0.01);
    float eastKm=lonRad*(frameData.sky.atmosphereConfig.bottomRadius)*cosLat;
    float northKm=latRad*frameData.sky.atmosphereConfig.bottomRadius;
    float curlUVScale=frameData.sky.atmosphereConfig.cloudCurlUVScale;
    float curlStrength=frameData.sky.atmosphereConfig.cloudCurlStrength;
    float curlContrast=frameData.sky.atmosphereConfig.cloudCurlContrast;
    // 旧硬编码等价：uvScale=0.001 → (time*speed*50 + km*1000)*1e-6 + 0.5
    vec2 curlUv=vec2(eastKm,northKm)*curlUVScale
        +vec2(frameData.appTime.x*cloudSpeed*(50.0*curlUVScale),0.0)
        +vec2(0.5);
    float localCoverage=0.0;
    if(curlStrength>1e-5)
    {
        localCoverage=texture(sampler2D(inCloudCurlNoise,linearRepeatSampler),curlUv).x;
        // saturate(x*c - c*0.25)*strength；c=3 → 旧式 *3-0.75
        localCoverage=saturate(localCoverage*curlContrast-curlContrast*0.25)*curlStrength;
    }
    //大尺度云量靠天气图，curl 只负责中小尺度碎块

    float coverage=saturate(Coverage*(localCoverage+weatherValue.x));
    //合成这一位置的最终覆盖率，Coverage全局云量参数，weatherValue.x天气图大尺度bias，三者相加再 × 全局coverage，最后saturate到[0,1]

    // 垂直廓形走 Profile LUT（cloud_profile.glsl）：weather.g 提供云类型，
    // 类型 0（Cu）起、1（Cb）止，三段插值得到该高度的廓形强度。
    // 廓形为近似 LWC 的钟形曲线（峰值中上部、顶部归零），云顶边界由此定义。
    // 类型轴映射（对比度/偏移）+ 三段插值（Cu/Tcu/Cb）+ 径向维度都在模块内完成。
    float gradientShape=cloudProfileHeightGradient(cloudProfileTypeAxis(weatherValue.g),normalizedHeight,sampleUv);

    // 屏幕足迹 LOD 的采样点距离。相机位置走 convertToAtmosphereUnit（米→km 并加上
    // kAtmoCameraOffsetKm 径向偏移），和 march 起点用的是同一个 worldPos，不另算一套。
    float lodDistKm=length(posKm-convertToAtmosphereUnit(frameData.camWorldPos.xyz,frameData));
    float lodPixelTheta=cloudLodPixelThetaFromFrame();

    float basicNoise=sampleCloudShapeNoise(posKm,windOffset,lodDistKm,lodPixelTheta,lodDistScale);
    //形状链：Basic/Meso/Fine 三档共用 inBasicNoise（Perlin-Worley，cloud_basicnoise.glsl 烘出来的），
    //只换世界频率 scale（1/km）；windOffset 让三档按同一个世界风速平移（物理上同一团云在动）。
    //一张 128³ 纹理有效频带 f=4…64（Nyquist），单档只能盖 ×16 的世界尺度，所以要多档接力。

    float basicCloudNoise=basicNoise*gradientShape;
    //3D 噪声 × 垂直高度包络 = 带正确层高分布的云块形状

    float basicCloudNoiseWithCoverage=coverage*remap(basicCloudNoise,1.0-coverage,1.0,0,1);
    //把 coverage 叠到大形状噪声上,coverage越小云生成阈值越大云越少越碎，越高则反

    //侵蚀链：接在形状链最细一档之后，只负责 Worley 碎边，幅度必须低于形状链末档，
    //否则细碎噪声会把大形状打散成雪花。幅度 0 时整档跳过采样。
    float detailAmp=max(frameData.sky.atmosphereConfig.cloudDetailAmplitude,0.0)
        *cloudBandScreenWeight(lodDistKm,
            cloudBandFinestPeriodKm(frameData.sky.atmosphereConfig.cloudDetailNoiseScale,kCloudDetailNyquistFreq),
            lodPixelTheta,frameData.sky.atmosphereConfig.cloudNoiseLodNMin,lodDistScale);
    float detailNoiseMixByHeight=0.0;
    if(detailAmp>kCloudBandAmpEpsilon)
    {
        vec3 sampleDetailNoise=posKm-windOffset*0.15;
        //与基础噪声略反风偏移，避免完全对齐

        float detailNoiseComposite=texture(sampler3D(inDetailNoise,linearRepeatSampler),sampleDetailNoise*frameData.sky.atmosphereConfig.cloudDetailNoiseScale).r;
        //3DWorley细节噪声采样，从烘焙好的细节3D纹理（cloud_detailnoise.glsl）采样

        detailNoiseMixByHeight=detailAmp*mix(detailNoiseComposite,1-detailNoiseComposite,saturate(normalizedHeight*10.0));
        //前10%高度云底用一种侵蚀，高层云顶用反相侵蚀，x*detail+(1-x)(1-detail)=1-x+(2x-1)*detail
    }


    float densityShape=saturate(0.01+normalizedHeight*1.15)*Density*
    remap(normalizedHeight,0.0,0.1,0.0,1.0);
    //0.01 + height*1.15：云底略薄，往上略增
    //remap(0, 0.1, 0, 1)：最底10%从0变浓至1
    //云顶边界由 Profile LUT 顶部归零负责，此处不再做顶部 fade（避免与 LUT 双倍削顶）

    float cloudDensity=remap(basicCloudNoiseWithCoverage,detailNoiseMixByHeight,1.0,0.0,1.0);
    //remap(..., detailMix, 1, 0, 1)，从 detailMix 到 1 映射成 [0,1]：细节区相当于「挖掉」一部分形状 → 边缘侵蚀、镂空

    // 低云（3D+LUT）与高空云（2D）并进同一密度场，主 march / 自阴影共用 ×1000 光学深度。
    // max 避免重叠带双计 OD；高云有自己的高度带，不乘低云 densityShape。
    const float loDensity=cloudDensity*densityShape;
    const float hiDensity=highCloudDensity(posKm,normalizedHeight);
    return max(loDensity,hiDensity);
}

// Cloud shape end.
////////////////////////////////////////////////////////////////////////////////////////

struct ParticipatingMediaPhase//多散射相位结构体
{
    float phase[MsCount];//第0阶：主散射，强方向性，朝太阳有亮边
    //第1阶：更高近似，更均匀/漫射
};

ParticipatingMediaPhase getParticipatingMediaPhase(float basePhase,float basePhaseFactor)//多散射各阶的相位函数
{
    ParticipatingMediaPhase participatingMediaPhase;
    participatingMediaPhase.phase[0]=basePhase;//basePhase来自dualLobPhase(...)(云的双瓣相位)

    const float uniformPhase=getUniformPhase();//getUniformPhase来自shared_functions.glsl，各向同性散射，1/(4π)
    float MsPhaseFactor=basePhaseFactor;

    for(int ms=1;ms<MsCount;ms++)
    {
        participatingMediaPhase.phase[ms]=mix(uniformPhase,participatingMediaPhase.phase[0],MsPhaseFactor);
        MsPhaseFactor*=MsPhaseFactor;//更高阶更接近各向同性
    }

    return participatingMediaPhase;//返回多散射相位的结构体
}

float powder(float opticalDepth)//简化版Power Effect粉末效应/银边效应；参数opticalDepth光学深度，沿光路径的累积消光，无量纲或密度×距离）
{
    return pow(opticalDepth*20.0,0.5)*frameData.sky.atmosphereConfig.cloudPowderScale;//*20再取1/2次方，光学深度越深，背光提亮越多
}

// 已弃用：cloudColorCompute 不再调用这个函数——放弃了 Sky-View LUT 方案，天空背景色
// 改成直接采样 atmo pass 已经渲染好的 inHdrSceneColor（同一视线方向，见 cloudColorCompute
// 开头 skyBackgroundColor 的定义）。函数体保留只是为了不大动这段代码，供参考。
vec3 lookupSkylight(vec3 worldDir,vec3 worldPos,float viewHeight,vec3 upVector,ivec2 workPos,in const AtmosphereParameters atmosphere)
//worldPos/viewHeight/upVector 由调用方用行星中心为原点的坐标算出（见 cloudColorCompute），
//对地面到轨道/深空的任意相机位置都成立，函数本体不需要改
{
    //worldDir视线方向；worldPos观察者世界位置；viewHeight观察者据行星中心的高度；upVector当地的地面法向量；workPos像素坐标;AtmosphereParameters大气参数;lutimage SkyviewLUT纹理

    const vec3 sunDirection=-normalize(frameData.sky.direction);
    float viewZenithCosAngle=dot(worldDir,upVector);
    //视天顶角:视线与 向上 夹角的 cos：朝 horizon ≈ 0，天顶 ≈ 1

    vec3 sideVector=normalize(cross(upVector,worldDir));
    vec3 forwardVector=normalize(cross(sideVector,upVector));
    //在 垂直于up的水平面上建局部坐标系，用来描述太阳相对方位。



    vec2 lightOnPlane=vec2(dot(sunDirection,forwardVector),dot(sunDirection,sideVector));
    lightOnPlane=normalize(lightOnPlane);
    float lightViewCosAngle=lightOnPlane.x;
    //太阳在水平面上的方向:把太阳方向投影到该平面，得到太阳相对视线的方位角 的 cos（LUT 的第二维）

    vec2 sampleUv;
    vec3 luminance;

    skyViewLutParamsToUv(atmosphere,false,viewZenithCosAngle,lightViewCosAngle,viewHeight,vec2(textureSize(inSkyViewLut,0)),sampleUv);
    luminance=texture(sampler2D(inSkyViewLut,linearClampEdgeSampler),sampleUv).rgb;
    //skyViewLutParamsToUv：Bruneton / Unreal 风格，把 (高度, 视天顶角, 太阳-视线角) 映射到 LUT 的 2D UV
    //false：射线 不穿过地面（从空中看天）
    //linearClampEdgeSampler：LUT 用 clamp（不 repeat），边缘不 wrap

    return luminance;
}

// 云点环境天光已抽到 cloud_lighting.glsl（sampleCloudAmbient，与高空云共用，
// 保证两层环境光同源：同一 μs LUT、同一 L_sun 标定、同一 analytic 混合）。

////////////////////////////////////////////////////////////////////////////////////////
// phi_fwd 物理漫射场（各向同性多重散射）
//
// 物理链路：RTE → 扩散近似（τ≫1，g_eff→0）→ Helmholtz + 格林函数 G=exp(-κr)/(4πr)
//          → 有效源 Q_eff（慢衰减 + 局部散射沉积 + 边界置信度）→ 沿太阳方向 1D 离散。
//
// 与 MsCount 的 Frostbite 假 MS 是两回事：那边是「逐阶衰减的乘性方向性散射」，
// 这里是「独立的加性各向同性漫射场」，负责厚云内部/云底不指数变黑。两者叠加。
//
// 单位约定：cloudMap 返回的 density 直接就是 σ_t（m⁻¹）——volumetricShadow 的
// exp(-accum*1000) 说明 accum 是 density×km，即 OD = density × 米。所以本节所有
// OD / κ / 1/r 一律走「米」，光步的 km 步长必须先 ×1000。

// 单散射反照率：水云 0.999~0.9999，写死不进 UBO（美术无需调，改它等于换介质）。
#define kPhiFwdOmega0 0.999
// g_eff=0 时 σ_tr=σ_t，κ = σ_t·sqrt(3(1-ω₀))；sqrt 提出来当常数，逐步只需乘本步 OD。
#define kPhiFwdKappaODScale 0.05477225575  // sqrt(3*(1-0.999))
// 底部软化的归一化高度基准尺度（云壳归一化高度单位）；再按云柱厚度代理放大。
#define kPhiFwdBottomSoftBase 0.1

// 云顶高度代理：只取天气图 coverage 的大尺度趋势，用来判断「最近相关边界是否朝向太阳」。
// 刻意不含 curl / 风向高度错位 / 3D 噪声——这里不评估密度本身，只要一个平滑的高度场，
// 否则差分出来的法线会被高频噪声打成雪花。UV 与 cloudMap 同一套球面经纬度。
float phiFwdTopHeightProxy(vec3 posKm)
{
    vec3 sphDir=normalize(posKm);
    float lonRad=atan(sphDir.y,sphDir.x);
    float latRad=asin(clamp(sphDir.z,-1.0,1.0));
    vec2 sampleUv=vec2(lonRad*(1.0/(2.0*kPI))+0.5, latRad*(1.0/kPI)+0.5);
    sampleUv*=frameData.sky.atmosphereConfig.cloudWeatherUVScale;

    float weatherX=texture(sampler2D(inWeatherTexture,linearRepeatSampler),sampleUv).x;
    return saturate(frameData.sky.atmosphereConfig.cloudCoverage*weatherX);
}

// 边界项 B_eff 的两个输入，一次算完（共用同一批 4 次天气图采样）：
//   .x = C_top    云顶/侧向受光置信度 = wrap(N_boundary · L)
//   .y = 云柱厚度代理（0~1），供 C_bottom 的软化尺度用（厚云底部恢复更慢）
//
// 为什么需要 C_top：高反照率云里吸收极弱，光靠 T_abs 的距离衰减不足以让背光侧变暗。
// 真正决定的是边界条件——受光边界能往云里注入太阳能量，背光/逃逸边界不能。
vec2 phiFwdBoundaryTerms(vec3 posKm,vec3 sunDirection,const in AtmosphereParameters atmosphere)
{
    // 局部切平面基：径向 up + 两个正交切向。用球面基而不是世界 XZ——行星尺度下
    // 相机可以绕到任意纬度，世界 XZ 差分在两极会退化。
    vec3 up=normalize(posKm);
    vec3 refAxis=(abs(up.z)<0.99)?vec3(0.0,0.0,1.0):vec3(1.0,0.0,0.0);
    vec3 tangentA=normalize(cross(refAxis,up));
    vec3 tangentB=cross(up,tangentA);

    // 差分步长取云壳厚度量级：太小会放大天气图双线性插值的台阶，逐帧闪烁。
    float epsKm=max(atmosphere.cloudAreaThickness,0.1);

    float hL=phiFwdTopHeightProxy(posKm-tangentA*epsKm);
    float hR=phiFwdTopHeightProxy(posKm+tangentA*epsKm);
    float hD=phiFwdTopHeightProxy(posKm-tangentB*epsKm);
    float hU=phiFwdTopHeightProxy(posKm+tangentB*epsKm);

    // coverage 代理 → 云顶海拔（km）：coverage 越高云顶越高，跨度取整个云壳厚度。
    float slabKm=max(atmosphere.cloudAreaThickness,1e-3);
    float dHdA=(hR-hL)*slabKm/(2.0*epsKm);
    float dHdB=(hU-hD)*slabKm/(2.0*epsKm);

    // 切平面内的高度梯度 → 云顶法线（局部坐标下 up 相当于 +Y）。
    vec3 topNormal=normalize(up-tangentA*dHdA-tangentB*dHdB);

    // SSS 风格 wrap：背光侧不是硬切到 0，保留一点漫射（真实云边界并非全黑）。
    const float kWrap=0.5;
    float lit=saturate((dot(topNormal,sunDirection)+kWrap)/(1.0+kWrap));

    float conf=saturate(frameData.sky.atmosphereConfig.cloudPhiFwdBoundaryConfidence);
    float columnProxy=saturate((hL+hR+hD+hU)*0.25);

    return vec2(mix(1.0,lit,conf),columnProxy);
}

// bPhiFwd：只有太阳方向那条 shadow march 才累加漫射场。天空/环境那条必须传 false，
// 否则同一份能量会在 ambientLit 和 phi_fwd 两条路上各注入一次（双计发白）。
ParticipatingMedia volumetricShadow(vec3 posKm,vec3 sunDirection,const in AtmosphereParameters atmosphere,int fixNum,float msExtinctionFactor,bool bPhiFwd)
{
    ParticipatingMedia participatingMedia;

    int ms=0;

    float extinctionAccumulation[MsCount];
    float extinctionCoefficients[MsCount];

    for(ms=0;ms<MsCount;ms++)
    {
        extinctionAccumulation[ms]=0.0f;
        extinctionCoefficients[ms]=0.0f;
    }//各散射阶累积消光初始化为0

    const float StepLMul=frameData.sky.atmosphereConfig.cloudLightStepMul;//每步后步长倍增
    const uint StepLightNum=fixNum>0?fixNum:frameData.sky.atmosphereConfig.cloudLightStepNum;//步数
    float stepL=frameData.sky.atmosphereConfig.cloudLightBasicStep;//单位为km,初始步长

    float d=stepL*0.5;//从半步开始

    // ── phi_fwd 逐步状态（Intensity=0 时全程不参与运算，行为回到接入前）──────────
    const float phiIntensity=frameData.sky.atmosphereConfig.cloudPhiFwdIntensity;
    const bool bPhiFwdOn=bPhiFwd&&(phiIntensity>0.0);

    float phiFwd=0.0;      // φ：累积的漫射辐照度
    float phiTAbs=1.0;     // T_abs：无逃逸漫射源的吸收存活率（按 σ_a=(1-ω₀)σ_t 极慢衰减）
    float phiKappaSum=0.0; // ∫κ ds：观测点→当前步入口，用于 G 的体积衰减 exp(-∫κ ds)
    float phiODSum=0.0;    // ∫σ_t ds：各向同性建立权重 C_iso 用

    // 观测点属性（整条光路共用，见 phiFwdBoundaryTerms）：
    //   phiCTop       —— 云顶/侧向受光置信度（2D 高度场的属性，光步几 km 内变化可忽略）
    //   phiColumnProx —— 云柱厚度代理，供 C_bottom 的软化尺度用
    //   C_bottom 故意不放这里：它评估的是「源点离云底多近」，必须用光步采样点自身高度，
    //   否则观测点在云底、光步往太阳进入云芯时，源点在较高处但置信度用了观测点的低高度，
    //   会把整条光步的漫射场全部砍掉——正好抵消 phi_fwd 修「云底不黑」的初衷。
    float phiCTop=1.0;
    float phiColumnProxy=0.0;
    const float phiDepthPow=frameData.sky.atmosphereConfig.cloudPhiFwdDepthPow;
    const float phiDepthBias=frameData.sky.atmosphereConfig.cloudPhiFwdDepthBias;

    if(bPhiFwdOn)
    {
        vec2 boundary=phiFwdBoundaryTerms(posKm,sunDirection,atmosphere);
        phiCTop=boundary.x;
        phiColumnProxy=boundary.y;
    }

    for(uint j=0;j<StepLightNum;j++)
    {
        vec3 samplePosKm=posKm+sunDirection*d;//单位为km,sunDirection*d为往太阳的采样方向*距离+要计算透光率和消光的点的位置即得采样位置

        float sampleHeightKm=length(samplePosKm);  
        float sampleDt=sampleHeightKm-atmosphere.cloudAreaStartHeight;

        float normalizedHeight=sampleDt/atmosphere.cloudAreaThickness;//云内归一化高度
        vec3 samplePosMeter=samplePosKm*1000.0f;

        extinctionCoefficients[0]=cloudMap(samplePosMeter,normalizedHeight,cloudLodShadowDistScale());//由cloudMap()得到云密度
        extinctionAccumulation[0]+=extinctionCoefficients[0]*stepL;//第0阶消光累积，云密度*步长

        float MsExtinctionFactor=msExtinctionFactor;

        for(ms=1;ms<MsCount;ms++)
        {
            extinctionCoefficients[ms]=extinctionCoefficients[ms-1]*MsExtinctionFactor;
            MsExtinctionFactor*=MsExtinctionFactor;
            extinctionAccumulation[ms]+=extinctionCoefficients[ms]*stepL;

        }//多阶散射消光

        // ── phi_fwd：1D 格林函数离散 φ += T_abs·(Q/D)·C_iso·B_eff·exp(-∫κ ds)·(1/r) ──
        if(bPhiFwdOn)
        {
            // 全部换成米：σ_t 就是 cloudMap 的 density（m⁻¹），步长 km→m。
            float stepM=stepL*1000.0;
            float sigmaT=extinctionCoefficients[0];
            float localOD=sigmaT*stepM;                 // 本步光学厚度 ∫σ_t ds
            float qSrc=sigmaT*kPhiFwdOmega0*stepM;      // Q：局部散射沉积 σ_s·Δs
            float invD=sigmaT;                          // 1/D ∝ 3σ_tr；常数 3 吸收进 Intensity

            // κ 积到步中心（与 samplePosKm/d 一致的中点法则），而不是步末端。
            float kappaStep=localOD*kPhiFwdKappaODScale;
            float volAtten=exp(-(phiKappaSum+kappaStep*0.5));

            // C_iso：扩散近似只在方向记忆被打散后成立；边界第一步仍归方向性散射管。
            float msBuild=1.0-exp(-(phiODSum+localOD*0.5)
                *max(frameData.sky.atmosphereConfig.cloudPhiFwdMsBuildScale,0.0));

            // 1/r 几何扩展：r 用「观测点到本步采样点」的真实距离（米）。
            float invR=1.0/max(d*1000.0,stepM*0.5);

            // C_bottom：底部入射置信度，用「本步源点」自身高度评估——太阳基本不从云体
            // 下方注入，靠近云底的源不可信。厚云柱底部恢复更慢（H_soft 随厚度放大）。
            // 源点在云芯/云顶时 cBottom→1，不压制；只有真的贴近云底才压低。
            float cBottom=1.0;
            if(phiDepthPow>0.0)
            {
                float srcLocalHeight=(length(samplePosKm)-atmosphere.cloudAreaStartHeight)/atmosphere.cloudAreaThickness;
                float hAnchored=max(srcLocalHeight+phiDepthBias,0.0);
                float hSoft=max(kPhiFwdBottomSoftBase*mix(1.0,4.0,phiColumnProxy),1e-3);
                cBottom=1.0-exp(-hAnchored/hSoft*phiDepthPow);
            }

            phiFwd+=phiTAbs*qSrc*invD*msBuild*phiCTop*cBottom*volAtten*invR*phiIntensity;

            phiODSum+=localOD;
            phiKappaSum+=kappaStep;
            // T_abs 只按吸收衰减（ω₀=0.999 → 每单位 OD 只掉 0.1%），
            // 这正是「厚云内部不指数变黑」的来源：不能用 exp(-OD) 那条单散射透射率。
            phiTAbs*=exp(-localOD*(1.0-kPhiFwdOmega0));
        }

        d+=stepL;//采样坐标继续前进
        stepL*=StepLMul;//步长变大
    }

    for(ms=0;ms<MsCount;ms++)
    {
        participatingMedia.transmittanceToLight[ms]=exp(-extinctionAccumulation[ms]*1000.0);//转换成米单位
        //转换成透射率，Beer定律 e^(-τ)
        participatingMedia.extinctionAcc[ms]=extinctionAccumulation[ms]*1000.0;
    }
    participatingMedia.phiFwd=phiFwd;

    return participatingMedia;//返回参与介质结构体
}//各散射阶的 transmittanceToLight，决定该点是亮（顺光薄云）还是暗（厚云阴影里），是体积云自阴影的核心

// 沿主 march 射线参数 t(km) 查 cloudMap；空步中/终点探测用，不算光照。
float cloudDensityAtRayT(float rayTKm, vec3 worldPos, vec3 worldDir, const in AtmosphereParameters atmosphere)
{
    vec3 samplePos = rayTKm * worldDir + worldPos;
    float sampleHeight = length(samplePos);
    float normalizedHeight = (sampleHeight - atmosphere.cloudAreaStartHeight) / atmosphere.cloudAreaThickness;
    return cloudMap(samplePos * 1000.0, normalizedHeight, kCloudLodDistScaleView);
}

//主函数：对一条射线算整朵云，返回RGB散射和A透射
vec4 cloudColorCompute(const in AtmosphereParameters atmosphere,
vec2 uv,
float blueNoise,
inout float cloudZ,
ivec2 workPos,
vec3 worldDir,
bool bFog,
inout vec4 lightingFog,
float fogNoise,
inout vec3 airTransmittance)
{
    // Hi-Z：Phase 2 待接入。inHiz 目前绑定占位纹理，且这个值本就未参与任何 tMax 截断判断
    // （见文件末 ROADMAP 注释），故不再读取，避免采样一张没意义的占位深度。
    // float sceneZ=textureLod(sampler2D(inHiz,pointClampEdgeSampler),uv,3).r;

    // 行星尺度修正：frameData.camWorldPos 现在是"相机相对当前行星中心"的偏移（C++ 侧每帧
    // 减去 planetCenter 后写入，见 vk_atmosphere_lut.h::bake()），因此不再需要 flower 原版
    // 假设固定地面参考点的 +bottomRadius 沿 Y 偏移——worldPos 直接就是以行星中心为原点、
    // 长度即为相机到地心真实距离的大气空间坐标，适用于地面到轨道/深空的任意相机位置。
    vec3 worldPos=convertToAtmosphereUnit(frameData.camWorldPos.xyz,frameData);
    //convertToAtmosphereUnit(...)来自shared_functions.glsl
    //frameData.camWorldPos.xyz相机的世界坐标，来自frameData的UBO
    //
    // 坐标系说明（下面所有 vec3(0.0) 球心都不需要改成 pc.planetCenter，
    // 是已经确认的设计，不是漏改）：
    // frameData.camWorldPos 在 C++ 侧（vk_cloud_system.h::fillCloudFrameData）填充时，
    // 就已经是 camPos - earth.center（相机相对行星中心的偏移，米），也就是说"减去行星中心"
    // 这一步在 CPU 端做过一次了，worldPos 到这里已经是"以行星中心为原点"的大气空间坐标。
    // 所以本文件里所有 raySphereIntersect*(..., vec3(0.0), ...) 的球心用字面量 0 就是对的，
    // 不需要也不应该再额外减一次 pc.planetCenter（那样等于减了两次，反而是错的）。
    // 同理 viewHeight=length(worldPos) 就是相机到行星中心的真实距离，不需要另外换算成
    // camAlt/camDist——这就是 camAlt 本身，只是命名不同。

    lightingFog.w=-1.0f;//Fog通道的无效标记，目前还没算fog

    float earthRadius=atmosphere.bottomRadius;//行星表面半径（km），对应 C++ 侧 earth.radius
    float radiusCloudStart=atmosphere.cloudAreaStartHeight;//云带内壁半径 = surfaceRadius + cloudMinAlt
    float radiusCloudEnd=radiusCloudStart+atmosphere.cloudAreaThickness;//云带外壁半径 = surfaceRadius + cloudMaxAlt

    float viewHeight=length(worldPos);//相机到行星中心的真实距离（km），即相机高度

    float tMin;
    float tMax;//沿着worldDir,云ray march的起止距离，单位km

    bool bEarlyOutCloud=false;//若为true,则本像素不算云(无云、穿地、太远)

    if(viewHeight<radiusCloudStart)//相机在云底之下（含地面）
    {
        //相机在云底之下的情况
        // 已修复（第二次）：贴地精度问题比想象的严重——球面求交公式里要对 |r0-s0|² 和 sR²
        // 这种~6371²(4000万)量级的数字相减，float32 在这个量级下绝对误差可能到几米甚至
        // 几十米（32 位浮点大约 7 位有效数字，6371² 的 ULP 已经有好几个 km²），单靠给 tEarth
        // 一个固定的小容差（之前给的 0.001km=1米）远远不够，仰角稍微大一点就又被误判成撞地。
        // 改成更稳的判断：先看射线方向和当地"上方"（行星径向朝外）夹角——只要射线朝上方
        // 有分量（dot>0，视线在当地地平线以上），物理上就不可能撞到自己脚下的地面，
        // 不需要再看那个在这个尺度下已经不可靠的求交距离数值。只有明确朝下方看
        // （dot<=0，地平线以下）才需要用求交距离进一步确认。
        vec3 localUp=normalize(worldPos);
        float tEarth = raySphereIntersectNearest(worldPos, worldDir, vec3(0.0), earthRadius);
        if(dot(worldDir,localUp)<=0.0 && tEarth > 0.0)//视线朝地平线以下，且确实求交命中，才算撞地
        {
            // Intersect with earth, pre-return.
            bEarlyOutCloud = true;
        }

        tMin = raySphereIntersectInside(worldPos, worldDir, vec3(0.0), radiusCloudStart);
        tMax = raySphereIntersectInside(worldPos, worldDir, vec3(0.0), radiusCloudEnd);
        //相机在云底之下,视线向上穿云时：从进入云底到离开云顶壳的弧段

    }

    else if(viewHeight>radiusCloudEnd)//相机在云顶球壳之外（含轨道/深空）
    {
        // Eye out of cloud area.

        vec2 t0t1 = vec2(0.0);
        const bool bIntersectionEnd = raySphereIntersectOutSide(worldPos, worldDir, vec3(0.0), radiusCloudEnd, t0t1);
        if(!bIntersectionEnd)//射线与云壳没相交
        {
            // No intersection.
            bEarlyOutCloud = true;
        }

        vec2 t2t3 = vec2(0.0);
        const bool bIntersectionStart = raySphereIntersectOutSide(worldPos, worldDir, vec3(0.0), radiusCloudStart, t2t3);
        if(bIntersectionStart)//射线完全穿云顶和云底
        {
            tMin = t0t1.x;
            tMax = t2t3.x;
        }
        else//射线只擦到云顶一段
        {
            tMin = t0t1.x;
            tMax = t0t1.y;
        }
    }
    else
    {
        // Eye inside cloud area.
        float tStart = raySphereIntersectNearest(worldPos, worldDir, vec3(0.0), radiusCloudStart);
        if(tStart > 0.0)//朝下看
        {
            tMax = tStart;
        }
        else//朝上看
        {
            tMax = raySphereIntersectInside(worldPos, worldDir, vec3(0.0), radiusCloudEnd);
        }

        tMin = 0.0f; // From camera.
    }

    tMin=max(tMin,0.0);
    tMax=max(tMax,0.0);

    if(tMax<=tMin||tMin>frameData.sky.atmosphereConfig.cloudTracingStartMaxDistance)//火箭模拟器的最大距离极大
    {
        bEarlyOutCloud=true;
    }//距离过远就跳过

    const float marchingDistance=min(frameData.sky.atmosphereConfig.cloudMaxTraceingDistance,tMax-tMin);//步进距离
    tMax=tMin+marchingDistance;

    const uint stepCountUnit = frameData.sky.atmosphereConfig.cloudMarchingStepNum; // Tuner: baseline步数
    const float segLenKm = (tMax - tMin); // km，实际云段长度

    // 第一层：固定步数的“步长过大”会导致掠射边缘粉末闪点。
    // 这里把步长做上限约束：当 segLen 很长时自动增加步数，让单步Δs不至于太大。
    const float stepTMinKm = max(1e-4f, frameData.sky.atmosphereConfig.cloudMarchStepMinKm);
    const float stepTMaxKm = max(stepTMinKm, frameData.sky.atmosphereConfig.cloudMarchStepMaxKm);
    const float stepTBase  = segLenKm / max(1.0f, float(stepCountUnit));
    const float stepT      = clamp(stepTBase, stepTMinKm, stepTMaxKm);

    const uint stepCountHardMax = uint(max(1, frameData.sky.atmosphereConfig.cloudMarchStepHardMax));
    const uint stepCount = min(stepCountHardMax, uint(ceil(segLenKm / max(stepT, 1e-6f))));

    // 第二层：壳内真空跳步。仅 density==0 时候选 emptyStep；中点+终点探测全 0 才跳。
    // fogNoise（Heitz 蓝噪声）抖动空步长，打破固定栅格纹；薄云(>0)一律细步+全光照。
    const bool bEmptySkip = (frameData.sky.atmosphereConfig.cloudMarchEmptySkip != 0);
    const float emptyStepKm = max(stepT, frameData.sky.atmosphereConfig.cloudMarchEmptyStepKm);
    const float emptyJitterAmp = clamp(frameData.sky.atmosphereConfig.cloudMarchEmptyTauMax, 0.0, 0.5);
    const float emptyJitterMul = 1.0 + emptyJitterAmp * (2.0 * fogNoise - 1.0);

    // 第三层：云缘密度梯度缩步。沿射线维护可变步长 marchStepT（初始=第一层 stepT）。
    // 检测相邻步光学深度增量跳变 |Δ(σ·Δs)|，在云缘/侵蚀边界把下一步步长缩小。
    // 算法同源 cloud.frag::edgeSharpness（L195），但用于改步长而非仅调粉末。
    const bool bEdgeRefine = (frameData.sky.atmosphereConfig.cloudMarchEdgeRefine != 0);
    const float edgeScaleMin = clamp(frameData.sky.atmosphereConfig.cloudMarchEdgeScaleMin, 0.005, 1.0);//最小缩小到0.005倍步长，最大1倍步长
    const float edgeGradLow  = max(1e-6f, frameData.sky.atmosphereConfig.cloudMarchEdgeGradLow);//云缘梯度最小值，小于它判定不在云的边缘，不需要变小步长
    const float edgeGradHigh = max(edgeGradLow, frameData.sky.atmosphereConfig.cloudMarchEdgeGradHigh);//大于它，则必须变小步长

    float sampleT=tMin+0.001*stepT;// slightly delta avoid self intersect，不从精确 tMin 开始，减少浮点/壳面自交

    //litter by blue noise
    sampleT+=stepT*blueNoise;//每像素随机偏移0~1步，减轻带状分层（banding）

    vec3 sunColor=frameData.sky.color*frameData.sky.intensity;//太阳RGB*强度
    vec3 sunDirection=-normalize(frameData.sky.direction);//从太阳照过来的光照方向

    float VoL=dot(worldDir,sunDirection);//视线·光向 cos，用于 相位函数 / powder / 银边

    //note:在我的工程中lightDir、cosTheta = dot(rayDir, lightDir)，需确认 frame.lightDir 与 flower 同向约定。

    // 场景 HDR（合成背景仍由 composite 读 inHdrSceneColor；这里只给 debug 用）。
    // 勿再当云的入射光照：太空俯视时经常是海面/大气肢蓝，会把云顶染蓝。
    vec3 sceneHdrColor=texture(sampler2D(inHdrSceneColor,pointClampEdgeSampler),uv).rgb;

    float transmittance=1.0;// 视线透射率 T，每步乘 stepTransmittance
    vec3 scatteredLight=vec3(0.0);//沿视线累积的云散射光
    //将在march循环中不断更新

    // dbgMode 11/13/14：加权平均中间量（在 !bEarlyOutCloud 的 march 里累加）
    float dbgSunVisAcc=0.0;
    float dbgSunVisWeight=0.0;
    float dbgPowderAcc=0.0;
    float dbgHeightAcc=0.0;
    float dbgSampleWeight=0.0;
    // dbgMode 16：单独累加 phi_fwd 的散射贡献（不与其它光照混在一起），
    // 用来确认漫射场本身是不是 0 / 有没有反过来主导画面。
    vec3 dbgPhiFwdAcc=vec3(0.0);

    // 环境天光：按云壳入口位置的当地 μs 查天空半球辐照度/π（见 sampleCloudAmbient）。
    // Flower 的 skyBackgroundColor 来自 Sky-View（视线天空色）；云管线 Sky-View 只 bakeFull
    // 一次会过期，这里用同源 Ambient（已 ×L_sun）代替，供雾散射等「环境填光」使用。
    vec3 cloudEntryPos=tMin*worldDir+worldPos;
    vec3 cloudEntryUp=normalize(cloudEntryPos);
    vec3 groundToCloudTransfertIsoScatter=sampleCloudAmbient(atmosphere,cloudEntryPos,sunDirection);
    vec3 skyBackgroundColor=groundToCloudTransfertIsoScatter;

    // ── 调试可视化（借用暂未使用的 cloudSunLitMapOctave 当调试模式开关，由 Cloud Tuner
    // 面板的 DebugMode 下拉框驱动）：直接输出某个中间量，排查"黑云"到底是哪一环出了 0/黑值。
    // 1=相机位置沿太阳方向的实时大气透射率；2=场景 HDR（像素后面看到的东西）；
    // 3=环境天光 Ambient；4=太阳强度*色（不经过任何大气衰减，验证曝光量级本身）。
    // 5=不透明度 1-T；6=散射光原始值；7=中点密度。
    // 厚云发暗诊断（与 6 同为 raw L、a=0，便于并排对比）：
    // 8=强制云自阴影 off（sunVis=skyVis=1）再积分散射；
    // 9=强制 atmosphereTransmittance=1 再积分；
    // 10=8+9 同时；11=视线加权平均 sunVis[0]（灰阶，厚暗区若近黑=自阴影杀死直射）。
    // 8/9 仍暗时：可见云顶往往本就 sunVis≈1，继续查 L_in 乘子与积分——
    // 12=强制 powder=1、phase=1、sunVis=1、T_atm=1，L_in=sunColor（测积分/Beer）；
    // 13=视线加权平均 powderEffect；14=加权平均 normalizedHeight；15=物理 Beer（禁用 Frostbite soft T）。
    // 16=只输出 phi_fwd 漫射场贡献（其余光照全部不累加）；PhiFwdIntensity=0 时应当全黑。
    // 注意：alpha 统一给 0.0，不是 1.0——合成公式是 result=背景×cloudColor.a+cloudColor.rgb，
    // alpha=1 会让 debug 数值叠加在满强度背景之上，数值一旦是 0 就会跟"完全没开调试"长得
    //一模一样，看不出到底是没生效还是真的算出 0。alpha=0 让 debug 数值完全替换背景，
    // 不含糊。
    int dbgMode=frameData.sky.atmosphereConfig.cloudSunLitMapOctave;
    if(dbgMode==1){ return vec4(realTimeTransmittance(worldPos,sunDirection,atmosphere,8),0.0); }
    if(dbgMode==2){ return vec4(sceneHdrColor,0.0); }
    if(dbgMode==3){ return vec4(groundToCloudTransfertIsoScatter,0.0); }
    if(dbgMode==7)
    {
        // 7=不经过整个 march 循环，直接在 tMin/tMax 中点采一次 cloudMap() 原始密度：
        // 排除"march 循环某处出问题"，只看密度函数本身在这个位置/方向是不是 0 或 NaN。
        // 绿色=正常密度值(×5 放大方便看)；红色=NaN/Inf（数值本身炸了）；
        // 黑色=bEarlyOutCloud（求交区间本身就无效，tMin/tMax 没找到合法范围）。
        if(bEarlyOutCloud){ return vec4(0.0,0.0,0.0,0.0); }
        float dbgT=0.5*(tMin+tMax);
        vec3 dbgSamplePos=dbgT*worldDir+worldPos;
        float dbgHeight=length(dbgSamplePos);
        float dbgNormHeight=(dbgHeight-atmosphere.cloudAreaStartHeight)/atmosphere.cloudAreaThickness;
        float dbgDensity=cloudMap(dbgSamplePos*1000.0f,dbgNormHeight,kCloudLodDistScaleView);
        if(isnan(dbgDensity)||isinf(dbgDensity)){ return vec4(1.0,0.0,0.0,0.0); }
        return vec4(0.0,saturate(dbgDensity*5.0),0.0,0.0);
    }
    if(dbgMode==4){ return vec4(frameData.sky.color*frameData.sky.intensity,0.0); }

    if(!bEarlyOutCloud)//主云ray march
    {
        //Combine backward and forward scattering to have details in all directions
        float phase=
            dualLobPhase(frameData.sky.atmosphereConfig.cloudPhaseForward,frameData.sky.atmosphereConfig.cloudPhaseBackward,frameData.sky.atmosphereConfig.cloudPhaseMixFactor,-VoL);
        //双瓣相位
        ParticipatingMediaPhase participatingMediaPhase=getParticipatingMediaPhase(phase,0.5);
        //计算介质结构体相位，0 阶用 phase；1 阶向各向同性混合


        // Average ray hit pos to evaluate air perspective and height fog.
        vec3 rayHitPos=vec3(0.0);//在这里声明命中位置
        // rayHitPos：沿视线 march 的「透射率加权平均命中位置」(km，大气空间坐标)。
        // 主循环里每步做 rayHitPos += samplePos * transmittance，越靠前、越亮的采样权重越大。
        // march 结束后除以 rayHitPosWeight，得到云层的代表深度，供 cloudZ 与空气透视(ToPoint)使用。

        float rayHitPosWeight=0.0;
        // rayHitPosWeight：与 rayHitPos 配套的权重累加和（各步 transmittance 之和）。
        // 若全程无云或 transmittance 为 0，则权重为 0，后面跳过空气透视。

        //Second evaluate transmittance due to participatingMedia
        // 下面两段在 march 循环外各查一次 Transmittance LUT，得到射线入口/出口处
        // 「太阳光穿过大气到达该点」的 RGB 透射率，循环内用 mix 线性插值，避免每步查表。
        // 已放弃 Transmittance LUT：改为在两个端点各做一次实时大气积分
        // （realTimeTransmittance，shared_atmosphere.glsl），复用 sampleMediumRGB 的
        // 指数密度模型——直接从该点出发朝太阳方向数值积分到大气顶，不需要预烘焙表，
        // 也没有"该传相机高度还是采样点高度"的参数化陷阱，因为压根没有 LUT 查表这一步，
        // 就是老老实实从这个点算到大气顶。
        vec3 atmosphereTransmittance0;
        // atmosphereTransmittance0：云 ray march 起点(sampleT)处，太阳直射光的大气 RGB 透射率 (0~1)。
        {
            vec3 samplePos=sampleT*worldDir+worldPos;
            // samplePos：march 第一个采样点的世界位置 (km)。
            // sampleT 已在 tMin 基础上加了蓝噪声抖动，避免壳面自交与带状分层。

            atmosphereTransmittance0=realTimeTransmittance(samplePos,sunDirection,atmosphere,8);
        }//在march入口算一次太阳方向 上，大气对阳光的 RGB 衰减
        vec3 atmosphereTransmittance1;
        // atmosphereTransmittance1：云 ray march 终点(tMax)处的大气 RGB 透射率，含义同 atmosphereTransmittance0。
        // 与 0 端点配对，供循环内 mix(..., sampleT/marchingDistance) 做沿射线线性插值。
        {
            vec3 samplePos=tMax*worldDir+worldPos;
            // samplePos：march 最后一个采样点（射线在云壳内的出口侧），单位 km。

            atmosphereTransmittance1=realTimeTransmittance(samplePos,sunDirection,atmosphere,8);
        }//在march出口算一次太阳方向 上，大气对阳光的 RGB 衰减
        // 为何只算两端再插值：云壳厚度相对大气尺度很薄，透射率沿短线段近似线性；
        // 每步都做 8 步积分成本较高，两端+mix 是性能/质量折中（原 flower 用 LUT 两端插值同理）。


        // groundToCloudTransfertIsoScatter = skyBackgroundColor;// mix(groundToCloudTransfertIsoScatter, skyBackgroundColor, sunDirection.y);
        const vec3 upScaleColor=sampleCloudAmbient(atmosphere,cloudEntryPos,sunDirection);
        // upScaleColor：云入口点当地环境天光（天空半球辐照度/π，昼夜由 μs 编码）。

        // marchStepT：第三层可变步长（km），每步末尾按 edgeSharpness 更新；第一层 stepT 为基准。
        float marchStepT = stepT;
        // prevStepCloudDensity：上一步 cloudMap 密度 σ；-1 表示尚无历史（循环首步）。
        float prevStepCloudDensity = -1.0;
        // prevCurStepT：上一步实际用于积分的 Δs（km），与 prevStepCloudDensity 配对算 Δ(σ·Δs)。
        float prevCurStepT = stepT;

        for(uint i=0;i<stepCount && sampleT<tMax;i++)
        // 主云体 ray march：沿视线从近到远逐步积分 in-scattering，同时累积透射率 T_view。
        // 物理模型：参与介质 RTE 的前向离散形式；每步把局部散射光叠到 scatteredLight，并衰减 transmittance。
        // 第二层：真空大步跳（循环末尾 stepAdvance）。
        // 第三层：marchStepT 在云缘自动缩小（循环末尾更新 marchStepT）。
        {
            const float curStepT = min(marchStepT, max(tMax - sampleT, 0.0));
            // curStepT：本步用于 Beer-Lambert 积分的有效步长 Δs（km）；≤ marchStepT，且不超过射线剩余长度。

            //World space sample pos,in km uint
            vec3 samplePos=sampleT*worldDir+worldPos;//采样位置
            // samplePos：当前步采样点世界坐标 (km)，大气空间（地心+offset）。

            float sampleHeight=length(samplePos);//采样高度
            // sampleHeight：采样点到地心距离 (km)。samplePos 已是行星中心为原点的坐标，
            // 这里不需要再减 pc.planetCenter。

            vec3 atmosphereTransmittance=mix(atmosphereTransmittance0,atmosphereTransmittance1,saturate(sampleT/marchingDistance));
            // atmosphereTransmittance：当前步太阳光的大气 RGB 透射率（两端实时积分结果线性插值，
            // 已弃用 Transmittance LUT，见 atmosphereTransmittance0/1 的定义处）。
            // 物理：太阳光到该采样点前，经 Rayleigh/Mie 的路径衰减；乘在 sunlightTerm / ambientLit 上。
            // dbg 9/10/12：去掉大气透射染色，隔离「厚云发暖/发暗是否来自 sunlightTerm×T_atm」。
            if(dbgMode==9||dbgMode==10||dbgMode==12)
            {
                atmosphereTransmittance=vec3(1.0);
            }
            // RocketSim3D：不移植 LUT 时，可仿 cloud.frag 用 getOpticalDepth + exp(-coeff*depth) 实时算。

            float normalizedHeight=(sampleHeight-atmosphere.cloudAreaStartHeight)/atmosphere.cloudAreaThickness;//归一化高度(samplePos-gCloudMinAlt)/(gCloudMaxAlt-gCloudMinAlt)
            // normalizedHeight：云壳内 0~1 归一化高度。0=云底，1=云顶；驱动 cloudMap 垂直 profile、powder、groundLit。

            //convert to meter
            vec3 samplePosMeter=samplePos*1000.0f;
            // samplePosMeter：cloudMap 输入为米；flower 噪声/weather 部分按 km 尺度，接口约定不一。

            float stepCloudDensity=cloudMap(samplePosMeter,normalizedHeight,kCloudLodDistScaleView);
            // stepCloudDensity：该点体积云密度 σ（无量纲标量，近似 σ_t 的密度因子）。
            // >0 才进入光照计算；薄云也完整算光，不跳过 shadow march。

            rayHitPos+=samplePos*transmittance;
            rayHitPosWeight+=transmittance;
            // 用当前视线透射率 transmittance 加权累加位置：越靠近相机、仍可见的云贡献越大。用当前步的视线透射率 transmittance 当权重，对采样位置做加权平均，得到一个近似亮度贡献中心
            // 用于 march 结束后算 cloudZ 与空气透视（见 637 行后）。
            //cloudZ是该像素云的代表深度，写入云深度RT，与场景inDepth比较，判断云在物体前/后
            //空气透视(Air perspective)，远处物体发蓝、发灰、对比度下降，物体颜色被大气色 洗淡，同时叠上一层雾色

            if(stepCloudDensity>0.0)
            {
                float opticalDepth=stepCloudDensity*curStepT*1000.0;//转化成米单位
                // opticalDepth τ_step = σ × Δs：本步沿视线的光学深度（米）。
                // curStepT(km)×1000 换米；Beer-Lambert 指数里的 ∫σ_t ds。

                //beer's lambert
                // Siggraph 2017's new step transmittance formula.
                float stepTransmittance=max(exp(-opticalDepth),exp(-opticalDepth*0.25)*0.7);
                // stepTransmittance：本步视线透射率 T_step（Frostbite / HZD SIGGRAPH 2017 近似）。
                // 纯 exp(-τ) 在大步长时会过暗；与 exp(-τ/4)*0.7 取 max，保留亮核、减轻「黑团」。
                // 仅在 ms==0 时乘入 transmittance，高阶散射只贡献光不额外扣透射（flower 约定）。
                // dbg 15：改回物理 Beer，排除 soft-T 在大 τ 上的能量异常。
                if(dbgMode==15)
                {
                    stepTransmittance=exp(-opticalDepth);
                }

                ParticipatingMedia participatingMedia=volumetricShadow(samplePos,sunDirection,atmosphere,-1,frameData.sky.atmosphereConfig.cloudMultiScatterExtinction,true);
                // participatingMedia：沿太阳方向短 march 得到的云自阴影 + 多散射阶透射率。
                // transmittanceToLight[0] 直射影深浅；[1] 高阶路径更 diffuse，背光不死黑。
                // fixNum=-1 表示用 cloudLightStepNum 默认步数。
                // RocketSim3D：cloud.frag 的 getOpticalDepth(lightDir) 等价思路，可保留其一。

                ParticipatingMedia participatingMediaAmbient;
                // participatingMediaAmbient：天空环境光的「自阴影」结果（与上面太阳 shadow march 对称的另一条路）。
                //
                // 【shadow march 是什么】
                // 从当前采样点 samplePos 出发，沿某一方向逐步走进云里，累加 cloudMap 密度 → 光学深度 τ；
                // 再算 T = exp(-τ) = 该方向来的光还能剩多少。这就是 volumetricShadow()（241 行）。
                // 太阳路：沿 sunDirection，回答「太阳直射光到这一点前被云挡了多少」。
                //
                // 【vec3(0,1,0)「向上」是什么】
                // flower 用地心坐标 + Y 轴朝上：局部「天顶」近似为世界 +Y。
                // 把 (0,1,0) 当作「环境天光从头顶照下来」的方向，沿 +Y 做 shadow march：
                // 从 samplePos 往云顶走，看头顶还有多少云挡在采样点与开阔天空之间。
                // 已改成径向 up：normalize(samplePos)（samplePos 已是行星中心为原点的坐标，
                // 见 cloudColorCompute 开头 worldPos 的构造），对球面任意位置都成立。
                //
                // 【这个变量代表什么】
                // 结构体同 ParticipatingMedia：transmittanceToLight[ms] = 第 ms 阶「天空方向」透射率。
                // 后面 skyVisibilityTerm = participatingMediaAmbient.transmittanceToLight[ms]，
                // 乘 ambientLit 得到「经云遮后的环境光」；云越厚头顶越暗，云底/云内更闷。
                // 仅在 cloudEnableGroundContribution 开启时才算（与 groundLit 同属地面/环境贡献一组）。

                if(frameData.sky.atmosphereConfig.cloudEnableGroundContribution!=0)
                {
                    participatingMediaAmbient=volumetricShadow(samplePos,normalize(samplePos),atmosphere,-1,SkyMsExtinction,false);
                    // 末位 false：天空路不累加 phi_fwd——漫射场的源已经由太阳路那条算过了，
                    // 这里再算一遍等于把同一份能量注入两次（ambientLit 和 phi_fwd 双计发白）。
                    // 天空方向用 SkyMsExtinction(0.5)系数衰减高阶，比太阳路径更「糊」。
                    // 若不做 ground contribution，可关此开关，省一半 light march。

                }

                // powder：互斥模式 Off / Legacy / Sugar（cloudPowderMode，默认 Legacy）。
                // Legacy → powderEffect 乘子（density×高×VoL）；Sugar → 改写 sunVisibility（见 ms 循环）；
                // Off/Sugar 时 powderEffect=1，避免与堆糖双计银边。
                float powderEffect=1.0;
                const int powderMode=frameData.sky.atmosphereConfig.cloudPowderMode;
                if(powderMode==CLOUD_POWDER_MODE_LEGACY)
                {
                    // depthProbability：厚度因子（非概率）。raw=density*8*Pow，clamp 到 Scale，再 √。
                    const float heightPowExponent=0.5;
                    float depthProbability=pow(clamp(stepCloudDensity*8.0*frameData.sky.atmosphereConfig.cloudPowderPow,0.0,frameData.sky.atmosphereConfig.cloudPowderScale),heightPowExponent);
                    depthProbability+=0.05;
                    float verticalProbability=pow(remap(normalizedHeight,0.07,0.22,0.1,1.0),0.8);
                    powderEffect=powderEffectNew(depthProbability,verticalProbability,VoL);
                }
                if(dbgMode==12)
                {
                    powderEffect=1.0;
                }
                if(dbgMode==13||dbgMode==14)
                {
                    dbgPowderAcc+=powderEffect*transmittance;
                    dbgHeightAcc+=normalizedHeight*transmittance;
                    dbgSampleWeight+=transmittance;
                }

                
                // Amount of sunlight that reaches the sample point through the cloud 
                // is the combination of ambient light and attenuated direct light.
                vec3 sunlightTerm=atmosphereTransmittance*frameData.sky.atmosphereConfig.cloudShadingSunLightScale*sunColor;
                // sunlightTerm：到达采样点的「直射太阳光」RGB（大气衰减 × 艺术缩放 × 太阳色）。
                // 尚未乘云影、相位、粉末；后面与 sunVisibilityTerm、phase、powder 组合。

                // groundLit：仅云底地面反弹（艺术简化），不是第二条环境天光。
                // Flower 原式 mix(视线天空, 天顶辐照, …) 在高处落到 skyBackground；RS3D 曾把
                // skyBackground 换成 HDR，太空俯视就把海/肢蓝灌进云顶。这里改为：
                //   反弹 ≈ groundAlbedo × Ambient（与 ambientLit 同源、已含 L_sun）
                //   × nearGround（GroundNoiseScale−h：云底强、云顶→0）
                //   × 高度遮挡项 × FogFade
                // 环境天光只走下面的 ambientLit，避免与 groundLit 在云顶双重叠加。
                float nearGround=saturate(frameData.sky.atmosphereConfig.cloudNoiseScale-normalizedHeight);
                vec3 groundLit=(atmosphere.groundAlbedo*groundToCloudTransfertIsoScatter)
                    *nearGround
                    *saturate(1.0-GroundOcc+normalizedHeight)
                    *frameData.sky.atmosphereConfig.cloudFogFade;

                vec3 ambientLit=upScaleColor*powderEffect;
                // ambientLit：云内环境光（天光经云自阴影后的散射照明）。
                // 不乘 atmosphereTransmittance：那是「太阳方向」的路径衰减，而环境光走的是
                // 整个天空半球；upScaleColor 本身就是大气积分的结果，已含各方向衰减，
                // 再乘一遍等于重复衰减，低太阳角时会把环境光额外压暗并染上太阳路径的红。
                // 昼夜/太阳高度由 sampleCloudAmbient 的 μs LUT 编码，不再乘 (1-μs) 正午压制。
                // 乘 powderEffect：Legacy 模式下环境路也带一点粉末感。

                // ── 介质光学系数：把 cloudMap 密度变成 RTE 里的 σ_s、σ_t ──
                // 参与介质渲染方程（沿视线）：dL/ds = -σ_t·L + σ_s·L_in，dT/ds = -σ_t·T
                // σ_t 消光系数：每走单位路程，光被「拿走」多少（散射+吸收）
                // σ_s 散射系数：被拿走的部分里，有多少重新散射进别的方向
                // 单散射 albedo ω = σ_s/σ_t；水云 ω≈1，即几乎只散射不吸收
                float sigmaS=stepCloudDensity;
                // 此处先把 cloudMap 输出当作 σ_t 的密度标量（flower 约定：密度即消光强度）。
                // 名字 sigmaS 是历史命名；真正用于积分的 σ_s 在 683 行还要 × albedo。

                float sigmaE=max(sigmaS,1e-8f);
                // sigmaE = σ_t（0 阶消光系数）。max(...,1e-8) 防 731 行 stepScatter 分母除零。
                // 尚未拆 albedo：σ_t 用原始密度；σ_s 用 σ_t×ω（见下）。

                vec3 scatteringCoefficients[MsCount];
                float extinctionCoefficients[MsCount];
                // 多散射 Frostbite 近似（MsCount=2）：虚构两层介质，0 阶=真实云，1 阶=更弱更漫的「假层」。
                // scatteringCoefficients[ms]：该阶 σ_s（vec3，RGB 可略有色散）
                // extinctionCoefficients[ms]：该阶 σ_t（标量）
                // 后面 743 行：scatteredLitStep = L_in × σ_s；731 行分母用 σ_t。

                vec3 albedeo=frameData.sky.atmosphereConfig.cloudAlbedo;
                // albedeo = ω：单次碰撞时散射能量占消光的比例，RGB 可不同（水云近 vec3(1)）。
                // 物理：σ_s = σ_t × ω，σ_a = σ_t × (1-ω) 吸收项；ω=1 则 σ_s=σ_t。

                scatteringCoefficients[0]=sigmaS*albedeo;
                // 0 阶 σ_s = 密度 × ω：有多少光会被散射出去（进 sunSkyLuminance 那条路）。
                extinctionCoefficients[0]=sigmaE;
                // 0 阶 σ_t = 密度：视线穿过本步时 BOTH 散射与吸收都按此衰减（stepTransmittance）。
                // 例：density=0.5, albedo=0.9 → σ_t=0.5, σ_s=0.45，5% 等价吸收。
                // RocketSim3D cloud.frag 721-724：sigmaE0=gCloudExtinction*density，sigmaS0=sigmaE0*cloudAlbedo，同一套分离。

                float MsExtinctionFactor=frameData.sky.atmosphereConfig.cloudMultiScatterExtinction;
                float MsScatterFactor=frameData.sky.atmosphereConfig.cloudMultiScatterScatter;
                // 高阶相对 0 阶的衰减因子（通常 <1），每阶再平方 → 更快趋近各向同性。

                int ms;
                for(ms=1;ms<MsCount;ms++)
                {
                    extinctionCoefficients[ms] = extinctionCoefficients[ms - 1] * MsExtinctionFactor;
                    scatteringCoefficients[ms] = scatteringCoefficients[ms - 1] * MsScatterFactor;
                    
                    MsExtinctionFactor *= MsExtinctionFactor;
                    MsScatterFactor    *= MsScatterFactor;
                }
                // 生成 1 阶系数：更弱、更 diffuse 的「虚拟」介质层，模拟多次散射亮部。

                // ── phi_fwd 各向同性漫射场注入（加性，与下面的 ms 循环正交）──────────
                // 放在 ms 循环之外、之前，理由有二：
                //   1) 只加一次。φ 已经是「多次散射后的各向同性辐照度」，本身不分阶；
                //      塞进 ms 循环会按阶数重复计入，等于凭空放大 MsCount 倍。
                //   2) 此刻 transmittance 还是本步入口的 T_view（ms==0 分支才会乘 stepTransmittance），
                //      与 Flower 体积积分式对 T_view 的取值约定一致。
                // 不乘 phase / powder：φ 是各向同性的，方向性瓣和粉末银边是方向性散射那条路的事。
                if(participatingMedia.phiFwd>0.0)
                {
                    // 软饱和：扩散场在厚云芯会持续累积，线性注入容易过曝。
                    // (1-exp(-x·c))/c 在 c→0 时退化为 x，所以 Compress=0 就是「不压缩」。
                    float phiCompress=frameData.sky.atmosphereConfig.cloudPhiFwdCompress;
                    float phiMapped=(phiCompress>0.0)
                        ?(1.0-exp(-participatingMedia.phiFwd*phiCompress))/phiCompress
                        :participatingMedia.phiFwd;

                    vec3 phiLit=sunlightTerm*phiMapped;
                    vec3 phiScatter=transmittance
                        *(phiLit-phiLit*stepTransmittance)
                        /max(1e-4f,extinctionCoefficients[0])
                        *scatteringCoefficients[0];
                    // 与 stepScatter 同一条体积积分式（ΔL = T_view × J × (1-T_step) / σ_t），
                    // 只是 L_in 换成漫射场；σ_t/σ_s 都取 0 阶（真实云那一层）。

                    scatteredLight+=phiScatter;

                    // dbg 16 单独留一份，用来把漫射场从其它光照里摘出来看（dbgMode 是 uniform，不产生分支发散）。
                    if(dbgMode==16)
                    {
                        dbgPhiFwdAcc+=phiScatter;
                    }
                }

                // 天空路云遮挡（仅 0 阶环境光用，与 groundLit 同属地面/环境一组）：
                // 沿当地天顶 shadow march 得 T∈[0,1]，云越厚头顶越暗。
                float skyVisibilityTerm=1.0;
                if(frameData.sky.atmosphereConfig.cloudEnableGroundContribution!=0)
                {
                    skyVisibilityTerm=participatingMediaAmbient.transmittanceToLight[0];
                    // dbg 8/10/12：环境/地面关掉，避免把「直射积分」测脏。
                    if(dbgMode==8||dbgMode==10||dbgMode==12)
                    {
                        skyVisibilityTerm=1.0;
                    }
                }

                for(ms=MsCount-1;ms>=0;ms--)// Should terminate at 0
                // 从高阶向 0 阶积分：先算模糊的高阶光，再叠 0 阶直射+地面。
                {
                    // ── 本 ms 阶：组装「照到当前采样点、再被云散射出去」的入射光 L_in ──
                    float sunVisibilityTerm=participatingMedia.transmittanceToLight[ms];
                    // sunVisibilityTerm：太阳路的云遮挡系数 T∈[0,1]（来自 volumetricShadow 沿 sunDirection）。
                    // 0=采样点与太阳之间云很厚，直射几乎全挡；1=之间几乎无云。
                    // ms=0 是真实云影；ms=1 是高阶近似，影更浅（多次散射把暗部提亮）。

                    // Sugar 模式：用光路 τ 套堆糖 energy=gain·Beer·(1-e^{-2τ})，替代纯 Beer。
                    // AllMs=0 时只改 ms0（推荐）；=1 时每阶都乘（易与 Frostbite Ms / phi_fwd 叠亮）。
                    if(powderMode==CLOUD_POWDER_MODE_SUGAR)
                    {
                        const bool applySugar=(frameData.sky.atmosphereConfig.cloudPowderSugarAllMs!=0)||(ms==0);
                        if(applySugar)
                        {
                            sunVisibilityTerm=powderSugarSunVisibility(
                                sunVisibilityTerm,
                                participatingMedia.extinctionAcc[ms],
                                VoL,
                                frameData.sky.atmosphereConfig.cloudPowderSugarGain,
                                frameData.sky.atmosphereConfig.cloudPowderSugarViewWeight);
                        }
                    }

                    // dbg 11：在强制影之前累加真实 sunVis[0]（只看直射影深，不看高阶）。
                    if(dbgMode==11&&ms==0)
                    {
                        dbgSunVisAcc+=sunVisibilityTerm*transmittance;
                        dbgSunVisWeight+=transmittance;
                    }
                    // dbg 8/10/12：关掉云自阴影，验证厚区 Mode6 发暗是否就是 volumetricShadow。
                    if(dbgMode==8||dbgMode==10||dbgMode==12)
                    {
                        sunVisibilityTerm=1.0;
                    }

                    // 组装本 ms 阶入射光 L_in（公式与高空云同源，见 cloud_lighting.glsl）：
                    //   太阳路 = 云影 × 直射 sunlightTerm × 相位 × 粉末
                    //   环境路（仅 ms==0 且开 GroundContribution）= 天空遮挡 × 环境天光 + 云底地面反弹
                    // 与 Frostbite / Flower 博文一致：环境光不进高阶 Ms，否则会抹平细节并（在
                    // 蓝天半球环境源下）把暗部二次染蓝。
                    vec3 sunSkyLuminance=cloudInScatterLuminance(
                        sunlightTerm, sunVisibilityTerm, participatingMediaPhase.phase[ms], powderEffect,
                        ambientLit, skyVisibilityTerm, groundLit,
                        (ms==0)&&(frameData.sky.atmosphereConfig.cloudEnableGroundContribution!=0)&&(dbgMode!=12));
                    // dbg 12：L_in 固定为太阳色（无 phase/powder），只测「不透明积分是否能到 ~L_sun」。
                    if(dbgMode==12)
                    {
                        sunSkyLuminance=sunVisibilityTerm*sunColor;
                    }

                    vec3 stepScatter=cloudScatterStep(
                        sunSkyLuminance, transmittance,
                        scatteringCoefficients[ms], extinctionCoefficients[ms], stepTransmittance);
                    // stepScatter：经典体积积分离散式 —— ΔL = T_view × (L_in × σ_s × (1 - T_step)) / σ_t。
                    // 与高空云共用（cloud_lighting.glsl::cloudScatterStep）。
                    scatteredLight+=stepScatter;
                    // 累加进像素总散射光 RGB。

                    if(ms==0)
                    {
                        transmittance*=stepTransmittance;
                        // 仅 0 阶更新视线透射；高阶不重复扣 T，避免过度变暗。
                    }                    
                }
            }
            if(transmittance<=0.001)
            {
                break;
                // 早期退出：后面几乎不透明，再 march 收益 < 成本。
                // RocketSim3D：cloud.frag 用 0.005 阈值，可按性能微调。

            }

            float stepAdvance = curStepT;
            if(bEmptySkip && stepCloudDensity <= 0.0)
            {
                const float remainKm = max(tMax - sampleT, 0.0);
                const float leapTargetKm = max(marchStepT, emptyStepKm * emptyJitterMul);
                const float leapKm = min(leapTargetKm, remainKm);
                if(leapKm > curStepT + 1e-6)
                {
                    const float dMid = cloudDensityAtRayT(sampleT + 0.5 * leapKm, worldPos, worldDir, atmosphere);
                    const float dEnd = cloudDensityAtRayT(sampleT + leapKm, worldPos, worldDir, atmosphere);
                    if(dMid <= 0.0 && dEnd <= 0.0)
                    {
                        stepAdvance = leapKm;
                    }
                }
            }

            // ── 第三层：为下一步更新 marchStepT，云边缘处减小步长 ─────────────────────────────────────
            // stepTauKm：本步光学深度增量 τ≈σ·Δs（density×km），与 cloud.frag 的 dC 同量纲。
            const float stepTauKm = stepCloudDensity * curStepT;
            if(bEdgeRefine)//如果开启边缘优化
            {
                float edgeSharpness = 0.0;
                // edgeSharpness∈[0,1]：0=密度平稳（云核内部），1=云缘/噪声边界（需细步）。
                if(prevStepCloudDensity >= 0.0)//采样到云密度大于0
                {
                    const float prevTauKm = prevStepCloudDensity * prevCurStepT;
                    // tauJumpPerKm：相邻步 τ 增量之差，除以当前步长，得到沿射线的归一化跳变强度。
                    const float tauJumpPerKm = abs(stepTauKm - prevTauKm) / max(curStepT, 1e-6);//防除0
                    edgeSharpness = smoothstep(edgeGradLow, edgeGradHigh, tauJumpPerKm);
                    // 真空↔有云：cloudMap 从 0 变 >0（或反向），必为边界，强制细步。
                    const bool bPrevCloud = prevStepCloudDensity > 0.0;//前一步是否有云
                    const bool bCurCloud  = stepCloudDensity > 0.0;//此步是否有云
                    if(bPrevCloud != bCurCloud)
                    {
                        edgeSharpness = 1.0;//从无云变有云或有云变无云，肯定是云边缘
                    }
                }
                // edgeScale：下一步步长相对第一层的缩放；edgeSharpness=1 → stepT×edgeScaleMin。
                const float edgeScale = mix(1.0, edgeScaleMin, edgeSharpness);//edgeSharpness越接近1，缩放越小
                marchStepT = max(stepTMinKm, stepT * edgeScale);//必须大于最小步长限定
            }
            else//不开就跳过
            {
                marchStepT = stepT;
            }

            prevStepCloudDensity = stepCloudDensity;
            prevCurStepT = curStepT;
            sampleT += stepAdvance;

        }

        // Apply some additional effect.
        // march 循环结束后：用加权平均命中点写 cloudZ，并对 scatteredLight 做空气透视（Inscatter ToPoint）。
        if(rayHitPosWeight>0.0)
        // rayHitPosWeight>0 表示本像素 march 中至少有过非零云密度且累加了权重；全无云则跳过。
        {
            // Get average hit pos.
            rayHitPos/=rayHitPosWeight;
            // rayHitPos：透射率加权平均云位置 (km，大气空间，行星中心为原点)。= Σ(samplePos×T) / Σ(T)。
            // 代表「这朵云在 3D 里大概在哪」，不是第一个/最后一个采样点。

            // 已修复（第二次，之前那版是错的）：camViewProj（camView/camProj 直接拷贝自
            // snap.view/proj）期望的是"浮动原点绝对世界坐标"，不是"相对相机"的坐标——
            // 证据见 terrain.vert：gl_Position = frame.proj*frame.view*vec4(worldPos,1.0)
            // 直接用绝对世界坐标 worldPos，相机位置是 lookAt() 构建 view 矩阵时内部处理的；
            // 那边"worldPos-frame.viewPos"（相机相对坐标）只是另外算出来给光照用的衍生量，
            // 根本不会喂给投影矩阵。上一版我把 rayHitPos 减了一次相机位置再投影，相当于
            // 手动减一次 + view 矩阵内部又减一次，两次相减在相机远离行星中心时（轨道视角）
            // 误差正比于相机到行星中心的距离，直接炸掉。
            //
            // 正确做法：用 camInvertView 的平移列恢复相机在这套绝对世界坐标系里的位置
            // （camInvertView*vec4(0,0,0,1) 就是相机世界坐标，标准技巧，不需要额外的 UBO
            // 字段），再加上"相机到云"这段（数值小、精度安全的）偏移，重建云的绝对世界坐标。
            // 这套世界坐标是 km 尺度（和 terrain.vert 的 worldPos 一致），不需要
            // convertToCameraUnit 的 km→米换算——那是当初照抄 flower"渲染空间用米"的
            // 假设，这里同样是错的，直接删掉。
            vec3 camWorldKm=(frameData.camInvertView*vec4(0.0,0.0,0.0,1.0)).xyz;
            vec3 offsetKm=rayHitPos-worldPos; // 相机→云，行星大气空间偏移 (km)，数值小
            vec3 cloudAbsKm=camWorldKm+offsetKm; // 浮动原点绝对世界坐标 (km)

            vec4 rayInH=frameData.camViewProj*vec4(cloudAbsKm,1.0);
            // 齐次裁剪空间：把代表点投到屏幕，用于深度缓冲。

            cloudZ =rayInH.z/rayInH.w;
            // cloudZ：该像素云的「代表深度」(NDC/clip z/w，标准深度)，写入云深度 RT（binding 14/16）。
            // 用途：与场景 inDepth 比较前后关系、1/4→全分辨率重建、TAA 重投影。
            // 不是云表面深度，是整段体积的单值近似。

            rayHitPos=offsetKm;
            // 后面 792 行 rayHitHeight 需要"相机到命中点"的偏移向量，正好是 offsetKm，
            // 直接复用（等价于原来晚一点做的 rayHitPos-=worldPos，只是不再影响上面的投影）。

            //apply air perspective
            // 物理空气透视（与场景 ToPoint 同构）：
            //   L' = L_air×L_sun + T_air×L_cloud
            //   T_air → 独立 RGB RT；composite：src×(T_air×T_cloud)+L'
            // AirPerspectiveScale∈[0,1]：整段 AP 强度（0=关，1=全物理）。
            // AirPerspectiveSunScale：只乘 L_air。
            if(frameData.sky.atmosphereConfig.cloudAirPerspectiveEnabled != 0)
            {
                vec3 cloudHitPos=worldPos+rayHitPos; // 行星中心相对 (km)
                CloudAirPerspective ap=cloudAirPerspectiveToPoint(
                    worldPos, cloudHitPos, worldDir, sunDirection, atmosphere);

                float apScale=saturate(frameData.sky.atmosphereConfig.cloudAirPerspectiveScale);
                vec3 Lsun=frameData.sky.color*frameData.sky.intensity;
                vec3 T_air=mix(vec3(1.0), ap.transmittance, apScale);
                vec3 L_air=ap.scattered*Lsun
                    *frameData.sky.atmosphereConfig.cloudAirPerspectiveSunScale*apScale;

                scatteredLight=L_air+T_air*scatteredLight;
                airTransmittance=T_air;
            }
        }
    }
    //===================================================
    if(bFog)//体积雾/God Ray
    // bFog=true 时走独立 pass：低层指数雾 + 云影遮挡 + 太阳 God Ray，与主云体 march 并行/可选。
    // 输出 lightingFog：.xyz 雾散射 RGB，.w 雾透射率；写入 binding 22 等云雾 RT。
    // 重要：worldPos 是行星中心相对坐标（km），绝不能再用 flower 的 worldPos.y / sunDirection.y
    // 平面假设——那会把「到云底」算成数千公里，σ·Δs 爆掉、T→0，composite 里 result*=fog.a 整天空涂黑。
    {
        const uint GodRaySteps=36;
        // 与 composite 近距 God Ray（400m）对齐的短雾兜底；仰视云底时用球壳求交距离。
        const float kFogNearFallbackKm=0.4;
        // 地平线掠射到云底可达数百 km；不封顶则近处密度×巨步长仍会把 T 打穿。
        const float kFogMarchMaxKm=20.0;

        vec3 camLocalUp=normalize(worldPos);
        float fogMarchKm=kFogNearFallbackKm;
        if(viewHeight<radiusCloudStart && dot(worldDir,camLocalUp)>0.0)
        {
            // 相机在云底球壳内且视线朝外：沿视线到云底壳的斜距（km），等价 flower 的
            // (cloudBase-y)/dir.y，但用球壳而非水平面。
            float tToCloudBase=raySphereIntersectInside(worldPos,worldDir,vec3(0.0),radiusCloudStart);
            if(tToCloudBase>0.0)
            {
                fogMarchKm=tToCloudBase;
            }
        }
        fogMarchKm=clamp(fogMarchKm,kFogNearFallbackKm,kFogMarchMaxKm);

        // 单位为米
        float stepLength=fogMarchKm*1000.0f/float(GodRaySteps);

        vec3 stepRay=worldDir*stepLength;
        // stepRay：每步位移向量 (米，因 stepLength 为米)。

        vec3 rayPosWP=frameData.camWorldPos.xyz+stepRay*(fogNoise+0.5);
        // rayPosWP：当前雾采样点世界坐标 (米，行星中心相对)。
        // 起点相机 + 半步 jitter：fogNoise∈[0,1] 来自调用方蓝噪声，减轻带状分层。

        vec3 transmittanceTotal=vec3(1.0);
        vec3 scatteredLightTotal=vec3(0.0,0.0,0.0);

        float miePhaseValue=hgPhase(atmosphere.miePhaseG,-VoL);
        float rayleighPhaseValue=rayleighPhase(VoL);

        for(uint i=0;i<GodRaySteps;i++)
        {
            vec3 P0=convertToAtmosphereUnit(rayPosWP,frameData);
            // P0：雾采样点大气空间 (km，行星中心原点)，与主云 march 同源。

            float p0Height=length(P0);
            vec3 p0Up=(p0Height>1e-4)?(P0/p0Height):camLocalUp;
            float sunElev=dot(sunDirection,p0Up);// 当地太阳高度 cos，替代 sunDirection.y

            float visibilityTerm=1.0;
            {
                const uint StepLight=8;
                float stepL=atmosphere.cloudAreaThickness/float(StepLight);//km
                // 太阳越低，穿过云壳路径越长；用当地径向 elev 替代 |sunDirection.y|。
                stepL=stepL/max(abs(sunElev),0.1);

                vec3 position=P0;
                // 采样点在云底以下：沿太阳方向抬到云底球壳，再做云影 march。
                if(p0Height<atmosphere.cloudAreaStartHeight && sunElev>0.0)
                {
                    float tLift=raySphereIntersectInside(P0,sunDirection,vec3(0.0),atmosphere.cloudAreaStartHeight);
                    if(tLift>0.0)
                    {
                        position+=sunDirection*tLift;
                    }
                }

                float d=stepL*0.01;
                float transmittanceShadow=0.0;

                for(uint j=0;j<StepLight;j++)
                {
                    vec3 samplePosKm=position+sunDirection*d;//单位为km
                    float sampleHeightKm=length(samplePosKm);// 径向高度，非 .y
                    float sampleDt=sampleHeightKm-atmosphere.cloudAreaStartHeight;
                    float normalizedHeight=sampleDt/atmosphere.cloudAreaThickness;
                    vec3 samplePosMeter=samplePosKm*1000.0f;

                    transmittanceShadow+=cloudMap(samplePosMeter,normalizedHeight,cloudLodShadowDistScale());
                    d+=stepL;
                }

                visibilityTerm=exp(-transmittanceShadow*stepL*1000.0);
            }

            // 太阳越低越向 1.0 mix，放松云影对 God Ray 的压制（日落光柱更长）。
            visibilityTerm=mix(visibilityTerm,1.0,saturate(1.0-sunElev*5.0));

            vec3 phaseTimesScattering=vec3(miePhaseValue+rayleighPhaseValue);

            vec3 atmosphereTransmittance;
            {
                float sampleViewHeight=p0Height;
                const vec3 upVector=p0Up;
                float viewZenithCosAngle=dot(sunDirection,upVector);
                vec2 sampleUv;
                lutTransmittanceParamsToUv(atmosphere,sampleViewHeight,viewZenithCosAngle,sampleUv);
                atmosphereTransmittance=texture(sampler2D(inTransmittanceLut,linearClampEdgeSampler),sampleUv).rgb;
            }
            float density=getDensity(distance(rayPosWP,frameData.camWorldPos.xyz));
            // getDensity：exp(-d×0.001)×系数×cloudGodRayScale；此处 d=到相机距离(米)。

            vec3 sigmaS=vec3(density);
            const float sigmaA=0.0;
            vec3 sigmaE=max(vec3(1e-6f),sigmaA+sigmaS);

            vec3 scatteredLitStep=(visibilityTerm*sunColor*phaseTimesScattering*atmosphereTransmittance+skyBackgroundColor)*sigmaS;

            vec3 stepTransmittance=exp(-sigmaS*stepLength);

            scatteredLightTotal+=transmittanceTotal*(scatteredLitStep-scatteredLitStep*stepTransmittance)/sigmaE;
            transmittanceTotal*=stepTransmittance;

            rayPosWP+=stepRay;
        }
        lightingFog.w=mean(transmittanceTotal);
        // lightingFog.w：RGB 透射率均值 → composite 用；须接近 1，否则天空被乘黑。
        // mean(vec3)：shared_functions.glsl，(R+G+B)/3。

        lightingFog.xyz=scatteredLightTotal;
    }

    //Dual mix transmittance
    // RGB = L'（含 AP）；A = T_cloud（云本身视线透射，不含 T_air）。
    // T_air 写入独立 RT；composite：src×(T_air×T_cloud)+L'
    vec4 result=vec4(scatteredLight,transmittance);

    if(any(isnan(result))||any(isinf(result)))
    {
        result=vec4(0.0,0.0,0.0,1.0);
        // NaN/Inf 保护：数值爆炸时视为「无云散射、完全透射」，避免污染 HDR 与 TAA history。
        // rgb=0 无散射；a=1 背景全透过。RocketSim3D cloud.frag 若加类似 guard 可同样处理。
    }
    // 5=不透明度(1-transmittance)：全黑=这个像素 march 全程没找到密度（bEarlyOutCloud 或
    // stepCloudDensity 恒为 0），问题在"找云"这一步；6=散射光原始值（未做任何后处理）。
    // 8/9/10/12/15：在对应光照覆盖下同样输出 raw L（与 6 对比厚区明暗）。
    // 11/13/14：中间量灰阶；需在有云 march 时才有累加（bEarlyOut 则为黑）。
    if(dbgMode==5){ return vec4(vec3(1.0-result.a),0.0); }
    if(dbgMode==6||dbgMode==8||dbgMode==9||dbgMode==10||dbgMode==12||dbgMode==15){ return vec4(result.rgb,0.0); }
    if(dbgMode==11)
    {
        float v=(dbgSunVisWeight>1e-6)?(dbgSunVisAcc/dbgSunVisWeight):0.0;
        return vec4(vec3(v),0.0);
    }
    if(dbgMode==13)
    {
        float v=(dbgSampleWeight>1e-6)?(dbgPowderAcc/dbgSampleWeight):0.0;
        return vec4(vec3(v),0.0);
    }
    if(dbgMode==14)
    {
        float v=(dbgSampleWeight>1e-6)?(dbgHeightAcc/dbgSampleWeight):0.0;
        return vec4(v,1.0-v,0.0,0.0); // 红=云顶 绿=云底
    }
    if(dbgMode==16)
    {
        // raw L：只含 phi_fwd 那一路的散射（16 号档在积分处已把它从 scatteredLight 里摘出来）。
        return vec4(dbgPhiFwdAcc,0.0);
    }
    return result;
    // cloudColorCompute 主返回值；bFog 时 lightingFog 走 inout 另路输出，不在 result 里。
}
#endif   




// ── 接入现状更新（本次改动） ──────────────────────────────────────────────
// 下面这一段和之后的 ROADMAP 大注释块是移植调研阶段留下的笔记，按 binding 0-30
// 的原计划已经真正接入（vk_cloud_system.h + vk_atmosphere_lut.h），不再是"参考不编译"：
//   • inWeatherTexture 仍是 vk_scene.h 生成的同一张地理天气图（保留原 weather map），
//     只是 cloudMap() 里的采样 UV 从 posKm.xz 平面坐标改成了球面经纬度。
//   • inTransmittanceLut / inSkyViewLut / inSkyIrradiance 现在是真实数据，由
//     vk_atmosphere_lut.h 每帧针对当前云所在行星重新烘焙（见该文件头注释）。
//   • inInscatterLut(ToPoint AP) / inSDSMShadowDepth+cascadeInfos / inHiz / inGBufferA
//     仍是 Phase 2 占位（中性哑资源），对应代码位置有单独注释标注，不影响主 march 路径。
// 以下历史笔记（Weather/3D 噪声/uThreshLo 等）针对的是"沿用 cloud.frag 老参数"的另一
// 条迁移路线，本次未采用，保留仅供参考。
//Weather：保留 cloud.frag 的球面 wUV，只借鉴 flower 的 localCoverage + weather.x 混合公式
//3D 噪声：可先继续用烘焙 uNoiseTex3D，不必立刻拆成 inBasicNoise / inDetailNoise
//保留 uThreshLo/Hi 等你已调好的参数，作为 flower 公式之上的额外一层
//若目标是 完整 flower 移植：按 binding 0–30 重建 descriptor + frameData UBO，并接受与现有 CloudTuneParams 的 breaking change。

/*
================================================================================
 ROADMAP — cloud_common.glsl 结构与数据流（参考注释，不参与编译逻辑）
================================================================================

【本文件在 flower 管线中的位置】
  外部输入 ──► cloudColorCompute() ──► 输出 RT ──► 后续 pass（不在本文件）

  [输入 Set0 binding]
    frameData(21)  inBasicNoise/Detail/Weather/Curl(6-9)
    inTransmittanceLut(10)  inSkyViewLut(20)  inInscatterLut(11)
    inSkyIrradiance(27)  inHiz(30)  blueNoise(SSBO Set2)

  [本文件核心]
    cloudMap ──┬──► volumetricShadow ──► cloudColorCompute ★
               └──────────────────────────► cloudColorCompute ★

  [输出]
    result(L,T)        → imageCloudRenderTexture(2)   RGB=scatteredLight A=transmittance
    cloudZ             → imageCloudDepthTexture(14)     云代表深度
    lightingFog(xyz,w) → imageCloudFogRenderTexture(22) bFog 时 God Ray

  [flower 后续 pass 不在本文件]
    上采样/重建(12-19) → HDR 合成(0-1): finalColor = result.rgb + background.rgb * result.a

  RocketSim3D：实际渲染走 cloud.frag；本文件为 flower 参考 + 公式库，未 #include。

--------------------------------------------------------------------------------
【文件结构 按行号】
  A  1-72    头、binding 0-30、shared_sampler/bluenoise include
  B  74-92   getDensity, MsCount, ParticipatingMedia struct, remap
  C  99-168  cloudMap() → 云密度标量
  D  lighting  getParticipatingMediaPhase, powderEffectNew, powderSugarSunVisibility, lookupSkylight
  E  241-297 volumetricShadow() → transmittanceToLight[MsCount] + phiFwd（各向同性漫射场）
       ← phiFwdTopHeightProxy / phiFwdBoundaryTerms（边界项 B_eff，天气图 coverage 差分）
  F  300-1014 cloudColorCompute() ★主入口
  G  1020+    移植笔记 + 本 ROADMAP

--------------------------------------------------------------------------------
【函数依赖】
  cloudMap
    ← remap, frameData, inWeatherTexture, inCloudCurlNoise, inBasicNoise, inDetailNoise
  volumetricShadow
    ← cloudMap, frameData.atmosphereConfig(步数/步长)
  cloudColorCompute
    ← cloudMap, volumetricShadow, powderEffectNew, getParticipatingMediaPhase
    ← lookupSkylight, getDensity, dualLobPhase, hgPhase, rayleighPhase
    ← convertToAtmosphereUnit, lutTransmittanceParamsToUv, raySphereIntersect*
    ← inTransmittanceLut, inSkyViewLut, inInscatterLut, inSkyIrradiance, inHiz

--------------------------------------------------------------------------------
【cloudMap 数据流  L99-168】
  输入: posMeter(m), normalizedHeight(0=云底,1=云顶)
  输出: cloudDensity

  posMeter ──► +wind×height×500 ──► posKm
  posKm.xz ──► inWeatherTexture ──► weatherValue.x ──┐
  posMeter.xz+time ──► inCloudCurlNoise ──► localCoverage ──┤──► coverage
  frameData.cloudCoverage ────────────────────────────────────┘
  normalizedHeight+weather.g ──► Profile LUT（Cu/Tcu/Cb）──► gradientShape
  posKm+windOffset ──► inBasicNoise × gradientShape ──► basicCloudNoiseWithCoverage
  posKm-windOffset ──► inDetailNoise ──► detailNoiseMixByHeight ──► remap侵蚀
  normalizedHeight ──► densityShape（仅底部淡入+全局密度，无顶部 fade）
  return max(cloudDensity×densityShape, highCloudDensity)  →  主 march stepCloudDensity

--------------------------------------------------------------------------------
【cloudColorCompute 入参/出参  L300】
  入: atmosphere, uv, blueNoise, worldDir, bFog, fogNoise, workPos
  出 inout: cloudZ, lightingFog
  出 return: result = vec4(scatteredLight, transmittance)

--------------------------------------------------------------------------------
【cloudColorCompute 阶段 0 — 初始化与求交  L310-431】

  sceneZ ← inHiz(uv)                          // Hi-Z 场景深度(本文件未用于截断tMax)
  worldPos ← convertToAtmosphereUnit(cam)+Y偏移 // 射线起点(km)；RS3D: camPos+planetCenter
  lightingFog.w ← -1                          // fog 未算标记

  viewHeight ← length(worldPos)
  三分支求交 → tMin, tMax (km):
    相机在云下: raySphereIntersectInside 云底/顶壳; 撞地→bEarlyOutCloud
    相机在云上: raySphereIntersectOutSide 云顶+云底
    相机在云内: tMin=0; tMax=朝下的云底交或朝上的云顶交
  clamp + marchingDistance + stepT
  sampleT ← tMin + jitter(blueNoise)

  sunColor, sunDirection, VoL ← frameData.sky
  skyBackgroundColor ← Ambient（sampleCloudAmbient；不再用 HDR / 过期 Sky-View）
  sceneHdrColor      ← inHdrSceneColor（仅 debug）
  transmittance ← 1.0;  scatteredLight ← 0
  groundToCloudTransfertIsoScatter ← sampleCloudAmbient

--------------------------------------------------------------------------------
【cloudColorCompute 阶段 1 — march 预备  L437-525】  (!bEarlyOutCloud)

  phase ← dualLobPhase(...)
  participatingMediaPhase ← getParticipatingMediaPhase(phase, 0.5)
  rayHitPos ← 0;  rayHitPosWeight ← 0
  atmosphereTransmittance0 ← LUT@sampleT入口
  atmosphereTransmittance1 ← LUT@tMax出口
  upScaleColor ← sampleCloudAmbient(cloudEntryPos)   // 单位太阳 LUT × L_sun × Exposure

--------------------------------------------------------------------------------
【cloudColorCompute 阶段 2 — 主 march 循环  L527-787】

  for i in 0..stepCountUnit:

    samplePos ← sampleT×worldDir + worldPos
    atmosphereTransmittance ← mix(T0,T1, sampleT/marchingDistance)
    normalizedHeight ← (sampleHeight - cloudStart) / cloudThickness
    stepCloudDensity ← cloudMap(samplePosMeter, normalizedHeight)

    rayHitPos       += samplePos × transmittance      // 加权命中位置
    rayHitPosWeight += transmittance

    if stepCloudDensity > 0:
      opticalDepth ← density × stepT × 1000
      stepTransmittance ← max(exp(-τ), exp(-τ/4)×0.7)   // Frostbite 步透射

      participatingMedia        ← volumetricShadow(samplePos, sunDirection, bPhiFwd=true)
      participatingMediaAmbient ← volumetricShadow(samplePos, vec3(0,1,0), bPhiFwd=false)  // 可选

      powderEffect ← Legacy? powderEffectNew(...) : 1
      // Sugar：ms 循环里 sunVis ← powderSugarSunVisibility(beer, τ, VoL, gain, viewW)

      sunlightTerm ← atmosphereTransmittance × sunColor × scale
      groundLit    ← albedo×Ambient × nearGround × FogFade   // 仅云底；受 GroundContribution
      ambientLit   ← upScaleColor × powder

      σ_t ← density;  σ_s ← density × albedo
      scatteringCoefficients[ms], extinctionCoefficients[ms]  // MsCount=2

      // phi_fwd 各向同性漫射场：加性、只加一次、不分 ms、不乘 phase/powder。
      // φ 由太阳那条 light march 沿路累积（Q_eff × B_eff × exp(-∫κ) × 1/r）。
      if φ > 0:                                              // PhiFwdIntensity=0 时恒为 0
        φ_mapped ← Compress>0 ? (1-exp(-φ×C))/C : φ          // 软饱和
        phiScatter ← transmittance × (sunlightTerm×φ_mapped)×(1-T_step) / σ_t[0] × σ_s[0]
        scatteredLight += phiScatter

      for ms = 1 → 0:
        sunSkyLuminance ← sunVisibilityTerm × sunlightTerm × phase[ms] × powder
        if ms==0:
                       + skyVisibilityTerm × ambientLit          // 可选；仅 0 阶
                       + groundLit                               // 同上开关；仅 0 阶 / 仅云底
        scatteredLitStep ← sunSkyLuminance × σ_s[ms]
        stepScatter ← transmittance × (J×(1-T_step)) / σ_t[ms]
        scatteredLight += stepScatter
        if ms==0: transmittance *= stepTransmittance

    if transmittance ≤ 0.001: break
    sampleT += stepT

  【单步关键变量对照】
    stepCloudDensity     cloudMap 密度
    stepTransmittance    本步视线透射 T_step
    sunVisibilityTerm    participatingMedia.transmittanceToLight[ms]  太阳云影
    skyVisibilityTerm    participatingMediaAmbient.transmittanceToLight[ms]  头顶云影
    sunSkyLuminance      入射光 L_in (太阳+环境+地面)
    scatteredLitStep     L_in × σ_s
    stepScatter          本步散射贡献 → 累加到 scatteredLight

--------------------------------------------------------------------------------
【cloudColorCompute 阶段 3 — 后处理  L789-853】

  rayHitPos /= rayHitPosWeight
  cloudZ ← (viewProj × rayHitPos).z/w          → 云深度 RT
  rayHitHeight ← length(rayHitPos - worldPos)
  airPerspective ← Inscatter ToPoint → L'=L_air+T_air×L，T_eff≈luma(T_air)×T_cloud

--------------------------------------------------------------------------------
【cloudColorCompute 阶段 4 — God Ray / bFog】

  步长：球壳求交到云底（clamp 0.4~20 km），禁止 worldPos.y 平面假设
  36步: rayPosWP 沿 worldDir 从相机前进
  每步:
    visibilityTerm ← exp(-∫cloudMap) 沿太阳 8步 shadow（径向高度 + 球壳抬升）
    density ← getDensity(到相机距离)   // 指数雾，非 cloudMap
    scatteredLitStep ← (visibility×sun×phase×atmTrans + skyBG) × σ_s
    scatteredLightTotal += 同主云积分式
  lightingFog.xyz ← scatteredLightTotal
  lightingFog.w   ← mean(transmittanceTotal)  // 须≈1，否则 composite 乘黑天空

--------------------------------------------------------------------------------
【cloudColorCompute 阶段 5 — 返回  L997-1014】

  result ← vec4(scatteredLight, transmittance)
  NaN/Inf → vec4(0,0,0,1)
  composite: finalColor = result.rgb + background.rgb * result.a

--------------------------------------------------------------------------------
【结构体与常量】
  ParticipatingMedia       transmittanceToLight[2], extinctionAcc[2]
  ParticipatingMediaPhase  phase[2]
  MsCount=2                Frostbite 双阶多散射
  SkyMsExtinction=0.5      天空路 shadow 高阶衰减
  GroundOcc=0.5            地面光遮挡

--------------------------------------------------------------------------------
【RocketSim3D cloud.frag 映射】
  cloudColorCompute     ↔  main() march 循环
  cloudMap              ↔  cloudDensity (+ geographic 坐标)
  volumetricShadow      ↔  getOpticalDepth (二选一)
  lookupSkylight        ↔  atmo pass 背景 (不移植 LUT)
  Transmittance LUT     ↔  exp(-coeff×depth) 实时
  Inscatter ToPoint AP  ↔  场景 atmo ToPoint（已接入）
  cloudZ + 深度 RT      ↔  无 (除非半分辨率+TAA)
  bFog / lightingFog    ↔  无 (atmo.frag 已有)
  result(L,T)           ↔  FragColor(color×α, α)  α=1-viewTrans

--------------------------------------------------------------------------------
【主线一句话】
  frameData+纹理 → 求交(tMin,tMax) → march: cloudMap→shadow→L_in→stepScatter
  → scatteredLight+transmittance → [cloudZ+空气透视] → result(L,T)
  可选 bFog → lightingFog

================================================================================
*/