#ifndef VOLUMETRIC_CLOUD_COMMON_GLSL
#define VOLUMETRIC_CLOUD_COMMON_GLSL

#extension GL_EXT_samplerless_texture_functions : enable

// Reference implement from https://www.slideshare.net/guerrillagames/the-realtime-volumetric-cloudscapes-of-horizon-zero-dawn.
// Reference from https://github.com/qiutang98/flower/blob/main/source/shader/cloud/cloud_common.glsl

#include "../common/shared_functions.glsl"
#include "../common/shared_atmosphere.glsl"

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

layout (set = 0, binding = 10) uniform texture2D inTransmittanceLut; // 大气透射率 LUT；采样点处太阳/天空大气衰减
layout (set = 0, binding = 11) uniform texture3D inFroxelScatter; // Froxel 体积散射 LUT；云与场景间的空气透视/高度雾

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

layout (set = 0, binding = 27) uniform textureCube inSkyIrradiance; // 天空辐照度立方体贴图；地面/向上环境光贡献（groundLit）

layout (set = 0, binding = 28) uniform texture2D inSDSMShadowDepth; // SDSM 级联阴影深度；云受太阳阴影影响
layout (set = 0, binding = 29) buffer SSBOCascadeInfoBuffer{ CascadeInfo cascadeInfos[]; }; // 阴影级联 SSBO；各 cascade 视锥与矩阵
//可读写的 storage 缓冲（SSBO），不是uniform
//块内有一个cascadeInfos[]可变长度数组成员，c++侧定义CascadeInfo结构体

layout (set = 0, binding = 30) uniform texture2D inHiz; // 层次化 Z（Hi-Z）；快速估算场景深度，提前终止云 ray march
//image2D和texture2D成对出现，image2D 用于 compute 的 imageStore 写入，texture2D 用于后续 pass 的 texture() 采样
//大括号{}定义interface block接口块的成员，大括号列出这块GPU缓存里有哪些成员



//用宏指定 Descriptor Set 编号，再引入公共采样器定义
#define SHARED_SAMPLER_SET 1
#include "../common/shared_sampler.glsl"
//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//note:现在是Set1 的 sampler3D uNoiseTex3D 等，layout 完全不同。替换时要 重排 descriptor，或改 cloud_common.glsl 的 binding 对齐现有 vk_descriptors.h


// 原版这里 #include shared_blue_noise.glsl，依赖 flower 自带的 Sobol/ranking/scrambling
// 三张预计算数据表（一整套单独的蓝噪声数据资产，本工程没有，也不属于"接入体积云算法"本身）。
// Phase 2 待接入：如果需要更高质量的时域抖动，再引入这套数据表。这次改用引擎已有的
// whangHashNoise()（shared_functions.glsl）代替 cloud_raymarching.glsl / cloud_composite.glsl
// 里对 samplerBlueNoiseErrorDistribution_128x128_OptimizedFor_2d2d2d2d() 的调用。

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

};//第 1 阶让云 背光面/云体内 不会死黑，更接近真实多次散射，代价只是多算一条 shadow march 和一个散射阶循环

float remap(float value,float originMin,float originMax,float newMin,float newMax)
{
    return newMin+(saturate((value-originMin)/(originMax-originMin))*(newMax-newMin));
}//saturate来自shared_functions.glsl用于把值钳制至[0,1]和cloud_basicnoise.glsl的remap()并不相同


//texture(sampler(纹理，采样器规则)，uv)是GLSL内置函数：用UV坐标从纹理中取色,返回值类型vec4
//采样规则：采样规则（怎么插值、怎么重复）
//uv:读哪一点，通常[0,1]可以超出

float cloudMap(vec3 posMeter,float normalizedHeight)//单位是米，根据位置和归一化高度计算云密度
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

    // curl 噪声同理换成局部东/北向 km 距离（对应 cloud_rs3d_common.glsl::worldNoiseKm 的思路），
    // 而不是绝对世界 xz（那样相机一动整张 curl 图案就跟着搬，行星尺度下明显不对）。
    float cosLat=max(cos(latRad),0.01);
    float eastKm=lonRad*(frameData.sky.atmosphereConfig.bottomRadius)*cosLat;
    float northKm=latRad*frameData.sky.atmosphereConfig.bottomRadius;
    float localCoverage =texture(sampler2D(inCloudCurlNoise,linearRepeatSampler),
    (frameData.appTime.x*cloudSpeed*50+vec2(eastKm,northKm)*1000.0)*0.000001+0.5
    ).x;
    //linearRepeatSampler在shared_sampler.glsl，在 Descriptor Set 1 上声明
    //linear线性滤波:UV 落在两个纹素之间时，做插值（2D 是双线性），而不是只取最近一格
    //Repeat重复寻址：UV 超出 [0, 1] 时 平铺重复，而不是 clamp 到边缘或变透明，UV = 1.3  →  等价于 0.3（循环）UV = -0.2 →  等价于 0.8

    localCoverage=saturate(localCoverage*3.0-0.75)*0.2;//* 3.0 - 0.75再saturate：提高对比度，大部分区域被压到0附近，把噪声拉成稀疏的局部云团,* 0.2：幅度缩小，避免盖掉全局 weather
    //大尺度云量靠天气图，curl 只负责 碎块、不均匀

    float coverage=saturate(Coverage*(localCoverage+weatherValue.x));
    //合成这一位置的最终覆盖率，Coverage全局云量参数，weatherValue.x天气图大尺度bias，三者相加再 × 全局coverage，最后saturate到[0,1]

    float gradientShape =remap(normalizedHeight,0.0,0.1,0.1,1.0)*remap(normalizedHeight,0.1,0.8,1.0,0.2);
    //云量在垂直方向上的分布（云底 / 云腰 / 云顶），两个remap相乘,高度 0.00~0.10：从 0.1 升到 1.0  → 云底逐渐变厚;高度 0.10~0.80：从 1.0 降到 0.2  → 中层最厚，往上变薄
    //中间鼓、底薄、顶薄 的积云垂直profile

    float basicNoise=texture(sampler3D(inBasicNoise,linearRepeatSampler),(posKm+windOffset)*vec3(frameData.sky.atmosphereConfig.cloudBasicNoiseScale)).r;
    //从烘焙好的3D基础噪声（Perlin-Worley，cloud_basicnoise.glsl 烘出来的）采样
    //(posKm + windOffset) * cloudBasicNoiseScale，posKm+windOffset位置+风动画，cloudBasicNoiseScale噪声频率/平铺周期
    //.r只读R通道

    float basicCloudNoise=basicNoise*gradientShape;
    //3D 噪声 × 垂直高度包络 = 带正确层高分布的云块形状

    float basicCloudNoiseWithCoverage=coverage*remap(basicCloudNoise,1.0-coverage,1.0,0,1);
    //把 coverage 叠到大形状噪声上,coverage越小云生成阈值越大云越少越碎，越高则反

    vec3 sampleDetailNoise=posKm-windOffset*0.15;
    //与基础噪声略反风偏移，避免完全对齐

    float detailNoiseComposite=texture(sampler3D(inDetailNoise,linearRepeatSampler),sampleDetailNoise*frameData.sky.atmosphereConfig.cloudDetailNoiseScale).r;
    //3DWorley细节噪声采样，从烘焙好的细节3D纹理（cloud_detailnoise.glsl）采样

    float detailNoiseMixByHeight=0.2*mix(detailNoiseComposite,1-detailNoiseComposite,saturate(normalizedHeight*10.0));
    //0.2是侵蚀强度上限，后续可参数化
    //前10%高度云底用一种侵蚀，高层云顶用反相侵蚀，x*detail+(1-x)(1-detail)=1-x+(2x-1)*detail


    float densityShape=saturate(0.01+normalizedHeight*1.15)*Density*
    remap(normalizedHeight,0.0,0.1,0.0,1.0)*
    remap(normalizedHeight,0.8,1.0,1.0,0.0);
    //0.01 + height*1.15：云底略薄，往上略增
    //remap(0, 0.1, 0, 1)：最底10%从0变浓至1
    //第二个remap：只在云顶衰减

    float cloudDensity=remap(basicCloudNoiseWithCoverage,detailNoiseMixByHeight,1.0,0.0,1.0);
    //remap(..., detailMix, 1, 0, 1)，从 detailMix 到 1 映射成 [0,1]：细节区相当于「挖掉」一部分形状 → 边缘侵蚀、镂空

    return cloudDensity*densityShape;//再乘垂直包络和 Density 全局缩放
    //返回值：该世界空间采样点的云密度标量（0 ≈ 无云，越大越厚）
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

float powderEffectNew(float depth,float height,float VoL)//是 Powder Effect（粉末/背光提亮）的完整版实用公式：让云在背光、厚、云底时不要太黑，并随朝向太阳变化
{
    //depth厚度;height垂直位置概率，云底、薄边更明显;VoL View和Light夹角的cos值
    float r = VoL * 0.5 + 0.5;//先把VoL映射到[0,1]
    r=r*r;
    height=height*(1.0-r)+r;//背光最强；顺光时减弱height约束，避免正面过曝
    return depth*height;
}

vec3 lookupSkylight(vec3 worldDir,vec3 worldPos,float viewHeight,vec3 upVector,ivec2 workPos,in const AtmosphereParameters atmosphere)
//已解决：inSkyViewLut 现在是 vk_atmosphere_lut.h 每帧针对当前行星实时重烘焙的真 LUT（非静态资源），
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

ParticipatingMedia volumetricShadow(vec3 posKm,vec3 sunDirection,const in AtmosphereParameters atmosphere,int fixNum,float msExtinctionFactor)
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

    for(uint j=0;j<StepLightNum;j++)
    {
        vec3 samplePosKm=posKm+sunDirection*d;//单位为km,sunDirection*d为往太阳的采样方向*距离+要计算透光率和消光的点的位置即得采样位置

        float sampleHeightKm=length(samplePosKm);  
        float sampleDt=sampleHeightKm-atmosphere.cloudAreaStartHeight;

        float normalizedHeight=sampleDt/atmosphere.cloudAreaThickness;//云内归一化高度
        vec3 samplePosMeter=samplePosKm*1000.0f;

        extinctionCoefficients[0]=cloudMap(samplePosMeter,normalizedHeight);//由cloudMap()得到云密度
        extinctionAccumulation[0]+=extinctionCoefficients[0]*stepL;//第0阶消光累积，云密度*步长

        float MsExtinctionFactor=msExtinctionFactor;

        for(ms=1;ms<MsCount;ms++)
        {
            extinctionCoefficients[ms]=extinctionCoefficients[ms-1]*MsExtinctionFactor;
            MsExtinctionFactor*=MsExtinctionFactor;
            extinctionAccumulation[ms]+=extinctionCoefficients[ms]*stepL;

        }//多阶散射消光

        d+=stepL;//采样坐标继续前进
        stepL*=StepLMul;//步长变大
    }

    for(ms=0;ms<MsCount;ms++)
    {
        participatingMedia.transmittanceToLight[ms]=exp(-extinctionAccumulation[ms]*1000.0);//转换成米单位
        //转换成透射率，Beer定律 e^(-τ)
        participatingMedia.extinctionAcc[ms]=extinctionAccumulation[ms]*1000.0;
    }
    
    return participatingMedia;//返回参与介质结构体
}//各散射阶的 transmittanceToLight，决定该点是亮（顺光薄云）还是暗（厚云阴影里），是体积云自阴影的核心

//主函数：对一条射线算整朵云，返回RGB散射和A透射
vec4 cloudColorCompute(const in AtmosphereParameters atmosphere,
vec2 uv,
float blueNoise,
inout float cloudZ,
ivec2 workPos,
vec3 worldDir,
bool bFog,
inout vec4 lightingFog,
float fogNoise)
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
    //note:我的工程中相机位置是frame.viewPos(km单位)而不是camWorldPos(m单位)
    //note:我的工程中行星中心是pc.planetCenter(浮点原点)而非vec3(0.0),求交球心同理
    //note:我的工程中坐标系为地心球坐标，camPos-ctr;而非Y-up+offfset坐标
    //note:我的工程中高度为camAlt=length(camPos-ctr)-surfaceRadius,而非length(worldPos)
    //(需要替换)射线起点用camPos(km单位)即可

    lightingFog.w=-1.0f;//Fog通道的无效标记，目前还没算fog

    float earthRadius=atmosphere.bottomRadius;//需替换成pc.surfaceRadius
    float radiusCloudStart=atmosphere.cloudAreaStartHeight;//需替换成pc.surfaceRadius + gCloudMinAlt
    float radiusCloudEnd=radiusCloudStart+atmosphere.cloudAreaThickness;//pc.surfaceRadius + gCloudMaxAlt

    float viewHeight=length(worldPos);//我的工程中高度camAlt已经减去surfaceRadius了，这里没减去,但可以换成camDist=length(camPos-ctr)

    float tMin;
    float tMax;//沿着worldDir,云ray march的起止距离，单位km

    bool bEarlyOutCloud=false;//若为true,则本像素不算云(无云、穿地、太远)

    if(viewHeight<radiusCloudStart)//需替换成camAlt<gCloudMinAlt或者camDist<pc.surfaceRadius + gCloudMinAlt
    {
        //相机在云底之下的情况
        float tEarth = raySphereIntersectNearest(worldPos, worldDir, vec3(0.0), earthRadius);
        if(tEarth > 0.0)//射线撞地，整个像素跳过，值得学习
        {
            // Intersect with earth, pre-return.
            bEarlyOutCloud = true;
        }

        tMin = raySphereIntersectInside(worldPos, worldDir, vec3(0.0), radiusCloudStart);
        tMax = raySphereIntersectInside(worldPos, worldDir, vec3(0.0), radiusCloudEnd);
        //相机在云底之下,视线向上穿云时：从进入云底到离开云顶壳的弧段
        //note:vec3(0.0)必须改成pc.planetCenter(ctr)
        

    }

    else if(viewHeight>radiusCloudEnd)//相机在云顶球壳之外
    {
        // Eye out of cloud area.

        vec2 t0t1 = vec2(0.0);
        const bool bIntersectionEnd = raySphereIntersectOutSide(worldPos, worldDir, vec3(0.0), radiusCloudEnd, t0t1);//依旧得改vec3(0.0)成pc.planetCenter(ctr)
        if(!bIntersectionEnd)//射线与云壳没相交
        {
            // No intersection.
            bEarlyOutCloud = true;
        }

        vec2 t2t3 = vec2(0.0);
        const bool bIntersectionStart = raySphereIntersectOutSide(worldPos, worldDir, vec3(0.0), radiusCloudStart, t2t3);//依旧得改vec3(0.0)成pc.planetCenter(ctr)
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
        float tStart = raySphereIntersectNearest(worldPos, worldDir, vec3(0.0), radiusCloudStart);//依旧得改vec3(0.0)成pc.planetCenter(ctr)
        if(tStart > 0.0)//朝下看
        {
            tMax = tStart;
        }
        else//朝上看
        {
            tMax = raySphereIntersectInside(worldPos, worldDir, vec3(0.0), radiusCloudEnd);//依旧得改vec3(0.0)成pc.planetCenter(ctr)
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

    const uint stepCountUnit=frameData.sky.atmosphereConfig.cloudMarchingStepNum;//后期可改成自适应步长
    const float stepCount=float(stepCountUnit);//云视线march步数
    const float stepT=(tMax-tMin)/stepCount;//步长

    float sampleT=tMin+0.001*stepT;// slightly delta avoid self intersect不要从精确 tMin 开始，减少浮点/壳面自交

    //litter by blue noise
    sampleT+=stepT*blueNoise;//每像素随机偏移0~1步，减轻带状分层（banding）

    vec3 sunColor=frameData.sky.color*frameData.sky.intensity;//太阳RGB*强度
    vec3 sunDirection=-normalize(frameData.sky.direction);//从太阳照过来的光照方向

    float VoL=dot(worldDir,sunDirection);//视线·光向 cos，用于 相位函数 / powder / 银边

    //note:在我的工程中lightDir、cosTheta = dot(rayDir, lightDir)，需确认 frame.lightDir 与 flower 同向约定。

    //Cloud background sky color
    vec3 skyBackgroundColor=lookupSkylight(worldDir,worldPos,viewHeight,normalize(worldPos),workPos,atmosphere);//该像素视线方向天空背景色
    //Sky View Lut不适合火箭模拟器，应采样atmo pass
    
    float transmittance=1.0;// 视线透射率 T，每步乘 stepTransmittance
    vec3 scatteredLight=vec3(0.0);//沿视线累积的云散射光
    //将在march循环中不断更新

    vec3 groundToCloudTransfertIsoScatter=texture(samplerCube(inSkyIrradiance,linearClampEdgeSampler),normalize(worldPos)).rgb;
    //从 天空辐照度 cubemap 取「当地径向向上」方向的环境光，用于后面 groundLit（云底、地面反射贡献）。
    //flower 原版固定采样 (0,1,0)（假设 Y 恒为当地向上，近地小范围内近似成立）；
    //行星尺度下相机可以在球面任意位置，改用 normalize(worldPos)（worldPos 已是行星中心为原点）
    //才是该点真正的当地向上方向。inSkyIrradiance 现由 vk_atmosphere_lut.h 每帧实时烘焙（binding 27）。

    // ── 调试可视化（借用暂未使用的 cloudSunLitMapOctave 当调试模式开关，由 Cloud Tuner
    // 面板的 DebugMode 下拉框驱动）：直接输出某个中间量，排查"黑云"到底是哪一级 LUT/项目
    // 出了 0/黑值。1=直接看太阳方向大气透射率 LUT；2=看天空视图 LUT 背景色；
    // 3=看天空辐照度 cubemap；4=看太阳强度*色（不经过任何 LUT，验证曝光量级本身）。
    int dbgMode=frameData.sky.atmosphereConfig.cloudSunLitMapOctave;
    if(dbgMode==1)
    {
        vec2 dbgUv;
        lutTransmittanceParamsToUv(atmosphere,viewHeight,dot(sunDirection,normalize(worldPos)),dbgUv);
        return vec4(texture(sampler2D(inTransmittanceLut,linearClampEdgeSampler),dbgUv).rgb,1.0);
    }
    if(dbgMode==2){ return vec4(skyBackgroundColor,1.0); }
    if(dbgMode==3){ return vec4(groundToCloudTransfertIsoScatter,1.0); }
    if(dbgMode==4){ return vec4(frameData.sky.color*frameData.sky.intensity,1.0); }

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
        // march 结束后除以 rayHitPosWeight，得到云层的代表深度，供 cloudZ 与空气透视(Froxel)使用。

        float rayHitPosWeight=0.0;
        // rayHitPosWeight：与 rayHitPos 配套的权重累加和（各步 transmittance 之和）。
        // 若全程无云或 transmittance 为 0，则权重为 0，后面跳过空气透视。

        //Second evaluate transmittance due to participatingMedia
        // 下面两段在 march 循环外各查一次 Transmittance LUT，得到射线入口/出口处
        // 「太阳光穿过大气到达该点」的 RGB 透射率，循环内用 mix 线性插值，避免每步查表。
        vec3 atmosphereTransmittance0;
        // atmosphereTransmittance0：云 ray march 起点(sampleT)处，太阳直射光的大气 RGB 透射率 (0~1)。
        // 物理含义：太阳光经 Rayleigh/Mie 散射与吸收后，还剩多少比例能照到云内采样点。
        {
            vec3 samplePos=sampleT*worldDir+worldPos;
            // samplePos：march 第一个采样点的世界位置 (km)。
            // sampleT 已在 tMin 基础上加了蓝噪声抖动，避免壳面自交与带状分层。

            float sampleHeight=length(samplePos);
            // sampleHeight：采样点到地心距离 (km)，即该点绝对高度（flower 地心+Y-up 坐标系）。
            // note: RocketSim3D 应改为 length(samplePos - pc.planetCenter)。

            const vec3 upVector=samplePos/sampleHeight;
            // upVector：采样点处当地「向上」单位向量（地心球坐标下为径向朝外）。
            // 用于在该点建立局部垂直参考，计算太阳相对天顶角。

            float viewZenithCosAngle=dot(sunDirection,upVector);
            // viewZenithCosAngle：太阳方向与当地 up 夹角的 cos（flower 命名沿用 LUT 接口）。
            // 虽叫 viewZenith，此处实际是 sunZenithCos：决定太阳光在大气中路径有多斜、路径多长。
            // cos≈1 正午顶光路径短；cos≈0 地平线附近路径长、衰减更重。

            vec2 sampleUv;
            // sampleUv：Transmittance LUT 的 2D 查表坐标，由高度+天顶角 cos 映射而来。

            lutTransmittanceParamsToUv(atmosphere,viewHeight,viewZenithCosAngle,sampleUv);
            // 将 (高度, 太阳天顶 cos) 编码为 LUT UV（Bruneton / Unreal 大气模型约定）。
            // note: 此处 height 传的是 viewHeight（相机高度），sampleHeight 未使用——flower 原版亦如此；
            // 更物理的做法是入口块传 sampleHeight。RocketSim3D 若不移植 LUT，可整段换 atmo pass 采样。

            atmosphereTransmittance0=texture(sampler2D(inTransmittanceLut,linearClampEdgeSampler),sampleUv).rgb;
            // 查表得 RGB 三通道透射率：R/G/B 各自衰减不同 → 日出日落偏红由此而来。

        }//在march入口算一次太阳方向 上，大气对阳光的 RGB 衰减
        vec3 atmosphereTransmittance1;
        // atmosphereTransmittance1：云 ray march 终点(tMax)处的大气 RGB 透射率，含义同 atmosphereTransmittance0。
        // 与 0 端点配对，供循环内 mix(..., sampleT/marchingDistance) 做沿射线线性插值。
        {
            vec3 samplePos=tMax*worldDir+worldPos;
            // samplePos：march 最后一个采样点（射线在云壳内的出口侧），单位 km。

            float sampleHeight=length(samplePos);
            // sampleHeight：出口采样点的地心距离 (km)。

            const vec3 upVector=samplePos/sampleHeight;
            // upVector：出口点当地径向「向上」。

            float viewZenithCosAngle=dot(sunDirection,upVector);
            // viewZenithCosAngle：出口点处的太阳天顶 cos（高处与低处 up 不同，cos 会略变）。

            vec2 sampleUv;
            lutTransmittanceParamsToUv(atmosphere,viewHeight,viewZenithCosAngle,sampleUv);
            // 同上；出口高度通常高于入口，大气透射率一般更大（光衰减更少）。

            atmosphereTransmittance1=texture(sampler2D(inTransmittanceLut,linearClampEdgeSampler),sampleUv).rgb;

        }//在march出口算一次太阳方向 上，大气对阳光的 RGB 衰减
        // 为何只算两端再插值：云壳厚度相对大气尺度很薄，透射率沿短线段近似线性；
        // 每步查 LUT 成本高，两端+mix 是 flower 的性能/质量折中（见 489 行 sunlightTerm）。


        // groundToCloudTransfertIsoScatter = skyBackgroundColor;// mix(groundToCloudTransfertIsoScatter, skyBackgroundColor, sunDirection.y);
        const vec3 upScaleColor=texture(samplerCube(inSkyIrradiance,linearClampEdgeSampler),normalize(worldPos)).rgb;
        // upScaleColor：天空辐照度 cubemap 在「当地径向向上」的 RGB，近似云底/云内收到的半球环境光强度
        // （同 453 行，行星尺度下不能固定采样 (0,1,0)）。
        // 物理：地面与低空大气把天光散射回云底；cube 预积分了半球 incoming radiance。

        for(uint i=0;i<stepCountUnit;i++)
        // 主云体 ray march：沿视线从近到远逐步积分 in-scattering，同时累积透射率 T_view。
        // 物理模型：参与介质 RTE 的前向离散形式；每步把局部散射光叠到 scatteredLight，并衰减 transmittance。
        {
            //World space sample pos,in km uint
            vec3 samplePos=sampleT*worldDir+worldPos;//采样位置
            // samplePos：当前步采样点世界坐标 (km)，大气空间（地心+offset）。

            float sampleHeight=length(samplePos);//采样高度
            // sampleHeight：采样点到地心距离 (km)。
            // RocketSim3D：改为 length(samplePos - pc.planetCenter)；归一化高度用 camAlt 云带。

            vec3 atmosphereTransmittance=mix(atmosphereTransmittance0,atmosphereTransmittance1,saturate(sampleT/marchingDistance));
            // atmosphereTransmittance：当前步太阳光的大气 RGB 透射率（两端 LUT 线性插值）。
            // 物理：太阳光到该采样点前，经 Rayleigh/Mie 的路径衰减；乘在 sunlightTerm / ambientLit 上。
            // RocketSim3D：不移植 LUT 时，可仿 cloud.frag 用 getOpticalDepth + exp(-coeff*depth) 实时算。

            float normalizedHeight=(sampleHeight-atmosphere.cloudAreaStartHeight)/atmosphere.cloudAreaThickness;//归一化高度(samplePos-gCloudMinAlt)/(gCloudMaxAlt-gCloudMinAlt)
            // normalizedHeight：云壳内 0~1 归一化高度。0=云底，1=云顶；驱动 cloudMap 垂直 profile、powder、groundLit。

            //convert to meter
            vec3 samplePosMeter=samplePos*1000.0f;
            // samplePosMeter：cloudMap 输入为米；flower 噪声/weather 部分按 km 尺度，接口约定不一。

            float stepCloudDensity=cloudMap(samplePosMeter,normalizedHeight);
            // stepCloudDensity：该点体积云密度 σ（无量纲标量，近似 σ_t 的密度因子）。
            // >0 才进入光照计算，省掉空步的 shadow march。

            rayHitPos+=samplePos*transmittance;
            rayHitPosWeight+=transmittance;
            // 用当前视线透射率 transmittance 加权累加位置：越靠近相机、仍可见的云贡献越大。用当前步的视线透射率 transmittance 当权重，对采样位置做加权平均，得到一个近似亮度贡献中心
            // 用于 march 结束后算 cloudZ 与空气透视（见 637 行后）。
            //cloudZ是该像素云的代表深度，写入云深度RT，与场景inDepth比较，判断云在物体前/后
            //空气透视(Air perspective)，远处物体发蓝、发灰、对比度下降，物体颜色被大气色 洗淡，同时叠上一层雾色

            if(stepCloudDensity>0.0)
            {
                float opticalDepth=stepCloudDensity*stepT*1000.0;//转化成米单位
                // opticalDepth τ_step = σ × Δs：本步沿视线的光学深度（米）。
                // stepT(km)×1000 换米；Beer-Lambert 指数里的 ∫σ_t ds。

                //beer's lambert
                // Siggraph 2017's new step transmittance formula.
                float stepTransmittance=max(exp(-opticalDepth),exp(-opticalDepth*0.25)*0.7);
                // stepTransmittance：本步视线透射率 T_step（Frostbite / HZD SIGGRAPH 2017 近似）。
                // 纯 exp(-τ) 在大步长时会过暗；与 exp(-τ/4)*0.7 取 max，保留亮核、减轻「黑团」。
                // 仅在 ms==0 时乘入 transmittance，高阶散射只贡献光不额外扣透射（flower 约定）。

                ParticipatingMedia participatingMedia=volumetricShadow(samplePos,sunDirection,atmosphere,-1,frameData.sky.atmosphereConfig.cloudMultiScatterExtinction);
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
                    participatingMediaAmbient=volumetricShadow(samplePos,normalize(samplePos),atmosphere,-1,SkyMsExtinction);
                    // 天空方向用 SkyMsExtinction(0.5)系数衰减高阶，比太阳路径更「糊」。
                    // 若不做 ground contribution，可关此开关，省一半 light march。

                }

                //compute powder term
                float powderEffect;
                // powderEffect：粉末/银边效应乘子；厚云背光、云底、掠射时提亮，避免死黑。
                {
                    // depthProbability：粉末的「厚度因子」（名字带 Probability，但不是统计学概率，而是 0~若干 的强度标量）。
                    // 物理直觉：局部越厚，光在微滴里多次散射越多，背光侧越容易出现银边/粉边；薄处应弱。
                    // 计算分三步：
                    //   (1) raw = stepCloudDensity * 8.0 * cloudPowderPow
                    //       stepCloudDensity 即本步云密度；*8 放大灵敏度（与 cloud.frag 的 powderDepth 同源）；
                    //       cloudPowderPow 是美术强度旋钮。
                    //   (2) clamp(raw, 0, cloudPowderScale) 封顶，防极密处粉末爆炸。
                    //   (3) pow(..., exponent)，exponent 由下面 remap 按高度给出。
                    float heightPowExponent=remap(normalizedHeight,0.3,0.85,0.5,2.0);
                    // heightPowExponent：pow 的指数，随云内归一化高度 normalizedHeight(0=云底,1=云顶) 变化。
                    // remap(v,0.3,0.85,0.5,2.0) 含义（见 89 行 remap 函数）：
                    //   把 v 在 [0.3, 0.85] 线性映射到 [0.5, 2.0]，区间外 saturate 钳到端点。
                    //   h≤0.3（云底~低空）→ 指数 0.5 → pow(x,0.5)=√x，同样密度下粉末更强（云底亮边）。
                    //   h=0.3~0.85（云腰）→ 指数 0.5→2.0 渐升，粉末对密度越来越「挑剔」。
                    //   h≥0.85（云顶）→ 指数 2.0 → pow(x,2)，需更密才明显粉末，云顶不易过亮。
                    // 数值例：h=0 → exp=0.5；h=0.575 → exp≈1.25；h=1 → exp=2.0。
                    float depthProbability=pow(clamp(stepCloudDensity*8.0*frameData.sky.atmosphereConfig.cloudPowderPow,0.0,frameData.sky.atmosphereConfig.cloudPowderScale),heightPowExponent);

                    depthProbability+=0.05;
                    // 保底 0.05，薄云/低密度也有轻微粉末，避免完全死黑。

                    float verticalProbability=pow(remap(normalizedHeight,0.07,0.22,0.1,1.0),0.8);
                    // verticalProbability：粉末的「高度因子」，与 depthProbability 正交。
                    // remap(h,0.07,0.22,0.1,1.0)：云底 7%~22% 高度带从 0.1 升到 1.0（积云下缘银边最明显）；
                    // h<0.07 → 0.1；h>0.22 → 1.0。再 pow(...,0.8) 略压曲线，过渡更柔。
                    // 最终 powderEffect=powderEffectNew(depthProbability, verticalProbability, VoL)：
                    //   depth×height 两因子相乘，VoL 控制背光时 height 权重（见 201 行）。

                    powderEffect=powderEffectNew(depthProbability,verticalProbability,VoL);//粉末效应
                    // VoL=视线·光向：背光 VoL 负 → powder 更强；顺光减弱，防正面过曝。
                    // RocketSim3D：cloud.frag 已用同一 powderEffectNew，可直接复用。

                }

                
                // Amount of sunlight that reaches the sample point through the cloud 
                // is the combination of ambient light and attenuated direct light.
                vec3 sunlightTerm=atmosphereTransmittance*frameData.sky.atmosphereConfig.cloudShadingSunLightScale*sunColor;
                // sunlightTerm：到达采样点的「直射太阳光」RGB（大气衰减 × 艺术缩放 × 太阳色）。
                // 尚未乘云影、相位、粉末；后面与 sunVisibilityTerm、phase、powder 组合。

                vec3 groundLit=mix(skyBackgroundColor,groundToCloudTransfertIsoScatter,saturate(frameData.sky.atmosphereConfig.cloudNoiseScale-normalizedHeight))
                *saturate(1.0-GroundOcc+normalizedHeight)*frameData.sky.atmosphereConfig.cloudFogFade;
                // groundLit：云底/低层收到的地面+下方天空反射光（非物理精确，艺术项）。
                // mix(skyBackground, groundIrradiance, …)：低处偏地面辐照，高处偏天空背景色。
                // GroundOcc(0.5)：云底遮挡一半地面光；normalizedHeight 越高 ground 项越强。

                vec3 ambientLit=upScaleColor*powderEffect*(1.0-dot(sunDirection,normalize(samplePos)))
                *atmosphereTransmittance;
                // ambientLit：云内环境光（天光经云自阴影后的散射照明）。
                // (1-dot(sunDirection,localUp))：太阳越贴近地平线（局部天顶角越大），环境天光占比越大；
                // 正午直射主导时减弱。原版用 sunDirection.y 假设 Y 恒为当地天顶，行星尺度下
                // 改成 dot(sunDirection, normalize(samplePos))（samplePos 已是行星中心为原点）。
                // 乘 powderEffect：环境路也带一点粉末感。

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

                for(ms=MsCount-1;ms>=0;ms--)// Should terminate at 0
                // 从高阶向 0 阶积分：先算模糊的高阶光，再叠 0 阶直射+地面。
                {
                    // ── 本 ms 阶：组装「照到当前采样点、再被云散射出去」的入射光 L_in ──
                    float sunVisibilityTerm=participatingMedia.transmittanceToLight[ms];
                    // sunVisibilityTerm：太阳路的云遮挡系数 T∈[0,1]（来自 volumetricShadow 沿 sunDirection）。
                    // 0=采样点与太阳之间云很厚，直射几乎全挡；1=之间几乎无云。
                    // ms=0 是真实云影；ms=1 是高阶近似，影更浅（多次散射把暗部提亮）。

                    vec3 sunSkyLuminance=sunVisibilityTerm * sunlightTerm * participatingMediaPhase.phase[ms] * powderEffect;
                    // sunSkyLuminance：本 ms 阶「太阳+天空直射路」的入射辐射度 RGB（单位：亮度，还不是最终像素色）。
                    // 四项相乘，各自含义：
                    //   sunVisibilityTerm  — 云自阴影（太阳方向 shadow march）
                    //   sunlightTerm       — 太阳直射本身（652 行：大气透射×太阳色×艺术缩放；尚未含云影）
                    //   phase[ms]          — 散射方向性：朝太阳亮边(ms=0) / 更均匀(ms=1)
                    //   powderEffect       — 粉末/银边美术乘子（背光厚云更亮）
                    // 此时 sunSkyLuminance 只含「太阳路」；环境天光、地面反射下面再加。

                    if(frameData.sky.atmosphereConfig.cloudEnableGroundContribution!=0)
                    {
                        float skyVisibilityTerm=participatingMediaAmbient.transmittanceToLight[ms];
                        // skyVisibilityTerm：天空路的云遮挡系数 T∈[0,1]（沿 vec3(0,1,0) 的 shadow march）。
                        // 0=采样点头顶云很厚，天光几乎进不来；1=头顶较通透。
                        // 与 sunVisibilityTerm 对称，但方向是「向上」环境光而非太阳。

                        sunSkyLuminance+=skyVisibilityTerm*ambientLit;
                        // 把环境光路叠进同一 L_in：
                        //   ambientLit（663 行）= 无云时该点应有的天光强度（辐照度×粉末×大气×太阳高度因子）
                        //   × skyVisibilityTerm = 经头顶云遮后的实际天光
                        // 加法因为太阳路与环境路是两种独立光源，都照到同一点后再一起被 σ_s 散射。

                    }

                    if(ms==0)
                    {
                        sunSkyLuminance+=groundLit;
                        // 仅 0 阶加地面反射；高阶不加，避免过亮。

                    }

                    vec3 scatteredLitStep=sunSkyLuminance*scatteringCoefficients[ms];
                    // scatteredLitStep：本阶介质在局部收到的「照明 × σ_s」= 散射源强度。

                    vec3 stepScatter=transmittance*(scatteredLitStep-scatteredLitStep*stepTransmittance)/max(1e-4f,extinctionCoefficients[ms]);
                    // stepScatter：经典体积积分离散式 —— ΔL = T_view × (J × (1 - T_step)) / σ_t。
                    // J = L_in × σ_s；T_view 是相机到当前点的透射，T_step 是本步消光。
                    // 物理：前向积分 in-scattering；分母 σ_t 来自 RTE 通量形式。
                    // RocketSim3D：cloud.frag 730-731 行同一公式，分 0/1 两阶手动写开。

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
            sampleT+=stepT;
            // 沿射线前进固定步长 stepT (km)。

        }

        // Apply some additional effect.
        // march 循环结束后：用加权平均命中点写 cloudZ，并对 scatteredLight 做空气透视（Froxel）。
        // RocketSim3D：单 pass cloud.frag 无 cloudZ / Froxel；背景已由 atmo pass 提供，通常整段可跳过。
        if(rayHitPosWeight>0.0)
        // rayHitPosWeight>0 表示本像素 march 中至少有过非零云密度且累加了权重；全无云则跳过。
        {
            // Get average hit pos.
            rayHitPos/=rayHitPosWeight;
            // rayHitPos：透射率加权平均云位置 (km，大气空间)。= Σ(samplePos×T) / Σ(T)。
            // 代表「这朵云在 3D 里大概在哪」，不是第一个/最后一个采样点。

            vec3 rayHitInRender=convertToCameraUnit(rayHitPos,frameData);
            // 大气空间 → 引擎渲染空间。不再减 bottomRadius Y 偏移，与上面 worldPos 构造
            // （行星中心为原点）互逆一致。

            vec4 rayInH=frameData.camViewProj*vec4(rayHitInRender,1.0);
            // 齐次裁剪空间：把代表点投到屏幕，用于深度缓冲。

            cloudZ =rayInH.z/rayInH.w;
            // cloudZ：该像素云的「代表深度」(NDC/clip z/w)，写入云深度 RT（binding 14/16）。
            // 用途：与场景 inDepth 比较前后关系、1/4→全分辨率重建、TAA 重投影。
            // 不是云表面深度，是整段体积的单值近似。

            rayHitPos-=worldPos;
            // 改为相对相机的偏移向量 (km)，worldPos 为 march 射线起点（316 行相机大气坐标）。

            float rayHitHeight=length(rayHitPos);
            // rayHitHeight：相机到加权命中点的直线距离 (km)。
            // 空气透视用「云离相机多远」查 Froxel，不是屏幕 z。

            //apply air perspective
            // 空气透视：相机与云之间的大气散射，使远处云发雾、发蓝、对比度降（观察路径上的雾，非太阳光路径）。
            {
                float slice =aerialPerspectiveDepthToSlice(rayHitHeight);//aerialPerspectiveDepthToSlice来源未知
                // slice：rayHitHeight → Froxel LUT 深度 slice 索引（浮点），预烘焙体积散射查表用。

                float weight=1.0;
                if(slice<0.5)
                {
                    //We multiply by weight to fade to 0 at depth 0. That works for luminance and opacity
                    weight =saturate(slice*2.0);
                    // 极近距离 slice<0.5：weight 0→1 线性 fade，避免相机贴脸时空气透视突变/闪烁。

                    slice=0.5;
                    // 同时把 slice 钳到 0.5，LUT 采样不低于此层，近处用 weight 淡出而非 extrapolate。
                }
                ivec3 sliceLutSize=textureSize(inFroxelScatter,0);//inFroxelScatter来源未知
                // inFroxelScatter：3D Froxel 体积散射 LUT（屏幕 uv × 深度 w × RGBA）。
                // .rgb=该位置应叠的大气散射色；.a=原云色应被洗淡的比例。

                float w=sqrt(slice/float(sliceLutSize.z));// squared distribution
                // 纹理第三维 w∈[0,1]：sqrt 使深度分布偏近处密、远处疏（更多 slice 留给近距细节）。

                vec4 airPerspective=weight*texture(sampler3D(inFroxelScatter,linearClampEdgeSampler),vec3(uv,w));
                // airPerspective：本像素、该云距离下的空气雾色(RGB)与洗淡量(A)，× weight 处理近距 fade。
                // uv=屏幕位置；w=深度层。

                // Phase 2 待接入：真正的 Froxel 3D 体积散射 LUT 还没有烘焙 pass，
                // inFroxelScatter 目前绑定的是中性占位纹理（(0,0,0,0)），airPerspective 恒为
                // vec4(0)，下面这行按原公式代入的话本就等于不改变 scatteredLight——直接跳过
                // 混合，保留计算过程（sliceLutSize/w/airPerspective）方便日后接入时对照。
                // scatteredLight=scatteredLight*(1.0-airPerspective.a)+airPerspective.rgb*(1.0-transmittance);
                // 混合：原云色 × (1-α) + 雾色 × (1-T_view)。
                // airPerspective.a：云 RGB 被大气「冲淡」多少；
                // (1-transmittance)：云越不透明，从观察路径叠进来的雾越少（云本身已挡掉不少）。
            }
        }
    }
    //===================================================
    if(bFog)//体积雾/God Ray
    // bFog=true 时走独立 pass：低层指数雾 + 云影遮挡 + 太阳 God Ray，与主云体 march 并行/可选。
    // 输出 lightingFog：.xyz 雾散射 RGB，.w 雾透射率；写入 binding 22 等云雾 RT。
    // RocketSim3D：无单独 God Ray pass；低空雾/atmo 已在 atmo.frag，通常整段不移植。
    {
        const uint GodRaySteps=36;
        // God RaySteps：沿视线从相机向云底方向固定 36 步 march。

        //单位为米
        float stepLength=1000.0f*(atmosphere.cloudAreaStartHeight-worldPos.y)/clamp(worldDir.y,0.1,1.0)/float(GodRaySteps);
        // stepLength：每步步长 (米)。
        // cloudAreaStartHeight-worldPos.y ≈ 相机到云底壳的垂直高差 (km)；×1000→米；
        // /worldDir.y：视线越平，沿射线到云底路径越长，步长加大；clamp(y,0.1,1) 防除零、限制极端掠射。
        // RocketSim3D：worldPos.y / Y-up 假设，应改为 camAlt 或射线-云底球壳求交距离。

        vec3 stepRay=worldDir*stepLength;
        // stepRay：每步位移向量 (米，因 stepLength 为米)。

        vec3 rayPosWP=frameData.camWorldPos.xyz+stepRay*(fogNoise+0.5);//fogNoise从哪来？
        // rayPosWP：当前雾采样点世界坐标 (米，引擎空间)。
        // 起点相机 + 半步 jitter：fogNoise∈[0,1] 来自调用方（308 行 fogNoise，同 blueNoise 类抖动），减轻带状分层。

        vec3 transmittanceTotal=vec3(1.0);
        // transmittanceTotal：雾 pass 累积视线透射 T (RGB 分通道，此处各通道相同源)。

        vec3 scatteredLightTotal=vec3(0.0,0.0,0.0);
        // scatteredLightTotal：雾 pass 累积散射光 RGB。

        float miePhaseValue=hgPhase(atmosphere.miePhaseG,-VoL);
        // miePhaseValue：Mie 相位 HG(g,-VoL)；VoL=视线·光向，负号因 sunDirection 与 light 约定。
        // 强前向散射 → 太阳方向看雾更亮（God Ray 光柱感）。

        float rayleighPhaseValue=rayleighPhase(VoL);
        // rayleighPhaseValue：Rayleigh 相位；各向同性一些，给雾一点天空蓝散射。
        // 两者在循环外算一次：本 pass 假设相位沿整条射线不变（近似）。

        for(uint i=0;i<GodRaySteps;i++)
        {
            vec3 P0=convertToAtmosphereUnit(rayPosWP,frameData);
            // P0：当前雾采样点转到大气空间 (km，行星中心为原点)，与主云 march 的 worldPos/samplePos 同一套坐标。
            // 本次 Phase 1 里 cloudGodRay 恒为 0（bFog 分支不会真正执行，见 cloud_raymarching.glsl），
            // God Ray 依赖的 SDSM 级联阴影也还是 Phase 2 占位，这段留着只为保证编译通过。

            float visibilityTerm=1.0;
            // visibilityTerm：太阳光到 P0 的路径上，云+雾遮挡后还剩多少（0~1）；下面 shadow 块更新。
            {
                const uint StepLight=8;
                float stepL=atmosphere.cloudAreaThickness/float(StepLight);//km
                // stepL：沿太阳方向的 light march 初始步长，约为云壳厚度均分 8 段。

                stepL=stepL/abs(sunDirection.y);
                // 太阳越贴近地平线，沿 sunDirection 穿过云壳的路径越长，步长放大。

                vec3 position=P0;
                position+=P0.y<=atmosphere.cloudAreaStartHeight?
                    sunDirection*(atmosphere.cloudAreaStartHeight-P0.y)/sunDirection.y:vec3(0.0);
                // 若 P0 在云底以下，先把起点抬到云底壳再沿太阳 march（Y-up 求交）。
                // RocketSim3D：改为球壳求交，不能用 P0.y。
                
                float d=stepL*0.01;
                float transmittanceShadow=0.0;
                // transmittanceShadow：沿太阳路累积 cloudMap 密度（非 exp，先积分再 exp）。

                for(uint j=0;j<StepLight;j++)
                {
                    vec3 samplePosKm=position+sunDirection*d;//单位为km
                    float sampleHeightKm=samplePosKm.y;
                    float sampleDt=sampleHeightKm-atmosphere.cloudAreaStartHeight;
                    float normalizedHeight=sampleDt/atmosphere.cloudAreaThickness;
                    vec3 samplePosMeter=samplePosKm*1000.0f;

                    transmittanceShadow+=cloudMap(samplePosMeter,normalizedHeight);
                    // 累加云密度：太阳到 P0 之间有多少云挡光（简化 shadow，非 volumetricShadow 多阶）。

                    d+=stepL;

                }

                visibilityTerm=exp(-transmittanceShadow*stepL*1000.0);
                // 密度积分 × 步长(米) → 光学深度 τ，T=exp(-τ)。
                // 云越厚 visibilityTerm 越小，God Ray 被云影切掉。
            }

            visibilityTerm=mix(visibilityTerm,1.0,saturate(1.0-sunDirection.y*5.0));
            // 太阳越低 (sunDirection.y 小)，越向 1.0 mix → 低角度少砍 God Ray（日落光柱更长）。
            // saturate(1-y*5)：y<0.2 时开始放松云影对 God Ray 的压制。

            vec3 phaseTimesScattering=vec3(miePhaseValue+rayleighPhaseValue);
            // 相位×散射强度标量（简化为 vec3 同值）；Mie+Rayleigh 合当作雾的散射权重。

            //Second evaluate transmittance due to participating media
            vec3 atmosphereTransmittance;
            {
                float viewHeight=length(P0);
                const vec3 upVector=P0/viewHeight;
                float viewZenithCosAngle=dot(sunDirection,upVector);
                vec2 sampleUv;
                lutTransmittanceParamsToUv(atmosphere,viewHeight,viewZenithCosAngle,sampleUv);
                atmosphereTransmittance=texture(sampler2D(inTransmittanceLut,linearClampEdgeSampler),sampleUv).rgb;
                // 太阳光到 P0 的大气 RGB 透射（与主云 652 行 sunlightTerm 同源 LUT）。
                // RocketSim3D：换 atmo 实时大气衰减，不移植 LUT。

            }
            float density=getDensity(distance(rayPosWP,frameData.camWorldPos.xyz));
            // getDensity(74行)：exp(-d×0.001)×系数×cloudGodRayScale，指数衰减雾密度。
            // 此处 d=采样点到相机距离(米)，非高度——近处雾浓、远处雾淡（参数名 heightMeter 在此用法下实为距离）。
            // 与主云 cloudMap 无关，是低空/global 雾层。

            vec3 sigmaS=vec3(density);
            // σ_s：雾散射系数，RGB 同值。

            const float sigmaA=0.0;
            vec3 sigmaE=max(vec3(1e-6f),sigmaA+sigmaS);
            // σ_t=σ_s（无吸收）；下限防除零。

            vec3 scatteredLitStep=(visibilityTerm*sunColor*phaseTimesScattering*atmosphereTransmittance+skyBackgroundColor)*sigmaS;
            // 本步雾散射源 J = L_in × σ_s。L_in 两路：
            //   直射：visibilityTerm × sunColor × 相位 × 大气透射（God Ray 主项）
            //   + skyBackgroundColor：环境天光被雾散射（LUT 天空色，RocketSim3D 换 atmo 采样）

            vec3 stepTransmittance=exp(-sigmaS*stepLength);
            // 本步雾对视线透射 T_step = exp(-σ_s × Δs)。

            scatteredLightTotal+=transmittanceTotal*(scatteredLitStep-scatteredLitStep*stepTransmittance)/sigmaE;
            // 与主云 731 行同式：ΔL = T_view × J × (1-T_step) / σ_t，前向积分雾散射。

            transmittanceTotal*=stepTransmittance;
            // 更新雾 pass 视线透射。

            //Step.
            rayPosWP+=stepRay;
            // 沿视线前进到下一雾采样点。

        }
        lightingFog.w=mean(transmittanceTotal);//mean()来源未知
        // lightingFog.w：RGB 三通道透射率取平均 → 单标量 alpha，合成 pass 用。
        // mean(vec3)：shared_functions.glsl 205-206 行，(R+G+B)/3。

        lightingFog.xyz=scatteredLightTotal;
        // lightingFog.xyz：本像素 God Ray / 体积雾散射 RGB，与主云 scatteredLight 分开输出。

    }

    //Dual mix transmittance
    // flower 合成约定：RGB 存「未乘透射的散射辐射」L_scatter，A 存视线透射 T_view（不是传统 premultiplied alpha）。
    // 后续 composite pass：finalColor = result.rgb + background.rgb * result.a
    // 等价于 alpha_opacity = 1 - T：云越厚 T 越小，背景被挡越多。
    vec4 result=vec4(scatteredLight,transmittance);
    // result.rgb = scatteredLight：整条视线 cloud march（+ 789 行空气透视）累积的云散射光 RGB，HDR 辐射度。
    // result.a   = transmittance：视线剩余透射 T_view∈[0,1]；1=全透明（无云），0=全挡背景。
    // 与 cloud.frag 对比：那边输出 vec4(cloudColor*alpha, alpha)，alpha=1-viewTrans，已 tone-map 并 premultiply；
    // flower 此处在 compute pass 写 raw L+T 到 imageCloudRenderTexture（binding 2），合成时再与 inHdrSceneColor 混合。

    if(any(isnan(result))||any(isinf(result)))
    {
        result=vec4(0.0,0.0,0.0,1.0);
        // NaN/Inf 保护：数值爆炸时视为「无云散射、完全透射」，避免污染 HDR 与 TAA history。
        // rgb=0 无散射；a=1 背景全透过。RocketSim3D cloud.frag 若加类似 guard 可同样处理。
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
//   • inFroxelScatter / inSDSMShadowDepth+cascadeInfos / inHiz / inGBufferA
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
    inTransmittanceLut(10)  inSkyViewLut(20)  inFroxelScatter(11)
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
  D  173-239  getParticipatingMediaPhase, powder, powderEffectNew, lookupSkylight
  E  241-297 volumetricShadow() → transmittanceToLight[MsCount]
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
    ← inTransmittanceLut, inSkyViewLut, inFroxelScatter, inSkyIrradiance, inHiz

--------------------------------------------------------------------------------
【cloudMap 数据流  L99-168】
  输入: posMeter(m), normalizedHeight(0=云底,1=云顶)
  输出: cloudDensity

  posMeter ──► +wind×height×500 ──► posKm
  posKm.xz ──► inWeatherTexture ──► weatherValue.x ──┐
  posMeter.xz+time ──► inCloudCurlNoise ──► localCoverage ──┤──► coverage
  frameData.cloudCoverage ────────────────────────────────────┘
  posKm+windOffset ──► inBasicNoise × gradientShape ──► basicCloudNoiseWithCoverage
  posKm-windOffset ──► inDetailNoise ──► detailNoiseMixByHeight ──► remap侵蚀
  normalizedHeight ──► densityShape
  return cloudDensity × densityShape  →  主 march 里称 stepCloudDensity

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
  skyBackgroundColor ← lookupSkylight(inSkyViewLut)   // RS3D: 换 atmo 采样
  transmittance ← 1.0;  scatteredLight ← 0
  groundToCloudTransfertIsoScatter ← inSkyIrradiance(0,1,0)

--------------------------------------------------------------------------------
【cloudColorCompute 阶段 1 — march 预备  L437-525】  (!bEarlyOutCloud)

  phase ← dualLobPhase(...)
  participatingMediaPhase ← getParticipatingMediaPhase(phase, 0.5)
  rayHitPos ← 0;  rayHitPosWeight ← 0
  atmosphereTransmittance0 ← LUT@sampleT入口
  atmosphereTransmittance1 ← LUT@tMax出口
  upScaleColor ← inSkyIrradiance(0,1,0)

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

      participatingMedia        ← volumetricShadow(samplePos, sunDirection)
      participatingMediaAmbient ← volumetricShadow(samplePos, vec3(0,1,0))  // 可选

      powderEffect ← powderEffectNew(depthProbability, verticalProbability, VoL)

      sunlightTerm ← atmosphereTransmittance × sunColor × scale
      groundLit    ← mix(skyBG, groundIrradiance, …)
      ambientLit   ← upScaleColor × powder × (1-sunDir.y) × atmosphereTrans

      σ_t ← density;  σ_s ← density × albedo
      scatteringCoefficients[ms], extinctionCoefficients[ms]  // MsCount=2

      for ms = 1 → 0:
        sunSkyLuminance ← sunVisibilityTerm × sunlightTerm × phase[ms] × powder
                       + skyVisibilityTerm × ambientLit          // 可选
                       + groundLit                               // ms==0 only
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
  airPerspective ← inFroxelScatter(uv, w)       // RS3D: 通常跳过
  scatteredLight ← L×(1-α) + fogRGB×(1-transmittance)

--------------------------------------------------------------------------------
【cloudColorCompute 阶段 4 — God Ray  L856-995】  (bFog)

  36步: rayPosWP 沿 worldDir 从相机向云底
  每步:
    visibilityTerm ← exp(-∫cloudMap) 沿太阳 8步 shadow
    density ← getDensity(到相机距离)   // 指数雾，非 cloudMap
    scatteredLitStep ← (visibility×sun×phase×atmTrans + skyBG) × σ_s
    scatteredLightTotal += 同主云积分式
  lightingFog.xyz ← scatteredLightTotal
  lightingFog.w   ← mean(transmittanceTotal)

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
  Froxel 空气透视       ↔  atmo 合成 (不移植)
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