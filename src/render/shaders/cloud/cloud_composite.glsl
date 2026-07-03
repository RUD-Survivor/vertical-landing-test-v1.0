
#version 460
#extension GL_GOOGLE_include_directive : enable
#extension GL_EXT_samplerless_texture_functions : enable

#include "cloud_common.glsl"


vec3 drawSun(vec3 rayDir, vec3 sunDir) 
{
    const float dT = dot(rayDir, sunDir);//视线与太阳夹角的cos值，cos值越大越靠近太阳

    const float theta = 0.1 * kPI / 180.0;//太阳盘半角，0.1度
    const vec3 sunCenterColor = vec3(1.0f, 0.92549, 0.87843) * 39.0f * 100;//HDR强度=3900

    const float cT = cos(theta);
    if (dT >= cT) //视线在太阳盘内
    {
        return sunCenterColor;
    }

    return vec3(0.0);
}//盘内外亮度突变，无Mie晕，且Main未调用


layout (local_size_x = 8, local_size_y = 8) in;//线程组布局，每个 work group 64 个线程，常见 GPU 友好尺寸，C++侧dispatch(ceil(w/8), ceil(h/8), 1) 覆盖整张 HDR RT
void main()
{
    //——————————————————————————————————composite pass 准备阶段——————————————————————————————————
    ivec2 texSize = imageSize(imageHdrSceneColor);//HDR 场景 RT 宽高（全分辨率，composite 输出目标
    ivec2 depthTextureSize = textureSize(inDepth, 0);//场景深度纹理尺寸
    ivec2 workPos = ivec2(gl_GlobalInvocationID.xy);//当前线程负责的 像素整数坐标

    if(workPos.x >= texSize.x || workPos.y >= texSize.y)
    {
        return;
    }//超出 RT 范围的直接 return，不写、不采样

    const vec2 uv = (vec2(workPos) + vec2(0.5f)) / vec2(texSize);//像素中心的UV坐标，[0,1]

    // Offset retarget for new seeds each frame
    //蓝噪声用于God Ray
    uvec2 offset = uvec2(vec2(0.754877669, 0.569840296) * (frameData.frameIndex.x) * uvec2(texSize));
    uvec2 offsetId = workPos.xy + offset;
    offsetId.x = offsetId.x % texSize.x;
    offsetId.y = offsetId.y % texSize.y;
    float blueNoise2 = samplerBlueNoiseErrorDistribution_128x128_OptimizedFor_2d2d2d2d(offsetId.x, offsetId.y, 0, 0u); 

    //屏幕UV坐标转换至世界视线方向
    vec4 clipSpace = vec4(uv.x * 2.0f - 1.0f, 1.0f - uv.y * 2.0f, 0.0, 1.0);
	vec4 viewPosH = frameData.camInvertProj * clipSpace;
    vec3 viewDir = viewPosH.xyz / viewPosH.w;
    vec3 worldDir = normalize((frameData.camInvertView * vec4(viewDir, 0.0)).xyz);

    //
    vec4 srcColor = imageLoad(imageHdrSceneColor, workPos);//直接按像素读，作为与云混合的背景色，已画好的 atmo + 地形/火箭等 HDR 颜色
    float sceneZ = texture(sampler2D(inDepth, pointClampEdgeSampler), uv).r;//场景深度，sceneZ决定是否叠云，Reversed-Z：≤ 0 表示天空(叠云)，有值表示碰到几何
    // vec4 cloudColor = kuwaharaFilter(inCloudReconstructionTexture, linearClampEdgeSampler,uv);

    //读来自cloud_reconstruct的三张全分辨率RT
    vec4 cloudColor = texture(sampler2D(inCloudReconstructionTexture, linearClampEdgeSampler), uv);//rgb是云散射光L，a是视线透射T[0,1]
    vec4 fogColor = texture(sampler2D(inCloudFogReconstructionTexture, linearClampEdgeSampler), uv);//
    //linearClampEdgeSampler：双线性 + 边缘 clamp

    float cloudDepth = texture(sampler2D(inCloudDepthReconstructionTexture, linearClampEdgeSampler), uv).r;//Reversed-Z云代表深度
    cloudDepth = max(1e-5f, cloudDepth); // very far cloud may be negative, use small value is enough.

    vec3 result = srcColor.rgb;//默认result是HDR场景色

    if(sceneZ <= 0.0f) // reverse z.,仅天空像素叠云/雾
    {
        // Composite planar cloud.

        result = srcColor.rgb * cloudColor.a + cloudColor.rgb;//背景颜色*透射率+云自身散射光  

        if(fogColor.a >= 0.0f)//有雾才叠，当a==-1时表示该像素未算雾
        {
            result.rgb = result.rgb * fogColor.a + max(vec3(0.0f), fogColor.rgb);//背景颜色*透射率+雾自身散射光 
        }
    }
//——————————————————————————————Goy Ray march——————————————————————————————————————————————————
        //远距离丁达尔效应不明显，若想要 km 级丁达尔（云缝阳光、远景光轴），需要例如：
        //加长 God Ray march（成本高）；
        //或在云 raymarch 内加强向太阳方向的散射（你们已有 powder/自阴影，更接近真实丁达尔）
    {
        const uint  kGodRaySteps = 64;//定步数
        const float kMaxMarchingDistance = 400.0f;//最大步进距离=步数*步长

        // We are revert z.
        //重复UV坐标转成世界空间视线方向，感觉没必要
        vec4 clipSpace = vec4(uv.x * 2.0f - 1.0f, 1.0f - uv.y * 2.0f, 0.0, 1.0);
        vec4 viewPosH = frameData.camInvertProj * clipSpace;
        vec3 viewSpaceDir = viewPosH.xyz / viewPosH.w;
        vec3 worldDir = normalize((frameData.camInvertView * vec4(viewSpaceDir, 0.0)).xyz);


        AtmosphereParameters atmosphere = getAtmosphereParameters(frameData);//大气参数
        const SkyInfo sky = frameData.sky;//天空参数
        vec3 worldPosWP      = getWorldPos(uv, sceneZ, frameData);//用场景深度反投天空像素坐标和几何像素坐标
        vec3 pixelToCameraWP = frameData.camWorldPos.xyz - worldPosWP;//从像素世界点指向相机的向量

        float pixelToCameraDistanceWP = max(1e-5f, length(pixelToCameraWP));//防除零，单位米
        vec3 rayDirWP = pixelToCameraWP / pixelToCameraDistanceWP;//归一化后成单位向量

        float marchingDistance = min(kMaxMarchingDistance, pixelToCameraDistanceWP);//如果起点过远，裁到最大步进距离内
        if(pixelToCameraDistanceWP > kMaxMarchingDistance)
        {
            worldPosWP = frameData.camWorldPos.xyz - rayDirWP * marchingDistance;//只 march 靠近相机的 400 m
        }

        vec3 sunDirection = -normalize(frameData.sky.direction);//指向太阳的方向
        float VoL = dot(worldDir, sunDirection);//视线与太阳夹角的cos值，cos值越大越靠近太阳
        
        float stepLength = marchingDistance / float(kGodRaySteps);//每步长度
        vec3 stepRay = rayDirWP * stepLength;//沿像素到相机每步的位移向量
    
        // Interval noise is better than blue noise here.
        float taaOffset = interleavedGradientNoise(workPos, frameData.frameIndex.x % frameData.jitterPeriod);//未使用

        vec3 rayPosWP = worldPosWP + stepRay * (blueNoise2 + 0.05);//第一步偏移0.05-1.05个步长，打破规则分层banding

        //累加器初始化
        float transmittance2  = 1.0;//透射率a
        vec3 scatteredLight2 = vec3(0.0, 0.0, 0.0);//散射光rgb

        //光照项预计算
        float miePhaseValue = hgPhase(atmosphere.miePhaseG, -VoL);//Mie相位
        float rayleighPhaseValue = rayleighPhase(VoL);//Rayleigh相位
        vec3 sunColor = frameData.sky.color * frameData.sky.intensity;//太阳HDR色*强度
        vec3 groundToCloudTransfertIsoScatter =  texture(samplerCube(inSkyIrradiance, linearClampEdgeSampler), vec3(0, 1, 0)).rgb;//天空立方体贴图向上(0,1,0)采样，各向同性环境/地面反射底光
        //接下来循环每步合成sunSkyLuminance = groundToCloudTransfertIsoScatter
        //        + visibilityTerm * sunColor * phase * atmosphereTransmittance;

        for(uint i = 0; i < kGodRaySteps; i ++)//步进循环
        {//————————————————————先算太阳光是否被其他物体遮挡住，即从太阳看过去，是不是已经有别的东西——————
            float visibilityTerm = 1.0;//默认全亮，无阴影
            {
                //SDSM Sample-Distribution Shadow Maps，Shadow Map是从太阳方向渲染一次深度图，每个像素存从太阳看该像素处的表面深度，每个雾点是否被遮挡只要和贴图中的深度比较
                // First find active cascade.//cascade是把世界按距离切成多段，每段单独一张「从太阳看的深度图」，近处用高分辨率 cascade，远处用低分辨率 cascade
                uint activeCascadeId = 0;//当前雾点rayPosWP应该查那一段cascade
                vec3 shadowCoord;//xy分量是在阴影贴图上的位置[0,1],z是该点相对太阳相机的位置

                // Loop to find suitable cascade.
                for(uint cascadeId = 0; cascadeId < sky.cacsadeConfig.cascadeCount; cascadeId ++)//cascadeCount有几段
                {   //cascadeInfos[cascadeId].viewProj把雾点世界坐标转换成该段的深度图的UV和深度
                    shadowCoord = projectPos(rayPosWP, cascadeInfos[cascadeId].viewProj);//viewProj把雾点世界坐标转换成该段的深度图的UV和深度
                    if(onRange(shadowCoord.xyz, vec3(sky.cacsadeConfig.cascadeBorderAdopt), vec3(1.0f - sky.cacsadeConfig.cascadeBorderAdopt)))//看该点是否落在此cascade的有效UV区，cascadeBorderAdopt是边缘调整处
                    {
                        break;//找到了，此时activeCascadeId就是
                    }
                    activeCascadeId ++;//从近到远试 cascade，第一个能包住 rayPosWP 的 就是 activeCascadeId
                }

                if(activeCascadeId < sky.cacsadeConfig.cascadeCount)
                {
                    const float perCascadeOffsetUV = 1.0f / sky.cacsadeConfig.cascadeCount;
                    const float shadowTexelSize = 1.0f / float(sky.cacsadeConfig.percascadeDimXY);

                    // Main cascsade shadow compute.
                    {
                        vec3 shadowPosOnAltas = shadowCoord;//雾点的uv坐标+深度
                        
                        // Also add altas bias and z bias.
                        shadowPosOnAltas.x = (shadowPosOnAltas.x + float(activeCascadeId)) * perCascadeOffsetUV;//多级 cascade 拼在同一张 inSDSMShadowDepth 里，所以要+ activeCascadeId 再 * (1/count) = 跳到对应条带平移uv对应到正确条带中
                        shadowPosOnAltas.z += 0.001 * (activeCascadeId + 1.0);//把当前点稍微推离太阳一点，防摩尔纹

                        float depthShadow = texture(sampler2D(inSDSMShadowDepth, pointClampEdgeSampler), shadowPosOnAltas.xy).r;//采样雾点所在位置的表面深度
                        visibilityTerm = shadowPosOnAltas.z > depthShadow ? 1.0 : 0.0;//深度比较，看雾在表面前面还是后面，在后面被遮挡设为0
                    }
                }//若全部超出 → activeCascadeId == cascadeCount → 不采样，保持 visibilityTerm = 1

            }//需要 C++ 绑定 inSDSMShadowDepth、cascadeInfos SSBO，且每帧更新

            // Second evaluate transmittance due to participating media
            vec3 atmosphereTransmittance;
            {
                vec3 P0 = rayPosWP * 0.001 + vec3(0.0, atmosphere.bottomRadius, 0.0); // meter -> kilometers.
                float viewHeight = length(P0);//离地心高度
                const vec3 upVector = P0 / viewHeight;

                float viewZenithCosAngle = dot(sunDirection, upVector);//太阳天顶角cos值
                vec2 sampleUv;
                lutTransmittanceParamsToUv(atmosphere, viewHeight, viewZenithCosAngle, sampleUv);
                atmosphereTransmittance = texture(sampler2D(inTransmittanceLut, linearClampEdgeSampler), sampleUv).rgb;//预烘焙 LUT：在该高度、该太阳角度下，光穿过大气还剩多少
            }//经过大气衰减过后的太阳透射率

            float density = getDensity(distance(rayPosWP, frameData.camWorldPos.xyz));//雾的浓度
            //getDensity来源未知

            float sigmaS = density;//散射系数
            float sigmaE = max(sigmaS, 1e-8f);//消光系数，约等于散射系数


            vec3 phaseTimesScattering = vec3(miePhaseValue + rayleighPhaseValue);
            vec3 sunSkyLuminance = groundToCloudTransfertIsoScatter + visibilityTerm * sunColor * phaseTimesScattering * atmosphereTransmittance;
            //groundToCloudTransfertIsoScatter环境底光；visibilityTerm * sun * phase * atmosphereTransmittance直射太阳光经相位大气衰减，被遮挡则为0
            //总入射光

            vec3 sactterLitStep = sunSkyLuminance * sigmaS;//雾越浓散射越强，这一步单位路径上「能散射出去的光」强度

            //Beer-Lambert 体积散射积分核心
            float stepTransmittance = exp(-sigmaE * stepLength);//这一步的雾透射率
            scatteredLight2 += transmittance2 * (sactterLitStep - sactterLitStep * stepTransmittance) / sigmaE; //总散射光，前面步进的总透射率*(这一步的散射光-这一步散射光*这一步的透射率)/消光系数
            transmittance2 *= stepTransmittance;//总的雾透射率

            // Step.
            rayPosWP += stepRay;沿采样点向相机再走一步
        }

        result.rgb = result.rgb * transmittance2 + scatteredLight2;//当前场景颜色HDR*雾的总透射率+雾的散射光
    }




    imageStore(imageHdrSceneColor, workPos, vec4(result.rgb, 1.0));//写回HDR场景RT
    //cloud_composite 的最终输出：带云、带近地 God Ray 的 HDR，供后续 tone map / 显示 / TAA。

}

//没有做 SDSM 阴影 pass渲染