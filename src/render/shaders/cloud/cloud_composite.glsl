
#version 460
#extension GL_GOOGLE_include_directive : enable
#extension GL_EXT_samplerless_texture_functions : enable

#include "cloud_common.glsl"

#define BLUE_NOISE_BUFFER_SET 2
#include "../common/shared_blue_noise.glsl"


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
// Bilinear reads clamped to a single atlas slice (four R32F loads).
float godRayCloudCascade(int c, vec2 uv)
{
    int dim = imageSize(imageCloudShadowTexture).y;
    vec2 p = clamp(uv * float(dim) - 0.5, vec2(0), vec2(dim - 1));
    ivec2 lo = ivec2(floor(p));
    ivec2 hi = min(lo + 1, ivec2(dim - 1));
    vec2 f = fract(p);
    ivec2 offset = ivec2(c * dim, 0);
    return mix(mix(imageLoad(imageCloudShadowTexture, offset + lo).r,
                   imageLoad(imageCloudShadowTexture, offset + ivec2(hi.x, lo.y)).r, f.x),
               mix(imageLoad(imageCloudShadowTexture, offset + ivec2(lo.x, hi.y)).r,
                   imageLoad(imageCloudShadowTexture, offset + hi).r, f.x), f.y);
}

float godRayCloudVisibility(vec3 posKm)
{
    int count = min(int(cloudShadowData.config.x), kCloudShadowMaxCascades);
    for (int c = 0; c < count; ++c)
    {
        vec3 uvd = cloudShadowProject(cloudShadowData, c, posKm);
        if (!cloudShadowInside(uvd)) continue;
        float t = godRayCloudCascade(c, uvd.xy);
        float fit = cloudShadowFit(uvd);
        if (c + 1 < count && fit > cloudShadowData.config.w)
        {
            vec3 next = cloudShadowProject(cloudShadowData, c + 1, posKm);
            if (cloudShadowInside(next))
                t = mix(t, godRayCloudCascade(c + 1, next.xy),
                    clamp((fit - cloudShadowData.config.w) / max(1.0 - cloudShadowData.config.w, 1e-4), 0.0, 1.0));
        }
        return mix(1.0, clamp(t, 0.0, 1.0), clamp(cloudShadowData.config.y, 0.0, 1.0));
    }
    return 1.0;
}

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
    float blueNoise2 = samplerBlueNoiseErrorDistribution_128x128_OptimizedFor_2d2d2d2d(
        offsetId.x, offsetId.y, 0u, 0u);

    //屏幕UV坐标转换至世界视线方向
    vec4 clipSpace = vec4(uv.x * 2.0f - 1.0f, 1.0f - uv.y * 2.0f, 0.0, 1.0);
	vec4 viewPosH = frameData.camInvertProj * clipSpace;
    vec3 viewDir = viewPosH.xyz / viewPosH.w;
    vec3 worldDir = normalize((frameData.camInvertView * vec4(viewDir, 0.0)).xyz);

    //
    vec4 srcColor = imageLoad(imageHdrSceneColor, workPos);//直接按像素读，作为与云混合的背景色，已画好的 atmo + 地形/火箭等 HDR 颜色
    float sceneZ = texture(sampler2D(inDepth, pointClampEdgeSampler), uv).r;
    // 场景深度。已修复：RocketSim3D 用的是标准深度（近=0，远=1），不是 flower 的 Reversed-Z
    // （近=1，远=0）——见 vk_taa.h 深度清除值 1.0f、vk_pipeline.h depthCompareOp=LESS，
    // 两者都是标准深度的写法。之前一直照抄 flower 的 Reversed-Z 假设，"sceneZ<=0 表示天空"
    // 从一开始就是错的（天空应该是清除值 1.0，不是 0），这是云一直显示不出来的根本原因。
    // vec4 cloudColor = kuwaharaFilter(inCloudReconstructionTexture, linearClampEdgeSampler,uv);

    //读来自cloud_reconstruct的三张全分辨率RT
    vec4 cloudColor = texture(sampler2D(inCloudReconstructionTexture, linearClampEdgeSampler), uv);//rgb是云散射光L'，a是云视线透射T_cloud
    vec4 fogColor = texture(sampler2D(inCloudFogReconstructionTexture, linearClampEdgeSampler), uv);//
    vec3 airT = texture(sampler2D(inCloudAirTReconstructionTexture, linearClampEdgeSampler), uv).rgb; // 相机→云大气 RGB 透射
    //linearClampEdgeSampler：双线性 + 边缘 clamp

    float cloudDepth = texture(sampler2D(inCloudDepthReconstructionTexture, pointClampEdgeSampler), uv).r;
    // Representative ray distance in km; no-cloud sentinel = 1e20.

    vec3 result = srcColor.rgb;//默认result是HDR场景色

    // 是否叠云：flower 原版只在"完全没有场景几何"（纯天空）的像素叠云，这对地面/近地视角
    // 成立（云永远比地形/建筑远），但 RocketSim3D 有轨道/全景相机——从太空看地球时，整个
    // 星球圆面全是有效场景深度（地形/地表网格），云层必须能画在星球圆面上，而不是只画在
    // 星球轮廓外的背景星空里。
    //
    // 标准深度下：天空/空白像素的深度是清除值 1.0（最远），"云比场景近"变成
    // Convert ray distance to scene device depth before comparing.
    // Convert the kilometre cloud distance to the same [0,1] convention as
    // scene depth, without subtracting large clip-space coordinates.
    float cloudViewDistance = cloudDepth * max(-normalize(viewDir).z, 1e-6);
    float cloudDeviceDepth = 0.5 - 0.5 * frameData.camProj[2][2]
        + 0.5 * frameData.camProj[3][2] / max(cloudViewDistance, 1e-6);
    bool bComposite = sceneZ >= 1.0f
        || (cloudDepth < 1e19 && cloudDeviceDepth <= sceneZ);

    if(bComposite)
    {
        // Composite：result = src×T_cloud + L'
        // L' 已含 (1-T_cloud)×L_air + T_air×L_cloud；T_cloud = cloudColor.a。
        result = srcColor.rgb * cloudColor.a + cloudColor.rgb;

        if(fogColor.a >= 0.0f)//有雾才叠，当a==-1时表示该像素未算雾
        {
            result.rgb = result.rgb * fogColor.a + max(vec3(0.0f), fogColor.rgb);//背景颜色*透射率+雾自身散射光 
        }
    }
//——————————————————————————————Goy Ray march——————————————————————————————————————————————————
        //远距离丁达尔效应不明显，若想要 km 级丁达尔（云缝阳光、远景光轴），需要例如：
        //加长 God Ray march（成本高）；
        //或在云 raymarch 内加强向太阳方向的散射（你们已有 powder/自阴影，更接近真实丁达尔）
    // 最长 8km、云底以下的 God Ray march。使用球面径向坐标。
    // 改成球面径向：rayPosWP/frameData.camWorldPos 都是"相对当前行星中心"的米制坐标（见
    // cloud_common.glsl::cloudMap() 顶部注释），*0.001 转公里后本身就已经是从行星中心出发的
    // 径向向量，不需要再加任何固定轴偏移——加了反而是错的（假设行星中心固定在 Y=-bottomRadius）。
    if (frameData.sky.atmosphereConfig.cloudGodRay != 0
        && frameData.sky.atmosphereConfig.cloudGodRayScale > 0.0
        && length(frameData.camWorldPos.xyz) * 0.001 < getAtmosphereParameters(frameData).cloudAreaStartHeight)
    {
        const uint  kGodRaySteps = 48;
        const float kMaxMarchingDistance = 8000.0;
        vec3 preGodRayResult = result.rgb;//NaN 兜底回退用（此时 result 已含云 + bFog 合成）

        AtmosphereParameters atmosphere = getAtmosphereParameters(frameData);//大气参数
        const SkyInfo sky = frameData.sky;//天空参数

        // 世界视线方向：shader 顶部已重建（camInvertView * 视空间单位方向），是纯单位向量，
        // 与行星中心原点无关。不要再走 getWorldPos() 反投影求视线——getWorldPos 返回相机
        // 绝对世界空间(km)，而 frameData.camWorldPos 是相对行星中心(米)，两者混减会被行星
        // 中心偏移(~1.5e8km)主导，把 rayDir 拉到「指向行星中心」的方向（旧代码的 bug）。
        vec3 sunDirection = -normalize(frameData.sky.direction);//指向太阳的方向
        float VoL = dot(worldDir, sunDirection);//视线与太阳夹角的cos值，cos值越大越靠近太阳

        // 场景几何距离（米）：用实际投影还原；天空直接使用 8km 上限。
        // snap.proj is OpenGL-style; geometry maps clip Z from [-1,1] to [0,1].
        // Undo that mapping before inverse projection. View-space units are km;
        // use ray length (not axial Z) so off-axis geometry clips fog correctly.
        float sceneDistM = kMaxMarchingDistance;
        if (sceneZ < 1.0)
        {
            vec4 sceneViewH = frameData.camInvertProj
                * vec4(uv.x * 2.0 - 1.0, 1.0 - uv.y * 2.0, sceneZ * 2.0 - 1.0, 1.0);
            sceneDistM = length(sceneViewH.xyz / sceneViewH.w) * 1000.0;
        }

        float marchingDistance = min(kMaxMarchingDistance, sceneDistM);//近处几何像素只 march 到物体表面，不穿过
        // Total-column shadow maps are valid below cloud base, not within clouds.
        float cloudBaseExitKm = raySphereIntersectInside(frameData.camWorldPos.xyz * 0.001,
            worldDir, vec3(0), atmosphere.cloudAreaStartHeight);
        marchingDistance = min(marchingDistance, max(cloudBaseExitKm, 0.0) * 1000.0);
        uint stepCount = uint(clamp(ceil(marchingDistance / 125.0), 8.0, float(kGodRaySteps)));

        float stepLength = marchingDistance / float(stepCount);

        // 从相机前方首个步长内开始，向远处积分；jitter 打破规则分层。
        // camWorldPos(米, 相对行星中心)+单位方向 → 与主云 march 一致的世界米制坐标。
        // Front-to-back integration: all jittered samples stay inside [0, distance].
        vec3 rayOffsetM = vec3(0);
        vec3 rayPosWP = frameData.camWorldPos.xyz + rayOffsetM;

        //累加器初始化
        float transmittance2  = 1.0;//透射率a
        vec3 scatteredLight2 = vec3(0.0, 0.0, 0.0);//散射光rgb

        //光照项预计算
        float miePhaseValue = hgPhase(atmosphere.miePhaseG, -VoL);//Mie相位
        float rayleighPhaseValue = rayleighPhase(VoL);//Rayleigh相位
        vec3 sunColor = frameData.sky.color * frameData.sky.intensity;//太阳HDR色*强度
        vec3 groundToCloudTransfertIsoScatter = sampleCloudAmbient(atmosphere,
            rayPosWP * 0.001, sunDirection);// μs LUT 环境底光
        //接下来循环每步合成sunSkyLuminance = groundToCloudTransfertIsoScatter
        //        + visibilityTerm * sunColor * phase * atmosphereTransmittance;

        for(uint i = 0; i < stepCount; i ++)
        {//————————————————————先算太阳光是否被其他物体遮挡住，即从太阳看过去，是不是已经有别的东西——————
            // Quadratic partition: fine near camera, coarse in the distance.
            float a = float(i) / float(stepCount);
            float b = float(i + 1u) / float(stepCount);
            float t0 = marchingDistance * a * a;
            float t1 = marchingDistance * b * b;
            stepLength = t1 - t0;
            rayOffsetM = worldDir * mix(t0, t1, clamp(blueNoise2, 0.0, 1.0));
            rayPosWP = frameData.camWorldPos.xyz + rayOffsetM;
            float density = getDensity(max(0.0, length(rayPosWP) - atmosphere.bottomRadius * 1000.0));
            if (density < 1e-8) continue;
            if (transmittance2 < 0.01) break;
            float visibilityTerm = 1.0;//默认全亮，无阴影
            {
                // 级联阴影已接入（vk_shadow_cascade.h::VkCascadeShadowSystem 每帧渲染地形深度图集）。
                // 阴影系统关闭（tuner 未开）/本帧无地形面片可画时，C++ 侧把 cascadeCount 落回 0，
                // 下面这个循环因此 0 次迭代、activeCascadeId 停在 0，紧接着 `activeCascadeId <
                // cascadeCount`（0<0）为假，直接跳过阴影采样、保留 visibilityTerm=1.0——
                // 不需要额外分支就能安全降级。
                //SDSM Sample-Distribution Shadow Maps，Shadow Map是从太阳方向渲染一次深度图，每个像素存从太阳看该像素处的表面深度，每个雾点是否被遮挡只要和贴图中的深度比较
                // First find active cascade.//cascade是把世界按距离切成多段，每段单独一张「从太阳看的深度图」，近处用高分辨率 cascade，远处用低分辨率 cascade
                uint activeCascadeId = 0;//当前雾点rayPosWP应该查那一段cascade
                vec3 shadowCoord;//xy分量是在阴影贴图上的位置[0,1],z是该点相对太阳相机的位置

                // Loop to find suitable cascade.
                for(uint cascadeId = 0; cascadeId < sky.cacsadeConfig.cascadeCount; cascadeId ++)//cascadeCount有几段
                {   //cascadeInfos[cascadeId].viewProj把雾点世界坐标转换成该段的深度图的UV和深度
                    // 级联矩阵是绝对世界空间(km)（shadow pass 里地形顶点 = planetCenterAbs + 局部偏移），
                    // 而 rayPosWP 是相对行星中心的米制，需先用相机绝对位置换算回绝对世界 km。
                    // Keep the small camera-relative offset separate for precision.
                    vec3 rayPosAbsKm = frameData.camPosAbsKm.xyz + rayOffsetM * 0.001;
                    shadowCoord = projectPos(rayPosAbsKm, cascadeInfos[cascadeId].viewProj);//viewProj把雾点世界坐标转换成该段的深度图的UV和深度
                    // terrain_shadow.vert applies this same OpenGL -> Vulkan Z mapping.
                    shadowCoord.z = shadowCoord.z * 0.5 + 0.5;
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
                        shadowPosOnAltas.z -= 0.001 * (activeCascadeId + 1.0); // receiver bias toward light

                        float depthShadow = texture(sampler2D(inSDSMShadowDepth, pointClampEdgeSampler), shadowPosOnAltas.xy).r;//采样雾点所在位置的表面深度
                        visibilityTerm = shadowPosOnAltas.z <= depthShadow ? 1.0 : 0.0;
                    }
                }//若全部超出 → activeCascadeId == cascadeCount → 不采样，保持 visibilityTerm = 1

            }//inSDSMShadowDepth/cascadeInfos 已由 vk_cloud_system.h::updateCloudDescriptorSet 每帧绑定为真实资源

            // Second evaluate transmittance due to participating media
            vec3 atmosphereTransmittance;
            {
                vec3 P0 = rayPosWP * 0.001; // meter -> kilometers，行星中心相对坐标，直接是径向向量
                float viewHeight = length(P0);//离地心高度
                const vec3 upVector = P0 / viewHeight;

                float viewZenithCosAngle = dot(sunDirection, upVector);//太阳天顶角cos值
                vec2 sampleUv;
                lutTransmittanceParamsToUv(atmosphere, viewHeight, viewZenithCosAngle, sampleUv);
                atmosphereTransmittance = texture(sampler2D(inTransmittanceLut, linearClampEdgeSampler), sampleUv).rgb;//预烘焙 LUT：在该高度、该太阳角度下，光穿过大气还剩多少
            }//经过大气衰减过后的太阳透射率

            //length(rayPosWP)=到行星中心距离（米，rayPosWP 是相对行星中心坐标）；
            //atmosphere.bottomRadius(km)*1000=地表半径（米）；差=相对地表高度。
            //外太空/轨道高度 → density≈0，God Ray 只存在于近地低空（丁达尔发生的地方）。

            float sigmaS = density;//散射系数
            float sigmaE = max(sigmaS, 1e-8f);//消光系数，约等于散射系数


            vec3 phaseTimesScattering = vec3(miePhaseValue + rayleighPhaseValue);
            visibilityTerm *= godRayCloudVisibility(rayPosWP * 0.001);
            vec3 sunSkyLuminance = groundToCloudTransfertIsoScatter + visibilityTerm * sunColor * phaseTimesScattering * atmosphereTransmittance;
            //groundToCloudTransfertIsoScatter环境底光；visibilityTerm * sun * phase * atmosphereTransmittance直射太阳光经相位大气衰减，被遮挡则为0
            //总入射光

            vec3 sactterLitStep = sunSkyLuminance * sigmaS;//雾越浓散射越强，这一步单位路径上「能散射出去的光」强度

            //Beer-Lambert 体积散射积分核心
            float stepTransmittance = exp(-sigmaE * stepLength);//这一步的雾透射率
            scatteredLight2 += transmittance2 * (sactterLitStep - sactterLitStep * stepTransmittance) / sigmaE; //总散射光，前面步进的总透射率*(这一步的散射光-这一步散射光*这一步的透射率)/消光系数
            transmittance2 *= stepTransmittance;//总的雾透射率

            // Step.
        }

        result.rgb = result.rgb * transmittance2 + scatteredLight2;//当前场景颜色HDR*雾的总透射率+雾的散射光

        if(any(isnan(result))||any(isinf(result)))
        {
            // NaN/Inf 兜底：数值爆炸时退回无 God Ray 的合成结果，避免 NaN 写入 HDR 渲染成纯黑
            // （与 cloud_common.glsl 主云 march 的 result NaN guard 同理）。
            result.rgb = preGodRayResult;
        }
    }




    imageStore(imageHdrSceneColor, workPos, vec4(result.rgb, 1.0));//写回HDR场景RT
    //cloud_composite 的最终输出：带云、带近地 God Ray 的 HDR，供后续 tone map / 显示 / TAA。

}

//没有做 SDSM 阴影 pass渲染
