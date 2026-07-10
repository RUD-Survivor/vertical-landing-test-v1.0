#ifndef ATMOSPHERE_LUT_COMMON_GLSL
#define ATMOSPHERE_LUT_COMMON_GLSL

#extension GL_EXT_samplerless_texture_functions : enable
#extension GL_GOOGLE_include_directive : enable

// 实时大气 LUT 烘焙公共库。
// 移植自 https://github.com/qiutang98/flower/blob/main/source/shader/atmosphere/atmosphere_common.glsl
// 参考 https://github.com/sebh/UnrealEngineSkyAtmosphere 与
// https://ebruneton.github.io/precomputed_atmospheric_scattering/ (Bruneton 2017)
//
// 与 flower 原版不同：本引擎是行星际飞行模拟器，相机会在同一帧序列里从地表飞到深空，
// 不能像 flower 那样把这些 LUT 当成静态资源只烘一次——必须每帧针对"当前云所在的
// 行星"重新烘焙（frameData.sky.atmosphereConfig 每帧由 C++ 侧按最近行星刷新）。
// 因此这里刻意去掉了 flower 原版对 SDSM 级联阴影 / G-Buffer / Froxel 的绑定依赖，
// 只保留云管线真正需要的最小集合：Transmittance/MultiScatter/SkyView LUT + Sky
// Irradiance cubemap。SDSM 级联阴影（getShadow）在下面直接 stub 成 1.0，待后续
// 阶段接入级联阴影渲染 pass 后再恢复真实实现。

#include "../common/shared_atmosphere.glsl"

layout (set = 0, binding = 0, rgba16f) uniform image2D imageTransmittanceLut;
layout (set = 0, binding = 1) uniform texture2D inTransmittanceLut;

layout (set = 0, binding = 2, rgba16f) uniform image2D imageSkyViewLut;
layout (set = 0, binding = 3) uniform texture2D inSkyViewLut;

layout (set = 0, binding = 4, rgba16f) uniform image2D imageMultiScatterLut;
layout (set = 0, binding = 5) uniform texture2D inMultiScatterLut;

// 场景深度；LUT 烘焙本身不合成到场景上（depthBufferValue 固定 -1），
// 保留这个绑定只是为了让 integrateScatteredLuminance() 的签名与 flower 一致。
layout (set = 0, binding = 6) uniform texture2D inDepth;

// Sky Irradiance 小 cubemap（sky_irradiance_capture.glsl 写入），直接采样
// Sky-View LUT 得到，不做单独的辐照度卷积（与 flower bake_capture.glsl 一致）。
layout (set = 0, binding = 7, rgba16f) writeonly uniform imageCube imageCubeEnv;

layout (set = 0, binding = 8) uniform UniformFrameData { PerFrameData frameData; };

// 本文件自带一个 linear/clamp 采样器，不依赖 shared_sampler.glsl 的全局 Set（
// 该全局 Set 在本引擎里尚未有 C++ 侧实现，云管线本身也是本次才接入，见 vk_atmosphere_lut.h）。
layout (set = 0, binding = 9) uniform sampler linearClampSampler;

AtmosphereParameters getAtmosphereParameters()
{
    return getAtmosphereParameters(frameData);
}

vec3 convertToAtmosphereUnit(vec3 o)
{
    return convertToAtmosphereUnit(o, frameData);
}

vec3 convertToCameraUnit(vec3 o)
{
    return convertToCameraUnit(o, frameData);
}

// SDSM 级联阴影查询。Phase 2 待接入：本引擎目前没有级联阴影渲染 pass，
// 云自身的自阴影由 cloud_common.glsl::volumetricShadow() 独立 march 完成，不受影响；
// 这里固定返回“完全可见”，只影响大气积分里额外的地表阴影项（本就很次要）。
float getShadow(in const AtmosphereParameters atmosphere, vec3 p)
{
    return 1.0f;
}

void uvToSkyViewLutParams(
    in  const AtmosphereParameters atmosphere,
    out float viewZenithCosAngle,
    out float lightViewCosAngle,
    in  float viewHeight,
    in  vec2  uv)
{
    // 限制在有效子像素范围内，避免天顶方向的导数问题在 LUT 上可见。
    vec2 lutSize = vec2(imageSize(imageSkyViewLut));
    uv = vec2(fromSubUvsToUnit(uv.x, lutSize.x), fromSubUvsToUnit(uv.y, lutSize.y));

    float vHorizon = sqrt(viewHeight * viewHeight - atmosphere.bottomRadius * atmosphere.bottomRadius);
    float cosBeta = vHorizon / viewHeight;

    float beta = acos(cosBeta);
    float zenithHorizonAngle = kPI - beta;

    if (uv.y < 0.5f)
    {
        float coord = 2.0 * uv.y;
        coord = 1.0 - coord;
        coord *= coord;
        coord = 1.0 - coord;
        viewZenithCosAngle = cos(zenithHorizonAngle * coord);
    }
    else
    {
        float coord = uv.y * 2.0 - 1.0;
        coord *= coord;
        viewZenithCosAngle = cos(zenithHorizonAngle + beta * coord);
    }

    float coord = uv.x;
    coord *= coord;
    lightViewCosAngle = -(coord * 2.0 - 1.0);
}

vec3 getMultipleScattering(
    in const AtmosphereParameters atmosphere,
    vec3  scattering,
    vec3  extinction,
    vec3  worldPos,
    float viewZenithCosAngle)
{
    const float viewHeight = getViewHeight(worldPos, atmosphere);
    vec2 lutSize = vec2(textureSize(inMultiScatterLut, 0));

    vec2 uv = saturate(vec2(viewZenithCosAngle * 0.5f + 0.5f, viewHeight / (atmosphere.topRadius - atmosphere.bottomRadius)));
    uv = vec2(fromUnitToSubUvs(uv.x, lutSize.x), fromUnitToSubUvs(uv.y, lutSize.y));

    return texture(sampler2D(inMultiScatterLut, linearClampSampler), uv).rgb;
}

struct SingleScatteringResult
{
    vec3 scatteredLight; // 积分后的散射光（luminance）
    vec3 opticalDepth;   // 光学深度（1/m 量级，实际单位取决于调用方步长单位）
    vec3 transmittance;  // 视线透射率 [0,1]

    vec3 multiScatAs1;
    vec3 newMultiScatStep0Out;
    vec3 newMultiScatStep1Out;
};

SingleScatteringResult buildSingleScatteringResultDefault()
{
    SingleScatteringResult result;
    result.scatteredLight = vec3(0.0);
    result.opticalDepth   = vec3(0.0);
    result.transmittance  = vec3(0.0);
    result.multiScatAs1         = vec3(0.0);
    result.newMultiScatStep0Out = vec3(0.0);
    result.newMultiScatStep1Out = vec3(0.0);
    return result;
}

const float kDefaultMaxT = 9000000.0f;

// 沿一条射线积分大气单次/多次散射，是 Transmittance/MultiScatter/SkyView LUT
// 三个 bake pass 共用的核心函数。逐行对应 flower atmosphere_common.glsl。
SingleScatteringResult integrateScatteredLuminance(
    in vec2  pixPos,
    in vec3  worldPos,
    in vec3  worldDir,
    in vec3  sunDir,
    in const AtmosphereParameters atmosphere,
    in bool  bGround,
    in float sampleCountIni,
    in float depthBufferValue,
    in bool  bMieRayPhase,
    in float tMaxMax,
    in bool  bVariableSampleCount)
{
    SingleScatteringResult result = buildSingleScatteringResultDefault();

    const vec3 kEarthOrigin = vec3(0.0);

    float tBottom = raySphereIntersectNearest(worldPos, worldDir, kEarthOrigin, atmosphere.bottomRadius);
    float tTop = raySphereIntersectNearest(worldPos, worldDir, kEarthOrigin, atmosphere.topRadius);

    float tMax = 0.0f;
    if (tBottom < 0.0f)
    {
        if (tTop < 0.0f)
        {
            return result;
        }
        else
        {
            tMax = tTop;
        }
    }
    else
    {
        if (tTop > 0.0f)
        {
            tMax = min(tTop, tBottom);
        }
    }

    if (depthBufferValue >= 0.0 && depthBufferValue <= 1.0)
    {
        vec2 sampleUv = (pixPos + vec2(0.5)) / vec2(textureSize(inDepth, 0));
        vec3 depthBufferWorldPos = getWorldPos(sampleUv, depthBufferValue, frameData);

        float tDepth = length(depthBufferWorldPos * 0.001 + vec3(0.0, atmosphere.bottomRadius, 0.0) - worldPos);
        if (tDepth < tMax)
        {
            tMax = tDepth;
        }
    }
    tMax = min(tMax, tMaxMax);

    float sampleCount = sampleCountIni;
    float sampleCountFloor = sampleCountIni;
    float tMaxFloor = tMax;
    if (bVariableSampleCount)
    {
        sampleCount = mix(float(atmosphere.viewRayMarchMinSPP), float(atmosphere.viewRayMarchMaxSPP), saturate(tMax * 0.01));
        sampleCountFloor = floor(sampleCount);
        tMaxFloor = tMax * sampleCountFloor / sampleCount;
    }

    float dt = tMax / sampleCount;

    const float uniformPhase = getUniformPhase();
    const vec3 wi = sunDir;
    const vec3 wo = worldDir;
    float cosTheta = dot(wi, wo);

    float miePhaseValue = hgPhase(atmosphere.miePhaseG, -cosTheta);
    float rayleighPhaseValue = rayleighPhase(cosTheta);

    vec3 globalL = frameData.sky.color * frameData.sky.intensity;

    vec3 L = vec3(0.0);
    vec3 throughput = vec3(1.0);
    vec3 opticalDepth = vec3(0.0);
    float t = 0.0f;
    const float sampleSegmentT = 0.3f;

    for (float s = 0.0; s < sampleCount; s += 1.0)
    {
        if (bVariableSampleCount)
        {
            float t0 = (s) / sampleCountFloor;
            float t1 = (s + 1.0f) / sampleCountFloor;

            t0 = t0 * t0;
            t1 = t1 * t1;
            t0 = tMaxFloor * t0;
            t1 = (t1 > 1.0) ? tMax : tMaxFloor * t1;

            t = t0 + (t1 - t0) * sampleSegmentT;
            dt = t1 - t0;
        }
        else
        {
            float newT = tMax * (s + sampleSegmentT) / sampleCount;
            dt = newT - t;
            t = newT;
        }

        vec3 P = worldPos + t * worldDir;
        MediumSampleRGB medium = sampleMediumRGB(P, atmosphere);

        const vec3 sampleOpticalDepth = medium.extinction * dt;
        const vec3 sampleTransmittance = exp(-sampleOpticalDepth);
        opticalDepth += sampleOpticalDepth;

        float pHeight = length(P);
        const vec3 upVector = P / pHeight;

        float sunZenithCosAngle = dot(sunDir, upVector);
        vec2 uv;
        lutTransmittanceParamsToUv(atmosphere, pHeight, sunZenithCosAngle, uv);
        vec3 transmittanceToSun = texture(sampler2D(inTransmittanceLut, linearClampSampler), uv).rgb;

        vec3 phaseTimesScattering;
        if (bMieRayPhase)
        {
            phaseTimesScattering = medium.scatteringMie * miePhaseValue + medium.scatteringRay * rayleighPhaseValue;
        }
        else
        {
            phaseTimesScattering = medium.scattering * uniformPhase;
        }

        float tEarth = raySphereIntersectNearest(P, sunDir, kEarthOrigin + kPlanetRadiusOffset * upVector, atmosphere.bottomRadius);
        float earthShadow = tEarth >= 0.0f ? 0.0f : 1.0f;

        vec3 multiScatteredLuminance = vec3(0.0);
#ifndef NO_MULTISCATAPPROX_ENABLED
        multiScatteredLuminance = getMultipleScattering(atmosphere, medium.scattering, medium.extinction, P, sunZenithCosAngle);
#endif
        float shadow = getShadow(atmosphere, P);

        vec3 S = globalL * (earthShadow * shadow * transmittanceToSun * phaseTimesScattering + (multiScatteredLuminance * medium.scattering));

        vec3 ms = medium.scattering;
        vec3 msint = (ms - ms * sampleTransmittance) / medium.extinction;
        result.multiScatAs1 += throughput * msint;

        {
            vec3 newMS;
            newMS = earthShadow * transmittanceToSun * medium.scattering * uniformPhase;
            result.newMultiScatStep0Out += throughput * (newMS - newMS * sampleTransmittance) / medium.extinction;

            newMS = medium.scattering * uniformPhase * multiScatteredLuminance;
            result.newMultiScatStep1Out += throughput * (newMS - newMS * sampleTransmittance) / medium.extinction;
        }

        vec3 sint = (S - S * sampleTransmittance) / medium.extinction;
        L += throughput * sint;
        throughput *= sampleTransmittance;
    }

    if (bGround && (tMax == tBottom) && (tBottom > 0.0))
    {
        vec3 P = worldPos + tBottom * worldDir;
        float pHeight = length(P);

        const vec3 upVector = P / pHeight;
        float sunZenithCosAngle = dot(sunDir, upVector);
        vec2 uv;
        lutTransmittanceParamsToUv(atmosphere, pHeight, sunZenithCosAngle, uv);
        vec3 transmittanceToSun = texture(sampler2D(inTransmittanceLut, linearClampSampler), uv).rgb;

        const float NdotL = saturate(dot(normalize(upVector), normalize(sunDir)));
        L += globalL * transmittanceToSun * throughput * NdotL * atmosphere.groundAlbedo / kPI;
    }

    result.scatteredLight = L;
    result.opticalDepth  = opticalDepth;
    result.transmittance = throughput;
    return result;
}

// Sky-View LUT 单个 texel 的散射光求值：把 (u,v) 反解成 (视天顶角, 太阳-视线方位角)，
// 再沿该方向做一次完整的大气积分。sky_irradiance_capture.glsl 采样 Sky-View LUT
// 时如果相机已经飞出大气层，会退回直接调用 integrateScatteredLuminance()（见该文件）。
vec3 getPosScatterLight(
    in const AtmosphereParameters atmosphere,
    in const vec3 inWorldPos,
    in const vec2 uv,
    in const bool bGround,
    in const vec2 pixPos)
{
    float viewHeight = length(inWorldPos);
    float viewZenithCosAngle;
    float lightViewCosAngle;
    uvToSkyViewLutParams(atmosphere, viewZenithCosAngle, lightViewCosAngle, viewHeight, uv);

    // 已修复（实机反馈定位：大气壳内出现一个固定在太阳/地平线方位上的巨大白色
    // 楔形，转头不消失，形状随相机纬度变化）：原来这里烘焙时用一个"规范坐标系"
    // ——worldPos 锁在 (0, viewHeight, 0)，太阳锁在全局 X-Y 平面（Z=0）——理论上
    // 靠"高度 + 视天顶角 + 太阳-视线相对方位角"这三个量的旋转不变性，应该和真实
    // 位置等价；但采样侧（atmo_inside.frag::skyViewLUT()）构造局部基是用真实
    // camRelPlanet/真实视线方向，只要这两套坐标系的旋转对应关系有任何一点没有
    // 完全对齐，两边算出来的 (viewZenithCosAngle, lightViewCosAngle) 就会对不上，
    // 采样时查到错误的 LUT texel——相机不在"规范坐标系"假设的那个纬度/经度时
    // 尤其明显，正好是一个跟着相机朝向、相对太阳方位固定的楔形误差区。
    //
    // 改成烘焙时也用真实局部系：worldPos 就是真实球面位置（不是合成的 (0,H,0)），
    // {up, side, forward} 这组基由真实 upVector 和真实太阳方向构造（forward=太阳
    // 水平投影方向），sunDir/worldDir 都在这组真实基里搭，和采样侧完全同构——
    // 不再依赖"规范坐标系等价"这个假设本身是否严格成立。
    vec3 upVector = normalize(inWorldPos);
    vec3 worldPos = upVector * viewHeight;

    vec3 realSunDir = -normalize(frameData.sky.direction); // 指向太阳
    vec3 sideCross = cross(upVector, realSunDir);
    float sideLen  = length(sideCross);
    // 太阳几乎正好在天顶/天底时退化，任选一个不平行的辅助轴兜底（和
    // atmo_inside.frag::safeOrthoAxis 同款处理，只影响这个奇点附近极小范围）。
    vec3 side = (sideLen > 1e-5)
        ? sideCross / sideLen
        : normalize(cross(upVector, (abs(upVector.x) < 0.9) ? vec3(1.0,0.0,0.0) : vec3(0.0,1.0,0.0)));
    vec3 forward = normalize(cross(side, upVector)); // 太阳方向的水平投影

    float sunZenithCosAngle = dot(upVector, realSunDir);
    float sunHorizSin = sqrt(max(0.0, 1.0 - sunZenithCosAngle * sunZenithCosAngle));
    vec3 sunDir = forward * sunHorizSin + upVector * sunZenithCosAngle;

    float viewZenithSinAngle = sqrt(max(0.0, 1.0 - viewZenithCosAngle * viewZenithCosAngle));
    // 视线相对太阳的方位角就是 acos(lightViewCosAngle)，直接在真实 {forward,side,up}
    // 基里把这个视线方向搭出来（forward 分量用 lightViewCosAngle，不再是合成坐标系
    // 里的 (X,Y,Z) 三元组）。
    vec3 worldDir = forward  * (viewZenithSinAngle * lightViewCosAngle)
                  + side     * (viewZenithSinAngle * sqrt(max(0.0, 1.0 - lightViewCosAngle * lightViewCosAngle)))
                  + upVector * viewZenithCosAngle;

    if (!moveToTopAtmosphere(worldPos, worldDir, atmosphere.topRadius))
    {
        return vec3(0.0);
    }

    const float sampleCountIni      = 30;
    const float depthBufferValue    = -1.0;
    const bool bMieRayPhase         = true;
    const float tMaxMax             = kDefaultMaxT;
    const bool bVariableSampleCount = true;

    SingleScatteringResult ss = integrateScatteredLuminance(
        pixPos, worldPos, worldDir, sunDir, atmosphere,
        bGround, sampleCountIni, depthBufferValue, bMieRayPhase, tMaxMax, bVariableSampleCount
    );
    return min(ss.scatteredLight, vec3(kMaxHalfFloat));
}

#endif // ATMOSPHERE_LUT_COMMON_GLSL
