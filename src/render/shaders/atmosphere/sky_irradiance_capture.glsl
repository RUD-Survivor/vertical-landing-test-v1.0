#version 460
#extension GL_GOOGLE_include_directive : enable

// Sky Irradiance cubemap：把 Sky-View LUT 采样进一张很小的 cubemap（不做单独的
// 半球辐照度卷积，和 flower bake_capture.glsl 一致），供 cloud_common.glsl 的
// groundLit/ambientLit（inSkyIrradiance, binding 27）使用。相机飞出大气层时
// Sky-View LUT 不再有效，退回直接对该方向做一次大气积分。
// 每帧重新烘焙，理由同其余 LUT。
#include "atmosphere_common.glsl"

layout (local_size_x = 8, local_size_y = 8, local_size_z = 1) in;
void main()
{
    ivec3 cubeCoord = ivec3(gl_GlobalInvocationID);
    ivec2 cubeSize = imageSize(imageCubeEnv);
    if (cubeCoord.x >= cubeSize.x || cubeCoord.y >= cubeSize.y || cubeCoord.z >= 6) return;

    const vec2 pixPos = vec2(cubeCoord.xy) + vec2(0.5f);
    const vec2 uv = pixPos / vec2(cubeSize);

    AtmosphereParameters atmosphere = getAtmosphereParameters();

    vec3 worldDir = getSamplingVector(cubeCoord.z, uv);
    const vec3 sunDirection = -normalize(frameData.sky.direction);

    // RocketSim3D: camWorldPos 已是相对当前行星中心的偏移，见 skyview_lut.glsl 同款注释。
    vec3 worldPos = convertToAtmosphereUnit(frameData.camWorldPos.xyz);
    float viewHeight = length(worldPos);

    vec3 result = vec3(0.0);
    const bool bCanUseSkyViewLut = viewHeight < atmosphere.topRadius;
    if (bCanUseSkyViewLut)
    {
        vec3 upVector = normalize(worldPos);
        float viewZenithCosAngle = dot(worldDir, upVector);

        vec3 sideVector = normalize(cross(upVector, worldDir));
        vec3 forwardVector = normalize(cross(sideVector, upVector));

        vec2 lightOnPlane = vec2(dot(sunDirection, forwardVector), dot(sunDirection, sideVector));
        lightOnPlane = normalize(lightOnPlane);
        float lightViewCosAngle = lightOnPlane.x;

        bool bIntersectGround = raySphereIntersectNearest(worldPos, worldDir, vec3(0.0), atmosphere.bottomRadius) >= 0.0f;

        vec2 sampleUv;
        skyViewLutParamsToUv(atmosphere, bIntersectGround, viewZenithCosAngle, lightViewCosAngle, viewHeight, vec2(textureSize(inSkyViewLut, 0)), sampleUv);
        result = texture(sampler2D(inSkyViewLut, linearClampSampler), sampleUv).rgb;
    }
    else if (moveToTopAtmosphere(worldPos, worldDir, atmosphere.topRadius))
    {
        result = integrateScatteredLuminance(
            pixPos, worldPos, worldDir, sunDirection, atmosphere,
            /*bGround*/ false, /*sampleCountIni*/ 0.0, /*depthBufferValue*/ -1.0,
            /*bMieRayPhase*/ true, /*tMaxMax*/ kDefaultMaxT, /*bVariableSampleCount*/ true
        ).scatteredLight;
    }

    imageStore(imageCubeEnv, cubeCoord, vec4(result, 1.0));
}
