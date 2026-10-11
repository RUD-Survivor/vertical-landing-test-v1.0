#version 460
#extension GL_GOOGLE_include_directive : enable
#extension GL_EXT_samplerless_texture_functions : enable
//允许写成 texture(sampler2D(tex, sampler), uv)，把 纹理对象 和 采样器状态（repeat/clamp、filter 等）分开传

#include "cloud_common.glsl"

#define BLUE_NOISE_BUFFER_SET 2
#include "../common/shared_blue_noise.glsl"

// 1/4 分辨率云渲染 + Bayer 时域抖动（cloudUseQuarterRes()==true），或全分辨率直写重建 RT
layout(local_size_x=8,local_size_y=8) in;

void main()
{
    const bool bQuarterRes = cloudUseQuarterRes();

    ivec2 texSize = bQuarterRes
        ? imageSize(imageCloudRenderTexture)
        : imageSize(imageCloudReconstructionTexture);
    ivec2 workPos = ivec2(gl_GlobalInvocationID);

    if(workPos.x >= texSize.x || workPos.y >= texSize.y)
    {
        return;
    }

    ivec2 fullResSize;
    ivec2 fullResWorkPos;
    if(bQuarterRes)
    {
        uint bayerIndex = frameData.frameIndex.x % 16;
        ivec2 bayerOffset = ivec2(kBayerMatrix16[bayerIndex] % 4, kBayerMatrix16[bayerIndex] / 4);
        // The final 4x4 tile may be partial after an arbitrary window resize.
        fullResSize = imageSize(imageCloudReconstructionTexture);
        fullResWorkPos = min(workPos * 4 + bayerOffset, fullResSize - 1);
    }
    else
    {
        fullResSize = texSize;
        fullResWorkPos = workPos;
    }

    const vec2 uv = (vec2(fullResWorkPos) + vec2(0.5)) / vec2(fullResSize);

    // Use the full pixel's spatial stratification in both sparse and fallback
    // traces. Changing integration noise each frame makes an unchanged volume
    // flicker and forces temporal reconstruction to hide it with long trails.
    float blueNoise2 = samplerBlueNoiseErrorDistribution_128x128_OptimizedFor_2d2d2d2d(
        uint(fullResWorkPos.x), uint(fullResWorkPos.y), 0u, 0u);

    vec4 clipSpace = vec4(uv.x * 2.0f - 1.0f, 1.0f - uv.y * 2.0f, 0.0, 1.0);
    vec4 viewPosH = frameData.camInvertProj * clipSpace;
    vec3 viewSpaceDir = viewPosH.xyz / viewPosH.w;
    vec3 worldDir = normalize((frameData.camInvertView * vec4(viewSpaceDir, 0.0)).xyz);

    AtmosphereParameters atmosphere = getAtmosphereParameters(frameData);

    float depth = 1e20; // no-cloud sentinel; cloud depths are ray distances in km
    vec4 fogLighting = vec4(0.0);
    vec3 airTransmittance = vec3(1.0);

    vec4 cloudColor = cloudColorCompute(atmosphere, uv, blueNoise2, depth, workPos, worldDir,
        frameData.sky.atmosphereConfig.cloudGodRay != 0, fogLighting, blueNoise2, airTransmittance);

    if(bQuarterRes)
    {
        imageStore(imageCloudRenderTexture, workPos, cloudColor);
        imageStore(imageCloudDepthTexture, workPos, vec4(depth));
        imageStore(imageCloudFogRenderTexture, workPos, fogLighting);
        imageStore(imageCloudAirTRenderTexture, workPos, vec4(airTransmittance, 1.0));
    }
    else
    {
        imageStore(imageCloudReconstructionTexture, workPos, cloudColor);
        imageStore(imageCloudDepthReconstructionTexture, workPos, vec4(depth));
        imageStore(imageCloudFogReconstructionTexture, workPos, fogLighting);
        imageStore(imageCloudAirTReconstructionTexture, workPos, vec4(airTransmittance, 1.0));
    }
}
