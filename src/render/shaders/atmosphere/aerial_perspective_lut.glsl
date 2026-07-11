#version 460
#extension GL_GOOGLE_include_directive : enable

// ==========================================================================
// aerial_perspective_lut.glsl — Froxel 3D 体积散射 LUT 烘焙。
//
// 供体积云（cloud_common.glsl::inFroxelScatter，binding 11，空气透视：相机
// 与云之间的大气散射让远处云发雾发蓝、被"洗淡"）消费——那边的采样/混合代码
// 早就写好了（shared_atmosphere.glsl::aerialPerspectiveDepthToSlice 分片规则、
// cloud_common.glsl 里的 airPerspective 混合公式），一直没有真正的烘焙 pass，
// 绑定的是中性占位纹理。这里补上这个烘焙 pass。
//
// 每个 froxel cell（屏幕 uv × 深度 slice）代表"沿这个屏幕方向，从相机到某个
// 固定距离之间，大气本身贡献了多少散射光、把这段路径上的东西洗淡了多少"，
// 和这条视线上实际有没有云/地形完全无关——纯粹是"相机到某个距离"这段空气本身
// 的性质，所以不需要真实场景深度，直接复用 atmosphere_common.glsl::
// integrateScatteredLuminance() 的 tMaxMax 参数把积分终点卡在这个 slice 对应
// 的距离上即可（tMaxMax 本来就是"取自然穿出大气层的距离和这个上限里更小的
// 那个"，恰好就是我们要的语义）。
//
// slice→距离的映射和 cloud_common.glsl 采样时用的 sqrt 分布必须一致（见
// shared_atmosphere.glsl::kAirPerspectiveKmPerSlice 的注释：越靠近相机分辨率
// 越高），这里烘焙时按同一个公式反过来算每个 texel 中心对应的真实距离。
// ==========================================================================

#include "atmosphere_common.glsl"

layout(local_size_x=8,local_size_y=8,local_size_z=1) in;

void main()
{
    ivec3 texSize = imageSize(imageFroxelScatter);
    ivec3 workPos = ivec3(gl_GlobalInvocationID);

    if (workPos.x>=texSize.x || workPos.y>=texSize.y || workPos.z>=texSize.z)
    {
        return;
    }

    AtmosphereParameters atmosphere = getAtmosphereParameters();

    const vec2 uv = (vec2(workPos.xy)+vec2(0.5)) / vec2(texSize.xy);

    // 重建这个屏幕位置的相机视线方向——和 cloud_raymarching.glsl 完全同一套
    // 写法（Vulkan uv 顶部=0，OpenGL/这里用的 NDC 约定顶部=+1，1.0-uv.y*2.0
    // 已经把这个翻转做掉了，不需要再额外处理 Y）。
    vec4 clipSpace   = vec4(uv.x*2.0-1.0, 1.0-uv.y*2.0, 0.0, 1.0);
    vec4 viewPosH    = frameData.camInvertProj * clipSpace;
    vec3 viewSpaceDir= viewPosH.xyz/viewPosH.w;
    vec3 worldDir    = normalize((frameData.camInvertView*vec4(viewSpaceDir,0.0)).xyz);

    vec3 worldPos = convertToAtmosphereUnit(frameData.camWorldPos.xyz);
    vec3 sunDir   = -normalize(frameData.sky.direction);

    // texel 中心 z → 这个 slice 代表的真实距离（km）。cloud_common.glsl 采样时
    // 用 w=sqrt(slice/sliceCount) 当纹理坐标（越靠近相机纹理密度越高），这里
    // 反过来：已知 texel 中心的归一化坐标 wCoord=(z+0.5)/sliceCount，还原出
    // 对应的线性 slice 值 linearSlice=sliceCount×wCoord²，再换算成公里数。
    const float wCoord = (float(workPos.z)+0.5) / float(texSize.z);
    const float linearSlice = float(texSize.z) * wCoord * wCoord;
    const float sliceDepthKm = aerialPerspectiveSliceToDepth(linearSlice);

    // 每个 froxel cell 只需要少量步数——这本身就是一张很粗的体积纹理（远比
    // 屏幕分辨率低），细节靠后续三线性采样插值，不需要在烘焙时就做精细 raymarch。
    const float sampleCountIni      = 4.0;
    const float depthBufferValue    = -1.0; // 不需要真实场景深度，见文件头注释
    const bool  bMieRayPhase        = true;
    const bool  bVariableSampleCount= false;

    SingleScatteringResult ss = integrateScatteredLuminance(
        vec2(workPos.xy), worldPos, worldDir, sunDir, atmosphere,
        /*bGround*/ false, sampleCountIni, depthBufferValue, bMieRayPhase,
        /*tMaxMax*/ sliceDepthKm, bVariableSampleCount
    );

    // .rgb = 这段路径上大气自身散射进视线的光（雾色）；.a = 这段路径的不透明度
    // （云/物体的原色应该被这个比例的雾色替换掉，见 cloud_common.glsl 的
    // airPerspective 混合公式）。
    float transLuma = dot(ss.transmittance, vec3(0.299,0.587,0.114));
    float alpha = clamp(1.0-transLuma, 0.0, 1.0);

    imageStore(imageFroxelScatter, workPos, vec4(ss.scatteredLight, alpha));
}
