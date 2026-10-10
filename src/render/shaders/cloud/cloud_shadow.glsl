#version 460
#extension GL_GOOGLE_include_directive : enable
#extension GL_EXT_samplerless_texture_functions : enable

#include "cloud_common.glsl"

// ==========================================================================
// cloud_shadow.glsl — 云→地表阴影级联图集的生产端
//
// 一次 dispatch 覆盖整张图集（宽 = perCascadeDim × cascadeCount，高 = perCascadeDim），
// 级联索引由 texel 的 x 坐标推出，不需要 push constant 或多次 dispatch。
//
// 每个 texel 代表一条太阳光线：从该级正交盒的太阳侧近平面出发，沿太阳→行星方向
// 穿过云壳，按 Beer-Lambert 累积光学深度 τ = ∫σ_t ds，输出直射透过率 T = exp(-τ)。
// 地表着色点经同一个 viewProj 投影后落在同一个 texel 上（两者本就在同一条光线上），
// 所以这张图天然是"该地表点头顶那根云柱的透过率"。
//
// 级联盒的几何构造（含低太阳角为何不会漏投射体）见 vk_cloud_shadow.h 文件头。
// ==========================================================================

layout(local_size_x = 8, local_size_y = 8) in;

// 光线与云壳（[rIn, rOut] 同心球壳）求交，返回壳内区段 [t0, t1]（沿 rd 的偏移，km）。
// 球壳是解析可解的，比任何加速结构都快，也自带行星曲率修正。
bool cloudShellRange(vec3 ro, vec3 rd, float rIn, float rOut, out float t0, out float t1)
{
    t0 = 0.0;
    t1 = 0.0;

    float b = dot(ro, rd);

    // 外球：没有交点说明这条光线整体在云层之上/之外，不可能有云。
    float cOut = dot(ro, ro) - rOut * rOut;
    float discOut = b * b - cOut;
    if (discOut <= 0.0) return false;
    float sqOut = sqrt(discOut);
    float tOutEnter = -b - sqOut;
    float tOutExit  = -b + sqOut;
    if (tOutExit < 0.0) return false;
    t0 = max(tOutEnter, 0.0);

    // 内球：命中则说明光线在到达内球前就穿出了云层，积分到那里为止
    //（内球以下是晴空，再往下就钻进行星了）。
    float tEnd = tOutExit;
    float cIn = dot(ro, ro) - rIn * rIn;
    float discIn = b * b - cIn;
    if (discIn > 0.0)
    {
        float sqIn = sqrt(discIn);
        float tInEnter = -b - sqIn;
        float tInExit  = -b + sqIn;
        if      (tInEnter > t0 && tInEnter < tEnd) tEnd = tInEnter;
        else if (tInExit  > t0 && tInExit  < tEnd) tEnd = tInExit;
    }

    t1 = max(tEnd, t0 + 1e-4);
    return t1 > t0;
}

void main()
{
    ivec2 atlasSize = imageSize(imageCloudShadowTexture);
    ivec2 workPos   = ivec2(gl_GlobalInvocationID.xy);
    if (workPos.x >= atlasSize.x || workPos.y >= atlasSize.y) return;

    int cascadeCount = min(int(cloudShadowData.config.x), kCloudShadowMaxCascades);
    if (cascadeCount <= 0)
    {
        imageStore(imageCloudShadowTexture, workPos, vec4(1.0));
        return;
    }

    // 图集横向 N 级并排 → 级联索引与级内 texel 坐标。
    int dim = atlasSize.x / cascadeCount;
    int cascadeIdx = clamp(workPos.x / dim, 0, cascadeCount - 1);
    ivec2 localPos = ivec2(workPos.x - cascadeIdx * dim, workPos.y);

    AtmosphereParameters atmosphere = getAtmosphereParameters(frameData);
    const mat4 invVP = cloudShadowData.cascades[cascadeIdx].viewProjInv;

    // texel 中心 → 该级近平面（太阳侧，biased 深度 0）上的世界点。
    // 逆矩阵已含 [0,1] bias，所以 UV 与深度都直接用 [0,1] 区间的值。
    vec2 uv01 = (vec2(localPos) + vec2(0.5)) / vec2(dim);
    vec4 nearH = invVP * vec4(uv01, 0.0, 1.0);
    vec3 rayOriginKm = nearH.xyz / nearH.w;

    // 盒深度轴 = biased +z = 太阳→行星方向。近平面与包围球相切，因此起点必在云层之上。
    vec3 marchDir = normalize((invVP * vec4(0.0, 0.0, 1.0, 0.0)).xyz);

    float rIn  = atmosphere.cloudAreaStartHeight;
    float rOut = atmosphere.cloudAreaStartHeight + atmosphere.cloudAreaThickness;

    float t0, t1;
    if (!cloudShellRange(rayOriginKm, marchDir, rIn, rOut, t0, t1))
    {
        imageStore(imageCloudShadowTexture, workPos, vec4(1.0)); // 没穿过云壳 = 完全透光
        return;
    }

    // Beer-Lambert 积分。cloudMap 取米制位置、返回米制消光系数 σ_t，
    // 而这里的步长是 km，所以最后统一乘 1000 换算（与主 march 的 τ=σ×Δs_km×1000 同一约定）。
    const float kMetersPerKm = 1000.0;
    // 中文说明：防止异常参数产生零步除法或无界工作量；保留高画质调参空间。
    int   steps = clamp(int(cloudShadowData.marchConfig.x), 1, 128);
    float dt    = (t1 - t0) / float(steps);
    float tau   = 0.0;
    float t     = t0 + dt * 0.5; // 中点法：同样步数下比左端点法更接近真值

    for (int i = 0; i < steps; i++)
    {
        vec3  samplePosKm = rayOriginKm + marchDir * t;
        float normalizedHeight = (length(samplePosKm) - atmosphere.cloudAreaStartHeight)
                               / atmosphere.cloudAreaThickness;
        tau += cloudMap(samplePosKm * kMetersPerKm, normalizedHeight, cloudLodShadowDistScale()) * dt;
        t   += dt;
    }

    imageStore(imageCloudShadowTexture, workPos, vec4(exp(-tau * kMetersPerKm)));
}
