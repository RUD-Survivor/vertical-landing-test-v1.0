#ifndef SHARED_CLOUD_SHADOW_GLSL
#define SHARED_CLOUD_SHADOW_GLSL

// ==========================================================================
// shared_cloud_shadow.glsl — 云→地表阴影级联（Cloud CSM）的公共契约
//
// 生产端：cloud/cloud_shadow.glsl（compute）沿太阳方向对云壳做 Beer-Lambert
//         积分，把直射透过率 T=exp(-τ) 写进级联图集（R32F，N 级横向并排）。
// 消费端：terrain.frag（以及将来的 vegetation / ocean 等任何地表着色器）调用
//         sampleCloudShadowTransmittance() 取回 T，只衰减太阳直射分量。
//
// 两端共用本文件的 CloudShadowData 结构 + 选级/采样函数，保证「矩阵约定、
// 图集切片布局、跨级混合规则」只有一处定义。新增一个消费端只需 include 本文件
// 并绑定同一个 UBO + 图集，不必改动生产端。
//
// C++ 侧逐字段镜像见 render/vulkan/vk_cloud_shadow.h::GpuCloudShadowData。
// ==========================================================================

// 级联上限。改这里必须同步 vk_cloud_shadow.h 的 kMaxCascades（有 static_assert 兜底）。
#define kCloudShadowMaxCascades 4

// 太阳角半径（弧度）。日面视直径约 0.53°，半径 0.265° ≈ 4.65e-3 rad。
// 用于按物理估算半影宽度：w ≈ 2·h·tan(θ)，h = 云底到着色点的高度。
#define kCloudShadowSunAngularRadius 4.65e-3

// 单级级联的描述。矩阵把「相对行星中心的 km 坐标」映射进该级的 [0,1] UV + 深度。
struct CloudShadowCascade
{
    mat4 viewProj;     // 世界(相对行星中心 km) → 级联内 [0,1] UV + [0,1] 深度（已含 bias）
    mat4 viewProjInv;  // 上式逆矩阵；compute 端把 texel 反投影回太阳侧近平面
    vec4 params;       // x=横向半径 km  y=texel 边长 km  z=图集 U 起点  w=图集 U 宽度
};

struct CloudShadowData
{
    CloudShadowCascade cascades[kCloudShadowMaxCascades];

    // x = 有效级数（0 = 本帧无阴影，消费端直接返回 1.0）
    // y = 阴影强度 [0,1]（1 = 纯 Beer-Lambert 物理值；<1 为美术减弱）
    // z = 云底相对地面的高度 km（算半影宽度用）
    // w = 跨级混合起点（该级归一化偏心度超过它就开始向下一级过渡）
    vec4 config;

    // x = 每条光线的积分步数  y/z/w = 预留（新增标量参数不必改结构体大小）
    vec4 marchConfig;
};

// --------------------------------------------------------------------------
// 投影与判定
// --------------------------------------------------------------------------

// 把世界坐标（相对行星中心 km）投进第 c 级，返回级联内 UV + 深度。
// 正交投影 w 恒为 1，且矩阵已含 [0,1] bias，所以直接取 xyz。
vec3 cloudShadowProject(in CloudShadowData data, int c, vec3 posRelPlanetKm)
{
    return (data.cascades[c].viewProj * vec4(posRelPlanetKm, 1.0)).xyz;
}

// 归一化偏心度：0 = 正中心，1 = 贴该级边界。选级与跨级混合都用它。
float cloudShadowFit(vec3 uvd)
{
    vec2 d = abs(uvd.xy - 0.5) * 2.0;
    return max(d.x, d.y);
}

bool cloudShadowInside(vec3 uvd)
{
    return uvd.z >= 0.0 && uvd.z <= 1.0 && cloudShadowFit(uvd) <= 1.0;
}

// --------------------------------------------------------------------------
// 单级采样
// --------------------------------------------------------------------------

// 级联内 UV → 图集 UV。N 级横向并排，所以只缩放/偏移 U。
// 先把 UV 夹在半 texel 内边，避免 PCF taps 跨到相邻级的切片上取到别人的数据。
vec2 cloudShadowAtlasUv(in CloudShadowData data, int c, vec2 uvCascade, float halfTexelUv)
{
    vec4 p = data.cascades[c].params;
    vec2 uv = clamp(uvCascade, vec2(halfTexelUv), vec2(1.0 - halfTexelUv));
    return vec2(p.z + uv.x * p.w, uv.y);
}

// 单级透过率采样，带物理尺度的 4-tap PCF。
//
// 半影：太阳不是点光源，云底高度 h 处的投影边缘有宽 w ≈ 2·h·tan(θ_sun) 的半影带。
// 把 w 换算成 texel 数作为滤波半径，并夹在 [0.5, 2]：下限保证至少抹掉一个 texel 的
// 锯齿（远级 texel 往往已经远大于真实半影，此时物理值会退化到下限，符合预期——
// 那种尺度上半影本就不可分辨）；上限避免近级把云影糊成一团。
float cloudShadowSampleCascade(in CloudShadowData data, sampler2D atlas, int c, vec2 uvCascade)
{
    vec4 p = data.cascades[c].params;

    // 该级 1 texel 在「级联内 UV」下的长度：半径 p.x（km）对应 UV 跨度 1.0。
    float texelUv = p.y / max(2.0 * p.x, 1e-6);

    float penumbraKm = 2.0 * data.config.z * kCloudShadowSunAngularRadius;
    float radiusTexels = clamp(penumbraKm / max(p.y, 1e-6) * 0.5, 0.5, 2.0);
    float rUv = radiusTexels * texelUv;
    float halfTexelUv = texelUv * 0.5;

    // 旋转网格 4-tap：比轴对齐 4-tap 少一点方向性偏差，成本相同。
    const vec2 kTaps[4] = vec2[4](
        vec2( 0.707,  0.707), vec2(-0.707,  0.707),
        vec2(-0.707, -0.707), vec2( 0.707, -0.707)
    );

    float sum = 0.0;
    for (int i = 0; i < 4; i++)
    {
        vec2 uv = cloudShadowAtlasUv(data, c, uvCascade + kTaps[i] * rUv, halfTexelUv);
        sum += texture(atlas, uv).r;
    }
    return sum * 0.25;
}

// --------------------------------------------------------------------------
// 对外主入口
// --------------------------------------------------------------------------

// 取回着色点处的太阳直射透过率 T ∈ [0,1]（1 = 完全无云遮挡）。
// posRelPlanetKm：着色点相对行星中心的位置（km），与 C++ 侧建盒时同一约定。
//
// 选级策略：级联半径由小到大，取第一个能包住该点的级 —— 即永远用可用的最细一张。
// 贴近该级边界时按偏心度向下一级线性过渡，消除级联接缝。
float sampleCloudShadowTransmittance(in CloudShadowData data, sampler2D atlas, vec3 posRelPlanetKm)
{
    int count = min(int(data.config.x), kCloudShadowMaxCascades);
    if (count <= 0) return 1.0;

    for (int c = 0; c < kCloudShadowMaxCascades; c++)
    {
        if (c >= count) break;

        vec3 uvd = cloudShadowProject(data, c, posRelPlanetKm);
        if (!cloudShadowInside(uvd)) continue;

        float t = cloudShadowSampleCascade(data, atlas, c, uvd.xy);

        float fit = cloudShadowFit(uvd);
        float blendStart = data.config.w;
        if (c + 1 < count && fit > blendStart)
        {
            vec3 uvdNext = cloudShadowProject(data, c + 1, posRelPlanetKm);
            if (cloudShadowInside(uvdNext))
            {
                float k = clamp((fit - blendStart) / max(1.0 - blendStart, 1e-4), 0.0, 1.0);
                t = mix(t, cloudShadowSampleCascade(data, atlas, c + 1, uvdNext.xy), k);
            }
        }
        return t;
    }

    // 落在所有级联之外（比地平线还远 / 背面）：按无遮挡处理，避免出现硬边界。
    return 1.0;
}

// 直射光的可见度因子：把物理透过率按强度旋钮减弱后返回。
// 消费端只应该用它乘「直射太阳项」，环境光/天空漫射不参与——云影里之所以不是
// 全黑，正是因为天空漫射仍然照得到。
float cloudShadowSunVisibility(in CloudShadowData data, sampler2D atlas, vec3 posRelPlanetKm)
{
    float strength = data.config.y;
    if (strength <= 0.0) return 1.0;
    float t = sampleCloudShadowTransmittance(data, atlas, posRelPlanetKm);
    return mix(1.0, t, clamp(strength, 0.0, 1.0));
}

#endif
