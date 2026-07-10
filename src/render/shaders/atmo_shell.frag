#version 450
#extension GL_GOOGLE_include_directive : enable
#extension GL_EXT_samplerless_texture_functions : enable

// ==========================================================================
// atmo_shell.frag — 大气渲染重构·壳外路径（相机在大气层以外看行星）
//
// 只在 atmo_shell.vert 输出的壳网格实际光栅化覆盖的屏幕像素上跑（一个行星在
// 宽镜头里通常只占一小圈），比旧版整屏 raymarch 便宜得多。
//
// 已修复两个 bug（用户实机反馈+分析定位）：
//   · 黑壳：最初这里照抄壳内路径，直接查 Sky-View LUT——但 Sky-View LUT 的
//     参数化只在"观察者高度 < 大气顶(topRadius)"时有效（本工程 sky_irradiance_
//     capture.glsl 自己也有 bCanUseSkyViewLut 这个同款判断），壳外路径的相机
//     按定义就在 topRadius 以外，越界喂给 acos/sqrt 产出 NaN，NaN 采样纹理在
//     很多实现下直接读回 0 → 全黑。改成和壳内 haze 一样的单趟 raymarch +
//     Transmittance LUT（atmo_scatter_common.glsl::raymarchAtmoSegment），
//     对任意相机位置都成立，不再需要"观察者高度"这个前提。
//   · "没有锚定在行星上"：VkAtmoShellPipeline 在管线里开了 depthTest=VK_TRUE，
//     但 vk_renderer3d.h::renderAtmoAfterGeometry 开的这个 render pass 根本
//     没挂深度 attachment（深度图这时候已经是只读纹理，不能同时又当 attachment
//     写，见该函数顶部注释）——depthTest 实际上无效/未定义，壳的正反两面
//     都会被画出来且不受最近物体遮挡，叠加在一起看起来就像壳没有正确固定在
//     行星位置上。改成手动判断：① 背面（法线背对相机的那半球）直接丢弃；
//     ② 用采样到的场景深度手动做"这个壳片元是不是被更近的物体挡住"的判断
//     （管线侧 depthTest 也同步改成了 VK_FALSE，见 vk_pipeline.h）。
// ==========================================================================

#define PRIMARY_STEPS 32

#include "atmo_scatter_common.glsl"

layout(set = 0, binding = 0) uniform FrameUBO {
    mat4  view;
    mat4  proj;
    vec3  lightDir;
    float _pad0;
    vec3  viewPos;
    float time;
} frame;

layout(push_constant) uniform PC {
    vec4  planetCenter;
    float innerRadius;
    float outerRadius;
    float surfaceRadius;
    int   planetIdx;
    float sunVisibility;
    float ringInner;
    float ringOuter;
    int   frameIndex;
    float tuneMinAlt;
    float tuneMaxAlt;
    float tuneExtinction;
    float showClouds;
    vec3  rayleighCoeff;
    float mieCoeff;
    float hRayleigh;
    float hMie;
    float gMie;
    vec3  ozoneCoeff;
    float ozoneCenter;
    float ozoneWidth;
    float spaceVisStart;   // 本文件不用（壳内专用），保留字段对齐 push constant 布局
    float spaceVisEnd;
    float limbBrightness;  // imgui_atmo_tuner.h 可调，默认 6.0
    float _padTune;
} pc;

layout(location = 0) in  vec3 vWorldPos;
layout(location = 1) in  vec3 vLocalNormal;
layout(location = 0) out vec4 FragColor;

void main() {
    vec3 camPos = frame.viewPos;
    vec3 sunDir = normalize(frame.lightDir);

    vec3  toCam          = camPos - vWorldPos;
    float distToCam       = length(toCam);
    vec3  viewDirFromFrag = toCam / max(distToCam, 1e-4); // 片元指向相机

    // ── 背面剔除（手动，和三角形绕向无关）──────────────────────────────────
    // 法线背对相机（这半球此刻在行星背面，从相机看不到）直接丢弃，避免壳的
    // 前后两面同时被光栅化、混合两次导致亮度/透明度都不对，也是"看起来没有
    // 固定在行星上"的一个成因（背面泄露进来的颜色和前面叠在一起，随相机转动
    // 呈现的花纹会跟着变，像是壳没跟着行星走）。
    if (dot(vLocalNormal, viewDirFromFrag) < 0.0) discard;

    // ── 手动深度测试（管线侧已关闭 depthTest，见 vk_pipeline.h 顶部注释）──
    // 用这个壳片元自己的裁剪空间深度，和采样到的场景深度比较；场景深度更近
    // （更小，标准深度近0远1）说明这里被更近的不透明物体挡住了，丢弃。
    //
    // 已修复（实机截图定位）：frame.proj 是 OpenGL 习惯的矩阵，clip.z/clip.w
    // 直接算出来是 [-1,1]（OpenGL NDC），但 inSceneDepth 里存的是 Vulkan 深度
    // 缓冲 [0,1]（且是经过 atmo_shell.vert 里那个 *0.5+w*0.5 remap 之后才写进
    // 深度图的），两者量纲不一样，之前直接比较，效果基本是随机的——有时把叠在
    // 行星盘面上的大气误判成"被挡住"丢弃掉，有时又把伸到行星轮廓外、正对着
    // 星空背景的部分误判成"没被挡"留下来，相机一动，这两批"谁被挡住了"的判断
    // 结果就跟着变，看起来像壳在绕着屏幕中心转。这里把 myNdcDepth 也做同样的
    // remap 换算成 Vulkan 范围再比较。screenUv 同理要做 Y 翻转（Vulkan 屏幕 v
    // 向下，clip.y 是 OpenGL 习惯向上），不然采样到的是上下颠倒的深度图。
    {
        vec4  clip = frame.proj * frame.view * vec4(vWorldPos, 1.0);
        float myNdcDepth = (clip.z / clip.w) * 0.5 + 0.5;
        vec2  ndcXY    = clip.xy / clip.w;
        vec2  screenUv = vec2(ndcXY.x, -ndcXY.y) * 0.5 + 0.5;
        float sceneDepthNdc = texture(sampler2D(inSceneDepth, samp), screenUv).r;
        if (sceneDepthNdc < myNdcDepth) discard;
    }

    vec3 planetCenter = pc.planetCenter.xyz;
    vec3 rayDir = -viewDirFromFrag; // 相机看向这个片元的方向

    // vWorldPos 本身就在 outerRadius 球面上（壳网格顶点直接按 outerRadius 放的，
    // 见 atmo_shell.vert），所以这条视线在大气壳上的近交点距离就是 distToCam；
    // 远交点需要另外求——可能是壳的另一侧，也可能先被行星本体挡住。
    float tNear = distToCam;
    float tFar  = tNear;
    {
        float t0, t1;
        if (intersectSphereAtCenter(camPos, rayDir, planetCenter, pc.outerRadius, t0, t1)) {
            tFar = max(t0, t1); // 取远交点（近交点就是 tNear，即这个片元本身）
        }
        float tS0, tS1;
        if (intersectSphereAtCenter(camPos, rayDir, planetCenter, pc.surfaceRadius, tS0, tS1) && tS0 > tNear) {
            tFar = min(tFar, tS0); // 视线在到达壳的另一侧之前先撞到行星本体
        }
    }

    if (tFar <= tNear) discard;

    float cosTheta = dot(rayDir, sunDir);
    AtmoMarchResult march = raymarchAtmoSegment(
        camPos, rayDir, planetCenter, tNear, tFar, PRIMARY_STEPS,
        pc.rayleighCoeff, pc.mieCoeff, pc.hRayleigh, pc.hMie, pc.gMie,
        pc.ozoneCoeff, pc.ozoneCenter, pc.ozoneWidth,
        pc.surfaceRadius, pc.outerRadius, sunDir, cosTheta);

    // 掠射角增强：法线和"看向相机"方向越垂直（掠射/limb），这条视线在大气壳里
    // 走的路径越长，raymarch 本身已经算出了更多散射光（march.scattered 自然
    // 更亮），这里再叠一个柔和的边缘增强，让 limb 轮廓更清晰。
    float grazing = 1.0 - abs(dot(vLocalNormal, viewDirFromFrag));
    float limbBoost = 0.4 + 0.6 * pow(clamp(grazing, 0.0, 1.0), 2.0);

    float nightFactor = mix(0.05, 1.0, pc.sunVisibility);
    vec3 finalColor = march.scattered * limbBoost * nightFactor * pc.limbBrightness;
    float alpha = clamp(march.opacity * limbBoost, 0.0, 1.0);

    // ACES filmic tone mapping（与壳内路径一致）
    vec3 x = max(finalColor, vec3(0.0));
    finalColor = (x * (2.51 * x + 0.03)) / (x * (2.43 * x + 0.59) + 0.14);

    FragColor = vec4(finalColor, alpha);
}
