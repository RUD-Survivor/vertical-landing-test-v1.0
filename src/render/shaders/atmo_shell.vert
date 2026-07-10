#version 450
// atmo_shell.vert — 大气渲染重构·壳外路径顶点着色器。
// 输入是 vk_scene.h::_buildAtmoSphere() 早就建好但从未被使用过的 128×128 UV
// 单位球（position-only，stride=12，半径=1，局部坐标原点=球心）；这里按
// pc.outerRadius 缩放、pc.planetCenter 平移，变成"这个星球的大气壳"网格。

layout(location = 0) in vec3 inPos; // 单位球局部坐标，|inPos|==1

layout(set = 0, binding = 0) uniform FrameUBO {
    mat4  view;
    mat4  proj;
    vec3  lightDir;
    float _pad0;
    vec3  viewPos;
    float time;
} frame;

// 与 atmo_shell.frag / atmo_inside.frag 共用同一个 AtmoPushConstants 布局
// （vk_descriptors.h），壳网格只用到 planetCenter/outerRadius，其余字段传进来
// 但顶点阶段不读，供片元阶段用。
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
    float spaceVisStart;
    float spaceVisEnd;
    float limbBrightness;
    float _padTune;
} pc;

layout(location = 0) out vec3 vWorldPos;
layout(location = 1) out vec3 vLocalNormal;

void main() {
    vWorldPos    = pc.planetCenter.xyz + inPos * pc.outerRadius;
    vLocalNormal = inPos; // 单位球：局部坐标本身就是外法线方向
    gl_Position  = frame.proj * frame.view * vec4(vWorldPos, 1.0);
}
