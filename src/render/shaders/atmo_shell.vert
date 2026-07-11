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
// 但顶点阶段不读，供片元阶段用。全部用标量 float（不用 vec3）——GLSL
// push_constant 块默认 std430 布局，vec3 成员要 16 字节对齐，容易和 C++ 侧
// 手算的紧凑偏移错位（这个 bug 真实发生过一次），标量没有这个陷阱。
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
    float rayleighCoeffX, rayleighCoeffY, rayleighCoeffZ;
    float mieCoeff;
    float hRayleigh;
    float hMie;
    float gMie;
    float ozoneCoeffX, ozoneCoeffY, ozoneCoeffZ;
    float ozoneCenter;
    float ozoneWidth;
    float spaceVisStart;
    float spaceVisEnd;
    float limbBrightness;
    float outerExposure;
    float sunDirX, sunDirY, sunDirZ;
    float innerExposureNear;
    float innerExposureFar;
    float limbSpaceStart;
    float limbSpaceEnd;
    float limbInsideScale;
    float limbPower;
} pc;

layout(location = 0) out vec3 vWorldPos;
layout(location = 1) out vec3 vLocalNormal;

void main() {
    vWorldPos    = pc.planetCenter.xyz + inPos * pc.outerRadius;
    vLocalNormal = inPos; // 单位球：局部坐标本身就是外法线方向
    gl_Position  = frame.proj * frame.view * vec4(vWorldPos, 1.0);

    // 已修复（实机截图定位）：frame.proj 是 OpenGL 习惯的投影矩阵（math3d.h::
    // Mat4::perspective，NDC z∈[-1,1]，Y 轴 view-space 向上对应 NDC y 为正），
    // 引擎里其它所有顶点着色器（mesh.vert 等）在算完 gl_Position 之后都手动做了
    // 这两步转换成 Vulkan 约定（Y 轴朝下、深度 z∈[0,1]）——这个文件当初漏写了，
    // 结果是壳网格在屏幕上整体上下镜像，看起来像一个跟行星脱节、悬在下方的
    // 独立圆盘（而不是套在行星外面），且深度值范围也和场景深度缓冲对不上。
    gl_Position.y = -gl_Position.y;
    gl_Position.z = gl_Position.z * 0.5 + gl_Position.w * 0.5;
}
