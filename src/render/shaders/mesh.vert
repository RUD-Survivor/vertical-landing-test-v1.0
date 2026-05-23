#version 450

// Set 0, Binding 0: per-frame data (view/proj/lighting)
layout(set = 0, binding = 0) uniform FrameUBO {
    mat4  view;
    mat4  proj;
    vec3  lightDir;
    float _pad0;
    vec3  viewPos;
    float time;
} frame;

// Push constants: per-object transform and material params
// 注意：当与星球 frag shader 配对时，push constant 块必须包含 planetCenter
// 以保持跨阶段一致性（Vulkan 规范要求 push constant 块布局相同）
layout(push_constant) uniform PC {
    mat4  model;
    vec4  baseColor;
    vec4  planetCenter;  // 仅在星球管线中使用（offset 80）
    float ambientStr;    // offset 96
    int   hasTexture;    // offset 100
} pc;

layout(location = 0) in vec3 aPos;
layout(location = 1) in vec3 aNormal;
layout(location = 2) in vec2 aUV;
layout(location = 3) in vec4 aColor;

layout(location = 0) out vec3 vWorldPos;
layout(location = 1) out vec3 vNormal;
layout(location = 2) out vec2 vUV;
layout(location = 3) out vec4 vColor;

void main() {
    vec4 worldPos = pc.model * vec4(aPos, 1.0);
    vWorldPos = worldPos.xyz;
    vNormal   = mat3(transpose(inverse(pc.model))) * aNormal;
    vUV       = aUV;
    vColor    = aColor;

    gl_Position = frame.proj * frame.view * worldPos;
    // Vulkan NDC: Y is flipped relative to OpenGL
    gl_Position.y = -gl_Position.y;
    // Vulkan depth range: [0, 1] instead of OpenGL [-1, 1]
    gl_Position.z = gl_Position.z * 0.5 + gl_Position.w * 0.5;
}
