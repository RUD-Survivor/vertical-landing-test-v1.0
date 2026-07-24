#version 450
// Ribbon trail vertex shader.
// Vertex layout: vec3 pos(0) + vec4 color(1) + float qSide(2), stride=32 bytes.

layout(set = 0, binding = 0) uniform FrameUBO {
    mat4  view;
    mat4  proj;
    vec3  lightDir;
    float _pad0;
    vec3  viewPos;
    float time;
} frame;

layout(push_constant) uniform PC {
    mat4  model;    // 64
    float coreEdge; // 视觉核心外缘（frag 使用）
    float _pad0;
    float _pad1;
    float _pad2;
} pc;

layout(location = 0) in vec3 aPos;
layout(location = 1) in vec4 aColor;
layout(location = 2) in float qSide;

layout(location = 0) out vec4 vColor;
layout(location = 1) out float vQSide;

void main() {
    vColor = aColor;
    vQSide = qSide;
    vec4 worldPos = pc.model * vec4(aPos, 1.0);
    gl_Position = frame.proj * frame.view * worldPos;
    gl_Position.y = -gl_Position.y;
    gl_Position.z = gl_Position.z * 0.5 + gl_Position.w * 0.5;
}
