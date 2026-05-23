#version 450

layout(set = 0, binding = 0) uniform FrameUBO {
    mat4  view;
    mat4  proj;
    vec3  lightDir;
    float _pad0;
    vec3  viewPos;
    float time;
} frame;

layout(location = 0) in vec3 vNormal;
layout(location = 1) in vec3 vWorldPos;
layout(location = 0) out vec4 FragColor;

void main() {
    vec3 N = normalize(vNormal);
    vec3 L = normalize(frame.lightDir);
    float diff = max(dot(N, L), 0.0);
    vec3 color = vec3(0.1, 0.4, 0.05);
    FragColor = vec4(color * (0.2 + 0.8 * diff), 1.0);
}
