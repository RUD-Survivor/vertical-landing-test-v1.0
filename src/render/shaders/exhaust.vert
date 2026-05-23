#version 450
// Exhaust plume vertex shader — unit box [-0.5,0.5]³, pos only (stride=12).
// MVP = proj * view * model computed in shader.

layout(set = 0, binding = 0) uniform FrameUBO {
    mat4  view;
    mat4  proj;
    vec3  lightDir;
    float _pad0;
    vec3  viewPos;
    float time;
} frame;

layout(push_constant) uniform PC {
    mat4  model;       // 64 bytes
    float throttle;    //  4 bytes
    float expansion;   //  4 bytes
    float groundDist;  //  4 bytes
    float plumeLen;    //  4 bytes
} pc;

layout(location = 0) in vec3 aPos;

layout(location = 0) out vec3 vLocalPos;
layout(location = 1) out vec3 vWorldPos;

void main() {
    vLocalPos = aPos;
    vec4 worldPos4 = pc.model * vec4(aPos, 1.0);
    vWorldPos = worldPos4.xyz;
    gl_Position = frame.proj * frame.view * worldPos4;
    gl_Position.y = -gl_Position.y;
    gl_Position.z = gl_Position.z * 0.5 + gl_Position.w * 0.5;
}
