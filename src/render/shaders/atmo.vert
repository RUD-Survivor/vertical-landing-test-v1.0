#version 450
// Atmosphere vertex shader — unit sphere scaled to outerRadius, centered at planetCenter

layout(set = 0, binding = 0) uniform FrameUBO {
    mat4  view;
    mat4  proj;
    vec3  lightDir;
    float _pad0;
    vec3  viewPos;
    float time;
} frame;

layout(push_constant) uniform PC {
    vec4  planetCenter;   // offset  0
    float innerRadius;    // offset 16
    float outerRadius;    // offset 20
    float surfaceRadius;  // offset 24
    int   planetIdx;      // offset 28
    float sunVisibility;  // offset 32
    float ringInner;      // offset 36
    float ringOuter;      // offset 40
    int   frameIndex;     // offset 44
    float tuneMinAlt;     // offset 48
    float tuneMaxAlt;     // offset 52
    float tuneExtinction; // offset 56
    float _pad;           // offset 60
} pc;                     // total: 64 bytes

layout(location = 0) in vec3 aPos;
layout(location = 0) out vec3 vWorldPos;

void main() {
    // Scale unit sphere to outer atmosphere radius, translate to planet center
    vec3 worldPos = pc.planetCenter.xyz + normalize(aPos) * pc.outerRadius;
    vWorldPos = worldPos;

    gl_Position = frame.proj * frame.view * vec4(worldPos, 1.0);
    gl_Position.y = -gl_Position.y;
    gl_Position.z = gl_Position.z * 0.5 + gl_Position.w * 0.5;
}
