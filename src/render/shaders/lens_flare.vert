#version 450
// Lens flare element vertex shader.
// Vertex: vec2 aPos (quad [-1,1]², stride=8).
// All positioning is in NDC via push constants.

layout(push_constant) uniform PC {
    vec2  sunScreenPos; // sun position in NDC
    float aspect;
    float intensity;
    vec2  scale;
    vec2  offset;       // NDC offset for this element
    vec4  color;
    int   shapeType;
    float _pad[3];
} pc;

layout(location = 0) in vec2 aPos;
layout(location = 0) out vec2 vUV;

void main() {
    vUV = aPos * 0.5 + 0.5;
    vec2 pos = aPos * pc.scale;
    pos.x /= pc.aspect;
    // sunScreenPos.x < -2 is sentinel for "hidden"
    gl_Position = vec4(pos + pc.offset,
                       pc.sunScreenPos.x > -2.0 ? 0.9999 : 0.0, 1.0);
}
