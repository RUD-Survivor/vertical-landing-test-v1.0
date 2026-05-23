#version 450
// 2D HUD vertex shader — screen-space quads for overlay text/UI

layout(push_constant) uniform PC {
    vec2  position;    // NDC position (-1 to 1)
    vec2  scale;       // NDC size
    vec4  color;       // tint color
    float _pad;
} pc;

layout(location = 0) in vec2 aPos;   // local quad coords (0,0)-(1,1)
layout(location = 1) in vec2 aUV;    // texture coords

layout(location = 0) out vec2 vUV;
layout(location = 1) out vec4 vColor;

void main() {
    vUV    = aUV;
    vColor = pc.color;
    gl_Position = vec4(pc.position + aPos * pc.scale, 0.0, 1.0);
}
