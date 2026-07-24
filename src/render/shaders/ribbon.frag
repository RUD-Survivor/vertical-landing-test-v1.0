#version 450

layout(push_constant) uniform PC {
    mat4  model;
    float coreEdge; // |qSide| < coreEdge → 不透明核心；coreEdge..1 → AA 余量羽化
    float _pad0;
    float _pad1;
    float _pad2;
} pc;

layout(location = 0) in vec4 vColor;
layout(location = 1) in float vQSide;
layout(location = 0) out vec4 FragColor;

// 细线 AA 关键：几何比视觉宽，外侧有像素可淡出。
// alpha = smoothstep(0, pad, 1-|qSide|)，pad 至少约 1 屏像素（fwidth）。
void main() {
    float s = abs(vQSide);
    float pad = max(1.0 - pc.coreEdge, 1e-4);
    float aa = max(fwidth(s), 1e-4);
    float fade = max(pad, aa);
    float alpha = smoothstep(0.0, fade, 1.0 - s);
    FragColor = vec4(vColor.rgb, vColor.a * alpha);
}
