#version 450
// 2D HUD vertex shader — screen-space quads for overlay text/UI
// 顶点数据直接存储 NDC 坐标和颜色，无需 push constants

layout(location = 0) in vec2 aPos;   // NDC 坐标 (-1..1)
layout(location = 1) in vec2 aUV;    // 纹理 UV
layout(location = 2) in vec4 aColor; // per-vertex 颜色

layout(location = 0) out vec2 vUV;
layout(location = 1) out vec4 vColor;

void main() {
    vUV    = aUV;
    vColor = aColor;
    gl_Position = vec4(aPos, 0.0, 1.0);
}
