#version 450

// Full-screen triangle — no vertex buffer needed.
// gl_VertexIndex: 0→(-1,-1), 1→(3,-1), 2→(-1,3) covers the entire NDC framebuffer.

layout(location = 0) out vec2 outUV;

void main() {
    vec2 pos = vec2(
        float(gl_VertexIndex & 1) * 4.0 - 1.0,
        float(gl_VertexIndex >> 1) * 4.0 - 1.0
    );
    gl_Position = vec4(pos, 0.0, 1.0);
    outUV = pos * 0.5 + 0.5;
}
