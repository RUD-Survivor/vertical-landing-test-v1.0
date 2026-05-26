#version 450
// Cloud vertex shader — fullscreen triangle, no vertex buffer required.

layout(location = 0) out vec2 vNDC;

void main() {
    const vec2 pos[3] = vec2[](
        vec2(-1.0, -1.0),
        vec2( 3.0, -1.0),
        vec2(-1.0,  3.0)
    );
    vNDC = pos[gl_VertexIndex];
    gl_Position = vec4(vNDC, 0.0, 1.0);
}
