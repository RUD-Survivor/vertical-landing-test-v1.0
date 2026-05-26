#version 450
// Atmosphere vertex shader — fullscreen triangle, no vertex buffer required.
// Three hardcoded vertices cover the entire screen; the fragment shader
// reconstructs the world-space ray direction from the NDC position.

layout(location = 0) out vec2 vNDC;

void main() {
    // Classic fullscreen triangle trick: covers [-1,1]x[-1,1] with 3 verts
    const vec2 pos[3] = vec2[](
        vec2(-1.0, -1.0),
        vec2( 3.0, -1.0),
        vec2(-1.0,  3.0)
    );
    vNDC = pos[gl_VertexIndex];
    // Output directly in Vulkan NDC — no matrix multiply, no Y-flip
    gl_Position = vec4(vNDC, 0.0, 1.0);
}
