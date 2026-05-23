#version 450
// Fullscreen triangle skybox — no vertex buffer needed, driven by gl_VertexIndex.
// Draw with vkCmdDraw(cmd, 3, 1, 0, 0).

layout(push_constant) uniform PC {
    mat4  invViewProj;   // 64 bytes — inverse(proj * view)
    float skyVibrancy;   //  4 bytes
    float _pad[3];       // 12 bytes
} pc;

layout(location = 0) out vec3 vRayDir;

void main() {
    // Generate fullscreen triangle covering [-1,1]² in clip space
    vec2 uv  = vec2((gl_VertexIndex << 1) & 2, gl_VertexIndex & 2);
    vec2 pos = uv * 2.0 - 1.0;
    // Place at near-far plane so it loses all depth tests (behind everything)
    gl_Position = vec4(pos, 0.9999, 1.0);
    // Flip Y back to match Vulkan convention (mesh.vert flips Y, here we reconstruct world ray)
    vec4 worldPos = pc.invViewProj * vec4(pos.x, -pos.y, 1.0, 1.0);
    vRayDir = worldPos.xyz / worldPos.w;
}
