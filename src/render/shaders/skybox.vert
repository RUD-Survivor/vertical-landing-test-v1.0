#version 450
// Fullscreen triangle skybox — no vertex buffer needed, driven by gl_VertexIndex.
// Draw with vkCmdDraw(cmd, 3, 1, 0, 0).

layout(push_constant) uniform PC {
    mat4  invViewProj;       // 64
    float skyVibrancy;       //  4 — offset 64
    float exposure;          //  4
    float starDensity;       //  4
    float starBrightness;    //  4
    float starSize;          //  4
    float glowStrength;      //  4
    float milkyWayIntensity; //  4
    float milkyWayWidth;     //  4
    float dustStrength;      //  4
    float galacticCoreBoost; //  4
    float colorVariety;      //  4
    float bgLevel;           //  4
    float galaxyYaw;         //  4
    float galaxyPitch;       //  4
    float _pad0;             //  4
    float _pad1;             //  4 — total 128
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
