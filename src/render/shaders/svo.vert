#version 450
// SVO vertex shader — camera-relative, per-vertex color
// Input: Vertex3D (48 bytes, same as mesh.vert)

layout(set = 0, binding = 0) uniform FrameUBO {
    mat4  view;
    mat4  proj;
    vec3  lightDir;
    float _pad0;
    vec3  viewPos;
    float time;
} frame;

// Push constants: svoMat transforms SVO local coords to camera-relative world
layout(push_constant) uniform PC {
    mat4 svoMat;
} pc;

layout(location = 0) in vec3 aPos;
layout(location = 1) in vec3 aNormal;
layout(location = 2) in vec2 aUV;
layout(location = 3) in vec4 aColor;

layout(location = 0) out vec3 vWorldPos;
layout(location = 1) out vec3 vNormal;
layout(location = 2) out vec2 vUV;
layout(location = 3) out vec4 vColor;

void main() {
    vec4 relPos = pc.svoMat * vec4(aPos, 1.0);
    vWorldPos = relPos.xyz;

    // svoMat upper 3x3 is pure rotation (orthonormal)
    vNormal = mat3(pc.svoMat) * aNormal;
    vUV     = aUV;
    vColor  = aColor;

    gl_Position = frame.proj * frame.view * relPos;
    gl_Position.y = -gl_Position.y;
    gl_Position.z = gl_Position.z * 0.5 + gl_Position.w * 0.5;
}
