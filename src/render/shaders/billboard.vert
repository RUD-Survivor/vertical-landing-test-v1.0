#version 450
// Spherical billboard vertex shader.
// Vertex: vec2 aPos (quad [-1,1]², stride=8).
// Center/size/color passed as push constants.

layout(set = 0, binding = 0) uniform FrameUBO {
    mat4  view;
    mat4  proj;
    vec3  lightDir;
    float _pad0;
    vec3  viewPos;
    float time;
} frame;

layout(push_constant) uniform PC {
    vec4  center;  // world-space center (w unused)
    vec2  size;
    vec2  _pad;
    vec4  color;
} pc;

layout(location = 0) in vec2 aPos;
layout(location = 0) out vec2 vUV;
layout(location = 1) out vec4 vColor;

void main() {
    vUV    = aPos * 0.5 + 0.5;
    vColor = pc.color;

    // Build screen-aligned right/up vectors from view matrix
    vec3 right = vec3(frame.view[0][0], frame.view[1][0], frame.view[2][0]);
    vec3 up    = vec3(frame.view[0][1], frame.view[1][1], frame.view[2][1]);

    vec3 worldPos = pc.center.xyz
                  + right * aPos.x * pc.size.x
                  + up    * aPos.y * pc.size.y;

    gl_Position = frame.proj * frame.view * vec4(worldPos, 1.0);
    gl_Position.y = -gl_Position.y;
    gl_Position.z = gl_Position.z * 0.5 + gl_Position.w * 0.5;
}
