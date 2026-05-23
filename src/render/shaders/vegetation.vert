#version 450
// Vegetation vertex shader — instanced rendering
// Binding 0 (per-vertex):   vec3 aPos (loc 0), vec3 aNormal (loc 1)
// Binding 1 (per-instance): vec3 iPos (loc 2), float iScale (loc 3), float iRot (loc 4)

layout(set = 0, binding = 0) uniform FrameUBO {
    mat4  view;
    mat4  proj;
    vec3  lightDir;
    float _pad0;
    vec3  viewPos;
    float time;
} frame;

// Per-vertex (binding 0)
layout(location = 0) in vec3 aPos;
layout(location = 1) in vec3 aNormal;
// Per-instance (binding 1)
layout(location = 2) in vec3  iPos;
layout(location = 3) in float iScale;
layout(location = 4) in float iRot;

layout(location = 0) out vec3 vNormal;
layout(location = 1) out vec3 vWorldPos;

void main() {
    // Instance Y-axis rotation
    float s = sin(iRot), c = cos(iRot);
    mat3 rot = mat3(c, 0.0, s,  0.0, 1.0, 0.0, -s, 0.0, c);

    // Orient mesh to planet surface (iPos IS the surface world-space point)
    vec3 up      = normalize(iPos);
    vec3 right   = normalize(cross(vec3(0.0, 1.0, 0.0), up));
    if (length(right) < 0.01) right = normalize(cross(vec3(1.0, 0.0, 0.0), up));
    vec3 forward = cross(up, right);
    mat3 orient  = mat3(right, up, forward);

    vec3 localPos = (rot * aPos) * iScale;
    vec3 worldPos = orient * localPos + iPos;
    vWorldPos = worldPos;
    vNormal   = orient * (rot * aNormal);

    gl_Position = frame.proj * frame.view * vec4(worldPos, 1.0);
    gl_Position.y = -gl_Position.y;
    gl_Position.z = gl_Position.z * 0.5 + gl_Position.w * 0.5;
}
