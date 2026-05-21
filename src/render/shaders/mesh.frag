#version 450

layout(set = 0, binding = 0) uniform FrameUBO {
    mat4  view;
    mat4  proj;
    vec3  lightDir;
    float _pad0;
    vec3  viewPos;
    float time;
} frame;

// Set 1, Binding 0: diffuse texture (bound per material)
layout(set = 1, binding = 0) uniform sampler2D uSampler;

layout(push_constant) uniform PC {
    mat4  model;
    vec4  baseColor;
    float ambientStr;
    int   hasTexture;
} pc;

layout(location = 0) in vec3 vWorldPos;
layout(location = 1) in vec3 vNormal;
layout(location = 2) in vec2 vUV;
layout(location = 3) in vec4 vColor;

layout(location = 0) out vec4 outColor;

void main() {
    vec3 N    = normalize(vNormal);
    vec3 L    = normalize(frame.lightDir);
    float diff = max(dot(N, L), 0.0);
    vec3 V    = normalize(frame.viewPos - vWorldPos);
    vec3 H    = normalize(L + V);
    float spec = pow(max(dot(N, H), 0.0), 32.0);

    vec4 texColor = (pc.hasTexture != 0) ? texture(uSampler, vUV) : vec4(1.0);
    vec4 color    = pc.baseColor * vColor * texColor;

    vec3 result = color.rgb * (pc.ambientStr + diff * 0.7 + spec * 0.3);
    outColor = vec4(result, color.a);
}
