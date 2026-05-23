#version 450
// SVO fragment shader — per-vertex color + triplanar noise + Phong

layout(set = 0, binding = 0) uniform FrameUBO {
    mat4  view;
    mat4  proj;
    vec3  lightDir;
    float _pad0;
    vec3  viewPos;
    float time;
} frame;

layout(push_constant) uniform PC {
    mat4  model;
    vec4  baseColor;   // unused — per-vertex color used instead
    float ambientStr;
    int   hasTexture;
} pc;

layout(location = 0) in vec3 vWorldPos;
layout(location = 1) in vec3 vNormal;
layout(location = 2) in vec2 vUV;
layout(location = 3) in vec4 vColor;
layout(location = 0) out vec4 FragColor;

float hash3(vec3 p) {
    p = fract(p * vec3(443.897, 441.423, 437.195));
    p += dot(p, p.yzx + 19.19);
    return fract((p.x + p.y) * p.z);
}

float noise3d(vec3 p) {
    vec3 i = floor(p); vec3 f = fract(p);
    f = f * f * (3.0 - 2.0 * f);
    float n000 = hash3(i),            n100 = hash3(i + vec3(1,0,0));
    float n010 = hash3(i + vec3(0,1,0)), n110 = hash3(i + vec3(1,1,0));
    float n001 = hash3(i + vec3(0,0,1)), n101 = hash3(i + vec3(1,0,1));
    float n011 = hash3(i + vec3(0,1,1)), n111 = hash3(i + vec3(1,1,1));
    float nx00 = mix(n000, n100, f.x), nx10 = mix(n010, n110, f.x);
    float nx01 = mix(n001, n101, f.x), nx11 = mix(n011, n111, f.x);
    return mix(mix(nx00, nx10, f.y), mix(nx01, nx11, f.y), f.z);
}

void main() {
    vec3 N = normalize(vNormal);
    vec3 L = normalize(frame.lightDir);
    vec3 V = normalize(frame.viewPos - vWorldPos);
    vec3 H = normalize(L + V);

    // Triplanar blend weights
    vec3 bw = abs(N);
    bw = pow(bw, vec3(4.0));
    bw /= (bw.x + bw.y + bw.z + 0.001);

    // Multi-scale noise for surface micro-detail
    vec3 wp = vWorldPos * 500.0;
    float n1 = noise3d(wp * 0.5) * bw.x + noise3d(wp.yzx * 0.5) * bw.y + noise3d(wp.zxy * 0.5) * bw.z;
    float n2 = noise3d(wp * 2.0) * bw.x + noise3d(wp.yzx * 2.0) * bw.y + noise3d(wp.zxy * 2.0) * bw.z;
    float detail = n1 * 0.6 + n2 * 0.4;

    vec3 baseColor = vColor.rgb * (0.85 + 0.3 * detail);

    float ambient   = 0.12;
    float diff      = max(dot(N, L), 0.0);
    float spec      = pow(max(dot(N, H), 0.0), 48.0) * 0.15;
    float terminator = smoothstep(-0.05, 0.15, dot(N, L));

    vec3 result = baseColor * (ambient + diff * 0.85 * terminator) + vec3(spec * terminator);
    result *= 0.85 + 0.15 * max(N.y, 0.0); // AO approx

    FragColor = vec4(result, 1.0);
}
