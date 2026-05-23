#version 450

layout(location = 0) in vec2 vUV;
layout(location = 1) in vec4 vColor;
layout(location = 0) out vec4 FragColor;

void main() {
    vec2 uv = vUV * 2.0 - 1.0;
    float d = length(uv);
    float alpha = 1.0 - smoothstep(0.7, 1.0, d);
    FragColor = vec4(vColor.rgb, vColor.a * alpha);
}
