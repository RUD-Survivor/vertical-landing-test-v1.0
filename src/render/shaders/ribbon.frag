#version 450

layout(location = 0) in vec4 vColor;
layout(location = 1) in float vQSide;
layout(location = 0) out vec4 FragColor;

void main() {
    float alpha = 1.0 - smoothstep(0.7, 1.0, abs(vQSide));
    FragColor = vec4(vColor.rgb, vColor.a * alpha);
}
