#version 450
// 2D HUD fragment shader — textured quad with alpha

layout(set = 0, binding = 0) uniform sampler2D uTexture;

layout(location = 0) in vec2 vUV;
layout(location = 1) in vec4 vColor;

layout(location = 0) out vec4 outColor;

void main() {
    float alpha = texture(uTexture, vUV).r;  // font atlas: R channel = alpha
    outColor = vec4(vColor.rgb, vColor.a * alpha);
}
