#version 450

layout(location = 0) in  vec2 inUV;
layout(location = 0) out vec4 outColor;

// set 0: current frame RGBA16F
layout(set = 0, binding = 0) uniform sampler2D currentColor;
// set 0: history frame RGBA16F (ping-pong)
layout(set = 0, binding = 1) uniform sampler2D historyColor;
// set 0: depth buffer D32 (for future reprojection; currently unused in basic TAA)
layout(set = 0, binding = 2) uniform sampler2D depthTex;

layout(push_constant) uniform TAAParams {
    float blendFactor;  // 0.1 = smooth TAA, 1.0 = no blending (debug)
} pc;

// Clamp history to current frame's neighborhood (AABB clamp, reduces ghosting)
vec4 clipHistoryToAABB(vec4 current, vec4 history) {
    // Sample a 3x3 neighborhood to find min/max bounds
    vec2 texelSize = 1.0 / vec2(textureSize(currentColor, 0));
    vec4 minColor = current, maxColor = current;
    for (int x = -1; x <= 1; ++x) {
        for (int y = -1; y <= 1; ++y) {
            vec4 s = texture(currentColor, inUV + vec2(x, y) * texelSize);
            minColor = min(minColor, s);
            maxColor = max(maxColor, s);
        }
    }
    return clamp(history, minColor, maxColor);
}

void main() {
    vec4 current = texture(currentColor, inUV);
    vec4 history = texture(historyColor, inUV);

    history = clipHistoryToAABB(current, history);

    outColor = mix(history, current, pc.blendFactor);
}
