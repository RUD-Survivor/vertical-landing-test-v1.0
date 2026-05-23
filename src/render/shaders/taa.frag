#version 450

layout(location = 0) in  vec2 inUV;
layout(location = 0) out vec4 outColor;

// Set 0: TAA resolve descriptor set
layout(set = 0, binding = 0) uniform sampler2D currentColor;   // RGBA16F 当前帧
layout(set = 0, binding = 1) uniform sampler2D historyColor;   // RGBA16F 历史帧 (ping-pong)
layout(set = 0, binding = 2) uniform sampler2D depthTex;       // D32_SFLOAT 深度缓冲

// Set 0, binding 3: reprojection matrices
layout(set = 0, binding = 3, std140) uniform TAAMatrices {
    mat4 invViewProj;    // 当前帧 inverse(proj * view)
    mat4 prevViewProj;   // 上一帧 proj * view
} mats;

layout(push_constant) uniform TAAParams {
    float blendFactor;   // 0.1 = 平滑 TAA，1.0 = 禁用
} pc;

void main() {
    vec4 current = texture(currentColor, inUV);
    vec2 texelSize = 1.0 / vec2(textureSize(currentColor, 0));

    // ============================================================
    // 方差裁剪 (3×3 neighborhood, gamma=1.0)
    // ============================================================
    vec3 m1 = vec3(0.0), m2 = vec3(0.0);
    for (int x = -1; x <= 1; x++) {
        for (int y = -1; y <= 1; y++) {
            vec3 s = texture(currentColor, inUV + vec2(x, y) * texelSize).rgb;
            m1 += s;
            m2 += s * s;
        }
    }
    vec3 mu    = m1 / 9.0;
    vec3 sigma = sqrt(max(vec3(0.0), m2 / 9.0 - mu * mu));
    float gamma = 1.0;
    vec3 cMin   = mu - gamma * sigma;
    vec3 cMax   = mu + gamma * sigma;

    // ============================================================
    // 深度重投影 (Depth-based reprojection)
    // ============================================================
    float depth = texture(depthTex, inUV).r;
    bool isSkyPixel = depth >= 0.9999;

    float blend     = pc.blendFactor;
    vec2  historyUV = inUV;

    if (!isSkyPixel) {
        // 从深度重建世界坐标 → 重投影到上一帧 UV
        vec4 clipPos   = vec4(inUV * 2.0 - 1.0, depth * 2.0 - 1.0, 1.0);
        vec4 worldPos  = mats.invViewProj * clipPos;
        worldPos      /= worldPos.w;
        vec4 prevClip  = mats.prevViewProj * worldPos;
        prevClip      /= prevClip.w;
        vec2 prevUV    = prevClip.xy * 0.5 + 0.5;

        // 检查重投影 UV 是否在有效范围内
        bool valid = all(greaterThan(prevUV, vec2(0.01))) &&
                     all(lessThan(prevUV, vec2(0.99)));
        if (valid) {
            historyUV = prevUV;
            // 运动自适应混合：移动越快，当前帧权重越高
            float motion = length(prevUV - inUV);
            blend = clamp(blend + motion * 4.0, blend, 0.9);
        } else {
            blend = 1.0;  // 无效重投影 → 全用当前帧
        }
    } else {
        // 天空/大气像素：相机旋转使其快速移动，提高 blend 抑制鬼影
        blend = max(blend, 0.4);
    }

    // ============================================================
    // 采样历史帧 + 方差裁剪
    // ============================================================
    vec4 history = texture(historyColor, historyUV);
    history.rgb  = clamp(history.rgb, cMin, cMax);

    outColor = mix(history, current, blend);
}
