
#version 330 core
in vec2 vUV;
uniform sampler2D uCurrentFrame;
uniform sampler2D uHistoryFrame;
uniform sampler2D uDepthTex;
uniform float uBlendFactor;
uniform mat4 uInvViewProj;
uniform mat4 uPrevViewProj;
out vec4 FragColor;

void main() {
  vec4 current = texture(uCurrentFrame, vUV);

  // 3x3 variance clipping (tighter gamma=1.0 vs old 1.5)
  vec2 texelSize = 1.0 / vec2(textureSize(uCurrentFrame, 0));
  vec3 m1 = vec3(0.0), m2 = vec3(0.0);
  for (int x = -1; x <= 1; x++) {
    for (int y = -1; y <= 1; y++) {
      vec3 s = texture(uCurrentFrame, vUV + vec2(float(x), float(y)) * texelSize).rgb;
      m1 += s; m2 += s * s;
    }
  }
  vec3 mu = m1 / 9.0;
  vec3 sigma = sqrt(max(vec3(0.0), m2 / 9.0 - mu * mu));
  float gamma = 1.0;
  vec3 cMin = mu - gamma * sigma;
  vec3 cMax = mu + gamma * sigma;

  // Depth-based reprojection
  float depth = texture(uDepthTex, vUV).r;
  bool isSkyPixel = depth >= 0.9999;

  float blend = uBlendFactor;
  vec2 historyUV = vUV;

  if (!isSkyPixel) {
    // Surface/geometry: reproject to previous frame UV
    vec4 clipPos = vec4(vUV * 2.0 - 1.0, depth * 2.0 - 1.0, 1.0);
    vec4 worldPos = uInvViewProj * clipPos;
    worldPos /= worldPos.w;
    vec4 prevClip = uPrevViewProj * worldPos;
    prevClip /= prevClip.w;
    vec2 prevUV = prevClip.xy * 0.5 + 0.5;
    bool valid = all(greaterThan(prevUV, vec2(0.01))) && all(lessThan(prevUV, vec2(0.99)));
    if (valid) {
      historyUV = prevUV;
      float motion = length(prevUV - vUV);
      blend = clamp(blend + motion * 4.0, blend, 0.9);
    } else {
      blend = 1.0;
    }
  } else {
    // Sky / atmosphere pixel: camera rotation moves these everywhere,
    // use higher blend so current frame dominates and ghosting is suppressed.
    blend = max(blend, 0.4);
  }

  vec4 history = texture(uHistoryFrame, historyUV);
  history.rgb = clamp(history.rgb, cMin, cMax);
  FragColor = mix(history, current, blend);
}
    
