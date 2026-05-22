
#version 330 core
in vec3 vWorldPos;
in vec3 vNormal;
in vec2 vUV;
in vec4 vColor;

uniform vec3 uLightDir;
uniform vec3 uViewPos;
uniform vec4 uBaseColor;
uniform float uAmbientStr;
uniform sampler2D uSampler;
uniform bool uHasTexture;

out vec4 FragColor;

void main() {
  vec3 N = normalize(vNormal);
  vec3 L = normalize(uLightDir);
  float ambient = uAmbientStr;
  float diff = max(dot(N, L), 0.0);
  vec3 V = normalize(uViewPos - vWorldPos);
  vec3 H = normalize(L + V);
  float spec = pow(max(dot(N, H), 0.0), 32.0);
  
  vec4 texColor = uHasTexture ? texture(uSampler, vUV) : vec4(1.0);
  vec4 color = uBaseColor * vColor * texColor;
  
  vec3 result = color.rgb * (ambient + diff * 0.7 + spec * 0.3);
  FragColor = vec4(result, color.a);
}
    
