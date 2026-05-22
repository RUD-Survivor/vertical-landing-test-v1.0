
#version 330 core
in vec3 vWorldPos;
in vec3 vNormal;
in vec2 vUV;
in vec3 vLocalPos;
uniform vec3 uLightDir;
uniform vec3 uViewPos;
uniform vec4 uBaseColor;
out vec4 FragColor;

float hash(vec3 p) {
  p = fract(p * vec3(443.897, 441.423, 437.195));
  p += dot(p, p.yzx + 19.19);
  return fract((p.x + p.y) * p.z);
}
float noise3d(vec3 p) {
  vec3 i = floor(p); vec3 f = fract(p);
  f = f * f * (3.0 - 2.0 * f);
  float n000 = hash(i); float n100 = hash(i + vec3(1,0,0));
  float n010 = hash(i + vec3(0,1,0)); float n110 = hash(i + vec3(1,1,0));
  float n001 = hash(i + vec3(0,0,1)); float n101 = hash(i + vec3(1,0,1));
  float n011 = hash(i + vec3(0,1,1)); float n111 = hash(i + vec3(1,1,1));
  float nx00 = mix(n000, n100, f.x); float nx10 = mix(n010, n110, f.x);
  float nx01 = mix(n001, n101, f.x); float nx11 = mix(n011, n111, f.x);
  float nxy0 = mix(nx00, nx10, f.y); float nxy1 = mix(nx01, nx11, f.y);
  return mix(nxy0, nxy1, f.z);
}
float fbm(vec3 p) {
  float v = 0.0; float amp = 0.5;
  for (int i = 0; i < 5; i++) { v += noise3d(p) * amp; p *= 2.1; amp *= 0.5; }
  return v;
}

void main() {
  vec3 N = normalize(vNormal);
  vec3 L = normalize(uLightDir);
  vec3 V = normalize(uViewPos - vWorldPos);
  vec3 texCoord = normalize(vLocalPos) * 5.0;
  
  float warp = fbm(texCoord + vec3(0, texCoord.y * 3.0, 0));
  float bands = sin(texCoord.y * 15.0 + warp * 5.0);
  bands = smoothstep(-1.0, 1.0, bands);
  
  vec3 color1 = uBaseColor.rgb;
  vec3 color2 = uBaseColor.rgb * 0.4 + vec3(0.3);
  vec3 surfColor = mix(color1, color2, bands);
  
  float storms = fbm(texCoord * 4.0);
  if (storms > 0.7) {
      surfColor = mix(surfColor, vec3(0.9, 0.8, 0.7), (storms - 0.7) * 3.3);
  }

  float diff = max(dot(N, L), 0.0);
  float ambient = 0.15;
  float rim = 1.0 - max(dot(N, V), 0.0);
  rim = pow(rim, 3.0);
  vec3 atmosColor = uBaseColor.rgb * rim * 0.5;

  vec3 result = surfColor * (ambient + diff * 0.85) + atmosColor;
  FragColor = vec4(result, 1.0);
}
    
