
#version 330 core
in vec3 vWorldPos; in vec3 vNormal; in vec3 vLocalPos;
uniform vec3 uLightDir; uniform vec3 uViewPos; uniform vec4 uBaseColor;
out vec4 FragColor;

float hash(vec3 p) { p=fract(p*vec3(443.897,441.423,437.195)); p+=dot(p,p.yzx+19.19); return fract((p.x+p.y)*p.z); }
float noise3d(vec3 p) {
  vec3 i=floor(p); vec3 f=fract(p); f=f*f*f*(f*(f*6.0-15.0)+10.0);
  float n000=hash(i),n100=hash(i+vec3(1,0,0)),n010=hash(i+vec3(0,1,0)),n110=hash(i+vec3(1,1,0));
  float n001=hash(i+vec3(0,0,1)),n101=hash(i+vec3(1,0,1)),n011=hash(i+vec3(0,1,1)),n111=hash(i+vec3(1,1,1));
  float nx00=mix(n000,n100,f.x),nx10=mix(n010,n110,f.x);
  float nx01=mix(n001,n101,f.x),nx11=mix(n011,n111,f.x);
  return mix(mix(nx00,nx10,f.y),mix(nx01,nx11,f.y),f.z);
}
float fbm(vec3 p, int oct) { float v=0.0,a=0.5; for(int i=0;i<oct;i++){v+=noise3d(p)*a;p=p*2.07+vec3(0.131,-0.217,0.344);a*=0.48;} return v; }

// Noise-based crater approximation (cheap, no loops)
float craterNoise(vec3 p, float freq) {
  float n = noise3d(p * freq);
  // Sharp circular-ish depressions from noise thresholding
  float bowl = smoothstep(0.55, 0.35, n); // crater interior
  float rim = smoothstep(0.55, 0.60, n) * smoothstep(0.68, 0.60, n); // raised rim
  return bowl * 0.6 + rim * 0.25;
}

void main() {
  vec3 N = normalize(vNormal); vec3 L = normalize(uLightDir);
  vec3 sph = normalize(vLocalPos);
  vec3 tc = sph * 6.0;

  // Multi-scale noise-based cratering
  float c1 = craterNoise(sph, 8.0);
  float c2 = craterNoise(sph, 16.0) * 0.5;
  float c3 = craterNoise(sph, 32.0) * 0.25;
  float c4 = craterNoise(sph, 64.0) * 0.12;
  float totalCrater = c1 + c2 + c3 + c4;

  // Base terrain noise
  float terrain = fbm(tc * 2.0, 6);

  // Lobate scarps (ridge-like features)
  float scarps = abs(noise3d(tc * 4.0 + vec3(100.0))) * abs(noise3d(tc * 8.0 + vec3(200.0)));
  scarps = smoothstep(0.1, 0.4, scarps) * 0.15;

  // Color: Mercury is very dark (albedo ~0.07-0.14)
  vec3 darkBasalt = vec3(0.08, 0.07, 0.06);
  vec3 warmBrown = vec3(0.14, 0.11, 0.08);
  vec3 brightRay = vec3(0.20, 0.18, 0.16);

  vec3 baseColor = mix(darkBasalt, warmBrown, terrain * 0.8 + 0.2);
  baseColor *= (1.0 - totalCrater * 0.4);
  float rayMask = smoothstep(0.3, 0.6, c1) * noise3d(tc * 20.0);
  baseColor = mix(baseColor, brightRay, rayMask * 0.4);
  baseColor += vec3(scarps);

  float diff = max(dot(N, L), 0.0);
  float ambient = 0.01;
  vec3 result = baseColor * (ambient + diff * 0.99);
  FragColor = vec4(result, 1.0);
}
    
