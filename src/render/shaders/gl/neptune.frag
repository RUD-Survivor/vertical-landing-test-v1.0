
#version 330 core
in vec3 vWorldPos; in vec3 vNormal; in vec3 vLocalPos;
uniform vec3 uLightDir; uniform vec3 uViewPos; uniform vec4 uBaseColor; uniform float uTime;
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

void main() {
  vec3 N = normalize(vNormal); vec3 L = normalize(uLightDir);
  vec3 V = normalize(uViewPos - vWorldPos);
  vec3 sph = normalize(vLocalPos);
  vec3 tc = sph * 5.0;
  float lat = sph.y;
  float timeOff = uTime * 0.00004;

  // Apply wind rotation around Z-axis (poles are on Z)
  float w1 = timeOff * -2.0; // Slow, uniform rotation
  float s1 = sin(w1); float c1 = cos(w1);
  vec3 windSph = vec3(sph.x * c1 - sph.y * s1, sph.x * s1 + sph.y * c1, sph.z);

  // === Strong banding (Neptune has more visible features than Uranus) ===
  float bandWarp = fbm(windSph * vec3(2.0, 1.0, 2.0), 5);
  float bands = sin(lat * 16.0 + bandWarp * 3.5) * 0.5 + 0.5;
  float fineBands = sin(lat * 40.0 + bandWarp * 1.5) * 0.1;
  bands = clamp(bands + fineBands, 0.0, 1.0);

  // === Great Dark Spot (GDS) analog ===
  float gAngle = timeOff * -2.0;
  float sg = sin(gAngle); float cg = cos(gAngle);
  vec3 gCenter = vec3(-0.3, -0.3, 0.5);
  vec3 gdsCenter = vec3(gCenter.x * cg - gCenter.z * sg, gCenter.y, gCenter.x * sg + gCenter.z * cg);
  float gdsDist = length(sph - normalize(gdsCenter));
  float gdsMask = smoothstep(0.2, 0.0, gdsDist);

  // === Methane cirrus streaks (bright white wispy clouds) ===
  // Slightly faster uniform wind for cirrus clouds (Z-axis rotation)
  float w2 = timeOff * -4.0;
  float s2 = sin(w2); float c2 = cos(w2);
  vec3 windSph2 = vec3(sph.x * c2 - sph.y * s2, sph.x * s2 + sph.y * c2, sph.z);
  float cirrus = fbm(windSph2 * vec3(8.0, 2.0, 8.0) + vec3(0.0, lat * 5.0, 0.0), 6);
  float cirrusMask = smoothstep(0.55, 0.75, cirrus) * 0.6;

  // === Color palette ===
  vec3 deepBlue = vec3(0.10, 0.15, 0.55);
  vec3 brightBlue = vec3(0.20, 0.35, 0.75);
  vec3 indigo = vec3(0.12, 0.10, 0.45);
  vec3 gdsColor = vec3(0.06, 0.08, 0.35); // Very dark blue
  vec3 cirrusWhite = vec3(0.85, 0.88, 0.95);
  vec3 polarDark = vec3(0.08, 0.10, 0.35);

  vec3 surfColor = mix(deepBlue, brightBlue, bands);
  surfColor = mix(surfColor, indigo, (1.0 - bands) * 0.3);

  // Wind-driven streaky detail
  float streaks = fbm(windSph * 20.0, 5) * 0.15;
  surfColor += vec3(streaks * 0.3, streaks * 0.4, streaks);

  // Great Dark Spot
  surfColor = mix(surfColor, gdsColor, gdsMask * 0.7);

  // Bright companion clouds near the GDS
  float companion = smoothstep(0.18, 0.22, gdsDist) * smoothstep(0.30, 0.22, gdsDist);
  companion *= fbm(windSph2 * 12.0, 4);
  surfColor = mix(surfColor, cirrusWhite, companion * 0.5);

  // Methane cirrus streaks
  surfColor = mix(surfColor, cirrusWhite, cirrusMask);

  // Polar darkening
  float polarMask = smoothstep(0.6, 0.85, abs(lat));
  surfColor = mix(surfColor, polarDark, polarMask * 0.5);

  // === Lighting ===
  float diff = max(dot(N, L), 0.0);
  float ambient = 0.04;
  float rim = 1.0 - max(dot(N, V), 0.0);
  float rimPow = pow(rim, 3.0);
  float NdotL = dot(N, L);
  vec3 atmosGlow = vec3(0.20, 0.35, 0.80) * rimPow * 0.40 * smoothstep(-0.1, 0.2, NdotL);

  vec3 result = surfColor * (ambient + diff * 0.82) + atmosGlow;
  FragColor = vec4(result, 1.0);
}
    
