
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
  float timeOff = uTime * 0.00003;

  // === Zonal banding (primary visual feature) ===
  // Multi-frequency band structure with turbulent boundaries
  float bandWarp = fbm(sph * vec3(2.0, 1.0, 2.0) + vec3(timeOff * 0.5, 0.0, timeOff * 0.3), 5);
  float bands1 = sin(lat * 22.0 + bandWarp * 4.0);
  float bands2 = sin(lat * 44.0 + bandWarp * 2.0 + 1.5) * 0.3;
  float bands3 = sin(lat * 88.0 + bandWarp * 1.0 + 3.0) * 0.1;
  float bandPattern = bands1 + bands2 + bands3;
  bandPattern = bandPattern * 0.5 + 0.5; // Normalize to 0-1

  // === Turbulent eddies at band boundaries ===
  float eddies = fbm(tc * 3.0 + vec3(timeOff, lat * 5.0, timeOff * 0.7), 7);
  float edgeTurb = 1.0 - abs(bandPattern * 2.0 - 1.0); // Strongest at band edges
  eddies *= edgeTurb;

  // === Great Red Spot (GRS) ===
  vec3 grsCenter = vec3(0.4, -0.35, 0.3); // South of equator
  float grsDist = length(sph - normalize(grsCenter));
  float grsAngle = atan(sph.z - grsCenter.z, sph.x - grsCenter.x);
  float grsSpiral = fbm(vec3(grsAngle * 3.0, grsDist * 20.0, timeOff * 2.0), 4);
  float grsMask = smoothstep(0.25, 0.0, grsDist);
  grsMask *= (0.7 + 0.3 * grsSpiral);

  // === Color palette ===
  // Zones (bright, cream-white): high-altitude ammonia clouds
  vec3 zoneColor = vec3(0.92, 0.88, 0.78);
  // Belts (dark, rusty brown): lower-altitude, chromophore-rich
  vec3 beltColor1 = vec3(0.55, 0.35, 0.18);
  vec3 beltColor2 = vec3(0.70, 0.48, 0.25);
  // GRS color
  vec3 grsColor = vec3(0.75, 0.30, 0.12);
  // Polar regions (darker, more subdued)
  vec3 polarColor = vec3(0.35, 0.30, 0.28);

  // Build surface color
  vec3 beltMix = mix(beltColor1, beltColor2, noise3d(tc * 6.0));
  vec3 surfColor = mix(beltMix, zoneColor, bandPattern);

  // Add turbulent eddy details
  vec3 eddyColor = mix(surfColor, zoneColor * 1.1, eddies * 0.4);
  surfColor = mix(surfColor, eddyColor, edgeTurb * 0.6);

  // Ammonia storm clusters (bright white spots in belts)
  float storms = noise3d(tc * 8.0 + vec3(timeOff * 2.0));
  float stormMask = smoothstep(0.72, 0.85, storms) * (1.0 - bandPattern) * 0.5;
  surfColor = mix(surfColor, vec3(0.98, 0.95, 0.90), stormMask);

  // Great Red Spot
  surfColor = mix(surfColor, grsColor, grsMask * 0.85);
  // GRS inner detail: swirling spiral
  if(grsMask > 0.1) {
    float spiral = fbm(vec3(grsAngle * 6.0 + timeOff * 4.0, grsDist * 40.0, 0.0), 5);
    vec3 grsInner = mix(grsColor, vec3(0.90, 0.55, 0.30), spiral);
    surfColor = mix(surfColor, grsInner, grsMask * 0.5);
  }

  // Polar darkening
  float polarMask = smoothstep(0.6, 0.9, abs(lat));
  surfColor = mix(surfColor, polarColor, polarMask);

  // === Lighting ===
  float diff = max(dot(N, L), 0.0);
  float ambient = 0.06;
  float rim = 1.0 - max(dot(N, V), 0.0);
  float rimPow = pow(rim, 3.5);
  float NdotL = dot(N, L);
  vec3 atmosGlow = vec3(0.80, 0.65, 0.45) * rimPow * 0.35 * smoothstep(-0.1, 0.2, NdotL);

  vec3 result = surfColor * (ambient + diff * 0.85) + atmosGlow;
  FragColor = vec4(result, 1.0);
}
    
