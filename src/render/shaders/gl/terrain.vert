
#version 330 core
layout(location=0) in vec3 aPos;
layout(location=1) in vec3 aNormal;
layout(location=2) in vec2 aUV;

uniform mat4 uMVP;
uniform mat4 uModel;
uniform vec3 uCamPos;
uniform vec3 uPlanetCenterRel; 
uniform float uPlanetRadius;
uniform float uMaxElevation;
uniform float uTime;
uniform int uNodeLevel;

uniform vec3 uNodePos;
uniform vec3 uNodeSide;
uniform vec3 uNodeUp;

uniform sampler2D uTectonicMap;
uniform sampler2D uHydroMap; 
uniform sampler2D uLocalHydroMap;
uniform int uHasLocalHydro;

out vec3 vRelViewPos; 
out vec3 vNormal;
out vec2 vUV;
out vec2 vLocalUV;
out vec3 vLocalPos;
out float vElevation;
out float vWaterDepth;
out float vSlope;

// --- INDUSTRIAL BICUBIC FILTERING ---
float w0(float a) { return (1.0/6.0)*(a*(a*(-a + 3.0) - 3.0) + 1.0); }
float w1(float a) { return (1.0/6.0)*(a*a*(3.0*a - 6.0) + 4.0); }
float w2(float a) { return (1.0/6.0)*(a*a*(-3.0*a + 3.0) + 3.0*a + 1.0); }
float w3(float a) { return (1.0/6.0)*(a*a*a); }
float g0(float a) { return w0(a) + w1(a); }
float g1(float a) { return w2(a) + w3(a); }
float h0(float a) { return -1.0 + w1(a) / (w0(a) + w1(a)); }
float h1(float a) { return 1.0 + w3(a) / (w2(a) + w3(a)); }

vec4 textureBicubic(sampler2D tex, vec2 uv) {
  vec2 res = vec2(textureSize(tex, 0));
  vec2 p = uv * res - 0.5;
  vec2 f = fract(p); p -= f;
  float x_h0 = h0(f.x), x_h1 = h1(f.x);
  float y_h0 = h0(f.y), y_h1 = h1(f.y);
  float x_g0 = g0(f.x), x_g1 = g1(f.x);
  float y_g0 = g0(f.y), y_g1 = g1(f.y);
  return (texture(tex, (p + vec2(x_h0, y_h0) + 0.5) / res) * x_g0 * y_g0 +
          texture(tex, (p + vec2(x_h1, y_h0) + 0.5) / res) * x_g1 * y_g0 +
          texture(tex, (p + vec2(x_h0, y_h1) + 0.5) / res) * x_g0 * y_g1 +
          texture(tex, (p + vec2(x_h1, y_h1) + 0.5) / res) * x_g1 * y_g1);
}

vec4 sampleHydroSmooth(vec2 uv) {
    return textureBicubic(uHydroMap, uv);
}

vec3 hash33(vec3 p) {
  p = fract(p * vec3(443.897, 441.423, 437.195));
  p += dot(p, p.yzx + 19.19);
  return fract((p.xxy + p.yzz) * p.zyx);
}

float worley(vec3 p) {
  vec3 i = floor(p); vec3 f = fract(p);
  float minDist = 1.0;
  for (int z = -1; z <= 1; z++) {
    for (int y = -1; y <= 1; y++) {
      for (int x = -1; x <= 1; x++) {
        vec3 neighbor = vec3(float(x), float(y), float(z));
        vec3 point = hash33(i + neighbor);
        vec3 diff = neighbor + point - f;
        minDist = min(minDist, length(diff));
      }
    }
  }
  return minDist;
}

float hash(vec3 p) {
  p = fract(p * vec3(443.897, 441.423, 437.195));
  p += dot(p, p.yzx + 19.19);
  return fract((p.x + p.y) * p.z);
}

float noise3d(vec3 p) {
  vec3 i = floor(p); vec3 f = fract(p);
  f = f * f * f * (f * (f * 6.0 - 15.0) + 10.0);
  return mix(mix(mix(hash(i), hash(i+vec3(1,0,0)), f.x), 
                 mix(hash(i+vec3(0,1,0)), hash(i+vec3(1,1,0)), f.x), f.y),
             mix(mix(hash(i+vec3(0,0,1)), hash(i+vec3(1,0,1)), f.x), 
                 mix(hash(i+vec3(0,1,1)), hash(i+vec3(1,1,1)), f.x), f.y), f.z);
}
float fbm(vec3 p, int oct) {
  float v = 0.0, a = 0.5;
  for(int i=0; i<oct; ++i) {
    v += noise3d(p) * a;
    p = p * 2.15 + vec3(0.13, -0.21, 0.34);
    a *= 0.47;
  }
  return v;
}
float ridgedFbm(vec3 p, int oct) {
  float v = 0.0, a = 0.5, f = 1.05;
  for(int i=0; i<oct; ++i) {
    float n = 1.0 - abs(noise3d(p * f) * 2.0 - 1.0);
    v += n * n * a;
    f *= 2.07; a *= 0.48;
  }
  return v;
}
float mountainNoise(vec3 p) {
  vec3 off = vec3(fbm(p*2.0, 3), fbm(p*2.0+5.2, 3), fbm(p*2.0+2.7, 3));
  return ridgedFbm(p + off * 0.15, 6);
}
float detail10m(vec3 p) {
  // High frequency micro-cells (approx 10m scale)
  float n = noise3d(p * 2500000.0);
  n += noise3d(p * 5000000.0) * 0.5;
  return n * 0.0000015; // ~10m on 6000km sphere
}

float gullyNoise(vec3 p, float angle) {
  float ca = cos(angle), sa = sin(angle);
  float sU = p.x * ca - p.z * sa;
  float sV = p.x * sa + p.z * ca;
  // Stretch in sV direction (aligned with gradient/slope)
  vec3 pS = vec3(sU * 1500000.0, p.y * 1500000.0, sV * 40000.0);
  return ridgedFbm(pS, 3);
}

float sampleTectonic(vec2 uv) {
    vec2 res = vec2(512.0, 256.0);
    vec2 st = uv * res - 0.5;
    vec2 i = floor(st);
    vec2 f = fract(st);
    float h00 = texture(uTectonicMap, i/res).r;
    float h10 = texture(uTectonicMap, (i+vec2(1,0))/res).r;
    float h01 = texture(uTectonicMap, (i+vec2(0,1))/res).r;
    float h11 = texture(uTectonicMap, (i+vec2(1,1))/res).r;
    float base = mix(mix(h00, h10, f.x), mix(h01, h11, f.x), f.y);
    float tGrad = length(vec2(h10-h00, h01-h00));
    if (tGrad > 0.05) {
        float sx = f.x * f.x * (3.0 - 2.0 * f.x);
        float sy = f.y * f.y * (3.0 - 2.0 * f.y);
        float sharp = mix(mix(h00, h10, sx), mix(h01, h11, sx), sy);
        base = mix(base, sharp, 0.45);
    }
    return base;
}

void main() {
  vLocalUV = aPos.xz + 0.5;
  vec3 cubePos = uNodePos + aPos.x * uNodeSide + aPos.z * uNodeUp;
  vec3 normPos = normalize(cubePos);
  float fV_phi = acos(clamp(normPos.y, -1.0, 1.0));
  float fV_theta = atan(normPos.z, normPos.x);
  vec2 geoUV = vec2(fV_theta / (2.0 * 3.14159) + 0.5, fV_phi / 3.14159);
  
  float plateBase = sampleTectonic(geoUV);
  float hRefined = plateBase;
  
  float mMask = smoothstep(0.55, 0.75, plateBase);
  float pMask = 1.0 - smoothstep(0.40, 0.62, plateBase);
  float cMask = smoothstep(0.435, 0.445, plateBase) * (1.0 - smoothstep(0.455, 0.465, plateBase));

  // 1.1 Mountain Folding + Thermal Erosion
  if (mMask > 0.01) {
      float epsG = 0.005;
      float hX = (sampleTectonic(geoUV + vec2(epsG, 0.0)) - sampleTectonic(geoUV - vec2(epsG, 0.0)));
      float hY = (sampleTectonic(geoUV + vec2(0.0, epsG)) - sampleTectonic(geoUV - vec2(0.0, epsG)));
      float sAngle = atan(-hX, hY);
      float ca = cos(sAngle), sa = sin(sAngle);
      float sU = normPos.x * ca - normPos.z * sa;
      float sV = normPos.x * sa + normPos.z * ca;
      vec3 pS = vec3(sU * 0.72, normPos.y, sV * 1.28); 
      float ridges = mountainNoise(pS * 6.5);
      hRefined += ridges * 0.18 * mMask;
      if (hRefined > 0.78) hRefined -= smoothstep(0.78, 0.96, hRefined) * 0.045;
  }

  // 1.2 Biome Features (Cliffs)
  float cliffNoise = 1.0 - abs(noise3d(normPos * 180.0) * 1.5 - 0.5);
  hRefined += smoothstep(0.6, 0.9, cliffNoise) * 0.007 * cMask;

  // 2.1 River Valley Carving (V-to-U shaped)
  vec4 hydro = sampleHydroSmooth(geoUV);
  if (uHasLocalHydro == 1) {
      hydro.r = textureBicubic(uLocalHydroMap, vLocalUV).r;
  }

  // Corrected Strahler scaling: B channel is order / 255.0f
  float actualStrahler = hydro.b * 255.0; 
  if (actualStrahler >= 2.0 && plateBase > 0.445) {
      float vNoise = noise3d(normPos * 250.0 + noise3d(normPos * 120.0) * 0.004);
      // River carving depth: Order 7 ~ 50 meters
      float depth = 0.000008 * (actualStrahler / 7.0); 
      float isLow = 1.0 - smoothstep(0.45, 0.55, plateBase);
      float profile = mix(abs(vNoise * 2.0 - 1.0), pow(abs(vNoise * 2.0 - 1.0), 0.4), isLow);
      hRefined -= (1.0 - profile) * depth;
  }

  // float hydroMask = smoothstep(0.0001, 0.01, hydro.r - plateBase);
  // hRefined = mix(hRefined, hydro.r, hydroMask * 0.97);

  float sLevel = 0.45;
  float hydroDiff = hydro.r - hRefined;
  float hydroMask = smoothstep(0.000001, 0.00002, hydroDiff);
  hRefined = mix(hRefined, hydro.r, hydroMask * 1.0);

  if (hRefined < sLevel && hRefined > 0.38) {
      float shelfT = (hRefined - 0.38) / (sLevel - 0.38);
      hRefined = mix(0.395, sLevel - 0.001, smoothstep(0.0, 1.0, shelfT));
  }

  float finalH = (hRefined < sLevel) ? 0.0 : (hRefined - sLevel) / (1.0 - sLevel) * uMaxElevation / uPlanetRadius;
  
  // 10m Scale detail (Land Only)
  float currentSlope = 0.0;
  if (hRefined >= sLevel) {
      float epsS = 0.0015;
      float hX = (sampleTectonic(geoUV + vec2(epsS, 0.0)) - sampleTectonic(geoUV - vec2(epsS, 0.0)));
      float hY = (sampleTectonic(geoUV + vec2(0.0, epsS)) - sampleTectonic(geoUV - vec2(0.0, epsS)));
      currentSlope = length(vec2(hX, hY)) * 100.0;
      float sAngle = atan(-hX, hY);
      
      float d10 = detail10m(normPos);
      
      // Slope-based high frequency details
      float rockCracks = (1.0 - worley(normPos * 1800000.0)) * 0.000003;
      float gullies = gullyNoise(normPos, sAngle) * 0.000002;
      float baseNoise = (noise3d(normPos * 4000000.0) - 0.5) * 0.0000005;
      
      float sHigh = smoothstep(0.12, 0.25, currentSlope);
      float sMid = smoothstep(0.04, 0.12, currentSlope) * (1.0 - sHigh);
      float sLow = 1.0 - smoothstep(0.04, 0.12, currentSlope);
      
      finalH += (rockCracks * sHigh + gullies * sMid + baseNoise * sLow);
      finalH += d10;
  }
  vec3 kscPos = vec3(0.1436, 0.478, 0.866);
  float distToKSC = length(normPos - kscPos);
  float kscMask = smoothstep(0.12, 0.04, distToKSC); 
  
  // Boost hRefined within KSC radius to ensure land-based coloring
  hRefined = mix(hRefined, sLevel + 0.015, kscMask);
  
  // Final displacement (5m above sea level)
  finalH = mix(finalH, 0.005 / uPlanetRadius, smoothstep(0.08, 0.02, distToKSC)); 
  
  mat3 localRotScale = mat3(uModel); 
  vRelViewPos = uPlanetCenterRel + localRotScale * (normPos * (1.0 + finalH));
  vElevation = hRefined; 
  vNormal = localRotScale * normPos; 
  vUV = aUV;
  vLocalPos = normPos;
  vWaterDepth = max(0.0, hydro.r - hRefined);
  vSlope = currentSlope;
  gl_Position = uMVP * vec4(vRelViewPos, 1.0);
}
    
