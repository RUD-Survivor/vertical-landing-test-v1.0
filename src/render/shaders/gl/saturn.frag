
#version 330 core
in vec3 vWorldPos; in vec3 vNormal; in vec3 vLocalPos;
uniform vec3 uLightDir; uniform vec3 uViewPos; uniform vec4 uBaseColor; uniform float uTime;
uniform float uRingInner; uniform float uRingOuter;
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
  float timeOff = uTime * 0.00002;

  // === Analytic Ring Shadow (Hardcore Realism) ===
  float ringShadow = 1.0;
  if (L.y != 0.0) {
      float t = -vLocalPos.y / L.y;
      if (t > 0.0) {
          vec3 hit = vLocalPos + t * L;
          float dist = length(hit.xz);
          if (dist >= uRingInner && dist <= uRingOuter) {
              // Use NASA Optical Depth (tau) Profile for shadow mask
              float tau = 0.0;
              if (dist < 1.24) tau = 0.005;
              else if (dist < 1.53) tau = 0.1;
              else if (dist < 1.95) tau = 1.8;
              else if (dist < 2.03) tau = 0.05;
              else if (dist < 2.27) tau = 0.6;
              else if (dist < 2.35) tau = 0.1;
              
              float transmittance = exp(-tau / abs(L.y));
              ringShadow = transmittance;
          }
      }
  }

  // === Ring-Shine (Indirect Lighting from Rings) ===
  // Sun hits rings -> Rings reflect back to planet night side
  float ringShine = 0.0;
  float ringAlbedo = 0.25; // Approximate average albedo of ice rings
  // Shine is strongest when Sun is high above ring plane (abs(L.y)) 
  // and surface faces the rings (abs(N.y))
  // Northern hemisphere (N.y > 0) sees Northern ring-face (lit if L.y > 0)
  // We use a simplified model for ambient bounce
  if (dot(N, L) < 0.0) {
      ringShine = abs(N.y) * abs(L.y) * ringAlbedo * 0.4;
  }

  // === Subtle banding structure ===
  float bandWarp = fbm(sph * vec3(1.5, 0.5, 1.5) + vec3(timeOff * 0.3), 4);
  float bands = sin(lat * 18.0 + bandWarp * 3.0) * 0.5 + 0.5;
  float fineBands = sin(lat * 50.0 + bandWarp * 1.5 + 2.0) * 0.15;
  bands = clamp(bands + fineBands, 0.0, 1.0);

  float turb = fbm(tc * 2.5 + vec3(timeOff, lat * 3.0, timeOff * 0.5), 6) * 0.2;

  float polarDist = length(sph - vec3(0.0, 1.0, 0.0));
  float hexAngle = atan(sph.z, sph.x);
  float hexPattern = cos(hexAngle * 6.0) * 0.5 + 0.5;
  float hexMask = smoothstep(0.45, 0.15, polarDist) * hexPattern;

  vec3 goldZone = vec3(0.90, 0.82, 0.60);
  vec3 paleBelt = vec3(0.78, 0.70, 0.52);
  vec3 warmBelt = vec3(0.65, 0.52, 0.32);
  vec3 hexColor = vec3(0.55, 0.45, 0.30);
  vec3 polarColor = vec3(0.50, 0.42, 0.28);

  vec3 surfColor = mix(warmBelt, goldZone, bands);
  surfColor = mix(surfColor, paleBelt, turb);

  float storm = noise3d(tc * 6.0 + vec3(timeOff * 3.0));
  float stormMask = smoothstep(0.78, 0.88, storm) * 0.4;
  surfColor = mix(surfColor, vec3(0.95, 0.92, 0.85), stormMask);
  surfColor = mix(surfColor, hexColor, hexMask * 0.5);

  float polarMask = smoothstep(0.65, 0.85, abs(lat));
  surfColor = mix(surfColor, polarColor, polarMask);

  float haze = smoothstep(0.0, 0.4, abs(lat));
  surfColor = mix(surfColor, goldZone * 1.05, (1.0 - haze) * 0.15);

  float diff = max(dot(N, L), 0.0);
  float ambient = 0.03;
  float rim = 1.0 - max(dot(N, V), 0.0);
  float rimPow = pow(rim, 3.0);
  float NdotL = dot(N, L);
  vec3 atmosGlow = vec3(0.88, 0.78, 0.55) * rimPow * 0.30 * smoothstep(-0.1, 0.2, NdotL);

  vec3 coreLighting = surfColor * (ambient + diff * 0.85 * ringShadow + ringShine);
  vec3 result = coreLighting + atmosGlow * ringShadow;
  FragColor = vec4(result, 1.0);
}
    
