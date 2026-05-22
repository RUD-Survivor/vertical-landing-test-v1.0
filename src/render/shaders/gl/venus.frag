
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
  float lat = sph.y;
  float timeOff = uTime * 0.00008; // Super-rotation (very slow)

  // === Cloud layer 1: Thick sulfuric acid main deck ===
  vec3 tc1 = sph * 3.0 + vec3(timeOff * 1.0, 0.0, timeOff * 0.5);
  float cloud1 = fbm(tc1, 8);
  // Banded structure (Venus has mild latitudinal banding in UV)
  float bandWarp = fbm(sph * vec3(1.0, 5.0, 1.0) + vec3(timeOff * 0.3), 4) * 0.3;
  float bands = sin(lat * 12.0 + bandWarp * 8.0) * 0.5 + 0.5;

  // === Cloud layer 2: Higher altitude wispy streaks ===
  vec3 tc2 = sph * 5.0 + vec3(-timeOff * 1.5, 50.0, timeOff * 0.8);
  float cloud2 = fbm(tc2, 6) * 0.6;

  // === Cloud layer 3: Dark UV absorber patches (mysterious!) ===
  vec3 tc3 = sph * 2.0 + vec3(timeOff * 0.7, 100.0, -timeOff * 0.4);
  float uvAbsorber = fbm(tc3, 5);
  uvAbsorber = smoothstep(0.35, 0.6, uvAbsorber) * 0.3;

  // Color palette: Venus appears yellow-white to pale cream
  vec3 brightCloud = vec3(0.95, 0.90, 0.75);
  vec3 midCloud = vec3(0.85, 0.78, 0.58);
  vec3 darkCloud = vec3(0.65, 0.55, 0.35);
  vec3 absorberTint = vec3(0.55, 0.45, 0.25); // Dark UV patches

  float cloudMix = cloud1 * 0.6 + cloud2 * 0.3 + bands * 0.1;
  vec3 surfColor = mix(darkCloud, brightCloud, cloudMix);
  surfColor = mix(surfColor, midCloud, bands * 0.4);
  surfColor = mix(surfColor, absorberTint, uvAbsorber);

  // Subtle sub-cloud volcanic glow on night side
  float NdotL = dot(N, L);
  float nightMask = smoothstep(0.0, -0.15, NdotL);
  float volcGlow = fbm(sph * 8.0, 4);
  volcGlow = smoothstep(0.55, 0.75, volcGlow) * 0.15;
  vec3 glowColor = vec3(1.0, 0.3, 0.05); // Deep orange-red
  surfColor += glowColor * volcGlow * nightMask;

  // Lighting
  float diff = max(NdotL, 0.0);
  float ambient = 0.03;

  // Atmosphere rim
  float rim = 1.0 - max(dot(N, V), 0.0);
  float rimPow = pow(rim, 3.0);
  vec3 atmosGlow = vec3(0.95, 0.82, 0.50) * rimPow * 0.5 * smoothstep(-0.1, 0.2, NdotL);

  vec3 result = surfColor * (ambient + diff * 0.85) + atmosGlow;
  FragColor = vec4(result, 1.0);
}
    
