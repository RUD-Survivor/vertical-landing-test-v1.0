#version 450

layout(set = 0, binding = 0) uniform FrameUBO {
    mat4  view;
    mat4  proj;
    vec3  lightDir;
    float _pad0;
    vec3  viewPos;
    float time;
} frame;

layout(push_constant) uniform PC {
    mat4  model;
    vec4  baseColor;
    vec4  planetCenter;
    float ambientStr;
    int   hasTexture;
} pc;

layout(location = 0) in vec3 vWorldPos;
layout(location = 1) in vec3 vNormal;
layout(location = 2) in vec2 vUV;
layout(location = 3) in vec4 vColor;

layout(location = 0) out vec4 FragColor;

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

float craterNoise(vec3 p, float freq) {
    float n = noise3d(p * freq);
    float bowl = smoothstep(0.55, 0.35, n);
    float rim = smoothstep(0.55, 0.60, n) * smoothstep(0.68, 0.60, n);
    return bowl * 0.6 + rim * 0.25;
}

void main() {
    vec3 N = normalize(vNormal); vec3 L = normalize(frame.lightDir);
    vec3 sph = normalize(vWorldPos - pc.planetCenter.xyz);
    vec3 tc = sph * 5.0;

    vec3 warp = vec3(fbm(sph * 2.0, 3), fbm(sph * 2.0 + vec3(10.0), 3), fbm(sph * 2.0 + vec3(20.0), 3));
    float maria = fbm(sph * 1.5 + warp * 0.8, 5);
    bool isMare = maria < 0.42;

    float c1 = craterNoise(sph, 6.0);
    float c2 = craterNoise(sph, 12.0) * 0.5;
    float c3 = craterNoise(sph, 24.0) * 0.25;
    float c4 = craterNoise(sph, 48.0) * 0.12;
    float totalCrater = c1 + c2 + c3 + c4;

    float regolith = fbm(tc * 8.0, 5) * 0.15;

    vec3 highlandColor = vec3(0.22, 0.21, 0.19);
    vec3 mareColor     = vec3(0.08, 0.08, 0.07);
    vec3 rayColor      = vec3(0.28, 0.27, 0.25);

    vec3 baseColor;
    if (isMare) {
        float mareBlend = smoothstep(0.42, 0.30, maria);
        baseColor = mix(highlandColor, mareColor, mareBlend);
    } else {
        baseColor = highlandColor;
    }

    baseColor *= (1.0 - totalCrater * 0.3);
    baseColor += vec3(regolith);

    float rayNoise = noise3d(sph * 30.0);
    float rayMask  = smoothstep(0.4, 0.7, c1) * smoothstep(0.5, 0.7, rayNoise);
    baseColor = mix(baseColor, rayColor, rayMask * 0.5);

    float diff = max(dot(N, L), 0.0);
    float ambient = 0.005;
    vec3 result = baseColor * (ambient + diff * 0.995);
    FragColor = vec4(result, 1.0);
}
