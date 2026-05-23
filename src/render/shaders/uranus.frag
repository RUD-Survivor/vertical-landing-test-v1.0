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

void main() {
    vec3 N = normalize(vNormal); vec3 L = normalize(frame.lightDir);
    vec3 V = normalize(frame.viewPos - vWorldPos);
    vec3 sph = normalize(vWorldPos - pc.planetCenter.xyz);
    float lat = sph.y;

    float bandWarp = fbm(sph * vec3(1.0, 0.3, 1.0), 3) * 0.3;
    float bands = sin(lat * 12.0 + bandWarp * 2.0) * 0.5 + 0.5;
    float clouds = fbm(sph * 4.0, 5) * 0.08;

    vec3 paleTeal  = vec3(0.58, 0.80, 0.85);
    vec3 deepCyan  = vec3(0.45, 0.72, 0.78);
    vec3 brightPole = vec3(0.72, 0.88, 0.90);

    vec3 surfColor = mix(deepCyan, paleTeal, bands * 0.6);
    surfColor += vec3(clouds);

    float polarBright = smoothstep(0.5, 0.9, abs(lat));
    surfColor = mix(surfColor, brightPole, polarBright * 0.4);

    float NdotL = dot(N, L);
    float diff = max(NdotL, 0.0);
    float ambient = 0.04;
    float rim = 1.0 - max(dot(N, V), 0.0);
    float rimPow = pow(rim, 3.5);
    vec3 atmosGlow = vec3(0.55, 0.78, 0.85) * rimPow * 0.35 * smoothstep(-0.1, 0.2, NdotL);

    vec3 result = surfColor * (ambient + diff * 0.82) + atmosGlow;
    FragColor = vec4(result, 1.0);
}
