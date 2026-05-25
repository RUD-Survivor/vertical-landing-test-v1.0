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
float ridgeNoise(vec3 p, int oct) {
    float v=0.0,a=0.5,prev=1.0;
    for(int i=0;i<oct;i++){float n=abs(noise3d(p));n=1.0-n;n=n*n;v+=n*a*prev;prev=n;p=p*2.07+vec3(0.131,-0.217,0.344);a*=0.48;}
    return v;
}

void main() {
    vec3 N = normalize(vNormal); vec3 L = normalize(frame.lightDir);
    vec3 V = normalize(frame.viewPos - vWorldPos);
    vec3 sph = normalize(vWorldPos - pc.planetCenter.xyz);
    vec3 tc = sph * 4.0;
    float lat = sph.z;
    float absLat = abs(lat);

    vec3 warp = vec3(fbm(tc, 3), fbm(tc + vec3(5.2), 3), fbm(tc + vec3(9.1), 3));
    float elevation = fbm(tc * 1.5 + warp * 1.2, 8);

    float ridges  = ridgeNoise(tc * 2.0 + vec3(42.0, 13.0, 7.0), 6);
    float canyons = 1.0 - ridgeNoise(tc * 1.5 + vec3(77.0, 21.0, 55.0), 5);

    vec3 volcCenter = vec3(0.3, 0.2, 0.5);
    float volcDist  = length(sph - volcCenter);
    float volcano   = smoothstep(0.5, 0.0, volcDist) * 0.3;

    float combinedH = elevation * 0.5 + ridges * 0.25 + volcano;

    vec3 rustRed   = vec3(0.55, 0.22, 0.08);
    vec3 darkRust  = vec3(0.35, 0.14, 0.06);
    vec3 paleOchre = vec3(0.72, 0.45, 0.22);
    vec3 darkBasalt= vec3(0.15, 0.10, 0.08);
    vec3 dustBright= vec3(0.80, 0.55, 0.30);
    vec3 iceCap    = vec3(0.90, 0.92, 0.95);

    float colorNoise = fbm(tc * 3.0, 4);
    vec3 surfColor = mix(darkRust, rustRed, elevation);
    surfColor = mix(surfColor, paleOchre, colorNoise * 0.4);

    float canyonMask = smoothstep(0.4, 0.2, canyons);
    surfColor = mix(surfColor, darkBasalt, canyonMask * 0.6);
    surfColor = mix(surfColor, darkBasalt, volcano);

    float dustMask = smoothstep(0.3, 0.5, elevation) * smoothstep(0.6, 0.45, absLat);
    surfColor = mix(surfColor, dustBright, dustMask * colorNoise * 0.3);

    float iceNoise = fbm(tc * 4.0 + vec3(200.0), 5);
    float iceEdge  = 0.75 - iceNoise * 0.08;
    float southBias = (lat < 0.0) ? 0.05 : 0.0;
    if (absLat > iceEdge - southBias) {
        float ice = smoothstep(iceEdge - southBias, iceEdge + 0.05 - southBias, absLat);
        surfColor = mix(surfColor, iceCap, ice * 0.92);
    }

    float stormTime = frame.time * 0.00005;
    float dust = fbm(sph * 2.5 + vec3(stormTime, 0.0, stormTime * 0.3), 5);
    dust = smoothstep(0.48, 0.7, dust) * 0.25;
    surfColor = mix(surfColor, dustBright, dust);

    float NdotL = dot(N, L);
    float diff   = max(NdotL, 0.0);
    float ambient = 0.008;
    float rim    = 1.0 - max(dot(N, V), 0.0);
    float rimPow = pow(rim, 4.0);
    vec3 marsAtmo = vec3(0.85, 0.55, 0.25) * rimPow * 0.25 * smoothstep(-0.1, 0.1, NdotL);

    vec3 result = surfColor * (ambient + diff * 0.92) + marsAtmo;
    FragColor = vec4(result, 1.0);
}
