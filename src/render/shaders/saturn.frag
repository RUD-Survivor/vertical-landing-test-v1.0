#version 450
// Saturn body shader — ring shadow and ring-shine computed analytically.
// pc.ambientStr is repurposed as planet radius in render units (for ring shadow intersection).
// Ring inner/outer radii are hardcoded as Saturn multiples (1.11–2.35).

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
    float ambientStr;   // = planetRadius in render units (for ring shadow)
    int   hasTexture;
} pc;

layout(location = 0) in vec3 vWorldPos;
layout(location = 1) in vec3 vNormal;
layout(location = 2) in vec2 vUV;
layout(location = 3) in vec4 vColor;

layout(location = 0) out vec4 FragColor;

const float RING_INNER = 1.11;
const float RING_OUTER = 2.35;

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
    // Local position relative to planet center (in render units)
    vec3 localPos = vWorldPos - pc.planetCenter.xyz;
    vec3 sph = normalize(localPos);
    vec3 tc = sph * 5.0;
    float lat = sph.y;
    float timeOff = frame.time * 0.00002;
    float pr = pc.ambientStr; // planet radius in render units

    // Ring shadow: ray from surface point in direction L, test intersection with ring plane (y=0)
    float ringShadow = 1.0;
    if (L.y != 0.0) {
        float t = -localPos.y / L.y;
        if (t > 0.0) {
            vec3 hit = localPos + t * L;
            float dist = length(hit.xz) / pr; // normalized to planet radii
            if (dist >= RING_INNER && dist <= RING_OUTER) {
                float tau = 0.0;
                if      (dist < 1.24) tau = 0.005;
                else if (dist < 1.53) tau = 0.1;
                else if (dist < 1.95) tau = 1.8;
                else if (dist < 2.03) tau = 0.05;
                else if (dist < 2.27) tau = 0.6;
                else if (dist < 2.35) tau = 0.1;
                ringShadow = exp(-tau / max(abs(L.y), 0.001));
            }
        }
    }

    // Ring-shine on night side
    float ringShine = 0.0;
    if (dot(N, L) < 0.0) {
        ringShine = abs(N.y) * abs(L.y) * 0.25 * 0.4;
    }

    float bandWarp = fbm(sph * vec3(1.5, 0.5, 1.5) + vec3(timeOff * 0.3), 4);
    float bands = sin(lat * 18.0 + bandWarp * 3.0) * 0.5 + 0.5;
    float fineBands = sin(lat * 50.0 + bandWarp * 1.5 + 2.0) * 0.15;
    bands = clamp(bands + fineBands, 0.0, 1.0);
    float turb = fbm(tc * 2.5 + vec3(timeOff, lat * 3.0, timeOff * 0.5), 6) * 0.2;

    float polarDist = length(sph - vec3(0.0, 1.0, 0.0));
    float hexAngle  = atan(sph.z, sph.x);
    float hexPattern = cos(hexAngle * 6.0) * 0.5 + 0.5;
    float hexMask = smoothstep(0.45, 0.15, polarDist) * hexPattern;

    vec3 goldZone  = vec3(0.90, 0.82, 0.60);
    vec3 paleBelt  = vec3(0.78, 0.70, 0.52);
    vec3 warmBelt  = vec3(0.65, 0.52, 0.32);
    vec3 hexColor  = vec3(0.55, 0.45, 0.30);
    vec3 polarColor= vec3(0.50, 0.42, 0.28);

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

    float NdotL = dot(N, L);
    float diff = max(NdotL, 0.0);
    float ambient = 0.03;
    float rim = 1.0 - max(dot(N, V), 0.0);
    float rimPow = pow(rim, 3.0);
    vec3 atmosGlow = vec3(0.88, 0.78, 0.55) * rimPow * 0.30 * smoothstep(-0.1, 0.2, NdotL);

    vec3 coreLighting = surfColor * (ambient + diff * 0.85 * ringShadow + ringShine);
    vec3 result = coreLighting + atmosGlow * ringShadow;
    FragColor = vec4(result, 1.0);
}
