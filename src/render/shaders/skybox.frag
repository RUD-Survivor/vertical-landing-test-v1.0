#version 450

layout(push_constant) uniform PC {
    mat4  invViewProj;
    float skyVibrancy;
    float _pad[3];
} pc;

layout(location = 0) in vec3 vRayDir;
layout(location = 0) out vec4 FragColor;

float hash13(vec3 p) {
    p = fract(p * vec3(443.897, 441.423, 437.195));
    p += dot(p, p.yzx + 19.19);
    return fract((p.x + p.y) * p.z);
}

float noise3d(vec3 p) {
    vec3 i = floor(p); vec3 f = fract(p);
    f = f*f*f*(f*(f*6.0-15.0)+10.0);
    float a = hash13(i), b = hash13(i+vec3(1,0,0));
    float c = hash13(i+vec3(0,1,0)), d = hash13(i+vec3(1,1,0));
    float e = hash13(i+vec3(0,0,1)), f1= hash13(i+vec3(1,0,1));
    float g = hash13(i+vec3(0,1,1)), h = hash13(i+vec3(1,1,1));
    return mix(mix(mix(a,b,f.x),mix(c,d,f.x),f.y),
               mix(mix(e,f1,f.x),mix(g,h,f.x),f.y),f.z);
}

float fbm(vec3 p, int oct) {
    float v = 0.0, a = 0.5;
    for (int i=0; i<oct; i++) { v += noise3d(p)*a; p = p*2.03+vec3(.31,-.17,.44); a *= 0.49; }
    return v;
}

void main() {
    vec3 rd = normalize(vRayDir);
    vec3 col = vec3(0.0);

    if (pc.skyVibrancy < 0.01) {
        FragColor = vec4(0.0, 0.0, 0.0, 1.0);
        return;
    }

    // === Star field (4 density layers) ===
    for (int layer = 0; layer < 4; layer++) {
        float cellSize = 60.0 + float(layer) * 100.0;
        vec3 cell = floor(rd * cellSize);
        vec3 localPos = fract(rd * cellSize) - 0.5;

        float sx = hash13(cell + vec3(0,0,float(layer)*17.0)) - 0.5;
        float sy = hash13(cell + vec3(1,0,float(layer)*17.0)) - 0.5;
        float sz = hash13(cell + vec3(0,1,float(layer)*17.0)) - 0.5;
        vec3 starPos = vec3(sx, sy, sz) * 0.8;
        float dist = length(localPos - starPos);
        float mag  = hash13(cell + vec3(2,3,float(layer)*7.0));

        if (mag > 0.65) {
            float brt = (mag - 0.65) / 0.35;
            brt = brt * brt * brt * 8.0;
            float sz2 = 0.006 + brt * 0.015;
            float star = smoothstep(sz2, sz2 * 0.1, dist);

            float temp = hash13(cell + vec3(5,7,float(layer)));
            vec3 sc;
            if      (temp < 0.10) sc = vec3(0.5, 0.7, 1.0);
            else if (temp < 0.30) sc = vec3(0.8, 0.9, 1.0);
            else if (temp < 0.55) sc = vec3(1.0, 1.0, 0.95);
            else if (temp < 0.80) sc = vec3(1.0, 0.95, 0.8);
            else                  sc = vec3(1.0, 0.7, 0.5);

            float cross = 0.0;
            if (brt > 0.8) {
                float dx = abs(localPos.x - starPos.x);
                float dy = abs(localPos.y - starPos.y);
                cross = exp(-dx*60.0)*exp(-dy*250.0) + exp(-dy*60.0)*exp(-dx*250.0);
                cross *= (brt - 0.8) * 3.0;
            }
            col += sc * (star * brt + cross * 0.6);
        }
    }

    // === Milky Way ===
    float sinGal = 0.8829; float cosGal = 0.4695;
    float galLat = rd.y * cosGal - (rd.x * 0.6 + rd.z * 0.35) * sinGal;
    float galLon = atan(rd.z, rd.x);

    float bandWidth = 0.10 + 0.05 * sin(galLon * 2.0);
    float band = exp(-galLat * galLat / (2.0 * bandWidth * bandWidth));

    vec3 galCoord = rd * 4.0;
    float neb       = fbm(galCoord * 1.5, 6);
    float nebDetail = fbm(galCoord * 4.0 + vec3(100.0), 5);
    float dust      = fbm(galCoord * 3.5 + vec3(50.0, -30.0, 20.0), 5);
    float dustAbs   = smoothstep(0.3, 0.7, dust) * band;

    float galEmission = neb * band * 0.8 + nebDetail * band * 0.4;
    galEmission *= (1.0 - dustAbs * 0.75);

    vec3 galCore = vec3(0.35, 0.25, 0.15) * galEmission;
    vec3 warmNeb = vec3(1.2, 0.6, 0.2) * nebDetail * band * 0.4;
    vec3 blueNeb = vec3(0.2, 0.3, 0.8) * smoothstep(0.4, 0.9, neb) * band * 0.3;

    for (int cl = 0; cl < 3; cl++) {
        float cs = 250.0 + float(cl) * 150.0;
        vec3 cc = floor(rd * cs);
        float cm = hash13(cc + vec3(99.0 + float(cl)*50.0));
        if (cm > 0.88 && band > 0.1) {
            vec3 lp = fract(rd * cs) - 0.5;
            float cd = length(lp);
            col += vec3(1.0, 0.95, 0.8) * smoothstep(0.012, 0.0, cd) * (cm-0.88)*15.0 * band;
        }
    }

    col += galCore + warmNeb + blueNeb;
    col += vec3(0.002, 0.003, 0.006);
    col *= pc.skyVibrancy;
    col = col / (col + 0.5); // Reinhard tone mapping

    FragColor = vec4(col, 1.0);
}
