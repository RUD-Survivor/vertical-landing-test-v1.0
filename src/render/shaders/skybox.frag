#version 450
// Procedural deep-sky: power-law stars on spherical cells + structured Milky Way.
// Tunables arrive via push constants (SkyTuneParams / SkyboxPushConstants).

layout(push_constant) uniform PC {
    mat4  invViewProj;       // 64
    float skyVibrancy;       //  4 — offset 64
    float exposure;          //  4
    float starDensity;       //  4
    float starBrightness;    //  4
    float starSize;          //  4
    float glowStrength;      //  4
    float milkyWayIntensity; //  4
    float milkyWayWidth;     //  4
    float dustStrength;      //  4
    float galacticCoreBoost; //  4
    float colorVariety;      //  4
    float bgLevel;           //  4
    float galaxyYaw;         //  4
    float galaxyPitch;       //  4
    float _pad0;
    float _pad1;
} pc;

layout(location = 0) in vec3 vRayDir;
layout(location = 0) out vec4 FragColor;

// ---- hash / noise ------------------------------------------------------------
float hash13(vec3 p) {
    p = fract(p * vec3(443.8975, 397.3213, 491.1279));
    p += dot(p, p.yzx + 19.19);
    return fract((p.x + p.y) * p.z);
}

vec3 hash33(vec3 p) {
    p = vec3(dot(p, vec3(127.1, 311.7,  74.7)),
             dot(p, vec3(269.5, 183.3, 246.1)),
             dot(p, vec3(113.5, 271.9, 124.6)));
    return fract(sin(p) * 43758.5453123);
}

float noise3d(vec3 p) {
    vec3 i = floor(p);
    vec3 f = fract(p);
    f = f * f * f * (f * (f * 6.0 - 15.0) + 10.0);
    float a = hash13(i);
    float b = hash13(i + vec3(1, 0, 0));
    float c = hash13(i + vec3(0, 1, 0));
    float d = hash13(i + vec3(1, 1, 0));
    float e = hash13(i + vec3(0, 0, 1));
    float f1 = hash13(i + vec3(1, 0, 1));
    float g = hash13(i + vec3(0, 1, 1));
    float h = hash13(i + vec3(1, 1, 1));
    return mix(mix(mix(a, b, f.x), mix(c, d, f.x), f.y),
               mix(mix(e, f1, f.x), mix(g, h, f.x), f.y), f.z);
}

float fbm(vec3 p, int oct) {
    float v = 0.0, a = 0.5;
    for (int i = 0; i < oct; i++) {
        v += noise3d(p) * a;
        p = p * 2.11 + vec3(0.31, -0.17, 0.44);
        a *= 0.5;
    }
    return v;
}

// Continuous blackbody-ish tint; t in [0,1] cool→hot→cool red giants.
vec3 starTint(float t) {
    // Piecewise smooth: blue → white → yellow → orange → red
    vec3 cool  = vec3(0.55, 0.70, 1.00);
    vec3 white = vec3(0.95, 0.97, 1.00);
    vec3 warm  = vec3(1.00, 0.92, 0.72);
    vec3 orange = vec3(1.00, 0.72, 0.42);
    vec3 red   = vec3(1.00, 0.45, 0.28);
    vec3 c;
    if (t < 0.25)      c = mix(cool,  white,  t / 0.25);
    else if (t < 0.45) c = mix(white, warm,   (t - 0.25) / 0.20);
    else if (t < 0.70) c = mix(warm,  orange, (t - 0.45) / 0.25);
    else               c = mix(orange, red,   (t - 0.70) / 0.30);
    return mix(vec3(1.0), c, pc.colorVariety);
}

// Soft Gaussian core + wider energy glow (no fake plus-shaped cross).
float starProfile(float angDist, float sigma, float glow) {
    float core = exp(-0.5 * (angDist * angDist) / max(sigma * sigma, 1e-10));
    float halo = exp(-0.5 * (angDist * angDist) / max((sigma * 4.5) * (sigma * 4.5), 1e-10));
    return core + halo * glow * 0.22;
}

// Rotate world dir into galactic frame (yaw then pitch).
vec3 toGalactic(vec3 rd) {
    float cy = cos(pc.galaxyYaw),  sy = sin(pc.galaxyYaw);
    float cp = cos(pc.galaxyPitch), sp = sin(pc.galaxyPitch);
    // yaw around Y
    vec3 r = vec3(cy * rd.x + sy * rd.z, rd.y, -sy * rd.x + cy * rd.z);
    // pitch around X'
    return vec3(r.x, cp * r.y - sp * r.z, sp * r.y + cp * r.z);
}

// One density layer: spherical cells via cube-face scaling of direction.
// Checks 3×3×3 neighbourhood so stars do not pop at cell borders.
vec3 starLayer(vec3 rd, float cellScale, float layerSeed, float densityMul,
               float sizeMul, float brightMul, float glowMul, float mwDensity) {
    vec3 col = vec3(0.0);
    vec3 p = rd * cellScale;
    vec3 base = floor(p);

    // Spawn threshold: lower = denser. Density / MW boost raise spawn rate.
    float thresh = 1.0 - clamp(0.18 * densityMul * (0.55 + 0.9 * mwDensity), 0.0, 0.92);

    for (int z = -1; z <= 1; z++)
    for (int y = -1; y <= 1; y++)
    for (int x = -1; x <= 1; x++) {
        vec3 cell = base + vec3(float(x), float(y), float(z));
        vec3 rnd = hash33(cell + vec3(layerSeed));

        // Power-law magnitude: few bright, many faint (u near 1 → rare bright).
        float u = rnd.x;
        if (u < thresh) continue;

        // Cell-centered star direction (jitter inside cell, then renormalize).
        vec3 starDir = normalize(cell + (rnd - 0.5) * 0.85 + 0.5);
        // Chord length ≈ angle for small separations (avoids acos per neighbor).
        float ang = length(rd - starDir);

        // Remap surviving u into [0,1] brightness parameter.
        float m = (u - thresh) / max(1.0 - thresh, 1e-4);
        // Steep power → mostly faint speckles + sparse beacons.
        float flux = pow(m, 6.5) * brightMul * 14.0;
        // Faint population filler (still sparse).
        flux += pow(m, 2.2) * brightMul * 0.55;

        float sigma = (0.0009 + 0.0028 * pow(m, 3.0)) * sizeMul * pc.starSize;
        float gAmt  = glowMul * pc.glowStrength * smoothstep(0.55, 0.95, m);
        float profile = starProfile(ang, sigma, gAmt);

        vec3 tint = starTint(rnd.y);
        col += tint * flux * profile;
    }
    return col;
}

void main() {
    vec3 rd = normalize(vRayDir);
    vec3 col = vec3(0.0);

    if (pc.skyVibrancy < 0.01) {
        FragColor = vec4(0.0, 0.0, 0.0, 1.0);
        return;
    }

    // ---- Galactic frame + band structure ------------------------------------
    vec3 g = toGalactic(rd);
    float galLat = asin(clamp(g.y, -1.0, 1.0));
    float galLon = atan(g.z, g.x); // [-pi, pi], ~0 near galactic centre bias below

    // Width varies with longitude (warp / local arm feel).
    float widthBase = 0.085 * pc.milkyWayWidth;
    float width = widthBase * (1.0 + 0.35 * sin(galLon * 2.0 + 0.4)
                                   + 0.18 * sin(galLon * 5.0 - 1.1));
    float band = exp(-0.5 * (galLat * galLat) / max(width * width, 1e-6));

    // Core toward lon≈0 in this frame (Sagittarius-ish).
    float coreAng = length(vec2(galLon * 0.55, galLat * 1.6));
    float core = exp(-coreAng * coreAng * 1.8) * pc.galacticCoreBoost;

    vec3 galCoord = g * 3.7;
    float neb    = fbm(galCoord * 1.35, 5);
    float detail = fbm(galCoord * 4.2 + vec3(17.0, 3.0, -9.0), 4);
    float dust   = fbm(galCoord * 3.1 + vec3(41.0, -22.0, 8.0), 5);
    // Dust lanes: stronger near midplane and core.
    float dustLane = smoothstep(0.35, 0.78, dust) * band
                   * (0.55 + 0.85 * core);
    dustLane *= pc.dustStrength;

    float mwDens = band * (0.35 + 0.65 * neb) * (1.0 - dustLane * 0.85)
                 + core * 0.55;

    // ---- Star layers (coarse → fine) ----------------------------------------
    float dens = pc.starDensity;
    float brit = pc.starBrightness;
    float glow = pc.glowStrength;

    col += starLayer(rd,  48.0, 11.0, dens * 0.55, 1.35, brit * 1.40, glow * 1.2, mwDens);
    col += starLayer(rd,  96.0, 23.0, dens * 0.85, 1.00, brit * 1.00, glow * 1.0, mwDens);
    col += starLayer(rd, 180.0, 47.0, dens * 1.10, 0.80, brit * 0.75, glow * 0.7, mwDens);
    col += starLayer(rd, 320.0, 71.0, dens * 1.35, 0.55, brit * 0.45, glow * 0.4, mwDens);

    // Dense unresolved fog of stars along the band (cheap soft grain).
    float grain = fbm(g * 55.0, 3);
    col += vec3(0.92, 0.94, 1.0) * band * grain * grain * 0.12 * dens * brit;

    // Open-cluster-like clumps along the plane.
    for (int cl = 0; cl < 2; cl++) {
        float cs = 220.0 + float(cl) * 140.0;
        vec3 p = rd * cs;
        vec3 cell = floor(p);
        for (int oy = -1; oy <= 1; oy++)
        for (int ox = -1; ox <= 1; ox++) {
            vec3 c = cell + vec3(float(ox), float(oy), 0.0);
            float cm = hash13(c + vec3(99.0 + float(cl) * 37.0));
            if (cm < 0.94) continue;
            vec3 clusterDir = normalize(c + hash33(c) - 0.5 + 0.5);
            float ang = length(rd - clusterDir);
            float mwW = mix(0.15, 1.0, band);
            if (mwW < 0.12) continue;
            float blob = exp(-0.5 * ang * ang / (0.0040 * 0.0040));
            col += vec3(1.0, 0.96, 0.88) * blob * (cm - 0.94) * 40.0 * mwW * brit;
        }
    }

    // ---- Nebular emission (structured, not flat Gaussian) --------------------
    float emission = (neb * 0.55 + detail * 0.35) * band;
    emission *= (1.0 - dustLane * 0.8);
    emission += core * (0.25 + 0.35 * neb);

    vec3 galWarm = vec3(0.55, 0.38, 0.22);
    vec3 galHot  = vec3(0.35, 0.42, 0.95);
    vec3 galDust = vec3(0.12, 0.08, 0.05);
    vec3 mwCol = galWarm * emission;
    mwCol += galHot  * smoothstep(0.42, 0.85, neb) * band * 0.28;
    mwCol += galWarm * core * detail * 0.55;
    mwCol -= galDust * dustLane * 0.35;
    mwCol = max(mwCol, vec3(0.0));
    col += mwCol * pc.milkyWayIntensity;

    // Residual cosmic background
    col += vec3(0.0016, 0.0020, 0.0035) * pc.bgLevel;

    col *= pc.skyVibrancy * pc.exposure;
    // Soft roll-off (keeps HDR headroom better than hard Reinhard mid-grey).
    col = col / (col + vec3(1.0));

    FragColor = vec4(col, 1.0);
}
