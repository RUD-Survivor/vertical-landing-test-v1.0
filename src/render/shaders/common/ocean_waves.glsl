#ifndef OCEAN_WAVES_GLSL
#define OCEAN_WAVES_GLSL
#include "ocean_resources.glsl"
const float OCEAN_PI = 3.141592653589793;
// Shared with ocean_model.h. Direction vectors live in planet-local 3D space:
// no longitude chart, pole singularity, cube-face seam or floating-origin phase.
const vec3 OCEAN_DIRECTIONS[12] = vec3[](
    vec3(1,0.17,0.31), vec3(0.73,0.24,0.64), vec3(0.92,-0.2,-0.36),
    vec3(0.6,0.51,0.62), vec3(0.84,0.1,-0.53), vec3(0.51,-0.35,0.79),
    vec3(0.34,0.81,0.47), vec3(-0.46,0.22,0.86), vec3(0.77,-0.56,0.3),
    vec3(-0.68,0.66,0.31), vec3(0.19,0.93,-0.32), vec3(0.42,-0.8,0.43));
const float OCEAN_LENGTHS[12] = float[](512,192,72,32,18,10,6,4,2.8,1.8,1.2,0.8);
const float OCEAN_AMPLITUDES[12] = float[](0.65,0.4,0.22,0.15,0.1,0.07,0.045,0.03,0.021,0.013,0.009,0.006);
struct OceanWaves { float heightM; vec3 slope; float variance; float crest; };
OceanWaves oceanWaves(vec3 p, float radiusM, float time, float footprintM, float depthM) {
    OceanWaves w = OceanWaves(0.0, vec3(0), 0.0, 0.0);
    float wetFade = smoothstep(0.0, 3.0, depthM);
    for (int i = 0; i < 12; ++i) {
        vec3 d = normalize(OCEAN_DIRECTIONS[i]);
        vec3 tangentD = d - p * dot(p, d);
        float k = 2.0 * OCEAN_PI / OCEAN_LENGTHS[i];
        float a = OCEAN_AMPLITUDES[i] * wetFade;
        // Nyquist fade: unresolved slope energy becomes BRDF variance, not sparkle.
        float resolved = 1.0 - smoothstep(0.18, 0.5, footprintM / OCEAN_LENGTHS[i]);
        float phase = k * dot((p - ocean.anchorRadius.xyz) * radiusM, d)
            + ocean.phases[i / 4][i % 4];
        // Only long bands displace geometry. Short bands modify the normal.
        if (i < 3) w.heightM += a * sin(phase);
        w.slope += tangentD * (a * k * cos(phase) * resolved);
        w.variance += 0.5 * a * a * k * k * dot(tangentD, tangentD)
            * (1.0 - resolved * resolved);
        w.crest += a * k * sin(phase) * resolved;
    }
    return w;
}
#endif
