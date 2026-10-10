#ifndef OCEAN_RESOURCES_GLSL
#define OCEAN_RESOURCES_GLSL
layout(set = 2, binding = 3, std140) uniform OceanData {
    mat4 invViewProj;
    vec4 anchorRadius; // local unit sphere anchor, radius in meters
    vec4 phases[3];    // rebased wave phases, radians
    vec4 sunRadius;    // world-space direction to sun, atmosphere top radius km
    vec4 rayleigh;     // scattering /km, density scale height km
    vec4 mie;          // scattering /km, density scale height km
    vec4 absorption;   // water absorption /m
    vec4 scattering;   // water scattering /m
    vec4 params;       // sun irradiance scale, sky scale, camera altitude m, Mie g
    vec4 ozone;        // extinction /km, layer center km
    vec4 ozoneShape;   // layer width km
} ocean;
#endif
