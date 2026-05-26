#pragma once
#include <glad/glad.h>
#include <string>
#include <vector>

// ─────────────────────────────────────────────────────────────────────────────
// CloudSystem — self-contained volumetric cloud GPU/CPU subsystem.
//
// Owns:
//   • Four baked GPU textures (noise 128³, cover 128³, detail 32³, weather 2048×1024)
//   • All cloud-related GLSL uniform locations
//   • Bake pipeline (called once at init, CPU-side only)
//
// Does NOT own:
//   • The atmosphere GLSL shader source (still assembled in renderer3d.h).
//     The cloud GLSL functions (curlSph, rawCoverage, cloudDensity, cloudPhase)
//     are clearly delimited in renderer3d.h with a comment block.
//   • The atmosphere OpenGL program object (passed in by the caller).
//
// Usage:
//   CloudSystem cs;
//   cs.init(atmoProg);                  // once, after shader compilation
//   // each frame:
//   cs.bind(atmoProg, time, frameIdx, camAltKm);
//   // on shutdown:
//   cs.destroy();
// ─────────────────────────────────────────────────────────────────────────────

struct CloudTuneParams {
    float covLo      = 0.150f;
    float covHi      = 0.720f;
    float threshLo   = 0.100f; // lowered to reduce percolation cutoff
    float threshHi   = 0.400f;
    float erosion    = 0.100f;
    float density    = 8.000f; // boosted for visibility
    float extinction = 0.800f;
    float minAlt     = 1.5f;
    float maxAlt     = 15.0f;
    int   debugMode  = 0;      // 0=off 1=force density for debug
};

class CloudSystem {
public:
    CloudTuneParams tuneParams;

    // Load cloud.glsl from disk; must be called BEFORE shader compilation.
    // Returns false and prints an error if the file cannot be opened.
    bool loadGLSL(const char* path);

    // Returns the loaded GLSL source (empty string if loadGLSL not yet called).
    const char* glslSource() const { return m_glsl.c_str(); }

    // Grab all uniform locations from prog and bake the four noise textures.
    // Must be called once after the atmosphere shader program has been compiled.
    void init(GLuint prog);

    // Delete all owned GPU textures (call on renderer teardown).
    void destroy();

    // Upload per-frame cloud uniforms and bind textures to texture units 8–11.
    //   time      – simulation wall-clock time in seconds (used for animation + phase)
    //   frameIdx  – TAA frame counter (for IGN dither animation)
    //   camAltKm  – camera altitude above planet surface in km (drives LOD step counts)
    void bind(GLuint prog, double time, int frameIdx, float camAltKm);

    // Re-bake weather map from procedural terrain height data.
    // Replaces the default ellipse-based land mask with the actual tectonic
    // height map, and adds planetary-wave + coastal-proximity modulation.
    //   heightMap – tectonic gridHeight (values in [0,1], seaLevel threshold for land)
    //   w, h      – dimensions of heightMap
    //   seaLevel  – threshold above which = land (default 0.45 matches Tectonic::CONT_THRESHOLD)
    void rebakeWeather(const std::vector<float>& heightMap, int w, int h, float seaLevel = 0.45f);

private:
    // GPU texture handles
    GLuint noiseT   = 0;   // 128³ RGBA8: R=PerlinWorley, G=Worley, B=fbm(3), A=fbm(5)
    GLuint coverT   = 0;   // 128³ RGBA8: R=curl_lo, G=curl_hi, B=latNoise, A=cov_med
    GLuint detailT  = 0;   // 32³  RGBA8: R/G/B=Worley(50/25/12.5m), A=blend
    GLuint weatherT = 0;   // 2048×1024 RGBA8: R=cov_bias, G=type, B=humidity, A=land

    // GLSL uniform locations (valid only after init())
    GLint ua_time = -1, ua_phaseSin = -1, ua_phaseCos = -1, ua_frameIdx = -1;
    GLint ua_cSteps = -1, ua_lSteps = -1;
    GLint ua_covLo = -1, ua_covHi = -1, ua_threshLo = -1, ua_threshHi = -1;
    GLint ua_erosion = -1, ua_density = -1, ua_extinction = -1;
    GLint ua_minAlt = -1, ua_maxAlt = -1, ua_debug = -1;
    GLint ua_noiseT = -1, ua_coverT = -1, ua_detailT = -1, ua_weatherT = -1;

    // Loaded GLSL source (cloud.glsl contents)
    std::string m_glsl;

    // CPU bake functions — each builds a texture from scratch at startup.
    void bakeNoise();    // Perlin-Worley + tileable Worley
    void bakeCover();    // curl noise FBMs for macro coverage
    void bakeDetail();   // multi-frequency Worley for surface erosion
    void bakeWeather();  // equirectangular climate/land map
};
