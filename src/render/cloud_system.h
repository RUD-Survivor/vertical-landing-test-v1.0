#pragma once
#include <string>
#include <vector>
#include <cstdio>
#include <fstream>
#include <sstream>

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
    float dbgFloat   = 0.f;    // float mirror for slider UI
};

class CloudSystem {
public:
    CloudTuneParams tuneParams;

    // Load cloud.glsl from disk.
    bool loadGLSL(const char* path) {
        std::ifstream f(path);
        if (!f.is_open()) { fprintf(stderr, "[cloud] Cannot open: %s\n", path); return false; }
        std::stringstream ss; ss << f.rdbuf(); m_glsl = ss.str(); return true;
    }

    // Returns the loaded GLSL source.
    const char* glslSource() const { return m_glsl.c_str(); }

    // Grab all uniform locations from prog and bake the four noise textures.
    void init(unsigned int prog) { (void)prog; }

    // Delete all owned GPU textures (call on renderer teardown).
    void destroy() {}

    // Upload per-frame cloud uniforms and bind textures.
    void bind(unsigned int prog, double time, int frameIdx, float camAltKm) {
        (void)prog; (void)time; (void)frameIdx; (void)camAltKm;
    }

    // Re-bake weather map from procedural terrain height data.
    // Replaces the default ellipse-based land mask with the actual tectonic
    // height map, and adds planetary-wave + coastal-proximity modulation.
    //   heightMap – tectonic gridHeight (values in [0,1], seaLevel threshold for land)
    //   w, h      – dimensions of heightMap
    //   seaLevel  – threshold above which = land (default 0.45 matches Tectonic::CONT_THRESHOLD)
    void rebakeWeather(const std::vector<float>& heightMap, int w, int h, float seaLevel = 0.45f);

private:
    // GPU texture handles (Vulkan: managed by VkTexture)
    unsigned int noiseT   = 0;
    unsigned int coverT   = 0;
    unsigned int detailT  = 0;
    unsigned int weatherT = 0;

    // GLSL uniform locations (Vulkan: managed by descriptor sets)
    int ua_time = -1, ua_phaseSin = -1, ua_phaseCos = -1, ua_frameIdx = -1;
    int ua_cSteps = -1, ua_lSteps = -1;
    int ua_covLo = -1, ua_covHi = -1, ua_threshLo = -1, ua_threshHi = -1;
    int ua_erosion = -1, ua_density = -1, ua_extinction = -1;
    int ua_minAlt = -1, ua_maxAlt = -1, ua_debug = -1;
    int ua_noiseT = -1, ua_coverT = -1, ua_detailT = -1, ua_weatherT = -1;

    // Loaded GLSL source (cloud.glsl contents)
    std::string m_glsl;

    // CPU bake functions — each builds a texture from scratch at startup.
    void bakeNoise();
    void bakeCover();
    void bakeDetail();
    void bakeWeather();
};
