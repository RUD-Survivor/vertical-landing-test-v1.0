#pragma once
#include "math/math3d.h"
#include "tectonic_sim.h"
#include "climate_sim.h"
#include "hydro_sim.h"
#include <vector>
#include <memory>
#include <cmath>
#include <algorithm>

namespace Terrain {

// --- High-Precision Noise matching GLSL Shader ---
struct Noise {
    static float fract(float x) { return x - floorf(x); }
    static Vec3 fract(const Vec3& v) { return Vec3(fract(v.x), fract(v.y), fract(v.z)); }
    static Vec3 floorVec(const Vec3& v) { return Vec3(floorf(v.x), floorf(v.y), floorf(v.z)); }
    
    static float hash(Vec3 p) {
        p.x = fract(p.x * 443.897f);
        p.y = fract(p.y * 441.423f);
        p.z = fract(p.z * 437.195f);
        float d = (float)(p.x * (p.y + 19.19) + p.y * (p.z + 19.19) + p.z * (p.x + 19.19));
        p.x += d; p.y += d; p.z += d;
        return fract((float)((p.x + p.y) * p.z));
    }
    
    static float noise(const Vec3& p) {
        Vec3 i = floorVec(p);
        Vec3 f = fract(p);
        // f = f * f * f * (f * (f * 6.0 - 15.0) + 10.0)
        Vec3 u(
            f.x*f.x*f.x*(f.x*(f.x*6.0f - 15.0f) + 10.0f),
            f.y*f.y*f.y*(f.y*(f.y*6.0f - 15.0f) + 10.0f),
            f.z*f.z*f.z*(f.z*(f.z*6.0f - 15.0f) + 10.0f)
        );
        
        float n000 = hash(i);
        float n100 = hash(i + Vec3(1,0,0));
        float n010 = hash(i + Vec3(0,1,0));
        float n110 = hash(i + Vec3(1,1,0));
        float n001 = hash(i + Vec3(0,0,1));
        float n101 = hash(i + Vec3(1,0,1));
        float n011 = hash(i + Vec3(0,1,1));
        float n111 = hash(i + Vec3(1,1,1));
        
        return mix(mix(mix(n000, n100, u.x), mix(n010, n110, u.x), u.y),
                   mix(mix(n001, n101, u.x), mix(n011, n111, u.x), u.y), u.z);
    }
    
    static float mix(float a, float b, float t) { return a + (b - a) * t; }

    static float fbm(Vec3 p, int octaves) {
        float v = 0.0f, amp = 0.5f;
        for (int i = 0; i < octaves; i++) {
            v += noise(p) * amp;
            p = p * 2.15f + Vec3(0.131f, -0.217f, 0.344f);
            amp *= 0.47f;
        }
        return v;
    }

    static float ridgedFbm(const Vec3& p, int octaves) {
        float v = 0.0f, amp = 0.5f, freq = 1.05f;
        for (int i = 0; i < octaves; i++) {
            float n = 1.0f - fabsf(noise(p * freq) * 2.0f - 1.0f);
            v += n * n * amp;
            freq *= 2.07f;
            amp *= 0.48f;
        }
        return v;
    }

    static float mountainNoise(Vec3 p) {
        // Domain Warp: offset sampling pos with another FBM
        Vec3 offset(
            fbm(p * 2.0f + Vec3(0.0f, 0.0f, 0.0f), 3),
            fbm(p * 2.0f + Vec3(5.2f, 1.3f, 0.7f), 3),
            fbm(p * 2.0f + Vec3(2.7f, 8.4f, 3.1f), 3)
        );
        return ridgedFbm(p + offset * 0.15f, 6);
    }
    static Vec3 hash33(Vec3 p) {
        p.x = fract(p.x * 443.897f);
        p.y = fract(p.y * 441.423f);
        p.z = fract(p.z * 437.195f);
        float d = (float)(p.x * (p.y + 19.19) + p.y * (p.z + 19.19) + p.z * (p.x + 19.19));
        p.x += d; p.y += d; p.z += d;
        return fract(p);
    }

    static float worley(Vec3 p) {
        Vec3 i = floorVec(p);
        Vec3 f = fract(p);
        float minDist = 1.0f;
        for (int z = -1; z <= 1; z++) {
            for (int y = -1; y <= 1; y++) {
                for (int x = -1; x <= 1; x++) {
                    Vec3 neighbor((float)x, (float)y, (float)z);
                    Vec3 point = hash33(i + neighbor);
                    Vec3 diff = neighbor + point - f;
                    minDist = std::min(minDist, (float)diff.length());
                }
            }
        }
        return minDist;
    }

    static float gullyNoise(Vec3 p, float angle) {
        float ca = cosf(angle), sa = sinf(angle);
        float sU = (float)p.x * ca - (float)p.z * sa;
        float sV = (float)p.x * sa + (float)p.z * ca;
        Vec3 pS(sU * 1500000.0f, (float)p.y * 1500000.0f, sV * 40000.0f);
        return ridgedFbm(pS, 3);
    }

    static float detail10m(const Vec3& p) {
        float n = noise(p * 2500000.0f);
        n += noise(p * 5000000.0f) * 0.5f;
        return n * 0.0000015f; 
    }
};

// --- Terrain Node for Quadtree ---
struct TerrainNode {
    Vec3 center;  // Center on the unit cube face
    Vec3 sideA;   // Horizontal vector spanning this node on cube face
    Vec3 sideB;   // Vertical vector spanning this node on cube face
    float size;   // Current logical size (1.0 for root)
    int level;
    bool isLeaf = true;
    std::unique_ptr<TerrainNode> children[4];
    
    GLuint localHydroTex = 0;
    bool hydroGenerated = false;

    TerrainNode(Vec3 c, Vec3 sa, Vec3 sb, float s, int l) 
        : center(c), sideA(sa), sideB(sb), size(s), level(l) {}

    ~TerrainNode() {
        if (localHydroTex) glDeleteTextures(1, &localHydroTex);
    }

    void subdivide() {
        if (!isLeaf) return;
        float s = size * 0.5f;
        // Correct relative scaling: children are always 50% width/height of parent
        Vec3 sa = sideA * 0.5f;
        Vec3 sb = sideB * 0.5f;
        // Quads: 0=top-left, 1=top-right, 2=bottom-left, 3=bottom-right
        children[0] = std::make_unique<TerrainNode>(center - sa * 0.5f + sb * 0.5f, sa, sb, s, level + 1);
        children[1] = std::make_unique<TerrainNode>(center + sa * 0.5f + sb * 0.5f, sa, sb, s, level + 1);
        children[2] = std::make_unique<TerrainNode>(center - sa * 0.5f - sb * 0.5f, sa, sb, s, level + 1);
        children[3] = std::make_unique<TerrainNode>(center + sa * 0.5f - sb * 0.5f, sa, sb, s, level + 1);
        isLeaf = false;
    }

    void collapse() {
        if (isLeaf) return;
        for(int i=0; i<4; i++) children[i].reset();
        isLeaf = true;
    }
};

class QuadtreeTerrain {
public:
    float planetRadius;
    float maxElevation = 25.0f; // 25km max height (Kilometers)
    Tectonic::TectonicSimulator* sim = nullptr;
    Climate::ClimateSimulator* climateSim = nullptr;
    Hydro::HydroSimulator* hydroSim = nullptr;
    std::unique_ptr<TerrainNode> roots[6];
    
    QuadtreeTerrain(float radius) : planetRadius(radius) {
        // Reduced simulation resolution (512x256) for massive performance boost
        sim = new Tectonic::TectonicSimulator(512, 256);
        sim->simulate(80); // 80 generations of tectonic evolution

        // Run Climate Simulation on finalized terrain
        climateSim = new Climate::ClimateSimulator(512, 256);
        climateSim->simulate(sim->gridHeight);

        // Run Hydrology Simulation (Rivers/Lakes)
        hydroSim = new Hydro::HydroSimulator(512, 256);
        hydroSim->simulate(sim->gridHeight, climateSim->data.precipitation, climateSim->data.temperature);
        
        // Final Sync: Inject hydrology data back into climate map for GPU visualization
        // Pack Strahler Order (Integer) and Log-Accumulation (Fraction) into ALPHA channel
        for (int i = 0; i < 512 * 256; i++) {
            float s = (float)hydroSim->data.strahler[i];
            float acc = hydroSim->data.accumulation[i];
            float logAcc = acc > 0.0f ? std::min(0.95f, log10f(1.0f + acc) / 6.0f) : 0.0f;
            climateSim->data.moisture[i] = s + logAcc;
        }
        climateSim->bake(); 
        hydroSim->bake(); // Dedicated hydro texture for vertex-side height correction

        // Initialize 6 faces of the cube
        // +X, -X, +Y, -Y, +Z, -Z
        roots[0] = std::make_unique<TerrainNode>(Vec3( 1, 0, 0), Vec3( 0, 0,-2), Vec3( 0, 2, 0), 1.0f, 0);
        roots[1] = std::make_unique<TerrainNode>(Vec3(-1, 0, 0), Vec3( 0, 0, 2), Vec3( 0, 2, 0), 1.0f, 0);
        roots[2] = std::make_unique<TerrainNode>(Vec3( 0, 1, 0), Vec3( 2, 0, 0), Vec3( 0, 0,-2), 1.0f, 0);
        roots[3] = std::make_unique<TerrainNode>(Vec3( 0,-1, 0), Vec3( 2, 0, 0), Vec3( 0, 0, 2), 1.0f, 0);
        roots[4] = std::make_unique<TerrainNode>(Vec3( 0, 0, 1), Vec3( 2, 0, 0), Vec3( 0, 2, 0), 1.0f, 0);
        roots[5] = std::make_unique<TerrainNode>(Vec3( 0, 0,-1), Vec3(-2, 0, 0), Vec3( 0, 2, 0), 1.0f, 0);
    }

    // camPosRel: Camera position relative to the planet center (Kilometers)
    // radius: Planet radius (Kilometers)
    void updateSubdivision(TerrainNode* node, const Vec3& camPosRel, float radius) {
        // 1. Calculate distance from camera to the closest point on the node's face
        // Simple approximation: distance to projected center
        Vec3 spherePos = node->center.normalized() * radius;
        float dist = (spherePos - camPosRel).length();
        
        // 2. Horizon culling for LOD (Kilometers)
        float camAlt = camPosRel.length();
        // Add maxElevation safety to horizon distance to prevent clipping high mountains
        float horizonDist = sqrtf(fmaxf(0.0f, camAlt * camAlt - radius * radius)) + maxElevation;
        
        // 3. Node visual size (Kilometers)
        float nodeKm = node->size * radius * 2.0f;
        bool shouldSubdivide = (dist < nodeKm * 3.0f) && (node->level < 20) && (dist < horizonDist + nodeKm * 2.0f);
        
        if (shouldSubdivide) {
            node->subdivide();
            for(int i=0; i<4; i++) updateSubdivision(node->children[i].get(), camPosRel, radius);
        } else {
            node->collapse();
            if (node->level >= 5) generateLocalHydro(node);
        }
    }

    void generateLocalHydro(TerrainNode* node) {
        if (node->hydroGenerated || !hydroSim) return;
        
        int res = 64;
        std::vector<float> grid(res * res);
        std::vector<float> boundaries(res * 4);

        // 1. Generate local height field
        for (int y = 0; y < res; y++) {
            for (int x = 0; x < res; x++) {
                float u = (float)x / (res - 1);
                float v = (float)y / (res - 1);
                Vec3 p = node->center + node->sideA * (u - 0.5f) + node->sideB * (v - 0.5f);
                // Sample raw tectonic + regional noise (matches shader's plateBase scale)
                // We use sLevel=0.45 as the baseline.
                grid[y * res + x] = getHeightRaw(p.normalized());
            }
        }

        // 2. Sample global filledHeights for boundaries
        auto getGlobalFilled = [&](const Vec3& p) {
            float phi = std::acos(std::clamp((float)p.y, -1.0f, 1.0f));
            float theta = std::atan2((float)p.z, (float)p.x);
            float u = theta / (2.0f * 3.14159f) + 0.5f;
            float v = phi / 3.14159f;
            int tx = std::clamp((int)(u * (hydroSim->width - 1)), 0, (int)hydroSim->width - 1);
            int ty = std::clamp((int)(v * (hydroSim->height - 1)), 0, (int)hydroSim->height - 1);
            return hydroSim->data.filledHeight[ty * (int)hydroSim->width + tx];
        };

        for (int i = 0; i < res; i++) {
            float t = (float)i / (res - 1);
            // Top
            boundaries[i] = getGlobalFilled((node->center - node->sideA*0.5f + node->sideB*0.5f + node->sideA*t).normalized());
            // Bottom
            boundaries[res + i] = getGlobalFilled((node->center - node->sideA*0.5f - node->sideB*0.5f + node->sideA*t).normalized());
            // Left
            boundaries[2*res + i] = getGlobalFilled((node->center - node->sideA*0.5f - node->sideB*0.5f + node->sideB*t).normalized());
            // Right
            boundaries[3*res + i] = getGlobalFilled((node->center + node->sideA*0.5f - node->sideB*0.5f + node->sideB*t).normalized());
        }

        // 3. Run Local Flood
        Hydro::HydroSimulator::fillDepressionsLocal(res, grid, boundaries);

        // 4. Upload to Texture
        if (node->localHydroTex == 0) glGenTextures(1, &node->localHydroTex);
        glBindTexture(GL_TEXTURE_2D, node->localHydroTex);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_R32F, res, res, 0, GL_RED, GL_FLOAT, grid.data());
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        
        node->hydroGenerated = true;
    }

    float getHeightRaw(const Vec3& normalizedPos) {
        float phi = std::acos(std::clamp((float)normalizedPos.y, -1.0f, 1.0f));
        float theta = std::atan2((float)normalizedPos.z, (float)normalizedPos.x);
        float u = theta / (2.0f * 3.14159f) + 0.5f;
        float v = phi / 3.14159f;

        float plateBase = sampleTectonic(u, v);
        float hRefined = plateBase;
        float mountainMask = smoothstep_local(0.55f, 0.75f, plateBase);
        float coastalMask = smoothstep_local(0.435f, 0.445f, plateBase) * (1.0f - smoothstep_local(0.455f, 0.465f, plateBase));

        if (mountainMask > 0.01f) {
            float epsG = 0.005f;
            float hX = (sampleTectonic(u + epsG, v) - sampleTectonic(u - epsG, v));
            float hY = (sampleTectonic(u, v + epsG) - sampleTectonic(u, v - epsG));
            float sAngle = std::atan2(-hX, hY);
            float ca = std::cos(sAngle), sa = std::sin(sAngle);
            float strikeU = (float)normalizedPos.x * ca - (float)normalizedPos.z * sa;
            float strikeV = (float)normalizedPos.x * sa + (float)normalizedPos.z * ca;
            Vec3 pS(strikeU * 0.72f, (float)normalizedPos.y, strikeV * 1.28f);
            hRefined += Noise::mountainNoise(pS * 6.5f) * 0.18f * mountainMask;
        }
        if (coastalMask > 0.1f) {
            float cliffNoise = 1.0f - fabsf(Noise::noise(normalizedPos * 180.0f) * 1.5f - 0.5f);
            hRefined += smoothstep_local(0.6f, 0.9f, cliffNoise) * 0.007f * coastalMask;
        }
        return hRefined;
    }

    float smoothstep_local(float edge0, float edge1, float x) {
        float t = std::clamp((x - edge0) / (edge1 - edge0), 0.0f, 1.0f);
        return t * t * (3.0f - 2.0f * t);
    }

    // Feature-Aware Sharply Interpolated Sampling
    float sampleTectonic(float u, float v) {
        if (!sim || sim->gridHeight.empty()) return 0.5f;
        float fx = u * (sim->width - 1);
        float fy = v * (sim->height - 1);
        int x0 = (int)fx, y0 = (int)fy;
        int x1 = (x0 + 1) % sim->width, y1 = std::min(sim->height - 1, y0 + 1);
        float tx = fx - x0, ty = fy - y0;

        float h00 = sim->gridHeight[y0 * sim->width + x0] / 255.0f;
        float h10 = sim->gridHeight[y0 * sim->width + x1] / 255.0f;
        float h01 = sim->gridHeight[y1 * sim->width + x0] / 255.0f;
        float h11 = sim->gridHeight[y1 * sim->width + x1] / 255.0f;

        // Bilinear base
        float base = h00*(1-tx)*(1-ty) + h10*tx*(1-ty) + h01*(1-tx)*ty + h11*tx*ty;
        
        // Feature detection: Sharpen ridges/valleys if gradient is high
        float grad = sqrtf((h10-h00)*(h10-h00) + (h01-h00)*(h01-h00));
        if (grad > 0.05f) {
            float fx_sharp = tx * tx * (3.0f - 2.0f * tx);
            float fy_sharp = ty * ty * (3.0f - 2.0f * ty);
            float sharp = (h00*(1-fx_sharp)*(1-fy_sharp) + h10*fx_sharp*(1-fy_sharp) + 
                           h01*(1-fx_sharp)*fy_sharp + h11*fx_sharp*fy_sharp);
            base = Noise::mix(base, sharp, 0.45f);
        }
        return base;
    }

    float getHeight(const Vec3& normalizedPos) {
        float phi = std::acos(std::clamp((float)normalizedPos.y, -1.0f, 1.0f));
        float theta = std::atan2((float)normalizedPos.z, (float)normalizedPos.x);
        float u = theta / (2.0f * PI) + 0.5f;
        float v = phi / PI;

        float plateBase = sampleTectonic(u, v);
        float hRefined = plateBase;
        
        // --- PRE-CALCULATE BIOMES ---
        float mountainMask = smoothstep_local(0.55f, 0.75f, plateBase);
        float plainsMask = 1.0f - smoothstep_local(0.40f, 0.62f, plateBase);
        float coastalMask = smoothstep_local(0.435f, 0.445f, plateBase) * (1.0f - smoothstep_local(0.455f, 0.465f, plateBase));
        
        // Climate lookup (for dunes/ice)
        float temp = 0.5f, precip = 0.5f;
        if (climateSim) {
            int tx = std::clamp((int)(u * (climateSim->width - 1)), 0, climateSim->width - 1);
            int ty = std::clamp((int)(v * (climateSim->height - 1)), 0, climateSim->height - 1);
            temp = (climateSim->data.temperature[ty * climateSim->width + tx] + 30.0f) / 70.0f;
            precip = climateSim->data.precipitation[ty * climateSim->width + tx] / 2000.0f;
        }

        // --- LAYER 1: REGIONAL GEOLOGICAL SCULPTING ---

        // 1.1 Mountains: Anisotropic Folding + Thermal Erosion
        if (mountainMask > 0.01f) {
            float epsG = 0.005f;
            float hX = (sampleTectonic(u + epsG, v) - sampleTectonic(u - epsG, v));
            float hY = (sampleTectonic(u, v + epsG) - sampleTectonic(u, v - epsG));
            float sAngle = std::atan2(-hX, hY);
            float ca = std::cos(sAngle), sa = std::sin(sAngle);
            
            float strikeU = normalizedPos.x * ca - normalizedPos.z * sa;
            float strikeV = normalizedPos.x * sa + normalizedPos.z * ca;
            Vec3 pS(strikeU * 0.72f, normalizedPos.y, strikeV * 1.28f);
            
            float ridges = Noise::mountainNoise(pS * 6.5f);
            hRefined += ridges * 0.18f * mountainMask;
            
            // Thermal Erosion: smooth sharp peaks
            if (hRefined > 0.78f) {
                float peakD = smoothstep_local(0.78f, 0.96f, hRefined);
                hRefined -= peakD * 0.045f;
            }
        }

        // 1.2 Biome-Specific Layer (Dunes / Glaciers)
        if (temp > 0.75f && precip < 0.15f && plainsMask > 0.2f) {
            // Desert Dunes: Stretched along "Wind" (approx longitude)
            float dune = 1.0f - fabsf(Noise::noise(Vec3(normalizedPos.x * 2.5f, normalizedPos.y, normalizedPos.z * 0.3f) * 150.0f) * 2.0f - 1.0f);
            hRefined += dune * 0.008f * plainsMask;
        }
        
        // 1.3 Coastal Cliffs
        if (coastalMask > 0.1f) {
            float cliffNoise = 1.0f - fabsf(Noise::noise(normalizedPos * 180.0f) * 1.5f - 0.5f);
            hRefined += smoothstep_local(0.6f, 0.9f, cliffNoise) * 0.007f * coastalMask;
        }

        // --- LAYER 2: LOCAL GEOLOGICAL SCULPTING (Physics-Ready) ---

        // 2.1 River Valley Carving (V-to-U shaped)
        if (hydroSim && hydroSim->data.strahler.size() > 0) {
            int tx = std::clamp((int)(u * (hydroSim->width - 1)), 0, (int)hydroSim->width - 1);
            int ty = std::clamp((int)(v * (hydroSim->height - 1)), 0, (int)hydroSim->height - 1);
            float s = (float)hydroSim->data.strahler[ty * hydroSim->width + tx];
            if (s >= 2.0f && plateBase > 0.445f) {
                // Approximate valley using local distance noise
                float valleyWarp = Noise::noise(normalizedPos * 120.0f) * 0.004f;
                float vNoise = Noise::noise(normalizedPos * 250.0f + Vec3(valleyWarp, valleyWarp, valleyWarp));
                float depth = 0.015f * (s / 7.0f); // Depth scale by Strahler
                
                // V-shape (Mountain) to U-shape (Plains) based on height
                float isLowland = 1.0f - smoothstep_local(0.45f, 0.55f, plateBase);
                float vShape = fabsf(vNoise * 2.0f - 1.0f);
                float uShape = powf(vShape, 0.4f);
                float valleyProfile = Noise::mix(vShape, uShape, isLowland);
                
                hRefined -= (1.0f - valleyProfile) * depth;
            }
        }

        // 2.2 Micro-Mounds & Boulders (Local collision)
        float mounds = Noise::noise(normalizedPos * 1200.0f);
        hRefined += (mounds - 0.5f) * 0.0008f;

        // --- Hydrology Integration (Lakes) ---
        if (hydroSim && hydroSim->data.filledHeight.size() > 0) {
            int tx = std::clamp((int)(u * (hydroSim->width - 1)), 0, hydroSim->width - 1);
            int ty = std::clamp((int)(v * (hydroSim->height - 1)), 0, hydroSim->height - 1);
            float filledH = hydroSim->data.filledHeight[ty * (int)hydroSim->width + tx] / 255.0f;
            float hydroMask = smoothstep_local(0.0001f, 0.01f, filledH - plateBase);
            hRefined = Noise::mix(hRefined, filledH, hydroMask * 0.97f);
        }

        // Sea Level Normalization
        float seaLevel = 0.45f;
        if (hRefined < seaLevel && hRefined > 0.38f) {
            float shelfT = (hRefined - 0.38f) / (seaLevel - 0.38f);
            hRefined = 0.395f + (seaLevel - 0.001f - 0.395f) * smoothstep_local(0.0f, 1.0f, shelfT);
        }

        // Translate normalized height [0,1] to kilometers
        float finalH = (hRefined < seaLevel) ? 0.0f : (hRefined - seaLevel) / (1.0f - seaLevel) * maxElevation / planetRadius;
        
        if (hRefined >= seaLevel) {
            float epsS = 0.0015f;
            float hX = (sampleTectonic(u + epsS, v) - sampleTectonic(u - epsS, v));
            float hY = (sampleTectonic(u, v + epsS) - sampleTectonic(u, v - epsS));
            float currentSlope = sqrtf(hX*hX + hY*hY) * 100.0f;
            float sAngle = std::atan2(-hX, hY);

            float sHigh = smoothstep_local(0.12f, 0.25f, currentSlope);
            float sMid = smoothstep_local(0.04f, 0.12f, currentSlope) * (1.0f - sHigh);
            float sLow = 1.0f - smoothstep_local(0.04f, 0.12f, currentSlope);

            float rockCracks = (1.0f - Noise::worley(normalizedPos * 1800000.0f)) * 0.000003f;
            float gullies = Noise::gullyNoise(normalizedPos, sAngle) * 0.000002f;
            float baseNoise = (Noise::noise(normalizedPos * 4000000.0f) - 0.5f) * 0.0000005f;

            finalH += (rockCracks * sHigh + gullies * sMid + baseNoise * sLow);
            finalH += Noise::detail10m(normalizedPos);
        }

        // KSC FLATTENING & LANDIFICATION
        Vec3 kscPos(0.1436, 0.478, 0.866);
        float distToKSC = (float)(normalizedPos - kscPos).length();
        float kscMask = smoothstep_local(0.12f, 0.04f, distToKSC); 
        
        // Ensure hRefined is above seaLevel at KSC for land-based coloring
        hRefined = Noise::mix(hRefined, seaLevel + 0.015f, kscMask);
        
        // Final displacement (5m above sea level)
        finalH = Noise::mix(finalH, 0.005f / planetRadius, smoothstep_local(0.08f, 0.02f, distToKSC));

        return finalH * planetRadius;
    }

    Vec3 getPosition(const Vec3& normalizedPos) {
        float h = getHeight(normalizedPos);
        return normalizedPos * (planetRadius + h);
    }

    GLuint getClimateTexture() {
        return climateSim ? climateSim->data.textureID : 0;
    }

    GLuint getHydroTexture() {
        return hydroSim ? hydroSim->textureID : 0;
    }
};

} // namespace Terrain
