#pragma once
#include "math/math3d.h"
#include "tectonic_sim.h"
#include "climate_sim.h"
#include "hydro_sim.h"
#include <vector>
#include <memory>
#include <cmath>

namespace Terrain {

// --- Simple Noise for CPU-side Height queries ---
struct Noise {
    static float hash(float n) { return fract(sinf(n) * 43758.5453123f); }
    static float fract(float x) { return x - floorf(x); }
    
    static float noise(const Vec3& x) {
        Vec3 p(floorf(x.x), floorf(x.y), floorf(x.z));
        Vec3 f(fract(x.x), fract(x.y), fract(x.z));
        f.x = f.x * f.x * (3.0f - 2.0f * f.x);
        f.y = f.y * f.y * (3.0f - 2.0f * f.y);
        f.z = f.z * f.z * (3.0f - 2.0f * f.z);
        float n = p.x + p.y * 57.0f + 113.0f * p.z;
        return mix(mix(mix(hash(n + 0.0f), hash(n + 1.0f), f.x),
                       mix(hash(n + 57.0f), hash(n + 58.0f), f.x), f.y),
                   mix(mix(hash(n + 113.0f), hash(n + 114.0f), f.x),
                       mix(hash(n + 170.0f), hash(n + 171.0f), f.x), f.y), f.z);
    }
    
    static float mix(float a, float b, float t) { return a + (b - a) * t; }

    static float fbm(Vec3 p, int octaves) {
        float v = 0.0f, amp = 0.5f;
        for (int i = 0; i < octaves; i++) {
            v += noise(p) * amp;
            p = p * 2.07f + Vec3(0.131f, -0.217f, 0.344f);
            amp *= 0.48f;
        }
        return v;
    }

    static float warpedFbm(Vec3 p) {
        Vec3 q(fbm(p, 4), fbm(p + Vec3(5.2f, 1.3f, 2.8f), 4), fbm(p + Vec3(9.1f, 4.7f, 3.1f), 4));
        return fbm(p + q * 1.6f, 8);
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
    
    TerrainNode(Vec3 c, Vec3 sa, Vec3 sb, float s, int l) 
        : center(c), sideA(sa), sideB(sb), size(s), level(l) {}

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
        climateSim->bake(); // Re-bake with river data in Alpha channel

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
        
        // 4. Subdivision logic: based on distance and visual size
        // Increased distance multiplier (2.5 -> 3.0) for better continuity
        // Relaxed horizon culling (nodeKm * 2.0f) to prevent gaps at large scales
        bool shouldSubdivide = (dist < nodeKm * 3.0f) && (node->level < 16) && (dist < horizonDist + nodeKm * 2.0f);
        
        if (shouldSubdivide) {
            node->subdivide();
            for(int i=0; i<4; i++) updateSubdivision(node->children[i].get(), camPosRel, radius);
        } else {
            node->collapse();
        }
    }

    float getHeight(const Vec3& normalizedPos) {
        float hStr = Noise::warpedFbm(normalizedPos * 4.8f);
        float seaLevel = 0.44f;
        if (hStr < seaLevel) return 0.0f; 
        return (hStr - seaLevel) / (1.0f - seaLevel) * maxElevation;
    }

    Vec3 getPosition(const Vec3& normalizedPos) {
        float h = getHeight(normalizedPos);
        return normalizedPos * (planetRadius + h);
    }

    GLuint getClimateTexture() {
        return climateSim ? climateSim->data.textureID : 0;
    }
};

} // namespace Terrain
