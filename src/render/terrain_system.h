#pragma once
#include "math/math3d.h"
#include "tectonic_sim.h"
#include "climate_sim.h"
#include "hydro_sim.h"
#include <vector>
#include <memory>
#include <cmath>

namespace Terrain {

// --- High-Precision Noise matching GLSL Shader ---
struct Noise {
    static float fract(float x) { return x - floorf(x); }
    static Vec3 fract(const Vec3& v) { return Vec3(fract(v.x), fract(v.y), fract(v.z)); }
    static Vec3 floorVec(const Vec3& v) { return Vec3(floorf(v.x), floorf(v.y), floorf(v.z)); }
    
    static float hash(Vec3 p) {
        p = fract(Vec3(p.x * 443.897f, p.y * 441.423f, p.z * 437.195f));
        p.x += p.x * (p.y + 19.19f); p.y += p.y * (p.z + 19.19f); p.z += p.z * (p.x + 19.19f);
        return fract((p.x + p.y) * p.z);
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

    static float warpedFbm(Vec3 p) {
        Vec3 q(fbm(p, 4), fbm(p + Vec3(5.2f, 1.3f, 2.8f), 4), fbm(p + Vec3(9.1f, 4.7f, 3.1f), 4));
        return fbm(p + q * 1.8f, 10);
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
        
        // 4. Subdivision logic: based on distance and visual size
        // Increased distance multiplier (2.5 -> 3.0) for better continuity
        // Relaxed horizon culling (nodeKm * 2.0f) to prevent gaps at large scales
        bool shouldSubdivide = (dist < nodeKm * 3.0f) && (node->level < 20) && (dist < horizonDist + nodeKm * 2.0f);
        
        if (shouldSubdivide) {
            node->subdivide();
            for(int i=0; i<4; i++) updateSubdivision(node->children[i].get(), camPosRel, radius);
        } else {
            node->collapse();
        }
    }

    float smoothstep_local(float edge0, float edge1, float x) {
        float t = std::clamp((x - edge0) / (edge1 - edge0), 0.0f, 1.0f);
        return t * t * (3.0f - 2.0f * t);
    }

    float getHeight(const Vec3& normalizedPos) {
        float phi = std::acos(std::clamp((float)normalizedPos.y, -1.0f, 1.0f));
        float theta = std::atan2((float)normalizedPos.z, (float)normalizedPos.x);
        float u = theta / (2.0f * PI) + 0.5f;
        float v = phi / PI;

        float plateBase = 0.5f;
        if (sim && !sim->gridHeight.empty()) {
            int tx = std::clamp((int)(u * sim->width), 0, sim->width - 1);
            int ty = std::clamp((int)(v * sim->height), 0, sim->height - 1);
            plateBase = sim->gridHeight[ty * sim->width + tx] / 255.0f;
        }

        float noise = Noise::warpedFbm(normalizedPos * 6.5f);
        float landMask = smoothstep_local(0.4f, 0.6f, plateBase);
        float hStr = plateBase + (noise - 0.5f) * (0.12f + 0.06f * landMask);

        float filledH = 0.0f;
        if (hydroSim && hydroSim->data.filledHeight.size() > 0) {
            int tx = std::clamp((int)(u * hydroSim->width), 0, hydroSim->width - 1);
            int ty = std::clamp((int)(v * hydroSim->height), 0, hydroSim->height - 1);
            filledH = hydroSim->data.filledHeight[ty * hydroSim->width + tx] / 255.0f;
        }

        float lakeMask = smoothstep_local(0.0001f, 0.01f, filledH - hStr);
        hStr = hStr + (filledH - hStr) * lakeMask * 0.98f;

        float seaLevel = 0.44f;
        if (hStr < seaLevel && hStr > 0.38f) {
            float shelfT = (hStr - 0.38f) / (seaLevel - 0.38f);
            hStr = 0.39f + (seaLevel - 0.002f - 0.39f) * smoothstep_local(0.0f, 1.0f, shelfT);
        }

        float height;
        if (hStr < seaLevel) height = 0.0f; // Ocean surface is exactly @ Sea Level (radius)
        else height = (hStr - seaLevel) / (1.0f - seaLevel) * maxElevation;

        // --- KSC FLATTENING ---
        Vec3 kscPos(0.1436f, 0.478f, 0.866f);
        float kscDist = (normalizedPos - kscPos).length();
        float kscMask = smoothstep_local(0.08f, 0.02f, kscDist); 
        // Flatten KSC to exactly 5 meters above sea level (0.005 km)
        height = Noise::mix(height, 0.005f, kscMask); 

        return height;
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
