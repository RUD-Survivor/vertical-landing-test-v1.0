#pragma once
#include "math/math3d.h"
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
    Vec3 center;
    float size;
    int level;
    bool isLeaf = true;
    std::unique_ptr<TerrainNode> children[4];
    
    // Geometric data
    unsigned int vao = 0, vbo = 0, ebo = 0;
    int indexCount = 0;

    TerrainNode(Vec3 c, float s, int l) : center(c), size(s), level(l) {}
    ~TerrainNode() {
        // Cleanup OpenGL resources would happen in a renderer call
    }
};

class QuadtreeTerrain {
public:
    float planetRadius;
    float maxElevation = 10000.0f; // 10km max height
    
    QuadtreeTerrain(float radius) : planetRadius(radius) {}

    float getHeight(const Vec3& normalizedPos) {
        // Match Earth shader logic: warpedFbm(sph * 3.5 * 1.2)
        float hStr = Noise::warpedFbm(normalizedPos * 4.2f);
        float seaLevel = 0.44f;
        if (hStr < seaLevel) return 0.0f; // Underwater
        return (hStr - seaLevel) / (1.0f - seaLevel) * maxElevation;
    }

    Vec3 getPosition(const Vec3& normalizedPos) {
        float h = getHeight(normalizedPos);
        return normalizedPos * (planetRadius + h);
    }
};

} // namespace Terrain
