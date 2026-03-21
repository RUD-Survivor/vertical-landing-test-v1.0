#pragma once
// ==========================================================
// svo_system.h — Sparse Voxel Octree System
// Core data structures, heightfield conversion, modification ops
// ==========================================================

#include "math/math3d.h"
#include "terrain_system.h"
#include <vector>
#include <cstdint>
#include <cstring>
#include <cmath>
#include <algorithm>
#include <queue>
#include <functional>

namespace SVO {

// ========== Material Types ==========
enum class Material : uint8_t {
    AIR = 0,
    ROCK = 1,
    SOIL = 2,
    SAND = 3,
    ICE = 4,
    WATER = 5,
    METAL = 6,
    CONCRETE = 7,
    COUNT
};

// Material color table (RGB, for vertex coloring)
inline void getMaterialColor(Material m, float& r, float& g, float& b) {
    switch (m) {
        case Material::ROCK:     r = 0.45f; g = 0.40f; b = 0.35f; break;
        case Material::SOIL:     r = 0.40f; g = 0.28f; b = 0.15f; break;
        case Material::SAND:     r = 0.76f; g = 0.70f; b = 0.50f; break;
        case Material::ICE:      r = 0.85f; g = 0.90f; b = 0.95f; break;
        case Material::WATER:    r = 0.10f; g = 0.30f; b = 0.60f; break;
        case Material::METAL:    r = 0.70f; g = 0.72f; b = 0.75f; break;
        case Material::CONCRETE: r = 0.65f; g = 0.65f; b = 0.63f; break;
        default:                 r = 0.00f; g = 0.00f; b = 0.00f; break;
    }
}

// ========== SVO Node ==========
// Each node is either a leaf (stores material data) or an internal node (stores child pointers).
// Nodes are stored in a flat pool array and referenced by uint32_t index.
struct SVONode {
    uint32_t children[8] = {0, 0, 0, 0, 0, 0, 0, 0}; // child indices in pool (0 = null)
    Material material = Material::AIR;
    uint8_t  density  = 0;       // 0=air, 255=solid; intermediate = SDF boundary
    uint8_t  flags    = 0x01;    // bit0: isLeaf (default true), bit1: isDirty, bit2: isHomogeneous
    uint8_t  _pad     = 0;

    bool isLeaf()        const { return flags & 0x01; }
    void setLeaf(bool v)       { if (v) flags |= 0x01; else flags &= ~0x01; }
    bool isDirty()       const { return flags & 0x02; }
    void setDirty(bool v)      { if (v) flags |= 0x02; else flags &= ~0x02; }
    bool isHomogeneous() const { return flags & 0x04; }
    void setHomogeneous(bool v){ if (v) flags |= 0x04; else flags &= ~0x04; }

    bool isSolid() const { return isLeaf() && material != Material::AIR; }
    bool isAir()   const { return isLeaf() && material == Material::AIR; }
};

// ========== Node Pool ==========
// Pre-allocated flat array of SVONodes with freelist recycling.
// Index 0 is reserved as NULL_NODE.
class NodePool {
public:
    static constexpr uint32_t POOL_SIZE = 16 * 1024 * 1024; // 16M nodes (~576 MB)
    static constexpr uint32_t NULL_NODE = 0;

    std::vector<SVONode> nodes;
    std::vector<uint32_t> freeList;
    uint32_t nextFree = 1; // 0 is reserved as NULL

    NodePool() {
        nodes.resize(POOL_SIZE);
        freeList.reserve(POOL_SIZE / 4);
    }

    uint32_t allocate() {
        if (!freeList.empty()) {
            uint32_t idx = freeList.back();
            freeList.pop_back();
            nodes[idx] = SVONode{};
            return idx;
        }
        if (nextFree >= POOL_SIZE) return NULL_NODE; // Pool exhausted
        return nextFree++;
    }

    void deallocate(uint32_t idx) {
        if (idx == NULL_NODE || idx >= POOL_SIZE) return;
        nodes[idx] = SVONode{};
        freeList.push_back(idx);
    }

    SVONode& get(uint32_t idx) { return nodes[idx]; }
    const SVONode& get(uint32_t idx) const { return nodes[idx]; }

    size_t usedCount() const { return (size_t)(nextFree - 1) - freeList.size(); }
    float usagePercent() const { return (float)usedCount() / POOL_SIZE * 100.0f; }
};

// ========== SVO Chunk ==========
// Each chunk is a cube in SVO-local space that manages one octree root.
struct SVOChunk {
    Vec3 localOrigin;      // Origin in SVO local frame (km)
    double size;           // Edge length (km)
    uint32_t rootNode;     // Root index in NodePool
    int maxDepth;          // Maximum octree depth (8 = ~1m for 256m chunk)
    bool active = false;
    bool meshDirty = true;

    // Mesh data (output of Marching Cubes)
    GLuint vao = 0, vbo = 0, ebo = 0;
    int indexCount = 0;
    int vertexCount = 0;

    void initGL() {
        if (!vao) {
            glGenVertexArrays(1, &vao);
            glGenBuffers(1, &vbo);
            glGenBuffers(1, &ebo);
        }
    }

    void destroyGL() {
        if (vao) glDeleteVertexArrays(1, &vao);
        if (vbo) glDeleteBuffers(1, &vbo);
        if (ebo) glDeleteBuffers(1, &ebo);
        vao = vbo = ebo = 0;
        indexCount = 0;
    }

    // Morton-order child index from position relative to center
    static int childIndex(const Vec3& pos, const Vec3& center) {
        int idx = 0;
        if (pos.x >= center.x) idx |= 1;
        if (pos.y >= center.y) idx |= 2;
        if (pos.z >= center.z) idx |= 4;
        return idx;
    }
};

// ========== SVO Local Frame ==========
// Defines a local East-North-Up tangent frame at a surface point.
// All SVO chunks are positioned in this local frame.
// Units: km (matching the rendering coordinate system)
struct SVORegion {
    Vec3 centerNorm;       // Normalized direction of colony/landing site center
    double centerRadius;   // Surface radius at center (planetRadius + terrainHeight) in km
    Vec3 east, north, up;  // Local tangent frame axes (unit vectors)
    double planetRadius;   // Planet radius in km

    // Initialize the local frame at a given normalized surface position
    void init(const Vec3& surfNorm, double pRadius, double terrainH) {
        centerNorm = surfNorm.normalized();
        planetRadius = pRadius;
        centerRadius = pRadius + terrainH;
        up = centerNorm;

        // Compute east/north tangent vectors
        // Use cross product with a reference vector that's not parallel to 'up'
        Vec3 ref = (std::abs(up.y) < 0.99) ? Vec3(0, 1, 0) : Vec3(1, 0, 0);
        east = up.cross(ref).normalized();
        north = east.cross(up).normalized();
        // Ensure right-handed: east × north = up (approximately)
        // Actually it should be: east = up × ref -> but let's ensure right-handedness
        // east × up = -north, so north = up × east
        north = up.cross(east).normalized();
        // Recalculate east for exact orthogonality
        east = north.cross(up).normalized();
    }

    // Convert SVO local position (km) to planet-local Cartesian (km)
    // SVO local: x=East, y=Up (radial), z=North
    Vec3 localToPlanetLocal(const Vec3& local) const {
        return centerNorm * centerRadius + east * local.x + up * local.y + north * local.z;
    }

    // Get the normalized direction on the unit sphere for a given local (x, z) position
    // This is used for heightfield sampling
    Vec3 localToNormDir(double x, double z) const {
        Vec3 planetPos = centerNorm * planetRadius + east * x + north * z;
        return planetPos.normalized();
    }

    // Get terrain height (km above sea level) at local position (x, z)
    // Returns the y-value in SVO local frame where the terrain surface is
    double getLocalSurfaceY(double x, double z,
                            Terrain::QuadtreeTerrain* heightField) const {
        Vec3 dir = localToNormDir(x, z);
        double terrH = heightField->getHeight(dir); // km above sea level
        // The surface in planet-local is at direction * (planetRadius + terrH)
        // We need to find what y-value in SVO local frame corresponds to this surface
        Vec3 surfacePoint = dir * (planetRadius + terrH);
        Vec3 offset = surfacePoint - centerNorm * centerRadius;
        // Project onto the 'up' axis to get local Y
        return offset.dot(up);
    }
};

// ========== SVO Manager ==========
// Main interface for the SVO system.
class SVOManager {
public:
    NodePool pool;
    std::vector<SVOChunk> chunks;
    SVORegion region;
    bool regionActive = false;

    // Statistics
    int totalNodesBuilt = 0;
    int chunksBuilt = 0;
    int chunksMeshed = 0;

    // Configuration - Tuned down for Phase 1 performance
    double chunkSize = 0.128;      // 128m = 0.128 km
    int maxDepth = 6;              // 128m / 2^6 = 2m resolution
    double activationRadius = 0.5; // 500m radius of active SVO region

    // Height field source (shared with existing terrain system)
    Terrain::QuadtreeTerrain* heightField = nullptr;

    // ===== ACTIVATION =====
    // Call this when the player lands or approaches the surface.
    // surfaceNormPos: normalized position on unit sphere where the SVO should be centered
    // planetRadius: planet radius in km
    void activate(const Vec3& surfaceNormPos, double planetRadius,
                  Terrain::QuadtreeTerrain* hf) {
        if (regionActive) deactivate();

        heightField = hf;
        double terrH = hf->getHeight(surfaceNormPos);
        region.init(surfaceNormPos, planetRadius, terrH);
        regionActive = true;

        // Create grid of chunks covering the activation radius
        // Horizontal grid: from -activationRadius to +activationRadius
        // Vertical: from -chunkSize*2 (underground) to +chunkSize*2 (above)
        int halfGrid = (int)std::ceil(activationRadius / chunkSize);
        int vertLayers = 4; // 2 below surface, 2 above

        totalNodesBuilt = 0;
        chunksBuilt = 0;

        for (int gz = -halfGrid; gz < halfGrid; gz++) {
            for (int gx = -halfGrid; gx < halfGrid; gx++) {
                double cx = (gx + 0.5) * chunkSize;
                double cz = (gz + 0.5) * chunkSize;

                // Check if within activation radius
                double dist2 = cx * cx + cz * cz;
                if (dist2 > activationRadius * activationRadius) continue;

                for (int gy = -2; gy < vertLayers - 2; gy++) {
                    double cy = gy * chunkSize;

                    SVOChunk chunk;
                    chunk.localOrigin = Vec3(cx - chunkSize * 0.5,
                                             cy,
                                             cz - chunkSize * 0.5);
                    chunk.size = chunkSize;
                    chunk.maxDepth = maxDepth;
                    chunk.active = true;
                    chunk.meshDirty = true;

                    // Allocate root node
                    chunk.rootNode = pool.allocate();
                    if (chunk.rootNode == NodePool::NULL_NODE) {
                        printf("[SVO] ERROR: Node pool exhausted during activation!\n");
                        break;
                    }

                    // Build octree from heightfield
                    buildChunkFromHeightField(chunk);
                    chunk.initGL();
                    chunks.push_back(chunk);
                    chunksBuilt++;
                }
            }
        }

        printf("[SVO] Activated: %d chunks, %zu nodes used (%.1f%% pool)\n",
               chunksBuilt, pool.usedCount(), pool.usagePercent());
    }

    // ===== DEACTIVATION =====
    void deactivate() {
        for (auto& chunk : chunks) {
            if (chunk.rootNode != NodePool::NULL_NODE) {
                freeSubtree(chunk.rootNode);
            }
            chunk.destroyGL();
        }
        chunks.clear();
        regionActive = false;
        totalNodesBuilt = 0;
        chunksBuilt = 0;
        chunksMeshed = 0;
    }

    // ===== HEIGHTFIELD → SVO CONVERSION =====
    void buildChunkFromHeightField(SVOChunk& chunk) {
        if (!heightField || chunk.rootNode == NodePool::NULL_NODE) return;
        buildNodeRecursive(chunk, chunk.rootNode,
                          chunk.localOrigin, chunk.size, 0);
    }

    // ===== DESTRUCTION (DIG) =====
    // center: position in SVO local frame (km)
    // radius: dig radius (km)
    void dig(const Vec3& center, double radius) {
        for (auto& chunk : chunks) {
            if (!chunk.active) continue;
            // AABB vs Sphere test
            if (!sphereIntersectsAABB(center, radius,
                                      chunk.localOrigin, chunk.size)) continue;
            digRecursive(chunk, chunk.rootNode, chunk.localOrigin, chunk.size,
                        center, radius, 0);
            chunk.meshDirty = true;
        }
    }

    // ===== CONSTRUCTION (BUILD) =====
    // center: position in SVO local frame (km)
    // radius: build radius (km)
    // mat: material to place
    void build(const Vec3& center, double radius, Material mat) {
        for (auto& chunk : chunks) {
            if (!chunk.active) continue;
            if (!sphereIntersectsAABB(center, radius,
                                      chunk.localOrigin, chunk.size)) continue;
            buildRecursive(chunk, chunk.rootNode, chunk.localOrigin, chunk.size,
                          center, radius, mat, 0);
            chunk.meshDirty = true;
        }
    }

    // ===== QUERIES =====
    bool hasActiveChunks() const { return regionActive && !chunks.empty(); }

    // Check if a world position (planet-local km) falls within the SVO region
    bool isInSVORegion(const Vec3& planetLocalPos) const {
        if (!regionActive) return false;
        Vec3 offset = planetLocalPos - region.centerNorm * region.centerRadius;
        double localX = offset.dot(region.east);
        double localZ = offset.dot(region.north);
        double dist2 = localX * localX + localZ * localZ;
        return dist2 < activationRadius * activationRadius;
    }

    // Convert planet-local position (km) to SVO local frame
    Vec3 planetLocalToSVOLocal(const Vec3& planetLocalPos) const {
        Vec3 offset = planetLocalPos - region.centerNorm * region.centerRadius;
        return Vec3(offset.dot(region.east),
                   offset.dot(region.up),
                   offset.dot(region.north));
    }

private:
    // ===== RECURSIVE BUILD =====
    void buildNodeRecursive(SVOChunk& chunk, uint32_t nodeIdx,
                           Vec3 nodeOrigin, double nodeSize, int depth) {
        if (nodeIdx == NodePool::NULL_NODE) return;

        // Sample SDF at corners and center to determine node content
        // SDF: negative = underground (solid), positive = above ground (air)
        double minSDF = 1e18, maxSDF = -1e18;
        double centerSDF = 0.0;

        Vec3 nodeCenter = nodeOrigin + Vec3(nodeSize * 0.5, nodeSize * 0.5, nodeSize * 0.5);

        // Sample 9 points: 8 corners + center
        for (int i = 0; i < 9; i++) {
            Vec3 samplePos;
            if (i < 8) {
                samplePos = nodeOrigin + Vec3(
                    (i & 1) ? nodeSize : 0,
                    (i & 2) ? nodeSize : 0,
                    (i & 4) ? nodeSize : 0
                );
            } else {
                samplePos = nodeCenter;
            }

            // Get terrain surface Y in local frame at this (x, z)
            double surfY = region.getLocalSurfaceY(
                samplePos.x, samplePos.z, heightField);

            // SDF: distance from sample point to surface (positive = air)
            double sdf = samplePos.y - surfY;

            minSDF = std::min(minSDF, sdf);
            maxSDF = std::max(maxSDF, sdf);
            if (i == 8) centerSDF = sdf;
        }

        SVONode& node = pool.get(nodeIdx);
        totalNodesBuilt++;

        // Tolerance for homogeneity (proportional to node size)
        double tolerance = nodeSize * 0.05;

        // Case 1: Entirely underground (all SDF < -tolerance)
        if (maxSDF < -tolerance) {
            node.setLeaf(true);
            node.setHomogeneous(true);
            node.material = Material::ROCK;
            node.density = 255;
            return;
        }

        // Case 2: Entirely above ground (all SDF > tolerance)
        if (minSDF > tolerance) {
            node.setLeaf(true);
            node.setHomogeneous(true);
            node.material = Material::AIR;
            node.density = 0;
            return;
        }

        // Case 3: Contains surface boundary
        if (depth >= chunk.maxDepth) {
            // At max depth: store as leaf with SDF-derived density
            node.setLeaf(true);
            node.setHomogeneous(false);
            node.density = (uint8_t)std::clamp(
                128.0 - centerSDF * 128.0 / nodeSize, 0.0, 255.0);
            node.material = (centerSDF < 0) ? Material::ROCK : Material::AIR;
            return;
        }

        // Subdivide: allocate 8 children
        node.setLeaf(false);
        double halfSize = nodeSize * 0.5;
        for (int i = 0; i < 8; i++) {
            Vec3 childOrigin = nodeOrigin + Vec3(
                (i & 1) ? halfSize : 0,
                (i & 2) ? halfSize : 0,
                (i & 4) ? halfSize : 0
            );
            node.children[i] = pool.allocate();
            if (node.children[i] != NodePool::NULL_NODE) {
                buildNodeRecursive(chunk, node.children[i],
                                  childOrigin, halfSize, depth + 1);
            }
        }
    }

    // ===== RECURSIVE DIG =====
    void digRecursive(SVOChunk& chunk, uint32_t nodeIdx,
                     Vec3 nodeOrigin, double nodeSize,
                     const Vec3& center, double radius, int depth) {
        if (nodeIdx == NodePool::NULL_NODE) return;

        SVONode& node = pool.get(nodeIdx);
        Vec3 nodeCenter = nodeOrigin + Vec3(nodeSize * 0.5, nodeSize * 0.5, nodeSize * 0.5);

        // Node entirely inside sphere → clear to air
        if (sphereContainsAABB(center, radius, nodeOrigin, nodeSize)) {
            if (!node.isLeaf()) freeChildren(nodeIdx);
            node.setLeaf(true);
            node.material = Material::AIR;
            node.density = 0;
            node.setHomogeneous(true);
            return;
        }

        // Node entirely outside sphere → no change
        if (!sphereIntersectsAABB(center, radius, nodeOrigin, nodeSize)) return;

        // Already air → nothing to dig
        if (node.isLeaf() && node.material == Material::AIR) return;

        // Partial overlap: need to subdivide if leaf
        if (node.isLeaf()) {
            if (depth < chunk.maxDepth) {
                // Expand this homogeneous leaf into 8 children with same material
                expandLeaf(nodeIdx, node.material, node.density);
            } else {
                // At max depth: check center
                double dist = (nodeCenter - center).length();
                if (dist < radius) {
                    node.material = Material::AIR;
                    node.density = 0;
                }
                return;
            }
        }

        // Recurse into children
        double halfSize = nodeSize * 0.5;
        for (int i = 0; i < 8; i++) {
            if (node.children[i] == NodePool::NULL_NODE) continue;
            Vec3 childOrigin = nodeOrigin + Vec3(
                (i & 1) ? halfSize : 0,
                (i & 2) ? halfSize : 0,
                (i & 4) ? halfSize : 0
            );
            digRecursive(chunk, node.children[i], childOrigin, halfSize,
                        center, radius, depth + 1);
        }

        // Try to collapse: if all children are same material leaves, merge
        tryCollapse(nodeIdx);
    }

    // ===== RECURSIVE BUILD =====
    void buildRecursive(SVOChunk& chunk, uint32_t nodeIdx,
                       Vec3 nodeOrigin, double nodeSize,
                       const Vec3& center, double radius, Material mat, int depth) {
        if (nodeIdx == NodePool::NULL_NODE) return;

        SVONode& node = pool.get(nodeIdx);
        Vec3 nodeCenter = nodeOrigin + Vec3(nodeSize * 0.5, nodeSize * 0.5, nodeSize * 0.5);

        // Node entirely inside sphere → fill with material
        if (sphereContainsAABB(center, radius, nodeOrigin, nodeSize)) {
            if (!node.isLeaf()) freeChildren(nodeIdx);
            node.setLeaf(true);
            node.material = mat;
            node.density = 255;
            node.setHomogeneous(true);
            return;
        }

        // Node entirely outside sphere → no change
        if (!sphereIntersectsAABB(center, radius, nodeOrigin, nodeSize)) return;

        // Already the target material → nothing to build
        if (node.isLeaf() && node.material == mat) return;

        // Partial overlap: subdivide
        if (node.isLeaf()) {
            if (depth < chunk.maxDepth) {
                expandLeaf(nodeIdx, node.material, node.density);
            } else {
                double dist = (nodeCenter - center).length();
                if (dist < radius) {
                    node.material = mat;
                    node.density = 255;
                }
                return;
            }
        }

        double halfSize = nodeSize * 0.5;
        for (int i = 0; i < 8; i++) {
            if (node.children[i] == NodePool::NULL_NODE) continue;
            Vec3 childOrigin = nodeOrigin + Vec3(
                (i & 1) ? halfSize : 0,
                (i & 2) ? halfSize : 0,
                (i & 4) ? halfSize : 0
            );
            buildRecursive(chunk, node.children[i], childOrigin, halfSize,
                          center, radius, mat, depth + 1);
        }

        tryCollapse(nodeIdx);
    }

    // ===== TREE MANAGEMENT =====
    void expandLeaf(uint32_t nodeIdx, Material mat, uint8_t density) {
        SVONode& node = pool.get(nodeIdx);
        node.setLeaf(false);
        for (int i = 0; i < 8; i++) {
            node.children[i] = pool.allocate();
            if (node.children[i] != NodePool::NULL_NODE) {
                SVONode& child = pool.get(node.children[i]);
                child.setLeaf(true);
                child.setHomogeneous(true);
                child.material = mat;
                child.density = density;
            }
        }
    }

    void freeChildren(uint32_t nodeIdx) {
        SVONode& node = pool.get(nodeIdx);
        for (int i = 0; i < 8; i++) {
            if (node.children[i] != NodePool::NULL_NODE) {
                freeSubtree(node.children[i]);
                node.children[i] = NodePool::NULL_NODE;
            }
        }
    }

    void freeSubtree(uint32_t nodeIdx) {
        if (nodeIdx == NodePool::NULL_NODE) return;
        SVONode& node = pool.get(nodeIdx);
        if (!node.isLeaf()) {
            for (int i = 0; i < 8; i++) {
                if (node.children[i] != NodePool::NULL_NODE) {
                    freeSubtree(node.children[i]);
                }
            }
        }
        pool.deallocate(nodeIdx);
    }

    void tryCollapse(uint32_t nodeIdx) {
        SVONode& node = pool.get(nodeIdx);
        if (node.isLeaf()) return;

        // Check if all children are same-material leaves
        Material firstMat = Material::AIR;
        bool allSame = true;
        bool first = true;

        for (int i = 0; i < 8; i++) {
            if (node.children[i] == NodePool::NULL_NODE) { allSame = false; break; }
            const SVONode& child = pool.get(node.children[i]);
            if (!child.isLeaf()) { allSame = false; break; }
            if (first) { firstMat = child.material; first = false; }
            else if (child.material != firstMat) { allSame = false; break; }
        }

        if (allSame && !first) {
            freeChildren(nodeIdx);
            node.setLeaf(true);
            node.setHomogeneous(true);
            node.material = firstMat;
            node.density = (firstMat == Material::AIR) ? 0 : 255;
        }
    }

    // ===== GEOMETRY HELPERS =====
    static bool sphereIntersectsAABB(const Vec3& center, double radius,
                                      const Vec3& aabbMin, double aabbSize) {
        Vec3 aabbMax = aabbMin + Vec3(aabbSize, aabbSize, aabbSize);
        double dist2 = 0;
        if (center.x < aabbMin.x) dist2 += (center.x - aabbMin.x) * (center.x - aabbMin.x);
        else if (center.x > aabbMax.x) dist2 += (center.x - aabbMax.x) * (center.x - aabbMax.x);
        if (center.y < aabbMin.y) dist2 += (center.y - aabbMin.y) * (center.y - aabbMin.y);
        else if (center.y > aabbMax.y) dist2 += (center.y - aabbMax.y) * (center.y - aabbMax.y);
        if (center.z < aabbMin.z) dist2 += (center.z - aabbMin.z) * (center.z - aabbMin.z);
        else if (center.z > aabbMax.z) dist2 += (center.z - aabbMax.z) * (center.z - aabbMax.z);
        return dist2 <= radius * radius;
    }

    static bool sphereContainsAABB(const Vec3& center, double radius,
                                    const Vec3& aabbMin, double aabbSize) {
        // Check all 8 corners are inside the sphere
        double r2 = radius * radius;
        for (int i = 0; i < 8; i++) {
            Vec3 corner = aabbMin + Vec3(
                (i & 1) ? aabbSize : 0,
                (i & 2) ? aabbSize : 0,
                (i & 4) ? aabbSize : 0
            );
            if ((corner - center).lengthSq() > r2) return false;
        }
        return true;
    }
};

} // namespace SVO
