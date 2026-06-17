#pragma once

#include "math/math3d.h"
#include <cstdint>
#include <vector>

namespace VoxelPhysics {

using VoxelId = int32_t;
using BondId = int32_t;
using ChunkId = int32_t;

constexpr VoxelId kInvalidVoxel = -1;
constexpr BondId kInvalidBond = -1;
constexpr ChunkId kInvalidChunk = -1;

struct VoxelCoord {
    int x = 0;
    int y = 0;
    int z = 0;

    bool operator==(const VoxelCoord& other) const {
        return x == other.x && y == other.y && z == other.z;
    }
};

enum class VoxelMaterial : uint8_t {
    Aluminum,
    Composite,
    Steel,
    Fuel,
    Engine,
    Structure,
    Unknown
};

struct MaterialProperties {
    double density = 2700.0;
    double tensile_strength = 1.5e7;
    double shear_strength = 1.0e7;
    double toughness = 2.0e5;
    double heat_limit = 900.0;
    bool contains_fuel = false;
};

struct VoxelCell {
    VoxelCoord coord;
    Vec3 center;
    VoxelMaterial material = VoxelMaterial::Unknown;
    int part_index = -1;
    double mass = 0.0;
    double damage = 0.0;
    bool surface = false;
    bool active = true;
    ChunkId chunk = 0;
};

struct VoxelBond {
    VoxelId a = kInvalidVoxel;
    VoxelId b = kInvalidVoxel;
    double rest_length = 0.0;
    double tensile_strength = 0.0;
    double shear_strength = 0.0;
    double toughness = 0.0;
    double damage = 0.0;
    bool broken = false;
};

struct VoxelSurfaceSample {
    VoxelId voxel = kInvalidVoxel;
    Vec3 local_pos;
    Vec3 normal;
    double area = 0.0;
    VoxelMaterial material = VoxelMaterial::Unknown;
};

struct VoxelChunkSummary {
    ChunkId id = kInvalidChunk;
    std::vector<VoxelId> voxels;
    std::vector<int> surface_indices;
    double mass = 0.0;
    Vec3 center_of_mass;
    Vec3 inertia_diag;
    Vec3 bounds_min;
    Vec3 bounds_max;
    double frontal_area = 0.0;
    double side_area = 0.0;
};

MaterialProperties GetMaterialProperties(VoxelMaterial material);
VoxelMaterial MaterialFromPartCategory(int category, bool hasFuel, bool hasThrust);

} // namespace VoxelPhysics
