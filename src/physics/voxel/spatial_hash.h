#pragma once

#include "voxel_types.h"
#include <cstdint>
#include <unordered_map>
#include <vector>

namespace VoxelPhysics {

struct VoxelCoordHash {
    size_t operator()(const VoxelCoord& c) const {
        uint64_t x = static_cast<uint32_t>(c.x) * 73856093u;
        uint64_t y = static_cast<uint32_t>(c.y) * 19349663u;
        uint64_t z = static_cast<uint32_t>(c.z) * 83492791u;
        return static_cast<size_t>(x ^ y ^ z);
    }
};

class SpatialHash {
public:
    void clear() { table.clear(); }

    void insert(const VoxelCoord& coord, VoxelId id) {
        table[coord] = id;
    }

    bool contains(const VoxelCoord& coord) const {
        return table.find(coord) != table.end();
    }

    VoxelId get(const VoxelCoord& coord) const {
        auto it = table.find(coord);
        return it == table.end() ? kInvalidVoxel : it->second;
    }

    std::vector<VoxelId> sixNeighbors(const VoxelCoord& coord) const {
        static const int dirs[6][3] = {
            { 1, 0, 0 }, { -1, 0, 0 },
            { 0, 1, 0 }, { 0, -1, 0 },
            { 0, 0, 1 }, { 0, 0, -1 }
        };
        std::vector<VoxelId> out;
        out.reserve(6);
        for (const auto& d : dirs) {
            VoxelId id = get({ coord.x + d[0], coord.y + d[1], coord.z + d[2] });
            if (id != kInvalidVoxel) out.push_back(id);
        }
        return out;
    }

    std::vector<VoxelId> radiusQuery(const VoxelCoord& center, int radius) const {
        std::vector<VoxelId> out;
        int r2 = radius * radius;
        for (int z = center.z - radius; z <= center.z + radius; z++) {
            for (int y = center.y - radius; y <= center.y + radius; y++) {
                for (int x = center.x - radius; x <= center.x + radius; x++) {
                    int dx = x - center.x;
                    int dy = y - center.y;
                    int dz = z - center.z;
                    if (dx * dx + dy * dy + dz * dz > r2) continue;
                    VoxelId id = get({ x, y, z });
                    if (id != kInvalidVoxel) out.push_back(id);
                }
            }
        }
        return out;
    }

    size_t size() const { return table.size(); }

private:
    std::unordered_map<VoxelCoord, VoxelId, VoxelCoordHash> table;
};

} // namespace VoxelPhysics
