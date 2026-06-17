#include "vessel_voxel_model.h"
#include <algorithm>
#include <cmath>
#include <queue>

namespace VoxelPhysics {

VesselVoxelModel::VesselVoxelModel(double voxelSizeMeters)
    : voxel_size(std::max(0.1, voxelSizeMeters)) {}

void VesselVoxelModel::clear() {
    hash.clear();
    cells.clear();
    bonds.clear();
    surfaces.clear();
    chunks.clear();
}

VoxelId VesselVoxelModel::addCell(const VoxelCoord& coord,
                                  VoxelMaterial material,
                                  int partIndex,
                                  double mass) {
    VoxelId existing = hash.get(coord);
    if (existing != kInvalidVoxel) {
        VoxelCell& cell = cells[(size_t)existing];
        cell.mass += mass;
        if (cell.material == VoxelMaterial::Unknown) cell.material = material;
        return existing;
    }

    VoxelId id = (VoxelId)cells.size();
    VoxelCell cell;
    cell.coord = coord;
    cell.center = coordToCenter(coord);
    cell.material = material;
    cell.part_index = partIndex;
    cell.mass = std::max(0.001, mass);
    cells.push_back(cell);
    hash.insert(coord, id);
    return id;
}

Vec3 VesselVoxelModel::coordToCenter(const VoxelCoord& coord) const {
    return Vec3((coord.x + 0.5) * voxel_size,
                (coord.y + 0.5) * voxel_size,
                (coord.z + 0.5) * voxel_size);
}

VoxelCoord VesselVoxelModel::localToCoord(const Vec3& p) const {
    return {
        (int)std::floor(p.x / voxel_size),
        (int)std::floor(p.y / voxel_size),
        (int)std::floor(p.z / voxel_size)
    };
}

VoxelId VesselVoxelModel::findCell(const VoxelCoord& coord) const {
    return hash.get(coord);
}

void VesselVoxelModel::rebuildDerivedData() {
    rebuildBonds();
    rebuildSurfaces();
    chunks = buildConnectedChunks();
}

void VesselVoxelModel::rebuildBonds() {
    bonds.clear();
    static const int dirs[3][3] = {
        { 1, 0, 0 }, { 0, 1, 0 }, { 0, 0, 1 }
    };

    for (VoxelId a = 0; a < (VoxelId)cells.size(); a++) {
        const VoxelCell& ca = cells[(size_t)a];
        if (!ca.active) continue;
        for (const auto& d : dirs) {
            VoxelCoord nc{ ca.coord.x + d[0], ca.coord.y + d[1], ca.coord.z + d[2] };
            VoxelId b = hash.get(nc);
            if (b == kInvalidVoxel || !cells[(size_t)b].active) continue;

            MaterialProperties pa = GetMaterialProperties(ca.material);
            MaterialProperties pb = GetMaterialProperties(cells[(size_t)b].material);
            VoxelBond bond;
            bond.a = a;
            bond.b = b;
            bond.rest_length = voxel_size;
            bond.tensile_strength = std::min(pa.tensile_strength, pb.tensile_strength);
            bond.shear_strength = std::min(pa.shear_strength, pb.shear_strength);
            bond.toughness = std::min(pa.toughness, pb.toughness);
            bonds.push_back(bond);
        }
    }
}

void VesselVoxelModel::rebuildSurfaces() {
    surfaces.clear();
    static const int dirs[6][3] = {
        { 1, 0, 0 }, { -1, 0, 0 },
        { 0, 1, 0 }, { 0, -1, 0 },
        { 0, 0, 1 }, { 0, 0, -1 }
    };
    static const Vec3 normals[6] = {
        Vec3(1, 0, 0), Vec3(-1, 0, 0),
        Vec3(0, 1, 0), Vec3(0, -1, 0),
        Vec3(0, 0, 1), Vec3(0, 0, -1)
    };

    auto bonded = [this](VoxelId a, VoxelId b) -> bool {
        for (const VoxelBond& bond : bonds) {
            if (bond.broken) continue;
            if ((bond.a == a && bond.b == b) || (bond.a == b && bond.b == a)) return true;
        }
        return false;
    };

    double faceArea = voxel_size * voxel_size;
    for (VoxelId id = 0; id < (VoxelId)cells.size(); id++) {
        VoxelCell& cell = cells[(size_t)id];
        cell.surface = false;
        if (!cell.active) continue;
        for (int i = 0; i < 6; i++) {
            VoxelCoord nc{ cell.coord.x + dirs[i][0], cell.coord.y + dirs[i][1], cell.coord.z + dirs[i][2] };
            VoxelId neighbor = hash.get(nc);
            if (neighbor != kInvalidVoxel && cells[(size_t)neighbor].active && bonded(id, neighbor)) continue;

            cell.surface = true;
            VoxelSurfaceSample sample;
            sample.voxel = id;
            sample.local_pos = cell.center;
            sample.normal = normals[i];
            sample.area = faceArea;
            sample.material = cell.material;
            surfaces.push_back(sample);
        }
    }
}

VoxelChunkSummary VesselVoxelModel::summarizeVoxels(const std::vector<VoxelId>& voxelIds, ChunkId chunkId) const {
    VoxelChunkSummary summary;
    summary.id = chunkId;
    summary.voxels = voxelIds;
    summary.bounds_min = Vec3(1e30, 1e30, 1e30);
    summary.bounds_max = Vec3(-1e30, -1e30, -1e30);

    Vec3 weighted(0, 0, 0);
    for (VoxelId id : voxelIds) {
        if (id < 0 || id >= (VoxelId)cells.size()) continue;
        const VoxelCell& cell = cells[(size_t)id];
        if (!cell.active) continue;
        summary.mass += cell.mass;
        weighted += cell.center * cell.mass;
        double half = voxel_size * 0.5;
        summary.bounds_min.x = std::min(summary.bounds_min.x, cell.center.x - half);
        summary.bounds_min.y = std::min(summary.bounds_min.y, cell.center.y - half);
        summary.bounds_min.z = std::min(summary.bounds_min.z, cell.center.z - half);
        summary.bounds_max.x = std::max(summary.bounds_max.x, cell.center.x + half);
        summary.bounds_max.y = std::max(summary.bounds_max.y, cell.center.y + half);
        summary.bounds_max.z = std::max(summary.bounds_max.z, cell.center.z + half);
    }

    if (summary.mass > 0.0) summary.center_of_mass = weighted / summary.mass;
    Vec3 inertia(0, 0, 0);
    for (VoxelId id : voxelIds) {
        const VoxelCell& cell = cells[(size_t)id];
        Vec3 r = cell.center - summary.center_of_mass;
        inertia.x += cell.mass * (r.y * r.y + r.z * r.z);
        inertia.y += cell.mass * (r.x * r.x + r.z * r.z);
        inertia.z += cell.mass * (r.x * r.x + r.y * r.y);
    }
    summary.inertia_diag = Vec3(std::max(1.0, inertia.x),
                                std::max(1.0, inertia.y),
                                std::max(1.0, inertia.z));

    for (int i = 0; i < (int)surfaces.size(); i++) {
        VoxelId voxel = surfaces[(size_t)i].voxel;
        if (std::find(voxelIds.begin(), voxelIds.end(), voxel) == voxelIds.end()) continue;
        summary.surface_indices.push_back(i);
        double axial = std::abs(surfaces[(size_t)i].normal.y);
        summary.frontal_area += surfaces[(size_t)i].area * axial;
        summary.side_area += surfaces[(size_t)i].area * (1.0 - axial);
    }
    return summary;
}

std::vector<VoxelChunkSummary> VesselVoxelModel::buildConnectedChunks() {
    std::vector<std::vector<VoxelId>> adjacency(cells.size());
    for (const VoxelBond& bond : bonds) {
        if (bond.broken || bond.a == kInvalidVoxel || bond.b == kInvalidVoxel) continue;
        if (!cells[(size_t)bond.a].active || !cells[(size_t)bond.b].active) continue;
        adjacency[(size_t)bond.a].push_back(bond.b);
        adjacency[(size_t)bond.b].push_back(bond.a);
    }

    std::vector<VoxelChunkSummary> result;
    std::vector<uint8_t> visited(cells.size(), 0);
    for (VoxelId start = 0; start < (VoxelId)cells.size(); start++) {
        if (visited[(size_t)start] || !cells[(size_t)start].active) continue;
        std::vector<VoxelId> group;
        std::queue<VoxelId> q;
        visited[(size_t)start] = 1;
        q.push(start);
        while (!q.empty()) {
            VoxelId id = q.front();
            q.pop();
            group.push_back(id);
            for (VoxelId next : adjacency[(size_t)id]) {
                if (visited[(size_t)next]) continue;
                visited[(size_t)next] = 1;
                q.push(next);
            }
        }
        ChunkId chunkId = (ChunkId)result.size();
        for (VoxelId id : group) cells[(size_t)id].chunk = chunkId;
        result.push_back(summarizeVoxels(group, chunkId));
    }
    chunks = result;
    return result;
}

void VesselVoxelModel::breakBond(BondId id) {
    if (id < 0 || id >= (BondId)bonds.size()) return;
    bonds[(size_t)id].broken = true;
}

void VesselVoxelModel::deactivateVoxels(const std::vector<VoxelId>& ids) {
    for (VoxelId id : ids) {
        if (id < 0 || id >= (VoxelId)cells.size()) continue;
        cells[(size_t)id].active = false;
    }
    for (VoxelBond& bond : bonds) {
        if (bond.broken) continue;
        bool aDead = bond.a == kInvalidVoxel || !cells[(size_t)bond.a].active;
        bool bDead = bond.b == kInvalidVoxel || !cells[(size_t)bond.b].active;
        if (aDead || bDead) bond.broken = true;
    }
}

int VesselVoxelModel::countActiveVoxels() const {
    int count = 0;
    for (const VoxelCell& cell : cells) {
        if (cell.active) count++;
    }
    return count;
}

void VesselVoxelModel::applyRadialDamage(const Vec3& localCenter, double radius, double impulse) {
    if (radius <= 0.0 || impulse <= 0.0) return;
    double r2 = radius * radius;
    for (VoxelBond& bond : bonds) {
        if (bond.broken) continue;
        Vec3 mid = (cells[(size_t)bond.a].center + cells[(size_t)bond.b].center) * 0.5;
        double d2 = (mid - localCenter).lengthSq();
        if (d2 > r2) continue;
        double falloff = 1.0 - std::sqrt(d2) / radius;
        double denom = std::max(250.0, bond.toughness * 0.03);
        bond.damage += impulse * falloff / denom;
        if (bond.damage >= 1.0) bond.broken = true;
    }
}

} // namespace VoxelPhysics
