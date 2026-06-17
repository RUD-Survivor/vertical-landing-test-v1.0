#pragma once

#include "spatial_hash.h"
#include <memory>

namespace VoxelPhysics {

class VesselVoxelModel {
public:
    explicit VesselVoxelModel(double voxelSizeMeters = 1.0);

    void clear();
    VoxelId addCell(const VoxelCoord& coord, VoxelMaterial material, int partIndex, double mass);
    void rebuildDerivedData();
    void rebuildBonds();
    void rebuildSurfaces();
    std::vector<VoxelChunkSummary> buildConnectedChunks();

    void breakBond(BondId id);
    void deactivateVoxels(const std::vector<VoxelId>& ids);
    int countActiveVoxels() const;
    void applyRadialDamage(const Vec3& localCenter, double radius, double impulse);

    VoxelId findCell(const VoxelCoord& coord) const;
    const std::vector<VoxelCell>& getCells() const { return cells; }
    std::vector<VoxelCell>& getCells() { return cells; }
    const std::vector<VoxelBond>& getBonds() const { return bonds; }
    std::vector<VoxelBond>& getBonds() { return bonds; }
    const std::vector<VoxelSurfaceSample>& getSurfaceSamples() const { return surfaces; }
    const std::vector<VoxelChunkSummary>& getChunks() const { return chunks; }

    double voxelSize() const { return voxel_size; }
    double voxelVolume() const { return voxel_size * voxel_size * voxel_size; }
    Vec3 coordToCenter(const VoxelCoord& coord) const;
    VoxelCoord localToCoord(const Vec3& p) const;

    VoxelChunkSummary summarizeVoxels(const std::vector<VoxelId>& voxelIds, ChunkId chunkId) const;

private:
    double voxel_size = 1.0;
    SpatialHash hash;
    std::vector<VoxelCell> cells;
    std::vector<VoxelBond> bonds;
    std::vector<VoxelSurfaceSample> surfaces;
    std::vector<VoxelChunkSummary> chunks;
};

using VesselVoxelModelPtr = std::shared_ptr<VesselVoxelModel>;

} // namespace VoxelPhysics
