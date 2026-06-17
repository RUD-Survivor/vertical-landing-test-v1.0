#pragma once

#include "vessel_voxel_model.h"
#include <memory>

struct RocketAssembly;

namespace VoxelPhysics {

struct VoxelizationSettings {
    double voxel_size = 1.0;
    int part_start_index = 0;
    int part_end_index = -1;
    int max_cells = 250000;
};

VesselVoxelModelPtr VoxelizeAssembly(const RocketAssembly& assembly,
                                     const VoxelizationSettings& settings = VoxelizationSettings());

} // namespace VoxelPhysics
