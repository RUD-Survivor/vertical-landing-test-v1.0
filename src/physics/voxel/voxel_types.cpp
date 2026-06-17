#include "voxel_types.h"

namespace VoxelPhysics {

MaterialProperties GetMaterialProperties(VoxelMaterial material) {
    MaterialProperties props;
    switch (material) {
        case VoxelMaterial::Aluminum:
            props.density = 2700.0;
            props.tensile_strength = 1.5e7;
            props.shear_strength = 9.0e6;
            props.toughness = 2.0e5;
            props.heat_limit = 850.0;
            break;
        case VoxelMaterial::Composite:
            props.density = 1600.0;
            props.tensile_strength = 2.2e7;
            props.shear_strength = 7.0e6;
            props.toughness = 1.5e5;
            props.heat_limit = 700.0;
            break;
        case VoxelMaterial::Steel:
            props.density = 7800.0;
            props.tensile_strength = 4.5e7;
            props.shear_strength = 2.8e7;
            props.toughness = 3.5e5;
            props.heat_limit = 1200.0;
            break;
        case VoxelMaterial::Fuel:
            props.density = 850.0;
            props.tensile_strength = 2.0e6;
            props.shear_strength = 1.0e6;
            props.toughness = 5.0e4;
            props.heat_limit = 450.0;
            props.contains_fuel = true;
            break;
        case VoxelMaterial::Engine:
            props.density = 5200.0;
            props.tensile_strength = 5.5e7;
            props.shear_strength = 3.0e7;
            props.toughness = 4.0e5;
            props.heat_limit = 1500.0;
            break;
        case VoxelMaterial::Structure:
            props.density = 3200.0;
            props.tensile_strength = 1.8e7;
            props.shear_strength = 1.2e7;
            props.toughness = 2.5e5;
            props.heat_limit = 900.0;
            break;
        case VoxelMaterial::Unknown:
        default:
            break;
    }
    return props;
}

VoxelMaterial MaterialFromPartCategory(int category, bool hasFuel, bool hasThrust) {
    if (hasThrust) return VoxelMaterial::Engine;
    if (hasFuel) return VoxelMaterial::Fuel;
    switch (category) {
        case 0: return VoxelMaterial::Composite;
        case 1: return VoxelMaterial::Aluminum;
        case 2: return VoxelMaterial::Aluminum;
        case 3: return VoxelMaterial::Engine;
        case 4: return VoxelMaterial::Steel;
        case 5: return VoxelMaterial::Structure;
        default: return VoxelMaterial::Unknown;
    }
}

} // namespace VoxelPhysics
