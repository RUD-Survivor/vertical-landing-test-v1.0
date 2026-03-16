#pragma once
#include "math/math3d.h"
#include <vector>
#include <cmath>

namespace Vegetation {

struct InstanceData {
    float pos[3];
    float scale;
    float rot; // Y-axis rotation
};

class VegetationSystem {
public:
    struct VegetationType {
        float minAlt, maxAlt;
        float minLat, maxLat;
        float probability;
        float density;
        float scaleMin, scaleMax;
    };

    std::vector<VegetationType> types;

    VegetationSystem() {
        // Define some default biomes (matching Earth shader logic)
        
        // Forests (Temperate)
        VegetationType temperateForest;
        temperateForest.minAlt = 0.05f; temperateForest.maxAlt = 0.4f;
        temperateForest.minLat = -0.6f; temperateForest.maxLat = 0.6f;
        temperateForest.probability = 0.4f;
        temperateForest.density = 0.05f;
        temperateForest.scaleMin = 0.8f; temperateForest.scaleMax = 1.8f;
        types.push_back(temperateForest);

        // Grass (Tundra/Grassland)
        VegetationType grassland;
        grassland.minAlt = 0.01f; grassland.maxAlt = 0.6f;
        grassland.minLat = -0.8f; grassland.maxLat = 0.8f;
        grassland.probability = 0.6f;
        grassland.density = 0.2f;
        grassland.scaleMin = 0.3f; grassland.scaleMax = 0.6f;
        types.push_back(grassland);
    }

    // Function to generate instances for a local patch
    // In a real implementation, this would be done on GPU or cached.
    // For this sim, we'll generate seeds based on position.
};

} // namespace Vegetation
