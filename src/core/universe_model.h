#pragma once
#include <vector>
#include "rocket_state.h"

class UniverseModel {
public:
    static UniverseModel& getInstance() {
        static UniverseModel instance;
        return instance;
    }

    std::vector<CelestialBody> solar_system;
    int current_soi_index = 3;
};
