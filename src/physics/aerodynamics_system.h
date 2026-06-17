#pragma once

#include "core/rocket_state.h"
#include "math/math3d.h"

namespace AerodynamicsSystem {

struct Sample {
    Vec3 force;
    double dynamic_pressure = 0.0;
    double mach = 0.0;
    double angle_of_attack = 0.0;
    double drag_coefficient = 0.0;
    double effective_area = 0.0;
};

double AirDensityAt(const CelestialBody& body, double altitude);
double AngularDampingCoefficient(const RocketConfig& config,
                                 const CelestialBody& body,
                                 double altitude,
                                 double speed);
Sample ComputeForces(const RocketConfig& config,
                     const AttitudeComponent& attitude,
                     const CelestialBody& body,
                     double sim_time,
                     const Vec3& position,
                     const Vec3& velocity,
                     const VoxelBodyComponent* voxel = nullptr,
                     const RigidChunkComponent* chunk = nullptr);

} // namespace AerodynamicsSystem
