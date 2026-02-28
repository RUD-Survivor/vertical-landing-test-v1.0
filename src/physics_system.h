#ifndef PHYSICS_SYSTEM_H
#define PHYSICS_SYSTEM_H

#include "rocket_state.h"

namespace PhysicsSystem {
    // Math Utilities
    double get_gravity(double r);
    double get_pressure(double h);
    double get_air_density(double h);
    void getOrbitParams(const RocketState& state, double& apoapsis, double& periapsis);
    // Planetary Ephemeris and Coordinates
    void InitSolarSystem();
    void UpdateCelestialBodies(double current_time_sec);
    void CheckSOI_Transitions(RocketState& state);
    double CalculateSolarOcclusion(const RocketState& state);
    
    // Core Simulation Update
    void Update(RocketState& state, const RocketConfig& config, const ControlInput& input, double dt);
    void FastGravityUpdate(RocketState& state, const RocketConfig& config, double dt_total);
    
    // Visual Effects Integration (Smoke)
    void EmitSmoke(RocketState& state, const RocketConfig& config, double dt);
    void UpdateSmoke(RocketState& state, double dt);
}

#endif // PHYSICS_SYSTEM_H
