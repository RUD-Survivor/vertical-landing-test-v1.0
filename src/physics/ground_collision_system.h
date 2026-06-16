#pragma once

#include "core/rocket_state.h"
#include "math/math3d.h"
#include <vector>

namespace GroundCollisionSystem {

struct ContactPoint {
    Vec3 local;
    Vec3 world;
    Vec3 normal;
    Vec3 groundVelocity;
    double penetration = 0.0;
    double normalSpeed = 0.0;
    double tangentSpeed = 0.0;
    bool isLandingLeg = false;
};

struct ContactResult {
    int contactCount = 0;
    int legContactCount = 0;
    double maxPenetration = 0.0;
    double maxClosingSpeed = 0.0;
    double maxTangentSpeed = 0.0;
    bool hardImpact = false;
    bool stable = false;
    bool anyNonLegHardContact = false;
};

double SampleTerrainHeightMeters(const Vec3& surfaceNormal, const TelemetryComponent& tele);
Vec3 SurfaceVelocityAt(const CelestialBody& body, double simTime, const Vec3& worldPos);
void FreezeSurfacePoint(TransformComponent& trans, const CelestialBody& body, double simTime);

ContactResult ResolveGroundContacts(const RocketConfig& config,
                                    const PropulsionComponent& prop,
                                    TransformComponent& trans,
                                    VelocityComponent& vel,
                                    AttitudeComponent& att,
                                    GuidanceComponent& guid,
                                    const TelemetryComponent& tele,
                                    const CelestialBody& body,
                                    double totalMass,
                                    double simTime,
                                    double dt,
                                    const Vec3& previousCom);

} // namespace GroundCollisionSystem
