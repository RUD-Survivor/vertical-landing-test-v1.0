#include "aerodynamics_system.h"
#include <algorithm>
#include <cmath>

namespace AerodynamicsSystem {

namespace {

constexpr double kGasConstantAir = 287.05;
constexpr double kGammaAir = 1.4;
constexpr double kEarthScaleHeight = 7000.0;
constexpr double kMinAtmosphereDensity = 1.0e-6;

static double clamp01(double v) {
    return std::max(0.0, std::min(1.0, v));
}

static double smoothstep(double a, double b, double x) {
    if (a == b) return x >= b ? 1.0 : 0.0;
    double t = clamp01((x - a) / (b - a));
    return t * t * (3.0 - 2.0 * t);
}

static double safePeriod(double period) {
    return std::abs(period) > 1.0 ? period : 1.0;
}

static double speedOfSound(const CelestialBody& body, double altitude) {
    double lapseTemp = body.average_temp - 0.0065 * std::max(0.0, altitude);
    double temp = std::max(120.0, std::min(body.average_temp, lapseTemp));
    return std::sqrt(kGammaAir * kGasConstantAir * temp);
}

static double profileFlatness(const AeroProfile& profile) {
    if (profile.sections.empty()) return 1.0;
    double weighted = 0.0;
    double areaSum = 0.0;
    for (const AeroCrossSection& section : profile.sections) {
        weighted += section.flatness_ratio * section.area;
        areaSum += section.area;
    }
    return areaSum > 1.0e-9 ? weighted / areaSum : 1.0;
}

static double fallbackFrontalArea(const RocketConfig& config) {
    double radius = std::max(0.1, config.diameter * 0.5);
    return PI * radius * radius;
}

static double fallbackSideArea(const RocketConfig& config) {
    return std::max(0.1, config.height) * std::max(0.1, config.diameter);
}

} // namespace

double AirDensityAt(const CelestialBody& body, double altitude) {
    if (altitude > 100000.0 || body.surface_pressure <= 1.0e-9) return 0.0;

    double surfacePressurePa = body.surface_pressure * 100.0;
    double surfaceTemp = std::max(80.0, body.average_temp);
    double rho0 = surfacePressurePa / (kGasConstantAir * surfaceTemp);
    if (!std::isfinite(rho0) || rho0 <= 0.0) return 0.0;

    double surfaceGravity = G_const * body.mass / std::max(1.0, body.radius * body.radius);
    double scaleHeight = kGasConstantAir * surfaceTemp / std::max(0.1, surfaceGravity);
    if (!std::isfinite(scaleHeight) || scaleHeight <= 0.0) scaleHeight = kEarthScaleHeight;
    scaleHeight = std::max(1500.0, std::min(50000.0, scaleHeight));

    double rho = rho0 * std::exp(-std::max(0.0, altitude) / scaleHeight);
    return std::isfinite(rho) ? rho : 0.0;
}

double AngularDampingCoefficient(const RocketConfig& config,
                                 const CelestialBody& body,
                                 double altitude,
                                 double speed) {
    if (speed <= 0.1 || altitude >= 100000.0) return 0.0;
    double rho = AirDensityAt(body, altitude);
    double referenceArea = config.aero_profile.valid()
        ? config.aero_profile.side_area
        : fallbackSideArea(config);
    double lengthScale = std::max(1.0, config.aero_profile.valid()
        ? config.aero_profile.reference_length
        : config.height);
    double normalizedArea = referenceArea / (lengthScale * lengthScale);
    return -0.08 * rho * speed * std::min(2.0, std::max(0.1, normalizedArea));
}

Sample ComputeForces(const RocketConfig& config,
                     const AttitudeComponent& attitude,
                     const CelestialBody& body,
                     double /*sim_time*/,
                     const Vec3& position,
                     const Vec3& velocity) {
    Sample sample;
    double radius = position.length();
    double altitude = radius - body.radius;
    double rho = AirDensityAt(body, altitude);
    if (rho <= kMinAtmosphereDensity) return sample;

    double omega = (2.0 * PI) / safePeriod(body.rotation_period);
    Vec3 atmosphereVelocity(-omega * position.y, omega * position.x, 0.0);
    Vec3 relVelocity = velocity - atmosphereVelocity;
    double speedSq = relVelocity.lengthSq();
    if (speedSq <= 0.01) return sample;

    double speed = std::sqrt(speedSq);
    Vec3 velDir = relVelocity / speed;
    Vec3 dragDir = -velDir;
    Vec3 bodyAxis = attitude.attitude.forward().normalized();
    double axialAlignment = std::abs(bodyAxis.dot(velDir));
    axialAlignment = std::max(0.0, std::min(1.0, axialAlignment));
    double sinAlpha = std::sqrt(std::max(0.0, 1.0 - axialAlignment * axialAlignment));
    sample.angle_of_attack = std::atan2(sinAlpha, axialAlignment);

    const AeroProfile& profile = config.aero_profile;
    double frontalArea = profile.valid() ? profile.frontal_area : fallbackFrontalArea(config);
    double sideArea = profile.valid() ? profile.side_area : fallbackSideArea(config);
    double flatness = profile.valid() ? profileFlatness(profile) : 1.0;
    double roughness = profile.valid() ? profile.area_roughness : 0.0;

    double axialArea = frontalArea * axialAlignment * axialAlignment;
    double broadsideArea = sideArea * sinAlpha * sinAlpha;
    sample.effective_area = std::max(0.05, axialArea + broadsideArea);
    sample.dynamic_pressure = 0.5 * rho * speedSq;
    sample.mach = speed / std::max(1.0, speedOfSound(body, altitude));

    double slenderCd = 0.16 + 0.035 * std::min(6.0, flatness - 1.0);
    double broadsideCd = 0.85;
    double baseCd = slenderCd * axialAlignment + broadsideCd * sinAlpha;

    double transonicRise = smoothstep(0.72, 1.05, sample.mach);
    double supersonicHold = 1.0 - 0.35 * smoothstep(1.2, 3.5, sample.mach);
    double waveCd = transonicRise * supersonicHold * (0.05 + 0.45 * std::min(2.0, roughness));
    sample.drag_coefficient = std::max(0.03, baseCd + waveCd);

    Vec3 dragForce = dragDir * (sample.dynamic_pressure * sample.drag_coefficient * sample.effective_area);

    Vec3 liftAxis = bodyAxis - velDir * bodyAxis.dot(velDir);
    Vec3 liftForce;
    if (liftAxis.lengthSq() > 1.0e-8) {
        double cl = 0.10 * std::min(2.0, flatness) * std::sin(2.0 * sample.angle_of_attack);
        liftForce = liftAxis.normalized() * (sample.dynamic_pressure * cl * sideArea * 0.25);
    }

    sample.force = dragForce + liftForce;
    return sample;
}

} // namespace AerodynamicsSystem
