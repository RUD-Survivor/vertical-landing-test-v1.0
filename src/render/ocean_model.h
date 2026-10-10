#pragma once
#include <algorithm>
#include <cmath>
// CPU reference and phase rebasing for the GPU ocean model. All optical units: m.
namespace OceanModel {
constexpr double pi = 3.14159265358979323846;
constexpr double seaLevel = 0.45;
constexpr double lengths[12] = {512,192,72,32,18,10,6,4,2.8,1.8,1.2,0.8};
constexpr double amplitudes[12] = {0.65,0.4,0.22,0.15,0.1,0.07,0.045,0.03,0.021,0.013,0.009,0.006};
constexpr double directions[12][3] = {
    {1,.17,.31},{.73,.24,.64},{.92,-.2,-.36},{.6,.51,.62},
    {.84,.1,-.53},{.51,-.35,.79},{.34,.81,.47},{-.46,.22,.86},
    {.77,-.56,.3},{-.68,.66,.31},{.19,.93,-.32},{.42,-.8,.43}};
inline double bedKm(double height, double maxElevationKm) {
    return height < seaLevel ? (height - seaLevel) / seaLevel * 6.0
        : (height - seaLevel) / (1.0 - seaLevel) * maxElevationKm;
}
inline double depthM(double bed, double filled, double maxElevationKm) {
    double level = bed < seaLevel ? 0.0 : bedKm(std::max(bed, filled), maxElevationKm);
    return std::max(0.0, level - bedKm(bed, maxElevationKm)) * 1000.0;
}
inline void phases(const float anchor[3], double radiusM, double time, float result[12]) {
    for (int i = 0; i < 12; ++i) {
        const auto& d = directions[i];
        double len = std::sqrt(d[0]*d[0] + d[1]*d[1] + d[2]*d[2]);
        double k = 2.0 * pi / lengths[i];
        double p = radiusM * (anchor[0]*d[0] + anchor[1]*d[1] + anchor[2]*d[2]) / len;
        result[i] = static_cast<float>(std::remainder(k*p - std::sqrt(9.81*k)*time, 2.0*pi));
    }
}
inline double fresnel(double cosTheta) {
    constexpr double f0 = ((1.333 - 1.0) / (1.333 + 1.0)) * ((1.333 - 1.0) / (1.333 + 1.0));
    return f0 + (1.0-f0)*std::pow(1.0-std::clamp(cosTheta,0.0,1.0),5);
}
inline double transmission(double extinctionPerM, double pathM) {
    return std::exp(-std::max(0.0,extinctionPerM)*std::max(0.0,pathM));
}
}
