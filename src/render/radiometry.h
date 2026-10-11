#pragma once
#include <algorithm>
#include <cmath>

// Relative linear radiometry. One sun at 1 AU has normalized irradiance 1.1;
// intensity is the snapshot's inverse-square distance factor, not exposure.
namespace Radiometry {
constexpr double pi=3.14159265358979323846;
constexpr float unitSolarIrradiance=1.1f;
// Calibrated with the hemispheric 18% gray fixture; lens appearance must be fit
// at this scene-wide exposure rather than dimming the scene to hide highlights.
constexpr float defaultDisplayExposure=3.25f;
inline float solarIrradiance(float intensity) {
    return std::isfinite(intensity)?unitSolarIrradiance*std::max(0.f,intensity):0.f;
}
inline double aces(double radiance,double exposure=1) {
    double x=std::clamp(radiance*std::max(0.0,exposure),0.0,1e6);
    // Normalize the fitted shoulder's asymptote instead of clipping it early.
    return std::clamp((x*(2.51*x+.03))/(x*(2.43*x+.59)+.14)*(2.43/2.51),0.0,1.0);
}
inline double srgb(double linear) {
    double x=std::clamp(linear,0.0,1.0);
    return x<=.0031308?12.92*x:1.055*std::pow(x,1/2.4)-.055;
}
inline double lambert(double reflectance,double irradiance) {
    return std::clamp(reflectance,0.0,1.0)*std::max(0.0,irradiance)/pi;
}
}
