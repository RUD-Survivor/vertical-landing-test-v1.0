#pragma once
#include "radiometry.h"
#include <cmath>

// Relative radiometry, kilometers. Camera kernel coefficients are an explicit
// fixed-f-number approximation, not a measured lens prescription.
namespace SolarOptics {
constexpr double auKm=149597870.7;
// NASA ISS067-E-242236: 35 mm f/2, shot at f/22. Seven-blade preset;
// transfer coefficients remain a reference-guided approximation, not a lens prescription.
constexpr int apertureBlades=7;
constexpr double apertureFNumber=22;
constexpr float sourceSpectrum[]={1,.98f,.95f};
// Appearance fit includes the ISS window and processed JPEG, not pure-lens
// metrology. Fixed display exposure; each positive component has its own budget.
constexpr double ghostFraction=.00005,starFraction=.029506803260962004;
constexpr int starLines(int blades){return blades>=3&&blades<=16?(blades%2?blades:blades/2):0;}
constexpr int starRays(int blades){return 2*starLines(blades);}
constexpr double starLongSigma=.045,starShortSigma=.0009;
constexpr double responseHalfExtent=.20;
// Positive Gaussian mixture for scatter only. Diffraction uses a separate
// linear-HDR pupil-response atlas. Reflection uses a separate aperture profile.
struct Kernel {double sigmaX,sigmaY,fraction,axis;float r,g,b;};
constexpr Kernel kernels[]={
    {.0015,.0015,.0016738859370063648,1,1,1,1},
    {.006,.006,.002199225960520301,1,1,1,1},
    {.016,.016,.021008434125075325,1,.64f,.84f,1},
    {.060,.060,.002931067150643973,1,.58f,.80f,1},
    {.15,.15,.0014258934073405603,1,.56f,.76f,1}
};
// Reference-guided reflection elements, independent of the diffraction kernel.
// Regular aperture images with off-axis ellipticity, not volumetric spheres.
// These coefficients are a camera profile, not a measured lens prescription.
struct Ghost {double radius,axis,fraction;float r,g,b;};
constexpr Ghost ghosts[]={
    {.045,-.72,ghostFraction*1.35,.18f,.86f,.52f},
    {.009,.38,ghostFraction*.65,.30f,.12f,1}
};
constexpr double scatteredBudget(){double sum=starFraction;for(auto k:kernels)sum+=k.fraction;for(auto g:ghosts)sum+=g.fraction;return sum;}
constexpr double scatteredFraction=scatteredBudget();
struct Source {double sinRadius=0,angularRadius=0,radiance=0,irradiance=0,solidAngle=0;bool valid=false;};
inline Source source(double distanceKm,double radiusKm) {
    Source s;
    if(!std::isfinite(distanceKm)||!std::isfinite(radiusKm)||radiusKm<=0||distanceKm<=radiusKm)return s;
    s.valid=true;s.sinRadius=radiusKm/distanceKm;s.angularRadius=std::asin(s.sinRadius);
    double cosine=std::sqrt(std::max(0.0,1-s.sinRadius*s.sinRadius));
    s.solidAngle=2*Radiometry::pi*s.sinRadius*s.sinRadius/(1+cosine);
    s.radiance=Radiometry::unitSolarIrradiance/(Radiometry::pi*std::pow(radiusKm/auKm,2));
    s.irradiance=Radiometry::unitSolarIrradiance*std::pow(auKm/distanceKm,2);
    return s;
}
inline double kernelSigma(double projectedRadius,double opticalSigma) {
    // Per-axis variance of the u=0.6 limb-darkened photosphere is 0.225 R^2.
    return std::hypot(opticalSigma,std::sqrt(.225)*projectedRadius);
}
}
