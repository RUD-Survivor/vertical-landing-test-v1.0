#include "render/solar_optics.h"
#include <cassert>
#include <cstdio>
#include <limits>
int main() {
    const double radius=696340,au=149597870.7;
    auto reference=SolarOptics::source(au,radius);
    assert(reference.valid);
    assert(std::abs(reference.angularRadius*360/Radiometry::pi-.533394)<.002);
    for(double factor:{.1,.25,.5,1.,2.,10.,100.,10000.}) {
        auto sun=SolarOptics::source(au*factor,radius);
        assert(sun.valid&&std::isfinite(sun.radiance)&&sun.irradiance>0);
        assert(std::abs(sun.irradiance*factor*factor/1.1-1)<1e-7);
        assert(sun.radiance==reference.radiance);
        assert(std::abs(std::sin(sun.angularRadius)-radius/(au*factor))<1e-12);
        assert(std::abs(sun.radiance*Radiometry::pi*sun.sinRadius*sun.sinRadius/sun.irradiance-1)<1e-12);
        printf("PASS SOLAR distance=%.4g AU diameter=%.9g deg irradiance=%.9g radiance=%.9g\n",factor,sun.angularRadius*360/Radiometry::pi,sun.irradiance,sun.radiance);
    }
    for(double distance:{0.,-1.,radius,radius*.5,std::numeric_limits<double>::infinity(),std::numeric_limits<double>::quiet_NaN()})
        assert(!SolarOptics::source(distance,radius).valid);
    assert(!SolarOptics::source(au,0).valid);
    auto near=SolarOptics::source(radius*(1+1e-9),radius);assert(near.valid&&std::isfinite(near.solidAngle));
    double integral=0;const int n=100000;
    for(int i=0;i<n;++i){double r=(i+.5)/n;integral+=2*r*(.4+.6*std::sqrt(1-r*r))/.8/n;}
    assert(std::abs(integral-1)<1e-7);
    assert(SolarOptics::scatteredFraction>0&&SolarOptics::scatteredFraction<1);
    double budget=SolarOptics::starFraction,redBudget=SolarOptics::starFraction;
    for(const auto& kernel:SolarOptics::kernels){
        assert(kernel.sigmaX>0&&kernel.sigmaY>0&&kernel.fraction>0);
        budget+=kernel.fraction;redBudget+=kernel.fraction*kernel.r;
        double retained=std::erf(4/std::sqrt(2.));
        assert(retained*retained>.9998);
    }
    for(const auto& ghost:SolarOptics::ghosts){assert(ghost.radius>0&&ghost.fraction>0);budget+=ghost.fraction;redBudget+=ghost.fraction*ghost.r;}
    // Positive additions: scale the roundoff bound with the energy budget.
    assert(std::abs(budget/SolarOptics::scatteredFraction-1)<16*std::numeric_limits<double>::epsilon());
    double redReference=SolarOptics::starFraction;
    for(const auto& kernel:SolarOptics::kernels)redReference+=kernel.fraction*kernel.r;
    for(const auto& ghost:SolarOptics::ghosts)redReference+=ghost.fraction*ghost.r;
    // Tints are stored as shader-compatible float, fractions as double.
    assert(std::abs(redBudget/redReference-1)<4*std::numeric_limits<float>::epsilon());
    assert(SolarOptics::starRays(7)==14&&SolarOptics::starRays(8)==8&&SolarOptics::starRays(9)==18);
    assert(SolarOptics::starRays(0)==0&&SolarOptics::starRays(17)==0);
    assert(Radiometry::aces(8)<Radiometry::aces(16)&&Radiometry::aces(16)<Radiometry::aces(32));
    double sigma=SolarOptics::kernelSigma(.04,.006);
    assert(std::abs(sigma*sigma-(.04*.04*.225+.006*.006))<1e-15);
    printf("PASS SOLAR limb integral=%.12g; optical energy budget=%.9g; invalid/near-surface boundaries\n",integral,SolarOptics::scatteredFraction);
    // Export current coefficients for the independent FP16 rounding model.
    FILE* profile=std::fopen("build/tests/solar-camera-profile.json","w");assert(profile);
    std::fprintf(profile,"{\"scattered_fraction\":%.17g,\"star_fraction\":%.17g,\"kernels\":[",SolarOptics::scatteredFraction,SolarOptics::starFraction);
    bool first=true;for(auto k:SolarOptics::kernels){std::fprintf(profile,"%s[%.17g,%.17g,%.17g,%.9g]",first?"":",",k.sigmaX,k.sigmaY,k.fraction,k.r);first=false;}
    std::fprintf(profile,"],\"ghosts\":[");first=true;
    for(auto g:SolarOptics::ghosts){std::fprintf(profile,"%s[%.17g,%.17g,%.9g]",first?"":",",g.radius,g.fraction,g.r);first=false;}
    std::fprintf(profile,"]}\n");std::fclose(profile);
}
