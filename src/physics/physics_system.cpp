#include "physics_system.h"
#include <algorithm>
#include <iostream>

std::vector<CelestialBody> SOLAR_SYSTEM;
int current_soi_index = 3; // Default to Earth

namespace PhysicsSystem {

// Constants for Ephemeris calculation (J2000 epoch: 2000-01-01 12:00:00)
// Julian Century = 36525 days. For simplicity we'll convert simulation seconds directly to Julian Centuries from J2000.
constexpr double SECONDS_PER_JULIAN_CENTURY = 36525.0 * 24.0 * 3600.0;

void InitSolarSystem() {
    SOLAR_SYSTEM.clear();
    
    double R_earth = 6371000.0;
    
    // 0: SUN
    CelestialBody sun;
    sun.name = "Sun";
    sun.mass = 1.989e30;
    sun.radius = 696340000.0;
    sun.type = STAR;
    sun.r = 1.0; sun.g = 0.9; sun.b = 0.8;
    sun.axial_tilt = 7.25 * PI / 180.0;
    sun.rotation_period = 25.38 * 24.0 * 3600.0;
    sun.prime_meridian_epoch = 0.0;
    sun.sma_base = 0.0; sun.sma_rate = 0.0;
    sun.ecc_base = 0.0; sun.ecc_rate = 0.0;
    sun.inc_base = 0.0; sun.inc_rate = 0.0;
    sun.lan_base = 0.0; sun.lan_rate = 0.0;
    sun.arg_peri_base = 0.0; sun.arg_peri_rate = 0.0;
    sun.mean_anom_base = 0.0; sun.mean_anom_rate = 0.0;
    SOLAR_SYSTEM.push_back(sun);

    // 1: MERCURY
    CelestialBody mercury;
    mercury.name = "Mercury";
    mercury.mass = 3.3011e23;
    mercury.radius = 2439700.0;
    mercury.type = TERRESTRIAL;
    mercury.r = 0.5; mercury.g = 0.5; mercury.b = 0.5;
    mercury.axial_tilt = 0.034 * PI / 180.0;
    mercury.rotation_period = 58.646 * 24.0 * 3600.0;
    mercury.prime_meridian_epoch = 0.0;
    mercury.sma_base = 0.38709927 * au_meters; mercury.sma_rate = 0.00000037 * au_meters;
    mercury.ecc_base = 0.20563593; mercury.ecc_rate = 0.00001906;
    mercury.inc_base = 7.00497902 * PI / 180.0; mercury.inc_rate = -0.00594749 * PI / 180.0;
    mercury.lan_base = 48.33076593 * PI / 180.0; mercury.lan_rate = -0.12534081 * PI / 180.0;
    mercury.arg_peri_base = 29.1241 * PI / 180.0; mercury.arg_peri_rate = 0.0;
    mercury.mean_anom_base = 174.796 * PI / 180.0; mercury.mean_anom_rate = (360.0 / 87.969) * PI / 180.0 / (24.0 * 3600.0);
    SOLAR_SYSTEM.push_back(mercury);
    
    // 2: VENUS
    CelestialBody venus;
    venus.name = "Venus";
    venus.mass = 4.8675e24;
    venus.radius = 6051800.0;
    venus.type = TERRESTRIAL;
    venus.r = 0.9; venus.g = 0.8; venus.b = 0.6;
    venus.axial_tilt = 177.36 * PI / 180.0;
    venus.rotation_period = -243.025 * 24.0 * 3600.0;
    venus.prime_meridian_epoch = 0.0;
    venus.sma_base = 0.72333199 * au_meters; venus.sma_rate = 0.00000039 * au_meters;
    venus.ecc_base = 0.00677323; venus.ecc_rate = -0.00004107;
    venus.inc_base = 3.39467605 * PI / 180.0; venus.inc_rate = -0.00078890 * PI / 180.0;
    venus.lan_base = 76.67984255 * PI / 180.0; venus.lan_rate = -0.27769418 * PI / 180.0;
    venus.arg_peri_base = 54.884 * PI / 180.0; venus.arg_peri_rate = 0.0;
    venus.mean_anom_base = 50.115 * PI / 180.0; venus.mean_anom_rate = (360.0 / 224.701) * PI / 180.0 / (24.0 * 3600.0);
    SOLAR_SYSTEM.push_back(venus);

    // 3: EARTH
    CelestialBody earth;
    earth.name = "Earth";
    earth.mass = 5.972e24;
    earth.radius = EARTH_RADIUS;
    earth.type = TERRESTRIAL;
    earth.r = 0.2; earth.g = 0.5; earth.b = 1.0;
    earth.axial_tilt = 23.439 * PI / 180.0;
    earth.rotation_period = 0.99726968 * 24.0 * 3600.0; // Sidereal day
    earth.prime_meridian_epoch = 0.0;
    earth.sma_base = 1.00000261 * au_meters; earth.sma_rate = 0.00000562 * au_meters;
    earth.ecc_base = 0.01671123; earth.ecc_rate = -0.00004392;
    earth.inc_base = -0.00001531 * PI / 180.0; earth.inc_rate = -0.01294668 * PI / 180.0;
    earth.lan_base = 0.0; earth.lan_rate = 0.0;
    earth.arg_peri_base = 114.20783 * PI / 180.0; earth.arg_peri_rate = 0.0;
    earth.mean_anom_base = 358.617 * PI / 180.0; earth.mean_anom_rate = (360.0 / 365.256) * PI / 180.0 / (24.0 * 3600.0);
    SOLAR_SYSTEM.push_back(earth);
    
    // 4: MOON (Luna)
    // Note: The moon's orbit is geocentric, but for VSOP87-like ephemeris in our N-body 
    // heliocentric solver, we will treat it as a body that orbits the Sun but stays close to Earth.
    // For a video-game approximation, we will update the Moon's position as Earth's position + lunar orbit
    // We'll give it a heliocentric approximation. To do this perfectly we'll just code its update differently
    // in UpdateCelestialBodies.
    CelestialBody moon;
    moon.name = "Moon";
    moon.mass = 7.342e22;
    moon.radius = 1737400.0;
    moon.type = MOON;
    moon.r = 0.7; moon.g = 0.7; moon.b = 0.7;
    moon.axial_tilt = 1.5424 * PI / 180.0;
    moon.rotation_period = 27.321 * 24.0 * 3600.0;
    moon.prime_meridian_epoch = 0.0;
    // We store Earth-relative elements in the moon's base fields
    moon.sma_base = 384400000.0; moon.sma_rate = 0.0;
    moon.ecc_base = 0.0549; moon.ecc_rate = 0.0;
    moon.inc_base = 5.145 * PI / 180.0; moon.inc_rate = 0.0;
    moon.lan_base = 125.08 * PI / 180.0; moon.lan_rate = -19.34 * PI / 180.0; // precesses 19.34 deg/year. Let's make it simpler
    moon.arg_peri_base = 318.15 * PI / 180.0; moon.arg_peri_rate = 0.0;
    moon.mean_anom_base = 115.3654 * PI / 180.0; moon.mean_anom_rate = (360.0 / 27.321) * PI / 180.0 / (24.0 * 3600.0);
    SOLAR_SYSTEM.push_back(moon);
    
    // 5: MARS
    CelestialBody mars;
    mars.name = "Mars";
    mars.mass = 6.4171e23;
    mars.radius = 3389500.0;
    mars.type = TERRESTRIAL;
    mars.r = 0.8; mars.g = 0.4; mars.b = 0.2;
    mars.axial_tilt = 25.19 * PI / 180.0;
    mars.rotation_period = 1.02595 * 24.0 * 3600.0;
    mars.prime_meridian_epoch = 0.0;
    mars.sma_base = 1.523679 * au_meters; mars.sma_rate = 0.0;
    mars.ecc_base = 0.0934006; mars.ecc_rate = 0.0000904;
    mars.inc_base = 1.8497 * PI / 180.0; mars.inc_rate = -0.0081 * PI / 180.0;
    mars.lan_base = 49.558 * PI / 180.0; mars.lan_rate = -0.294 * PI / 180.0;
    mars.arg_peri_base = 286.502 * PI / 180.0; mars.arg_peri_rate = 0.0;
    mars.mean_anom_base = 19.387 * PI / 180.0; mars.mean_anom_rate = (360.0 / 686.980) * PI / 180.0 / (24.0 * 3600.0);
    SOLAR_SYSTEM.push_back(mars);
    
    // 6: JUPITER
    CelestialBody jupiter;
    jupiter.name = "Jupiter";
    jupiter.mass = 1.8982e27;
    jupiter.radius = 69911000.0;
    jupiter.type = GAS_GIANT;
    jupiter.r = 0.8; jupiter.g = 0.7; jupiter.b = 0.6;
    jupiter.axial_tilt = 3.13 * PI / 180.0;
    jupiter.rotation_period = 0.41354 * 24.0 * 3600.0;
    jupiter.prime_meridian_epoch = 0.0;
    jupiter.sma_base = 5.2044 * au_meters; jupiter.sma_rate = 0.0;
    jupiter.ecc_base = 0.048498; jupiter.ecc_rate = -0.00016;
    jupiter.inc_base = 1.303 * PI / 180.0; jupiter.inc_rate = -0.003 * PI / 180.0;
    jupiter.lan_base = 100.46 * PI / 180.0; jupiter.lan_rate = 0.17 * PI / 180.0;
    jupiter.arg_peri_base = 273.867 * PI / 180.0; jupiter.arg_peri_rate = 0.0;
    jupiter.mean_anom_base = 20.02 * PI / 180.0; jupiter.mean_anom_rate = (360.0 / 4332.589) * PI / 180.0 / (24.0 * 3600.0);
    SOLAR_SYSTEM.push_back(jupiter);
    
    // 7: SATURN
    CelestialBody saturn;
    saturn.name = "Saturn";
    saturn.mass = 5.6834e26;
    saturn.radius = 58232000.0;
    saturn.type = RINGED_GAS_GIANT;
    saturn.r = 0.9; saturn.g = 0.8; saturn.b = 0.5;
    saturn.axial_tilt = 26.73 * PI / 180.0;
    saturn.rotation_period = 0.444 * 24.0 * 3600.0;
    saturn.prime_meridian_epoch = 0.0;
    saturn.sma_base = 9.5826 * au_meters; saturn.sma_rate = 0.0;
    saturn.ecc_base = 0.05555; saturn.ecc_rate = -0.00034;
    saturn.inc_base = 2.484 * PI / 180.0; saturn.inc_rate = 0.006 * PI / 180.0;
    saturn.lan_base = 113.66 * PI / 180.0; saturn.lan_rate = -0.288 * PI / 180.0;
    saturn.arg_peri_base = 339.39 * PI / 180.0; saturn.arg_peri_rate = 0.0;
    saturn.mean_anom_base = 317.02 * PI / 180.0; saturn.mean_anom_rate = (360.0 / 10759.22) * PI / 180.0 / (24.0 * 3600.0);
    SOLAR_SYSTEM.push_back(saturn);
    
    // 8: URANUS
    CelestialBody uranus;
    uranus.name = "Uranus";
    uranus.mass = 8.681e25;
    uranus.radius = 25362000.0;
    uranus.type = GAS_GIANT;
    uranus.r = 0.6; uranus.g = 0.8; uranus.b = 0.9;
    uranus.axial_tilt = 97.77 * PI / 180.0;
    uranus.rotation_period = -0.718 * 24.0 * 3600.0;
    uranus.prime_meridian_epoch = 0.0;
    uranus.sma_base = 19.201 * au_meters; uranus.sma_rate = 0.0;
    uranus.ecc_base = 0.046381; uranus.ecc_rate = -0.000027;
    uranus.inc_base = 0.7725 * PI / 180.0; uranus.inc_rate = -0.002 * PI / 180.0;
    uranus.lan_base = 74.0 * PI / 180.0; uranus.lan_rate = 0.08 * PI / 180.0;
    uranus.arg_peri_base = 96.66 * PI / 180.0; uranus.arg_peri_rate = 0.0;
    uranus.mean_anom_base = 142.59 * PI / 180.0; uranus.mean_anom_rate = (360.0 / 30685.4) * PI / 180.0 / (24.0 * 3600.0);
    SOLAR_SYSTEM.push_back(uranus);
    
    // 9: NEPTUNE
    CelestialBody neptune;
    neptune.name = "Neptune";
    neptune.mass = 1.02413e26;
    neptune.radius = 24622000.0;
    neptune.type = GAS_GIANT;
    neptune.r = 0.2; neptune.g = 0.3; neptune.b = 0.8;
    neptune.axial_tilt = 28.32 * PI / 180.0;
    neptune.rotation_period = 0.671 * 24.0 * 3600.0;
    neptune.prime_meridian_epoch = 0.0;
    neptune.sma_base = 30.047 * au_meters; neptune.sma_rate = 0.0;
    neptune.ecc_base = 0.009456; neptune.ecc_rate = 0.000006;
    neptune.inc_base = 1.769 * PI / 180.0; neptune.inc_rate = -0.002 * PI / 180.0;
    neptune.lan_base = 131.78 * PI / 180.0; neptune.lan_rate = -0.006 * PI / 180.0;
    neptune.arg_peri_base = 273.187 * PI / 180.0; neptune.arg_peri_rate = 0.0;
    neptune.mean_anom_base = 256.228 * PI / 180.0; neptune.mean_anom_rate = (360.0 / 60189.0) * PI / 180.0 / (24.0 * 3600.0);
    SOLAR_SYSTEM.push_back(neptune);

    // Compute SOI for all bodies (Laplace Sphere of Influence)
    for (size_t i = 1; i < SOLAR_SYSTEM.size(); i++) {
        if (i == 4) { // MOON special case
            SOLAR_SYSTEM[i].soi_radius = SOLAR_SYSTEM[i].sma_base * std::pow(SOLAR_SYSTEM[i].mass / SOLAR_SYSTEM[3].mass, 2.0/5.0);
        } else {
            SOLAR_SYSTEM[i].soi_radius = SOLAR_SYSTEM[i].sma_base * std::pow(SOLAR_SYSTEM[i].mass / sun.mass, 2.0/5.0);
        }
    }
    SOLAR_SYSTEM[0].soi_radius = INFINITY; // Sun has infinite SOI basically
    
    // Execute a starting position update
    UpdateCelestialBodies(0.0);
}

void UpdateCelestialBodies(double current_time_sec) {
    double T = current_time_sec / SECONDS_PER_JULIAN_CENTURY; // Julian centuries from epoch
    
    // Sun is at center
    SOLAR_SYSTEM[0].px = 0.0; SOLAR_SYSTEM[0].py = 0.0; SOLAR_SYSTEM[0].pz = 0.0;
    SOLAR_SYSTEM[0].vx = 0.0; SOLAR_SYSTEM[0].vy = 0.0; SOLAR_SYSTEM[0].vz = 0.0;

    for (size_t i = 1; i < SOLAR_SYSTEM.size(); i++) {
        CelestialBody& b = SOLAR_SYSTEM[i];
        
        // Linear secular perturbation update
        double a = b.sma_base + b.sma_rate * T;
        double e = b.ecc_base + b.ecc_rate * T;
        double i_inc = b.inc_base + b.inc_rate * T;
        double lan = b.lan_base + b.lan_rate * T;
        double arg_p = b.arg_peri_base + b.arg_peri_rate * T;
        double M = b.mean_anom_base + b.mean_anom_rate * current_time_sec;
        
        // Solve Kepler's Equation for Eccentric Anomaly (E)
        M = std::fmod(M, 2.0 * PI);
        if (M < 0) M += 2.0 * PI;
        double E = M;
        for (int k = 0; k < 5; k++) {
            double dE = (E - e * std::sin(E) - M) / (1.0 - e * std::cos(E));
            E -= dE;
            if (std::abs(dE) < 1e-6) break;
        }
        
        // True Anomaly (nu)
        double nu = 2.0 * std::atan2(std::sqrt(1.0 + e) * std::sin(E / 2.0), std::sqrt(1.0 - e) * std::cos(E / 2.0));
        
        // Distance
        double r = a * (1.0 - e * std::cos(E));
        
        // Orbital plane coordinates
        double o_x = r * std::cos(nu);
        double o_y = r * std::sin(nu);
        
        // Orbital plane velocities
        double mu = (i == 4) ? (G_const * SOLAR_SYSTEM[3].mass) : GM_sun; // Moon orbits Earth
        double p = a * (1.0 - e*e); // semi-latus rectum
        double h = std::sqrt(mu * p);
        double o_vx = -(mu / h) * std::sin(nu);
        double o_vy = (mu / h) * (e + std::cos(nu));
        
        // Transform to 3D Space (Euler angles: LAN, Inclination, Arg of Periapsis)
        // Cos and Sin values
        double c_O = std::cos(lan), s_O = std::sin(lan);
        double c_w = std::cos(arg_p), s_w = std::sin(arg_p);
        double c_i = std::cos(i_inc), s_i = std::sin(i_inc);
        
        // Transformation Matrix
        double x_x = c_O * c_w - s_O * s_w * c_i;
        double x_y = -c_O * s_w - s_O * c_w * c_i;
        double y_x = s_O * c_w + c_O * s_w * c_i;
        double y_y = -s_O * s_w + c_O * c_w * c_i;
        double z_x = s_w * s_i;
        double z_y = c_w * s_i;
        
        // Heliocentric Position
        b.px = x_x * o_x + x_y * o_y;
        b.py = y_x * o_x + y_y * o_y;
        b.pz = z_x * o_x + z_y * o_y;
        
        // Heliocentric Velocity
        b.vx = x_x * o_vx + x_y * o_vy;
        b.vy = y_x * o_vx + y_y * o_vy;
        b.vz = z_x * o_vx + z_y * o_vy;
        
        // Special Case: Moon is Geocentric in our orbital calculation, move it to Heliocentric
        if (i == 4) { // MOON
            b.px += SOLAR_SYSTEM[3].px;
            b.py += SOLAR_SYSTEM[3].py;
            b.pz += SOLAR_SYSTEM[3].pz;
            b.vx += SOLAR_SYSTEM[3].vx;
            b.vy += SOLAR_SYSTEM[3].vy;
            b.vz += SOLAR_SYSTEM[3].vz;
        }
    }
}


void CheckSOI_Transitions(RocketState& state) {
    if (SOLAR_SYSTEM.empty()) return;

    // Rocket's current heliocentric position
    CelestialBody& current_body = SOLAR_SYSTEM[current_soi_index];
    // Update absolute coordinates
    state.abs_px = current_body.px + state.px;
    state.abs_py = current_body.py + state.py;
    state.abs_pz = current_body.pz + state.pz;
    
    // Check if we entered any body's SOI
    int best_soi = 0; // Default to Sun
    for (size_t i = 1; i < SOLAR_SYSTEM.size(); i++) {
        CelestialBody& b = SOLAR_SYSTEM[i];
        double dx = state.abs_px - b.px;
        double dy = state.abs_py - b.py;
        double dz = state.abs_pz - b.pz;
        double dist = std::sqrt(dx*dx + dy*dy + dz*dz);
        if (dist < b.soi_radius) {
            best_soi = i;
        }
    }
    
    if (best_soi != current_soi_index) {
        CelestialBody& new_body = SOLAR_SYSTEM[best_soi];
        double h_vx = current_body.vx + state.vx;
        double h_vy = current_body.vy + state.vy;
        double h_vz = current_body.vz + state.vz;
        
        state.px = state.abs_px - new_body.px;
        state.py = state.abs_py - new_body.py;
        state.pz = state.abs_pz - new_body.pz;
        state.vx = h_vx - new_body.vx;
        state.vy = h_vy - new_body.vy;
        state.vz = h_vz - new_body.vz;
        
        current_soi_index = best_soi;
        std::cout << ">> [SOI TRANSITION] ENTERED " << new_body.name << " SOI" << std::endl;
    }
}

double CalculateSolarOcclusion(const RocketState& state) {
    if (SOLAR_SYSTEM.empty()) return 1.0;
    
    double min_occlusion = 1.0;
    
    // Ray from rocket to Sun (Sun is at 0,0,0)
    double dx = -state.abs_px, dy = -state.abs_py, dz = -state.abs_pz;
    double dist_to_sun = std::sqrt(dx*dx + dy*dy + dz*dz);
    double dir_x = dx / dist_to_sun;
    double dir_y = dy / dist_to_sun;
    double dir_z = dz / dist_to_sun;
    
    for (size_t i = 1; i < SOLAR_SYSTEM.size(); i++) {
        CelestialBody& b = SOLAR_SYSTEM[i];
        
        double bx = b.px - state.abs_px;
        double by = b.py - state.abs_py;
        double bz = b.pz - state.abs_pz;
        
        double proj = bx * dir_x + by * dir_y + bz * dir_z;
        if (proj > 0 && proj < dist_to_sun) {
            double px = bx - proj * dir_x;
            double py = by - proj * dir_y;
            double pz = bz - proj * dir_z;
            double pass_dist = std::sqrt(px*px + py*py + pz*pz);
            
            if (pass_dist < b.radius * 1.05) { 
                double occ = (pass_dist - b.radius) / (b.radius * 0.05);
                occ = std::max(0.0, std::min(1.0, occ));
                min_occlusion = std::min(min_occlusion, occ);
            }
        }
    }
    return min_occlusion;
}

double get_gravity(double r) { 
    return G0 * pow(EARTH_RADIUS / r, 2); 
}

double get_pressure(double h) {
    if (h > 100000) return 0;
    if (h < 0) return SLP;
    return SLP * std::exp(-h / 7000.0); // Simple exponential atmosphere
}

double get_air_density(double h) {
    if (h > 100000) return 0;
    return 1.225 * std::exp(-h / 7000.0);
}

void getOrbitParams(const RocketState& state, double& apoapsis, double& periapsis) {
    if (SOLAR_SYSTEM.empty()) return;
    CelestialBody& current_body = SOLAR_SYSTEM[current_soi_index];
    
    double r = std::sqrt(state.px * state.px + state.py * state.py + state.pz * state.pz);
    double v_sq = state.vx * state.vx + state.vy * state.vy + state.vz * state.vz;
    double mu = G_const * current_body.mass; // Standard gravitational parameter

    double energy = v_sq / 2.0 - mu / r; // Specific orbital energy
    
    // 3D Specific Angular Momentum: h = r x v
    double hx = state.py * state.vz - state.pz * state.vy;
    double hy = state.pz * state.vx - state.px * state.vz;
    double hz = state.px * state.vy - state.py * state.vx;
    double h_sq = hx * hx + hy * hy + hz * hz;

    // Squared eccentricity
    double e_sq = 1.0 + 2.0 * energy * h_sq / (mu * mu);
    double e = (e_sq > 0) ? std::sqrt(e_sq) : 0;

    if (energy >= 0) { // Escape orbit
        apoapsis = 999999999;
        periapsis = (h_sq / mu) / (1.0 + e) - current_body.radius; 
    } else {           // Closed elliptical orbit
        double a = -mu / (2.0 * energy); // Semi-major axis
        apoapsis = a * (1.0 + e) - current_body.radius;
        periapsis = a * (1.0 - e) - current_body.radius;
    }
}

void Update(RocketState& state, const RocketConfig& config, const ControlInput& input, double dt) {
    if (SOLAR_SYSTEM.empty()) return;
    CelestialBody& current_body_launch = SOLAR_SYSTEM[current_soi_index];

    if (state.status == PRE_LAUNCH) {
        state.px = 0;
        state.py = current_body_launch.radius;
        state.pz = 0;
        
        // Planet rotates around local Z axis (Equator = XY plane)
        double omega = (2.0 * PI) / current_body_launch.rotation_period;
        state.vx = -omega * current_body_launch.radius;
        state.vy = 0;
        state.vz = 0;
        
        state.altitude = 0;
        return;
    }
    if (state.status == LANDED || state.status == CRASHED)
        return;

    // A. Base State Calculation
    CelestialBody& current_body = SOLAR_SYSTEM[current_soi_index];
    double r = std::sqrt(state.px * state.px + state.py * state.py); // Horizontal distance for derived info
    double r_3d = std::sqrt(state.px * state.px + state.py * state.py + state.pz * state.pz);
    state.altitude = r_3d - current_body.radius;
    
    // Update SOI transitions and Absolute Positions
    CheckSOI_Transitions(state);
    
    // Update Solar Occlusion
    state.solar_occlusion = CalculateSolarOcclusion(state);

    // B. Force Analysis
    double total_mass = config.dry_mass + state.fuel;

    // We will do Gravity inside calc_accel directly now
    state.sim_time += dt;

    // 2. Thrust
    state.thrust_power = 0;
    if (state.fuel > 0) {
        double max_thrust = config.specific_impulse * G0 * config.cosrate;
        double pressure_loss = get_pressure(state.altitude) * config.nozzle_area;
        double current_thrust = input.throttle * max_thrust - pressure_loss;
        if (current_thrust < 0) current_thrust = 0;
        state.thrust_power = current_thrust;

        double m_dot = state.thrust_power / (config.specific_impulse * G0);
        state.fuel -= m_dot * dt;
        state.fuel_consumption_rate = m_dot; 
    } else {
        state.thrust_power = 0;
        state.fuel_consumption_rate = 0;
    }

    // --- 3D Geometric Frame and Thrust Projection ---
    double r_mag = std::sqrt(state.px*state.px + state.py*state.py + state.pz*state.pz);
    double Ux = state.px / r_mag;
    double Uy = state.py / r_mag;
    double Uz = state.pz / r_mag;

    double r_xy_mag = std::sqrt(Ux*Ux + Uy*Uy);
    double Rx = 0, Ry = 0, Rz = 0;
    if (r_xy_mag > 1e-6) {
        Rx = -Uy / r_xy_mag;
        Ry = Ux / r_xy_mag;
        Rz = 0.0;
    } else {
        Rx = 1.0; Ry = 0.0; Rz = 0.0;
    }

    double Nx = Uy * Rz - Uz * Ry;
    double Ny = Uz * Rx - Ux * Rz;
    double Nz = Ux * Ry - Uy * Rx;
    double N_mag = std::sqrt(Nx*Nx + Ny*Ny + Nz*Nz);
    Nx /= N_mag; Ny /= N_mag; Nz /= N_mag;

    double cos_a = std::cos(state.angle);
    double sin_a = std::sin(state.angle);
    double Fx = Ux * cos_a + Rx * sin_a;
    double Fy = Uy * cos_a + Ry * sin_a;
    double Fz = Uz * cos_a + Rz * sin_a;

    double cos_z = std::cos(state.angle_z);
    double sin_z = std::sin(state.angle_z);
    
    double thrust_dir_x = Fx * cos_z + Nx * sin_z;
    double thrust_dir_y = Fy * cos_z + Ny * sin_z;
    double thrust_dir_z = Fz * cos_z + Nz * sin_z;

    double Ft_x = state.thrust_power * thrust_dir_x;
    double Ft_y = state.thrust_power * thrust_dir_y;
    double Ft_z = state.thrust_power * thrust_dir_z;

    // 3. Aerodynamic Drag & Torque
    double v_sq = state.vx * state.vx + state.vy * state.vy + state.vz * state.vz;
    double v_mag = std::sqrt(v_sq);
    double local_up_angle = std::atan2(state.py, state.px);
    double Fd_x = 0, Fd_y = 0;
    double aero_torque = 0;

    if (v_mag > 0.1 && state.altitude < 80000) {
        double rho = get_air_density(state.altitude);

        double base_area = 10.0;
        double side_area = config.height * config.diameter; 
        double effective_area = base_area + side_area * std::abs(std::sin(state.angle_z));

        // Drag (F = 0.5 * rho * v^2 * Cd * A), assuming Cd=0.5
        double drag_mag = 0.5 * rho * v_sq * 0.5 * effective_area;
        Fd_x = -drag_mag * (state.vx / v_mag);
        Fd_y = -drag_mag * (state.vy / v_mag);

        // Structural limits check
        double dynamic_pressure = 0.5 * rho * v_sq;
        if (dynamic_pressure > 50000.0 && std::abs(state.angle_z) > 0.35) { // ~20 degrees
            state.status = CRASHED;
            state.mission_msg = ">> STRUCTURAL FAILURE: HIGH Q OUT-OF-PLANE PITCH!";
            state.vx = 0; state.vy = 0; 
            state.ang_vel = 0; state.ang_vel_z = 0;
            return;
        }

        // Aerodynamic Torques
        aero_torque -= state.ang_vel * 0.1 * v_mag * rho;
    }

    // C. Integration (RK4)
    auto calc_accel = [&](double temp_px, double temp_py, double temp_pz, double temp_vx, double temp_vy, double temp_vz, double& out_ax, double& out_ay, double& out_az) {
        double r_inner_sq = temp_px * temp_px + temp_py * temp_py + temp_pz * temp_pz;
        double r_inner = std::sqrt(r_inner_sq);
        double r3_inner = r_inner_sq * r_inner;
        
        // 1. Direct gravity from current SOI body
        double Fgx = 0, Fgy = 0, Fgz = 0;
        if (r3_inner > 0) {
            double GM = G_const * current_body.mass;
            Fgx = -GM * temp_px / r3_inner * total_mass;
            Fgy = -GM * temp_py / r3_inner * total_mass;
            Fgz = -GM * temp_pz / r3_inner * total_mass;
        }
        
        // 2. N-Body Perturbation Gravity
        for (size_t i = 0; i < SOLAR_SYSTEM.size(); i++) {
            if (i == current_soi_index) continue; // Handled as primary
            CelestialBody& body = SOLAR_SYSTEM[i];
            
            double dx_body = body.px - current_body.px;
            double dy_body = body.py - current_body.py;
            double dz_body = body.pz - current_body.pz;
            
            double rdx = dx_body - temp_px;
            double rdy = dy_body - temp_py;
            double rdz = dz_body - temp_pz;
            double r_rocket_sq = rdx*rdx + rdy*rdy + rdz*rdz;
            double r_rocket3 = r_rocket_sq * std::sqrt(r_rocket_sq);
            
            double dist_body_sq = dx_body*dx_body + dy_body*dy_body + dz_body*dz_body;
            double dist_body3 = dist_body_sq * std::sqrt(dist_body_sq);
            
            double GM = G_const * body.mass;
            Fgx += GM * (rdx / r_rocket3 - dx_body / dist_body3) * total_mass;
            Fgy += GM * (rdy / r_rocket3 - dy_body / dist_body3) * total_mass;
            Fgz += GM * (rdz / r_rocket3 - dz_body / dist_body3) * total_mass;
        }
        
        // Air Drag
        double Fdx = 0, Fdy = 0, Fdz = 0;
        double temp_v_sq = temp_vx * temp_vx + temp_vy * temp_vy + temp_vz * temp_vz;
        double temp_v_mag = std::sqrt(temp_v_sq);
        double alt = r_inner - current_body.radius;
        if (temp_v_mag > 0.1 && alt < 80000) {
            double rho = get_air_density(alt);
            double base_area = 10.0;
            double side_area = config.height * config.diameter;
            double effective_area = base_area + side_area * std::abs(std::sin(state.angle_z));
            double drag_mag = 0.5 * rho * temp_v_sq * 0.5 * effective_area;
            Fdx = -drag_mag * (temp_vx / temp_v_mag);
            Fdy = -drag_mag * (temp_vy / temp_v_mag);
            Fdz = -drag_mag * (temp_vz / temp_v_mag);
        }
        
        out_ax = (Fgx + Ft_x + Fdx) / total_mass;
        out_ay = (Fgy + Ft_y + Fdy) / total_mass;
        out_az = (Fgz + Ft_z + Fdz) / total_mass;
    };

    double k1_vx, k1_vy, k1_vz, k1_px, k1_py, k1_pz;
    calc_accel(state.px, state.py, state.pz, state.vx, state.vy, state.vz, k1_vx, k1_vy, k1_vz);
    k1_px = state.vx; k1_py = state.vy; k1_pz = state.vz;

    double k2_vx, k2_vy, k2_vz, k2_px, k2_py, k2_pz;
    calc_accel(state.px + 0.5 * dt * k1_px, state.py + 0.5 * dt * k1_py, state.pz + 0.5 * dt * k1_pz, 
               state.vx + 0.5 * dt * k1_vx, state.vy + 0.5 * dt * k1_vy, state.vz + 0.5 * dt * k1_vz, 
               k2_vx, k2_vy, k2_vz);
    k2_px = state.vx + 0.5 * dt * k1_vx; k2_py = state.vy + 0.5 * dt * k1_vy; k2_pz = state.vz + 0.5 * dt * k1_vz;

    double k3_vx, k3_vy, k3_vz, k3_px, k3_py, k3_pz;
    calc_accel(state.px + 0.5 * dt * k2_px, state.py + 0.5 * dt * k2_py, state.pz + 0.5 * dt * k2_pz, 
               state.vx + 0.5 * dt * k2_vx, state.vy + 0.5 * dt * k2_vy, state.vz + 0.5 * dt * k2_vz, 
               k3_vx, k3_vy, k3_vz);
    k3_px = state.vx + 0.5 * dt * k2_vx; k3_py = state.vy + 0.5 * dt * k2_vy; k3_pz = state.vz + 0.5 * dt * k2_vz;

    double k4_vx, k4_vy, k4_vz, k4_px, k4_py, k4_pz;
    calc_accel(state.px + dt * k3_px, state.py + dt * k3_py, state.pz + dt * k3_pz, 
               state.vx + dt * k3_vx, state.vy + dt * k3_vy, state.vz + dt * k3_vz, 
               k4_vx, k4_vy, k4_vz);
    k4_px = state.vx + dt * k3_vx; k4_py = state.vy + dt * k3_vy; k4_pz = state.vz + dt * k3_vz;

    state.vx += (dt / 6.0) * (k1_vx + 2.0 * k2_vx + 2.0 * k3_vx + k4_vx);
    state.vy += (dt / 6.0) * (k1_vy + 2.0 * k2_vy + 2.0 * k3_vy + k4_vy);
    state.vz += (dt / 6.0) * (k1_vz + 2.0 * k2_vz + 2.0 * k3_vz + k4_vz);
    
    state.px += (dt / 6.0) * (k1_px + 2.0 * k2_px + 2.0 * k3_px + k4_px);
    state.py += (dt / 6.0) * (k1_py + 2.0 * k2_py + 2.0 * k3_py + k4_py);
    state.pz += (dt / 6.0) * (k1_pz + 2.0 * k2_pz + 2.0 * k3_pz + k4_pz);
    
    // Update displayed acceleration
    double final_ax, final_ay, final_az;
    calc_accel(state.px, state.py, state.pz, state.vx, state.vy, state.vz, final_ax, final_ay, final_az);
    state.acceleration = std::sqrt(final_ax * final_ax + final_ay * final_ay + final_az * final_az);

    // Derived velocities
    state.velocity = state.vx * std::cos(local_up_angle) + state.vy * std::sin(local_up_angle);
    state.local_vx = -state.vx * std::sin(local_up_angle) + state.vy * std::cos(local_up_angle);

    // E. Collision Detection
    double current_r = std::sqrt(state.px * state.px + state.py * state.py + state.pz * state.pz);
    double current_alt = current_r - current_body.radius;

    if (current_alt <= 0.0) {
        if (state.status == ASCEND) {
            state.px = 0; state.py = current_body.radius;
            state.vx = 0; state.vy = 0;
            state.altitude = 0;
        } else if (state.velocity < 0.1) {
            state.altitude = 0;
            state.px = 0; state.py = current_body.radius;

            if (state.status != PRE_LAUNCH) {
                if (std::abs(state.velocity) > 10 || std::abs(state.local_vx) > 10) {
                    state.status = CRASHED;
                } else {
                    state.status = LANDED;
                }
                state.vx = 0; state.vy = 0;
                state.ang_vel = 0; state.ang_vel_z = 0;
                state.angle = 0; state.angle_z = 0;
                state.suicide_burn_locked = false; 
            }
        } else {
            state.altitude = current_alt;
        }
    } else {
        state.altitude = current_alt;
    }

    // Angular Motion
    double moment_of_inertia = 50000.0;
    
    double final_torque = input.torque_cmd;
    double ang_accel = (final_torque + aero_torque) / moment_of_inertia;
    state.ang_vel += ang_accel * dt;
    state.angle += state.ang_vel * dt;

    // Z Axis Out of plane Pitch
    double rho_z = (state.altitude < 80000) ? get_air_density(state.altitude) : 0.0;
    double aero_torque_z = -state.ang_vel_z * 0.1 * v_mag * rho_z; 
    double ang_accel_z = (input.torque_cmd_z + aero_torque_z) / moment_of_inertia;
    state.ang_vel_z += ang_accel_z * dt;

    if (state.altitude > 80000) {
        state.ang_vel_z *= std::pow(0.95, dt);
    }
    state.angle_z += state.ang_vel_z * dt;

    while (state.angle_z > PI) state.angle_z -= 2 * PI;
    while (state.angle_z < -PI) state.angle_z += 2 * PI;
}

void FastGravityUpdate(RocketState& state, const RocketConfig& config, double dt_total) {
    if (state.status == PRE_LAUNCH || state.status == LANDED || state.status == CRASHED) return;

    // Scale dt_step dynamically to avoid stalling the CPU at extreme warps
    double dt_step = std::max(5.0, dt_total / 200.0); 
    double t_remaining = dt_total;
    
    double total_mass = config.dry_mass + state.fuel;

    while (t_remaining > 0) {
        double dt = std::min(t_remaining, dt_step);
        t_remaining -= dt;
        state.sim_time += dt;
        
        // Ensure celestial bodies and SOI are updated as time passes rapidly
        UpdateCelestialBodies(state.sim_time);
        CheckSOI_Transitions(state);
      
        auto calc_accel = [&](double temp_px, double temp_py, double temp_pz, double temp_time, double& out_ax, double& out_ay, double& out_az) {
            CelestialBody& current_body = SOLAR_SYSTEM[current_soi_index];
            double r_inner_sq = temp_px * temp_px + temp_py * temp_py + temp_pz * temp_pz;
            double r_inner = std::sqrt(r_inner_sq);
            double r3_inner = r_inner_sq * r_inner;
            
            double Fgx = 0, Fgy = 0, Fgz = 0;
            if (r3_inner > 0) {
                double GM = G_const * current_body.mass;
                Fgx = -GM * temp_px / r3_inner * total_mass;
                Fgy = -GM * temp_py / r3_inner * total_mass;
                Fgz = -GM * temp_pz / r3_inner * total_mass;
            }
            
            for (size_t i = 0; i < SOLAR_SYSTEM.size(); i++) {
                if (i == current_soi_index) continue;
                CelestialBody& body = SOLAR_SYSTEM[i];
                
                double dx_body = body.px - current_body.px;
                double dy_body = body.py - current_body.py;
                double dz_body = body.pz - current_body.pz;
                
                double rdx = dx_body - temp_px;
                double rdy = dy_body - temp_py;
                double rdz = dz_body - temp_pz;
                double r_rocket_sq = rdx*rdx + rdy*rdy + rdz*rdz;
                double r_rocket3 = r_rocket_sq * std::sqrt(r_rocket_sq);
                
                double dist_body_sq = dx_body*dx_body + dy_body*dy_body + dz_body*dz_body;
                double dist_body3 = dist_body_sq * std::sqrt(dist_body_sq);
                
                double GM = G_const * body.mass;
                Fgx += GM * (rdx / r_rocket3 - dx_body / dist_body3) * total_mass;
                Fgy += GM * (rdy / r_rocket3 - dy_body / dist_body3) * total_mass;
                Fgz += GM * (rdz / r_rocket3 - dz_body / dist_body3) * total_mass;
            }
            
            out_ax = Fgx / total_mass;
            out_ay = Fgy / total_mass;
            out_az = Fgz / total_mass;
        };

        double k1_vx, k1_vy, k1_vz, k1_px, k1_py, k1_pz;
        calc_accel(state.px, state.py, state.pz, state.sim_time - dt, k1_vx, k1_vy, k1_vz);
        k1_px = state.vx; k1_py = state.vy; k1_pz = state.vz;

        double k2_vx, k2_vy, k2_vz, k2_px, k2_py, k2_pz;
        calc_accel(state.px + 0.5 * dt * k1_px, state.py + 0.5 * dt * k1_py, state.pz + 0.5 * dt * k1_pz, state.sim_time - dt + 0.5 * dt, k2_vx, k2_vy, k2_vz);
        k2_px = state.vx + 0.5 * dt * k1_vx; k2_py = state.vy + 0.5 * dt * k1_vy; k2_pz = state.vz + 0.5 * dt * k1_vz;

        double k3_vx, k3_vy, k3_vz, k3_px, k3_py, k3_pz;
        calc_accel(state.px + 0.5 * dt * k2_px, state.py + 0.5 * dt * k2_py, state.pz + 0.5 * dt * k2_pz, state.sim_time - dt + 0.5 * dt, k3_vx, k3_vy, k3_vz);
        k3_px = state.vx + 0.5 * dt * k2_vx; k3_py = state.vy + 0.5 * dt * k2_vy; k3_pz = state.vz + 0.5 * dt * k2_vz;

        double k4_vx, k4_vy, k4_vz, k4_px, k4_py, k4_pz;
        calc_accel(state.px + dt * k3_px, state.py + dt * k3_py, state.pz + dt * k3_pz, state.sim_time, k4_vx, k4_vy, k4_vz);
        k4_px = state.vx + dt * k3_vx; k4_py = state.vy + dt * k3_vy; k4_pz = state.vz + dt * k3_vz;


        state.vx += (dt / 6.0) * (k1_vx + 2.0 * k2_vx + 2.0 * k3_vx + k4_vx);
        state.vy += (dt / 6.0) * (k1_vy + 2.0 * k2_vy + 2.0 * k3_vy + k4_vy);
        state.vz += (dt / 6.0) * (k1_vz + 2.0 * k2_vz + 2.0 * k3_vz + k4_vz);
        state.px += (dt / 6.0) * (k1_px + 2.0 * k2_px + 2.0 * k3_px + k4_px);
        state.py += (dt / 6.0) * (k1_py + 2.0 * k2_py + 2.0 * k3_py + k4_py);
        state.pz += (dt / 6.0) * (k1_pz + 2.0 * k2_pz + 2.0 * k3_pz + k4_pz);
        
        state.altitude = std::sqrt(state.px*state.px + state.py*state.py + state.pz*state.pz) - EARTH_RADIUS;
        
        if (state.altitude <= 0.0) {
            state.altitude = 0;
            state.status = CRASHED;
            break;
        }
    }
}

void EmitSmoke(RocketState& state, const RocketConfig& config, double dt) {
    if (state.thrust_power < 1000.0) return;
    double local_up = std::atan2(state.py, state.px);
    double nozzle_dir = local_up + state.angle + PI;
    double nozzle_wx = state.px + std::cos(nozzle_dir) * 20.0;
    double nozzle_wy = state.py + std::sin(nozzle_dir) * 20.0;
    
    for (int k = 0; k < 3; k++) {
        SmokeParticle& p = state.smoke[state.smoke_idx % RocketState::MAX_SMOKE];
        float rnd1 = hash11(state.smoke_idx * 1337 + k * 997) - 0.5f;
        float rnd2 = hash11(state.smoke_idx * 7919 + k * 773) - 0.5f;
        p.wx = nozzle_wx + rnd1 * 15.0;
        p.wy = nozzle_wy + rnd2 * 15.0;
        
        double exhaust_speed = 30.0 + hash11(state.smoke_idx * 3571 + k) * 20.0;
        p.vwx = std::cos(nozzle_dir) * exhaust_speed + rnd1 * 10.0;
        p.vwy = std::sin(nozzle_dir) * exhaust_speed + rnd2 * 10.0;
        p.alpha = 0.6f;
        p.size = 10.0f + hash11(state.smoke_idx * 4567 + k) * 8.0f;
        p.life = 1.0f;
        p.active = true;
        state.smoke_idx++;
    }
}

void UpdateSmoke(RocketState& state, double dt) {
    for (int i = 0; i < RocketState::MAX_SMOKE; i++) {
        SmokeParticle& p = state.smoke[i];
        if (!p.active) continue;
        
        p.life -= (float)(dt * 0.25);
        p.alpha = std::min(0.25f, p.life * 0.3f); 
        p.size += (float)(dt * 20.0);

        p.wx += p.vwx * dt;
        p.wy += p.vwy * dt;

        double r = std::sqrt(p.wx * p.wx + p.wy * p.wy);
        if (r < EARTH_RADIUS && r > 0) {
            p.wx = p.wx / r * EARTH_RADIUS;
            p.wy = p.wy / r * EARTH_RADIUS;
            double nx = p.wx / r, ny = p.wy / r;
            double v_radial = p.vwx * nx + p.vwy * ny;
            double v_tang = -p.vwx * ny + p.vwy * nx;
            v_radial = std::abs(v_radial) * 0.3;
            float rnd_dir = (hash11(i * 8731) - 0.5f) * 2.0f;
            v_tang = std::abs(v_tang) * (1.5f + rnd_dir) * (rnd_dir > 0 ? 1.0 : -1.0);
            p.vwx = nx * v_radial - ny * v_tang;
            p.vwy = ny * v_radial + nx * v_tang;
            p.size += 5.0f;
        }

        if (r > 0) {
            p.vwx += (p.wx / r) * dt * 8.0;
            p.vwy += (p.wy / r) * dt * 8.0;
        }
        p.vwx *= (1.0 - dt * 0.8); 
        p.vwy *= (1.0 - dt * 0.8);

        if (p.life <= 0) p.active = false;
    }
}

} // namespace PhysicsSystem
