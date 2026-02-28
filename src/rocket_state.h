#ifndef ROCKET_STATE_H
#define ROCKET_STATE_H

#include <string>
#include <vector>
#include <cmath>

// Constants
constexpr double PI = 3.14159265358979323846;
constexpr double G0 = 9.80665;             // Standard gravity (m/s^2)
constexpr double EARTH_RADIUS = 6371000.0; // Earth radius (m)
constexpr double SLP = 1013.25;            // Sea level pressure (hPa)
const double au_meters = 149597870700.0;
const double G_const = 6.67430e-11;
const double M_sun = 1.989e30;
const double GM_sun = G_const * M_sun;

enum BodyType {
    STAR,
    TERRESTRIAL,
    GAS_GIANT,
    MOON,
    RINGED_GAS_GIANT
};

struct CelestialBody {
    std::string name;
    double mass;
    double radius;
    BodyType type;
    
    // Rendering colors
    float r, g, b;

    // Rotation parameters
    double axial_tilt;                 // Obliquity to orbit (radians)
    double rotation_period;            // Sidereal day (seconds)
    double prime_meridian_epoch;       // Prime meridian angle at epoch (radians)
    
    // Ephemeris elements (VSOP87 / Secular perturbations)
    double sma_base;          // Semi-major axis base (meters)
    double sma_rate;          // Rate of change (meters/century)
    double ecc_base;          // Eccentricity base
    double ecc_rate;          // Rate of change (1/century)
    double inc_base;          // Inclination base (radians)
    double inc_rate;          // Rate of change (rad/century)
    double lan_base;          // Longitude of ascending node base (radians)
    double lan_rate;          // Rate of change (rad/century)
    double arg_peri_base;     // Argument of periapsis base (radians)
    double arg_peri_rate;     // Rate of change (rad/century)
    double mean_anom_base;    // Mean anomaly at epoch (radians)
    double mean_anom_rate;    // Mean motion (rad/sec)
    
    // Dynamic physics state
    double px, py, pz;        // Heliocentric position (meters)
    double vx, vy, vz;        // Heliocentric velocity (meters/sec)
    
    // Pre-calculated SOI
    double soi_radius;        // Sphere of Influence radius (meters)
};

extern std::vector<CelestialBody> SOLAR_SYSTEM;
extern int current_soi_index;


enum MissionState {
    PRE_LAUNCH,
    ASCEND,
    DESCEND,
    LANDED,
    CRASHED
};

// Simple utility function needed by state logic
inline float hash11(int n) {
    n = (n << 13) ^ n;
    int nn = (n * (n * n * 15731 + 789221) + 1376312589) & 0x7fffffff;
    return 1.0f - ((float)nn / 1073741824.0f);
}

// PID Controller Struct
struct PID {
    double kp, ki, kd;
    double integral = 0;
    double prev_error = 0;
    double integral_limit = 50.0;

    double update(double target, double current, double dt) {
        if (dt <= 0.0) return 0.0;
        double error = target - current;
        integral += error * dt;

        if (integral > integral_limit) integral = integral_limit;
        if (integral < -integral_limit) integral = -integral_limit;

        double derivative = (error - prev_error) / dt;
        prev_error = error;
        return kp * error + ki * integral + kd * derivative;
    }

    void reset() {
        integral = 0;
        prev_error = 0;
    }
};

// Smoke Particle Data
struct SmokeParticle {
    double wx, wy;     // World coordinates
    double vwx, vwy;   // World velocity
    float alpha;       // Alpha
    float size;        // Size
    float life;        // Remaining life (0~1)
    bool active;
};

// Static Rocket Configuration (immutable during flight)
struct RocketConfig {
    double dry_mass;
    double diameter;
    double height;
    int stages;
    double specific_impulse;
    double cosrate; // fuel_consumption_rate / mass flow rate parameter
    double nozzle_area;
};

// Control Inputs (actuators driven by Player/AI)
struct ControlInput {
    double throttle = 0.0;    // 0.0 to 1.0
    double torque_cmd = 0.0;  // Z-axis torque command (pitch in 2D plane)
    double torque_cmd_z = 0.0; // X/Y axis torque command (out of plane pitch)
};

// Dynamic Rocket State (updated by physics)
struct RocketState {
    // Basic properties
    double fuel = 0.0;
    
    // Position/Velocity relative to current SOI origin
    double px = 0.0, py = EARTH_RADIUS + 0.1, pz = 0.0;
    double vx = 0.0, vy = 0.0, vz = 0.0;
    
    // Absolute Heliocentric Position (used for continuous global tracking and Eclipse checks)
    double abs_px = 0.0, abs_py = 0.0, abs_pz = 0.0;
    
    // Attitude
    double angle = 0.0;      // Yaw (in 2D plane)
    double ang_vel = 0.0;
    double angle_z = 0.0;    // Out-of-plane pitch
    double ang_vel_z = 0.0;
    
    // Physics simulation metadata
    double sim_time = 0.0;
    double altitude = 0.0;
    double velocity = 0.0;    // Radial velocity (vertical)
    double local_vx = 0.0;    // Tangential velocity (horizontal against terrain)
    
    // Engine states
    double fuel_consumption_rate = 0.0;
    double thrust_power = 0.0;
    double acceleration = 0.0;
    
    // Environment
    double solar_occlusion = 1.0;    // 0.0 to 1.0 (0=Umbra, 1=Fully lit)
    
    // AI / Autopilot flags
    bool suicide_burn_locked = false;
    MissionState status = PRE_LAUNCH;
    std::string mission_msg = "SYSTEM READY";
    int mission_phase = 0;
    double mission_timer = 0.0;
    bool auto_mode = true;
    double leg_deploy_progress = 0.0;
    
    // Particle System state 
    static const int MAX_SMOKE = 300;
    SmokeParticle smoke[MAX_SMOKE];
    int smoke_idx = 0;
    
    // Autopilot PID controllers (stored with state so they persist)
    PID pid_vert = {0.5, 0.001, 1.2};       
    PID pid_pos = {0.001, 0.0, 0.2};        
    PID pid_att = {40000.0, 0.0, 100000.0}; 
    PID pid_att_z = {40000.0, 0.0, 100000.0};
};

#endif // ROCKET_STATE_H
