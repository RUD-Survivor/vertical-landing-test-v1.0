#ifndef ROCKET_STATE_H
#define ROCKET_STATE_H

#include <string>
#include <vector>
#include <cmath>
#include <iostream>
#include <memory>
#include <mutex>
#include "math/math3d.h"

// Constants
#ifndef PI
constexpr double PI = 3.14159265358979323846;
#endif
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

enum SASMode {
    SAS_STABILITY,
    SAS_PROGRADE,
    SAS_RETROGRADE,
    SAS_NORMAL,
    SAS_ANTINORMAL,
    SAS_RADIAL_IN,
    SAS_RADIAL_OUT,
    SAS_MANEUVER
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

// Maneuver Node data
struct ManeuverNode {
    double sim_time;          // Time of maneuver
    Vec3 delta_v;             // (Prograde, Normal, Radial) components in meters/sec
    bool active = true;
    bool selected = false;
    int burn_mode = 0;        // 0 = Impulse, 1 = Sustained

    // Fixed orbital elements to prevent jitter during burns
    double ref_a=0, ref_ecc=0, ref_M0=0, ref_n=0;
    Vec3 ref_e_dir, ref_p_dir, ref_center;
    int ref_body=-1;
    
    // Snapshot of absolute state at node creation/update for stable post-burn orbit prediction
    double snap_px=0, snap_py=0, snap_pz=0; // absolute position at node time
    double snap_vx=0, snap_vy=0, snap_vz=0; // absolute velocity at node time
    bool snap_valid = false;                 // whether snapshot is populated
};

// Per-stage physical configuration
struct StageConfig {
    double dry_mass = 0;          // Dry mass of this stage (kg)
    double fuel_capacity = 0;     // Total fuel capacity (kg)
    double specific_impulse = 0;  // Weighted ISP (seconds)
    double consumption_rate = 0;  // Total fuel consumption rate (kg/s)
    double thrust = 0;            // Total thrust (N)
    double height = 0;            // Height of this stage (m)
    double diameter = 0;          // Max diameter (m)
    double nozzle_area = 0;       // Nozzle area
    int part_start_index = 0;     // Start index in assembly parts list
    int part_end_index = 0;       // End index (exclusive) in assembly parts list
};

// Static Rocket Configuration (immutable during flight)
struct RocketConfig {
    // Current active stage shorthand (synced by StageManager)
    double dry_mass;
    double diameter;
    double height;
    int stages;
    double specific_impulse;
    double cosrate; // fuel_consumption_rate / mass flow rate parameter
    double nozzle_area;

    // Multi-stage configurations (stage 0 = bottom, fires first)
    std::vector<StageConfig> stage_configs;

    // Total mass of stages above the active one (payload for current stage)
    double upper_stages_mass = 0;
};

// Control Inputs (actuators driven by Player/AI)
struct ControlInput {
    double throttle = 0.0;    // 0.0 to 1.0
    double torque_cmd = 0.0;  // Z-axis torque command (pitch in 2D plane)
    double torque_cmd_z = 0.0; // X/Y axis torque command (out of plane pitch)
    double torque_cmd_roll = 0.0; // Roll torque command
};

// Dynamic Rocket State (updated by physics)
struct RocketState {
    // Basic properties
    double fuel = 0.0;
    
    // Multi-stage state
    int current_stage = 0;              // Active stage index (0 = bottom, fires first)
    int total_stages = 1;               // Total number of stages
    std::vector<double> stage_fuels;    // Remaining fuel per stage
    double jettisoned_mass = 0.0;       // Cumulative mass of jettisoned stages
    
    // Position/Velocity relative to current SOI origin
    double px = 0.0, py = EARTH_RADIUS + 0.1, pz = 0.0;
    double vx = 0.0, vy = 0.0, vz = 0.0;
    
    // Absolute Heliocentric Position/Velocity (used for continuous global tracking and Eclipse checks)
    double abs_px = 0.0, abs_py = 0.0, abs_pz = 0.0;
    double abs_vx = 0.0, abs_vy = 0.0, abs_vz = 0.0;
    
    // Body-fixed surface coordinates (relative to planet center, rotated frame)
    double surf_px = 0.0, surf_py = EARTH_RADIUS, surf_pz = 0.0;
    
    // Attitude
    Quat attitude;           // True 3D attitude
    bool attitude_initialized = false;
    double angle = 0.0;      // Yaw (in 2D plane)
    double ang_vel = 0.0;
    double angle_z = 0.0;    // Out-of-plane pitch
    double ang_vel_z = 0.0;
    double angle_roll = 0.0; // Self-spin (Roll)
    double ang_vel_roll = 0.0;
    
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
    bool sas_active = true;
    bool rcs_active = true;
    SASMode sas_mode = SAS_STABILITY;
    Vec3 sas_target_vec = {0, 0, 0}; // Normalized target vector in world space (relative to body)
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
    PID pid_att_roll = {40000.0, 0.0, 100000.0};

    // Maneuver Nodes
    std::vector<ManeuverNode> maneuvers;
    int selected_maneuver_index = -1;

    // Asynchronous Prediction Results
    mutable std::shared_ptr<std::mutex> path_mutex = std::make_shared<std::mutex>();
    std::vector<Vec3> predicted_path;
    std::vector<Vec3> predicted_mnv_path;
    std::vector<Vec3> predicted_ground_track;
    std::vector<Vec3> predicted_mnv_ground_track;
    double last_prediction_sim_time = -1.0;
    bool prediction_in_progress = false;
};

#endif // ROCKET_STATE_H
