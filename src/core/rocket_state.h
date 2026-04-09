#ifndef ROCKET_STATE_H
#define ROCKET_STATE_H

#include <string>
#include <vector>
#include <cmath>
#include <iostream>
#include <memory>
#include <mutex>
#include "math/math3d.h"

// --- 物理常数 (Constants) ---
// 这些是宇宙运行的基本规则。初学者可以把它们看作是“游戏规则的参数”。
#ifndef PI
constexpr double PI = 3.14159265358979323846; // 圆周率：半圆的角度（180度）
#endif
constexpr double G0 = 9.80665;             // 地表重力加速度：物体在地球表面下落的速度变化率 (m/s^2)
constexpr double EARTH_RADIUS = 6371000.0; // 地球平均半径：从地心到海平面的距离 (m)
constexpr double SLP = 1013.25;            // 海平面大气压：呼吸时的标准压力 (hPa)
const double au_meters = 149597870700.0;   // 天文单位：地球到太阳的平均距离
const double G_const = 6.67430e-11;        // 万有引力常数：决定两个物体之间引力大小的关键系数
const double M_sun = 1.989e30;             // 太阳质量 (kg)
const double GM_sun = G_const * M_sun;     // 太阳引力参数：用于快速计算绕日轨道

enum BodyType {
    STAR,
    TERRESTRIAL,
    GAS_GIANT,
    MOON,
    RINGED_GAS_GIANT
};

// 天体结构体 (Celestial Body)
// 描述一颗行星、恒星或卫星。它既包含了它的物理属性，也包含了它在宇宙中的运动状态。
struct CelestialBody {
    std::string name;        // 名字 (如 "Earth" 或 "Mars")
    double mass;             // 质量 (kg)：决定引力的大小
    double radius;           // 半径 (m)：决定碰撞检测和地表渲染
    BodyType type;           // 类型：恒星、类地行星、气态巨行星等
    
    // 渲染颜色
    float r, g, b;

    // 自转参数 (Rotation)
    double axial_tilt;                 // 轴倾角 (弧度)：决定季节变化和极昼极夜
    double rotation_period;            // 自转周期 (秒)：一个“昼夜”的长度
    double prime_meridian_epoch;       // 初始相位：在初始时刻，0度经线对着哪个方向
    
    // 轨道根数 (Orbital Elements)
    // 这些变量描述了星星是如何绕着母星转动的。
    double sma_base;          // 半长轴 (m)：椭圆轨道的“平均半径”
    double sma_rate;          // 半长轴变化率：轨道是否在慢慢变大或变小
    double ecc_base;          // 离心率：0 是正圆，越接近 1 轨道越扁（长椭圆）
    double ecc_rate;          // 离心率变化率
    double inc_base;          // 轨道倾角 (弧度)：轨道相对于基准平面的倾斜程度
    double inc_rate;          // 倾角变化率
    double lan_base;          // 升交点黄经：轨道在基准平面上“指”的方向
    double lan_rate;          // 升交点变化率
    double arg_peri_base;     // 近地点幅角：轨道椭圆的“尖端”朝向哪里
    double arg_peri_rate;     // 近地点幅角变化率
    double mean_anom_base;    // 平近点角：决定了星星在轨道上的初始位置
    double mean_anom_rate;    // 平均运动速度：星星跑多快 (rad/sec)
    
    // 实时物理状态 (Dynamic State)
    double px, py, pz;        // 以太阳为中心的 3D 坐标 (x, y, z)
    double vx, vy, vz;        // 3D 运行速度 (m/s)
    
    // 引力范围 (Sphere of Influence)
    double soi_radius;        // 只有进入这个半径范围，我们才认为飞船是绕着这个天体转的

    // Galaxy Info Fields
    int parent_index = -1;             // Index of parent body (for moons)
    double surface_pressure = 0.0;     // (hPa)
    double average_temp = 0.0;         // (K)
    double scattering_coef = 0.1;      // (Albedo approximation)
    double eccentricity = 0.0;         // Current calculated eccentricity
    double inclination = 0.0;          // Current calculated inclination (rad)
    double orbital_period = 0.0;       // (seconds)
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
// PID 控制器 (Proportional-Integral-Derivative)
// 这是一个非常经典的控制算法。对于萌新来说：
// kp (比例)：发现误差，立刻用力纠正（误差越大，力越大）。
// ki (积分)：发现误差一直消除不了，积攒怒气，加大力度。
// kd (微分)：“刹车”项。快要到达目标时，提前减速，防止冲过头。
struct PID {
    double kp, ki, kd;
    double integral = 0;             // 误差的累计
    double prev_error = 0;           // 上一次的误差（用于计算微分）
    double integral_limit = 50.0;    // 防止“怒气”积攒无上限导致失控

    // 更新函数：输入你想要达到的目标（target）和当前实际值（current），返回你应该施加的力。
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
    Vec3 locked_burn_dir;     // Captured inertial direction for guidance
    double burn_duration = 0; // Calculated duration using rocket equation

    // Fixed orbital elements to prevent jitter during burns
    double ref_a=0, ref_ecc=0, ref_M0=0, ref_n=0;
    Vec3 ref_e_dir, ref_p_dir, ref_center;
    int ref_body=-1;
    
    // Snapshot of absolute state at node creation/update for stable post-burn orbit prediction
    double snap_px=0, snap_py=0, snap_pz=0; // absolute position at node completion
    double snap_vx=0, snap_vy=0, snap_vz=0; // absolute velocity at node completion
    
    // Relative state snapshots (Principia style) for dynamic guidance stability
    double snap_rel_px=0, snap_rel_py=0, snap_rel_pz=0; // relative to ref_body
    double snap_rel_vx=0, snap_rel_vy=0, snap_rel_vz=0; // relative to ref_body
    
    double snap_time = -1.0;                // simulation time of completion
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
// 静态火箭配置 (Rocket Configuration)
// 描述火箭的物理结构，这些属性通常在飞行中不会改变（直到你抛弃一级火箭）。
struct RocketConfig {
    double dry_mass;          // 干重 (kg)：燃料烧完后的重量
    double diameter;          // 直径 (m)：影响空气阻力
    double height;            // 高度 (m)
    double bounds_bottom = 0; // 底部偏移：通常为负数，表示从火箭中心到最底部的距离
    int stages;               // 总级数 (如 2 或 3 段)
    double specific_impulse;  // 比冲 (s)：衡量发动机效率，数值越高燃料越耐烧
    double cosrate;           // 燃料消耗率参数 (kg/s)
    double nozzle_area;       // 喷嘴面积：在不同气压环境下影响推力

    // 多级配置 (Stage 0 = 底部第一级)
    std::vector<StageConfig> stage_configs;

    // 上方级别的总质量：除了当前正在喷火的那一级，上面还背了多少东西
    double upper_stages_mass = 0;
};

// Control Inputs (actuators driven by Player/AI)
struct ControlInput {
    double throttle = 0.0;    // 0.0 to 1.0
    double torque_cmd = 0.0;  // Z-axis torque command (pitch in 2D plane)
    double torque_cmd_z = 0.0; // X/Y axis torque command (out of plane pitch)
    double torque_cmd_roll = 0.0; // Roll torque command
};

// ==========================================
// ECS 组件定义 (ECS Components)
// ------------------------------------------
// 这些是将要取代原来庞大 RocketState 的细分组件。
// 它们是纯数据结构（POD），可独立作为实体组装积木。
// ==========================================

// 1. 位置与变换组件 (Transform Component)
struct TransformComponent {
    double px = 0.0, py = EARTH_RADIUS + 0.1, pz = 0.0; // 3D 位置 (m)
    double abs_px = 0.0, abs_py = 0.0, abs_pz = 0.0;    // 绝对坐标 (以太阳为中心)
    double surf_px = 0.0, surf_py = EARTH_RADIUS, surf_pz = 0.0; // 地表坐标
    
    // 发射场参考
    double launch_latitude = 28.5;  
    double launch_longitude = -80.6;
    double launch_site_px = 0.0, launch_site_py = EARTH_RADIUS, launch_site_pz = 0.0;
};

// 2. 速度与动力学组件 (Velocity Component)
struct VelocityComponent {
    double vx = 0.0, vy = 0.0, vz = 0.0;       // 3D 速度 (m/s)
    double abs_vx = 0.0, abs_vy = 0.0, abs_vz = 0.0; // 绝对速度
    double acceleration = 0.0;                 // 当前总加速度 (m/s^2)
    double vertical_velocity = 0.0;            // 垂直分量
    double horizontal_velocity = 0.0;          // 水平分量
};

// 3. 姿态组件 (Attitude Component)
struct AttitudeComponent {
    Quat attitude;           // 四元数：描述火箭在 3D 空间里的朝向
    bool initialized = false;
    double angle = 0.0;      // 偏航角
    double ang_vel = 0.0;    // 偏航角速度
    double angle_z = 0.0;    // 俯仰角
    double ang_vel_z = 0.0;
    double angle_roll = 0.0; // 滚转角
    double ang_vel_roll = 0.0;
};

// 4. 燃料与发动机状态组件 (Propulsion Component)
struct PropulsionComponent {
    double fuel = 0.0;                  // 当前剩余可用燃料 (kg)
    int current_stage = 0;              // 指向当前活跃的级（0 是底部，越往上越大）
    int total_stages = 1;               // 火箭总共有几段
    std::vector<double> stage_fuels;    // 每一级各自还剩多少燃料
    double jettisoned_mass = 0.0;       // 已经扔掉的空壳总重量 (kg)
    double fuel_consumption_rate = 0.0; // 实时燃料流速 (kg/s)
    double thrust_power = 0.0;          // 当前推力百分比或功率 (0~1)
};

// 5. 遥测与计算环境组件 (Telemetry Component)
struct TelemetryComponent {
    double sim_time = 0.0;          // 游戏内的总时间 (s)
    double altitude = 0.0;          // 海拔高度 (m)
    double terrain_altitude = 0.0;  // 地形高度 (m)
    double velocity = 0.0;          // 垂直速度 (m/s)
    double local_vx = 0.0;          // 水平平移速度 (m/s)
    double solar_occlusion = 1.0;   // 光照遮挡率 (1.0 = 全阳光，0.0 = 阴影)
};

// 6. 任务与自动驾驶组件 (Mission & Guidance Component)
struct GuidanceComponent {
    MissionState status = PRE_LAUNCH; // 任务大状态
    std::string mission_msg = "SYSTEM READY";
    int mission_phase = 0;           // 任务细分阶段
    double mission_timer = 0.0;      // 阶段计时器
    bool auto_mode = true;           // 自动驾驶开关
    bool sas_active = true;          // 姿态稳定开关
    bool rcs_active = true;          // 姿态发动机 (RCS) 开关
    SASMode sas_mode = SAS_STABILITY;
    Vec3 sas_target_vec = {0, 0, 0}; 
    double leg_deploy_progress = 0.0;
    bool suicide_burn_locked = false; // 是否已经锁定“自杀点火”逻辑
    bool show_absolute_time = false;  // 是否显示绝对时间 (UT)

    // Autopilot PID controllers
    PID pid_vert = {0.5, 0.001, 1.2};       
    PID pid_pos = {0.001, 0.0, 0.2};        
    PID pid_att = {40000.0, 0.0, 100000.0}; 
    PID pid_att_z = {40000.0, 0.0, 100000.0};
    PID pid_att_roll = {40000.0, 0.0, 100000.0};
};

// 7. 轨道几何与异步预测组件 (Orbit Component)
struct OrbitComponent {
    struct Apsis {
        bool is_apoapsis;
        Vec3 local_pos;   
        double sim_time;
        double altitude;
    };

    // 轨迹预测结果 (当前轨道)
    std::vector<Vec3> predicted_path;
    std::vector<Apsis> predicted_apsides;
    std::vector<Vec3> predicted_ground_track;

    // 轨迹预测结果 (变轨后轨道)
    std::vector<Vec3> predicted_mnv_path;
    std::vector<Apsis> predicted_mnv_apsides;
    std::vector<Vec3> predicted_mnv_ground_track;
    
    // 异步预测同步锁
    mutable std::shared_ptr<std::mutex> path_mutex = std::make_shared<std::mutex>();
    double last_prediction_sim_time = -1.0;
    bool prediction_in_progress = false;
};

// 8. 变轨计划组件 (Maneuver Component)
struct ManeuverComponent {
    std::vector<ManeuverNode> maneuvers;
    int selected_maneuver_index = -1;
};

// 9. 视觉特效组件 (VFX Component)
struct VFXComponent {
    static const int MAX_SMOKE = 300;
    SmokeParticle smoke[MAX_SMOKE];
    int smoke_idx = 0;
};

// Dynamic Rocket State (updated by physics)
// 实时火箭状态 (Rocket State)
// 这是整个物理模拟中最核心的结构体，记录了火箭在这一秒钟里的所有状态。
// [ECS 过渡阶段] 这个结构体将作为 legacy 桥接数据长期存在，直到它的内容被完全掏空。
struct RocketState {

    // 1. 燃料与分级状态
    double fuel = 0.0;                  // 当前剩余可用燃料 (kg)
    int current_stage = 0;              // 指向当前活跃的级（0 是底部，越往上越大）
    int total_stages = 1;               // 火箭总共有几段
    std::vector<double> stage_fuels;    // 每一级各自还剩多少燃料
    double jettisoned_mass = 0.0;       // 已经扔掉的空壳总重量 (kg)
    
    // 2. 坐标与运动 (相对于当前所在的星球中心)
    double px = 0.0, py = EARTH_RADIUS + 0.1, pz = 0.0; // 3D 位置 (m)
    double vx = 0.0, vy = 0.0, vz = 0.0;               // 3D 速度 (m/s)
    
    // 绝对坐标 (以太阳为中心)
    double abs_px = 0.0, abs_py = 0.0, abs_pz = 0.0;
    double abs_vx = 0.0, abs_vy = 0.0, abs_vz = 0.0;

    // 地表坐标 (相对于行星中心，但在行星自转坐标系下)
    double surf_px = 0.0, surf_py = EARTH_RADIUS, surf_pz = 0.0;
    
    // 发射场坐标与经纬度
    double launch_latitude = 28.5;  // 纬度 (度)
    double launch_longitude = -80.6; // 经度 (度)
    double launch_site_px = 0.0, launch_site_py = EARTH_RADIUS, launch_site_pz = 0.0;
    
    // 3. 姿态控制 (Orientance)
    Quat attitude;           // 四元数：描述火箭在 3D 空间里的朝向
    bool attitude_initialized = false;
    double angle = 0.0;      // 偏航角
    double ang_vel = 0.0;    // 偏航角速度
    double angle_z = 0.0;    // 俯仰角
    double ang_vel_z = 0.0;
    double angle_roll = 0.0; // 滚转角
    double ang_vel_roll = 0.0;
    
    // 4. 环境感知与物理量
    double sim_time = 0.0;          // 游戏内的总时间 (s)
    double altitude = 0.0;          // 海拔高度 (m)
    double terrain_altitude = 0.0;  // 地形高度 (m)
    double velocity = 0.0;          // 垂直速度 (m/s)
    double local_vx = 0.0;          // 水平平移速度 (m/s)
    
    // 发动机实时数据
    double fuel_consumption_rate = 0.0; // 实时燃料流速 (kg/s)
    double thrust_power = 0.0;          // 当前推力百分比或功率 (0~1)
    double acceleration = 0.0;          // 当前总加速度 (m/s^2)
    double solar_occlusion = 1.0;       // 光照遮挡率 (1.0 = 全阳光，0.0 = 阴影)
    
    // 5. 任务与自动驾驶
    bool suicide_burn_locked = false; // 是否已经锁定“自杀点火”逻辑
    MissionState status = PRE_LAUNCH; // 任务大状态
    std::string mission_msg = "SYSTEM READY";
    int mission_phase = 0;           // 任务细分阶段 (如：0=上升, 1=滑行...)
    double mission_timer = 0.0;      // 阶段计时器
    bool show_absolute_time = false; // 是否显示绝对时间 (UT)
    bool auto_mode = true;           // 自动驾驶开关
    bool sas_active = true;          // 姿态稳定开关
    bool rcs_active = true;          // 姿态发动机 (RCS) 开关
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

    // Apsis markers
    struct Apsis {
        bool is_apoapsis;
        Vec3 local_pos;   // position relative to reference body in the selected reference frame
        double sim_time;
        double altitude;
    };

    // Asynchronous Prediction Results
    mutable std::shared_ptr<std::mutex> path_mutex = std::make_shared<std::mutex>();
    std::vector<Vec3> predicted_path;
    std::vector<Vec3> predicted_mnv_path;
    std::vector<Apsis> predicted_apsides;
    std::vector<Apsis> predicted_mnv_apsides;
    std::vector<Vec3> predicted_ground_track;
    std::vector<Vec3> predicted_mnv_ground_track;
    double last_prediction_sim_time = -1.0;
    bool prediction_in_progress = false;
};

#endif // ROCKET_STATE_H
