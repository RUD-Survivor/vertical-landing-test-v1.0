#include "core/universe_model.h"
#include "physics_system.h"
#include "aerodynamics_system.h"
#include "ground_collision_system.h"
#include "math/math3d.h"
#include <algorithm>
#include <iostream>

/* 
 * 物理系统实现文件 (physics_system.cpp)
 * -----------------------------------------
 * 这里是模拟器的核心逻辑所在地。我们在这里定义了太阳系、计算星球轨道、
 * 处理空气动力学、以及最核心的数值积分器（RK4）。
 */

// 太阳系天体容器：存储太阳、行星、卫星等所有大型天体。
// 当前引力范围 (SOI) 的索引。
// 游戏开始时默认在地球 (Earth) 的 SOI 内。在 UniverseModel::getInstance().solar_system 向量中，地球的索引是 3。
namespace PhysicsSystem {

// 历元常数：J2000 历元是天文学中常用的基准时间（2000年1月1日正午）。
// 很多星球的轨道参数都是相对于这个时间点给出的。
// 一个儒略世纪 = 36525 天。
constexpr double SECONDS_PER_JULIAN_CENTURY = 36525.0 * 24.0 * 3600.0;

namespace {

static bool finiteScalar(double v) {
    return std::isfinite(v);
}

static bool finiteVec(const Vec3& v) {
    return finiteScalar(v.x) && finiteScalar(v.y) && finiteScalar(v.z);
}

static bool finiteQuat(const Quat& q) {
    return finiteScalar(q.w) && finiteScalar(q.x) && finiteScalar(q.y) && finiteScalar(q.z);
}

static Quat surfaceRotation(const CelestialBody& body, double simTime) {
    double theta = body.prime_meridian_epoch + (simTime * 2.0 * PI / body.rotation_period);
    Quat rot = Quat::fromAxisAngle(Vec3(0, 0, 1), theta);
    Quat tilt = Quat::fromAxisAngle(Vec3(1, 0, 0), body.axial_tilt);
    return tilt * rot;
}

static void syncAbsoluteState(TransformComponent& trans,
                              VelocityComponent& vel,
                              const CelestialBody& body) {
    trans.abs_px = body.px + trans.px;
    trans.abs_py = body.py + trans.py;
    trans.abs_pz = body.pz + trans.pz;
    vel.abs_vx = body.vx + vel.vx;
    vel.abs_vy = body.vy + vel.vy;
    vel.abs_vz = body.vz + vel.vz;
}

static void lockEntityToSurface(TransformComponent& trans,
                                VelocityComponent& vel,
                                TelemetryComponent& tele,
                                const CelestialBody& body,
                                double simTime) {
    Vec3 localPos(trans.surf_px, trans.surf_py, trans.surf_pz);
    Vec3 worldPos = surfaceRotation(body, simTime).rotate(localPos);
    trans.px = worldPos.x;
    trans.py = worldPos.y;
    trans.pz = worldPos.z;
    Vec3 worldVel = GroundCollisionSystem::SurfaceVelocityAt(body, simTime, worldPos);
    vel.vx = worldVel.x;
    vel.vy = worldVel.y;
    vel.vz = worldVel.z;
    tele.altitude = 0.0;
    tele.velocity = 0.0;
    tele.local_vx = 0.0;
    syncAbsoluteState(trans, vel, body);
}

static void sanitizeDynamicState(TransformComponent& trans,
                                 VelocityComponent& vel,
                                 AttitudeComponent& att,
                                 PropulsionComponent& prop,
                                 TelemetryComponent& tele,
                                 const RocketConfig& config,
                                 const CelestialBody& body,
                                 double simTime) {
    Vec3 pos(trans.px, trans.py, trans.pz);
    Vec3 linVel(vel.vx, vel.vy, vel.vz);
    if (!finiteVec(pos) || pos.lengthSq() < 1.0) {
        Vec3 local(trans.surf_px, trans.surf_py, trans.surf_pz);
        if (!finiteVec(local) || local.lengthSq() < 1.0) {
            local = Vec3(0, body.radius - config.bounds_bottom + 10.0, 0);
        }
        Vec3 worldPos = surfaceRotation(body, simTime).rotate(local);
        trans.px = worldPos.x;
        trans.py = worldPos.y;
        trans.pz = worldPos.z;
        vel.vx = vel.vy = vel.vz = 0.0;
        tele.altitude = 0.0;
    } else if (!finiteVec(linVel)) {
        vel.vx = vel.vy = vel.vz = 0.0;
    }

    if (!finiteQuat(att.attitude)) {
        att.attitude = Quat();
        att.initialized = false;
    } else {
        att.attitude = att.attitude.normalized();
        if (!finiteQuat(att.attitude)) {
            att.attitude = Quat();
            att.initialized = false;
        }
    }

    Vec3 omega(att.ang_vel_z, att.ang_vel_roll, att.ang_vel);
    if (!finiteVec(omega)) {
        att.ang_vel_z = 0.0;
        att.ang_vel_roll = 0.0;
        att.ang_vel = 0.0;
    }

    if (!finiteScalar(prop.fuel)) prop.fuel = 0.0;
    if (!finiteScalar(prop.thrust_power)) prop.thrust_power = 0.0;
    if (!finiteScalar(prop.fuel_consumption_rate)) prop.fuel_consumption_rate = 0.0;
}

struct PhysicsStepSnapshot {
    TransformComponent trans;
    VelocityComponent vel;
    AttitudeComponent att;
    PropulsionComponent prop;
    TelemetryComponent tele;
    GuidanceComponent guid;
};

static bool stateIsFinite(const TransformComponent& trans,
                          const VelocityComponent& vel,
                          const AttitudeComponent& att,
                          const PropulsionComponent& prop,
                          const TelemetryComponent& tele) {
    return finiteVec(Vec3(trans.px, trans.py, trans.pz)) &&
           finiteVec(Vec3(trans.abs_px, trans.abs_py, trans.abs_pz)) &&
           finiteVec(Vec3(vel.vx, vel.vy, vel.vz)) &&
           finiteVec(Vec3(vel.abs_vx, vel.abs_vy, vel.abs_vz)) &&
           finiteQuat(att.attitude) &&
           finiteVec(Vec3(att.ang_vel_z, att.ang_vel_roll, att.ang_vel)) &&
           finiteScalar(prop.fuel) &&
           finiteScalar(prop.thrust_power) &&
           finiteScalar(prop.fuel_consumption_rate) &&
           finiteScalar(tele.sim_time) &&
           finiteScalar(tele.altitude) &&
           finiteScalar(tele.velocity) &&
           finiteScalar(tele.local_vx);
}

static void restoreSnapshot(const PhysicsStepSnapshot& snapshot,
                            TransformComponent& trans,
                            VelocityComponent& vel,
                            AttitudeComponent& att,
                            PropulsionComponent& prop,
                            TelemetryComponent& tele,
                            GuidanceComponent& guid) {
    trans = snapshot.trans;
    vel = snapshot.vel;
    att = snapshot.att;
    prop = snapshot.prop;
    tele = snapshot.tele;
    guid = snapshot.guid;
    guid.mission_msg = "PHYSICS STEP ROLLED BACK: NON-FINITE SOLVER RESULT";
}

} // namespace

/**
 * @brief 初始化太阳系 (InitSolarSystem)
 * 
 * 这里的参数来源于 NASA 的行星历表简表。为了让新手理解：
 * - mass: 质量 (kg)。决定了引力的大小。
 * - radius: 半径 (m)。用于碰撞检测和表面高度计算。
 * - sma (Semi-major axis): 半长轴。即轨道的“平均半径”。
 * - ecc (Eccentricity): 离心率。决定轨道有多“扁”，0 是正圆，越接近 1 越扁。
 * - inc (Inclination): 轨道倾角。相对于黄道面的倾斜程度。
 * - mean_anom (Mean Anomaly): 平近点角。决定星球在轨道上的起始位置。
 * 
 * 细心的同学会发现有 _base 和 _rate 两种参数。
 * 这是因为行星的轨道不是永恒不变的，受到其它行星的影响，它们会缓慢“漂移”。
 * _rate 就是每世纪的变化率。
 */
void InitSolarSystem() {
    UniverseModel::getInstance().solar_system.clear();
    
    double R_earth = 6371000.0;
    
    // --- 0: 太阳 (Sun) ---
    // 太阳是太阳系的中心，在我们的模拟中它固定在 (0,0,0) 位置。
    CelestialBody sun;
    sun.name = "Sun";
    sun.mass = 1.989e30; // 质量极其巨大，占太阳系总质量的 99.8%
    sun.radius = 696340000.0;
    sun.type = STAR;
    sun.r = 1.0; sun.g = 0.9; sun.b = 0.8; // 发光颜色：微黄
    sun.axial_tilt = 7.25 * PI / 180.0; // 自转轴倾角
    sun.rotation_period = 25.38 * 24.0 * 3600.0; // 自转周期
    sun.prime_meridian_epoch = 0.0;
    // 太阳不绕别人转，所以轨道参数全为 0
    sun.sma_base = 0.0; sun.sma_rate = 0.0;
    sun.ecc_base = 0.0; sun.ecc_rate = 0.0;
    sun.inc_base = 0.0; sun.inc_rate = 0.0;
    sun.lan_base = 0.0; sun.lan_rate = 0.0;
    sun.mean_anom_base = 0.0; sun.mean_anom_rate = 0.0;
    sun.surface_pressure = 0.0; sun.average_temp = 5778.0; sun.scattering_coef = 1.0;
    sun.eccentricity = 0.0; sun.inclination = 0.0; sun.orbital_period = 0.0;
    UniverseModel::getInstance().solar_system.push_back(sun);

    // --- 1: 水星 (Mercury) ---
    // 离太阳最近，公转速度最快。
    CelestialBody mercury;
    mercury.name = "Mercury";
    mercury.mass = 3.3011e23;
    mercury.radius = 2439700.0;
    mercury.type = TERRESTRIAL; // 类地行星
    mercury.r = 0.5; mercury.g = 0.5; mercury.b = 0.5; // 灰色
    mercury.axial_tilt = 0.034 * PI / 180.0;
    mercury.rotation_period = 58.646 * 24.0 * 3600.0;
    mercury.prime_meridian_epoch = 0.0;
    // au_meters 是“天文单位”，即地球到太阳的平均距离
    mercury.sma_base = 0.38709927 * au_meters; mercury.sma_rate = 0.00000037 * au_meters;
    mercury.ecc_base = 0.20563593; mercury.ecc_rate = 0.00001906;
    mercury.inc_base = 7.00497902 * PI / 180.0; mercury.inc_rate = -0.00594749 * PI / 180.0;
    mercury.lan_base = 48.33076593 * PI / 180.0; mercury.lan_rate = -0.12534081 * PI / 180.0;
    mercury.arg_peri_base = 29.1241 * PI / 180.0; mercury.arg_peri_rate = 0.0;
    mercury.mean_anom_base = 174.796 * PI / 180.0; mercury.mean_anom_rate = (360.0 / 87.969) * PI / 180.0 / (24.0 * 3600.0);
    mercury.surface_pressure = 1.0e-12; // 几乎没有大气
    mercury.average_temp = 440.0; mercury.scattering_coef = 0.106;
    mercury.eccentricity = 0.2056; mercury.inclination = 7.0 * PI / 180.0; mercury.orbital_period = 87.969 * 24.0 * 3600.0;
    UniverseModel::getInstance().solar_system.push_back(mercury);
    
    // --- 2: 金星 (Venus) ---
    // 拥有极其浓厚的大气层，表面压力很大。
    CelestialBody venus;
    venus.name = "Venus";
    venus.mass = 4.8675e24;
    venus.radius = 6051800.0;
    venus.type = TERRESTRIAL;
    venus.r = 0.9; venus.g = 0.8; venus.b = 0.6; // 黄白色
    venus.axial_tilt = 177.36 * PI / 180.0; // 金星是躺着倒转的！
    venus.rotation_period = -243.025 * 24.0 * 3600.0; // 负值表示逆向自转
    venus.prime_meridian_epoch = 0.0;
    venus.sma_base = 0.72333199 * au_meters; venus.sma_rate = 0.00000039 * au_meters;
    venus.ecc_base = 0.00677323; venus.ecc_rate = -0.00004107;
    venus.inc_base = 3.39467605 * PI / 180.0; venus.inc_rate = -0.00078890 * PI / 180.0;
    venus.lan_base = 76.67984255 * PI / 180.0; venus.lan_rate = -0.27769418 * PI / 180.0;
    venus.arg_peri_base = 54.884 * PI / 180.0; venus.arg_peri_rate = 0.0;
    venus.mean_anom_base = 50.115 * PI / 180.0; venus.mean_anom_rate = (360.0 / 224.701) * PI / 180.0 / (24.0 * 3600.0);
    venus.surface_pressure = 92000.0; // 表面压力是地球的 92 倍
    venus.average_temp = 737.0; venus.scattering_coef = 0.689;
    venus.eccentricity = 0.0067; venus.inclination = 3.39 * PI / 180.0; venus.orbital_period = 224.7 * 24.0 * 3600.0;
    UniverseModel::getInstance().solar_system.push_back(venus);

    // --- 3: 地球 (Earth) ---
    // 我们最熟悉的家园，也是大多数任务的起点。
    CelestialBody earth;
    earth.name = "Earth";
    earth.mass = 5.972e24;
    earth.radius = EARTH_RADIUS; // 约 6371 km
    earth.type = TERRESTRIAL;
    earth.r = 0.2; earth.g = 0.5; earth.b = 1.0; // 蓝色星球
    earth.axial_tilt = 23.439 * PI / 180.0; // 地轴偏斜，导致了四季变化
    earth.rotation_period = 0.99726968 * 24.0 * 3600.0; // 恒星日（Sidereal day），约 23小时56分
    earth.prime_meridian_epoch = 0.0;
    earth.sma_base = 1.00000261 * au_meters; earth.sma_rate = 0.00000562 * au_meters;
    earth.ecc_base = 0.01671123; earth.ecc_rate = -0.00004392;
    earth.inc_base = -0.00001531 * PI / 180.0; earth.inc_rate = -0.01294668 * PI / 180.0;
    earth.lan_base = 0.0; earth.lan_rate = 0.0;
    earth.arg_peri_base = 114.20783 * PI / 180.0; earth.arg_peri_rate = 0.0;
    earth.mean_anom_base = 358.617 * PI / 180.0; earth.mean_anom_rate = (360.0 / 365.256) * PI / 180.0 / (24.0 * 3600.0);
    earth.surface_pressure = 1013.25; // 1 标准大气压 (hPa)
    earth.average_temp = 288.0; // 15 摄氏度
    earth.scattering_coef = 0.306; // 反照率/散射系数
    earth.eccentricity = 0.0167; earth.inclination = 0.0; earth.orbital_period = 365.256 * 24.0 * 3600.0;
    UniverseModel::getInstance().solar_system.push_back(earth);
    
    // 4: MOON (Luna)
    // Note: The moon's orbit is geocentric, but for VSOP87-like ephemeris in our N-body 
    // heliocentric solver, we will treat it as a body that orbits the Sun but stays close to Earth.
    // For a video-game approximation, we will update the Moon's position as Earth's position + lunar orbit
    // We'll give it a heliocentric approximation. To do this perfectly we'll just code its update differently
    // in UpdateCelestialBodies.
    // --- 4: 月球 (Moon / Luna) ---
    // 注意：在物理代码中，月球的坐标通常相对于地球（地心坐标系）。
    // 在 UpdateCelestialBodies 中，我们会把月球的位置加上地球的位置，合并成太阳坐标系下的位置。
    CelestialBody moon;
    moon.name = "Moon";
    moon.mass = 7.342e22;
    moon.radius = 1737400.0;
    moon.type = MOON;
    moon.r = 0.7; moon.g = 0.7; moon.b = 0.7;
    moon.axial_tilt = 1.5424 * PI / 180.0;
    moon.rotation_period = 27.321 * 24.0 * 3600.0; // 潮汐锁定：自转周期等于公转周期
    moon.prime_meridian_epoch = 0.0;
    moon.sma_base = 384400000.0; // 约 38.4 万公里
    moon.ecc_base = 0.0549; moon.ecc_rate = 0.0;
    moon.inc_base = 5.145 * PI / 180.0; moon.inc_rate = 0.0;
    moon.lan_base = 125.08 * PI / 180.0; moon.lan_rate = -19.34 * PI / 180.0; 
    moon.arg_peri_base = 318.15 * PI / 180.0; moon.arg_peri_rate = 0.0;
    moon.mean_anom_base = 115.3654 * PI / 180.0; moon.mean_anom_rate = (360.0 / 27.321) * PI / 180.0 / (24.0 * 3600.0);
    moon.parent_index = 3; // 父天体索引为 3 (地球)
    moon.surface_pressure = 1.0e-11; // 真空
    moon.average_temp = 250.0; moon.scattering_coef = 0.11;
    moon.eccentricity = 0.0549; moon.inclination = 5.145 * PI / 180.0; moon.orbital_period = 27.321 * 24.0 * 3600.0;
    UniverseModel::getInstance().solar_system.push_back(moon);
    
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
    mars.surface_pressure = 6.36; mars.average_temp = 210.0; mars.scattering_coef = 0.25;
    mars.eccentricity = 0.0934; mars.inclination = 1.85 * PI / 180.0; mars.orbital_period = 686.98 * 24.0 * 3600.0;
    UniverseModel::getInstance().solar_system.push_back(mars);
    
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
    jupiter.surface_pressure = 1.0e6; jupiter.average_temp = 165.0; jupiter.scattering_coef = 0.343;
    jupiter.eccentricity = 0.0485; jupiter.inclination = 1.3 * PI / 180.0; jupiter.orbital_period = 4332.59 * 24.0 * 3600.0;
    UniverseModel::getInstance().solar_system.push_back(jupiter);
    
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
    saturn.surface_pressure = 1.0e6; saturn.average_temp = 134.0; saturn.scattering_coef = 0.342;
    saturn.eccentricity = 0.0556; saturn.inclination = 2.48 * PI / 180.0; saturn.orbital_period = 10759.22 * 24.0 * 3600.0;
    UniverseModel::getInstance().solar_system.push_back(saturn);
    
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
    uranus.surface_pressure = 1.0e6; uranus.average_temp = 76.0; uranus.scattering_coef = 0.3;
    uranus.eccentricity = 0.0464; uranus.inclination = 0.77 * PI / 180.0; uranus.orbital_period = 30685.4 * 24.0 * 3600.0;
    UniverseModel::getInstance().solar_system.push_back(uranus);
    
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
    neptune.surface_pressure = 1.0e6; neptune.average_temp = 72.0; neptune.scattering_coef = 0.29;
    neptune.eccentricity = 0.0095; neptune.inclination = 1.77 * PI / 180.0; neptune.orbital_period = 60189.0 * 24.0 * 3600.0;
    UniverseModel::getInstance().solar_system.push_back(neptune);

    // 计算所有天体的引力范围 (SOI - Sphere of Influence)
    // SOI 是一个概念：在这个范围内，该天体的引力占据主导地位。
    // 我们使用拉普拉斯半径公式：r_soi = a * (m/M)^(2/5)
    // 其中 a 是半长轴，m 是行星质量，M 是太阳质量。
    for (size_t i = 1; i < UniverseModel::getInstance().solar_system.size(); i++) {
        if (i == 4) { // 月球特殊情况：它是绕着地球转的
            UniverseModel::getInstance().solar_system[i].soi_radius = UniverseModel::getInstance().solar_system[i].sma_base * std::pow(UniverseModel::getInstance().solar_system[i].mass / UniverseModel::getInstance().solar_system[3].mass, 2.0/5.0);
        } else {
            UniverseModel::getInstance().solar_system[i].soi_radius = UniverseModel::getInstance().solar_system[i].sma_base * std::pow(UniverseModel::getInstance().solar_system[i].mass / UniverseModel::getInstance().solar_system[0].mass, 2.0/5.0);
        }
    }
    UniverseModel::getInstance().solar_system[0].soi_radius = INFINITY; // 太阳是最终的主宰
    
    UpdateCelestialBodies(0.0);
}

// 获取天体在特定时间的 3D 状态 (位置和速度)
// 这个函数是物理引擎的核心。它使用“轨道根数”通过数学公式推导出星球在宇宙中的精确坐标。
// 即使在没有进行 1 步 0.02s 的物理迭代时，我们也能算出它在 100 年后的位置，这就是“解析解”的威力。
// 参数：i (星球索引), current_time_sec (游戏时间), px/py/pz (输出位置), vx/vy/vz (输出速度)
void GetCelestialStateAt(int i, double current_time_sec, double& px, double& py, double& pz, double& vx, double& vy, double& vz) {
    if (i < 0 || i >= (int)UniverseModel::getInstance().solar_system.size()) return;
    if (i == 0) { px=0; py=0; pz=0; vx=0; vy=0; vz=0; return; } // 太阳固定在原点

    const CelestialBody& b = UniverseModel::getInstance().solar_system[i];
    double T = current_time_sec / SECONDS_PER_JULIAN_CENTURY; // 转换为相对于 J2000 的世纪数
    
    // 1. 计算当前时刻的轨道根数 (考虑到长期的摄动变化)
    // 轨道根数（Orbital Elements）是描述轨道的 6 个基本参数。
    double a = b.sma_base + b.sma_rate * T; // 当前半长轴 (轨道大小)
    double e = b.ecc_base + b.ecc_rate * T; // 当前离心率 (轨道形状)
    double i_inc = b.inc_base + b.inc_rate * T; // 当前倾角 (轨道平面倾斜度)
    double lan = b.lan_base + b.lan_rate * T; // 升交点经度 (轨道平面旋转度)
    double arg_p = b.arg_peri_base + b.arg_peri_rate * T; // 近拱点幅角 (轨道在平面内的旋转度)
    double M = b.mean_anom_base + b.mean_anom_rate * current_time_sec; // 平近点角 (星球在轨道上的进度)
    
    // 2. 解开普勒方程：M = E - e * sin(E)
    // 这是天体力学中最著名的方程之一。由于物体在椭圆轨道上运行不是匀速的（近快远慢），
    // 我们不能直接算出它在哪里，必须通过牛顿迭代法求出偏近点角 E。
    M = std::fmod(M, 2.0 * PI);
    if (M < 0) M += 2.0 * PI;
    double E = M;
    for (int k = 0; k < 5; k++) {
        double dE = (E - e * std::sin(E) - M) / (1.0 - e * std::cos(E));
        E -= dE;
        if (std::abs(dE) < 1e-6) break;
    }
    
    // 3. 计算轨道平面内的位置 (Orbital Plane Coordinates)
    // 真近点角 nu：星球相对于近拱点的实际角度。
    double nu = 2.0 * std::atan2(std::sqrt(1.0 + e) * std::sin(E / 2.0), std::sqrt(1.0 - e) * std::cos(E / 2.0));
    double r = a * (1.0 - e * std::cos(E)); // 距离焦点的距离
    
    // 在轨道平面 (x-y) 上的位置
    double o_x = r * std::cos(nu);
    double o_y = r * std::sin(nu);
    
    // 计算速度向量
    // 使用活力公式 (Vis-Viva Equation) 的变形来求速度。
    double mu = (i == 4) ? (G_const * UniverseModel::getInstance().solar_system[3].mass) : (G_const * UniverseModel::getInstance().solar_system[0].mass);
    double p = a * (1.0 - e*e);
    double h_ang = std::sqrt(mu * p);
    double o_vx = -(mu / h_ang) * std::sin(nu);
    double o_vy = (mu / h_ang) * (e + std::cos(nu));
    
    // 4. 将平面坐标旋转到 3D 空间 (坐标系转换)
    // 我们需要通过 LAN, Inclination, Argument of Periapsis 三次旋转，
    // 把轨道平面内的 (o_x, o_y) 转换到全宇宙通用的惯性坐标系 (px, py, pz)。
    double c_O = std::cos(lan), s_O = std::sin(lan);
    double c_w = std::cos(arg_p), s_w = std::sin(arg_p);
    double c_i = std::cos(i_inc), s_i = std::sin(i_inc);
    
    // 旋转矩阵元素
    double x_x = c_O * c_w - s_O * s_w * c_i;
    double x_y = -c_O * s_w - s_O * c_w * c_i;
    double y_x = s_O * c_w + c_O * s_w * c_i;
    double y_y = -s_O * s_w + c_O * c_w * c_i;
    double z_x = s_w * s_i;
    double z_y = c_w * s_i;
    
    px = x_x * o_x + x_y * o_y;
    py = y_x * o_x + y_y * o_y;
    pz = z_x * o_x + z_y * o_y;
    vx = x_x * o_vx + x_y * o_vy;
    vy = y_x * o_vx + y_y * o_vy;
    vz = z_x * o_vx + z_y * o_vy;
    
    if (i == 4) { // 月球特殊处理：月球是绕着地球转的
        double epx, epy, epz, evx, evy, evz;
        // 先算出地球在太阳系里的位置
        GetCelestialStateAt(3, current_time_sec, epx, epy, epz, evx, evy, evz);
        // 月球相对于太阳的位置 = 地球相对于太阳的位置 + 月球相对于地球的位置
        px += epx; py += epy; pz += epz;
        vx += evx; vy += evy; vz += evz;
    }
}

void GetCelestialPositionAt(int i, double t, double& px, double& py, double& pz) {
    double vx, vy, vz;
    GetCelestialStateAt(i, t, px, py, pz, vx, vy, vz);
}

void UpdateCelestialBodies(double current_time_sec) {
    for (size_t i = 0; i < UniverseModel::getInstance().solar_system.size(); i++) {
        GetCelestialStateAt((int)i, current_time_sec, UniverseModel::getInstance().solar_system[i].px, UniverseModel::getInstance().solar_system[i].py, UniverseModel::getInstance().solar_system[i].pz, UniverseModel::getInstance().solar_system[i].vx, UniverseModel::getInstance().solar_system[i].vy, UniverseModel::getInstance().solar_system[i].vz);
    }
}

// 获取旋转坐标轴（四元数）
// 它定义了你是以什么“背景”来观察宇宙。
// ref_mode: 0-惯性系, 1-共同旋转系, 2-地表系
Quat GetFrameRotation(int ref_mode, int ref_body, int sec_body, double t) {
    if (ref_mode == 0) return Quat(1, 0, 0, 0); // 1. 惯性系 (Inertial)：背景星空不动，坐标轴固定。
    
    if (ref_mode == 2) { // 2. 地表系 (Surface)：坐标轴随着星球一起转。
        CelestialBody& body = UniverseModel::getInstance().solar_system[ref_body];
        // 计算当前时刻的自转角度
        double theta = body.prime_meridian_epoch + (t * 2.0 * PI / body.rotation_period);
        Quat rot = Quat::fromAxisAngle(Vec3(0, 0, 1), (float)theta); // 绕自转轴旋转
        Quat tilt = Quat::fromAxisAngle(Vec3(1, 0, 0), (float)body.axial_tilt); // 加上地轴倾角
        return tilt * rot; // 先旋转，再倾斜
    }
    
    if (ref_mode == 1) { // 3. 共同旋转系 (Co-rotating)：坐标轴始终指向目标星球。
        // 常用于对接或者是行星际航行，比如让 X 轴永远指向地球。
        if (sec_body < 0 || sec_body >= (int)UniverseModel::getInstance().solar_system.size() || sec_body == ref_body) return Quat(1, 0, 0, 0);
        double p1x, p1y, p1z, v1x, v1y, v1z;
        GetCelestialStateAt(ref_body, t, p1x, p1y, p1z, v1x, v1y, v1z);
        double p2x, p2y, p2z, v2x, v2y, v2z;
        GetCelestialStateAt(sec_body, t, p2x, p2y, p2z, v2x, v2y, v2z);
        
        Vec3 r_rel((float)(p2x - p1x), (float)(p2y - p1y), (float)(p2z - p1z));
        Vec3 v_rel((float)(v2x - v1x), (float)(v2y - v1y), (float)(v2z - v1z));
        
        Vec3 X = r_rel.normalized(); // X 轴指向目标
        Vec3 Z = r_rel.cross(v_rel).normalized(); // Z 轴垂直于轨道平面 (动量矩方向)
        if (Z.length() < 0.5f) Z = Vec3(0, 0, 1);
        Vec3 Y = Z.cross(X).normalized(); // Y 轴补全右手系
        
        // 从 3D 坐标轴向量构造四元数
        float m00 = X.x, m01 = Y.x, m02 = Z.x;
        float m10 = X.y, m11 = Y.y, m12 = Z.y;
        float m20 = X.z, m21 = Y.z, m22 = Z.z;
        float tr = m00 + m11 + m22;
        Quat q;
        if (tr > 0) {
            float S = std::sqrt(tr + 1.0f) * 2.0f; 
            q.w = 0.25f * S;
            q.x = (m21 - m12) / S;
            q.y = (m02 - m20) / S; 
            q.z = (m10 - m01) / S; 
        } else if ((m00 > m11) && (m00 > m22)) {
            float S = std::sqrt(1.0f + m00 - m11 - m22) * 2.0f; 
            q.w = (m21 - m12) / S;
            q.x = 0.25f * S;
            q.y = (m01 + m10) / S; 
            q.z = (m02 + m20) / S; 
        } else if (m11 > m22) {
            float S = std::sqrt(1.0f + m11 - m00 - m22) * 2.0f; 
            q.w = (m02 - m20) / S;
            q.x = (m01 + m10) / S; 
            q.y = 0.25f * S;
            q.z = (m12 + m21) / S; 
        } else {
            float S = std::sqrt(1.0f + m22 - m00 - m11) * 2.0f; 
            q.w = (m10 - m01) / S;
            q.x = (m02 + m20) / S;
            q.y = (m12 + m21) / S;
            q.z = 0.25f * S;
        }
        return q;
    }
    return Quat(1, 0, 0, 0);
}


// 检查引力范围 (SOI) 的切换
// 如果飞船飞离了地球太远，进入了月球或太阳的引力场，
// 我们就需要切换“参照系”，让坐标计算更稳定、更符合直觉。
// 这个逻辑让模拟器能够支持真正的行星际旅行（Interplanetary Travel）。
void CheckSOI_Transitions(entt::registry& registry, entt::entity entity) {

    auto& trans = registry.get<TransformComponent>(entity);
    auto& vel = registry.get<VelocityComponent>(entity);
    auto& tele = registry.get<TelemetryComponent>(entity);

    if (UniverseModel::getInstance().solar_system.empty()) return;

    // 获取当前中心天体
    CelestialBody& current_body = UniverseModel::getInstance().solar_system[UniverseModel::getInstance().current_soi_index];
    // 更新飞船在全太阳系（赫利奥西斯坐标系 / 日心系）下的绝对坐标
    trans.abs_px = current_body.px + trans.px;
    trans.abs_py = current_body.py + trans.py;
    trans.abs_pz = current_body.pz + trans.pz;
    
    // NaN 防御：如果绝对坐标出现 NaN，跳过 SOI 检查
    if (std::isnan(trans.abs_px) || std::isnan(trans.abs_py) || std::isnan(trans.abs_pz)) {
        return;
    }
    
    // 检查我们是否进入了任何星球的 SOI
    int best_soi = 0; // 默认退回到太阳 (太阳是最终的兜底 SOI)
    for (size_t i = 1; i < UniverseModel::getInstance().solar_system.size(); i++) {
        CelestialBody& b = UniverseModel::getInstance().solar_system[i];
        double dx = trans.abs_px - b.px;
        double dy = trans.abs_py - b.py;
        double dz = trans.abs_pz - b.pz;
        double dist = std::sqrt(dx*dx + dy*dy + dz*dz);
        // 如果距离小于该星球的 SOI 半径，说明我们被它成功“俘获”了
        if (dist < b.soi_radius) {
            best_soi = i;
        }
        // DEBUG: log Earth check
        if (i == 3) {
            static int soi_dbg=0;
            if(soi_dbg<20){ printf("[SOI] Earth: dist=%.0f soi=%.0f best=%d cur=%d\n",
                dist, b.soi_radius, best_soi, UniverseModel::getInstance().current_soi_index); soi_dbg++; }
        }
    }
    
    // 如果 SOI 发生了变化，我们需要进行坐标原点的平滑切换
    if (best_soi != UniverseModel::getInstance().current_soi_index) {
        printf("[SOI] TRANSITION %d->%d dist_to_earth=%.0f soi=%.0f\n",
            UniverseModel::getInstance().current_soi_index, best_soi,
            sqrt((trans.abs_px-UniverseModel::getInstance().solar_system[3].px)*(trans.abs_px-UniverseModel::getInstance().solar_system[3].px)+
                 (trans.abs_py-UniverseModel::getInstance().solar_system[3].py)*(trans.abs_py-UniverseModel::getInstance().solar_system[3].py)+
                 (trans.abs_pz-UniverseModel::getInstance().solar_system[3].pz)*(trans.abs_pz-UniverseModel::getInstance().solar_system[3].pz)),
            UniverseModel::getInstance().solar_system[3].soi_radius);
        CelestialBody& new_body = UniverseModel::getInstance().solar_system[best_soi];
        // 计算当前在日心系下的绝对速度
        double h_vx = current_body.vx + vel.vx;
        double h_vy = current_body.vy + vel.vy;
        double h_vz = current_body.vz + vel.vz;
        
        // 关键步骤：重新计算相对于新中心天体的位置和速度
        // px_new = px_abs - px_new_body
        trans.px = trans.abs_px - new_body.px;
        trans.py = trans.abs_py - new_body.py;
        trans.pz = trans.abs_pz - new_body.pz;
        // vx_new = vx_abs - vx_new_body
        vel.vx = h_vx - new_body.vx;
        vel.vy = h_vy - new_body.vy;
        vel.vz = h_vz - new_body.vz;
        
        UniverseModel::getInstance().current_soi_index = best_soi;
        std::cout << ">> [SOI TRANSITION] ENTERED " << new_body.name << " SOI" << std::endl;
    }
}

double CalculateSolarOcclusion(entt::registry& registry, entt::entity entity) {

    auto& trans = registry.get<TransformComponent>(entity);
    auto& vel = registry.get<VelocityComponent>(entity);
    auto& tele = registry.get<TelemetryComponent>(entity);

    if (UniverseModel::getInstance().solar_system.empty()) return 1.0;
    
    double min_occlusion = 1.0;
    
    // Ray from rocket to Sun (Sun is at 0,0,0)
    double dx = -trans.abs_px, dy = -trans.abs_py, dz = -trans.abs_pz;
    double dist_to_sun = std::sqrt(dx*dx + dy*dy + dz*dz);
    double dir_x = dx / dist_to_sun;
    double dir_y = dy / dist_to_sun;
    double dir_z = dz / dist_to_sun;
    
    for (size_t i = 1; i < UniverseModel::getInstance().solar_system.size(); i++) {
        CelestialBody& b = UniverseModel::getInstance().solar_system[i];
        
        double bx = b.px - trans.abs_px;
        double by = b.py - trans.abs_py;
        double bz = b.pz - trans.abs_pz;
        
        double proj = bx * dir_x + by * dir_y + bz * dir_z;
        if (proj > 0 && proj < dist_to_sun) {
            double px = bx - proj * dir_x;
            double py = by - proj * dir_y;
            double pz = bz - proj * dir_z;
            double pass_dist = std::sqrt(px*px + py*py + pz*pz);
            
            // 计算遮挡率：
            // 如果飞船和太阳之间有一颗星球，那么太阳光就会被遮住（进入阴影/日食）。
            if (pass_dist < b.radius * 1.05) { 
                double occ = (pass_dist - b.radius) / (b.radius * 0.05);
                occ = std::max(0.0, std::min(1.0, occ));
                min_occlusion = std::min(min_occlusion, occ);
            }
        }
    }
    return min_occlusion;
}

// 计算引力加速度：g = GM / r^2
// 这里的 r 是飞船到星球中心的距离。
// 注意：该函数返回标量加速度，用于计算受力大小。
double get_gravity(double r) { 
    CelestialBody& current_body = UniverseModel::getInstance().solar_system[UniverseModel::getInstance().current_soi_index];
    double GM = G_const * current_body.mass;
    return GM / (r * r);
}

// 计算大气压力 (Static Pressure)
// 这是一个极度简化的气压模型：P = P0 * exp(-h / H)
// 其中 H 是标高 (Scale Height)，地球上大约是 7 7000 米。
double get_pressure(double h) {
    if (h > 100000) return 0; // 100km 以上视为真空（卡门线以上）
    if (h < 0) return SLP;
    return SLP * std::exp(-h / 7000.0); 
}

// 计算空气密度 (Air Density)：由高度决定的指数衰减。
// 空气密度不仅受到高度影响，理论上还受气温影响，但这里采用标准指数模型。
// 密度 rho 决定了动压 (Dynamic Pressure)，进而决定了空气阻力。
double get_air_density(double h) {
    if (UniverseModel::getInstance().solar_system.empty()) {
        if (h > 100000) return 0;
        return 1.225 * std::exp(-h / 7000.0);
    }
    const CelestialBody& body =
        UniverseModel::getInstance().solar_system[UniverseModel::getInstance().current_soi_index];
    return AerodynamicsSystem::AirDensityAt(body, h);
}

// 获取轨道核心参数：远拱点 (AP) 和近拱点 (PE)
// 这个函数实现了如何从飞船的“状态向量”(位置 r, 速度 v) 提取出“轨道根数”。
void getOrbitParams(entt::registry& registry, entt::entity entity, double& apoapsis, double& periapsis) {

    auto& trans = registry.get<TransformComponent>(entity);
    auto& vel = registry.get<VelocityComponent>(entity);
    auto& tele = registry.get<TelemetryComponent>(entity);

    if (UniverseModel::getInstance().solar_system.empty()) return;
    CelestialBody& current_body = UniverseModel::getInstance().solar_system[UniverseModel::getInstance().current_soi_index];
    
    // 1. 获取距离 r 和速度 v 的模长 (Magnitude)
    double r = std::sqrt(trans.px * trans.px + trans.py * trans.py + trans.pz * trans.pz);
    double v_sq = vel.vx * vel.vx + vel.vy * vel.vy + vel.vz * vel.vz;
    double mu = G_const * current_body.mass; // 标准引力参数

    // 2. 计算比轨道能量 (Specific Orbital Energy)
    // 能量 E = (v^2 / 2) - (mu / r)
    // - E < 0: 闭合椭圆轨道
    // - E = 0: 抛物线轨道 (逃逸速度)
    // - E > 0: 双曲线轨道
    double energy = v_sq / 2.0 - mu / r; 
    
    // 3. 计算比角动量向量 (Specific Angular Momentum): h = r x v
    double hx = trans.py * vel.vz - trans.pz * vel.vy;
    double hy = trans.pz * vel.vx - trans.px * vel.vz;
    double hz = trans.px * vel.vy - trans.py * vel.vx;
    double h_sq = hx * hx + hy * hy + hz * hz;

    // 4. 计算离心率 e (Eccentricity)
    // 通过能量和角动量计算离心率向量的长度。
    double e_sq = 1.0 + 2.0 * energy * h_sq / (mu * mu);
    double e = (e_sq > 0) ? std::sqrt(e_sq) : 0;

    if (energy >= 0) { // 逃逸轨道 (双曲线/抛物线)
        apoapsis = 999999999; // 逃逸了，不再由于闭合的远拱点
        periapsis = (h_sq / mu) / (1.0 + e) - current_body.radius; 
    } else {           // 闭合轨道 (椭圆)
        double a = -mu / (2.0 * energy); // 计算轨道半长轴 a
        apoapsis = a * (1.0 + e) - current_body.radius; // AP = a * (1 + e)
        periapsis = a * (1.0 - e) - current_body.radius; // PE = a * (1 - e)
    }
}

// 核心步进更新函数 (Update)
// 每一帧（通常是 0.02秒）都会进入这个函数，它是模拟器最忙碌的地方。
void Update(entt::registry& registry, entt::entity entity, double dt) {

    auto& trans = registry.get<TransformComponent>(entity);
    auto& vel = registry.get<VelocityComponent>(entity);
    auto& att = registry.get<AttitudeComponent>(entity);
    auto& prop = registry.get<PropulsionComponent>(entity);
    auto& tele = registry.get<TelemetryComponent>(entity);
    auto& guid = registry.get<GuidanceComponent>(entity);
    auto& orb = registry.get<OrbitComponent>(entity);
    auto& mnv = registry.get<ManeuverComponent>(entity);
    auto& vfx = registry.get<VFXComponent>(entity);
    const auto& config = registry.get<RocketConfig>(entity);
    const auto& input = registry.get<ControlInput>(entity);

    if (UniverseModel::getInstance().solar_system.empty()) return;
    PhysicsStepSnapshot snapshot{trans, vel, att, prop, tele, guid};
    
    // 累加模拟时间。
    tele.sim_time += dt;
    // 如果任务已经开始（不是待发射状态），更新任务计时器。
    if (guid.status != PRE_LAUNCH) {
        guid.mission_timer += dt;
    }
    
    // 1. 同步所有天体的位置
    // 确保我们用于物理计算的星球位置是当前 sim_time 的最新值。
    UpdateCelestialBodies(tele.sim_time);
    
    // 2. 检查引力范围转换 (SOI Transition)
    // 非常重要：在每一帧开始前检查 SOI。
    // 这保证了后续所有海拔、引力、阻力计算都基于正确的参考星体。
    CheckSOI_Transitions(registry, entity);
    
    // 获取最新的中心天体引用
    CelestialBody& current_body = UniverseModel::getInstance().solar_system[UniverseModel::getInstance().current_soi_index];
    sanitizeDynamicState(trans, vel, att, prop, tele, config, current_body, tele.sim_time);

    // 3. 处理地面逻辑 (Ground Handling)
    // 待发射/已着陆时先随地表旋转；是否离地稍后由实际推力和接触反力决定。
    bool grounded_locked = (guid.status == PRE_LAUNCH);
    if (grounded_locked) {
        lockEntityToSurface(trans, vel, tele, current_body, tele.sim_time);
    }
    // CRASHED 当前只作为任务/显示状态保留；没有碎裂系统前不让它冻结物理。

    // 4. 基础状态刷新
    // 计算相对于星体中心的高度
    double r_3d = std::sqrt(trans.px * trans.px + trans.py * trans.py + trans.pz * trans.pz);
    tele.altitude = r_3d - current_body.radius + config.bounds_bottom;
    
    // 更新日食/遮挡状态
    tele.solar_occlusion = CalculateSolarOcclusion(registry, entity);

    // B. Force Analysis
    // 5. 受力分析 (Force Analysis)
    // 飞船的总质量 = 结构质量 + 当前燃料质量 + 后方级段的质量
    double total_mass = config.dry_mass + prop.fuel + config.upper_stages_mass;
    // NaN 防御（skip_builder 空配置下可能产生 NaN fuel）
    if (std::isnan(total_mass) || total_mass <= 0.0) total_mass = 1.0;

    // 6. 推力计算 (Thrust Calculation)
    prop.thrust_power = 0;
    if (prop.fuel > 0) {
        // 真空推力 = ISP * g0 * 质量流量 (cosrate)
        double max_thrust = config.specific_impulse * G0 * config.cosrate;
        // 压力损失：在稠密大气中，喷口内外的压力差会降低实际推力。
        // 公式：F_actual = F_vacuum - P_atm * A_nozzle
        double pressure_loss = get_pressure(tele.altitude) * config.nozzle_area;
        double throttle = std::isfinite(input.throttle)
            ? std::max(0.0, std::min(1.0, input.throttle))
            : 0.0;
        double current_thrust = throttle * max_thrust - pressure_loss;
        
        if (current_thrust < 0) current_thrust = 0;
        prop.thrust_power = current_thrust;

        // 计算质量流量 (m_dot)：每秒消耗多少燃料
        // m_dot = F / (Isp * g0)
        double m_dot = 0.0;
        if (prop.thrust_power > 0.0 && config.specific_impulse > 0.0) {
            m_dot = prop.thrust_power / (config.specific_impulse * G0);
        }
        prop.fuel -= m_dot * dt;
        
        // 分级燃料同步
        if (prop.current_stage < (int)prop.stage_fuels.size()) {
            prop.stage_fuels[prop.current_stage] = prop.fuel;
        }
        prop.fuel_consumption_rate = m_dot; 
    } else {
        prop.thrust_power = 0;
        prop.fuel_consumption_rate = 0;
    }

    // 7. 3D 几何坐标轴更新 (Geometric Frame)
    // 根据飞船相对于星球中心的当前位置，确定“本地上方向”。
    double r_mag_geo = std::sqrt(trans.px*trans.px + trans.py*trans.py + trans.pz*trans.pz);
    if (!std::isfinite(r_mag_geo) || r_mag_geo < 1.0) {
        sanitizeDynamicState(trans, vel, att, prop, tele, config, current_body, tele.sim_time);
        r_mag_geo = std::sqrt(trans.px*trans.px + trans.py*trans.py + trans.pz*trans.pz);
    }
    double Ux = trans.px / r_mag_geo;
    double Uy = trans.py / r_mag_geo;
    double Uz = trans.pz / r_mag_geo;

    // 计算本地坐标轴 (Up, Right, North)，用于后续的姿态控制辅助计算。
    double r_xy_mag_geo = std::sqrt(Ux*Ux + Uy*Uy);
    double Rx = 0, Ry = 0, Rz = 0;
    if (r_xy_mag_geo > 1e-6) {
        Rx = -Uy / r_xy_mag_geo;
        Ry = Ux / r_xy_mag_geo;
        Rz = 0.0;
    } else {
        Rx = 1.0; Ry = 0.0; Rz = 0.0;
    }
    double Nx = Uy * Rz - Uz * Ry;
    double Ny = Uz * Rx - Ux * Rz;
    double Nz = Ux * Ry - Uy * Rx;
    double N_mag_geo = std::sqrt(Nx*Nx + Ny*Ny + Nz*Nz);
    Nx /= N_mag_geo; Ny /= N_mag_geo; Nz /= N_mag_geo;

    // 8. 姿态角更新 (3D 姿态四元数)
    // 飞船的方向不再使用简单的欧拉角（Yaw, Pitch, Roll），
    // 而是使用四元数 (Quaternion)，这可以完全避免“万向节死锁”问题。
    if (!att.initialized || guid.status == PRE_LAUNCH) {
        // 初始对准：让火箭垂直于地面。
        Vec3 initialUp = Vec3((float)Ux, (float)Uy, (float)Uz);
        Vec3 defaultUp(0, 1, 0);
        Vec3 axis = defaultUp.cross(initialUp);
        float dot = defaultUp.dot(initialUp);
        Quat base;
        if (axis.length() > 1e-6f) {
            base = Quat::fromAxisAngle(axis.normalized(), std::acos(std::fmax(-1.0f, std::fmin(1.0f, dot))));
        } else if (dot < -0.99f) {
            // 翻转 180 度的情况
            base = Quat::fromAxisAngle(Vec3(1, 0, 0), (float)PI);
        }
        Quat q_pitch = Quat::fromAxisAngle(Vec3(1, 0, 0), (float)att.angle); 
        Quat q_yaw = Quat::fromAxisAngle(Vec3(0, 0, 1), (float)att.angle_z);
        Quat q_roll = Quat::fromAxisAngle(Vec3(0, 1, 0), (float)att.angle_roll);
        att.attitude = base * q_yaw * q_pitch * q_roll;
        att.initialized = true;
    }

    // 应用本地角速度：更新四元数
    // 四元数的变化率 dq/dt = 0.5 * q * omega
    Vec3 local_rot_vel(att.ang_vel_z, att.ang_vel_roll, att.ang_vel);
    if (local_rot_vel.lengthSq() > 1e-12f) {
        double rot_mag = local_rot_vel.length();
        Quat dq = Quat::fromAxisAngle(local_rot_vel / rot_mag, rot_mag * dt);
        att.attitude = (att.attitude * dq).normalized();
        if (!finiteQuat(att.attitude)) {
            restoreSnapshot(snapshot, trans, vel, att, prop, tele, guid);
            return;
        }
    }

    // 9. 获取推力向量
    // 根据当前的 3D 姿态（四元数）直接导出推力的 3D 方向。
    Vec3 thrust_dir = att.attitude.forward();
    double Ft_x = prop.thrust_power * thrust_dir.x;
    double Ft_y = prop.thrust_power * thrust_dir.y;
    double Ft_z = prop.thrust_power * thrust_dir.z;

    if (grounded_locked) {
        Vec3 pos(trans.px, trans.py, trans.pz);
        Vec3 groundNormal = pos.lengthSq() > 1e-9 ? pos.normalized() : Vec3(0, 1, 0);
        double r_ground = std::max(1.0, pos.length());
        double local_g = G_const * current_body.mass / (r_ground * r_ground);
        double upward_thrust = Vec3(Ft_x, Ft_y, Ft_z).dot(groundNormal);
        if (upward_thrust <= total_mass * local_g * 1.02) {
            lockEntityToSurface(trans, vel, tele, current_body, tele.sim_time);
            return;
        }
        guid.status = ASCEND;
        guid.has_lifted_off = false;
        guid.mission_msg = (input.throttle > 0.01) ? "LIFTOFF!" : "CONTACT RELEASED";
        grounded_locked = false;
    }

    // 3. Aerodynamic Drag & RK4 Integration
    double v_sq = vel.vx * vel.vx + vel.vy * vel.vy + vel.vz * vel.vz;
    double v_mag = std::sqrt(v_sq);
    double local_up_angle = std::atan2(trans.py, trans.px);

    // 计算加速度：这是物理模拟中最耗时也最重要的部分。
    // 计算原理：F = m * a -> a = F / m
    // 总力 F = 引力 + 推力 + 空气阻力
    auto calc_accel = [&](double temp_px, double temp_py, double temp_pz, double temp_vx, double temp_vy, double temp_vz, double& out_ax, double& out_ay, double& out_az) {
        // (1) 母星引力 (Newton's Law of Universal Gravitation)
        // 遵循平方反比定律。
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
        
        // (2) 其它天体的微扰引力 (N-body Perturbation)
        // 虽然我们在某行星的 SOI 内，但远方的恒星和其它行星依然会有微弱影响。
        // 这让轨道更真实，支持复杂的引力助推（Gravity Assist）计算。
        for (size_t i = 0; i < UniverseModel::getInstance().solar_system.size(); i++) {
            if (i == (size_t)UniverseModel::getInstance().current_soi_index) continue;
            CelestialBody& body = UniverseModel::getInstance().solar_system[i];
            
            // 计算其它天体到飞船的相对位置
            double dx = body.px - (current_body.px + temp_px);
            double dy = body.py - (current_body.py + temp_py);
            double dz = body.pz - (current_body.pz + temp_pz);
            double d_sq = dx*dx + dy*dy + dz*dz;
            double d = std::sqrt(d_sq);
            
            // 计算其它天体对中心天体的引力 (Tidal Effect / Indirect Term)
            // 这部分很有深度：我们需要的是“差异引力”，否则整个坐标系会一起平移。
            double dx_c = body.px - current_body.px;
            double dy_c = body.py - current_body.py;
            double dz_c = body.pz - current_body.pz;
            double dc_sq = dx_c*dx_c + dy_c*dy_c + dz_c*dz_c;
            double dc = std::sqrt(dc_sq);
            
            double common = G_const * body.mass * total_mass;
            Fgx += common * (dx / (d_sq * d) - dx_c / (dc_sq * dc));
            Fgy += common * (dy / (d_sq * d) - dy_c / (dc_sq * dc));
            Fgz += common * (dz / (d_sq * d) - dz_c / (dc_sq * dc));
        }
        
        // (3) 气动力 (Aerodynamics)
        // 独立气动系统基于整船体素截面和面积律估算阻力/体升力。
        AerodynamicsSystem::Sample aero = AerodynamicsSystem::ComputeForces(
            config,
            att,
            current_body,
            tele.sim_time,
            Vec3(temp_px, temp_py, temp_pz),
            Vec3(temp_vx, temp_vy, temp_vz));
        double Fdx = aero.force.x;
        double Fdy = aero.force.y;
        double Fdz = aero.force.z;
        out_ax = (Fgx + Ft_x + Fdx) / std::max(total_mass, 1.0);
        out_ay = (Fgy + Ft_y + Fdy) / std::max(total_mass, 1.0);
        out_az = (Fgz + Ft_z + Fdz) / std::max(total_mass, 1.0);
    };

    Vec3 previousCom((float)trans.px, (float)trans.py, (float)trans.pz);

    // --- 11. 核心算法：RK4 (Runge-Kutta 4th Order) 积分 ---
    // 简单的 "速度 += 加速度 * 时间" 会有很大的误差（欧拉积分）。
    // 在轨道力学中，误差会随时间呈指数级累积。
    // RK4 通过在当前的 1/2 步和 1 步位置进行 4 次采样，
    // 大大提高了模拟的稳定性，让轨道不会因为数值误差而慢慢“飘走”。
    // (1) 步骤 1：在当前位置采样
    double k1_vx, k1_vy, k1_vz, k1_px, k1_py, k1_pz;
    calc_accel(trans.px, trans.py, trans.pz, vel.vx, vel.vy, vel.vz, k1_vx, k1_vy, k1_vz);
    k1_px = vel.vx; k1_py = vel.vy; k1_pz = vel.vz;

    // (2) 步骤 2：在 1/2 dt 步长处采样（基于步骤 1 的斜率）
    double k2_vx, k2_vy, k2_vz, k2_px, k2_py, k2_pz;
    calc_accel(trans.px + 0.5 * dt * k1_px, trans.py + 0.5 * dt * k1_py, trans.pz + 0.5 * dt * k1_pz, 
               vel.vx + 0.5 * dt * k1_vx, vel.vy + 0.5 * dt * k1_vy, vel.vz + 0.5 * dt * k1_vz, 
               k2_vx, k2_vy, k2_vz);
    k2_px = vel.vx + 0.5 * dt * k1_vx; k2_py = vel.vy + 0.5 * dt * k1_vy; k2_pz = vel.vz + 0.5 * dt * k1_vz;
    // (3) 步骤 3：再次在 1/2 dt 步长处采样（基于步骤 2 的斜率）
    double k3_vx, k3_vy, k3_vz, k3_px, k3_py, k3_pz;
    calc_accel(trans.px + 0.5 * dt * k2_px, trans.py + 0.5 * dt * k2_py, trans.pz + 0.5 * dt * k2_pz, 
               vel.vx + 0.5 * dt * k2_vx, vel.vy + 0.5 * dt * k2_vy, vel.vz + 0.5 * dt * k2_vz, 
               k3_vx, k3_vy, k3_vz);
    k3_px = vel.vx + 0.5 * dt * k2_vx; k3_py = vel.vy + 0.5 * dt * k2_py; k3_pz = vel.vz + 0.5 * dt * k2_vz;

    // (4) 步骤 4：在 1.0 dt 步长处采样（基于步骤 3 的斜率）
    double k4_vx, k4_vy, k4_vz, k4_px, k4_py, k4_pz;
    calc_accel(trans.px + dt * k3_px, trans.py + dt * k3_py, trans.pz + dt * k3_pz, 
               vel.vx + dt * k3_vx, vel.vy + dt * k3_vy, vel.vz + dt * k3_vz, 
               k4_vx, k4_vy, k4_vz);
    k4_px = vel.vx + dt * k3_vx; k4_py = vel.vy + dt * k3_vy; k4_pz = vel.vz + dt * k3_vz;

    vel.vx += (dt / 6.0) * (k1_vx + 2.0 * k2_vx + 2.0 * k3_vx + k4_vx);
    vel.vy += (dt / 6.0) * (k1_vy + 2.0 * k2_vy + 2.0 * k3_vy + k4_vy);
    vel.vz += (dt / 6.0) * (k1_vz + 2.0 * k2_vz + 2.0 * k3_vz + k4_vz);
    trans.px += (dt / 6.0) * (k1_px + 2.0 * k2_px + 2.0 * k3_px + k4_px);
    trans.py += (dt / 6.0) * (k1_py + 2.0 * k2_py + 2.0 * k3_py + k4_py);
    trans.pz += (dt / 6.0) * (k1_pz + 2.0 * k2_pz + 2.0 * k3_pz + k4_pz);
    
    // 非有限检测：RK4 后位置/速度必须保持有效，避免 NaN 传播到碰撞、推力和 HUD。
    if (!finiteVec(Vec3(trans.px, trans.py, trans.pz)) ||
        !finiteVec(Vec3(vel.vx, vel.vy, vel.vz)) ||
        !finiteVec(Vec3(k4_vx, k4_vy, k4_vz))) {
        restoreSnapshot(snapshot, trans, vel, att, prop, tele, guid);
        return;
    }
    
    // 12. 更新派生物理量
    vel.acceleration = std::sqrt(k4_vx * k4_vx + k4_vy * k4_vy + k4_vz * k4_vz); 
    // 垂直速度（上升/下降率）
    tele.velocity = vel.vx * Ux + vel.vy * Uy + vel.vz * Uz;
    // 地表水平速度：考虑到星球自转的相对速度
    double surface_rotation_speed = (2.0 * PI / current_body.rotation_period) * std::sqrt(trans.px * trans.px + trans.py * trans.py);
    tele.local_vx = (-vel.vx * std::sin(local_up_angle) + vel.vy * std::cos(local_up_angle)) - surface_rotation_speed;

    // 13. 多点刚体接触求解 (Collision & Final Sync)
    GroundCollisionSystem::ContactResult contact =
        GroundCollisionSystem::ResolveGroundContacts(config, prop, trans, vel, att, guid, tele,
                                                     current_body, total_mass, tele.sim_time, dt, previousCom);

    Vec3 postPos(trans.px, trans.py, trans.pz);
    Vec3 postNormal = postPos.lengthSq() > 1e-9 ? postPos.normalized() : Vec3(0, 1, 0);
    double terrainH = GroundCollisionSystem::SampleTerrainHeightMeters(postNormal, tele);
    double com_alt = postPos.length() - current_body.radius;
    double current_alt_f = com_alt - terrainH + config.bounds_bottom;
    tele.altitude = (contact.contactCount > 0) ? 0.0 : std::max(0.0, current_alt_f);

    Vec3 relVelPost = Vec3(vel.vx, vel.vy, vel.vz) -
        GroundCollisionSystem::SurfaceVelocityAt(current_body, tele.sim_time, postPos);
    tele.velocity = relVelPost.dot(postNormal);
    tele.local_vx = (relVelPost - postNormal * tele.velocity).length();

    if (contact.contactCount == 0 && guid.status != PRE_LAUNCH) {
        guid.has_lifted_off = true;
    }

    if (contact.contactCount > 0) {
        if (contact.hardImpact) {
            guid.status = (tele.velocity > 0.5) ? ASCEND : DESCEND;
            GroundCollisionSystem::FreezeSurfacePoint(trans, current_body, tele.sim_time);
        } else if (contact.stable) {
            GroundCollisionSystem::FreezeSurfacePoint(trans, current_body, tele.sim_time);
            if (guid.status == LANDED || guid.status == DESCEND || guid.has_lifted_off) {
                guid.status = LANDED;
                guid.mission_msg = "LANDED - CONTACTS STABLE";
            } else if (guid.status == ASCEND) {
                guid.mission_msg = (input.throttle > 0.01)
                    ? "IGNITION - HOLDING CONTACT"
                    : "GROUND CONTACT - WAITING FOR LIFTOFF";
            }
        } else {
            guid.status = (tele.velocity > 0.5) ? ASCEND : DESCEND;
            guid.mission_msg = (contact.maxTangentSpeed > 2.0)
                ? "GROUND CONTACT - SLIDING"
                : "GROUND CONTACT - BOUNCING";
            GroundCollisionSystem::FreezeSurfacePoint(trans, current_body, tele.sim_time);
        }
    } else {
        if (guid.status == LANDED) {
            guid.status = ASCEND;
        } else if (guid.status != PRE_LAUNCH) {
            guid.status = (tele.velocity < -0.5) ? DESCEND : ASCEND;
        }
        GroundCollisionSystem::FreezeSurfacePoint(trans, current_body, tele.sim_time);
    }
    // 不在这里因为 CRASHED 跳出；硬碰撞仍然继续姿态/运动更新。

    // 15. 角速度更新 (Angular Velocity)
    // 根据指令力矩和转动惯量更新旋转速度。
    double base_moi = 50000.0;
    double moment_of_inertia = base_moi * (total_mass / 50000.0); 
    // 气动阻尼力矩由独立气动系统估算。
    double aero_torque =
        AerodynamicsSystem::AngularDampingCoefficient(config, current_body, tele.altitude, v_mag);
    att.ang_vel_z += (input.torque_cmd_z / moment_of_inertia + att.ang_vel_z * aero_torque) * dt;
    att.ang_vel += (input.torque_cmd / moment_of_inertia + att.ang_vel * aero_torque) * dt;
    att.ang_vel_roll += (input.torque_cmd_roll / moment_of_inertia + att.ang_vel_roll * aero_torque) * dt;

    // 如果在真空中且没有操作，角速度会由于数值稳定性而极缓慢衰减
    if (tele.altitude > 80000 && std::abs(input.torque_cmd)<0.1 && std::abs(input.torque_cmd_z)<0.1 && std::abs(input.torque_cmd_roll)<0.1) {
        att.ang_vel *= std::pow(0.99, dt); att.ang_vel_z *= std::pow(0.99, dt); att.ang_vel_roll *= std::pow(0.99, dt);
    }

    // 16. 同步旧版角度数据 (Legacy Sync for HUD)
    // 为了兼容现有的仪表盘 UI，我们需要将四元数转回欧拉角。
    Vec3 fwd_sync = att.attitude.forward();
    att.angle = std::atan2(fwd_sync.dot(Vec3((float)Rx,(float)Ry,(float)Rz)), fwd_sync.dot(Vec3((float)Ux,(float)Uy,(float)Uz)));
    att.angle_z = std::asin(std::fmax(-1.0, std::fmin(1.0, (double)fwd_sync.dot(Vec3((float)Nx,(float)Ny,(float)Nz)))));
    Vec3 local_up_s = att.attitude.rotate(Vec3(0, 0, 1));
    Vec3 world_top_s = Vec3((float)Ux,(float)Uy,(float)Uz).cross(fwd_sync).normalized();
    if (world_top_s.length() > 0.1f) att.angle_roll = std::atan2(local_up_s.dot(world_top_s.cross(fwd_sync).normalized()), local_up_s.dot(world_top_s));

    if (!stateIsFinite(trans, vel, att, prop, tele)) {
        restoreSnapshot(snapshot, trans, vel, att, prop, tele, guid);
        return;
    }
    syncAbsoluteState(trans, vel, current_body);
    if (!stateIsFinite(trans, vel, att, prop, tele)) {
        restoreSnapshot(snapshot, trans, vel, att, prop, tele, guid);
        return;
    }
}

// 高倍速率引力更新 (FastGravityUpdate)
// 当用户开启 100x, 1000x 甚至更高倍的时间加速时，
// 我们不再计算空气阻力、姿态控制或推力，只计算纯粹的引力。
// 这样可以极大地提高计算效率，防止 CPU 过载导致游戏卡死。
void FastGravityUpdate(entt::registry& registry, entt::entity entity, double dt_total) {

    auto& trans = registry.get<TransformComponent>(entity);
    auto& vel = registry.get<VelocityComponent>(entity);
    auto& att = registry.get<AttitudeComponent>(entity);
    auto& prop = registry.get<PropulsionComponent>(entity);
    auto& tele = registry.get<TelemetryComponent>(entity);
    auto& guid = registry.get<GuidanceComponent>(entity);
    auto& orb = registry.get<OrbitComponent>(entity);
    auto& mnv = registry.get<ManeuverComponent>(entity);
    auto& vfx = registry.get<VFXComponent>(entity);
    const auto& config = registry.get<RocketConfig>(entity);
    const auto& input = registry.get<ControlInput>(entity);

    // 1. 处理高倍率下的地面锁定
    // 如果飞船在地面上，直接根据星球自转同步位置。
    if (guid.status == PRE_LAUNCH) {
        tele.sim_time += dt_total;
        if (guid.status != PRE_LAUNCH) {
            guid.mission_timer += dt_total;
        }
        UpdateCelestialBodies(tele.sim_time);
        
        CelestialBody& current_body = UniverseModel::getInstance().solar_system[UniverseModel::getInstance().current_soi_index];
        lockEntityToSurface(trans, vel, tele, current_body, tele.sim_time);
        
        CheckSOI_Transitions(registry, entity);
        return;
    }

    // 2. 动态步长调整
    // 防止在高倍加速（如 10000x）下每帧计算太多步。
    double dt_step = std::max(5.0, dt_total / 200.0); 
    double t_remaining = dt_total;
    
    double total_mass = config.dry_mass + prop.fuel + config.upper_stages_mass;

    // 循环迭代：将大步长拆解为物理上可接受的小步长
    while (t_remaining > 0) {
        double dt = std::min(t_remaining, dt_step);
        t_remaining -= dt;
        tele.sim_time += dt;
        if (guid.status != PRE_LAUNCH) {
            guid.mission_timer += dt;
        }
        
        // 更新天体位置和 SOI
        UpdateCelestialBodies(tele.sim_time);
        CheckSOI_Transitions(registry, entity);
      
        // 高速引力计算闭包 (只考虑重力)
        auto calc_accel_fast = [&](double temp_px, double temp_py, double temp_pz, double temp_time, double& out_ax, double& out_ay, double& out_az) {
            CelestialBody& current_body = UniverseModel::getInstance().solar_system[UniverseModel::getInstance().current_soi_index];
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
            
            // N-body 叠加
            for (size_t i = 0; i < UniverseModel::getInstance().solar_system.size(); i++) {
                if (i == (size_t)UniverseModel::getInstance().current_soi_index) continue;
                CelestialBody& body = UniverseModel::getInstance().solar_system[i];
                
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

        // 在这里同样使用 RK4 积分，保持高速轨道下的高精度
        double k1_vx, k1_vy, k1_vz, k1_px, k1_py, k1_pz;
        calc_accel_fast(trans.px, trans.py, trans.pz, tele.sim_time - dt, k1_vx, k1_vy, k1_vz);
        k1_px = vel.vx; k1_py = vel.vy; k1_pz = vel.vz;

        double k2_vx, k2_vy, k2_vz, k2_px, k2_py, k2_pz;
        calc_accel_fast(trans.px + 0.5 * dt * k1_px, trans.py + 0.5 * dt * k1_py, trans.pz + 0.5 * dt * k1_pz, tele.sim_time - dt + 0.5 * dt, k2_vx, k2_vy, k2_vz);
        k2_px = vel.vx + 0.5 * dt * k1_vx; k2_py = vel.vy + 0.5 * dt * k1_vy; k2_pz = vel.vz + 0.5 * dt * k1_vz;

        double k3_vx, k3_vy, k3_vz, k3_px, k3_py, k3_pz;
        calc_accel_fast(trans.px + 0.5 * dt * k2_px, trans.py + 0.5 * dt * k2_py, trans.pz + 0.5 * dt * k2_pz, tele.sim_time - dt + 0.5 * dt, k3_vx, k3_vy, k3_vz);
        k3_px = vel.vx + 0.5 * dt * k2_vx; k3_py = vel.vy + 0.5 * dt * k2_vy; k3_pz = vel.vz + 0.5 * dt * k2_vz;

        double k4_vx, k4_vy, k4_vz, k4_px, k4_py, k4_pz;
        calc_accel_fast(trans.px + dt * k3_px, trans.py + dt * k3_py, trans.pz + dt * k3_pz, tele.sim_time, k4_vx, k4_vy, k4_vz);
        k4_px = vel.vx + dt * k3_vx; k4_py = vel.vy + dt * k3_vy; k4_pz = vel.vz + dt * k3_vz;

        vel.vx += (dt / 6.0) * (k1_vx + 2.0 * k2_vx + 2.0 * k3_vx + k4_vx);
        vel.vy += (dt / 6.0) * (k1_vy + 2.0 * k2_vy + 2.0 * k3_vy + k4_vy);
        vel.vz += (dt / 6.0) * (k1_vz + 2.0 * k2_vz + 2.0 * k3_vz + k4_vz);
        trans.px += (dt / 6.0) * (k1_px + 2.0 * k2_px + 2.0 * k3_px + k4_px);
        trans.py += (dt / 6.0) * (k1_py + 2.0 * k2_py + 2.0 * k3_py + k4_py);
        trans.pz += (dt / 6.0) * (k1_pz + 2.0 * k2_pz + 2.0 * k3_pz + k4_pz);
        
        // 3. 高倍率碰撞检测
        CelestialBody& body_now = UniverseModel::getInstance().solar_system[UniverseModel::getInstance().current_soi_index];
        Vec3 pos_now(trans.px, trans.py, trans.pz);
        Vec3 normal_now = pos_now.lengthSq() > 1e-9 ? pos_now.normalized() : Vec3(0, 1, 0);
        double terrain_now = GroundCollisionSystem::SampleTerrainHeightMeters(normal_now, tele);
        double current_com_alt = pos_now.length() - body_now.radius;
        if (current_com_alt + config.bounds_bottom <= terrain_now) {
            tele.altitude = 0.0;
            guid.status = DESCEND;
            guid.mission_msg = "HARD IMPACT: WARP TERRAIN CONTACT";
            vel.vx = 0; vel.vy = 0; vel.vz = 0;
            double targetR = body_now.radius + terrain_now - config.bounds_bottom;
            if (pos_now.lengthSq() > 1e-9) {
                Vec3 corrected = normal_now * targetR;
                trans.px = corrected.x;
                trans.py = corrected.y;
                trans.pz = corrected.z;
            }
            syncAbsoluteState(trans, vel, body_now);
            break;
        }
    }

    if (!UniverseModel::getInstance().solar_system.empty()) {
        CelestialBody& body_now = UniverseModel::getInstance().solar_system[UniverseModel::getInstance().current_soi_index];
        syncAbsoluteState(trans, vel, body_now);
    }
}

// 产生烟雾粒子 (EmitSmoke)
// 当引擎开启时，在喷管处生成粒子，增强视觉效果。
void EmitSmoke(entt::registry& registry, entt::entity entity, double dt) {

    auto& trans = registry.get<TransformComponent>(entity);
    auto& vel = registry.get<VelocityComponent>(entity);
    auto& att = registry.get<AttitudeComponent>(entity);
    auto& prop = registry.get<PropulsionComponent>(entity);
    auto& tele = registry.get<TelemetryComponent>(entity);
    auto& guid = registry.get<GuidanceComponent>(entity);
    auto& orb = registry.get<OrbitComponent>(entity);
    auto& mnv = registry.get<ManeuverComponent>(entity);
    auto& vfx = registry.get<VFXComponent>(entity);
    const auto& config = registry.get<RocketConfig>(entity);
    const auto& input = registry.get<ControlInput>(entity);

    if (prop.thrust_power < 1000.0) return;
    
    // 计算喷口在世界坐标系下的位置
    double local_up = std::atan2(trans.py, trans.px);
    double nozzle_dir = local_up + att.angle + PI;
    double nozzle_wx = trans.px + std::cos(nozzle_dir) * 20.0;
    double nozzle_wy = trans.py + std::sin(nozzle_dir) * 20.0;
    
    // 一次产生 3 个粒子
    for (int k = 0; k < 3; k++) {
        SmokeParticle& p = vfx.smoke[vfx.smoke_idx % VFXComponent::MAX_SMOKE];
        // 使用哈希函数产生伪随机偏移，让烟雾看起来更自然
        float rnd1 = hash11(vfx.smoke_idx * 1337 + k * 997) - 0.5f;
        float rnd2 = hash11(vfx.smoke_idx * 7919 + k * 773) - 0.5f;
        p.wx = nozzle_wx + rnd1 * 15.0;
        p.wy = nozzle_wy + rnd2 * 15.0;
        
        // 粒子的初始速度 (喷出速度)
        double exhaust_speed = 30.0 + hash11(vfx.smoke_idx * 3571 + k) * 20.0;
        p.vwx = std::cos(nozzle_dir) * exhaust_speed + rnd1 * 10.0;
        p.vwy = std::sin(nozzle_dir) * exhaust_speed + rnd2 * 10.0;
        p.alpha = 0.6f;
        p.size = 10.0f + hash11(vfx.smoke_idx * 4567 + k) * 8.0f;
        p.life = 1.0f;
        p.active = true;
        vfx.smoke_idx++;
    }
}

// 更新烟雾粒子 (UpdateSmoke)
// 让粒子随时间漂移、膨胀并逐渐消失。
void UpdateSmoke(entt::registry& registry, entt::entity entity, double dt) {
    auto& vfx = registry.get<VFXComponent>(entity);
    auto& trans = registry.get<TransformComponent>(entity);
    auto& vel = registry.get<VelocityComponent>(entity);
    for (int i = 0; i < VFXComponent::MAX_SMOKE; i++) {
        SmokeParticle& p = vfx.smoke[i];
        if (!p.active) continue;
        
        // 寿命缩减，透明度降低
        p.life -= (float)(dt * 0.25);
        p.alpha = std::min(0.25f, p.life * 0.3f); 
        // 粒子随时间扩散变大
        p.size += (float)(dt * 20.0);

        // 位置累加
        p.wx += p.vwx * dt;
        p.wy += p.vwy * dt;

        // 地面碰撞：如果烟雾撞到地表，发生反弹扩散
        double r = std::sqrt(p.wx * p.wx + p.wy * p.wy);
        double planet_r = UniverseModel::getInstance().solar_system[UniverseModel::getInstance().current_soi_index].radius;
        if (r < planet_r && r > 0) {
            p.wx = p.wx / r * planet_r;
            p.wy = p.wy / r * planet_r;
            double nx = p.wx / r, ny = p.wy / r;
            double v_radial = p.vwx * nx + p.vwy * ny;
            double v_tang = -p.vwx * ny + p.vwy * nx;
            v_radial = std::abs(v_radial) * 0.3; // 垂直分量反弹削弱
            float rnd_dir = (hash11(i * 8731) - 0.5f) * 2.0f;
            v_tang = std::abs(v_tang) * (1.5f + rnd_dir) * (rnd_dir > 0 ? 1.0 : -1.0);
            p.vwx = nx * v_radial - ny * v_tang;
            p.vwy = ny * v_radial + nx * v_tang;
            p.size += 5.0f;
        }

        // 微弱的向心引力，让烟雾稍微下沉
        if (r > 0) {
            p.vwx += (p.wx / r) * dt * 8.0;
            p.vwy += (p.wy / r) * dt * 8.0;
        }
        // 空气阻力：减慢粒子速度
        p.vwx *= (1.0 - dt * 0.8); 
        p.vwy *= (1.0 - dt * 0.8);

        if (p.life <= 0) p.active = false;
    }
}

} // namespace PhysicsSystem
