/* 
 * 物理系统 (PhysicsSystem) 模块
 * -----------------------------------------
 * 这是 RocketSim3D 的心脏，负责模拟宇宙中的一切物理法则。
 * 它不仅处理火箭的飞行，还管理着行星的运动、引力场、大气模拟以及
 * 跨越不同引力范围（SOI - Sphere of Influence）的逻辑切换。
 * 
 * 对于新手来说，理解物理引擎的工作原理是掌握仿真模拟器的关键。
 * 这里的每一行代码都对应着现实世界中的科学原理。
 */

#ifndef PHYSICS_SYSTEM_H
#define PHYSICS_SYSTEM_H

#include "core/rocket_state.h"
#include "math/math3d.h"

/**
 * @namespace PhysicsSystem
 * @brief 包含所有物理计算函数的命名空间。
 */
namespace PhysicsSystem {
    
    // --- 数学与环境工具函数 (Math & Environment) ---

    /**
     * @brief 计算给定距离下的引力加速度 (g)。
     * 公式：g = GM / r^2
     * @param r 距离星体中心的距离 (单位：米)。
     * @return 该距离下的引力场强度 (单位：m/s^2)。
     */
    double get_gravity(double r);

    /**
     * @brief 计算给定高度的大气压 (Static Pressure)。
     * 在不同的行星上，大气压力的分布模型不同，这里使用简化的指数衰减模型。
     * @param h 海拔高度 (单位：米)。
     * @return 静态大气压 (单位：帕斯卡 Pa 或 毫巴 hPa，取决于具体实现)。
     */
    double get_pressure(double h);

    /**
     * @brief 计算给定高度的空气密度 (Air Density)。
     * 空气密度决定了火箭飞行时的空气阻力 (Drag) 大小。
     * @param h 海拔高度 (单位：米)。
     * @return 空气密度 (单位：kg/m^3)。
     */
    double get_air_density(double h);

    /**
     * @brief 获取当前轨道的远拱点 (Apoapsis) 和近拱点 (Periapsis)。
     * 这是轨道力学的核心参数：
     * - 远拱点：轨道上离中心天体最远的点。
     * - 近拱点：轨道上离中心天体最近的点。
     * @param state 飞船当前的物理状态 (位置、速度等)。
     * @param apoapsis [输出] 远拱点高度 (单位：米)。
     * @param periapsis [输出] 近拱点高度 (单位：米)。
     */
    void getOrbitParams(const RocketState& state, double& apoapsis, double& periapsis);

    // --- 行星历表与坐标转换 (Planetary Ephemeris & Coordinates) ---

    /**
     * @brief 初始化整个太阳系。
     * 设置太阳、地球、月球、火星等各天体的物理常数（质量、半径）及轨道根数。
     */
    void InitSolarSystem();

    /**
     * @brief 更新所有天体在当前时间点的宇宙坐标。
     * 这是一个基于时间的解析解更新（Analytical Update），保证了长期运动的准确性。
     * @param current_time_sec 游戏开始后的累计物理时间。
     */
    void UpdateCelestialBodies(double current_time_sec);

    /**
     * @brief 获取特定天体在某一时刻 of 3D 位置。
     * @param body_idx 天体的索引（例如：0 为太阳，3 为地球）。
     * @param t 时间。
     * @param px, py, pz [输出] 世界坐标系下的位置数据。
     */
    void GetCelestialPositionAt(int body_idx, double t, double& px, double& py, double& pz);

    /**
     * @brief 获取天体的完整状态（位置 + 速度）。
     * 用于计算飞船相对于天体的相对速度，或是进行轨道交会计算。
     * @param body_idx 天体索引。
     * @param t 时间。
     * @param bpx, bpy, bpz [输出] 位置。
     * @param bvx, bvy, bvz [输出] 速度向量。
     */
    void GetCelestialStateAt(int body_idx, double t, double& bpx, double& bpy, double& bpz, double& bvx, double& bvy, double& bvz);

    /**
     * @brief 获取特定参照系下的旋转四元数。
     * 这个函数非常关键，它定义了你是以什么视角在看宇宙：
     * - 惯性系 (Inertial)：背景星空不动。
     * - 地表系 (Surface)：随星球自转，让你能计算相对于地面的速度。
     */
    Quat GetFrameRotation(int ref_mode, int ref_body, int sec_body, double t);

    /**
     * @brief 检查并处理引力范围 (SOI) 的切换。
     * 比如当你离开地球飞向月球，这个函数会自动检测你是否进入了月球的引力统治区。
     * 切换 SOI 是为了防止在大尺度下出现坐标抖动和数值计算误差。
     */
    void CheckSOI_Transitions(RocketState& state);

    /**
     * @brief 计算太阳遮挡率 (Solar Occlusion)。
     * 简单来说就是检查有没有星球挡在你和太阳中间。
     * 用于渲染光影、电池板充电逻辑或是大气散射效果。
     * @return 遮挡百分比 (0 到 1)。
     */
    double CalculateSolarOcclusion(const RocketState& state);
    
    // --- 核心仿真更新逻辑 (Core Simulation Update) ---

    /**
     * @brief 每一帧的物理步进更新函数。
     * 这是物理引擎的主循环，执行力学分析、数值积分、姿态计算和碰撞检测。
     * @param state 飞船状态。
     * @param config 飞船的配置（质量分布、引擎参数等）。
     * @param input 控制输入（油门、旋转等）。
     * @param dt 步进时间 (Delta Time)，通常为固定的 0.02s，以保证数值稳定性。
     */
    void Update(RocketState& state, const RocketConfig& config, const ControlInput& input, double dt);

    /**
     * @brief 精简版的快速引力更新，常用于预测未来的轨迹（轨迹预测线）。
     */
    void FastGravityUpdate(RocketState& state, const RocketConfig& config, double dt_total);
    
    // --- 视觉效果集成 (Visual Effects) ---

    /**
     * @brief 尾迹/烟雾发射逻辑。
     * 根据引擎推力的大小，在喷口处产生视觉粒子。
     */
    void EmitSmoke(RocketState& state, const RocketConfig& config, double dt);

    /**
     * @brief 更新现有的烟雾粒子。
     */
    void UpdateSmoke(RocketState& state, double dt);
}

#endif // PHYSICS_SYSTEM_H
