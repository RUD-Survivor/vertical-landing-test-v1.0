#pragma once
#include <thread>
#include <atomic>
#include <vector>
#include "core/rocket_state.h"

namespace Simulation {

// 异步轨道预测器 (AsyncOrbitPredictor)
// 该类负责在后台线程中预计算火箭未来的飞行轨迹（轨道预测）。
// 这样做可以避免在主线程进行耗时的物理积分，确保游戏画面流畅。
class AsyncOrbitPredictor {
public:
    AsyncOrbitPredictor();
    ~AsyncOrbitPredictor();

    // 启动/停止后台预测线程
    void Start();
    void Stop();

    // 请求一次新的轨道预测更新
    // 参数包含：预测时长 (pred_days)、采样迭代次数 (iters)、参考系模式 (ref_mode) 等。
    void RequestUpdate(RocketState* target, const RocketState& state, const RocketConfig& config, double pred_days, int iters, int ref_mode, int ref_body, int secondary_ref_body, bool force_reset = false);

    // 检查预测器是否正在忙碌（后台计算中）
    bool IsBusy() const { return m_busy; }

private:
    void WorkerLoop(); // 后台工作线程循环

    // 预测请求结构 (Prediction Request)
    // 包含了启动一次物理积分所需的所有初始条件
    struct PredictionRequest {
        RocketState* target = nullptr; // 预测结果最终要写入的目标状态对象
        RocketState state;             // 初始状态（位置、速度、燃料等）
        RocketConfig config;           // 火箭配置（质量、推力、比冲）
        double pred_days;              // 需要预测的时间跨度（天）
        int iters;                     // 积分采样点数量
        int ref_mode;                  // 坐标参考系模式
        int ref_body;                  // 主参考天体
        int secondary_ref_body;        // 次要参考天体
        std::vector<CelestialBody> celestial_snapshot; // 太阳系天体位置的快照
        bool force_reset = false;      // 是否强制重置预测缓存
        bool pending = false;          // 是否有待处理的更新请求
    };

    // 预测上下文 (Prediction Context)
    // 存储了积分计算的中间状态，支持断点继续预测和变轨模拟。
    struct PredContext {
        double t_epoch = -1.0;  // 本次预测序列开始时的仿真时间
        double t_last = -1.0;   // 当前积分缓冲区结束的时间点
        // 纯惯性外推状态 (Heliocentric State)
        double px, py, pz;      
        double vx, vy, vz;
        // 受操控影响的预测状态 (Maneuvered Orbit)
        // 用于模拟执行“计划中的变轨”后的轨迹
        double mnv_px, mnv_py, mnv_pz; 
        double mnv_vx, mnv_vy, mnv_vz;
        
        std::vector<Vec3> points;            // 预测的路径点集合
        std::vector<double> point_times;     // 每个路径点对应的时间戳
        std::vector<Vec3> mnv_points;        // 变轨后的路径点
        std::vector<double> mnv_point_times;
        
        bool mnv_done = false;               // 变轨计算是否已完成
        bool mnv_burning = false;            // 是否正在进行有限时长点火模拟
        double mnv_remaining_dv = 0;         // 剩余的增量速度 (Delta-V)
        double mnv_mass = 0;                 // 变轨过程中的实时质量变化
        std::vector<ManeuverNode> last_maneuvers; // 用户的变轨计划列表
        Vec3 mnv_thrust_dir;                 // 点火时的推力锁定方向
        
        int last_ref_body = -1;
        int last_secondary_ref_body = -1;
        
        // 轨道特征点追踪 (Apsis Tracking)
        // 记录近地点 (Periapsis) 和远地点 (Apoapsis)
        std::vector<RocketState::Apsis> apsides;
        std::vector<RocketState::Apsis> mnv_apsides;
        double last_r = -1.0;
        double last_dr = 0.0;
        double last_mnv_r = -1.0;
        double last_mnv_dr = 0.0;
        
        // 初始状态记录，用于平滑性校验
        double init_px, init_py, init_pz;
        double init_vx, init_vy, init_vz;
        bool crashed = false;     // 是否坠毁
        bool mnv_crashed = false; // 变轨轨迹是否坠毁
    };

    std::thread m_worker;               // 工作线程
    std::atomic<bool> m_running;        // 线程运行标志
    std::atomic<bool> m_busy;           // 忙碌状态
    
    std::mutex m_request_mutex;         // 请求互斥锁
    PredictionRequest m_request;        // 当前挂起的请求
    PredContext m_context;              // 当前计算上下文
    
    std::condition_variable m_cv;       // 条件变量，用于线程间协作
};

} // namespace Simulation

