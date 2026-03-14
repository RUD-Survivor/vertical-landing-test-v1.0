#pragma once
#include <thread>
#include <atomic>
#include <vector>
#include "core/rocket_state.h"

namespace Simulation {

class AsyncOrbitPredictor {
public:
    AsyncOrbitPredictor();
    ~AsyncOrbitPredictor();

    void Start();
    void Stop();

    // Requests a new prediction based on the provided state
    void RequestUpdate(RocketState* target, const RocketState& state, double pred_days, int iters, int ref_mode, int ref_body, int secondary_ref_body, bool force_reset = false);

    // Checks if the predictor is busy
    bool IsBusy() const { return m_busy; }

private:
    void WorkerLoop();

    struct PredictionRequest {
        RocketState* target = nullptr;
        RocketState state;
        double pred_days;
        int iters;
        int ref_mode;
        int ref_body;
        int secondary_ref_body;
        std::vector<CelestialBody> celestial_snapshot;
        bool force_reset = false; // New flag to force clear
        bool pending = false;
    };

    struct PredContext {
        double t_epoch = -1.0;  // Sim time when this sequence started
        double t_last = -1.0;   // Sim time at the end of current buffer
        double px, py, pz;      // Absolute heliocentric state at t_last (pure prediction)
        double vx, vy, vz;
        double mnv_px, mnv_py, mnv_pz; // Absolute heliocentric state for maneuvered orbit
        double mnv_vx, mnv_vy, mnv_vz;
        std::vector<Vec3> points;
        std::vector<Vec3> mnv_points;
        bool mnv_done = false;
        std::vector<ManeuverNode> last_maneuvers;
        int last_ref_body = -1;
        int last_secondary_ref_body = -1;
        
        // Initial state at t_epoch for continuity checks
        double init_px, init_py, init_pz;
        double init_vx, init_vy, init_vz;
    };

    std::thread m_worker;
    std::atomic<bool> m_running;
    std::atomic<bool> m_busy;
    
    std::mutex m_request_mutex;
    PredictionRequest m_request;
    PredContext m_context;
    
    std::condition_variable m_cv;
};

} // namespace Simulation
