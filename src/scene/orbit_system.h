#pragma once
#include <vector>
#include "math/math3d.h"
struct FlightHUD;
// OrbitSystem — Vulkan stub
class OrbitSystem {
public:
    std::vector<Vec3> cached_rel_pts;
    std::vector<Vec3> cached_mnv_rel_pts;
    size_t last_draw_points_size = 0;
    size_t last_draw_mnv_points_size = 0;
    Vec3 last_first_pt = {0,0,0}, last_mnv_first_pt = {0,0,0};
    struct DVec3 { double x,y,z; };
    struct TrajPoint { DVec3 e; DVec3 s; };
    std::vector<TrajPoint> traj_history;
    
    // Vulkan: orbit rendering handled by VkEffectsSystem; these are no-ops
    template<typename... Args> void render(Args&&...) {}
    template<typename... Args> void recordTrajectory(Args&&...) {}
    template<typename... Args> void extractRibbons(Args&&...) {}
};
