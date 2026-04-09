#pragma once
#include "core/rocket_state.h"
#include "math/math3d.h"
#include "render/renderer_2d.h"
#include <vector>

struct ManeuverFrame {
    Vec3 prograde;
    Vec3 normal;
    Vec3 radial;
    Vec3 position; // world relative to body
    Vec3 velocity; // world relative to body
};

class ManeuverSystem {
public:
    // Calculates the orbit frame (Prograde, Normal, Radial) at a given state
    static ManeuverFrame getFrame(const Vec3& pos, const Vec3& vel) {
        ManeuverFrame f;
        f.position = pos;
        f.velocity = vel;
        f.prograde = vel.normalized();
        Vec3 h = pos.cross(vel);
        f.normal = h.normalized();
        if (f.normal.length() < 0.001f) f.normal = Vec3(0, 0, 1);
        f.radial = f.normal.cross(f.prograde).normalized();
        return f;
    }

    // Projects a 3D point to 2D screen coordinates [-1, 1]
    static Vec2 projectToScreen(const Vec3& p_world, const Mat4& view, const Mat4& proj, float aspect) {
        Vec3 p_view = view.transformPoint(p_world);
        // We use orthographic or perspective depending on main.cpp, but here we assume perspective projection as per Mat4::perspective
        
        // Manual projection calculation to be safe
        float fov_y = 0.8f; // From main.cpp
        float tan_half_fov = tanf(fov_y * 0.5f);
        
        if (std::abs(p_view.z) < 1e-6) return Vec2(0, 0);
        
        float x = p_view.x / (-p_view.z * tan_half_fov * aspect);
        float y = p_view.y / (-p_view.z * tan_half_fov);
        
        return Vec2(x, y);
    }

    // Returns color for handle (0-5: Pro, Retro, Norm, Anti-Norm, Rad-Out, Rad-In)
    static Vec3 getHandleColor(int idx) {
        switch(idx) {
            case 0: return Vec3(1.0f, 0.9f, 0.1f); // Prograde: Yellow/Gold
            case 1: return Vec3(0.5f, 0.45f, 0.05f); // Retrograde: Dark Yellow
            case 2: return Vec3(1.0f, 0.1f, 1.0f); // Normal: Purple
            case 3: return Vec3(0.5f, 0.05f, 0.5f); // Anti-Normal: Dark Purple
            case 4: return Vec3(0.1f, 0.8f, 1.0f); // Radial Out: Cyan
            case 5: return Vec3(0.05f, 0.4f, 0.5f); // Radial In: Dark Cyan
            default: return Vec3(1, 1, 1);
        }
    }

    // Returns offset vector for handle in world space
    static Vec3 getHandleDir(const ManeuverFrame& frame, int idx) {
        switch(idx) {
            case 0: return frame.prograde;
            case 1: return -frame.prograde;
            case 2: return frame.normal;
            case 3: return -frame.normal;
            case 4: return frame.radial;
            case 5: return -frame.radial;
            default: return Vec3(0, 0, 0);
        }
    }

    // High-Precision Remaining Delta-V calculation (Shared by HUD and Autopilot)
    static Vec3 calculateRemainingDV(const VelocityComponent& vel, const TelemetryComponent& tele, const ManeuverNode& node) {
        if (!node.snap_valid) return Vec3(0, 0, 0);

        int ref_idx = (node.ref_body >= 0) ? node.ref_body : current_soi_index;
        const CelestialBody& ref_b = SOLAR_SYSTEM[ref_idx];
        
        // Current relative velocity in the reference body's frame
        // abs_v = rocket_rel_v + soi_v
        // target_rel_v = abs_v - ref_b_v
        double cur_rel_vx = vel.vx + SOLAR_SYSTEM[current_soi_index].vx - ref_b.vx;
        double cur_rel_vy = vel.vy + SOLAR_SYSTEM[current_soi_index].vy - ref_b.vy;
        double cur_rel_vz = vel.vz + SOLAR_SYSTEM[current_soi_index].vz - ref_b.vz;
        
        double mu_ref = 6.67430e-11 * ref_b.mass;
        double dt_snap = tele.sim_time - node.snap_time;
        double tpx, tpy, tpz, tvx, tvy, tvz;
        
        // Use Keplerian propagation to project the target state to the current time
        // This prevents "DV inflation" caused by just subtracting static vectors
        void get3DStateAtTime(double px, double py, double pz, double vx, double vy, double vz, double mu, double dt,
                              double& out_px, double& out_py, double& out_pz, double& out_vx, double& out_vy, double& out_vz);

        get3DStateAtTime(node.snap_rel_px, node.snap_rel_py, node.snap_rel_pz,
                         node.snap_rel_vx, node.snap_rel_vy, node.snap_rel_vz,
                         mu_ref, dt_snap, tpx, tpy, tpz, tvx, tvy, tvz);
        
        return Vec3((float)(tvx - cur_rel_vx), (float)(tvy - cur_rel_vy), (float)(tvz - cur_rel_vz));
    }
};
