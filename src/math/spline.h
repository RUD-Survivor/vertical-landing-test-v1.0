#pragma once

#include "math3d.h"
#include <vector>
#include <cmath>

/**
 * Centripetal Catmull-Rom Spline implementation.
 * Passes exactly through all points and avoids self-intersections/overshoot.
 */
class CatmullRomSpline {
public:
    /**
     * Compute a point on the spline segment between P1 and P2.
     * P0 and P3 are the surrounding control points.
     * t: normalized parameter in [0, 1] between P1 and P2.
     * alpha: 0.5 for Centripetal (NASA standard for trajectories), 0 for uniform, 1 for chordal.
     */
    static Vec3 solve(const Vec3& p0, const Vec3& p1, const Vec3& p2, const Vec3& p3, float t, float alpha = 0.5f) {
        auto get_t = [&](float t_prev, const Vec3& a, const Vec3& b) {
            float d = (b - a).lengthSq();
            return std::pow(d, alpha * 0.5f) + t_prev;
        };

        float t0 = 0.0f;
        float t1 = get_t(t0, p0, p1);
        float t2 = get_t(t1, p1, p2);
        float t3 = get_t(t2, p2, p3);

        // Map normalized t [0, 1] to global range [t1, t2]
        float global_t = t1 + t * (t2 - t1);

        Vec3 a1 = (t1 - global_t) / (t1 - t0) * p0 + (global_t - t0) / (t1 - t0) * p1;
        Vec3 a2 = (t2 - global_t) / (t2 - t1) * p1 + (global_t - t1) / (t2 - t1) * p2;
        Vec3 a3 = (t3 - global_t) / (t3 - t2) * p2 + (global_t - t2) / (t3 - t2) * p3;

        Vec3 b1 = (t2 - global_t) / (t2 - t0) * a1 + (global_t - t0) / (t2 - t0) * a2;
        Vec3 b2 = (t3 - global_t) / (t3 - t1) * a2 + (global_t - t1) / (t3 - t1) * a3;

        Vec3 c = (t2 - global_t) / (t2 - t1) * b1 + (global_t - t1) / (t2 - t1) * b2;

        return c;
    }

    /**
     * Generate a high-resolution smooth path from discrete points.
     */
    static std::vector<Vec3> interpolate(const std::vector<Vec3>& points, int segmentsPerKnot = 8) {
        if (points.size() < 2) return points;

        std::vector<Vec3> result;
        int n = (int)points.size();

        for (int i = 0; i < n - 1; ++i) {
            // Control points
            const Vec3& p0 = (i == 0) ? points[i] - (points[i+1] - points[i]) : points[i - 1];
            const Vec3& p1 = points[i];
            const Vec3& p2 = points[i + 1];
            const Vec3& p3 = (i == n - 2) ? points[i+1] + (points[i+1] - points[i]) : points[i + 2];

            for (int s = 0; s < segmentsPerKnot; ++s) {
                float t = (float)s / segmentsPerKnot;
                result.push_back(solve(p0, p1, p2, p3, t));
            }
        }
        result.push_back(points.back());
        return result;
    }
};
