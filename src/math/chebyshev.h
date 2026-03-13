#pragma once

#include "math3d.h"
#include <vector>
#include <cmath>
#include <algorithm>

/**
 * ChebyshevFitting provides a way to fit a series of 3D points
 * to a Chebyshev polynomial approximation for smooth interpolation.
 */
class ChebyshevFitting {
public:
    ChebyshevFitting() = default;

    /**
     * Fit a set of points to a Chebyshev polynomial of a given degree.
     * points: The input points to fit.
     * startIdx, endIdx: The range of points [startIdx, endIdx) to fit.
     * degree: The degree of the polynomial.
     */
    void fit(const std::vector<Vec3>& points, int startIdx, int endIdx, int degree) {
        int n = endIdx - startIdx;
        if (n < 2) return;
        
        m_degree = std::min(degree, n);
        m_coeffs.assign(m_degree, Vec3(0, 0, 0));

        for (int k = 0; k < m_degree; ++k) {
            Vec3 sum(0, 0, 0);
            for (int i = 0; i < n; ++i) {
                // Map local index i to t in [0, 1] then to x in [-1, 1]
                double t = (double)i / (n - 1);
                double x = 2.0 * t - 1.0;
                sum += points[startIdx + i] * (float)std::cos(k * std::acos(x));
            }
            m_coeffs[k] = sum * (2.0f / n);
        }
    }

    /**
     * Evaluate the fitted polynomial at a local normalized parameter t in [0, 1].
     */
    Vec3 evaluate(double t) const {
        if (m_coeffs.empty()) return Vec3(0, 0, 0);
        
        double x = 2.0 * t - 1.0;
        x = std::max(-1.0, std::min(1.0, x));

        // Clenshaw's algorithm
        Vec3 d1(0, 0, 0), d2(0, 0, 0);
        for (int j = m_degree - 1; j >= 1; --j) {
            Vec3 tmp = d1;
            d1 = m_coeffs[j] + (float)(2.0 * x) * d1 - d2;
            d2 = tmp;
        }
        return m_coeffs[0] * 0.5f + (float)x * d1 - d2;
    }

private:
    int m_degree = 0;
    std::vector<Vec3> m_coeffs;
};

/**
 * SegmentedChebyshev automatically splits a large trajectory into segments
 * and fits each individually, avoiding NASA-grade distortion (global aliasing).
 */
class SegmentedChebyshev {
public:
    /**
     * Fit continuous segments to the provided points.
     * pointsPerSegment: Roughly how many raw points to include in each polynomial fit.
     * degreePerSegment: Degree of the polynomial for each segment.
     */
    void fit(const std::vector<Vec3>& points, int pointsPerSegment = 15, int degreePerSegment = 10) {
        m_segments.clear();
        m_totalPoints = (int)points.size();
        if (m_totalPoints < 2) return;

        int numSegments = (m_totalPoints + pointsPerSegment - 2) / (pointsPerSegment - 1);
        if (numSegments < 1) numSegments = 1;

        for (int s = 0; s < numSegments; ++s) {
            int start = s * (pointsPerSegment - 1);
            int end = std::min(start + pointsPerSegment, m_totalPoints);
            if (end - start < 2) break;

            ChebyshevFitting fitter;
            fitter.fit(points, start, end, degreePerSegment);
            m_segments.push_back({(double)start / (m_totalPoints - 1), (double)(end - 1) / (m_totalPoints - 1), fitter});
        }
    }

    /**
     * Evaluate the global trajectory at t in [0, 1].
     */
    Vec3 evaluate(double t) const {
        if (m_segments.empty()) return Vec3(0, 0, 0);
        t = std::max(0.0, std::min(1.0, t));

        // Find the segment containing t
        for (const auto& seg : m_segments) {
            if (t >= seg.tStart && t <= seg.tEnd) {
                double localT = (t - seg.tStart) / (seg.tEnd - seg.tStart);
                return seg.fitter.evaluate(localT);
            }
        }
        return m_segments.back().fitter.evaluate(1.0);
    }

private:
    struct Segment {
        double tStart, tEnd;
        ChebyshevFitting fitter;
    };
    std::vector<Segment> m_segments;
    int m_totalPoints = 0;
};
