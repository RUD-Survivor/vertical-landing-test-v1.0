#include <iostream>
#include <vector>
#include <cmath>
#include <iomanip>
#include "../src/math/chebyshev.h"

int main() {
    std::cout << "Testing Chebyshev Fitting..." << std::endl;

    // Create points on a semicircle
    std::vector<Vec3> points;
    const int num_points = 10;
    for (int i = 0; i < num_points; ++i) {
        float angle = (float)i / (num_points - 1) * 3.14159f;
        points.push_back(Vec3(std::cos(angle), std::sin(angle), 0));
    }

    std::cout << "Input points:" << std::endl;
    for (const auto& p : points) {
        std::cout << "(" << p.x << ", " << p.y << ", " << p.z << ")" << std::endl;
    }

    ChebyshevFitting fitter;
    fitter.fit(points, 5); // Use degree 5

    std::cout << "\nInterpolated points (higher resolution):" << std::endl;
    const int test_points = 20;
    for (int i = 0; i <= test_points; ++i) {
        double t = (double)i / test_points;
        Vec3 p = fitter.evaluate(t);
        std::cout << "t=" << std::fixed << std::setprecision(2) << t 
                  << " -> (" << std::setprecision(4) << p.x << ", " << p.y << ", " << p.z << ")" << std::endl;
    }

    // Check start and end points
    Vec3 start = fitter.evaluate(0.0);
    Vec3 end = fitter.evaluate(1.0);
    std::cout << "\nStart error: " << (start - points.front()).length() << std::endl;
    std::cout << "End error: " << (end - points.back()).length() << std::endl;

    return 0;
}
