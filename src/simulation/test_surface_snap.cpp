#include <iostream>
#include <vector>
#include <cmath>
#include <cassert>

// Mock Vec3 and Quat if not included
struct Vec3 {
    float x, y, z;
    Vec3(float x=0, float y=0, float z=0) : x(x), y(y), z(z) {}
    Vec3 operator+(const Vec3& v) const { return Vec3(x+v.x, y+v.y, z+v.z); }
    Vec3 operator-(const Vec3& v) const { return Vec3(x-v.x, y-v.y, z-v.z); }
    Vec3 operator*(float s) const { return Vec3(x*s, y*s, z*s); }
    float length() const { return sqrtf(x*x + y*y + z*z); }
    Vec3 normalized() const { float l = length(); return (l > 0.0001f) ? (*this * (1.0f/l)) : Vec3(); }
    float dot(const Vec3& v) const { return x*v.x + y*v.y + z*v.z; }
    Vec3 cross(const Vec3& v) const { return Vec3(y*v.z - z*v.y, z*v.x - x*v.z, x*v.y - y*v.x); }
};

struct Quat {
    float x, y, z, w;
    Quat(float x=0, float y=0, float z=0, float w=1) : x(x), y(y), z(z), w(w) {}
};

#define PI 3.1415926535f

// Minimal RocketSim3D definitions needed for the test
#include "simulation/rocket_builder.h"

int main() {
    RocketAssembly assembly;
    
    std::cout << "Testing Surface Snap Logic..." << std::endl;

    // 1. Add a central tank
    assembly.addPart(6); // Medium Tank 100t
    assembly.parts[0].pos = Vec3(0, 0, 0);
    assembly.recalculate();
    
    std::cout << "Added central tank at (0,0,0). Height: " << PART_CATALOG[6].height << ", Diameter: " << PART_CATALOG[6].diameter << std::endl;

    // 2. Test surface snap for a Fin Set (Part ID 16)
    // Part size: diameter 3.7, height 2.0
    // Try to snap near the middle of the tank surface
    Vec3 mousePos(4.0f, 15.0f, 0.0f); // Near the side (r=1.85)
    auto snap = assembly.findSurfaceSnap(16, mousePos, Vec3(-1, 0, 0));

    std::cout << "Snap test (Fin near tank):" << std::endl;
    std::cout << "  Parent Index: " << snap.parent_idx << std::endl;
    std::cout << "  Snap Score: " << snap.score << std::endl;
    std::cout << "  Snap Position: (" << snap.pos.x << ", " << snap.pos.y << ", " << snap.pos.z << ")" << std::endl;

    assert(snap.parent_idx == 0);
    assert(snap.score < 1.0f);
    assert(std::abs(snap.pos.x - 1.85f) < 0.01f);

    // 3. Test surface snap for new Radial Decoupler (Part ID 22)
    auto snap2 = assembly.findSurfaceSnap(22, Vec3(0.0f, 20.0f, 5.0f), Vec3(0, 0, -1));
    std::cout << "Snap test (Radial Decoupler near tank):" << std::endl;
    std::cout << "  Parent Index: " << snap2.parent_idx << std::endl;
    std::cout << "  Snap Position: (" << snap2.pos.x << ", " << snap2.pos.y << ", " << snap2.pos.z << ")" << std::endl;

    assert(snap2.parent_idx == 0);
    assert(std::abs(snap2.pos.z - 1.85f) < 0.01f);

    // 4. Test snap to Engine (Part ID 9, Raptor) - Should now be supported
    assembly.addPart(9, 0); // Attach engine to tank
    assembly.parts[1].pos = Vec3(0, -4.0f, 0); // Below tank
    assembly.recalculate();

    auto snap3 = assembly.findSurfaceSnap(16, Vec3(2.5f, -2.0f, 0.0f), Vec3(-1, 0, 0));
    std::cout << "Snap test (Fin near engine):" << std::endl;
    std::cout << "  Parent Index: " << snap3.parent_idx << std::endl;
    std::cout << "  Snap Score: " << snap3.score << std::endl;

    assert(snap3.parent_idx == 1); // Should snap to engine
    assert(snap3.score < 1.0f);

    std::cout << "All surface snap tests PASSED!" << std::endl;
    return 0;
}
