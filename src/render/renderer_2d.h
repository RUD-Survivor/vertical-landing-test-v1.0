#pragma once
// ==========================================================
// renderer_2d.h — Vulkan-only stub (2D HUD 由 ImGui/VkHUD 处理)
// ==========================================================
#include <vector>
#include <cmath>
#include <string>
#include <map>
#include <algorithm>
#include "core/rocket_state.h"
#include "math/math3d.h"

#ifndef PI
#define PI 3.14159265358979323846f
#endif

// Utility functions
inline float my_lerp(float a, float b, float t) { return a + t * (b - a); }
inline float fract(float x) { return x - std::floor(x); }
inline float hash11(float p) {
    p = fract(p * 0.1031f);
    p *= p + 33.33f;
    p *= p + p;
    return fract(p);
}

inline Vec2 rotateVec(float x, float y, float angle) {
  float s = sinf(angle), c = cosf(angle);
  return {x * c - y * s, x * s + y * c};
}

// ==========================================
// Renderer — 最小化桩（Vulkan: UI 由 ImGui 处理）
// ==========================================
class Renderer {
public:
  enum Align { LEFT, CENTER, RIGHT };

  void beginFrame() { vertices.clear(); }
  Renderer() {}
  ~Renderer() {}

  void addVertex(float x, float y, float r, float g, float b, float a, float=0, float=0) {}
  void addVertexWhite(float, float, float, float, float, float) {}
  void addRect(float, float, float, float, float, float, float, float=1.0f) {}
  void addRectOutline(float, float, float, float, float, float, float, float=1.0f, float=0.002f) {}
  void addLine(float, float, float, float, float, float, float, float, float=1.0f) {}
  void addCircle(float, float, float, float, float, float, float, int=32) {}
  void addCircleOutline(float, float, float, float, float, float, float, float=1.0f, int=32) {}
  void addAtmosphereGlow(float, float, float, float) {}
  void addTriangle(float, float, float, float, float, float, float, float, float, float) {}

  void drawInt(float, float, int, float=0.02f, float=1,float=1,float=1,float=1,bool=false,Align=CENTER) {}
  void drawText(float x, float y, const std::string&, float=0.02f,
                float=1,float=1,float=1,float=1,bool=false,Align=CENTER) { (void)x; (void)y; }
  void drawTextWrapped(float, float, const std::string&, float, float,
                       float=1,float=1,float=1,float=1,bool=false,Align=CENTER) {}
  void drawPlanetIcon(float, float, float, const CelestialBody&, float) {}
  void drawAttitudeSphere(float, float, float, const Quat&, Vec3, Vec3, Vec3, bool, bool, Vec3, Vec3, Vec3, Vec3) {}
  void addRotatedTri(float, float, float, float, float, float, float, float, float) {}

  void endFrame() {}

private:
  std::vector<float> vertices;
};
