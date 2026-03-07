#pragma once
#include <glad/glad.h>
#include <vector>
#include <cmath>
#include <string>
#include <iostream>

#ifndef PI
#define PI 3.14159265358979323846f
#endif

// ==========================================
// Part 0: 数学常量与辅助工具
// ==========================================

// 简单的二维向量(现统一使用 math3d.h 的 Vec2)
#include "math/math3d.h"

inline Vec2 rotateVec(float x, float y, float angle) {
  float s = sinf(angle);
  float c = cosf(angle);
  return {x * c - y * s, x * s + y * c};
}

inline float my_lerp(float a, float b, float t) { return a + t * (b - a); }

// Utility fract function
inline float fract(float x) { return x - std::floor(x); }

// Simple hash for procedural generation
inline float hash11(float p) {
    p = fract(p * 0.1031f);
    p *= p + 33.33f;
    p *= p + p;
    return fract(p);
}

// ==========================================
// Renderer Class: modern OpenGL GUI/HUD
// ==========================================

static const char *vertexShaderSource2D = R"(
    #version 330 core
    layout (location = 0) in vec2 aPos;
    layout (location = 1) in vec4 aColor;
    out vec4 ourColor;
    void main() {
        gl_Position = vec4(aPos.x, aPos.y, 0.0, 1.0);
        ourColor = aColor;
    }
)";

static const char *fragmentShaderSource2D = R"(
    #version 330 core
    out vec4 FragColor;
    in vec4 ourColor;
    void main() {
        FragColor = ourColor;
    }
)";

class Renderer {
private:
  unsigned int shaderProgram, VBO, VAO;
  std::vector<float> vertices;
  unsigned int compileShader(unsigned int type, const char *source) {
    unsigned int id = glCreateShader(type);
    glShaderSource(id, 1, &source, NULL);
    glCompileShader(id);
    return id;
  }

public:
  Renderer() {
    unsigned int v = compileShader(GL_VERTEX_SHADER, vertexShaderSource2D);
    unsigned int f = compileShader(GL_FRAGMENT_SHADER, fragmentShaderSource2D);
    shaderProgram = glCreateProgram();
    glAttachShader(shaderProgram, v);
    glAttachShader(shaderProgram, f);

    glLinkProgram(shaderProgram);
    glDeleteShader(v);
    glDeleteShader(f);

    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);
    glBindVertexArray(VAO);
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    // 预分配显存
    glBufferData(GL_ARRAY_BUFFER, sizeof(float) * 5000000, NULL,
                 GL_DYNAMIC_DRAW);

    // 位置属性 (location=0)
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 6 * sizeof(float),
                          (void *)0);
    glEnableVertexAttribArray(0);
    // 颜色属性 (location=1)
    glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, 6 * sizeof(float),
                          (void *)(2 * sizeof(float)));
    glEnableVertexAttribArray(1);
  }

  void beginFrame() { vertices.clear(); }

  void addVertex(float x, float y, float r, float g, float b, float a) {
    vertices.insert(vertices.end(), {x, y, r, g, b, a});
  }

  // 普通矩形
  void addRect(float x, float y, float w, float h, float r, float g, float b,
               float a = 1.0f) {
    addVertex(x - w / 2, y - h / 2, r, g, b, a);
    addVertex(x + w / 2, y - h / 2, r, g, b, a);
    addVertex(x + w / 2, y + h / 2, r, g, b, a);
    addVertex(x + w / 2, y + h / 2, r, g, b, a);
    addVertex(x - w / 2, y + h / 2, r, g, b, a);
    addVertex(x - w / 2, y - h / 2, r, g, b, a);
  }

  // 旋转矩形
  void addRotatedRect(float cx, float cy, float w, float h, float angle,
                      float r, float g, float b, float a = 1.0f) {
    Vec2 p1 = {-w / 2, -h / 2}, p2 = {w / 2, -h / 2}, p3 = {w / 2, h / 2},
         p4 = {-w / 2, h / 2};
    p1 = rotateVec(p1.x, p1.y, angle);
    p2 = rotateVec(p2.x, p2.y, angle);
    p3 = rotateVec(p3.x, p3.y, angle);
    p4 = rotateVec(p4.x, p4.y, angle);
    addVertex(cx + p1.x, cy + p1.y, r, g, b, a);
    addVertex(cx + p2.x, cy + p2.y, r, g, b, a);
    addVertex(cx + p3.x, cy + p3.y, r, g, b, a);
    addVertex(cx + p3.x, cy + p3.y, r, g, b, a);
    addVertex(cx + p4.x, cy + p4.y, r, g, b, a);
    addVertex(cx + p1.x, cy + p1.y, r, g, b, a);
  }

  // 旋转三角形
  void addRotatedTri(float cx, float cy, float w, float h, float angle, float r,
                     float g, float b, float a = 1.0f) {
    Vec2 p1 = {0, h / 2}, p2 = {-w / 2, -h / 2}, p3 = {w / 2, -h / 2};
    p1 = rotateVec(p1.x, p1.y, angle);
    p2 = rotateVec(p2.x, p2.y, angle);
    p3 = rotateVec(p3.x, p3.y, angle);
    addVertex(cx + p1.x, cy + p1.y, r, g, b, a);
    addVertex(cx + p2.x, cy + p2.y, r, g, b, a);
    addVertex(cx + p3.x, cy + p3.y, r, g, b, a);
  }

  // 画圆形
  // 画圆形
  void addCircle(float cx, float cy, float radius, float r, float g, float b, float a = 1.0f) {
    int segments = 64; // 从 360 减少到 64 提高性能
    for (int i = 0; i < segments; i++) {
        float theta1 = 2.0f * PI * float(i) / float(segments);
        float theta2 = 2.0f * PI * float(i + 1) / float(segments);
        addVertex(cx, cy, r, g, b, a);
        addVertex(cx + radius * cosf(theta1), cy + radius * sinf(theta1), r, g, b, a);
        addVertex(cx + radius * cosf(theta2), cy + radius * sinf(theta2), r, g, b, a);
    }
  }

  // 画直线
  void addLine(float x1, float y1, float x2, float y2, float thick, float r, float g, float b, float a = 1.0f) {
      float dx = x2 - x1;
      float dy = y2 - y1;
      float len = sqrtf(dx*dx + dy*dy);
      if (len < 1e-6f) return;
      float angle = atan2f(dy, dx);
      addRotatedRect((x1 + x2) * 0.5f, (y1 + y2) * 0.5f, len, thick, angle, r, g, b, a);
  }

  // 精细地球渲染
  void addEarthWithContinents(float cx, float cy, float radius,
                              float cam_angle) {
    int segments = 3600;
    const int NUM_CONTINENTS = 8;
    float cont_center[NUM_CONTINENTS];
    float cont_half_w[NUM_CONTINENTS];
    for (int c = 0; c < NUM_CONTINENTS; c++) {
      cont_center[c] = (float)c / NUM_CONTINENTS + hash11(c * 9973.0f) * 0.08f;
      if (cont_center[c] > 1.0f) cont_center[c] -= 1.0f;
      cont_half_w[c] = 0.03f + hash11(c * 7333.0f) * 0.07f;
    }

    for (int i = 0; i < segments; i++) {
      float theta1 = 2.0f * PI * float(i) / float(segments) + cam_angle;
      float theta2 = 2.0f * PI * float(i + 1) / float(segments) + cam_angle;
      float pos = float(i) / float(segments);
      bool is_land = false;
      for (int c = 0; c < NUM_CONTINENTS; c++) {
        float dist = std::abs(pos - cont_center[c]);
        if (dist > 0.5f) dist = 1.0f - dist;
        float coast_noise = hash11(i * 137.0f + c * 5171.0f) * 0.015f;
        if (dist < cont_half_w[c] + coast_noise) {
          is_land = true;
          break;
        }
      }
      float biome_hash = hash11(float(i * 419));
      float r, g, b;
      float lat = std::abs(float(i % 1800) - 900.0f) / 900.0f;
      if (!is_land) {
        float depth = hash11(float(i * 503)) * 0.3f;
        r = 0.05f + lat * 0.1f;
        g = 0.25f + depth * 0.15f + lat * 0.15f;
        b = 0.55f + depth * 0.25f + lat * 0.1f;
      } else {
        if (lat > 0.85f) {
          r = 0.85f; g = 0.9f; b = 0.95f;
        } else if (lat < 0.3f && biome_hash > 0.5f) {
          r = 0.7f + biome_hash * 0.15f; g = 0.55f + biome_hash * 0.1f; b = 0.3f;
        } else {
          float lush = hash11(float(i * 631)) * 0.15f;
          r = 0.15f + lush; g = 0.4f + lush + lat * 0.1f; b = 0.12f + lush * 0.5f;
        }
      }
      addVertex(cx, cy, r, g, b, 1.0f);
      addVertex(cx + radius * std::cos(theta1), cy + radius * std::sin(theta1), r, g, b, 1.0f);
      addVertex(cx + radius * std::cos(theta2), cy + radius * std::sin(theta2), r, g, b, 1.0f);
    }
  }

  // 大气光环
  void addAtmosphereGlow(float cx, float cy, float radius, float cam_angle) {
    int segments = 360;
    float atmo_t = radius * 0.015f;
    for (int i = 0; i < segments; i++) {
      float t1 = 2.0f * PI * float(i) / float(segments) + cam_angle;
      float t2 = 2.0f * PI * float(i + 1) / float(segments) + cam_angle;
      float ir = radius, or_ = radius + atmo_t;
      addVertex(cx + ir * std::cos(t1), cy + ir * std::sin(t1), 0.4f, 0.6f, 1.0f, 0.4f);
      addVertex(cx + ir * std::cos(t2), cy + ir * std::sin(t2), 0.4f, 0.6f, 1.0f, 0.4f);
      addVertex(cx + or_ * std::cos(t1), cy + or_ * std::sin(t1), 0.3f, 0.5f, 0.9f, 0.0f);
      addVertex(cx + or_ * std::cos(t1), cy + or_ * std::sin(t1), 0.3f, 0.5f, 0.9f, 0.0f);
      addVertex(cx + ir * std::cos(t2), cy + ir * std::sin(t2), 0.4f, 0.6f, 1.0f, 0.4f);
      addVertex(cx + or_ * std::cos(t2), cy + or_ * std::sin(t2), 0.3f, 0.5f, 0.9f, 0.0f);
    }
  }

  // 姿态球 (Navball)
  // pitch: [-PI/2, PI/2], yaw: [0, 2*PI], roll: [-PI, PI]
  void drawAttitudeSphere(float cx, float cy, float radius, const Quat& qRocket, const Vec3& localRight, const Vec3& localUp, const Vec3& localNorth, bool sas_active, bool rcs_active) {
    // 1. 基础圆盘与底座
    addCircle(cx, cy, radius * 1.05f, 0.1f, 0.1f, 0.12f, 0.9f);
    addCircle(cx, cy, radius, 0.2f, 0.2f, 0.22f, 1.0f);

    // --- SAS & RCS Indicators ---
    float ind_r = radius * 0.15f;
    float ind_y = cy + radius * 0.7f;
    
    // SAS Indicator (Left)
    float sas_x = cx - radius * 1.25f;
    addCircle(sas_x, ind_y, ind_r, 0.1f, 0.1f, 0.1f, 0.8f); // Background
    if (sas_active) {
        addCircle(sas_x, ind_y, ind_r * 0.8f, 0.2f, 1.0f, 0.4f, 1.0f); // Green Glow
        drawText(sas_x, ind_y - ind_r * 2.2f, "SAS", radius * 0.12f, 0.2f, 1.0f, 0.4f, 1.0f, true, CENTER);
    } else {
        addCircle(sas_x, ind_y, ind_r * 0.8f, 0.3f, 0.1f, 0.1f, 1.0f); // Dim Red
        drawText(sas_x, ind_y - ind_r * 2.2f, "SAS", radius * 0.12f, 0.5f, 0.5f, 0.5f, 0.6f, true, CENTER);
    }

    // RCS Indicator (Right)
    float rcs_x = cx + radius * 1.25f;
    addCircle(rcs_x, ind_y, ind_r, 0.1f, 0.1f, 0.1f, 0.8f); // Background
    if (rcs_active) {
        addCircle(rcs_x, ind_y, ind_r * 0.8f, 0.2f, 0.8f, 1.0f, 1.0f); // Blue Glow
        drawText(rcs_x, ind_y - ind_r * 2.2f, "RCS", radius * 0.12f, 0.2f, 0.8f, 1.0f, 1.0f, true, CENTER);
    } else {
        addCircle(rcs_x, ind_y, ind_r * 0.8f, 0.3f, 0.1f, 0.1f, 1.0f); // Dim Red
        drawText(rcs_x, ind_y - ind_r * 2.2f, "RCS", radius * 0.12f, 0.5f, 0.5f, 0.5f, 0.6f, true, CENTER);
    }

    // 2. 坐标转换逻辑 (直接基于四元数投影，消除万向锁)
    // p 处于水平参考系 (X=East, Y=Up, Z=North)
    auto project = [&](Vec3 p) -> Vec2 {
        // 先转到世界空间
        Vec3 pW = localRight * p.x + localUp * p.y + localNorth * p.z;
        // 再转到火箭局部空间
        Vec3 pR = qRocket.conjugate().rotate(pW);
        // 火箭纵轴为 Y+ (Depth)，左右为 X+，上下为 Z+ (Screen Y)
        if (pR.y < 0) return {999, 999}; 
        return {cx + pR.x * radius, cy + pR.z * radius};
    };

    auto getSpherePt = [&](float lat, float lon) -> Vec3 {
        float phi = lat * (PI / 180.0f), theta = lon * (PI / 180.0f);
        return { cosf(phi) * sinf(theta), sinf(phi), cosf(phi) * cosf(theta) };
    };

    // 3. 绘制色块 (天空与地面) - 提高精细度
    int lat_steps = 24; // 增加步长
    int lon_steps = 48;
    for (int i = 0; i < lat_steps; i++) {
        float lat1 = -90.0f + (float)i / lat_steps * 180.0f;
        float lat2 = -90.0f + (float)(i + 1) / lat_steps * 180.0f;
        for (int j = 0; j < lon_steps; j++) {
            float lon1 = (float)j / lon_steps * 360.0f;
            float lon2 = (float)(j + 1) / lon_steps * 360.0f;
            Vec3 v1 = getSpherePt(lat1, lon1), v2 = getSpherePt(lat1, lon2);
            Vec3 v3 = getSpherePt(lat2, lon2), v4 = getSpherePt(lat2, lon1);
            Vec2 p1 = project(v1), p2 = project(v2), p3 = project(v3), p4 = project(v4);
            if (p1.x < 900 || p2.x < 900 || p3.x < 900 || p4.x < 900) {
                Vec3 col = (lat1 + lat2 > 0) ? Vec3(0.08f, 0.45f, 0.88f) : Vec3(0.58f, 0.32f, 0.15f);
                auto safeAddTri = [&](Vec2 a, Vec2 b, Vec2 c) {
                    if (a.x > 900 || b.x > 900 || c.x > 900) return;
                    addVertex(a.x, a.y, col.x, col.y, col.z, 1.0f);
                    addVertex(b.x, b.y, col.x, col.y, col.z, 1.0f);
                    addVertex(c.x, c.y, col.x, col.y, col.z, 1.0f);
                };
                safeAddTri(p1, p2, p3); safeAddTri(p1, p3, p4);
            }
        }
    }

    // 4. 绘制地平线 (Horizon Line)
    for (int j = 0; j < 128; j++) {
        float lon1 = (float)j / 128 * 360.0f, lon2 = (float)(j + 1) / 128 * 360.0f;
        Vec2 p1 = project(getSpherePt(0, lon1)), p2 = project(getSpherePt(0, lon2));
        if (p1.x < 900 && p2.x < 900) addLine(p1.x, p1.y, p2.x, p2.y, 0.004f, 1, 1, 1, 1.0f);
    }

    // 5. 绘制俯仰刻度 (Pitch Lines) 与 刻字
    for (int lat = -80; lat <= 80; lat += 10) {
        if (lat == 0) continue;
        for (int j = 0; j < 96; j++) {
            float lon1 = (float)j / 96 * 360.0f, lon2 = (float)(j + 1) / 96 * 360.0f;
            Vec2 p1 = project(getSpherePt((float)lat, lon1)), p2 = project(getSpherePt((float)lat, lon2));
            if (p1.x < 900 && p2.x < 900) {
                float opacity = (lat % 30 == 0) ? 0.6f : 0.3f;
                addLine(p1.x, p1.y, p2.x, p2.y, 0.0012f, 1, 1, 1, opacity);
            }
        }
        // 刻字：不仅在 0 度，更在环绕四周，并随边缘淡出
        for (int deg = 0; deg < 360; deg += 90) {
            Vec3 vL = getSpherePt((float)lat, (float)deg);
            Vec3 pW = localRight * vL.x + localUp * vL.y + localNorth * vL.z;
            Vec3 pR = qRocket.conjugate().rotate(pW);
            if (pR.y > 0.1f) { // y 是深度，y > 0 在正面
                float alpha = fminf(1.0f, pR.y * 3.0f); // 边缘淡出以增强“球体感”
                Vec2 plabel = {cx + pR.x * radius, cy + pR.z * radius};
                char buf[8]; snprintf(buf, sizeof(buf), "%d", lat);
                drawText(plabel.x, plabel.y, buf, radius * 0.09f, 1, 1, 1, 0.8f * alpha, true, CENTER);
            }
        }
    }

    // 6. 绘制垂直经线 (Meridians) 与 航向标签
    for (int lon = 0; lon < 360; lon += 30) {
        for (int i = 0; i < 36; i++) {
            float lat1 = -90.0f + (float)i * 5.0f, lat2 = lat1 + 5.0f;
            Vec2 p1 = project(getSpherePt(lat1, (float)lon)), p2 = project(getSpherePt(lat2, (float)lon));
            if (p1.x < 900 && p2.x < 900) addLine(p1.x, p1.y, p2.x, p2.y, 0.001f, 1, 1, 1, 0.3f);
        }
        // 航向标签 (N, E, S, W) - 增加淡出以增强“刻在球上”的感觉
        const char* marks[] = {"N", "30", "60", "E", "120", "150", "S", "210", "240", "W", "300", "330"};
        Vec3 vM = getSpherePt(0, (float)lon);
        Vec3 pW_m = localRight * vM.x + localUp * vM.y + localNorth * vM.z;
        Vec3 pR_m = qRocket.conjugate().rotate(pW_m);
        if (pR_m.y > 0.05f) {
            float alpha = fminf(1.0f, pR_m.y * 4.0f);
            Vec2 pmark = {cx + pR_m.x * radius, cy + pR_m.z * radius};
            drawText(pmark.x, pmark.y, marks[lon/30], radius * 0.12f, 1, 1, 0.1f, 0.9f * alpha, true, CENTER);
        }
    }

    // 7. 绘制中央固定指示器 (Level Indicator) - "The Wings"
    // 增加一个半透明黑色底座来消除各种细碎线条造成的闪烁/遮光感
    addCircle(cx, cy, radius * 0.12f, 0, 0, 0, 0.4f);
    float iw = radius * 0.45f;
    float ih = radius * 0.045f;
    // 阴影/描边 (黑色)
    addRect(cx - iw * 0.7f, cy, iw * 0.52f, ih * 1.5f, 0, 0, 0, 0.8f);
    addRect(cx + iw * 0.7f, cy, iw * 0.52f, ih * 1.5f, 0, 0, 0, 0.8f);
    addRect(cx, cy, ih * 3.5f, ih * 3.5f, 0, 0, 0, 0.8f);
    // 主体 (橘色)
    addRect(cx - iw * 0.7f, cy, iw * 0.5f, ih, 1.0f, 0.6f, 0.0f, 1.0f); // L
    addRect(cx + iw * 0.7f, cy, iw * 0.5f, ih, 1.0f, 0.6f, 0.0f, 1.0f); // R
    addRect(cx, cy, ih * 2.5f, ih * 2.5f, 1.0f, 0.6f, 0.0f, 1.0f);      // C
  }

  enum Align { LEFT, CENTER, RIGHT };
  void drawText(float x, float y, const char* text, float size, float r, float g, float b, float a = 1.0f, bool shadow = true, Align align = LEFT) {
    int len = 0; while (text[len]) len++;
    float px = size * 0.15f;
    float cw = px * 6.0f;
    float total_w = (len > 0) ? (len * cw - px) : 0;
    float ox_start = x;
    if (align == CENTER) ox_start -= total_w * 0.5f;
    else if (align == RIGHT) ox_start -= total_w;

    if (shadow) {
        drawText(ox_start + size * 0.08f, y - size * 0.08f, text, size, 0.0f, 0.0f, 0.0f, a * 0.8f, false, LEFT);
    }

    auto getGlyph5x7 = [](char c) -> uint64_t {
      if (c >= 'a' && c <= 'z') c -= 32;
      switch (c) {
        case '0': return 0x1C22262A32221CULL; case '1': return 0x080C080808081CULL;
        case '2': return 0x1C22020C10203EULL; case '3': return 0x1C22020C02221CULL;
        case '4': return 0x040C14243E0404ULL; case '5': return 0x3E203C0202221CULL;
        case '6': return 0x1C22203C22221CULL; case '7': return 0x3E020408101010ULL;
        case '8': return 0x1C22221C22221CULL; case '9': return 0x1C22221E02221CULL;
        case 'A': return 0x1C22223E222222ULL; case 'B': return 0x3C22223C22223CULL;
        case 'C': return 0x1C22202020221CULL; case 'D': return 0x3C22222222223CULL;
        case 'E': return 0x3E20203C20203EULL; case 'F': return 0x3E20203C202020ULL;
        case 'G': return 0x1C22202E22221CULL; case 'H': return 0x2222223E222222ULL;
        case 'I': return 0x1C08080808081CULL; case 'J': return 0x0E040404042418ULL;
        case 'K': return 0x22242830282422ULL; case 'L': return 0x2020202020203EULL;
        case 'M': return 0x22362A22222222ULL; case 'N': return 0x22322A26222222ULL;
        case 'O': return 0x1C22222222221CULL; case 'P': return 0x3C22223C202020ULL;
        case 'Q': return 0x1C2222222A241AULL; case 'R': return 0x3C22223C282422ULL;
        case 'S': return 0x1C22100C02221CULL; case 'T': return 0x3E080808080808ULL;
        case 'U': return 0x2222222222221CULL; case 'V': return 0x22222222221408ULL;
        case 'W': return 0x2222222A2A3622ULL; case 'X': return 0x22221408142222ULL;
        case 'Y': return 0x22222214080808ULL; case 'Z': return 0x3E02040810203EULL;
        case '.': return 0x00000000000008ULL; case ':': return 0x00080000000800ULL;
        case '-': return 0x0000001C000000ULL; case '/': return 0x02040810204000ULL;
        case '%': return 0x22040810222412ULL; case '(': return 0x04080808080804ULL;
        case ')': return 0x08040404040408ULL; case '+': return 0x0008083E080800ULL;
        case '[': return 0x1E10101010101EULL; case ']': return 0x1E02020202021EULL;
        case '<': return 0x04081020100804ULL; case '>': return 0x10080402040810ULL;
        case ' ': return 0x0ULL;
        default: return 0;
      }
    };

    int idx = 0;
    while (text[idx]) {
      uint64_t gl = getGlyph5x7(text[idx]);
      if (gl != 0 || text[idx] == ' ') {
        float ox = ox_start + idx * cw;
        for (int row = 0; row < 7; row++) {
          int row_data = (int)((gl >> ((6 - row) * 8)) & 0xFF);
          for (int col = 0; col < 5; col++) {
            if (row_data & (1 << (5 - col))) {
              addRect(ox + col * px, y + (3 - row) * px, px * 1.35f, px * 1.35f, r, g, b, a);
            }
          }
        }
      }
      idx++;
    }
  }

  void drawInt(float x, float y, int val, float size, float r, float g, float b, float a = 1.0f, Align align = LEFT) {
      char buf[32]; snprintf(buf, sizeof(buf), "%d", val);
      drawText(x, y, buf, size, r, g, b, a, true, align);
  }

  void endFrame() {
    if (vertices.empty()) return;
    glUseProgram(shaderProgram);
    glBindVertexArray(VAO);
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferSubData(GL_ARRAY_BUFFER, 0, vertices.size() * sizeof(float), vertices.data());
    glDrawArrays(GL_TRIANGLES, 0, (GLsizei)(vertices.size() / 6));
  }

  ~Renderer() {
    glDeleteVertexArrays(1, &VAO);
    glDeleteBuffers(1, &VBO);
    glDeleteProgram(shaderProgram);
  }

private:
};
