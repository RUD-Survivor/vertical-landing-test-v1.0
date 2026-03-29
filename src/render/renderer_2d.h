#pragma once
#include <glad/glad.h>
#include <vector>
#include <cmath>
#include <string>
#include <iostream>
#include <map>
#include <unordered_map>
#include <algorithm>
#include <fstream>
#include "core/rocket_state.h"


#include "stb_truetype.h"

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

// -------------------------------------------------------------
// 2D 渲染顶点着色器 (Vertex Shader)
// 负责将 2D 屏幕坐标转换为 OpenGL 的标准化设备坐标 (NDC: -1 to 1)。
// -------------------------------------------------------------
static const char *vertexShaderSource2D = R"(
    #version 330 core
    layout (location = 0) in vec2 aPos;      // 顶点位置 (x, y)
    layout (location = 1) in vec4 aColor;    // 顶点颜色 (r, g, b, a)
    layout (location = 2) in vec2 aTexCoord; // 纹理坐标 (u, v)
    out vec4 ourColor;
    out vec2 TexCoord;
    void main() {
        // 直接使用传入的坐标，假定调用者已处理好屏幕比例。
        gl_Position = vec4(aPos.x, aPos.y, 0.0, 1.0);
        ourColor = aColor;
        TexCoord = aTexCoord;
    }
)";

// -------------------------------------------------------------
// 2D 渲染片段着色器 (Fragment Shader)
// 处理颜色混合和文字/图标的纹理采样。
// -------------------------------------------------------------
static const char *fragmentShaderSource2D = R"(
    #version 330 core
    out vec4 FragColor;
    in vec4 ourColor;
    in vec2 TexCoord;
    uniform sampler2D ourTexture; // 文字图集或 UI 图块
    void main() {
        // 对于文字，纹理只有单通道（红色），用它作为 Alpha 值来实现抗锯齿渲染。
        float alpha = texture(ourTexture, TexCoord).r;
        FragColor = vec4(ourColor.rgb, ourColor.a * alpha);
    }
)";

class Renderer {
private:
  unsigned int shaderProgram, VBO, VAO, fontTexture;
  std::vector<float> vertices;
  
  struct GlyphInfo {
      float u1, v1, u2, v2;
      int w, h;
      float xoff, yoff, xadvance;
  };
  std::map<unsigned int, GlyphInfo> glyphCache;
  unsigned char* fontBuffer = nullptr;
  stbtt_fontinfo fontInfo;

  unsigned int compileShader(unsigned int type, const char *source) {
    unsigned int id = glCreateShader(type);
    glShaderSource(id, 1, &source, NULL);
    glCompileShader(id);
    return id;
  }
  // 编译着色器函数，根据给定的类型和源代码创建并返回一个着色器ID

public:
  enum Align { LEFT, CENTER, RIGHT };
  void beginFrame() { vertices.clear(); }
  Renderer() {
    // 1. 编译并链接 2D 着色器程序
    unsigned int v = compileShader(GL_VERTEX_SHADER, vertexShaderSource2D);
    unsigned int f = compileShader(GL_FRAGMENT_SHADER, fragmentShaderSource2D);
    shaderProgram = glCreateProgram();
    glAttachShader(shaderProgram, v);
    glad_glAttachShader(shaderProgram, f);
    glLinkProgram(shaderProgram);
    glDeleteShader(v);
    glDeleteShader(f);

    // 2. 初始化顶点数组与缓冲区 (VAO/VBO)
    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);
    glBindVertexArray(VAO);
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    // 预分配大容量缓冲区，用于批量渲染（Batch Rendering）
    glBufferData(GL_ARRAY_BUFFER, sizeof(float) * 5000000, NULL, GL_DYNAMIC_DRAW);

    // X, Y (位置属性)
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void *)0);
    glEnableVertexAttribArray(0);
    // R, G, B, A (颜色属性)
    glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void *)(2 * sizeof(float)));
    glEnableVertexAttribArray(1);
    // U, V (纹理坐标属性)
    glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void *)(6 * sizeof(float)));
    glEnableVertexAttribArray(2);
    
    // 3. 构建字体图集 (Font Atlas)
    buildFontAtlas();
  }

  // Unified Atlas build logic
  // Moved into buildFontAtlas above

  // 向顶点缓冲区添加一个顶点
  void addVertex(float x, float y, float r, float g, float b, float a, float u = 0, float v = 0) {
    vertices.push_back(x); vertices.push_back(y);
    vertices.push_back(r); vertices.push_back(g); vertices.push_back(b); vertices.push_back(a);
    vertices.push_back(u); vertices.push_back(v);
  }
  
  // 添加一个“纯色”顶点（使用图集中预留的白色像素）
  void addVertexWhite(float x, float y, float r, float g, float b, float a) {
      addVertex(x, y, r, g, b, a, 1.0f/512.0f, 1.0f/512.0f);
  }

  // 普通矩形
  void addRect(float x, float y, float w, float h, float r, float g, float b, float a = 1.0f) {
    addVertexWhite(x - w / 2, y - h / 2, r, g, b, a);
    addVertexWhite(x + w / 2, y - h / 2, r, g, b, a);
    addVertexWhite(x + w / 2, y + h / 2, r, g, b, a);
    addVertexWhite(x + w / 2, y + h / 2, r, g, b, a);
    addVertexWhite(x - w / 2, y + h / 2, r, g, b, a);
    addVertexWhite(x - w / 2, y - h / 2, r, g, b, a);
  }

  // 矩形边框
  void addRectOutline(float x, float y, float w, float h, float r, float g, float b, float a = 1.0f, float thick = 0.002f) {
      addLine(x - w / 2, y - h / 2, x + w / 2, y - h / 2, thick, r, g, b, a);
      addLine(x + w / 2, y - h / 2, x + w / 2, y + h / 2, thick, r, g, b, a);
      addLine(x + w / 2, y + h / 2, x - w / 2, y + h / 2, thick, r, g, b, a);
      addLine(x - w / 2, y + h / 2, x - w / 2, y - h / 2, thick, r, g, b, a);
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

  void addCircleOutline(float cx, float cy, float radius, float thick, float r, float g, float b, float a = 1.0f) {
    int segments = 64;
    for (int i = 0; i < segments; i++) {
        float theta1 = 2.0f * PI * float(i) / float(segments);
        float theta2 = 2.0f * PI * float(i + 1) / float(segments);
        addLine(cx + radius * cosf(theta1), cy + radius * sinf(theta1),
                cx + radius * cosf(theta2), cy + radius * sinf(theta2),
                thick, r, g, b, a);
    }
  }

  // 画直线（通过生成一个长条矩形实现）
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
      addVertexWhite(cx, cy, r, g, b, 1.0f);
      addVertexWhite(cx + radius * std::cos(theta1), cy + radius * std::sin(theta1), r, g, b, 1.0f);
      addVertexWhite(cx + radius * std::cos(theta2), cy + radius * std::sin(theta2), r, g, b, 1.0f);
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
      addVertexWhite(cx + ir * std::cos(t1), cy + ir * std::sin(t1), 0.4f, 0.6f, 1.0f, 0.4f);
      addVertexWhite(cx + ir * std::cos(t2), cy + ir * std::sin(t2), 0.4f, 0.6f, 1.0f, 0.4f);
      addVertexWhite(cx + or_ * std::cos(t1), cy + or_ * std::sin(t1), 0.3f, 0.5f, 0.9f, 0.0f);
      addVertexWhite(cx + or_ * std::cos(t1), cy + or_ * std::sin(t1), 0.3f, 0.5f, 0.9f, 0.0f);
      addVertexWhite(cx + ir * std::cos(t2), cy + ir * std::sin(t2), 0.4f, 0.6f, 1.0f, 0.4f);
      addVertexWhite(cx + or_ * std::cos(t2), cy + or_ * std::sin(t2), 0.3f, 0.5f, 0.9f, 0.0f);
    }
  }

  // 姿态球 (Navball)
  // pitch: [-PI/2, PI/2], yaw: [0, 2*PI], roll: [-PI, PI]
  void drawAttitudeSphere(float cx, float cy, float radius, const Quat& qRocket, const Vec3& localRight, const Vec3& localUp, const Vec3& localNorth, bool sas_active, bool rcs_active, 
                          const Vec3& vPrograde, const Vec3& vNormal, const Vec3& vRadial, const Vec3& vManeuver = Vec3(0,0,0)) {
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
    // 此算法将虚拟球体上的 3D 点投影到 2D 屏幕上的“姿态球”圆面上。
    auto project = [&](Vec3 p) -> Vec2 {
        // 先根据火箭所在的地理参考系（Up/North/East）转到世界空间
        Vec3 pW = localRight * p.x + localUp * p.y + localNorth * p.z;
        // 再通过火箭姿态四元数的共轭，转到火箭视角下的局部空间
        Vec3 pR = qRocket.conjugate().rotate(pW);
        // 如果点在火箭“脑后”（pR.y < 0），则不绘制（或返回无效坐标）
        if (pR.y < 0) return {999, 999}; 
        // 最终映射到屏幕圆盘坐标
        return {cx + pR.x * radius, cy + pR.z * radius};
    };

    // 获取球面上的 3D 坐标（经纬度转笛卡尔）
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
                    addVertexWhite(a.x, a.y, col.x, col.y, col.z, 1.0f);
                    addVertexWhite(b.x, b.y, col.x, col.y, col.z, 1.0f);
                    addVertexWhite(c.x, c.y, col.x, col.y, col.z, 1.0f);
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

    // 8. 绘制轨道标记 (Orbital Markers)
    auto drawMarker = [&](Vec3 v, const char* type, float r, float g, float b) {
        if (v.length() < 0.1f) return;
        
        // 直接投影到火箭局部空间 (v 已经是世界方向向量)
        Vec3 pR = qRocket.conjugate().rotate(v);
        
        if (pR.y < 0.05f) return; // 球背面不绘制
        
        float alpha = fminf(1.0f, pR.y * 5.0f);
        float mx = cx + pR.x * radius;
        float my = cy + pR.z * radius;
        float ms = radius * 0.14f;
        float lw = 0.003f;
        
        // Hollow designs: Use outlines only to avoid blocking text
        if (std::string(type) == "PRO") { // Prograde: Hollow Green Circle with wings
            addCircle(mx, my, ms * 0.8f, r, g, b, alpha * 0.3f); // Very faint internal circle
            addCircleOutline(mx, my, ms * 0.8f, lw, r, g, b, alpha);
            addLine(mx, my - ms * 0.8f, mx, my - ms * 1.4f, lw, r, g, b, alpha);
            addLine(mx - ms * 0.8f, my, mx - ms * 1.4f, my, lw, r, g, b, alpha);
            addLine(mx + ms * 0.8f, my, mx + ms * 1.4f, my, lw, r, g, b, alpha);
            addCircle(mx, my, ms * 0.15f, r, g, b, alpha); // Small center dot
        } else if (std::string(type) == "RET") { // Retrograde: Hollow Yellow Circle with wings
            addCircleOutline(mx, my, ms * 0.8f, lw, r, g, b, alpha);
            addLine(mx, my - ms * 0.4f, mx, my - ms * 0.8f, lw, r, g, b, alpha);
            addLine(mx - ms * 0.4f, my, mx - ms * 0.8f, my, lw, r, g, b, alpha);
            addLine(mx + ms * 0.4f, my, mx + ms * 0.8f, my, lw, r, g, b, alpha);
            drawText(mx, my, "X", ms * 0.8f, r, g, b, alpha, true, Renderer::CENTER);
        } else if (std::string(type) == "NRM") { // Normal: Hollow Purple Triangle
            addCircleOutline(mx, my, ms * 0.4f, lw, r, g, b, alpha * 0.5f);
            // Draw hollow triangle using 3 lines
            float h = ms * 1.8f;
            float w2 = h * 0.6f;
            float ty = my - h * 0.4f;
            addLine(mx, ty, mx - w2, ty + h, lw, r, g, b, alpha);
            addLine(mx - w2, ty + h, mx + w2, ty + h, lw, r, g, b, alpha);
            addLine(mx + w2, ty + h, mx, ty, lw, r, g, b, alpha);
        } else if (std::string(type) == "ANT") { // Anti-normal: Hollow Purple Inv-Triangle with X
            float h = ms * 1.8f;
            float w2 = h * 0.6f;
            float ty = my + h * 0.4f;
            addLine(mx, ty, mx - w2, ty - h, lw, r, g, b, alpha);
            addLine(mx - w2, ty - h, mx + w2, ty - h, lw, r, g, b, alpha);
            addLine(mx + w2, ty - h, mx, ty, lw, r, g, b, alpha);
            drawText(mx, my - ms * 0.2f, "X", ms * 0.7f, r, g, b, alpha, true, Renderer::CENTER);
        } else if (std::string(type) == "R-I") { // Radial In: Deep Blue hollow circle with inward lines
            addCircleOutline(mx, my, ms * 1.0f, lw, r, g, b, alpha);
            for(int k=0; k<4; k++){
                float ang = k * PI/2.0f;
                addLine(mx + cosf(ang)*ms, my + sinf(ang)*ms, mx + cosf(ang)*ms*0.4f, my + sinf(ang)*ms*0.4f, lw, r, g, b, alpha);
            }
        } else if (std::string(type) == "R-O") { // Radial Out: Deep Blue hollow circle with outward lines
            addCircleOutline(mx, my, ms * 0.7f, lw, r, g, b, alpha);
            for(int k=0; k<4; k++){
                float ang = (k * PI/2.0f) + (PI/4.0f);
                addLine(mx + cosf(ang)*ms*0.7f, my + sinf(ang)*ms*0.7f, mx + cosf(ang)*ms*1.3f, my + sinf(ang)*ms*1.3f, lw, r, g, b, alpha);
            }
        } else if (std::string(type) == "MNV") { // Maneuver: Blue "Star" / Cross
            ms *= 1.1f;
            drawText(mx, my, "+", ms * 1.5f, r, g, b, alpha, true, CENTER);
        }
    };

    drawMarker(vPrograde, "PRO", 0.2f, 0.8f, 0.2f);
    drawMarker(vPrograde * -1.0f, "RET", 0.8f, 0.8f, 0.1f);
    drawMarker(vNormal, "NRM", 0.7f, 0.2f, 0.8f);
    drawMarker(vNormal * -1.0f, "ANT", 0.7f, 0.2f, 0.8f);
    drawMarker(vRadial, "R-I", 0.1f, 0.2f, 0.9f);
    drawMarker(vRadial * -1.0f, "R-O", 0.1f, 0.2f, 0.9f);
    if (vManeuver.length() > 0.1f) drawMarker(vManeuver, "MNV", 0.2f, 0.6f, 1.0f);
  }

  // 构建字体图集 (Font Atlas)
  // 此函数使用 stb_truetype 将 .otf 字体文件转化为一个大的 2D 纹理网格。
  void buildFontAtlas() {
      const int ATLAS_SIZE = 1024; // 1024x1024 的单通道纹理
      std::vector<unsigned char> atlasData(ATLAS_SIZE * ATLAS_SIZE, 0);
      
      // 在纹理的最左上角（0,0处）预留几个白色像素，用于绘制不带纹理的纯色几何体。
      for(int y=0; y<4; y++) for(int x=0; x<4; x++) atlasData[y*ATLAS_SIZE + x] = 255;
      glyphCache[0] = {0.5f/ATLAS_SIZE, 0.5f/ATLAS_SIZE, 3.5f/ATLAS_SIZE, 3.5f/ATLAS_SIZE, 4, 4, 0, 0, 0};

      // 读取中文字体文件
      std::ifstream file("assets/fonts/SourceHanSansCN-Regular.otf", std::ios::binary | std::ios::ate);
      if (!file.is_open()) {
          std::cerr << "Failed to open font file: assets/fonts/SourceHanSansCN-Regular.otf" << std::endl;
          return;
      }
      std::streamsize size = file.tellg();
      file.seekg(0, std::ios::beg);
      fontBuffer = new unsigned char[size];
      if (!file.read((char*)fontBuffer, size)) return;
      
      // 初始化 stb_truetype
      if (!stbtt_InitFont(&fontInfo, fontBuffer, 0)) {
          std::cerr << "Failed to init stb_truetype" << std::endl;
          delete[] fontBuffer; fontBuffer = nullptr;
          return;
      }

      const int FONT_HEIGHT = 24; // 基础字号
      int curX = 8, curY = 0, maxHeight = 0;
      float scale = stbtt_ScaleForPixelHeight(&fontInfo, (float)FONT_HEIGHT);

      // 将特定的 Unicode 字符光栅化并存入图集
      auto bakeChar = [&](unsigned int unicode) {
          if (glyphCache.count(unicode)) return;
          int w, h, xoff, yoff;
          // 生成字符位图
          unsigned char* bitmap = stbtt_GetCodepointBitmap(&fontInfo, scale, scale, (int)unicode, &w, &h, &xoff, &yoff);
          if (bitmap) {
              // 自动换行算法：如果当前行放不下了，就换到下一行。
              if (curX + w + 1 >= ATLAS_SIZE) { curX = 0; curY += maxHeight + 1; maxHeight = 0; }
              if (curY + h + 1 >= ATLAS_SIZE) { stbtt_FreeBitmap(bitmap, 0); return; }
              // 拷贝位图到图集数据中
              for (int j = 0; j < h; j++) {
                  for (int k = 0; k < w; k++) {
                      atlasData[(curY + j) * ATLAS_SIZE + (curX + k)] = bitmap[j * w + k];
                  }
              }
              int advance;
              stbtt_GetCodepointHMetrics(&fontInfo, (int)unicode, &advance, nullptr);
              // 记录字符在图集中的 UV 坐标
              glyphCache[unicode] = {
                  (float)curX / ATLAS_SIZE, (float)curY / ATLAS_SIZE,
                  (float)(curX + w) / ATLAS_SIZE, (float)(curY + h) / ATLAS_SIZE,
                  w, h, (float)xoff, (float)yoff, (float)advance * scale
              };
              curX += w + 1;
              if (h > maxHeight) maxHeight = h;
              stbtt_FreeBitmap(bitmap, 0);
          }
      };

      // 1. 烘焙 ASCII 字符
      for (int i = 32; i < 127; i++) bakeChar(i);
      // 2. 烘焙 UI 中常用的中文字符
      const unsigned int common_cn[] = {
          0x8BBE, 0x7F6E, 0x8BED, 0x8A00, 0x7EE7, 0x7EED, 0x65B0, 0x6E38, 0x620F, 0x9000, 0x51FA, 0x8FD4, 0x56DE, 0x786E, 0x8BA4, 0x4E2D,
          0x6587, 0x82F1, 0x8D44, 0x6E90, 0x5929, 0x77FF, 0x673A, 0x91D1, 0x544A, 0x8B66, 0x7194, 0x7089, 0x88C5, 0x914D, 0x4ED3, 0x5E93,
          0x5DE5, 0x5382, 0x6982, 0x89C2, 0x72B6, 0x6001, 0x5728, 0x7EBF, 0x79BB, 0x94F1, 0x96F6, 0x4EF6, 0x6CA1, 0x6709, 0x7269, 0x4EA7,
          0x7EC4, 0x8F66, 0x95F4, 0x53D1, 0x5C04, 0x573A, 0x8FDB, 0x5165, 0x6A21, 0x5F0F, 0x4E3B, 0x83DC, 0x5355, 0x822A, 0x5C40, 0x5C06,
          0x5220, 0x9664, 0x5B58, 0x6863, 0x4F7F, 0x7528, 0x4E0A, 0x4E0B, 0x952E, 0x9009, 0x62E9, 0x6309, 0x5FC3, 0x706B, 0x7BAD
      };
      for (unsigned int u : common_cn) bakeChar(u);

      // 创建 OpenGL 纹理对象
      glGenTextures(1, &fontTexture);
      glBindTexture(GL_TEXTURE_2D, fontTexture);
      glTexImage2D(GL_TEXTURE_2D, 0, GL_RED, ATLAS_SIZE, ATLAS_SIZE, 0, GL_RED, GL_UNSIGNED_BYTE, atlasData.data());
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  }

  // 绘制文本 (Draw Text)
  // 支持 UTF-8 编码，可以混合绘制中英文。
  void drawText(float x, float y, const char* text, float size, float r, float g, float b, float a = 1.0f, bool shadow = true, Align align = LEFT) {
    if (!text || !text[0]) return;
    float px = size * 0.05f;
    struct CachedChar { unsigned int unicode; float xadvance; };
    std::vector<CachedChar> cached;
    float total_w = 0;
    const unsigned char* p = (const unsigned char*)text;
    
    // 1. UTF-8 解码与宽度预计算
    while (*p) {
        unsigned int u = 0; int step = 0;
        if (*p < 0x80) { u = *p; step = 1; } // 单字节 (ASCII)
        else if ((*p & 0xE0) == 0xE0) { u = ((*p & 0x0F) << 12) | ((*(p+1) & 0x3F) << 6) | (*(p+2) & 0x3F); step = 3; } // 三字节 (常用中文)
        else { step = 1; u = *p; }
        
        auto it = glyphCache.find(u);
        float adv = (it != glyphCache.end()) ? it->second.xadvance * px : (u > 128 ? px * 24.0f : px * 12.0f);
        cached.push_back({u, adv});
        total_w += adv;
        p += step;
    }
    
    // 2. 处理文字对齐 (左对齐、居中、右对齐)
    float ox = x;
    if (align == CENTER) ox -= total_w * 0.5f;
    else if (align == RIGHT) ox -= total_w;
    
    auto renderString = [&](float startX, float startY, float cr, float cg, float cb, float ca) {
        float curX = startX;
        for (const auto& cc : cached) {
            auto it = glyphCache.find(cc.unicode);
            if (it != glyphCache.end()) {
                const GlyphInfo& gi = it->second;
                // 计算字符在屏幕上的矩形位置
                float x1 = curX + gi.xoff * px;
                float x2 = x1 + gi.w * px;
                float y2 = startY - gi.yoff * px;
                float y1 = y2 - gi.h * px;
                
                // 添加两个三角形（6个顶点）组成一个字符矩形
                addVertex(x1, y1, cr, cg, cb, ca, gi.u1, gi.v2);
                addVertex(x2, y1, cr, cg, cb, ca, gi.u2, gi.v2);
                addVertex(x2, y2, cr, cg, cb, ca, gi.u2, gi.v1);
                addVertex(x2, y2, cr, cg, cb, ca, gi.u2, gi.v1);
                addVertex(x1, y2, cr, cg, cb, ca, gi.u1, gi.v1);
                addVertex(x1, y1, cr, cg, cb, ca, gi.u1, gi.v2);
            }
            curX += cc.xadvance;
        }
    };
    
    // 绘制阴影（偏移一点坐标并使用半透明黑色）
    if (shadow) renderString(ox + size * 0.01f, y + size * 0.01f, 0, 0, 0, a * 0.5f);
    // 绘制主体文字
    renderString(ox, y, r, g, b, a);
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
    glBindTexture(GL_TEXTURE_2D, fontTexture);
    glDrawArrays(GL_TRIANGLES, 0, (GLsizei)(vertices.size() / 8));
  }

  // 绘制星球图标 (Procedural 2.5D Thumbnail)
  void drawPlanetIcon(float cx, float cy, float radius, const CelestialBody& body, float t_icon) {
    // 1. 底座背景 (半透明黑)
    addCircle(cx, cy, radius * 1.1f, 0, 0, 0, 0.6f);
    
    // 2. 模拟 3D 旋转的投影 (类似姿态球但更简化)
    // 旋转四元数: 绕 Y 轴缓慢旋转
    float rot_ang = t_icon * 0.5f;
    Quat icon_rot = Quat::fromAxisAngle(Vec3(0, 1, 0), rot_ang);
    
    auto project_icon = [&](Vec3 p) -> Vec2 {
        Vec3 pR = icon_rot.conjugate().rotate(p);
        if (pR.y < 0) return {999, 999}; 
        return {cx + pR.x * radius, cy + pR.z * radius};
    };

    auto getSpherePt = [&](float lat_deg, float lon_deg) -> Vec3 {
        float phi = lat_deg * (PI / 180.0f), theta = lon_deg * (PI / 180.0f);
        return { cosf(phi) * sinf(theta), sinf(phi), cosf(phi) * cosf(theta) };
    };

    // 绘制色块与纹理感
    int lat_steps = 10;
    int lon_steps = 20;
    for (int i = 0; i < lat_steps; i++) {
        float lat1 = -90.0f + (float)i / lat_steps * 180.0f;
        float lat2 = -90.0f + (float)(i + 1) / lat_steps * 180.0f;
        for (int j = 0; j < lon_steps; j++) {
            float lon1 = (float)j / lon_steps * 360.0f;
            float lon2 = (float)(j + 1) / lon_steps * 360.0f;
            Vec3 v1 = getSpherePt(lat1, lon1), v2 = getSpherePt(lat1, lon2);
            Vec3 v3 = getSpherePt(lat2, lon2), v4 = getSpherePt(lat2, lon1);
            Vec2 p1 = project_icon(v1), p2 = project_icon(v2), p3 = project_icon(v3), p4 = project_icon(v4);
            
            if (p1.x < 900) {
                // 基础颜色
                float cr = body.r, cg = body.g, cb = body.b;
                
                // 简单噪声纹理 (基于经纬度)
                float noise = hash11(lat1 * 0.13f + lon1 * 0.17f);
                float dark = 0.85f + 0.15f * noise;
                
                // 极地效果
                if (std::abs(lat1) > 75.0f && (body.type == TERRESTRIAL || body.type == MOON)) {
                    cr = std::min(1.0f, cr + 0.4f); cg = std::min(1.0f, cg + 0.4f); cb = std::min(1.0f, cb + 0.4f);
                }
                
                // 气态巨行星条带
                if (body.type == GAS_GIANT || body.type == RINGED_GAS_GIANT) {
                    float band = sinf(lat1 * 0.2f) * 0.5f + 0.5f;
                    dark *= (0.7f + 0.3f * band);
                }

                // 光照阴影 (假定光从右上角来)
                float light = fmaxf(0.1f, v1.x * 0.5f + v1.y * 0.5f + 0.5f);
                
                auto safeAdd = [&](Vec2 va, Vec2 vb, Vec2 vc) {
                    if (va.x > 900 || vb.x > 900 || vc.x > 900) return;
                    addVertexWhite(va.x, va.y, cr * dark * light, cg * dark * light, cb * dark * light, 1.0f);
                    addVertexWhite(vb.x, vb.y, cr * dark * light, cg * dark * light, cb * dark * light, 1.0f);
                    addVertexWhite(vc.x, vc.y, cr * dark * light, cg * dark * light, cb * dark * light, 1.0f);
                };
                safeAdd(p1, p2, p3); safeAdd(p1, p3, p4);
            }
        }
    }
    
    // 大气光圈 (如果是恒星或行星)
    if (body.type != MOON) {
        addAtmosphereGlow(cx, cy, radius, rot_ang);
    }
    
    // 描边
    addCircleOutline(cx, cy, radius, 0.002f, 1, 1, 1, 0.4f);
  }

  ~Renderer() {
    glDeleteVertexArrays(1, &VAO);
    glDeleteBuffers(1, &VBO);
    glDeleteProgram(shaderProgram);
    glDeleteTextures(1, &fontTexture);
  }
};
