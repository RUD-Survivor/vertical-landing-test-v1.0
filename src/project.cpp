#include<glad/glad.h>
#include <GLFW/glfw3.h>
#include <algorithm>
#include <chrono>
#include <cmath>

#include "math3d.h"
#include "renderer3d.h"
#include "rocket_state.h"
#include "physics_system.h"
#include "control_system.h"

#include <iostream>
#include <string>
#include <thread>
#include <vector>

using namespace std;

// ==========================================
// Part X: 火箭建造系统部件定义
// ==========================================

// 鼠标滚轮全局变量
static float g_scroll_y = 0.0f;
static void scroll_callback(GLFWwindow* /*w*/, double /*xoffset*/, double yoffset) {
  g_scroll_y += (float)yoffset;
}
// ==========================================
// Part 0: 数学常量与辅助工具
// ==========================================

// 简单的二维向量工具
struct Vec2 {
  double x, y;
};
Vec2 rotateVec(double x, double y, double angle) {
  double s = sin(angle);
  double c = cos(angle);
  return {x * c - y * s, x * s + y * c};
}
float my_lerp(float a, float b, float t) { return a + t * (b - a); }

// ==========================================
// Part 1: 现代 OpenGL 渲染引擎 (升级版：支持旋转)
// ==========================================
const char *vertexShaderSource = R"(
    #version 330 core
    layout (location = 0) in vec2 aPos;
    layout (location = 1) in vec4 aColor;
    out vec4 ourColor;
    void main() {
        gl_Position = vec4(aPos.x, aPos.y, 0.0, 1.0);
        ourColor = aColor;
    }
)";

const char *fragmentShaderSource = R"(
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
  vector<float> vertices;
  unsigned int compileShader(unsigned int type, const char *source) {
    unsigned int id = glCreateShader(type);
    glShaderSource(id, 1, &source, NULL);
    glCompileShader(id);
    return id;
  }

public:
  Renderer() {
    unsigned int v = compileShader(GL_VERTEX_SHADER, vertexShaderSource);
    unsigned int f = compileShader(GL_FRAGMENT_SHADER, fragmentShaderSource);
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
    // 预分配更大的显存
    glBufferData(GL_ARRAY_BUFFER, sizeof(float) * 5000000, NULL,
                 GL_DYNAMIC_DRAW);

    // 【核心修改】：步长从 5 改为 6，因为多了 Alpha 通道
    // 位置属性 (location=0): 偏移 0
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 6 * sizeof(float),
                          (void *)0);
    glEnableVertexAttribArray(0);
    // 颜色属性 (location=1): 偏移 2个float，读取 4个float (RGBA)
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
  // 旋转三角形 (已修复 alpha 传透)
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
  void addCircle(float cx, float cy, float radius, float r, float g, float b) {
    int segments = 360;
    for (int i = 0; i < segments; i++) {
      float theta1 = 2.0f * PI * float(i) / float(segments);
      float theta2 = 2.0f * PI * float(i + 1) / float(segments);
      addVertex(cx, cy, r, g, b, 1.0f);
      addVertex(cx + radius * cos(theta1), cy + radius * sin(theta1), r, g, b,
                1.0f);
      addVertex(cx + radius * cos(theta2), cy + radius * sin(theta2), r, g, b,
                1.0f);
    }
  }

  // 精细地球渲染：大陆级噪声 + 不规则海岸线 + 深浅海洋 + 森林/沙漠/冰盖
  void addEarthWithContinents(float cx, float cy, float radius,
                              float cam_angle) {
    int segments = 3600;

    // 预定义 8 块大陆的中心和宽度（用固定seed生成，保证每帧一致）
    const int NUM_CONTINENTS = 8;
    float cont_center[NUM_CONTINENTS];
    float cont_half_w[NUM_CONTINENTS];
    for (int c = 0; c < NUM_CONTINENTS; c++) {
      // 大陆中心均匀分布 + 随机偏移
      cont_center[c] = (float)c / NUM_CONTINENTS + hash11(c * 9973) * 0.08f;
      if (cont_center[c] > 1.0f) cont_center[c] -= 1.0f;
      // 大陆半宽度：0.03 ~ 0.10（占地球周长的 3%~10%）
      cont_half_w[c] = 0.03f + hash11(c * 7333) * 0.07f;
    }

    for (int i = 0; i < segments; i++) {
      float theta1 = 2.0f * PI * float(i) / float(segments) + cam_angle;
      float theta2 = 2.0f * PI * float(i + 1) / float(segments) + cam_angle;

      float pos = float(i) / float(segments); // 0~1 归一化位置

      // 判断是否在任何大陆范围内
      bool is_land = false;
      for (int c = 0; c < NUM_CONTINENTS; c++) {
        float dist = abs(pos - cont_center[c]);
        if (dist > 0.5f) dist = 1.0f - dist; // 环绕处理
        // 海岸线细节扰动
        float coast_noise = hash11(i * 137 + c * 5171) * 0.015f;
        if (dist < cont_half_w[c] + coast_noise) {
          is_land = true;
          break;
        }
      }

      float biome_hash = hash11(i * 419);
      float r, g, b;
      float lat = abs(float(i % 1800) - 900.0f) / 900.0f;

      if (!is_land) {
        // 海洋
        float depth = hash11(i * 503) * 0.3f;
        r = 0.05f + lat * 0.1f;
        g = 0.25f + depth * 0.15f + lat * 0.15f;
        b = 0.55f + depth * 0.25f + lat * 0.1f;
      } else {
        // 陆地
        if (lat > 0.85f) {
          // 极地冰盖
          r = 0.85f;
          g = 0.9f;
          b = 0.95f;
        } else if (lat < 0.3f && biome_hash > 0.5f) {
          // 热带沙漠
          r = 0.7f + biome_hash * 0.15f;
          g = 0.55f + biome_hash * 0.1f;
          b = 0.3f;
        } else {
          // 森林/草原
          float lush = hash11(i * 631) * 0.15f;
          r = 0.15f + lush;
          g = 0.4f + lush + lat * 0.1f;
          b = 0.12f + lush * 0.5f;
        }
      }
      addVertex(cx, cy, r, g, b, 1.0f);
      addVertex(cx + radius * cos(theta1), cy + radius * sin(theta1), r, g, b,
                1.0f);
      addVertex(cx + radius * cos(theta2), cy + radius * sin(theta2), r, g, b,
                1.0f);
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
      addVertex(cx + ir * cos(t1), cy + ir * sin(t1), 0.4f, 0.6f, 1.0f, 0.4f);
      addVertex(cx + ir * cos(t2), cy + ir * sin(t2), 0.4f, 0.6f, 1.0f, 0.4f);
      addVertex(cx + or_ * cos(t1), cy + or_ * sin(t1), 0.3f, 0.5f, 0.9f, 0.0f);
      addVertex(cx + or_ * cos(t1), cy + or_ * sin(t1), 0.3f, 0.5f, 0.9f, 0.0f);
      addVertex(cx + ir * cos(t2), cy + ir * sin(t2), 0.4f, 0.6f, 1.0f, 0.4f);
      addVertex(cx + or_ * cos(t2), cy + or_ * sin(t2), 0.3f, 0.5f, 0.9f, 0.0f);
    }
  }

  // 七段数码管渲染器
  void drawDigit(float x, float y, int digit, float size, float r, float g,
                 float b, float a = 1.0f) {
    float sw = size * 0.6f, sh = size * 0.12f;
    float hw = sw / 2.0f, hh = size / 2.0f, qh = size / 4.0f;
    const int segs[10][7] = {{1, 1, 1, 0, 1, 1, 1}, {0, 1, 1, 0, 0, 0, 0},
                             {1, 1, 0, 1, 0, 1, 1}, {1, 1, 1, 1, 0, 0, 1},
                             {0, 1, 1, 1, 1, 0, 0}, {1, 0, 1, 1, 1, 0, 1},
                             {1, 0, 1, 1, 1, 1, 1}, {1, 1, 1, 0, 0, 0, 0},
                             {1, 1, 1, 1, 1, 1, 1}, {1, 1, 1, 1, 1, 0, 1}};
    if (digit < 0 || digit > 9)
      return;
    float d = 0.25f;
    auto segH = [&](float sx, float sy, bool on) {
      addRect(sx, sy, sw, sh, on ? r : r * d, on ? g : g * d, on ? b : b * d,
              on ? a : a * 0.2f);
    };
    auto segV = [&](float sx, float sy, bool on) {
      addRect(sx, sy, sh, qh, on ? r : r * d, on ? g : g * d, on ? b : b * d,
              on ? a : a * 0.2f);
    };
    segH(x, y + hh, segs[digit][0]);
    segV(x + hw, y + qh, segs[digit][1]);
    segV(x + hw, y - qh, segs[digit][2]);
    segH(x, y, segs[digit][3]);
    segV(x - hw, y + qh, segs[digit][4]);
    segV(x - hw, y - qh, segs[digit][5]);
    segH(x, y - hh, segs[digit][6]);
  }
  void drawNumber(float x, float y, int number, float digitSize, float r,
                  float g, float b, float a = 1.0f) {
    bool neg = number < 0;
    if (neg)
      number = -number;
    char buf[16];
    int len = 0;
    if (number == 0) {
      buf[len++] = 0;
    } else {
      while (number > 0 && len < 15) {
        buf[len++] = (char)(number % 10);
        number /= 10;
      }
    }
    float sp = digitSize * 0.8f;
    float sx = x - (len * sp) / 2.0f + sp / 2.0f;
    for (int i = len - 1; i >= 0; i--)
      drawDigit(sx + (len - 1 - i) * sp, y, buf[i], digitSize, r, g, b, a);
    if (neg)
      addRect(sx - sp, y, digitSize * 0.4f, digitSize * 0.1f, r, g, b, a);
  }

  // 3x5 像素字体绘制单位标签
  void drawLabel(float x, float y, const char* text, float size, float r,
                 float g, float b, float a = 1.0f) {
    auto getGlyph = [](char c) -> int {
      switch (c) {
        case 'm': return 0b101111101101101;
        case 's': return 0b011100010001110;
        case 'k': return 0b101110100110101;
        case 'g': return 0b011101011011010;
        case '/': return 0b001001010100100;
        case '%': return 0b101001010100101;
        default:  return 0;
      }
    };
    float px = size * 0.18f;
    float cw = px * 4.0f;
    int idx = 0;
    while (text[idx]) {
      int gl = getGlyph(text[idx]);
      if (gl) {
        float ox = x + idx * cw;
        for (int row = 0; row < 5; row++)
          for (int col = 0; col < 3; col++)
            if (gl & (1 << (14 - row * 3 - col)))
              addRect(ox + col * px, y + (2 - row) * px,
                      px * 0.9f, px * 0.9f, r, g, b, a);
      }
      idx++;
    }
  }

  void endFrame() {
    if (vertices.empty())
      return;
    glUseProgram(shaderProgram);
    glBindVertexArray(VAO);
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferSubData(GL_ARRAY_BUFFER, 0, vertices.size() * sizeof(float),
                    vertices.data());
    glDrawArrays(GL_TRIANGLES, 0, vertices.size() / 6);
  }
  ~Renderer() {
    glDeleteVertexArrays(1, &VAO);
    glDeleteBuffers(1, &VBO);
    glDeleteProgram(shaderProgram);
  }
};

// ==========================================
// Part 2: Explorer Class -> Replaced by ECS (RocketState, PhysicsSystem)
// ==========================================

// ==========================================
// Part 3: 主函数
// ==========================================

void framebuffer_size_callback(GLFWwindow *window, int width, int height) {
  glViewport(0, 0, width, height);
}

Renderer *renderer;

// 根据当前状态，计算并画出预测轨道
void drawOrbit(Renderer *renderer, double px, double py, double vx, double vy,
               double body_x, double body_y, double body_vx, double body_vy, 
               double mu, double body_radius,
               double scale, float cx, float cy, double cam_angle, double cam_rocket_r) {
  double rel_px = px - body_x;
  double rel_py = py - body_y;               
  double r_mag = sqrt(rel_px * rel_px + rel_py * rel_py);

  // 【物理修复】：相对参考系的速度必须相减！否则开普勒轨道会变成花瓣形！
  double rel_vx = vx - body_vx;
  double rel_vy = vy - body_vy;

  double h = rel_px * rel_vy - rel_py * rel_vx; // 轨道角动量
  if (abs(h) < 1.0)
    return;

  // 计算偏心率矢量 (e)
  double ex = (rel_vy * h) / mu - rel_px / r_mag;
  double ey = (-rel_vx * h) / mu - rel_py / r_mag;
  double e_mag = sqrt(ex * ex + ey * ey);

  double p = (h * h) / mu;      // 半通径
  double omega = atan2(ey, ex); // 近地点幅角

  int segments = e_mag > 0.9 ? 8000 : 2000; // 轨道分段渲染
  float prev_x = 0, prev_y = 0;
  bool first = true;

  double sin_c = sin(cam_angle);
  double cos_c = cos(cam_angle);

  for (int i = 0; i <= segments; i++) {
    double nu = 2.0 * PI * i / segments; // 真近点角
    double denom = 1.0 + e_mag * cos(nu - omega);
    if (denom <= 0.05) {
      first = true;
      continue; // 忽略逃逸轨道的无限远端
    }

    double r_nu = p / denom;
    if (r_nu > body_radius * 50.0 && r_nu > 1000000000.0) {
      first = true;
      continue;
    }

    double x_orbit_rel = r_nu * cos(nu);
    double y_orbit_rel = r_nu * sin(nu);
    
    // 转回地球中心坐标系
    double x_abs = x_orbit_rel + body_x;
    double y_abs = y_orbit_rel + body_y;

    // 旋转映射到摄像机屏幕
    double rx = x_abs * cos_c - y_abs * sin_c;
    double ry = x_abs * sin_c + y_abs * cos_c;
    float screen_x = (float)(rx * scale + cx);
    float screen_y = (float)((ry - cam_rocket_r) * scale + cy);

    if (!first) {
      // 大气层内或地下显示红色警告，安全轨道显示为黄色/绿色
      float r_col = 0.0f, g_col = 1.0f, b_col = 0.0f; // 默认绿色
      if (r_nu < body_radius + (body_radius > 10000000 ? 0.0 : 80000.0)) {
        r_col = 1.0f;
        g_col = 0.0f;
        b_col = 0.0f;
      } else if (r_nu < body_radius * 1.5) {
        // 近地轨道泛黄
        r_col = 1.0f;
        g_col = 0.8f;
        b_col = 0.0f;
      }

      // 画出预测轨迹线
      float thickness = 0.005f;
      float dx = screen_x - prev_x, dy = screen_y - prev_y;
      float len = sqrt(dx * dx + dy * dy);
      if (len > 0 && len < 0.5f) { // 避免突变的长线
        float nx = -dy / len * thickness, ny = dx / len * thickness;
        renderer->addVertex(prev_x + nx, prev_y + ny, r_col, g_col, b_col, 1.0f);
        renderer->addVertex(prev_x - nx, prev_y - ny, r_col, g_col, b_col, 1.0f);
        renderer->addVertex(screen_x + nx, screen_y + ny, r_col, g_col, b_col, 1.0f);
        renderer->addVertex(screen_x + nx, screen_y + ny, r_col, g_col, b_col, 1.0f);
        renderer->addVertex(prev_x - nx, prev_y - ny, r_col, g_col, b_col, 1.0f);
        renderer->addVertex(screen_x - nx, screen_y - ny, r_col, g_col, b_col, 1.0f);
      } else {
        first = true;
      }
    }
    prev_x = screen_x;
    prev_y = screen_y;
    first = false;
  }
}

// ==========================================================
// Part 5: 火箭建造系统 (Rocket Builder)
// ==========================================================
struct RocketPart {
  const char* name;
  float dry_mass;     // 干重 (kg)
  float fuel;         // 燃料量 (kg)
  float isp;          // 比冲 (s)，仅引擎有效
  float consumption;  // 燃料消耗率 (kg/s)，仅引擎有效
  float height_add;   // 增加的高度 (m)
  float r, g, b;      // 显示颜色
};

// 鼻锥选项
const RocketPart NOSE_OPTIONS[] = {
  {"Standard",    500,  0, 0, 0, 8,   0.85f, 0.85f, 0.9f},
  {"Aerodynamic", 350,  0, 0, 0, 10,  0.7f,  0.7f,  0.75f},
  {"Payload",     1200, 0, 0, 0, 12,  0.9f,  0.9f,  0.85f},
};
// 燃料箱选项
const RocketPart TANK_OPTIONS[] = {
  {"Small 50t",   3000,  50000,  0, 0, 30, 0.92f, 0.92f, 0.92f},
  {"Medium 100t", 5000,  100000, 0, 0, 40, 0.90f, 0.90f, 0.92f},
  {"Large 200t",  8000,  200000, 0, 0, 55, 0.88f, 0.88f, 0.92f},
};
// 引擎选项
const RocketPart ENGINE_OPTIONS[] = {
  {"Raptor",   2000, 0, 1500, 100, 4,  0.35f, 0.35f, 0.38f},
  {"Merlin",   1500, 0, 1200, 80,  3,  0.4f,  0.35f, 0.3f},
  {"SRB",      3000, 0, 800,  200, 5,  0.5f,  0.4f,  0.3f},
};
const int NUM_OPTIONS = 3;

void drawBuilderUI(Renderer* r, int col, int nose_sel, int tank_sel, int eng_sel) {
  // 背景
  r->addRect(0.0f, 0.0f, 2.0f, 2.0f, 0.08f, 0.08f, 0.12f, 1.0f);

  // 星空点缀
  for (int i = 0; i < 80; i++) {
    float sx = hash11(i * 3917) * 2.0f - 1.0f;
    float sy = hash11(i * 7121) * 2.0f - 1.0f;
    float ss = 0.002f + hash11(i * 2131) * 0.003f;
    r->addRect(sx, sy, ss, ss, 1.0f, 1.0f, 1.0f, 0.4f + hash11(i * 991) * 0.4f);
  }

  // 标题
  float title_size = 0.04f;
  r->drawLabel(-0.15f, 0.85f, "m", title_size * 1.5f, 0.3f, 0.8f, 1.0f);

  // === 三列部件选择 ===
  const char* col_names[] = {"nose", "tank", "engine"};  // 显示用 (不使用)  
  float col_x[] = {-0.65f, -0.65f, -0.65f};
  float col_y_start[] = {0.65f, 0.25f, -0.15f};
  int selections[] = {nose_sel, tank_sel, eng_sel};
  const RocketPart* options[] = {NOSE_OPTIONS, TANK_OPTIONS, ENGINE_OPTIONS};
  const char* headers[] = {"< NOSE >", "< TANK >", "< ENGINE >"};

  for (int c = 0; c < 3; c++) {
    float hx = -0.65f, hy = col_y_start[c] + 0.12f;
    // 分类标题
    float hdr_r = (c == col) ? 0.2f : 0.5f;
    float hdr_g = (c == col) ? 0.9f : 0.5f;
    float hdr_b = (c == col) ? 0.4f : 0.5f;
    // 用数码管画列号
    r->drawNumber(hx - 0.15f, hy, c + 1, 0.025f, hdr_r, hdr_g, hdr_b);

    for (int i = 0; i < NUM_OPTIONS; i++) {
      float iy = col_y_start[c] - i * 0.1f;
      bool selected = (selections[c] == i);
      bool active_col = (col == c);

      // 选中高亮背景
      if (selected) {
        float bg_alpha = active_col ? 0.4f : 0.15f;
        float bg_r = active_col ? 0.1f : 0.15f;
        float bg_g = active_col ? 0.4f : 0.2f;
        float bg_b = active_col ? 0.2f : 0.25f;
        r->addRect(hx + 0.1f, iy, 0.35f, 0.08f, bg_r, bg_g, bg_b, bg_alpha);
        // 选中指示器
        r->addRect(hx - 0.05f, iy, 0.015f, 0.04f, 0.2f, 1.0f, 0.4f, 0.9f);
      }

      // 部件颜色块
      const RocketPart& part = options[c][i];
      r->addRect(hx, iy, 0.03f, 0.05f, part.r, part.g, part.b, 0.9f);

      // 参数数字
      float nr = selected ? 0.9f : 0.5f;
      float ng = selected ? 0.9f : 0.5f;
      float nb = selected ? 0.9f : 0.5f;
      if (c == 1) { // 燃料箱显示燃料量
        r->drawNumber(hx + 0.12f, iy, (int)(part.fuel / 1000), 0.02f, nr, ng, nb);
        r->drawLabel(hx + 0.23f, iy, "kg", 0.015f, nr * 0.7f, ng * 0.7f, nb * 0.7f);
      } else if (c == 2) { // 引擎显示ISP
        r->drawNumber(hx + 0.12f, iy, (int)part.isp, 0.02f, nr, ng, nb);
        r->drawLabel(hx + 0.23f, iy, "s", 0.015f, nr * 0.7f, ng * 0.7f, nb * 0.7f);
      } else { // 鼻锥显示质量
        r->drawNumber(hx + 0.12f, iy, (int)part.dry_mass, 0.02f, nr, ng, nb);
        r->drawLabel(hx + 0.23f, iy, "kg", 0.015f, nr * 0.7f, ng * 0.7f, nb * 0.7f);
      }
    }
  }

  // === 右侧：火箭预览 ===
  float preview_x = 0.35f;
  float preview_y = 0.1f;
  float rw = 0.06f; // 火箭宽度

  const RocketPart& nose = NOSE_OPTIONS[nose_sel];
  const RocketPart& tank = TANK_OPTIONS[tank_sel];
  const RocketPart& eng  = ENGINE_OPTIONS[eng_sel];

  float total_h = (nose.height_add + tank.height_add + eng.height_add) * 0.005f;

  // 引擎（底部）
  float eng_h = eng.height_add * 0.005f;
  float eng_y = preview_y - total_h / 2.0f + eng_h / 2.0f;
  r->addRotatedTri(preview_x, eng_y - eng_h * 0.3f, rw * 1.3f, eng_h * 0.6f,
                   PI, eng.r, eng.g, eng.b);
  r->addRect(preview_x, eng_y + eng_h * 0.1f, rw, eng_h * 0.4f,
             eng.r * 0.8f, eng.g * 0.8f, eng.b * 0.8f);

  // 燃料箱（中段）
  float tank_h = tank.height_add * 0.005f;
  float tank_y = eng_y + eng_h / 2.0f + tank_h / 2.0f;
  r->addRect(preview_x, tank_y, rw, tank_h, tank.r, tank.g, tank.b);
  // 级间段
  r->addRect(preview_x, tank_y + tank_h / 2.0f, rw * 1.05f, 0.008f,
             0.2f, 0.2f, 0.2f);
  r->addRect(preview_x, tank_y - tank_h / 2.0f, rw * 1.05f, 0.008f,
             0.2f, 0.2f, 0.2f);

  // 鼻锥（顶部）
  float nose_h = nose.height_add * 0.005f;
  float nose_y = tank_y + tank_h / 2.0f + nose_h / 2.0f;
  r->addRotatedTri(preview_x, nose_y + nose_h * 0.2f, rw * 0.9f, nose_h * 0.7f,
                   0.0f, nose.r, nose.g, nose.b);
  r->addRect(preview_x, nose_y - nose_h * 0.15f, rw * 0.95f, nose_h * 0.3f,
             nose.r * 0.95f, nose.g * 0.95f, nose.b * 0.95f);

  // === 底部统计面板 ===
  float stat_y = -0.65f;
  r->addRect(0.0f, stat_y, 1.6f, 0.2f, 0.05f, 0.05f, 0.08f, 0.7f);

  float total_mass = nose.dry_mass + tank.dry_mass + eng.dry_mass + tank.fuel;
  float dry_mass = nose.dry_mass + tank.dry_mass + eng.dry_mass;
  float delta_v = (float)(eng.isp * G0 * log((double)total_mass / (double)dry_mass));

  // 总质量
  r->drawNumber(-0.55f, stat_y + 0.03f, (int)(total_mass / 1000), 0.025f,
                0.9f, 0.9f, 0.9f);
  r->drawLabel(-0.4f, stat_y + 0.03f, "kg", 0.018f, 0.6f, 0.6f, 0.6f);

  // 燃料
  r->drawNumber(-0.1f, stat_y + 0.03f, (int)(tank.fuel / 1000), 0.025f,
                0.9f, 0.7f, 0.2f);
  r->drawLabel(0.05f, stat_y + 0.03f, "kg", 0.018f, 0.6f, 0.5f, 0.2f);

  // ΔV
  r->drawNumber(0.35f, stat_y + 0.03f, (int)delta_v, 0.025f,
                0.3f, 0.9f, 0.4f);
  r->drawLabel(0.5f, stat_y + 0.03f, "m/s", 0.018f, 0.2f, 0.6f, 0.3f);

  // 标签行
  r->drawNumber(-0.55f, stat_y - 0.04f, 0, 0.0f, 0, 0, 0, 0); // spacer
  r->drawLabel(-0.55f, stat_y - 0.04f, "m", 0.013f, 0.4f, 0.4f, 0.4f); // "mass" 简写

  // === 发射提示 ===
  // 闪烁效果
  float blink = 0.5f + 0.5f * (float)sin(glfwGetTime() * 3.0);
  r->addRect(0.35f, -0.85f, 0.25f, 0.06f, 0.1f, 0.4f * blink, 0.1f, 0.6f);
  r->drawLabel(0.28f, -0.85f, "s", 0.025f, 0.3f, 1.0f * blink, 0.4f);
}

void Report_Status(const RocketState& state, const ControlInput& input) {
  double apo = 0, peri = 0;
  PhysicsSystem::getOrbitParams(state, apo, peri);
  cout << "\n----------------------------------" << endl;
  cout << ">>> [MISSION CONTROL]: " << state.mission_msg << " <<<" << endl;
  cout << "----------------------------------" << endl;
  cout << "[Alt]: " << state.altitude << " m | [Vert_Vel]: " << state.velocity << " m/s";
  
  if (!state.auto_mode && state.altitude > 100000.0 && input.throttle < 0.01) {
      cout << " | [WARP READY]" << endl;
  } else {
      cout << endl;
  }
  double velocity_mag = sqrt(state.vx*state.vx + state.vy*state.vy + state.vz*state.vz);
  cout << "[Pos_X]: " << state.px << " m | [Horz_Vel]: " << state.vx << " m/s" << endl;
  cout << "[Angle]: " << state.angle * 180.0 / PI
       << " deg | [Throttle]: " << input.throttle * 100 << "%" << endl;
  cout << "[Ground_Horz_Vel]: " << state.local_vx
       << " m/s | [Orbit_Vel]: " << velocity_mag << " m/s" << endl;
  cout << "[Thrust]: " << state.thrust_power / 1000 << " kN | [Fuel]: " << state.fuel
       << " kg" << endl;
  cout << "[Apoapsis]: " << apo / 1000.0
       << " km | [Periapsis]: " << peri / 1000.0 << " km" << endl;
  cout << "[Status]: " << state.status << endl;
}

int main() {

  glfwInit();
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

  // 请求一个包含 4倍多重采样 的帧缓冲
  glfwWindowHint(GLFW_SAMPLES, 4);

  GLFWwindow *window = glfwCreateWindow(1000, 800, "3D Rocket Sim", NULL, NULL);
  if (!window) {
    glfwTerminate();
    return -1;
  }
  glfwMakeContextCurrent(window);
  glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);
  glfwSetScrollCallback(window, scroll_callback);

  if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
    return -1;
  // ：开启透明度混合
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  // 激活硬件级多重采样抗锯齿
  glEnable(GL_MULTISAMPLE);
  renderer = new Renderer();

  // =========================================================
  // BUILD 阶段：火箭组装界面
  // =========================================================
  int build_col = 0;     // 0=鼻锥, 1=燃料箱, 2=引擎
  int nose_sel = 0, tank_sel = 1, eng_sel = 0; // 默认选择
  bool build_done = false;

  // 方向键防抖
  bool up_prev = false, down_prev = false, left_prev = false, right_prev = false;

  while (!build_done && !glfwWindowShouldClose(window)) {
    glfwPollEvents();
    if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
      glfwSetWindowShouldClose(window, true);

    // 方向键导航（带防抖）
    bool up_now = glfwGetKey(window, GLFW_KEY_UP) == GLFW_PRESS;
    bool down_now = glfwGetKey(window, GLFW_KEY_DOWN) == GLFW_PRESS;
    bool left_now = glfwGetKey(window, GLFW_KEY_LEFT) == GLFW_PRESS;
    bool right_now = glfwGetKey(window, GLFW_KEY_RIGHT) == GLFW_PRESS;

    int* current_sel = (build_col == 0) ? &nose_sel : (build_col == 1) ? &tank_sel : &eng_sel;

    if (up_now && !up_prev) {
      *current_sel = (*current_sel - 1 + NUM_OPTIONS) % NUM_OPTIONS;
    }
    if (down_now && !down_prev) {
      *current_sel = (*current_sel + 1) % NUM_OPTIONS;
    }
    if (left_now && !left_prev) {
      build_col = (build_col - 1 + 3) % 3;
    }
    if (right_now && !right_prev) {
      build_col = (build_col + 1) % 3;
    }
    up_prev = up_now; down_prev = down_now;
    left_prev = left_now; right_prev = right_now;

    // SPACE = 发射
    if (glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_PRESS) {
      build_done = true;
    }

    // 渲染
    glClearColor(0.08f, 0.08f, 0.12f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);
    renderer->beginFrame();
    drawBuilderUI(renderer, build_col, nose_sel, tank_sel, eng_sel);
    renderer->endFrame();
    glfwSwapBuffers(window);

    this_thread::sleep_for(chrono::milliseconds(16));
  }

  // =========================================================
  // 用选好的部件构造 Explorer
  // =========================================================
  const RocketPart& sel_nose = NOSE_OPTIONS[nose_sel];
  const RocketPart& sel_tank = TANK_OPTIONS[tank_sel];
  const RocketPart& sel_eng  = ENGINE_OPTIONS[eng_sel];

  float total_dry = sel_nose.dry_mass + sel_tank.dry_mass + sel_eng.dry_mass;
  float total_fuel = sel_tank.fuel;
  float total_height = sel_nose.height_add + sel_tank.height_add + sel_eng.height_add;

  RocketConfig rocket_config = {
      total_dry,          // dry_mass
      3.7,                // diameter
      total_height,       // height
      1,                  // stages
      sel_eng.isp,        // specific_impulse
      sel_eng.consumption,// cosrate
      0.5                 // nozzle_area
  };

  RocketState rocket_state;
  rocket_state.fuel = total_fuel;
  rocket_state.status = PRE_LAUNCH;
  
  ControlInput control_input;

  // =========================================================
  // 初始化 3D 渲染器和网格
  // =========================================================
  Renderer3D* r3d = new Renderer3D();
  Mesh earthMesh = MeshGen::sphere(48, 64, 1.0f);  // 单位球，用 model 矩阵缩放
  Mesh rocketBody = MeshGen::cylinder(32, 1.0f, 1.0f);
  Mesh rocketNose = MeshGen::cone(32, 1.0f, 1.0f);
  Mesh rocketBox  = MeshGen::box(1.0f, 1.0f, 1.0f);
  int cam_mode_3d = 0; // 0=自由轨道, 1=跟踪, 2=全景
  static bool c_was_pressed = false;
  // 四元数轨道球相机
  Quat cam_quat; // 相机方位四元数 (单位四元数 = 初始位置)
  float cam_zoom_chase = 1.0f; // 轨道/跟踪模式缩放
  float cam_zoom_pan = 1.0f;   // 全景模式独立缩放
  double prev_mx = 0, prev_my = 0;
  bool mouse_dragging = false;

  cout << ">> ROCKET ASSEMBLED!" << endl;
  cout << ">>   Dry Mass: " << (int)total_dry << " kg" << endl;
  cout << ">>   Fuel: " << (int)total_fuel << " kg" << endl;
  cout << ">>   Height: " << (int)total_height << " m" << endl;
  cout << ">>   Engine ISP: " << (int)sel_eng.isp << " s" << endl;
  cout << ">> PRESS [SPACE] TO LAUNCH!" << endl;
  cout << ">> [TAB] Toggle Auto/Manual | [WASD] Thrust & Attitude" << endl;
  cout << ">> [Z] Full Throttle | [X] Kill Throttle | [1-4] Time Warp" << endl;
  cout << ">> [O] Toggle Orbit Display" << endl;

  double dt = 0.02; // 50Hz 物理步长
  
  static bool tab_was_pressed = false; // Tab 防抖

  struct DVec3 { double x, y, z; };
  struct TrajPoint { DVec3 e; DVec3 s; };
  std::vector<TrajPoint> traj_history; // 记录火箭历史飞行的 3D 轨迹点

  while ((rocket_state.status == PRE_LAUNCH || rocket_state.status == ASCEND || rocket_state.status == DESCEND) && !glfwWindowShouldClose(window)) {
    glfwPollEvents();
    if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
      glfwSetWindowShouldClose(window, true);
    if (glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_PRESS) {
      if (rocket_state.status == PRE_LAUNCH) rocket_state.status = ASCEND;
    }

    // --- C 键切换 3D 视角 ---
    bool c_now = glfwGetKey(window, GLFW_KEY_C) == GLFW_PRESS;
    if (c_now && !c_was_pressed) {
      cam_mode_3d = (cam_mode_3d + 1) % 3;
      const char* names[] = {"Orbit", "Chase", "Panorama"};
      cout << ">> Camera: " << names[cam_mode_3d] << endl;
    }
    c_was_pressed = c_now;

    // --- 鼠标轨道控制 (3D模式下右键拖动) ---
    {
      double mx, my;
      glfwGetCursorPos(window, &mx, &my);
      bool rmb = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS;
      if (rmb) {
        if (mouse_dragging) {
          float dx = (float)(mx - prev_mx) * 0.003f;
          float dy = (float)(my - prev_my) * 0.003f;
          // 四元数增量旋转：屏幕水平拖动 = 绕全局Y旋转，垂直拖动 = 绕局部X旋转
          // 用绕全局上方向旋转 (yaw) 和绕相机右方向旋转 (pitch)
          Quat yaw_rot = Quat::fromAxisAngle(Vec3(0.0f, 0.0f, 1.0f), -dx);
          // 相机的右方向 = cam_quat 旋转后的 X 轴
          Vec3 cam_right = cam_quat.rotate(Vec3(1.0f, 0.0f, 0.0f));
          Quat pitch_rot = Quat::fromAxisAngle(cam_right, -dy);
          cam_quat = (yaw_rot * pitch_rot * cam_quat).normalized();
        }
        mouse_dragging = true;
      } else {
        mouse_dragging = false;
      }
      prev_mx = mx;
      prev_my = my;

      // 滚轮缩放：根据当前模式分离缩放级别
      if (g_scroll_y != 0.0f) {
        if (cam_mode_3d == 2) {
            // 全景模式：允许极其庞大的缩放以俯瞰太阳系
            cam_zoom_pan *= powf(0.85f, g_scroll_y);
            if (cam_zoom_pan < 0.05f) cam_zoom_pan = 0.05f;
            if (cam_zoom_pan > 500000.0f) cam_zoom_pan = 500000.0f; 
        } else {
            // 轨道/跟踪模式：限制缩放防止火箭消失
            cam_zoom_chase *= powf(0.85f, g_scroll_y);
            if (cam_zoom_chase < 0.05f) cam_zoom_chase = 0.05f;
            if (cam_zoom_chase > 20.0f) cam_zoom_chase = 20.0f;
        }
        g_scroll_y = 0.0f;
      }
    }

    // --- H 键切换 HUD 显示 ---
    static bool show_hud = true;
    static bool h_was_pressed = false;
    if (glfwGetKey(window, GLFW_KEY_H) == GLFW_PRESS) {
      if (!h_was_pressed) {
        show_hud = !show_hud;
        h_was_pressed = true;
      }
    } else {
      h_was_pressed = false;
    }

    // --- Tab 键切换模式（带防抖）---
    if (glfwGetKey(window, GLFW_KEY_TAB) == GLFW_PRESS) {
      if (!tab_was_pressed) {
        rocket_state.auto_mode = !rocket_state.auto_mode;
        if (rocket_state.auto_mode) {
          rocket_state.pid_vert.reset();
          rocket_state.pid_pos.reset();
          rocket_state.pid_att.reset();
          rocket_state.pid_att_z.reset();
          rocket_state.mission_msg = ">> AUTOPILOT ENGAGED";
        } else {
          rocket_state.mission_msg = ">> MANUAL CONTROL ACTIVE";
        }
        tab_was_pressed = true;
      }
    } else {
      tab_was_pressed = false;
    }

    // --- O 键切换轨道显示 ---
    static bool show_orbit = true;
    static bool o_was_pressed = false;
    static bool orbit_reference_sun = false; // 0=地球, 1=太阳
    if (glfwGetKey(window, GLFW_KEY_O) == GLFW_PRESS) {
      if (!o_was_pressed) {
        show_orbit = !show_orbit; // 这里为了兼容旧逻辑，如果是双击才算切换？不如我们用 R 键来切换参考系
        o_was_pressed = true;
      }
    } else {
      o_was_pressed = false;
    }

    static bool r_was_pressed = false;
    if (glfwGetKey(window, GLFW_KEY_R) == GLFW_PRESS) {
      if (!r_was_pressed) {
        orbit_reference_sun = !orbit_reference_sun;
        cout << "[REF FRAME] " << (orbit_reference_sun ? "SUN" : "EARTH") << endl;
        r_was_pressed = true;
      }
    } else {
      r_was_pressed = false;
    }
    
    // 全局帧计数器 (用于限制控制台打印频率)
    static int frame = 0;
    frame++;

    // --- 时间加速逻辑 ---
    static int time_warp = 1;
    // 条件：手动模式、没开推力(或者没燃料了)、处于真空(真空设为>100000m) 才可以开启极速加速 (5,6,7,8)
    bool can_super_warp = (!rocket_state.auto_mode && rocket_state.altitude > 100000.0 && (rocket_state.thrust_power == 0 || rocket_state.fuel <= 0));

    if (rocket_state.auto_mode) {
      if (glfwGetKey(window, GLFW_KEY_1) == GLFW_PRESS || glfwGetKey(window, GLFW_KEY_KP_1) == GLFW_PRESS) time_warp = 1;
      if (glfwGetKey(window, GLFW_KEY_2) == GLFW_PRESS || glfwGetKey(window, GLFW_KEY_KP_2) == GLFW_PRESS) time_warp = 10;
      if (glfwGetKey(window, GLFW_KEY_3) == GLFW_PRESS || glfwGetKey(window, GLFW_KEY_KP_3) == GLFW_PRESS) time_warp = 100;
      if (glfwGetKey(window, GLFW_KEY_4) == GLFW_PRESS || glfwGetKey(window, GLFW_KEY_KP_4) == GLFW_PRESS) time_warp = 1000;
      // 在自动模式中降级时间加速
      if (time_warp > 1000) time_warp = 1; 
    } else {
      if (glfwGetKey(window, GLFW_KEY_1) == GLFW_PRESS || glfwGetKey(window, GLFW_KEY_KP_1) == GLFW_PRESS) time_warp = 1;
      if (glfwGetKey(window, GLFW_KEY_2) == GLFW_PRESS || glfwGetKey(window, GLFW_KEY_KP_2) == GLFW_PRESS) time_warp = 10;
      if (glfwGetKey(window, GLFW_KEY_3) == GLFW_PRESS || glfwGetKey(window, GLFW_KEY_KP_3) == GLFW_PRESS) time_warp = 100;
      if (glfwGetKey(window, GLFW_KEY_4) == GLFW_PRESS || glfwGetKey(window, GLFW_KEY_KP_4) == GLFW_PRESS) time_warp = 1000;

      if (can_super_warp) {
          if (glfwGetKey(window, GLFW_KEY_5) == GLFW_PRESS || glfwGetKey(window, GLFW_KEY_KP_5) == GLFW_PRESS) { time_warp = 10000; cout << "WARP: 10K SPEED ENGAGED!" << endl; }
          if (glfwGetKey(window, GLFW_KEY_6) == GLFW_PRESS || glfwGetKey(window, GLFW_KEY_KP_6) == GLFW_PRESS) { time_warp = 100000; cout << "WARP: 100K SPEED ENGAGED!" << endl; }
          if (glfwGetKey(window, GLFW_KEY_7) == GLFW_PRESS || glfwGetKey(window, GLFW_KEY_KP_7) == GLFW_PRESS) { time_warp = 1000000; cout << "WARP: 1M SPEED ENGAGED!" << endl; }
          if (glfwGetKey(window, GLFW_KEY_8) == GLFW_PRESS || glfwGetKey(window, GLFW_KEY_KP_8) == GLFW_PRESS) { time_warp = 10000000; cout << "WARP: 10M SPEED ENGAGED!" << endl; }
      } else {
          // 如果条件不满足，强制降级到最高 1000倍
          if (time_warp > 1000) {
              time_warp = 1; 
              static int last_warn = 0;
              if (frame - last_warn > 60) {
                 cout << ">> WARP DISENGAGED: Unsafe conditions (Alt < 100km or Engine Active)" << endl;
                 last_warn = frame;
              }
          }
      }
    }

    ControlSystem::ManualInputs manual;
    manual.throttle_up = glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS;
    manual.throttle_down = glfwGetKey(window, GLFW_KEY_LEFT_CONTROL) == GLFW_PRESS;
    manual.throttle_max = glfwGetKey(window, GLFW_KEY_Z) == GLFW_PRESS;
    manual.throttle_min = glfwGetKey(window, GLFW_KEY_X) == GLFW_PRESS;
    manual.pitch_left = glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS || glfwGetKey(window, GLFW_KEY_LEFT) == GLFW_PRESS;
    manual.pitch_right = glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS || glfwGetKey(window, GLFW_KEY_RIGHT) == GLFW_PRESS;
    manual.pitch_forward = glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS || glfwGetKey(window, GLFW_KEY_UP) == GLFW_PRESS;
    manual.pitch_backward = glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS || glfwGetKey(window, GLFW_KEY_DOWN) == GLFW_PRESS;

    // --- 物理更新 ---
    if (time_warp > 1000) {
        // 超级时间加速！绕过气动和引擎模拟，直接当作质点切分迭代
        PhysicsSystem::FastGravityUpdate(rocket_state, rocket_config, dt * time_warp);
    } else {
        // 普通循环执行实现加速
        for (int i = 0; i < time_warp; i++) {
          if (rocket_state.auto_mode) ControlSystem::UpdateAutoPilot(rocket_state, rocket_config, control_input, dt);
          else ControlSystem::UpdateManualControl(rocket_state, control_input, manual, dt);
          PhysicsSystem::Update(rocket_state, rocket_config, control_input, dt);
          if (rocket_state.status == LANDED || rocket_state.status == CRASHED) break;
        }

        // --- 额外一步物理更新 (用于尾烟特效) ---
        if (time_warp == 1) {
            PhysicsSystem::EmitSmoke(rocket_state, rocket_config, dt);
            PhysicsSystem::UpdateSmoke(rocket_state, dt);
        }
    }

    // 只有每隔一定帧数才打印，防止控制台看不清
    if (frame % 10 == 0)
      Report_Status(rocket_state, control_input);

    this_thread::sleep_for(chrono::milliseconds(20)); // 限制帧率

    // 画面刷新
    // 画面刷新
    float alt_factor = (float)min(rocket_state.altitude / 50000.0, 1.0);
    if (cam_mode_3d == 2) {
       // 全景视角(Panorama)下背景应该始终为全黑(太空)
       glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    } else {
       // 地球表面起飞的蓝天渐变
       glClearColor(my_lerp(0.5f, 0.0f, alt_factor), my_lerp(0.7f, 0.0f, alt_factor),
                    my_lerp(1.0f, 0.0f, alt_factor), 1.0f);
    }
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // ================= 3D 渲染通道 =================
    {
      // 世界坐标缩放
      double ws_d = 0.001;
      float earth_r = (float)EARTH_RADIUS * (float)ws_d;

      // 2D→3D 坐标映射 (Double precision)
      double r_px = rocket_state.px * ws_d;
      double r_py = rocket_state.py * ws_d;
      double r_pz = rocket_state.pz * ws_d;
      
      // ===== 太阳位置与昼夜交替 (Double precision) =====
      double G_const_d = 6.67430e-11;
      double M_sun_d = 1.989e30;
      double GM_sun_d = G_const_d * M_sun_d;
      double au_meters_d = 149597870700.0;
      double sun_angular_vel = sqrt(GM_sun_d / (au_meters_d * au_meters_d * au_meters_d));
      
      double sun_angle_d = -1.2 + sun_angular_vel * rocket_state.sim_time;
      double sun_dist_d = au_meters_d * ws_d;
      double sun_px = cos(sun_angle_d) * sun_dist_d;
      double sun_py = sin(sun_angle_d) * sun_dist_d;
      double sun_pz = 0.0;
      
      float sun_radius = earth_r * 109.2f;

      // ===== 按键监听：视点切换 =====
      static int focus_target = 0; // 0=地球, 1=太阳
      static bool comma_prev = false;
      static bool period_prev = false;
      bool comma_now = glfwGetKey(window, GLFW_KEY_COMMA) == GLFW_PRESS;
      bool period_now = glfwGetKey(window, GLFW_KEY_PERIOD) == GLFW_PRESS;
      if (comma_now && !comma_prev) focus_target = 0;
      if (period_now && !period_prev) focus_target = 1;
      comma_prev = comma_now; period_prev = period_now;

      // --- FLOATING ORIGIN FIX (True Double Precision) ---
      // 决定渲染世界的绝对中心：Camera Target
      double ro_x = 0; double ro_y = 0; double ro_z = 0;
      if (cam_mode_3d == 0 || cam_mode_3d == 1) {
          ro_x = r_px; ro_y = r_py; ro_z = r_pz;
      } else {
          if (focus_target == 0) { ro_x = 0.0; ro_y = 0.0; ro_z = 0.0; }
          else { ro_x = sun_px; ro_y = sun_py; ro_z = sun_pz; }
      }
      
      // 计算相对于渲染中心的 3D 物体坐标 (直接转成 float 后不会再有远距离网格撕裂)
      Vec3 renderEarth((float)(0.0 - ro_x), (float)(0.0 - ro_y), (float)(0.0 - ro_z));
      Vec3 renderSun((float)(sun_px - ro_x), (float)(sun_py - ro_y), (float)(sun_pz - ro_z));
      Vec3 renderRocketBase((float)(r_px - ro_x), (float)(r_py - ro_y), (float)(r_pz - ro_z));

      // 更新全局光照方向
      r3d->lightDir = renderSun.normalized();

      // 【深空姿态锁定】：使用精度更高的几何距离防止抖动
      double dist_to_earth = sqrt(r_px*r_px + r_py*r_py + r_pz*r_pz);
      Vec3 rocketUp((float)(r_px / dist_to_earth), (float)(r_py / dist_to_earth), 0.0f);
      if (rocket_state.altitude > 2000000.0) { // 2000公里外，完全脱离近地轨道，进入深空
          rocketUp = Vec3(0.0f, 1.0f, 0.0f);
      }
      
      // ===== 构建火箭完整的 3D 朝向四元数 =====
      // 保证局部坐标系的单位正交
      double local_xy_mag = sqrt(rocketUp.x*rocketUp.x + rocketUp.y*rocketUp.y);
      Vec3 localRight(0,0,0);
      if (local_xy_mag > 1e-4) {
          localRight = Vec3((float)(-rocketUp.y/local_xy_mag), (float)(rocketUp.x/local_xy_mag), 0.0f);
      } else {
          localRight = Vec3(1.0f, 0.0f, 0.0f);
      }
      Vec3 localNorth = rocketUp.cross(localRight).normalized();

      // 构建 2D 面内主朝向
      float rocket_angle = (float)rocket_state.angle;
      Vec3 rocketDir2D = rocketUp * cosf(rocket_angle) + localRight * sinf(rocket_angle);

      // 构建对应的主旋转四元数
      Vec3 defaultUp(0.0f, 1.0f, 0.0f);
      Vec3 rotAxis = defaultUp.cross(rocketDir2D);
      float rotAngle = acosf(fminf(fmaxf(defaultUp.dot(rocketDir2D), -1.0f), 1.0f));
      Quat baseQuat;
      if (rotAxis.length() > 1e-6f)
        baseQuat = Quat::fromAxisAngle(rotAxis.normalized(), rotAngle);

      // 提取动态翻滚轴 (Rotated Right) 用作俯仰轴，防止 Gimbal Lock 导致 angle_z 偏航失效
      Vec3 dynamicRight = localNorth.cross(rocketDir2D).normalized();

      // 组合基础 2D 旋转与局部的轴外俯仰 (Pitch) 旋转
      Quat pitchQuat = Quat::fromAxisAngle(dynamicRight, (float)rocket_state.angle_z);
      Quat rocketQuat = pitchQuat * baseQuat;
      
      // 更新 rocketDir 为包含 3D 俯仰的真实朝向
      Vec3 rocketDir = rocketQuat.rotate(Vec3(0.0f, 1.0f, 0.0f));

      // 火箭尺寸
      float rocket_vis_scale = 1.0f;
      float rh = (float)rocket_config.height * (float)ws_d * rocket_vis_scale;
      float rw_3d = (float)rocket_config.diameter * (float)ws_d * 0.5f * rocket_vis_scale;

      // 火箭渲染锚点（将火箭向上偏移，解决2D物理质心带来的穿模问题）
      Vec3 renderRocketPos = renderRocketBase + rocketUp * (rh * 0.425f);

      // 相机设置
      int ww, wh;
      glfwGetFramebufferSize(window, &ww, &wh);
      float aspect = (float)ww / (float)wh;

      Vec3 camEye_rel, camTarget_rel, camUpVec;
      if (cam_mode_3d == 0) {
        float orbit_dist = rh * 8.0f * cam_zoom_chase;
        Vec3 cam_offset = cam_quat.rotate(Vec3(0.0f, 0.0f, orbit_dist));
        camEye_rel = renderRocketPos + cam_offset;
        camTarget_rel = renderRocketPos;
        camUpVec = cam_quat.rotate(Vec3(0.0f, 1.0f, 0.0f));
      } else if (cam_mode_3d == 1) {
        float chase_dist = rh * 8.0f * cam_zoom_chase;
        Vec3 chase_base = rocketDir * (-chase_dist * 0.4f) + rocketUp * (chase_dist * 0.15f);
        Vec3 slight_off = cam_quat.rotate(Vec3(0.0f, 0.0f, 1.0f));
        camEye_rel = renderRocketPos + chase_base + slight_off * (chase_dist * 0.05f);
        camTarget_rel = renderRocketPos + rocketDir * (rh * 3.0f);
        camUpVec = rocketUp;
      } else {
        float pan_dist = earth_r * 4.0f * cam_zoom_pan;
        if (focus_target == 0) {
            Vec3 cam_offset = cam_quat.rotate(Vec3(0.0f, 0.0f, pan_dist));
            camEye_rel = renderEarth + cam_offset;
            camTarget_rel = renderEarth;
        } else {
            Vec3 cam_offset = cam_quat.rotate(Vec3(0.0f, 0.0f, pan_dist * 110.0f));
            camEye_rel = renderSun + cam_offset;
            camTarget_rel = renderSun;
        }
        camUpVec = cam_quat.rotate(Vec3(0.0f, 1.0f, 0.0f));
      }

      // 动态远裁剪面 
      float cam_dist = (camEye_rel - camTarget_rel).length();
      float dist_to_earth_surf = fmaxf(1.0f, (camEye_rel - renderEarth).length() - earth_r);
      float far_plane = fmaxf(cam_dist * 3.0f, dist_to_earth_surf + earth_r * 3.0f);
      if (focus_target == 1 || cam_dist > sun_dist_d * 0.1) {
          far_plane = fmaxf(far_plane, (float)sun_dist_d * 2.5f);
      }
      
      Mat4 viewMat = Mat4::lookAt(camEye_rel, camTarget_rel, camUpVec);

      // =========== PASS 1: MACRO BACKGROUND ===========
      // 宏观天体专用的相机矩阵：基于相机距地表的高度动态推断 clipping
      float macro_near = fmaxf(0.1f, dist_to_earth_surf * 0.05f);
      if (focus_target == 1 && cam_dist > sun_dist_d * 0.5) macro_near = 10000.0f;
      macro_near = fmaxf(macro_near, 0.01f);
      macro_near = fminf(macro_near, fmaxf(0.5f, cam_dist * 0.5f));
      
      Mat4 macroProjMat = Mat4::perspective(0.8f, aspect, macro_near, far_plane);
      r3d->beginFrame(viewMat, macroProjMat, camEye_rel);

      // ===== 太阳物理本体 =====
      if (cam_mode_3d == 2) { 
          // 仅在全景模式渲染巨型的物理太阳模型避免遮盖火箭本体细节
          Mat4 sunModel = Mat4::scale(Vec3(sun_radius, sun_radius, sun_radius));
          sunModel = Mat4::translate(renderSun) * sunModel;
          // 复用 earthMesh，修改极高环境光(ambient=2.0)让其纯亮发白发黄
          r3d->drawMesh(earthMesh, sunModel, 1.0f, 0.95f, 0.9f, 1.0f, 2.0f); 
          
          // 地球绕日轨道线 (以太阳为中心画一个1AU半径的完美圆)
          std::vector<Vec3> earth_orbit_pts;
          // 新设计：使用一根线多次调用 drawRibbon，因为当前的 drawRibbon 只能单色。或者直接修改顶点！
          // 由于 drawRibbon 目前是定色，我们可以拆分成120段单独画。
          int orbit_res = 120;
          float orbit_w = earth_r * 0.05f * fmaxf(1.0f, fminf(cam_zoom_pan * 0.05f, 50000.0f));
          for (int i=0; i<orbit_res; i++) {
              float theta = (float)i / orbit_res * 6.2831853f;
              float next_theta = (float)(i+1) / orbit_res * 6.2831853f;
              
              double dx1 = cos(theta) * sun_dist_d;
              double dy1 = sin(theta) * sun_dist_d;
              Vec3 p1( (float)(sun_px - dx1 - ro_x), (float)(sun_py - dy1 - ro_y), (float)(-ro_z) );
              
              double dx2 = cos(next_theta) * sun_dist_d;
              double dy2 = sin(next_theta) * sun_dist_d;
              Vec3 p2( (float)(sun_px - dx2 - ro_x), (float)(sun_py - dy2 - ro_y), (float)(-ro_z) );

              float a1 = (float)theta;
              if (a1 < 0) a1 += 6.2831853f; // 0 to 2PI

              // 最亮处直接设为太阳经过的真正相位角
              float earth_a = fmod((float)sun_angle_d, 6.2831853f);
              if (earth_a < 0) earth_a += 6.2831853f;
              
              float delta = earth_a - a1;
              if (delta < 0) delta += 6.2831853f;
              float brightness = 1.0f - delta / 6.2831853f;
              brightness = fmaxf(0.1f, brightness);
              
              // Fade out the macro orbit line when camera is zooming in tightly
              float macro_fade = fminf(1.0f, fmaxf(0.0f, (cam_zoom_pan - 0.05f) / 0.1f));
              if (macro_fade > 0.01f) {
                 std::vector<Vec3> seg = {p1, p2};
                 r3d->drawRibbon(seg, orbit_w, 0.3f*brightness+0.1f, 0.7f*brightness+0.1f, 1.0f*brightness+0.2f, fmaxf(0.3f, 0.8f*brightness) * macro_fade);
              }
          }
      }

      // 地球
      Mat4 earthModel = Mat4::scale(Vec3(earth_r, earth_r, earth_r));
      earthModel = Mat4::translate(renderEarth) * earthModel;
      r3d->drawEarth(earthMesh, earthModel);

      // ===== 大气层散射壳 =====
      Mat4 atmoModel = Mat4::scale(Vec3(earth_r * 1.025f, earth_r * 1.025f, earth_r * 1.025f));
      atmoModel = Mat4::translate(renderEarth) * atmoModel;
      r3d->drawAtmosphere(earthMesh, atmoModel);

      // ===== 太阳与镜头光晕 (所有模式可见) =====
      r3d->drawSunAndFlare(renderSun, renderEarth, earth_r, ww, wh);

      // ===== 历史轨迹线 (实际走过的路径) =====
      {
        DVec3 curPos = {r_px, r_py, r_pz};
        if (traj_history.empty()) {
            traj_history.push_back({curPos, {r_px - sun_px, r_py - sun_py, r_pz - sun_pz}});
        } else {
            DVec3 bk = traj_history.back().e;
            double move_dist = sqrt((r_px-bk.x)*(r_px-bk.x) + (r_py-bk.y)*(r_py-bk.y) + (r_pz-bk.z)*(r_pz-bk.z));
            if (move_dist > earth_r * 0.002) {
               traj_history.push_back({curPos, {r_px - sun_px, r_py - sun_py, r_pz - sun_pz}});
               if (traj_history.size() > 800) {
                 traj_history.erase(traj_history.begin());
               }
            }
        }
        
        // 渲染历史轨迹 (更亮实线: 黄绿色), 增加基于相机拉远的线宽补偿 (仅在 Panorama 显示)
        if (cam_mode_3d == 2 && traj_history.size() >= 2) {
           float hist_w = earth_r * 0.003f * fmaxf(1.0f, cam_zoom_pan * 0.5f);
           float macro_fade = fminf(1.0f, fmaxf(0.0f, (cam_zoom_pan - 0.05f) / 0.1f));
           if (macro_fade > 0.01f) {
              std::vector<Vec3> relative_traj;
              for(auto& pt : traj_history) {
                 double w_px, w_py, w_pz;
                 if (orbit_reference_sun) {
                     w_px = sun_px + pt.s.x;
                     w_py = sun_py + pt.s.y;
                     w_pz = sun_pz + pt.s.z;
                 } else {
                     w_px = pt.e.x;
                     w_py = pt.e.y;
                     w_pz = pt.e.z;
                 }
                 relative_traj.push_back(Vec3((float)(w_px - ro_x), (float)(w_py - ro_y), (float)(w_pz - ro_z)));
              }
              r3d->drawRibbon(relative_traj, hist_w, 0.4f, 1.0f, 0.3f, 0.8f * macro_fade);
           }
        }
      }

      // ===== 火箭自身高亮标注 (方便在远景找到) =====
      if (cam_mode_3d == 2) {
        float marker_size = fminf(earth_r * 0.1f, earth_r * 0.02f * fmaxf(1.0f, cam_zoom_pan * 0.8f));
        r3d->drawBillboard(renderRocketBase, marker_size, 0.2f, 1.0f, 0.4f, 0.9f);
      }

      // ===== 轨道预测线 (开普勒轨道) =====
      if (cam_mode_3d == 2) {
        // We calculate and draw BOTH Earth-relative AND Sun-relative orbits concurrently!
        for (int ref_idx = 0; ref_idx < 2; ref_idx++) {
          bool is_sun_ref = (ref_idx == 1);

          // 选择参考系
          double mu_body = is_sun_ref ? (9.81 * earth_r * earth_r * ws_d * 333000.0) : (9.81 * earth_r * earth_r * ws_d); 

          // 全物理量双精度计算
          double abs_px = r_px, abs_py = r_py, abs_pz = r_pz;
          double abs_vx = rocket_state.vx * ws_d, abs_vy = rocket_state.vy * ws_d, abs_vz = rocket_state.vz * ws_d;

          if (is_sun_ref) {
              // Use exact same derived sun_angle and sun_angular_vel as physics engine (Burn)
              double G_const = 6.67430e-11;
              double M_sun = 1.989e30;
              double GM_sun = G_const * M_sun;
              double au_meters = 149597870700.0;
              double sun_angular_vel = sqrt(GM_sun / (au_meters * au_meters * au_meters));
              
              double exact_sun_angle = -1.2 + sun_angular_vel * rocket_state.sim_time;
              double sun_vx = -sin(exact_sun_angle) * au_meters * sun_angular_vel;
              double sun_vy = cos(exact_sun_angle) * au_meters * sun_angular_vel;
              
              // 地心相对日心为 -sunPos
              abs_px -= cos(exact_sun_angle) * au_meters * ws_d; 
              abs_py -= sin(exact_sun_angle) * au_meters * ws_d; 
              abs_pz -= sun_pz;

              // V_rocket/Sun = V_rocket/Earth - V_Sun/Earth
              abs_vx -= sun_vx * ws_d; 
              abs_vy -= sun_vy * ws_d;
          }

          double r_len = sqrt(abs_px*abs_px + abs_py*abs_py + abs_pz*abs_pz);
          double v_len = sqrt(abs_vx*abs_vx + abs_vy*abs_vy + abs_vz*abs_vz);

          if (v_len > 0.001f && r_len > earth_r * 0.5f) {
            double energy = 0.5 * v_len * v_len - mu_body / r_len;
            
            Vec3 h_vec( (float)(abs_py * abs_vz - abs_pz * abs_vy), 
                        (float)(abs_pz * abs_vx - abs_px * abs_vz),
                        (float)(abs_px * abs_vy - abs_py * abs_vx) );

            float h = h_vec.length();
            double a = -mu_body / (2.0 * energy);
            Vec3 v_vec((float)abs_vx, (float)abs_vy, (float)abs_vz);
            Vec3 p_vec((float)abs_px, (float)abs_py, (float)abs_pz);
            
            Vec3 e_vec = v_vec.cross(h_vec) / (float)mu_body - p_vec / (float)r_len;
            float ecc = e_vec.length();

            float opacity = (is_sun_ref == orbit_reference_sun) ? 0.9f : 0.3f;

            if (ecc < 1.0f) {
              // --- 椭圆轨道 (a > 0) ---
              float b = (float)a * sqrtf(fmaxf(0.0f, 1.0f - ecc * ecc));
              Vec3 e_dir = ecc > 1e-6f ? e_vec / ecc : Vec3(1.0f, 0.0f, 0.0f);
              Vec3 perp_dir = h_vec.normalized().cross(e_dir);

              float periapsis = (float)a * (1.0f - ecc);
              float apoapsis = (float)a * (1.0f + ecc);
              bool will_reenter = periapsis < earth_r && !is_sun_ref;

              Vec3 center_off = e_dir * (-(float)a * ecc);

              // 生成预测轨迹点集
              std::vector<Vec3> orbit_points;
              int orbit_segs = 120;
              for (int i = 0; i <= orbit_segs; i++) {
                float ang = (float)i / orbit_segs * 6.2831853f;
                Vec3 pt_rel = center_off + e_dir * ((float)a * cosf(ang)) + perp_dir * (b * sinf(ang));
                double p_x = pt_rel.x, p_y = pt_rel.y, p_z = pt_rel.z;
                if (is_sun_ref) { p_x += sun_px; p_y += sun_py; p_z += sun_pz; }
                Vec3 pt = Vec3((float)(p_x - ro_x), (float)(p_y - ro_y), (float)(p_z - ro_z));
                if (!is_sun_ref && pt.length() < earth_r * 0.98f) continue;
                orbit_points.push_back(pt);
              }
              
              // 渲染预测轨迹
              float pred_w = earth_r * 0.0025f * fmaxf(1.0f, cam_zoom_pan * 0.5f);
              float macro_fade = fminf(1.0f, fmaxf(0.0f, (cam_zoom_pan - 0.05f) / 0.1f));
              if (macro_fade > 0.01f) {
                  if (will_reenter) {
                    r3d->drawRibbon(orbit_points, pred_w, 1.0f, 0.4f, 0.1f, opacity * macro_fade);
                  } else {
                    if (is_sun_ref) {
                       r3d->drawRibbon(orbit_points, pred_w, 0.2f, 0.6f, 1.0f, opacity * macro_fade); // Dimmer/bluer for Sun
                    } else {
                       r3d->drawRibbon(orbit_points, pred_w, 0.2f, 0.8f, 1.0f, opacity * macro_fade); 
                    }
                  }
              }

              float apsis_size = fminf(earth_r * 0.12f, earth_r * 0.025f * fmaxf(1.0f, cam_zoom_pan * 0.8f));
              apsis_size *= (is_sun_ref ? 10.0f : 1.0f);
              
              // 远地点标记
              Vec3 apoPos = e_dir * (-apoapsis);
              double ax = apoPos.x, ay = apoPos.y, az = apoPos.z;
              if (is_sun_ref) { ax += sun_px; ay += sun_py; az += sun_pz; }
              Vec3 w_apoPos((float)(ax - ro_x), (float)(ay - ro_y), (float)(az - ro_z));
              if (apoapsis > earth_r * 1.002f) {
                r3d->drawBillboard(w_apoPos, apsis_size, 0.2f, 0.4f, 1.0f, opacity);
              }

              // 近地点标记
              Vec3 periPos = e_dir * periapsis;
              double px = periPos.x, py = periPos.y, pz = periPos.z;
              if (is_sun_ref) { px += sun_px; py += sun_py; pz += sun_pz; }
              Vec3 w_periPos((float)(px - ro_x), (float)(py - ro_y), (float)(pz - ro_z));
              if (periapsis > earth_r * 1.002f) {
                r3d->drawBillboard(w_periPos, apsis_size, 1.0f, 0.5f, 0.1f, opacity);
              }
            } else {
              // --- 双曲线/抛物线逃逸轨道 (ecc >= 1.0) ---
              float a_hyp = fabs(a); 
              float b_hyp = a_hyp * sqrtf(fmaxf(0.0f, ecc * ecc - 1.0f));
              Vec3 e_dir = e_vec / ecc;
              Vec3 perp_dir = h_vec.normalized().cross(e_dir);
              
              float periapsis = a_hyp * (ecc - 1.0f);
              Vec3 center_off = e_dir * (a_hyp * ecc);

              std::vector<Vec3> escape_points;
              int escape_segs = 60;
              float max_sinh = 3.0f; 
              for (int i = -escape_segs; i <= escape_segs; i++) {
                 float t = (float)i / escape_segs * max_sinh;
                 Vec3 pt_rel = center_off - e_dir * (a_hyp * coshf(t)) + perp_dir * (b_hyp * sinhf(t));
                 double px = pt_rel.x, py = pt_rel.y, pz = pt_rel.z;
                 if (is_sun_ref) { px += sun_px; py += sun_py; pz += sun_pz; }
                 
                 Vec3 pt((float)(px - ro_x), (float)(py - ro_y), (float)(pz - ro_z));
                 if (!is_sun_ref && pt.length() < earth_r * 0.98f) continue;
                 
                 if (escape_points.empty() || (pt - escape_points.back()).length() > earth_r * 0.05f) {
                    escape_points.push_back(pt);
                 }
              }

              // 逃逸轨道的 Ribbon (紫色代表逃逸)
              float pred_w = earth_r * 0.0025f * fmaxf(1.0f, cam_zoom_pan * 0.5f);
              float macro_fade = fminf(1.0f, fmaxf(0.0f, (cam_zoom_pan - 0.05f) / 0.1f));
              if (macro_fade > 0.01f) {
                  if (is_sun_ref) {
                       r3d->drawRibbon(escape_points, pred_w, 0.9f, 0.2f, 0.8f, opacity * macro_fade);
                  } else {
                       r3d->drawRibbon(escape_points, pred_w, 0.8f, 0.3f, 1.0f, opacity * macro_fade);
                  }
              }

              // 近地点标记
              float apsis_size = fminf(earth_r * 0.12f, earth_r * 0.025f * fmaxf(1.0f, cam_zoom_pan * 0.8f));
              apsis_size *= (is_sun_ref ? 10.0f : 1.0f);
              Vec3 periPos = center_off - e_dir * a_hyp; 
              double px = periPos.x, py = periPos.y, pz = periPos.z;
              if (is_sun_ref) { px += sun_px; py += sun_py; pz += sun_pz; }
              Vec3 w_periPos((float)(px - ro_x), (float)(py - ro_y), (float)(pz - ro_z));
              if (periapsis > earth_r * 1.002f) {
                r3d->drawBillboard(w_periPos, apsis_size, 1.0f, 0.5f, 0.1f, opacity);
              }
            }
          }
        }
      }

      // =========== PASS 2: MICRO FOREGROUND ===========
      // 微观近景火箭专用的相机矩阵 (极近裁剪面，用于精确绘制 40米的火箭)
      if (cam_mode_3d != 2) {
          // 在近景模式下，清空深度缓存，将火箭置于绝对顶层，杜绝共用一套深度衰减。
          // 全景模式下不清空，保留真实的物理穿模(躲在地球后面会被遮挡)的正确视角。
          glClear(GL_DEPTH_BUFFER_BIT);
      }
      
      float micro_near = fmaxf(rh * 0.05f, cam_dist * 0.002f);
      float micro_far  = fmaxf(cam_dist * 10.0f, 15000.0f);
      Mat4 microProjMat = Mat4::perspective(0.8f, aspect, micro_near, micro_far);
      r3d->beginFrame(viewMat, microProjMat, camEye_rel);



      // ===== 火箭涂装 (Builder颜色) =====
      // 1. 机体 (主燃料箱与分段)
      Mat4 bodyModel = Mat4::TRS(renderRocketPos, rocketQuat,
                                  Vec3(rw_3d, rh * 0.6f, rw_3d));
      r3d->drawMesh(rocketBody, bodyModel,
                     sel_tank.r, sel_tank.g, sel_tank.b, 1.0f, 0.25f);

      // 级间段/结构环 (Interstage Ring) - 金属灰暗色
      Vec3 interstagePos = renderRocketPos - rocketDir * (rh * 0.325f);
      Mat4 interstageModel = Mat4::TRS(interstagePos, rocketQuat,
                                  Vec3(rw_3d * 1.02f, rh * 0.05f, rw_3d * 1.02f));
      r3d->drawMesh(rocketBody, interstageModel, 0.2f, 0.2f, 0.25f, 1.0f, 0.4f);

      // 2. 鼻锥头部
      Vec3 nosePos = renderRocketPos + rocketDir * (rh * 0.30f);
      Mat4 noseModel = Mat4::TRS(nosePos, rocketQuat,
                                  Vec3(rw_3d, rh * 0.25f, rw_3d));
      r3d->drawMesh(rocketNose, noseModel,
                     sel_nose.r, sel_nose.g, sel_nose.b, 1.0f, 0.25f);

      // 前端热盾/整流罩尖顶 (钛合金黑色)
      Vec3 tipPos = renderRocketPos + rocketDir * (rh * 0.50f);
      Mat4 tipModel = Mat4::TRS(tipPos, rocketQuat,
                                  Vec3(rw_3d * 0.2f, rh * 0.05f, rw_3d * 0.2f));
      r3d->drawMesh(rocketNose, tipModel, 0.15f, 0.15f, 0.15f, 1.0f, 0.5f);

      // 3. 气动翼片/栅格舵 (Fins & Grid Fins)
      for (int i = 0; i < 4; i++) {
        float angle = i * 1.570796f; // 90 degrees
        Quat finRot = rocketQuat * Quat::fromAxisAngle(Vec3(0.0f, 1.0f, 0.0f), angle);
        
        // 尾部主翼
        Vec3 mainFinPos = renderRocketPos - rocketDir * (rh * 0.25f) + finRot.rotate(Vec3(rw_3d * 1.2f, 0.0f, 0.0f));
        Mat4 mainFinModel = Mat4::TRS(mainFinPos, finRot, Vec3(rw_3d * 1.5f, rh * 0.1f, rw_3d * 0.05f));
        r3d->drawMesh(rocketBox, mainFinModel, sel_tank.r * 0.8f, sel_tank.g * 0.8f, sel_tank.b * 0.8f, 1.0f, 0.2f);
        
        // 头部栅格舵
        Vec3 gridFinPos = renderRocketPos + rocketDir * (rh * 0.2f) + finRot.rotate(Vec3(rw_3d * 1.1f, 0.0f, 0.0f));
        Mat4 gridFinModel = Mat4::TRS(gridFinPos, finRot, Vec3(rw_3d * 0.4f, rh * 0.02f, rw_3d * 0.3f));
        r3d->drawMesh(rocketBox, gridFinModel, 0.2f, 0.2f, 0.2f, 1.0f, 0.3f);
      }

      // 4. 引擎喷管 (Detailed Engine)
      Vec3 engBasePos = renderRocketPos - rocketDir * (rh * 0.375f);
      // 发动机机械基座
      Mat4 engBaseModel = Mat4::TRS(engBasePos, rocketQuat, Vec3(rw_3d * 0.6f, rh * 0.05f, rw_3d * 0.6f));
      r3d->drawMesh(rocketBody, engBaseModel, 0.15f, 0.15f, 0.15f, 1.0f, 0.5f);

      // 喷管钟罩
      // 我们用无翻转的锥体，宽底面朝下，尖端朝上刺入发动机基座，来模拟真实的钟形喷口 (Bell Nozzle)
      Vec3 engNozzlePos = renderRocketPos - rocketDir * (rh * 0.50f);
      Mat4 engNozzleModel = Mat4::TRS(engNozzlePos, rocketQuat,
                                  Vec3(rw_3d * 0.9f, rh * 0.10f, rw_3d * 0.9f));
      r3d->drawMesh(rocketNose, engNozzleModel,
                     sel_eng.r, sel_eng.g, sel_eng.b, 1.0f, 0.4f);

      // ===== 3D 火焰/尾焰粒子 =====
      if (rocket_state.thrust_power > 0.0) {
        float thrust = (float)control_input.throttle;
        float flame_base_size = rw_3d * 1.5f;
        float game_time = (float)glfwGetTime();

        // 真空中火焰更大（大气压缩效果）
        float vacuum_scale = 1.0f + (float)fmin(rocket_state.altitude / 50000.0, 2.0);

        for (int i = 0; i < 10; i++) {
          // 粒子沿推力反方向分布
          float t_off = (float)i / 10.0f;
          float rand_off = sinf(game_time * 17.3f + i * 7.1f) * 0.3f + 0.5f;
          float dist = (t_off * 0.5f + rand_off * 0.2f) * rh * thrust * vacuum_scale;
          Vec3 particlePos = engNozzlePos - rocketDir * dist;

          // 随机侧向偏移
          float side_x = sinf(game_time * 23.7f + i * 3.3f) * rw_3d * 0.3f;
          float side_z = cosf(game_time * 19.1f + i * 5.7f) * rw_3d * 0.3f;
          particlePos = particlePos + localRight * side_x + Vec3(0.0f, 0.0f, 1.0f) * side_z;

          // 颜色: 近喷口=白, 中=黄, 远=橙红
          float cr, cg, cb;
          if (t_off < 0.3f) {
            cr = 1.0f; cg = 0.95f; cb = 0.8f;
          } else if (t_off < 0.6f) {
            cr = 1.0f; cg = 0.7f; cb = 0.2f;
          } else {
            cr = 1.0f; cg = 0.4f; cb = 0.05f;
          }

          float size = flame_base_size * (0.5f + t_off * 1.5f) * vacuum_scale;
          float alpha = thrust * (1.0f - t_off * 0.7f) * 0.6f;
          r3d->drawBillboard(particlePos, size, cr, cg, cb, alpha);
        }
      }

      r3d->endFrame();
    }

    // ================= 2D HUD 叠加层 =================
    glDisable(GL_DEPTH_TEST);
    renderer->beginFrame();

    // 坐标转换变量（HUD也需要）
    double scale = 1.0 / (rocket_state.altitude * 1.5 + 200.0);
    float cx = 0.0f;
    float cy = 0.0f;
    double rocket_r = sqrt(rocket_state.px * rocket_state.px + rocket_state.py * rocket_state.py);
    double rocket_theta = atan2(rocket_state.py, rocket_state.px);
    double cam_angle = PI / 2.0 - rocket_theta;
    double sin_c = sin(cam_angle);
    double cos_c = cos(cam_angle);
    auto toScreenX = [&](double wx, double wy) {
      double rx = wx * cos_c - wy * sin_c;
      return (float)(rx * scale + cx);
    };
    auto toScreenY = [&](double wx, double wy) {
      double ry = wx * sin_c + wy * cos_c;
      return (float)((ry - rocket_r) * scale + cy);
    };
    float w = max(0.015f, (float)(10.0 * scale));
    float h = max(0.06f, (float)(40.0 * scale));
    float y_offset = -h / 2.0f;


    // ====================================================================
    // ===== 2D 叠加层 HUD =====
    
    if (show_hud) {  // 用户按 H 键切换开关
        
        // --- 强制重置 2D 渲染器批处理状态 ---
        // 结束之前的可能遗留的 2D 绘制 (如烟雾特效)
        renderer->endFrame();
        // 彻底重置 OpenGL 混合和深度测试状态，防止 3D 尾焰泄露
        glUseProgram(0);
        glDisable(GL_DEPTH_TEST);
        glDisable(GL_CULL_FACE); // 确保2D矩形不会因为绘制方向被意外剔除
        glDepthMask(GL_TRUE);
        glEnable(GL_BLEND);
        glBlendEquation(GL_FUNC_ADD); // 修复：引擎(Shock Diamonds)用了 GL_MAX 导致 HUD 的 alpha 混合失效
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glBindVertexArray(0);
        glBindBuffer(GL_ARRAY_BUFFER, 0);
        // 重新开启一个新的 2D 批处理专供 HUD 使用 (确保着色器正确绑定)
        renderer->beginFrame();

        float hud_opacity = 0.8f;

        float gauge_w = 0.03f;
        float gauge_h = 0.45f;
        float gauge_y_center = 0.4f;
        float gauge_vel_x = -0.92f;
        float gauge_alt_x = -0.84f;
        float gauge_fuel_x = -0.76f;

    double current_vel = sqrt(rocket_state.vx*rocket_state.vx + rocket_state.vy*rocket_state.vy + rocket_state.vz*rocket_state.vz);
    double current_alt = rocket_state.altitude;
    int current_vvel = (int)rocket_state.velocity;

    if (orbit_reference_sun) {
        double G_const = 6.67430e-11;
        double M_sun = 1.989e30;
        double GM_sun = G_const * M_sun;
        double au = 149597870700.0;
        double sun_angular_vel = sqrt(GM_sun / (au * au * au));
        double sun_angle = -1.2 + sun_angular_vel * rocket_state.sim_time;
        double current_sun_px = cos(sun_angle) * au;
        double current_sun_py = sin(sun_angle) * au;
        double current_sun_vx = -sin(sun_angle) * au * sun_angular_vel;
        double current_sun_vy = cos(sun_angle) * au * sun_angular_vel;

        double rel_vx = rocket_state.vx - current_sun_vx;
        double rel_vy = rocket_state.vy - current_sun_vy;
        double rel_vz = rocket_state.vz;
        double rel_px = rocket_state.px - current_sun_px;
        double rel_py = rocket_state.py - current_sun_py;
        double rel_pz = rocket_state.pz;
        
        current_vel = sqrt(rel_vx * rel_vx + rel_vy * rel_vy + rel_vz * rel_vz);
        double dist_to_sun = sqrt(rel_px * rel_px + rel_py * rel_py + rel_pz * rel_pz);
        current_alt = dist_to_sun - 696340000.0; 
        
        double rel_vvel_real = (rel_vx * rel_px + rel_vy * rel_py + rel_vz * rel_pz) / dist_to_sun;
        current_vvel = (int)rel_vvel_real;
    }

    // --- 1. 速度计 (Velocity Gauge) ---
    float max_gauge_vel = 3000.0f;
    float vel_ratio = (float)min(1.0, current_vel / max_gauge_vel);
    renderer->addRect(gauge_vel_x, gauge_y_center, gauge_w * 1.2f,
                      gauge_h + 0.02f, 0.1f, 0.1f, 0.1f, 0.5f * hud_opacity);
    float r_vel = vel_ratio * 2.0f;
    if (r_vel > 1.0f)
      r_vel = 1.0f;
    float g_vel = (1.0f - vel_ratio) * 2.0f;
    if (g_vel > 1.0f)
      g_vel = 1.0f;
    float fill_h_vel = gauge_h * vel_ratio;
    renderer->addRect(
        gauge_vel_x, gauge_y_center - (gauge_h - fill_h_vel) / 2.0f,
        gauge_w * 0.8f, fill_h_vel, r_vel, g_vel, 0.0f, hud_opacity);

    // --- 2. 高度计 (Altitude Gauge) ---
    float max_gauge_alt = 100000.0f;
    float alt_ratio = (float)min(1.0, current_alt / max_gauge_alt);
    renderer->addRect(gauge_alt_x, gauge_y_center, gauge_w * 1.2f,
                      gauge_h + 0.02f, 0.1f, 0.1f, 0.1f, 0.5f * hud_opacity);
    float fill_h_alt = gauge_h * alt_ratio;
    renderer->addRect(gauge_alt_x,
                      gauge_y_center - (gauge_h - fill_h_alt) / 2.0f,
                      gauge_w * 0.8f, fill_h_alt, alt_ratio * 0.8f,
                      0.3f + alt_ratio * 0.7f, 1.0f, hud_opacity);

    // --- 3. 燃油计 (Fuel Gauge) ---
    double current_fuel = rocket_state.fuel;
    float max_gauge_fuel = 100000.0f; // 对应火箭初始化时的 100 吨满载燃料
    float fuel_ratio = (float)max(0.0, min(1.0, current_fuel / max_gauge_fuel));

    // 背景槽
    renderer->addRect(gauge_fuel_x, gauge_y_center, gauge_w * 1.2f,
                      gauge_h + 0.02f, 0.1f, 0.1f, 0.1f, 0.5f * hud_opacity);

    // 动态颜色：满燃料为绿，耗尽前变红
    float r_fuel = (1.0f - fuel_ratio) * 2.0f;
    if (r_fuel > 1.0f)
      r_fuel = 1.0f;
    float g_fuel = fuel_ratio * 2.0f;
    if (g_fuel > 1.0f)
      g_fuel = 1.0f;

    float fill_h_fuel = gauge_h * fuel_ratio;
    renderer->addRect(
        gauge_fuel_x, gauge_y_center - (gauge_h - fill_h_fuel) / 2.0f,
        gauge_w * 0.8f, fill_h_fuel, r_fuel, g_fuel, 0.1f, hud_opacity);

    // --- 4. 装饰性刻度线与标签框 ---
    int num_ticks = 10;
    for (int i = 0; i <= num_ticks; i++) {
      float tick_ratio = (float)i / num_ticks;
      float tick_y = (gauge_y_center - gauge_h / 2.0f) + gauge_h * tick_ratio;
      float tick_w = 0.02f;
      float tick_h = 0.003f;
      float alpha = (i % 5 == 0) ? 0.8f : 0.4f;
      if (i % 5 == 0)
        tick_w *= 1.5f;

      renderer->addRect(gauge_vel_x - gauge_w, tick_y, tick_w, tick_h, 1.0f,
                        1.0f, 1.0f, alpha * hud_opacity);
      renderer->addRect(gauge_alt_x + gauge_w, tick_y, tick_w, tick_h, 1.0f,
                        1.0f, 1.0f, alpha * hud_opacity);
      renderer->addRect(gauge_fuel_x + gauge_w, tick_y, tick_w, tick_h, 1.0f,
                        1.0f, 1.0f, alpha * hud_opacity);
    }

    // 底部颜色标签框
    renderer->addRect(gauge_vel_x, gauge_y_center - gauge_h / 2.0f - 0.03f,
                      gauge_w * 2.0f, 0.03f, 0.8f, 0.2f, 0.2f,
                      hud_opacity); // 红: VEL
    renderer->addRect(gauge_alt_x, gauge_y_center - gauge_h / 2.0f - 0.03f,
                      gauge_w * 2.0f, 0.03f, 0.2f, 0.5f, 0.9f,
                      hud_opacity); // 蓝: ALT
    renderer->addRect(gauge_fuel_x, gauge_y_center - gauge_h / 2.0f - 0.03f,
                      gauge_w * 2.0f, 0.03f, 0.9f, 0.6f, 0.1f,
                      hud_opacity); // 橙: FUEL
    // --- 5. 数字读数 (七段数码管) + 黑色背景 + 像素字体单位 ---
    float num_x = -0.65f;
    float num_size = 0.035f;
    float bg_w = 0.20f;  // 数字背景宽 (加宽放单位)
    float bg_h = 0.05f;  // 数字背景高
    float label_x = num_x + 0.08f; // 单位文字起始X

    // 速度读数 + m/s
    renderer->addRect(num_x, 0.7f, bg_w, bg_h, 0.0f, 0.0f, 0.0f, 0.5f);
    renderer->drawNumber(num_x - 0.02f, 0.7f, (int)current_vel, num_size, 1.0f, 0.4f,
                         0.4f, hud_opacity);
    renderer->drawLabel(label_x, 0.7f, "m/s", num_size * 0.7f,
                        1.0f, 0.6f, 0.6f, hud_opacity);

    // 海拔读数 + m 或 km
    renderer->addRect(num_x, 0.55f, bg_w, bg_h, 0.0f, 0.0f, 0.0f, 0.5f);
    if (current_alt > 10000) {
      renderer->drawNumber(num_x - 0.02f, 0.55f, (int)(current_alt / 1000.0), num_size,
                           0.4f, 0.7f, 1.0f, hud_opacity);
      renderer->drawLabel(label_x, 0.55f, "km", num_size * 0.7f,
                          0.5f, 0.8f, 1.0f, hud_opacity);
    } else {
      renderer->drawNumber(num_x - 0.02f, 0.55f, (int)current_alt, num_size, 0.4f, 0.7f,
                           1.0f, hud_opacity);
      renderer->drawLabel(label_x, 0.55f, "m", num_size * 0.7f,
                          0.5f, 0.8f, 1.0f, hud_opacity);
    }

    // 燃油读数 + kg
    renderer->addRect(num_x, 0.4f, bg_w, bg_h, 0.0f, 0.0f, 0.0f, 0.5f);
    renderer->drawNumber(num_x - 0.02f, 0.4f, (int)current_fuel, num_size, 0.9f, 0.7f,
                         0.2f, hud_opacity);
    renderer->drawLabel(label_x, 0.4f, "kg", num_size * 0.7f,
                        1.0f, 0.8f, 0.3f, hud_opacity);

    // 油门读数 + %
    renderer->addRect(num_x, 0.25f, bg_w * 0.8f, bg_h * 0.8f, 0.0f, 0.0f, 0.0f, 0.5f);
    renderer->drawNumber(num_x - 0.02f, 0.25f, (int)(control_input.throttle * 100),
                         num_size * 0.8f, 0.8f, 0.8f, 0.8f, hud_opacity);
    renderer->drawLabel(label_x - 0.01f, 0.25f, "%", num_size * 0.6f,
                        0.8f, 0.8f, 0.8f, hud_opacity);

    // 俯仰读数 (Pitch) + 度数
    renderer->addRect(num_x, 0.10f, bg_w * 0.8f, bg_h * 0.8f, 0.0f, 0.0f, 0.0f, 0.5f);
    int pitch_deg = (int)(abs(rocket_state.angle_z) * 180.0 / PI);
    float pr = 0.8f, pg = 0.8f, pb = 0.8f;
    if (pitch_deg > 20 && current_vel > 200.0) {
        pr = 1.0f; pg = 0.2f; pb = 0.2f; // Red Warning (high Q area)
    } else if (pitch_deg > 20) {
        pr = 1.0f; pg = 0.7f; pb = 0.2f; // Yellow Warning (Safe speed)
    }
    renderer->drawNumber(num_x - 0.02f, 0.10f, pitch_deg, num_size * 0.8f,
                         pr, pg, pb, hud_opacity);
    renderer->drawLabel(label_x - 0.01f, 0.10f, "deg", num_size * 0.5f,
                        pr, pg, pb, hud_opacity);

    // 垂直速度读数 + m/s (右侧 HUD)
    renderer->addRect(0.85f, 0.4f, 0.22f, 0.06f, 0.05f, 0.05f, 0.05f, 0.5f);
    float vr = current_vvel < 0 ? 1.0f : 0.3f;
    float vg = current_vvel >= 0 ? 1.0f : 0.3f;
    renderer->drawNumber(0.83f, 0.4f, current_vvel, num_size * 0.9f, vr, vg, 0.3f,
                         hud_opacity);
    renderer->drawLabel(0.93f, 0.4f, "m/s", num_size * 0.5f,
                        0.7f, 0.7f, 0.7f, hud_opacity);

    // --- 6. 控制模式指示器 (HUD 右上角) ---
    float mode_x = 0.85f;
    float mode_y = 0.85f;
    // 背景框
    renderer->addRect(mode_x, mode_y, 0.22f, 0.06f, 0.05f, 0.05f, 0.05f, 0.7f);
    if (rocket_state.auto_mode) {
      // 绿色 AUTO 指示
      renderer->addRect(mode_x, mode_y, 0.20f, 0.04f, 0.1f, 0.8f, 0.2f, 0.9f);
    } else {
      // 橙色 MANUAL 指示
      renderer->addRect(mode_x, mode_y, 0.20f, 0.04f, 1.0f, 0.6f, 0.1f, 0.9f);
    }

    // --- 6. 油门指示条 (HUD 底部中央) ---
    float thr_bar_x = 0.0f;
    float thr_bar_y = -0.92f;
    float thr_bar_w = 0.5f;
    float thr_bar_h = 0.025f;
    // 背景
    renderer->addRect(thr_bar_x, thr_bar_y, thr_bar_w + 0.02f,
                      thr_bar_h + 0.01f, 0.1f, 0.1f, 0.1f, 0.5f * hud_opacity);
    // 填充
    float thr_fill = thr_bar_w * (float)control_input.throttle;
    float thr_fill_x = thr_bar_x - thr_bar_w / 2.0f + thr_fill / 2.0f;
    float thr_r = (float)control_input.throttle > 0.8f
                      ? 1.0f
                      : 0.3f + (float)control_input.throttle * 0.7f;
    float thr_g = (float)control_input.throttle < 0.5f
                      ? 0.8f
                      : 0.8f - ((float)control_input.throttle - 0.5f) * 1.6f;
    renderer->addRect(thr_fill_x, thr_bar_y, thr_fill, thr_bar_h, thr_r, thr_g,
                      0.1f, hud_opacity);

    // ========================================================================
    } // end if (show_hud)
    
    renderer->endFrame();
    glfwSwapBuffers(window);
  }

  delete renderer;
  earthMesh.destroy();
  rocketBody.destroy();
  rocketNose.destroy();
  delete r3d;
  glfwTerminate();
  if (rocket_state.status == LANDED || rocket_state.status == CRASHED) {
    cout << "\n>> SIMULATION ENDED. PRESS ENTER." << endl;
    cin.get();
  }
  return 0;
}