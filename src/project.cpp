#include<glad/glad.h>
#include <GLFW/glfw3.h>
#include <algorithm>
#include <chrono>
#include <cmath>

#include "math3d.h"
#include "renderer3d.h"

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
const double PI = 3.14159265358979323846;
const double G0 = 9.80665;             // 海平面标准重力
const double EARTH_RADIUS = 6371000.0; // 地球半径 (米)

// 引擎底层：无状态伪随机哈希函数
// 输入一个整数网格索引，输出一个极其均匀的 0.0 到 1.0 的浮点数
float hash11(int n) {
  n = (n << 13) ^ n;
  int nn = (n * (n * n * 60493 + 19990303) + 1376312589) & 0x7fffffff;
  return ((float)nn / (float)0x7fffffff);
}
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

// PID 控制器结构体
// 修改 Part 0 中的 PID 结构体
struct PID {
  double kp, ki, kd;
  double integral = 0;
  double prev_error = 0;

  double integral_limit = 50.0;

  double update(double target, double current, double dt) {
    double error = target - current;
    integral += error * dt;

    if (integral > integral_limit)
      integral = integral_limit;
    if (integral < -integral_limit)
      integral = -integral_limit;

    double derivative = (error - prev_error) / dt;
    prev_error = error;
    return kp * error + ki * integral + kd * derivative;
  }

  void reset() {
    integral = 0;
    prev_error = 0;
  }
};

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
// Part 2: Explorer 类 (集成 2D 物理与控制)
// ==========================================
const double SLP = 1013.25;

// 世界坐标烟雾粒子
struct SmokeParticle {
  double wx, wy;     // 世界坐标
  double vwx, vwy;   // 世界速度
  float alpha;       // 当前透明度
  float size;        // 当前大小
  float life;        // 剩余寿命 (0~1)
  bool active;
};

class Explorer {
private:
  double fuel;
  double mass; // 干重
  double diameter;
  double height;
public:
  double getHeight() const { return height; }
  double getDiameter() const { return diameter; }
private:
  bool suicide_burn_locked = false;
  double altitude; // 海拔高度
  double velocity; // 垂直速度 (径向速度)
  double specific_impulse;
  double fuel_consumption_rate; // 当前油门下的消耗率
  double thrust_power;
  double acceleration;
  double cosrate; // 满油门消耗率
  double nozzle_area;

  // 坐标系：地心为原点 (0,0)。发射点(北极)为 (0, EARTH_RADIUS)
  public:
  double local_vx;   // 真实的相对地表水平速度 (切向速度)
  double px, py;     // 2D 位置
  double vx, vy;     // 2D 速度
  double angle;      // 姿态角 (0=垂直向上/径向, 正=向左倾斜, 负=向右倾斜)
  double ang_vel;    // 角速度
  double throttle;   // 油门 (0.0 - 1.0)
  double torque_cmd; // 力矩指令
  double sim_time;   // 物理时间积累

  // PID 控制器实例
  PID pid_vert = {0.5, 0.001, 1.2};       // 高度环
  PID pid_pos = {0.001, 0.0, 0.2};        // 位置环 (参数很小因为距离很大)
  PID pid_att = {40000.0, 0.0, 100000.0}; // 姿态环 (需要快速响应)

public:
  enum State { PRE_LAUNCH, ASCEND, DESCEND, LANDED, CRASHED } status;
  string mission_msg = "SYSTEM READY";
  int stages;
  int mission_phase = 0;            // 记录当前太空任务阶段
  double mission_timer = 0;         // 任务计时器
  bool auto_mode = true;            // true=自动驾驶 false=手动控制
  double leg_deploy_progress = 0.0; // 着陆腿展开进度 0~1
  static const int MAX_SMOKE = 300;
  SmokeParticle smoke[MAX_SMOKE];
  int smoke_idx = 0;

  void emitSmoke(double dt) {
    if (thrust_power < 1000.0) return;
    double local_up = atan2(py, px);
    double nozzle_dir = local_up + angle + PI;
    double nozzle_wx = px + cos(nozzle_dir) * 20.0;
    double nozzle_wy = py + sin(nozzle_dir) * 20.0;
    // 每帧发射 3 个粒子，形成浓密烟雾
    for (int k = 0; k < 3; k++) {
      SmokeParticle& p = smoke[smoke_idx % MAX_SMOKE];
      float rnd1 = hash11(smoke_idx * 1337 + k * 997) - 0.5f;
      float rnd2 = hash11(smoke_idx * 7919 + k * 773) - 0.5f;
      p.wx = nozzle_wx + rnd1 * 15.0;
      p.wy = nozzle_wy + rnd2 * 15.0;
      // 给粒子初速度：沿喷口方向向下喷射
      double exhaust_speed = 30.0 + hash11(smoke_idx * 3571 + k) * 20.0;
      p.vwx = cos(nozzle_dir) * exhaust_speed + rnd1 * 10.0;
      p.vwy = sin(nozzle_dir) * exhaust_speed + rnd2 * 10.0;
      p.alpha = 0.6f;
      p.size = 10.0f + hash11(smoke_idx * 4567 + k) * 8.0f;
      p.life = 1.0f;
      p.active = true;
      smoke_idx++;
    }
  }

  void updateSmoke(double dt) {
    for (int i = 0; i < MAX_SMOKE; i++) {
      if (!smoke[i].active) continue;
      smoke[i].life -= (float)(dt * 0.25);
      smoke[i].alpha = min(0.25f, smoke[i].life * 0.3f); // 75% 透明
      smoke[i].size += (float)(dt * 20.0);

      smoke[i].wx += smoke[i].vwx * dt;
      smoke[i].wy += smoke[i].vwy * dt;

      // 地面碰撞反弹
      double r = sqrt(smoke[i].wx * smoke[i].wx + smoke[i].wy * smoke[i].wy);
      if (r < EARTH_RADIUS && r > 0) {
        smoke[i].wx = smoke[i].wx / r * EARTH_RADIUS;
        smoke[i].wy = smoke[i].wy / r * EARTH_RADIUS;
        double nx = smoke[i].wx / r, ny = smoke[i].wy / r;
        double v_radial = smoke[i].vwx * nx + smoke[i].vwy * ny;
        double v_tang = -smoke[i].vwx * ny + smoke[i].vwy * nx;
        v_radial = abs(v_radial) * 0.3;
        // 随机方向反弹 — 每个粒子弹向不同方向
        float rnd_dir = (hash11(i * 8731) - 0.5f) * 2.0f;
        v_tang = abs(v_tang) * (1.5f + rnd_dir) * (rnd_dir > 0 ? 1.0 : -1.0);
        smoke[i].vwx = nx * v_radial - ny * v_tang;
        smoke[i].vwy = ny * v_radial + nx * v_tang;
        smoke[i].size += 5.0f;
      }

      // 热气上升 + 速度衰减
      if (r > 0) {
        smoke[i].vwx += (smoke[i].wx / r) * dt * 8.0;
        smoke[i].vwy += (smoke[i].wy / r) * dt * 8.0;
      }
      smoke[i].vwx *= (1.0 - dt * 0.8); // 空气阻力
      smoke[i].vwy *= (1.0 - dt * 0.8);

      if (smoke[i].life <= 0) smoke[i].active = false;
    }
  }

  Explorer(double fuel, double mass, double diameter, double height, int stages,
           double specific_impulse, double fuel_consumption_rate,
           double nozzle_area) {
    // 2D 初始化：放在地球北极
    px = 0;
    py = EARTH_RADIUS;
    vx = 0;
    vy = 0;
    angle = 0;
    ang_vel = 0;
    sim_time = 0;

    // 兼容旧变量初始化
    altitude = 0;
    velocity = 0;

    this->fuel = fuel;
    this->mass = mass;
    this->diameter = diameter;
    this->height = height;
    this->stages = stages;
    this->specific_impulse = specific_impulse;

    this->cosrate = fuel_consumption_rate;
    this->nozzle_area = nozzle_area;

    this->fuel_consumption_rate = 0;
    this->thrust_power = 0;
    this->throttle = 0;
    this->torque_cmd = 0;

    status = PRE_LAUNCH;
  }

  double getAltitude() const { return altitude; }
  double getThrust() const { return thrust_power; }
  double getVelocityMag() const { return sqrt(vx * vx + vy * vy); }
  double getVerticalVel() const { return velocity; }
  double getFuel() const { return fuel; }
  // 实时计算远地点(Apoapsis)和近地点(Periapsis)
  void getOrbitParams(double &apoapsis, double &periapsis) {
    double r = sqrt(px * px + py * py);
    double v_sq = vx * vx + vy * vy;
    double mu = G0 * EARTH_RADIUS * EARTH_RADIUS; // 标准引力参数

    double energy = v_sq / 2.0 - mu / r; // 轨道比能
    double h = px * vy - py * vx;        // 比角动量

    double e_sq = 1.0 + 2.0 * energy * h * h / (mu * mu); // 离心率平方
    double e = (e_sq > 0) ? sqrt(e_sq) : 0;

    if (energy >= 0) { // 逃逸轨道
      apoapsis = 999999999;
      periapsis = (h * h / mu) / (1.0 + e) - EARTH_RADIUS;
    } else {                           // 闭合椭圆轨道
      double a = -mu / (2.0 * energy); // 半长轴
      apoapsis = a * (1.0 + e) - EARTH_RADIUS;
      periapsis = a * (1.0 - e) - EARTH_RADIUS;
    }
  }
  // --- 物理计算 ---
  // 1. 重力随高度变化 (平方反比定律)
  double get_gravity(double r) { return G0 * pow(EARTH_RADIUS / r, 2); }

  // 2. 气压/密度模型
  double get_pressure(double h) {
    if (h > 100000)
      return 0;
    if (h < 0)
      return SLP;
    return SLP * exp(-h / 7000.0); // 简化指数大气
  }
  double get_air_density(double h) {
    if (h > 100000)
      return 0;
    return 1.225 * exp(-h / 7000.0);
  }

  void Report_Status() {
    double apo, peri;
    getOrbitParams(apo, peri);
    cout << "\n----------------------------------" << endl;
    cout << ">>> [MISSION CONTROL]: " << mission_msg << " <<<" << endl;
    cout << "----------------------------------" << endl;
    cout << "[Alt]: " << altitude << " m | [Vert_Vel]: " << velocity << " m/s";
    
    // 如果处于极高的时间加速状态，额外打印当前的加速倍率
    if (!auto_mode && altitude > 100000.0 && throttle < 0.01) {
        cout << " | [WARP READY]" << endl;
    } else {
        cout << endl;
    }
    cout << "[Pos_X]: " << px << " m | [Horz_Vel]: " << vx << " m/s" << endl;
    cout << "[Angle]: " << angle * 180.0 / PI
         << " deg | [Throttle]: " << throttle * 100 << "%" << endl;
    cout << "[Ground_Horz_Vel]: " << local_vx
         << " m/s | [Orbit_Vel]: " << getVelocityMag() << " m/s" << endl;
    cout << "[Thrust]: " << thrust_power / 1000 << " kN | [Fuel]: " << fuel
         << " kg" << endl;
    cout << "[Apoapsis]: " << apo / 1000.0
         << " km | [Periapsis]: " << peri / 1000.0 << " km" << endl;
    cout << "[Status]: " << status << endl;
  }

  // 核心物理引擎 (2D 矢量版)
  void Burn(double dt) {
    if (status == PRE_LAUNCH) {
      px = 0;
      py = EARTH_RADIUS;
      vx = 0;
      vy = 0;
      altitude = 0;
      return;
    }
    if (status == LANDED || status == CRASHED)
      return;

    // A. 基础状态计算
    double r = sqrt(px * px + py * py); // 距地心距离
    altitude = r - EARTH_RADIUS;

    // B. 受力分析
    double total_mass = mass + fuel;

    // 1. 地球重力 (万有引力，指向地心)
    double g_earth = get_gravity(r);
    double Fg_x = -g_earth * (px / r) * total_mass;
    double Fg_y = -g_earth * (py / r) * total_mass;

    // 1.5 太阳重力 (双星引力系统) - 潮汐力模型 (Third Body Perturbation)
    sim_time += dt;
    double au_meters = 149597870700.0;
    
    // 太阳真实引力常数 (G * M_sun)
    // 太阳质量 1.989e30 kg, 地球质量 5.972e24 kg
    // 万有引力常数 G = 6.67430e-11
    double G_const = 6.67430e-11;
    double M_sun = 1.989e30;
    double GM_sun = G_const * M_sun;
    
    // 强制公转角速度完美匹配太阳重力，避免预测轨道出现偏心率震荡
    double sun_angular_vel = sqrt(GM_sun / (au_meters * au_meters * au_meters));
    
    // 这里 sun_angle 是地球看太阳的角度
    double sun_angle = -1.2 + sun_angular_vel * sim_time; 
    
    // 太阳在以地球为中心的坐标系中的位置 (地球看太阳)
    double sun_px = cos(sun_angle) * au_meters;
    double sun_py = sin(sun_angle) * au_meters;
    
    // 火箭指向太阳的向量
    double dx_sun = sun_px - px;
    double dy_sun = sun_py - py;
    
    double r_sun_rocket_sq = dx_sun*dx_sun + dy_sun*dy_sun;
    double dist_sun_rocket = sqrt(r_sun_rocket_sq);
    double r_sun_rocket3 = r_sun_rocket_sq * dist_sun_rocket;
    
    double dist_sun_earth = au_meters;
    double r_sun_earth3 = dist_sun_earth * dist_sun_earth * dist_sun_earth;
    
    // 第三体扰动：太阳对火箭的引力 减去 太阳对地球的引力 
    // (因为我们的坐标系原点是随地球加速运动的)
    double Fg_sun_x = GM_sun * (dx_sun / r_sun_rocket3 - sun_px / r_sun_earth3) * total_mass;
    double Fg_sun_y = GM_sun * (dy_sun / r_sun_rocket3 - sun_py / r_sun_earth3) * total_mass;
    
    Fg_x += Fg_sun_x;
    Fg_y += Fg_sun_y;

    // 2. 推力 (基于角度和油门)
    thrust_power = 0;
    if (fuel > 0) {
      // 计算推力 (真空推力 - 背压损失)
      double max_thrust = specific_impulse * G0 * cosrate;
      double pressure_loss = 100 * get_pressure(altitude) * nozzle_area;
      double current_thrust = throttle * max_thrust - pressure_loss;
      if (current_thrust < 0)
        current_thrust = 0;
      thrust_power = current_thrust;

      // 消耗燃料
      double m_dot = thrust_power / (specific_impulse * G0);
      fuel -= m_dot * dt;
      fuel_consumption_rate = m_dot; // 记录当前消耗率供显示
    } else {
      thrust_power = 0;
      fuel_consumption_rate = 0;
    }

    // 推力方向：假设 angle=0 是沿径向向上。
    // 计算局部垂直方向 (Local Vertical) 的角度
    double local_up_angle = atan2(py, px);
    // 实际推力方向 = 局部垂直 + 火箭偏角
    double thrust_dir = local_up_angle + angle;

    double Ft_x = thrust_power * cos(thrust_dir);
    double Ft_y = thrust_power * sin(thrust_dir);

    // 3. 空气阻力 & 气动扭矩
    double v_sq = vx * vx + vy * vy;
    double v_mag = sqrt(v_sq);
    double Fd_x = 0, Fd_y = 0;
    double aero_torque = 0;

    if (v_mag > 0.1 && altitude < 80000) {
      double rho = get_air_density(altitude);

      // 阻力 (F = 0.5 * rho * v^2 * Cd * A)
      // 假设 Cd = 0.5, A = 10 (简化)
      double drag_mag = 0.5 * rho * v_sq * 0.5 * 10.0;
      Fd_x = -drag_mag * (vx / v_mag);
      Fd_y = -drag_mag * (vy / v_mag);

      // 气动扭矩 (风向标效应)
      // 攻角 Alpha = 火箭指向 - 速度方向
      double vel_angle = atan2(vy, vx);
      // 这里比较复杂，简化为：由于 angle 是相对于局部垂直的，我们需要绝对角度
      // 绝对火箭角 = local_up_angle + angle
      double rocket_abs_angle = local_up_angle + angle;
      double alpha = rocket_abs_angle - vel_angle;

      // 归一化到 -PI ~ PI
      while (alpha > PI)
        alpha -= 2 * PI;
      while (alpha < -PI)
        alpha += 2 * PI;

      // 恢复力矩：屁股朝下飞时(alpha接近PI)，如果不稳定会翻转
      // 我们假设有栅格翼，提供强大的稳定性 (Stability Factor)
      // 扭矩尝试减小 alpha
      double stability = 10.0; // 稳定性系数
      // 特殊情况：如果倒飞 (alpha ~ 180度)，我们需要它稳定在倒飞状态
      // 简化处理：仅仅施加阻尼和控制力矩，假设栅格翼工作良好

      // 这里的扭矩我们主要模拟“旋转阻力”，防止转个不停
      aero_torque -= ang_vel * 0.1 * v_mag * rho;
    }

    // C. 积分 (牛顿第二定律) - RK4 (Runge-Kutta 4th Order) 积分器
    // 为了支持极高的时间加速而不会因为 Euler 积分发散导致解体
    
    // 内部定义一个闭包，给定当前位置 (p) 和速度 (v) 计算加速度 (a)
    auto calc_accel = [&](double temp_px, double temp_py, double temp_vx, double temp_vy, double& out_ax, double& out_ay) {
        double r = sqrt(temp_px * temp_px + temp_py * temp_py);
        double alt = r - EARTH_RADIUS;
        
        // 重力
        double g_earth = get_gravity(r);
        double Fgx = -g_earth * (temp_px / r) * total_mass;
        double Fgy = -g_earth * (temp_py / r) * total_mass;
        
        // 太阳重力 (假设这段 dt 内太阳位置不变)
        double dx = sun_px - temp_px;
        double dy = sun_py - temp_py;
        double r_rocket_sq = dx*dx + dy*dy;
        double dist_rocket = sqrt(r_rocket_sq);
        double r_rocket3 = r_rocket_sq * dist_rocket;
        double Fg_sun_x = GM_sun * (dx / r_rocket3 - sun_px / r_sun_earth3) * total_mass;
        double Fg_sun_y = GM_sun * (dy / r_rocket3 - sun_py / r_sun_earth3) * total_mass;
        
        // 空气阻力
        double Fdx = 0, Fdy = 0;
        double temp_v_sq = temp_vx * temp_vx + temp_vy * temp_vy;
        double temp_v_mag = sqrt(temp_v_sq);
        if (temp_v_mag > 0.1 && alt < 80000) {
            double rho = get_air_density(alt);
            double drag_mag = 0.5 * rho * temp_v_sq * 0.5 * 10.0;
            Fdx = -drag_mag * (temp_vx / temp_v_mag);
            Fdy = -drag_mag * (temp_vy / temp_v_mag);
        }
        
        out_ax = (Fgx + Fg_sun_x + Ft_x + Fdx) / total_mass;
        out_ay = (Fgy + Fg_sun_y + Ft_y + Fdy) / total_mass;
    };

    double k1_vx, k1_vy, k1_px, k1_py;
    calc_accel(px, py, vx, vy, k1_vx, k1_vy);
    k1_px = vx; 
    k1_py = vy;

    double k2_vx, k2_vy, k2_px, k2_py;
    calc_accel(px + 0.5 * dt * k1_px, py + 0.5 * dt * k1_py, vx + 0.5 * dt * k1_vx, vy + 0.5 * dt * k1_vy, k2_vx, k2_vy);
    k2_px = vx + 0.5 * dt * k1_vx;
    k2_py = vy + 0.5 * dt * k1_vy;

    double k3_vx, k3_vy, k3_px, k3_py;
    calc_accel(px + 0.5 * dt * k2_px, py + 0.5 * dt * k2_py, vx + 0.5 * dt * k2_vx, vy + 0.5 * dt * k2_vy, k3_vx, k3_vy);
    k3_px = vx + 0.5 * dt * k2_vx;
    k3_py = vy + 0.5 * dt * k2_vy;

    double k4_vx, k4_vy, k4_px, k4_py;
    calc_accel(px + dt * k3_px, py + dt * k3_py, vx + dt * k3_vx, vy + dt * k3_vy, k4_vx, k4_vy);
    k4_px = vx + dt * k3_vx;
    k4_py = vy + dt * k3_vy;

    vx += (dt / 6.0) * (k1_vx + 2.0 * k2_vx + 2.0 * k3_vx + k4_vx);
    vy += (dt / 6.0) * (k1_vy + 2.0 * k2_vy + 2.0 * k3_vy + k4_vy);
    px += (dt / 6.0) * (k1_px + 2.0 * k2_px + 2.0 * k3_px + k4_px);
    py += (dt / 6.0) * (k1_py + 2.0 * k2_py + 2.0 * k3_py + k4_py);
    
    // 我们也需要更新显示用的加速度变量 (基于最新的 state)
    double final_ax, final_ay;
    calc_accel(px, py, vx, vy, final_ax, final_ay);
    acceleration = sqrt(final_ax * final_ax + final_ay * final_ay);

    // 角运动在下方统一计算（已修复重复积分 bug）
    double moment_of_inertia = 50000.0; // 假定转动惯量
    double ang_accel = 0;               // 此处仅声明，后续统一积分

    // 1. 真实垂直速度 (径向，正为向上)
    velocity = vx * cos(local_up_angle) + vy * sin(local_up_angle);

    // 2. 真实水平速度 (切向，正为顺时针飞行)
    // 利用向量点乘切向单位向量 (-sin, cos) 算出
    local_vx = -vx * sin(local_up_angle) + vy * cos(local_up_angle);

    // E. 碰撞检测
    double current_r = sqrt(px * px + py * py);
    double current_alt = current_r - EARTH_RADIUS;

    if (current_alt <= 0.0) {

      if (status == ASCEND) {
        // 如果是上升状态，说明只是推力还没把火箭推起来 (TWR < 1)
        // 此时强制锁在地面
        px = 0;
        py = EARTH_RADIUS;
        vx = 0;
        vy = 0;
        altitude = 0;
        // 不改变 status，继续积攒推力
      } else if (velocity < 0.1) {
        // 只有非上升状态，且没有明显向上速度时，才算着陆/坠毁
        altitude = 0;
        px = 0;
        py = EARTH_RADIUS;

        if (status != PRE_LAUNCH) {
          if (abs(velocity) > 10 || abs(local_vx) > 10) {
            status = CRASHED;
            cout << ">> CRASH! Impact Vel: " << velocity << endl;
          } else {
            status = LANDED;
            cout << ">> LANDED! Smoothly." << endl;
          }
          vx = 0;
          vy = 0;
          throttle = 0;
          ang_vel = 0;
          angle = 0;
          suicide_burn_locked = false; // 重置标志位
        }
      } else {

        altitude = current_alt;
      }
    } else {
      altitude = current_alt;
    }

    // 角运动 (I * alpha = Torque)
    moment_of_inertia = 50000.0;

    // 【物理升级】力矩不再是凭空产生的，而是推力矢量产生的
    // 假设喷管最大能偏转 5度 (0.087 rad)
    // PID 输出的 torque_cmd 现在代表“喷管偏转百分比 (-1.0 到 1.0)”

    // 如果引擎没开 (thrust_power=0)，就没有控制力矩 (RCS除外)
    // 这里模拟：主引擎推力 * 力臂(20米) * sin(喷管角度)
    double gimbal_torque = 0;
    if (thrust_power > 0) {
      // 限制 PID 输出在 -1 到 1 之间作为喷管指令
      double gimbal_cmd = max(-1.0, min(1.0, torque_cmd / 10000.0));
      double max_gimbal_angle = 0.1; // 约 5.7度

      // 力矩 = 推力 * sin(偏转角) * 质心距离
      gimbal_torque =
          thrust_power * sin(gimbal_cmd * max_gimbal_angle) * (height / 2);
    }

    // double final_torque = torque_cmd;

    double final_torque = torque_cmd;

    ang_accel = (final_torque + aero_torque) / moment_of_inertia;
    ang_vel += ang_accel * dt;
    angle += ang_vel * dt;
  }

  // 高帧率大步长纯重力积分 (时间加速极高倍率专供)
  void FastGravityUpdate(double dt_total) {
    if (status == PRE_LAUNCH || status == LANDED || status == CRASHED) return;

    // 无论时间加速多少倍，内部依然做最高 5.0 秒的小碎步切分计算，保证轨道非常稳定且不穿模
    double dt_step = 5.0; 
    double t_remaining = dt_total;
    
    double total_mass = mass + fuel;
    double au_meters = 149597870700.0;
    
    double G_const = 6.67430e-11;
    double M_sun = 1.989e30;
    double GM_sun = G_const * M_sun;
    double sun_angular_vel = sqrt(GM_sun / (au_meters * au_meters * au_meters));
    double mu_earth = G0 * pow(EARTH_RADIUS, 2);

    while (t_remaining > 0) {
      double dt = min(t_remaining, dt_step);
      t_remaining -= dt;
      sim_time += dt;
      
      auto calc_accel = [&](double temp_px, double temp_py, double temp_time, double& out_ax, double& out_ay) {
          double r2 = temp_px * temp_px + temp_py * temp_py;
          double r = sqrt(r2);
          
          double g_earth = mu_earth / r2;
          double Fgx = -g_earth * (temp_px / r) * total_mass;
          double Fgy = -g_earth * (temp_py / r) * total_mass;
          
          double sun_angle = -1.2 + sun_angular_vel * temp_time; 
          double sun_px = cos(sun_angle) * au_meters;
          double sun_py = sin(sun_angle) * au_meters;
          
          double dx = sun_px - temp_px;
          double dy = sun_py - temp_py;
          double r_rocket_sq = dx*dx + dy*dy;
          double dist_rocket = sqrt(r_rocket_sq);
          double r_rocket3 = r_rocket_sq * dist_rocket;
          double r_sun_earth3 = au_meters * au_meters * au_meters;
          
          double Fg_sun_x = GM_sun * (dx / r_rocket3 - sun_px / r_sun_earth3) * total_mass;
          double Fg_sun_y = GM_sun * (dy / r_rocket3 - sun_py / r_sun_earth3) * total_mass;
          
          out_ax = (Fgx + Fg_sun_x) / total_mass;
          out_ay = (Fgy + Fg_sun_y) / total_mass;
      };

      double k1_vx, k1_vy, k1_px, k1_py;
      calc_accel(px, py, sim_time - dt, k1_vx, k1_vy);
      k1_px = vx; k1_py = vy;

      double k2_vx, k2_vy, k2_px, k2_py;
      calc_accel(px + 0.5 * dt * k1_px, py + 0.5 * dt * k1_py, sim_time - dt + 0.5 * dt, k2_vx, k2_vy);
      k2_px = vx + 0.5 * dt * k1_vx; k2_py = vy + 0.5 * dt * k1_vy;

      double k3_vx, k3_vy, k3_px, k3_py;
      calc_accel(px + 0.5 * dt * k2_px, py + 0.5 * dt * k2_py, sim_time - dt + 0.5 * dt, k3_vx, k3_vy);
      k3_px = vx + 0.5 * dt * k2_vx; k3_py = vy + 0.5 * dt * k2_vy;

      double k4_vx, k4_vy, k4_px, k4_py;
      calc_accel(px + dt * k3_px, py + dt * k3_py, sim_time, k4_vx, k4_vy);
      k4_px = vx + dt * k3_vx; k4_py = vy + dt * k3_vy;

      vx += (dt / 6.0) * (k1_vx + 2.0 * k2_vx + 2.0 * k3_vx + k4_vx);
      vy += (dt / 6.0) * (k1_vy + 2.0 * k2_vy + 2.0 * k3_vy + k4_vy);
      px += (dt / 6.0) * (k1_px + 2.0 * k2_px + 2.0 * k3_px + k4_px);
      py += (dt / 6.0) * (k1_py + 2.0 * k2_py + 2.0 * k3_py + k4_py);
      
      altitude = sqrt(px*px + py*py) - EARTH_RADIUS;
      
      if (altitude <= 0.0) {
          altitude = 0;
          status = CRASHED;
          break;
      }
    }
  }

  bool is_Flying() {
    return (status == PRE_LAUNCH || status == ASCEND || status == DESCEND);
  }
  bool is_IntoSpace() { return (altitude > 100000); }
  bool is_IntoOrbit() { return (altitude > 100000 && abs(vx) > 7000); }

  // ---  AutoPilot  ---
  void AutoPilot(double dt) {
    if (status == PRE_LAUNCH || status == LANDED || status == CRASHED)
      return;

    // 1. 获取物理参数
    double max_thrust_vac = specific_impulse * G0 * cosrate;
    double current_total_mass = mass + fuel;
    double current_g = get_gravity(sqrt(px * px + py * py));

    // 因为我们预判接下来会大幅侧身，垂直分力不足 100%
    double conservative_thrust = max_thrust_vac * 0.95;
    double max_accel_avail =
        (conservative_thrust / current_total_mass) - current_g;

    // 2. 状态机逻辑

    if (status == ASCEND) {
      double apo, peri;
      getOrbitParams(apo, peri); // 获取实时开普勒轨道参数

      if (mission_phase == 0) {
        // 阶段 0：上升与重力转弯 (抬高远地点)
        throttle = 1.0;
        // 平滑重力转弯
        double target_pitch = 0;
        if (altitude > 1000) {
          target_pitch = -min(1.2, (altitude - 1000) / 80000.0 * 1.57);
        }
        torque_cmd = pid_att.update(target_pitch, angle, dt);

        // 目标远地点 150km 达成，立刻关机！
        if (apo > 150000) {
          mission_phase = 1;
          mission_msg = "MECO! COASTING TO APOAPSIS.";
        }
      } else if (mission_phase == 1) {
        // 阶段 1：无动力滑行至远地点
        throttle = 0;
        torque_cmd = pid_att.update(-PI / 2.0, angle, dt); // 飞船改平

        // 当火箭爬升到接近远地点 (垂直速度极小) 时，准备圆轨
        if (altitude > 130000 && velocity < 50) {
          mission_phase = 2;
          mission_msg = "CIRCULARIZATION BURN STARTED!";
        }
        // 保底：如果还没到 130km 就往下掉了，强行切入圆轨
        if (velocity < -50)
          mission_phase = 2;
      } else if (mission_phase == 2) {
        // 阶段 2：远地点圆轨点火 (拉高近地点)
        throttle = 1.0;
        torque_cmd =
            pid_att.update(-PI / 2.0, angle, dt); // 死死按住水平方向喷射

        // 当近地点突破 140km，说明轨道完美变圆！
        if (peri > 140000) {
          mission_phase = 3;
          mission_timer = 0;
          mission_msg = "ORBIT CIRCULARIZED! CRUISING.";
        }
      } else if (mission_phase == 3) {
        // 阶段 3：在轨巡航
        throttle = 0;
        torque_cmd = pid_att.update(-PI / 2.0, angle, dt);

        mission_timer += dt;
        // 绕轨巡航 6000 秒
        if (mission_timer > 5000.0) {
          mission_phase = 4;
          mission_msg = "DE-ORBIT SEQUENCE START.";
        }
      } else if (mission_phase == 4) {
        // 阶段 4：脱轨点火 (降低近地点)
        double vel_angle = atan2(vy, vx);
        double align_angle = (vel_angle + PI) - atan2(py, px); // 逆向对准
        while (align_angle > PI)
          align_angle -= 2 * PI;
        while (align_angle < -PI)
          align_angle += 2 * PI;

        torque_cmd = pid_att.update(align_angle, angle, dt);

        if (abs(angle - align_angle) < 0.1)
          throttle = 1.0;
        else
          throttle = 0.0;

        // 目标脱轨轨道：让近地点砸进 30km 的浓密大气层
        if (peri < 30000) {
          throttle = 0;
          status = DESCEND; // 移交着陆系统
          pid_vert.reset();
          mission_msg = ">> RE-ENTRY BURN COMPLETE. AEROBRAKING INITIATED.";
        }
      }
      return;
    }
    // --- 下落阶段 ---
    status = DESCEND;

    // 1. 独立计算 Y 轴绝对保命推力 (最低限度抗重力 + 垂直减速)
    double req_a_y = (velocity * velocity) / (2.0 * max(1.0, altitude));
    if (velocity > 0)
      req_a_y = 0; // 防反弹
    double req_F_y = current_total_mass * (current_g + req_a_y);

    // 2. 独立计算 X 轴消除速度所需推力 (【核心升级】：时间同步刹车)
    // 让水平和垂直速度在触地时“同时”归零！
    // 假设落地时间 T = 2 * 高度 / 垂直下落速度
    // 需要的水平加速度 a_x = v_x / T = (v_x * v_y) / (2 * 高度)
    double ref_vel = max(2.0, abs(velocity));
    double req_a_x = (-local_vx * ref_vel) / (2.0 * max(1.0, altitude));
    req_a_x += -local_vx * 0.5;
    double req_F_x = current_total_mass * req_a_x;

    // 3. 矢量合成与点火判断
    double req_thrust = sqrt(req_F_x * req_F_x + req_F_y * req_F_y);
    bool need_burn = suicide_burn_locked;

    // 3. 触发逻辑：当总需求推力达到引擎上限的 95%，且高度逼近低空时，极限点火！
    if (!need_burn && req_thrust > max_thrust_vac * 0.95 && altitude < 4000) {
      suicide_burn_locked = true;
      need_burn = true;
      mission_msg = ">> SYNCHRONIZED HOVERSLAM IGNITION!";
    }

    // 4. 防悬停锁：如果空气阻力极其给力，导致需求推力甚至连重力的 80% 都不到了
    // 坚决关机，白嫖自由落体！不到低空绝不悬停！
    if (suicide_burn_locked && altitude > 100.0 &&
        req_thrust < current_total_mass * current_g * 0.8) {
      suicide_burn_locked = false;
      need_burn = false;
      mission_msg = ">> AEROBRAKING COAST... WAITING.";
    }

    if (!need_burn) {
      throttle = 0;
      // 自由落体时，保持逆气流方向减阻
      double vel_angle = atan2(vy, vx);
      double align_angle = (vel_angle + PI) - atan2(py, px);
      while (align_angle > PI)
        align_angle -= 2 * PI;
      while (align_angle < -PI)
        align_angle += 2 * PI;
      torque_cmd = pid_att.update(align_angle, angle, dt);
    } else {

      // 理论最完美的侧滑角
      double target_angle = atan2(req_F_x, req_F_y);

      // 防撞地绝对极限倾角
      double max_safe_tilt = 1.5;
      if (req_F_y < max_thrust_vac) {
        max_safe_tilt = acos(req_F_y / max_thrust_vac);
      } else {
        max_safe_tilt = 0.0;
      }

      if (altitude < 100.0) {
        // 基础允许倾角随高度降低而逐渐减小
        double base_tilt = (altitude / 100.0) * 0.5;
        // 只要横向速度没消完，就允许借用倾角去疯狂刹车！(每1m/s侧滑借用约1.5度)
        double rescue_tilt = abs(local_vx) * 0.025;
        max_safe_tilt = min(max_safe_tilt, base_tilt + rescue_tilt);
      }

      // 只有当高度<2米且横向速度<1m/s 时，才真正立正触地！
      if (altitude < 2.0 && abs(local_vx) < 1.0)
        max_safe_tilt = 0.0;

      if (target_angle > max_safe_tilt)
        target_angle = max_safe_tilt;
      if (target_angle < -max_safe_tilt)
        target_angle = -max_safe_tilt;

      torque_cmd = pid_att.update(target_angle, angle, dt);

      // 最终油门计算
      if (altitude > 2.0) {
        double final_thrust = req_F_y / max(0.1, cos(target_angle));
        throttle = final_thrust / max_thrust_vac;
      } else {
        // 最后 2 米触地段，同样需要姿态补偿！
        double hover_throttle =
            (current_total_mass * current_g) / max_thrust_vac;
        throttle = hover_throttle + ((-1.5 - velocity) * 2.0);
        throttle /= max(0.8, cos(target_angle));
      }
    }

    if (throttle > 1.0)
      throttle = 1.0;
    if (throttle < 0.0)
      throttle = 0.0;
  }

  // 手动发射
  void ManualLaunch() {
    if (status == PRE_LAUNCH) {
      status = ASCEND;
    }
  }

  // --- 手动控制模式 ---
  void ManualControl(GLFWwindow *window, double dt) {
    if (status == PRE_LAUNCH || status == LANDED || status == CRASHED)
      return;

    // 油门控制
    if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS ||
        glfwGetKey(window, GLFW_KEY_UP) == GLFW_PRESS)
      throttle = min(1.0, throttle + 1.5 * dt);
    if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS ||
        glfwGetKey(window, GLFW_KEY_DOWN) == GLFW_PRESS)
      throttle = max(0.0, throttle - 1.5 * dt);
    if (glfwGetKey(window, GLFW_KEY_Z) == GLFW_PRESS)
      throttle = 1.0;
    if (glfwGetKey(window, GLFW_KEY_X) == GLFW_PRESS)
      throttle = 0.0;

    // 姿态控制 (施加力矩)
    torque_cmd = 0;
    if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS ||
        glfwGetKey(window, GLFW_KEY_LEFT) == GLFW_PRESS)
      torque_cmd = 30000.0; // 逆时针 (向左倾)
    if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS ||
        glfwGetKey(window, GLFW_KEY_RIGHT) == GLFW_PRESS)
      torque_cmd = -30000.0; // 顺时针 (向右倾)

    // 维持 status 更新
    if (status == ASCEND && velocity < 0 && altitude > 1000)
      status = DESCEND;
  }

  // 切换控制模式
  void ToggleMode() {
    auto_mode = !auto_mode;
    if (auto_mode) {
      // 切回自动：重置 PID 积分项，防止积分饱和
      pid_vert.reset();
      pid_pos.reset();
      pid_att.reset();
      mission_msg = ">> AUTOPILOT ENGAGED";
    } else {
      mission_msg = ">> MANUAL CONTROL ACTIVE";
    }
  }
};

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

int main() {

  glfwInit();
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

  // 请求一个包含 4倍多重采样 的帧缓冲
  glfwWindowHint(GLFW_SAMPLES, 4);

  GLFWwindow *window = glfwCreateWindow(1000, 800, "2D Rocket Sim", NULL, NULL);
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

  Explorer baba1(total_fuel, total_dry, 3.7, total_height,
                 1, sel_eng.isp, sel_eng.consumption, 0.5);

  // =========================================================
  // 初始化 3D 渲染器和网格
  // =========================================================
  Renderer3D* r3d = new Renderer3D();
  Mesh earthMesh = MeshGen::sphere(48, 64, 1.0f);  // 单位球，用 model 矩阵缩放
  Mesh rocketBody = MeshGen::cylinder(16, 1.0f, 1.0f);
  Mesh rocketNose = MeshGen::cone(16, 1.0f, 1.0f);
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

  while (baba1.is_Flying() && !glfwWindowShouldClose(window)) {
    glfwPollEvents();
    if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
      glfwSetWindowShouldClose(window, true);
    if (glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_PRESS)
      baba1.ManualLaunch();

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
        baba1.ToggleMode();
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
    bool can_super_warp = (!baba1.auto_mode && baba1.getAltitude() > 100000.0 && (baba1.getThrust() == 0 || baba1.getFuel() <= 0));

    if (baba1.auto_mode) {
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

    // --- 物理更新 ---
    if (time_warp > 1000) {
        // 超级时间加速！绕过气动和引擎模拟，直接当作质点切分迭代
        baba1.FastGravityUpdate(dt * time_warp);
    } else {
        // 普通循环执行实现加速
        for (int i = 0; i < time_warp; i++) {
          if (baba1.auto_mode) baba1.AutoPilot(dt);
          else baba1.ManualControl(window, dt);
          baba1.Burn(dt);
          if (!baba1.is_Flying()) break;
        }

        // --- 额外一步物理更新 (用于尾烟特效) ---
        if (time_warp == 1) {
            baba1.emitSmoke(dt);
            baba1.updateSmoke(dt);
        }
    }

    // 只有每隔一定帧数才打印，防止控制台看不清
    if (frame % 10 == 0)
      baba1.Report_Status();

    this_thread::sleep_for(chrono::milliseconds(20)); // 限制帧率

    // 画面刷新
    // 画面刷新
    float alt_factor = (float)min(baba1.getAltitude() / 50000.0, 1.0);
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
      double r_px = baba1.px * ws_d;
      double r_py = baba1.py * ws_d;
      double r_pz = 0.0;
      
      // ===== 太阳位置与昼夜交替 (Double precision) =====
      double sun_angular_vel = 6.2831853 / 31557600.0;
      double sun_angle_d = -1.2 + sun_angular_vel * baba1.sim_time;
      double au_meters_d = 149597870700.0;
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
      if (baba1.getAltitude() > 2000000.0) { // 2000公里外，完全脱离近地轨道，进入深空
          rocketUp = Vec3(0.0f, 1.0f, 0.0f);
      }
      
      float rocket_angle = (float)baba1.angle;
      Vec3 localUp = rocketUp;
      Vec3 localRight(-rocketUp.y, rocketUp.x, 0.0f);
      Vec3 rocketDir(
        localUp.x * cosf(rocket_angle) + localRight.x * sinf(rocket_angle),
        localUp.y * cosf(rocket_angle) + localRight.y * sinf(rocket_angle),
        0.0f
      );

      // 火箭尺寸
      float rocket_vis_scale = 1.0f;
      float rh = (float)baba1.getHeight() * (float)ws_d * rocket_vis_scale;
      float rw_3d = (float)baba1.getDiameter() * (float)ws_d * 0.5f * rocket_vis_scale;

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
        // 选择参考系
        double mu_body = orbit_reference_sun ? (9.81 * earth_r * earth_r * ws_d * 333000.0) : (9.81 * earth_r * earth_r * ws_d); 

        // 全物理量双精度计算
        double abs_px = r_px, abs_py = r_py, abs_pz = r_pz;
        double abs_vx = baba1.vx * ws_d, abs_vy = baba1.vy * ws_d, abs_vz = 0.0;

        if (orbit_reference_sun) {
            // Use exact same derived sun_angle and sun_angular_vel as physics engine (Burn)
            double G_const = 6.67430e-11;
            double M_sun = 1.989e30;
            double GM_sun = G_const * M_sun;
            double au_meters = 149597870700.0;
            double sun_angular_vel = sqrt(GM_sun / (au_meters * au_meters * au_meters));
            
            double exact_sun_angle = -1.2 + sun_angular_vel * baba1.sim_time;
            
            // Earth's exact orbital velocity matching the physics steps
            double earth_vx = -sin(exact_sun_angle) * au_meters * sun_angular_vel;
            double earth_vy = cos(exact_sun_angle) * au_meters * sun_angular_vel;
            
            // 地心相对日心为 -sunPos
            abs_px -= cos(exact_sun_angle) * au_meters; 
            abs_py -= sin(exact_sun_angle) * au_meters; 
            abs_pz -= sun_pz;

            // Notice we are NOT scaling the velocity by ws_d, because earth_vx is already correct scale
            abs_vx += earth_vx * ws_d; 
            abs_vy += earth_vy * ws_d;
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


          if (ecc < 1.0f) {
            // --- 椭圆轨道 (a > 0) ---
            float b = (float)a * sqrtf(fmaxf(0.0f, 1.0f - ecc * ecc));
            Vec3 e_dir = ecc > 1e-6f ? e_vec / ecc : Vec3(1.0f, 0.0f, 0.0f);
            Vec3 perp_dir = h_vec.normalized().cross(e_dir);

            float periapsis = (float)a * (1.0f - ecc);
            float apoapsis = (float)a * (1.0f + ecc);
            bool will_reenter = periapsis < earth_r && !orbit_reference_sun;

            Vec3 center_off = e_dir * (-(float)a * ecc);

            // 生成预测轨迹点集
            std::vector<Vec3> orbit_points;
            int orbit_segs = 120;
            for (int i = 0; i <= orbit_segs; i++) {
              float ang = (float)i / orbit_segs * 6.2831853f;
              Vec3 pt_rel = center_off + e_dir * ((float)a * cosf(ang)) + perp_dir * (b * sinf(ang));
              double p_x = pt_rel.x, p_y = pt_rel.y, p_z = pt_rel.z;
              if (orbit_reference_sun) { p_x += sun_px; p_y += sun_py; p_z += sun_pz; }
              Vec3 pt = Vec3((float)(p_x - ro_x), (float)(p_y - ro_y), (float)(p_z - ro_z));
              if (!orbit_reference_sun && pt.length() < earth_r * 0.98f) continue;
              orbit_points.push_back(pt);
            }
            
            // 渲染预测轨迹
            float pred_w = earth_r * 0.0025f * fmaxf(1.0f, cam_zoom_pan * 0.5f);
            float macro_fade = fminf(1.0f, fmaxf(0.0f, (cam_zoom_pan - 0.05f) / 0.1f));
            if (macro_fade > 0.01f) {
                if (will_reenter) {
                  r3d->drawRibbon(orbit_points, pred_w, 1.0f, 0.4f, 0.1f, 0.8f * macro_fade);
                } else {
                  r3d->drawRibbon(orbit_points, pred_w, 0.2f, 0.8f, 1.0f, 0.8f * macro_fade); 
                }
            }

            float apsis_size = fminf(earth_r * 0.12f, earth_r * 0.025f * fmaxf(1.0f, cam_zoom_pan * 0.8f));
            apsis_size *= (orbit_reference_sun ? 10.0f : 1.0f);
            
            // 远地点标记
            Vec3 apoPos = e_dir * (-apoapsis);
            double ax = apoPos.x, ay = apoPos.y, az = apoPos.z;
            if (orbit_reference_sun) { ax += sun_px; ay += sun_py; az += sun_pz; }
            Vec3 w_apoPos((float)(ax - ro_x), (float)(ay - ro_y), (float)(az - ro_z));
            if (apoapsis > earth_r * 1.002f) {
              r3d->drawBillboard(w_apoPos, apsis_size, 0.2f, 0.4f, 1.0f, 0.9f);
            }

            // 近地点标记
            Vec3 periPos = e_dir * periapsis;
            double px = periPos.x, py = periPos.y, pz = periPos.z;
            if (orbit_reference_sun) { px += sun_px; py += sun_py; pz += sun_pz; }
            Vec3 w_periPos((float)(px - ro_x), (float)(py - ro_y), (float)(pz - ro_z));
            if (periapsis > earth_r * 1.002f) {
              r3d->drawBillboard(w_periPos, apsis_size, 1.0f, 0.5f, 0.1f, 0.9f);
            }
          } else {
            // --- 双曲线/抛物线逃逸轨道 (ecc >= 1.0) ---
            // 半长轴 a 为负值
            float a_hyp = fabs(a); // 实际 a 是负的，我们取绝对值方便计算
            float b_hyp = a_hyp * sqrtf(fmaxf(0.0f, ecc * ecc - 1.0f));
            Vec3 e_dir = e_vec / ecc;
            Vec3 perp_dir = h_vec.normalized().cross(e_dir);
            
            float periapsis = a_hyp * (ecc - 1.0f);
            Vec3 center_off = e_dir * (a_hyp * ecc);

            // 绘制双曲线的一支（火箭前进方向的）
            std::vector<Vec3> escape_points;
            int escape_segs = 60;
            // 真近点角渐近线极限 acos(-1/ecc)
            float max_theta = acosf(-1.0f / ecc) * 0.9f; 
            
            // 我们沿着参数方程扫描，不画完整的，只画离开地球的一段距离
            float max_sinh = 3.0f; 
            for (int i = -escape_segs; i <= escape_segs; i++) {
               float t = (float)i / escape_segs * max_sinh;
               // 取轨迹方程 : center - a * cosh(t) * e_dir + b * sinh(t) * perp_dir
               Vec3 pt_rel = center_off - e_dir * (a_hyp * coshf(t)) + perp_dir * (b_hyp * sinhf(t));
               double px = pt_rel.x, py = pt_rel.y, pz = pt_rel.z;
               if (orbit_reference_sun) { px += sun_px; py += sun_py; pz += sun_pz; }
               
               Vec3 pt((float)(px - ro_x), (float)(py - ro_y), (float)(pz - ro_z));
               if (!orbit_reference_sun && pt.length() < earth_r * 0.98f) continue;
               
               // 只保留未来要飞的，或者火箭当前位置附近的点(基于点乘判断行进方向)
               if (escape_points.empty() || (pt - escape_points.back()).length() > earth_r * 0.05f) {
                  escape_points.push_back(pt);
               }
            }

            // 逃逸轨道的 Ribbon (紫色代表逃逸)
            float pred_w = earth_r * 0.0025f * fmaxf(1.0f, cam_zoom_pan * 0.5f);
            float macro_fade = fminf(1.0f, fmaxf(0.0f, (cam_zoom_pan - 0.05f) / 0.1f));
            if (macro_fade > 0.01f) {
                r3d->drawRibbon(escape_points, pred_w, 0.8f, 0.3f, 1.0f, 0.8f * macro_fade);
            }

            // 近地点标记
            float apsis_size = fminf(earth_r * 0.12f, earth_r * 0.025f * fmaxf(1.0f, cam_zoom_pan * 0.8f));
            apsis_size *= (orbit_reference_sun ? 10.0f : 1.0f);
            Vec3 periPos = center_off - e_dir * a_hyp; // 顶点就是近地点
            double px = periPos.x, py = periPos.y, pz = periPos.z;
            if (orbit_reference_sun) { px += sun_px; py += sun_py; pz += sun_pz; }
            Vec3 w_periPos((float)(px - ro_x), (float)(py - ro_y), (float)(pz - ro_z));
            if (periapsis > earth_r * 1.002f) {
              r3d->drawBillboard(w_periPos, apsis_size, 1.0f, 0.5f, 0.1f, 0.9f);
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

      // ===== 火箭朝向四元数 =====
      Vec3 defaultUp(0.0f, 1.0f, 0.0f);
      Vec3 rotAxis = defaultUp.cross(rocketDir);
      float rotAngle = acosf(fminf(fmaxf(defaultUp.dot(rocketDir), -1.0f), 1.0f));
      Quat rocketQuat;
      if (rotAxis.length() > 1e-6f)
        rocketQuat = Quat::fromAxisAngle(rotAxis.normalized(), rotAngle);

      // ===== 火箭涂装 (Builder颜色) =====
      // 机体 (燃料箱颜色)
      Mat4 bodyModel = Mat4::TRS(renderRocketPos, rocketQuat,
                                  Vec3(rw_3d, rh * 0.7f, rw_3d));
      r3d->drawMesh(rocketBody, bodyModel,
                     sel_tank.r, sel_tank.g, sel_tank.b, 1.0f, 0.15f);

      // 鼻锥 (鼻锥颜色)
      Vec3 nosePos = renderRocketPos + rocketDir * (rh * 0.35f);
      Mat4 noseModel = Mat4::TRS(nosePos, rocketQuat,
                                  Vec3(rw_3d, rh * 0.3f, rw_3d));
      r3d->drawMesh(rocketNose, noseModel,
                     sel_nose.r, sel_nose.g, sel_nose.b, 1.0f, 0.15f);

      // 引擎喷管 (引擎颜色)
      Vec3 engPos = renderRocketPos - rocketDir * (rh * 0.35f);
      Quat flipQuat = Quat::fromAxisAngle(
        rotAxis.length() > 1e-6f ? rotAxis.normalized() : Vec3(0.0f, 0.0f, 1.0f),
        rotAngle + 3.14159f);
      Mat4 engModel = Mat4::TRS(engPos, flipQuat,
                                 Vec3(rw_3d * 1.2f, rh * 0.15f, rw_3d * 1.2f));
      r3d->drawMesh(rocketNose, engModel,
                     sel_eng.r, sel_eng.g, sel_eng.b, 1.0f, 0.2f);

      // ===== 3D 火焰/尾焰粒子 =====
      if (baba1.getThrust() > 0.0) {
        float thrust = (float)baba1.throttle;
        float flame_base_size = rw_3d * 1.5f;
        float game_time = (float)glfwGetTime();

        // 真空中火焰更大（大气压缩效果）
        float vacuum_scale = 1.0f + (float)fmin(baba1.getAltitude() / 50000.0, 2.0);

        for (int i = 0; i < 10; i++) {
          // 粒子沿推力反方向分布
          float t_off = (float)i / 10.0f;
          float rand_off = sinf(game_time * 17.3f + i * 7.1f) * 0.3f + 0.5f;
          float dist = (t_off * 0.5f + rand_off * 0.2f) * rh * thrust * vacuum_scale;
          Vec3 particlePos = engPos - rocketDir * dist;

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
    double scale = 1.0 / (baba1.getAltitude() * 1.5 + 200.0);
    float cx = 0.0f;
    float cy = 0.0f;
    double rocket_r = sqrt(baba1.px * baba1.px + baba1.py * baba1.py);
    double rocket_theta = atan2(baba1.py, baba1.px);
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

    double current_vel = baba1.getVelocityMag();
    double current_alt = baba1.getAltitude();
    int current_vvel = (int)baba1.getVerticalVel();

    if (orbit_reference_sun) {
        double sun_angular_vel = 6.2831853 / 31557600.0;
        double sun_angle = -1.2 + sun_angular_vel * baba1.sim_time;
        double au = 149597870700.0;
        double current_sun_px = cos(sun_angle) * au;
        double current_sun_py = sin(sun_angle) * au;
        double current_sun_vx = -sin(sun_angle) * au * sun_angular_vel;
        double current_sun_vy = cos(sun_angle) * au * sun_angular_vel;

        double rel_vx = baba1.vx - current_sun_vx;
        double rel_vy = baba1.vy - current_sun_vy;
        double rel_px = baba1.px - current_sun_px;
        double rel_py = baba1.py - current_sun_py;
        
        current_vel = sqrt(rel_vx * rel_vx + rel_vy * rel_vy);
        double dist_to_sun = sqrt(rel_px * rel_px + rel_py * rel_py);
        current_alt = dist_to_sun - 696340000.0; 
        
        double rel_vvel_real = (rel_vx * rel_px + rel_vy * rel_py) / dist_to_sun;
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
    double current_fuel = baba1.getFuel();
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
    renderer->drawNumber(num_x - 0.02f, 0.25f, (int)(baba1.throttle * 100),
                         num_size * 0.8f, 0.8f, 0.8f, 0.8f, hud_opacity);
    renderer->drawLabel(label_x - 0.01f, 0.25f, "%", num_size * 0.6f,
                        0.8f, 0.8f, 0.8f, hud_opacity);

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
    if (baba1.auto_mode) {
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
    float thr_fill = thr_bar_w * (float)baba1.throttle;
    float thr_fill_x = thr_bar_x - thr_bar_w / 2.0f + thr_fill / 2.0f;
    float thr_r = (float)baba1.throttle > 0.8f
                      ? 1.0f
                      : 0.3f + (float)baba1.throttle * 0.7f;
    float thr_g = (float)baba1.throttle < 0.5f
                      ? 0.8f
                      : 0.8f - ((float)baba1.throttle - 0.5f) * 1.6f;
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
  if (baba1.status == Explorer::LANDED || baba1.status == Explorer::CRASHED) {
    cout << "\n>> SIMULATION ENDED. PRESS ENTER." << endl;
    cin.get();
  }
  return 0;
}