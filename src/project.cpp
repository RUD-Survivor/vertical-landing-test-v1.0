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
    cout << "[Alt]: " << altitude << " m | [Vert_Vel]: " << velocity << " m/s"
         << endl;
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
    // 强制公转周期为 precies 365.25 天 (以秒为单位) = 31557600s
    // 太阳视角角速度: 2 * PI / 31557600
    double sun_angular_vel = 6.2831853 / 31557600.0;
    double sun_angle = -1.2 + sun_angular_vel * sim_time; 
    
    // 太阳在以地球为中心的坐标系中的位置
    double sun_px = cos(sun_angle) * au_meters;
    double sun_py = sin(sun_angle) * au_meters;
    
    double dx_sun = sun_px - px;
    double dy_sun = sun_py - py;
    
    double r_sun_rocket_sq = dx_sun*dx_sun + dy_sun*dy_sun;
    double dist_sun_rocket = sqrt(r_sun_rocket_sq);
    double r_sun_rocket3 = r_sun_rocket_sq * dist_sun_rocket;
    
    double dist_sun_earth = au_meters;
    double r_sun_earth3 = dist_sun_earth * dist_sun_earth * dist_sun_earth;
    
    // 太阳质量引力常数 (G * M_sun)
    double GM_sun = G0 * pow(EARTH_RADIUS, 2) * 333000.0;
    
    // 第三体扰动：太阳对火箭的引力 减去 太阳对地球的引力 
    // (因为我们的坐标系原点是地球，地球本身在随太阳运动)
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

    // C. 积分 (牛顿第二定律)
    // 线运动
    double ax = (Fg_x + Ft_x + Fd_x) / total_mass;
    double ay = (Fg_y + Ft_y + Fd_y) / total_mass;

    vx += ax * dt;
    vy += ay * dt;
    px += vx * dt;
    py += vy * dt;

    // 角运动在下方统一计算（已修复重复积分 bug）
    double moment_of_inertia = 50000.0; // 假定转动惯量
    double ang_accel = 0;               // 此处仅声明，后续统一积分

    // 1. 真实垂直速度 (径向，正为向上)
    velocity = vx * cos(local_up_angle) + vy * sin(local_up_angle);

    // 2. 真实水平速度 (切向，正为顺时针飞行)
    // 利用向量点乘切向单位向量 (-sin, cos) 算出
    local_vx = -vx * sin(local_up_angle) + vy * cos(local_up_angle);

    acceleration = sqrt(ax * ax + ay * ay);

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
               double scale, float cx, float cy, double cam_angle) {
  double r_mag = sqrt(px * px + py * py);
  double mu = 9.80665 * 6371000.0 * 6371000.0; // 地球标准引力参数

  double h = px * vy - py * vx; // 轨道角动量
  if (abs(h) < 1.0)
    return;

  // 计算偏心率矢量 (e)
  double ex = (vy * h) / mu - px / r_mag;
  double ey = (-vx * h) / mu - py / r_mag;
  double e_mag = sqrt(ex * ex + ey * ey);

  double p = (h * h) / mu;      // 半通径
  double omega = atan2(ey, ex); // 近地点幅角

  int segments = 5000; // 轨道分段渲染
  float prev_x = 0, prev_y = 0;
  bool first = true;

  double sin_c = sin(cam_angle);
  double cos_c = cos(cam_angle);

  for (int i = 0; i <= segments; i++) {
    double nu = 2.0 * PI * i / segments; // 真近点角
    double denom = 1.0 + e_mag * cos(nu - omega);
    if (denom <= 0.05)
      continue; // 忽略逃逸轨道的无限远端

    double r_nu = p / denom;
    if (r_nu > EARTH_RADIUS * 5) {
      first = true;
      continue;
    }; // 太远的就不画了

    double x_orb = r_nu * cos(nu);
    double y_orb = r_nu * sin(nu);

    // 旋转映射到摄像机屏幕
    double rx = x_orb * cos_c - y_orb * sin_c;
    double ry = x_orb * sin_c + y_orb * cos_c;
    float screen_x = (float)(rx * scale + cx);
    float screen_y = (float)((ry - r_mag) * scale + cy);

    if (!first) {
      // 大气层内或地下显示红色警告，安全轨道显示为黄色
      float r_col = 1.0f, g_col = 0.8f, b_col = 0.0f;
      if (r_nu < EARTH_RADIUS + 80000.0) {
        r_col = 1.0f;
        g_col = 0.0f;
        b_col = 0.0f;
      }

      // 画出预测轨迹线
      float thickness = 0.005f;
      float dx = screen_x - prev_x, dy = screen_y - prev_y;
      float len = sqrt(dx * dx + dy * dy);
      if (len > 0) {
        float nx = -dy / len * thickness, ny = dx / len * thickness;
        renderer->addVertex(prev_x + nx, prev_y + ny, r_col, g_col, b_col,
                            1.0f);
        renderer->addVertex(prev_x - nx, prev_y - ny, r_col, g_col, b_col,
                            1.0f);
        renderer->addVertex(screen_x + nx, screen_y + ny, r_col, g_col, b_col,
                            1.0f);

        renderer->addVertex(screen_x + nx, screen_y + ny, r_col, g_col, b_col,
                            1.0f);
        renderer->addVertex(prev_x - nx, prev_y - ny, r_col, g_col, b_col,
                            1.0f);
        renderer->addVertex(screen_x - nx, screen_y - ny, r_col, g_col, b_col,
                            1.0f);
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
  static bool v_was_pressed = false;
  bool view_3d = false;
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

  std::vector<Vec3> traj_history; // 记录火箭历史飞行的 3D 轨迹点

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

    // --- V 键切换 2D/3D 视图 ---
    bool v_now = glfwGetKey(window, GLFW_KEY_V) == GLFW_PRESS;
    if (v_now && !v_was_pressed) {
      view_3d = !view_3d;
      cout << ">> View: " << (view_3d ? "3D" : "2D") << endl;
    }
    v_was_pressed = v_now;

    // --- 鼠标轨道控制 (3D模式下右键拖动) ---
    if (view_3d) {
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
    if (glfwGetKey(window, GLFW_KEY_O) == GLFW_PRESS) {
      if (!o_was_pressed) {
        show_orbit = !show_orbit;
        o_was_pressed = true;
      }
    } else {
      o_was_pressed = false;
    }

    // --- 时间加速逻辑 ---
    static int time_warp = 1;
    // 手动模式下禁止时间加速
    if (baba1.auto_mode) {
      if (glfwGetKey(window, GLFW_KEY_1) == GLFW_PRESS)
        time_warp = 1;
      if (glfwGetKey(window, GLFW_KEY_2) == GLFW_PRESS)
        time_warp = 10;
      if (glfwGetKey(window, GLFW_KEY_3) == GLFW_PRESS)
        time_warp = 100;
      if (glfwGetKey(window, GLFW_KEY_4) == GLFW_PRESS)
        time_warp = 1000;
    } else {
      time_warp = 1; // 手动模式强制 1x 速度
    }

    // --- 物理更新 (循环执行实现加速) ---
    for (int i = 0; i < time_warp; i++) {
      if (baba1.auto_mode)
        baba1.AutoPilot(dt);
      else
        baba1.ManualControl(window, dt);
      baba1.Burn(dt);
      if (!baba1.is_Flying())
        break;
    }

    // --- 额外一步物理更新 ---
    if (baba1.auto_mode)
      baba1.AutoPilot(dt);
    else
      baba1.ManualControl(window, dt);
    baba1.Burn(dt);
    baba1.emitSmoke(dt);
    baba1.updateSmoke(dt);

    // 只有每隔一定帧数才打印，防止控制台看不清
    static int frame = 0;
    if (frame++ % 10 == 0)
      baba1.Report_Status();

    this_thread::sleep_for(chrono::milliseconds(20)); // 限制帧率

    // 画面刷新
    // 画面刷新
    float alt_factor = (float)min(baba1.getAltitude() / 50000.0, 1.0);
    if (view_3d && cam_mode_3d == 2) {
       // 全景视角(Panorama)下背景应该始终为全黑(太空)
       glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    } else {
       // 地球表面起飞的蓝天渐变
       glClearColor(my_lerp(0.5f, 0.0f, alt_factor), my_lerp(0.7f, 0.0f, alt_factor),
                    my_lerp(1.0f, 0.0f, alt_factor), 1.0f);
    }
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // ================= 3D 渲染通道 (仅3D模式) =================
    if (view_3d) {
      // 世界坐标缩放
      float ws = 0.001f;
      float earth_r = (float)EARTH_RADIUS * ws;

      // 2D→3D 坐标映射
      Vec3 rocketPos((float)baba1.px * ws, (float)baba1.py * ws, 0.0f);
      Vec3 rocketUp = rocketPos.normalized();
      float rocket_angle = (float)baba1.angle;
      Vec3 localUp = rocketUp;
      Vec3 localRight(-rocketUp.y, rocketUp.x, 0.0f);
      Vec3 rocketDir(
        localUp.x * cosf(rocket_angle) + localRight.x * sinf(rocket_angle),
        localUp.y * cosf(rocket_angle) + localRight.y * sinf(rocket_angle),
        0.0f
      );

      // 火箭尺寸：放大500倍让它可见 (实际40m，放大后= 20单位 vs 地球6371单位)
      float rocket_vis_scale = 500.0f;
      float rh = (float)baba1.getHeight() * ws * rocket_vis_scale;
      float rw_3d = (float)baba1.getDiameter() * ws * 0.5f * rocket_vis_scale;

      // 相机设置
      int ww, wh;
      glfwGetFramebufferSize(window, &ww, &wh);
      float aspect = (float)ww / (float)wh;

      Vec3 camEye, camTarget, camUpVec;

      // ===== 太阳位置与昼夜交替 =====
      // 获取极其精确的物理时间，计算出真正的 365天 公转角度
      double sun_angular_vel = 6.2831853 / 31557600.0;
      float sun_angle = -1.2f + (float)(sun_angular_vel * baba1.sim_time);
      
      // 太阳距离设为 1 AU (1天文单位)
      float au_meters = 149597870700.0f;
      float sun_dist = au_meters * ws;
      float sun_radius = earth_r * 109.2f; // 物理半径比例
      
      // 让太阳在赤道面上带 11.3度 倾角旋转 (完美圆形，保持 1 AU 绝对距离)
      // 11.3 度 = 0.197 弧度. sin(11.3)=0.196, cos(11.3)=0.980
      Vec3 sunPos(cosf(sun_angle) * sun_dist, 
                  sinf(sun_angle) * sun_dist * 0.196f, 
                  sinf(sun_angle) * sun_dist * 0.980f);
      
      // 更新全局光照方向 (方向为从物体指向光源)
      r3d->lightDir = sunPos.normalized();

      // ===== 按键监听：视点切换 =====
      static int focus_target = 0; // 0=地球, 1=太阳
      static bool comma_prev = false;
      static bool period_prev = false;
      bool comma_now = glfwGetKey(window, GLFW_KEY_COMMA) == GLFW_PRESS;
      bool period_now = glfwGetKey(window, GLFW_KEY_PERIOD) == GLFW_PRESS;
      if (comma_now && !comma_prev) focus_target = 0;
      if (period_now && !period_prev) focus_target = 1;
      comma_prev = comma_now; period_prev = period_now;

      if (cam_mode_3d == 0) {
        // 轨道视角：四元数控制绕火箭旋转
        float orbit_dist = rh * 8.0f * cam_zoom_chase;
        Vec3 cam_offset = cam_quat.rotate(Vec3(0.0f, 0.0f, orbit_dist));
        camEye = rocketPos + cam_offset;
        camTarget = rocketPos;
        camUpVec = cam_quat.rotate(Vec3(0.0f, 1.0f, 0.0f));
      } else if (cam_mode_3d == 1) {
        // 跟踪视角：固定在火箭后上方
        float chase_dist = rh * 8.0f * cam_zoom_chase;
        Vec3 chase_base = rocketDir * (-chase_dist * 0.4f) +
                          rocketUp * (chase_dist * 0.15f);
        Vec3 slight_off = cam_quat.rotate(Vec3(0.0f, 0.0f, 1.0f));
        camEye = rocketPos + chase_base + slight_off * (chase_dist * 0.05f);
        camTarget = rocketPos + rocketDir * (rh * 3.0f);
        camUpVec = rocketUp;
      } else {
        // 全景视角：支持聚焦点切换 (地球/太阳)
        float pan_dist = earth_r * 4.0f * cam_zoom_pan;
        if (focus_target == 0) {
            Vec3 cam_offset = cam_quat.rotate(Vec3(0.0f, 0.0f, pan_dist));
            camEye = cam_offset; // 绕地心旋转
            camTarget = Vec3(0.0f, 0.0f, 0.0f);
        } else {
            // 聚焦太阳时，由于太阳体积是地球109倍，相机初始距离也要拉远109倍才能看全
            Vec3 cam_offset = cam_quat.rotate(Vec3(0.0f, 0.0f, pan_dist * 110.0f));
            camEye = sunPos + cam_offset; // 绕日心旋转
            camTarget = sunPos;
        }
        camUpVec = cam_quat.rotate(Vec3(0.0f, 1.0f, 0.0f));
      }

      // 动态远裁剪面 
      float cam_dist = (camEye - camTarget).length();
      float dist_to_origin = camEye.length();
      float far_plane = fmaxf(cam_dist * 3.0f, dist_to_origin + earth_r * 2.0f);
      // 如果相机离得很远（看太阳或极远视角），必须要拓展远裁面涵盖整个 AU 距离
      if (focus_target == 1 || cam_dist > sun_dist * 0.1f) {
          far_plane = fmaxf(far_plane, sun_dist * 2.5f);
      }
      
      // 近裁剪面：安全覆盖摄像机前的物体，并且不能比摄像机的距离大（否则裁剪目标）
      float safe_near = fmaxf(rh * 0.1f, cam_dist * 0.005f);
      float near_plane = safe_near;
      Mat4 projMat = Mat4::perspective(0.8f, aspect, near_plane, far_plane);

      Mat4 viewMat = Mat4::lookAt(camEye, camTarget, camUpVec);
      r3d->beginFrame(viewMat, projMat, camEye);

      // ===== 太阳物理本体 =====
      if (cam_mode_3d == 2) { 
          // 仅在全景模式渲染巨型的物理太阳模型避免遮盖火箭本体细节
          Mat4 sunModel = Mat4::scale(Vec3(sun_radius, sun_radius, sun_radius));
          sunModel = Mat4::translate(sunPos) * sunModel;
          // 复用 earthMesh，修改极高环境光(ambient=2.0)让其纯亮发白发黄
          r3d->drawMesh(earthMesh, sunModel, 1.0f, 0.95f, 0.9f, 1.0f, 2.0f); 
          
          // 地球绕日轨道线 (以太阳为中心画一个1AU半径的完美圆)
          std::vector<Vec3> earth_orbit_pts;
          int orbit_res = 120;
          for (int i=0; i<=orbit_res; i++) {
              float a = (float)i / orbit_res * 6.2831853f;
              // 轨道面由昼夜夹角倾斜计算一致 (完美圆)
              earth_orbit_pts.push_back(sunPos + Vec3(cosf(a)*(-sun_dist), 
                                                      sinf(a)*(-sun_dist)*0.196f, 
                                                      sinf(a)*(-sun_dist)*0.980f));
          }
          // 轨道线宽度，保持适当厚度
          float orbit_w = sun_radius * 0.01f * fmaxf(1.0f, fminf(cam_zoom_pan * 0.1f, 500.0f));
          r3d->drawRibbon(earth_orbit_pts, orbit_w, 0.2f, 0.6f, 1.0f, 0.25f);
      }

      // 地球
      Mat4 earthModel = Mat4::scale(Vec3(earth_r, earth_r, earth_r));
      r3d->drawEarth(earthMesh, earthModel);

      // ===== 大气层散射壳 =====
      r3d->drawAtmosphere(earthMesh, earth_r * 1.025f);

      // ===== 太阳与镜头光晕 (所有模式可见) =====
      r3d->drawSunAndFlare(sunPos, Vec3(0.0f, 0.0f, 0.0f), earth_r, ww, wh);

      // ===== 历史轨迹线 (实际走过的路径) =====
      {
        if (traj_history.empty() || (rocketPos - traj_history.back()).length() > earth_r * 0.002f) {
           traj_history.push_back(rocketPos);
           if (traj_history.size() > 800) {
             traj_history.erase(traj_history.begin());
           }
        }
        
        // 渲染历史轨迹 (更亮实线: 黄绿色), 增加基于相机拉远的线宽补偿 (仅在 Panorama 显示)
        if (cam_mode_3d == 2 && traj_history.size() >= 2) {
           float hist_w = earth_r * 0.003f * fmaxf(1.0f, cam_zoom_pan * 0.5f);
           r3d->drawRibbon(traj_history, hist_w, 0.4f, 1.0f, 0.3f, 0.8f);
        }
      }

      // ===== 火箭自身高亮标注 (方便在远景找到) =====
      if (cam_mode_3d == 2) {
        float marker_size = fminf(earth_r * 0.1f, earth_r * 0.02f * fmaxf(1.0f, cam_zoom_pan * 0.8f));
        r3d->drawBillboard(rocketPos, marker_size, 0.2f, 1.0f, 0.4f, 0.9f);
      }

      // ===== 轨道预测线 (开普勒轨道) =====
      if (cam_mode_3d == 2) {
        // 用2D速度转3D
        float vx3d = (float)baba1.vx * ws;
        float vy3d = (float)baba1.vy * ws;
        Vec3 vel3d(vx3d, vy3d, 0.0f);
        // 引力参数 (由于距离乘了ws，需要给g配平)
        float mu = 9.81f * earth_r * earth_r * ws; 

        float r_len = rocketPos.length();
        float v_len = vel3d.length();

        if (v_len > 0.001f && r_len > earth_r * 0.5f) {
          float energy = 0.5f * v_len * v_len - mu / r_len;
          Vec3 h_vec = rocketPos.cross(vel3d);
          float h = h_vec.length();
          float a = -mu / (2.0f * energy);
          Vec3 e_vec = vel3d.cross(h_vec) / mu - rocketPos / r_len;
          float ecc = e_vec.length();

          if (ecc < 0.999f) {
            // --- 椭圆轨道 (a > 0) ---
            float b = a * sqrtf(fmaxf(0.0f, 1.0f - ecc * ecc));
            Vec3 e_dir = ecc > 1e-6f ? e_vec / ecc : Vec3(1.0f, 0.0f, 0.0f);
            Vec3 perp_dir(-e_dir.y, e_dir.x, 0.0f);

            float periapsis = a * (1.0f - ecc);
            float apoapsis = a * (1.0f + ecc);
            bool will_reenter = periapsis < earth_r;

            Vec3 center_off = e_dir * (-a * ecc);

            // 生成预测轨迹点集
            std::vector<Vec3> orbit_points;
            int orbit_segs = 120;
            for (int i = 0; i <= orbit_segs; i++) {
              float ang = (float)i / orbit_segs * 6.2831853f;
              Vec3 pt = center_off + e_dir * (a * cosf(ang)) + perp_dir * (b * sinf(ang));
              if (pt.length() < earth_r * 0.98f) continue;
              orbit_points.push_back(pt);
            }
            
            // 渲染预测轨迹
            float pred_w = earth_r * 0.0025f * fmaxf(1.0f, cam_zoom_pan * 0.5f);
            if (will_reenter) {
              r3d->drawRibbon(orbit_points, pred_w, 1.0f, 0.4f, 0.1f, 0.6f);
            }

            float apsis_size = fminf(earth_r * 0.12f, earth_r * 0.025f * fmaxf(1.0f, cam_zoom_pan * 0.8f));
            
            // 远地点标记
            Vec3 apoPos = e_dir * (-apoapsis);
            if (apoapsis > earth_r * 1.002f) {
              r3d->drawBillboard(apoPos, apsis_size, 0.2f, 0.4f, 1.0f, 0.9f);
            }

            // 近地点标记
            Vec3 periPos = e_dir * periapsis;
            if (periapsis > earth_r * 1.002f) {
              r3d->drawBillboard(periPos, apsis_size, 1.0f, 0.5f, 0.1f, 0.9f);
            }
          } else {
            // --- 双曲线/抛物线逃逸轨道 (ecc >= 1.0) ---
            // 半长轴 a 为负值
            float a_hyp = fabs(a); // 实际 a 是负的，我们取绝对值方便计算
            float b_hyp = a_hyp * sqrtf(fmaxf(0.0f, ecc * ecc - 1.0f));
            Vec3 e_dir = e_vec / ecc;
            Vec3 perp_dir(-e_dir.y, e_dir.x, 0.0f);
            
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
               Vec3 pt = center_off - e_dir * (a_hyp * coshf(t)) + perp_dir * (b_hyp * sinhf(t));
               
               if (pt.length() < earth_r * 0.98f) continue;
               
               // 只保留未来要飞的，或者火箭当前位置附近的点(基于点乘判断行进方向)
               if (escape_points.empty() || (pt - escape_points.back()).length() > earth_r * 0.05f) {
                  escape_points.push_back(pt);
               }
            }

            // 逃逸轨道的 Ribbon (紫色代表逃逸)
            float pred_w = earth_r * 0.0025f * fmaxf(1.0f, cam_zoom_pan * 0.5f);
            r3d->drawRibbon(escape_points, pred_w, 0.8f, 0.3f, 1.0f, 0.7f);

            // 近地点标记
            float apsis_size = fminf(earth_r * 0.12f, earth_r * 0.025f * fmaxf(1.0f, cam_zoom_pan * 0.8f));
            Vec3 periPos = center_off - e_dir * a_hyp; // 顶点就是近地点
            if (periapsis > earth_r * 1.002f) {
              r3d->drawBillboard(periPos, apsis_size, 1.0f, 0.5f, 0.1f, 0.9f);
            }
          }
        }
      }

      // ===== 火箭朝向四元数 =====
      Vec3 defaultUp(0.0f, 1.0f, 0.0f);
      Vec3 rotAxis = defaultUp.cross(rocketDir);
      float rotAngle = acosf(fminf(fmaxf(defaultUp.dot(rocketDir), -1.0f), 1.0f));
      Quat rocketQuat;
      if (rotAxis.length() > 1e-6f)
        rocketQuat = Quat::fromAxisAngle(rotAxis.normalized(), rotAngle);

      // ===== 火箭涂装 (Builder颜色) =====
      // 机体 (燃料箱颜色)
      Mat4 bodyModel = Mat4::TRS(rocketPos, rocketQuat,
                                  Vec3(rw_3d, rh * 0.7f, rw_3d));
      r3d->drawMesh(rocketBody, bodyModel,
                     sel_tank.r, sel_tank.g, sel_tank.b, 1.0f, 0.15f);

      // 鼻锥 (鼻锥颜色)
      Vec3 nosePos = rocketPos + rocketDir * (rh * 0.35f);
      Mat4 noseModel = Mat4::TRS(nosePos, rocketQuat,
                                  Vec3(rw_3d, rh * 0.3f, rw_3d));
      r3d->drawMesh(rocketNose, noseModel,
                     sel_nose.r, sel_nose.g, sel_nose.b, 1.0f, 0.15f);

      // 引擎喷管 (引擎颜色)
      Vec3 engPos = rocketPos - rocketDir * (rh * 0.35f);
      Quat flipQuat = Quat::fromAxisAngle(
        rotAxis.length() > 1e-6f ? rotAxis.normalized() : Vec3(0.0f, 0.0f, 1.0f),
        rotAngle + 3.14159f);
      Mat4 engModel = Mat4::TRS(engPos, flipQuat,
                                 Vec3(rw_3d * 1.2f, rh * 0.15f, rw_3d * 1.2f));
      r3d->drawMesh(rocketNose, engModel,
                     sel_eng.r, sel_eng.g, sel_eng.b, 1.0f, 0.2f);

      // ===== 3D 火焰/尾焰粒子 =====
      if (baba1.throttle > 0.01) {
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

    if (!view_3d) {
    // =================【太空背景星空】=================
    if (baba1.getAltitude() > 5000.0) {
      float star_alpha =
          (float)min(1.0, (baba1.getAltitude() - 5000.0) / 45000.0);
      for (int i = 0; i < 200; i++) {
        float sx = hash11(i * 7919) * 2.0f - 1.0f; // -1 ~ 1
        float sy = hash11(i * 6271) * 2.0f - 1.0f;
        float brightness = 0.5f + hash11(i * 3571) * 0.5f;
        float star_size = 0.002f + hash11(i * 4219) * 0.003f;
        renderer->addRect(sx, sy, star_size, star_size, brightness, brightness,
                          brightness * 0.9f, star_alpha * brightness);
      }
    }


    // 1. 画地球 (加上 y_offset)
    renderer->addEarthWithContinents(toScreenX(0, 0),
                                     toScreenY(0, 0) + y_offset,
                                     EARTH_RADIUS * scale, cam_angle);
    renderer->addAtmosphereGlow(toScreenX(0, 0), toScreenY(0, 0) + y_offset,
                                EARTH_RADIUS * scale, cam_angle);
    // 轨道线延迟到再入特效之后绘制（见下方）

    // 3. 画发射台 (放在地表，加上 y_offset)
    renderer->addRotatedRect(
        toScreenX(0, EARTH_RADIUS),
        toScreenY(0, EARTH_RADIUS) + y_offset - 25.0f * scale, 100.0f * scale,
        50.0f * scale, (float)-cam_angle, 0.5f, 0.5f, 0.5f);

    // =================【新增功能：无限程序化生成植被系统】=================
    // 高度限制：只在 20km (20000米) 以下渲染树木，内存占用 O(1)！
    if (baba1.getAltitude() < 20000.0) {
      float baseTrunkW = 5.0f;
      float baseTrunkH = 15.0f;
      float baseLeafW = 18.0f;
      float baseLeafH = 28.0f;

      // 1. 获取火箭当前的绝对极角 (0 ~ 2PI)
      double rocket_theta = atan2(baba1.py, baba1.px);
      if (rocket_theta < 0)
        rocket_theta += 2.0 * PI;

      // 2. 定义视距：只计算火箭前后各 0.03 弧度 (约 200 公里范围)
      double view_range = 0.03;

      // 3. 将整个地球划分为无数个 15 米宽的虚拟网格
      double grid_step = 15.0 / EARTH_RADIUS;
      int total_grids = (int)ceil((2.0 * PI) / grid_step); // 地球一圈总网格数

      // 计算视野内覆盖了哪些网格索引
      int start_idx = (int)floor((rocket_theta - view_range) / grid_step);
      int end_idx = (int)ceil((rocket_theta + view_range) / grid_step);

      for (int i = start_idx; i <= end_idx; ++i) {
        // 解决绕地球一圈后的索引越界问题
        int wrapped_i = (i % total_grids + total_grids) % total_grids;

        // 【核心魔法】：用网格索引算命！
        float random_presence = hash11(wrapped_i * 12345);
        // 只有 35% 的概率这个槽位会有一棵树，形成错落有致的森林
        if (random_presence > 0.35f)
          continue;

        // 通过索引反推这棵树在地球上的真实物理极角
        double tree_angle = wrapped_i * grid_step;

        // 【海洋剔除】：同步大陆级地球渲染逻辑
        float tree_pos = (float)(tree_angle / (2.0 * PI)); // 0~1
        bool tree_on_land = false;
        for (int c = 0; c < 8; c++) {
          float cc = (float)c / 8.0f + hash11(c * 9973) * 0.08f;
          if (cc > 1.0f) cc -= 1.0f;
          float chw = 0.03f + hash11(c * 7333) * 0.07f;
          float d = abs(tree_pos - cc);
          if (d > 0.5f) d = 1.0f - d;
          if (d < chw) { tree_on_land = true; break; }
        }
        if (!tree_on_land)
          continue; // 海洋区域，跳过！

        // 发射台避让区 (PI/2 附近 80 米范围)
        if (abs(tree_angle - PI / 2.0) < 0.000012)
          continue;

        // 计算随机大小 (0.5 ~ 1.5 倍)
        float random_size = 0.5f + hash11(wrapped_i * 54321) * 1.0f;

        // 计算坐标
        double wx = EARTH_RADIUS * cos(tree_angle);
        double wy = EARTH_RADIUS * sin(tree_angle);
        float screenX = toScreenX(wx, wy);
        float screenY = toScreenY(wx, wy) + y_offset;

        float finalScale = (float)scale * random_size;
        float tW = baseTrunkW * finalScale;
        float tH = baseTrunkH * finalScale;
        float lW = baseLeafW * finalScale;
        float lH = baseLeafH * finalScale;

        // 计算法线角度：让树木向外生长，同时考虑视角的相对旋转
        double angle_diff =
            tree_angle -
            (rocket_theta > PI ? rocket_theta - 2 * PI : rocket_theta);
        if (angle_diff > PI)
          angle_diff -= 2 * PI;
        if (angle_diff < -PI)
          angle_diff += 2 * PI;
        float screen_normal_angle = (float)angle_diff;

        Vec2 trunk_offset = {0, tH / 2.0f};
        trunk_offset =
            rotateVec(trunk_offset.x, trunk_offset.y, screen_normal_angle);
        Vec2 leaf_offset = {0, tH + lH / 2.0f};
        leaf_offset =
            rotateVec(leaf_offset.x, leaf_offset.y, screen_normal_angle);

        renderer->addRotatedRect(screenX + trunk_offset.x,
                                 screenY + trunk_offset.y, tW, tH,
                                 screen_normal_angle, 0.4f, 0.25f, 0.1f);
        renderer->addRotatedTri(screenX + leaf_offset.x,
                                screenY + leaf_offset.y, lW, lH,
                                screen_normal_angle, 0.1f, 0.6f, 0.2f);
      }
    }
    // =================================================================
    // 绘制轨道线（可用 O 键切换）
    if (show_orbit)
      drawOrbit(renderer, baba1.px, baba1.py, baba1.vx, baba1.vy, scale, cx, cy,
                cam_angle);
    // ================= 特效 1: 弓形再入激波 + 流星尾 (Meteor Reentry) =================
    double speed = baba1.getVelocityMag();
    double alt = baba1.getAltitude();
    if (alt < 70000.0 && speed > 2000.0) {
      float intensity = (float)min(1.0, (speed - 2000.0) / 3000.0) *
                        (float)(1.0 - alt / 70000.0);
      double svx = baba1.vx * cos_c - baba1.vy * sin_c;
      double svy = baba1.vx * sin_c + baba1.vy * cos_c;
      double move_angle = atan2(svx, svy);
      float render_angle = (float)(move_angle + PI);

      // A. 弓形激波 — 真正的弧形，用多个小块排列成弧线
      float bow_base = h / 2.0f + w * 0.3f;
      for (int layer = 0; layer < 6; layer++) {
        float lf = (float)layer / 6.0f;
        float arc_radius = w * (1.5f + lf * 2.0f) * (0.8f + intensity * 0.5f);
        float dist = bow_base + layer * w * 0.15f;
        int arc_pts = 7 + layer * 2; // 弧线细分点数
        for (int a = 0; a < arc_pts; a++) {
          float af = (float)a / (float)(arc_pts - 1) - 0.5f; // -0.5 ~ 0.5
          // 沿弧线分布
          float arc_x = af * arc_radius * 2.0f;
          float arc_y = dist - af * af * arc_radius * 1.5f; // 弓形：中心最前，两端向后扫
          // 旋转到速度方向
          float rx = (float)(arc_x * cos(move_angle) + arc_y * sin(move_angle));
          float ry = (float)(-arc_x * sin(move_angle) + arc_y * cos(move_angle));
          // 抖动
          float jx = (hash11(layer * 2917 + a * 137 + frame * 3) - 0.5f) * w * 0.2f;
          float jy = (hash11(layer * 4111 + a * 211 + frame * 5) - 0.5f) * w * 0.2f;
          float bpx = cx + rx + jx;
          float bpy = cy + ry + jy;
          float block_size = w * (0.4f + lf * 0.3f) * (1.0f - abs(af) * 0.5f);
          float lr = 1.0f;
          float lg = 0.95f - lf * 0.5f - abs(af) * 0.3f;
          float lb = 0.7f - lf * 0.6f;
          float la = intensity * (0.9f - lf * 0.1f) * (1.0f - abs(af) * 0.6f);
          renderer->addRect(bpx, bpy, block_size, block_size * 0.7f,
                            lr, lg, lb, la);
        }
      }

      // B. 侧向扩散火焰翅膀 — 更长更宽
      float wing_len = w * (3.0f + intensity * 8.0f);
      float wing_w = h * 0.5f * intensity;
      float side_nx = (float)cos(move_angle);
      float side_ny = (float)-sin(move_angle);
      float tail_dx = -(float)sin(move_angle);
      float tail_dy = -(float)cos(move_angle);
      for (int side = -1; side <= 1; side += 2) {
        for (int f = 0; f < 5; f++) {
          float ff = (float)f / 5.0f;
          float jitter = (hash11(f * 3917 + side * 100 + frame * 7) - 0.5f) * w * 0.5f;
          float sx = cx + side_nx * (w * 0.5f + wing_len * ff) * side
                       + tail_dx * (wing_w * ff * 0.5f + jitter);
          float sy = cy + side_ny * (w * 0.5f + wing_len * ff) * side
                       + tail_dy * (wing_w * ff * 0.5f + jitter);
          float fw = w * 0.5f * (1.0f - ff * 0.4f);
          float fh = wing_w * (1.0f - ff * 0.5f);
          float fa_wing = intensity * (0.7f - ff * 0.15f);
          renderer->addRotatedRect(sx, sy, fw, fh, render_angle,
                                   1.0f, 0.8f - ff * 0.6f, 0.3f - ff * 0.3f,
                                   fa_wing);
        }
      }

      // C. 流星尾迹 — 最长 20 倍火箭高度！
      float tail_total_len = h * (8.0f + intensity * 12.0f);
      int tail_segments = 30;
      for (int t = 0; t < tail_segments; t++) {
        float tt = (float)(t + 1) / (float)tail_segments;
        float seg_dist = tail_total_len * tt;
        // 尾部随机晃动
        float jitter_x = (hash11(t * 3917 + frame) - 0.5f) * w * 1.5f * tt;
        float jitter_y = (hash11(t * 7121 + frame) - 0.5f) * w * 0.5f * tt;
        float tx = cx + tail_dx * seg_dist + side_nx * jitter_x + tail_dx * jitter_y;
        float ty = cy + tail_dy * seg_dist + side_ny * jitter_x + tail_dy * jitter_y;
        // 宽度从粗到细 (缓慢收窄)
        float seg_w = w * (1.5f - tt * tt * 1.0f);
        float seg_h = tail_total_len / tail_segments * 2.2f; // 加大重叠消除明暗相间
        // 颜色：前 70% 保持浓密白黄，最后才变暗红消散
        float fade = tt * tt * tt; // 立方衰减：前80%几乎不变，最后急速消退
        float cr = 1.0f;
        float cg = max(0.0f, 0.95f - fade * 1.2f);
        float cb = max(0.0f, 0.7f - fade * 1.5f);
        float ca = intensity * (0.9f - fade * 0.9f);
        if (ca > 0.01f) {
          renderer->addRotatedRect(tx, ty, seg_w, seg_h, render_angle,
                                   cr, cg, cb, ca);
        }
      }

      // D. 等离子粒子点缀
      for (int p = 0; p < 12; p++) {
        float rnd_x = (hash11(p * 3917 + frame) - 0.5f) * w * 3.0f;
        float rnd_y = -(hash11(p * 7121 + frame) * h * 1.5f);
        Vec2 poff = rotateVec(rnd_x, rnd_y, move_angle);
        float pa = intensity * (0.3f + hash11(p * 2131) * 0.3f);
        float ps = w * 0.15f * (0.5f + hash11(p * 4513) * 0.5f);
        renderer->addRect(cx + poff.x, cy + poff.y, ps, ps, 1.0f, 0.6f, 0.2f,
                          pa);
      }
    }
    // ================= 特效 2 & 3: 动态粒子尾焰 & 着陆烟尘 =================
    double thrust_kN = baba1.getThrust() / 1000.0;
    if (thrust_kN > 10.0) { // 有推力就显示
      float throttle_factor = (float)(baba1.throttle);
      float flameLen =
          (thrust_kN / 3000.0f) * h * 2.5f;
      // 真空中尾焰扩散（最大1.8倍）
      float vacuum_expand = 1.0f + (float)min(0.8, baba1.getAltitude() / 50000.0);
      float flameW = w * 0.9f * vacuum_expand;

      // 计算尾焰尖端是否触地
      double nozzle_up = atan2(baba1.py, baba1.px);
      double nozzle_dir = nozzle_up + baba1.angle + PI;
      double flame_tip_r_world = flameLen / scale; // 尾焰长度转世界坐标
      double tip_wx = baba1.px + cos(nozzle_dir) * (20.0 + flame_tip_r_world);
      double tip_wy = baba1.py + sin(nozzle_dir) * (20.0 + flame_tip_r_world);
      double tip_r = sqrt(tip_wx * tip_wx + tip_wy * tip_wy);
      bool flame_hits_ground = (tip_r < EARTH_RADIUS && baba1.getAltitude() < 300.0);

      // --- A. 动态火焰粒子群 ---
      int flame_particles = 15 + (int)(throttle_factor * 10);
      for (int p = 0; p < flame_particles; p++) {
        float pf = (float)p / (float)flame_particles;
        // 每个粒子沿尾焰轴线分布，带随机偏移
        float dist = flameLen * (0.1f + pf * 0.9f);
        float jitter_x = (hash11(p * 3917 + frame * 7) - 0.5f) * flameW * 1.2f;
        float jitter_y = (hash11(p * 7121 + frame * 13) - 0.5f) * flameLen * 0.15f;
        Vec2 fp_off = {jitter_x, -h / 2.0f - dist + jitter_y};
        fp_off = rotateVec(fp_off.x, fp_off.y, baba1.angle);
        // 大小从粗到细
        float ps = flameW * (1.0f - pf * 0.7f) * (0.6f + hash11(p * 2131 + frame * 3) * 0.4f);
        // 颜色从白黄到橙红
        float cr = 1.0f;
        float cg = max(0.0f, 0.9f - pf * 0.8f + (hash11(p * 4513 + frame * 5) - 0.5f) * 0.2f);
        float cb = max(0.0f, 0.5f - pf * 0.8f);
        float ca = throttle_factor * (0.8f - pf * 0.5f);
        // 地面偏转：火焰粒子不穿模，撞到地面向两边扩散
        float draw_x = cx + fp_off.x;
        float draw_y = cy + fp_off.y;
        if (flame_hits_ground) {
          // 检查粒子世界位置是否在地下
          double p_wx = baba1.px + (double)(fp_off.x) / scale;
          double p_wy = baba1.py + (double)(fp_off.y) / scale;
          double p_r = sqrt(p_wx * p_wx + p_wy * p_wy);
          if (p_r < EARTH_RADIUS) {
            // 将粒子推到地表并沿切向偏移
            double pnx = p_wx / p_r, pny = p_wy / p_r;
            p_wx = pnx * EARTH_RADIUS;
            p_wy = pny * EARTH_RADIUS;
            // 切向偏移（向两侧扩散）
            double ptx = -pny, pty = pnx;
            float side_spread = (hash11(p * 1171 + frame * 3) - 0.5f) * 2.0f;
            p_wx += ptx * side_spread * (double)(flameW / scale) * 1.5;
            p_wy += pty * side_spread * (double)(flameW / scale) * 1.5;
            draw_x = toScreenX(p_wx, p_wy);
            draw_y = toScreenY(p_wx, p_wy) + y_offset;
            // 粒子变扁，贴地扩散
            renderer->addRect(draw_x, draw_y, ps * 1.5f, ps * 0.3f,
                              cr, cg, cb, ca * 0.7f);
            continue;
          }
        }
        renderer->addRect(draw_x, draw_y, ps, ps * 0.6f,
                          cr, cg, cb, ca);
      }

      // --- A2. 火焰撞地时产生烟雾 ---
      if (flame_hits_ground && baba1.getAltitude() < 200.0) {
        float ground_factor = (float)(1.0 - baba1.getAltitude() / 200.0);
        double up_angle = atan2(baba1.py, baba1.px);
        double impact_wx = EARTH_RADIUS * cos(up_angle);
        double impact_wy = EARTH_RADIUS * sin(up_angle);
        double inx = cos(up_angle), iny = sin(up_angle);
        double itx = -iny, ity = inx;
        int num_impact_smoke = (int)(2.0f * ground_factor * (float)baba1.throttle) + 1;
        for (int k = 0; k < num_impact_smoke; k++) {
          SmokeParticle& sp = baba1.smoke[baba1.smoke_idx % Explorer::MAX_SMOKE];
          float dir = (hash11(baba1.smoke_idx * 2917 + k) - 0.5f) * 2.0f;
          float spd = 10.0f + hash11(baba1.smoke_idx * 4111 + k) * 30.0f * ground_factor;
          sp.wx = impact_wx + itx * dir * 10.0 + inx * 3.0;
          sp.wy = impact_wy + ity * dir * 10.0 + iny * 3.0;
          sp.vwx = itx * dir * spd + inx * spd * 0.3;
          sp.vwy = ity * dir * spd + iny * spd * 0.3;
          sp.alpha = 0.25f * ground_factor;
          sp.size = 6.0f + hash11(baba1.smoke_idx * 7331 + k) * 8.0f;
          sp.life = 1.0f;
          sp.active = true;
          baba1.smoke_idx++;
        }
      }

      // --- B. 动态马赫环 (Shock Diamonds) ---
      if (throttle_factor > 0.1f) {
        int num_diamonds = max(1, (int)(throttle_factor * 5));
        float diamond_spacing = flameLen / (num_diamonds + 1);
        for (int i = 1; i <= num_diamonds; i++) {
          // 每帧随机抖动 + 闪烁
          float d_jx = (hash11(i * 4937 + frame * 11) - 0.5f) * flameW * 0.3f;
          float d_jy = (hash11(i * 6173 + frame * 17) - 0.5f) * diamond_spacing * 0.1f;
          float d_flicker = 0.7f + hash11(i * 8291 + frame * 7) * 0.3f;
          float d_size = flameW * (0.4f + hash11(i * 3571 + frame * 3) * 0.2f);
          Vec2 diamond_pos = {d_jx, -h / 2.0f - diamond_spacing * i + d_jy};
          diamond_pos = rotateVec(diamond_pos.x, diamond_pos.y, baba1.angle);
          renderer->addRotatedRect(cx + diamond_pos.x, cy + diamond_pos.y,
                                   d_size, d_size,
                                   (float)baba1.angle + PI / 4.0f, 1.0f, 0.9f,
                                   0.7f, d_flicker * throttle_factor);
        }
      }
    }
    // ====================================================================
    // ===== 精细火箭箔体 (无缝拼接版) =====
    float fa = (float)baba1.angle;
    // 布局 (从底到顶，总高 h)：
    //   发动机喷口: 倒三角，顶边在 -h/2
    //   下段燃料箱: 高度 0.5h， 中心在 -h*0.10
    //   级间段:     高度 0.08h，中心在 h*0.15
    //   上段设备舱: 高度 0.30h，中心在 h*0.34
    //   鼻锥:       顶边在 h/2

    // 下段（燃料箱）白色 — 高0.50h, 中心 y = -0.10h (底边=-0.35h, 顶边=0.15h 不重叠)
    float lower_h = h * 0.52f;
    Vec2 lower_off = {0, -h * 0.08f};
    lower_off = rotateVec(lower_off.x, lower_off.y, fa);
    renderer->addRotatedRect(cx + lower_off.x, cy + lower_off.y, w, lower_h,
                             fa, 0.92f, 0.92f, 0.92f);
    // 级间段（深灰）— 高0.08h, 中心 y = 0.18h (底=0.14h, 顶=0.22h)
    float inter_h = h * 0.10f;
    Vec2 inter_off = {0, h * 0.18f};
    inter_off = rotateVec(inter_off.x, inter_off.y, fa);
    renderer->addRotatedRect(cx + inter_off.x, cy + inter_off.y, w * 1.05f,
                             inter_h, fa, 0.3f, 0.3f, 0.3f);
    // 上段（设备舱）浅灰 — 高0.28h, 中心 y = 0.34h (底=0.20h, 顶=0.48h)
    float upper_h = h * 0.30f;
    Vec2 upper_off = {0, h * 0.34f};
    upper_off = rotateVec(upper_off.x, upper_off.y, fa);
    renderer->addRotatedRect(cx + upper_off.x, cy + upper_off.y, w * 0.95f,
                             upper_h, fa, 0.85f, 0.85f, 0.88f);
    // 黑色涂装带
    Vec2 band_off = {0, h * 0.18f};
    band_off = rotateVec(band_off.x, band_off.y, fa);
    renderer->addRotatedRect(cx + band_off.x, cy + band_off.y, w * 1.02f,
                             h * 0.06f, fa, 0.1f, 0.1f, 0.1f);
    // 栏格翼 x2
    Vec2 gf_l = {-w * 0.6f, h * 0.2f};
    gf_l = rotateVec(gf_l.x, gf_l.y, fa);
    Vec2 gf_r = {w * 0.6f, h * 0.2f};
    gf_r = rotateVec(gf_r.x, gf_r.y, fa);
    renderer->addRotatedRect(cx + gf_l.x, cy + gf_l.y, w * 0.25f, h * 0.12f, fa,
                             0.25f, 0.25f, 0.25f);
    renderer->addRotatedRect(cx + gf_r.x, cy + gf_r.y, w * 0.25f, h * 0.12f, fa,
                             0.25f, 0.25f, 0.25f);
    // 发动机裙部 — 倒三角朝下，顶边紧贴燃料箱底部 (-0.34h)
    float eng_tri_h = w * 0.35f;
    Vec2 eng_off = {0, -h * 0.34f - eng_tri_h / 2.0f};
    eng_off = rotateVec(eng_off.x, eng_off.y, fa);
    renderer->addRotatedTri(cx + eng_off.x, cy + eng_off.y, w * 1.3f, eng_tri_h,
                            fa + (float)PI, 0.35f, 0.35f, 0.38f);
    // 鼻锥（深灰）— 底边紧贴上段顶部 (0.49h)
    float nose_h = w * 0.9f;
    Vec2 nose_offset = {0, h * 0.49f + nose_h / 2.0f};
    nose_offset = rotateVec(nose_offset.x, nose_offset.y, fa);
    renderer->addRotatedTri(cx + nose_offset.x, cy + nose_offset.y, w, nose_h, fa,
                            0.35f, 0.35f, 0.4f);

    // =================【新增特效：RCS 侧推喷射火焰】=================
    // 条件：太空中 (>80km) 且有力矩指令
    if (baba1.getAltitude() > 80000.0 && abs(baba1.torque_cmd) > 1000.0) {
      float rcs_len = w * 0.6f;
      float rcs_w = w * 0.3f;
      float rcs_alpha = (float)min(1.0, abs(baba1.torque_cmd) / 30000.0) * 0.8f;

      // 上部RCS位置 (火箭顶部侧面)
      Vec2 rcs_top_offset = {(baba1.torque_cmd > 0 ? -w / 2.0f : w / 2.0f),
                             h * 0.35f};
      rcs_top_offset =
          rotateVec(rcs_top_offset.x, rcs_top_offset.y, baba1.angle);
      float rcs_angle_top =
          (float)baba1.angle +
          (baba1.torque_cmd > 0 ? (float)PI / 2.0f : -(float)PI / 2.0f);
      renderer->addRotatedTri(cx + rcs_top_offset.x, cy + rcs_top_offset.y,
                              rcs_w, rcs_len, rcs_angle_top, 1.0f, 0.9f, 0.7f,
                              rcs_alpha);

      // 底部RCS位置 (火箭底部反向)
      Vec2 rcs_bot_offset = {(baba1.torque_cmd > 0 ? w / 2.0f : -w / 2.0f),
                             -h * 0.35f};
      rcs_bot_offset =
          rotateVec(rcs_bot_offset.x, rcs_bot_offset.y, baba1.angle);
      float rcs_angle_bot =
          (float)baba1.angle +
          (baba1.torque_cmd > 0 ? -(float)PI / 2.0f : (float)PI / 2.0f);
      renderer->addRotatedTri(cx + rcs_bot_offset.x, cy + rcs_bot_offset.y,
                              rcs_w, rcs_len, rcs_angle_bot, 1.0f, 0.9f, 0.7f,
                              rcs_alpha);
    }

    // =================【新增特效：着陆腿展开动画】=================
    if (baba1.status == Explorer::DESCEND && baba1.getAltitude() < 5000.0) {
      // 展开进度 0~1 动画
      baba1.leg_deploy_progress = min(1.0, baba1.leg_deploy_progress + 0.02);
    }
    if (baba1.leg_deploy_progress > 0.01) {
      float leg_angle_max = (float)(PI / 4.0); // 最大展开 45度
      float leg_current = leg_angle_max * (float)baba1.leg_deploy_progress;
      float leg_len = h * 0.35f;
      float leg_w_px = w * 0.12f;

      // 左腿
      Vec2 leg_base_L = {-w * 0.3f, -h / 2.0f};
      leg_base_L = rotateVec(leg_base_L.x, leg_base_L.y, baba1.angle);
      float leg_angle_L = (float)baba1.angle - leg_current;
      Vec2 leg_ext_L = {0, -leg_len / 2.0f};
      leg_ext_L = rotateVec(leg_ext_L.x, leg_ext_L.y, (double)leg_angle_L);
      renderer->addRotatedRect(cx + leg_base_L.x + leg_ext_L.x,
                               cy + leg_base_L.y + leg_ext_L.y, leg_w_px,
                               leg_len, leg_angle_L, 0.4f, 0.4f, 0.4f);

      // 右腿
      Vec2 leg_base_R = {w * 0.3f, -h / 2.0f};
      leg_base_R = rotateVec(leg_base_R.x, leg_base_R.y, baba1.angle);
      float leg_angle_R = (float)baba1.angle + leg_current;
      Vec2 leg_ext_R = {0, -leg_len / 2.0f};
      leg_ext_R = rotateVec(leg_ext_R.x, leg_ext_R.y, (double)leg_angle_R);
      renderer->addRotatedRect(cx + leg_base_R.x + leg_ext_R.x,
                               cy + leg_base_R.y + leg_ext_R.y, leg_w_px,
                               leg_len, leg_angle_R, 0.4f, 0.4f, 0.4f);
    }

    // =================【新增特效：着陆烟尘】=================
    // 只在下降状态才显示，防止起飞时出现横杠
    if (baba1.status == Explorer::DESCEND && baba1.getThrust() > 0 &&
        baba1.getAltitude() < 200.0 && baba1.getAltitude() > 0.5) {
      float dust_intensity =
          (float)(1.0 - baba1.getAltitude() / 200.0) * (float)(baba1.throttle);
      float dust_spread = w * 3.0f * dust_intensity;
      float dust_h = h * 0.15f;

      // 左右两团烟尘
      Vec2 dust_base = {0, -h / 2.0f - dust_h};
      dust_base = rotateVec(dust_base.x, dust_base.y, baba1.angle);

      renderer->addRotatedRect(cx + dust_base.x - dust_spread, cy + dust_base.y,
                               dust_spread * 1.5f, dust_h, (float)baba1.angle,
                               0.6f, 0.5f, 0.4f, 0.4f * dust_intensity);
      renderer->addRotatedRect(cx + dust_base.x + dust_spread, cy + dust_base.y,
                               dust_spread * 1.5f, dust_h, (float)baba1.angle,
                               0.6f, 0.5f, 0.4f, 0.4f * dust_intensity);
      // 中心热浪
      renderer->addRotatedRect(
        cx + dust_base.x, cy + dust_base.y - dust_h * 0.5f,
          dust_spread * 0.8f, dust_h * 2.0f, (float)baba1.angle, 0.8f, 0.6f,
          0.3f, 0.2f * dust_intensity);
    }

    // =================【新增特效：世界坐标烟雾粒子】=================
    for (int i = 0; i < Explorer::MAX_SMOKE; i++) {
      if (!baba1.smoke[i].active) continue;
      float sx = toScreenX(baba1.smoke[i].wx, baba1.smoke[i].wy);
      float sy = toScreenY(baba1.smoke[i].wx, baba1.smoke[i].wy) + y_offset;
      float ss = (float)(baba1.smoke[i].size * scale);
      if (ss < 0.001f) continue;
      // 随机旋转角 — 让粒子不是正方形
      float rot = hash11(i * 5431) * (float)PI;
      // 新粒子更白，老粒子变灰（模拟真实火箭烟雾）
      float smoke_age = 1.0f - baba1.smoke[i].life;
      float sr = 0.95f - smoke_age * 0.25f;
      float sg = 0.93f - smoke_age * 0.28f;
      float sb = 0.90f - smoke_age * 0.30f;
      renderer->addRotatedRect(sx, sy, ss, ss * (0.6f + hash11(i * 3217) * 0.4f),
                               rot, sr, sg, sb, baba1.smoke[i].alpha);
    }

    } // end if (!view_3d)

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

    // --- 1. 速度计 (Velocity Gauge) ---
    double current_vel = baba1.getVelocityMag();
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
    double current_alt = baba1.getAltitude();
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
    int vvel = (int)baba1.getVerticalVel();
    float vr = vvel < 0 ? 1.0f : 0.3f;
    float vg = vvel >= 0 ? 1.0f : 0.3f;
    renderer->drawNumber(0.83f, 0.4f, vvel, num_size * 0.9f, vr, vg, 0.3f,
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
    
    // 提交最后的 2D 渲染(如HUD)
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