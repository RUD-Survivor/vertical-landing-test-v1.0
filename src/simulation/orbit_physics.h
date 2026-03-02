#pragma once
#include "render/renderer_2d.h"
#include <cmath>
#include <algorithm>

#ifndef PI
#define PI 3.14159265358979323846f
#endif

// 根据当前状态，计算并画出预测轨道
inline void drawOrbit(Renderer *renderer, double px, double py, double vx, double vy,
               double body_x, double body_y, double body_vx, double body_vy, 
               double mu, double body_radius,
               double scale, float cx, float cy, double cam_angle, double cam_rocket_r) {
  double rel_px = px - body_x;
  double rel_py = py - body_y;               
  double r_mag = std::sqrt(rel_px * rel_px + rel_py * rel_py);

  // 【物理修复】：相对参考系的速度必须相减！否则开普勒轨道会变成花瓣形！
  double rel_vx = vx - body_vx;
  double rel_vy = vy - body_vy;

  double h = rel_px * rel_vy - rel_py * rel_vx; // 轨道角动量
  if (std::abs(h) < 1.0)
    return;

  // 计算偏心率矢量 (e)
  double ex = (rel_vy * h) / mu - rel_px / r_mag;
  double ey = (-rel_vx * h) / mu - rel_py / r_mag;
  double e_mag = std::sqrt(ex * ex + ey * ey);

  double p = (h * h) / mu;      // 半通径
  double omega = std::atan2(ey, ex); // 近地点幅角

  int segments = e_mag > 0.9 ? 8000 : 2000; // 轨道分段渲染
  float prev_x = 0, prev_y = 0;
  bool first = true;

  double sin_c = std::sin(cam_angle);
  double cos_c = std::cos(cam_angle);

  for (int i = 0; i <= segments; i++) {
    double nu = 2.0 * PI * i / segments; // 真近点角
    double denom = 1.0 + e_mag * std::cos(nu - omega);
    if (denom <= 0.05) {
      first = true;
      continue; // 忽略逃逸轨道的无限远端
    }

    double r_nu = p / denom;
    if (r_nu > body_radius * 50.0 && r_nu > 1000000000.0) {
      first = true;
      continue;
    }

    double x_orbit_rel = r_nu * std::cos(nu);
    double y_orbit_rel = r_nu * std::sin(nu);
    
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
      float len = std::sqrt(dx * dx + dy * dy);
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
