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
               double scale, float cx, float cy, double cam_angle, double cam_rocket_r,
               bool is_dashed = false, float thickness = 0.005f) {
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
      float dx = screen_x - prev_x, dy = screen_y - prev_y;
      float len = std::sqrt(dx * dx + dy * dy);
      
      bool skip_segment = false;
      if (is_dashed) {
          static float dash_accum = 0;
          dash_accum += len;
          if (std::fmod(dash_accum, 0.04f) > 0.02f) skip_segment = true;
      }

      if (len > 0 && len < 0.5f && !skip_segment) { // 避免突变的长线
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

// Predict state (pos, vel) at a future time T relative to current state
// px, py, vx, vy: Current relative state to body
// mu: Gravitational parameter
// dt: Time into future (seconds)
// Predict 3D state (pos, vel) at a future time T relative to current state
inline void get3DStateAtTime(double px, double py, double pz, double vx, double vy, double vz, double mu, double dt,
                        double& out_px, double& out_py, double& out_pz, double& out_vx, double& out_vy, double& out_vz) {
    Vec3 r_vec((float)px, (float)py, (float)pz);
    Vec3 v_vec((float)vx, (float)vy, (float)vz);
    double r_mag = r_vec.length();
    double v_sq = v_vec.lengthSq();
    Vec3 h_vec = r_vec.cross(v_vec);
    double h_mag = h_vec.length();
    
    if (h_mag < 1e-3 || r_mag < 1.0) { 
        out_px = px; out_py = py; out_pz = pz;
        out_vx = vx; out_vy = vy; out_vz = vz;
        return;
    }

    double energy = 0.5 * v_sq - mu / r_mag;
    double a = -mu / (2.0 * energy);
    
    Vec3 e_vec = v_vec.cross(h_vec) / (float)mu - r_vec / (float)r_mag;
    double e = e_vec.length();

    if (e >= 1.0) { // Hyperbolic (Return linear approximation)
        out_px = px + vx * dt; out_py = py + vy * dt; out_pz = pz + vz * dt;
        out_vx = vx; out_vy = vy; out_vz = vz;
        return;
    }

    double b = a * std::sqrt(std::max(0.0, 1.0 - e*e));
    Vec3 e_dir, n_dir, p_dir;
    n_dir = h_vec.normalized();

    if (e > 1e-7) {
        e_dir = e_vec.normalized();
    } else {
        // For circular orbits, pick any arbitrary vector in the orbital plane
        if (std::abs(n_dir.x) < 0.9f) e_dir = n_dir.cross(Vec3(1, 0, 0)).normalized();
        else e_dir = n_dir.cross(Vec3(0, 1, 0)).normalized();
    }
    p_dir = n_dir.cross(e_dir).normalized(); 
    
    // Decompose current position into this frame to find E0
    double r_dot_e = r_vec.dot(e_dir);
    double r_dot_p = r_vec.dot(p_dir);
    double cos_E = (r_dot_e / a) + e;
    double sin_E = r_dot_p / b;
    
    double E0 = std::atan2(sin_E, cos_E);
    double M0 = E0 - e * std::sin(E0);
    double period = 2.0 * PI * std::sqrt(a * a * a / mu);
    double M_target = M0 + (2.0 * PI * dt / period);
    
    double E = M_target;
    for (int i = 0; i < 10; i++) {
        E = E - (E - e * std::sin(E) - M_target) / (1.0 - e * std::cos(E));
    }

    double x_p = a * (std::cos(E) - e);
    double y_p = b * std::sin(E);
    Vec3 r_new = e_dir * (float)x_p + p_dir * (float)y_p;
    out_px = r_new.x; out_py = r_new.y; out_pz = r_new.z;

    double E_dot = std::sqrt(mu / (a * a * a)) / (1.0 - e * std::cos(E));
    double vx_p = -a * std::sin(E) * E_dot;
    double vy_p = b * std::cos(E) * E_dot;
    Vec3 v_new = e_dir * (float)vx_p + p_dir * (float)vy_p;
    out_vx = v_new.x; out_vy = v_new.y; out_vz = v_new.z;
}

inline void getStateAtTime(double px, double py, double vx, double vy, double mu, double dt,
                    double& out_px, double& out_py, double& out_vx, double& out_vy) {
    double opz, ovz;
    get3DStateAtTime(px, py, 0, vx, vy, 0, mu, dt, out_px, out_py, opz, out_vx, out_vy, ovz);
}
