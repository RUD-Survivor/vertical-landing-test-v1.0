#pragma once
// ==========================================================
// math3d.h — Header-only 3D math library
// Vec3, Quat (quaternion), Mat4
// Zero external dependencies, MSVC compatible
// ==========================================================

#include <cmath>
#include <algorithm>

// ==========================================================
// Vec3
// ==========================================================
struct Vec3 {
  float x, y, z;

  Vec3() : x(0), y(0), z(0) {}
  Vec3(float x, float y, float z) : x(x), y(y), z(z) {}

  Vec3 operator+(const Vec3& b) const { return Vec3(x + b.x, y + b.y, z + b.z); }
  Vec3 operator-(const Vec3& b) const { return Vec3(x - b.x, y - b.y, z - b.z); }
  Vec3 operator*(float s) const { return Vec3(x * s, y * s, z * s); }
  Vec3 operator/(float s) const { float inv = 1.0f / s; return Vec3(x * inv, y * inv, z * inv); }
  Vec3 operator-() const { return Vec3(-x, -y, -z); }
  Vec3& operator+=(const Vec3& b) { x += b.x; y += b.y; z += b.z; return *this; }
  Vec3& operator-=(const Vec3& b) { x -= b.x; y -= b.y; z -= b.z; return *this; }
  Vec3& operator*=(float s) { x *= s; y *= s; z *= s; return *this; }

  float dot(const Vec3& b) const { return x * b.x + y * b.y + z * b.z; }
  Vec3 cross(const Vec3& b) const {
    return Vec3(y * b.z - z * b.y, z * b.x - x * b.z, x * b.y - y * b.x);
  }

  float length() const { return sqrtf(x * x + y * y + z * z); }
  float lengthSq() const { return x * x + y * y + z * z; }

  Vec3 normalized() const {
    float len = length();
    if (len < 1e-12f) return Vec3(0.0f, 0.0f, 0.0f);
    return *this / len;
  }

  static Vec3 lerp(const Vec3& a, const Vec3& b, float t) {
    return a + (b - a) * t;
  }
};

inline Vec3 operator*(float s, const Vec3& v) { return v * s; }

// ==========================================================
// Vec4
// ==========================================================
struct Vec4 {
  float x, y, z, w;
  Vec4() : x(0), y(0), z(0), w(0) {}
  Vec4(float x, float y, float z, float w) : x(x), y(y), z(z), w(w) {}
};

// ==========================================================
// Quat — 四元数 (w, x, y, z) 表示旋转
// ==========================================================
struct Quat {
  float w, x, y, z;

  Quat() : w(1), x(0), y(0), z(0) {}
  Quat(float w, float x, float y, float z) : w(w), x(x), y(y), z(z) {}

  static Quat fromAxisAngle(const Vec3& axis, float angle) {
    float half = angle * 0.5f;
    float s = sinf(half);
    return Quat(cosf(half), axis.x * s, axis.y * s, axis.z * s);
  }

  static Quat fromEuler(float pitch, float yaw, float roll) {
    Quat qx = fromAxisAngle(Vec3(1.0f, 0.0f, 0.0f), pitch);
    Quat qy = fromAxisAngle(Vec3(0.0f, 1.0f, 0.0f), yaw);
    Quat qz = fromAxisAngle(Vec3(0.0f, 0.0f, 1.0f), roll);
    return qy * qx * qz;
  }

  Quat operator*(const Quat& q) const {
    return Quat(
      w * q.w - x * q.x - y * q.y - z * q.z,
      w * q.x + x * q.w + y * q.z - z * q.y,
      w * q.y - x * q.z + y * q.w + z * q.x,
      w * q.z + x * q.y - y * q.x + z * q.w
    );
  }

  Quat conjugate() const { return Quat(w, -x, -y, -z); }

  Quat normalized() const {
    float len = sqrtf(w * w + x * x + y * y + z * z);
    if (len < 1e-12f) return Quat(1, 0, 0, 0);
    float inv = 1.0f / len;
    return Quat(w * inv, x * inv, y * inv, z * inv);
  }

  Vec3 rotate(const Vec3& v) const {
    Vec3 u(x, y, z);
    float s = w;
    return 2.0f * u.dot(v) * u + (s * s - u.dot(u)) * v + 2.0f * s * u.cross(v);
  }

  Vec3 forward() const { return rotate(Vec3(0.0f, 1.0f, 0.0f)); }
  Vec3 right()   const { return rotate(Vec3(1.0f, 0.0f, 0.0f)); }
  Vec3 up()      const { return rotate(Vec3(0.0f, 0.0f, 1.0f)); }

  static Quat slerp(const Quat& a, const Quat& b, float t) {
    float d = a.w * b.w + a.x * b.x + a.y * b.y + a.z * b.z;
    Quat b2 = b;
    if (d < 0) {
      b2 = Quat(-b.w, -b.x, -b.y, -b.z);
      d = -d;
    }
    if (d > 0.9995f) {
      return Quat(
        a.w + t * (b2.w - a.w),
        a.x + t * (b2.x - a.x),
        a.y + t * (b2.y - a.y),
        a.z + t * (b2.z - a.z)
      ).normalized();
    }
    float theta = acosf(d);
    float sin_t = sinf(theta);
    float wa = sinf((1.0f - t) * theta) / sin_t;
    float wb = sinf(t * theta) / sin_t;
    return Quat(
      wa * a.w + wb * b2.w,
      wa * a.x + wb * b2.x,
      wa * a.y + wb * b2.y,
      wa * a.z + wb * b2.z
    );
  }

  void toMat4(float* m) const {
    float xx = x * x, yy = y * y, zz = z * z;
    float xy = x * y, xz = x * z, yz = y * z;
    float wx = w * x, wy = w * y, wz = w * z;

    m[0]  = 1 - 2 * (yy + zz); m[4]  = 2 * (xy - wz);     m[8]  = 2 * (xz + wy);     m[12] = 0;
    m[1]  = 2 * (xy + wz);     m[5]  = 1 - 2 * (xx + zz); m[9]  = 2 * (yz - wx);     m[13] = 0;
    m[2]  = 2 * (xz - wy);     m[6]  = 2 * (yz + wx);     m[10] = 1 - 2 * (xx + yy); m[14] = 0;
    m[3]  = 0;                  m[7]  = 0;                  m[11] = 0;                  m[15] = 1;
  }
};

// ==========================================================
// Mat4 — 4x4 矩阵 (列主序, OpenGL 兼容)
// ==========================================================
struct Mat4 {
  float m[16];

  Mat4() {
    for (int i = 0; i < 16; i++) m[i] = 0;
    m[0] = m[5] = m[10] = m[15] = 1;
  }

  Mat4 operator*(const Mat4& b) const {
    Mat4 r;
    for (int i = 0; i < 16; i++) r.m[i] = 0;
    for (int col = 0; col < 4; col++)
      for (int row = 0; row < 4; row++)
        for (int k = 0; k < 4; k++)
          r.m[col * 4 + row] += m[k * 4 + row] * b.m[col * 4 + k];
    return r;
  }

  Vec3 transformPoint(const Vec3& v) const {
    float rx = m[0]*v.x + m[4]*v.y + m[8]*v.z  + m[12];
    float ry = m[1]*v.x + m[5]*v.y + m[9]*v.z  + m[13];
    float rz = m[2]*v.x + m[6]*v.y + m[10]*v.z + m[14];
    float rw = m[3]*v.x + m[7]*v.y + m[11]*v.z + m[15];
    if (fabsf(rw) > 1e-12f) { rx /= rw; ry /= rw; rz /= rw; }
    return Vec3(rx, ry, rz);
  }

  Vec3 transformDir(const Vec3& v) const {
    return Vec3(
      m[0]*v.x + m[4]*v.y + m[8]*v.z,
      m[1]*v.x + m[5]*v.y + m[9]*v.z,
      m[2]*v.x + m[6]*v.y + m[10]*v.z
    );
  }

  static Mat4 perspective(float fovY_rad, float aspect, float zNear, float zFar) {
    Mat4 r;
    for (int i = 0; i < 16; i++) r.m[i] = 0;
    float f = 1.0f / tanf(fovY_rad * 0.5f);
    r.m[0]  = f / aspect;
    r.m[5]  = f;
    r.m[10] = (zFar + zNear) / (zNear - zFar);
    r.m[11] = -1.0f;
    r.m[14] = (2.0f * zFar * zNear) / (zNear - zFar);
    return r;
  }

  static Mat4 ortho(float left, float right, float bottom, float top,
                    float zNear, float zFar) {
    Mat4 r;
    for (int i = 0; i < 16; i++) r.m[i] = 0;
    r.m[0]  = 2.0f / (right - left);
    r.m[5]  = 2.0f / (top - bottom);
    r.m[10] = -2.0f / (zFar - zNear);
    r.m[12] = -(right + left) / (right - left);
    r.m[13] = -(top + bottom) / (top - bottom);
    r.m[14] = -(zFar + zNear) / (zFar - zNear);
    r.m[15] = 1.0f;
    return r;
  }

  static Mat4 lookAt(const Vec3& eye, const Vec3& target, const Vec3& worldUp) {
    Vec3 f = (target - eye).normalized();
    Vec3 r = f.cross(worldUp).normalized();
    Vec3 u = r.cross(f);

    Mat4 result;
    result.m[0] = r.x;  result.m[4] = r.y;  result.m[8]  = r.z;  result.m[12] = -r.dot(eye);
    result.m[1] = u.x;  result.m[5] = u.y;  result.m[9]  = u.z;  result.m[13] = -u.dot(eye);
    result.m[2] = -f.x; result.m[6] = -f.y; result.m[10] = -f.z; result.m[14] = f.dot(eye);
    result.m[3] = 0;    result.m[7] = 0;    result.m[11] = 0;    result.m[15] = 1;
    return result;
  }

  static Mat4 translate(const Vec3& t) {
    Mat4 r;
    r.m[12] = t.x; r.m[13] = t.y; r.m[14] = t.z;
    return r;
  }

  static Mat4 scale(const Vec3& s) {
    Mat4 r;
    r.m[0] = s.x; r.m[5] = s.y; r.m[10] = s.z;
    return r;
  }

  static Mat4 fromQuat(const Quat& q) {
    Mat4 r;
    q.toMat4(r.m);
    return r;
  }

  static Mat4 TRS(const Vec3& pos, const Quat& rot, const Vec3& scl) {
    Mat4 T = translate(pos);
    Mat4 R = fromQuat(rot);
    Mat4 S = scale(scl);
    return T * R * S;
  }
};
