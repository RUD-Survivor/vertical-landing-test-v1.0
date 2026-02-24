#pragma once
// ==========================================================
// renderer3d.h — 3D Rendering Pipeline
// Requires: glad, math3d.h
// ==========================================================

#include "math3d.h"
// NOTE: glad must be included BEFORE this header in the main translation unit
#include <vector>

// ==========================================================
// 3D Vertex Format
// ==========================================================
struct Vertex3D {
  float px, py, pz;     // position
  float nx, ny, nz;     // normal
  float u, v;           // UV
  float r, g, b, a;     // color
};

// ==========================================================
// Mesh — 可重用的3D几何体
// ==========================================================
struct Mesh {
  GLuint vao = 0, vbo = 0, ebo = 0;
  int indexCount = 0;

  void upload(const std::vector<Vertex3D>& verts,
              const std::vector<unsigned int>& indices) {
    indexCount = (int)indices.size();
    if (!vao) {
      glGenVertexArrays(1, &vao);
      glGenBuffers(1, &vbo);
      glGenBuffers(1, &ebo);
    }
    glBindVertexArray(vao);

    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, verts.size() * sizeof(Vertex3D),
                 verts.data(), GL_STATIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(unsigned int),
                 indices.data(), GL_STATIC_DRAW);

    // pos
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex3D),
                          (void*)offsetof(Vertex3D, px));
    glEnableVertexAttribArray(0);
    // normal
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex3D),
                          (void*)offsetof(Vertex3D, nx));
    glEnableVertexAttribArray(1);
    // uv
    glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, sizeof(Vertex3D),
                          (void*)offsetof(Vertex3D, u));
    glEnableVertexAttribArray(2);
    // color
    glVertexAttribPointer(3, 4, GL_FLOAT, GL_FALSE, sizeof(Vertex3D),
                          (void*)offsetof(Vertex3D, r));
    glEnableVertexAttribArray(3);

    glBindVertexArray(0);
  }

  void draw() const {
    if (!vao || indexCount == 0) return;
    glBindVertexArray(vao);
    glDrawElements(GL_TRIANGLES, indexCount, GL_UNSIGNED_INT, 0);
    glBindVertexArray(0);
  }

  void destroy() {
    if (vao) glDeleteVertexArrays(1, &vao);
    if (vbo) glDeleteBuffers(1, &vbo);
    if (ebo) glDeleteBuffers(1, &ebo);
    vao = vbo = ebo = 0;
  }
};

// ==========================================================
// Mesh Generators
// ==========================================================
namespace MeshGen {

// 球体 (UV sphere)
inline Mesh sphere(int latSegs, int lonSegs, float radius) {
  std::vector<Vertex3D> verts;
  std::vector<unsigned int> indices;
  const float PI_f = 3.14159265358979f;

  for (int lat = 0; lat <= latSegs; lat++) {
    float theta = (float)lat / latSegs * PI_f;
    float sinT = sinf(theta), cosT = cosf(theta);
    for (int lon = 0; lon <= lonSegs; lon++) {
      float phi = (float)lon / lonSegs * 2.0f * PI_f;
      float sinP = sinf(phi), cosP = cosf(phi);

      float x = cosP * sinT;
      float y = cosT;
      float z = sinP * sinT;

      Vertex3D v;
      v.px = x * radius; v.py = y * radius; v.pz = z * radius;
      v.nx = x; v.ny = y; v.nz = z;
      v.u = (float)lon / lonSegs;
      v.v = (float)lat / latSegs;
      v.r = 1; v.g = 1; v.b = 1; v.a = 1;
      verts.push_back(v);
    }
  }
  for (int lat = 0; lat < latSegs; lat++) {
    for (int lon = 0; lon < lonSegs; lon++) {
      int a = lat * (lonSegs + 1) + lon;
      int b = a + lonSegs + 1;
      indices.push_back(a); indices.push_back(b); indices.push_back(a + 1);
      indices.push_back(b); indices.push_back(b + 1); indices.push_back(a + 1);
    }
  }
  Mesh m;
  m.upload(verts, indices);
  return m;
}

// 圆柱体 (沿 Y 轴, 中心在原点)
inline Mesh cylinder(int segs, float radius, float height) {
  std::vector<Vertex3D> verts;
  std::vector<unsigned int> indices;
  const float PI_f = 3.14159265358979f;
  float halfH = height * 0.5f;

  for (int i = 0; i <= segs; i++) {
    float ang = (float)i / segs * 2.0f * PI_f;
    float cs = cosf(ang), sn = sinf(ang);
    float u = (float)i / segs;

    // 上顶点
    Vertex3D vt;
    vt.px = cs * radius; vt.py = halfH; vt.pz = sn * radius;
    vt.nx = cs; vt.ny = 0; vt.nz = sn;
    vt.u = u; vt.v = 0;
    vt.r = 1; vt.g = 1; vt.b = 1; vt.a = 1;
    verts.push_back(vt);

    // 下顶点
    Vertex3D vb;
    vb.px = cs * radius; vb.py = -halfH; vb.pz = sn * radius;
    vb.nx = cs; vb.ny = 0; vb.nz = sn;
    vb.u = u; vb.v = 1;
    vb.r = 1; vb.g = 1; vb.b = 1; vb.a = 1;
    verts.push_back(vb);
  }
  for (int i = 0; i < segs; i++) {
    int a = i * 2, b = a + 1, c = a + 2, d = a + 3;
    indices.push_back(a); indices.push_back(b); indices.push_back(c);
    indices.push_back(b); indices.push_back(d); indices.push_back(c);
  }

  Mesh m;
  m.upload(verts, indices);
  return m;
}

// 圆锥 (底部在 y=0, 尖端在 y=height)
inline Mesh cone(int segs, float radius, float height) {
  std::vector<Vertex3D> verts;
  std::vector<unsigned int> indices;
  const float PI_f = 3.14159265358979f;
  float slope = radius / sqrtf(radius * radius + height * height);
  float ny_cone = height / sqrtf(radius * radius + height * height);

  // 尖端
  Vertex3D tip;
  tip.px = 0; tip.py = height; tip.pz = 0;
  tip.nx = 0; tip.ny = ny_cone; tip.nz = 0;
  tip.u = 0.5f; tip.v = 0;
  tip.r = 1; tip.g = 1; tip.b = 1; tip.a = 1;
  verts.push_back(tip); // index 0

  // 底部顶点
  for (int i = 0; i <= segs; i++) {
    float ang = (float)i / segs * 2.0f * PI_f;
    float cs = cosf(ang), sn = sinf(ang);
    Vertex3D v;
    v.px = cs * radius; v.py = 0; v.pz = sn * radius;
    v.nx = cs * ny_cone; v.ny = slope; v.nz = sn * ny_cone;
    v.u = (float)i / segs; v.v = 1;
    v.r = 1; v.g = 1; v.b = 1; v.a = 1;
    verts.push_back(v); // index 1 + i
  }
  for (int i = 0; i < segs; i++) {
    indices.push_back(0);
    indices.push_back(1 + i);
    indices.push_back(2 + i);
  }

  Mesh m;
  m.upload(verts, indices);
  return m;
}

} // namespace MeshGen

// ==========================================================
// Renderer3D
// ==========================================================
class Renderer3D {
public:
  GLuint program3d = 0;
  GLuint earthProgram = 0;
  GLint u_mvp = -1, u_model = -1, u_lightDir = -1, u_viewPos = -1;
  GLint u_baseColor = -1, u_ambientStr = -1;
  // Earth shader uniforms
  GLint ue_mvp = -1, ue_model = -1, ue_lightDir = -1, ue_viewPos = -1;

  Mat4 view, proj;
  Vec3 camPos;
  Vec3 lightDir;

  Renderer3D() {
    // --- Standard 3D Shader (Phong) ---
    const char* vertSrc = R"(
      #version 330 core
      layout(location=0) in vec3 aPos;
      layout(location=1) in vec3 aNormal;
      layout(location=2) in vec2 aUV;
      layout(location=3) in vec4 aColor;

      uniform mat4 uMVP;
      uniform mat4 uModel;

      out vec3 vWorldPos;
      out vec3 vNormal;
      out vec2 vUV;
      out vec4 vColor;

      void main() {
        vec4 worldPos = uModel * vec4(aPos, 1.0);
        vWorldPos = worldPos.xyz;
        vNormal = mat3(transpose(inverse(uModel))) * aNormal;
        vUV = aUV;
        vColor = aColor;
        gl_Position = uMVP * vec4(aPos, 1.0);
      }
    )";

    const char* fragSrc = R"(
      #version 330 core
      in vec3 vWorldPos;
      in vec3 vNormal;
      in vec2 vUV;
      in vec4 vColor;

      uniform vec3 uLightDir;
      uniform vec3 uViewPos;
      uniform vec4 uBaseColor;
      uniform float uAmbientStr;

      out vec4 FragColor;

      void main() {
        vec3 N = normalize(vNormal);
        vec3 L = normalize(uLightDir);
        float ambient = uAmbientStr;
        float diff = max(dot(N, L), 0.0);
        vec3 V = normalize(uViewPos - vWorldPos);
        vec3 H = normalize(L + V);
        float spec = pow(max(dot(N, H), 0.0), 32.0);
        vec4 color = uBaseColor * vColor;
        vec3 result = color.rgb * (ambient + diff * 0.7 + spec * 0.3);
        FragColor = vec4(result, color.a);
      }
    )";

    program3d = compileProgram(vertSrc, fragSrc);
    u_mvp = glGetUniformLocation(program3d, "uMVP");
    u_model = glGetUniformLocation(program3d, "uModel");
    u_lightDir = glGetUniformLocation(program3d, "uLightDir");
    u_viewPos = glGetUniformLocation(program3d, "uViewPos");
    u_baseColor = glGetUniformLocation(program3d, "uBaseColor");
    u_ambientStr = glGetUniformLocation(program3d, "uAmbientStr");

    // --- Earth Shader (procedural continents) ---
    const char* earthFragSrc = R"(
      #version 330 core
      in vec3 vWorldPos;
      in vec3 vNormal;
      in vec2 vUV;
      in vec4 vColor;

      uniform vec3 uLightDir;
      uniform vec3 uViewPos;

      out vec4 FragColor;

      // Simple 3D hash noise for procedural terrain
      float hash(vec3 p) {
        p = fract(p * vec3(443.897, 441.423, 437.195));
        p += dot(p, p.yzx + 19.19);
        return fract((p.x + p.y) * p.z);
      }

      float noise3d(vec3 p) {
        vec3 i = floor(p);
        vec3 f = fract(p);
        f = f * f * (3.0 - 2.0 * f); // smoothstep
        float n000 = hash(i);
        float n100 = hash(i + vec3(1,0,0));
        float n010 = hash(i + vec3(0,1,0));
        float n110 = hash(i + vec3(1,1,0));
        float n001 = hash(i + vec3(0,0,1));
        float n101 = hash(i + vec3(1,0,1));
        float n011 = hash(i + vec3(0,1,1));
        float n111 = hash(i + vec3(1,1,1));
        float nx00 = mix(n000, n100, f.x);
        float nx10 = mix(n010, n110, f.x);
        float nx01 = mix(n001, n101, f.x);
        float nx11 = mix(n011, n111, f.x);
        float nxy0 = mix(nx00, nx10, f.y);
        float nxy1 = mix(nx01, nx11, f.y);
        return mix(nxy0, nxy1, f.z);
      }

      float fbm(vec3 p) {
        float v = 0.0;
        float amp = 0.5;
        for (int i = 0; i < 5; i++) {
          v += noise3d(p) * amp;
          p *= 2.1;
          amp *= 0.5;
        }
        return v;
      }

      void main() {
        vec3 N = normalize(vNormal);
        vec3 L = normalize(uLightDir);
        vec3 V = normalize(uViewPos - vWorldPos);

        // Use normal as 3D coordinate for procedural texture
        vec3 texCoord = normalize(vWorldPos) * 3.0;
        float continent = fbm(texCoord * 1.5);

        // Latitude for ice caps
        float lat = abs(N.y);

        // Color mapping
        vec3 ocean = vec3(0.05, 0.15, 0.45);
        vec3 shallow = vec3(0.1, 0.3, 0.6);
        vec3 land_low = vec3(0.15, 0.45, 0.12);
        vec3 land_high = vec3(0.55, 0.45, 0.25);
        vec3 mountain = vec3(0.5, 0.42, 0.35);
        vec3 snow = vec3(0.9, 0.92, 0.95);

        vec3 surfColor;
        if (continent < 0.42) {
          surfColor = mix(ocean, shallow, continent / 0.42);
        } else if (continent < 0.48) {
          float t = (continent - 0.42) / 0.06;
          surfColor = mix(shallow, land_low, t);
        } else if (continent < 0.65) {
          float t = (continent - 0.48) / 0.17;
          surfColor = mix(land_low, land_high, t);
        } else if (continent < 0.8) {
          float t = (continent - 0.65) / 0.15;
          surfColor = mix(land_high, mountain, t);
        } else {
          surfColor = mountain;
        }

        // Ice caps at poles
        if (lat > 0.85) {
          float ice = smoothstep(0.85, 0.95, lat);
          surfColor = mix(surfColor, snow, ice);
        }

        // Cloud layer (separate noise layer)
        float clouds = fbm(texCoord * 2.5 + vec3(0.0, 100.0, 0.0));
        clouds = smoothstep(0.45, 0.7, clouds) * 0.5;
        surfColor = mix(surfColor, vec3(1.0), clouds);

        // Lighting
        float diff = max(dot(N, L), 0.0);
        float ambient = 0.08;

        // Atmosphere rim glow
        float rim = 1.0 - max(dot(N, V), 0.0);
        rim = pow(rim, 3.0);
        vec3 atmosColor = vec3(0.3, 0.5, 1.0) * rim * 0.6;

        vec3 result = surfColor * (ambient + diff * 0.85) + atmosColor;

        // Night side city lights (subtle)
        if (diff < 0.05) {
          float city = noise3d(texCoord * 10.0);
          if (continent > 0.48 && city > 0.7) {
            result += vec3(1.0, 0.8, 0.3) * 0.1 * (1.0 - diff / 0.05);
          }
        }

        FragColor = vec4(result, 1.0);
      }
    )";

    earthProgram = compileProgram(vertSrc, earthFragSrc);
    ue_mvp = glGetUniformLocation(earthProgram, "uMVP");
    ue_model = glGetUniformLocation(earthProgram, "uModel");
    ue_lightDir = glGetUniformLocation(earthProgram, "uLightDir");
    ue_viewPos = glGetUniformLocation(earthProgram, "uViewPos");

    lightDir = Vec3(0.5f, 0.8f, 0.3f).normalized();
  }

  void beginFrame(const Mat4& viewMat, const Mat4& projMat, const Vec3& cameraPos) {
    view = viewMat;
    proj = projMat;
    camPos = cameraPos;
    glEnable(GL_DEPTH_TEST);
    glClear(GL_DEPTH_BUFFER_BIT);
  }

  // 绘制单个 Mesh, 指定 Model 矩阵和颜色
  void drawMesh(const Mesh& mesh, const Mat4& model,
                float cr = 1, float cg = 1, float cb = 1, float ca = 1,
                float ambient = 0.15f) {
    glUseProgram(program3d);

    Mat4 mvp = proj * view * model;
    glUniformMatrix4fv(u_mvp, 1, GL_FALSE, mvp.m);
    glUniformMatrix4fv(u_model, 1, GL_FALSE, model.m);
    glUniform3f(u_lightDir, lightDir.x, lightDir.y, lightDir.z);
    glUniform3f(u_viewPos, camPos.x, camPos.y, camPos.z);
    glUniform4f(u_baseColor, cr, cg, cb, ca);
    glUniform1f(u_ambientStr, ambient);

    mesh.draw();
  }

  // 用地球专用着色器绘制
  void drawEarth(const Mesh& mesh, const Mat4& model) {
    glUseProgram(earthProgram);
    Mat4 mvp = proj * view * model;
    glUniformMatrix4fv(ue_mvp, 1, GL_FALSE, mvp.m);
    glUniformMatrix4fv(ue_model, 1, GL_FALSE, model.m);
    glUniform3f(ue_lightDir, lightDir.x, lightDir.y, lightDir.z);
    glUniform3f(ue_viewPos, camPos.x, camPos.y, camPos.z);
    mesh.draw();
  }

  void endFrame() {
    glDisable(GL_DEPTH_TEST);
  }

private:
  GLuint compileShader(const char* src, GLenum type) {
    GLuint s = glCreateShader(type);
    glShaderSource(s, 1, &src, nullptr);
    glCompileShader(s);
    int ok;
    glGetShaderiv(s, GL_COMPILE_STATUS, &ok);
    if (!ok) {
      char log[512];
      glGetShaderInfoLog(s, 512, nullptr, log);
      printf("Shader Error: %s\n", log);
    }
    return s;
  }

  GLuint compileProgram(const char* vert, const char* frag) {
    GLuint vs = compileShader(vert, GL_VERTEX_SHADER);
    GLuint fs = compileShader(frag, GL_FRAGMENT_SHADER);
    GLuint prog = glCreateProgram();
    glAttachShader(prog, vs);
    glAttachShader(prog, fs);
    glLinkProgram(prog);
    int ok;
    glGetProgramiv(prog, GL_LINK_STATUS, &ok);
    if (!ok) {
      char log[512];
      glGetProgramInfoLog(prog, 512, nullptr, log);
      printf("Link Error: %s\n", log);
    }
    glDeleteShader(vs);
    glDeleteShader(fs);
    return prog;
  }
};
