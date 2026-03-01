#pragma once
// ==========================================================
// renderer3d.h — 3D Rendering Pipeline
// Requires: glad, math3d.h
// ==========================================================

#include "math3d.h"
#include "rocket_state.h"
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

// 立方体/长方体 (中心在原点)
inline Mesh box(float width, float height, float depth) {
  std::vector<Vertex3D> verts;
  std::vector<unsigned int> indices;

  float dx = width * 0.5f;
  float dy = height * 0.5f;
  float dz = depth * 0.5f;

  Vec3 p[8] = {
    Vec3(-dx, -dy,  dz), Vec3( dx, -dy,  dz), Vec3( dx,  dy,  dz), Vec3(-dx,  dy,  dz), // Front
    Vec3(-dx, -dy, -dz), Vec3( dx, -dy, -dz), Vec3( dx,  dy, -dz), Vec3(-dx,  dy, -dz)  // Back
  };

  Vec3 n[6] = {
    Vec3( 0.0f,  0.0f,  1.0f), Vec3( 0.0f,  0.0f, -1.0f), // Front, Back
    Vec3(-1.0f,  0.0f,  0.0f), Vec3( 1.0f,  0.0f,  0.0f), // Left, Right
    Vec3( 0.0f,  1.0f,  0.0f), Vec3( 0.0f, -1.0f,  0.0f)  // Top, Bottom
  };

  // Face definitions (4 vertices per face)
  int f[6][4] = {
    {0, 1, 2, 3}, // Front
    {5, 4, 7, 6}, // Back
    {4, 0, 3, 7}, // Left
    {1, 5, 6, 2}, // Right
    {3, 2, 6, 7}, // Top
    {4, 5, 1, 0}  // Bottom
  };

  for (int i = 0; i < 6; i++) {
    for (int j = 0; j < 4; j++) {
      Vertex3D v;
      Vec3 pos = p[f[i][j]];
      v.px = pos.x; v.py = pos.y; v.pz = pos.z;
      v.nx = n[i].x; v.ny = n[i].y; v.nz = n[i].z;
      v.u = (j == 1 || j == 2) ? 1.0f : 0.0f;
      v.v = (j == 2 || j == 3) ? 1.0f : 0.0f;
      v.r = 1; v.g = 1; v.b = 1; v.a = 1;
      verts.push_back(v);
    }
    int base = i * 4;
    indices.push_back(base + 0); indices.push_back(base + 1); indices.push_back(base + 2);
    indices.push_back(base + 0); indices.push_back(base + 2); indices.push_back(base + 3);
  }

  Mesh m;
  m.upload(verts, indices);
  return m;
}

// 环形图 (用于土星环, 原点中心, 位于XZ平面)
inline Mesh ring(int segs, float innerRadius, float outerRadius) {
  std::vector<Vertex3D> verts;
  std::vector<unsigned int> indices;
  const float PI_f = 3.14159265358979f;

  for (int i = 0; i <= segs; i++) {
    float ang = (float)i / segs * 2.0f * PI_f;
    float cs = cosf(ang), sn = sinf(ang);

    Vertex3D vi;
    vi.px = cs * innerRadius; vi.py = 0; vi.pz = sn * innerRadius;
    vi.nx = 0; vi.ny = 1; vi.nz = 0;
    vi.u = (float)i / segs; vi.v = 0;
    vi.r = 1; vi.g = 1; vi.b = 1; vi.a = 1;

    Vertex3D vo;
    vo.px = cs * outerRadius; vo.py = 0; vo.pz = sn * outerRadius;
    vo.nx = 0; vo.ny = 1; vo.nz = 0;
    vo.u = (float)i / segs; vo.v = 1;
    vo.r = 1; vo.g = 1; vo.b = 1; vo.a = 1;

    verts.push_back(vi);
    verts.push_back(vo);
  }

  for (int i = 0; i < segs; i++) {
    int v0 = i * 2;
    int v1 = v0 + 1;
    int v2 = v0 + 2;
    int v3 = v0 + 3;

    indices.push_back(v0); indices.push_back(v2); indices.push_back(v1);
    indices.push_back(v1); indices.push_back(v2); indices.push_back(v3);
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
  GLuint gasGiantProgram = 0;
  GLuint barrenProgram = 0;
  GLuint ringProgram = 0;
  GLuint billboardProg = 0;
  GLuint atmoProg = 0;
  GLuint skyboxProg = 0;
  GLuint skyboxVAO = 0, skyboxVBO = 0;
  GLuint billboardVAO = 0, billboardVBO = 0;
  GLint u_mvp = -1, u_model = -1, u_lightDir = -1, u_viewPos = -1;
  GLint u_baseColor = -1, u_ambientStr = -1;
  GLint ue_mvp = -1, ue_model = -1, ue_lightDir = -1, ue_viewPos = -1, ue_time = -1;
  GLint ugg_mvp = -1, ugg_model = -1, ugg_lightDir = -1, ugg_viewPos = -1, ugg_baseColor = -1;
  GLint uba_mvp = -1, uba_model = -1, uba_lightDir = -1, uba_viewPos = -1, uba_baseColor = -1;
  GLint uri_mvp = -1, uri_model = -1, uri_lightDir = -1, uri_viewPos = -1, uri_baseColor = -1;
  // Billboard uniforms
  GLint ub_vp = -1, ub_proj = -1, ub_pos = -1, ub_size = -1, ub_color = -1;
  // Atmosphere uniforms
  GLint ua_mvp = -1, ua_model = -1, ua_lightDir = -1, ua_viewPos = -1;
  // Skybox uniforms
  GLint us_invViewProj = -1;

  // ===== RSS-Reborn Per-Planet Shader Programs =====
  GLuint mercuryProgram = 0, venusProgram = 0, moonProgram = 0, marsProgram = 0;
  GLuint jupiterProgram = 0, saturnProgram = 0, uranusProgram = 0, neptuneProgram = 0;
  // Per-planet uniform locations (all share same uniform names for simplicity)
  struct PlanetUniforms { GLint mvp=-1, model=-1, lightDir=-1, viewPos=-1, baseColor=-1, time=-1; };
  PlanetUniforms u_mercury, u_venus, u_moon, u_mars, u_jupiter, u_saturn, u_uranus, u_neptune;

  Mat4 view, proj;
  Vec3 camPos;
  Vec3 lightDir;

  // Ribbon Renderer properties
  GLuint ribbonProg, ribbonVAO, ribbonVBO;
  GLint ur_mvp;

  // Lens Flare properties
  GLuint lensFlareProg, lfVAO, lfVBO;
  GLint ulf_sunScreenPos, ulf_aspect, ulf_color, ulf_intensity;

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
      out vec3 vLocalPos;

      void main() {
        vec4 worldPos = uModel * vec4(aPos, 1.0);
        vWorldPos = worldPos.xyz;
        vNormal = mat3(transpose(inverse(uModel))) * aNormal;
        vUV = aUV;
        vColor = aColor;
        vLocalPos = aPos;
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

    // --- Earth Shader (RSS-Reborn quality procedural rendering) ---
    const char* earthFragSrc = R"(
      #version 330 core
      in vec3 vWorldPos;
      in vec3 vNormal;
      in vec2 vUV;
      in vec4 vColor;
      in vec3 vLocalPos;

      uniform vec3 uLightDir;
      uniform vec3 uViewPos;
      uniform float uTime;

      out vec4 FragColor;

      // ---- Noise Primitives ----
      float hash(vec3 p) {
        p = fract(p * vec3(443.897, 441.423, 437.195));
        p += dot(p, p.yzx + 19.19);
        return fract((p.x + p.y) * p.z);
      }
      float hash1(float n) { return fract(sin(n) * 43758.5453123); }

      float noise3d(vec3 p) {
        vec3 i = floor(p);
        vec3 f = fract(p);
        f = f * f * f * (f * (f * 6.0 - 15.0) + 10.0); // quintic smoothstep
        float n000 = hash(i); float n100 = hash(i + vec3(1,0,0));
        float n010 = hash(i + vec3(0,1,0)); float n110 = hash(i + vec3(1,1,0));
        float n001 = hash(i + vec3(0,0,1)); float n101 = hash(i + vec3(1,0,1));
        float n011 = hash(i + vec3(0,1,1)); float n111 = hash(i + vec3(1,1,1));
        float nx00 = mix(n000, n100, f.x); float nx10 = mix(n010, n110, f.x);
        float nx01 = mix(n001, n101, f.x); float nx11 = mix(n011, n111, f.x);
        float nxy0 = mix(nx00, nx10, f.y); float nxy1 = mix(nx01, nx11, f.y);
        return mix(nxy0, nxy1, f.z);
      }

      float fbm(vec3 p, int octaves) {
        float v = 0.0, amp = 0.5;
        for (int i = 0; i < octaves; i++) {
          v += noise3d(p) * amp;
          p = p * 2.07 + vec3(0.131, -0.217, 0.344);
          amp *= 0.48;
        }
        return v;
      }

      // Domain-warped FBM for more organic continent shapes
      float warpedFbm(vec3 p) {
        vec3 q = vec3(fbm(p, 4), fbm(p + vec3(5.2, 1.3, 2.8), 4), fbm(p + vec3(9.1, 4.7, 3.1), 4));
        return fbm(p + q * 1.6, 8);
      }

      // Ridge noise for mountain ranges
      float ridgeNoise(vec3 p, int octaves) {
        float v = 0.0, amp = 0.5, prev = 1.0;
        for (int i = 0; i < octaves; i++) {
          float n = abs(noise3d(p));
          n = 1.0 - n;  // Invert to create ridges
          n = n * n;     // Sharpen ridges
          v += n * amp * prev;
          prev = n;
          p = p * 2.07 + vec3(0.131, -0.217, 0.344);
          amp *= 0.48;
        }
        return v;
      }

      void main() {
        vec3 N = normalize(vNormal);
        vec3 L = normalize(uLightDir);
        vec3 V = normalize(uViewPos - vWorldPos);
        vec3 H = normalize(L + V);

        // 3D texture coordinate from unit sphere surface
        vec3 sph = normalize(vLocalPos);
        vec3 texCoord = sph * 3.5;
        float lat = sph.y; // -1 to 1 (south to north pole)
        float absLat = abs(lat);

        // ---- TERRAIN (domain-warped 8-octave FBM) ----
        float continent = warpedFbm(texCoord * 1.2);

        // Bias continent generation: more ocean (Earth is 71% water)
        float seaLevel = 0.44;
        bool isLand = continent > seaLevel;
        float landHeight = isLand ? (continent - seaLevel) / (1.0 - seaLevel) : 0.0;

        // ---- SURFACE COLOR ----
        // Ocean palette
        vec3 deepOcean   = vec3(0.012, 0.045, 0.14);
        vec3 midOcean    = vec3(0.03, 0.10, 0.30);
        vec3 shallowSea  = vec3(0.06, 0.22, 0.42);
        vec3 coastWater  = vec3(0.08, 0.35, 0.50);

        // Land palette
        vec3 beach       = vec3(0.76, 0.70, 0.50);
        vec3 lowland     = vec3(0.18, 0.42, 0.10);
        vec3 forest      = vec3(0.08, 0.30, 0.06);
        vec3 highland    = vec3(0.45, 0.38, 0.22);
        vec3 mountain    = vec3(0.52, 0.46, 0.40);
        vec3 peak        = vec3(0.78, 0.76, 0.74);
        vec3 snow        = vec3(0.92, 0.94, 0.96);
        vec3 desert      = vec3(0.72, 0.58, 0.32);
        vec3 tundra      = vec3(0.55, 0.58, 0.50);

        vec3 surfColor;
        float specMask = 0.0; // 1.0 for water (enables specular)

        if (!isLand) {
          // Ocean depth gradient
          float oceanDepth = (seaLevel - continent) / seaLevel;
          if (oceanDepth < 0.05) surfColor = mix(coastWater, shallowSea, oceanDepth / 0.05);
          else if (oceanDepth < 0.3) surfColor = mix(shallowSea, midOcean, (oceanDepth - 0.05) / 0.25);
          else surfColor = mix(midOcean, deepOcean, min(1.0, (oceanDepth - 0.3) / 0.7));
          specMask = 1.0;
        } else {
          // Biome distribution based on latitude + altitude + noise + moisture
          float biomeNoise = noise3d(texCoord * 6.0);
          float moisture = fbm(texCoord * 3.0 + vec3(42.0), 4); // Moisture gradient
          float ridgeH = ridgeNoise(texCoord * 2.5, 6); // Mountain ridge detail
          float combinedH = landHeight * 0.7 + ridgeH * 0.3;

          if (combinedH < 0.02) {
            surfColor = beach;
          } else if (absLat < 0.25 && moisture < 0.4 && combinedH < 0.3) {
            // Tropical desert
            vec3 sandDark = vec3(0.60, 0.45, 0.25);
            surfColor = mix(desert, sandDark, biomeNoise * 0.5);
          } else if (absLat < 0.20 && moisture > 0.45 && combinedH < 0.35) {
            // Tropical rainforest
            vec3 jungle = vec3(0.05, 0.25, 0.04);
            surfColor = mix(forest, jungle, moisture);
          } else if (absLat < 0.55 && combinedH < 0.4) {
            // Temperate forests/grassland
            float forestBlend = smoothstep(0.02, 0.15, combinedH);
            vec3 grassland = vec3(0.25, 0.48, 0.12);
            surfColor = mix(grassland, forest, forestBlend * moisture);
            surfColor = mix(surfColor, lowland, biomeNoise * 0.3);
          } else if (absLat > 0.65 && combinedH < 0.5) {
            // Tundra / taiga
            vec3 taiga = vec3(0.20, 0.32, 0.18);
            surfColor = mix(tundra, taiga, moisture * (1.0 - absLat));
          } else {
            // Highland/mountain with ridge detail
            if (combinedH < 0.3) surfColor = mix(lowland, highland, combinedH / 0.3);
            else if (combinedH < 0.55) surfColor = mix(highland, mountain, (combinedH - 0.3) / 0.25);
            else surfColor = mix(mountain, peak, (combinedH - 0.55) / 0.45);
          }

          // High-altitude snow line (affected by ridge height)
          float snowLine = 0.65 - absLat * 0.45;
          if (combinedH > snowLine) {
            float snowBlend = smoothstep(snowLine, snowLine + 0.12, combinedH);
            snowBlend *= (0.7 + 0.3 * ridgeH); // More snow on ridges
            surfColor = mix(surfColor, snow, snowBlend);
          }
        }

        // ---- POLAR ICE CAPS ----
        float iceNoise = fbm(texCoord * 4.0 + vec3(100.0), 4);
        float iceEdge = 0.82 - iceNoise * 0.08; // Fractal ice edge
        if (absLat > iceEdge) {
          float ice = smoothstep(iceEdge, iceEdge + 0.06, absLat);
          surfColor = mix(surfColor, snow, ice * 0.95);
          specMask = mix(specMask, 0.3, ice); // Ice has some specular
        }

        // ---- CLOUD LAYERS (3 types) ----
        float timeOff = uTime * 0.0001;
        // Cumulus (large convective clouds)
        float clouds1 = fbm(texCoord * 2.0 + vec3(timeOff, 50.0, timeOff * 0.5), 7);
        clouds1 = smoothstep(0.36, 0.62, clouds1);
        // Stratocumulus (mid-level blankets)
        float clouds2 = fbm(texCoord * 3.5 + vec3(-timeOff * 0.8, 120.0, timeOff * 0.3), 5);
        clouds2 = smoothstep(0.42, 0.68, clouds2) * 0.6;
        // Cirrus (high, thin, wispy)
        float clouds3 = fbm(texCoord * 6.0 + vec3(-timeOff * 1.5, 200.0, timeOff * 0.7), 4);
        clouds3 = smoothstep(0.52, 0.78, clouds3) * 0.3;
        float cloudCover = min(1.0, clouds1 + clouds2 + clouds3);

        // Cloud self-shadowing (lit side brighter)
        float cloudLit = max(0.6, dot(N, L) * 0.5 + 0.5);
        vec3 cloudColorDay = vec3(0.95, 0.96, 0.98) * cloudLit;
        vec3 cloudShadowColor = vec3(0.45, 0.48, 0.55);

        // Sunset/sunrise cloud coloring (golden-orange at low sun angles)
        float sunAngle = dot(N, L);
        float sunsetZone = smoothstep(-0.05, 0.15, sunAngle) * smoothstep(0.40, 0.05, sunAngle);
        vec3 sunsetCloudLit = vec3(1.0, 0.72, 0.30);    // Golden tops
        vec3 sunsetCloudDark = vec3(0.85, 0.35, 0.10);   // Deep orange-red underside
        vec3 cloudColorSunset = mix(sunsetCloudDark, sunsetCloudLit, cloudLit);

        // Blend between day and sunset cloud colors
        vec3 cloudColor = mix(cloudColorDay, cloudColorSunset, sunsetZone);
        cloudColor = mix(cloudShadowColor, cloudColor, smoothstep(-0.12, 0.25, sunAngle));

        // Cloud shadow on surface
        float cloudShadow = 1.0 - cloudCover * 0.35;
        surfColor *= cloudShadow;

        // Blend clouds over surface
        surfColor = mix(surfColor, cloudColor, cloudCover * 0.88);

        // ---- LIGHTING ----
        float NdotL = dot(N, L);
        float diff = max(NdotL, 0.0);
        float ambient = 0.008; // Very dark ambient for space realism

        // Terminator softening (smooth day-night transition)
        float terminator = smoothstep(-0.1, 0.15, NdotL);

        // Specular: ocean sun glint (Blinn-Phong)
        float NdotH = max(dot(N, H), 0.0);
        float specPower = 256.0; // Sharp sun glint
        float spec = pow(NdotH, specPower) * specMask * 2.5;
        // Fresnel enhancement at grazing angles
        float fresnel = pow(1.0 - max(dot(N, V), 0.0), 4.0);
        spec *= (1.0 + fresnel * 3.0);

        // ---- ATMOSPHERE RIM ----
        float rim = 1.0 - max(dot(N, V), 0.0);
        float rimPow = pow(rim, 4.0);
        // Rayleigh blue on day side, warm sunset at terminator
        vec3 dayRim = vec3(0.25, 0.50, 1.0);
        vec3 sunsetRim = vec3(1.0, 0.35, 0.08);
        float rimSunsetZone = smoothstep(-0.05, 0.12, NdotL) * smoothstep(0.35, 0.0, NdotL);
        vec3 atmosColor = mix(sunsetRim, dayRim, smoothstep(0.0, 0.3, NdotL));
        atmosColor = atmosColor * rimPow * 0.65 * smoothstep(-0.15, 0.1, NdotL);
        // Sunset boost
        atmosColor += sunsetRim * rimSunsetZone * rimPow * 0.5;

        vec3 result = surfColor * (ambient + diff * 0.92) + atmosColor;
        result += vec3(1.0, 0.95, 0.85) * spec * diff; // Sun glint only on lit side

        // ---- NIGHT SIDE CITY LIGHTS ----
        if (NdotL < 0.03 && isLand) {
          float nightFade = smoothstep(0.03, -0.08, NdotL);
          // Multi-scale city noise for clustering
          float city1 = noise3d(texCoord * 12.0);
          float city2 = noise3d(texCoord * 25.0 + vec3(7.7));
          float city3 = noise3d(texCoord * 50.0 + vec3(13.1));
          // Cities cluster in lowlands, avoid mountains and deserts
          float habitability = smoothstep(0.5, 0.0, landHeight) * smoothstep(0.0, 0.1, landHeight);
          habitability *= (1.0 - smoothstep(0.6, 0.85, absLat)); // Less at poles
          float cityBrightness = 0.0;
          if (city1 > 0.55) cityBrightness += (city1 - 0.55) * 4.0;
          if (city2 > 0.6) cityBrightness += (city2 - 0.6) * 2.0;
          if (city3 > 0.65) cityBrightness += (city3 - 0.65) * 1.0;
          cityBrightness *= habitability * nightFade;
          cityBrightness = min(cityBrightness, 1.2);
          // Warm amber/sodium light color
          vec3 cityColor = mix(vec3(1.0, 0.7, 0.2), vec3(1.0, 0.9, 0.6), city2);
          result += cityColor * cityBrightness * 0.8;
        }

        FragColor = vec4(result, 1.0);
      }
    )";

    earthProgram = compileProgram(vertSrc, earthFragSrc);
    ue_mvp = glGetUniformLocation(earthProgram, "uMVP");
    ue_model = glGetUniformLocation(earthProgram, "uModel");
    ue_lightDir = glGetUniformLocation(earthProgram, "uLightDir");
    ue_viewPos = glGetUniformLocation(earthProgram, "uViewPos");
    ue_time = glGetUniformLocation(earthProgram, "uTime");

    // --- Gas Giant Shader ---
    const char* gasGiantFragSrc = R"(
      #version 330 core
      in vec3 vWorldPos;
      in vec3 vNormal;
      in vec2 vUV;
      in vec3 vLocalPos;
      uniform vec3 uLightDir;
      uniform vec3 uViewPos;
      uniform vec4 uBaseColor;
      out vec4 FragColor;

      float hash(vec3 p) {
        p = fract(p * vec3(443.897, 441.423, 437.195));
        p += dot(p, p.yzx + 19.19);
        return fract((p.x + p.y) * p.z);
      }
      float noise3d(vec3 p) {
        vec3 i = floor(p); vec3 f = fract(p);
        f = f * f * (3.0 - 2.0 * f);
        float n000 = hash(i); float n100 = hash(i + vec3(1,0,0));
        float n010 = hash(i + vec3(0,1,0)); float n110 = hash(i + vec3(1,1,0));
        float n001 = hash(i + vec3(0,0,1)); float n101 = hash(i + vec3(1,0,1));
        float n011 = hash(i + vec3(0,1,1)); float n111 = hash(i + vec3(1,1,1));
        float nx00 = mix(n000, n100, f.x); float nx10 = mix(n010, n110, f.x);
        float nx01 = mix(n001, n101, f.x); float nx11 = mix(n011, n111, f.x);
        float nxy0 = mix(nx00, nx10, f.y); float nxy1 = mix(nx01, nx11, f.y);
        return mix(nxy0, nxy1, f.z);
      }
      float fbm(vec3 p) {
        float v = 0.0; float amp = 0.5;
        for (int i = 0; i < 5; i++) { v += noise3d(p) * amp; p *= 2.1; amp *= 0.5; }
        return v;
      }

      void main() {
        vec3 N = normalize(vNormal);
        vec3 L = normalize(uLightDir);
        vec3 V = normalize(uViewPos - vWorldPos);
        vec3 texCoord = normalize(vLocalPos) * 5.0;
        
        float warp = fbm(texCoord + vec3(0, texCoord.y * 3.0, 0));
        float bands = sin(texCoord.y * 15.0 + warp * 5.0);
        bands = smoothstep(-1.0, 1.0, bands);
        
        vec3 color1 = uBaseColor.rgb;
        vec3 color2 = uBaseColor.rgb * 0.4 + vec3(0.3);
        vec3 surfColor = mix(color1, color2, bands);
        
        float storms = fbm(texCoord * 4.0);
        if (storms > 0.7) {
            surfColor = mix(surfColor, vec3(0.9, 0.8, 0.7), (storms - 0.7) * 3.3);
        }

        float diff = max(dot(N, L), 0.0);
        float ambient = 0.15;
        float rim = 1.0 - max(dot(N, V), 0.0);
        rim = pow(rim, 3.0);
        vec3 atmosColor = uBaseColor.rgb * rim * 0.5;

        vec3 result = surfColor * (ambient + diff * 0.85) + atmosColor;
        FragColor = vec4(result, 1.0);
      }
    )";
    gasGiantProgram = compileProgram(vertSrc, gasGiantFragSrc);
    ugg_mvp = glGetUniformLocation(gasGiantProgram, "uMVP");
    ugg_model = glGetUniformLocation(gasGiantProgram, "uModel");
    ugg_lightDir = glGetUniformLocation(gasGiantProgram, "uLightDir");
    ugg_viewPos = glGetUniformLocation(gasGiantProgram, "uViewPos");
    ugg_baseColor = glGetUniformLocation(gasGiantProgram, "uBaseColor");

    // --- Barren/Crater Shader ---
    const char* barrenFragSrc = R"(
      #version 330 core
      in vec3 vWorldPos;
      in vec3 vNormal;
      in vec3 vLocalPos;
      uniform vec3 uLightDir;
      uniform vec3 uViewPos;
      uniform vec4 uBaseColor;
      out vec4 FragColor;

      float hash(vec3 p) {
        p = fract(p * vec3(443.897, 441.423, 437.195));
        p += dot(p, p.yzx + 19.19);
        return fract((p.x + p.y) * p.z);
      }
      float noise3d(vec3 p) {
        vec3 i = floor(p); vec3 f = fract(p);
        f = f * f * (3.0 - 2.0 * f);
        float n000 = hash(i); float n100 = hash(i + vec3(1,0,0));
        float n010 = hash(i + vec3(0,1,0)); float n110 = hash(i + vec3(1,1,0));
        float n001 = hash(i + vec3(0,0,1)); float n101 = hash(i + vec3(1,0,1));
        float n011 = hash(i + vec3(0,1,1)); float n111 = hash(i + vec3(1,1,1));
        float nx00 = mix(n000, n100, f.x); float nx10 = mix(n010, n110, f.x);
        float nx01 = mix(n001, n101, f.x); float nx11 = mix(n011, n111, f.x);
        float nxy0 = mix(nx00, nx10, f.y); float nxy1 = mix(nx01, nx11, f.y);
        return mix(nxy0, nxy1, f.z);
      }
      float fbm(vec3 p) {
        float v = 0.0; float amp = 0.5;
        for (int i = 0; i < 5; i++) { v += noise3d(p) * amp; p *= 2.1; amp *= 0.5; }
        return v;
      }

      void main() {
        vec3 N = normalize(vNormal);
        vec3 L = normalize(uLightDir);
        vec3 texCoord = normalize(vLocalPos) * 8.0;
        
        float craters = fbm(texCoord * 4.0);
        craters = abs(craters * 2.0 - 1.0);
        
        vec3 surfColor = uBaseColor.rgb * (0.5 + 0.5 * craters);
        float diff = max(dot(N, L), 0.0);
        float ambient = 0.05;

        vec3 result = surfColor * (ambient + diff * 0.95);
        FragColor = vec4(result, 1.0);
      }
    )";
    barrenProgram = compileProgram(vertSrc, barrenFragSrc);
    uba_mvp = glGetUniformLocation(barrenProgram, "uMVP");
    uba_model = glGetUniformLocation(barrenProgram, "uModel");
    uba_lightDir = glGetUniformLocation(barrenProgram, "uLightDir");
    uba_viewPos = glGetUniformLocation(barrenProgram, "uViewPos");
    uba_baseColor = glGetUniformLocation(barrenProgram, "uBaseColor");

    // --- Ring Shader ---
    const char* ringFragSrc = R"(
      #version 330 core
      in vec3 vNormal;
      in vec3 vLocalPos;
      uniform vec3 uLightDir;
      uniform vec4 uBaseColor;
      out vec4 FragColor;
      
      void main() {
        vec3 N = normalize(vNormal);
        vec3 L = normalize(uLightDir);
        float diff = max(abs(dot(N, L)), 0.0); // thin ring lit from both sides
        float ambient = 0.1;
        
        float dist = length(vLocalPos.xz);
        float band = sin(dist * 2.0) * cos(dist * 4.0 + 1.2);
        float alpha = smoothstep(-0.5, 0.5, band) * 0.7 + 0.1;
        
        vec3 result = uBaseColor.rgb * (ambient + diff * 0.9);
        FragColor = vec4(result, alpha * uBaseColor.a);
      }
    )";
    ringProgram = compileProgram(vertSrc, ringFragSrc);
    uri_mvp = glGetUniformLocation(ringProgram, "uMVP");
    uri_model = glGetUniformLocation(ringProgram, "uModel");
    uri_lightDir = glGetUniformLocation(ringProgram, "uLightDir");
    uri_baseColor = glGetUniformLocation(ringProgram, "uBaseColor");

    lightDir = Vec3(0.5f, 0.8f, 0.3f).normalized();

    // ===== Helper: compile planet shader and fetch uniforms =====
    auto setupPlanetProg = [&](GLuint& prog, PlanetUniforms& u, const char* fragSrc) {
        prog = compileProgram(vertSrc, fragSrc);
        u.mvp = glGetUniformLocation(prog, "uMVP");
        u.model = glGetUniformLocation(prog, "uModel");
        u.lightDir = glGetUniformLocation(prog, "uLightDir");
        u.viewPos = glGetUniformLocation(prog, "uViewPos");
        u.baseColor = glGetUniformLocation(prog, "uBaseColor");
        u.time = glGetUniformLocation(prog, "uTime");
    };

    // ========================================================================
    // RSS-REBORN PER-PLANET SHADERS
    // Each planet/moon gets a dedicated procedural fragment shader with
    // scientifically accurate features, high-octave noise, and realistic colors.
    // ========================================================================

    // --- Common noise GLSL preamble (shared across all planet shaders) ---
    // We'll embed the noise functions in each shader string since GLSL has no #include.

    // ===== 1. MERCURY SHADER =====
    // Heavily cratered basalt, lobate scarps, color variation from dark gray to warm brown.
    // Very low albedo overall, numerous overlapping craters of varying sizes.
    const char* mercuryFragSrc = R"(
      #version 330 core
      in vec3 vWorldPos; in vec3 vNormal; in vec3 vLocalPos;
      uniform vec3 uLightDir; uniform vec3 uViewPos; uniform vec4 uBaseColor;
      out vec4 FragColor;

      float hash(vec3 p) { p=fract(p*vec3(443.897,441.423,437.195)); p+=dot(p,p.yzx+19.19); return fract((p.x+p.y)*p.z); }
      float noise3d(vec3 p) {
        vec3 i=floor(p); vec3 f=fract(p); f=f*f*f*(f*(f*6.0-15.0)+10.0);
        float n000=hash(i),n100=hash(i+vec3(1,0,0)),n010=hash(i+vec3(0,1,0)),n110=hash(i+vec3(1,1,0));
        float n001=hash(i+vec3(0,0,1)),n101=hash(i+vec3(1,0,1)),n011=hash(i+vec3(0,1,1)),n111=hash(i+vec3(1,1,1));
        float nx00=mix(n000,n100,f.x),nx10=mix(n010,n110,f.x);
        float nx01=mix(n001,n101,f.x),nx11=mix(n011,n111,f.x);
        return mix(mix(nx00,nx10,f.y),mix(nx01,nx11,f.y),f.z);
      }
      float fbm(vec3 p, int oct) { float v=0.0,a=0.5; for(int i=0;i<oct;i++){v+=noise3d(p)*a;p=p*2.07+vec3(0.131,-0.217,0.344);a*=0.48;} return v; }

      // Noise-based crater approximation (cheap, no loops)
      float craterNoise(vec3 p, float freq) {
        float n = noise3d(p * freq);
        // Sharp circular-ish depressions from noise thresholding
        float bowl = smoothstep(0.55, 0.35, n); // crater interior
        float rim = smoothstep(0.55, 0.60, n) * smoothstep(0.68, 0.60, n); // raised rim
        return bowl * 0.6 + rim * 0.25;
      }

      void main() {
        vec3 N = normalize(vNormal); vec3 L = normalize(uLightDir);
        vec3 sph = normalize(vLocalPos);
        vec3 tc = sph * 6.0;

        // Multi-scale noise-based cratering
        float c1 = craterNoise(sph, 8.0);
        float c2 = craterNoise(sph, 16.0) * 0.5;
        float c3 = craterNoise(sph, 32.0) * 0.25;
        float c4 = craterNoise(sph, 64.0) * 0.12;
        float totalCrater = c1 + c2 + c3 + c4;

        // Base terrain noise
        float terrain = fbm(tc * 2.0, 6);

        // Lobate scarps (ridge-like features)
        float scarps = abs(noise3d(tc * 4.0 + vec3(100.0))) * abs(noise3d(tc * 8.0 + vec3(200.0)));
        scarps = smoothstep(0.1, 0.4, scarps) * 0.15;

        // Color: Mercury is very dark (albedo ~0.07-0.14)
        vec3 darkBasalt = vec3(0.08, 0.07, 0.06);
        vec3 warmBrown = vec3(0.14, 0.11, 0.08);
        vec3 brightRay = vec3(0.20, 0.18, 0.16);

        vec3 baseColor = mix(darkBasalt, warmBrown, terrain * 0.8 + 0.2);
        baseColor *= (1.0 - totalCrater * 0.4);
        float rayMask = smoothstep(0.3, 0.6, c1) * noise3d(tc * 20.0);
        baseColor = mix(baseColor, brightRay, rayMask * 0.4);
        baseColor += vec3(scarps);

        float diff = max(dot(N, L), 0.0);
        float ambient = 0.01;
        vec3 result = baseColor * (ambient + diff * 0.99);
        FragColor = vec4(result, 1.0);
      }
    )";
    setupPlanetProg(mercuryProgram, u_mercury, mercuryFragSrc);

    // ===== 2. VENUS SHADER =====
    // Thick multi-layer sulfuric acid cloud deck, atmospheric super-rotation,
    // subtle sub-surface glow (volcanism) visible through thin cloud gaps.
    const char* venusFragSrc = R"(
      #version 330 core
      in vec3 vWorldPos; in vec3 vNormal; in vec3 vLocalPos;
      uniform vec3 uLightDir; uniform vec3 uViewPos; uniform vec4 uBaseColor; uniform float uTime;
      out vec4 FragColor;

      float hash(vec3 p) { p=fract(p*vec3(443.897,441.423,437.195)); p+=dot(p,p.yzx+19.19); return fract((p.x+p.y)*p.z); }
      float noise3d(vec3 p) {
        vec3 i=floor(p); vec3 f=fract(p); f=f*f*f*(f*(f*6.0-15.0)+10.0);
        float n000=hash(i),n100=hash(i+vec3(1,0,0)),n010=hash(i+vec3(0,1,0)),n110=hash(i+vec3(1,1,0));
        float n001=hash(i+vec3(0,0,1)),n101=hash(i+vec3(1,0,1)),n011=hash(i+vec3(0,1,1)),n111=hash(i+vec3(1,1,1));
        float nx00=mix(n000,n100,f.x),nx10=mix(n010,n110,f.x);
        float nx01=mix(n001,n101,f.x),nx11=mix(n011,n111,f.x);
        return mix(mix(nx00,nx10,f.y),mix(nx01,nx11,f.y),f.z);
      }
      float fbm(vec3 p, int oct) { float v=0.0,a=0.5; for(int i=0;i<oct;i++){v+=noise3d(p)*a;p=p*2.07+vec3(0.131,-0.217,0.344);a*=0.48;} return v; }

      void main() {
        vec3 N = normalize(vNormal); vec3 L = normalize(uLightDir);
        vec3 V = normalize(uViewPos - vWorldPos);
        vec3 sph = normalize(vLocalPos);
        float lat = sph.y;
        float timeOff = uTime * 0.00008; // Super-rotation (very slow)

        // === Cloud layer 1: Thick sulfuric acid main deck ===
        vec3 tc1 = sph * 3.0 + vec3(timeOff * 1.0, 0.0, timeOff * 0.5);
        float cloud1 = fbm(tc1, 8);
        // Banded structure (Venus has mild latitudinal banding in UV)
        float bandWarp = fbm(sph * vec3(1.0, 5.0, 1.0) + vec3(timeOff * 0.3), 4) * 0.3;
        float bands = sin(lat * 12.0 + bandWarp * 8.0) * 0.5 + 0.5;

        // === Cloud layer 2: Higher altitude wispy streaks ===
        vec3 tc2 = sph * 5.0 + vec3(-timeOff * 1.5, 50.0, timeOff * 0.8);
        float cloud2 = fbm(tc2, 6) * 0.6;

        // === Cloud layer 3: Dark UV absorber patches (mysterious!) ===
        vec3 tc3 = sph * 2.0 + vec3(timeOff * 0.7, 100.0, -timeOff * 0.4);
        float uvAbsorber = fbm(tc3, 5);
        uvAbsorber = smoothstep(0.35, 0.6, uvAbsorber) * 0.3;

        // Color palette: Venus appears yellow-white to pale cream
        vec3 brightCloud = vec3(0.95, 0.90, 0.75);
        vec3 midCloud = vec3(0.85, 0.78, 0.58);
        vec3 darkCloud = vec3(0.65, 0.55, 0.35);
        vec3 absorberTint = vec3(0.55, 0.45, 0.25); // Dark UV patches

        float cloudMix = cloud1 * 0.6 + cloud2 * 0.3 + bands * 0.1;
        vec3 surfColor = mix(darkCloud, brightCloud, cloudMix);
        surfColor = mix(surfColor, midCloud, bands * 0.4);
        surfColor = mix(surfColor, absorberTint, uvAbsorber);

        // Subtle sub-cloud volcanic glow on night side
        float NdotL = dot(N, L);
        float nightMask = smoothstep(0.0, -0.15, NdotL);
        float volcGlow = fbm(sph * 8.0, 4);
        volcGlow = smoothstep(0.55, 0.75, volcGlow) * 0.15;
        vec3 glowColor = vec3(1.0, 0.3, 0.05); // Deep orange-red
        surfColor += glowColor * volcGlow * nightMask;

        // Lighting
        float diff = max(NdotL, 0.0);
        float ambient = 0.03;

        // Atmosphere rim
        float rim = 1.0 - max(dot(N, V), 0.0);
        float rimPow = pow(rim, 3.0);
        vec3 atmosGlow = vec3(0.95, 0.82, 0.50) * rimPow * 0.5 * smoothstep(-0.1, 0.2, NdotL);

        vec3 result = surfColor * (ambient + diff * 0.85) + atmosGlow;
        FragColor = vec4(result, 1.0);
      }
    )";
    setupPlanetProg(venusProgram, u_venus, venusFragSrc);

    // ===== 3. MOON (Luna) SHADER =====
    // Maria (dark basalt plains) vs highlands, impact craters with ejecta rays,
    // regolith color variation gray-to-brown, very low albedo.
    const char* moonFragSrc = R"(
      #version 330 core
      in vec3 vWorldPos; in vec3 vNormal; in vec3 vLocalPos;
      uniform vec3 uLightDir; uniform vec3 uViewPos; uniform vec4 uBaseColor;
      out vec4 FragColor;

      float hash(vec3 p) { p=fract(p*vec3(443.897,441.423,437.195)); p+=dot(p,p.yzx+19.19); return fract((p.x+p.y)*p.z); }
      float noise3d(vec3 p) {
        vec3 i=floor(p); vec3 f=fract(p); f=f*f*f*(f*(f*6.0-15.0)+10.0);
        float n000=hash(i),n100=hash(i+vec3(1,0,0)),n010=hash(i+vec3(0,1,0)),n110=hash(i+vec3(1,1,0));
        float n001=hash(i+vec3(0,0,1)),n101=hash(i+vec3(1,0,1)),n011=hash(i+vec3(0,1,1)),n111=hash(i+vec3(1,1,1));
        float nx00=mix(n000,n100,f.x),nx10=mix(n010,n110,f.x);
        float nx01=mix(n001,n101,f.x),nx11=mix(n011,n111,f.x);
        return mix(mix(nx00,nx10,f.y),mix(nx01,nx11,f.y),f.z);
      }
      float fbm(vec3 p, int oct) { float v=0.0,a=0.5; for(int i=0;i<oct;i++){v+=noise3d(p)*a;p=p*2.07+vec3(0.131,-0.217,0.344);a*=0.48;} return v; }

      // Noise-based crater approximation
      float craterNoise(vec3 p, float freq) {
        float n = noise3d(p * freq);
        float bowl = smoothstep(0.55, 0.35, n);
        float rim = smoothstep(0.55, 0.60, n) * smoothstep(0.68, 0.60, n);
        return bowl * 0.6 + rim * 0.25;
      }

      void main() {
        vec3 N = normalize(vNormal); vec3 L = normalize(uLightDir);
        vec3 sph = normalize(vLocalPos);
        vec3 tc = sph * 5.0;

        // Maria detection (large dark basalt plains)
        vec3 warp = vec3(fbm(sph * 2.0, 3), fbm(sph * 2.0 + vec3(10.0), 3), fbm(sph * 2.0 + vec3(20.0), 3));
        float maria = fbm(sph * 1.5 + warp * 0.8, 5);
        bool isMare = maria < 0.42;

        // Multi-scale noise-based cratering
        float c1 = craterNoise(sph, 6.0);
        float c2 = craterNoise(sph, 12.0) * 0.5;
        float c3 = craterNoise(sph, 24.0) * 0.25;
        float c4 = craterNoise(sph, 48.0) * 0.12;
        float totalCrater = c1 + c2 + c3 + c4;

        // Regolith fine detail
        float regolith = fbm(tc * 8.0, 5) * 0.15;

        // Colors
        vec3 highlandColor = vec3(0.22, 0.21, 0.19);
        vec3 mareColor = vec3(0.08, 0.08, 0.07);
        vec3 rayColor = vec3(0.28, 0.27, 0.25);

        vec3 baseColor;
        if(isMare) {
          float mareBlend = smoothstep(0.42, 0.30, maria);
          baseColor = mix(highlandColor, mareColor, mareBlend);
        } else {
          baseColor = highlandColor;
        }

        baseColor *= (1.0 - totalCrater * 0.3);
        baseColor += vec3(regolith);

        float rayNoise = noise3d(sph * 30.0);
        float rayMask = smoothstep(0.4, 0.7, c1) * smoothstep(0.5, 0.7, rayNoise);
        baseColor = mix(baseColor, rayColor, rayMask * 0.5);

        float diff = max(dot(N, L), 0.0);
        float ambient = 0.005;
        vec3 result = baseColor * (ambient + diff * 0.995);
        FragColor = vec4(result, 1.0);
      }
    )";
    setupPlanetProg(moonProgram, u_moon, moonFragSrc);

    // ===== 4. MARS SHADER =====
    // Iron oxide red-orange terrain, polar CO2 ice caps, Olympus Mons volcanic shield,
    // Valles Marineris canyon, dust storms, thin atmosphere rim.
    const char* marsFragSrc = R"(
      #version 330 core
      in vec3 vWorldPos; in vec3 vNormal; in vec3 vLocalPos;
      uniform vec3 uLightDir; uniform vec3 uViewPos; uniform vec4 uBaseColor; uniform float uTime;
      out vec4 FragColor;

      float hash(vec3 p) { p=fract(p*vec3(443.897,441.423,437.195)); p+=dot(p,p.yzx+19.19); return fract((p.x+p.y)*p.z); }
      float noise3d(vec3 p) {
        vec3 i=floor(p); vec3 f=fract(p); f=f*f*f*(f*(f*6.0-15.0)+10.0);
        float n000=hash(i),n100=hash(i+vec3(1,0,0)),n010=hash(i+vec3(0,1,0)),n110=hash(i+vec3(1,1,0));
        float n001=hash(i+vec3(0,0,1)),n101=hash(i+vec3(1,0,1)),n011=hash(i+vec3(0,1,1)),n111=hash(i+vec3(1,1,1));
        float nx00=mix(n000,n100,f.x),nx10=mix(n010,n110,f.x);
        float nx01=mix(n001,n101,f.x),nx11=mix(n011,n111,f.x);
        return mix(mix(nx00,nx10,f.y),mix(nx01,nx11,f.y),f.z);
      }
      float fbm(vec3 p, int oct) { float v=0.0,a=0.5; for(int i=0;i<oct;i++){v+=noise3d(p)*a;p=p*2.07+vec3(0.131,-0.217,0.344);a*=0.48;} return v; }
      float ridgeNoise(vec3 p, int oct) {
        float v=0.0,a=0.5,prev=1.0;
        for(int i=0;i<oct;i++){float n=abs(noise3d(p));n=1.0-n;n=n*n;v+=n*a*prev;prev=n;p=p*2.07+vec3(0.131,-0.217,0.344);a*=0.48;}
        return v;
      }

      void main() {
        vec3 N = normalize(vNormal); vec3 L = normalize(uLightDir);
        vec3 V = normalize(uViewPos - vWorldPos);
        vec3 sph = normalize(vLocalPos);
        vec3 tc = sph * 4.0;
        float lat = sph.y;
        float absLat = abs(lat);

        // === Terrain elevation ===
        // Domain-warped FBM for varied terrain
        vec3 warp = vec3(fbm(tc, 3), fbm(tc + vec3(5.2), 3), fbm(tc + vec3(9.1), 3));
        float elevation = fbm(tc * 1.5 + warp * 1.2, 8);

        // Ridge/canyon features (Valles Marineris analog)
        float ridges = ridgeNoise(tc * 2.0 + vec3(42.0, 13.0, 7.0), 6);
        float canyons = 1.0 - ridgeNoise(tc * 1.5 + vec3(77.0, 21.0, 55.0), 5);

        // Volcanic shield (large smooth elevated dome)
        vec3 volcCenter = vec3(0.3, 0.2, 0.5); // Approximate Tharsis region
        float volcDist = length(sph - volcCenter);
        float volcano = smoothstep(0.5, 0.0, volcDist) * 0.3;

        float combinedH = elevation * 0.5 + ridges * 0.25 + volcano;

        // === Color palette ===
        vec3 rustRed = vec3(0.55, 0.22, 0.08);
        vec3 darkRust = vec3(0.35, 0.14, 0.06);
        vec3 paleOchre = vec3(0.72, 0.45, 0.22);
        vec3 darkBasalt = vec3(0.15, 0.10, 0.08);
        vec3 dustBright = vec3(0.80, 0.55, 0.30);
        vec3 iceCap = vec3(0.90, 0.92, 0.95);

        // Base terrain color
        float colorNoise = fbm(tc * 3.0, 4);
        vec3 surfColor = mix(darkRust, rustRed, elevation);
        surfColor = mix(surfColor, paleOchre, colorNoise * 0.4);

        // Canyon floors are darker basalt
        float canyonMask = smoothstep(0.4, 0.2, canyons);
        surfColor = mix(surfColor, darkBasalt, canyonMask * 0.6);

        // Volcanic regions are darker (basalt flows)
        surfColor = mix(surfColor, darkBasalt, volcano);

        // Bright dust deposits in lowlands
        float dustMask = smoothstep(0.3, 0.5, elevation) * smoothstep(0.6, 0.45, absLat);
        surfColor = mix(surfColor, dustBright, dustMask * colorNoise * 0.3);

        // === Polar CO2 ice caps ===
        float iceNoise = fbm(tc * 4.0 + vec3(200.0), 5);
        float iceEdge = 0.75 - iceNoise * 0.08;
        float southBias = (lat < 0.0) ? 0.05 : 0.0; // South cap is larger
        if(absLat > iceEdge - southBias) {
          float ice = smoothstep(iceEdge - southBias, iceEdge + 0.05 - southBias, absLat);
          surfColor = mix(surfColor, iceCap, ice * 0.92);
        }

        // === Dust storm clouds (very thin, animated) ===
        float stormTime = uTime * 0.00005;
        float dust = fbm(sph * 2.5 + vec3(stormTime, 0.0, stormTime * 0.3), 5);
        dust = smoothstep(0.48, 0.7, dust) * 0.25;
        surfColor = mix(surfColor, dustBright, dust);

        // === Lighting ===
        float NdotL = dot(N, L);
        float diff = max(NdotL, 0.0);
        float ambient = 0.008;

        // Thin atmospheric rim (Mars has a very thin atmosphere)
        float rim = 1.0 - max(dot(N, V), 0.0);
        float rimPow = pow(rim, 4.0);
        vec3 marsAtmo = vec3(0.85, 0.55, 0.25) * rimPow * 0.25 * smoothstep(-0.1, 0.1, NdotL);

        vec3 result = surfColor * (ambient + diff * 0.92) + marsAtmo;
        FragColor = vec4(result, 1.0);
      }
    )";
    setupPlanetProg(marsProgram, u_mars, marsFragSrc);

    // ===== 5. JUPITER SHADER =====
    // Detailed alternating zonal bands, Great Red Spot anticyclone, turbulent eddies,
    // ammonia crystal storms, strong belt/zone color contrast.
    const char* jupiterFragSrc = R"(
      #version 330 core
      in vec3 vWorldPos; in vec3 vNormal; in vec3 vLocalPos;
      uniform vec3 uLightDir; uniform vec3 uViewPos; uniform vec4 uBaseColor; uniform float uTime;
      out vec4 FragColor;

      float hash(vec3 p) { p=fract(p*vec3(443.897,441.423,437.195)); p+=dot(p,p.yzx+19.19); return fract((p.x+p.y)*p.z); }
      float noise3d(vec3 p) {
        vec3 i=floor(p); vec3 f=fract(p); f=f*f*f*(f*(f*6.0-15.0)+10.0);
        float n000=hash(i),n100=hash(i+vec3(1,0,0)),n010=hash(i+vec3(0,1,0)),n110=hash(i+vec3(1,1,0));
        float n001=hash(i+vec3(0,0,1)),n101=hash(i+vec3(1,0,1)),n011=hash(i+vec3(0,1,1)),n111=hash(i+vec3(1,1,1));
        float nx00=mix(n000,n100,f.x),nx10=mix(n010,n110,f.x);
        float nx01=mix(n001,n101,f.x),nx11=mix(n011,n111,f.x);
        return mix(mix(nx00,nx10,f.y),mix(nx01,nx11,f.y),f.z);
      }
      float fbm(vec3 p, int oct) { float v=0.0,a=0.5; for(int i=0;i<oct;i++){v+=noise3d(p)*a;p=p*2.07+vec3(0.131,-0.217,0.344);a*=0.48;} return v; }

      void main() {
        vec3 N = normalize(vNormal); vec3 L = normalize(uLightDir);
        vec3 V = normalize(uViewPos - vWorldPos);
        vec3 sph = normalize(vLocalPos);
        vec3 tc = sph * 5.0;
        float lat = sph.y;
        float timeOff = uTime * 0.00003;

        // === Zonal banding (primary visual feature) ===
        // Multi-frequency band structure with turbulent boundaries
        float bandWarp = fbm(sph * vec3(2.0, 1.0, 2.0) + vec3(timeOff * 0.5, 0.0, timeOff * 0.3), 5);
        float bands1 = sin(lat * 22.0 + bandWarp * 4.0);
        float bands2 = sin(lat * 44.0 + bandWarp * 2.0 + 1.5) * 0.3;
        float bands3 = sin(lat * 88.0 + bandWarp * 1.0 + 3.0) * 0.1;
        float bandPattern = bands1 + bands2 + bands3;
        bandPattern = bandPattern * 0.5 + 0.5; // Normalize to 0-1

        // === Turbulent eddies at band boundaries ===
        float eddies = fbm(tc * 3.0 + vec3(timeOff, lat * 5.0, timeOff * 0.7), 7);
        float edgeTurb = 1.0 - abs(bandPattern * 2.0 - 1.0); // Strongest at band edges
        eddies *= edgeTurb;

        // === Great Red Spot (GRS) ===
        vec3 grsCenter = vec3(0.4, -0.35, 0.3); // South of equator
        float grsDist = length(sph - normalize(grsCenter));
        float grsAngle = atan(sph.z - grsCenter.z, sph.x - grsCenter.x);
        float grsSpiral = fbm(vec3(grsAngle * 3.0, grsDist * 20.0, timeOff * 2.0), 4);
        float grsMask = smoothstep(0.25, 0.0, grsDist);
        grsMask *= (0.7 + 0.3 * grsSpiral);

        // === Color palette ===
        // Zones (bright, cream-white): high-altitude ammonia clouds
        vec3 zoneColor = vec3(0.92, 0.88, 0.78);
        // Belts (dark, rusty brown): lower-altitude, chromophore-rich
        vec3 beltColor1 = vec3(0.55, 0.35, 0.18);
        vec3 beltColor2 = vec3(0.70, 0.48, 0.25);
        // GRS color
        vec3 grsColor = vec3(0.75, 0.30, 0.12);
        // Polar regions (darker, more subdued)
        vec3 polarColor = vec3(0.35, 0.30, 0.28);

        // Build surface color
        vec3 beltMix = mix(beltColor1, beltColor2, noise3d(tc * 6.0));
        vec3 surfColor = mix(beltMix, zoneColor, bandPattern);

        // Add turbulent eddy details
        vec3 eddyColor = mix(surfColor, zoneColor * 1.1, eddies * 0.4);
        surfColor = mix(surfColor, eddyColor, edgeTurb * 0.6);

        // Ammonia storm clusters (bright white spots in belts)
        float storms = noise3d(tc * 8.0 + vec3(timeOff * 2.0));
        float stormMask = smoothstep(0.72, 0.85, storms) * (1.0 - bandPattern) * 0.5;
        surfColor = mix(surfColor, vec3(0.98, 0.95, 0.90), stormMask);

        // Great Red Spot
        surfColor = mix(surfColor, grsColor, grsMask * 0.85);
        // GRS inner detail: swirling spiral
        if(grsMask > 0.1) {
          float spiral = fbm(vec3(grsAngle * 6.0 + timeOff * 4.0, grsDist * 40.0, 0.0), 5);
          vec3 grsInner = mix(grsColor, vec3(0.90, 0.55, 0.30), spiral);
          surfColor = mix(surfColor, grsInner, grsMask * 0.5);
        }

        // Polar darkening
        float polarMask = smoothstep(0.6, 0.9, abs(lat));
        surfColor = mix(surfColor, polarColor, polarMask);

        // === Lighting ===
        float diff = max(dot(N, L), 0.0);
        float ambient = 0.06;
        float rim = 1.0 - max(dot(N, V), 0.0);
        float rimPow = pow(rim, 3.5);
        float NdotL = dot(N, L);
        vec3 atmosGlow = vec3(0.80, 0.65, 0.45) * rimPow * 0.35 * smoothstep(-0.1, 0.2, NdotL);

        vec3 result = surfColor * (ambient + diff * 0.85) + atmosGlow;
        FragColor = vec4(result, 1.0);
      }
    )";
    setupPlanetProg(jupiterProgram, u_jupiter, jupiterFragSrc);

    // ===== 6. SATURN SHADER =====
    // Muted gold/amber banding, hexagonal polar vortex, haze layer,
    // more subtle contrast than Jupiter.
    const char* saturnFragSrc = R"(
      #version 330 core
      in vec3 vWorldPos; in vec3 vNormal; in vec3 vLocalPos;
      uniform vec3 uLightDir; uniform vec3 uViewPos; uniform vec4 uBaseColor; uniform float uTime;
      out vec4 FragColor;

      float hash(vec3 p) { p=fract(p*vec3(443.897,441.423,437.195)); p+=dot(p,p.yzx+19.19); return fract((p.x+p.y)*p.z); }
      float noise3d(vec3 p) {
        vec3 i=floor(p); vec3 f=fract(p); f=f*f*f*(f*(f*6.0-15.0)+10.0);
        float n000=hash(i),n100=hash(i+vec3(1,0,0)),n010=hash(i+vec3(0,1,0)),n110=hash(i+vec3(1,1,0));
        float n001=hash(i+vec3(0,0,1)),n101=hash(i+vec3(1,0,1)),n011=hash(i+vec3(0,1,1)),n111=hash(i+vec3(1,1,1));
        float nx00=mix(n000,n100,f.x),nx10=mix(n010,n110,f.x);
        float nx01=mix(n001,n101,f.x),nx11=mix(n011,n111,f.x);
        return mix(mix(nx00,nx10,f.y),mix(nx01,nx11,f.y),f.z);
      }
      float fbm(vec3 p, int oct) { float v=0.0,a=0.5; for(int i=0;i<oct;i++){v+=noise3d(p)*a;p=p*2.07+vec3(0.131,-0.217,0.344);a*=0.48;} return v; }

      void main() {
        vec3 N = normalize(vNormal); vec3 L = normalize(uLightDir);
        vec3 V = normalize(uViewPos - vWorldPos);
        vec3 sph = normalize(vLocalPos);
        vec3 tc = sph * 5.0;
        float lat = sph.y;
        float timeOff = uTime * 0.00002;

        // === Subtle banding structure ===
        float bandWarp = fbm(sph * vec3(1.5, 0.5, 1.5) + vec3(timeOff * 0.3), 4);
        float bands = sin(lat * 18.0 + bandWarp * 3.0) * 0.5 + 0.5;
        float fineBands = sin(lat * 50.0 + bandWarp * 1.5 + 2.0) * 0.15;
        bands = clamp(bands + fineBands, 0.0, 1.0);

        // Turbulence (much more subtle than Jupiter)
        float turb = fbm(tc * 2.5 + vec3(timeOff, lat * 3.0, timeOff * 0.5), 6) * 0.2;

        // === Hexagonal polar vortex (North pole) ===
        float polarDist = length(sph - vec3(0.0, 1.0, 0.0));
        float hexAngle = atan(sph.z, sph.x);
        float hexPattern = cos(hexAngle * 6.0) * 0.5 + 0.5;
        float hexMask = smoothstep(0.45, 0.15, polarDist) * hexPattern;

        // === Color palette (Saturn is more muted, golden) ===
        vec3 goldZone = vec3(0.90, 0.82, 0.60);
        vec3 paleBelt = vec3(0.78, 0.70, 0.52);
        vec3 warmBelt = vec3(0.65, 0.52, 0.32);
        vec3 hexColor = vec3(0.55, 0.45, 0.30);
        vec3 polarColor = vec3(0.50, 0.42, 0.28);

        vec3 surfColor = mix(warmBelt, goldZone, bands);
        surfColor = mix(surfColor, paleBelt, turb);

        // Storm features (rare, white spots)
        float storm = noise3d(tc * 6.0 + vec3(timeOff * 3.0));
        float stormMask = smoothstep(0.78, 0.88, storm) * 0.4;
        surfColor = mix(surfColor, vec3(0.95, 0.92, 0.85), stormMask);

        // Hexagonal vortex
        surfColor = mix(surfColor, hexColor, hexMask * 0.5);

        // Polar regions
        float polarMask = smoothstep(0.65, 0.85, abs(lat));
        surfColor = mix(surfColor, polarColor, polarMask);

        // Atmospheric haze (Saturn has a thick haze that softens everything)
        float haze = smoothstep(0.0, 0.4, abs(lat));
        surfColor = mix(surfColor, goldZone * 1.05, (1.0 - haze) * 0.15);

        // === Lighting ===
        float diff = max(dot(N, L), 0.0);
        float ambient = 0.05;
        float rim = 1.0 - max(dot(N, V), 0.0);
        float rimPow = pow(rim, 3.0);
        float NdotL = dot(N, L);
        vec3 atmosGlow = vec3(0.88, 0.78, 0.55) * rimPow * 0.30 * smoothstep(-0.1, 0.2, NdotL);

        vec3 result = surfColor * (ambient + diff * 0.85) + atmosGlow;
        FragColor = vec4(result, 1.0);
      }
    )";
    setupPlanetProg(saturnProgram, u_saturn, saturnFragSrc);

    // ===== 7. URANUS SHADER =====
    // Pale teal/cyan from methane absorption, subtle banding, polar brightening,
    // nearly featureless appearance with very faint cloud structures.
    const char* uranusFragSrc = R"(
      #version 330 core
      in vec3 vWorldPos; in vec3 vNormal; in vec3 vLocalPos;
      uniform vec3 uLightDir; uniform vec3 uViewPos; uniform vec4 uBaseColor;
      out vec4 FragColor;

      float hash(vec3 p) { p=fract(p*vec3(443.897,441.423,437.195)); p+=dot(p,p.yzx+19.19); return fract((p.x+p.y)*p.z); }
      float noise3d(vec3 p) {
        vec3 i=floor(p); vec3 f=fract(p); f=f*f*f*(f*(f*6.0-15.0)+10.0);
        float n000=hash(i),n100=hash(i+vec3(1,0,0)),n010=hash(i+vec3(0,1,0)),n110=hash(i+vec3(1,1,0));
        float n001=hash(i+vec3(0,0,1)),n101=hash(i+vec3(1,0,1)),n011=hash(i+vec3(0,1,1)),n111=hash(i+vec3(1,1,1));
        float nx00=mix(n000,n100,f.x),nx10=mix(n010,n110,f.x);
        float nx01=mix(n001,n101,f.x),nx11=mix(n011,n111,f.x);
        return mix(mix(nx00,nx10,f.y),mix(nx01,nx11,f.y),f.z);
      }
      float fbm(vec3 p, int oct) { float v=0.0,a=0.5; for(int i=0;i<oct;i++){v+=noise3d(p)*a;p=p*2.07+vec3(0.131,-0.217,0.344);a*=0.48;} return v; }

      void main() {
        vec3 N = normalize(vNormal); vec3 L = normalize(uLightDir);
        vec3 V = normalize(uViewPos - vWorldPos);
        vec3 sph = normalize(vLocalPos);
        float lat = sph.y;

        // === Very subtle banding (Uranus appears almost featureless) ===
        float bandWarp = fbm(sph * vec3(1.0, 0.3, 1.0), 3) * 0.3;
        float bands = sin(lat * 12.0 + bandWarp * 2.0) * 0.5 + 0.5;

        // Faint methane cloud features
        float clouds = fbm(sph * 4.0, 5) * 0.08;

        // === Color palette ===
        vec3 paleTeal = vec3(0.58, 0.80, 0.85);
        vec3 deepCyan = vec3(0.45, 0.72, 0.78);
        vec3 brightPole = vec3(0.72, 0.88, 0.90);

        vec3 surfColor = mix(deepCyan, paleTeal, bands * 0.6);
        surfColor += vec3(clouds);

        // Polar brightening (southern pole was sunlit during Voyager flyby)
        float polarBright = smoothstep(0.5, 0.9, abs(lat));
        surfColor = mix(surfColor, brightPole, polarBright * 0.4);

        // === Lighting ===
        float diff = max(dot(N, L), 0.0);
        float ambient = 0.04;
        float rim = 1.0 - max(dot(N, V), 0.0);
        float rimPow = pow(rim, 3.5);
        float NdotL = dot(N, L);
        vec3 atmosGlow = vec3(0.55, 0.78, 0.85) * rimPow * 0.35 * smoothstep(-0.1, 0.2, NdotL);

        vec3 result = surfColor * (ambient + diff * 0.82) + atmosGlow;
        FragColor = vec4(result, 1.0);
      }
    )";
    setupPlanetProg(uranusProgram, u_uranus, uranusFragSrc);

    // ===== 8. NEPTUNE SHADER =====
    // Deep azure blue (strongest methane absorption), Great Dark Spot analog,
    // bright methane cirrus streaks, vivid blue-to-indigo band contrast,
    // dynamic cloud features.
    const char* neptuneFragSrc = R"(
      #version 330 core
      in vec3 vWorldPos; in vec3 vNormal; in vec3 vLocalPos;
      uniform vec3 uLightDir; uniform vec3 uViewPos; uniform vec4 uBaseColor; uniform float uTime;
      out vec4 FragColor;

      float hash(vec3 p) { p=fract(p*vec3(443.897,441.423,437.195)); p+=dot(p,p.yzx+19.19); return fract((p.x+p.y)*p.z); }
      float noise3d(vec3 p) {
        vec3 i=floor(p); vec3 f=fract(p); f=f*f*f*(f*(f*6.0-15.0)+10.0);
        float n000=hash(i),n100=hash(i+vec3(1,0,0)),n010=hash(i+vec3(0,1,0)),n110=hash(i+vec3(1,1,0));
        float n001=hash(i+vec3(0,0,1)),n101=hash(i+vec3(1,0,1)),n011=hash(i+vec3(0,1,1)),n111=hash(i+vec3(1,1,1));
        float nx00=mix(n000,n100,f.x),nx10=mix(n010,n110,f.x);
        float nx01=mix(n001,n101,f.x),nx11=mix(n011,n111,f.x);
        return mix(mix(nx00,nx10,f.y),mix(nx01,nx11,f.y),f.z);
      }
      float fbm(vec3 p, int oct) { float v=0.0,a=0.5; for(int i=0;i<oct;i++){v+=noise3d(p)*a;p=p*2.07+vec3(0.131,-0.217,0.344);a*=0.48;} return v; }

      void main() {
        vec3 N = normalize(vNormal); vec3 L = normalize(uLightDir);
        vec3 V = normalize(uViewPos - vWorldPos);
        vec3 sph = normalize(vLocalPos);
        vec3 tc = sph * 5.0;
        float lat = sph.y;
        float timeOff = uTime * 0.00004;

        // === Strong banding (Neptune has more visible features than Uranus) ===
        float bandWarp = fbm(sph * vec3(2.0, 1.0, 2.0) + vec3(timeOff * 0.5, 0.0, timeOff * 0.3), 5);
        float bands = sin(lat * 16.0 + bandWarp * 3.5) * 0.5 + 0.5;
        float fineBands = sin(lat * 40.0 + bandWarp * 1.5) * 0.1;
        bands = clamp(bands + fineBands, 0.0, 1.0);

        // === Great Dark Spot (GDS) analog ===
        vec3 gdsCenter = vec3(-0.3, -0.3, 0.5);
        float gdsDist = length(sph - normalize(gdsCenter));
        float gdsMask = smoothstep(0.2, 0.0, gdsDist);

        // === Methane cirrus streaks (bright white wispy clouds) ===
        float cirrus = fbm(sph * vec3(8.0, 2.0, 8.0) + vec3(timeOff * 3.0, lat * 5.0, timeOff * 1.5), 6);
        float cirrusMask = smoothstep(0.55, 0.75, cirrus) * 0.6;

        // === Color palette ===
        vec3 deepBlue = vec3(0.10, 0.15, 0.55);
        vec3 brightBlue = vec3(0.20, 0.35, 0.75);
        vec3 indigo = vec3(0.12, 0.10, 0.45);
        vec3 gdsColor = vec3(0.06, 0.08, 0.35); // Very dark blue
        vec3 cirrusWhite = vec3(0.85, 0.88, 0.95);
        vec3 polarDark = vec3(0.08, 0.10, 0.35);

        vec3 surfColor = mix(deepBlue, brightBlue, bands);
        surfColor = mix(surfColor, indigo, (1.0 - bands) * 0.3);

        // Wind-driven streaky detail
        float streaks = fbm(tc * 4.0 + vec3(timeOff * 2.0, 0.0, 0.0), 5) * 0.15;
        surfColor += vec3(streaks * 0.3, streaks * 0.4, streaks);

        // Great Dark Spot
        surfColor = mix(surfColor, gdsColor, gdsMask * 0.7);

        // Bright companion clouds near the GDS
        float companion = smoothstep(0.18, 0.22, gdsDist) * smoothstep(0.30, 0.22, gdsDist);
        companion *= fbm(sph * 12.0 + vec3(timeOff * 5.0), 4);
        surfColor = mix(surfColor, cirrusWhite, companion * 0.5);

        // Methane cirrus streaks
        surfColor = mix(surfColor, cirrusWhite, cirrusMask);

        // Polar darkening
        float polarMask = smoothstep(0.6, 0.85, abs(lat));
        surfColor = mix(surfColor, polarDark, polarMask * 0.5);

        // === Lighting ===
        float diff = max(dot(N, L), 0.0);
        float ambient = 0.04;
        float rim = 1.0 - max(dot(N, V), 0.0);
        float rimPow = pow(rim, 3.0);
        float NdotL = dot(N, L);
        vec3 atmosGlow = vec3(0.20, 0.35, 0.80) * rimPow * 0.40 * smoothstep(-0.1, 0.2, NdotL);

        vec3 result = surfColor * (ambient + diff * 0.82) + atmosGlow;
        FragColor = vec4(result, 1.0);
      }
    )";
    setupPlanetProg(neptuneProgram, u_neptune, neptuneFragSrc);

    // ===== Enhanced Ring Shader (Cassini Division, A/B/C rings) =====
    // Overwrite ring program with upgraded version
    const char* ringFragSrcV2 = R"(
      #version 330 core
      in vec3 vNormal; in vec3 vLocalPos; in vec3 vWorldPos;
      uniform vec3 uLightDir; uniform vec3 uViewPos; uniform vec4 uBaseColor;
      out vec4 FragColor;

      void main() {
        vec3 N = normalize(vNormal);
        vec3 L = normalize(uLightDir);
        vec3 V = normalize(uViewPos - vWorldPos);
        float diff = max(abs(dot(N, L)), 0.0); // Thin ring lit from both sides
        float ambient = 0.08;

        // Radial distance from center (0=inner, 1=outer)
        float dist = length(vLocalPos.xz);
        // Normalized ring position (innerRadius=1.0 of planet, rings span 1.0-2.2)
        float ringPos = (dist - 0.0) / 1.0; // Already in local space scaled

        // === Ring structure: A, B, C rings with Cassini Division ===
        // The mesh spans innerRadius to outerRadius (0 to 1 in V coordinate)
        // C ring: 0.0 - 0.25 (faint, dusty)
        // B ring: 0.25 - 0.55 (brightest, densest)
        // Cassini Division: 0.55 - 0.62 (nearly empty gap)
        // A ring: 0.62 - 0.90 (moderate density)
        // F ring: 0.90 - 1.0 (very thin, faint)

        float v = length(vLocalPos.xz); // Use actual distance
        // Normalize based on mesh geometry: inner=innerR, outer=outerR
        // The ring mesh spans [innerRadius, outerRadius], we use the UV.v for radial pos
        // Actually let's just use a simple approach based on distance pattern
        float band = sin(dist * 3.0) * cos(dist * 6.0 + 1.2);

        // Ring density profile
        float density = 0.0;
        float t = fract(dist); // Use fractional distance for ring bands

        // Create structured ring pattern
        float ring_t = dist; // Radial coordinate in ring mesh local coords

        // Multiple ring gaps and density variations
        float cassiniGap = smoothstep(0.45, 0.48, ring_t) * smoothstep(0.55, 0.52, ring_t);
        float enckeGap = smoothstep(0.72, 0.73, ring_t) * smoothstep(0.75, 0.74, ring_t);

        // Fine ring structure (many thin ringlets)
        float fineRings = sin(ring_t * 80.0) * 0.3 + sin(ring_t * 200.0) * 0.1;
        fineRings = fineRings * 0.5 + 0.5;

        // Overall density envelope
        float envelope = 0.0;
        if(ring_t < 0.3) envelope = ring_t / 0.3 * 0.3; // C ring (faint)
        else if(ring_t < 0.45) envelope = 0.9; // B ring (bright)
        else if(ring_t < 0.55) envelope = 0.05; // Cassini Division
        else if(ring_t < 0.85) envelope = 0.6; // A ring
        else envelope = max(0.0, 1.0 - (ring_t - 0.85) / 0.15) * 0.15; // F ring

        float alpha = envelope * fineRings;
        alpha = clamp(alpha, 0.0, 0.85);

        // Color: ice-white to dusty amber gradient
        vec3 iceWhite = vec3(0.88, 0.85, 0.80);
        vec3 dustyAmber = vec3(0.75, 0.65, 0.50);
        vec3 ringColor = mix(dustyAmber, iceWhite, envelope);

        // Back-lighting effect (rings glow when backlit)
        float NdotL = dot(N, L);
        float backlit = max(-NdotL, 0.0) * 0.3;

        vec3 result = ringColor * (ambient + diff * 0.8 + backlit);
        FragColor = vec4(result, alpha);
      }
    )";
    // Re-compile ring shader with enhanced version
    ringProgram = compileProgram(vertSrc, ringFragSrcV2);
    uri_mvp = glGetUniformLocation(ringProgram, "uMVP");
    uri_model = glGetUniformLocation(ringProgram, "uModel");
    uri_lightDir = glGetUniformLocation(ringProgram, "uLightDir");
    uri_viewPos = glGetUniformLocation(ringProgram, "uViewPos");
    uri_baseColor = glGetUniformLocation(ringProgram, "uBaseColor");



    // --- Billboard Shader (fire/glow particles, markers) ---
    const char* bbVertSrc = R"(
      #version 330 core
      layout(location=0) in vec2 aOffset;
      // Need View and Proj separately precisely for billboarding
      uniform mat4 uView;
      uniform mat4 uProj;
      uniform vec3 uCenter;
      uniform vec2 uSize;
      out vec2 vUV;
      void main() {
        vUV = aOffset * 0.5 + 0.5;
        // Exact spherical billboarding: extract camera right and up from View Matrix
        // uView[0][0], uView[1][0], uView[2][0] is the right vector
        // uView[0][1], uView[1][1], uView[2][1] is the up vector
        vec3 right = vec3(uView[0][0], uView[1][0], uView[2][0]);
        vec3 up    = vec3(uView[0][1], uView[1][1], uView[2][1]);
        
        // Compute world position of this vertex
        vec3 worldPos = uCenter + right * aOffset.x * uSize.x + up * aOffset.y * uSize.y;
        
        // Project to screen
        gl_Position = uProj * uView * vec4(worldPos, 1.0);
      }
    )";
    const char* bbFragSrc = R"(
      #version 330 core
      in vec2 vUV;
      uniform vec4 uColor;
      out vec4 FragColor;
      void main() {
        float d = distance(vUV, vec2(0.5));
        float alpha = smoothstep(0.5, 0.0, d);
        FragColor = vec4(uColor.rgb, uColor.a * alpha);
      }
    )";
    billboardProg = compileProgram(bbVertSrc, bbFragSrc);
    ub_vp = glGetUniformLocation(billboardProg, "uView"); // Actually uView now
    ub_proj = glGetUniformLocation(billboardProg, "uProj");
    ub_pos = glGetUniformLocation(billboardProg, "uCenter");
    ub_size = glGetUniformLocation(billboardProg, "uSize");
    ub_color = glGetUniformLocation(billboardProg, "uColor");

    // Billboard quad VAO
    float quad[] = { -1,-1,  1,-1,  -1,1,  1,1 };
    glGenVertexArrays(1, &billboardVAO);
    glGenBuffers(1, &billboardVBO);
    glBindVertexArray(billboardVAO);
    glBindBuffer(GL_ARRAY_BUFFER, billboardVBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(quad), quad, GL_STATIC_DRAW);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 0, 0);
    glEnableVertexAttribArray(0);
    glBindVertexArray(0);

    // --- Atmosphere Shader (Rayleigh + Mie Scattering) ---
    const char* atmoFragSrc = R"(
      #version 330 core
      in vec3 vWorldPos;
      in vec3 vNormal;
      in vec2 vUV;
      in vec4 vColor;
      uniform vec3 uLightDir;
      uniform vec3 uViewPos;
      out vec4 FragColor;
      void main() {
        vec3 N = normalize(vNormal);
        vec3 V = normalize(uViewPos - vWorldPos);
        vec3 L = normalize(uLightDir);

        // Geometry
        float NdotV = dot(N, V);
        float NdotL = dot(N, L);
        float VdotL = dot(V, L);

        // Rim intensity (atmosphere optical depth increases at limb)
        float rim = 1.0 - abs(NdotV);
        float rimSoft = pow(rim, 2.0);   // Outer glow visible at wider angles
        float rimHard = pow(rim, 5.0);   // Sharp inner limb

        // --- Rayleigh Scattering ---
        // Wavelength-dependent: blue strongest, red weakest
        vec3 rayleighCoeff = vec3(0.15, 0.35, 0.95); // λ^-4 approximation
        // Phase function: (1 + cos²θ) where θ = angle between view and light
        float cosTheta = VdotL;
        float rayleighPhase = 0.75 * (1.0 + cosTheta * cosTheta);
        vec3 rayleigh = rayleighCoeff * rayleighPhase;

        // --- Mie Scattering (forward-peaked haze) ---
        float g = 0.76; // Asymmetry factor
        float g2 = g * g;
        float miePhase = 1.5 * ((1.0 - g2) / (2.0 + g2)) *
                         (1.0 + cosTheta * cosTheta) /
                         pow(1.0 + g2 - 2.0 * g * cosTheta, 1.5);
        vec3 mieColor = vec3(0.95, 0.85, 0.7); // Warm white-gold haze
        vec3 mie = mieColor * miePhase * 0.15;

        // --- Sunset / Sunrise coloring ---
        // At low sun angles, blue is scattered out, leaving orange/red
        float sunsetFactor = smoothstep(-0.05, 0.15, NdotL) * smoothstep(0.35, 0.0, NdotL);
        vec3 sunsetColor = vec3(1.0, 0.30, 0.05);
        vec3 deepSunset = vec3(0.6, 0.1, 0.3); // Purple twilight

        // Combine scattering
        vec3 scatter = rayleigh + mie;
        // Add sunset warmth
        scatter += sunsetColor * sunsetFactor * 1.8;
        // Purple twilight at deep terminator
        float twilightZone = smoothstep(0.05, -0.08, NdotL) * smoothstep(-0.2, -0.05, NdotL);
        scatter += deepSunset * twilightZone * 0.6;

        // --- Visibility control ---
        // Atmosphere visible from day limb through twilight; fully dark on deep night side
        float visibility = smoothstep(-0.18, 0.05, NdotL);
        // Extra glow at backlit limb (light shining through atmosphere from behind)
        float backlitGlow = pow(max(0.0, -NdotV), 3.0) * smoothstep(-0.3, -0.05, NdotL) * 0.3;

        float alpha = (rimSoft * visibility + rimHard * 0.3 + backlitGlow) * 0.8;
        alpha = min(alpha, 0.92); // Never fully opaque

        vec3 finalColor = scatter;
        FragColor = vec4(finalColor, alpha);
      }
    )";
    atmoProg = compileProgram(vertSrc, atmoFragSrc);
    ua_mvp = glGetUniformLocation(atmoProg, "uMVP");
    ua_model = glGetUniformLocation(atmoProg, "uModel");
    ua_lightDir = glGetUniformLocation(atmoProg, "uLightDir");
    ua_viewPos = glGetUniformLocation(atmoProg, "uViewPos");

    // --- Ribbon Shader (Trajectory Trails) ---
    const char* ribbonVertSrc = R"(
      #version 330 core
      layout(location=0) in vec3 aPos;
      layout(location=1) in vec4 aColor;
      uniform mat4 uMVP;
      out vec4 vColor;
      void main() {
        vColor = aColor;
        gl_Position = uMVP * vec4(aPos, 1.0);
      }
    )";
    const char* ribbonFragSrc = R"(
      #version 330 core
      in vec4 vColor;
      out vec4 FragColor;
      void main() {
        FragColor = vColor;
      }
    )";
    ribbonProg = compileProgram(ribbonVertSrc, ribbonFragSrc);
    ur_mvp = glGetUniformLocation(ribbonProg, "uMVP");

    glGenVertexArrays(1, &ribbonVAO);
    glGenBuffers(1, &ribbonVBO);
    glBindVertexArray(ribbonVAO);
    glBindBuffer(GL_ARRAY_BUFFER, ribbonVBO);
    glBufferData(GL_ARRAY_BUFFER, (sizeof(Vec3) + sizeof(Vec4)) * 4000, nullptr, GL_DYNAMIC_DRAW);
    
    struct RibVert { Vec3 p; Vec4 c; };
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(RibVert), (void*)0);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, sizeof(RibVert), (void*)(sizeof(Vec3)));
    glEnableVertexAttribArray(1);
    glBindVertexArray(0);

    // --- Lens Flare Shader (Procedural 2D Screen-Space) ---
    const char* lfVertSrc = R"(
      #version 330 core
      layout(location=0) in vec2 aPos;
      
      uniform vec2 uSunScreenPos; // Position of sun in NDC (-1 to 1)
      uniform float uAspect;      // Screen aspect ratio
      uniform vec2 uScale;        // Scale of this specific flare element
      uniform vec2 uOffset;       // Screen-space offset relative to sun or center
      
      out vec2 vUV;
      
      void main() {
        vUV = aPos * 0.5 + 0.5; // 0 to 1 UVs
        // Base quad is -1 to 1. Scale it.
        vec2 pos = aPos * uScale;
        // Fix aspect ratio so circles are round
        pos.x /= uAspect;
        // Apply offset (center of the element)
        gl_Position = vec4(pos + uOffset, 0.0, 1.0);
      }
    )";
    const char* lfFragSrc = R"(
      #version 330 core
      in vec2 vUV;
      
      uniform vec4 uColor;
      uniform float uIntensity;
      uniform int uShapeType; // 0=glow, 1=anamorphic, 2=ghost, 3=starburst
      
      out vec4 FragColor;
      
      void main() {
        vec2 rUV = vUV * 2.0 - 1.0;
        float d = length(rUV);
        // Edge kill: smoothly fade to zero near quad boundary - prevents visible rectangle
        float edgeFade = smoothstep(1.0, 0.65, d);
        float alpha = 0.0;
        
        if (uShapeType == 0) {
           // Clean circular glow with gaussian falloff
           float core = exp(-d * d * 12.0) * 2.5;
           float mid  = exp(-d * d * 3.0) * 0.6;
           float wide = exp(-d * d * 0.8) * 0.15;
           alpha = (core + mid + wide) * edgeFade;
           
           // 4-point diffraction spikes (like real camera lens)
           float angle = atan(rUV.y, rUV.x);
           float spike1 = pow(abs(cos(angle)), 300.0);
           float spike2 = pow(abs(sin(angle)), 300.0);
           float spikes = (spike1 + spike2) * exp(-d * 1.8) * 0.4;
           alpha += spikes * edgeFade;
           
        } else if (uShapeType == 1) {
           // Anamorphic horizontal streak - soft edges
           float dx = abs(rUV.x);
           float dy = abs(rUV.y);
           float edgeFadeX = smoothstep(1.0, 0.7, dx);
           alpha = exp(-dx * dx * 0.5) * exp(-dy * dy * 600.0) * edgeFadeX;
           alpha += exp(-dx * dx * 1.5) * exp(-dy * dy * 2000.0) * 0.6 * edgeFadeX;
           
        } else if (uShapeType == 2) {
           // Subtle ghost disk (not hard hexagon ring - those look fake)
           float softDisk = exp(-d * d * 4.0) * 0.4;
           float ring = smoothstep(0.95, 0.75, d) * smoothstep(0.55, 0.7, d) * 0.3;
           alpha = (softDisk + ring) * edgeFade;
           
        } else if (uShapeType == 3) {
           // Starburst: very subtle radial rays
           float angle = atan(rUV.y, rUV.x);
           float rays = pow(abs(sin(angle * 6.0)), 80.0) * 0.4;
           rays += pow(abs(cos(angle * 8.0 + 0.3)), 120.0) * 0.2;
           alpha = rays * exp(-d * d * 3.0) * edgeFade;
           alpha += exp(-d * d * 10.0) * 0.2 * edgeFade;
        }
        
        FragColor = vec4(uColor.rgb, uColor.a * alpha * uIntensity);
      }
    )";
    
    lensFlareProg = compileProgram(lfVertSrc, lfFragSrc);
    ulf_sunScreenPos = glGetUniformLocation(lensFlareProg, "uSunScreenPos");
    ulf_aspect = glGetUniformLocation(lensFlareProg, "uAspect");
    ulf_color = glGetUniformLocation(lensFlareProg, "uColor");
    ulf_intensity = glGetUniformLocation(lensFlareProg, "uIntensity");

    // Re-use billboard VAO/VBO for Lens Flare since both are just 2D quads
    lfVAO = billboardVAO;
    lfVBO = billboardVBO;

    // --- Skybox Shader (Procedural Starfield + Milky Way) ---
    const char* skyboxVertSrc = R"(
      #version 330 core
      layout(location=0) in vec2 aPos;
      uniform mat4 uInvViewProj;
      out vec3 vRayDir;
      void main() {
        gl_Position = vec4(aPos, 0.999, 1.0);
        // Reconstruct world-space ray direction from NDC
        vec4 worldPos = uInvViewProj * vec4(aPos, 1.0, 1.0);
        vRayDir = worldPos.xyz / worldPos.w;
      }
    )";
    const char* skyboxFragSrc = R"(
      #version 330 core
      in vec3 vRayDir;
      out vec4 FragColor;

      // --- High quality noise ---
      float hash13(vec3 p) {
        p = fract(p * vec3(443.897, 441.423, 437.195));
        p += dot(p, p.yzx + 19.19);
        return fract((p.x + p.y) * p.z);
      }

      float noise3d(vec3 p) {
        vec3 i = floor(p); vec3 f = fract(p);
        f = f*f*f*(f*(f*6.0-15.0)+10.0);
        float a = hash13(i), b = hash13(i+vec3(1,0,0));
        float c = hash13(i+vec3(0,1,0)), d = hash13(i+vec3(1,1,0));
        float e = hash13(i+vec3(0,0,1)), f1 = hash13(i+vec3(1,0,1));
        float g = hash13(i+vec3(0,1,1)), h = hash13(i+vec3(1,1,1));
        return mix(mix(mix(a,b,f.x),mix(c,d,f.x),f.y),
                   mix(mix(e,f1,f.x),mix(g,h,f.x),f.y),f.z);
      }

      float fbm(vec3 p, int oct) {
        float v = 0.0, a = 0.5;
        for(int i=0; i<oct; i++) { v += noise3d(p)*a; p = p*2.03+vec3(.31,-.17,.44); a *= 0.49; }
        return v;
      }

      void main() {
        vec3 rd = normalize(vRayDir);
        vec3 col = vec3(0.0);

        // === STAR FIELD (4 density layers) ===
        for (int layer = 0; layer < 4; layer++) {
          float cellSize = 60.0 + float(layer) * 100.0;
          vec3 cell = floor(rd * cellSize);
          vec3 localPos = fract(rd * cellSize) - 0.5;

          float sx = hash13(cell + vec3(0,0,float(layer)*17.0)) - 0.5;
          float sy = hash13(cell + vec3(1,0,float(layer)*17.0)) - 0.5;
          float sz = hash13(cell + vec3(0,1,float(layer)*17.0)) - 0.5;
          vec3 starPos = vec3(sx, sy, sz) * 0.8;
          float dist = length(localPos - starPos);
          float mag = hash13(cell + vec3(2,3,float(layer)*7.0));

          if (mag > 0.68) {
            float brt = (mag - 0.68) / 0.32;
            brt = brt * brt * brt * 3.0;
            float sz2 = 0.006 + brt * 0.012;
            float star = smoothstep(sz2, sz2 * 0.1, dist);

            // Spectral class coloring (O B A F G K M)
            float temp = hash13(cell + vec3(5,7,float(layer)));
            vec3 sc;
            if (temp < 0.08) sc = vec3(0.6, 0.7, 1.0);       // O/B blue
            else if (temp < 0.25) sc = vec3(0.75, 0.85, 1.0); // A blue-white
            else if (temp < 0.50) sc = vec3(1.0, 0.98, 0.95); // F/G white
            else if (temp < 0.75) sc = vec3(1.0, 0.90, 0.70); // K yellow
            else sc = vec3(1.0, 0.65, 0.35);                  // M red

            // Diffraction cross for brightest stars
            float cross = 0.0;
            if (brt > 0.7) {
              float dx = abs(localPos.x - starPos.x);
              float dy = abs(localPos.y - starPos.y);
              cross = exp(-dx*80.0)*exp(-dy*300.0) + exp(-dy*80.0)*exp(-dx*300.0);
              cross *= (brt - 0.7) * 1.5;
            }

            col += sc * (star * brt + cross * 0.4);
          }
        }

        // === MILKY WAY (smooth noise-based) ===
        // Galactic coordinate transform (tilted 62.87° from equatorial)
        float sinGal = 0.8829; float cosGal = 0.4695;
        float galLat = rd.y * cosGal - (rd.x * 0.6 + rd.z * 0.35) * sinGal;
        float galLon = atan(rd.z, rd.x);

        // Gaussian band with varying width
        float bandWidth = 0.08 + 0.04 * sin(galLon * 2.0);
        float band = exp(-galLat * galLat / (2.0 * bandWidth * bandWidth));

        // Smooth nebulosity using FBM noise
        vec3 galCoord = rd * 5.0;
        float neb = fbm(galCoord * 1.5, 6);
        float nebDetail = fbm(galCoord * 4.0 + vec3(100.0), 5);

        // Dark dust lanes (absorption)
        float dust = fbm(galCoord * 3.0 + vec3(50.0, -30.0, 20.0), 5);
        float dustAbsorption = smoothstep(0.35, 0.65, dust) * band;

        // Milky Way emission
        float galEmission = neb * band * 0.4;
        galEmission += nebDetail * band * 0.15;
        galEmission *= (1.0 - dustAbsorption * 0.7);

        // Color: warm brown core with amber nebulae (matching RSS-Reborn)
        vec3 galCore = vec3(0.20, 0.14, 0.08) * galEmission;
        vec3 warmNeb = vec3(0.25, 0.12, 0.05) * nebDetail * band * 0.15;
        vec3 blueNeb = vec3(0.08, 0.06, 0.12) * smoothstep(0.5, 0.8, neb) * band * 0.10;

        // Dense star clusters within the band
        for (int cl = 0; cl < 2; cl++) {
          float cs = 300.0 + float(cl) * 200.0;
          vec3 cc = floor(rd * cs);
          float cm = hash13(cc + vec3(99.0 + float(cl)*50.0));
          if (cm > 0.90 && band > 0.15) {
            vec3 lp = fract(rd * cs) - 0.5;
            float cd = length(lp);
            col += vec3(1.0, 0.90, 0.70) * smoothstep(0.015, 0.0, cd) * (cm-0.90)*12.0 * band;
          }
        }

        col += galCore + warmNeb + blueNeb;

        // Very subtle cosmic background
        col += vec3(0.001, 0.002, 0.004);

        // Tone mapping for HDR stars
        col = col / (col + 0.8);

        FragColor = vec4(col, 1.0);
      }
    )";
    skyboxProg = compileProgram(skyboxVertSrc, skyboxFragSrc);
    us_invViewProj = glGetUniformLocation(skyboxProg, "uInvViewProj");

    // Fullscreen quad VAO for skybox
    float fsQuad[] = { -1,-1,  1,-1,  -1,1,  1,1 };
    glGenVertexArrays(1, &skyboxVAO);
    glGenBuffers(1, &skyboxVBO);
    glBindVertexArray(skyboxVAO);
    glBindBuffer(GL_ARRAY_BUFFER, skyboxVBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(fsQuad), fsQuad, GL_STATIC_DRAW);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 0, 0);
    glEnableVertexAttribArray(0);
    glBindVertexArray(0);
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

  // ==== Procedural Screen Space Lens Flare (Feature 5) ====
  void drawSunAndFlare(const Vec3& sunWorldPos, const std::vector<Vec4>& occluders, int screenW, int screenH) {
     // 1. Ray-Sphere Occlusion Test against occluders (is the sun behind a planet?)
     Vec3 rayDir = (sunWorldPos - camPos).normalized();
     float occlusionFade = 1.0f;
     
     for (const auto& occ : occluders) {
         Vec3 occPos(occ.x, occ.y, occ.z);
         float occRadius = occ.w;
         
         Vec3 oc = camPos - occPos;
         float b = 2.0f * oc.dot(rayDir);
         float c = oc.dot(oc) - occRadius * occRadius;
         float discriminant = b * b - 4 * c;
         
         float t_closest = -oc.dot(rayDir);
         float localOccFade = 1.0f;
         
         if (t_closest >= 0) {
             if (discriminant > 0) {
                 float t1 = t_closest - sqrtf(discriminant) / 2.0f;
                 float sunDist = (sunWorldPos - camPos).length();
                 if (t1 > 0 && t1 < sunDist) {
                     localOccFade = 0.0f; // Eclipsed entirely by planet core
                 }
             } else {
                 // Ray clears planet. How close did it get to the surface?
                 float passDist = oc.cross(rayDir).length();
                 float distToLimb = passDist - occRadius;
                 // Fade smoothly through the thin upper atmosphere
                 if (distToLimb >= 0.0f && distToLimb < occRadius * 0.05f) {
                     localOccFade *= distToLimb / (occRadius * 0.05f);
                 }
             }
         }
         occlusionFade = fminf(occlusionFade, localOccFade);
         if (occlusionFade <= 0.01f) break; // Completely hidden
     }

     if (occlusionFade <= 0.01f) return;

     // 2. Compute Screen Space (NDC) position of the sun
     // We manually multiply the 4D vector to avoid needing a Vec4 class
     Mat4 vp = proj * view;
     float cx = vp.m[0]*sunWorldPos.x + vp.m[4]*sunWorldPos.y + vp.m[8]*sunWorldPos.z + vp.m[12];
     float cy = vp.m[1]*sunWorldPos.x + vp.m[5]*sunWorldPos.y + vp.m[9]*sunWorldPos.z + vp.m[13];
     float cz = vp.m[2]*sunWorldPos.x + vp.m[6]*sunWorldPos.y + vp.m[10]*sunWorldPos.z + vp.m[14];
     float cw = vp.m[3]*sunWorldPos.x + vp.m[7]*sunWorldPos.y + vp.m[11]*sunWorldPos.z + vp.m[15];
     
     if (cw <= 0.0f) return; // Behind camera
     
     Vec3 ndcPos = Vec3(cx / cw, cy / cw, cz / cw);
     
     // If far off screen, do not draw flares (increased bounds to prevent sudden popping)
     if (fabs(ndcPos.x) > 3.5f || fabs(ndcPos.y) > 3.5f) return;

     // 3. Render Setup
     glUseProgram(lensFlareProg);
     glUniform2f(ulf_sunScreenPos, ndcPos.x, ndcPos.y);
     float aspect = (float)screenW / (float)screenH;
     glUniform1f(ulf_aspect, aspect);
     
     glBlendFunc(GL_SRC_ALPHA, GL_ONE); // Additive blending for light
     glDepthMask(GL_FALSE);
     glDisable(GL_DEPTH_TEST);
     glDisable(GL_CULL_FACE);

     glBindVertexArray(lfVAO);
     glBindBuffer(GL_ARRAY_BUFFER, lfVBO);
     glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 0, (void*)0);
     glEnableVertexAttribArray(0);

     // Define a helper to draw one flare element
     // Let's get locations inside the lambda for safety
     auto drawFlare = [&](int shape, float scaleX, float scaleY, float ndcOffsetMult, 
                              float r, float g, float b, float aFactor) {
          glUniform1i(glGetUniformLocation(lensFlareProg, "uShapeType"), shape);
          glUniform2f(glGetUniformLocation(lensFlareProg, "uScale"), scaleX, scaleY);
          glUniform2f(glGetUniformLocation(lensFlareProg, "uOffset"), ndcPos.x * ndcOffsetMult, ndcPos.y * ndcOffsetMult);
          glUniform4f(ulf_color, r, g, b, 1.0f);
          glUniform1f(ulf_intensity, aFactor * occlusionFade);
          glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
     };

     // Clean sun glow (reference: small bright core, soft warm halo)
     drawFlare(0, 0.12f, 0.12f, 1.0f,  1.0f, 0.98f, 0.95f, 2.5f); // Hot white core
     drawFlare(0, 0.30f, 0.30f, 1.0f,  1.0f, 0.92f, 0.75f, 0.8f); // Warm halo
     drawFlare(0, 0.65f, 0.65f, 1.0f,  0.9f, 0.75f, 0.55f, 0.2f); // Faint warm bloom

     // Subtle anamorphic streak (thin, barely visible)
     drawFlare(1, 3.0f, 0.025f, 1.0f,  0.7f, 0.8f, 1.0f, 0.35f); // Faint blue streak
     drawFlare(1, 1.5f, 0.01f, 1.0f,   1.0f, 0.95f, 0.9f, 0.5f);  // White core streak

     // Very subtle ghost disks (barely noticeable)
     drawFlare(2, 0.08f, 0.08f, -0.35f, 0.7f, 0.85f, 1.0f, 0.08f);
     drawFlare(2, 0.12f, 0.12f, -0.65f, 1.0f, 0.8f, 0.6f, 0.05f);
     drawFlare(2, 0.05f, 0.05f, -1.0f,  0.6f, 0.7f, 1.0f, 0.06f);

     glBindVertexArray(0);

     glEnable(GL_DEPTH_TEST);
     glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
     glDepthMask(GL_TRUE);
  }

  void drawPlanet(const Mesh& mesh, const Mat4& model, BodyType type, float cr, float cg, float cb, float ca, float time = 0.0f, int bodyIdx = -1) {
    GLuint prog = earthProgram;
    GLint mvpLoc = ue_mvp, modelLoc = ue_model, lightLoc = ue_lightDir, viewLoc = ue_viewPos, colorLoc = -1, timeLoc = ue_time;

    // === RSS-Reborn: Per-planet shader routing by body index ===
    PlanetUniforms* pu = nullptr;
    switch (bodyIdx) {
      case 1: prog = mercuryProgram; pu = &u_mercury; break; // Mercury
      case 2: prog = venusProgram;   pu = &u_venus;   break; // Venus
      case 3: prog = earthProgram;   break;                   // Earth (original)
      case 4: prog = moonProgram;    pu = &u_moon;    break; // Moon
      case 5: prog = marsProgram;    pu = &u_mars;    break; // Mars
      case 6: prog = jupiterProgram; pu = &u_jupiter; break; // Jupiter
      case 7: prog = saturnProgram;  pu = &u_saturn;  break; // Saturn
      case 8: prog = uranusProgram;  pu = &u_uranus;  break; // Uranus
      case 9: prog = neptuneProgram; pu = &u_neptune; break; // Neptune
      default: // Fallback to old type-based routing
        if (type == GAS_GIANT || type == RINGED_GAS_GIANT) {
            prog = gasGiantProgram;
            mvpLoc = ugg_mvp; modelLoc = ugg_model; lightLoc = ugg_lightDir; viewLoc = ugg_viewPos; colorLoc = ugg_baseColor;
        } else if (type == MOON || (type == TERRESTRIAL && (cr != 0.2f || cg != 0.5f))) {
            prog = barrenProgram;
            mvpLoc = uba_mvp; modelLoc = uba_model; lightLoc = uba_lightDir; viewLoc = uba_viewPos; colorLoc = uba_baseColor;
        }
        break;
    }
    if (pu) {
        mvpLoc = pu->mvp; modelLoc = pu->model; lightLoc = pu->lightDir;
        viewLoc = pu->viewPos; colorLoc = pu->baseColor; timeLoc = pu->time;
    }

    glUseProgram(prog);
    Mat4 mvp = proj * view * model;
    glUniformMatrix4fv(mvpLoc, 1, GL_FALSE, mvp.m);
    glUniformMatrix4fv(modelLoc, 1, GL_FALSE, model.m);
    glUniform3f(lightLoc, lightDir.x, lightDir.y, lightDir.z);
    if (viewLoc != -1) glUniform3f(viewLoc, camPos.x, camPos.y, camPos.z);
    if (colorLoc != -1) glUniform4f(colorLoc, cr, cg, cb, ca);
    // Pass time uniform for animated shaders (Earth clouds, Venus atmosphere, etc.)
    if (timeLoc != -1) glUniform1f(timeLoc, time);
    
    mesh.draw();
  }

  void drawRing(const Mesh& ringMesh, const Mat4& model, float cr, float cg, float cb, float ca) {
    glUseProgram(ringProgram);
    Mat4 mvp = proj * view * model;
    glUniformMatrix4fv(uri_mvp, 1, GL_FALSE, mvp.m);
    glUniformMatrix4fv(uri_model, 1, GL_FALSE, model.m);
    glUniform3f(uri_lightDir, lightDir.x, lightDir.y, lightDir.z);
    glUniform4f(uri_baseColor, cr, cg, cb, ca);
    
    glDisable(GL_CULL_FACE);
    ringMesh.draw();
    glEnable(GL_CULL_FACE);
  }

  // 火焰/发光 Billboard (面向相机的面片)
  void drawBillboard(const Vec3& worldPos, float size,
                     float cr, float cg, float cb, float ca) {
    glUseProgram(billboardProg);
    
    // Provide View and Proj separately
    glUniformMatrix4fv(ub_vp, 1, GL_FALSE, view.m);
    glUniformMatrix4fv(ub_proj, 1, GL_FALSE, proj.m);
    
    glUniform3f(ub_pos, worldPos.x, worldPos.y, worldPos.z);
    glUniform2f(ub_size, size, size);
    glUniform4f(ub_color, cr, cg, cb, ca);
    
    // 加法混合 (火焰发光)
    glBlendFunc(GL_SRC_ALPHA, GL_ONE);
    glDepthMask(GL_FALSE);
    glDisable(GL_CULL_FACE); // 防止视口翻转导致背面剔除
    
    glBindVertexArray(billboardVAO);
    glBindBuffer(GL_ARRAY_BUFFER, billboardVBO);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 0, (void*)0);
    glEnableVertexAttribArray(0);

    glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
    
    glBindVertexArray(0);
    glEnable(GL_CULL_FACE);
    glDepthMask(GL_TRUE);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  }

  // 大气层散射壳
  void drawAtmosphere(const Mesh& sphereMesh, const Mat4& model) {
    glUseProgram(atmoProg);
    Mat4 mvp = proj * view * model;
    glUniformMatrix4fv(ua_mvp, 1, GL_FALSE, mvp.m);
    glUniformMatrix4fv(ua_model, 1, GL_FALSE, model.m);
    glUniform3f(ua_lightDir, lightDir.x, lightDir.y, lightDir.z);
    glUniform3f(ua_viewPos, camPos.x, camPos.y, camPos.z);
    // 加法混合 (大气光晕)
    glBlendFunc(GL_SRC_ALPHA, GL_ONE);
    glDepthMask(GL_FALSE);
    glCullFace(GL_FRONT); // 只绘制内表面
    glEnable(GL_CULL_FACE);
    sphereMesh.draw();
    glDisable(GL_CULL_FACE);
    glDepthMask(GL_TRUE);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  }

  // Procedural Starfield + Milky Way background
  void drawSkybox() {
    glUseProgram(skyboxProg);
    // Compute inverse view-projection matrix for ray reconstruction
    Mat4 vp = proj * view;
    // Simple 4x4 matrix inverse (brute force cofactor expansion)
    float inv[16];
    float* m = vp.m;
    float det;
    inv[0] = m[5]*m[10]*m[15] - m[5]*m[11]*m[14] - m[9]*m[6]*m[15] + m[9]*m[7]*m[14] + m[13]*m[6]*m[11] - m[13]*m[7]*m[10];
    inv[4] = -m[4]*m[10]*m[15] + m[4]*m[11]*m[14] + m[8]*m[6]*m[15] - m[8]*m[7]*m[14] - m[12]*m[6]*m[11] + m[12]*m[7]*m[10];
    inv[8] = m[4]*m[9]*m[15] - m[4]*m[11]*m[13] - m[8]*m[5]*m[15] + m[8]*m[7]*m[13] + m[12]*m[5]*m[11] - m[12]*m[7]*m[9];
    inv[12] = -m[4]*m[9]*m[14] + m[4]*m[10]*m[13] + m[8]*m[5]*m[14] - m[8]*m[6]*m[13] - m[12]*m[5]*m[10] + m[12]*m[6]*m[9];
    inv[1] = -m[1]*m[10]*m[15] + m[1]*m[11]*m[14] + m[9]*m[2]*m[15] - m[9]*m[3]*m[14] - m[13]*m[2]*m[11] + m[13]*m[3]*m[10];
    inv[5] = m[0]*m[10]*m[15] - m[0]*m[11]*m[14] - m[8]*m[2]*m[15] + m[8]*m[3]*m[14] + m[12]*m[2]*m[11] - m[12]*m[3]*m[10];
    inv[9] = -m[0]*m[9]*m[15] + m[0]*m[11]*m[13] + m[8]*m[1]*m[15] - m[8]*m[3]*m[13] - m[12]*m[1]*m[11] + m[12]*m[3]*m[9];
    inv[13] = m[0]*m[9]*m[14] - m[0]*m[10]*m[13] - m[8]*m[1]*m[14] + m[8]*m[2]*m[13] + m[12]*m[1]*m[10] - m[12]*m[2]*m[9];
    inv[2] = m[1]*m[6]*m[15] - m[1]*m[7]*m[14] - m[5]*m[2]*m[15] + m[5]*m[3]*m[14] + m[13]*m[2]*m[7] - m[13]*m[3]*m[6];
    inv[6] = -m[0]*m[6]*m[15] + m[0]*m[7]*m[14] + m[4]*m[2]*m[15] - m[4]*m[3]*m[14] - m[12]*m[2]*m[7] + m[12]*m[3]*m[6];
    inv[10] = m[0]*m[5]*m[15] - m[0]*m[7]*m[13] - m[4]*m[1]*m[15] + m[4]*m[3]*m[13] + m[12]*m[1]*m[7] - m[12]*m[3]*m[5];
    inv[14] = -m[0]*m[5]*m[14] + m[0]*m[6]*m[13] + m[4]*m[1]*m[14] - m[4]*m[2]*m[13] - m[12]*m[1]*m[6] + m[12]*m[2]*m[5];
    inv[3] = -m[1]*m[6]*m[11] + m[1]*m[7]*m[10] + m[5]*m[2]*m[11] - m[5]*m[3]*m[10] - m[9]*m[2]*m[7] + m[9]*m[3]*m[6];
    inv[7] = m[0]*m[6]*m[11] - m[0]*m[7]*m[10] - m[4]*m[2]*m[11] + m[4]*m[3]*m[10] + m[8]*m[2]*m[7] - m[8]*m[3]*m[6];
    inv[11] = -m[0]*m[5]*m[11] + m[0]*m[7]*m[9] + m[4]*m[1]*m[11] - m[4]*m[3]*m[9] - m[8]*m[1]*m[7] + m[8]*m[3]*m[5];
    inv[15] = m[0]*m[5]*m[10] - m[0]*m[6]*m[9] - m[4]*m[1]*m[10] + m[4]*m[2]*m[9] + m[8]*m[1]*m[6] - m[8]*m[2]*m[5];
    det = m[0]*inv[0] + m[1]*inv[4] + m[2]*inv[8] + m[3]*inv[12];
    if (fabsf(det) > 1e-12f) {
      det = 1.0f / det;
      for (int i = 0; i < 16; i++) inv[i] *= det;
    }

    glUniformMatrix4fv(us_invViewProj, 1, GL_FALSE, inv);

    glDisable(GL_DEPTH_TEST);
    glDepthMask(GL_FALSE);
    glDisable(GL_CULL_FACE);

    glBindVertexArray(skyboxVAO);
    glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
    glBindVertexArray(0);

    glEnable(GL_DEPTH_TEST);
    glDepthMask(GL_TRUE);
  }

  // Camera-facing dynamic ribbon (trajectory trail) with per-vertex colors
  void drawRibbon(const std::vector<Vec3>& points, const std::vector<Vec4>& colors, float width) {
    if (points.size() < 2 || points.size() != colors.size()) return;

    struct RibVert { Vec3 p; Vec4 c; };
    std::vector<RibVert> stripVerts;
    stripVerts.reserve(points.size() * 2);

    for (size_t i = 0; i < points.size(); ++i) {
      Vec3 p = points[i];
      
      Vec3 forward;
      if (i < points.size() - 1) {
        forward = (points[i+1] - p);
      } else {
        forward = (p - points[i-1]);
      }
      if (forward.length() < 1e-6f) forward = Vec3(0.0f, 1.0f, 0.0f); else forward = forward.normalized();

      Vec3 toCam = (camPos - p);
      if (toCam.length() < 1e-6f) toCam = Vec3(1.0f, 0.0f, 0.0f); else toCam = toCam.normalized();

      Vec3 right = forward.cross(toCam);
      if (right.length() < 1e-6f) {
          right = forward.cross(Vec3(0.0f, 1.0f, 0.0f));
          if (right.length() < 1e-6f) right = forward.cross(Vec3(1.0f, 0.0f, 0.0f));
      }
      right = right.normalized();

      float halfW = width * 0.5f;
      // Zigzag for triangle strip: first left, then right
      stripVerts.push_back({p + right * halfW, colors[i]});
      stripVerts.push_back({p - right * halfW, colors[i]});
    }

    glUseProgram(ribbonProg);
    Mat4 mvp = proj * view; // No model matrix needed, points are in world space
    glUniformMatrix4fv(ur_mvp, 1, GL_FALSE, mvp.m);

    // Buffer dynamic data
    glBindVertexArray(ribbonVAO);
    glBindBuffer(GL_ARRAY_BUFFER, ribbonVBO);
    glBufferData(GL_ARRAY_BUFFER, stripVerts.size() * sizeof(RibVert), stripVerts.data(), GL_DYNAMIC_DRAW);
    
    // Re-establish vertex layout
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(RibVert), (void*)0);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, sizeof(RibVert), (void*)(sizeof(Vec3)));
    glEnableVertexAttribArray(1);

    glDisable(GL_CULL_FACE);
    glDepthMask(GL_FALSE);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE); // Additive blending for glows

    glDrawArrays(GL_TRIANGLE_STRIP, 0, (GLsizei)stripVerts.size());

    glEnable(GL_CULL_FACE);
    glDepthMask(GL_TRUE);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glBindVertexArray(0);
  }

  // Convenience overload: single uniform color for all points
  void drawRibbon(const std::vector<Vec3>& points, float width,
                  float cr, float cg, float cb, float ca) {
    std::vector<Vec4> colors(points.size(), Vec4(cr, cg, cb, ca));
    drawRibbon(points, colors, width);
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
