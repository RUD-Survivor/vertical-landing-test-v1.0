#pragma once
// ==========================================================
// renderer3d.h — 3D Rendering Pipeline
// Requires: glad, math3d.h
// ==========================================================

#include "math/math3d.h"
#include "core/rocket_state.h"
// NOTE: glad must be included BEFORE this header in the main translation unit
#include <vector>
#include <string>
#include <map>
#include <fstream>
#include "terrain_system.h"
#include "vegetation_system.h"
#include "svo_system.h"
#include "svo_meshing.h"

inline void sendMat4(GLint loc, const Mat4& m) {
    float arr[16];
    m.toFloatArray(arr);
    glUniformMatrix4fv(loc, 1, GL_FALSE, arr);
}

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
// Texture — 2D 纹理对象
// ==========================================================
struct Texture {
  GLuint id = 0;
  int width = 0, height = 0;

  void destroy() {
    if (id) glDeleteTextures(1, &id);
    id = 0;
  }

  void bind(GLuint slot = 0) const {
    glActiveTexture(GL_TEXTURE0 + slot);
    glBindTexture(GL_TEXTURE_2D, id);
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

// 基础平面 Patch (用于地形, 范围 [-0.5, 0.5])
inline Mesh patch(int units) {
  std::vector<Vertex3D> verts;
  std::vector<unsigned int> indices;
  float step = 1.0f / (float)units;
  for (int z = 0; z <= units; z++) {
    for (int x = 0; x <= units; x++) {
      Vertex3D v;
      v.px = (float)x * step - 0.5f;
      v.py = 0;
      v.pz = (float)z * step - 0.5f;
      v.nx = 0; v.ny = 1; v.nz = 0;
      v.u = (float)x * step;
      v.v = (float)z * step;
      v.r = 1; v.g = 1; v.b = 1; v.a = 1;
      verts.push_back(v);
    }
  }
  for (int z = 0; z < units; z++) {
    for (int x = 0; x < units; x++) {
      int i0 = z * (units + 1) + x;
      int i1 = i0 + 1;
      int i2 = i0 + (units + 1);
      int i3 = i2 + 1;
      indices.push_back(i0); indices.push_back(i2); indices.push_back(i1);
      indices.push_back(i1); indices.push_back(i2); indices.push_back(i3);
    }
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
  // Simple TGA Loader (for uncompressed 24/32-bit TGA)
  static Texture loadTGA(const char* filepath) {
    Texture tex;
    std::ifstream file(filepath, std::ios::binary);
    if (!file.is_open()) return tex;

    unsigned char header[18];
    file.read((char*)header, 18);
    
    int w = header[12] | (header[13] << 8);
    int h = header[14] | (header[15] << 8);
    int bpp = header[16];

    if (bpp != 24 && bpp != 32) return tex;

    int size = w * h * (bpp / 8);
    std::vector<unsigned char> data(size);
    file.read((char*)data.data(), size);

    // BGR(A) to RGB(A)
    for (int i = 0; i < size; i += (bpp / 8)) {
        unsigned char tmp = data[i];
        data[i] = data[i + 2];
        data[i + 2] = tmp;
    }

    glGenTextures(1, &tex.id);
    glBindTexture(GL_TEXTURE_2D, tex.id);
    glTexImage2D(GL_TEXTURE_2D, 0, (bpp == 32) ? GL_RGBA : GL_RGB, w, h, 0, (bpp == 32) ? GL_RGBA : GL_RGB, GL_UNSIGNED_BYTE, data.data());
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);

    tex.width = w; tex.height = h;
    return tex;
  }

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
  GLint u_baseColor = -1, u_ambientStr = -1, u_sampler = -1, u_hasTexture = -1;
  GLint ue_mvp = -1, ue_model = -1, ue_lightDir = -1, ue_viewPos = -1, ue_time = -1;
  GLint ugg_mvp = -1, ugg_model = -1, ugg_lightDir = -1, ugg_viewPos = -1, ugg_baseColor = -1;
  GLint uba_mvp = -1, uba_model = -1, uba_lightDir = -1, uba_viewPos = -1, uba_baseColor = -1;
  GLint uri_mvp = -1, uri_model = -1, uri_lightDir = -1, uri_viewPos = -1, uri_baseColor = -1;
  // Billboard uniforms
  GLint ub_vp = -1, ub_proj = -1, ub_pos = -1, ub_size = -1, ub_color = -1;
  // Atmosphere uniforms
  GLint ua_mvp = -1, ua_model = -1, ua_lightDir = -1, ua_viewPos = -1;
  // Skybox uniforms
  GLint us_invViewProj = -1, us_skyVibrancy = -1;

  // === Terrain & Vegetation ===
  GLuint terrainProg = 0;
  GLint ut_mvp = -1, ut_model = -1, ut_camPos = -1, ut_lightDir = -1, ut_viewPos = -1, ut_time = -1;
  GLint ut_nodePos = -1, ut_nodeSide = -1, ut_nodeUp = -1; // For patch warping
  GLint ut_nodeLevel = -1;      // New: Quadtree subdivision level
  GLint ut_planetCenterRel = -1; // New: Planet center relative to camera
  GLint ut_tectonicMap = -1;    // New: Baked Macro-Skeleton from simulation
  GLint ut_climateMap = -1;     // New: Baked Climate Data (Temp, Precip, Pressure)
  GLint ut_viewMode = -1;       // New: Visualization Mode (0: Normal, 1: Temp, 2: Precip, 3: Pressure)
  int climateViewMode = 0;       // Current view mode
  void setClimateViewMode(int mode) { climateViewMode = mode; }
  GLuint vegProg = 0;
  GLint uv_vp = -1, uv_proj = -1, uv_lightDir = -1, uv_viewPos = -1, uv_time = -1;
  GLuint treeVAO = 0, treeVBO = 0, treeEBO = 0, treeInstanceVBO = 0;
  int treeIndexCount = 0;
  GLuint grassVAO = 0, grassVBO = 0, grassInstanceVBO = 0;
  int grassIndexCount = 0;

  // Exhaust uniforms
  GLuint exhaustProg = 0;
  GLint uex_mvp = -1, uex_model = -1, uex_invModel = -1, uex_viewPos = -1, uex_time = -1;
  GLint uex_throttle = -1, uex_expansion = -1;
  GLint uex_groundDist = -1, uex_plumeLen = -1;

  // ===== RSS-Reborn Per-Planet Shader Programs =====
  GLuint mercuryProgram = 0, venusProgram = 0, moonProgram = 0, marsProgram = 0;
  GLuint jupiterProgram = 0, saturnProgram = 0, uranusProgram = 0, neptuneProgram = 0;
  // Per-planet uniform locations (all share same uniform names for simplicity)
  struct PlanetUniforms { GLint mvp=-1, model=-1, lightDir=-1, viewPos=-1, baseColor=-1, time=-1; };
  PlanetUniforms u_mercury, u_venus, u_moon, u_mars, u_jupiter, u_saturn, u_uranus, u_neptune;

  Mat4 view, proj;
  Vec3 camPos;
  Vec3 lightDir;

  GLint ut_hydroMap = -1;
  GLint ut_localHydroMap = -1;
  GLint ut_hasLocalHydro = -1;
  Mesh sharedPatchMesh;
  Terrain::QuadtreeTerrain* terrain = nullptr;
  Vegetation::VegetationSystem* vegSystem = nullptr;

  // === SVO (Sparse Voxel Octree) System ===
  GLuint svoProg = 0;
  GLint usvo_mvp = -1, usvo_lightDir = -1, usvo_viewPos = -1;
  GLint usvo_svoMat = -1;
  SVO::SVOManager* svoManager = nullptr;

  // Cached meshes and textures for parts
  std::map<std::string, Mesh> meshCache;
  std::map<std::string, Texture> textureCache;

  // Ribbon Renderer properties
  GLuint ribbonProg, ribbonVAO, ribbonVBO;
  GLint ur_mvp;

  // Lens Flare properties
  GLuint lensFlareProg, lfVAO, lfVBO;
  GLint ulf_sunScreenPos, ulf_aspect, ulf_color, ulf_intensity;

  // ===== TAA (Temporal Anti-Aliasing) Infrastructure =====
  GLuint taaFBO[2] = {0, 0};        // Ping-pong framebuffers
  GLuint taaColorTex[2] = {0, 0};   // Color textures for ping-pong
  GLuint taaDepthTex = 0;           // Shared depth texture (for reprojection)
  GLuint taaProg = 0;               // TAA resolve shader
  GLuint taaVAO = 0;                // Fullscreen quad VAO (reuse skybox)
  int taaWidth = 0, taaHeight = 0;  // Current FBO dimensions
  int taaFrameIndex = 0;            // Frame counter for noise animation
  bool taaInitialized = false;
  float taaBlendOverride = -1.0f;   // If >= 0, use this blend factor instead of default

  Mat4 prevViewProj;                // History matrix
  Mat4 invViewProj;                 // Current inverse matrix

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
      uniform sampler2D uSampler;
      uniform bool uHasTexture;

      out vec4 FragColor;

      void main() {
        vec3 N = normalize(vNormal);
        vec3 L = normalize(uLightDir);
        float ambient = uAmbientStr;
        float diff = max(dot(N, L), 0.0);
        vec3 V = normalize(uViewPos - vWorldPos);
        vec3 H = normalize(L + V);
        float spec = pow(max(dot(N, H), 0.0), 32.0);
        
        vec4 texColor = uHasTexture ? texture(uSampler, vUV) : vec4(1.0);
        vec4 color = uBaseColor * vColor * texColor;
        
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
    u_sampler = glGetUniformLocation(program3d, "uSampler");
    u_hasTexture = glGetUniformLocation(program3d, "uHasTexture");

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
        float seaLevel = 0.45;
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

        // ---- CLOUD LAYERS REMOVED (Replaced by Volumetric Atmo Shader) ----

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
      uniform float uRingInner; uniform float uRingOuter;
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

        // === Analytic Ring Shadow (Hardcore Realism) ===
        float ringShadow = 1.0;
        if (L.y != 0.0) {
            float t = -vLocalPos.y / L.y;
            if (t > 0.0) {
                vec3 hit = vLocalPos + t * L;
                float dist = length(hit.xz);
                if (dist >= uRingInner && dist <= uRingOuter) {
                    // Use NASA Optical Depth (tau) Profile for shadow mask
                    float tau = 0.0;
                    if (dist < 1.24) tau = 0.005;
                    else if (dist < 1.53) tau = 0.1;
                    else if (dist < 1.95) tau = 1.8;
                    else if (dist < 2.03) tau = 0.05;
                    else if (dist < 2.27) tau = 0.6;
                    else if (dist < 2.35) tau = 0.1;
                    
                    float transmittance = exp(-tau / abs(L.y));
                    ringShadow = transmittance;
                }
            }
        }

        // === Ring-Shine (Indirect Lighting from Rings) ===
        // Sun hits rings -> Rings reflect back to planet night side
        float ringShine = 0.0;
        float ringAlbedo = 0.25; // Approximate average albedo of ice rings
        // Shine is strongest when Sun is high above ring plane (abs(L.y)) 
        // and surface faces the rings (abs(N.y))
        // Northern hemisphere (N.y > 0) sees Northern ring-face (lit if L.y > 0)
        // We use a simplified model for ambient bounce
        if (dot(N, L) < 0.0) {
            ringShine = abs(N.y) * abs(L.y) * ringAlbedo * 0.4;
        }

        // === Subtle banding structure ===
        float bandWarp = fbm(sph * vec3(1.5, 0.5, 1.5) + vec3(timeOff * 0.3), 4);
        float bands = sin(lat * 18.0 + bandWarp * 3.0) * 0.5 + 0.5;
        float fineBands = sin(lat * 50.0 + bandWarp * 1.5 + 2.0) * 0.15;
        bands = clamp(bands + fineBands, 0.0, 1.0);

        float turb = fbm(tc * 2.5 + vec3(timeOff, lat * 3.0, timeOff * 0.5), 6) * 0.2;

        float polarDist = length(sph - vec3(0.0, 1.0, 0.0));
        float hexAngle = atan(sph.z, sph.x);
        float hexPattern = cos(hexAngle * 6.0) * 0.5 + 0.5;
        float hexMask = smoothstep(0.45, 0.15, polarDist) * hexPattern;

        vec3 goldZone = vec3(0.90, 0.82, 0.60);
        vec3 paleBelt = vec3(0.78, 0.70, 0.52);
        vec3 warmBelt = vec3(0.65, 0.52, 0.32);
        vec3 hexColor = vec3(0.55, 0.45, 0.30);
        vec3 polarColor = vec3(0.50, 0.42, 0.28);

        vec3 surfColor = mix(warmBelt, goldZone, bands);
        surfColor = mix(surfColor, paleBelt, turb);

        float storm = noise3d(tc * 6.0 + vec3(timeOff * 3.0));
        float stormMask = smoothstep(0.78, 0.88, storm) * 0.4;
        surfColor = mix(surfColor, vec3(0.95, 0.92, 0.85), stormMask);
        surfColor = mix(surfColor, hexColor, hexMask * 0.5);

        float polarMask = smoothstep(0.65, 0.85, abs(lat));
        surfColor = mix(surfColor, polarColor, polarMask);

        float haze = smoothstep(0.0, 0.4, abs(lat));
        surfColor = mix(surfColor, goldZone * 1.05, (1.0 - haze) * 0.15);

        float diff = max(dot(N, L), 0.0);
        float ambient = 0.03;
        float rim = 1.0 - max(dot(N, V), 0.0);
        float rimPow = pow(rim, 3.0);
        float NdotL = dot(N, L);
        vec3 atmosGlow = vec3(0.88, 0.78, 0.55) * rimPow * 0.30 * smoothstep(-0.1, 0.2, NdotL);

        vec3 coreLighting = surfColor * (ambient + diff * 0.85 * ringShadow + ringShine);
        vec3 result = coreLighting + atmosGlow * ringShadow;
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

        // Apply wind rotation around Z-axis (poles are on Z)
        float w1 = timeOff * -2.0; // Slow, uniform rotation
        float s1 = sin(w1); float c1 = cos(w1);
        vec3 windSph = vec3(sph.x * c1 - sph.y * s1, sph.x * s1 + sph.y * c1, sph.z);

        // === Strong banding (Neptune has more visible features than Uranus) ===
        float bandWarp = fbm(windSph * vec3(2.0, 1.0, 2.0), 5);
        float bands = sin(lat * 16.0 + bandWarp * 3.5) * 0.5 + 0.5;
        float fineBands = sin(lat * 40.0 + bandWarp * 1.5) * 0.1;
        bands = clamp(bands + fineBands, 0.0, 1.0);

        // === Great Dark Spot (GDS) analog ===
        float gAngle = timeOff * -2.0;
        float sg = sin(gAngle); float cg = cos(gAngle);
        vec3 gCenter = vec3(-0.3, -0.3, 0.5);
        vec3 gdsCenter = vec3(gCenter.x * cg - gCenter.z * sg, gCenter.y, gCenter.x * sg + gCenter.z * cg);
        float gdsDist = length(sph - normalize(gdsCenter));
        float gdsMask = smoothstep(0.2, 0.0, gdsDist);

        // === Methane cirrus streaks (bright white wispy clouds) ===
        // Slightly faster uniform wind for cirrus clouds (Z-axis rotation)
        float w2 = timeOff * -4.0;
        float s2 = sin(w2); float c2 = cos(w2);
        vec3 windSph2 = vec3(sph.x * c2 - sph.y * s2, sph.x * s2 + sph.y * c2, sph.z);
        float cirrus = fbm(windSph2 * vec3(8.0, 2.0, 8.0) + vec3(0.0, lat * 5.0, 0.0), 6);
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
        float streaks = fbm(windSph * 20.0, 5) * 0.15;
        surfColor += vec3(streaks * 0.3, streaks * 0.4, streaks);

        // Great Dark Spot
        surfColor = mix(surfColor, gdsColor, gdsMask * 0.7);

        // Bright companion clouds near the GDS
        float companion = smoothstep(0.18, 0.22, gdsDist) * smoothstep(0.30, 0.22, gdsDist);
        companion *= fbm(windSph2 * 12.0, 4);
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
      uniform float uPlanetRadius; 
      out vec4 FragColor;

      float henyeyGreenstein(float cosTheta, float g) {
        float g2 = g * g;
        return (1.0 - g2) / (4.0 * 3.14159 * pow(1.0 + g2 - 2.0 * g * cosTheta, 1.5));
      }

      // Exact Saturn Ring Optical Depth Profiles (NASA Voyager/Cassini)
      float getRingOpticalDepth(float r) {
        // D Ring: 1.11 - 1.24 (faint)
        if (r < 1.11) return 0.0;
        if (r < 1.24) return 0.005;
        // C Ring: 1.24 - 1.53 (moderate)
        if (r < 1.53) return mix(0.05, 0.15, smoothstep(1.24, 1.53, r));
        // B Ring: 1.53 - 1.95 (brightest, densest)
        if (r < 1.95) return mix(1.2, 2.5, sin(r * 40.0) * 0.2 + 0.8);
        // Cassini Division: 1.95 - 2.03 (gap)
        if (r < 2.03) return 0.02;
        // A Ring: 2.03 - 2.27 (dense)
        if (r < 2.27) {
            float base = 0.5;
            float encke = smoothstep(2.21, 2.22, r) * smoothstep(2.23, 2.22, r);
            return mix(base, 0.01, encke);
        }
        // F Ring: ~2.33 (thin)
        if (r < 2.31) return 0.0;
        if (r < 2.34) return 0.1;
        return 0.0;
      }

      void main() {
        vec3 N = normalize(vNormal);
        vec3 L = normalize(uLightDir);
        vec3 V = normalize(uViewPos - vWorldPos);
        float cosTheta = dot(V, -L);
        float NdotV = abs(dot(N, V));
        
        // --- Physically Based Scattering ---
        // Industrial Ice Particle Phase: Strong forward peak (diffraction) + backward lobe
        float phaseForward = henyeyGreenstein(cosTheta, 0.85); // Cinematic diffraction
        float phaseBack = henyeyGreenstein(cosTheta, -0.3);
        float phase = mix(phaseForward, phaseBack, 0.4);
        
        // --- Hardcore Planet Shadow ---
        float b = 2.0 * dot(vLocalPos, L);
        float c = dot(vLocalPos, vLocalPos) - uPlanetRadius * uPlanetRadius;
        float disc = b * b - 4.0 * c;
        float shadow = 1.0;
        if (disc > 0.0) {
            float t1 = (-b + sqrt(disc)) / 2.0;
            if (t1 > 0.0) shadow = 0.0; 
        }

        float r = length(vLocalPos.xz);
        float tau = getRingOpticalDepth(r);
        
        // --- Physical Transparency Model ---
        // Transmittance T = e^(-tau / cos(theta_view))
        float transmittance = exp(-tau / max(NdotV, 0.001));
        float alpha = 1.0 - transmittance;

        // Apply high-frequency micro-structure (Cassini/Encke gaps)
        float fineS = sin(r * 300.0) * 0.1 + sin(r * 800.0) * 0.05;
        alpha *= (1.0 + fineS);
        alpha = clamp(alpha, 0.0, 0.95);

        vec3 iceWhite = vec3(0.92, 0.90, 0.88);
        vec3 shadowColor = vec3(0.1, 0.08, 0.05); // Ambient ring tone
        
        // Multiple scattering approximation for albedo
        vec3 albedo = mix(vec3(0.8, 0.7, 0.5), iceWhite, smoothstep(0.1, 1.0, tau));
        
        // Combine all components
        vec3 result = albedo * (0.05 + 1.5 * phase * shadow);
        
        // Add "Interstellar" glow in backlighting
        if (cosTheta > 0.95) {
            result += iceWhite * pow(cosTheta, 64.0) * 4.0 * shadow;
        }

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
    
    // Add new ring uniforms and set defaults for Saturn
    glUseProgram(ringProgram);
    glUniform1f(glGetUniformLocation(ringProgram, "uPlanetRadius"), 1.0f); // Default (Mesh unit is planet radius)

    glUseProgram(saturnProgram);
    glUniform1f(glGetUniformLocation(saturnProgram, "uRingInner"), 1.11f);
    glUniform1f(glGetUniformLocation(saturnProgram, "uRingOuter"), 2.35f);

    glUseProgram(atmoProg);
    glUniform1f(glGetUniformLocation(atmoProg, "uRingInner"), 1.11f);
    glUniform1f(glGetUniformLocation(atmoProg, "uRingOuter"), 2.35f);
    glUseProgram(0);



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

    // --- Atmosphere Shader (Volumetric Raymarching Scattering) ---
    const char* atmoVertSrc = R"(
      #version 330 core
      layout(location=0) in vec3 aPos;
      uniform mat4 uMVP;
      uniform mat4 uModel;
      out vec3 vWorldPos;
      void main() {
        vWorldPos = (uModel * vec4(aPos, 1.0)).xyz;
        gl_Position = uMVP * vec4(aPos, 1.0);
      }
    )";

    // --- Atmosphere Shader (HARDCORE Rayleigh + Mie + Ozone Volumetric Raymarching) ---
    // Designed for smooth ground-to-space transition with dramatic scattering
    const char* atmoFragSrc = R"(
      #version 330 core
      in vec3 vWorldPos;
      
      uniform vec3 uCamPos;
      uniform vec3 uLightDir;
      uniform vec3 uPlanetCenter;
      uniform float uInnerRadius;
      uniform float uOuterRadius;
      uniform float uSurfaceRadius;
      uniform float uTime;
      uniform int uPlanetIdx;
      uniform float uSunVisibility; // Global sun visibility from camera (0 to 1)
      uniform float uRingInner; // Ring inner radius (local units)
      uniform float uRingOuter; // Ring outer radius (local units)
      uniform float uShowClouds; // Toggle clouds
      
      out vec4 FragColor;

      #define PI 3.14159265359

      // High step counts for quality inside-atmosphere rendering
      const int PRIMARY_STEPS = 48;
      const int LIGHT_STEPS = 16;
      
      // --- Dynamic Atmosphere Parameters ---
      vec3 gRayleighCoeff;
      float gMieCoeff;
      float gHRayleigh;
      float gHMie;
      float gGMie;
      vec3 gOzoneCoeff;
      float gHOzoneCenter;
      float gHOzoneWidth;

      float gCloudMinAlt;
      float gCloudMaxAlt;
      vec3 gCloudExtinction;
      vec3 gCloudScattering;

      void setupPlanetProfile() {
          if (uPlanetIdx == 3) {
              // EARTH
              gRayleighCoeff = vec3(0.0058, 0.0135, 0.0331) * 1.2;
              gMieCoeff = 0.005; gHRayleigh = 8.0; gHMie = 1.0; gGMie = 0.8;
              gOzoneCoeff = vec3(0.00035, 0.00085, 0.00009); gHOzoneCenter = 25.0; gHOzoneWidth = 15.0;
              gCloudMinAlt = 3.0; gCloudMaxAlt = 14.0;
              gCloudExtinction = vec3(0.08); gCloudScattering = vec3(0.45);
          } else if (uPlanetIdx == 2) {
              // VENUS - thick CO2 atmosphere with sulfuric acid clouds
              // Rayleigh must follow physical spectrum (blue > green > red) to avoid blue terminator artifact
              gRayleighCoeff = vec3(0.005, 0.012, 0.028);
              gMieCoeff = 0.04; gHRayleigh = 15.0; gHMie = 5.0; gGMie = 0.76;
              // Sulfur UV absorber strips blue, creates yellow/orange sky color
              gOzoneCoeff = vec3(0.0005, 0.005, 0.02); gHOzoneCenter = 50.0; gHOzoneWidth = 20.0;
              gCloudMinAlt = 45.0; gCloudMaxAlt = 65.0;
              // Low scattering so cloud texture is visible, not blown out white
              gCloudExtinction = vec3(0.08); gCloudScattering = vec3(0.15);
          } else if (uPlanetIdx == 5) {
              // MARS
              gRayleighCoeff = vec3(0.01, 0.005, 0.001);
              gMieCoeff = 0.015; gHRayleigh = 11.0; gHMie = 3.0; gGMie = 0.85;
              gOzoneCoeff = vec3(0.0); gHOzoneCenter = 1.0; gHOzoneWidth = 1.0;
              gCloudMinAlt = 100.0; gCloudMaxAlt = 101.0;
              gCloudExtinction = vec3(0.01); gCloudScattering = vec3(0.01);
          } else if (uPlanetIdx == 6) {
              // JUPITER
              gRayleighCoeff = vec3(0.018, 0.015, 0.012);
              gMieCoeff = 0.01; gHRayleigh = 27.0; gHMie = 10.0; gGMie = 0.8;
              gOzoneCoeff = vec3(0.0); gHOzoneCenter = 1.0; gHOzoneWidth = 1.0;
              gCloudMinAlt = 10.0; gCloudMaxAlt = 50.0;
              gCloudExtinction = vec3(0.1); gCloudScattering = vec3(0.6);
          } else if (uPlanetIdx == 7) {
              // SATURN
              gRayleighCoeff = vec3(0.015, 0.013, 0.009);
              gMieCoeff = 0.01; gHRayleigh = 60.0; gHMie = 15.0; gGMie = 0.8;
              gOzoneCoeff = vec3(0.0); gHOzoneCenter = 1.0; gHOzoneWidth = 1.0;
              gCloudMinAlt = 10.0; gCloudMaxAlt = 50.0;
              gCloudExtinction = vec3(0.1); gCloudScattering = vec3(0.6);
          } else if (uPlanetIdx == 8) {
              // URANUS
              gRayleighCoeff = vec3(0.002, 0.015, 0.025);
              gMieCoeff = 0.002; gHRayleigh = 28.0; gHMie = 5.0; gGMie = 0.8;
              gOzoneCoeff = vec3(0.001, 0.0, 0.0); gHOzoneCenter = 20.0; gHOzoneWidth = 10.0;
              gCloudMinAlt = 10.0; gCloudMaxAlt = 50.0;
              gCloudExtinction = vec3(0.1); gCloudScattering = vec3(0.6);
          } else if (uPlanetIdx == 9) {
              // NEPTUNE
              gRayleighCoeff = vec3(0.001, 0.008, 0.035);
              gMieCoeff = 0.001; gHRayleigh = 20.0; gHMie = 5.0; gGMie = 0.8;
              gOzoneCoeff = vec3(0.001, 0.0, 0.0); gHOzoneCenter = 20.0; gHOzoneWidth = 10.0;
              gCloudMinAlt = 10.0; gCloudMaxAlt = 50.0;
              gCloudExtinction = vec3(0.1); gCloudScattering = vec3(0.6);
          } else {
              // DEFAULT (fallback)
              gRayleighCoeff = vec3(0.0058, 0.0135, 0.0331);
              gMieCoeff = 0.005; gHRayleigh = 8.0; gHMie = 1.0; gGMie = 0.8;
              gOzoneCoeff = vec3(0.0); gHOzoneCenter = 1.0; gHOzoneWidth = 1.0;
              gCloudMinAlt = 100.0; gCloudMaxAlt = 101.0;
              gCloudExtinction = vec3(0.0); gCloudScattering = vec3(0.0);
          }
      }

      bool intersectSphere(vec3 ro, vec3 rd, float radius, out float t0, out float t1) {
          // Stable geometric intersection to prevent precision loss and catastrophic cancellation
          // (which causes concentric rings and missing atmosphere at >10,000 km)
          vec3 L = ro - uPlanetCenter;
          float tca = -dot(L, rd);
          vec3 perp = L + tca * rd;
          float d2 = dot(perp, perp);
          float radius2 = radius * radius;
          if (d2 > radius2) return false;
          float thc = sqrt(radius2 - d2);
          t0 = tca - thc;
          t1 = tca + thc;
          return true;
      }

      float rayleighPhase(float cosTheta) {
          return 3.0 / (16.0 * PI) * (1.0 + cosTheta * cosTheta);
      }

      float miePhase(float cosTheta) {
          // Cornette-Shanks phase function (improved Henyey-Greenstein)
          float g2 = gGMie * gGMie;
          float num = 3.0 * (1.0 - g2) * (1.0 + cosTheta * cosTheta);
          float denom = 8.0 * PI * (2.0 + g2) * pow(1.0 + g2 - 2.0 * gGMie * cosTheta, 1.5);
          return num / max(denom, 1e-6);
      }

      // Ozone density at altitude h (Gaussian profile around 25km)
      float ozoneDensity(float h) {
          float d = (h - gHOzoneCenter) / gHOzoneWidth;
          return exp(-0.5 * d * d);
      }

      // --- NOISE & CLOUDS ---
      // Interleaved Gradient Noise (Jimenez 2014, used in AAA engines)
      // Produces much more uniform spatial distribution than white noise,
      // reducing visible clumping artifacts dramatically.
      uniform sampler2D uDepthTex;
      uniform vec2 uResolution;
      uniform mat4 uInvProj;

      uniform int uFrameIndex; // Animated frame counter for temporal variation
      uniform float uCloudTime;
      uniform float uCloudPhaseSin;
      uniform float uCloudPhaseCos;
      float screenHash(vec2 uv) {
          // IGN base: very uniform spatial distribution
          float ign = fract(52.9829189 * fract(dot(uv, vec2(0.06711056, 0.00583715))));
          // Animate per-frame using golden ratio offset for low-discrepancy sequence
          float animated = fract(ign + float(uFrameIndex % 32) * 0.6180339887);
          return animated;
      }

      // Pseudo-random gradient vector from integer lattice point (non-periodic)
      vec3 hashGrad(vec3 p) {
        // Purely arithmetic hash — no sin() periodicity
        vec3 q = vec3(dot(p, vec3(127.1, 311.7, 74.7)),
                      dot(p, vec3(269.5, 183.3, 246.1)),
                      dot(p, vec3(113.5, 271.9, 124.6)));
        return normalize(-1.0 + 2.0 * fract(q * fract(q * 0.3183099)));
      }
      // Gradient noise (Perlin-style) — no grid-aligned artifacts
      float noise3d(vec3 p) {
        vec3 i = floor(p); vec3 f = fract(p);
        // Quintic Hermite interpolation (C2 continuous)
        vec3 u = f * f * f * (f * (f * 6.0 - 15.0) + 10.0);
        
        return mix(mix(mix(dot(hashGrad(i + vec3(0,0,0)), f - vec3(0,0,0)),
                           dot(hashGrad(i + vec3(1,0,0)), f - vec3(1,0,0)), u.x),
                       mix(dot(hashGrad(i + vec3(0,1,0)), f - vec3(0,1,0)),
                           dot(hashGrad(i + vec3(1,1,0)), f - vec3(1,1,0)), u.x), u.y),
                   mix(mix(dot(hashGrad(i + vec3(0,0,1)), f - vec3(0,0,1)),
                           dot(hashGrad(i + vec3(1,0,1)), f - vec3(1,0,1)), u.x),
                       mix(dot(hashGrad(i + vec3(0,1,1)), f - vec3(0,1,1)),
                           dot(hashGrad(i + vec3(1,1,1)), f - vec3(1,1,1)), u.x), u.y), u.z)
               * 0.5 + 0.5; // Remap from [-1,1] to [0,1]
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
      
      float cloudDensity(vec3 p, float h, bool highDetail) {
          if (uShowClouds < 0.5) return 0.0;
          if (h < gCloudMinAlt || h > gCloudMaxAlt) return 0.0;
          
          vec3 sph = normalize(p - uPlanetCenter);
          float t = uCloudTime * 0.004; // Wrapped time for noise
          float density = 0.0;
          
          if (uPlanetIdx == 3) {
              // EARTH Cloud layers
              // Layer 1: Low-altitude Cumulus/Stratus (Dense, massive blobs)
              if (h >= gCloudMinAlt && h <= 7.0) {
                  float altNorm = (h - gCloudMinAlt) / (7.0 - gCloudMinAlt);
                  float altShape = 4.0 * altNorm * (1.0 - altNorm);

                  // Industrial Grade: Phase Separation for rotation
                  // Replace sin(t*speed + warp) with sin(A + B) = sinA*cosB + cosA*sinB
                  // uCloudPhaseSin/Cos represent the temporal rotation component
                  float latWarp = sph.z * sph.z * sph.z * 0.8;
                  float sB = sin(latWarp);
                  float cB = cos(latWarp);
                  float s1 = uCloudPhaseSin * cB + uCloudPhaseCos * sB;
                  float c1 = uCloudPhaseCos * cB - uCloudPhaseSin * sB;
                  
                  vec3 windSph1 = vec3(sph.x * c1 - sph.y * s1, sph.x * s1 + sph.y * c1, sph.z);
                  
                  // ====== CYCLONE ATTRACTOR (Hurricane) ======
                  float cycAngle = t * -0.1; // Slower cyclone movement
                  vec3 cycCenter = normalize(vec3(0.6 * cos(cycAngle), 0.6 * sin(cycAngle), 0.35)); // Mid-latitude
                  float cycDist = distance(windSph1, cycCenter);
                  float cycRadius = 0.25; // Realistic proportion (~1500km on Earth)
                  
                  // Gradual feathered edge: twist fades smoothly at boundary
                  float cycInfluence = smoothstep(cycRadius, cycRadius * 0.3, cycDist); // 0 at edge, 1 near center
                  if (cycInfluence > 0.001) {
                      float twist = pow(cycInfluence, 1.5) * 8.0; // Vortex intensity
                      // Rodrigues rotation around the cyclone center
                      float ct = cos(twist); float st = sin(twist);
                      vec3 warped = windSph1 * ct + cross(cycCenter, windSph1) * st + cycCenter * dot(cycCenter, windSph1) * (1.0 - ct);
                      windSph1 = mix(windSph1, warped, cycInfluence); // Smooth blend at edges
                  }
                  
                  // ====== MACRO CLOUD COVERAGE (Fronts & Clear Oceans) ======
                  vec3 coverageSph = vec3(sph.x * c1 - sph.y * s1, sph.x * s1 + sph.y * c1, sph.z);
                  float coverage = fbm(coverageSph * 2.5 + vec3(t * 0.01), 3);
                  // Target ~66% cloud cover: raise bounds so more area falls below threshold
                  coverage = smoothstep(0.28, 0.52, coverage);
                  
                  // Force cloud buildup around cyclone, carve out eye
                  if (cycInfluence > 0.001) {
                      float eye = 0.012; // Tiny eye (~75km)
                      float eyewall = smoothstep(eye, eye + 0.03, cycDist) * smoothstep(cycRadius * 0.6, cycRadius * 0.15, cycDist);
                      coverage = max(coverage, eyewall * 0.95); 
                      coverage *= smoothstep(eye * 0.3, eye, cycDist); 
                  }
                  
                  // Detail noise
                  vec3 tc = windSph1 * 14.0;
                  float n = 0.0;
                  if (highDetail) {
                      float warp = fbm(tc * 0.5, 3);
                      n = fbm(tc + vec3(warp * 1.2) + vec3(t * 0.02), 6); 
                      float detail = fbm(tc * 3.0 - vec3(t * 0.04), 4);
                      n = n - detail * 0.3; 
                  } else {
                      n = fbm(tc + vec3(t * 0.02), 2);
                  }
                  
                  // coverage=1.0 -> threshold=0.20 (dense fronts)
                  // coverage=0.0 -> threshold=0.58 (clear skies, only occasional puffs)
                  float threshold = mix(0.58, 0.20, coverage);
                  float mask = smoothstep(threshold, threshold + 0.12, n) * altShape;
                  density = max(density, mask * 4.0);
              }
              
              // Layer 2: High-altitude Cirrus/Anvils (Thin, sweeping streaks)
              if (h >= 8.0 && h <= gCloudMaxAlt) {
                  float altNorm = (h - 8.0) / (gCloudMaxAlt - 8.0);
                  // Flatter top shape for anvils
                  float altShape = 1.0 - pow(abs(altNorm * 2.0 - 1.0), 2.0);
                  // Slightly faster uniform wind for cirrus (Z-axis)
                  float angle2 = t * -0.5;
                  // Coriolis Warping for high cirrus
                  float latWarp2 = sph.z * sph.z * sph.z * 0.4; // Z is the polar axis
                  float s2 = sin(angle2 + latWarp2); 
                  float c2 = cos(angle2 + latWarp2);
                  vec3 windSph2 = vec3(sph.x * c2 - sph.y * s2, sph.x * s2 + sph.y * c2, sph.z); // Rotate around Z axis
                  vec3 tc = windSph2 * 12.0;
                  
                  // Cirrus shape
                  float n = 0.0;
                  if (highDetail) {
                      float warp = fbm(tc * 1.2, 2);
                      n = fbm(tc + vec3(warp * 2.5) + vec3(t * 0.04), 4);
                      // Cirrus Erosion: subtract wispy fractal detail
                      float detail = fbm(tc * 4.0 + vec3(t * 0.08), 2);
                      n = n - detail * 0.2;
                  } else {
                      n = fbm(tc + vec3(t * 0.04), 1); // Fast path for shadow rays
                  }
                  
                  // Thin clouds: wider threshold, soft borders
                  float mask = smoothstep(0.35 + altNorm * 0.15, 0.85, n) * altShape;
                  density = max(density, mask * 0.35); // Thinner density
              }
          } else if (uPlanetIdx == 2) {
              // VENUS global acid cloud deck
              float altNorm = (h - gCloudMinAlt) / (gCloudMaxAlt - gCloudMinAlt);
              float altShape = 4.0 * altNorm * (1.0 - altNorm);
              
              float angle = t * -0.1; // slow super-rotation
              float s1 = sin(angle); float c1 = cos(angle);
              vec3 windSph = vec3(sph.x * c1 - sph.y * s1, sph.x * s1 + sph.y * c1, sph.z);
              
              // Higher frequency to show swirling texture, lower density to avoid whiteout
              float n = highDetail ? fbm(windSph * 8.0 + vec3(t * 0.05), 3) : fbm(windSph * 8.0 + vec3(t * 0.05), 1);
              density = (0.3 + 0.7 * n) * altShape * 0.8; 
          }
          
          return density;
      }
)"
R"(
      float cloudPhase(float cosTheta) {
          // Dual-Lobe Henyey-Greenstein (Silver lining + backscatter)
          // Look towards sun: strong forward scattering (silver lining)
          float g1 = 0.85; 
          float num1 = 1.0 - g1 * g1;
          float denom1 = pow(1.0 + g1 * g1 - 2.0 * g1 * cosTheta, 1.5);
          float phase1 = (1.0 / (4.0 * PI)) * (num1 / denom1);
          
          // Look away from sun: subtle backward scattering (cloud brightness)
          float g2 = -0.3;
          float num2 = 1.0 - g2 * g2;
          float denom2 = pow(1.0 + g2 * g2 - 2.0 * g2 * cosTheta, 1.5);
          float phase2 = (1.0 / (4.0 * PI)) * (num2 / denom2);
          
          // Mix lobes: mostly forward scattering, some backward
          float w = 0.7; // Weight of forward lobe
          // Reduced boost from 12.0 to 1.5 to prevent blowing out exposure
          return mix(phase2, phase1, w) * 1.5; 
      }

      void getOpticalDepth(vec3 p, vec3 lightDir, out float dR, out float dM, out float dO, out float dC) {
          dR = 0.0; dM = 0.0; dO = 0.0; dC = 0.0;

          // Ground shadow check: if light ray hits planet surface, full occlusion
          // Physically, the sun should be blocked as soon as it touches the horizon.
          // Using a slight offset to account for non-zero planet height features.
          float tS0, tS1;
          if (intersectSphere(p, lightDir, uInnerRadius, tS0, tS1) && tS0 > 0.0) {
              dR = 1e6; dM = 1e6; dO = 1e6; dC = 1e6;
              return;
          }

          // Ring shadow check (if planet has rings)
          if (uRingOuter > 0.0 && lightDir.y != 0.0) {
              float tR = -(p - uPlanetCenter).y / lightDir.y;
              if (tR > 0.0) {
                  vec3 hit = (p - uPlanetCenter) + tR * lightDir;
                  float d = length(hit.xz);
                  if (d >= uRingInner && d <= uRingOuter) {
                      float tau = 0.0;
                      if (d < 1.24) tau = 0.005;
                      else if (d < 1.53) tau = 0.1;
                      else if (d < 1.95) tau = 1.8;
                      else if (d < 2.03) tau = 0.05;
                      else if (d < 2.27) tau = 0.6;
                      else if (d < 2.35) tau = 0.1;

                      float atten = exp(-tau / abs(lightDir.y));
                      // Block light by boosting optical thickness of the atmosphere segment
                      float block = (1.0 - atten) * 8.0;
                      dR += block; dM += block; dO += block; dC += block;
                  }
              }
          }

          float t0, t1;
          if (!intersectSphere(p, lightDir, uOuterRadius, t0, t1) || t1 <= 0.0) return;
          float start = max(t0, 0.0);
          float stepSize = (t1 - start) / float(LIGHT_STEPS);
          for (int i = 0; i < LIGHT_STEPS; i++) {
              vec3 pos = p + lightDir * (start + (float(i) + 0.5) * stepSize);
              float h = length(pos - uPlanetCenter) - uSurfaceRadius;
              // Hard cutoff for underground points to prevent light leaking through the planet core
              if (h < -0.1) { dR = 1e6; dM = 1e6; dO = 1e6; dC = 1e6; return; }
              dR += exp(-h / gHRayleigh) * stepSize;
              dM += exp(-h / gHMie) * stepSize;
              dO += ozoneDensity(h) * stepSize;
              dC += cloudDensity(pos, h, false) * stepSize;
          }
      }

      void main() {
          setupPlanetProfile();
          vec3 rayDir = normalize(vWorldPos - uCamPos);
          
          // Reconstruct terrain distance from depth buffer
          vec2 uv = gl_FragCoord.xy / uResolution;
          float dVal = texture(uDepthTex, uv).r;
          float terrainDist = 1e6;
          if (dVal < 1.0) {
              vec4 clip = vec4(uv * 2.0 - 1.0, dVal * 2.0 - 1.0, 1.0);
              vec4 viewPos = uInvProj * clip;
              viewPos /= viewPos.w;
              terrainDist = length(viewPos.xyz);
          }
          
          // Compute camera altitude for adaptive behavior
          float camDist = length(uCamPos - uPlanetCenter);
          float camAlt = max(camDist - uSurfaceRadius, 0.0);
          float atmoThickness = uOuterRadius - uSurfaceRadius;
          bool cameraInside = camDist < uOuterRadius;
          
          float tNear, tFar;
          if (!intersectSphere(uCamPos, rayDir, uOuterRadius, tNear, tFar)) discard;
          
          // Occlude atmosphere against physical terrain distance
          if (dVal < 1.0) {
              float analyticalSurf = 1e6;
              float tSurf0, tSurf1;
              if (intersectSphere(uCamPos, rayDir, uInnerRadius, tSurf0, tSurf1) && tSurf0 > 0.0) {
                  analyticalSurf = tSurf0;
              }
              
              float diff = abs(terrainDist - analyticalSurf);
              // Error margin scales with distance because 24-bit depth buffer precision degrades at huge distances
              float errorMargin = camAlt * 0.05 + 150.0;
              
              if (diff < errorMargin) {
                  // Eliminate Z-fighting and concentric rings by smoothly blending depth into analytical sphere at large distances
                  // At < 100km, we fully trust depth buffer for crisp mountain occlusion.
                  // At > 1000km, we fully trust mathematical sphere to prevent precision banding.
                  float tScale = clamp((camAlt - 100.0) / 1000.0, 0.0, 1.0);
                  tFar = min(tFar, mix(terrainDist, analyticalSurf, tScale));
              } else {
                  tFar = min(tFar, terrainDist); // e.g. looking at a spaceship
              }
          } else {
              // Clamp mathematical surface intersection if looking at space
              float tSurf0, tSurf1;
              if (intersectSphere(uCamPos, rayDir, uInnerRadius, tSurf0, tSurf1) && tSurf0 > 0.0) {
                  tFar = min(tFar, tSurf0);
              }
          }
          
          // When camera is inside, ray starts from camera position
          tNear = max(tNear, 0.0);
          float segmentLength = tFar - tNear;
          if (segmentLength <= 0.0) discard;
          
          float stepSize = segmentLength / float(PRIMARY_STEPS);
          
          // Apply screen-space dithering to eliminate banding artifacts
          float jitter = screenHash(gl_FragCoord.xy);
          tNear += jitter * stepSize;
          
          vec3 sumRayleigh = vec3(0.0);
          vec3 sumMie = vec3(0.0);
          vec3 sumCloud = vec3(0.0);
          float optDepthR = 0.0;
          float optDepthM = 0.0;
          float optDepthO = 0.0;
          float optDepthC = 0.0;
          float cosTheta = dot(rayDir, uLightDir);
          
          for (int i = 0; i < PRIMARY_STEPS; i++) {
              vec3 pos = uCamPos + rayDir * (tNear + (float(i) + 0.5) * stepSize);
              float h = max(length(pos - uPlanetCenter) - uSurfaceRadius, 0.0);
              
              float dR = exp(-h / gHRayleigh) * stepSize;
              float dM = exp(-h / gHMie) * stepSize;
              float dO = ozoneDensity(h) * stepSize;
              float dC = cloudDensity(pos, h, true) * stepSize;
              optDepthR += dR;
              optDepthM += dM;
              optDepthO += dO;
              optDepthC += dC;
              
              // Light ray optical depth to sun
              float lightR, lightM, lightO, lightC;
              getOpticalDepth(pos, uLightDir, lightR, lightM, lightO, lightC);
              
              // Total extinction including ozone absorption
              vec3 tauBase = gRayleighCoeff * (optDepthR + lightR) 
                           + gMieCoeff * 1.1 * (optDepthM + lightM)
                           + gOzoneCoeff * (optDepthO + lightO);
                           
              // Cloud Extinction & Powder Effect Approx
              float tauCloud = gCloudExtinction.x * (optDepthC + lightC);
              vec3 attenuationBase = exp(-tauBase);
              
              // Beer-Lambert attenuation (direct absorption)
              float beer = exp(-tauCloud);
              
              // Powder effect (multiple scattering illuminates interior edges)
              // 1.0 - exp(-d*2.0) creates a soft "bump" of light just inside the cloud
              float powder = 1.0 - exp(-tauCloud * 2.0);
              
              // Mix them: outer edges rely on powder, deep interiors rely on beer
              float cloudAttenuation = beer * mix(1.0, powder, 0.7); 
              
              // Combine atmosphere and cloud attenuation
              vec3 attenuation = attenuationBase * cloudAttenuation;
              
              sumRayleigh += dR * attenuationBase * exp(-gCloudExtinction.x * optDepthC); // Air only attenuated by local cloud, not sun cloud
              sumMie += dM * attenuationBase * exp(-gCloudExtinction.x * optDepthC);
              sumCloud += dC * attenuation;
              
              // Early Ray Termination (ERT)
              // If transmittance (attenuation) drops below 1%, stop raymarching to save massive GPU cycles
              float maxTransmittance = max(attenuation.x, max(attenuation.y, attenuation.z));
              if (maxTransmittance < 0.01) {
                  break; 
              }
          }
          
          // Phase functions
          float phaseR = rayleighPhase(cosTheta);
          float phaseM = miePhase(cosTheta);
          float phaseC = cloudPhase(cosTheta);
          
          // Ambient multi-scattering for clouds (prevents black clouds on day side)
          // Uses a broader, softer phase approximation and less attenuation
          float ambientPhaseC = 0.5; // Isotropic-ish
          vec3 ambientCloud = sumCloud * gCloudScattering * ambientPhaseC * 0.4; // 40% ambient boost
          
          // Inscattering color
          vec3 color = sumRayleigh * gRayleighCoeff * phaseR + 
                       sumMie * gMieCoeff * phaseM + 
                       sumCloud * gCloudScattering * phaseC +
                       ambientCloud;
          
          // === Altitude-adaptive exposure ===
          // Earth-like exposure values for a natural look
          float altNorm = clamp(camAlt / atmoThickness, 0.0, 1.0);
          // Darken exposure significantly at night to mimic human eye adaptation and prevent amplification of residual twilight
          float nightFactor = mix(0.01, 1.0, uSunVisibility);
          float exposure = mix(10.0, 5.0, smoothstep(0.0, 0.6, altNorm)) * nightFactor;

          // Subtle boost when looking horizontally at ground level
          if (cameraInside) {
              vec3 localUp = normalize(uCamPos - uPlanetCenter);
              float upDot = dot(rayDir, localUp);
              float horizBoost = 1.0 + 0.25 * exp(-upDot * upDot * 8.0) * (1.0 - altNorm);
              exposure *= horizBoost;
          }
          
          color *= exposure;
          
          // === Physical transmittance-based opacity ===
          vec3 tau_view = gRayleighCoeff * optDepthR + gMieCoeff * optDepthM + gOzoneCoeff * optDepthO + gCloudExtinction * optDepthC;
          vec3 viewTransmittance = exp(-tau_view);
          float transLuma = dot(viewTransmittance, vec3(0.299, 0.587, 0.114));
          float opacity = clamp(1.0 - transLuma, 0.0, 1.0);
          
          // Night-side opacity: reduce opacity at the horizon at night to let stars shine through
          // This prevents the "milky black horizon" effect
          if (uSunVisibility < 0.1) {
              opacity *= smoothstep(0.0, 0.1, uSunVisibility + 0.05);
          }

          // Minimal opacity boost when inside for a clear sky appearance
          if (cameraInside) {
              opacity = clamp(opacity * mix(1.2, 1.0, smoothstep(0.0, 0.5, altNorm)), 0.0, 1.0);
          }
          
          // ACES filmic tone mapping (richer sunset/sunrise colors than Reinhard)
          vec3 x = max(color, vec3(0.0));
          color = (x * (2.51 * x + 0.03)) / (x * (2.43 * x + 0.59) + 0.14);
          
          FragColor = vec4(color, opacity);
      }
    )";
    atmoProg = compileProgram(atmoVertSrc, atmoFragSrc);
    ua_mvp = glGetUniformLocation(atmoProg, "uMVP");
    ua_model = glGetUniformLocation(atmoProg, "uModel");

    // --- Ribbon Shader (Trajectory Trails) ---
    const char* ribbonVertSrc = R"(
      #version 330 core
      layout(location=0) in vec3 aPos;
      layout(location=1) in vec4 aColor;
      layout(location=2) in float qSide; // -1 to 1
      uniform mat4 uMVP;
      out vec4 vColor;
      out float vSide;
      void main() {
        vColor = aColor;
        vSide = qSide;
        gl_Position = uMVP * vec4(aPos, 1.0);
      }
    )";
    const char* ribbonFragSrc = R"(
      #version 330 core
      in vec4 vColor;
      in float vSide;
      out vec4 FragColor;
      void main() {
        float alpha = 1.0 - smoothstep(0.7, 1.0, abs(vSide));
        FragColor = vec4(vColor.rgb, vColor.a * alpha);
      }
    )";
    ribbonProg = compileProgram(ribbonVertSrc, ribbonFragSrc);
    ur_mvp = glGetUniformLocation(ribbonProg, "uMVP");

    glGenVertexArrays(1, &ribbonVAO);
    glGenBuffers(1, &ribbonVBO);
    glBindVertexArray(ribbonVAO);
    glBindBuffer(GL_ARRAY_BUFFER, ribbonVBO);
    glBufferData(GL_ARRAY_BUFFER, (sizeof(Vec3) + sizeof(Vec4) + sizeof(float)) * 4000, nullptr, GL_DYNAMIC_DRAW);
    
    struct RibVert { Vec3 p; Vec4 c; float side; };
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(RibVert), (void*)0);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, sizeof(RibVert), (void*)(sizeof(Vec3)));
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(2, 1, GL_FLOAT, GL_FALSE, sizeof(RibVert), (void*)(sizeof(Vec3) + sizeof(Vec4)));
    glEnableVertexAttribArray(2);
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
        vUV = aPos * 0.5 + 0.5;
        vec2 pos = aPos * uScale;
        pos.x /= uAspect;
        // Output at the sun's actual screen-space depth to allow occlusion
        gl_Position = vec4(pos + uOffset, uSunScreenPos.x > -2.0 ? 0.9999 : 0.0, 1.0);
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

      uniform float uSkyVibrancy; // 1.0 = full space visibility, 0.0 = washed out by atmosphere

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

        if (uSkyVibrancy < 0.01) {
            FragColor = vec4(0.0, 0.0, 0.0, 1.0);
            return;
        }

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

          if (mag > 0.65) {
            float brt = (mag - 0.65) / 0.35;
            brt = brt * brt * brt * 8.0; // Boosted brightness
            float sz2 = 0.006 + brt * 0.015;
            float star = smoothstep(sz2, sz2 * 0.1, dist);

            // Spectral class coloring (O B A F G K M)
            float temp = hash13(cell + vec3(5,7,float(layer)));
            vec3 sc;
            if (temp < 0.10) sc = vec3(0.5, 0.7, 1.0);       // O/B bright blue
            else if (temp < 0.30) sc = vec3(0.8, 0.9, 1.0);   // A white-blue
            else if (temp < 0.55) sc = vec3(1.0, 1.0, 0.95);  // F/G pure white
            else if (temp < 0.80) sc = vec3(1.0, 0.95, 0.8);  // K warm white
            else sc = vec3(1.0, 0.7, 0.5);                    // M red-orange

            // Diffraction cross for brightest stars
            float cross = 0.0;
            if (brt > 0.8) {
              float dx = abs(localPos.x - starPos.x);
              float dy = abs(localPos.y - starPos.y);
              // Bolder spikes
              cross = exp(-dx*60.0)*exp(-dy*250.0) + exp(-dy*60.0)*exp(-dx*250.0);
              cross *= (brt - 0.8) * 3.0; // Boosted spikes
            }

            col += sc * (star * brt + cross * 0.6);
          }
        }

        // === MILKY WAY (smooth noise-based) ===
        // Galactic coordinate transform (tilted 62.87° from equatorial)
        float sinGal = 0.8829; float cosGal = 0.4695;
        float galLat = rd.y * cosGal - (rd.x * 0.6 + rd.z * 0.35) * sinGal;
        float galLon = atan(rd.z, rd.x);

        // Gaussian band with varying width
        float bandWidth = 0.10 + 0.05 * sin(galLon * 2.0);
        float band = exp(-galLat * galLat / (2.0 * bandWidth * bandWidth));

        // Smooth nebulosity using FBM noise
        vec3 galCoord = rd * 4.0;
        float neb = fbm(galCoord * 1.5, 6);
        float nebDetail = fbm(galCoord * 4.0 + vec3(100.0), 5);

        // Dark dust lanes (absorption)
        float dust = fbm(galCoord * 3.5 + vec3(50.0, -30.0, 20.0), 5);
        float dustAbsorption = smoothstep(0.3, 0.7, dust) * band;

        // Milky Way emission
        float galEmission = neb * band * 0.8; // Boosted
        galEmission += nebDetail * band * 0.4; // Boosted
        galEmission *= (1.0 - dustAbsorption * 0.75);

        // Color: warm brown core with amber nebulae
        vec3 galCore = vec3(0.35, 0.25, 0.15) * galEmission; // More saturated/bright
        vec3 warmNeb = vec3(1.2, 0.6, 0.2) * nebDetail * band * 0.4; // Intense amber
        vec3 blueNeb = vec3(0.2, 0.3, 0.8) * smoothstep(0.4, 0.9, neb) * band * 0.3; // More vivid blue

        // Dense star clusters within the band
        for (int cl = 0; cl < 3; cl++) {
          float cs = 250.0 + float(cl) * 150.0;
          vec3 cc = floor(rd * cs);
          float cm = hash13(cc + vec3(99.0 + float(cl)*50.0));
          if (cm > 0.88 && band > 0.1) {
            vec3 lp = fract(rd * cs) - 0.5;
            float cd = length(lp);
            col += vec3(1.0, 0.95, 0.8) * smoothstep(0.012, 0.0, cd) * (cm-0.88)*15.0 * band;
          }
        }

        col += galCore + warmNeb + blueNeb;

        // Very subtle cosmic background
        col += vec3(0.002, 0.003, 0.006);

        // Apply vibrancy (atmosphere washout)
        col *= uSkyVibrancy;

        // Tone mapping for HDR stars - adjusted for punchier highlights
        col = col / (col + 0.5);

        FragColor = vec4(col, 1.0);
      }
    )";
    skyboxProg = compileProgram(skyboxVertSrc, skyboxFragSrc);
    us_invViewProj = glGetUniformLocation(skyboxProg, "uInvViewProj");
    us_skyVibrancy = glGetUniformLocation(skyboxProg, "uSkyVibrancy");

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

    // --- Volumetric Exhaust Shader (Mach Diamonds & Expansion) ---
    const char* exVertSrc = R"(
      #version 330 core
      layout(location=0) in vec3 aPos;
      uniform mat4 uMVP;
      uniform mat4 uModel;
      out vec3 vLocalPos;
      out vec3 vWorldPos;
      void main() {
        vLocalPos = aPos; // Box from -0.5 to 0.5
        vWorldPos = (uModel * vec4(aPos, 1.0)).xyz;
        gl_Position = uMVP * vec4(aPos, 1.0);
      }
    )";
    const char* exFragSrc = R"(
      #version 330 core
      in vec3 vLocalPos;
      in vec3 vWorldPos;
      uniform vec3 uViewPos;
      uniform mat4 uInvModel;
      uniform float uTime;
      uniform float uThrottle;
      uniform float uExpansion;
      uniform float uGroundDist;
      uniform float uPlumeLen;
      out vec4 FragColor;

      float hash(float n) { return fract(sin(n) * 43758.5453123); }
      float noise(vec3 x) {
        vec3 p = floor(x); vec3 f = fract(x);
        f = f*f*(3.0-2.0*f);
        float n = p.x + p.y*57.0 + 113.0*p.z;
        return mix(mix(mix(hash(n+0.0),hash(n+1.0),f.x),mix(hash(n+57.0),hash(n+58.0),f.x),f.y),
                   mix(mix(hash(n+113.0),hash(n+114.0),f.x),mix(hash(n+170.0),hash(n+171.0),f.x),f.y),f.z);
      }
      
      float fbm(vec3 p) {
        float v = 0.0;
        float a = 0.5;
        for (int i=0; i<3; i++) {
          v += a * noise(p);
          p *= 2.0;
          a *= 0.5;
        }
        return v;
      }

      vec2 rayBoxInter(vec3 ro, vec3 rd, vec3 boxMin, vec3 boxMax) {
        vec3 t0 = (boxMin - ro) / rd;
        vec3 t1 = (boxMax - ro) / rd;
        vec3 tmin = min(t0, t1);
        vec3 tmax = max(t0, t1);
        float dNear = max(max(tmin.x, tmin.y), tmin.z);
        float dFar = min(min(tmax.x, tmax.y), tmax.z);
        return vec2(dNear, dFar);
      }

      void main() {
        vec3 viewDirWorld = normalize(vWorldPos - uViewPos);
        vec3 ro = (uInvModel * vec4(uViewPos, 1.0)).xyz;
        vec3 rd = normalize((uInvModel * vec4(viewDirWorld, 0.0)).xyz);
        
        vec2 bounds = rayBoxInter(ro, rd, vec3(-0.5), vec3(0.5));
        if (bounds.x > bounds.y || bounds.y < 0.0) discard;
        
        float start = max(0.0, bounds.x);
        float end = bounds.y;
        
        // High-frequency jitter
        float dither = hash(dot(gl_FragCoord.xy, vec2(12.9898, 78.233))) * 0.015;
        float t = start + dither;
        
        vec4 finalCol = vec4(0.0);
        int steps = 60; // Slightly more steps for thin splash precision
        float stepSize = (end - start) / float(steps);

        // Ground collision normalized height (0 at end of box, 1 at nozzle)
        float zGround = 1.0 - uGroundDist / uPlumeLen;

        for (int i=0; i<steps; i++) {
          vec3 p = ro + rd * t;
          float z = p.y + 0.5; // 0 (end) to 1 (nozzle)
          
          // GROUND CLIP
          if (z < zGround) {
             t += stepSize;
             continue;
          }

          float distToGround = z - zGround;
          // TIGHT LOCALIZED SPLASH: Only within few % of plume length near ground
          float splashZone = smoothstep(0.06, 0.0, distToGround) * smoothstep(-0.2, 0.05, zGround);
          
          float r = length(p.xz) * 2.0; 
          
          // Waterfall-style Flare: Multi-stage expansion
          float coreFlare = mix(0.35, 1.0 + uExpansion * 4.0, pow(1.0 - z, 1.4));
          
          // PROPORTIONAL RADIAL SPLASH
          // Radius is proportional to the depth of truncated plume (zGround)
          float truncDepth = max(0.0, zGround);
          float radAngle = atan(p.x, p.z);
          float radialWildness = 0.7 + 0.6 * noise(vec3(radAngle * 4.0, uTime * 18.0, zGround * 8.0));
          // splashRadiusBoost covers more of the local box radius (0.5-0.7)
          float splashRadiusBoost = truncDepth * 1.6 * splashZone * radialWildness;
          
          coreFlare += splashRadiusBoost;
          
          float sheathFlare = coreFlare * (1.1 + uExpansion * 2.0);
          
          if (r < sheathFlare) {
            float normR = r / coreFlare;
            
            // 1. Base Physic-Informed Density (Solid Core Focus)
            float d = exp(-normR * 6.0) * pow(z, 0.45) * (1.1 - normR);
            
            // Splash Density Boost: Make the splash feel thick/viscous
            // Density falloff is flatter horizontally
            d += splashZone * 0.7 * exp(-normR * 2.0) * (1.0 - normR * 0.95);

            // Tail Billowing: High-frequency radial distortion at the dissipation zone
            float billowValue = noise(vec3(p.xz * 12.0, uTime * 20.0)) * (1.0 - z) * 0.15;
            float taperedR = normR + billowValue;
            
            // Tail Tapering: Smooth falloff at the bottom/ground
            float tailTaper = (zGround > -0.2) ? smoothstep(-0.01, 0.04, distToGround) : smoothstep(0.0, 0.2, z);
            d *= tailTaper * (1.0 / (1.0 + billowValue * 5.0));
            d = max(0.0, d);

            // 2. High-Velocity Dynamics & FBM Turbulence
            float gasSpeed = uTime * (40.0 + uThrottle * 20.0);
            float streaks = noise(vec3(atan(p.x, p.z) * 8.0, p.y * 50.0, gasSpeed * 0.8)); 
            float turb = fbm(p * 18.0 - vec3(0, gasSpeed, 0)); 
            d *= (0.4 + 0.5 * turb + 0.35 * streaks);
            
            // 3. View-Dependent Alpha Falloff (Improved Side-View)
            float edgeSoftness = smoothstep(0.0, 0.4, 1.0 - taperedR);
            d *= (0.2 + 0.8 * edgeSoftness);
            
            // 4. Dynamic Pulse/Flicker (Synced with Throttle)
            float flicker = 0.94 + 0.12 * hash(uTime * 150.0 * uThrottle); 
            float pulse = 1.0 + 0.08 * sin(uTime * 100.0) * uThrottle; 
            d *= (flicker * pulse);

            // 5. High-Dynamic Color Profiling (Aggressive Glow)
            vec3 core = vec3(1.2, 1.2, 1.1);
            vec3 mid = vec3(1.0, 0.5, 0.05);
            vec3 outer = vec3(0.6, 0.12, 0.04);
            
            // Splash color shift: brighter near the ground impact
            mid = mix(mid, vec3(1.3, 1.0, 0.4), splashZone * 0.8);

            vec3 col = mix(outer, mid, smoothstep(0.05, 0.4, d));
            col = mix(col, core, smoothstep(0.4, 0.9, d));
            
            float alpha = d * stepSize * 65.0; 
            finalCol.rgb += (1.0 - finalCol.a) * col * alpha;
            finalCol.a += (1.0 - finalCol.a) * alpha;
          }
          
          t += stepSize;
          if (finalCol.a > 0.99) break;
        }

        FragColor = vec4(finalCol.rgb * uThrottle * 5.0, finalCol.a * uThrottle);
      }
    )";
    exhaustProg = compileProgram(exVertSrc, exFragSrc);
    uex_mvp = glGetUniformLocation(exhaustProg, "uMVP");
    uex_model = glGetUniformLocation(exhaustProg, "uModel");
    uex_invModel = glGetUniformLocation(exhaustProg, "uInvModel");
    uex_viewPos = glGetUniformLocation(exhaustProg, "uViewPos");
    uex_time = glGetUniformLocation(exhaustProg, "uTime");
    uex_throttle = glGetUniformLocation(exhaustProg, "uThrottle");
    uex_expansion = glGetUniformLocation(exhaustProg, "uExpansion");
    uex_groundDist = glGetUniformLocation(exhaustProg, "uGroundDist");
    uex_plumeLen = glGetUniformLocation(exhaustProg, "uPlumeLen");

    // --- TAA Resolve Shader ---
    const char* taaVertSrc = R"(
      #version 330 core
      layout(location=0) in vec2 aPos;
      out vec2 vUV;
      void main() {
        vUV = aPos * 0.5 + 0.5;
        gl_Position = vec4(aPos, 0.0, 1.0);
      }
    )";
    const char* taaFragSrc = R"(
      #version 330 core
      in vec2 vUV;
      uniform sampler2D uCurrentFrame;
      uniform sampler2D uHistoryFrame;
      uniform sampler2D uDepthTex;
      uniform float uBlendFactor;
      
      uniform mat4 uInvViewProj;
      uniform mat4 uPrevViewProj;

      out vec4 FragColor;

      void main() {
        // 1. SAMPLING: Pure screen-space history
        vec4 current = texture(uCurrentFrame, vUV);
        vec4 history = texture(uHistoryFrame, vUV);

        // 2. VARIANCE CLIPPING (High quality anti-ghosting)
        vec2 texelSize = 1.0 / vec2(textureSize(uCurrentFrame, 0));
        vec3 m1 = vec3(0.0), m2 = vec3(0.0);
        
        // 3x3 neighborhood statistics for "soft" clamping
        for (int x = -1; x <= 1; x++) {
          for (int y = -1; y <= 1; y++) {
            vec3 s = texture(uCurrentFrame, vUV + vec2(float(x), float(y)) * texelSize).rgb;
            m1 += s;
            m2 += s * s;
          }
        }
        
        vec3 mu = m1 / 9.0;
        vec3 sigma = sqrt(max(vec3(0.0), m2 / 9.0 - mu * mu));
        
        // Clamp history to mu +/- N*sigma (variance box)
        float gamma = 1.5; // Controls the "tightness" of history rejection
        vec3 cMin = mu - gamma * sigma;
        vec3 cMax = mu + gamma * sigma;
        
        history.rgb = clamp(history.rgb, cMin, cMax);

        // Standard blend factor (smooth accumulation when static, rejection when moving)
        FragColor = mix(history, current, uBlendFactor);
      }
    )";
    taaProg = compileProgram(taaVertSrc, taaFragSrc);
    taaVAO = skyboxVAO; // Reuse the fullscreen quad

    // --- Terrain Shader (High-res displacement mapping) ---
    const char* terrainVertSrc = R"(
      #version 330 core
      layout(location=0) in vec3 aPos;
      layout(location=1) in vec3 aNormal;
      layout(location=2) in vec2 aUV;
      
      uniform mat4 uMVP;
      uniform mat4 uModel;
      uniform vec3 uCamPos;
      uniform vec3 uPlanetCenterRel; 
      uniform float uPlanetRadius;
      uniform float uMaxElevation;
      uniform float uTime;
      uniform int uNodeLevel;

      uniform vec3 uNodePos;
      uniform vec3 uNodeSide;
      uniform vec3 uNodeUp;
      
      uniform sampler2D uTectonicMap;
      uniform sampler2D uHydroMap; 
      uniform sampler2D uLocalHydroMap;
      uniform int uHasLocalHydro;

      out vec3 vRelViewPos; 
      out vec3 vNormal;
      out vec2 vUV;
      out vec2 vLocalUV;
      out vec3 vLocalPos;
      out float vElevation;
      out float vWaterDepth;
      out float vSlope;

      vec3 hash33(vec3 p) {
        p = fract(p * vec3(443.897, 441.423, 437.195));
        p += dot(p, p.yzx + 19.19);
        return fract((p.xxy + p.yzz) * p.zyx);
      }
      
      float worley(vec3 p) {
        vec3 i = floor(p); vec3 f = fract(p);
        float minDist = 1.0;
        for (int z = -1; z <= 1; z++) {
          for (int y = -1; y <= 1; y++) {
            for (int x = -1; x <= 1; x++) {
              vec3 neighbor = vec3(float(x), float(y), float(z));
              vec3 point = hash33(i + neighbor);
              vec3 diff = neighbor + point - f;
              minDist = min(minDist, length(diff));
            }
          }
        }
        return minDist;
      }

      float hash(vec3 p) {
        p = fract(p * vec3(443.897, 441.423, 437.195));
        p += dot(p, p.yzx + 19.19);
        return fract((p.x + p.y) * p.z);
      }
      
      vec4 sampleHydroSmooth(vec2 uv) {
          vec2 res = vec2(512.0, 256.0);
          vec2 st = uv * res - 0.5;
          vec2 i = floor(st);
          vec2 f = fract(st);
          vec4 t00 = texture(uHydroMap, (i + vec2(0.5, 0.5)) / res);
          vec4 t10 = texture(uHydroMap, (i + vec2(1.5, 0.5)) / res);
          vec4 t01 = texture(uHydroMap, (i + vec2(0.5, 1.5)) / res);
          vec4 t11 = texture(uHydroMap, (i + vec2(1.5, 1.5)) / res);
          float r = mix(mix(t00.r, t10.r, f.x), mix(t01.r, t11.r, f.x), f.y);
          float g = mix(mix(t00.g, t10.g, f.x), mix(t01.g, t11.g, f.x), f.y);
          vec4 point = texture(uHydroMap, uv);
          return vec4(r, g, point.b, point.a);
      }

      float noise3d(vec3 p) {
        vec3 i = floor(p); vec3 f = fract(p);
        f = f * f * f * (f * (f * 6.0 - 15.0) + 10.0);
        return mix(mix(mix(hash(i), hash(i+vec3(1,0,0)), f.x), 
                       mix(hash(i+vec3(0,1,0)), hash(i+vec3(1,1,0)), f.x), f.y),
                   mix(mix(hash(i+vec3(0,0,1)), hash(i+vec3(1,0,1)), f.x), 
                       mix(hash(i+vec3(0,1,1)), hash(i+vec3(1,1,1)), f.x), f.y), f.z);
      }
      float fbm(vec3 p, int oct) {
        float v = 0.0, a = 0.5;
        for(int i=0; i<oct; ++i) {
          v += noise3d(p) * a;
          p = p * 2.15 + vec3(0.13, -0.21, 0.34);
          a *= 0.47;
        }
        return v;
      }
      float ridgedFbm(vec3 p, int oct) {
        float v = 0.0, a = 0.5, f = 1.05;
        for(int i=0; i<oct; ++i) {
          float n = 1.0 - abs(noise3d(p * f) * 2.0 - 1.0);
          v += n * n * a;
          f *= 2.07; a *= 0.48;
        }
        return v;
      }
      float mountainNoise(vec3 p) {
        vec3 off = vec3(fbm(p*2.0, 3), fbm(p*2.0+5.2, 3), fbm(p*2.0+2.7, 3));
        return ridgedFbm(p + off * 0.15, 6);
      }
      float detail10m(vec3 p) {
        // High frequency micro-cells (approx 10m scale)
        float n = noise3d(p * 2500000.0);
        n += noise3d(p * 5000000.0) * 0.5;
        return n * 0.0000015; // ~10m on 6000km sphere
      }

      float gullyNoise(vec3 p, float angle) {
        float ca = cos(angle), sa = sin(angle);
        float sU = p.x * ca - p.z * sa;
        float sV = p.x * sa + p.z * ca;
        // Stretch in sV direction (aligned with gradient/slope)
        vec3 pS = vec3(sU * 1500000.0, p.y * 1500000.0, sV * 40000.0);
        return ridgedFbm(pS, 3);
      }

      float sampleTectonic(vec2 uv) {
          vec2 res = vec2(512.0, 256.0);
          vec2 st = uv * res - 0.5;
          vec2 i = floor(st);
          vec2 f = fract(st);
          float h00 = texture(uTectonicMap, i/res).r;
          float h10 = texture(uTectonicMap, (i+vec2(1,0))/res).r;
          float h01 = texture(uTectonicMap, (i+vec2(0,1))/res).r;
          float h11 = texture(uTectonicMap, (i+vec2(1,1))/res).r;
          float base = mix(mix(h00, h10, f.x), mix(h01, h11, f.x), f.y);
          float tGrad = length(vec2(h10-h00, h01-h00));
          if (tGrad > 0.05) {
              float sx = f.x * f.x * (3.0 - 2.0 * f.x);
              float sy = f.y * f.y * (3.0 - 2.0 * f.y);
              float sharp = mix(mix(h00, h10, sx), mix(h01, h11, sx), sy);
              base = mix(base, sharp, 0.45);
          }
          return base;
      }

      void main() {
        vLocalUV = aPos.xz + 0.5;
        vec3 cubePos = uNodePos + aPos.x * uNodeSide + aPos.z * uNodeUp;
        vec3 normPos = normalize(cubePos);
        float fV_phi = acos(clamp(normPos.y, -1.0, 1.0));
        float fV_theta = atan(normPos.z, normPos.x);
        vec2 geoUV = vec2(fV_theta / (2.0 * 3.14159) + 0.5, fV_phi / 3.14159);
        
        float plateBase = sampleTectonic(geoUV);
        float hRefined = plateBase;
        
        float mMask = smoothstep(0.55, 0.75, plateBase);
        float pMask = 1.0 - smoothstep(0.40, 0.62, plateBase);
        float cMask = smoothstep(0.435, 0.445, plateBase) * (1.0 - smoothstep(0.455, 0.465, plateBase));

        // 1.1 Mountain Folding + Thermal Erosion
        if (mMask > 0.01) {
            float epsG = 0.005;
            float hX = (sampleTectonic(geoUV + vec2(epsG, 0.0)) - sampleTectonic(geoUV - vec2(epsG, 0.0)));
            float hY = (sampleTectonic(geoUV + vec2(0.0, epsG)) - sampleTectonic(geoUV - vec2(0.0, epsG)));
            float sAngle = atan(-hX, hY);
            float ca = cos(sAngle), sa = sin(sAngle);
            float sU = normPos.x * ca - normPos.z * sa;
            float sV = normPos.x * sa + normPos.z * ca;
            vec3 pS = vec3(sU * 0.72, normPos.y, sV * 1.28); 
            float ridges = mountainNoise(pS * 6.5);
            hRefined += ridges * 0.18 * mMask;
            if (hRefined > 0.78) hRefined -= smoothstep(0.78, 0.96, hRefined) * 0.045;
        }

        // 1.2 Biome Features (Cliffs)
        float cliffNoise = 1.0 - abs(noise3d(normPos * 180.0) * 1.5 - 0.5);
        hRefined += smoothstep(0.6, 0.9, cliffNoise) * 0.007 * cMask;

        // 2.1 River Valley Carving (V-to-U shaped)
        vec4 hydro = sampleHydroSmooth(geoUV);
        if (uHasLocalHydro == 1) {
            hydro.r = texture(uLocalHydroMap, vLocalUV).r;
        }
        float strahler = hydro.b; 
        if (strahler >= 2.0 && plateBase > 0.445) {
            float vNoise = noise3d(normPos * 250.0 + noise3d(normPos * 120.0) * 0.004);
            float depth = 0.015 * (strahler / 7.0);
            float isLow = 1.0 - smoothstep(0.45, 0.55, plateBase);
            float profile = mix(abs(vNoise * 2.0 - 1.0), pow(abs(vNoise * 2.0 - 1.0), 0.4), isLow);
            hRefined -= (1.0 - profile) * depth;
        }

        // float hydroMask = smoothstep(0.0001, 0.01, hydro.r - plateBase);
        // hRefined = mix(hRefined, hydro.r, hydroMask * 0.97);

        float sLevel = 0.45;
        float hydroDiff = hydro.r - hRefined;
        float hydroMask = smoothstep(0.0001, 0.005, hydroDiff);
        hRefined = mix(hRefined, hydro.r, hydroMask * 1.0);

        if (hRefined < sLevel && hRefined > 0.38) {
            float shelfT = (hRefined - 0.38) / (sLevel - 0.38);
            hRefined = mix(0.395, sLevel - 0.001, smoothstep(0.0, 1.0, shelfT));
        }

        float finalH = (hRefined < sLevel) ? 0.0 : (hRefined - sLevel) / (1.0 - sLevel) * uMaxElevation / uPlanetRadius;
        
        // 10m Scale detail (Land Only)
        float currentSlope = 0.0;
        if (hRefined >= sLevel) {
            float epsS = 0.0015;
            float hX = (sampleTectonic(geoUV + vec2(epsS, 0.0)) - sampleTectonic(geoUV - vec2(epsS, 0.0)));
            float hY = (sampleTectonic(geoUV + vec2(0.0, epsS)) - sampleTectonic(geoUV - vec2(0.0, epsS)));
            currentSlope = length(vec2(hX, hY)) * 100.0;
            float sAngle = atan(-hX, hY);
            
            float d10 = detail10m(normPos);
            
            // Slope-based high frequency details
            float rockCracks = (1.0 - worley(normPos * 1800000.0)) * 0.000003;
            float gullies = gullyNoise(normPos, sAngle) * 0.000002;
            float baseNoise = (noise3d(normPos * 4000000.0) - 0.5) * 0.0000005;
            
            float sHigh = smoothstep(0.12, 0.25, currentSlope);
            float sMid = smoothstep(0.04, 0.12, currentSlope) * (1.0 - sHigh);
            float sLow = 1.0 - smoothstep(0.04, 0.12, currentSlope);
            
            finalH += (rockCracks * sHigh + gullies * sMid + baseNoise * sLow);
            finalH += d10;
        }
        vec3 kscPos = vec3(0.1436, 0.478, 0.866);
        float distToKSC = length(normPos - kscPos);
        float kscMask = smoothstep(0.12, 0.04, distToKSC); 
        
        // Boost hRefined within KSC radius to ensure land-based coloring
        hRefined = mix(hRefined, sLevel + 0.015, kscMask);
        
        // Final displacement (5m above sea level)
        finalH = mix(finalH, 0.005 / uPlanetRadius, smoothstep(0.08, 0.02, distToKSC)); 
        
        mat3 localRotScale = mat3(uModel); 
        vRelViewPos = uPlanetCenterRel + localRotScale * (normPos * (1.0 + finalH));
        vElevation = hRefined; 
        vNormal = localRotScale * normPos; 
        vUV = aUV;
        vLocalPos = normPos;
        vWaterDepth = max(0.0, hydroDiff);
        vSlope = currentSlope;
        gl_Position = uMVP * vec4(vRelViewPos, 1.0);
      }
    )";

    const char* terrainFragSrc = R"(
      #version 330 core
      in vec3 vRelViewPos; 
      in vec3 vNormal;
      in vec2 vUV;
      in vec3 vLocalPos;
      in float vElevation;
      in float vWaterDepth;
      in vec2 vLocalUV;
      in float vSlope;

      uniform sampler2D uHydroMap;
      uniform sampler2D uLocalHydroMap;
      uniform int uHasLocalHydro;
      uniform int uViewMode; 
      uniform vec3 uLightDir;
      uniform vec3 uCamPos;
      uniform float uTime;
      uniform sampler2D uTectonicMap;
      uniform sampler2D uClimateMap;
      out vec4 FragColor;

      float hash(vec3 p) {
        p = fract(p * vec3(443.897, 441.423, 437.195));
        p += dot(p, p.yzx + 19.19);
        return fract((p.x + p.y) * p.z);
      }

      vec4 sampleHydroSmooth(vec2 uv) {
          vec2 res = vec2(512.0, 256.0);
          vec2 st = uv * res - 0.5;
          vec2 i = floor(st);
          vec2 f = fract(st);
          vec4 t00 = texture(uHydroMap, (i + vec2(0.5, 0.5)) / res);
          vec4 t10 = texture(uHydroMap, (i + vec2(1.5, 0.5)) / res);
          vec4 t01 = texture(uHydroMap, (i + vec2(0.5, 1.5)) / res);
          vec4 t11 = texture(uHydroMap, (i + vec2(1.5, 1.5)) / res);
          float r = mix(mix(t00.r, t10.r, f.x), mix(t01.r, t11.r, f.x), f.y);
          float g = mix(mix(t00.g, t10.g, f.x), mix(t01.g, t11.g, f.x), f.y);
          vec4 point = texture(uHydroMap, uv);
          return vec4(r, g, point.b, point.a);
      }

      float noise3d(vec3 p) {
        vec3 i = floor(p); vec3 f = fract(p);
        f = f * f * f * (f * (f * 6.0 - 15.0) + 10.0);
        return mix(mix(mix(hash(i), hash(i+vec3(1,0,0)), f.x), 
                       mix(hash(i+vec3(0,1,0)), hash(i+vec3(1,1,0)), f.x), f.y),
                   mix(mix(hash(i+vec3(0,0,1)), hash(i+vec3(1,0,1)), f.x), 
                       mix(hash(i+vec3(0,1,1)), hash(i+vec3(1,1,1)), f.x), f.y), f.z);
      }
      
      float getMicroHeight(vec3 p, float distFade) {
          float h = noise3d(p * 800.0) * 0.5;
          h += noise3d(p * 8000.0) * 0.15 * clamp(1.0 - distFade * 0.2, 0.0, 1.0);
          return h;
      }

      float warpedNoise(vec3 p) {
          vec3 q = vec3(noise3d(p + vec3(0.0, 0.0, 0.0)),
                        noise3d(p + vec3(5.2, 1.3, 0.7)),
                        noise3d(p + vec3(2.7, 8.4, 3.1)));
          return noise3d(p + q * 0.5);
      }

      void main() {
        vec3 L = normalize(uLightDir);
        vec3 V = normalize(-vRelViewPos); 
        float distToCam = length(vRelViewPos); 
        float detailFade = distToCam / 1000.0; 
        
        vec3 fV_normPos = normalize(vLocalPos);
        float fV_phi = acos(clamp(fV_normPos.y, -1.0, 1.0));
        float fV_theta = atan(fV_normPos.z, fV_normPos.x);
        vec2 fV_geoUV = vec2(fV_theta / (2.0 * 3.14159) + 0.5, fV_phi / 3.14159);

        // --- Shader Layer: Micro-Details ---
        float hRefined = vElevation; 
        float mMask = smoothstep(0.55, 0.75, hRefined);
        float pMask = 1.0 - smoothstep(0.40, 0.62, hRefined);
        
        // --- Normal Calculation with Micro-Perturbation ---
        vec3 N = normalize(vNormal);
        if (vWaterDepth < 0.0001 && distToCam < 8.0) {
            float nScale = 1500.0 * (1.0 + mMask * 2.0); 
            float nAmp = 0.05 * (1.0 - smoothstep(1.0, 8.0, distToCam));
            
            // 10m level split
            float d10 = noise3d(vLocalPos * 3000000.0) * 0.4;
            
            vec3 pMic = vLocalPos * nScale;
            float h0 = noise3d(pMic) + d10;
            float h1 = noise3d(pMic + vec3(0.005, 0, 0)) + d10;
            float h2 = noise3d(pMic + vec3(0, 0.005, 0)) + d10;
            
            vec3 microN = normalize(vec3(h0 - h1, h0 - h2, 1.0));
            vec3 TN = normalize(cross(N, vec3(0,1,0)));
            if (abs(N.y) > 0.99) TN = normalize(cross(N, vec3(0,0,1)));
            vec3 BN = cross(N, TN);
            N = normalize(N + (TN * microN.x + BN * microN.y) * nAmp * 2.0);
        }

        // --- Colors & Lighting: Silk Smooth Base with Albedo Variation ---
        vec3 deepWater = vec3(0.01, 0.06, 0.15);
        vec3 shallowWater = vec3(0.05, 0.35, 0.45);
        vec3 beach = vec3(0.72, 0.68, 0.52);
        vec3 lowland = vec3(0.12, 0.32, 0.10);
        vec3 forest = vec3(0.06, 0.22, 0.05);
        vec3 mountainColor = vec3(0.42, 0.38, 0.35);
        vec3 snow = vec3(0.92, 0.94, 1.0);
        vec3 concrete = vec3(0.45, 0.45, 0.48); // Industrial gray
        
        vec3 surfColor;
        float sLevel = 0.45;
        
        if (hRefined < sLevel || vWaterDepth > 0.0001) {
            float depth = (hRefined < sLevel) ? (sLevel - hRefined) : vWaterDepth * 2.0;
            surfColor = mix(shallowWater, deepWater, smoothstep(0.0, 0.15, depth));
        } else {
            float landH = (hRefined - sLevel) / (1.0 - sLevel);
            // Albedo Variation (Subtle, no black spots)
            float albedoVar = noise3d(vLocalPos * 120.0) * 0.1 + 0.95;
            
            vec3 cMid = mix(beach, lowland, smoothstep(0.0, 0.1, landH));
            cMid = mix(cMid, forest, smoothstep(0.1, 0.45, landH));
            cMid = mix(cMid, mountainColor, smoothstep(0.45, 0.8, landH));
            
            float snowMask = smoothstep(0.8, 0.95, landH);
            // Reduce snow on steep slopes (vSlope ranges ~0.0 to 0.5+)
            snowMask *= (1.0 - smoothstep(0.08, 0.22, vSlope)); 
            
            surfColor = mix(cMid, snow, snowMask) * albedoVar;
            
            // Apply Concrete for KSC
            vec3 kscPos = vec3(0.1436, 0.478, 0.866);
            float distToKSC = length(fV_normPos - kscPos);
            // Concrete pad: sharp circle with slight fade
            float concreteMask = smoothstep(0.015, 0.008, distToKSC); 
            surfColor = mix(surfColor, concrete, concreteMask);
        }

        float diff = max(dot(N, L), 0.0);
        float ambient = 0.18; 
        vec3 result = surfColor * (ambient + diff * (1.1 - ambient));
        
        if (uViewMode > 0) {
            vec4 climate = texture(uClimateMap, fV_geoUV);
            vec3 vColor = vec3(0.0);
            if (uViewMode == 1) {
                float t = (climate.r + 30.0) / 70.0;
                vColor = mix(vec3(0,0,1), vec3(1,0,0), smoothstep(0.0, 1.0, t));
            } else if (uViewMode == 2) {
                float p = climate.g / 3000.0;
                vColor = mix(vec3(0.9, 0.9, 0.8), vec3(0.1, 0.4, 0.9), smoothstep(0.0, 1.0, p));
            } else if (uViewMode == 4) {
                vec4 hydroData = sampleHydroSmooth(fV_geoUV);
                vColor = mix(vec3(0.9, 0.8, 0.7), vec3(0.0, 0.4, 0.9), smoothstep(1.0, 7.0, hydroData.b));
                if (vWaterDepth > 0.0001) vColor = vec3(0.0, 0.5, 1.0);
            }
            FragColor = vec4(vColor * (0.6 + 0.4 * diff), 1.0);
            return;
        }

        // --- River Overlay (Segment-based Distance Field) ---
        {
            vec2 res = vec2(512.0, 256.0);
            vec2 pixelPos = fV_geoUV * res;
            vec2 cellCoords = floor(pixelPos);
            vec2 localPos = fract(pixelPos); 
            
            float minDist = 1.0;
            float maxStrahler = 0.0;
            
            const vec2 deltas[8] = vec2[](
                vec2(1,0), vec2(1,1), vec2(0,1), vec2(-1,1),
                vec2(-1,0), vec2(-1,-1), vec2(0,-1), vec2(1,-1)
            );

            // Search 3x3 for any cell (including current) that belongs to a river
            for (int dy = -1; dy <= 1; dy++) {
                for (int dx = -1; dx <= 1; dx++) {
                    vec2 nCoord = cellCoords + vec2(dx, dy);
                    vec2 nUV = (nCoord + 0.5) / res;
                    nUV.x = fract(nUV.x); // Global wrap
                    // Use discrete sample for logic
                    vec4 nData = texture(uHydroMap, nUV);
                    
                    if (nData.b >= 4.0) {
                        // Outflow segment from this neighbor to its target
                        int dir = int(nData.a);
                        if (dir >= 0 && dir < 8) {
                            vec2 p1 = nCoord + 0.5;
                            vec2 p2 = nCoord + 0.5 + deltas[dir];
                            
                            // Segment distance
                            vec2 pa = pixelPos - p1, ba = p2 - p1;
                            float h = clamp(dot(pa, ba) / dot(ba, ba), 0.0, 1.0);
                            float d = length(pa - ba * h);
                            
                            if (d < minDist) {
                                minDist = d;
                                maxStrahler = nData.b;
                            }
                        }
                    }
                }
            }

            if (minDist < 2.0) {
                float simPixelScale = length(fwidth(pixelPos));
                float screenRadius = minDist / max(simPixelScale, 0.0001);
                float targetWidth = (maxStrahler >= 5.0) ? 1.5 : 0.8; 
                
                if (screenRadius < targetWidth) {
                    float edge = smoothstep(targetWidth, targetWidth - 0.5, screenRadius);
                    result = mix(result, vec3(0.01, 0.08, 0.25), 0.7 * edge);
                }
            }
        }

        FragColor = vec4(result, 1.0);
      }

    )";

    terrainProg = compileProgram(terrainVertSrc, terrainFragSrc);
    ut_mvp = glGetUniformLocation(terrainProg, "uMVP");
    ut_model = glGetUniformLocation(terrainProg, "uModel");
    ut_camPos = glGetUniformLocation(terrainProg, "uCamPos");
    ut_lightDir = glGetUniformLocation(terrainProg, "uLightDir");
    ut_viewPos = glGetUniformLocation(terrainProg, "uViewPos");
    ut_time = glGetUniformLocation(terrainProg, "uTime");
    ut_nodePos = glGetUniformLocation(terrainProg, "uNodePos");
    ut_nodeSide = glGetUniformLocation(terrainProg, "uNodeSide");
    ut_nodeUp = glGetUniformLocation(terrainProg, "uNodeUp");
    ut_planetCenterRel = glGetUniformLocation(terrainProg, "uPlanetCenterRel");
    ut_tectonicMap = glGetUniformLocation(terrainProg, "uTectonicMap");
    ut_climateMap = glGetUniformLocation(terrainProg, "uClimateMap");
    ut_hydroMap = glGetUniformLocation(terrainProg, "uHydroMap");
    ut_viewMode = glGetUniformLocation(terrainProg, "uViewMode");
    ut_nodeLevel = glGetUniformLocation(terrainProg, "uNodeLevel");
    ut_localHydroMap = glGetUniformLocation(terrainProg, "uLocalHydroMap");
    ut_hasLocalHydro = glGetUniformLocation(terrainProg, "uHasLocalHydro");

    // --- Vegetation Shader (Instanced) ---
    const char* vegVertSrc = R"(
      #version 330 core
      layout(location=0) in vec3 aPos;
      layout(location=1) in vec3 aNormal;
      layout(location=4) in vec3 iPos;     // Instance position
      layout(location=5) in float iScale;  // Instance scale
      layout(location=6) in float iRot;    // Instance Y-rotation

      uniform mat4 uView;
      uniform mat4 uProj;
      uniform vec3 uPlanetCenter;

      out vec3 vNormal;
      out vec3 vWorldPos;

      void main() {
        // Construct instance rotation matrix (Y-axis)
        float s = sin(iRot);
        float c = cos(iRot);
        mat3 rot = mat3(c, 0, s,  0, 1, 0, -s, 0, c);
        
        // Transform local position
        vec3 pos = (rot * aPos) * iScale;
        
        // Orient to planet surface
        vec3 up = normalize(iPos);
        vec3 right = normalize(cross(vec3(0,1,0), up));
        if (length(right) < 0.01) right = normalize(cross(vec3(1,0,0), up));
        vec3 forward = cross(up, right);
        mat3 orient = mat3(right, up, forward);
        
        vec3 worldPos = orient * pos + iPos;
        vWorldPos = worldPos;
        vNormal = orient * (rot * aNormal);
        
        gl_Position = uProj * uView * vec4(worldPos, 1.0);
      }
    )";

    const char* vegFragSrc = R"(
      #version 330 core
      in vec3 vNormal;
      in vec3 vWorldPos;
      uniform vec3 uLightDir;
      uniform vec3 uViewPos;
      out vec4 FragColor;
      void main() {
        vec3 N = normalize(vNormal);
        vec3 L = normalize(uLightDir);
        float diff = max(dot(N, L), 0.0);
        vec3 color = vec3(0.1, 0.4, 0.05); // Standard forest green
        FragColor = vec4(color * (0.2 + 0.8 * diff), 1.0);
      }
    )";

    vegProg = compileProgram(vegVertSrc, vegFragSrc);
    uv_vp = glGetUniformLocation(vegProg, "uView");
    uv_proj = glGetUniformLocation(vegProg, "uProj");
    uv_lightDir = glGetUniformLocation(vegProg, "uLightDir");
    uv_viewPos = glGetUniformLocation(vegProg, "uViewPos");

    // --- Create Vegetation Geometry ---
    // Simple Tree (cylinder stump + cone top)
    std::vector<Vertex3D> treeVerts;
    std::vector<unsigned int> treeIndices;
    // ... (simplified generation for now or reuse existing)
    
    // For now, let's just initialize the buffers
    glGenVertexArrays(1, &treeVAO);
    glGenBuffers(1, &treeVBO);
    glGenBuffers(1, &treeEBO);
    glGenBuffers(1, &treeInstanceVBO);
    
    initVegetationGeometry();
    sharedPatchMesh = MeshGen::patch(32); // 32x32 segments per patch
    terrain = new Terrain::QuadtreeTerrain(EARTH_RADIUS);
    vegSystem = new Vegetation::VegetationSystem();

    // --- SVO Shader (Camera-Relative, Vertex-Colored Phong) ---
    const char* svoVertSrc = R"(
      #version 330 core
      layout(location=0) in vec3 aPos;
      layout(location=1) in vec3 aNormal;
      layout(location=2) in vec2 aUV;
      layout(location=3) in vec4 aColor;

      uniform mat4 uMVP;
      uniform mat4 uSvoMat; // Transforms SVO local to Camera-Relative World

      out vec3 vRelPos;
      out vec3 vNormal;
      out vec4 vColor;

      void main() {
        vec4 relPos = uSvoMat * vec4(aPos, 1.0);
        vRelPos = relPos.xyz;
        
        // As SvoMat's upper 3x3 is pure rotation (orthonormal), 
        // we can just multiply directly to transform normals:
        vNormal = mat3(uSvoMat) * aNormal;
        
        vColor = aColor;
        gl_Position = uMVP * relPos;
      }
    )";
    const char* svoFragSrc = R"(
      #version 330 core
      in vec3 vRelPos;
      in vec3 vNormal;
      in vec4 vColor;

      uniform float uTime;
      uniform vec3 uLightDir;

      out vec4 FragColor;

      // Simple hash for terrain detail noise
      float hash3(vec3 p) {
        p = fract(p * vec3(443.897, 441.423, 437.195));
        p += dot(p, p.yzx + 19.19);
        return fract((p.x + p.y) * p.z);
      }
      float noise3d(vec3 p) {
        vec3 i = floor(p); vec3 f = fract(p);
        f = f * f * (3.0 - 2.0 * f);
        float n000 = hash3(i), n100 = hash3(i + vec3(1,0,0));
        float n010 = hash3(i + vec3(0,1,0)), n110 = hash3(i + vec3(1,1,0));
        float n001 = hash3(i + vec3(0,0,1)), n101 = hash3(i + vec3(1,0,1));
        float n011 = hash3(i + vec3(0,1,1)), n111 = hash3(i + vec3(1,1,1));
        float nx00 = mix(n000, n100, f.x), nx10 = mix(n010, n110, f.x);
        float nx01 = mix(n001, n101, f.x), nx11 = mix(n011, n111, f.x);
        return mix(mix(nx00, nx10, f.y), mix(nx01, nx11, f.y), f.z);
      }

      void main() {
        vec3 N = normalize(vNormal);
        vec3 L = normalize(uLightDir);
        vec3 V = normalize(-vRelPos);
        vec3 H = normalize(L + V);

        // Triplanar blending weights for natural texture detail
        vec3 blendW = abs(N);
        blendW = pow(blendW, vec3(4.0));
        blendW /= (blendW.x + blendW.y + blendW.z + 0.001);

        // Multi-scale noise for surface micro-detail
        vec3 wp = vRelPos * 500.0; // Scale to meters
        float n1 = noise3d(wp * 0.5) * blendW.x
                 + noise3d(wp.yzx * 0.5) * blendW.y
                 + noise3d(wp.zxy * 0.5) * blendW.z;
        float n2 = noise3d(wp * 2.0) * blendW.x
                 + noise3d(wp.yzx * 2.0) * blendW.y
                 + noise3d(wp.zxy * 2.0) * blendW.z;
        float detail = n1 * 0.6 + n2 * 0.4;

        // Material color with noise variation
        vec3 baseColor = vColor.rgb * (0.85 + 0.3 * detail);

        // Phong lighting (terrain-consistent)
        float ambient = 0.12;
        float diff = max(dot(N, L), 0.0);
        float spec = pow(max(dot(N, H), 0.0), 48.0) * 0.15;

        // Soft shadow via terminator smoothstep
        float terminator = smoothstep(-0.05, 0.15, dot(N, L));

        vec3 result = baseColor * (ambient + diff * 0.85 * terminator) + vec3(spec * terminator);

        // Subtle AO approximation from normal dot up
        float ao = 0.85 + 0.15 * max(N.y, 0.0);
        result *= ao;

        FragColor = vec4(result, 1.0);
      }
    )";
    svoProg = compileProgram(svoVertSrc, svoFragSrc);
    usvo_mvp = glGetUniformLocation(svoProg, "uMVP");
    usvo_lightDir = glGetUniformLocation(svoProg, "uLightDir");
    usvo_viewPos = glGetUniformLocation(svoProg, "uViewPos");
    usvo_svoMat = glGetUniformLocation(svoProg, "uSvoMat");
    svoManager = new SVO::SVOManager();
  }

  // Initialize or resize TAA framebuffers
  void initTAA(int width, int height) {
    if (taaInitialized && taaWidth == width && taaHeight == height) return;

    // Delete old resources if resizing
    if (taaInitialized) {
      glDeleteFramebuffers(2, taaFBO);
      glDeleteTextures(2, taaColorTex);
      glDeleteTextures(1, &taaDepthTex);
    }

    taaWidth = width;
    taaHeight = height;

    // Create two color textures for ping-pong
    glGenTextures(2, taaColorTex);
    for (int i = 0; i < 2; i++) {
        glBindTexture(GL_TEXTURE_2D, taaColorTex[i]);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA16F, width, height, 0, GL_RGBA, GL_HALF_FLOAT, nullptr);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    }

    // Shared depth texture (CRITICAL for reprojection)
    glGenTextures(1, &taaDepthTex);
    glBindTexture(GL_TEXTURE_2D, taaDepthTex);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT24, width, height, 0, GL_DEPTH_COMPONENT, GL_FLOAT, nullptr);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

    // Create two FBOs
    glGenFramebuffers(2, taaFBO);
    for (int i = 0; i < 2; i++) {
      glBindFramebuffer(GL_FRAMEBUFFER, taaFBO[i]);
      glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, taaColorTex[i], 0);
      glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, taaDepthTex, 0);
      GLenum status = glCheckFramebufferStatus(GL_FRAMEBUFFER);
      if (status != GL_FRAMEBUFFER_COMPLETE) {
        printf("TAA FBO %d incomplete: 0x%x\n", i, status);
      }
    }

    // Clear both textures to black
    for (int i = 0; i < 2; i++) {
      glBindFramebuffer(GL_FRAMEBUFFER, taaFBO[i]);
      glClearColor(0, 0, 0, 0);
      glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    }
    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    taaInitialized = true;
  }

  // Begin rendering the current frame into TAA FBO
  void beginTAAPass() {
    int currentIdx = taaFrameIndex % 2;
    glBindFramebuffer(GL_FRAMEBUFFER, taaFBO[currentIdx]);
    glClearColor(0, 0, 0, 0);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  }

  // Resolve TAA: blend current frame with history, output to default framebuffer
  void resolveTAA() {
    int currentIdx = taaFrameIndex % 2;
    int historyIdx = 1 - currentIdx;

    // Bind back to the default framebuffer
    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    glUseProgram(taaProg);

    // Bind current frame to texture unit 0
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, taaColorTex[currentIdx]);
    glUniform1i(glGetUniformLocation(taaProg, "uCurrentFrame"), 0);

    // Bind history frame to texture unit 1
    glActiveTexture(GL_TEXTURE1);
    glBindTexture(GL_TEXTURE_2D, taaColorTex[historyIdx]);
    glUniform1i(glGetUniformLocation(taaProg, "uHistoryFrame"), 1);

    // Bind depth texture to texture unit 2
    glActiveTexture(GL_TEXTURE2);
    glBindTexture(GL_TEXTURE_2D, taaDepthTex);
    glUniform1i(glGetUniformLocation(taaProg, "uDepthTex"), 2);

    // Pass matrices for reprojection
    sendMat4(glGetUniformLocation(taaProg, "uInvViewProj"), invViewProj);
    sendMat4(glGetUniformLocation(taaProg, "uPrevViewProj"), prevViewProj);

    // Blend factor: 10% new, 90% history for smooth accumulation
    // First few frames use more of the current frame to converge faster
    float blend = (taaFrameIndex < 8) ? 0.5f : 0.1f;
    if (taaBlendOverride >= 0.0f) blend = taaBlendOverride; // Allow external override
    glUniform1f(glGetUniformLocation(taaProg, "uBlendFactor"), blend);

    // Render fullscreen quad with alpha blending to composite over existing scene
    glEnable(GL_BLEND);
    glBlendFunc(GL_ONE, GL_ONE_MINUS_SRC_ALPHA); // Premultiplied-alpha style
    glDisable(GL_DEPTH_TEST);
    glDepthMask(GL_FALSE);

    glBindVertexArray(taaVAO);
    glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
    glBindVertexArray(0);

    glEnable(GL_DEPTH_TEST);
    glDepthMask(GL_TRUE);
    glDisable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glActiveTexture(GL_TEXTURE0);

    // Copy resolved result back to history buffer for next frame
    // We read from default FB and write into the current TAA texture
    glBindTexture(GL_TEXTURE_2D, taaColorTex[currentIdx]);
    glCopyTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, 0, 0, taaWidth, taaHeight);

    taaFrameIndex++;
  }

  void beginFrame(const Mat4& viewMat, const Mat4& projMat, const Vec3& cameraPos) {
    view = viewMat;
    proj = projMat;
    camPos = cameraPos;

    // Track matrices for TAA reprojection
    Mat4 currentVP = proj * view;
    // Note: main.cpp will handle setting prevViewProj before this is called or using a setter
    invViewProj = currentVP.inverse();

    glEnable(GL_DEPTH_TEST);
    glDisable(GL_CULL_FACE); // Global default for hollow rocket parts
    glFrontFace(GL_CCW);
  }

  // 绘制单个 Mesh, 指定 Model 矩阵和颜色
  void drawMesh(const Mesh& mesh, const Mat4& model,
                float cr = 1, float cg = 1, float cb = 1, float ca = 1,
                float ambient = 0.15f) {
    glUseProgram(program3d);

    Mat4 mvp = proj * view * model;
    sendMat4(u_mvp, mvp);
    sendMat4(u_model, model);
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
     
     // Dynamic Scaling based on distance (Feature Request: Sun brightness/size change)
     float currentDist = (sunWorldPos - camPos).length();
     float refDist = 149597870.0f; // 1 AU in scaled units (km)
     float distFactor = refDist / currentDist;
     
     // Increase brightness as we get closer (intensity uses non-linear power for visual punch)
     float intensityScale = powf(distFactor, 1.25f);
     intensityScale = fminf(fmaxf(intensityScale, 0.15f), 8.0f);
     
     // Scale size linearly with distance factor
     float sizeScale = distFactor;
     sizeScale = fminf(fmaxf(sizeScale, 0.2f), 10.0f);

     glEnable(GL_BLEND);
     glBlendFunc(GL_SRC_ALPHA, GL_ONE); // Additive blending for light
     glDepthMask(GL_FALSE);
     // Enable depth test so mountains/terrain occlude the sun flare
     glEnable(GL_DEPTH_TEST);
     glDepthFunc(GL_LEQUAL);
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
          glUniform2f(glGetUniformLocation(lensFlareProg, "uScale"), scaleX * sizeScale, scaleY * sizeScale);
          glUniform2f(glGetUniformLocation(lensFlareProg, "uOffset"), ndcPos.x * ndcOffsetMult, ndcPos.y * ndcOffsetMult);
          glUniform4f(ulf_color, r, g, b, 1.0f);
          glUniform1f(ulf_intensity, aFactor * occlusionFade * intensityScale);
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
     glDisable(GL_BLEND);
  }

  // ==== Quadtree Terrain & Vegetation Implementation ====
  void initVegetationGeometry() {
      // 1. Generate Tree Mesh (Stump + Top)
      std::vector<Vertex3D> verts;
      std::vector<unsigned int> indices;
      
      // Stump (Cylinder)
      float radius = 0.15f, height = 0.8f;
      int segs = 8;
      for (int i = 0; i <= segs; i++) {
          float ang = (i / (float)segs) * 2.0f * 3.14159f;
          float cs = cosf(ang), sn = sinf(ang);
          Vertex3D v; v.nx = cs; v.ny = 0; v.nz = sn; v.u = i/(float)segs; v.r=0.4f; v.g=0.25f; v.b=0.1f; v.a=1;
          v.px = cs * radius; v.py = 0; v.pz = sn * radius; v.v = 1; verts.push_back(v); // Bottom
          v.px = cs * radius; v.py = height; v.pz = sn * radius; v.v = 0; verts.push_back(v); // Top
      }
      for (int i = 0; i < segs; i++) {
          int a = i*2, b = a+1, c = a+2, d = a+3;
          indices.push_back(a); indices.push_back(b); indices.push_back(c);
          indices.push_back(b); indices.push_back(d); indices.push_back(c);
      }
      
      // Top (Cone)
      int base = (int)verts.size();
      float cRad = 0.6f, cHeight = 2.0f;
      Vertex3D tip; tip.px=0; tip.py=height+cHeight; tip.pz=0; tip.nx=0; tip.ny=1; tip.nz=0; tip.u=0.5f; tip.v=0; tip.r=0.1f; tip.g=0.35f; tip.b=0.05f; tip.a=1;
      verts.push_back(tip);
      for (int i = 0; i <= segs; i++) {
          float ang = (i / (float)segs) * 2.0f * 3.14159f;
          float cs = cosf(ang), sn = sinf(ang);
          Vertex3D v; v.px = cs*cRad; v.py = height; v.pz = sn*cRad; v.nx=cs; v.ny=0.3f; v.nz=sn; v.u=i/(float)segs; v.v=1; v.r=0.1f; v.g=0.35f; v.b=0.05f; v.a=1;
          verts.push_back(v);
      }
      for (int i = 0; i < segs; i++) {
          indices.push_back(base); indices.push_back(base+1+i); indices.push_back(base+2+i);
      }
      
      treeIndexCount = (int)indices.size();
      glBindVertexArray(treeVAO);
      glBindBuffer(GL_ARRAY_BUFFER, treeVBO);
      glBufferData(GL_ARRAY_BUFFER, verts.size() * sizeof(Vertex3D), verts.data(), GL_STATIC_DRAW);
      glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, treeEBO);
      glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(unsigned int), indices.data(), GL_STATIC_DRAW);
      
      glEnableVertexAttribArray(0); glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex3D), (void*)0);
      glEnableVertexAttribArray(1); glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex3D), (void*)12);
      
      // Instance attributes (loc 4, 5, 6)
      glBindBuffer(GL_ARRAY_BUFFER, treeInstanceVBO);
      glBufferData(GL_ARRAY_BUFFER, 4096 * sizeof(Vegetation::InstanceData), nullptr, GL_DYNAMIC_DRAW);
      glEnableVertexAttribArray(4); glVertexAttribPointer(4, 3, GL_FLOAT, GL_FALSE, sizeof(Vegetation::InstanceData), (void*)0); glVertexAttribDivisor(4, 1);
      glEnableVertexAttribArray(5); glVertexAttribPointer(5, 1, GL_FLOAT, GL_FALSE, sizeof(Vegetation::InstanceData), (void*)12); glVertexAttribDivisor(5, 1);
      glEnableVertexAttribArray(6); glVertexAttribPointer(6, 1, GL_FLOAT, GL_FALSE, sizeof(Vegetation::InstanceData), (void*)16); glVertexAttribDivisor(6, 1);
      glBindVertexArray(0);
  }

  void drawTerrainPatch(const Mesh& mesh, const Mat4& model, float maxElev, float time) {
      glUseProgram(terrainProg);
      Mat4 mvp = proj * view * model;
      sendMat4(ut_mvp, mvp);
      sendMat4(ut_model, model);
      glUniform3f(ut_lightDir, lightDir.x, lightDir.y, lightDir.z);
      glUniform3f(ut_camPos, camPos.x, camPos.y, camPos.z); // Added ut_camPos
      glUniform3f(ut_viewPos, camPos.x, camPos.y, camPos.z);
      glUniform1f(ut_time, time);
      glUniform1f(glGetUniformLocation(terrainProg, "uMaxElevation"), maxElev);
      glUniform1f(glGetUniformLocation(terrainProg, "uPlanetRadius"), 6371000.0f); 
      
      mesh.draw();
  }

  void renderNode(Terrain::TerrainNode* node, float planetRadius, const Mat4& model, float time) {
      if (!node) return;
      if (node->isLeaf) {
          // Bind uniforms for this patch
          glUniform3f(ut_nodePos, node->center.x, node->center.y, node->center.z);
          glUniform3f(ut_nodeSide, node->sideA.x, node->sideA.y, node->sideA.z);
          glUniform3f(ut_nodeUp, node->sideB.x, node->sideB.y, node->sideB.z);
          glUniform1i(ut_nodeLevel, node->level);
          
          if (node->localHydroTex != 0) {
              glActiveTexture(GL_TEXTURE8);
              glBindTexture(GL_TEXTURE_2D, node->localHydroTex);
              glUniform1i(ut_localHydroMap, 8);
              glUniform1i(ut_hasLocalHydro, 1);
          } else {
              glUniform1i(ut_hasLocalHydro, 0);
          }

          sharedPatchMesh.draw();
      } else {
          for (int i = 0; i < 4; i++) {
              renderNode(node->children[i].get(), planetRadius, model, time);
          }
      }
  }

  void drawVegetation(const std::vector<Vegetation::InstanceData>& instances) {
      if (instances.empty()) return;
      glUseProgram(vegProg);
      sendMat4(uv_vp, view);
      sendMat4(uv_proj, proj);
      glUniform3f(uv_lightDir, lightDir.x, lightDir.y, lightDir.z);
      glUniform3f(uv_viewPos, camPos.x, camPos.y, camPos.z);
      
      glBindVertexArray(treeVAO);
      glBindBuffer(GL_ARRAY_BUFFER, treeInstanceVBO);
      glBufferSubData(GL_ARRAY_BUFFER, 0, instances.size() * sizeof(Vegetation::InstanceData), instances.data());
      
      glDrawElementsInstanced(GL_TRIANGLES, treeIndexCount, GL_UNSIGNED_INT, 0, (GLsizei)instances.size());
      glBindVertexArray(0);
  }

  void drawPlanet(const Mesh& mesh, const Mat4& model, BodyType type, float cr, float cg, float cb, float ca, float radius, float time = 0.0f, int bodyIdx = -1) {
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

    // === TERRAIN INTEGRATION FOR EARTH ===
    if (bodyIdx == 3) {
        // --- TERRAIN LOD PASS ---
        glUseProgram(terrainProg);
        
        // --- Explicit State Management ---
        // Ensure Earth terrain is visible by fixing potential state leaks.
        // All terrain patches are wound in Clockwise (CW) order.
        glDisable(GL_CULL_FACE); 
        glFrontFace(GL_CW);
        glEnable(GL_DEPTH_TEST);
        glDepthMask(GL_TRUE);
        
        // --- High-Precision Camera-Relative Rendering ---
        // We use a View-Projection matrix that has NO translation (camera at origin)
        // because the vertex shader computes positions relative to the camera.
        Mat4 viewOnlyRot = view;
        viewOnlyRot.m[12] = 0.0f; viewOnlyRot.m[13] = 0.0f; viewOnlyRot.m[14] = 0.0f;
        Mat4 vpRel = proj * viewOnlyRot;
        
        sendMat4(ut_mvp, vpRel);
        sendMat4(ut_model, model);
        glUniform3f(ut_lightDir, lightDir.x, lightDir.y, lightDir.z);
        glUniform3f(ut_viewPos, camPos.x, camPos.y, camPos.z);
        glUniform3f(ut_camPos, camPos.x, camPos.y, camPos.z);
        glUniform1f(ut_time, time);
        glUniform1f(glGetUniformLocation(terrainProg, "uMaxElevation"), 25.0f); // Kilometers!
        glUniform1f(glGetUniformLocation(terrainProg, "uPlanetRadius"), radius); 
        
        // --- Discrete Tectonic Simulation Texture ---
        if (terrain && terrain->sim && terrain->sim->textureID != 0) {
            glActiveTexture(GL_TEXTURE4);
            glBindTexture(GL_TEXTURE_2D, terrain->sim->textureID);
            glUniform1i(ut_tectonicMap, 4);
        }
        
        // --- Climate Simulation Texture ---
        if (terrain && terrain->climateSim && terrain->getClimateTexture() != 0) {
            glActiveTexture(GL_TEXTURE5);
            glBindTexture(GL_TEXTURE_2D, terrain->getClimateTexture());
            glUniform1i(ut_climateMap, 5);
        }
        if (terrain && terrain->getHydroTexture() != 0) {
            glActiveTexture(GL_TEXTURE6);
            glBindTexture(GL_TEXTURE_2D, terrain->getHydroTexture());
            glUniform1i(ut_hydroMap, 6);
        }
        glUniform1i(ut_viewMode, climateViewMode);
        
        // 1. Planet center from model matrix translation components
        Vec3 planetCenter(model.m[12], model.m[13], model.m[14]);
        Vec3 planetCenterRel = planetCenter - camPos;
        glUniform3f(ut_planetCenterRel, planetCenterRel.x, planetCenterRel.y, planetCenterRel.z);
        
        // 2. Extract rotation axes from model matrix to transform camera into Local Space
        // Columns 0, 1, 2 represent X, Y, Z axes scaled by planet radius
        Vec3 axisX(model.m[0], model.m[1], model.m[2]);
        Vec3 axisY(model.m[4], model.m[5], model.m[6]);
        Vec3 axisZ(model.m[8], model.m[9], model.m[10]);
        // Normalize to get pure rotation
        axisX = axisX.normalized();
        axisY = axisY.normalized();
        axisZ = axisZ.normalized();

        // 3. Project camera-relative vector onto local axes (Inverse Rotation = Transpose)
        Vec3 camPosInertialRel = -planetCenterRel; // vector from planet to camera
        Vec3 camPosLocalRel(
            camPosInertialRel.dot(axisX),
            camPosInertialRel.dot(axisY),
            camPosInertialRel.dot(axisZ)
        );

        // Update Quadtree subdivision based on local-space camera position
        if (terrain) {
            for (int i = 0; i < 6; i++) {
                terrain->updateSubdivision(terrain->roots[i].get(), camPosLocalRel, radius);
                renderNode(terrain->roots[i].get(), radius, model, time);
            }
        }
        
        // --- SVO PASS (If activated and close enough) ---
        float camAlt = (camPos - planetCenter).length() - radius;
        if (svoManager && svoManager->hasActiveChunks()) {
            // Mesh dirty chunks
            SVO::meshAllDirty(svoManager->chunks, svoManager->pool, svoManager->region);

            // Render SVO chunks with proper shader
            glUseProgram(svoProg);
            glEnable(GL_DEPTH_TEST);
            glDepthFunc(GL_LEQUAL);
            glDepthMask(GL_TRUE);
            glEnable(GL_CULL_FACE);
            glCullFace(GL_BACK);

            // Camera-relative VP (no translation in view matrix)
            Mat4 viewOnlyRot2 = view;
            viewOnlyRot2.m[12] = 0.0; viewOnlyRot2.m[13] = 0.0; viewOnlyRot2.m[14] = 0.0;
            Mat4 vpRel2 = proj * viewOnlyRot2;
            sendMat4(usvo_mvp, vpRel2);
            glUniform3f(usvo_lightDir, lightDir.x, lightDir.y, lightDir.z);
            glUniform1f(glGetUniformLocation(svoProg, "uTime"), time);

            // Build SVO-local-to-camera-relative transformation matrix
            Vec3 E = svoManager->region.east;
            Vec3 U = svoManager->region.up;
            Vec3 N = svoManager->region.north;
            Vec3 OLocal = svoManager->region.centerNorm * svoManager->region.centerRadius;

            // Project planet-local vectors into Camera-Inertial space
            Vec3 wE = axisX * E.x + axisY * E.y + axisZ * E.z;
            Vec3 wU = axisX * U.x + axisY * U.y + axisZ * U.z;
            Vec3 wN = axisX * N.x + axisY * N.y + axisZ * N.z;
            Vec3 wO = axisX * OLocal.x + axisY * OLocal.y + axisZ * OLocal.z;
            
            wE = wE.normalized(); wU = wU.normalized(); wN = wN.normalized();
            Vec3 wOCamRel = wO + planetCenterRel;

            Mat4 svoMat;
            svoMat.m[0] = wE.x; svoMat.m[1] = wE.y; svoMat.m[2] = wE.z; svoMat.m[3] = 0;
            svoMat.m[4] = wU.x; svoMat.m[5] = wU.y; svoMat.m[6] = wU.z; svoMat.m[7] = 0;
            svoMat.m[8] = wN.x; svoMat.m[9] = wN.y; svoMat.m[10] = wN.z; svoMat.m[11] = 0;
            svoMat.m[12]= wOCamRel.x; svoMat.m[13]= wOCamRel.y; svoMat.m[14]= wOCamRel.z; svoMat.m[15]= 1;

            sendMat4(usvo_svoMat, svoMat);

            // Draw all active chunks
            for (auto& chunk : svoManager->chunks) {
                if (!chunk.active || chunk.indexCount == 0) continue;
                glBindVertexArray(chunk.vao);
                glDrawElements(GL_TRIANGLES, chunk.indexCount, GL_UNSIGNED_INT, 0);
                glBindVertexArray(0);
            }

            // Restore state
            glDepthFunc(GL_LESS);
            glDisable(GL_CULL_FACE);
        }

        // --- VEGETATION PASS (If close enough) ---
        if (camAlt < 300.0f) { // 300km threshold
            std::vector<Vegetation::InstanceData> instances;
            Vec3 camNorm = (camPos - planetCenter).normalized();
            for (int i = 0; i < 500; i++) {
                float offX = (hash11(i * 7) * 2 - 1) * 0.002f;
                float offZ = (hash11(i * 13) * 2 - 1) * 0.002f;
                Vec3 p = (camNorm + Vec3(offX, 0, offZ)).normalized();
                float h = terrain->getHeight(p) / radius;
                if (h > 0.0f) { 
                    Vegetation::InstanceData id;
                    Vec3 localPos = p * (radius + h * radius);
                    Vec3 worldP = localPos + planetCenter; // Keep coordinates in KM
                    id.pos[0] = (float)worldP.x; id.pos[1] = (float)worldP.y; id.pos[2] = (float)worldP.z;
                    id.scale = (0.5f + 0.5f * hash11(i)) * (0.02f); // 20m trees -> 0.02km
                    id.rot = hash11(i * 3) * 6.28f;
                    instances.push_back(id);
                }
            }
            drawVegetation(instances);
        }
        return;
    }

    glUseProgram(prog);
    Mat4 mvp = proj * view * model;
    sendMat4(mvpLoc, mvp);
    sendMat4(modelLoc, model);
    glUniform3f(lightLoc, lightDir.x, lightDir.y, lightDir.z);
    if (viewLoc != -1) glUniform3f(viewLoc, camPos.x, camPos.y, camPos.z);
    if (colorLoc != -1) glUniform4f(colorLoc, cr, cg, cb, ca);
    // Pass time uniform for animated shaders (Earth clouds, Venus atmosphere, etc.)
    if (timeLoc != -1) glUniform1f(timeLoc, time);
    
    // Pass Saturn ring shadows
    if (bodyIdx == 7) {
        glUniform1f(glGetUniformLocation(prog, "uRingInner"), 1.11f);
        glUniform1f(glGetUniformLocation(prog, "uRingOuter"), 2.35f);
    }
    
    glEnable(GL_CULL_FACE);
    glCullFace(GL_BACK);
    glFrontFace(GL_CW);
    mesh.draw();
    glFrontFace(GL_CCW);
    glDisable(GL_CULL_FACE);
  }

  void drawRing(const Mesh& ringMesh, const Mat4& model, float cr, float cg, float cb, float ca) {
    glUseProgram(ringProgram);
    Mat4 mvp = proj * view * model;
    sendMat4(uri_mvp, mvp);
    sendMat4(uri_model, model);
    glUniform3f(uri_lightDir, lightDir.x, lightDir.y, lightDir.z);
    glUniform4f(uri_baseColor, cr, cg, cb, ca);
    glUniform1f(glGetUniformLocation(ringProgram, "uPlanetRadius"), 1.0f);
    
    ringMesh.draw();
  }

  // 火焰/发光 Billboard (面向相机的面片)
  void drawBillboard(const Vec3& worldPos, float size,
                     float cr, float cg, float cb, float ca) {
    glUseProgram(billboardProg);
    
    // Provide View and Proj separately
    sendMat4(ub_vp, view);
    sendMat4(ub_proj, proj);
    
    glUniform3f(ub_pos, worldPos.x, worldPos.y, worldPos.z);
    glUniform2f(ub_size, size, size);
    glUniform4f(ub_color, cr, cg, cb, ca);
    
    // 加法混合 (火焰发光)
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE);
    glDepthMask(GL_FALSE);
    
    glBindVertexArray(billboardVAO);
    glBindBuffer(GL_ARRAY_BUFFER, billboardVBO);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 0, (void*)0);
    glEnableVertexAttribArray(0);

    glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
    
    glBindVertexArray(0);
    glDepthMask(GL_TRUE);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glDisable(GL_BLEND);
  }

  // 工业级 3D 体积尾焰 (Raymarched Plume)
  void drawExhaustVolumetric(const Mesh& cylinderMesh, const Mat4& model, 
                             float throttle, float expansion, float time,
                             float groundDist, float plumeLen) {
    glUseProgram(exhaustProg);
    
    // Enable additive blending for the flame glow
    glEnable(GL_BLEND);
    glDisable(GL_CULL_FACE);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE); 
    glDepthMask(GL_FALSE); 

    Mat4 mvp = proj * view * model;
    Mat4 invModel = model.inverse();
    sendMat4(uex_mvp, mvp);
    sendMat4(uex_model, model);
    sendMat4(uex_invModel, invModel);
    glUniform3f(uex_viewPos, camPos.x, camPos.y, camPos.z);
    glUniform1f(uex_time, time);
    glUniform1f(uex_throttle, throttle);
    glUniform1f(uex_expansion, expansion);
    glUniform1f(uex_groundDist, groundDist);
    glUniform1f(uex_plumeLen, plumeLen);

    cylinderMesh.draw();

    // Revert state
    glDepthMask(GL_TRUE);
    glEnable(GL_CULL_FACE);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glDisable(GL_BLEND);
    glUseProgram(0);
  }

  void drawAtmosphere(const Mesh& sphereMesh, const Mat4& model, 
                      const Vec3& camPos, const Vec3& lightDir, 
                      const Vec3& planetCenter, float surfaceRadius, float outerRadius, double time, int planetIdx, float sunVisibility, bool showClouds) {
    glUseProgram(atmoProg);

    Mat4 mvp = proj * view * model;
    GLint u_atmo_mvp = glGetUniformLocation(atmoProg, "uMVP");
    GLint u_atmo_model = glGetUniformLocation(atmoProg, "uModel");
    GLint u_camPos = glGetUniformLocation(atmoProg, "uCamPos");
    GLint u_lightDir = glGetUniformLocation(atmoProg, "uLightDir");
    GLint u_planetCenter = glGetUniformLocation(atmoProg, "uPlanetCenter");
    GLint u_innerRadius = glGetUniformLocation(atmoProg, "uInnerRadius");
    GLint u_outerRadius = glGetUniformLocation(atmoProg, "uOuterRadius");
    GLint u_surfaceRadius = glGetUniformLocation(atmoProg, "uSurfaceRadius");
    GLint u_time = glGetUniformLocation(atmoProg, "uTime");
    GLint u_planetIdx = glGetUniformLocation(atmoProg, "uPlanetIdx");
    GLint u_sunVisibility = glGetUniformLocation(atmoProg, "uSunVisibility");

    sendMat4(u_atmo_mvp, mvp);
    sendMat4(u_atmo_model, model);
    glUniform3f(u_camPos, camPos.x, camPos.y, camPos.z);
    glUniform3f(u_lightDir, lightDir.x, lightDir.y, lightDir.z);
    glUniform3f(u_planetCenter, planetCenter.x, planetCenter.y, planetCenter.z);
    
    // Slightly push inner radius into the planet to hide mesh-sphere gaps
    // Increased safety margin to 0.998 to resolve edge cases in terminator shadowing
    float innerRadius = surfaceRadius * 0.9995f; // Tighter shadow sphere for realism
    glUniform1f(u_innerRadius, innerRadius);
    glUniform1f(u_outerRadius, outerRadius);
    glUniform1f(u_surfaceRadius, surfaceRadius);
    glUniform1f(u_time, (float)time);
    glUniform1i(u_planetIdx, planetIdx);
    glUniform1f(u_sunVisibility, sunVisibility);

    // Pass ring shadows for Saturn's atmosphere
    if (planetIdx == 7) {
        glUniform1f(glGetUniformLocation(atmoProg, "uRingInner"), 1.11f);
        glUniform1f(glGetUniformLocation(atmoProg, "uRingOuter"), 2.35f);
    } else {
        glUniform1f(glGetUniformLocation(atmoProg, "uRingOuter"), 0.0f);
    }

    // Industrial Grade: Phase Separation & Time Wrapping
    GLint u_cloudTime = glGetUniformLocation(atmoProg, "uCloudTime");
    GLint u_cloudPhaseSin = glGetUniformLocation(atmoProg, "uCloudPhaseSin");
    GLint u_cloudPhaseCos = glGetUniformLocation(atmoProg, "uCloudPhaseCos");

    // Wrap time for noise (reset every 10,000s) to keep float precision high
    float wrappedTime = (float)fmod(time, 10000.0);
    glUniform1f(u_cloudTime, wrappedTime);

    // Precompute global phase for clouds on CPU using double precision
    // Speed matches shader logic (t * -0.2 * 0.004)
    double phase = time * 0.004 * -0.2;
    glUniform1f(u_cloudPhaseSin, (float)sin(phase));
    glUniform1f(u_cloudPhaseCos, (float)cos(phase));

    // Pass frame index for animated noise (TAA jitter)
    GLint u_frameIdx = glGetUniformLocation(atmoProg, "uFrameIndex");
    glUniform1i(u_frameIdx, taaFrameIndex);

    // Pass cloud toggle
    glUniform1f(glGetUniformLocation(atmoProg, "uShowClouds"), showClouds ? 1.0f : 0.0f);

    // Bind Depth Texture for Physical Terrain Occlusion
    GLint u_res = glGetUniformLocation(atmoProg, "uResolution");
    glUniform2f(u_res, (float)taaWidth, (float)taaHeight);
    GLint u_invProj = glGetUniformLocation(atmoProg, "uInvProj");
    sendMat4(u_invProj, proj.inverse());
    glActiveTexture(GL_TEXTURE7);
    glBindTexture(GL_TEXTURE_2D, taaDepthTex);
    glUniform1i(glGetUniformLocation(atmoProg, "uDepthTex"), 7);

    // --- Graphics State for Volumetric Shell ---
    glDisable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_ONE, GL_ONE_MINUS_SRC_ALPHA);
    glDepthMask(GL_FALSE);
    
    // Front face culling guarantees we draw exactly ONE layer of the bounding sphere per pixel, 
    // eliminating double-drawing artifacts, and it never gets clipped by the near plane.
    glEnable(GL_CULL_FACE);
    glCullFace(GL_FRONT);
    
    sphereMesh.draw();

    // Revert ALL state
    glCullFace(GL_BACK);
    glDisable(GL_BLEND);
    glEnable(GL_DEPTH_TEST);
    glDepthMask(GL_TRUE);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glUseProgram(0);
  }

  // Procedural Starfield + Milky Way background
  void drawSkybox(float vibrancy = 1.0f, float aspect = 1.0f) {
    glUseProgram(skyboxProg);
    glUniform1f(us_skyVibrancy, vibrancy);
    // Compute inverse view-projection matrix for ray reconstruction
    // FIX: Use a stable projection matrix and a view matrix with zero translation
    // This avoids numerical instability when the global macro_near is extreme.
    Mat4 stableProj = Mat4::perspective(0.8f, aspect, 0.1f, 10.0f); 
    Mat4 viewNoTrans = view;
    viewNoTrans.m[12] = 0.0f; viewNoTrans.m[13] = 0.0f; viewNoTrans.m[14] = 0.0f;
    Mat4 vp = stableProj * viewNoTrans;
    // Simple 4x4 matrix inverse (brute force cofactor expansion)
    float inv[16];
    const double* m = vp.m;
    double det;
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

    glBindVertexArray(skyboxVAO);
    glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
    glBindVertexArray(0);

    glEnable(GL_DEPTH_TEST);
    glDepthMask(GL_TRUE);
  }

  // Camera-facing dynamic ribbon (trajectory trail) with per-vertex colors
  void drawRibbon(const std::vector<Vec3>& points, const std::vector<Vec4>& colors, float width) {
    if (points.size() < 2 || points.size() != colors.size()) return;

    struct RibVert { float px, py, pz; float cr, cg, cb, ca; float side; };
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
      Vec3 pL = p + right * halfW;
      stripVerts.push_back({(float)pL.x, (float)pL.y, (float)pL.z, (float)colors[i].x, (float)colors[i].y, (float)colors[i].z, (float)colors[i].w, 1.0f});
      Vec3 pR = p - right * halfW;
      stripVerts.push_back({(float)pR.x, (float)pR.y, (float)pR.z, (float)colors[i].x, (float)colors[i].y, (float)colors[i].z, (float)colors[i].w, -1.0f});
    }

    glUseProgram(ribbonProg);
    Mat4 mvp = proj * view; // No model matrix needed, points are in world space
    sendMat4(ur_mvp, mvp);

    // Buffer dynamic data
    glBindVertexArray(ribbonVAO);
    glBindBuffer(GL_ARRAY_BUFFER, ribbonVBO);
    glBufferData(GL_ARRAY_BUFFER, stripVerts.size() * sizeof(RibVert), stripVerts.data(), GL_DYNAMIC_DRAW);
    
    // Re-establish vertex layout
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(RibVert), (void*)0);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, sizeof(RibVert), (void*)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(2, 1, GL_FLOAT, GL_FALSE, sizeof(RibVert), (void*)(7 * sizeof(float)));
    glEnableVertexAttribArray(2);

    glDepthMask(GL_FALSE);
    glDisable(GL_CULL_FACE); // Ribbons are 2D strips, must not be culled
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
