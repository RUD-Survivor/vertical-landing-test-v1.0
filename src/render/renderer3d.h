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
#include <sstream>
#include "terrain_system.h"
#include "vegetation_system.h"
#include "svo_system.h"
#include "svo_meshing.h"
#include "cloud_system.h"

inline void sendMat4(GLint loc, const Mat4& m) {
    float arr[16];
    m.toFloatArray(arr);
    glUniformMatrix4fv(loc, 1, GL_FALSE, arr);
}

// ==========================================================
// GLSL #include 预处理 + 文件加载 (OpenGL 运行时用)
// ==========================================================
#ifndef SHADER_DIR
#define SHADER_DIR "src/render/shaders/gl/"
#endif

static constexpr int kMaxIncludeDepth = 16;

inline std::string readFile(const std::string& path) {
    std::ifstream f(path);
    if (!f.is_open()) { fprintf(stderr, "[shader] Cannot open: %s\n", path.c_str()); return ""; }
    std::stringstream ss; ss << f.rdbuf(); return ss.str();
}

inline std::string resolveIncludes(const std::string& source,
                                    const std::string& baseDir, int depth = 0) {
    if (depth > kMaxIncludeDepth) return source;
    std::string result;
    std::istringstream ss(source);
    std::string line;
    while (std::getline(ss, line)) {
        size_t p = line.find_first_not_of(" \t\r");
        if (p != std::string::npos && line.compare(p, 8, "#include") == 0) {
            size_t q = line.find('"', p + 8);
            size_t r = (q != std::string::npos) ? line.find('"', q + 1) : std::string::npos;
            if (q != std::string::npos && r != std::string::npos) {
                std::string inc = readFile(baseDir + line.substr(q + 1, r - q - 1));
                if (!inc.empty()) { result += resolveIncludes(inc, baseDir, depth + 1) + "\n"; continue; }
            }
        }
        result += line + "\n";
    }
    return result;
}

inline std::string loadShaderFile(const std::string& filename) {
    std::string src = readFile(std::string(SHADER_DIR) + filename);
    if (src.empty()) return "";
    return resolveIncludes(src, SHADER_DIR);
}

// ==========================================================
// 3D 顶点格式 (Vertex3D)
// 定义了渲染一个 3D 点所需的所有属性。
// ========================================================== 
struct Vertex3D {
  float px, py, pz;     // 世界/局部空间位置
  float nx, ny, nz;     // 法线向量 (用于光照计算)
  float u, v;           // 纹理贴图坐标
  float r, g, b, a;     // 顶点颜色
};

// ==========================================================
// Mesh — 3D 网格模型
// 封装了 OpenGL 的 VAO/VBO/EBO 排队逻辑，负责将几何数据上传到 GPU。
// ==========================================================
struct Mesh {
  GLuint vao = 0, vbo = 0, ebo = 0;
  int indexCount = 0;
#ifdef USE_VULKAN
  std::vector<Vertex3D> cpuVerts;
  std::vector<uint32_t> cpuIndices;
#endif

  // Geometric properties for automatic scaling
  float minX = 0, minY = 0, minZ = 0;
  float maxX = 0, maxY = 0, maxZ = 0;
  float width = 1.0f, height = 1.0f, depth = 1.0f;
  float centerX = 0, centerY = 0, centerZ = 0;

  // 将顶点与索引数据上传至显存
  void upload(const std::vector<Vertex3D>& verts,
              const std::vector<unsigned int>& indices) {
    indexCount = (int)indices.size();
#ifdef USE_VULKAN
    cpuVerts = verts;
    cpuIndices.assign(indices.begin(), indices.end());
    return;
#endif
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
#ifndef USE_VULKAN
    if (!vao || indexCount == 0) return;
    glBindVertexArray(vao);
    glDrawElements(GL_TRIANGLES, indexCount, GL_UNSIGNED_INT, 0);
    glBindVertexArray(0);
#endif
  }

  void destroy() {
#ifndef USE_VULKAN
    if (vao) glDeleteVertexArrays(1, &vao);
    if (vbo) glDeleteBuffers(1, &vbo);
    if (ebo) glDeleteBuffers(1, &ebo);
    vao = vbo = ebo = 0;
#endif
  }
};

// ==========================================================
// Texture — 2D 纹理对象
// 负责管理显存中的图像数据，支持不同的贴图槽位。
// ==========================================================
struct Texture {
  GLuint id = 0;
  int width = 0, height = 0;

  void destroy() {
    if (id) glDeleteTextures(1, &id);
    id = 0;
  }

  // 将纹理绑定到指定的着色器槽位 (Slot)
  void bind(GLuint slot = 0) const {
    glActiveTexture(GL_TEXTURE0 + slot);
    glBindTexture(GL_TEXTURE_2D, id);
  }
};

// ==========================================================
// MeshGen — 几何体生成器
// 提供了一组静态函数，用于快速生成球体、圆柱、圆锥等基础形状。
// ==========================================================
namespace MeshGen {

// 生成球体 (UV Sphere)
// 广泛用于行星、恒星的基础几何模型。
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
  // 生成三角形索引
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
  // 简易 TGA 图像加载器 (支持 24/32 位无压缩 TGA)
  // 用于加载各种贴图（地球、木星、背景等）
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

    // BGR(A) 转换为 RGB(A)，因为 OpenGL 通常期望 RGB 顺序
    for (int i = 0; i < size; i += (bpp / 8)) {
        unsigned char tmp = data[i];
        data[i] = data[i + 2];
        data[i + 2] = tmp;
    }

#ifdef USE_VULKAN
    tex.width = w; tex.height = h;
    return tex;
#endif
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

  CloudSystem cloudSystem;

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
  GLint ua_camPos = -1, ua_planetCenter = -1, ua_innerRadius = -1, ua_outerRadius = -1, ua_surfaceRadius = -1;
  GLint ua_time = -1, ua_planetIdx = -1, ua_sunVisibility = -1, ua_showClouds = -1;
  GLint ua_ringInner = -1, ua_ringOuter = -1;
  GLint ua_res = -1, ua_invProj = -1, ua_depthTex = -1;
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
    // --- 标准 3D 着色器 (Phong 光照模型) ---
    // 用于渲染火箭组件和其他普通 3D 物体。
    std::string vertSrc = loadShaderFile("mesh.vert");

    std::string fragSrc = loadShaderFile("mesh.frag");

    program3d = compileProgram(vertSrc.c_str(), fragSrc.c_str());
    u_mvp = glGetUniformLocation(program3d, "uMVP");
    u_model = glGetUniformLocation(program3d, "uModel");
    u_lightDir = glGetUniformLocation(program3d, "uLightDir");
    u_viewPos = glGetUniformLocation(program3d, "uViewPos");
    u_baseColor = glGetUniformLocation(program3d, "uBaseColor");
    u_ambientStr = glGetUniformLocation(program3d, "uAmbientStr");
    u_sampler = glGetUniformLocation(program3d, "uSampler");
    u_hasTexture = glGetUniformLocation(program3d, "uHasTexture");

    // --- 地球着色器 (RSS-Reborn 级程序化渲染) ---
    // 这是最核心的着色器之一：它不需要外部纹理，而是通过数学噪声实时生成陆地、海洋和山脉。
    std::string earthFragSrc = loadShaderFile("earth.frag");

    earthProgram = compileProgram(vertSrc.c_str(), earthFragSrc.c_str());
    ue_mvp = glGetUniformLocation(earthProgram, "uMVP");
    ue_model = glGetUniformLocation(earthProgram, "uModel");
    ue_lightDir = glGetUniformLocation(earthProgram, "uLightDir");
    ue_viewPos = glGetUniformLocation(earthProgram, "uViewPos");
    ue_time = glGetUniformLocation(earthProgram, "uTime");

    // --- Gas Giant Shader ---
    std::string gasGiantFragSrc = loadShaderFile("gas_giant.frag");
    gasGiantProgram = compileProgram(vertSrc.c_str(), gasGiantFragSrc.c_str());
    ugg_mvp = glGetUniformLocation(gasGiantProgram, "uMVP");
    ugg_model = glGetUniformLocation(gasGiantProgram, "uModel");
    ugg_lightDir = glGetUniformLocation(gasGiantProgram, "uLightDir");
    ugg_viewPos = glGetUniformLocation(gasGiantProgram, "uViewPos");
    ugg_baseColor = glGetUniformLocation(gasGiantProgram, "uBaseColor");

    // --- Barren/Crater Shader ---
    std::string barrenFragSrc = loadShaderFile("barren.frag");
    barrenProgram = compileProgram(vertSrc.c_str(), barrenFragSrc.c_str());
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
    ringProgram = compileProgram(vertSrc.c_str(), ringFragSrc);
    uri_mvp = glGetUniformLocation(ringProgram, "uMVP");
    uri_model = glGetUniformLocation(ringProgram, "uModel");
    uri_lightDir = glGetUniformLocation(ringProgram, "uLightDir");
    uri_baseColor = glGetUniformLocation(ringProgram, "uBaseColor");

    lightDir = Vec3(0.5f, 0.8f, 0.3f).normalized();

    // ===== Helper: compile planet shader and fetch uniforms =====
    auto setupPlanetProg = [&](GLuint& prog, PlanetUniforms& u, const char* fragSrc) {
        prog = compileProgram(vertSrc.c_str(), fragSrc);
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
    std::string mercuryFragSrc = loadShaderFile("mercury.frag");
    setupPlanetProg(mercuryProgram, u_mercury, mercuryFragSrc.c_str());

    // ===== 2. VENUS SHADER =====
    // Thick multi-layer sulfuric acid cloud deck, atmospheric super-rotation,
    // subtle sub-surface glow (volcanism) visible through thin cloud gaps.
    std::string venusFragSrc = loadShaderFile("venus.frag");
    setupPlanetProg(venusProgram, u_venus, venusFragSrc.c_str());

    // ===== 3. MOON (Luna) SHADER =====
    // Maria (dark basalt plains) vs highlands, impact craters with ejecta rays,
    // regolith color variation gray-to-brown, very low albedo.
    std::string moonFragSrc = loadShaderFile("moon.frag");
    setupPlanetProg(moonProgram, u_moon, moonFragSrc.c_str());

    // ===== 4. MARS SHADER =====
    // Iron oxide red-orange terrain, polar CO2 ice caps, Olympus Mons volcanic shield,
    // Valles Marineris canyon, dust storms, thin atmosphere rim.
    std::string marsFragSrc = loadShaderFile("mars.frag");
    setupPlanetProg(marsProgram, u_mars, marsFragSrc.c_str());

    // ===== 5. JUPITER SHADER =====
    // Detailed alternating zonal bands, Great Red Spot anticyclone, turbulent eddies,
    // ammonia crystal storms, strong belt/zone color contrast.
    std::string jupiterFragSrc = loadShaderFile("jupiter.frag");
    setupPlanetProg(jupiterProgram, u_jupiter, jupiterFragSrc.c_str());

    // ===== 6. SATURN SHADER =====
    // Muted gold/amber banding, hexagonal polar vortex, haze layer,
    // more subtle contrast than Jupiter.
    std::string saturnFragSrc = loadShaderFile("saturn.frag");
    setupPlanetProg(saturnProgram, u_saturn, saturnFragSrc.c_str());

    // ===== 7. URANUS SHADER =====
    // Pale teal/cyan from methane absorption, subtle banding, polar brightening,
    // nearly featureless appearance with very faint cloud structures.
    std::string uranusFragSrc = loadShaderFile("uranus.frag");
    setupPlanetProg(uranusProgram, u_uranus, uranusFragSrc.c_str());

    // ===== 8. NEPTUNE SHADER =====
    // Deep azure blue (strongest methane absorption), Great Dark Spot analog,
    // bright methane cirrus streaks, vivid blue-to-indigo band contrast,
    // dynamic cloud features.
    std::string neptuneFragSrc = loadShaderFile("neptune.frag");
    setupPlanetProg(neptuneProgram, u_neptune, neptuneFragSrc.c_str());

    // ===== Enhanced Ring Shader (Cassini Division, A/B/C rings) =====
    // Overwrite ring program with upgraded version
    std::string ringFragSrcV2 = loadShaderFile("ring.frag");
    // Re-compile ring shader with enhanced version
    ringProgram = compileProgram(vertSrc.c_str(), ringFragSrcV2.c_str());
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
    std::string bbVertSrc = loadShaderFile("billboard.vert");
    std::string bbFragSrc = loadShaderFile("billboard.frag");
    billboardProg = compileProgram(bbVertSrc.c_str(), bbFragSrc.c_str());
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

    // --- 大气层顶点着色器 ---
    std::string atmoVertSrc = loadShaderFile("atmo.vert");

    // --- 大气层片段着色器 (核心：基于物理权重的分步光线步进) ---
    // 该着色器模拟了瑞利散射、米氏散射和臭氧吸收，实现了极其逼真的星球边缘和日落效果。
    std::string s_atmoFrag1 = loadShaderFile("atmosphere_preamble.frag");
    cloudSystem.loadGLSL("src/render/shaders/cloud.glsl");
    std::string atmoFragSrc = std::string(s_atmoFrag1)
        + cloudSystem.glslSource()
    + R"(
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
          // Uniform steps for atmospheric gases (preserves Rayleigh/Mie/ozone accuracy over full path)
          float stepSize = (t1 - start) / float(uLightSteps);
          for (int i = 0; i < 20; i++) {
              if (i >= uLightSteps) break;
              vec3 pos = p + lightDir * (start + (float(i) + 0.5) * stepSize);
              float h = length(pos - uPlanetCenter) - uSurfaceRadius;
              if (h < -0.1) { dR = 1e6; dM = 1e6; dO = 1e6; dC = 1e6; return; }
              dR += exp(-h / gHRayleigh) * stepSize;
              dM += exp(-h / gHMie) * stepSize;
              dO += ozoneDensity(h) * stepSize;
          }
          // Exponential steps for cloud self-shadow — concentrate samples near p where the
          // shadow gradient is steepest (cloud surface), then coarsen outward.
          // 7 steps: 0.08→0.16→0.32→0.64→1.28→2.56→5.12 km = 10.16 km total (covers cloud layer).
          // First step center at 40 m: resolves 80 m bumps casting shadows on neighbors.
          float cStep = 0.08;
          float cTPos = cStep * 0.5;
          for (int j = 0; j < 7; j++) {
              if (cTPos >= t1) break;
              vec3 cPos = p + lightDir * cTPos;
              float cH = max(length(cPos - uPlanetCenter) - uSurfaceRadius, 0.0);
              dC += cloudDensity(cPos, cH, false) * cStep;
              cTPos += cStep;
              cStep *= 2.0;
          }
      }
)"
R"(
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
          
          // ── tFar: root-cause fix for atmospheric polygon banding ───────────────
          // Problem: the depth buffer returns the terrain *mesh* distance.  LOD creates
          // large polygons far from the camera; every pixel in the same polygon gets the
          // same terrainDist → same tFar → identical atmospheric color within the polygon,
          // sharp color jump at polygon boundaries → horizontal banding rings.
          //
          // Root-cause solution (same as Bruneton 2008/2017, KSP, NMS, SpaceEngine, etc.):
          // Use the smooth mathematical sphere intersection (analyticalSurf) as tFar for
          // ALL distant terrain.  The analytical sphere varies continuously with ray
          // direction and contains no polygon steps → zero banding, regardless of LOD.
          //
          // The only exception: objects closer than 5 km (rocket body, launch pad, close
          // terrain) still use the depth buffer so they correctly occlude the atmosphere.
          // At <5 km the mesh is dense enough that polygon steps create no visible bands.
          {
              // MUST use uSurfaceRadius (actual terrain sphere), NOT uInnerRadius.
              // uInnerRadius = surfaceRadius * 0.9995 is 3.19 km below the actual surface.
              // From 1 km altitude, rays within ±3.3° of horizontal miss uInnerRadius entirely
              // → fall into the near-horizontal fallback → distant ocean (50-100 km) has
              // no tFar clip → atmosphere over-integrates → bright "horizon break" band.
              // With uSurfaceRadius the blind zone shrinks to ±1°, essentially just the tangent.
              float tSurf0, tSurf1;
              if (dVal < 1.0) {
                  float analyticalSurf = 1e6;
                  if (intersectSphere(uCamPos, rayDir, uSurfaceRadius, tSurf0, tSurf1) && tSurf0 > 0.0)
                      analyticalSurf = tSurf0;

                  if (analyticalSurf < 1e5) {
                      // Surface sphere found: use smooth sphere for distant terrain; depth buffer
                      // only for close foreground objects (rocket, launch pad, etc.).
                      tFar = min(tFar, terrainDist < 5.0 ? terrainDist : analyticalSurf);
                  } else {
                      // Truly tangential ray (within ~1° of horizontal) — sphere not intersected.
                      // Only clip tFar for close objects; tangent-horizon rays naturally fade.
                      if (terrainDist < 20.0) tFar = min(tFar, terrainDist);
                  }
              } else {
                  // Sky pixel: clamp at surface sphere so the ray doesn't leave the atmosphere.
                  if (intersectSphere(uCamPos, rayDir, uSurfaceRadius, tSurf0, tSurf1) && tSurf0 > 0.0)
                      tFar = min(tFar, tSurf0);
              }
          }
          
          // When camera is inside, ray starts from camera position
          tNear = max(tNear, 0.0);
          float segmentLength = tFar - tNear;
          if (segmentLength <= 0.0) discard;
          
          // Fixed 2 km step: the root cause of horizontal banding is adaptive step size.
          // segmentLength/N varies between adjacent rows (different elevation angles have
          // different path lengths), so step size varies → optical depth integration error
          // varies → adjacent rows get different colors → visible horizontal bands.
          // A fixed step size gives every pixel identical integration quality: no banding.
          // PRIMARY_STEPS=512 covers 1024 km, handling the ~1100 km horizon ray from 5 km alt.
          const float STEP_KM = 2.0;
          // 2D IGN jitter: using only gl_FragCoord.y means all pixels in the same screen row
          // share identical jitter → when looking straight down, same screen-y = same ring
          // radius → perfect concentric rings. Adding x breaks the radial symmetry.
          float ign = fract(52.9829189 * fract(dot(gl_FragCoord.xy, vec2(0.06711056, 0.00583715))));
          float jitter = fract(ign + float(uFrameIndex % 64) * 0.6180339887);
          float tCur = tNear + jitter * STEP_KM;

          vec3 sumRayleigh = vec3(0.0);
          vec3 sumMie = vec3(0.0);
          vec3 sumCloud = vec3(0.0);
          float optDepthR = 0.0;
          float optDepthM = 0.0;
          float optDepthO = 0.0;
          float optDepthC = 0.0;
          float cosTheta = dot(rayDir, uLightDir);

          for (int i = 0; i < PRIMARY_STEPS; i++) {
              if (tCur >= tFar) break;
              float dt = min(STEP_KM, tFar - tCur);
              vec3 pos = uCamPos + rayDir * (tCur + dt * 0.5);
              float h = max(length(pos - uPlanetCenter) - uSurfaceRadius, 0.0);

              float dR = exp(-h / gHRayleigh) * dt;
              float dM = exp(-h / gHMie) * dt;
              float dO = ozoneDensity(h) * dt;
              optDepthR += dR;
              optDepthM += dM;
              optDepthO += dO;

              // Light ray optical depth to sun (no cloud in primary pass)
              float lightR, lightM, lightO, lightC_dummy;
              getOpticalDepth(pos, uLightDir, lightR, lightM, lightO, lightC_dummy);

              // Total extinction including ozone absorption
              vec3 tauBase = gRayleighCoeff * (optDepthR + lightR)
                           + gMieCoeff * 1.1 * (optDepthM + lightM)
                           + gOzoneCoeff * (optDepthO + lightO);
              vec3 attenuationBase = exp(-tauBase);

              sumRayleigh += dR * attenuationBase;
              sumMie += dM * attenuationBase;

              tCur += STEP_KM;

              // Early Ray Termination
              float maxTransmittance = max(attenuationBase.x, max(attenuationBase.y, attenuationBase.z));
              if (maxTransmittance < 0.01) break;
          }
)"
R"(
          // Cloud Pass (marchClouds from cloud.glsl)
          marchClouds(rayDir, tFar, cosTheta, optDepthR, optDepthM, optDepthO, sumCloud, optDepthC);
          // ─────────────────────────────────────────────────────────────────────

          // Phase functions
          float phaseR = rayleighPhase(cosTheta);
          float phaseM = miePhase(cosTheta);
          float phaseC = cloudPhase(cosTheta);

          // Ambient multi-scattering for clouds (prevents black clouds on dark side)
          float ambientPhaseC = 0.5;
          vec3 ambientCloud = sumCloud * gCloudScattering * ambientPhaseC * 0.4;

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

          // F3 debug modes (cycle with F3 key):
          //   1 = coverage grayscale (pre-smoothstep)
          //   2 = curl field color: R=curlU, G=curlV, B=0.5
          //       vortex pattern (colors rotate around center) → curl working
          //       uniform color → amplitude too small / EPS wrong
          //       salt-and-pepper noise → EPS too small
          if (uDebugCoverage > 0.5 && uPlanetIdx == 3) {
              float cOuter = uSurfaceRadius + gCloudMaxAlt;
              float dbgN, dbgF;
              if (intersectSphere(uCamPos, rayDir, cOuter, dbgN, dbgF) && dbgF > 0.0) {
                  vec3 dbgPos = uCamPos + rayDir * max(dbgN, 0.0);
                  vec3 dbgSph = normalize(dbgPos - uPlanetCenter);
                  float dt = uCloudTime * 0.004;
                  if (uDebugCoverage > 1.5) {
                      // Curl field visualization
                      float latW = dbgSph.z * dbgSph.z * dbgSph.z * 0.3;
                      float sB2 = sin(latW); float cB2 = cos(latW);
                      float s1d = uCloudPhaseSin * cB2 + uCloudPhaseCos * sB2;
                      float c1d = uCloudPhaseCos * cB2 - uCloudPhaseSin * sB2;
                      vec3 cs2 = vec3(dbgSph.x*c1d - dbgSph.y*s1d, dbgSph.x*s1d + dbgSph.y*c1d, dbgSph.z);
                      vec3 ax2 = normalize(cross(cs2, vec3(0.0, 0.0, 1.0) + cs2 * 0.01));
                      vec3 va2 = cross(cs2, ax2);
                      const float EPSD = 0.06;
                      vec3 slD = vec3(dt * 0.003, 0.0, dt * 0.002);
                      // scale 1.5 matches rawCoverage; raw curl ≈ ±0.36 → scale by 1.8 fills [0,1]
                      float pu = fbm(cs2 * 1.5 + ax2 * EPSD + slD, 2);
                      float mu = fbm(cs2 * 1.5 - ax2 * EPSD + slD, 2);
                      float pv = fbm(cs2 * 1.5 + va2 * EPSD + slD, 2);
                      float mv = fbm(cs2 * 1.5 - va2 * EPSD + slD, 2);
                      float cU = -(pv - mv);
                      float cV =  (pu - mu);
                      // vortex center → colors rotate around it; smooth grad → too-large vortices
                      FragColor = vec4(cU * 1.8 + 0.5, cV * 1.8 + 0.5, 0.5, 1.0);
                  } else {
                      float cov = rawCoverage(dbgSph, dt);
                      FragColor = vec4(vec3(cov), 1.0);
                  }
                  return;
              }
              FragColor = vec4(0.0, 0.0, 0.0, 1.0);
              return;
          }

          FragColor = vec4(color, opacity);
      }
    )";
    atmoProg = compileProgram(atmoVertSrc.c_str(), atmoFragSrc.c_str());
    ua_mvp = glGetUniformLocation(atmoProg, "uMVP");
    ua_model = glGetUniformLocation(atmoProg, "uModel");
    ua_camPos = glGetUniformLocation(atmoProg, "uCamPos");
    ua_lightDir = glGetUniformLocation(atmoProg, "uLightDir");
    ua_planetCenter = glGetUniformLocation(atmoProg, "uPlanetCenter");
    ua_innerRadius = glGetUniformLocation(atmoProg, "uInnerRadius");
    ua_outerRadius = glGetUniformLocation(atmoProg, "uOuterRadius");
    ua_surfaceRadius = glGetUniformLocation(atmoProg, "uSurfaceRadius");
    ua_time = glGetUniformLocation(atmoProg, "uTime");
    ua_planetIdx = glGetUniformLocation(atmoProg, "uPlanetIdx");
    ua_sunVisibility = glGetUniformLocation(atmoProg, "uSunVisibility");
    ua_showClouds = glGetUniformLocation(atmoProg, "uShowClouds");
    ua_ringInner = glGetUniformLocation(atmoProg, "uRingInner");
    ua_ringOuter = glGetUniformLocation(atmoProg, "uRingOuter");
    ua_res = glGetUniformLocation(atmoProg, "uResolution");
    ua_invProj = glGetUniformLocation(atmoProg, "uInvProj");
    ua_depthTex = glGetUniformLocation(atmoProg, "uDepthTex");
    cloudSystem.init(atmoProg);

    // --- Ribbon Shader (Trajectory Trails) ---
    std::string ribbonVertSrc = loadShaderFile("ribbon.vert");
    std::string ribbonFragSrc = loadShaderFile("ribbon.frag");
    ribbonProg = compileProgram(ribbonVertSrc.c_str(), ribbonFragSrc.c_str());
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
    std::string lfVertSrc = loadShaderFile("lens_flare.vert");
    std::string lfFragSrc = loadShaderFile("lens_flare.frag");
    
    lensFlareProg = compileProgram(lfVertSrc.c_str(), lfFragSrc.c_str());
    ulf_sunScreenPos = glGetUniformLocation(lensFlareProg, "uSunScreenPos");
    ulf_aspect = glGetUniformLocation(lensFlareProg, "uAspect");
    ulf_color = glGetUniformLocation(lensFlareProg, "uColor");
    ulf_intensity = glGetUniformLocation(lensFlareProg, "uIntensity");

    // Re-use billboard VAO/VBO for Lens Flare since both are just 2D quads
    lfVAO = billboardVAO;
    lfVBO = billboardVBO;

    // --- Skybox Shader (Procedural Starfield + Milky Way) ---
    std::string skyboxVertSrc = loadShaderFile("skybox.vert");
    std::string skyboxFragSrc = loadShaderFile("skybox.frag");
    skyboxProg = compileProgram(skyboxVertSrc.c_str(), skyboxFragSrc.c_str());
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
    std::string exVertSrc = loadShaderFile("exhaust.vert");
    std::string exFragSrc = loadShaderFile("exhaust.frag");
    exhaustProg = compileProgram(exVertSrc.c_str(), exFragSrc.c_str());
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
    std::string taaVertSrc = loadShaderFile("taa.vert");
    std::string taaFragSrc = loadShaderFile("taa.frag");
    taaProg = compileProgram(taaVertSrc.c_str(), taaFragSrc.c_str());
    taaVAO = skyboxVAO; // Reuse the fullscreen quad

    // --- Terrain Shader (High-res displacement mapping) ---
    std::string terrainVertSrc = loadShaderFile("terrain.vert");

    std::string terrainFragSrc = loadShaderFile("terrain.frag");

    terrainProg = compileProgram(terrainVertSrc.c_str(), terrainFragSrc.c_str());
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
    std::string vegVertSrc = loadShaderFile("vegetation.vert");

    std::string vegFragSrc = loadShaderFile("vegetation.frag");

    vegProg = compileProgram(vegVertSrc.c_str(), vegFragSrc.c_str());
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
    // Re-bake weather map using procedural terrain (replaces ellipse mask)
    if (terrain->sim && !terrain->sim->gridHeight.empty()) {
        cloudSystem.rebakeWeather(terrain->sim->gridHeight,
            terrain->sim->width, terrain->sim->height,
            Tectonic::TectonicSimulator::CONT_THRESHOLD);
    }
    vegSystem = new Vegetation::VegetationSystem();

    // --- SVO Shader (Camera-Relative, Vertex-Colored Phong) ---
    std::string svoVertSrc = loadShaderFile("svo.vert");
    std::string svoFragSrc = loadShaderFile("svo.frag");
    svoProg = compileProgram(svoVertSrc.c_str(), svoFragSrc.c_str());
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

    // Blend factor: current-frame weight (higher = less ghosting, less AA smoothing)
    float blend = (taaFrameIndex < 4) ? 0.6f : 0.2f;
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
    glDepthMask(GL_TRUE);
    glDisable(GL_BLEND);
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
        
        // CRITICAL FIX: Restore global GL state before returning
        // If we don't restore glFrontFace(GL_CCW), the subsequent drawAtmosphere 
        // will cull the wrong faces (culling back instead of front), making it invisible!
        glFrontFace(GL_CCW);
        glDisable(GL_CULL_FACE);
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
    // Front face culling guarantees we draw exactly ONE layer of the bounding sphere per pixel,
    // eliminating double-drawing artifacts, and it never gets clipped by the near plane.
    // NOTE: MeshGen::sphere generates vertices with CW winding on the outside!
    // Since GL_CCW is the standard front face, the outer shell is considered GL_BACK.
    // To cull the outer shell and KEEP the inner shell, we must cull GL_BACK.
    glFrontFace(GL_CCW);
    glEnable(GL_CULL_FACE);
    glCullFace(GL_BACK);
    glUseProgram(atmoProg);

    Mat4 mvp = proj * view * model;
    sendMat4(ua_mvp, mvp);
    sendMat4(ua_model, model);
    glUniform3f(ua_camPos, camPos.x, camPos.y, camPos.z);
    glUniform3f(ua_lightDir, lightDir.x, lightDir.y, lightDir.z);
    glUniform3f(ua_planetCenter, planetCenter.x, planetCenter.y, planetCenter.z);
    
    // Slightly push inner radius into the planet to hide mesh-sphere gaps
    // Increased safety margin to 0.998 to resolve edge cases in terminator shadowing
    float innerRadius = surfaceRadius * 0.9995f; // Tighter shadow sphere for realism
    glUniform1f(ua_innerRadius, innerRadius);
    glUniform1f(ua_outerRadius, outerRadius);
    glUniform1f(ua_surfaceRadius, surfaceRadius);
    glUniform1f(ua_time, (float)time);
    glUniform1i(ua_planetIdx, planetIdx);
    glUniform1f(ua_sunVisibility, sunVisibility);

    // Pass ring shadows for Saturn's atmosphere
    if (planetIdx == 7) {
        glUniform1f(ua_ringInner, 1.11f);
        glUniform1f(ua_ringOuter, 2.35f);
    } else {
        glUniform1f(ua_ringOuter, 0.0f);
    }

    // Pass cloud toggle
    glUniform1f(ua_showClouds, showClouds ? 1.0f : 0.0f);

    // Bind Depth Texture for Physical Terrain Occlusion
    glUniform2f(ua_res, (float)taaWidth, (float)taaHeight);
    sendMat4(ua_invProj, proj.inverse());
    glActiveTexture(GL_TEXTURE7);
    glBindTexture(GL_TEXTURE_2D, taaDepthTex);
    glUniform1i(ua_depthTex, 7);

    {
        float camAlt = (camPos - planetCenter).length() - surfaceRadius;
        cloudSystem.bind(atmoProg, time, taaFrameIndex, camAlt);
    }

    // --- Graphics State for Volumetric Shell ---
    glDisable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_ONE, GL_ONE_MINUS_SRC_ALPHA);
    glDepthMask(GL_FALSE);
    
    // Front face culling guarantees we draw exactly ONE layer of the bounding sphere per pixel, 
    // eliminating double-drawing artifacts, and it never gets clipped by the near plane.
    // NOTE: Culling was already set correctly to GL_BACK at the top of the function.
    // We do NOT override it here.

    sphereMesh.draw();

    // Revert ALL state
    glCullFace(GL_BACK);
    glFrontFace(GL_CCW);
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
