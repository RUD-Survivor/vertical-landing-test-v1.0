#pragma once
// ==========================================================
// renderer3d.h — Vulkan-only 3D data structures & minimal stub
// ==========================================================

#include "math/math3d.h"
#include "core/rocket_state.h"
#include <vector>
#include <string>
#include <map>
#include "terrain_system.h"
#include "vegetation_system.h"
#include "svo_system.h"
#include "svo_meshing.h"
#include "cloud_system.h"

// ==========================================================
// 3D 顶点格式 (Vertex3D)
// ========================================================== 
struct Vertex3D {
  float px, py, pz;     // 世界/局部空间位置
  float nx, ny, nz;     // 法线向量 (用于光照计算)
  float u, v;           // 纹理贴图坐标
  float r, g, b, a;     // 顶点颜色
};

// ==========================================================
// Mesh — 3D 网格模型（Vulkan：CPU 端几何数据容器）
// ==========================================================
struct Mesh {
  int indexCount = 0;
  std::vector<Vertex3D> cpuVerts;
  std::vector<uint32_t> cpuIndices;

  // Geometric properties for automatic scaling
  float minX = 0, minY = 0, minZ = 0;
  float maxX = 0, maxY = 0, maxZ = 0;
  float width = 1.0f, height = 1.0f, depth = 1.0f;
  float centerX = 0, centerY = 0, centerZ = 0;

  void upload(const std::vector<Vertex3D>& verts,
              const std::vector<unsigned int>& indices) {
    indexCount = (int)indices.size();
    cpuVerts = verts;
    cpuIndices.assign(indices.begin(), indices.end());
  }

  void draw() const { /* Vulkan: VkRenderer3D */ }
  void destroy() { cpuVerts.clear(); cpuIndices.clear(); }
};

// ==========================================================
// Renderer3D — 最小化桩（Vulkan-only）
// 保留字段供 flight_scene / flight_input_system 引用
// 实际渲染委托给 VkRenderer3D
// ==========================================================

// Forward declare MeshGen (defined later in this file)
namespace MeshGen { Mesh patch(int units); }

class Renderer3D {
public:
  CloudSystem cloudSystem;

  Mat4 view, proj;
  Vec3 camPos;
  Vec3 lightDir;

  Mesh sharedPatchMesh;
  Terrain::QuadtreeTerrain* terrain = nullptr;
  Vegetation::VegetationSystem* vegSystem = nullptr;

  SVO::SVOManager* svoManager = nullptr;

  // Cached meshes (for part_renderer.h API compatibility)
  std::map<std::string, Mesh> meshCache;

  int climateViewMode = 0;
  void setClimateViewMode(int mode) { climateViewMode = mode; }

  Renderer3D() {
    sharedPatchMesh = MeshGen::patch(32);
    terrain = new Terrain::QuadtreeTerrain(EARTH_RADIUS);
    if (terrain->sim && !terrain->sim->gridHeight.empty()) {
        cloudSystem.rebakeWeather(terrain->sim->gridHeight,
            terrain->sim->width, terrain->sim->height,
            Tectonic::TectonicSimulator::CONT_THRESHOLD);
    }
    vegSystem = new Vegetation::VegetationSystem();
    svoManager = new SVO::SVOManager();
  }

  // No-op stubs (Vulkan rendering handled by VkRenderer3D)
  void initTAA(int, int) {}
  void beginTAAPass() {}
  void resolveTAA() {}
  void beginFrame(const Mat4& v, const Mat4& p, const Vec3& c) { view=v; proj=p; camPos=c; }
  void endFrame() {}
  void drawMesh(const Mesh&, const Mat4&, float=1,float=1,float=1,float=1,float=0.15f) {}
  void drawSunAndFlare(const Vec3&, const std::vector<Vec4>&, int, int) {}
  void drawTerrainPatch(const Mesh&, const Mat4&, float, float) {}
  void drawVegetation(const std::vector<Vegetation::InstanceData>&) {}
  void drawPlanet(const Mesh&, const Mat4&, BodyType, float,float,float,float,float,float=0,int=-1) {}
  void drawRing(const Mesh&, const Mat4&, float,float,float,float) {}
  void drawBillboard(const Vec3&, float, float,float,float,float) {}
  void drawExhaustVolumetric(const Mesh&, const Mat4&, float,float,float,float,float) {}
  void drawAtmosphere(const Mesh&, const Mat4&, const Vec3&, const Vec3&, const Vec3&, float,float,double,int,float,bool) {}
  void drawSkybox(float=1, float=1) {}
  void drawRibbon(const std::vector<Vec3>&, const std::vector<Vec4>&, float) {}
  void drawRibbon(const std::vector<Vec3>&, float, float,float,float,float) {}
};


// ==========================================================
// MeshGen — 几何体生成器
// ==========================================================
namespace MeshGen {

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
      float x = cosP * sinT, y = cosT, z = sinP * sinT;
      Vertex3D v;
      v.px = x * radius; v.py = y * radius; v.pz = z * radius;
      v.nx = x; v.ny = y; v.nz = z;
      v.u = (float)lon / lonSegs; v.v = (float)lat / latSegs;
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
  Mesh m; m.upload(verts, indices); return m;
}

inline Mesh cylinder(int segs, float radius, float height) {
  std::vector<Vertex3D> verts;
  std::vector<unsigned int> indices;
  const float PI_f = 3.14159265358979f;
  float halfH = height * 0.5f;
  for (int i = 0; i <= segs; i++) {
    float ang = (float)i / segs * 2.0f * PI_f;
    float cs = cosf(ang), sn = sinf(ang);
    float u = (float)i / segs;
    Vertex3D vt; vt.px = cs * radius; vt.py = halfH; vt.pz = sn * radius;
    vt.nx = cs; vt.ny = 0; vt.nz = sn; vt.u = u; vt.v = 0;
    vt.r = 1; vt.g = 1; vt.b = 1; vt.a = 1; verts.push_back(vt);
    Vertex3D vb; vb.px = cs * radius; vb.py = -halfH; vb.pz = sn * radius;
    vb.nx = cs; vb.ny = 0; vb.nz = sn; vb.u = u; vb.v = 1;
    vb.r = 1; vb.g = 1; vb.b = 1; vb.a = 1; verts.push_back(vb);
  }
  for (int i = 0; i < segs; i++) {
    int a = i * 2, b = a + 1, c = a + 2, d = a + 3;
    indices.push_back(a); indices.push_back(b); indices.push_back(c);
    indices.push_back(b); indices.push_back(d); indices.push_back(c);
  }
  Mesh m; m.upload(verts, indices); return m;
}

inline Mesh cone(int segs, float radius, float height) {
  std::vector<Vertex3D> verts;
  std::vector<unsigned int> indices;
  const float PI_f = 3.14159265358979f;
  float slope = radius / sqrtf(radius * radius + height * height);
  float ny_cone = height / sqrtf(radius * radius + height * height);
  Vertex3D tip; tip.px = 0; tip.py = height; tip.pz = 0;
  tip.nx = 0; tip.ny = ny_cone; tip.nz = 0; tip.u = 0.5f; tip.v = 0;
  tip.r = 1; tip.g = 1; tip.b = 1; tip.a = 1; verts.push_back(tip);
  for (int i = 0; i <= segs; i++) {
    float ang = (float)i / segs * 2.0f * PI_f;
    float cs = cosf(ang), sn = sinf(ang);
    Vertex3D v; v.px = cs * radius; v.py = 0; v.pz = sn * radius;
    v.nx = cs * ny_cone; v.ny = slope; v.nz = sn * ny_cone;
    v.u = (float)i / segs; v.v = 1;
    v.r = 1; v.g = 1; v.b = 1; v.a = 1; verts.push_back(v);
  }
  for (int i = 0; i < segs; i++) {
    indices.push_back(0); indices.push_back(1 + i); indices.push_back(2 + i);
  }
  Mesh m; m.upload(verts, indices); return m;
}

inline Mesh box(float width, float height, float depth) {
  std::vector<Vertex3D> verts;
  std::vector<unsigned int> indices;
  float dx = width * 0.5f, dy = height * 0.5f, dz = depth * 0.5f;
  Vec3 p[8] = {
    Vec3(-dx, -dy,  dz), Vec3( dx, -dy,  dz), Vec3( dx,  dy,  dz), Vec3(-dx,  dy,  dz),
    Vec3(-dx, -dy, -dz), Vec3( dx, -dy, -dz), Vec3( dx,  dy, -dz), Vec3(-dx,  dy, -dz)
  };
  Vec3 n[6] = {
    Vec3( 0,  0,  1), Vec3( 0,  0, -1), Vec3(-1,  0,  0),
    Vec3( 1,  0,  0), Vec3( 0,  1,  0), Vec3( 0, -1,  0)
  };
  int f[6][4] = {
    {0,1,2,3},{5,4,7,6},{4,0,3,7},{1,5,6,2},{3,2,6,7},{4,5,1,0}
  };
  for (int i = 0; i < 6; i++) {
    for (int j = 0; j < 4; j++) {
      Vertex3D v; Vec3 pos = p[f[i][j]];
      v.px = pos.x; v.py = pos.y; v.pz = pos.z;
      v.nx = n[i].x; v.ny = n[i].y; v.nz = n[i].z;
      v.u = (j == 1 || j == 2) ? 1.0f : 0.0f;
      v.v = (j == 2 || j == 3) ? 1.0f : 0.0f;
      v.r = 1; v.g = 1; v.b = 1; v.a = 1; verts.push_back(v);
    }
    int base = i * 4;
    indices.push_back(base+0); indices.push_back(base+1); indices.push_back(base+2);
    indices.push_back(base+0); indices.push_back(base+2); indices.push_back(base+3);
  }
  Mesh m; m.upload(verts, indices); return m;
}

inline Mesh ring(int segs, float innerRadius, float outerRadius) {
  std::vector<Vertex3D> verts;
  std::vector<unsigned int> indices;
  const float PI_f = 3.14159265358979f;
  for (int i = 0; i <= segs; i++) {
    float ang = (float)i / segs * 2.0f * PI_f;
    float cs = cosf(ang), sn = sinf(ang);
    Vertex3D vi; vi.px = cs * innerRadius; vi.py = 0; vi.pz = sn * innerRadius;
    vi.nx = 0; vi.ny = 1; vi.nz = 0; vi.u = (float)i / segs; vi.v = 0;
    vi.r = 1; vi.g = 1; vi.b = 1; vi.a = 1; verts.push_back(vi);
    Vertex3D vo; vo.px = cs * outerRadius; vo.py = 0; vo.pz = sn * outerRadius;
    vo.nx = 0; vo.ny = 1; vo.nz = 0; vo.u = (float)i / segs; vo.v = 1;
    vo.r = 1; vo.g = 1; vo.b = 1; vo.a = 1; verts.push_back(vo);
  }
  for (int i = 0; i < segs; i++) {
    int v0 = i*2, v1 = v0+1, v2 = v0+2, v3 = v0+3;
    indices.push_back(v0); indices.push_back(v2); indices.push_back(v1);
    indices.push_back(v1); indices.push_back(v2); indices.push_back(v3);
  }
  Mesh m; m.upload(verts, indices); return m;
}

inline Mesh patch(int units) {
  std::vector<Vertex3D> verts;
  std::vector<unsigned int> indices;
  float step = 1.0f / (float)units;
  for (int z = 0; z <= units; z++) {
    for (int x = 0; x <= units; x++) {
      Vertex3D v;
      v.px = (float)x * step - 0.5f; v.py = 0; v.pz = (float)z * step - 0.5f;
      v.nx = 0; v.ny = 1; v.nz = 0; v.u = (float)x * step; v.v = (float)z * step;
      v.r = 1; v.g = 1; v.b = 1; v.a = 1; verts.push_back(v);
    }
  }
  for (int z = 0; z < units; z++) {
    for (int x = 0; x < units; x++) {
      int i0 = z * (units + 1) + x, i1 = i0 + 1, i2 = i0 + (units + 1), i3 = i2 + 1;
      indices.push_back(i0); indices.push_back(i2); indices.push_back(i1);
      indices.push_back(i1); indices.push_back(i2); indices.push_back(i3);
    }
  }
  Mesh m; m.upload(verts, indices); return m;
}

} // namespace MeshGen
