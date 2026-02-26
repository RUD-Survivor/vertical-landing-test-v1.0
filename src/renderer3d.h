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
  GLuint billboardProg = 0;
  GLuint atmoProg = 0;
  GLuint billboardVAO = 0, billboardVBO = 0;
  GLint u_mvp = -1, u_model = -1, u_lightDir = -1, u_viewPos = -1;
  GLint u_baseColor = -1, u_ambientStr = -1;
  GLint ue_mvp = -1, ue_model = -1, ue_lightDir = -1, ue_viewPos = -1;
  // Billboard uniforms
  GLint ub_vp = -1, ub_proj = -1, ub_pos = -1, ub_size = -1, ub_color = -1;
  // Atmosphere uniforms
  GLint ua_mvp = -1, ua_model = -1, ua_lightDir = -1, ua_viewPos = -1;

  Mat4 view, proj;
  Vec3 camPos;
  Vec3 lightDir;

  // Ribbon Renderer properties
  GLuint ribbonProg, ribbonVAO, ribbonVBO;
  GLint ur_mvp, ur_color;

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

    // --- Earth Shader (procedural continents) ---
    const char* earthFragSrc = R"(
      #version 330 core
      in vec3 vWorldPos;
      in vec3 vNormal;
      in vec2 vUV;
      in vec4 vColor;
      in vec3 vLocalPos;

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
        vec3 texCoord = normalize(vLocalPos) * 3.0;
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
        float ambient = 0.20; // Increased ambient for dark scenes

        // Atmosphere rim glow
        float rim = 1.0 - max(dot(N, V), 0.0);
        rim = pow(rim, 3.0);
        vec3 atmosColor = vec3(0.3, 0.5, 1.0) * rim * 0.6;

        vec3 result = surfColor * (ambient + diff * 0.85) + atmosColor;

        // Night side city lights (subtle)
        if (diff < 0.05) {
          float city = noise3d(texCoord * 10.0);
          if (continent > 0.48 && city > 0.7) {
            result += vec3(1.0, 0.8, 0.3) * 0.8 * (1.0 - diff / 0.05); // Brighter city lights
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

    // --- Atmosphere Shader ---
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
        // Rim intensity (edge-on = bright)
        float rim = 1.0 - abs(dot(N, V));
        rim = pow(rim, 2.5);
        // Rayleigh scattering color
        vec3 scatter = vec3(0.3, 0.55, 1.0); // blue
        // Sunset at terminator
        float sunAngle = dot(N, L);
        float terminator = smoothstep(-0.1, 0.15, sunAngle);
        vec3 sunset = vec3(1.0, 0.4, 0.1);
        vec3 atmoColor = mix(sunset * 0.5, scatter, terminator);
        // Only visible on lit side + terminator
        float visibility = smoothstep(-0.3, 0.0, sunAngle) + 0.15;
        float alpha = rim * visibility * 0.7;
        FragColor = vec4(atmoColor, alpha);
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
      uniform mat4 uMVP;
      void main() {
        gl_Position = uMVP * vec4(aPos, 1.0);
      }
    )";
    const char* ribbonFragSrc = R"(
      #version 330 core
      uniform vec4 uColor;
      out vec4 FragColor;
      void main() {
        FragColor = uColor;
      }
    )";
    ribbonProg = compileProgram(ribbonVertSrc, ribbonFragSrc);
    ur_mvp = glGetUniformLocation(ribbonProg, "uMVP");
    ur_color = glGetUniformLocation(ribbonProg, "uColor");

    glGenVertexArrays(1, &ribbonVAO);
    glGenBuffers(1, &ribbonVBO);
    glBindVertexArray(ribbonVAO);
    glBindBuffer(GL_ARRAY_BUFFER, ribbonVBO);
    // Allocate an initial buffer size (e.g., 2000 points * 2 edges * sizeof(Vec3))
    glBufferData(GL_ARRAY_BUFFER, sizeof(Vec3) * 4000, nullptr, GL_DYNAMIC_DRAW);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vec3), (void*)0);
    glEnableVertexAttribArray(0);
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
      uniform int uShapeType; // 0 = soft circle, 1 = anamorphic streak, 2 = hexagon bokeh
      
      out vec4 FragColor;
      
      float hexDist(vec2 p) {
        p = abs(p);
        float c = dot(p, normalize(vec2(1.0, 1.73205081)));
        return max(c, p.x);
      }
      
      void main() {
        vec2 rUV = vUV * 2.0 - 1.0; // -1 to 1
        float alpha = 0.0;
        
        if (uShapeType == 0) {
           // Soft core / Glow
           float d = length(rUV);
           alpha = pow(max(0.0, 1.0 - d), 2.0);
        } else if (uShapeType == 1) {
           // Anamorphic horizontal streak
           float dx = abs(rUV.x);
           float dy = abs(rUV.y);
           // very sharp falloff vertically, gradual horizontally
           alpha = pow(max(0.0, 1.0 - dx), 3.0) * pow(max(0.0, 1.0 - dy*20.0), 3.0);
        } else if (uShapeType == 2) {
           // Hexagonal/Polygonal Bokeh Ring
           float d = hexDist(rUV);
           // Ring shape: sharp outside, softer inside
           float ring = smoothstep(1.0, 0.85, d) * smoothstep(0.6, 0.8, d);
           alpha = ring * 0.5 + smoothstep(1.0, 0.9, d) * 0.1; 
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
  void drawSunAndFlare(const Vec3& sunWorldPos, const Vec3& earthPos, float earthRadius, int screenW, int screenH) {
     // 1. Ray-Sphere Occlusion Test against Earth (is the sun behind the planet?)
     Vec3 rayDir = (sunWorldPos - camPos).normalized();
     Vec3 oc = camPos - earthPos;
     float b = 2.0f * oc.dot(rayDir);
     float c = oc.dot(oc) - earthRadius * earthRadius;
     float discriminant = b * b - 4 * c;
     float occlusionFade = 1.0f;
     float distToLimb = 1000000.0f;
     
     // Closest distance point along the ray
     float t_closest = -oc.dot(rayDir);
     
     if (t_closest < 0) {
         // The earth is strictly BEHIND the camera; no occlusion possible.
         occlusionFade = 1.0f;
     } else {
         if (discriminant > 0) {
             float t1 = t_closest - sqrtf(discriminant) / 2.0f;
             float sunDist = (sunWorldPos - camPos).length();
             if (t1 > 0 && t1 < sunDist) {
                 occlusionFade = 0.0f; // Eclipsed entirely by earth core
                 distToLimb = 0.0f;
             }
         } else {
             // Ray clears earth. How close did it get to the surface?
             float passDist = oc.cross(rayDir).length();
             distToLimb = passDist - earthRadius;
             // Fade smoothly through the thin upper atmosphere
             if (distToLimb >= 0.0f && distToLimb < earthRadius * 0.05f) {
                 occlusionFade *= distToLimb / (earthRadius * 0.05f);
             }
         }
     }

     if (occlusionFade <= 0.01f) return; // Completely hidden

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

     // Core Sun
     drawFlare(0, 0.25f, 0.25f, 1.0f,  1.0f, 0.9f, 0.8f, 2.0f); // Main Glow
     drawFlare(0, 0.60f, 0.60f, 1.0f,  0.5f, 0.4f, 0.6f, 0.5f); // Outer Halo

     // Anamorphic Streak (wide horizontal line)
     drawFlare(1, 3.0f, 0.05f, 1.0f,   0.3f, 0.6f, 1.0f, 1.2f); // Blue Streak
     drawFlare(1, 1.2f, 0.02f, 1.0f,   1.0f, 1.0f, 1.0f, 1.0f); // White Core Streak

     // Bokeh Ghosts (mirrored across center 0,0 by negative multipliers)
     // The further the sun is from center, the further the ghosts spread
     drawFlare(2, 0.15f, 0.15f, -0.3f, 0.2f, 0.8f, 0.4f, 0.3f); // Green hex
     drawFlare(2, 0.30f, 0.30f, -0.6f, 0.8f, 0.2f, 0.6f, 0.2f); // Pink hex
     drawFlare(2, 0.08f, 0.08f, -1.2f, 0.1f, 0.5f, 0.9f, 0.5f); // Blue hex
     
     // Some soft blob ghosts
     drawFlare(0, 0.4f, 0.4f,  0.5f,  0.5f, 0.2f, 0.2f, 0.15f); // Reddish blob
     drawFlare(0, 0.2f, 0.2f, -0.9f,  0.2f, 0.2f, 0.8f, 0.15f); // Blueish blob

     glBindVertexArray(0);

     glEnable(GL_DEPTH_TEST);
     glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
     glDepthMask(GL_TRUE);
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

  // Camera-facing dynamic ribbon (trajectory trail)
  void drawRibbon(const std::vector<Vec3>& points, float width, float r, float g, float b, float a) {
    if (points.size() < 2) return;

    std::vector<Vec3> stripVerts;
    stripVerts.reserve(points.size() * 2);

    for (size_t i = 0; i < points.size(); ++i) {
      Vec3 p = points[i];
      
      // forward vector along the path
      Vec3 forward;
      if (i < points.size() - 1) {
        forward = (points[i+1] - p);
      } else {
        forward = (p - points[i-1]);
      }
      if (forward.length() < 1e-6f) forward = Vec3(0.0f, 1.0f, 0.0f); else forward = forward.normalized();

      // Vector from point to camera
      Vec3 toCam = (camPos - p);
      if (toCam.length() < 1e-6f) toCam = Vec3(1.0f, 0.0f, 0.0f); else toCam = toCam.normalized();

      // Right vector (perpendicular to path and facing camera)
      Vec3 right = forward.cross(toCam);
      if (right.length() < 1e-6f) {
          // Fallback if aligned with camera
          right = forward.cross(Vec3(0.0f, 1.0f, 0.0f));
          if (right.length() < 1e-6f) right = forward.cross(Vec3(1.0f, 0.0f, 0.0f));
      }
      right = right.normalized();

      float halfW = width * 0.5f;
      // Zigzag for triangle strip: first left, then right
      stripVerts.push_back(p + right * halfW); 
      stripVerts.push_back(p - right * halfW);
    }

    glUseProgram(ribbonProg);
    Mat4 mvp = proj * view; // No model matrix needed, points are in world space
    glUniformMatrix4fv(ur_mvp, 1, GL_FALSE, mvp.m);
    glUniform4f(ur_color, r, g, b, a);

    // Buffer dynamic data
    glBindVertexArray(ribbonVAO);
    glBindBuffer(GL_ARRAY_BUFFER, ribbonVBO);
    if (stripVerts.size() * sizeof(Vec3) > sizeof(Vec3) * 1000 * 2) {
      // Reallocate if somehow we exceed initial buffer size
      glBufferData(GL_ARRAY_BUFFER, stripVerts.size() * sizeof(Vec3), stripVerts.data(), GL_DYNAMIC_DRAW);
    } else {
      glBufferSubData(GL_ARRAY_BUFFER, 0, stripVerts.size() * sizeof(Vec3), stripVerts.data());
    }

    // Force re-bind attribute pointer just in case another mesh corrupted our VAO state
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vec3), (void*)0);
    glEnableVertexAttribArray(0);

    // Glow blending
    glBlendFunc(GL_SRC_ALPHA, GL_ONE);
    glDepthMask(GL_FALSE); // Disable depth write so trails don't occlude themselves
    glDisable(GL_CULL_FACE); // Ensure triangle strips are never culled

    glDrawArrays(GL_TRIANGLE_STRIP, 0, stripVerts.size());

    // Restore
    glEnable(GL_CULL_FACE);
    glDepthMask(GL_TRUE);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glBindVertexArray(0);
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
