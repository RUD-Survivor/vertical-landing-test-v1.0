#pragma once
// ==========================================================================
// vk_scene.h — 不透明几何渲染子系统
// 拥有: Mesh / SVO / Planet×11 / Ring / Atmosphere / Vegetation 管线
// 调用顺序: beginFrame() → draw* × N
// ==========================================================================

#include "../vk_context.h"
#include "../vk_mesh.h"
#include "../vk_descriptors.h"
#include "../vk_pipeline.h"
#include "../vk_texture.h"
#include "../vk_cloud_bake.h"
#include "../weather_map_bake.h"
#include "../vk_atmosphere_lut.h" // 大气重构：VkAtmoLutCache + fillAtmoLutFrameData
#include "../vk_cloud_shadow.h"  // GpuCloudShadowData：地形 Set0 binding 3 的 UBO 布局
#include "../../math/math3d.h"   // Vec3, Mat4

#include <unordered_map>
#include <string>
#include <vector>
#include <cstring>
#include <cstdio>
#include <cmath>
static constexpr float kPI_f = 3.14159265358979323846f;

struct VkSceneSystem {
    VkMeshPipeline       meshPipe;
    VkSVOPipeline        svoPipe;
    VkRingPipeline       ringPipe;
    VkAtmoPipeline       atmoPipe; // 旧版全屏 raymarch，保留未删（不再由 render() 调用，见 vk_renderer3d.h）
    VkAtmoInsidePipeline atmoInsidePipe; // 大气重构·壳内路径（atmo_inside.frag）
    VkAtmoShellPipeline  atmoShellPipe;  // 大气重构·壳外路径（atmo_shell.vert/frag，画 atmoSphereVbo）
    VkAtmoLutCache        atmoLutCache;  // 按 bodyIdx 缓存每颗星球的 Transmittance/SkyView LUT
    VkVegetationPipeline vegPipe;
    VkTerrainPipeline    terrainPipe;

    std::unordered_map<std::string, VkPlanetPipeline> planetPipes;

    VkDescriptorManager* desc       = nullptr;
    VkTexture2D          nullTex;
    VkDescriptorSet      nullTexSet = VK_NULL_HANDLE;

    std::unordered_map<std::string, VkMesh> meshCache;

    // 大气球体（pos-only，unit sphere，着色器内按 outerRadius 缩放）
    VkBuffer      atmoSphereVbo   = VK_NULL_HANDLE;
    VmaAllocation atmoSphereAlloc = VK_NULL_HANDLE;
    uint32_t      atmoSphereVerts = 0;

    // 排气羽流立方体（pos-only，unit cube，exhaust 着色器体积光线步进）
    VkBuffer      exhaustCubeVbo      = VK_NULL_HANDLE;
    VmaAllocation exhaustCubeAlloc    = VK_NULL_HANDLE;
    VkBuffer      exhaustCubeIbo      = VK_NULL_HANDLE;
    VmaAllocation exhaustCubeIboAlloc = VK_NULL_HANDLE;
    uint32_t      exhaustCubeIcount   = 0;

    // Billboard quad VBO（vec2 × 4，镜头光晕 + 粒子）
    VkBuffer      billboardQuadVbo    = VK_NULL_HANDLE;
    VmaAllocation billboardQuadAlloc  = VK_NULL_HANDLE;

    // Terrain patch mesh（共享 32×32 patch，Vertex3D 格式）
    VkBuffer      terrainPatchVbo     = VK_NULL_HANDLE;
    VmaAllocation terrainPatchAlloc   = VK_NULL_HANDLE;
    VkBuffer      terrainPatchIbo     = VK_NULL_HANDLE;
    VmaAllocation terrainPatchIboAlloc= VK_NULL_HANDLE;
    uint32_t      terrainPatchIcount  = 0;

    // Terrain 专属 UBO + descriptor set（Set0 binding 0=FrameUBO, binding 1=TerrainData）
    VkBuffer      terrainDataUbo[2]   = {VK_NULL_HANDLE, VK_NULL_HANDLE};
    VmaAllocation terrainDataAlloc[2] = {VK_NULL_HANDLE, VK_NULL_HANDLE};
    void*         terrainDataMapped[2]= {nullptr, nullptr};
    VkDescriptorSet terrainSet0[2]    = {VK_NULL_HANDLE, VK_NULL_HANDLE};

    // 云影级联 UBO 的占位：云系统还没起来（或不在有云的星球上）时绑它。内容恒为全零，
    // 即 CloudShadowData.config.x = 0，采样函数会直接返回 1，等价于「无阴影」。
    static constexpr VkDeviceSize kTerrainCloudShadowFallbackSize = sizeof(GpuCloudShadowData);
    VkBuffer      terrainCloudShadowFallbackUbo   = VK_NULL_HANDLE;
    VmaAllocation terrainCloudShadowFallbackAlloc = VK_NULL_HANDLE;

    // Terrain 全局纹理（Set1: tectonic/hydro/climate/null）
    VkTexture2D     terrainTectonicTex;
    VkTexture2D     terrainHydroTex;
    VkTexture2D     terrainNullTex2;
    VkDescriptorSet terrainGlobalTexSet = VK_NULL_HANDLE;

    // Cloud textures for Flower compute path (basic/detail/weather/curl)
    VkTexture3D     cloudNoiseTex;
    VkTexture3D     cloudDetailTex;
    VkTexture2D     cloudWeatherTex;
    // cloud_common.glsl::cloudMap() 湍流位移矢量场（binding 9，64³ RGBA8 真 curl noise）
    VkTexture3D     cloudCurlTex;
    // Curl 纹理 ImGui 预览：3D 纹理的 z=0 切片 2D view（ImGui 只能显示 2D）
    VkImageView     curlVisView = VK_NULL_HANDLE;
    // ── 双层云：Profile LUT（binding 31）+ 高空云 Hi weather map（binding 32）──
    VkTexture2D     cloudProfileLutTex;
    VkTexture2D     cloudHiWeatherTex;
    // Weather map ImGui 预览：同 image 上的通道 swizzle view（R/G/B/A → 灰度 RGB）
    VkImageView     weatherVisViewR = VK_NULL_HANDLE;
    VkImageView     weatherVisViewG = VK_NULL_HANDLE;
    VkImageView     weatherVisViewB = VK_NULL_HANDLE;
    VkImageView     weatherVisViewA = VK_NULL_HANDLE;
    VkCloudBake     cloudBake;

    // Vegetation 实例 VBO + 环形缓冲
    VkBuffer      vegInstanceVbo      = VK_NULL_HANDLE;
    VmaAllocation vegInstanceAlloc    = VK_NULL_HANDLE;
    void*         vegInstanceMapped   = nullptr;

    // Vegetation 顶点 mesh（树几何）
    VkBuffer      vegVertVbo          = VK_NULL_HANDLE;
    VmaAllocation vegVertAlloc        = VK_NULL_HANDLE;
    uint32_t      vegVertCount        = 0;

    int _frameIdx = 0;

    VkDevice _device = VK_NULL_HANDLE; // 供 updateTerrainCloudShadow 等拿不到 ctx 的方法使用

    // -----------------------------------------------------------------------
    bool init(VulkanContext& ctx, VkDescriptorManager& d,
              VkFormat colorFmt, VkFormat depthFmt) {
        desc = &d;
        _device = ctx.device;

        if (!meshPipe.init(ctx, d, colorFmt, depthFmt,
                "src/render/shaders/spirv/mesh.vert.spv",
                "src/render/shaders/spirv/mesh.frag.spv")) {
            fprintf(stderr, "[VkScene] Failed: mesh pipeline\n"); return false;
        }
        if (!svoPipe.init(ctx, d, colorFmt, depthFmt,
                "src/render/shaders/spirv/mesh.vert.spv",
                "src/render/shaders/spirv/svo.frag.spv")) {
            fprintf(stderr, "[VkScene] Failed: svo pipeline\n"); return false;
        }
        if (!ringPipe.init(ctx, d, colorFmt, depthFmt,
                "src/render/shaders/spirv/mesh.vert.spv",
                "src/render/shaders/spirv/ring.frag.spv")) {
            fprintf(stderr, "[VkScene] Failed: ring pipeline\n"); return false;
        }
        if (!atmoPipe.init(ctx, d, colorFmt, depthFmt,
                "src/render/shaders/spirv/atmo.vert.spv",
                "src/render/shaders/spirv/atmo.frag.spv")) {
            fprintf(stderr, "[VkScene] Failed: atmo pipeline\n"); return false;
        }
        // 大气重构：按 bodyIdx 缓存的 per-planet LUT + 壳内/壳外两条新绘制路径。
        // atmoLutCache 要先于 atmoInsidePipe/atmoShellPipe 初始化——后两者的 Set 1
        // 需要引用 atmoLutCache.pipelines.renderSetLayout。
        if (!atmoLutCache.init(ctx)) {
            fprintf(stderr, "[VkScene] Failed: atmo LUT cache\n"); return false;
        }
        if (!atmoInsidePipe.init(ctx, d, atmoLutCache.pipelines.renderSetLayout, colorFmt, depthFmt,
                "src/render/shaders/spirv/atmo.vert.spv",
                "src/render/shaders/spirv/atmo_inside.frag.spv")) {
            fprintf(stderr, "[VkScene] Failed: atmo inside pipeline\n"); return false;
        }
        if (!atmoShellPipe.init(ctx, d, atmoLutCache.pipelines.renderSetLayout, colorFmt, depthFmt,
                "src/render/shaders/spirv/atmo_shell.vert.spv",
                "src/render/shaders/spirv/atmo_shell.frag.spv")) {
            fprintf(stderr, "[VkScene] Failed: atmo shell pipeline\n"); return false;
        }
        if (!vegPipe.init(ctx, d, colorFmt, depthFmt,
                "src/render/shaders/spirv/vegetation.vert.spv",
                "src/render/shaders/spirv/vegetation.frag.spv")) {
            fprintf(stderr, "[VkScene] Failed: vegetation pipeline\n"); return false;
        }
        if (!terrainPipe.init(ctx, colorFmt, depthFmt,
                "src/render/shaders/spirv/terrain.vert.spv",
                "src/render/shaders/spirv/terrain.frag.spv")) {
            fprintf(stderr, "[VkScene] Failed: terrain pipeline\n"); return false;
        }

        // 预注册所有 11 个星球着色器，避免运行时按需创建管线的停顿
        static const struct { const char* key; const char* frag; } kPlanets[] = {
            { "earth",     "src/render/shaders/spirv/earth.frag.spv"     },
            { "gas_giant", "src/render/shaders/spirv/gas_giant.frag.spv" },
            { "barren",    "src/render/shaders/spirv/barren.frag.spv"    },
            { "mercury",   "src/render/shaders/spirv/mercury.frag.spv"   },
            { "moon",      "src/render/shaders/spirv/moon.frag.spv"      },
            { "venus",     "src/render/shaders/spirv/venus.frag.spv"     },
            { "mars",      "src/render/shaders/spirv/mars.frag.spv"      },
            { "jupiter",   "src/render/shaders/spirv/jupiter.frag.spv"   },
            { "saturn",    "src/render/shaders/spirv/saturn.frag.spv"    },
            { "uranus",    "src/render/shaders/spirv/uranus.frag.spv"    },
            { "neptune",   "src/render/shaders/spirv/neptune.frag.spv"   },
        };
        for (const auto& p : kPlanets) {
            VkPlanetPipeline pp;
            if (!pp.init(ctx, d, colorFmt, depthFmt,
                    "src/render/shaders/spirv/mesh.vert.spv", p.frag)) {
                fprintf(stderr, "[VkScene] Failed: planet pipeline '%s'\n", p.key);
                return false;
            }
            planetPipes.emplace(p.key, std::move(pp));
        }

        static const uint8_t white[4] = {255, 255, 255, 255};
        if (!nullTex.upload(ctx, white, 1, 1)) {
            fprintf(stderr, "[VkScene] Failed: null texture\n"); return false;
        }
        nullTexSet = d.allocateTextureSet(ctx.device, nullTex.view, nullTex.sampler);
        if (nullTexSet == VK_NULL_HANDLE) {
            fprintf(stderr, "[VkScene] Failed: null texture set\n"); return false;
        }

        if (!_buildAtmoSphere(ctx)) {
            fprintf(stderr, "[VkScene] Failed: atmosphere sphere VBO\n"); return false;
        }
        if (!_buildExhaustCube(ctx)) {
            fprintf(stderr, "[VkScene] Failed: exhaust cube VBO\n"); return false;
        }
        if (!_buildBillboardQuad(ctx)) {
            fprintf(stderr, "[VkScene] Failed: billboard quad\n"); return false;
        }
        if (!_buildTerrainPatch(ctx)) {
            fprintf(stderr, "[VkScene] Failed: terrain patch\n"); return false;
        }
        if (!_initTerrainDescriptors(ctx)) {
            fprintf(stderr, "[VkScene] Failed: terrain descriptors\n"); return false;
        }
        if (!_buildVegetationMesh(ctx)) {
            fprintf(stderr, "[VkScene] Failed: vegetation mesh\n"); return false;
        }

        printf("[VkScene] Initialized: %zu planet pipelines, atmo sphere %u verts\n",
               planetPipes.size(), atmoSphereVerts);
        return true;
    }

    void shutdown(VulkanContext& ctx) {
        if (exhaustCubeIbo != VK_NULL_HANDLE) {
            vmaDestroyBuffer(ctx.allocator, exhaustCubeIbo, exhaustCubeIboAlloc);
            exhaustCubeIbo = VK_NULL_HANDLE;
        }
        if (exhaustCubeVbo != VK_NULL_HANDLE) {
            vmaDestroyBuffer(ctx.allocator, exhaustCubeVbo, exhaustCubeAlloc);
            exhaustCubeVbo = VK_NULL_HANDLE;
        }
        if (atmoSphereVbo != VK_NULL_HANDLE) {
            vmaDestroyBuffer(ctx.allocator, atmoSphereVbo, atmoSphereAlloc);
            atmoSphereVbo = VK_NULL_HANDLE;
        }
        if (terrainPatchIbo != VK_NULL_HANDLE) {
            vmaDestroyBuffer(ctx.allocator, terrainPatchIbo, terrainPatchIboAlloc);
            terrainPatchIbo = VK_NULL_HANDLE;
        }
        if (terrainPatchVbo != VK_NULL_HANDLE) {
            vmaDestroyBuffer(ctx.allocator, terrainPatchVbo, terrainPatchAlloc);
            terrainPatchVbo = VK_NULL_HANDLE;
        }
        if (billboardQuadVbo != VK_NULL_HANDLE) {
            vmaDestroyBuffer(ctx.allocator, billboardQuadVbo, billboardQuadAlloc);
            billboardQuadVbo = VK_NULL_HANDLE;
        }
        for (int i = 0; i < 2; i++) {
            if (terrainDataUbo[i] != VK_NULL_HANDLE) {
                vmaDestroyBuffer(ctx.allocator, terrainDataUbo[i], terrainDataAlloc[i]);
                terrainDataUbo[i] = VK_NULL_HANDLE;
            }
        }
        if (terrainCloudShadowFallbackUbo != VK_NULL_HANDLE) {
            vmaDestroyBuffer(ctx.allocator, terrainCloudShadowFallbackUbo, terrainCloudShadowFallbackAlloc);
            terrainCloudShadowFallbackUbo = VK_NULL_HANDLE;
        }
        if (vegInstanceVbo != VK_NULL_HANDLE) {
            vmaDestroyBuffer(ctx.allocator, vegInstanceVbo, vegInstanceAlloc);
            vegInstanceVbo = VK_NULL_HANDLE;
        }
        if (vegVertVbo != VK_NULL_HANDLE) {
            vmaDestroyBuffer(ctx.allocator, vegVertVbo, vegVertAlloc);
            vegVertVbo = VK_NULL_HANDLE;
        }
        for (auto& e : meshCache)   e.second.destroy(ctx);
        meshCache.clear();
        nullTex.destroy(ctx);
        terrainTectonicTex.destroy(ctx);
        terrainHydroTex.destroy(ctx);
        terrainNullTex2.destroy(ctx);
        for (auto& e : planetPipes) e.second.shutdown(ctx.device);
        planetPipes.clear();
        vegPipe.shutdown(ctx.device);
        terrainPipe.shutdown(ctx.device);
        atmoPipe.shutdown(ctx.device);
        atmoInsidePipe.shutdown(ctx.device);
        atmoShellPipe.shutdown(ctx.device);
        atmoLutCache.shutdown(ctx);
        ringPipe.shutdown(ctx.device);
        svoPipe.shutdown(ctx.device);
        meshPipe.shutdown(ctx.device);

        // Cloud textures
        cloudBake.shutdown(ctx.device);
        auto destroyVis = [&](VkImageView& v) {
            if (v != VK_NULL_HANDLE) { vkDestroyImageView(ctx.device, v, nullptr); v = VK_NULL_HANDLE; }
        };
        destroyVis(weatherVisViewR);
        destroyVis(weatherVisViewG);
        destroyVis(weatherVisViewB);
        destroyVis(weatherVisViewA);
        destroyVis(curlVisView);
        cloudNoiseTex.destroy(ctx);
        cloudDetailTex.destroy(ctx);
        cloudWeatherTex.destroy(ctx);
        cloudCurlTex.destroy(ctx);
        cloudProfileLutTex.destroy(ctx);
        cloudHiWeatherTex.destroy(ctx);
    }

    // -----------------------------------------------------------------------
    bool registerMesh(VulkanContext& ctx, const std::string& id,
                      const void* vdata, size_t vsize,
                      const uint32_t* idata, uint32_t icount) {
        auto it = meshCache.find(id);
        if (it != meshCache.end()) it->second.destroy(ctx);
        VkMesh& m = meshCache[id];
        if (!m.upload(ctx, vdata, (VkDeviceSize)vsize, idata, icount)) {
            fprintf(stderr, "[VkScene] Failed to upload mesh: %s\n", id.c_str());
            meshCache.erase(id);
            return false;
        }
        return true;
    }

    bool hasMesh(const std::string& id) const { return meshCache.count(id) > 0; }

    // -----------------------------------------------------------------------
    // 帧开始：更新 FrameUBO，绑定 mesh 管线为默认状态
    void beginFrame(VkCommandBuffer cmd, int frameIdx,
                    const float view[16], const float proj[16],
                    const float lightDir[3], const float viewPos[3], float time) {
        _frameIdx = frameIdx;
        FrameUBO ubo{};
        memcpy(ubo.view,     view,     64);
        memcpy(ubo.proj,     proj,     64);
        memcpy(ubo.lightDir, lightDir, 12);
        memcpy(ubo.viewPos,  viewPos,  12);
        ubo.time = time;
        desc->updateFrameUBO(frameIdx, ubo);
        _bindMeshPipe(cmd);
    }

    // 仅更新 UBO 中的光照方向（逐行星切换，匹配 OpenGL per-planet lightDir）
    void updateLightDir(int frameIdx, float lx, float ly, float lz) {
        float ld = sqrtf(lx*lx + ly*ly + lz*lz);
        if (ld > 1e-6f) { lx /= ld; ly /= ld; lz /= ld; }
        float* ptr = (float*)desc->uboMapped[frameIdx];
        memcpy(ptr + 32, &lx, 4);  // lightDir.x at float offset 32 (byte offset 128)
        memcpy(ptr + 33, &ly, 4);
        memcpy(ptr + 34, &lz, 4);
    }

    // -----------------------------------------------------------------------
    void drawMesh(VkCommandBuffer cmd, const std::string& id, const float model[16],
                  float r=1.f, float g=1.f, float b=1.f, float a=1.f, float ambient=0.15f) {
        auto it = meshCache.find(id);
        if (it == meshCache.end()) return;
        _bindMeshPipe(cmd);  // 确保管线+描述符集已绑定
        MeshPushConstants pc{};
        memcpy(pc.model, model, 64);
        pc.baseColor[0]=r; pc.baseColor[1]=g; pc.baseColor[2]=b; pc.baseColor[3]=a;
        pc.ambientStr = ambient; pc.hasTexture = 0;
        vkCmdPushConstants(cmd, meshPipe.layout,
            VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT,
            0, sizeof(MeshPushConstants), &pc);
        it->second.bind(cmd);
        it->second.draw(cmd);
    }

    // -----------------------------------------------------------------------
    // shaderKey: "earth"/"mars"/"venus"/"jupiter"/"gas_giant"/"barren"/"mercury"/
    //            "moon"/"saturn"/"uranus"/"neptune"，默认 "earth"
    // lightDir*: 已归一化「行星→太阳」；写入 push constant，勿依赖 FrameUBO 逐 draw 改写
    void drawPlanet(VkCommandBuffer cmd, const std::string& meshId, const float model[16],
                    float cx, float cy, float cz,
                    float lx, float ly, float lz,
                    float r=1.f, float g=1.f, float b=1.f,
                    const std::string& shaderKey="earth") {
        auto pit = planetPipes.find(shaderKey);
        if (pit == planetPipes.end()) { drawMesh(cmd, meshId, model, r, g, b); return; }
        auto mit = meshCache.find(meshId);
        if (mit == meshCache.end()) return;

        // planet/mesh 使用相同 set0Layout+set1Layout，管线布局兼容，描述符集保持有效
        pit->second.bind(cmd);
        PlanetPushConstants pc{};
        memcpy(pc.model, model, 64);
        pc.baseColor[0]=r; pc.baseColor[1]=g; pc.baseColor[2]=b; pc.baseColor[3]=1.f;
        pc.planetCenter[0]=cx; pc.planetCenter[1]=cy; pc.planetCenter[2]=cz; pc.planetCenter[3]=0.f;
        pc.ambientStr = 0.008f; pc.hasTexture = 0;
        pc.lightDir[0]=lx; pc.lightDir[1]=ly; pc.lightDir[2]=lz; pc._padLight = 0.f;
        vkCmdPushConstants(cmd, pit->second.layout,
            VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT,
            0, sizeof(PlanetPushConstants), &pc);
        mit->second.bind(cmd);
        mit->second.draw(cmd);
        _bindMeshPipe(cmd);
    }

    // -----------------------------------------------------------------------
    void drawRing(VkCommandBuffer cmd, const std::string& meshId, const float model[16],
                  float cx, float cy, float cz, float planetRadius,
                  float lx, float ly, float lz) {
        if (ringPipe.pipeline == VK_NULL_HANDLE) return;
        auto it = meshCache.find(meshId);
        if (it == meshCache.end()) return;

        ringPipe.bind(cmd);
        PlanetPushConstants pc{};
        memcpy(pc.model, model, 64);
        pc.baseColor[0]=pc.baseColor[1]=pc.baseColor[2]=pc.baseColor[3]=1.f;
        pc.planetCenter[0]=cx; pc.planetCenter[1]=cy; pc.planetCenter[2]=cz; pc.planetCenter[3]=0.f;
        pc.ambientStr = planetRadius;
        pc.hasTexture = 0;
        pc.lightDir[0]=lx; pc.lightDir[1]=ly; pc.lightDir[2]=lz; pc._padLight = 0.f;
        vkCmdPushConstants(cmd, ringPipe.layout,
            VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT,
            0, sizeof(PlanetPushConstants), &pc);
        it->second.bind(cmd);
        it->second.draw(cmd);
        _bindMeshPipe(cmd);
    }

    // -----------------------------------------------------------------------
    // SVO — 与 mesh 相同顶点格式（mesh.vert），走 svo.frag（per-vertex 颜色 + 三平面噪声）
    void drawSVO(VkCommandBuffer cmd, const std::string& meshId, const float model[16]) {
        auto it = meshCache.find(meshId);
        if (it == meshCache.end()) return;

        svoPipe.bind(cmd);
        MeshPushConstants pc{};
        memcpy(pc.model, model, 64);
        pc.baseColor[0]=pc.baseColor[1]=pc.baseColor[2]=pc.baseColor[3]=1.f;
        pc.ambientStr = 0.12f; pc.hasTexture = 0;
        vkCmdPushConstants(cmd, svoPipe.layout,
            VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT,
            0, sizeof(MeshPushConstants), &pc);
        it->second.bind(cmd);
        it->second.draw(cmd);
        _bindMeshPipe(cmd);
    }

    // -----------------------------------------------------------------------
    // Atmosphere — fullscreen triangle, ray direction reconstructed in shader.
    // No vertex buffer needed; pipeline uses gl_VertexIndex to generate 3 verts.
    void drawAtmosphere(VkCommandBuffer cmd, const AtmoPushConstants& pc) {
        if (atmoPipe.pipeline == VK_NULL_HANDLE) return;

        atmoPipe.bind(cmd);
        vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS,
            atmoPipe.layout, 0, 1, &desc->set0[_frameIdx], 0, nullptr);
        vkCmdPushConstants(cmd, atmoPipe.layout,
            VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT,
            0, sizeof(AtmoPushConstants), &pc);
        vkCmdDraw(cmd, 3, 1, 0, 0);  // fullscreen triangle — no VBO
        _bindMeshPipe(cmd);
    }

    // -----------------------------------------------------------------------
    // 大气重构·壳内路径：相机在这颗星球的大气层以内时调用（全场景最多同时一颗
    // 星球满足，见 vk_renderer3d.h 的 cameraInside 分支路由）。内部依次：
    //   ① 从 atmoLutCache 取得（必要时创建 + profile 变化则重烤）这颗星球的
    //      LUT 实例；② 每帧刷新它的 Sky-View LUT（bakeSkyViewOnly，便宜，相机
    //      高度/太阳角每帧都可能变）；③ 刷新深度绑定（TAA ping-pong 每帧不同）；
    //      ④ 画全屏三角形（复用 atmo.vert）。
    // bodyIdx：稳定的星球场景索引（CelestialDraw::bodyIdx），LUT 缓存的 key。
    // camPosKm/lightDirToSun：调用方已经算好的相机世界坐标 / 该星球"指向太阳"
    // 方向（vk_renderer3d.h 循环里已经在为每颗星球重算 light 方向，直接传进来）。
    void drawAtmoInside(VulkanContext& ctx, VkCommandBuffer cmd, int frameSlot,
                        VkImageView depthView, VkImageView sceneColorView,
                        int32_t bodyIdx, const AtmoPushConstants& pc,
                        const float camPosKm[3], const float lightDirToSun[3],
                        const AtmoTuneParams& tune) {
        if (atmoInsidePipe.pipeline == VK_NULL_HANDLE) return;

        const float thicknessKm = pc.outerRadius - pc.surfaceRadius;
        GpuPerFrameData fd = fillAtmoLutFrameData(camPosKm, pc.planetCenter, pc.surfaceRadius,
                                                   thicknessKm, pc.planetIdx, lightDirToSun, &tune);
        AtmoLutProfileKey key = AtmoLutProfileKey::fromTune(pc.planetIdx, pc.surfaceRadius, thicknessKm, tune);
        VkAtmosphereLut* lut = atmoLutCache.getOrCreate(ctx, cmd, frameSlot, bodyIdx, key, fd);
        if (!lut) return;
        lut->bakeSkyViewOnly(ctx, cmd, frameSlot, atmoLutCache.pipelines, fd);
        lut->updateRenderBindings(ctx, frameSlot, depthView, sceneColorView);

        atmoInsidePipe.bind(cmd);
        VkDescriptorSet sets[2] = { desc->set0[_frameIdx], lut->renderSet[frameSlot % FRAMES_IN_FLIGHT] };
        vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS,
            atmoInsidePipe.layout, 0, 2, sets, 0, nullptr);
        vkCmdPushConstants(cmd, atmoInsidePipe.layout,
            VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT,
            0, sizeof(AtmoPushConstants), &pc);
        vkCmdDraw(cmd, 3, 1, 0, 0); // fullscreen triangle — no VBO
        _bindMeshPipe(cmd);
    }

    // -----------------------------------------------------------------------
    // 大气重构·壳外路径：相机在这颗星球的大气层以外看它时调用。画
    // atmoSphereVbo（_buildAtmoSphere() 早就建好、之前从未被使用的单位球壳网格），
    // 只在这颗星球实际占的那一小圈屏幕像素上跑 frag，深度测试开启，比全屏 raymarch
    // 便宜得多。LUT 获取/刷新逻辑与 drawAtmoInside 完全一致（同一个 atmoLutCache）。
    void drawAtmoShell(VulkanContext& ctx, VkCommandBuffer cmd, int frameSlot,
                       VkImageView depthView, VkImageView sceneColorView,
                       int32_t bodyIdx, const AtmoPushConstants& pc,
                       const float camPosKm[3], const float lightDirToSun[3],
                       const AtmoTuneParams& tune) {
        if (atmoShellPipe.pipeline == VK_NULL_HANDLE || atmoSphereVbo == VK_NULL_HANDLE) return;

        const float thicknessKm = pc.outerRadius - pc.surfaceRadius;
        GpuPerFrameData fd = fillAtmoLutFrameData(camPosKm, pc.planetCenter, pc.surfaceRadius,
                                                   thicknessKm, pc.planetIdx, lightDirToSun, &tune);
        AtmoLutProfileKey key = AtmoLutProfileKey::fromTune(pc.planetIdx, pc.surfaceRadius, thicknessKm, tune);
        VkAtmosphereLut* lut = atmoLutCache.getOrCreate(ctx, cmd, frameSlot, bodyIdx, key, fd);
        if (!lut) return;
        lut->bakeSkyViewOnly(ctx, cmd, frameSlot, atmoLutCache.pipelines, fd);
        // 壳外 frag 不采样场景深度（深度测试本身已经处理了"被更近物体挡住"），
        // 但 Set 1 的 binding 0 仍需要一个合法视图满足布局要求，同样刷新一下。
        lut->updateRenderBindings(ctx, frameSlot, depthView, sceneColorView);

        atmoShellPipe.bind(cmd);
        VkDescriptorSet sets[2] = { desc->set0[_frameIdx], lut->renderSet[frameSlot % FRAMES_IN_FLIGHT] };
        vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS,
            atmoShellPipe.layout, 0, 2, sets, 0, nullptr);
        vkCmdPushConstants(cmd, atmoShellPipe.layout,
            VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT,
            0, sizeof(AtmoPushConstants), &pc);
        VkDeviceSize offset = 0;
        vkCmdBindVertexBuffers(cmd, 0, 1, &atmoSphereVbo, &offset);
        vkCmdDraw(cmd, atmoSphereVerts, 1, 0, 0);
        _bindMeshPipe(cmd);
    }

    // -----------------------------------------------------------------------
    // Bake + upload Flower cloud textures (basic/detail/curl/weather).
    // Weather 像素由 weather_map_bake.h 生成；本函数只做 GPU bake / upload / 预览 view。
    // Must be called once after VulkanContext is initialized.
    bool initCloudTextures(VulkanContext& ctx) {
        printf("[VkScene] Baking cloud textures...\n");

        if (!cloudBake.init(ctx)) {
            fprintf(stderr, "[VkScene] Cloud bake pipeline init failed\n");
            return false;
        }

        // ── 1) Basic noise 128³ R8 (GPU) ─────────────────────────────────
        if (!cloudNoiseTex.createStorage3D(ctx, 128, 128, 128, VK_FORMAT_R8_UNORM) ||
            !cloudBake.bakeBasicNoise(ctx, cloudNoiseTex)) {
            fprintf(stderr, "[VkScene] GPU basic noise bake failed\n");
            return false;
        }

        // ── 2) Detail noise 32³ R8 (GPU) ─────────────────────────────────
        if (!cloudDetailTex.createStorage3D(ctx, 32, 32, 32, VK_FORMAT_R8_UNORM) ||
            !cloudBake.bakeDetailNoise(ctx, cloudDetailTex)) {
            fprintf(stderr, "[VkScene] GPU detail noise bake failed\n");
            return false;
        }

        // ── 3) Curl noise 64³ RGBA8 (GPU)：真实无散矢量场（RGB=方向 A=幅度）──
        if (!cloudCurlTex.createStorage3D(ctx, 64, 64, 64, VK_FORMAT_R8G8B8A8_UNORM) ||
            !cloudBake.bakeCurlNoise(ctx, cloudCurlTex)) {
            fprintf(stderr, "[VkScene] GPU curl noise bake failed\n");
            return false;
        }
        // ImGui 预览只能显示 2D：为 curl 3D 纹理建一个 z=0 切片的 2D view。
        {
            VkImageViewCreateInfo vci{VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO};
            vci.image                       = cloudCurlTex.image;
            vci.viewType                    = VK_IMAGE_VIEW_TYPE_2D;
            vci.format                      = VK_FORMAT_R8G8B8A8_UNORM;
            vci.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
            vci.subresourceRange.levelCount = 1;
            vci.subresourceRange.layerCount = 1;
            if (vkCreateImageView(ctx.device, &vci, nullptr, &curlVisView) != VK_SUCCESS) {
                fprintf(stderr, "[VkScene] Curl vis view create failed\n");
                curlVisView = VK_NULL_HANDLE;
            }
        }

        // ── 4) Weather map：CPU 生成像素 → upload → ImGui swizzle view ───
        {
            std::vector<uint8_t> data;
            if (!WeatherMapBake::generate(data)) {
                fprintf(stderr, "[VkScene] Weather map bake failed\n");
                return false;
            }
            if (!cloudWeatherTex.upload(ctx, data.data(), WeatherMapBake::kWidth, WeatherMapBake::kHeight)) {
                fprintf(stderr, "[VkScene] Cloud weather upload failed\n");
                return false;
            }

            auto makeSwizzleView = [&](VkComponentSwizzle c) -> VkImageView {
                VkImageViewCreateInfo vci{VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO};
                vci.image = cloudWeatherTex.image;
                vci.viewType = VK_IMAGE_VIEW_TYPE_2D;
                vci.format = VK_FORMAT_R8G8B8A8_UNORM;
                vci.components = {c, c, c, VK_COMPONENT_SWIZZLE_ONE};
                vci.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
                vci.subresourceRange.levelCount = 1;
                vci.subresourceRange.layerCount = 1;
                VkImageView v = VK_NULL_HANDLE;
                if (vkCreateImageView(ctx.device, &vci, nullptr, &v) != VK_SUCCESS) {
                    fprintf(stderr, "[VkScene] Weather vis view create failed\n");
                    return VK_NULL_HANDLE;
                }
                return v;
            };
            weatherVisViewR = makeSwizzleView(VK_COMPONENT_SWIZZLE_R);
            weatherVisViewG = makeSwizzleView(VK_COMPONENT_SWIZZLE_G);
            weatherVisViewB = makeSwizzleView(VK_COMPONENT_SWIZZLE_B);
            weatherVisViewA = makeSwizzleView(VK_COMPONENT_SWIZZLE_A);
            if (!weatherVisViewR || !weatherVisViewG || !weatherVisViewB || !weatherVisViewA)
                return false;
        }

        // ── 5) Profile LUT（云类型廓形，CPU bake）───────────────────────────
        {
            std::vector<uint8_t> data;
            if (!ProfileLutBake::generate(data)) {
                fprintf(stderr, "[VkScene] Profile LUT bake failed\n");
                return false;
            }
            if (!cloudProfileLutTex.upload(ctx, data.data(),
                    ProfileLutBake::kWidth, ProfileLutBake::kHeight)) {
                fprintf(stderr, "[VkScene] Profile LUT upload failed\n");
                return false;
            }
        }

        // ── 6) 高空云 Hi weather map（CPU bake，第二套天气图）──────────────
        {
            std::vector<uint8_t> data;
            if (!HiWeatherMapBake::generate(data)) {
                fprintf(stderr, "[VkScene] Hi weather map bake failed\n");
                return false;
            }
            if (!cloudHiWeatherTex.upload(ctx, data.data(),
                    HiWeatherMapBake::kWidth, HiWeatherMapBake::kHeight)) {
                fprintf(stderr, "[VkScene] Hi weather map upload failed\n");
                return false;
            }
        }

        printf("[VkScene] Cloud textures baked and uploaded.\n");
        return true;
    }

    // -----------------------------------------------------------------------
    // Vegetation — 两 VBO 实例化
    // vertVbo: stride=24 (vec3 aPos + vec3 aNormal)
    // instVbo: stride=20 (vec3 iPos + float iScale + float iRot)
    void drawVegetation(VkCommandBuffer cmd,
                        VkBuffer vertVbo, uint32_t vertCount,
                        VkBuffer instVbo, uint32_t instCount) {
        if (vegPipe.pipeline == VK_NULL_HANDLE || instCount == 0) return;

        vegPipe.bind(cmd);
        vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS,
            vegPipe.layout, 0, 1, &desc->set0[_frameIdx], 0, nullptr);
        VkBuffer     vbos[] = { vertVbo, instVbo };
        VkDeviceSize offs[] = { 0, 0 };
        vkCmdBindVertexBuffers(cmd, 0, 2, vbos, offs);
        vkCmdDraw(cmd, vertCount, instCount, 0, 0);
        _bindMeshPipe(cmd);
    }

    // -----------------------------------------------------------------------
    // 上传地形纹理并创建 Set1 descriptor set（每次飞行开始时调用一次）
    // tectonicData: float[w*h], values in [0,255] (normalized to [0,1] as RGBA8 R channel)
    // hydroData:    float[w*h*4] RGBA32F (R=filledHeight, G=accumulation, B=strahler, A=flowDir)
    bool uploadTerrainTextures(VulkanContext& ctx,
                               const uint8_t* tectonicRGBA8, uint32_t tw, uint32_t th,
                               const float*   hydroFloat4,   uint32_t hw, uint32_t hh) {
        if (!terrainTectonicTex.upload(ctx, tectonicRGBA8, tw, th))
            return false;
        if (!terrainHydroTex.uploadFloat4(ctx, hydroFloat4, hw, hh))
            return false;

        // 1×1 白色占位（climate + localHydro 绑定槽）
        static const uint8_t white4[4] = {255,255,255,255};
        if (terrainNullTex2.image == VK_NULL_HANDLE) {
            if (!terrainNullTex2.upload(ctx, white4, 1, 1)) return false;
        }

        // 分配 descriptor set（Set1，4 个 combined image sampler）
        VkDescriptorSetAllocateInfo ai{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO};
        ai.descriptorPool     = desc->pool;
        ai.descriptorSetCount = 1;
        ai.pSetLayouts        = &terrainPipe.set1Layout;
        if (vkAllocateDescriptorSets(ctx.device, &ai, &terrainGlobalTexSet) != VK_SUCCESS) {
            fprintf(stderr, "[VkScene] Failed to alloc terrain global tex set\n");
            return false;
        }

        VkDescriptorImageInfo imgs[4]{};
        imgs[0] = {terrainTectonicTex.sampler, terrainTectonicTex.view, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL};
        imgs[1] = {terrainHydroTex.sampler,    terrainHydroTex.view,    VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL};
        imgs[2] = {terrainNullTex2.sampler,    terrainNullTex2.view,    VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL};
        imgs[3] = {terrainNullTex2.sampler,    terrainNullTex2.view,    VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL};

        VkWriteDescriptorSet w[4]{};
        for (int i = 0; i < 4; i++) {
            w[i].sType           = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
            w[i].dstSet          = terrainGlobalTexSet;
            w[i].dstBinding      = (uint32_t)i;
            w[i].descriptorType  = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
            w[i].descriptorCount = 1;
            w[i].pImageInfo      = &imgs[i];
        }
        vkUpdateDescriptorSets(ctx.device, 4, w, 0, nullptr);
        printf("[VkScene] Terrain textures uploaded: tectonic %ux%u, hydro %ux%u\n", tw, th, hw, hh);
        return true;
    }

    // -----------------------------------------------------------------------
    // Terrain — Quadtree patch 渲染（Phase B 骨架）
    // 每 leaf patch 调用一次：绑定 terrain 管线 + push constants
    // patchVbo/patchIbo: sharedPatchMesh 的 GPU 缓冲
    // td: TerrainData UBO 内容（planetCenterRel + nodePos/Side/Up）
    // pc: push constants（model + planetRadius + maxElevation + nodeLevel + hasLocalHydro）
    // texSets: 4 纹理的 descriptor set（Set 1 = tectonic/hydro/climate/localHydro）
    void drawTerrainPatch(VkCommandBuffer cmd,
                          VkBuffer patchVbo, VkBuffer patchIbo, uint32_t indexCount,
                          const TerrainPushConstants& pc,
                          const TerrainDataUBO& td,
                          VkDescriptorSet texSet) {
        if (terrainPipe.pipeline == VK_NULL_HANDLE) return;

        // 更新 TerrainData UBO
        int fi = _frameIdx;
        if (terrainDataMapped[fi]) memcpy(terrainDataMapped[fi], &td, sizeof(TerrainDataUBO));

        // 若调用方没有提供纹理集，使用全局地形纹理集
        VkDescriptorSet resolvedTexSet = (texSet != VK_NULL_HANDLE) ? texSet : terrainGlobalTexSet;
        if (resolvedTexSet == VK_NULL_HANDLE) return; // 纹理未就绪，跳过

        terrainPipe.bind(cmd);
        VkDescriptorSet sets[] = { terrainSet0[fi], resolvedTexSet };
        vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS,
            terrainPipe.layout, 0, 2, sets, 0, nullptr);
        vkCmdPushConstants(cmd, terrainPipe.layout,
            VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT,
            0, sizeof(TerrainPushConstants), &pc);
        VkDeviceSize off = 0;
        vkCmdBindVertexBuffers(cmd, 0, 1, &patchVbo, &off);
        vkCmdBindIndexBuffer(cmd, patchIbo, 0, VK_INDEX_TYPE_UINT32);
        vkCmdDrawIndexed(cmd, indexCount, 1, 0, 0, 0);
        _bindMeshPipe(cmd);
    }

    // 每帧把云→地表阴影级联的资源接到地形 Set0（binding 2 = 图集，binding 3 = 级联描述）。
    //
    // 这些资源由 VkCloudShadowSystem 拥有，本系统只是引用；之所以每帧重绑而不是 init
    // 时绑死，是因为云系统可能在窗口 resize 时重建图集，句柄会变。任一句柄为空时回退到
    // 占位资源（1x1 白纹理 + 全零 UBO → config.x=0 → 采样函数直接返回 1，等价于无阴影）。
    //
    // 只更新当前帧的 set：另一帧的 set 可能还在被上一帧的 command buffer 使用。
    void updateTerrainCloudShadow(VkImageView atlasView, VkSampler atlasSampler, VkBuffer cascadeUbo, VkDeviceSize cascadeUboSize) {
        int fi = _frameIdx % 2;

        VkDescriptorImageInfo atlasInfo{};
        atlasInfo.sampler     = atlasSampler ? atlasSampler : nullTex.sampler;
        atlasInfo.imageView   = atlasView    ? atlasView    : nullTex.view;
        atlasInfo.imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;

        VkDescriptorBufferInfo uboInfo{};
        uboInfo.buffer = cascadeUbo ? cascadeUbo : terrainCloudShadowFallbackUbo;
        uboInfo.offset = 0;
        uboInfo.range  = cascadeUbo ? cascadeUboSize : kTerrainCloudShadowFallbackSize;
        if (uboInfo.buffer == VK_NULL_HANDLE) return;

        VkWriteDescriptorSet w[2]{};
        w[0].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
        w[0].dstSet = terrainSet0[fi]; w[0].dstBinding = 2;
        w[0].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
        w[0].descriptorCount = 1; w[0].pImageInfo = &atlasInfo;
        w[1].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
        w[1].dstSet = terrainSet0[fi]; w[1].dstBinding = 3;
        w[1].descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
        w[1].descriptorCount = 1; w[1].pBufferInfo = &uboInfo;
        vkUpdateDescriptorSets(_device, 2, w, 0, nullptr);
    }

private:
    // 绑定 mesh 管线 + set0 (UBO) + set1 (null 纹理) — 帧内默认状态
    void _bindMeshPipe(VkCommandBuffer cmd) {
        meshPipe.bind(cmd);
        VkDescriptorSet sets[] = { desc->set0[_frameIdx], nullTexSet };
        vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS,
            meshPipe.layout, 0, 2, sets, 0, nullptr);
    }

    // 生成 unit cube（8 顶点，36 索引，pos-only stride=12），用于排气羽流体积渲染
    bool _buildExhaustCube(VulkanContext& ctx) {
        static const float verts[] = {
            -0.5f,-0.5f,-0.5f,  0.5f,-0.5f,-0.5f,  0.5f, 0.5f,-0.5f, -0.5f, 0.5f,-0.5f,
            -0.5f,-0.5f, 0.5f,  0.5f,-0.5f, 0.5f,  0.5f, 0.5f, 0.5f, -0.5f, 0.5f, 0.5f,
        };
        static const uint32_t idx[] = {
            0,1,2, 0,2,3,  // -Z 面
            4,6,5, 4,7,6,  // +Z 面
            0,5,1, 0,4,5,  // -Y 面
            2,6,3, 3,6,7,  // +Y 面
            0,3,7, 0,7,4,  // -X 面
            1,5,6, 1,6,2,  // +X 面
        };
        exhaustCubeIcount = 36;

        VkBufferCreateInfo bci{ VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO };
        VmaAllocationCreateInfo aci{};
        aci.usage = VMA_MEMORY_USAGE_CPU_TO_GPU;
        aci.flags = VMA_ALLOCATION_CREATE_MAPPED_BIT;
        VmaAllocationInfo info;

        bci.size  = sizeof(verts);
        bci.usage = VK_BUFFER_USAGE_VERTEX_BUFFER_BIT;
        if (vmaCreateBuffer(ctx.allocator, &bci, &aci, &exhaustCubeVbo, &exhaustCubeAlloc, &info) != VK_SUCCESS)
            return false;
        memcpy(info.pMappedData, verts, sizeof(verts));

        bci.size  = sizeof(idx);
        bci.usage = VK_BUFFER_USAGE_INDEX_BUFFER_BIT;
        if (vmaCreateBuffer(ctx.allocator, &bci, &aci, &exhaustCubeIbo, &exhaustCubeIboAlloc, &info) != VK_SUCCESS)
            return false;
        memcpy(info.pMappedData, idx, sizeof(idx));
        return true;
    }

    // 生成 UV sphere（24×24 格，pos-only stride=12），上传到 CPU_TO_GPU 缓冲
    bool _buildAtmoSphere(VulkanContext& ctx) {
        const int R = 128, S = 128;
        std::vector<float> pts;
        pts.reserve(R * S * 6 * 3);

        for (int r = 0; r < R; r++) {
            float phi0 = kPI_f * r       / R;
            float phi1 = kPI_f * (r + 1) / R;
            for (int s = 0; s < S; s++) {
                float th0 = 2.f * kPI_f * s       / S;
                float th1 = 2.f * kPI_f * (s + 1) / S;
                float p[4][3] = {
                    { sinf(phi0)*cosf(th0), cosf(phi0), sinf(phi0)*sinf(th0) },
                    { sinf(phi0)*cosf(th1), cosf(phi0), sinf(phi0)*sinf(th1) },
                    { sinf(phi1)*cosf(th0), cosf(phi1), sinf(phi1)*sinf(th0) },
                    { sinf(phi1)*cosf(th1), cosf(phi1), sinf(phi1)*sinf(th1) },
                };
                int tris[2][3] = { {0, 2, 1}, {1, 2, 3} };
                for (auto& t : tris)
                    for (int v : t)
                        for (int k = 0; k < 3; k++)
                            pts.push_back(p[v][k]);
            }
        }

        atmoSphereVerts = (uint32_t)(pts.size() / 3);
        VkDeviceSize size = pts.size() * sizeof(float);

        VkBufferCreateInfo bci{ VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO };
        bci.size  = size;
        bci.usage = VK_BUFFER_USAGE_VERTEX_BUFFER_BIT;

        VmaAllocationCreateInfo aci{};
        aci.usage = VMA_MEMORY_USAGE_CPU_TO_GPU;
        aci.flags = VMA_ALLOCATION_CREATE_MAPPED_BIT;

        VmaAllocationInfo info;
        if (vmaCreateBuffer(ctx.allocator, &bci, &aci,
                            &atmoSphereVbo, &atmoSphereAlloc, &info) != VK_SUCCESS)
            return false;
        memcpy(info.pMappedData, pts.data(), (size_t)size);
        return true;
    }

    // Billboard quad: 4 × vec2（fullscreen quad for lens flare / particles）
    bool _buildBillboardQuad(VulkanContext& ctx) {
        static const float verts[] = { -1,-1, 1,-1, -1,1, 1,1 };
        VkBufferCreateInfo bci{ VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO };
        bci.size  = sizeof(verts);
        bci.usage = VK_BUFFER_USAGE_VERTEX_BUFFER_BIT;
        VmaAllocationCreateInfo aci{};
        aci.usage = VMA_MEMORY_USAGE_CPU_TO_GPU;
        aci.flags = VMA_ALLOCATION_CREATE_MAPPED_BIT;
        VmaAllocationInfo info;
        if (vmaCreateBuffer(ctx.allocator, &bci, &aci, &billboardQuadVbo, &billboardQuadAlloc, &info) != VK_SUCCESS)
            return false;
        memcpy(info.pMappedData, verts, sizeof(verts));
        return true;
    }

    // Terrain shared patch mesh: 32×32 grid, Vertex3D format
    bool _buildTerrainPatch(VulkanContext& ctx) {
        const int U = 32;
        std::vector<float> v; std::vector<uint32_t> idx;
        float step = 1.f / U;
        for (int z = 0; z <= U; z++) {
            for (int x = 0; x <= U; x++) {
                float px = x*step - 0.5f, pz = z*step - 0.5f;
                float vb[] = { px, 0, pz,  0,1,0,  x*step, z*step,  1,1,1,1 };
                v.insert(v.end(), vb, vb + 12);
            }
        }
        for (int z = 0; z < U; z++) {
            for (int x = 0; x < U; x++) {
                int i0 = z*(U+1)+x, i1=i0+1, i2=i0+U+1, i3=i2+1;
                idx.insert(idx.end(), { (uint32_t)i0,(uint32_t)i2,(uint32_t)i1, (uint32_t)i1,(uint32_t)i2,(uint32_t)i3 });
            }
        }
        terrainPatchIcount = (uint32_t)idx.size();
        VmaAllocationCreateInfo aci{}; aci.usage=VMA_MEMORY_USAGE_CPU_TO_GPU; aci.flags=VMA_ALLOCATION_CREATE_MAPPED_BIT;
        VmaAllocationInfo info;
        VkBufferCreateInfo bci{VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO};
        bci.size=v.size()*4; bci.usage=VK_BUFFER_USAGE_VERTEX_BUFFER_BIT;
        if(vmaCreateBuffer(ctx.allocator,&bci,&aci,&terrainPatchVbo,&terrainPatchAlloc,&info)!=VK_SUCCESS)return false;
        memcpy(info.pMappedData,v.data(),v.size()*4);
        bci.size=idx.size()*4; bci.usage=VK_BUFFER_USAGE_INDEX_BUFFER_BIT;
        if(vmaCreateBuffer(ctx.allocator,&bci,&aci,&terrainPatchIbo,&terrainPatchIboAlloc,&info)!=VK_SUCCESS)return false;
        memcpy(info.pMappedData,idx.data(),idx.size()*4);
        return true;
    }

    // 为 terrain 管线创建专属 descriptor set（Set0 = FrameUBO + TerrainData）
    bool _initTerrainDescriptors(VulkanContext& ctx) {
        VkDescriptorSetLayout layout = terrainPipe.set0Layout;
        if (layout == VK_NULL_HANDLE) return false;

        // 每帧一份 TerrainData UBO
        for (int i = 0; i < 2; i++) {
            VkBufferCreateInfo bci{VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO};
            bci.size = sizeof(TerrainDataUBO);
            bci.usage = VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT;
            VmaAllocationCreateInfo aci{};
            aci.usage = VMA_MEMORY_USAGE_CPU_TO_GPU;
            aci.flags = VMA_ALLOCATION_CREATE_MAPPED_BIT;
            VmaAllocationInfo info;
            if (vmaCreateBuffer(ctx.allocator, &bci, &aci, &terrainDataUbo[i], &terrainDataAlloc[i], &info) != VK_SUCCESS)
                return false;
            terrainDataMapped[i] = info.pMappedData;
        }

        // 云影级联的占位 UBO：全零写一次即可，之后不再改动（内容语义 = 无阴影）。
        {
            VkBufferCreateInfo bci{VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO};
            bci.size = kTerrainCloudShadowFallbackSize;
            bci.usage = VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT;
            VmaAllocationCreateInfo aci{};
            aci.usage = VMA_MEMORY_USAGE_CPU_TO_GPU;
            aci.flags = VMA_ALLOCATION_CREATE_MAPPED_BIT;
            VmaAllocationInfo info;
            if (vmaCreateBuffer(ctx.allocator, &bci, &aci,
                                &terrainCloudShadowFallbackUbo, &terrainCloudShadowFallbackAlloc, &info) != VK_SUCCESS)
                return false;
            GpuCloudShadowData zero{};
            memcpy(info.pMappedData, &zero, sizeof(zero));
        }

        // 分配 descriptor sets（terrain 使用自己的 pool）
        // 共享 desc->pool
        for (int i = 0; i < 2; i++) {
            VkDescriptorSetAllocateInfo ai{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO};
            ai.descriptorPool = desc->pool;
            ai.descriptorSetCount = 1;
            ai.pSetLayouts = &layout;
            if (vkAllocateDescriptorSets(ctx.device, &ai, &terrainSet0[i]) != VK_SUCCESS)
                return false;

            VkDescriptorBufferInfo fubi{};  // binding 0 = FrameUBO
            fubi.buffer = desc->uboBuffers[i];
            fubi.offset = 0; fubi.range = sizeof(FrameUBO);

            VkDescriptorBufferInfo tdbi{};  // binding 1 = TerrainData
            tdbi.buffer = terrainDataUbo[i];
            tdbi.offset = 0; tdbi.range = sizeof(TerrainDataUBO);

            // binding 2/3 = 云影级联图集 + 级联描述。初始绑占位资源；云系统就绪后由
            // updateTerrainCloudShadow() 每帧重定向到真实资源。
            VkDescriptorImageInfo csdi{};
            csdi.sampler = nullTex.sampler;
            csdi.imageView = nullTex.view;
            csdi.imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;

            VkDescriptorBufferInfo csbi{};
            csbi.buffer = terrainCloudShadowFallbackUbo;
            csbi.offset = 0; csbi.range = kTerrainCloudShadowFallbackSize;

            VkWriteDescriptorSet w[4]{};
            w[0].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
            w[0].dstSet = terrainSet0[i]; w[0].dstBinding = 0;
            w[0].descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
            w[0].descriptorCount = 1; w[0].pBufferInfo = &fubi;
            w[1].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
            w[1].dstSet = terrainSet0[i]; w[1].dstBinding = 1;
            w[1].descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
            w[1].descriptorCount = 1; w[1].pBufferInfo = &tdbi;
            w[2].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
            w[2].dstSet = terrainSet0[i]; w[2].dstBinding = 2;
            w[2].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
            w[2].descriptorCount = 1; w[2].pImageInfo = &csdi;
            w[3].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
            w[3].dstSet = terrainSet0[i]; w[3].dstBinding = 3;
            w[3].descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
            w[3].descriptorCount = 1; w[3].pBufferInfo = &csbi;
            vkUpdateDescriptorSets(ctx.device, 4, w, 0, nullptr);
        }
        return true;
    }

    // 构建植被顶点 mesh（树桩 + 锥形顶部）
    bool _buildVegetationMesh(VulkanContext& ctx) {
        struct VegV { float px,py,pz, nx,ny,nz, u,v, cr,cg,cb,ca; };
        std::vector<VegV> verts;
        // 树桩圆柱体
        float r=0.15f, h=0.8f; int segs=8;
        for (int i=0;i<=segs;i++) {
            float a=i*(6.28318530718f/segs), cs=cosf(a), sn=sinf(a);
            verts.push_back({cs*r,0,sn*r, cs,0,sn, i/(float)segs,1.f, 0.4f,0.25f,0.1f,1});
            verts.push_back({cs*r,h,sn*r, cs,0,sn, i/(float)segs,0.f, 0.4f,0.25f,0.1f,1});
        }
        // 锥形顶部
        verts.push_back({0,h+2.f,0, 0,1,0, 0.5f,0, 0.1f,0.35f,0.05f,1});
        float cR=0.6f;
        for (int i=0;i<=segs;i++) {
            float a=i*(6.28318530718f/segs), cs=cosf(a), sn=sinf(a);
            verts.push_back({cs*cR,h,sn*cR, cs,0.3f,sn, i/(float)segs,1.f, 0.1f,0.35f,0.05f,1});
        }
        vegVertCount = (uint32_t)verts.size();

        VkBufferCreateInfo bci{VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO};
        bci.size=verts.size()*sizeof(VegV); bci.usage=VK_BUFFER_USAGE_VERTEX_BUFFER_BIT;
        VmaAllocationCreateInfo aci{}; aci.usage=VMA_MEMORY_USAGE_CPU_TO_GPU; aci.flags=VMA_ALLOCATION_CREATE_MAPPED_BIT;
        VmaAllocationInfo info;
        if(vmaCreateBuffer(ctx.allocator,&bci,&aci,&vegVertVbo,&vegVertAlloc,&info)!=VK_SUCCESS)return false;
        memcpy(info.pMappedData,verts.data(),verts.size()*sizeof(VegV));

        // 实例 VBO（预分配 4096 实例）
        bci.size=4096*20; bci.usage=VK_BUFFER_USAGE_VERTEX_BUFFER_BIT;
        aci.flags=VMA_ALLOCATION_CREATE_MAPPED_BIT|VMA_ALLOCATION_CREATE_HOST_ACCESS_SEQUENTIAL_WRITE_BIT;
        if(vmaCreateBuffer(ctx.allocator,&bci,&aci,&vegInstanceVbo,&vegInstanceAlloc,&info)!=VK_SUCCESS)return false;
        vegInstanceMapped=info.pMappedData;
        return true;
    }
};
