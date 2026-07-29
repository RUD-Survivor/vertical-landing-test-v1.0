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
#include "../vk_atmosphere_lut.h" // 大气重构：VkAtmoLutCache + fillAtmoLutFrameData
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

    // Terrain 全局纹理（Set1: tectonic/hydro/climate/null）
    VkTexture2D     terrainTectonicTex;
    VkTexture2D     terrainHydroTex;
    VkTexture2D     terrainNullTex2;
    VkDescriptorSet terrainGlobalTexSet = VK_NULL_HANDLE;

    // Cloud textures for Flower compute path (basic/detail/weather/curl)
    VkTexture3D     cloudNoiseTex;
    VkTexture3D     cloudDetailTex;
    VkTexture2D     cloudWeatherTex;
    // cloud_common.glsl::cloudMap() localCoverage（binding 9）
    VkTexture2D     cloudCurlTex;
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

    // -----------------------------------------------------------------------
    bool init(VulkanContext& ctx, VkDescriptorManager& d,
              VkFormat colorFmt, VkFormat depthFmt) {
        desc = &d;

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
        cloudNoiseTex.destroy(ctx);
        cloudDetailTex.destroy(ctx);
        cloudWeatherTex.destroy(ctx);
        cloudCurlTex.destroy(ctx);
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

        // ── CPU FBM helpers (weather map) ────────────────────────────────
        auto cpuFract = [](float x) { return x - floorf(x); };
        auto cpuHash = [&](float px, float py, float pz) {
            px=cpuFract(px*443.897f); py=cpuFract(py*441.423f); pz=cpuFract(pz*437.195f);
            float d=px*(py+19.19f)+py*(pz+19.19f)+pz*(px+19.19f);
            px+=d; py+=d; pz+=d;
            return cpuFract((px+py)*pz);
        };
        auto cpuNoise3d = [&](float px, float py, float pz) {
            float ix=floorf(px),iy=floorf(py),iz=floorf(pz);
            float fx=px-ix,fy=py-iy,fz=pz-iz;
            fx=fx*fx*fx*(fx*(fx*6-15)+10); fy=fy*fy*fy*(fy*(fy*6-15)+10); fz=fz*fz*fz*(fz*(fz*6-15)+10);
            float n000=cpuHash(ix,iy,iz),n100=cpuHash(ix+1,iy,iz),n010=cpuHash(ix,iy+1,iz),n110=cpuHash(ix+1,iy+1,iz);
            float n001=cpuHash(ix,iy,iz+1),n101=cpuHash(ix+1,iy,iz+1),n011=cpuHash(ix,iy+1,iz+1),n111=cpuHash(ix+1,iy+1,iz+1);
            float nx00=n000+fx*(n100-n000),nx10=n010+fx*(n110-n010),nx01=n001+fx*(n101-n001),nx11=n011+fx*(n111-n011);
            return (nx00+fy*(nx10-nx00))+fz*((nx01+fy*(nx11-nx01))-(nx00+fy*(nx10-nx00)));
        };
        auto cpuFbm = [&](float px, float py, float pz, int oct) {
            float v=0,amp=0.5f;
            for(int i=0;i<oct;i++){v+=cpuNoise3d(px,py,pz)*amp;px=px*2.07f+0.131f;py=py*2.07f-0.217f;pz=pz*2.07f+0.344f;amp*=0.48f;}
            return v;
        };
        auto clampU8 = [](float v) { int i=(int)(v*255+0.5f); return (uint8_t)(i<0?0:i>255?255:i); };

        // ── 2) Detail noise 32³ R8 (GPU) ─────────────────────────────────
        if (!cloudDetailTex.createStorage3D(ctx, 32, 32, 32, VK_FORMAT_R8_UNORM) ||
            !cloudBake.bakeDetailNoise(ctx, cloudDetailTex)) {
            fprintf(stderr, "[VkScene] GPU detail noise bake failed\n");
            return false;
        }

        // ── 3) Curl noise 256² R8 (GPU) ──────────────────────────────────
        if (!cloudCurlTex.createStorage2D(ctx, 256, 256, VK_FORMAT_R8_UNORM) ||
            !cloudBake.bakeCurlNoise(ctx, cloudCurlTex)) {
            fprintf(stderr, "[VkScene] GPU curl noise bake failed\n");
            return false;
        }

        // ── 4) Weather 2048×1024 ─────────────────────────────────────────
        // 气候带均值 × 球面 domain-warped 天气场 → 锋面/斑块/破碎 ITCZ，
        // 而不是纯纬度平滑色带。RGBA: R=coverage G=cloudType B=humidity A=land。
        {
            const int W=2048,H=1024; std::vector<uint8_t> data(W*H*4);
            constexpr float kDeg2Rad = kPI_f / 180.f;
            auto sat01=[](float v){return v<0.f?0.f:(v>1.f?1.f:v);};
            auto ellipse=[&](float lon,float lat,float cLon,float cLat,float hW,float hH){
                float dlon=lon-cLon;if(dlon>180)dlon-=360;if(dlon<-180)dlon+=360;
                float dlat=lat-cLat;float r=sqrtf((dlon/hW)*(dlon/hW)+(dlat/hH)*(dlat/hH));
                if(r>=1.3f)return 0.f;if(r<=0.7f)return 1.f;
                float tt=(r-0.7f)/0.6f;return 1-tt*tt*(3-2*tt);
            };
            auto gauss=[](float x,float c,float s){float d=(x-c)/s;return expf(-0.5f*d*d);};
            printf("[VkScene] Baking weather map (climate × domain-warp)...\n");
            for(int y=0;y<H;y++){
                float lat=((float)y/H-0.5f)*180.f;
                for(int x=0;x<W;x++){
                    float lon=((float)x/W-0.5f)*360.f;
                    float land=0;
                    land=fmaxf(land,ellipse(lon,lat,-98,52,42,30));
                    land=fmaxf(land,ellipse(lon,lat,-90,18,13,12));
                    land=fmaxf(land,ellipse(lon,lat,-60,-14,22,27));
                    land=fmaxf(land,ellipse(lon,lat,12,52,22,18));
                    land=fmaxf(land,ellipse(lon,lat,17,20,38,22));
                    land=fmaxf(land,ellipse(lon,lat,22,-10,28,26));
                    land=fmaxf(land,ellipse(lon,lat,45,23,18,13));
                    land=fmaxf(land,ellipse(lon,lat,73,28,18,14));
                    land=fmaxf(land,ellipse(lon,lat,108,36,30,28));
                    land=fmaxf(land,ellipse(lon,lat,80,62,65,17));
                    land=fmaxf(land,ellipse(lon,lat,102,15,15,12));
                    land=fmaxf(land,ellipse(lon,lat,118,-3,20,7));
                    land=fmaxf(land,ellipse(lon,lat,134,-26,22,16));
                    land=fmaxf(land,ellipse(lon,lat,-42,72,22,12));
                    {float tt=fminf(1.f,fmaxf(0.f,(-lat-70.f)/14.f));land=fmaxf(land,tt*tt*(3.f-2.f*tt));}
                    land=fminf(land,1.f);

                    float absLat=fabsf(lat);
                    float wITCZ=gauss(absLat,2.f,6.f),wSubDry=gauss(absLat,27.f,10.f);
                    float wTemperate=gauss(absLat,52.f,13.f),wPolar=gauss(absLat,75.f,9.f);
                    float wSum=wITCZ+wSubDry+wTemperate+wPolar+1e-6f;
                    wITCZ/=wSum;wSubDry/=wSum;wTemperate/=wSum;wPolar/=wSum;

                    float cov_oc=wITCZ*0.82f+wSubDry*0.35f+wTemperate*0.68f+wPolar*0.52f;
                    float type_oc=wITCZ*0.84f+wSubDry*0.15f+wTemperate*0.48f+wPolar*0.12f;
                    float hum_oc=wITCZ*0.88f+wSubDry*0.52f+wTemperate*0.72f+wPolar*0.58f;
                    float cov_la=wITCZ*0.55f+wSubDry*0.28f+wTemperate*0.48f+wPolar*0.38f;
                    float type_la=wITCZ*0.65f+wSubDry*0.30f+wTemperate*0.38f+wPolar*0.08f;
                    float hum_la=wITCZ*0.62f+wSubDry*0.28f+wTemperate*0.52f+wPolar*0.42f;
                    float climateCov=cov_oc+(cov_la-cov_oc)*land;
                    float climateType=type_oc+(type_la-type_oc)*land;
                    float climateHum=hum_oc+(hum_la-hum_oc)*land;

                    // 球面坐标（无缝），再做两级 domain warp 打破纬线圈
                    float lonR=lon*kDeg2Rad, latR=lat*kDeg2Rad;
                    float cl=cosf(latR);
                    float sx=cl*cosf(lonR), sy=cl*sinf(lonR), sz=sinf(latR);

                    float wx=cpuFbm(sx*2.1f+17.1f,sy*2.1f-3.4f,sz*2.1f+9.2f,3);
                    float wy=cpuFbm(sx*2.1f-8.7f,sy*2.1f+22.3f,sz*2.1f-4.1f,3);
                    float wz=cpuFbm(sx*2.1f+5.5f,sy*2.1f+11.0f,sz*2.1f+30.6f,3);
                    const float warp1=0.55f;
                    float qx=sx+(wx*2.f-1.f)*warp1;
                    float qy=sy+(wy*2.f-1.f)*warp1;
                    float qz=sz+(wz*2.f-1.f)*warp1;

                    float w2x=cpuFbm(qx*3.3f+41.f,qy*3.3f,qz*3.3f-19.f,2);
                    float w2y=cpuFbm(qx*3.3f-27.f,qy*3.3f+8.f,qz*3.3f,2);
                    float w2z=cpuFbm(qx*3.3f,qy*3.3f+13.f,qz*3.3f+7.f,2);
                    const float warp2=0.35f;
                    float rx=qx+(w2x*2.f-1.f)*warp2;
                    float ry=qy+(w2y*2.f-1.f)*warp2;
                    float rz=qz+(w2z*2.f-1.f)*warp2;

                    // 天气尺度场 + 温带 storm-track（略沿经向拉长）
                    float synoptic=cpuFbm(rx*4.5f,ry*4.5f,rz*4.5f,4);
                    float midLat=gauss(absLat,48.f,14.f);
                    float track=cpuFbm(sx*1.2f+sy*3.8f,sz*2.0f,sx*0.5f-sy*0.3f+40.f,3);
                    float band=sat01(synoptic*0.72f+track*0.48f*midLat);

                    // 破碎 ITCZ（不是一整圈均匀云带）
                    float itczBreak=cpuFbm(sx*6.f+3.f,sy*6.f,sz*1.5f+50.f,3);
                    float itczMask=gauss(absLat,2.f,7.f)*sat01(itczBreak*1.65f-0.22f);

                    float weatherField=sat01(band*0.85f+itczMask*0.55f);
                    weatherField=sat01((weatherField-0.28f)/0.42f); // 高对比：大片晴空 vs 浓云
                    weatherField=weatherField*weatherField*(3.f-2.f*weatherField);
                    weatherField=powf(weatherField,1.12f);

                    float coverage=sat01(climateCov*(0.20f+0.98f*weatherField));
                    float coast=land*(1.f-land)*4.f; // 海岸抬升一点点
                    coverage=sat01(coverage+coast*0.08f*weatherField);

                    float cloudTypeV=sat01(climateType*(0.35f+0.75f*weatherField)+(1.f-land)*0.04f);
                    float humidity=sat01(climateHum*(0.30f+0.80f*(0.45f*weatherField+0.55f*synoptic)));

                    int idx=(y*W+x)*4;
                    data[idx]  =clampU8(coverage);
                    data[idx+1]=clampU8(cloudTypeV);
                    data[idx+2]=clampU8(humidity);
                    data[idx+3]=clampU8(land);
                }
            }
            if(!cloudWeatherTex.upload(ctx,data.data(),W,H)){fprintf(stderr,"[VkScene] Cloud weather upload failed\n");return false;}

            // ImGui 预览用通道 swizzle view（不复制像素）
            auto makeSwizzleView=[&](VkComponentSwizzle c)->VkImageView{
                VkImageViewCreateInfo vci{VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO};
                vci.image=cloudWeatherTex.image;
                vci.viewType=VK_IMAGE_VIEW_TYPE_2D;
                vci.format=VK_FORMAT_R8G8B8A8_UNORM;
                vci.components={c,c,c,VK_COMPONENT_SWIZZLE_ONE};
                vci.subresourceRange.aspectMask=VK_IMAGE_ASPECT_COLOR_BIT;
                vci.subresourceRange.levelCount=1;
                vci.subresourceRange.layerCount=1;
                VkImageView v=VK_NULL_HANDLE;
                if(vkCreateImageView(ctx.device,&vci,nullptr,&v)!=VK_SUCCESS){
                    fprintf(stderr,"[VkScene] Weather vis view create failed\n");
                    return VK_NULL_HANDLE;
                }
                return v;
            };
            weatherVisViewR=makeSwizzleView(VK_COMPONENT_SWIZZLE_R);
            weatherVisViewG=makeSwizzleView(VK_COMPONENT_SWIZZLE_G);
            weatherVisViewB=makeSwizzleView(VK_COMPONENT_SWIZZLE_B);
            weatherVisViewA=makeSwizzleView(VK_COMPONENT_SWIZZLE_A);
            if(!weatherVisViewR||!weatherVisViewG||!weatherVisViewB||!weatherVisViewA)return false;
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

        // 创建 TerrainData UBO（每帧 64 bytes）
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

            VkWriteDescriptorSet w[2]{};
            w[0].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
            w[0].dstSet = terrainSet0[i]; w[0].dstBinding = 0;
            w[0].descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
            w[0].descriptorCount = 1; w[0].pBufferInfo = &fubi;
            w[1].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
            w[1].dstSet = terrainSet0[i]; w[1].dstBinding = 1;
            w[1].descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
            w[1].descriptorCount = 1; w[1].pBufferInfo = &tdbi;
            vkUpdateDescriptorSets(ctx.device, 2, w, 0, nullptr);
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
