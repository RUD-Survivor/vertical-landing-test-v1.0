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

#include <unordered_map>
#include <string>
#include <vector>
#include <cstring>
#include <cstdio>
#include <cmath>

struct VkSceneSystem {
    VkMeshPipeline       meshPipe;
    VkSVOPipeline        svoPipe;
    VkRingPipeline       ringPipe;
    VkAtmoPipeline       atmoPipe;
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
        for (auto& e : meshCache)   e.second.destroy(ctx);
        meshCache.clear();
        nullTex.destroy(ctx);
        for (auto& e : planetPipes) e.second.shutdown(ctx.device);
        planetPipes.clear();
        vegPipe.shutdown(ctx.device);
        terrainPipe.shutdown(ctx.device);
        atmoPipe.shutdown(ctx.device);
        ringPipe.shutdown(ctx.device);
        svoPipe.shutdown(ctx.device);
        meshPipe.shutdown(ctx.device);
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

    // -----------------------------------------------------------------------
    void drawMesh(VkCommandBuffer cmd, const std::string& id, const float model[16],
                  float r=1.f, float g=1.f, float b=1.f, float a=1.f, float ambient=0.15f) {
        auto it = meshCache.find(id);
        if (it == meshCache.end()) return;
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
    void drawPlanet(VkCommandBuffer cmd, const std::string& meshId, const float model[16],
                    float cx, float cy, float cz,
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
        vkCmdPushConstants(cmd, pit->second.layout,
            VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT,
            0, sizeof(PlanetPushConstants), &pc);
        mit->second.bind(cmd);
        mit->second.draw(cmd);
        _bindMeshPipe(cmd);
    }

    // -----------------------------------------------------------------------
    void drawRing(VkCommandBuffer cmd, const std::string& meshId, const float model[16],
                  float cx, float cy, float cz, float planetRadius) {
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
    // Atmosphere — 使用内建 unit sphere VBO（着色器按 pc.outerRadius 缩放）
    // 须在不透明几何之后、天空盒之前调用（alpha 混合，深度测试 LESS_OR_EQUAL）
    void drawAtmosphere(VkCommandBuffer cmd, const AtmoPushConstants& pc) {
        if (atmoPipe.pipeline == VK_NULL_HANDLE || atmoSphereVerts == 0) return;

        atmoPipe.bind(cmd);
        // atmoPipe 只有 set0；显式重绑确保切换后 set0 有效
        vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS,
            atmoPipe.layout, 0, 1, &desc->set0[_frameIdx], 0, nullptr);
        vkCmdPushConstants(cmd, atmoPipe.layout,
            VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT,
            0, sizeof(AtmoPushConstants), &pc);
        VkDeviceSize off = 0;
        vkCmdBindVertexBuffers(cmd, 0, 1, &atmoSphereVbo, &off);
        vkCmdDraw(cmd, atmoSphereVerts, 1, 0, 0);
        _bindMeshPipe(cmd);  // 恢复 set1（切走后已失效）
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
    // Terrain — Quadtree patch 渲染（Phase B 骨架）
    // 每 leaf patch 调用一次：绑定 terrain 管线 + push constants
    // patchVbo/patchIbo: sharedPatchMesh 的 GPU 缓冲
    // td: TerrainData UBO 内容（planetCenterRel + nodePos/Side/Up）
    // pc: push constants（model + planetRadius + maxElevation + nodeLevel + hasLocalHydro）
    // texSets: 4 纹理的 descriptor set（Set 1 = tectonic/hydro/climate/localHydro）
    void drawTerrainPatch(VkCommandBuffer cmd,
                          VkBuffer patchVbo, VkBuffer patchIbo, uint32_t indexCount,
                          const TerrainPushConstants& pc,
                          VkDescriptorSet texSet) {
        if (terrainPipe.pipeline == VK_NULL_HANDLE) return;

        terrainPipe.bind(cmd);
        // Set 0 = FrameUBO + TerrainData, Set 1 = textures
        VkDescriptorSet sets[] = { desc->set0[_frameIdx], texSet };
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
        const int R = 24, S = 24;
        std::vector<float> pts;
        pts.reserve(R * S * 6 * 3);

        for (int r = 0; r < R; r++) {
            float phi0 = (float)M_PI * r       / R;
            float phi1 = (float)M_PI * (r + 1) / R;
            for (int s = 0; s < S; s++) {
                float th0 = 2.f * (float)M_PI * s       / S;
                float th1 = 2.f * (float)M_PI * (s + 1) / S;
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
};
