#pragma once
// ==========================================================================
// vk_renderer3d.h — Vulkan 等价的 Renderer3D::drawMesh 路径
//
// 使用方式（每帧）：
//   1. beginFrame(cmd, frameIdx, view, proj, lightDir, viewPos, time)
//   2. drawMesh(cmd, id, model, r, g, b, a, ambient)  × N
//
// 网格注册（首次）：
//   registerMesh(ctx, "rocket_body", verts.data(), verts.size()*48,
//                indices.data(), indexCount)
//
// model/view/proj 均为列主序 float[16]（与 Mat4::toFloatArray 输出一致）
// ==========================================================================

#include "vk_context.h"
#include "vk_mesh.h"
#include "vk_descriptors.h"
#include "vk_pipeline.h"
#include "vk_texture.h"

#include <unordered_map>
#include <string>
#include <cstring>
#include <cstdio>

struct VkRenderer3D {
    VkMeshPipeline*      pipe = nullptr;
    VkDescriptorManager* desc = nullptr;

    // 1×1 白色占位纹理（无纹理网格使用）
    VkTexture2D     nullTex;
    VkDescriptorSet nullTexSet = VK_NULL_HANDLE;

    // 网格缓存：string ID → VkMesh（稳定键，适合 static 和 cached 模型）
    std::unordered_map<std::string, VkMesh> meshCache;

    // -----------------------------------------------------------------------
    bool init(VulkanContext& ctx, VkMeshPipeline& p, VkDescriptorManager& d) {
        pipe = &p;
        desc = &d;
        static const uint8_t white[4] = {255, 255, 255, 255};
        if (!nullTex.upload(ctx, white, 1, 1)) {
            fprintf(stderr, "[VkRenderer3D] Failed to create null texture\n");
            return false;
        }
        nullTexSet = d.allocateTextureSet(ctx.device, nullTex.view, nullTex.sampler);
        if (nullTexSet == VK_NULL_HANDLE) {
            fprintf(stderr, "[VkRenderer3D] Failed to allocate null texture descriptor\n");
            return false;
        }
        printf("[VkRenderer3D] Initialized\n");
        return true;
    }

    void shutdown(VulkanContext& ctx) {
        for (auto& entry : meshCache)
            entry.second.destroy(ctx);
        meshCache.clear();
        nullTex.destroy(ctx);
    }

    // -----------------------------------------------------------------------
    // 注册（或更新）一个网格。
    //   id         : 唯一字符串键，如 "rocket_body"、"terrain_0"
    //   vertexData : Vertex3D 数组的指针（每个顶点 48 字节）
    //   vertexSize : 字节总数 = vertCount * 48
    //   indexData  : uint32_t 索引数组
    //   indexCount : 索引数量
    // -----------------------------------------------------------------------
    bool registerMesh(VulkanContext& ctx,
                      const std::string& id,
                      const void*     vertexData, size_t vertexSize,
                      const uint32_t* indexData,  uint32_t indexCount) {
        auto it = meshCache.find(id);
        if (it != meshCache.end())
            it->second.destroy(ctx);

        VkMesh& m = meshCache[id];
        if (!m.upload(ctx, vertexData, (VkDeviceSize)vertexSize, indexData, indexCount)) {
            fprintf(stderr, "[VkRenderer3D] Failed to upload mesh: %s\n", id.c_str());
            meshCache.erase(id);
            return false;
        }
        return true;
    }

    bool hasMesh(const std::string& id) const {
        return meshCache.count(id) > 0;
    }

    // -----------------------------------------------------------------------
    // 帧开始：更新 FrameUBO，绑定 Pipeline + Set 0（UBO）+ Set 1（null tex）。
    // 必须在同一帧的所有 drawMesh 之前调用一次。
    //   view / proj : 列主序 float[16]，由 Mat4::toFloatArray 获得
    //   lightDir    : float[3]，单位向量
    //   viewPos     : float[3]，世界空间相机位置
    // -----------------------------------------------------------------------
    void beginFrame(VkCommandBuffer cmd, int frameIdx,
                    const float view[16], const float proj[16],
                    const float lightDir[3], const float viewPos[3],
                    float time) {
        FrameUBO ubo{};
        memcpy(ubo.view,     view,     64);
        memcpy(ubo.proj,     proj,     64);
        memcpy(ubo.lightDir, lightDir, 12);
        memcpy(ubo.viewPos,  viewPos,  12);
        ubo.time = time;
        desc->updateFrameUBO(frameIdx, ubo);

        pipe->bind(cmd);
        VkDescriptorSet sets[] = { desc->set0[frameIdx], nullTexSet };
        vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS,
            pipe->layout, 0, 2, sets, 0, nullptr);
    }

    // -----------------------------------------------------------------------
    // 绘制一个已注册的网格。
    //   id     : registerMesh 时使用的字符串键
    //   model  : 列主序 float[16]，由 Mat4::toFloatArray 获得
    //   cr/cg/cb/ca : 基础颜色（0~1）
    //   ambient    : 环境光强度（0~1，默认 0.15）
    // -----------------------------------------------------------------------
    void drawMesh(VkCommandBuffer cmd,
                  const std::string& id,
                  const float model[16],
                  float cr = 1.0f, float cg = 1.0f, float cb = 1.0f, float ca = 1.0f,
                  float ambient = 0.15f) {
        auto it = meshCache.find(id);
        if (it == meshCache.end()) return;

        MeshPushConstants pc{};
        memcpy(pc.model, model, 64);
        pc.baseColor[0] = cr;  pc.baseColor[1] = cg;
        pc.baseColor[2] = cb;  pc.baseColor[3] = ca;
        pc.ambientStr = ambient;
        pc.hasTexture = 0;
        vkCmdPushConstants(cmd, pipe->layout,
            VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT,
            0, sizeof(MeshPushConstants), &pc);

        it->second.bind(cmd);
        it->second.draw(cmd);
    }
};
