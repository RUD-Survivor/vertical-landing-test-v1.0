#pragma once
// ==========================================================================
// vk_sky.h — 天空盒渲染子系统 + 程序化星空调参
// 全屏三角形（无顶点缓冲），几何 pass 早期绘制。
// ==========================================================================

#include "../vk_context.h"
#include "../vk_descriptors.h"
#include "../vk_pipeline.h"

#include <cstring>
#include <cmath>

// 程序化星空 / 银河调参（imgui_sky_tuner.h，F5）
struct SkyTuneParams {
    float exposure          = 0.28f;
    float starDensity       = 1.5f;
    float starBrightness    = 0.18f;
    float starSize          = 0.25f;
    float glowStrength      = 0.45f;
    float milkyWayIntensity = 1.4f;
    float milkyWayWidth     = 0.5f;
    float dustStrength      = 1.35f;
    float galacticCoreBoost = 1.2f;
    float colorVariety      = 1.0f;
    float bgLevel           = 1.0f;
    float galaxyYaw         = 0.55f;  // rad — 银道朝向
    float galaxyPitch       = 1.05f;  // rad — ~60° 倾角观感
};

struct VkSkySystem {
    VkSkyboxPipeline     pipe;
    VkDescriptorManager* desc = nullptr;
    SkyTuneParams        tune;

    // -----------------------------------------------------------------------
    bool init(VulkanContext& ctx, VkDescriptorManager& d,
              VkFormat colorFmt, VkFormat depthFmt) {
        desc = &d;
        if (!pipe.init(ctx, d, colorFmt, depthFmt,
                       "src/render/shaders/spirv/skybox.vert.spv",
                       "src/render/shaders/spirv/skybox.frag.spv")) {
            fprintf(stderr, "[VkSky] Failed: skybox pipeline\n"); return false;
        }
        printf("[VkSky] Initialized\n");
        return true;
    }

    void shutdown(VkDevice d) { pipe.shutdown(d); }

    // -----------------------------------------------------------------------
    // 绘制天空盒
    //   view/proj : 列主序 float[16]
    //   vibrancy  : 场景昼夜衰减（1.0 = 正常；白天由 flight_scene 压低）
    void draw(VkCommandBuffer cmd, int frameIdx,
              const float view[16], const float proj[16],
              float vibrancy = 1.0f) {
        if (pipe.pipeline == VK_NULL_HANDLE) return;
        float vp[16], invVP[16];
        mul4x4(proj, view, vp);
        if (!inv4x4(vp, invVP)) return;

        pipe.bind(cmd);
        vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS,
            pipe.layout, 0, 1, &desc->set0[frameIdx], 0, nullptr);

        SkyboxPushConstants pc{};
        memcpy(pc.invViewProj, invVP, 64);
        pc.skyVibrancy       = vibrancy;
        pc.exposure          = tune.exposure;
        pc.starDensity       = tune.starDensity;
        pc.starBrightness    = tune.starBrightness;
        pc.starSize          = tune.starSize;
        pc.glowStrength      = tune.glowStrength;
        pc.milkyWayIntensity = tune.milkyWayIntensity;
        pc.milkyWayWidth     = tune.milkyWayWidth;
        pc.dustStrength      = tune.dustStrength;
        pc.galacticCoreBoost = tune.galacticCoreBoost;
        pc.colorVariety      = tune.colorVariety;
        pc.bgLevel           = tune.bgLevel;
        pc.galaxyYaw         = tune.galaxyYaw;
        pc.galaxyPitch       = tune.galaxyPitch;
        vkCmdPushConstants(cmd, pipe.layout,
            VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT,
            0, sizeof(SkyboxPushConstants), &pc);
        vkCmdDraw(cmd, 3, 1, 0, 0);
    }

private:
    static void mul4x4(const float A[16], const float B[16], float C[16]) {
        for (int r = 0; r < 4; r++)
            for (int c = 0; c < 4; c++) {
                float s = 0.f;
                for (int k = 0; k < 4; k++) s += A[r + k*4] * B[k + c*4];
                C[r + c*4] = s;
            }
    }

    static bool inv4x4(const float M[16], float inv[16]) {
        float s0=M[0]*M[5]-M[4]*M[1], s1=M[0]*M[9]-M[8]*M[1], s2=M[0]*M[13]-M[12]*M[1];
        float s3=M[4]*M[9]-M[8]*M[5], s4=M[4]*M[13]-M[12]*M[5], s5=M[8]*M[13]-M[12]*M[9];
        float c5=M[10]*M[15]-M[14]*M[11], c4=M[6]*M[15]-M[14]*M[7], c3=M[6]*M[11]-M[10]*M[7];
        float c2=M[2]*M[15]-M[14]*M[3], c1=M[2]*M[11]-M[10]*M[3], c0=M[2]*M[7]-M[6]*M[3];
        float det = s0*c5-s1*c4+s2*c3+s3*c2-s4*c1+s5*c0;
        if (fabsf(det) < 1e-10f) return false;
        float id = 1.f / det;
        inv[0] =( M[5]*c5-M[9]*c4+M[13]*c3)*id; inv[4] =(-M[4]*c5+M[8]*c4-M[12]*c3)*id;
        inv[8] =( M[7]*s5-M[11]*s4+M[15]*s3)*id; inv[12]=(-M[6]*s5+M[10]*s4-M[14]*s3)*id;
        inv[1] =(-M[1]*c5+M[9]*c2-M[13]*c1)*id; inv[5] =( M[0]*c5-M[8]*c2+M[12]*c1)*id;
        inv[9] =(-M[3]*s5+M[11]*s2-M[15]*s1)*id; inv[13]=( M[2]*s5-M[10]*s2+M[14]*s1)*id;
        inv[2] =( M[1]*c4-M[5]*c2+M[13]*c0)*id; inv[6] =(-M[0]*c4+M[4]*c2-M[12]*c0)*id;
        inv[10]=( M[3]*s4-M[7]*s2+M[15]*s0)*id; inv[14]=(-M[2]*s4+M[6]*s2-M[14]*s0)*id;
        inv[3] =(-M[1]*c3+M[5]*c1-M[9]*c0 )*id; inv[7] =( M[0]*c3-M[4]*c1+M[8]*c0 )*id;
        inv[11]=(-M[3]*s3+M[7]*s1-M[11]*s0)*id; inv[15]=( M[2]*s3-M[6]*s1+M[10]*s0)*id;
        return true;
    }
};
