#pragma once
// ==========================================================================
// vk_renderer3d.h — 薄协调层，持有所有渲染子系统
//
// 子系统职责：
//   VkSceneSystem   — 不透明几何：mesh/svo/planet×11/ring/atmosphere/vegetation
//   VkSkySystem     — 天空盒（全屏三角形，几何 pass 末尾调用）
//   VkEffectsSystem — 透明/加法特效：exhaust / ribbon / billboard / lens_flare
//
// 每帧调用顺序（在 TAA geometry pass 内）：
//   beginFrame() → drawMesh/drawPlanet/drawRing/drawSVO/drawAtmosphere/drawVegetation × N
//   → drawSkybox()
//   → drawExhaust/drawRibbon/drawBillboard/drawLensFlare × N
// ==========================================================================

#include "systems/vk_scene.h"
#include "systems/vk_sky.h"
#include "systems/vk_effects.h"
#include "../../render/scene_snapshot.h"

struct VkRenderer3D {
    VkSceneSystem   scene;
    VkSkySystem     sky;
    VkEffectsSystem effects;

    // -----------------------------------------------------------------------
    bool init(VulkanContext& ctx, VkDescriptorManager& desc,
              VkFormat colorFmt, VkFormat depthFmt) {
        if (!scene.init(ctx, desc, colorFmt, depthFmt))   return false;
        if (!sky.init(ctx, desc, colorFmt, depthFmt))     return false;
        if (!effects.init(ctx, desc, colorFmt, depthFmt)) return false;
        effects.initRibbonRing(ctx);  // 环形 VBO 用于丝带渲染
        printf("[VkRenderer3D] All systems initialized\n");
        return true;
    }

    void shutdown(VulkanContext& ctx) {
        effects.shutdown(ctx.device);
        sky.shutdown(ctx.device);
        scene.shutdown(ctx);
    }

    // -----------------------------------------------------------------------
    // 网格管理（转发到 scene）
    bool registerMesh(VulkanContext& ctx, const std::string& id,
                      const void* vdata, size_t vsize,
                      const uint32_t* idata, uint32_t icount) {
        return scene.registerMesh(ctx, id, vdata, vsize, idata, icount);
    }
    bool hasMesh(const std::string& id) const { return scene.hasMesh(id); }

    // -----------------------------------------------------------------------
    // 帧开始（更新 FrameUBO，绑定网格管线）
    void beginFrame(VkCommandBuffer cmd, int frameIdx,
                    const float view[16], const float proj[16],
                    const float lightDir[3], const float viewPos[3], float time) {
        scene.beginFrame(cmd, frameIdx, view, proj, lightDir, viewPos, time);
        // 暂存 view/proj 供 drawSkybox 使用（避免调用方重复传参）
        memcpy(_view, view, 64);
        memcpy(_proj, proj, 64);
        _frameIdx = frameIdx;
    }

    // -----------------------------------------------------------------------
    // 不透明几何
    void drawMesh(VkCommandBuffer cmd, const std::string& id, const float model[16],
                  float r=1.f, float g=1.f, float b=1.f, float a=1.f, float ambient=0.15f) {
        scene.drawMesh(cmd, id, model, r, g, b, a, ambient);
    }

    // shaderKey: "earth"/"mars"/"venus"/"jupiter"/"gas_giant"/"barren"/"mercury"/"moon"/
    //            "saturn"/"uranus"/"neptune"，未知 key 降级为 drawMesh
    void drawPlanet(VkCommandBuffer cmd, const std::string& meshId, const float model[16],
                    float cx, float cy, float cz,
                    float r=1.f, float g=1.f, float b=1.f,
                    const std::string& shaderKey="earth") {
        scene.drawPlanet(cmd, meshId, model, cx, cy, cz, r, g, b, shaderKey);
    }

    // planetRadius: render units，传给 ring.frag 做归一化
    void drawRing(VkCommandBuffer cmd, const std::string& meshId, const float model[16],
                  float cx, float cy, float cz, float planetRadius) {
        scene.drawRing(cmd, meshId, model, cx, cy, cz, planetRadius);
    }

    // SVO — 与普通 mesh 相同顶点格式，走 svo.frag（per-vertex 颜色 + 三平面噪声）
    void drawSVO(VkCommandBuffer cmd, const std::string& meshId, const float model[16]) {
        scene.drawSVO(cmd, meshId, model);
    }

    // Atmosphere — 内建 unit sphere，着色器按 pc.outerRadius 缩放
    // 须在不透明几何之后、天空盒之前调用
    void drawAtmosphere(VkCommandBuffer cmd, const AtmoPushConstants& pc) {
        scene.drawAtmosphere(cmd, pc);
    }

    static bool isGasGiant(int atmoIdx) {
        return atmoIdx == 6 || atmoIdx == 7 || atmoIdx == 8 || atmoIdx == 9;
    }

    // Vegetation — 顶点 VBO (stride=24) + 实例 VBO (stride=20)
    void drawVegetation(VkCommandBuffer cmd,
                        VkBuffer vertVbo, uint32_t vertCount,
                        VkBuffer instVbo, uint32_t instCount) {
        scene.drawVegetation(cmd, vertVbo, vertCount, instVbo, instCount);
    }

    // Terrain — Quadtree leaf patch（Phase B）
    void drawTerrainPatch(VkCommandBuffer cmd,
                          VkBuffer patchVbo, VkBuffer patchIbo, uint32_t indexCount,
                          const TerrainPushConstants& pc,
                          VkDescriptorSet texSet) {
        scene.drawTerrainPatch(cmd, patchVbo, patchIbo, indexCount, pc, texSet);
    }

    // -----------------------------------------------------------------------
    // 天空盒（beginFrame 之后、特效之前调用）
    // 使用稳定投影矩阵 + 零平移视图，匹配 OpenGL drawSkybox 的行为
    void drawSkybox(VkCommandBuffer cmd, float vibrancy = 1.0f, float aspect = 1.0f) {
        // 稳定投影（FOV 0.8, near 0.1, far 10.0，与 OpenGL 一致）
        float stableProj[16];
        _buildStableProj(stableProj, aspect);
        // 零平移视图
        float viewNoTrans[16];
        memcpy(viewNoTrans, _view, 64);
        viewNoTrans[12] = 0.f; viewNoTrans[13] = 0.f; viewNoTrans[14] = 0.f;
        sky.draw(cmd, _frameIdx, viewNoTrans, stableProj, vibrancy);
    }

    // -----------------------------------------------------------------------
    // 特效

    // 排气羽流 — 使用内建 unit cube VBO+IBO（pos-only stride=12）
    // 调用方只需提供 model 矩阵（TRS：位置=喷管中点，旋转=喷管朝向，缩放=plume_dia/len/dia）
    void drawExhaust(VkCommandBuffer cmd, const float model[16],
                     float throttle, float expansion, float groundDist, float plumeLen) {
        if (scene.exhaustCubeVbo == VK_NULL_HANDLE) return;
        effects.drawExhaust(cmd, _frameIdx,
                            scene.exhaustCubeVbo, scene.exhaustCubeIbo, scene.exhaustCubeIcount,
                            model, throttle, expansion, groundDist, plumeLen);
    }

    // 保留旧接口以兼容任意外部 VBO 的调用（如 SVO 动态排气，暂未使用）
    void drawExhaustVBO(VkCommandBuffer cmd,
                        VkBuffer vbo, VkBuffer ibo, uint32_t icount,
                        const float model[16],
                        float throttle, float expansion, float groundDist, float plumeLen) {
        effects.drawExhaust(cmd, _frameIdx, vbo, ibo, icount,
                            model, throttle, expansion, groundDist, plumeLen);
    }

    void drawRibbon(VkCommandBuffer cmd,
                    VkBuffer vbo, uint32_t vertCount, const float model[16]) {
        effects.drawRibbon(cmd, _frameIdx, vbo, vertCount, model);
    }

    void drawBillboard(VkCommandBuffer cmd, VkBuffer vbo,
                       float cx, float cy, float cz, float w, float h,
                       float r, float g, float b, float a) {
        effects.drawBillboard(cmd, _frameIdx, vbo, cx, cy, cz, w, h, r, g, b, a);
    }

    void drawLensFlare(VkCommandBuffer cmd, VkBuffer vbo,
                       float sunSX, float sunSY, float aspect, float intensity,
                       float scaleX, float scaleY, float offX, float offY,
                       float r, float g, float b, float a, int shapeType) {
        effects.drawLensFlare(cmd, vbo, sunSX, sunSY, aspect, intensity,
                              scaleX, scaleY, offX, offY, r, g, b, a, shapeType);
    }

    // ========================================================================
    // render(snapshot) — 从 SceneSnapshot 完整渲染一帧的所有几何
    //
    // 调用方需已设置好 TAA geometry pass（beginGeometryPass + setViewportScissor）
    // 此方法只负责提交 draw call，不管理 render pass / swapchain
    // ========================================================================
    void render(VkCommandBuffer cmd, int frameIdx, const SceneSnapshot& snap) {
        (void)frameIdx; // 当前未使用（如有需要可用于 per-frame noise jitter）

        // ---- 0. 太阳物理球体 (Block E, 全景模式) ----
        if (snap.drawSunBody && hasMesh("earth")) {
            drawMesh(cmd, "earth", snap.sunBody.model, 1.0f, 0.95f, 0.9f, 1.0f, 2.0f);
        }

        // ---- 1. 帧开始（更新 UBO） ----
        beginFrame(cmd, snap.frameIndex % 2,
                   snap.view, snap.proj,
                   snap.lightDir, snap.camPos, snap.time);

        // ---- 2. 行星表面（不透明，共用 "earth" unit sphere mesh） ----
        for (const auto& cd : snap.celestials) {
            drawPlanet(cmd, "earth", cd.model,
                       cd.center[0], cd.center[1], cd.center[2],
                       cd.r, cd.g, cd.b, cd.shaderKey);

            // 土星环
            if (cd.hasRing && hasMesh("ring")) {
                drawRing(cmd, "ring", cd.model,
                         cd.center[0], cd.center[1], cd.center[2],
                         cd.radius);
            }
        }

        // ---- 3. 火箭（全零件渲染，先清深度确保始终在最前层） ----
        {
            VkClearAttachment clearAttach{};
            clearAttach.aspectMask = VK_IMAGE_ASPECT_DEPTH_BIT;
            clearAttach.clearValue.depthStencil.depth = 1.0f;
            VkClearRect cr{ {{0,0},{0,0}}, 1, 1 }; // full extent — caller sets viewport/scissor
            vkCmdClearAttachments(cmd, 1, &clearAttach, 1, &cr);
        }
        if (snap.rocketParts.empty()) {
            drawMesh(cmd, "rocketBody", snap.rocketBodyModel, 0.92f, 0.92f, 0.92f);
            drawMesh(cmd, "rocketNose", snap.rocketNoseModel, 0.88f, 0.88f, 0.88f);
        } else {
            for (const auto& rp : snap.rocketParts) {
                const char* mid = rp.meshType == 1 ? "rocketNose" :
                                  rp.meshType == 2 ? "rocketBox" : "rocketBody";
                drawMesh(cmd, mid, rp.model, rp.r, rp.g, rp.b);
            }
        }

        // ---- 4. 大气层 ----
        for (const auto& cd : snap.celestials) {
            if (cd.atmoIdx == 0) continue;
            float sunVis = snap.dayBlend;  // 使用预计算的 day_blend（含行星遮挡）

            AtmoPushConstants apc{};
            apc.planetCenter[0] = cd.center[0];
            apc.planetCenter[1] = cd.center[1];
            apc.planetCenter[2] = cd.center[2];
            apc.innerRadius    = cd.radius * 0.9995f;  // 匹配 OpenGL 的 tighter shadow sphere
            apc.outerRadius    = cd.radius + 160.0f;   // 所有天体统一 160km 大气高度
            apc.surfaceRadius  = cd.radius;
            apc.planetIdx      = cd.atmoIdx;
            apc.sunVisibility  = sunVis;
            apc.ringInner      = cd.hasRing ? cd.radius * 1.11f : 0.f;
            apc.ringOuter      = cd.hasRing ? cd.radius * 2.35f : 0.f;
            apc.frameIndex     = snap.frameIndex;
            apc.tuneMinAlt     = 0.f;
            apc.tuneMaxAlt     = 25.f;
            apc.tuneExtinction = 1.f;
            apc.showClouds     = cd.showClouds ? 1.f : 0.f;

            drawAtmosphere(cmd, apc);
        }

        // ---- 5. 天空盒 ----
        drawSkybox(cmd, snap.skyVibrancy, snap.aspect);

        // ---- 6. 排气羽流（加法混合） ----
        for (const auto& ed : snap.plumes) {
            drawExhaust(cmd, ed.model, ed.throttle, ed.expansion,
                        ed.groundDist, ed.plumeLen);
        }

        // ---- 7. 镜头光晕（含遮挡测试，匹配 OpenGL drawSunAndFlare） ----
        if (fabsf(snap.sunScreen[0]) < 3.5f && fabsf(snap.sunScreen[1]) < 3.5f
            && scene.billboardQuadVbo != VK_NULL_HANDLE) {
            // --- Ray-sphere occlusion test ---
            float occFade = 1.f;
            for (const auto& oc : snap.occluders) {
                Vec3 ocPos(oc.cx, oc.cy, oc.cz);
                Vec3 toSun(snap.sunWorldPos[0]-snap.camPos[0],
                           snap.sunWorldPos[1]-snap.camPos[1],
                           snap.sunWorldPos[2]-snap.camPos[2]);
                float sunDist = sqrtf(toSun.x*toSun.x+toSun.y*toSun.y+toSun.z*toSun.z);
                if (sunDist<1e-6f) continue;
                Vec3 rayDir = toSun * (1.f/sunDist);
                Vec3 ocv = Vec3(snap.camPos[0],snap.camPos[1],snap.camPos[2]) - ocPos;
                float b = 2.f * ocv.dot(rayDir);
                float c = ocv.dot(ocv) - oc.radius*oc.radius;
                float disc = b*b - 4.f*c;
                float tClosest = -ocv.dot(rayDir);
                if (tClosest >= 0.f) {
                    if (disc > 0.f) {
                        float t1 = tClosest - sqrtf(disc)*0.5f;
                        if (t1 > 0.f && t1 < sunDist) { occFade = 0.f; break; }
                    } else {
                        float passDist = ocv.cross(rayDir).length();
                        float distToLimb = passDist - oc.radius;
                        if (distToLimb >= 0.f && distToLimb < oc.radius*0.05f)
                            occFade *= distToLimb / (oc.radius*0.05f);
                    }
                }
                if (occFade <= 0.01f) break;
            }
            if (occFade > 0.01f) {
                float sx=snap.sunScreen[0], sy=snap.sunScreen[1], asp=snap.aspect, si=snap.sunIntensity*occFade;
                // 距离缩放因子（匹配 OpenGL）
                float currentDist = sqrtf(
                    (snap.sunWorldPos[0]-snap.camPos[0])*(snap.sunWorldPos[0]-snap.camPos[0])+
                    (snap.sunWorldPos[1]-snap.camPos[1])*(snap.sunWorldPos[1]-snap.camPos[1])+
                    (snap.sunWorldPos[2]-snap.camPos[2])*(snap.sunWorldPos[2]-snap.camPos[2]));
                float distFactor = 149597870.f / fmaxf(1.f, currentDist);
                float iScale = fminf(fmaxf(powf(distFactor, 1.25f), 0.15f), 8.f);
                float sScale = fminf(fmaxf(distFactor, 0.2f), 10.f);
                auto df = [&](int shape, float scX, float scY, float offM, float cr, float cg, float cb, float a) {
                    effects.drawLensFlare(cmd, scene.billboardQuadVbo, sx, sy, asp, si*a*iScale,
                        scX*sScale, scY*sScale, sx*offM, sy*offM, cr, cg, cb, 1, shape);
                };
                df(0, 0.12f,0.12f, 1.0f,  1.0f,0.98f,0.95f, 2.5f); // Hot core
                df(0, 0.30f,0.30f, 1.0f,  1.0f,0.92f,0.75f, 0.8f); // Warm halo
                df(0, 0.65f,0.65f, 1.0f,  0.9f,0.75f,0.55f, 0.2f); // Faint bloom
                df(1, 3.0f,0.025f,1.0f,  0.7f,0.8f,1.0f,  0.35f);  // Anamorphic blue
                df(1, 1.5f,0.01f, 1.0f,  1.0f,0.95f,0.9f,  0.5f);  // White core streak
                df(2, 0.08f,0.08f,-0.35f,0.7f,0.85f,1.0f, 0.08f);   // Ghost 1
                df(2, 0.12f,0.12f,-0.65f,1.0f,0.8f,0.6f,  0.05f);   // Ghost 2
                df(2, 0.05f,0.05f,-1.0f, 0.6f,0.7f,1.0f,  0.06f);   // Ghost 3
            }
        }

        // ---- 8. 发射台 (Block C) ----
        if (snap.hasLaunchPad && hasMesh("rocketBox")) {
            drawMesh(cmd, "rocketBox", snap.launchPad.model, 0.6f, 0.6f, 0.65f);
        }

        // ---- 9. 轨道丝带 (Block A) ----
        for (const auto& rib : snap.ribbons) {
            effects.drawRibbonData(cmd, snap.frameIndex % 2,
                rib.points, rib.colors, rib.width,
                Vec3(snap.camPos[0], snap.camPos[1], snap.camPos[2]));
        }

        // ---- 10. 变轨节点广告牌 (Block A) ----
        for (const auto& bd : snap.billboards) {
            if (scene.billboardQuadVbo != VK_NULL_HANDLE) {
                drawBillboard(cmd, scene.billboardQuadVbo,
                    bd.pos[0], bd.pos[1], bd.pos[2],
                    bd.size, bd.size, bd.r, bd.g, bd.b, bd.a);
            }
        }

        // ---- 11. SVO + Terrain (Block D) ----
        for (const auto& sc : snap.svoChunks) {
            drawSVO(cmd, "svo_chunk", sc.svoMat);
        }
        // Terrain patches extracted, rendering deferred (needs TerrainData UBO descriptor)
        (void)snap.terrainPatches;
    }

private:
    float _view[16]{};
    float _proj[16]{};
    int   _frameIdx = 0;

    // 构建 OpenGL 兼容的稳定投影矩阵（列主序）
    static void _buildStableProj(float out[16], float aspect) {
        float fov = 0.8f;  // 与 OpenGL Mat4::perspective(0.8f, ...) 一致
        float n = 0.1f, f = 10.0f;
        float t = n * tanf(fov * 0.5f);
        float b = -t, r = t * aspect, l = -r;
        memset(out, 0, 64);
        out[0] = 2.f*n/(r-l);
        out[5] = 2.f*n/(t-b);
        out[8] = (r+l)/(r-l);
        out[9] = (t+b)/(t-b);
        out[10] = -(f+n)/(f-n);
        out[11] = -1.f;
        out[14] = -2.f*f*n/(f-n);
    }
};
