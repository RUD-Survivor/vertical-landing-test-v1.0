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
#include "vk_cloud_system.h"
#include "../../render/scene_snapshot.h"

struct VkRenderer3D {
    VkSceneSystem   scene;
    VkSkySystem     sky;
    VkEffectsSystem effects;
    VkCloudSystem   cloudSystem;
    AtmoTuneParams  atmoTune; // 大气重构调参（imgui_atmo_tuner.h），见 renderAtmoAfterGeometry

    // -----------------------------------------------------------------------
    bool init(VulkanContext& ctx, VkDescriptorManager& desc,
              VkFormat colorFmt, VkFormat depthFmt) {
        if (!scene.init(ctx, desc, colorFmt, depthFmt))   return false;
        if (!scene.initCloudTextures(ctx))                 return false;
        if (!sky.init(ctx, desc, colorFmt, depthFmt))     return false;
        if (!effects.init(ctx, desc, colorFmt, depthFmt)) return false;
        effects.initRibbonRing(ctx, 524288);  // 环形 VBO (512K verts) 用于丝带渲染，需足够大以容纳高级轨道预测+机动虚线
        printf("[VkRenderer3D] All systems initialized\n");
        return true;
    }

    void shutdown(VulkanContext& ctx) {
        cloudSystem.shutdown(ctx);
        effects.shutdownRibbonRing();
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

    // Cloud — legacy fullscreen frag path (disabled when cloudSystem.enabled)
    void drawCloud(VkCommandBuffer cmd, const AtmoPushConstants& pc, double simTime, float camAlt) {
        scene.drawCloud(cmd, pc, simTime, camAlt);
    }

    // 大气重构·壳内/壳外调度 — 必须在 TAA endGeometryPass() 之后、
    // renderFlowerCloudAfterGeometry() 之前调用（顺序见 main.cpp）。
    //
    // 为什么不能留在几何 pass 内：新的两条路径（atmo_inside.frag/atmo_shell.frag）
    // 都要采样场景深度做空气透视/深度测试，而深度图只有在几何 pass 结束、
    // endGeometryPass() 把它转成 SHADER_READ_ONLY_OPTIMAL 之后才能被当纹理采样——
    // 几何 pass 内它还是"正在写入的 attachment"，Vulkan 不允许同一张图在同一个
    // render pass 里既是 attachment 又被采样。所以这里单独开一个几何 pass 之后的
    // 小 render pass，颜色目标复用 TAA 的 HDR 颜色（LOAD，不清空，保留几何 pass
    // 已经画好的内容），不挂深度 attachment（深度测试改成在 shader 里手动用采样到
    // 的场景深度比较）。
    //
    // 必须排在体积云之前：云管线（renderFlowerCloudAfterGeometry）进来时要求
    // HDR 颜色已经是 SHADER_READ_ONLY_OPTIMAL（它自己再转成 GENERAL 做 compute
    // 读写，见 vk_cloud_system.h::renderPhase0 开头的 transitionImage），所以这里
    // 结束前必须转回同一个 layout，衔接上云管线的假设。
    void renderAtmoAfterGeometry(VulkanContext& ctx, VkCommandBuffer cmd, VkTAA& taa,
                                  const SceneSnapshot& snap, int frameSlot) {
        bool anyAtmo = false;
        for (const auto& cd : snap.celestials) { if (cd.atmoIdx != 0) { anyAtmo = true; break; } }
        if (!anyAtmo) return;

        VkImage     colorImg  = taa.colorImages[taa.currentIdx];
        VkImageView colorView = taa.colorViews[taa.currentIdx];

        // endGeometryPass() 已经把它转成 SHADER_READ_ONLY_OPTIMAL（供 TAA/云读取），
        // 这里要先转回 COLOR_ATTACHMENT_OPTIMAL 才能重新当渲染目标写入。
        transitionImage(cmd, colorImg,
            VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL,
            VK_PIPELINE_STAGE_2_FRAGMENT_SHADER_BIT, VK_ACCESS_2_SHADER_READ_BIT,
            VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT, VK_ACCESS_2_COLOR_ATTACHMENT_WRITE_BIT);

        VkRenderingAttachmentInfo colorAI{ VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO };
        colorAI.imageView   = colorView;
        colorAI.imageLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
        colorAI.loadOp      = VK_ATTACHMENT_LOAD_OP_LOAD;  // 保留几何 pass 已经画好的内容
        colorAI.storeOp     = VK_ATTACHMENT_STORE_OP_STORE;

        VkRenderingInfo ri{ VK_STRUCTURE_TYPE_RENDERING_INFO };
        ri.renderArea           = { {0, 0}, _extent };
        ri.layerCount           = 1;
        ri.colorAttachmentCount = 1;
        ri.pColorAttachments    = &colorAI;
        // 不挂深度 attachment，理由见函数顶部注释。

        vkCmdBeginRendering(cmd, &ri);
        {
            VkViewport vp{ 0.0f, 0.0f, (float)_extent.width, (float)_extent.height, 0.0f, 1.0f };
            vkCmdSetViewport(cmd, 0, 1, &vp);
            VkRect2D scissor{ {0, 0}, _extent };
            vkCmdSetScissor(cmd, 0, 1, &scissor);
        }

        for (const auto& cd : snap.celestials) {
            if (cd.atmoIdx == 0) continue;

            // 该星球"指向太阳"的方向（与几何 pass 里 updateLightDir 用的是同一套算法，
            // 但这里不改全局 light 方向状态——壳内/壳外 shader 直接把方向当参数传，
            // 不依赖 FrameUBO 里那个会被后续几何/特效覆盖的全局 lightDir）。
            float lightDirToSun[3] = {
                snap.sunWorldPos[0] - cd.center[0],
                snap.sunWorldPos[1] - cd.center[1],
                snap.sunWorldPos[2] - cd.center[2],
            };
            float len = sqrtf(lightDirToSun[0]*lightDirToSun[0] + lightDirToSun[1]*lightDirToSun[1] + lightDirToSun[2]*lightDirToSun[2]);
            if (len > 1e-4f) { lightDirToSun[0]/=len; lightDirToSun[1]/=len; lightDirToSun[2]/=len; }

            AtmoPushConstants apc{};
            apc.planetCenter[0] = cd.center[0];
            apc.planetCenter[1] = cd.center[1];
            apc.planetCenter[2] = cd.center[2];
            apc.innerRadius    = cd.radius;
            apc.outerRadius    = cd.radius + atmoTune.shellThicknessKm;
            apc.surfaceRadius  = cd.radius;
            apc.planetIdx      = cd.atmoIdx;
            apc.sunVisibility  = snap.dayBlend;
            apc.ringInner      = cd.hasRing ? cd.radius * 1.11f : 0.f;
            apc.ringOuter      = cd.hasRing ? cd.radius * 2.35f : 0.f;
            apc.frameIndex     = snap.frameIndex;

            PlanetScatteringCoeffs sc = getPlanetScatteringCoeffs(cd.atmoIdx);
            apc.rayleighCoeff[0]=sc.rayleighCoeff[0]; apc.rayleighCoeff[1]=sc.rayleighCoeff[1]; apc.rayleighCoeff[2]=sc.rayleighCoeff[2];
            apc.mieCoeff = sc.mieCoeff;
            apc.hRayleigh = sc.hRayleigh; apc.hMie = sc.hMie; apc.gMie = sc.gMie;
            apc.ozoneCoeff[0]=sc.ozoneCoeff[0]; apc.ozoneCoeff[1]=sc.ozoneCoeff[1]; apc.ozoneCoeff[2]=sc.ozoneCoeff[2];
            apc.ozoneCenter = sc.ozoneCenter; apc.ozoneWidth = sc.ozoneWidth;

            apc.spaceVisStart  = atmoTune.spaceVisStart;
            apc.spaceVisEnd    = atmoTune.spaceVisEnd;
            apc.limbBrightness = atmoTune.limbBrightness;
            apc.outerExposure  = atmoTune.outerExposure;
            apc.innerExposureNear = atmoTune.innerExposureNear;
            apc.innerExposureFar  = atmoTune.innerExposureFar;
            apc.limbSpaceStart  = atmoTune.limbSpaceStart;
            apc.limbSpaceEnd    = atmoTune.limbSpaceEnd;
            apc.limbInsideScale = atmoTune.limbInsideScale;
            apc.limbPower       = atmoTune.limbPower;

            // 已修复：太阳方向传真正"这颗星球→太阳"的方向（上面刚算好的
            // lightDirToSun），不再依赖 shader 里读的那个全局 frame.lightDir
            // UBO（几何 pass 循环结束后已经被重置成"火箭→太阳"，全景模式
            // 相机离火箭很远时和当前星球的太阳方向偏差很大）。
            apc.sunDir[0] = lightDirToSun[0];
            apc.sunDir[1] = lightDirToSun[1];
            apc.sunDir[2] = lightDirToSun[2];

            float camDx = snap.camPos[0]-cd.center[0], camDy = snap.camPos[1]-cd.center[1], camDz = snap.camPos[2]-cd.center[2];
            float camDist = sqrtf(camDx*camDx + camDy*camDy + camDz*camDz);
            bool  cameraInside = camDist < apc.outerRadius;

            if (cameraInside) {
                scene.drawAtmoInside(ctx, cmd, frameSlot, taa.depthView, cd.bodyIdx, apc, snap.camPos, lightDirToSun);
            } else {
                scene.drawAtmoShell(ctx, cmd, frameSlot, taa.depthView, cd.bodyIdx, apc, snap.camPos, lightDirToSun);
            }
        }

        vkCmdEndRendering(cmd);

        // 转回 SHADER_READ_ONLY_OPTIMAL：云管线期望进来时 HDR 颜色已经是这个 layout。
        transitionImage(cmd, colorImg,
            VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
            VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT, VK_ACCESS_2_COLOR_ATTACHMENT_WRITE_BIT,
            VK_PIPELINE_STAGE_2_FRAGMENT_SHADER_BIT, VK_ACCESS_2_SHADER_READ_BIT);
    }

    // Flower Phase0 — call after TAA endGeometryPass, before resolve
    void renderFlowerCloudAfterGeometry(VulkanContext& ctx, VkCommandBuffer cmd, VkTAA& taa,
                                        const SceneSnapshot& snap, int frameSlot) {
        bool showClouds = false;
        for (const auto& cd : snap.celestials) {
            if (cd.atmoIdx == 3 && cd.showClouds) { showClouds = true; break; }
        }
        cloudSystem.renderPhase0(ctx, cmd, taa, snap, frameSlot, showClouds);
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
                          const TerrainDataUBO& td,
                          VkDescriptorSet texSet) {
        scene.drawTerrainPatch(cmd, patchVbo, patchIbo, indexCount, pc, td, texSet);
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

    void drawBillboardOverlay(VkCommandBuffer cmd, VkBuffer vbo,
                              float cx, float cy, float cz, float w, float h,
                              float r, float g, float b, float a) {
        effects.drawBillboardOverlay(cmd, _frameIdx, vbo, cx, cy, cz, w, h, r, g, b, a);
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
    void render(VkCommandBuffer cmd, int frameIdx, const SceneSnapshot& snap, VkExtent2D extent = {1920, 1080}) {
        (void)frameIdx; // 当前未使用（如有需要可用于 per-frame noise jitter）
        _extent = extent;

        // ---- 1. 帧开始 ----
        beginFrame(cmd, snap.frameIndex % 2, snap.view, snap.proj, snap.lightDir, snap.camPos, snap.time);

        // ---- 星空（先于行星，匹配 OpenGL 顺序） ----
        drawSkybox(cmd, snap.skyVibrancy, snap.aspect);

        // ---- 2. 太阳 ----
        if (snap.drawSunBody && hasMesh("earth")) {
            drawMesh(cmd, "earth", snap.sunBody.model, 1.0f, 0.95f, 0.9f, 1.0f, 20.0f);
        }

        // ---- 2. 行星表面（不透明，共用 "earth" unit sphere mesh） ----
        // 逐行星更新光照方向（匹配 OpenGL per-planet lightDir）
        for (const auto& cd : snap.celestials) {
            // 当地形四叉树补丁存在时，跳过地球程序噪声球体（terrain 已覆盖）
            if (cd.bodyIdx == 3 && !snap.terrainPatches.empty()) continue;

            float lx = snap.sunWorldPos[0] - cd.center[0];
            float ly = snap.sunWorldPos[1] - cd.center[1];
            float lz = snap.sunWorldPos[2] - cd.center[2];
            scene.updateLightDir(snap.frameIndex % 2, lx, ly, lz);

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
        // 恢复全局光照方向（火箭/发射台等用）
        scene.updateLightDir(snap.frameIndex % 2,
            snap.lightDir[0], snap.lightDir[1], snap.lightDir[2]);

        // ---- 3. 地形（不透明，与行星表面同层，先于大气/轨道线） ----
        if (!snap.terrainPatches.empty()) {
            TerrainDataUBO planetTd{};
            planetTd.planetCenterRel[0] = snap.terrainPatches[0].planetCenterRel[0];
            planetTd.planetCenterRel[1] = snap.terrainPatches[0].planetCenterRel[1];
            planetTd.planetCenterRel[2] = snap.terrainPatches[0].planetCenterRel[2];
            planetTd.planetCenterRel[3] = 0.f;
            for (const auto& tp : snap.terrainPatches) {
                TerrainPushConstants tpc{};
                memcpy(tpc.model, tp.model, 64);
                tpc.planetRadius  = tp.planetRadius;
                tpc.maxElevation  = tp.maxElev;
                tpc.nodeLevel     = tp.nodeLevel;
                tpc.hasLocalHydro = 0;
                tpc.nodePos[0]    = tp.nodeCenter[0];
                tpc.nodePos[1]    = tp.nodeCenter[1];
                tpc.nodePos[2]    = tp.nodeCenter[2];
                tpc.nodeSide[0]   = tp.nodeSideA[0];
                tpc.nodeSide[1]   = tp.nodeSideA[1];
                tpc.nodeSide[2]   = tp.nodeSideA[2];
                tpc.nodeUp[0]     = tp.nodeSideB[0];
                tpc.nodeUp[1]     = tp.nodeSideB[1];
                tpc.nodeUp[2]     = tp.nodeSideB[2];
                drawTerrainPatch(cmd, scene.terrainPatchVbo, scene.terrainPatchIbo,
                    scene.terrainPatchIcount, tpc, planetTd, scene.terrainGlobalTexSet);
            }
        }

        // ---- 4. 体积云（地形之后、大气之前；预乘 alpha 混合）----
        // ---- 5. 大气层 ----
        for (const auto& cd : snap.celestials) {
            if (cd.atmoIdx == 0) continue;
            // 更新为该行星的光照方向（太阳→行星）
            float alx = snap.sunWorldPos[0] - cd.center[0];
            float aly = snap.sunWorldPos[1] - cd.center[1];
            float alz = snap.sunWorldPos[2] - cd.center[2];
            scene.updateLightDir(snap.frameIndex % 2, alx, aly, alz);
            float sunVis = snap.dayBlend;  // 使用预计算的 day_blend（含行星遮挡）

            AtmoPushConstants apc{};
            apc.planetCenter[0] = cd.center[0];
            apc.planetCenter[1] = cd.center[1];
            apc.planetCenter[2] = cd.center[2];
            apc.innerRadius    = cd.radius;
            apc.outerRadius    = cd.radius + 160.0f;
            apc.surfaceRadius  = cd.radius;
            apc.planetIdx      = cd.atmoIdx;
            apc.sunVisibility  = sunVis;
            apc.ringInner      = cd.hasRing ? cd.radius * 1.11f : 0.f;
            apc.ringOuter      = cd.hasRing ? cd.radius * 2.35f : 0.f;
            apc.frameIndex     = snap.frameIndex;
            apc.tuneMinAlt     = 0.f;     // atmosphere params (overridden for clouds later)
            apc.tuneMaxAlt     = 25.f;
            apc.tuneExtinction = 1.f;
            apc.showClouds     = cd.showClouds ? 1.f : 0.f;

            // 大气散射系数：唯一数据源 getPlanetScatteringCoeffs()（vk_atmosphere_lut.h），
            // 壳内/壳外新 shader（atmo_inside.frag/atmo_shell.frag）直接读这些字段，
            // 不再各自硬编码一份 setupPlanetProfile() 系数表。
            {
                PlanetScatteringCoeffs sc = getPlanetScatteringCoeffs(cd.atmoIdx);
                apc.rayleighCoeff[0]=sc.rayleighCoeff[0]; apc.rayleighCoeff[1]=sc.rayleighCoeff[1]; apc.rayleighCoeff[2]=sc.rayleighCoeff[2];
                apc.mieCoeff = sc.mieCoeff;
                apc.hRayleigh = sc.hRayleigh; apc.hMie = sc.hMie; apc.gMie = sc.gMie;
                apc.ozoneCoeff[0]=sc.ozoneCoeff[0]; apc.ozoneCoeff[1]=sc.ozoneCoeff[1]; apc.ozoneCoeff[2]=sc.ozoneCoeff[2];
                apc.ozoneCenter = sc.ozoneCenter; apc.ozoneWidth = sc.ozoneWidth;
            }

            // 大气散射：不再在这里画。旧版 drawAtmosphere()（全屏 raymarch，无深度采样）
            // 需要放在几何 pass 内才能用无深度测试的全屏三角形简单实现；新的壳内/壳外
            // 路径（atmo_inside.frag/atmo_shell.frag）需要采样场景深度做空气透视/深度
            // 测试，而深度只有在几何 pass 结束、TAA 转成只读纹理之后才能被采样——两者
            // 没法在同一个 render pass 里共存。改到 renderAtmoAfterGeometry()（本文件下方），
            // 由 main.cpp 在 endGeometryPass() 之后、renderFlowerCloudAfterGeometry() 之前调用。

            // 云：仅地球；flower Phase0 在 geometry pass 后 compute 合成
            if (cd.atmoIdx == 3 && cd.showClouds && !cloudSystem.enabled) {
                apc.tuneMinAlt     = 2.0f;
                apc.tuneMaxAlt     = 14.0f;
                apc.tuneExtinction = 1.2f;
                apc.showClouds     = 1.f;
                float dx = snap.camPos[0] - apc.planetCenter[0];
                float dy = snap.camPos[1] - apc.planetCenter[1];
                float dz = snap.camPos[2] - apc.planetCenter[2];
                float camAltKm = sqrtf(dx*dx+dy*dy+dz*dz) - apc.surfaceRadius;
                drawCloud(cmd, apc, (double)snap.time, camAltKm);
            }
        }
        // 恢复全局光照方向
        scene.updateLightDir(snap.frameIndex % 2,
            snap.lightDir[0], snap.lightDir[1], snap.lightDir[2]);

        // ---- 5. 火箭（云和大气之后渲染，确保火箭不被远处的云/大气遮挡）----
        if (snap.rocketParts.empty()) {
            drawMesh(cmd, "rocketBody", snap.rocketBodyModel, 0.92f, 0.92f, 0.92f);
            drawMesh(cmd, "rocketNose", snap.rocketNoseModel, 0.88f, 0.88f, 0.88f);
        } else {
            for (const auto& rp : snap.rocketParts) {
                if (!rp.meshId.empty() && hasMesh(rp.meshId)) {
                    drawMesh(cmd, rp.meshId, rp.model, rp.r, rp.g, rp.b);
                } else {
                    const char* mid = rp.meshType == 1 ? "rocketNose" :
                                      rp.meshType == 2 ? "rocketBox" : "rocketBody";
                    drawMesh(cmd, mid, rp.model, rp.r, rp.g, rp.b);
                }
            }
        }

        // ---- 排气羽流（加法混合，在火箭之后、光晕之前） ----
        for (const auto& ed : snap.plumes) {
            drawExhaust(cmd, ed.model, ed.throttle, ed.expansion, ed.groundDist, ed.plumeLen);
        }

        // ---- 6. 天空盒（跳过——debug 测试） ----
        // drawSkybox(cmd, snap.skyVibrancy, snap.aspect);

        // ---- 6. 镜头光晕（太阳光晕 + 辉光 + 条纹） ----
        if (scene.billboardQuadVbo != VK_NULL_HANDLE
            && snap.sunScreen[0] > -2.0f && snap.sunIntensity > 0.001f) {

            // 软件遮挡检测：匹配 OpenGL drawSunAndFlare 的渐变逻辑
            float occlusionFade = 1.0f;
            {
                float sdx = snap.sunWorldPos[0] - snap.camPos[0];
                float sdy = snap.sunWorldPos[1] - snap.camPos[1];
                float sdz = snap.sunWorldPos[2] - snap.camPos[2];
                float sunDist = sqrtf(sdx*sdx + sdy*sdy + sdz*sdz);
                float sunDx = sdx / sunDist;
                float sunDy = sdy / sunDist;
                float sunDz = sdz / sunDist;
                for (const auto& oc : snap.occluders) {
                    float Lx = oc.cx - snap.camPos[0];
                    float Ly = oc.cy - snap.camPos[1];
                    float Lz = oc.cz - snap.camPos[2];
                    float t_ca = Lx*sunDx + Ly*sunDy + Lz*sunDz;
                    if (t_ca < 0.0f) continue; // 遮挡体在相机后面
                    float d2 = (Lx*Lx + Ly*Ly + Lz*Lz) - t_ca*t_ca;
                    float r2 = oc.radius * oc.radius;
                    float localOccFade = 1.0f;
                    if (d2 < r2) {
                        // 射线命中球体 → 太阳在行星后面
                        float dt = sqrtf(r2 - d2);
                        float t_hit = t_ca - dt;
                        if (t_hit > 0.0f && t_hit < sunDist) {
                            localOccFade = 0.0f; // 完全被遮挡
                        }
                    } else {
                        // 射线擦过：计算距行星边缘的距离，薄上层大气渐变
                        float passDist = sqrtf(d2); // 射线到球心的最近距离
                        float distToLimb = passDist - oc.radius;
                        if (distToLimb >= 0.0f && distToLimb < oc.radius * 0.05f) {
                            localOccFade *= distToLimb / (oc.radius * 0.05f);
                        }
                    }
                    occlusionFade = fminf(occlusionFade, localOccFade);
                    if (occlusionFade <= 0.01f) break;
                }
            }
            if (occlusionFade <= 0.01f) goto skip_lens_flare;

            // 动态强度/缩放（匹配 OpenGL 距离衰减）
            float refDist = 149597.87f; // 1 AU in render units
            float sunDx = snap.sunWorldPos[0] - snap.camPos[0];
            float sunDy = snap.sunWorldPos[1] - snap.camPos[1];
            float sunDz = snap.sunWorldPos[2] - snap.camPos[2];
            float curDist = sqrtf(sunDx*sunDx + sunDy*sunDy + sunDz*sunDz);
            float distFactor = refDist / fmaxf(1.0f, curDist);
            float intensityScale = powf(distFactor, 1.25f) * occlusionFade;
            intensityScale = fminf(fmaxf(intensityScale, 0.15f), 8.0f);
            float sizeScale = fminf(fmaxf(distFactor, 0.2f), 10.0f);

            float sx = snap.sunScreen[0], sy = snap.sunScreen[1];
            float asp = snap.aspect;

            // 热白核心（太阳位置）
            drawLensFlare(cmd, scene.billboardQuadVbo, sx, sy, asp, 175.f * intensityScale,
                          0.10f*sizeScale, 0.10f*sizeScale, sx, sy,
                          1.0f, 0.98f, 0.95f, 1.0f, 0);
            // 暖色光晕
            drawLensFlare(cmd, scene.billboardQuadVbo, sx, sy, asp, 60.f * intensityScale,
                          0.22f*sizeScale, 0.22f*sizeScale, sx, sy,
                          1.0f, 0.92f, 0.75f, 1.0f, 0);
            // 淡暖色散开
            drawLensFlare(cmd, scene.billboardQuadVbo, sx, sy, asp, 18.f * intensityScale,
                          0.50f*sizeScale, 0.50f*sizeScale, sx, sy,
                          0.9f, 0.75f, 0.55f, 1.0f, 0);
            // 蓝色变形条纹（穿过太阳，窄条不长）
            drawLensFlare(cmd, scene.billboardQuadVbo, sx, sy, asp, 25.f * intensityScale,
                          0.8f*sizeScale, 0.012f*sizeScale, sx, sy,
                          0.7f, 0.8f, 1.0f, 1.0f, 1);
            // 白色核心条纹
            drawLensFlare(cmd, scene.billboardQuadVbo, sx, sy, asp, 35.f * intensityScale,
                          0.5f*sizeScale, 0.006f*sizeScale, sx, sy,
                          1.0f, 0.95f, 0.9f, 1.0f, 1);
            // 衍射条纹
            drawLensFlare(cmd, scene.billboardQuadVbo, sx, sy, asp, 12.f * intensityScale,
                          0.35f*sizeScale, 0.35f*sizeScale, sx, sy,
                          0.6f, 0.85f, 1.0f, 1.0f, 2);
            // 星芒
            drawLensFlare(cmd, scene.billboardQuadVbo, sx, sy, asp, 20.f * intensityScale,
                          0.40f*sizeScale, 0.40f*sizeScale, sx, sy,
                          1.0f, 0.97f, 0.9f, 1.0f, 3);
        }
        skip_lens_flare:;

        // ---- 8. 发射台 (Block C) ----
        if (snap.hasLaunchPad) {
            if (snap.launchPad.useObjMesh && hasMesh("launchPad")) {
                drawMesh(cmd, "launchPad", snap.launchPad.model, 1.0f, 1.0f, 1.0f);
            } else if (hasMesh("rocketBox")) {
                drawMesh(cmd, "rocketBox", snap.launchPad.model, 0.6f, 0.6f, 0.65f);
            }
        }

        // ---- 9. 轨道丝带 (Block A) ----
        effects.beginRibbonFrame();
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

        // ---- 11. SVO (Block D) ----
        for (const auto& sc : snap.svoChunks) {
            drawSVO(cmd, "svo_chunk", sc.svoMat);
        }
        // 植被实例渲染 — 从 snapshot 上传到 GPU
        if (snap.vegInstanceCount > 0 && scene.vegVertVbo != VK_NULL_HANDLE
            && scene.vegInstanceMapped) {
            size_t sz = snap.vegInstanceCount * 5 * sizeof(float);
            if (sz <= 4096 * 20) {
                memcpy(scene.vegInstanceMapped, snap.vegInstanceData.data(), sz);
                drawVegetation(cmd, scene.vegVertVbo, scene.vegVertCount,
                    scene.vegInstanceVbo, snap.vegInstanceCount);
            }
        }

    }

private:
    float      _view[16]{};
    float      _proj[16]{};
    int        _frameIdx = 0;
    VkExtent2D _extent   = {1920, 1080};

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
