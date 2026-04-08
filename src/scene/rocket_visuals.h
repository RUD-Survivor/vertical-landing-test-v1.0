#pragma once
#include "core/rocket_state.h"
#include "render/renderer3d.h"
#include "simulation/rocket_builder.h"
#include "render/part_renderer.h"
#include "camera/camera_director.h"

/**
 * RocketVisuals - 火箭视觉表现层
 * =========================================================================
 * 从 FlightScene 中抽离出的专注于微观火箭本体的渲染逻辑。
 *
 * 职责：
 *  1. 设置 PASS 2: MICRO FOREGROUND 的近景相机矩阵 (microProjMat)
 *  2. 深度清理 (保留在太空中与地球的正确穿模，清理近景遮挡)
 *  3. 基于当前级的火箭组装体 (Assembly) 进行逐零件 (Per-part) 的渲染涂装
 *  4. 正确处理火箭构型的多维对称 (Symmetry) 操作
 */
class RocketVisuals {
public:
    void render(Renderer3D* r3d, const CameraDirector& cam, float cam_dist, float rh, float aspect,
                const RocketState& rocket_state, const RocketConfig& rocket_config, const RocketAssembly& assembly,
                const Vec3& renderRocketBase, const Quat& rocketQuat, double ws_d,
                Mesh& rocketBody, Mesh& rocketNose, Mesh& rocketBox,
                const Mat4& viewMat, const Vec3& camEye_rel)
    {
        // 微观近景火箭专用的相机矩阵 (极近裁剪面，用于精确绘制 40米的火箭)
        if (cam.mode != 2) {
            // 在近景模式下，清空深度缓存，将火箭置于绝对顶层，杜绝共用一套深度衰减。这里有点问题，被地形遮挡也能显示
            // 全景模式下不清空，保留真实的物理穿模(躲在地球后面会被遮挡)的正确视角。
            glClear(GL_DEPTH_BUFFER_BIT);
        }
        float micro_near = fmaxf(rh * 0.05f, cam_dist * 0.002f);
        float micro_far = fmaxf(cam_dist * 10.0f, 15000.0f);
        Mat4 microProjMat = Mat4::perspective(0.8f, aspect, micro_near, micro_far);
        r3d->beginFrame(viewMat, microProjMat, camEye_rel);

        // ===== 火箭涂装 (Assembly-based per-part rendering) =====
        Vec3 engNozzlePos = renderRocketBase; // Fallback to base
        {
            int render_start = 0;
            if (rocket_state.current_stage < (int)rocket_config.stage_configs.size()) {
                render_start = rocket_config.stage_configs[rocket_state.current_stage].part_start_index;
            }
            // Find lowest active engine for flame positioning (保留这段可能的遗留逻辑计算)
            float min_ey = 1e10f;
            for (int pi = render_start; pi < (int)assembly.parts.size(); pi++) {
                const PlacedPart& pp = assembly.parts[pi];
                if (PART_CATALOG[pp.def_id].category == CAT_ENGINE) {
                    if (pp.pos.y < min_ey) {
                        min_ey = pp.pos.y;
                        engNozzlePos = renderRocketBase + rocketQuat.rotate(pp.pos * (float)ws_d);
                    }
                }
            }
            // 正式渲染每个零件
            for (int pi = render_start; pi < (int)assembly.parts.size(); pi++) {
                const PlacedPart& pp = assembly.parts[pi];
                const PartDef& def = PART_CATALOG[pp.def_id];
                for (int s = 0; s < pp.symmetry; s++) {
                    float symAngle = (s * 2.0f * 3.14159f) / pp.symmetry;
                    Vec3 localPos = pp.pos;
                    if (pp.symmetry > 1) {
                        float dist = sqrtf(pp.pos.x * pp.pos.x + pp.pos.z * pp.pos.z);
                        if (dist > 0.01f) {
                            float curAngle = atan2f(pp.pos.z, pp.pos.x);
                            localPos.x = cosf(curAngle + symAngle) * dist;
                            localPos.z = sinf(curAngle + symAngle) * dist;
                        }
                    }
                    Vec3 partWorldPos = renderRocketBase + rocketQuat.rotate(localPos * (float)ws_d);
                    Quat partWorldRot = rocketQuat * pp.rot * Quat::fromAxisAngle(Vec3(0, 1, 0), symAngle);
                    
                    PartRenderer::drawPartWithSymmetry(r3d, PART_CATALOG[pp.def_id], partWorldPos, partWorldRot, 
                                                      rocketBody, rocketNose, rocketBox,
                                                      (float)ws_d, false, false, 1.0f, 1);
                }
            }
        }
    }
};
