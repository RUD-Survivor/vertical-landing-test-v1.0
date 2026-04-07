#pragma once
#include "core/rocket_state.h"
#include "render/renderer3d.h"
#include "simulation/rocket_builder.h" // RocketAssembly, PlacedPart, PART_CATALOG, PartDef
#include "physics/physics_system.h"    // PhysicsSystem::get_air_density

/**
 * PlumeManager - 工业级 3D 体积尾焰 (Volumetric Raymarched Plume) 管理器
 * =========================================================================
 * 从 FlightScene 中抽离出的独立引擎尾焰渲染子系统。
 *
 * 职责：
 *  1. 遍历当前级所有活跃引擎 (CAT_ENGINE)
 *  2. 处理对称放置 (symmetry) 的多喷管布局
 *  3. 计算每个喷管的羽流几何参数：
 *     - 推力缩放 (thrust_scale)
 *     - 大气膨胀系数 (expansion)
 *     - 羽流长度/直径 (plume_len / plume_dia)
 *     - 地面接触飞溅效应 (splash_factor)
 *  4. 调用 Renderer3D::drawExhaustVolumetric() 进行体积光线步进渲染
 */
class PlumeManager {
public:
    /**
     * 渲染所有活跃引擎的尾焰
     *
     * @param rocket_state   火箭当前物理状态 (推力、高度、当前级等)
     * @param rocket_config  火箭配置 (级起始索引)
     * @param control_input  控制输入 (油门)
     * @param assembly       火箭组装体 (零件列表)
     * @param r3d            3D 渲染器
     * @param rocketBox      用于羽流几何的立方体 Mesh
     * @param rocketQuat     火箭姿态四元数
     * @param renderRocketBase 火箭相对渲染坐标
     * @param ws_d           世界缩放因子 (km -> 渲染单位)
     */
    void render(RocketState& rocket_state, RocketConfig& rocket_config, ControlInput& control_input,
                const RocketAssembly& assembly, Renderer3D* r3d, const Mesh& rocketBox,
                const Quat& rocketQuat, const Vec3& renderRocketBase, double ws_d)
    {
        if (rocket_state.thrust_power < 0.01) return;

        float thrust = (float)control_input.throttle;
        float expansion = (float)fmax(0.0, 1.0 - PhysicsSystem::get_air_density(rocket_state.altitude) / 1.225);
        float thrust_scale = 0.25f + 0.75f * powf(thrust, 1.5f);

        int render_start = 0;
        if (rocket_state.current_stage < (int)rocket_config.stage_configs.size()) {
            render_start = rocket_config.stage_configs[rocket_state.current_stage].part_start_index;
        }

        // 遍历所有活跃引擎渲染羽流
        for (int pi = render_start; pi < (int)assembly.parts.size(); pi++) {
            const PlacedPart& pp = assembly.parts[pi];
            const PartDef& def = PART_CATALOG[pp.def_id];
            if (def.category == CAT_ENGINE) {
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
                    Vec3 nozzleWorldPos = renderRocketBase + rocketQuat.rotate(localPos * (float)ws_d);
                    Quat nozzleWorldRot = rocketQuat * pp.rot * Quat::fromAxisAngle(Vec3(0, 1, 0), symAngle);
                    Vec3 nozzleDir = nozzleWorldRot.rotate(Vec3(0, 1, 0)); // 引擎朝向
                    float engine_ph = def.height * (float)ws_d;
                    float plume_len = engine_ph * 4.2f * thrust_scale * (1.0f + expansion * 1.0f);
                    float plume_dia = def.diameter * (float)ws_d * 2.0f * (1.1f + expansion * 4.2f);
                    float groundDist = (float)rocket_state.altitude * (float)ws_d;
                    float ground_contact_depth = fmaxf(0.0f, plume_len - groundDist);
                    float splash_factor = ground_contact_depth / fmaxf(0.001f, plume_len);
                    plume_dia *= (1.0f + splash_factor * 8.0f);
                    Vec3 plumePos = nozzleWorldPos - nozzleDir * (plume_len * 0.5f);
                    Mat4 plumeMdl = Mat4::TRS(plumePos, nozzleWorldRot, Vec3(plume_dia, plume_len, plume_dia));
                    r3d->drawExhaustVolumetric(rocketBox, plumeMdl, thrust, expansion, (float)glfwGetTime(), groundDist, plume_len);
                }
            }
        }
    }
};
