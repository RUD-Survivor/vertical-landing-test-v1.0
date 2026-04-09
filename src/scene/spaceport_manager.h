#pragma once
#include "core/rocket_state.h"
#include "render/renderer3d.h"
#include "simulation/orbit_physics.h" // For SOLAR_SYSTEM celestial bodies
#include "render/part_renderer.h"     // For ModelLoader (if needed)

/**
 * SpaceportManager - 发射中心/发射台管理器
 * =========================================================================
 * 从 FlightScene 中抽离出的独立发射台渲染子系统。
 *
 * 职责：
 *  1. 加载和管理发射台的物理与贴图资产 (launch_pad.obj / .tga)
 *  2. 根据当前跟随的星体(地球等)自转与参数动态旋转发射场
 *  3. 给出发射台绝对的地表位置和基于镜头原点的相对漫游坐标
 *  4. 提供无模型资产情况下的回退(Fallback)堆叠几何体渲染
 */
class SpaceportManager {
public:
    Mesh launchPadMesh;
    Texture launchPadTexture;
    bool has_launch_pad = false;

    /**
     * 场景初始化时调用，加载发设台静态资源
     */
    void init() {
        launchPadMesh = ModelLoader::loadOBJ("assets/launch_pad.obj");
        launchPadTexture = Renderer3D::loadTGA("assets/launch_pad.tga");
        has_launch_pad = (launchPadMesh.indexCount > 0);
    }

    /**
     * 渲染发射台
     *
     * @param r3d               3D渲染器指针
     * @param rocket_state      火箭物理状态 (读取高度与起飞发射场坐标)
     * @param current_soi_index 当前所属星球SOI索引
     * @param ws_d              世界缩放因子
     * @param ro_x, ro_y, ro_z  相对渲染原点偏移
     * @param rocketBox         用于回退(Fallback)渲染的堆叠方形Mesh
     * @param rw_3d             渲染用火箭半径 (Fallback 缩放用)
     * @param rh                渲染用火箭高度 (Fallback 缩放用)
     */
    void render(Renderer3D* r3d, entt::registry& registry, entt::entity entity, int current_soi_index,
                double ws_d, double ro_x, double ro_y, double ro_z,
                const Mesh& rocketBox, float rw_3d, float rh)
    {
        auto& tele = registry.get<TelemetryComponent>(entity);
        auto& trans = registry.get<TransformComponent>(entity);
        // 高度大于12000米则从视野中剔除发射台
        if (tele.altitude >= 12000.0) return;

        CelestialBody& body = SOLAR_SYSTEM[current_soi_index];
        // Launch pad should rotate with the body (Axial Tilt + Rotation)
        double theta = body.prime_meridian_epoch + (tele.sim_time * 2.0 * PI / body.rotation_period);
        Quat rot = Quat::fromAxisAngle(Vec3(0, 0, 1), (float)theta);
        Quat tilt = Quat::fromAxisAngle(Vec3(1, 0, 0), (float)body.axial_tilt);
        Quat full_rot = tilt * rot;
        
        // Use FIXED launch site coordinates instead of dynamic ground track
        Vec3 s_pos((float)trans.launch_site_px, (float)trans.launch_site_py, (float)trans.launch_site_pz);
        
        // Normalized local up vector
        Vec3 localUp = s_pos.normalized();
        // Position on surface (exactly at terrain elevation)
        Vec3 s_surf = localUp * s_pos.length();
        // Rotate to inertial frame
        Vec3 i_surf = full_rot.rotate(s_surf);
        Vec3 i_up = full_rot.rotate(localUp);
        
        Vec3 padCenter(
            (float)((body.px + (double)i_surf.x) * ws_d - ro_x),
            (float)((body.py + (double)i_surf.y) * ws_d - ro_y),
            (float)((body.pz + (double)i_surf.z) * ws_d - ro_z)
        );
        
        // Calculate local orientation for the pad
        Vec3 padUp = i_up;
        // Calculate a local right/forward to orient the pad
        Vec3 defaultRight(1, 0, 0);
        if (fabs(padUp.dot(defaultRight)) > 0.9f) defaultRight = Vec3(0, 1, 0);
        Vec3 padRight = padUp.cross(defaultRight).normalized();
        Vec3 padForward = padUp.cross(padRight).normalized();
        
        // Construct rotation matrix for the pad to face "up"
        Mat4 padRot;
        padRot.m[0] = padRight.x; padRot.m[1] = padRight.y; padRot.m[2] = padRight.z; padRot.m[3] = 0;
        padRot.m[4] = padUp.x;    padRot.m[5] = padUp.y;    padRot.m[6] = padUp.z;    padRot.m[7] = 0;
        padRot.m[8] = padForward.x; padRot.m[9] = padForward.y; padRot.m[10] = padForward.z; padRot.m[11] = 0;
        padRot.m[12] = 0;         padRot.m[13] = 0;         padRot.m[14] = 0;         padRot.m[15] = 1;
        
        if (has_launch_pad) {
            float pad_scale = (float)ws_d; // 1:1 Physical Scale
            Mat4 padModel = Mat4::scale(Vec3(pad_scale, pad_scale, pad_scale));
            padModel = padRot * padModel; // Orient pad
            Vec3 correctedPos = padCenter - padUp * (8.0f * (float)ws_d);
            padModel = Mat4::translate(correctedPos) * padModel;
            
            bool hasTex = (launchPadTexture.id != 0);
            if (hasTex) {
                launchPadTexture.bind(0);
                glUniform1i(r3d->u_hasTexture, 1);
                glUniform1i(r3d->u_sampler, 0);
            }
            
            r3d->drawMesh(launchPadMesh, padModel, 1.0f, 1.0f, 1.0f, 1.0f, 0.2f);
            
            if (hasTex) glUniform1i(r3d->u_hasTexture, 0);
        }
        else {
            float pad_w = rw_3d * 20.0f;
            float pad_h = rh * 0.4f; // Increased pad height to bury it deeper
            Mat4 baseMdl = Mat4::scale(Vec3(pad_w, pad_h, pad_w));
            baseMdl = padRot * baseMdl;
            // Bury the bottom half of the pad base into the planet to eliminate gaps
            baseMdl = Mat4::translate(padCenter - padUp * (pad_h * 0.45f)) * baseMdl;
            r3d->drawMesh(rocketBox, baseMdl, 0.4f, 0.4f, 0.4f, 1.0f, 0.1f);
            
            float tower_h = rh * 1.5f;
            float tower_w = rw_3d * 3.0f;
            Vec3 towerCenter = padCenter + padRight * (rw_3d * 4.0f) + padUp * (tower_h * 0.5f - pad_h * 0.5f);
            Mat4 towerMdl = Mat4::scale(Vec3(tower_w, tower_h, tower_w));
            towerMdl = padRot * towerMdl;
            towerMdl = Mat4::translate(towerCenter) * towerMdl;
            r3d->drawMesh(rocketBox, towerMdl, 0.7f, 0.15f, 0.15f, 1.0f, 0.3f);
        }
    }
};
