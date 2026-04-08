#pragma once
#include "core/rocket_state.h"
#include "render/renderer3d.h"
#include "simulation/orbit_physics.h" // For SOLAR_SYSTEM
#include "camera/camera_director.h"
#include <iostream>

using namespace std;

/**
 * CelestialRenderer - 天体系统渲染器
 * =========================================================================
 * 负责渲染太阳系的所有自然天体，从微观到宏观无缝缩放：
 * 
 * 职责：
 *  1. 动态自适应远/近裁剪面 (Macro Pass Projection) 避免 Z-fighting
 *  2.  procedural 星空背景绘制 (Skybox/Milky Way)
 *  3. 天体物理球体渲染 (地球、月球、火星、太阳本体)
 *  4. 动态体积大气散射与云层动画 (Volumetric Scattering & Clouds)
 *  5. 气态巨行星星环绘制 (Ringed Gas Giants)
 *  6. 全景模式(Map)下的天体轨道线(带轨道参数推演)
 *  7. 屏幕空间镜头光晕与星球遮蔽测试 (Sun and Lens Flare)
 */
class CelestialRenderer {
public:
    void renderMacroPass(Renderer3D* r3d, const RocketState& rocket_state, 
                         int current_soi_index, const CameraDirector& cam, 
                         const Vec3& camEye_rel, float cam_dist,
                         double ws_d, double ro_x, double ro_y, double ro_z, 
                         float aspect, float day_blend, float alt_factor, 
                         bool show_clouds, int frame, int ww, int wh, 
                         const Vec3& renderRocketBase, const Vec3& renderSun, 
                         float sun_radius, float earth_r,
                         const Mesh& earthMesh, const Mesh& ringMesh,
                         const Mat4& viewMat,
                         Mat4& out_macroProjMat, Vec3& out_lightVec)
    {
        // 动态远裁剪面 
        // Ensure far clipping plane covers the entire solar system (expanded to 1000 AU)
        float far_plane = 1000.0f * 149597870.0f; // 1000 AU in km (ws_d)
        
        // =========== PASS 1: MACRO BACKGROUND ===========
        // Compute a smart near plane that keeps the depth buffer ratio (far/near) within
        // ~1e6 to avoid z-fighting on planet surfaces. Find the closest planet surface
        // distance from the camera and use a fraction of that as the near plane.
        float closest_planet_dist = far_plane;
        for (size_t i = 1; i < SOLAR_SYSTEM.size(); i++) {
            Vec3 rp((float)(SOLAR_SYSTEM[i].px * ws_d - ro_x),
                (float)(SOLAR_SYSTEM[i].py * ws_d - ro_y),
                (float)(SOLAR_SYSTEM[i].pz * ws_d - ro_z));
            float body_r = (float)SOLAR_SYSTEM[i].radius * (float)ws_d;
            float atmo_thickness = (SOLAR_SYSTEM[i].type == GAS_GIANT || SOLAR_SYSTEM[i].type == RINGED_GAS_GIANT) ? body_r * 0.05f : 160.0f;
            float dist_to_center = (camEye_rel - rp).length();
            float true_surf_dist = dist_to_center - body_r;
            // Consider terrain displacement (max 25km for Earth)
            float terrain_buffer = (i == 3) ? 25.0f : 0.0f;
            float effective_surf_dist = true_surf_dist - terrain_buffer;
            float atmo_surf_dist = dist_to_center - (body_r + atmo_thickness);
            
            // Use the atmosphere boundary for camera clipping if it exists
            float geo_dist = (atmo_surf_dist > 0.0f) ? atmo_surf_dist : effective_surf_dist;
            if (geo_dist < closest_planet_dist) {
                closest_planet_dist = geo_dist;
            }
        }
        
        // Also consider distance to the Sun
        {
            float sun_surf = (camEye_rel - renderSun).length() - sun_radius;
            if (sun_surf < closest_planet_dist)
                closest_planet_dist = sun_surf;
        }
        
        // Industrial Grade Near Plane:
        // We use a small fraction of the distance to the actual surface (including mountains).
        // If we are extremely close (less than 1km), we force a very small near plane for micro-detail.
        float macro_near = fmaxf(0.00001f, closest_planet_dist * 0.05f); // 5% of distance
        macro_near = fminf(macro_near, 1.0f); // Cap macro near plane at 1km
        if (closest_planet_dist < 1.0f) macro_near = fminf(macro_near, 0.0001f); // 10cm when near ground
        macro_near = fmaxf(macro_near, cam_dist * 0.0001f); // but never clip behind target
        
        out_macroProjMat = Mat4::perspective(0.8f, aspect, macro_near, far_plane);
        
        r3d->beginTAAPass();
        r3d->beginFrame(viewMat, out_macroProjMat, camEye_rel);
        
        // ===== SKYBOX: Procedural Starfield + Milky Way =====
        // Calculate vibrancy: 1.0 in space or at night, 0.0 during bright day on ground
        // Note: sky_day factor is used to wash out stars when looking through a lit atmosphere
        float sky_day_local = (float)(day_blend * (1.0 - alt_factor));
        float sky_vibrancy = 1.0f - sky_day_local;
        r3d->drawSkybox(sky_vibrancy, aspect);
        
        // ===== 太阳物理本体 =====
        if (cam.mode == 2) {
            // 仅在全景模式渲染巨型的物理太阳模型避免遮盖火箭本体细节
            Mat4 sunModel = Mat4::scale(Vec3(sun_radius, sun_radius, sun_radius));
            sunModel = Mat4::translate(renderSun) * sunModel;
            // 复用 earthMesh，修改极高环境光(ambient=2.0)让其纯亮发白发黄
            r3d->drawMesh(earthMesh, sunModel, 1.0f, 0.95f, 0.9f, 1.0f, 2.0f);
        }
        
        // 渲染整个太阳系
        CelestialBody& sun_body = SOLAR_SYSTEM[0];
        for (size_t i = 1; i < SOLAR_SYSTEM.size(); i++) {
            CelestialBody& b = SOLAR_SYSTEM[i];
            float r = (float)b.radius * ws_d;
            Vec3 renderPlanet((float)(b.px * ws_d - ro_x), (float)(b.py * ws_d - ro_y), (float)(b.pz * ws_d - ro_z));
            
            // 应用主体的自转与极轴倾斜
            Quat align_to_z = Quat::fromAxisAngle(Vec3(1.0f, 0.0f, 0.0f), -PI / 2.0f);
            Quat spin = Quat::fromAxisAngle(Vec3(0.0f, 0.0f, 1.0f), b.prime_meridian_epoch + (rocket_state.sim_time * 2.0 * PI / b.rotation_period));
            Quat tilt = Quat::fromAxisAngle(Vec3(1.0f, 0.0f, 0.0f), b.axial_tilt);
            Quat rotation_quat = tilt * spin * align_to_z;
            
            Mat4 planetModel = Mat4::scale(Vec3(r, r, r));
            planetModel = Mat4::fromQuat(rotation_quat) * planetModel; // Apply rotation
            planetModel = Mat4::translate(renderPlanet) * planetModel;
            
            // Compute per-planet lightDir in double precision for correct sun-facing
            {
                double light_dx = sun_body.px - b.px;
                double light_dy = sun_body.py - b.py;
                double light_dz = sun_body.pz - b.pz;
                double light_len = sqrt(light_dx * light_dx + light_dy * light_dy + light_dz * light_dz);
                if (light_len > 1.0) {
                    r3d->lightDir = Vec3((float)(light_dx / light_len), (float)(light_dy / light_len), (float)(light_dz / light_len));
                }
            }
            
            r3d->drawPlanet(earthMesh, planetModel, b.type, b.r, b.g, b.b, 1.0f, r, (float)rocket_state.sim_time, (int)i);
            
            if ((b.type == TERRESTRIAL || b.type == GAS_GIANT) && i != 1 && i != 4) {
                float atmo_radius = r + 160.0f;
                Mat4 atmoModel = Mat4::scale(Vec3(atmo_radius, atmo_radius, atmo_radius));
                atmoModel = Mat4::translate(renderPlanet) * atmoModel;
                r3d->drawAtmosphere(earthMesh, atmoModel, camEye_rel, r3d->lightDir, renderPlanet, r, atmo_radius, (float)rocket_state.sim_time, (int)i, (float)day_blend, show_clouds);
                
                if (frame <= 2) cout << "[Atmo] body=" << i << " r=" << r << " atmo_r=" << atmo_radius << " day=" << day_blend << " clouds=" << show_clouds << " planet=(" << renderPlanet.x << "," << renderPlanet.y << "," << renderPlanet.z << ")" << endl;
            }
            
            if (b.type == RINGED_GAS_GIANT) {
                Mat4 ringModel = Mat4::scale(Vec3(r, r, r));
                ringModel = Mat4::fromQuat(rotation_quat) * ringModel;
                ringModel = Mat4::translate(renderPlanet) * ringModel;
                r3d->drawRing(ringMesh, ringModel, b.r, b.g, b.b, 0.4f);
            }
            
            // 渲染行星轨道和标签 (仅在全景模式下显示)
            if (cam.mode == 2) {
                double a = b.sma_base;
                double e = b.ecc_base;
                double i_inc = b.inc_base;
                double lan = b.lan_base;
                double arg_p = b.arg_peri_base;
                
                double planet_px = b.px; double planet_py = b.py; double planet_pz = b.pz;
                if (i == 4) { planet_px -= SOLAR_SYSTEM[3].px; planet_py -= SOLAR_SYSTEM[3].py; planet_pz -= SOLAR_SYSTEM[3].pz; }
                
                int segs = 181;
                float orbit_center_dist = renderPlanet.length();
                float cam_to_origin = camEye_rel.length();
                float ref_dist = fmaxf(cam_to_origin, orbit_center_dist);
                float ring_w = fmaxf(earth_r * 0.008f, ref_dist * 0.0035f);
                if (i == 4) ring_w *= 0.5f;
                
                double c_O = cos(lan), s_O = sin(lan);
                double c_w = cos(arg_p), s_w = sin(arg_p);
                double c_i = cos(i_inc), s_i = sin(i_inc);
                
                std::vector<Vec3> orbit_pts;
                std::vector<Vec4> orbit_cols;
                orbit_pts.reserve(segs);
                orbit_cols.reserve(segs);
                
                float orb_r = fminf(1.0f, b.r * 0.6f + 0.3f);
                float orb_g = fminf(1.0f, b.g * 0.6f + 0.3f);
                float orb_b = fminf(1.0f, b.b * 0.6f + 0.3f);
                float orb_a = 0.7f;
                
                for (int k = 0; k < segs; k++) {
                    double E_k = (double)k / (segs - 1) * 2.0 * PI;
                    double nu_k = 2.0 * atan2(sqrt(1.0 + e) * sin(E_k / 2.0), sqrt(1.0 - e) * cos(E_k / 2.0));
                    double r_dist_k = a * (1.0 - e * cos(E_k));
                    double o_xk = r_dist_k * cos(nu_k);
                    double o_yk = r_dist_k * sin(nu_k);
                    
                    double wx = (c_O * c_w - s_O * s_w * c_i) * o_xk + (-c_O * s_w - s_O * c_w * c_i) * o_yk;
                    double wy = (s_O * c_w + c_O * s_w * c_i) * o_xk + (-s_O * s_w + c_O * c_w * c_i) * o_yk;
                    double wz = (s_w * s_i) * o_xk + (c_w * s_i) * o_yk;
                    
                    if (i == 4) { // Moon orbits Earth
                        wx += SOLAR_SYSTEM[3].px;
                        wy += SOLAR_SYSTEM[3].py;
                        wz += SOLAR_SYSTEM[3].pz;
                    }
                    orbit_pts.push_back(Vec3(
                        (float)(wx * ws_d - ro_x),
                        (float)(wy * ws_d - ro_y),
                        (float)(wz * ws_d - ro_z)
                    ));
                    orbit_cols.push_back(Vec4(orb_r, orb_g, orb_b, orb_a));
                }
                r3d->drawRibbon(orbit_pts, orbit_cols, ring_w);
                
                float dist_to_cam = (renderPlanet - camEye_rel).length();
                float marker_size = fmaxf((float)b.radius * (float)ws_d * 2.0f, dist_to_cam * 0.015f);
                r3d->drawBillboard(renderPlanet, marker_size, b.r, b.g, b.b, 0.9f);
            }
        }
        
        // Restore lightDir to rocket's own Sun direction (for rocket mesh, trajectory rendering, etc.)
        out_lightVec = renderSun - renderRocketBase;
        r3d->lightDir = out_lightVec.normalized();
        
        // ===== 太阳与镜头光晕 (所有模式可见) =====
        std::vector<Vec4> sun_occluders;
        for (size_t i = 1; i < SOLAR_SYSTEM.size(); i++) {
            CelestialBody& b = SOLAR_SYSTEM[i];
            sun_occluders.push_back(Vec4(
                (float)(b.px * ws_d - ro_x),
                (float)(b.py * ws_d - ro_y),
                (float)(b.pz * ws_d - ro_z),
                (float)b.radius * (float)ws_d
            ));
        }
        r3d->drawSunAndFlare(renderSun, sun_occluders, ww, wh);
    }
};
