#pragma once
#include "render/renderer3d.h"
#include "render/model_loader.h"
#include "simulation/rocket_builder.h"
#include <string>
#include <map>

namespace PartRenderer {

    // Shared rendering logic for a single part, supporting custom OBJ models and procedural fallbacks.
    inline void drawPart(Renderer3D* r3d, const PartDef& def, Vec3 pos, Quat rot, 
                        Mesh& fallbackBody, Mesh& fallbackNose, Mesh& fallbackBox,
                        float scale = 1.0f, bool highlight = false, bool selected = false, float alpha = 1.0f, 
                        float rm = 1.0f, float gm = 1.0f, float bm = 1.0f) {
        
        float r = def.r * rm, g = def.g * gm, b = def.b * bm;
        
        // Selection/Highlight effects
        if (highlight) {
            float blink = 0.5f + 0.5f * sinf((float)glfwGetTime() * 8.0f);
            r = std::min(1.0f, r + 0.4f * blink); 
            g = std::min(1.0f, g + 0.6f * blink); 
            b = std::min(1.0f, b + 0.3f * blink);
        }
        if (selected) {
            r = r * 0.4f; g = g * 0.6f; b = std::min(1.0f, b + 0.7f);
        }

        // --- Custom Model and Texture Loading ---
        Mesh* customMesh = nullptr;
        bool hasTexture = false;
        
        if (def.model_path) {
            std::string mp(def.model_path);
            if (r3d->meshCache.find(mp) == r3d->meshCache.end()) {
                r3d->meshCache[mp] = ModelLoader::loadOBJ(mp);
            }
            customMesh = &r3d->meshCache[mp];
        }
        
        if (def.texture_path) {
            std::string tp(def.texture_path);
            if (r3d->textureCache.find(tp) == r3d->textureCache.end()) {
                r3d->textureCache[tp] = Renderer3D::loadTGA(def.texture_path);
            }
            if (r3d->textureCache[tp].id != 0) {
                r3d->textureCache[tp].bind(0);
                hasTexture = true;
            }
        }
        
        glUniform1i(r3d->u_hasTexture, hasTexture ? 1 : 0);
        glUniform1i(r3d->u_sampler, 0);

        if (customMesh && customMesh->indexCount > 0) {
            // Use custom model (apply uniform scale if provided)
            Mat4 partMat = Mat4::translate(pos) * Mat4::fromQuat(rot) * Mat4::scale({scale, scale, scale});
            r3d->drawMesh(*customMesh, partMat, r, g, b, alpha, 0.2f);
        } else {
            // Procedural fallback
            float pd = def.diameter * scale;
            float ph = def.height * scale;
            if (def.category == CAT_NOSE_CONE) {
                Mat4 partMat = Mat4::translate(pos) * Mat4::fromQuat(rot) * Mat4::scale({pd, ph, pd});
                r3d->drawMesh(fallbackNose, partMat, r, g, b, alpha, 0.2f);
            } else if (def.category == CAT_ENGINE) {
                float bf = 0.4f; float nf = 1.0f - bf;
                Mat4 rotMat = Mat4::fromQuat(rot);
                Mat4 bodyMat = Mat4::translate(pos) * rotMat * Mat4::translate(Vec3(0, ph*(1.0f-bf*0.5f), 0)) * Mat4::scale({pd*0.6f, ph*bf, pd*0.6f});
                r3d->drawMesh(fallbackBody, bodyMat, 0.2f*rm, 0.2f*gm, 0.22f*bm, alpha, 0.4f);
                Mat4 bellMat = Mat4::translate(pos) * rotMat * Mat4::scale({pd*0.85f, ph*nf, pd*0.85f});
                r3d->drawMesh(fallbackNose, bellMat, r*0.8f, g*0.8f, b*0.8f, alpha, 0.1f);
            } else if (def.category == CAT_STRUCTURAL) {
                if (strstr(def.name, "Fin") || strstr(def.name, "Solar")) {
                    Mat4 finMat = Mat4::translate(pos + rot.rotate(Vec3(0, ph*0.5f, 0))) * Mat4::fromQuat(rot) * Mat4::scale({pd*0.05f, ph, pd*0.5f});
                    r3d->drawMesh(fallbackBox, finMat, r, g, b, alpha, 0.1f);
                } else if (strstr(def.name, "Leg")) {
                    Mat4 legMat = Mat4::translate(pos + rot.rotate(Vec3(0, ph*0.5f, 0))) * Mat4::fromQuat(rot) * Mat4::scale({pd*0.15f, ph, pd*0.15f});
                    r3d->drawMesh(fallbackBox, legMat, r, g, b, alpha, 0.1f);
                } else {
                    Mat4 partMat = Mat4::translate(pos) * Mat4::fromQuat(rot) * Mat4::translate(Vec3(0, ph*0.5f, 0)) * Mat4::scale({pd, ph, pd});
                    r3d->drawMesh(fallbackBody, partMat, r, g, b, alpha, 0.2f);
                }
            } else {
                Mat4 partMat = Mat4::translate(pos) * Mat4::fromQuat(rot) * Mat4::translate(Vec3(0, ph*0.5f, 0)) * Mat4::scale({pd, ph, pd});
                r3d->drawMesh(fallbackBody, partMat, r, g, b, alpha, 0.2f);
            }
        }
        
        // Cleanup texture binding
        glUniform1i(r3d->u_hasTexture, 0);
    }

    // Helper for symmetry rendering
    inline void drawPartWithSymmetry(Renderer3D* r3d, const PartDef& def, Vec3 pos, Quat rot, 
                                    Mesh& fallbackBody, Mesh& fallbackNose, Mesh& fallbackBox,
                                    float scale = 1.0f, bool highlight = false, bool selected = false, float alpha = 1.0f, int sym = 1, 
                                    float rm = 1.0f, float gm = 1.0f, float bm = 1.0f) {
        
        for (int s = 0; s < sym; s++) {
            float angle = (s * 2.0f * 3.14159f) / sym;
            Vec3 symPos = pos;
            if (sym > 1) {
                float dist = sqrt(pos.x*pos.x + pos.z*pos.z);
                if (dist > 0.01f) {
                   float curAngle = atan2(pos.z, pos.x);
                   symPos.x = cos(curAngle + angle) * dist;
                   symPos.z = sin(curAngle + angle) * dist;
                }
            }
            
            // Adjust rotation for symmetry
            Quat symRot = rot * Quat::fromAxisAngle(Vec3(0, 1, 0), angle);
            
            drawPart(r3d, def, symPos, symRot, fallbackBody, fallbackNose, fallbackBox, scale, highlight, selected, alpha, rm, gm, bm);
        }
    }
}
