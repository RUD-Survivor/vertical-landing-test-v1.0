#version 450
#extension GL_GOOGLE_include_directive : enable
#include "common/water_surface.glsl"
// terrain_shadow.vert — 级联阴影贴图的地形深度专用顶点着色器。
//
// Height, morph and skirts use the same shared sources as terrain.vert.

layout(push_constant) uniform PC {
    mat4  model;           // offset   0
    float planetRadius;    // offset  64
    float maxElevation;    // offset  68
    int   nodeLevel;       // offset  72
    int   anchorGeometry;   // offset  76
    vec3  nodePos;         // offset  80
    float lodMorph;
    vec3  nodeSide;        // offset  96
    float skirtDepthKm;
    vec3  nodeUp;          // offset 112
    float detailWeight;
    mat4  lightViewProj;   // offset 128
} pc;

// 复用 vk_scene.h::VkTerrainPipeline::set1Layout（同一个 VkDescriptorSetLayout
// 句柄，见 vk_shadow_cascade.h::VkCascadeShadowSystem::init 的 pSetLayouts），
// 所以这里直接绑到 set=0（本管线只有这一个 set），运行时复用同一份
// terrainGlobalTexSet，不需要另外分配/维护一份纹理描述符集。
layout(set = 0, binding = 0) uniform sampler2D uTectonicMap;
layout(set = 0, binding = 1) uniform sampler2D uHydroMap;
layout(set = 0, binding = 2) uniform sampler2D uClimateMap;
layout(set = 0, binding = 3) uniform sampler2D uLocalHydroMap;

layout(location = 0) in vec3 aPos;
layout(location = 1) in vec3 aNormal;
layout(location = 2) in vec2 aUV;

#include "common/terrain_sampling.glsl"
#include "common/terrain_height_shared.glsl"
#include "common/terrain_geometry.glsl"
void main(){
    vec3 cube=pc.nodePos+aPos.x*pc.nodeSide+aPos.z*pc.nodeUp;
    vec3 local=thGeometry(cube,aPos.y,thPoint(cube));
    vec3 origin=(pc.anchorGeometry&1)==1?vec3(pc.model[0].w,pc.model[1].w,pc.model[2].w):pc.model[3].xyz;
    vec3 world=origin+(mat3(pc.model)/pc.planetRadius)*local;
    gl_Position=pc.lightViewProj*vec4(world,1.0);
    gl_Position.y=-gl_Position.y;
    gl_Position.z=gl_Position.z*0.5+gl_Position.w*0.5;
}
