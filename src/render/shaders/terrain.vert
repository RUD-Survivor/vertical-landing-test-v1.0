#version 450
#extension GL_GOOGLE_include_directive : enable
#include "common/water_surface.glsl"
#ifdef OCEAN_SURFACE
#include "common/ocean_waves.glsl"
#endif
// Terrain vertex shader — Vulkan port, logic identical to OpenGL version

layout(set = 0, binding = 0) uniform FrameUBO {
    mat4  view;
    mat4  proj;
    vec3  lightDir;
    float _pad0;
    vec3  viewPos;
    float time;
} frame;

layout(set = 0, binding = 1, std140) uniform TerrainData {
    vec4  planetCenterRel;
} terrain;

layout(set = 1, binding = 0) uniform sampler2D uTectonicMap;
layout(set = 1, binding = 1) uniform sampler2D uHydroMap;
layout(set = 1, binding = 2) uniform sampler2D uClimateMap;
layout(set = 1, binding = 3) uniform sampler2D uLocalHydroMap;

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
} pc;

layout(location = 0) in vec3 aPos;
layout(location = 1) in vec3 aNormal;
layout(location = 2) in vec2 aUV;

layout(location = 0) out vec3 vRelViewPos;
layout(location = 1) out vec3 vNormal;
layout(location = 2) out vec2 vUV;
layout(location = 3) out vec2 vLocalUV;
layout(location = 4) out vec3 vLocalPos;
layout(location = 5) out float vElevation;
layout(location = 6) out float vWaterDepth;
layout(location = 7) out float vSlope;

#include "common/terrain_sampling.glsl"
#include "common/terrain_height_shared.glsl"
#include "common/terrain_geometry.glsl"

void main() {
    vLocalUV=aPos.xz+0.5;
    vec3 cube=pc.nodePos+aPos.x*pc.nodeSide+aPos.z*pc.nodeUp;
    vec3 normPos=normalize(cube);
    float bedKm=thHeight(normPos,pc.planetRadius,pc.maxElevation,pc.detailWeight);
    float waterKm=thWater(normPos,pc.planetRadius,pc.maxElevation,bedKm);
    vElevation=thElevation(bedKm,pc.maxElevation);
    vWaterDepth=max(0.0,waterKm-bedKm)*1000.0;
    vec3 planetPos=thGeometry(cube,aPos.y,thSurfacePoint(cube,normPos,bedKm,waterKm));
    mat3 rotation=mat3(pc.model)/pc.planetRadius;
    vec3 origin=(pc.anchorGeometry&1)==1?vec3(pc.model[0].w,pc.model[1].w,pc.model[2].w):pc.model[3].xyz-frame.viewPos;
    vRelViewPos=origin+rotation*planetPos;
    // The fragment derives the actual rendered triangle normal, including
    // LOD morph and skirts. Four extra height evaluations per vertex are avoided.
    vNormal=rotation*normPos;
    vSlope=0.0;
    vUV=aUV;vLocalPos=normPos;
    gl_Position=frame.proj*vec4(mat3(frame.view)*vRelViewPos,1.0);
    gl_Position.y=-gl_Position.y;
    gl_Position.z=gl_Position.z*0.5+gl_Position.w*0.5;
}
