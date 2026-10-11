#version 450
#extension GL_GOOGLE_include_directive : enable
#include "common/solar_optics.glsl"
layout(location=0) in vec2 aPos;
layout(location=0) out vec2 vUV;
layout(location=1) out vec2 vNdc;
layout(location=2) flat out vec3 vTransmission;
layout(location=3) flat out vec4 vSourceBounds;
layout(std430,set=0,binding=3) readonly buffer SourceCache {
 vec4 bounds;vec4 transmission;vec4 weights[256];
} cache;
void main(){
 vUV=aPos*.5+.5;
 vec2 extent=pc.shapeType==6?vec2(pc.scale.x):pc.scale;
 vNdc=aPos*extent/vec2(pc.aspect,1)+pc.offset;
 vSourceBounds=cache.bounds;
 vTransmission=cache.transmission.rgb;
 gl_Position=vec4(vNdc,.9999,1);
}
