#version 450
#extension GL_GOOGLE_include_directive : enable
#include "common/solar_optics.glsl"
layout(location=0) in vec2 aPos;
layout(location=0) out vec2 vUV;
layout(location=1) out vec2 vNdc;
layout(location=2) flat out vec3 vTransmission;
layout(location=3) flat out vec4 vSourceBounds;
void main() {
 vSourceBounds=vec4(0);
 vUV=aPos*.5+.5;vNdc=aPos*pc.scale/vec2(pc.aspect,1)+pc.offset;
 vTransmission=vec3(1);gl_Position=vec4(vNdc,.9999,1);
}
