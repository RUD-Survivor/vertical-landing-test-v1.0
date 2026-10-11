#version 450
#extension GL_GOOGLE_include_directive : enable
#include "common/camera_bloom.glsl"
layout(location=0) in vec2 uv;
layout(location=0) out vec4 FragColor;
layout(set=0,binding=0) uniform sampler2D sceneImage;
layout(set=0,binding=1) uniform sampler2D levels[5];
layout(set=0,binding=2) uniform sampler2D depthImage;
void main(){
 // Sun core already has a dedicated diffraction/scatter budget. Partition the
 // source image, so scene-wide bloom cannot process that same source twice.
 vec3 source=solarPixel(uv,depthImage)?vec3(0):max(textureLod(sceneImage,uv,0).rgb,vec3(0));
 vec3 spread=.15*textureLod(levels[0],uv,0).rgb+.25*textureLod(levels[1],uv,0).rgb
             +.30*textureLod(levels[2],uv,0).rgb+.20*textureLod(levels[3],uv,0).rgb
             +.10*textureLod(levels[4],uv,0).rgb;
 // Add a redistribution delta into HDR. All channels stay nonnegative after
 // blending; constant input is preserved away from the cropped field edges.
 FragColor=vec4(pc.fraction*(spread-source),0);
}
