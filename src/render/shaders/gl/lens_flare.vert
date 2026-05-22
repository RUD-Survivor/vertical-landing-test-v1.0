
#version 330 core
layout(location=0) in vec2 aPos;

uniform vec2 uSunScreenPos; // Position of sun in NDC (-1 to 1)
uniform float uAspect;      // Screen aspect ratio
uniform vec2 uScale;        // Scale of this specific flare element
uniform vec2 uOffset;       // Screen-space offset relative to sun or center

out vec2 vUV;

void main() {
  vUV = aPos * 0.5 + 0.5;
  vec2 pos = aPos * uScale;
  pos.x /= uAspect;
  // Output at the sun's actual screen-space depth to allow occlusion
  gl_Position = vec4(pos + uOffset, uSunScreenPos.x > -2.0 ? 0.9999 : 0.0, 1.0);
}
    
