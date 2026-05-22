
#version 330 core
layout(location=0) in vec2 aPos;
uniform mat4 uInvViewProj;
out vec3 vRayDir;
void main() {
  gl_Position = vec4(aPos, 0.999, 1.0);
  // Reconstruct world-space ray direction from NDC
  vec4 worldPos = uInvViewProj * vec4(aPos, 1.0, 1.0);
  vRayDir = worldPos.xyz / worldPos.w;
}
    
