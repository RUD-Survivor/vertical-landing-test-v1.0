
#version 330 core
layout(location=0) in vec2 aOffset;
// Need View and Proj separately precisely for billboarding
uniform mat4 uView;
uniform mat4 uProj;
uniform vec3 uCenter;
uniform vec2 uSize;
out vec2 vUV;
void main() {
  vUV = aOffset * 0.5 + 0.5;
  // Exact spherical billboarding: extract camera right and up from View Matrix
  // uView[0][0], uView[1][0], uView[2][0] is the right vector
  // uView[0][1], uView[1][1], uView[2][1] is the up vector
  vec3 right = vec3(uView[0][0], uView[1][0], uView[2][0]);
  vec3 up    = vec3(uView[0][1], uView[1][1], uView[2][1]);
  
  // Compute world position of this vertex
  vec3 worldPos = uCenter + right * aOffset.x * uSize.x + up * aOffset.y * uSize.y;
  
  // Project to screen
  gl_Position = uProj * uView * vec4(worldPos, 1.0);
}
    
