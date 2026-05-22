
#version 330 core
layout(location=0) in vec3 aPos;
layout(location=1) in vec3 aNormal;
layout(location=4) in vec3 iPos;     // Instance position
layout(location=5) in float iScale;  // Instance scale
layout(location=6) in float iRot;    // Instance Y-rotation

uniform mat4 uView;
uniform mat4 uProj;
uniform vec3 uPlanetCenter;

out vec3 vNormal;
out vec3 vWorldPos;

void main() {
  // Construct instance rotation matrix (Y-axis)
  float s = sin(iRot);
  float c = cos(iRot);
  mat3 rot = mat3(c, 0, s,  0, 1, 0, -s, 0, c);
  
  // Transform local position
  vec3 pos = (rot * aPos) * iScale;
  
  // Orient to planet surface
  vec3 up = normalize(iPos);
  vec3 right = normalize(cross(vec3(0,1,0), up));
  if (length(right) < 0.01) right = normalize(cross(vec3(1,0,0), up));
  vec3 forward = cross(up, right);
  mat3 orient = mat3(right, up, forward);
  
  vec3 worldPos = orient * pos + iPos;
  vWorldPos = worldPos;
  vNormal = orient * (rot * aNormal);
  
  gl_Position = uProj * uView * vec4(worldPos, 1.0);
}
    
