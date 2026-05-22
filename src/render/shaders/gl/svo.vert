
#version 330 core
layout(location=0) in vec3 aPos;
layout(location=1) in vec3 aNormal;
layout(location=2) in vec2 aUV;
layout(location=3) in vec4 aColor;

uniform mat4 uMVP;
uniform mat4 uSvoMat; // Transforms SVO local to Camera-Relative World

out vec3 vRelPos;
out vec3 vNormal;
out vec4 vColor;

void main() {
  vec4 relPos = uSvoMat * vec4(aPos, 1.0);
  vRelPos = relPos.xyz;
  
  // As SvoMat's upper 3x3 is pure rotation (orthonormal), 
  // we can just multiply directly to transform normals:
  vNormal = mat3(uSvoMat) * aNormal;
  
  vColor = aColor;
  gl_Position = uMVP * relPos;
}
    
