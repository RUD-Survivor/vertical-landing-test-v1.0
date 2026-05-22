
#version 330 core
layout(location=0) in vec3 aPos;
layout(location=1) in vec3 aNormal;
layout(location=2) in vec2 aUV;
layout(location=3) in vec4 aColor;

uniform mat4 uMVP;     // 投影矩阵
uniform mat4 uModel;   // 模型变换矩阵

out vec3 vWorldPos;
out vec3 vNormal;
out vec2 vUV;
out vec4 vColor;
out vec3 vLocalPos;

void main() {
  vec4 worldPos = uModel * vec4(aPos, 1.0);
  vWorldPos = worldPos.xyz;
  vNormal = mat3(transpose(inverse(uModel))) * aNormal;
  vUV = aUV;
  vColor = aColor;
  vLocalPos = aPos;
  gl_Position = uMVP * vec4(aPos, 1.0);
}
    
