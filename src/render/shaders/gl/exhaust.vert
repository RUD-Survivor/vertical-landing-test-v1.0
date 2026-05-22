
#version 330 core
layout(location=0) in vec3 aPos;
uniform mat4 uMVP;
uniform mat4 uModel;
out vec3 vLocalPos;
out vec3 vWorldPos;
void main() {
  vLocalPos = aPos; // Box from -0.5 to 0.5
  vWorldPos = (uModel * vec4(aPos, 1.0)).xyz;
  gl_Position = uMVP * vec4(aPos, 1.0);
}
    
