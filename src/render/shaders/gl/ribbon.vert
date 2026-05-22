
#version 330 core
layout(location=0) in vec3 aPos;
layout(location=1) in vec4 aColor;
layout(location=2) in float qSide; // -1 to 1
uniform mat4 uMVP;
out vec4 vColor;
out float vSide;
void main() {
  vColor = aColor;
  vSide = qSide;
  gl_Position = uMVP * vec4(aPos, 1.0);
}
    
