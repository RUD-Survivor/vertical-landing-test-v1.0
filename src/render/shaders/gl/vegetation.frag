
#version 330 core
in vec3 vNormal;
in vec3 vWorldPos;
uniform vec3 uLightDir;
uniform vec3 uViewPos;
out vec4 FragColor;
void main() {
  vec3 N = normalize(vNormal);
  vec3 L = normalize(uLightDir);
  float diff = max(dot(N, L), 0.0);
  vec3 color = vec3(0.1, 0.4, 0.05); // Standard forest green
  FragColor = vec4(color * (0.2 + 0.8 * diff), 1.0);
}
    
