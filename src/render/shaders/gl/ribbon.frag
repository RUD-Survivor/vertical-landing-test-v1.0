
#version 330 core
in vec4 vColor;
in float vSide;
out vec4 FragColor;
void main() {
  float alpha = 1.0 - smoothstep(0.7, 1.0, abs(vSide));
  FragColor = vec4(vColor.rgb, vColor.a * alpha);
}
    
