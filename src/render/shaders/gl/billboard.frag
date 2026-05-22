
#version 330 core
in vec2 vUV;
uniform vec4 uColor;
out vec4 FragColor;
void main() {
  float d = distance(vUV, vec2(0.5));
  float alpha = smoothstep(0.5, 0.0, d);
  FragColor = vec4(uColor.rgb, uColor.a * alpha);
}
    
