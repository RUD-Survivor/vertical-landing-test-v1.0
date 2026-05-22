
#version 330 core
in vec2 vUV;

uniform vec4 uColor;
uniform float uIntensity;
uniform int uShapeType; // 0=glow, 1=anamorphic, 2=ghost, 3=starburst

out vec4 FragColor;

void main() {
  vec2 rUV = vUV * 2.0 - 1.0;
  float d = length(rUV);
  // Edge kill: smoothly fade to zero near quad boundary - prevents visible rectangle
  float edgeFade = smoothstep(1.0, 0.65, d);
  float alpha = 0.0;
  
  if (uShapeType == 0) {
     // Clean circular glow with gaussian falloff
     float core = exp(-d * d * 12.0) * 2.5;
     float mid  = exp(-d * d * 3.0) * 0.6;
     float wide = exp(-d * d * 0.8) * 0.15;
     alpha = (core + mid + wide) * edgeFade;
     
     // 4-point diffraction spikes (like real camera lens)
     float angle = atan(rUV.y, rUV.x);
     float spike1 = pow(abs(cos(angle)), 300.0);
     float spike2 = pow(abs(sin(angle)), 300.0);
     float spikes = (spike1 + spike2) * exp(-d * 1.8) * 0.4;
     alpha += spikes * edgeFade;
     
  } else if (uShapeType == 1) {
     // Anamorphic horizontal streak - soft edges
     float dx = abs(rUV.x);
     float dy = abs(rUV.y);
     float edgeFadeX = smoothstep(1.0, 0.7, dx);
     alpha = exp(-dx * dx * 0.5) * exp(-dy * dy * 600.0) * edgeFadeX;
     alpha += exp(-dx * dx * 1.5) * exp(-dy * dy * 2000.0) * 0.6 * edgeFadeX;
     
  } else if (uShapeType == 2) {
     // Subtle ghost disk (not hard hexagon ring - those look fake)
     float softDisk = exp(-d * d * 4.0) * 0.4;
     float ring = smoothstep(0.95, 0.75, d) * smoothstep(0.55, 0.7, d) * 0.3;
     alpha = (softDisk + ring) * edgeFade;
     
  } else if (uShapeType == 3) {
     // Starburst: very subtle radial rays
     float angle = atan(rUV.y, rUV.x);
     float rays = pow(abs(sin(angle * 6.0)), 80.0) * 0.4;
     rays += pow(abs(cos(angle * 8.0 + 0.3)), 120.0) * 0.2;
     alpha = rays * exp(-d * d * 3.0) * edgeFade;
     alpha += exp(-d * d * 10.0) * 0.2 * edgeFade;
  }
  
  FragColor = vec4(uColor.rgb, uColor.a * alpha * uIntensity);
}
    
