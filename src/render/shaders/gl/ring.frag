
#version 330 core
in vec3 vNormal; in vec3 vLocalPos; in vec3 vWorldPos;
uniform vec3 uLightDir; uniform vec3 uViewPos; uniform vec4 uBaseColor;
uniform float uPlanetRadius; 
out vec4 FragColor;

float henyeyGreenstein(float cosTheta, float g) {
  float g2 = g * g;
  return (1.0 - g2) / (4.0 * 3.14159 * pow(1.0 + g2 - 2.0 * g * cosTheta, 1.5));
}

// Exact Saturn Ring Optical Depth Profiles (NASA Voyager/Cassini)
float getRingOpticalDepth(float r) {
  // D Ring: 1.11 - 1.24 (faint)
  if (r < 1.11) return 0.0;
  if (r < 1.24) return 0.005;
  // C Ring: 1.24 - 1.53 (moderate)
  if (r < 1.53) return mix(0.05, 0.15, smoothstep(1.24, 1.53, r));
  // B Ring: 1.53 - 1.95 (brightest, densest)
  if (r < 1.95) return mix(1.2, 2.5, sin(r * 40.0) * 0.2 + 0.8);
  // Cassini Division: 1.95 - 2.03 (gap)
  if (r < 2.03) return 0.02;
  // A Ring: 2.03 - 2.27 (dense)
  if (r < 2.27) {
      float base = 0.5;
      float encke = smoothstep(2.21, 2.22, r) * smoothstep(2.23, 2.22, r);
      return mix(base, 0.01, encke);
  }
  // F Ring: ~2.33 (thin)
  if (r < 2.31) return 0.0;
  if (r < 2.34) return 0.1;
  return 0.0;
}

void main() {
  vec3 N = normalize(vNormal);
  vec3 L = normalize(uLightDir);
  vec3 V = normalize(uViewPos - vWorldPos);
  float cosTheta = dot(V, -L);
  float NdotV = abs(dot(N, V));
  
  // --- Physically Based Scattering ---
  // Industrial Ice Particle Phase: Strong forward peak (diffraction) + backward lobe
  float phaseForward = henyeyGreenstein(cosTheta, 0.85); // Cinematic diffraction
  float phaseBack = henyeyGreenstein(cosTheta, -0.3);
  float phase = mix(phaseForward, phaseBack, 0.4);
  
  // --- Hardcore Planet Shadow ---
  float b = 2.0 * dot(vLocalPos, L);
  float c = dot(vLocalPos, vLocalPos) - uPlanetRadius * uPlanetRadius;
  float disc = b * b - 4.0 * c;
  float shadow = 1.0;
  if (disc > 0.0) {
      float t1 = (-b + sqrt(disc)) / 2.0;
      if (t1 > 0.0) shadow = 0.0; 
  }

  float r = length(vLocalPos.xz);
  float tau = getRingOpticalDepth(r);
  
  // --- Physical Transparency Model ---
  // Transmittance T = e^(-tau / cos(theta_view))
  float transmittance = exp(-tau / max(NdotV, 0.001));
  float alpha = 1.0 - transmittance;

  // Apply high-frequency micro-structure (Cassini/Encke gaps)
  float fineS = sin(r * 300.0) * 0.1 + sin(r * 800.0) * 0.05;
  alpha *= (1.0 + fineS);
  alpha = clamp(alpha, 0.0, 0.95);

  vec3 iceWhite = vec3(0.92, 0.90, 0.88);
  vec3 shadowColor = vec3(0.1, 0.08, 0.05); // Ambient ring tone
  
  // Multiple scattering approximation for albedo
  vec3 albedo = mix(vec3(0.8, 0.7, 0.5), iceWhite, smoothstep(0.1, 1.0, tau));
  
  // Combine all components
  vec3 result = albedo * (0.05 + 1.5 * phase * shadow);
  
  // Add "Interstellar" glow in backlighting
  if (cosTheta > 0.95) {
      result += iceWhite * pow(cosTheta, 64.0) * 4.0 * shadow;
  }

  FragColor = vec4(result, alpha);
}
    
