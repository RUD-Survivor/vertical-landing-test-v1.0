
#version 330 core
in vec3 vLocalPos;
in vec3 vWorldPos;
uniform vec3 uViewPos;
uniform mat4 uInvModel;
uniform float uTime;
uniform float uThrottle;
uniform float uExpansion;
uniform float uGroundDist;
uniform float uPlumeLen;
out vec4 FragColor;

float hash(float n) { return fract(sin(n) * 43758.5453123); }
float noise(vec3 x) {
  vec3 p = floor(x); vec3 f = fract(x);
  f = f*f*(3.0-2.0*f);
  float n = p.x + p.y*57.0 + 113.0*p.z;
  return mix(mix(mix(hash(n+0.0),hash(n+1.0),f.x),mix(hash(n+57.0),hash(n+58.0),f.x),f.y),
             mix(mix(hash(n+113.0),hash(n+114.0),f.x),mix(hash(n+170.0),hash(n+171.0),f.x),f.y),f.z);
}

float fbm(vec3 p) {
  float v = 0.0;
  float a = 0.5;
  for (int i=0; i<3; i++) {
    v += a * noise(p);
    p *= 2.0;
    a *= 0.5;
  }
  return v;
}

vec2 rayBoxInter(vec3 ro, vec3 rd, vec3 boxMin, vec3 boxMax) {
  vec3 t0 = (boxMin - ro) / rd;
  vec3 t1 = (boxMax - ro) / rd;
  vec3 tmin = min(t0, t1);
  vec3 tmax = max(t0, t1);
  float dNear = max(max(tmin.x, tmin.y), tmin.z);
  float dFar = min(min(tmax.x, tmax.y), tmax.z);
  return vec2(dNear, dFar);
}

void main() {
  vec3 viewDirWorld = normalize(vWorldPos - uViewPos);
  vec3 ro = (uInvModel * vec4(uViewPos, 1.0)).xyz;
  vec3 rd = normalize((uInvModel * vec4(viewDirWorld, 0.0)).xyz);
  
  vec2 bounds = rayBoxInter(ro, rd, vec3(-0.5), vec3(0.5));
  if (bounds.x > bounds.y || bounds.y < 0.0) discard;
  
  float start = max(0.0, bounds.x);
  float end = bounds.y;
  
  // High-frequency jitter
  float dither = hash(dot(gl_FragCoord.xy, vec2(12.9898, 78.233))) * 0.015;
  float t = start + dither;
  
  vec4 finalCol = vec4(0.0);
  int steps = 60; // Slightly more steps for thin splash precision
  float stepSize = (end - start) / float(steps);

  // Ground collision normalized height (0 at end of box, 1 at nozzle)
  float zGround = 1.0 - uGroundDist / uPlumeLen;

  for (int i=0; i<steps; i++) {
    vec3 p = ro + rd * t;
    float z = p.y + 0.5; // 0 (end) to 1 (nozzle)
    
    // GROUND CLIP
    if (z < zGround) {
       t += stepSize;
       continue;
    }

    float distToGround = z - zGround;
    // TIGHT LOCALIZED SPLASH: Only within few % of plume length near ground
    float splashZone = smoothstep(0.06, 0.0, distToGround) * smoothstep(-0.2, 0.05, zGround);
    
    float r = length(p.xz) * 2.0; 
    
    // Waterfall-style Flare: Multi-stage expansion
    float coreFlare = mix(0.35, 1.0 + uExpansion * 4.0, pow(1.0 - z, 1.4));
    
    // PROPORTIONAL RADIAL SPLASH
    // Radius is proportional to the depth of truncated plume (zGround)
    float truncDepth = max(0.0, zGround);
    float radAngle = atan(p.x, p.z);
    float radialWildness = 0.7 + 0.6 * noise(vec3(radAngle * 4.0, uTime * 18.0, zGround * 8.0));
    // splashRadiusBoost covers more of the local box radius (0.5-0.7)
    float splashRadiusBoost = truncDepth * 1.6 * splashZone * radialWildness;
    
    coreFlare += splashRadiusBoost;
    
    float sheathFlare = coreFlare * (1.1 + uExpansion * 2.0);
    
    if (r < sheathFlare) {
      float normR = r / coreFlare;
      
      // 1. Base Physic-Informed Density (Solid Core Focus)
      float d = exp(-normR * 6.0) * pow(z, 0.45) * (1.1 - normR);
      
      // Splash Density Boost: Make the splash feel thick/viscous
      // Density falloff is flatter horizontally
      d += splashZone * 0.7 * exp(-normR * 2.0) * (1.0 - normR * 0.95);

      // Tail Billowing: High-frequency radial distortion at the dissipation zone
      float billowValue = noise(vec3(p.xz * 12.0, uTime * 20.0)) * (1.0 - z) * 0.15;
      float taperedR = normR + billowValue;
      
      // Tail Tapering: Smooth falloff at the bottom/ground
      float tailTaper = (zGround > -0.2) ? smoothstep(-0.01, 0.04, distToGround) : smoothstep(0.0, 0.2, z);
      d *= tailTaper * (1.0 / (1.0 + billowValue * 5.0));
      d = max(0.0, d);

      // 2. High-Velocity Dynamics & FBM Turbulence
      float gasSpeed = uTime * (40.0 + uThrottle * 20.0);
      float streaks = noise(vec3(atan(p.x, p.z) * 8.0, p.y * 50.0, gasSpeed * 0.8)); 
      float turb = fbm(p * 18.0 - vec3(0, gasSpeed, 0)); 
      d *= (0.4 + 0.5 * turb + 0.35 * streaks);
      
      // 3. View-Dependent Alpha Falloff (Improved Side-View)
      float edgeSoftness = smoothstep(0.0, 0.4, 1.0 - taperedR);
      d *= (0.2 + 0.8 * edgeSoftness);
      
      // 4. Dynamic Pulse/Flicker (Synced with Throttle)
      float flicker = 0.94 + 0.12 * hash(uTime * 150.0 * uThrottle); 
      float pulse = 1.0 + 0.08 * sin(uTime * 100.0) * uThrottle; 
      d *= (flicker * pulse);

      // 5. High-Dynamic Color Profiling (Aggressive Glow)
      vec3 core = vec3(1.2, 1.2, 1.1);
      vec3 mid = vec3(1.0, 0.5, 0.05);
      vec3 outer = vec3(0.6, 0.12, 0.04);
      
      // Splash color shift: brighter near the ground impact
      mid = mix(mid, vec3(1.3, 1.0, 0.4), splashZone * 0.8);

      vec3 col = mix(outer, mid, smoothstep(0.05, 0.4, d));
      col = mix(col, core, smoothstep(0.4, 0.9, d));
      
      float alpha = d * stepSize * 65.0; 
      finalCol.rgb += (1.0 - finalCol.a) * col * alpha;
      finalCol.a += (1.0 - finalCol.a) * alpha;
    }
    
    t += stepSize;
    if (finalCol.a > 0.99) break;
  }

  FragColor = vec4(finalCol.rgb * uThrottle * 5.0, finalCol.a * uThrottle);
}
    
