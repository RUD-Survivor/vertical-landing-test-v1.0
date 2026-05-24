#version 450

layout(push_constant) uniform PC {
    vec2  sunScreenPos;
    float aspect;
    float intensity;
    vec2  scale;
    vec2  offset;
    vec4  color;
    int   shapeType;
    float _pad[3];
} pc;

layout(location = 0) in vec2 vUV;
layout(location = 0) out vec4 FragColor;

void main() {
    vec2 rUV = vUV * 2.0 - 1.0;
    float d = length(rUV);
    float edgeFade = smoothstep(1.0, 0.65, d);
    float alpha = 0.0;

    if (pc.shapeType == 0) {
        float core = exp(-d * d * 12.0) * 2.5;
        float mid  = exp(-d * d * 3.0)  * 0.6;
        float wide = exp(-d * d * 0.8)  * 0.15;
        alpha = (core + mid + wide) * edgeFade;
        float angle  = atan(rUV.y, rUV.x);
        float spike1 = pow(abs(cos(angle)), 300.0);
        float spike2 = pow(abs(sin(angle)), 300.0);
        alpha += (spike1 + spike2) * exp(-d * 1.8) * 0.4 * edgeFade;

    } else if (pc.shapeType == 1) {
        float dx = abs(rUV.x); float dy = abs(rUV.y);
        float efx = smoothstep(1.0, 0.7, dx);
        alpha = exp(-dx*dx*0.5) * exp(-dy*dy*600.0) * efx;
        alpha += exp(-dx*dx*1.5) * exp(-dy*dy*2000.0) * 0.6 * efx;

    } else if (pc.shapeType == 2) {
        float softDisk = exp(-d * d * 4.0) * 0.4;
        float ring = smoothstep(0.95, 0.75, d) * smoothstep(0.55, 0.7, d) * 0.3;
        alpha = (softDisk + ring) * edgeFade;

    } else if (pc.shapeType == 3) {
        float angle = atan(rUV.y, rUV.x);
        float rays  = pow(abs(sin(angle * 6.0)), 80.0) * 0.4;
        rays += pow(abs(cos(angle * 8.0 + 0.3)), 120.0) * 0.2;
        alpha = rays * exp(-d * d * 3.0) * edgeFade;
        alpha += exp(-d * d * 10.0) * 0.2 * edgeFade;
    }

    FragColor = vec4(pc.color.rgb * pc.intensity * alpha, 1.0);
}
