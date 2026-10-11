#version 450
layout(location=0) noperspective in vec2 offsetPixels;
layout(location=1) flat in vec3 radiance;
layout(location=2) flat in float sigma;
layout(location=3) flat in float haloFraction;
layout(location=0) out vec4 FragColor;
// Abramowitz-Stegun erf approximation; pixel integration prevents subpixel
// point-sampling flicker. Both PSFs carry unit integrated flux.
vec2 erfApprox(vec2 x) {
    vec2 t=1.0/(1.0+0.3275911*abs(x));
    vec2 p=(((((1.061405429*t-1.453152027)*t)+1.421413741)*t-0.284496736)*t+0.254829592)*t;
    return sign(x)*(1.0-p*exp(-x*x));
}
float pixelMass(float width) {
    vec2 lo=(offsetPixels-0.5)/(1.41421356237*width);
    vec2 hi=(offsetPixels+0.5)/(1.41421356237*width);
    vec2 mass=0.5*(erfApprox(hi)-erfApprox(lo));
    return max(mass.x*mass.y,0.0);
}
void main() {
    float mass=mix(pixelMass(sigma),pixelMass(sigma*2.0),haloFraction);
    FragColor=vec4(radiance*mass,0.0);
}
