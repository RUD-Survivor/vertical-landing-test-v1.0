#ifndef RADIOMETRY_GLSL
#define RADIOMETRY_GLSL
const float RADIOMETRY_PI=3.141592653589793;
vec3 radiometryToneMap(vec3 radiance,float exposure) {
    vec3 x=clamp(radiance*max(exposure,0.0),vec3(0),vec3(1e6));
    return clamp((x*(2.51*x+0.03))/(x*(2.43*x+0.59)+0.14),0.0,1.0);
}
vec3 radiometrySrgb(vec3 linear) {
    vec3 c=clamp(linear,0.0,1.0);
    return mix(1.055*pow(c,vec3(1.0/2.4))-0.055,12.92*c,
        lessThanEqual(c,vec3(0.0031308)));
}
#endif
