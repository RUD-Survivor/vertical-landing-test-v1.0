// Shared 64-byte ABI; former padding now carries projection/source data.
layout(push_constant) uniform PC {
 vec2 sunScreenPos; float aspect; float intensity;
 vec2 scale; vec2 offset; vec4 color; int shapeType;
 float focalY; float sinRadius; float sourceFlux;
} pc;
float solarErf(float x) {
 float t=1.0/(1.0+0.3275911*abs(x));
 float y=1.0-(((((1.061405429*t-1.453152027)*t)+1.421413741)*t-0.284496736)*t+0.254829592)*t*exp(-x*x);
 return sign(x)*y;
}
float solarPixelMass(float x,float sigma) {
 return .5*(solarErf((x+.5)/(1.41421356237*sigma))-solarErf((x-.5)/(1.41421356237*sigma)));
}
// Mask each bilinear tap before interpolation. Testing depth only at the
// interpolated sample position discards visible solar energy beside an opaque
// edge, making half eclipses lose more than half the optical response.
vec3 solarObserved(sampler2D scene,sampler2D depth,vec2 uv) {
 ivec2 size=textureSize(scene,0);
 vec2 p=uv*vec2(size)-.5;
 ivec2 base=ivec2(floor(p));vec2 f=fract(p);
 vec3 result=vec3(0);
 for(int y=0;y<2;++y)for(int x=0;x<2;++x){
   ivec2 q=clamp(base+ivec2(x,y),ivec2(0),size-1);
   float weight=(x==0?1-f.x:f.x)*(y==0?1-f.y:f.y);
   if(texelFetch(depth,q,0).r>=1.0)result+=max(texelFetch(scene,q,0).rgb,vec3(0))*weight;
 }
 return result;
}
