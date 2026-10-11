layout(push_constant) uniform BloomPC {
 vec2 sun;float aspect;float focalY;
 float sinRadius;int removeSolar;int operation;float sigma;
 ivec2 axis;float fraction;float pad;
} pc;
bool solarPixel(vec2 uv,sampler2D depth){
 if(pc.removeSolar==0||textureLod(depth,uv,0).r<1)return false;
 vec2 focal=vec2(pc.focalY/pc.aspect,pc.focalY);
 vec3 source=normalize(vec3(pc.sun/focal,1));
 vec2 size=vec2(textureSize(depth,0));
 float radiusPixels=pc.focalY*pc.sinRadius/(2/size.y*max(source.z*source.z,1e-6));
 if(radiusPixels<1){
   // Match the production unresolved photosphere's finite pixel-filter support.
   // An angular-disc mask alone leaks its reconstruction tails into Bloom.
   float sigma=sqrt(.65*.65+.225*radiusPixels*radiusPixels);
   vec2 relativePixels=(uv*2-1-pc.sun)*size/2;
   return max(abs(relativePixels.x),abs(relativePixels.y))<=8*sigma;
 }
 vec3 ray=normalize(vec3((uv*2-1)/focal,1));
 float footprint=length(vec2(2)/size/focal);
 return dot(source,ray)>0&&length(cross(source,ray))<=pc.sinRadius+footprint;
}
