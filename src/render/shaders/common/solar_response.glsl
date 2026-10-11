// Positive low-order field response, not a recovered lens prescription.
// Aberration axes follow incidence; the aperture remains fixed in the lens.
void solarField(out vec2 radial,out vec2 tangent,out float coma,out float astig){
 vec2 field=pc.sunScreenPos*vec2(pc.aspect,1)/pc.focalY;
 float radius=length(field),field2=min(dot(field,field),.36);
 radial=radius>1e-6?field/radius:vec2(1,0);
 tangent=vec2(-radial.y,radial.x);
 coma=.020*field2;astig=.008*field2;
}
