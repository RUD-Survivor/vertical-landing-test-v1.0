#version 450
layout(location=0) in vec2 aPos;
layout(location=0) out vec2 uv;
void main(){uv=aPos*.5+.5;gl_Position=vec4(aPos,0,1);}
