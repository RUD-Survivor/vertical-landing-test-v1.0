#pragma once
#include "math/math3d.h"
// Terrain PC is 128 bytes. Its affine matrix bottom row (3,7,11), otherwise
// unused by these shaders, carries a high-precision patch anchor. The rotation
// and planet translation stay intact. Only the PC copy is packed, not snapshots.
inline void packTerrainAnchor(float model[16],const float cubeCenter[3],float radius,const float camera[3]){
    Vec3 dir=Vec3(cubeCenter[0],cubeCenter[1],cubeCenter[2]).normalized();
    Vec3 x=Vec3(model[0],model[1],model[2]).normalized();
    Vec3 y=Vec3(model[4],model[5],model[6]).normalized();
    Vec3 z=Vec3(model[8],model[9],model[10]).normalized();
    Vec3 anchor=Vec3(model[12],model[13],model[14])+(x*dir.x+y*dir.y+z*dir.z)*radius-
        Vec3(camera[0],camera[1],camera[2]);
    model[3]=float(anchor.x);model[7]=float(anchor.y);model[11]=float(anchor.z);
}
