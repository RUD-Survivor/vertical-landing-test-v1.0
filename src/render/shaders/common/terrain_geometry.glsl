// Planet-space km. Shared by surface/water/shadow so LOD transitions agree.
vec3 thSurfacePoint(vec3 cube,vec3 p,float bed,float water) {
    float h=bed;
#ifdef OCEAN_SURFACE
    h=water;
    vec3 cameraLocal=transpose(mat3(pc.model)/pc.planetRadius)*(frame.viewPos-pc.model[3].xyz);
    float altitude=max(0.0,(length(cameraLocal)-pc.planetRadius-h)*1000.0);
    float fade=1.0-smoothstep(100.0,1500.0,altitude);
    if(fade>0.0) {
        OceanWaves waves=oceanWaves(p,pc.planetRadius*1000.0,frame.time,0.0,max(0.0,h-bed)*1000.0);
        h+=waves.heightM*fade*0.001;
    }
#endif
    // Split large radial term from relief to retain small height differences.
    vec3 base=p*pc.planetRadius;
    if((pc.anchorGeometry&1)==1) {
        vec3 delta=cube-pc.nodePos;
        float a=length(pc.nodePos),c=length(cube);
        // Rationalized norm difference retains centimeter-scale tangent offsets.
        float lengthDifference=dot(delta,cube+pc.nodePos)/(a+c);
        base=(delta-pc.nodePos*(lengthDifference/a))*(pc.planetRadius/c);
    }
    return base+p*h;
}
vec3 thPoint(vec3 cube) {
    vec3 p=normalize(cube);float bed=thHeight(p,pc.planetRadius,pc.maxElevation,pc.detailWeight);
    return thSurfacePoint(cube,p,bed,thWater(p,pc.planetRadius,pc.maxElevation,bed));
}
vec3 thEdgePoint(vec3 cube,vec3 along,float step) {
    float coordinate=dot(cube,along)+1.0;
    float grid=coordinate/step;
    float cell=floor(grid+0.00001),f=clamp(grid-cell,0.0,1.0);
    vec3 first=cube+along*((cell-grid)*step);
    return mix(thPoint(first),thPoint(first+along*step),f);
}
vec3 thGeometry(vec3 cube,float skirt,vec3 fine) {
    float morph=clamp(pc.lodMorph,0.0,1.0);
    if(pc.nodeLevel>0 && morph<1.0) {
        vec3 a=normalize(pc.nodeSide),b=normalize(pc.nodeUp);
        float step=length(pc.nodeSide)/16.0;
        vec2 uv=vec2(dot(cube,a)+1.0,dot(cube,b)+1.0)/step;
        vec2 cell=floor(uv+0.00001),f=clamp(uv-cell,0.0,1.0);
        vec3 origin=cube+a*((cell.x-uv.x)*step)+b*((cell.y-uv.y)*step);
        vec3 p00=thPoint(origin),p11=thPoint(origin+(a+b)*step);
        vec3 coarse;
        // Same diagonal as _buildTerrainPatch: (00,01,10), (10,01,11).
        if(f.x+f.y<=1.0) coarse=p00*(1.0-f.x-f.y)+thPoint(origin+a*step)*f.x+thPoint(origin+b*step)*f.y;
        else coarse=p11*(f.x+f.y-1.0)+thPoint(origin+a*step)*(1.0-f.y)+thPoint(origin+b*step)*(1.0-f.x);
        fine=mix(coarse,fine,morph);
    }
    if((uint(pc.anchorGeometry)&(1u<<30u))!=0u) {
        vec3 a=normalize(pc.nodeSide),b=normalize(pc.nodeUp),d=cube-pc.nodePos;
        float x=dot(d,a)/length(pc.nodeSide),y=dot(d,b)/length(pc.nodeUp);
        int edge=-1;
        if(abs(x+0.5)<0.00001)edge=0;
        else if(abs(x-0.5)<0.00001)edge=1;
        else if(abs(y+0.5)<0.00001)edge=2;
        else if(abs(y-0.5)<0.00001)edge=3;
        if(edge>=0){
            uint state=uint(pc.anchorGeometry);
            int difference=int((state>>uint(1+edge*3))&7u);
            float edgeMorph=float((state>>uint(13+edge*4))&15u)/8.0;
            vec3 along=edge<2?b:a;
            float step=length(pc.nodeSide)/32.0*exp2(float(difference));
            vec3 aligned=thEdgePoint(cube,along,step);
            if(edgeMorph<1.0)aligned=mix(thEdgePoint(cube,along,step*2.0),aligned,edgeMorph);
            fine=aligned;
        }
    }
    // A 3m guard closes subpixel rasterization differences. Deep skirts on
    // closed edges create visible walls at orbital pixel footprints.
    if(skirt<0.0) {
        bool stitched=(uint(pc.anchorGeometry)&(1u<<30u))!=0u;
        fine-=normalize(cube)*(stitched?min(pc.skirtDepthKm,0.003):pc.skirtDepthKm);
    }
    return fine;
}
