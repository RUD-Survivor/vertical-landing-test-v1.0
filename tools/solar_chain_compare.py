"""Archive each actual GPU stage and compare angular structure to NASA JPEG.
No photographic enhancement, synthetic reference, or absolute photometry claim.
"""
import argparse
import json
import math
import shutil
import hashlib
from pathlib import Path
import numpy as np
from PIL import Image

ROOT=Path(__file__).resolve().parents[1]/'build/tests'
def interpolate(a,x,y):
    h,w=a.shape[:2];ix,iy=np.floor(x).astype(int),np.floor(y).astype(int)
    valid=(ix>=0)&(iy>=0)&(ix<w-1)&(iy<h-1)
    ix,iy=np.clip(ix,0,w-2),np.clip(iy,0,h-2);tx,ty=(x-ix)[...,None],(y-iy)[...,None]
    value=(1-tx)*(1-ty)*a[iy,ix]+tx*(1-ty)*a[iy,ix+1]+(1-tx)*ty*a[iy+1,ix]+tx*ty*a[iy+1,ix+1]
    return value,valid

def polar_samples(path,nasa=False,max_degrees=6):
    a=np.asarray(Image.open(path).convert('RGB'),dtype=float)/255;h,w=a.shape[:2]
    if nasa:
        yy,xx=np.mgrid[:h,:w]
        mask=(a.min(axis=2)>.985)&(xx>.68*w)&(xx<.86*w)&(yy>.28*h)&(yy<.50*h)
        y,x=np.nonzero(mask);cx,cy=x.mean(),y.mean()
    else:cx,cy=.755*w-.5,.395*h-.5
    fy=35/12;fx=fy/(w/h)
    source=np.array([(2*(cx+.5)/w-1)/fx,(2*(cy+.5)/h-1)/fy,1]);source/=np.linalg.norm(source)
    east=np.cross([0,1,0],source);east/=np.linalg.norm(east);north=np.cross(source,east)
    radii=np.linspace(.025,max_degrees,int(max_degrees*40));angles=np.linspace(0,2*np.pi,720,endpoint=False)
    t=np.radians(radii)[:,None,None];p=angles[None,:,None]
    rays=np.cos(t)*source+np.sin(t)*(np.cos(p)*east+np.sin(p)*north)
    x=(rays[:,:,0]/rays[:,:,2]*fx+1)*w/2-.5;y=(rays[:,:,1]/rays[:,:,2]*fy+1)*h/2-.5
    rgb,valid=interpolate(a,x,y)
    return rgb,valid,radii,(cx,cy)

def measure(path,nasa=False):
    rgb,valid,radii,(cx,cy)=polar_samples(path,nasa);code=rgb@np.array([.2126,.7152,.0722])
    records=[]
    for lo,hi in [(0,.5),(.5,1),(1,2),(2,4),(4,6)]:
        rows=(radii>=lo)&(radii<hi);values=code[rows][valid[rows]];colors=rgb[rows][valid[rows]]
        angular=np.median(code[rows],axis=0);p05,p50,p95=np.percentile(angular,[5,50,95])
        records.append({'degrees':[lo,hi],'median_code':float(np.median(values)),
                        'angular_contrast_p95_minus_p05':float(p95-p05),
                        'angular_contrast_relative':float((p95-p05)/max(p50,1e-8)),
                        'mean_rgb_code':colors.mean(axis=0).tolist(),
                        'blue_share':float(np.mean(colors[:,2]/np.maximum(colors.sum(axis=1),1e-8)))})
    saturated=(rgb.min(axis=2)>.985)&valid
    # Angular-area weighting: saturation in a 3-degree source-centred cone.
    area=float((saturated*(radii[:,None]<=3)*radii[:,None]).sum()/720*(radii[1]-radii[0])*2*math.pi)
    return {'center_pixels':[float(cx),float(cy)],'rings':records,'saturated_equivalent_radius_degrees':math.sqrt(area/math.pi)}

def main(stage,shape_only=False):
    folder=ROOT/'solar-chain'/f'stage{stage}';folder.mkdir(parents=True,exist_ok=True)
    if stage:
        for name,case in [('nasa-framing',44),('orbit-context',38),('day-context',39),('partial',9)]:
            Image.open(ROOT/f'solar-optics-{case}.ppm').save(folder/f'{name}.png')
        captures=[0,9,14,17,38,39,41,44,45,46,47]+list(range(53,61)) if shape_only else [29,33,41,42,43,44,45,46,47]
        for case in captures:
            if shape_only:Image.open(ROOT/f'solar-optics-{case}.ppm').save(folder/f'case-{case}.png')
            for kind in ['body','response','final']:
                path=ROOT/f'solar-{kind}-{case}.bin'
                if path.exists():shutil.copyfile(path,folder/path.name)
        if stage>=7:
            for path in [ROOT/'solar-camera-profile.json',ROOT.parents[1]/'assets/data/solar/aperture-response.json']:
                shutil.copyfile(path,folder/path.name)
            project=ROOT.parents[1]
            for relative in ['src/render/solar_optics.h','tools/build_solar_response.py']:
                shutil.copyfile(project/relative,folder/Path(relative).name)
            if stage>=12:
                for relative in ['src/render/shaders/lens_flare.frag','src/render/shaders/solar_response.comp','src/render/shaders/common/solar_response.glsl']:
                    shutil.copyfile(project/relative,folder/Path(relative).name)
            atlas=project/'assets/data/solar/aperture-response.rgba32f'
            (folder/'atlas-sha256.txt').write_text(hashlib.sha256(atlas.read_bytes()).hexdigest()+'\n',encoding='utf-8')
    reference=measure(ROOT/'solar-reference/ISS067-E-242236.JPG',True)
    rendered=measure(folder/'nasa-framing.png')
    report={'stage':stage,'nasa':reference,'render':rendered,
            'limits':'Processed JPEG, nominal 35mm full-frame view; lens/window/processing contributions cannot be separated.'}
    report['radial_median_rms']=math.sqrt(np.mean([(x['median_code']-y['median_code'])**2 for x,y in zip(reference['rings'][1:4],rendered['rings'][1:4])]))
    report['angular_contrast_rms']=math.sqrt(np.mean([(x['angular_contrast_p95_minus_p05']-y['angular_contrast_p95_minus_p05'])**2 for x,y in zip(reference['rings'][2:],rendered['rings'][2:])]))
    (folder/'comparison.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    print(f'STAGE {stage} radialRMS={report["radial_median_rms"]:.6f} angularContrastRMS={report["angular_contrast_rms"]:.6f}; saturation radius NASA/render={reference["saturated_equivalent_radius_degrees"]:.4f}/{rendered["saturated_equivalent_radius_degrees"]:.4f}')
    print('Rings:',[(r['degrees'],round(r['median_code'],3),round(r['angular_contrast_relative'],3),round(r['blue_share'],3)) for r in rendered['rings']])

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('stage',type=int);parser.add_argument('--shape-only',action='store_true')
    args=parser.parse_args();main(args.stage,args.shape_only)
