"""Validate rendered aperture topology and compare NASA camera-space profiles.

The NASA JPEG is a processed photograph with window/lens artifacts, not a
radiometric ground truth. Profile measurements deliberately retain that caveat.
"""
import json
import math
from pathlib import Path
import numpy as np
from PIL import Image

root=Path(__file__).resolve().parents[1]/'build/tests'

def bilinear(a,x,y):
    ix,iy=np.floor(x).astype(int),np.floor(y).astype(int)
    tx,ty=x-ix,y-iy
    return ((1-tx)*(1-ty)*a[iy,ix]+tx*(1-ty)*a[iy,ix+1]
            +(1-tx)*ty*a[iy+1,ix]+tx*ty*a[iy+1,ix+1])

result={}
angles=np.arange(1440)*2*math.pi/1440
for case,rays in [(41,14),(42,8),(43,18)]:
    a=np.fromfile(root/f'solar-response-{case}.bin',dtype='<f2').reshape(720,1280,4)[:,:,0].astype(float)
    profile=np.zeros_like(angles)
    for radius in np.arange(25,46,2):
        profile+=bilinear(a,639.5+radius*np.cos(angles),359.5+radius*np.sin(angles))
    profile/=11
    # Integrate a 7.75-degree angular footprint: a diffraction ray contains
    # genuine side fringes, which are not separate aperture-edge directions.
    # This remains below half the smallest tested ray spacing (20 degrees).
    profile=sum(np.roll(profile,i) for i in range(-15,16))/31
    peaks=np.flatnonzero((profile>np.roll(profile,1))&(profile>=np.roll(profile,-1))
                        &(profile>profile.min()+.25*(profile.max()-profile.min())))
    assert len(peaks)==rays,(case,len(peaks),rays)
    result[f'aperture_{case}']={'expected_rays':rays,'measured_rays':len(peaks),
                              'angular_contrast':float(profile.max()/profile.min())}
    print(f'PASS aperture case={case}: measured {len(peaks)} rays from HDR annulus')

def radial(path,center=None):
    a=np.asarray(Image.open(path).convert('RGB'),dtype=np.float32)/255
    h,w=a.shape[:2]
    if center is None:
        # Locate the solar saturated core in its known region, excluding ISS hardware.
        x=np.arange(w)[None,:];y=np.arange(h)[:,None]
        ys,xs=np.nonzero((a.min(axis=2)>.985)&(x>.68*w)&(x<.86*w)&(y>.28*h)&(y<.50*h))
        center=(float(xs.mean()),float(ys.mean()))
    cx,cy=center
    # 35 mm on a nominal 36x24 mm sensor. Angle between camera rays, not a
    # raw pixel-radius comparison between different source sizes/resolutions.
    fy=35/12;fx=fy/(w/h)
    source=np.array([(2*(cx+.5)/w-1)/fx,(2*(cy+.5)/h-1)/fy,1.])
    source/=np.linalg.norm(source)
    nx=(2*(np.arange(w)+.5)/w-1)/fx
    ny=(2*(np.arange(h)+.5)/h-1)/fy
    cosine=(source[0]*nx[None,:]+source[1]*ny[:,None]+source[2])/np.sqrt(1+nx[None,:]**2+ny[:,None]**2)
    theta=np.degrees(np.arccos(np.clip(cosine,-1,1)))
    code=a@np.array([.2126,.7152,.0722])
    bins=[0,.15,.3,.5,1,2,4]
    stats=[]
    for lo,hi in zip(bins[:-1],bins[1:]):
        values=code[(theta>=lo)&(theta<hi)]
        stats.append({'degrees':[lo,hi],'median_code':float(np.median(values)),
                      'p90_code':float(np.percentile(values,90))})
    return {'center_pixels':center,'annuli':stats}

result['nasa']=radial(root/'solar-reference/ISS067-E-242236.JPG')
result['render']=radial(root/'solar-nasa-framing.png',(.755*1200-.5,.395*800-.5))
differences=[abs(a['median_code']-b['median_code']) for a,b in zip(result['nasa']['annuli'][2:],result['render']['annuli'][2:])]
result['appearance_median_max_error']=max(differences)
result['appearance_median_rms_error']=math.sqrt(sum(d*d for d in differences)/len(differences))
assert max(differences)<.04 and result['appearance_median_rms_error']<.025,'NASA annular appearance outside target'
result['limits']='Processed NASA JPEG; unknown optical transmission and window artifacts. Appearance comparison only.'
(root/'solar-nasa-measurements.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
for tag in ['nasa','render']:
    print(tag,[(s['degrees'],round(s['median_code'],3),round(s['p90_code'],3)) for s in result[tag]['annuli']])
