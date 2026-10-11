"""Independent spatial/temporal checks on actual GPU HDR readbacks."""
import json
from pathlib import Path
import numpy as np

ROOT=Path(__file__).resolve().parents[1]/'build/tests'
def read(case,w=1280,h=720,kind='response'):
    return np.fromfile(ROOT/f'solar-{kind}-{case}.bin',dtype='<f2').reshape(h,w,4)[:,:,:3].astype(float)
def measure(a):
    y=a@np.array([.2126,.7152,.0722]);h,w=y.shape
    total=y.sum()
    assert np.isfinite(a).all() and a.min()>=0 and total>0
    return {'flux':float(total*4/(w*h)),
            'cx':float((y.sum(axis=0)*np.arange(w)).sum()/total),
            'cy':float((y.sum(axis=1)*np.arange(h)).sum()/total)}

full=measure(read(29));half=measure(read(45));phase=measure(read(46));odd=measure(read(47,1281,721))
print('Spatial measurements:',full,half,phase,odd)
assert abs(half['flux']/full['flux']-.5)<.01,'half eclipse optical energy'
assert half['cx']>full['cx']+.15,'left occluder should shift the response toward the visible right limb'
assert abs(phase['flux']/full['flux']-1)<.015,'subpixel energy instability'
assert abs(phase['cx']-full['cx']-.5)<.12,'subpixel response centroid'
assert abs(odd['flux']/full['flux']*(1281/721)/(1280/720)-1)<.015,'odd resolution energy'
assert abs(odd['cx']-full['cx']-.5)<.12,'odd resolution centroid'
report={'full':full,'half':half,'half_pixel':phase,'odd_resolution':odd,
        'limits':'Checks source-shape transfer and subpixel translation, not long-term TAA motion history.'}
(ROOT/'solar-structural-measurements.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
print('PASS source-shape transfer, subpixel energy/centroid, odd resolution:',report)
